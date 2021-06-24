//******************************************************************************
// Copyright (c) 2013 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Functional Units
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// If regfile bypassing is disabled, then the functional unit must do its own
// bypassing in here on the WB stage (i.e., bypassing the io.resp.data)
//
// TODO: explore possibility of conditional IO fields? if a branch unit... how to add extra to IO in subclass?

package boom.exu

import chisel3._
import chisel3.util._
import chisel3.experimental.chiselName

import freechips.rocketchip.config.Parameters
import freechips.rocketchip.rocket.ALU._
import freechips.rocketchip.util._
import freechips.rocketchip.tile
import freechips.rocketchip.rocket.{DecodeLogic,PipelinedMultiplier,BP,BreakpointUnit,Causes,CSR,VConfig,VType}

import boom.common._
import boom.common.MicroOpcodes._
import boom.ifu._
import boom.util._

/**t
 * Functional unit constants
 */
object FUConstants
{
  // bit mask, since a given execution pipeline may support multiple functional units
  val FUC_SZ = 13
  val FU_X   = BitPat.dontCare(FUC_SZ)
  val FU_ALU_ID = 0
  val FU_JMP_ID = 1
  val FU_MEM_ID = 2
  val FU_MUL_ID = 3
  val FU_DIV_ID = 4
  val FU_CSR_ID = 5
  val FU_FPU_ID = 6
  val FU_FDV_ID = 7
  val FU_I2F_ID = 8
  val FU_F2I_ID = 9
  val FU_VMX_ID = 10  // vec load /store index vec store data
  val FU_MAC_ID = 11
  val FU_VMASKU_ID = 12
  val FU_ALU = (1<<FU_ALU_ID).U(FUC_SZ.W)
  val FU_JMP = (1<<FU_JMP_ID).U(FUC_SZ.W)
  val FU_MEM = (1<<FU_MEM_ID).U(FUC_SZ.W)
  val FU_MUL = (1<<FU_MUL_ID).U(FUC_SZ.W)
  val FU_DIV = (1<<FU_DIV_ID).U(FUC_SZ.W)
  val FU_CSR = (1<<FU_CSR_ID).U(FUC_SZ.W)
  val FU_FPU = (1<<FU_FPU_ID).U(FUC_SZ.W)
  val FU_FDV = (1<<FU_FDV_ID).U(FUC_SZ.W)
  val FU_I2F = (1<<FU_I2F_ID).U(FUC_SZ.W)
  val FU_F2I = (1<<FU_F2I_ID).U(FUC_SZ.W)
  val FU_VMX = (1<<FU_VMX_ID).U(FUC_SZ.W)
  val FU_MAC = (1<<FU_MAC_ID).U(FUC_SZ.W)
  val FU_VMASKU = (1<<FU_VMASKU_ID).U(FUC_SZ.W)

  // FP stores generate data through FP F2I, and generate address through MemAddrCalc
  def FU_F2IMEM = ((1 << FU_MEM_ID) | (1 << FU_F2I_ID)).U(FUC_SZ.W)
  // VEC load / store, vs3 read by ALU of Vec Exe Unit
  def FU_MEMV =   ((1 << FU_MEM_ID) | (1 << FU_VMX_ID)).U(FUC_SZ.W)
}
import FUConstants._

/**
 * Class to tell the FUDecoders what units it needs to support
 *
 * @param alu support alu unit?
 * @param bru support br unit?
 * @param mem support mem unit?
 * @param muld support multiple div unit?
 * @param fpu support FP unit?
 * @param csr support csr writing unit?
 * @param fdiv support FP div unit?
 * @param ifpu support int to FP unit?
 */
class SupportedFuncUnits(
  val alu: Boolean  = false,
  val jmp: Boolean  = false,
  val mem: Boolean  = false,
  val muld: Boolean = false,
  val fpu: Boolean  = false,
  val csr: Boolean  = false,
  val fdiv: Boolean = false,
  val ifpu: Boolean = false,
  val vector: Boolean = false)
{
}


/**
 * Bundle for signals sent to the functional unit
 *
 * @param dataWidth width of the data sent to the functional unit
 */
class FuncUnitReq(val dataWidth: Int)(implicit p: Parameters) extends BoomBundle
  with HasBoomUOP
{
  val numOperands = 3

  val rs1_data = UInt(dataWidth.W)
  val rs2_data = UInt(dataWidth.W)
  val rs3_data = UInt(dataWidth.W) // only used for FMA units
  val pred_data = Bool()

  val kill = Bool() // kill everything
}

/**
 * Bundle for the signals sent out of the function unit
 *
 * @param dataWidth data sent from the functional unit
 */
class FuncUnitResp(val dataWidth: Int)(implicit p: Parameters) extends BoomBundle
  with HasBoomUOP
{
  val predicated = Bool() // Was this response from a predicated-off instruction
  val data = UInt(dataWidth.W)
  val fflags = new ValidIO(new FFlagsResp)
  val addr = UInt((vaddrBits+1).W) // only for maddr -> LSU
  val mxcpt = new ValidIO(UInt((freechips.rocketchip.rocket.Causes.all.max+2).W)) //only for maddr->LSU
  val sfence = Valid(new freechips.rocketchip.rocket.SFenceReq) // only for mcalc
}

/**
 * Branch resolution information given from the branch unit
 */
class BrResolutionInfo(implicit p: Parameters) extends BoomBundle
{
  val uop        = new MicroOp
  val valid      = Bool()
  val mispredict = Bool()
  val taken      = Bool()                     // which direction did the branch go?
  val cfi_type   = UInt(CFI_SZ.W)

  // Info for recalculating the pc for this branch
  val pc_sel     = UInt(2.W)

  val jalr_target = UInt(vaddrBitsExtended.W)
  val target_offset = SInt()
}

class BrUpdateInfo(implicit p: Parameters) extends BoomBundle
{
  // On the first cycle we get masks to kill registers
  val b1 = new BrUpdateMasks
  // On the second cycle we get indices to reset pointers
  val b2 = new BrResolutionInfo
}

class BrUpdateMasks(implicit p: Parameters) extends BoomBundle
{
  val resolve_mask = UInt(maxBrCount.W)
  val mispredict_mask = UInt(maxBrCount.W)
}


/**
 * Abstract top level functional unit class that wraps a lower level hand made functional unit
 *
 * @param isPipelined is the functional unit pipelined?
 * @param numStages how many pipeline stages does the functional unit have
 * @param numBypassStages how many bypass stages does the function unit have
 * @param dataWidth width of the data being operated on in the functional unit
 * @param hasBranchUnit does this functional unit have a branch unit?
 */
abstract class FunctionalUnit(
  val isPipelined: Boolean,
  val numStages: Int,
  val numBypassStages: Int,
  val dataWidth: Int,
  val isJmpUnit: Boolean = false,
  val isAluUnit: Boolean = false,
  val isCsrUnit: Boolean = false,
  val isMemAddrCalcUnit: Boolean = false,
  val needsFcsr: Boolean = false)
  (implicit p: Parameters) extends BoomModule
{
  val io = IO(new Bundle {
    val req    = Flipped(new DecoupledIO(new FuncUnitReq(dataWidth)))
    val resp   = (new DecoupledIO(new FuncUnitResp(dataWidth)))

    val brupdate = Input(new BrUpdateInfo())

    val bypass = Output(Vec(numBypassStages, Valid(new ExeUnitResp(dataWidth))))

    // only used by the fpu unit
    val fcsr_rm = if (needsFcsr) Input(UInt(tile.FPConstants.RM_SZ.W)) else null
    val vconfig = if (usingVector & isCsrUnit) Input(new VConfig) else null

    // only used by branch unit
    val brinfo     = if (isAluUnit) Output(new BrResolutionInfo()) else null
    val get_ftq_pc = if (isJmpUnit) Flipped(new GetPCFromFtqIO()) else null
    val status     = if (isMemAddrCalcUnit) Input(new freechips.rocketchip.rocket.MStatus()) else null

    // only used by memaddr calc unit
    val bp = if (isMemAddrCalcUnit) Input(Vec(nBreakpoints, new BP)) else null
    val mcontext = if (isMemAddrCalcUnit) Input(UInt(coreParams.mcontextWidth.W)) else null
    val scontext = if (isMemAddrCalcUnit) Input(UInt(coreParams.scontextWidth.W)) else null

  })
}

/**
 * Abstract top level pipelined functional unit
 *
 * Note: this helps track which uops get killed while in intermediate stages,
 * but it is the job of the consumer to check for kills on the same cycle as consumption!!!
 *
 * @param numStages how many pipeline stages does the functional unit have
 * @param numBypassStages how many bypass stages does the function unit have
 * @param earliestBypassStage first stage that you can start bypassing from
 * @param dataWidth width of the data being operated on in the functional unit
 * @param hasBranchUnit does this functional unit have a branch unit?
 */
abstract class PipelinedFunctionalUnit(
  numStages: Int,
  numBypassStages: Int,
  earliestBypassStage: Int,
  dataWidth: Int,
  isJmpUnit: Boolean = false,
  isAluUnit: Boolean = false,
  isCsrUnit: Boolean = false,
  isMemAddrCalcUnit: Boolean = false,
  needsFcsr: Boolean = false
  )(implicit p: Parameters) extends FunctionalUnit(
    isPipelined = true,
    numStages = numStages,
    numBypassStages = numBypassStages,
    dataWidth = dataWidth,
    isJmpUnit = isJmpUnit,
    isAluUnit = isAluUnit,
    isCsrUnit = isCsrUnit,
    isMemAddrCalcUnit = isMemAddrCalcUnit,
    needsFcsr = needsFcsr)
{
  // Pipelined functional unit is always ready.
  io.req.ready := true.B

  if (numStages > 0) {
    val r_valids = RegInit(VecInit(Seq.fill(numStages) { false.B }))
    val r_uops   = Reg(Vec(numStages, new MicroOp()))

    // handle incoming request
    r_valids(0) := io.req.valid && !IsKilledByBranch(io.brupdate, io.req.bits.uop) && !io.req.bits.kill
    r_uops(0)   := io.req.bits.uop
    r_uops(0).br_mask := GetNewBrMask(io.brupdate, io.req.bits.uop)

    // handle middle of the pipeline
    for (i <- 1 until numStages) {
      r_valids(i) := r_valids(i-1) && !IsKilledByBranch(io.brupdate, r_uops(i-1)) && !io.req.bits.kill
      r_uops(i)   := r_uops(i-1)
      r_uops(i).br_mask := GetNewBrMask(io.brupdate, r_uops(i-1))

      if (numBypassStages > 0) {
        io.bypass(i-1).bits.uop := r_uops(i-1)
      }
    }

    // handle outgoing (branch could still kill it)
    // consumer must also check for pipeline flushes (kills)
    io.resp.valid    := r_valids(numStages-1) && !IsKilledByBranch(io.brupdate, r_uops(numStages-1))
    io.resp.bits.predicated := false.B
    io.resp.bits.uop := r_uops(numStages-1)
    io.resp.bits.uop.br_mask := GetNewBrMask(io.brupdate, r_uops(numStages-1))

    // bypassing (TODO allow bypass vector to have a different size from numStages)
    if (numBypassStages > 0 && earliestBypassStage == 0) {
      io.bypass(0).bits.uop := io.req.bits.uop

      for (i <- 1 until numBypassStages) {
        io.bypass(i).bits.uop := r_uops(i-1)
      }
    }
  } else {
    require (numStages == 0)
    // pass req straight through to response

    // valid doesn't check kill signals, let consumer deal with it.
    // The LSU already handles it and this hurts critical path.
    io.resp.valid    := io.req.valid && !IsKilledByBranch(io.brupdate, io.req.bits.uop)
    io.resp.bits.predicated := false.B
    io.resp.bits.uop := io.req.bits.uop
    io.resp.bits.uop.br_mask := GetNewBrMask(io.brupdate, io.req.bits.uop)
  }
}

/**
 * Functional unit that wraps RocketChips ALU
 *
 * @param isBranchUnit is this a branch unit?
 * @param numStages how many pipeline stages does the functional unit have
 * @param dataWidth width of the data being operated on in the functional unit
 */
@chiselName
class ALUUnit(
  isJmpUnit: Boolean = false,
  isCsrUnit: Boolean = false,
  numStages: Int = 1,
  dataWidth: Int,
  hasVMX: Boolean = false,
  vector: Boolean = false
  )
(implicit p: Parameters)
  extends PipelinedFunctionalUnit(
    numStages = numStages,
    numBypassStages = numStages,
    isAluUnit = true,
    earliestBypassStage = 0,
    dataWidth = dataWidth,
    isCsrUnit = isCsrUnit,
    isJmpUnit = isJmpUnit)
  with boom.ifu.HasBoomFrontendParameters
{
  val uop = io.req.bits.uop

  // immediate generation
  val imm_xprlen = ImmGen(uop.imm_packed, uop.ctrl.imm_sel)
  val rs1_data = io.req.bits.rs1_data
  val rs2_data = io.req.bits.rs2_data

  // operand 1 select
  var op1_data: UInt = null
  if (isJmpUnit) {
    // Get the uop PC for jumps
    val block_pc = AlignPCToBoundary(io.get_ftq_pc.pc, icBlockBytes)
    val uop_pc = (block_pc | uop.pc_lob) - Mux(uop.edge_inst, 2.U, 0.U)

    op1_data = Mux(uop.ctrl.op1_sel.asUInt === OP1_RS1 , io.req.bits.rs1_data,
               Mux(uop.ctrl.op1_sel.asUInt === OP1_PC  , Sext(uop_pc, xLen),
                                                         0.U))
  } else {
    op1_data = Mux(uop.ctrl.op1_sel.asUInt === OP1_RS1 , io.req.bits.rs1_data,
               Mux(uop.ctrl.op1_sel.asUInt === OP1_VS2 , io.req.bits.rs2_data,
               Mux(uop.ctrl.op1_sel.asUInt === OP1_INV_VS2, ~io.req.bits.rs2_data,
                                                         0.U)))
  }

  // operand 2 select
  val op2_data = WireInit(0.U(xLen.W))
  if (usingVector & isCsrUnit) {
    val vsetvl    = (uop.uopc === uopVSETVL)
    val vsetvli   = (uop.uopc === uopVSETVLI)
    val vsetivli  = (uop.uopc === uopVSETIVLI)
    val vtypei    = Mux(vsetvl, rs2_data(7,0), uop.imm_packed(15,8))
    // prs1 takes the value of lrs1 when lrs1 is INT x0, or lrs1 is not INT/FP/VEC
    val useCurrentVL = (vsetvli | vsetvl) & (uop.ldst === 0.U) & (uop.prs1 === 0.U)
    val useMaxVL     = (vsetvli | vsetvl) & (uop.ldst =/= 0.U) & (uop.prs1 === 0.U)
    val avl       = Mux(vsetivli, uop.prs1, rs1_data)
    val new_vl    = VType.computeVL(avl, vtypei, io.vconfig.vl, useCurrentVL, useMaxVL, false.B)

    op2_data:= Mux(uop.ctrl.op2_sel === OP2_IMM,     Sext(imm_xprlen.asUInt, xLen),
               Mux(uop.ctrl.op2_sel === OP2_IMMC,    uop.prs1(4,0),
               Mux(uop.ctrl.op2_sel === OP2_RS2 ,    io.req.bits.rs2_data,
               Mux(uop.ctrl.op2_sel === OP2_INV_VS1, ~io.req.bits.rs1_data,
               Mux(uop.ctrl.op2_sel === OP2_NEXT,    Mux(uop.is_rvc, 2.U, 4.U),
               Mux(uop.ctrl.op2_sel === OP2_VL,      new_vl, 0.U))))))

    val set_vconfig = vsetvl | vsetvli | vsetivli
    io.resp.bits.uop.vconfig.vl := RegEnable(new_vl, set_vconfig)
    io.resp.bits.uop.vconfig.vtype := RegEnable(VType.fromUInt(vtypei), set_vconfig)
  } else {
    op2_data:= Mux(uop.ctrl.op2_sel === OP2_IMM,     Sext(imm_xprlen.asUInt, xLen),
               Mux(uop.ctrl.op2_sel === OP2_IMMC,    uop.prs1(4,0),
               Mux(uop.ctrl.op2_sel === OP2_RS2 ,    io.req.bits.rs2_data,
               Mux(uop.ctrl.op2_sel === OP2_NEXT,    Mux(uop.is_rvc, 2.U, 4.U),
               Mux(uop.ctrl.op2_sel === OP2_VS1,     io.req.bits.rs1_data,
               Mux(uop.ctrl.op2_sel === OP2_INV_VS1, ~io.req.bits.rs1_data,
                                                     0.U))))))
  }

  val alu = Module(new freechips.rocketchip.rocket.ALU(withCarryIO = usingVector && vector))

  alu.io.in1 := op1_data.asUInt
  alu.io.in2 := op2_data.asUInt
  alu.io.fn  := uop.ctrl.op_fcn
  alu.io.dw  := uop.ctrl.fcn_dw
  if (usingVector && vector) {
    alu.io.ci  := uop.uopc.isOneOf(uopVADC, uopVSBC, uopVMADC, uopVMSBC) & ~uop.v_unmasked & uop.v_active
  }

  // Did I just get killed by the previous cycle's branch,
  // or by a flush pipeline?
  val killed = WireInit(false.B)
  when (io.req.bits.kill || IsKilledByBranch(io.brupdate, uop)) {
    killed := true.B
  }

  val br_eq  = (rs1_data === rs2_data)
  val br_ltu = (rs1_data.asUInt < rs2_data.asUInt)
  val br_lt  = (~(rs1_data(xLen-1) ^ rs2_data(xLen-1)) & br_ltu |
                rs1_data(xLen-1) & ~rs2_data(xLen-1)).asBool

  val pc_sel = MuxLookup(uop.ctrl.br_type, PC_PLUS4,
                 Seq(   BR_N   -> PC_PLUS4,
                        BR_NE  -> Mux(!br_eq,  PC_BRJMP, PC_PLUS4),
                        BR_EQ  -> Mux( br_eq,  PC_BRJMP, PC_PLUS4),
                        BR_GE  -> Mux(!br_lt,  PC_BRJMP, PC_PLUS4),
                        BR_GEU -> Mux(!br_ltu, PC_BRJMP, PC_PLUS4),
                        BR_LT  -> Mux( br_lt,  PC_BRJMP, PC_PLUS4),
                        BR_LTU -> Mux( br_ltu, PC_BRJMP, PC_PLUS4),
                        BR_J   -> PC_BRJMP,
                        BR_JR  -> PC_JALR
                        ))

  val is_taken = io.req.valid &&
                   !killed &&
                   (uop.is_br || uop.is_jalr || uop.is_jal) &&
                   (pc_sel =/= PC_PLUS4)

  // "mispredict" means that a branch has been resolved and it must be killed
  val mispredict = WireInit(false.B)

  val is_br          = io.req.valid && !killed && uop.is_br && !uop.is_sfb
  val is_jal         = io.req.valid && !killed && uop.is_jal
  val is_jalr        = io.req.valid && !killed && uop.is_jalr

  when (is_br || is_jalr) {
    if (!isJmpUnit) {
      assert (pc_sel =/= PC_JALR)
    }
    when (pc_sel === PC_PLUS4) {
      mispredict := uop.taken
    }
    when (pc_sel === PC_BRJMP) {
      mispredict := !uop.taken
    }
  }

  val brinfo = Wire(new BrResolutionInfo)

  // note: jal doesn't allocate a branch-mask, so don't clear a br-mask bit
  brinfo.valid          := is_br || is_jalr
  brinfo.mispredict     := mispredict
  brinfo.uop            := uop
  brinfo.cfi_type       := Mux(is_jalr, CFI_JALR,
                           Mux(is_br  , CFI_BR, CFI_X))
  brinfo.taken          := is_taken
  brinfo.pc_sel         := pc_sel

  brinfo.jalr_target    := DontCare


  // Branch/Jump Target Calculation
  // For jumps we read the FTQ, and can calculate the target
  // For branches we emit the offset for the core to redirect if necessary
  val target_offset = imm_xprlen(20,0).asSInt
  brinfo.jalr_target := DontCare
  if (isJmpUnit) {
    def encodeVirtualAddress(a0: UInt, ea: UInt) = if (vaddrBitsExtended == vaddrBits) {
      ea
    } else {
      // Efficient means to compress 64-bit VA into vaddrBits+1 bits.
      // (VA is bad if VA(vaddrBits) != VA(vaddrBits-1)).
      val a = a0.asSInt >> vaddrBits
      val msb = Mux(a === 0.S || a === -1.S, ea(vaddrBits), !ea(vaddrBits-1))
      Cat(msb, ea(vaddrBits-1,0))
    }


    val jalr_target_base = io.req.bits.rs1_data.asSInt
    val jalr_target_xlen = Wire(UInt(xLen.W))
    jalr_target_xlen := (jalr_target_base + target_offset).asUInt
    val jalr_target = (encodeVirtualAddress(jalr_target_xlen, jalr_target_xlen).asSInt & -2.S).asUInt

    brinfo.jalr_target := jalr_target
    val cfi_idx = ((uop.pc_lob ^ Mux(io.get_ftq_pc.entry.start_bank === 1.U, 1.U << log2Ceil(bankBytes), 0.U)))(log2Ceil(fetchWidth),1)

    when (pc_sel === PC_JALR) {
      mispredict := !io.get_ftq_pc.next_val ||
                    (io.get_ftq_pc.next_pc =/= jalr_target) ||
                    !io.get_ftq_pc.entry.cfi_idx.valid ||
                    (io.get_ftq_pc.entry.cfi_idx.bits =/= cfi_idx)
    }
  }

  brinfo.target_offset := target_offset


  io.brinfo := brinfo



// Response
// TODO add clock gate on resp bits from functional units
//   io.resp.bits.data := RegEnable(alu.io.out, io.req.valid)
//   val reg_data = Reg(outType = Bits(width = xLen))
//   reg_data := alu.io.out
//   io.resp.bits.data := reg_data

  val r_val  = RegInit(VecInit(Seq.fill(numStages) { false.B }))
  val r_data = Reg(Vec(numStages, UInt(xLen.W)))
  val r_pred = Reg(Vec(numStages, Bool()))
  val alu_out = WireInit(alu.io.out)
  if (usingVector && vector) {
    val vstart = uop.vstart
    val vl = uop.vconfig.vl

    val isVMerge: Bool = uop.is_rvv && uop.uopc === uopMERGE
    val v_inactive = uop.is_rvv && !uop.v_active
    val v_tail = (vstart >= vl)

    val byteWidth = 3.U
    val vsew64bit = 3.U
    val is_multiple_of_64 = vl(5,0) === 0.U
    val vmlogic_vl = vl(5,0).orR +& (vl>>(byteWidth +& vsew64bit))
    val is_vmlogic_last_split = vstart === (vmlogic_vl-1.U)

    val vmlogic_insn = uop.ctrl.is_vmlogic
    val vmlogic_mask = boom.util.MaskGen(0.U, vl(5,0), 64)
    val vmlogic_alu_result = Mux(uop.uopc.isOneOf(uopVMNAND, uopVMNOR, uopVMXNOR), ~alu.io.out, alu.io.out)
    val vmlogic_last_result = (vmlogic_alu_result & vmlogic_mask) | (io.req.bits.rs3_data & (~vmlogic_mask))
    val vmlogic_result = Mux(is_vmlogic_last_split & (~is_multiple_of_64), vmlogic_last_result, vmlogic_alu_result)

    val vmscmp = uop.ctrl.is_vmscmp
    when (io.req.valid && vmscmp) { assert(uop.rt(RD, isMaskVD), "Problematic vmcompare") }
    val vadc   = uop.uopc === uopVADC
    val vsbc   = uop.uopc === uopVSBC
    val vmadc  = uop.uopc === uopVMADC
    val vmsbc  = uop.uopc === uopVMSBC
    when (io.req.valid && (vadc || vsbc)) { assert(!uop.v_unmasked, "Problematic vadc/vsbc") }
    val alu_co = Mux1H(UIntToOH(uop.vconfig.vtype.vsew(1,0)), Seq(alu.io.out(8), alu.io.out(16), alu.io.out(32), alu.io.co))

    alu_out := Mux(uop.is_sfb_shadow && io.req.bits.pred_data, Mux(uop.ldst_is_rs1, io.req.bits.rs1_data, io.req.bits.rs2_data),
               Mux(uop.uopc === uopMOV, io.req.bits.rs2_data,
               Mux(isVMerge, Mux(!uop.v_active, io.req.bits.rs2_data, io.req.bits.rs1_data),
               Mux(uop.uopc.isOneOf(uopVMIN, uopVMINU) && !v_inactive, Mux(alu.io.out(0), io.req.bits.rs2_data, io.req.bits.rs1_data),
               Mux(uop.uopc.isOneOf(uopVMAX, uopVMAXU) && !v_inactive, Mux(alu.io.out(0), io.req.bits.rs1_data, io.req.bits.rs2_data),
               Mux(uop.rt(RD, isReduceV) && v_inactive, io.req.bits.rs1_data,
               Mux(vmlogic_insn,   vmlogic_result,
               Mux(vadc  || vsbc,  Mux(v_tail, io.req.bits.rs3_data, alu.io.out),
               Mux(vmadc || vmsbc, Cat(0.U((eLen-1).W), Mux(v_tail, io.req.bits.rs3_data(0), alu_co)),
               Mux(vmscmp,         Cat(0.U((eLen-1).W), Mux(v_tail || v_inactive, io.req.bits.rs3_data(0), alu.io.cmp_out)),
               Mux(uop.is_rvv && (uop.ctrl.is_load || uop.ctrl.is_sta) || v_inactive, io.req.bits.rs3_data,
                   alu.io.out)))))))))))
  } else {
    alu_out := Mux(uop.is_sfb_shadow && io.req.bits.pred_data, Mux(uop.ldst_is_rs1, io.req.bits.rs1_data, io.req.bits.rs2_data),
               Mux(uop.uopc === uopMOV, io.req.bits.rs2_data,
                   alu.io.out))
  }
  r_val (0) := io.req.valid
  r_data(0) := Mux(uop.is_sfb_br, pc_sel === PC_BRJMP, alu_out)
  r_pred(0) := uop.is_sfb_shadow && io.req.bits.pred_data
  for (i <- 1 until numStages) {
    r_val(i)  := r_val(i-1)
    r_data(i) := r_data(i-1)
    r_pred(i) := r_pred(i-1)
  }
  io.resp.bits.data := r_data(numStages-1)
  io.resp.bits.predicated := r_pred(numStages-1)
  // Bypass
  // for the ALU, we can bypass same cycle as compute
  require (numStages >= 1)
  require (numBypassStages >= 1)
  io.bypass(0).valid := io.req.valid
  io.bypass(0).bits.data := Mux(uop.is_sfb_br, pc_sel === PC_BRJMP, alu_out)
  for (i <- 1 until numStages) {
    io.bypass(i).valid := r_val(i-1)
    io.bypass(i).bits.data := r_data(i-1)
  }

  // Exceptions
  io.resp.bits.fflags.valid := false.B
}

/**
 * Functional unit that passes in base+imm to calculate addresses, and passes store data
 * to the LSU.
 * For floating point, 65bit FP store-data needs to be decoded into 64bit FP form
 */
class MemAddrCalcUnit(implicit p: Parameters)
  extends PipelinedFunctionalUnit(
    numStages = 0,
    numBypassStages = 0,
    earliestBypassStage = 0,
    dataWidth = 65, // TODO enable this only if FP is enabled?
    isMemAddrCalcUnit = true)
  with freechips.rocketchip.rocket.constants.MemoryOpConstants
  with freechips.rocketchip.rocket.constants.ScalarOpConstants
{
  val uop = io.req.bits.uop
  // unit stride
  val v_ls_ew = Mux(usingVector.B & uop.is_rvv, uop.v_ls_ew, 0.U)
  val uop_sew = Mux(usingVector.B & uop.is_rvv, uop.vconfig.vtype.vsew, 0.U)
  // unit stride load/store
  val vec_us_ls = usingVector.B & uop.is_rvv & uop.uopc.isOneOf(uopVL, uopVSA) // or uopVLFF
  val vec_cs_ls = usingVector.B & uop.is_rvv & uop.uopc.isOneOf(uopVLS, uopVSSA)
  val vec_idx_ls = usingVector.B & uop.is_rvv & uop.uopc.isOneOf(uopVLUX, uopVSUXA, uopVLOX, uopVSOXA)
  val op1 = io.req.bits.rs1_data.asSInt
  // TODO: optimize multiplications here
  val op2 = Mux(vec_us_ls, ((uop.vstart * uop.v_seg_nf + uop.v_seg_f) << v_ls_ew).asSInt,
            Mux(vec_cs_ls, io.req.bits.rs2_data.asSInt * uop.vstart.asUInt + Cat(0.U(1.W), uop.v_seg_f << v_ls_ew).asSInt,
            Mux(vec_idx_ls, uop.v_xls_offset.asSInt + Cat(0.U(1.W), uop.v_seg_f << uop_sew).asSInt,
                uop.imm_packed(19,8).asSInt)))

  // perform address calculation
  val sum = (op1 + op2).asUInt
  val ea_sign = Mux(sum(vaddrBits-1), ~sum(63,vaddrBits) === 0.U,
                                       sum(63,vaddrBits) =/= 0.U)
  val effective_address = Cat(ea_sign, sum(vaddrBits-1,0)).asUInt

  val store_data = io.req.bits.rs2_data

  io.resp.bits.addr := effective_address
  io.resp.bits.data := store_data

  if (dataWidth > 63) {
    assert (!(io.req.valid && uop.ctrl.is_std &&
      io.resp.bits.data(64).asBool === true.B), "65th bit set in MemAddrCalcUnit.")

    assert (!(io.req.valid && uop.ctrl.is_std && uop.fp_val),
      "FP store-data should now be going through a different unit.")
  }

  assert (!(uop.fp_val && io.req.valid && uop.uopc =/= uopLD && uop.uopc =/= uopSTA) &&
          !(uop.is_rvv && io.req.valid && !uop.uopc.isOneOf(uopVL, uopVSA, uopVLS, uopVSSA, uopVLUX, uopVSUXA, uopVLOX, uopVSOXA)),
          "[maddrcalc] assert we never get store data in here.")

  // Handle misaligned exceptions
  val size = uop.mem_size
  val misaligned =
    (size === 1.U && (effective_address(0) =/= 0.U)) ||
    (size === 2.U && (effective_address(1,0) =/= 0.U)) ||
    (size === 3.U && (effective_address(2,0) =/= 0.U))

  val bkptu = Module(new BreakpointUnit(nBreakpoints))
  bkptu.io.status   := io.status
  bkptu.io.bp       := io.bp
  bkptu.io.pc       := DontCare
  bkptu.io.ea       := effective_address
  bkptu.io.mcontext := io.mcontext
  bkptu.io.scontext := io.scontext

  val ma_ld, ma_st, dbg_bp, bp = Wire(Bool())
  if (usingVector) {
    ma_ld  := io.req.valid && uop.ctrl.is_load && misaligned
    ma_st  := io.req.valid && uop.ctrl.is_sta && misaligned
    dbg_bp := io.req.valid && ((uop.ctrl.is_load && bkptu.io.debug_ld) ||
                               (uop.ctrl.is_sta  && bkptu.io.debug_st))
    bp     := io.req.valid && ((uop.ctrl.is_load && bkptu.io.xcpt_ld) ||
                               (uop.ctrl.is_sta  && bkptu.io.xcpt_st))
  } else {
    ma_ld  := io.req.valid && uop.uopc === uopLD && misaligned
    ma_st  := io.req.valid && (uop.uopc === uopSTA || uop.uopc === uopAMO_AG) && misaligned
    dbg_bp := io.req.valid && ((uop.uopc === uopLD  && bkptu.io.debug_ld) ||
                               (uop.uopc === uopSTA && bkptu.io.debug_st))
    bp     := io.req.valid && ((uop.uopc === uopLD  && bkptu.io.xcpt_ld) ||
                               (uop.uopc === uopSTA && bkptu.io.xcpt_st))
  }

  def checkExceptions(x: Seq[(Bool, UInt)]) =
    (x.map(_._1).reduce(_||_), PriorityMux(x))
  val (xcpt_val, xcpt_cause) = checkExceptions(List(
    (ma_ld,  (Causes.misaligned_load).U),
    (ma_st,  (Causes.misaligned_store).U),
    (dbg_bp, (CSR.debugTriggerCause).U),
    (bp,     (Causes.breakpoint).U)))

  io.resp.bits.mxcpt.valid := xcpt_val
  io.resp.bits.mxcpt.bits  := xcpt_cause
  assert (!(ma_ld && ma_st), "Mutually-exclusive exceptions are firing.")

  io.resp.bits.sfence.valid := io.req.valid && uop.mem_cmd === M_SFENCE
  io.resp.bits.sfence.bits.rs1 := uop.mem_size(0)
  io.resp.bits.sfence.bits.rs2 := uop.mem_size(1)
  io.resp.bits.sfence.bits.addr := io.req.bits.rs1_data
  io.resp.bits.sfence.bits.asid := io.req.bits.rs2_data
}


/**
 * Functional unit to wrap lower level FPU
 *
 * Currently, bypassing is unsupported!
 * All FP instructions are padded out to the max latency unit for easy
 * write-port scheduling.
 */
class FPUUnit(vector: Boolean = false)(implicit p: Parameters)
  extends PipelinedFunctionalUnit(
    numStages = p(tile.TileKey).core.fpu.get.dfmaLatency,
    numBypassStages = 0,
    earliestBypassStage = 0,
    dataWidth = 65,
    needsFcsr = true)
{
  val fpu = Module(new FPU(vector = vector))
  fpu.io.req.valid         := io.req.valid
  fpu.io.req.bits.uop      := io.req.bits.uop
  fpu.io.req.bits.rs1_data := io.req.bits.rs1_data
  fpu.io.req.bits.rs2_data := io.req.bits.rs2_data
  fpu.io.req.bits.rs3_data := io.req.bits.rs3_data
  fpu.io.req.bits.fcsr_rm  := io.fcsr_rm

  io.resp.bits.data              := fpu.io.resp.bits.data
  io.resp.bits.fflags.valid      := fpu.io.resp.bits.fflags.valid
  io.resp.bits.fflags.bits.uop   := io.resp.bits.uop
  io.resp.bits.fflags.bits.flags := fpu.io.resp.bits.fflags.bits.flags // kill me now
}

/**
 * Int to FP conversion functional unit
 *
 * @param latency the amount of stages to delay by
 */
class IntToFPUnit(latency: Int, vector: Boolean = false)(implicit p: Parameters)
  extends PipelinedFunctionalUnit(
    numStages = latency,
    numBypassStages = 0,
    earliestBypassStage = 0,
    dataWidth = 65,
    needsFcsr = true)
  with tile.HasFPUParameters
{
  val fp_decoder = Module(new UOPCodeFPUDecoder(vector)) // TODO use a simpler decoder
  val io_req = io.req.bits
  fp_decoder.io.uopc := io_req.uop.uopc
  val fp_ctrl = WireInit(fp_decoder.io.sigs)
  val fp_rm = Mux(ImmGenRm(io_req.uop.imm_packed) === 7.U || io_req.uop.is_rvv, io.fcsr_rm, ImmGenRm(io_req.uop.imm_packed))
  val vd_widen = io_req.uop.rt(RD , isWidenV)
  val vs2_widen= io_req.uop.rt(RS2, isWidenV)
  if (vector) {
    val vsew = io_req.uop.vconfig.vtype.vsew
    val vd_sew  = Mux(vd_widen, vsew+1.U, vsew)
    val vs2_sew = Mux(vs2_widen, vsew+1.U, vsew)
    val vd_fmt  = Mux(vd_sew  === 3.U, D, Mux(vd_sew  === 2.U, S, H))
    val vs1_fmt = Mux(vsew    === 3.U, D, Mux(vsew    === 2.U, S, H))
    val vs2_fmt = Mux(vs2_sew === 3.U, D, Mux(vs2_sew === 2.U, S, H))
    when (io.req.valid && io_req.uop.is_rvv) {
      assert(io_req.uop.fp_val, "unexpected fp_val")
      assert(io_req.uop.v_active, "unexpected inactive split")
      assert(vsew >= 1.U && vsew <= 3.U, "unsupported vsew")
      assert(vd_sew >= 1.U && vd_sew <= 3.U, "unsupported vd_sew")
    }
   
    when (io_req.uop.is_rvv) {
      // TODO considering widening and narrowing operations
      fp_ctrl.typeTagIn := vs2_fmt
      fp_ctrl.typeTagOut:= vd_fmt
    }
  }
  val req = Wire(new tile.FPInput)
  val tag = fp_ctrl.typeTagIn

  req <> fp_ctrl

  req.rm := fp_rm
  req.in1 := Mux(fp_ctrl.swap12, unbox(io_req.rs2_data, tag, None), unbox(io_req.rs1_data, tag, None))
  req.in2 := unbox(io_req.rs2_data, tag, None)
  req.in3 := DontCare
  when(fp_ctrl.wflags) {
    req.typeTagIn := fp_ctrl.typeTagOut      // IntToFP typeTagIn, based on float width, not integer
  }
  val typ1 = Mux(tag === D, 1.U(1.W), 0.U(1.W))
  req.typ := Mux(io_req.uop.is_rvv, ImmGenTypRVV(typ1, io_req.uop.imm_packed), ImmGenTyp(io_req.uop.imm_packed))
  req.fmt := Mux(tag === H, 2.U, Mux(tag === S, 0.U, 1.U)) // TODO support Zfh and avoid special-case below
  req.fmaCmd := DontCare

  assert (!(io.req.valid && fp_ctrl.fromint && req.in1(xLen).asBool),
    "[func] IntToFP integer input has 65th high-order bit set!")

  assert (!(io.req.valid && !fp_ctrl.fromint),
    "[func] Only support fromInt micro-ops.")

  val ifpu = Module(new tile.IntToFP(latency))
  ifpu.io.in.valid := io.req.valid
  ifpu.io.in.bits := req
  ifpu.io.in.bits.in1 := Mux(fp_ctrl.swap12, io_req.rs2_data, io_req.rs1_data)
  val outTypeTag = Pipe(io.req.valid, fp_ctrl.typeTagOut, latency).bits
  val outRVV     = Pipe(io.req.valid, io_req.uop.is_rvv,  latency).bits

//io.resp.bits.data              := box(ifpu.io.out.bits.data, !io.resp.bits.uop.fp_single)
  io.resp.bits.data              := Mux(outRVV, ieee(box(ifpu.io.out.bits.data, outTypeTag)),
                                        box(ifpu.io.out.bits.data, outTypeTag))
  io.resp.bits.fflags.valid      := ifpu.io.out.valid
  io.resp.bits.fflags.bits.uop   := io.resp.bits.uop
  io.resp.bits.fflags.bits.flags := ifpu.io.out.bits.exc
}

/**
 * Iterative/unpipelined functional unit, can only hold a single MicroOp at a time
 * assumes at least one register between request and response
 *
 * TODO allow up to N micro-ops simultaneously.
 *
 * @param dataWidth width of the data to be passed into the functional unit
 */
abstract class IterativeFunctionalUnit(dataWidth: Int)(implicit p: Parameters)
  extends FunctionalUnit(
    isPipelined = false,
    numStages = 1,
    numBypassStages = 0,
    dataWidth = dataWidth)
{
  val r_uop = Reg(new MicroOp())

  val do_kill = Wire(Bool())
  do_kill := io.req.bits.kill // irrelevant default

  when (io.req.fire()) {
    // update incoming uop
    do_kill := IsKilledByBranch(io.brupdate, io.req.bits.uop) || io.req.bits.kill
    r_uop := io.req.bits.uop
    r_uop.br_mask := GetNewBrMask(io.brupdate, io.req.bits.uop)
  } .otherwise {
    do_kill := IsKilledByBranch(io.brupdate, r_uop) || io.req.bits.kill
    r_uop.br_mask := GetNewBrMask(io.brupdate, r_uop)
  }

  // assumes at least one pipeline register between request and response
  io.resp.bits.uop := r_uop
}

/**
 * Divide functional unit.
 *
 * @param dataWidth data to be passed into the functional unit
 */
class DivUnit(dataWidth: Int)(implicit p: Parameters)
  extends IterativeFunctionalUnit(dataWidth)
{

  // We don't use the iterative multiply functionality here.
  // Instead we use the PipelinedMultiplier
  val div = Module(new freechips.rocketchip.rocket.MulDiv(mulDivParams, width = dataWidth))

  // request
  div.io.req.valid    := io.req.valid && !this.do_kill
  div.io.req.bits.dw  := io.req.bits.uop.ctrl.fcn_dw
  div.io.req.bits.fn  := io.req.bits.uop.ctrl.op_fcn
  div.io.req.bits.in1 := io.req.bits.rs1_data
  div.io.req.bits.in2 := io.req.bits.rs2_data
  div.io.req.bits.tag := DontCare
  io.req.ready        := div.io.req.ready

  // handle pipeline kills and branch misspeculations
  div.io.kill         := this.do_kill

  // response
  io.resp.valid       := div.io.resp.valid && !this.do_kill
  div.io.resp.ready   := io.resp.ready
  io.resp.bits.data   := div.io.resp.bits.data
}

/**
 * Pipelined multiplier functional unit that wraps around the RocketChip pipelined multiplier
 *
 * @param numStages number of pipeline stages
 * @param dataWidth size of the data being passed into the functional unit
 */
class PipelinedMulUnit(numStages: Int, dataWidth: Int)(implicit p: Parameters)
  extends PipelinedFunctionalUnit(
    numStages = numStages,
    numBypassStages = 0,
    earliestBypassStage = 0,
    dataWidth = dataWidth)
{
  val imul = Module(new PipelinedMultiplier(xLen, numStages))
  // request
  imul.io.req.valid    := io.req.valid
  imul.io.req.bits.fn  := io.req.bits.uop.ctrl.op_fcn
  imul.io.req.bits.dw  := io.req.bits.uop.ctrl.fcn_dw
  imul.io.req.bits.in1 := io.req.bits.rs1_data
  imul.io.req.bits.in2 := io.req.bits.rs2_data
  imul.io.req.bits.tag := DontCare
  // response
  io.resp.bits.data    := imul.io.resp.bits.data
}

/**
 * Pipelined multiply-accumulator functional unit that wraps around the RocketChip pipelined multiplier
 *
 * @param numStages number of pipeline stages
 * @param dataWidth size of the data being passed into the functional unit
 */
class IntMulAcc(numStages: Int, dataWidth: Int)(implicit p: Parameters)
  extends PipelinedFunctionalUnit(
    numStages = numStages,
    numBypassStages = 0,
    earliestBypassStage = 0,
    dataWidth = dataWidth
) {
  val in_req = WireInit(io.req)
  when (io.req.bits.uop.uopc.isOneOf(uopVMADD, uopVNMSUB)) {
    in_req.bits.rs2_data := io.req.bits.rs3_data
    in_req.bits.rs3_data := io.req.bits.rs2_data
  }
  when (io.req.bits.uop.uopc.isOneOf(uopVNMSAC, uopVNMSUB)) {
    in_req.bits.rs1_data := ~io.req.bits.rs1_data +& 1.U
  }
  val in = Pipe(in_req.valid, in_req.bits)

  val decode = List(
    FN_MUL    -> List(N, X, X, N, N),
    FN_MULH   -> List(Y, Y, Y, N, N),
    FN_MULHU  -> List(Y, N, N, N, N),
    FN_MULHSU -> List(Y, Y, N, N, N),
    FN_MULU   -> List(N, N, N, N, N),
    FN_MULSU  -> List(N, Y, N, N, N),
    FN_MACC   -> List(N, Y, Y, Y, Y),
    FN_MACCU  -> List(N, N, N, Y, N),
    FN_MACCSU -> List(N, Y, N, Y, N),
    FN_MACCUS -> List(N, N, Y, Y, N),
    )
  val cmdHi :: lhsSigned :: rhsSigned :: doAcc :: accSigned :: Nil =
    DecodeLogic(in.bits.uop.ctrl.op_fcn, List(X, X, X, X, X), decode).map(_.asBool)

  val lhs = Cat(lhsSigned && in.bits.rs1_data(dataWidth-1), in.bits.rs1_data).asSInt
  val rhs = Cat(rhsSigned && in.bits.rs2_data(dataWidth-1), in.bits.rs2_data).asSInt
  val acc = Mux(doAcc, Cat(accSigned && in.bits.rs3_data(dataWidth-1), in.bits.rs3_data).asSInt, 0.S)
  when (in.valid && doAcc) { assert(in.bits.uop.frs3_en, "Incorrect frs3_en") }
  val macc = lhs * rhs +& acc
  val vd_eew = in.bits.uop.vd_eew
  val unsigned = in.bits.uop.rt(RD, isUnsignedV)
  val hi  = Mux1H(UIntToOH(vd_eew(1,0)), Seq(Cat(Mux(unsigned, 0.U((dataWidth-8).W),  Fill(dataWidth-8,  macc(15))), macc(15,  8)),
                                   Cat(Mux(unsigned, 0.U((dataWidth-16).W), Fill(dataWidth-16, macc(31))), macc(31, 16)),
                                   Cat(Mux(unsigned, 0.U((dataWidth-32).W), Fill(dataWidth-32, macc(63))), macc(63, 32)),
                                   macc(127, 64)))
  val muxed = Mux(cmdHi, hi, macc(dataWidth-1, 0))

  val resp = Pipe(in, numStages-1)
  io.resp.valid := resp.valid
  io.resp.bits.data := Pipe(in.valid, muxed, numStages-1).bits
}

/**
  * Pipelined vmask computing functional unit that wraps around the VMaskUnit
  *
  * @param numStages number of pipeline stages
  * @param dataWidth size of the data being passed into the functional unit
  */
class PipelinedVMaskUnit(numStages: Int, dataWidth: Int)(implicit p: Parameters)
  extends PipelinedFunctionalUnit(
    numStages = 0,
    numBypassStages = 0,
    earliestBypassStage = 0,
    dataWidth = dataWidth)
{
  val uop = io.req.bits.uop
  val vl = uop.vconfig.vl

  // immediate generation
  val rs1_data = io.req.bits.rs1_data
  val rs2_data = io.req.bits.rs2_data

  val vmaskUnit = Module(new VMaskUnit())

  val byteWidth = 3.U
  val vsew64bit = 3.U
  val is_multiple_of_64 = vl(5,0) === 0.U
  val vmaskInsn_vl = vl(5,0).orR +& (vl>>(byteWidth +& vsew64bit))
  val is_vmaskInsn_last_split = uop.vstart === (vmaskInsn_vl-1.U)
  val vmaskInsn_mask = boom.util.MaskGen(0.U, vl(5,0), 64)
  val vmaskInsn_rs2_data = Mux(is_vmaskInsn_last_split & (~is_multiple_of_64), (rs2_data & vmaskInsn_mask), rs2_data)

  // operand 1 select
  var op1_data: UInt = null
  op1_data = Mux(uop.ctrl.op1_sel.asUInt === OP1_RS1 , rs1_data,
             Mux(uop.ctrl.op1_sel.asUInt === OP1_VS2 , vmaskInsn_rs2_data,
                 0.U))

  val init_popc = Mux(uop.vstart === 0.U, 0.U, RegNext(vmaskUnit.io.out))
  val init_first_idx = Mux(uop.vstart === 0.U, 0.U, 64.U)

  // operand 2 select
  val op2_data = WireInit(0.U(xLen.W))
  op2_data:= Mux(uop.ctrl.op2_sel === OP2_RS2 , rs2_data,
             Mux(uop.uopc.isOneOf(uopVPOPC), init_popc,
             Mux(uop.uopc.isOneOf(uopVFIRST), init_first_idx,
             Mux(uop.ctrl.op2_sel === OP2_VS1,  rs1_data, 0.U))))

  vmaskUnit.io.in := op1_data.asUInt
  vmaskUnit.io.in_addend := op2_data.asUInt
  vmaskUnit.io.fn  := uop.ctrl.op_fcn

  val is_0_op_num = (~vmaskInsn_mask & op1_data) === 0.U
  val firstIdx_result = Mux(is_vmaskInsn_last_split & is_0_op_num , ~0.U(xLen.W), vmaskUnit.io.out)

  val vmaskUnit_out = Mux(uop.uopc.isOneOf(uopVFIRST), firstIdx_result, vmaskUnit.io.out)

  // vl => last
  io.resp.bits.data := vmaskUnit_out
}
