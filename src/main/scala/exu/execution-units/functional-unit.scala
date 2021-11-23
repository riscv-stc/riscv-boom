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
import freechips.rocketchip.tile.FPConstants._
import freechips.rocketchip.tile.{FPUCtrlSigs, HasFPUParameters}
import freechips.rocketchip.tile
import freechips.rocketchip.rocket
import freechips.rocketchip.rocket.{DecodeLogic,PipelinedMultiplier,BP,BreakpointUnit,Causes,CSR,VConfig,VType}

import FUConstants._
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
  val FUC_SZ = 14
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
  val FU_FR7_ID = 12  // vfrsqrt7 / vfrec7
  val FU_VMASKU_ID = 13
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
  val FU_FR7 = (1<<FU_FR7_ID).U(FUC_SZ.W)
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
  val fr7:  Boolean = false,
  val vmx:  Boolean = false,
  val vector: Boolean = false)
{
}


/**
 * Bundle for signals sent to the functional unit
 *
 * @param dataWidth width of the data sent to the functional unit
 */
class FuncUnitReq(val dataWidth: Int, val vector: Boolean = false)(implicit p: Parameters) extends BoomBundle
  with HasBoomUOP
{
  val numOperands = 3

  val rs1_data = UInt(dataWidth.W)
  val rs2_data = UInt(dataWidth.W)
  val rs3_data = UInt(dataWidth.W) // used for FMA, vector units
  val rvm_data = UInt((dataWidth/8).W)
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
  val isFixMulAcc: Boolean = false,
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

    // only used by the Fixed unit
    val vxrm    = if (isFixMulAcc) Input(UInt(2.W)) else null

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
  isFixMulAcc: Boolean = false,
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
    isFixMulAcc = isFixMulAcc,
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
  dataWidth: Int
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
               Mux(uop.ctrl.op1_sel.asUInt === OP1_PC  , Sext(uop_pc, xLen), 0.U))
  } else {
    op1_data = Mux(uop.ctrl.op1_sel.asUInt === OP1_RS1 , io.req.bits.rs1_data, 0.U)
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
               Mux(uop.ctrl.op2_sel === OP2_NEXT,    Mux(uop.is_rvc, 2.U, 4.U),
               Mux(uop.ctrl.op2_sel === OP2_VL,      new_vl, 0.U)))))

    val set_vconfig = vsetvl | vsetvli | vsetivli
    io.resp.bits.uop.vconfig.vl := RegEnable(new_vl, set_vconfig)
    io.resp.bits.uop.vconfig.vtype := RegEnable(VType.fromUInt(vtypei), set_vconfig)
  } else {
    op2_data:= Mux(uop.ctrl.op2_sel === OP2_IMM,     Sext(imm_xprlen.asUInt, xLen),
               Mux(uop.ctrl.op2_sel === OP2_IMMC,    uop.prs1(4,0),
               Mux(uop.ctrl.op2_sel === OP2_RS2 ,    io.req.bits.rs2_data,
               Mux(uop.ctrl.op2_sel === OP2_NEXT,    Mux(uop.is_rvc, 2.U, 4.U), 0.U))))
  }

  val alu = Module(new freechips.rocketchip.rocket.ALU(withCarryIO = false))

  alu.io.in1 := op1_data.asUInt
  alu.io.in2 := op2_data.asUInt
  alu.io.fn  := uop.ctrl.op_fcn
  alu.io.dw  := uop.ctrl.fcn_dw

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
  alu_out := Mux(uop.is_sfb_shadow && io.req.bits.pred_data, Mux(uop.ldst_is_rs1, io.req.bits.rs1_data, io.req.bits.rs2_data),
             Mux(uop.uopc === uopMOV, io.req.bits.rs2_data, alu.io.out))
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
  val op1 = io.req.bits.rs1_data.asSInt
  val op2 = WireInit(uop.imm_packed(19,8).asSInt)
  // unit stride
  if (usingVector) {
    val v_ls_ew = Mux(usingVector.B & uop.is_rvv, uop.v_ls_ew, 0.U)
    val uop_sew = Mux(usingVector.B & uop.is_rvv, uop.vconfig.vtype.vsew, 0.U)
    // unit stride load/store
    val vec_us_ls = usingVector.B & uop.is_rvv & uop.uopc.isOneOf(uopVL, uopVLFF, uopVSA) // or uopVLFF
    val vec_cs_ls = usingVector.B & uop.is_rvv & uop.uopc.isOneOf(uopVLS, uopVSSA)
    val vec_idx_ls = usingVector.B & uop.is_rvv & uop.uopc.isOneOf(uopVLUX, uopVSUXA, uopVLOX, uopVSOXA)
    // TODO: optimize multiplications here
    op2 := Mux(vec_us_ls, ((uop.v_eidx * uop.v_seg_nf + uop.v_seg_f) << v_ls_ew).asSInt,
           Mux(vec_cs_ls, io.req.bits.rs2_data.asSInt * uop.v_eidx.asUInt + Cat(0.U(1.W), uop.v_seg_f << v_ls_ew).asSInt,
           Mux(vec_idx_ls, uop.v_xls_offset.asSInt + Cat(0.U(1.W), uop.v_seg_f << uop_sew).asSInt,
           uop.imm_packed(19,8).asSInt)))
  }

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

  assert (!(uop.fp_val && io.req.valid && uop.uopc =/= uopLD && uop.uopc =/= uopSTA),
          "[maddrcalc] assert we never get store data in here.")

  if (usingVector) {
    assert (!(uop.is_rvv && io.req.valid && !uop.uopc.isOneOf(uopVL, uopVLFF, uopVSA, uopVLS, uopVSSA, uopVLUX, uopVSUXA, uopVLOX, uopVSOXA)))
  }

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
  val fp_rm = WireInit(io.fcsr_rm)
  fp_rm := Mux(ImmGenRm(io_req.uop.imm_packed) === 7.U, io.fcsr_rm, ImmGenRm(io_req.uop.imm_packed))
  val vd_widen = io_req.uop.rt(RD , isWidenV)
  val vs2_widen= io_req.uop.rt(RS2, isWidenV)
  if (vector) {
    val vsew = io_req.uop.vconfig.vtype.vsew
    val vd_sew  = io_req.uop.vd_eew
    val vs2_sew = io_req.uop.vs2_eew
    val vd_fmt  = Mux(vd_sew  === 3.U, D, Mux(vd_sew  === 2.U, S, H))
    val vs1_fmt = Mux(vsew    === 3.U, D, Mux(vsew    === 2.U, S, H))
    val vs2_fmt = Mux(vs2_sew === 3.U, D, Mux(vs2_sew === 2.U, S, H))
    when (io.req.valid && io_req.uop.is_rvv) {
      assert(io_req.uop.fp_val, "unexpected fp_val")
      assert(io_req.uop.v_active, "unexpected inactive split")
      assert(vsew <= 3.U, "unsupported vsew")
      assert(vd_sew >= 1.U && vd_sew <= 3.U, "unsupported vd_sew")
    }

    when (io_req.uop.is_rvv) {
      // TODO considering widening and narrowing operations
      fp_rm := io.fcsr_rm
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
  if (vector) {
    when (io.req.valid) {
      assert(io_req.uop.is_rvv)
    }
    req.typ := ImmGenTypRVV(typ1, io_req.uop.imm_packed)
  } else {
    req.typ := ImmGenTyp(io_req.uop.imm_packed)
  }
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
  val outRVV     = if (vector) Pipe(io.req.valid, io_req.uop.is_rvv,  latency).bits else false.B

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
  if(usingVector) {
    when(io.req.bits.uop.is_rvv && !io.req.bits.uop.v_active) {
      div.io.req.bits.in1 := io.req.bits.rs3_data
      div.io.req.bits.in2 := 1.U
    } .elsewhen(io.req.bits.uop.is_rvv) {
      div.io.req.bits.in1 := io.req.bits.rs2_data
      div.io.req.bits.in2 := io.req.bits.rs1_data
    }
  }
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

class FixMulAccCtrlSigs extends Bundle
{
  val cmdHi     = Bool() // use Hi part
  val lhsSigned = Bool() // lhs of mul is signed
  val rhsSigned = Bool() // rhs of mul is signed
  val doAcc     = Bool() // do accumulation after mul
  val negAcc    = Bool() // acc is subtraction
  val accSigned = Bool() // addend is signed
  val lhsOne    = Bool() // rhs of mul is one
  val doRO      = Bool() // need rounding
  val roSigned  = Bool() // do signed rounding
  val srType    = UInt(2.W) // shift right Type: 0, 1, SEW-1, RS1
  val doClip    = Bool() // do Clipping between high/low limits

  def decoder(uopc: UInt, table: Iterable[(BitPat, List[BitPat])]) = {
    val DC2     = BitPat.dontCare(2)
    val decoder = DecodeLogic(uopc, List(X, X, X, X, X, X, X, X, X, DC2, X), table)
    val sigs    = Seq(cmdHi, lhsSigned, rhsSigned, doAcc, negAcc, accSigned, lhsOne, doRO, roSigned, srType, doClip)
    sigs zip decoder map {case(s,d) => s := d}
    this
  }
}

/**
 * Pipelined fixed-point multiply-accumulator functional unit
 *
 * @param numStages number of pipeline stages
 * @param dataWidth size of the data being passed into the functional unit
 */
class FixMulAcc(numStages: Int, dataWidth: Int)(implicit p: Parameters)
  extends PipelinedFunctionalUnit (
    numStages = numStages,
    numBypassStages = 0,
    earliestBypassStage = 0,
    dataWidth = dataWidth,
    isFixMulAcc = true
) with ShouldBeRetimed {
  val e8 = (dataWidth == 8)
  val e16= (dataWidth == 16)
  val e32= (dataWidth == 32)
  val e64= (dataWidth == 64)
  val in_req = WireInit(io.req)
  val in = Pipe(in_req.valid, in_req.bits)
  val io_vs1_eew  = io.req.bits.uop.vs1_eew
  val io_vs2_eew  = io.req.bits.uop.vs2_eew
  val io_vd_eew   = io.req.bits.uop.vd_eew
  val io_unsigned = io.req.bits.uop.rt(RD, isUnsignedV)
  val u_max = Wire(UInt((dataWidth+1).W))
  val hi, s_max, s_min, rs1_data, rs2_data, rs3_data = Wire(UInt(dataWidth.W))
  val stale_rs3_data = Pipe(in_req.valid, rs3_data).bits
  if (e64) {
    rs1_data := Mux1H(UIntToOH(io_vs1_eew(1,0)), Seq(Mux(io_unsigned, io.req.bits.rs1_data(7,0),  io.req.bits.rs1_data(7,0).sextTo(dataWidth)),
                                                     Mux(io_unsigned, io.req.bits.rs1_data(15,0), io.req.bits.rs1_data(15,0).sextTo(dataWidth)),
                                                     Mux(io_unsigned, io.req.bits.rs1_data(31,0), io.req.bits.rs1_data(31,0).sextTo(dataWidth)),
                                                     io.req.bits.rs1_data))
    rs2_data := Mux1H(UIntToOH(io_vs2_eew(1,0)), Seq(Mux(io_unsigned, io.req.bits.rs2_data(7,0),  io.req.bits.rs2_data(7,0).sextTo(dataWidth)),
                                                     Mux(io_unsigned, io.req.bits.rs2_data(15,0), io.req.bits.rs2_data(15,0).sextTo(dataWidth)),
                                                     Mux(io_unsigned, io.req.bits.rs2_data(31,0), io.req.bits.rs2_data(31,0).sextTo(dataWidth)),
                                                     io.req.bits.rs2_data))
    rs3_data := Mux1H(UIntToOH(io_vd_eew(1,0)), Seq(Mux(io_unsigned, io.req.bits.rs3_data(7,0),  io.req.bits.rs3_data(7,0).sextTo(dataWidth)),
                                                    Mux(io_unsigned, io.req.bits.rs3_data(15,0), io.req.bits.rs3_data(15,0).sextTo(dataWidth)),
                                                    Mux(io_unsigned, io.req.bits.rs3_data(31,0), io.req.bits.rs3_data(31,0).sextTo(dataWidth)),
                                                    io.req.bits.rs3_data))
  } else if (e32) {
    rs1_data := Mux1H(UIntToOH(io_vs1_eew(1,0)), Seq(Mux(io_unsigned, io.req.bits.rs1_data(7,0),  io.req.bits.rs1_data(7,0).sextTo(dataWidth)),
                                                     Mux(io_unsigned, io.req.bits.rs1_data(15,0), io.req.bits.rs1_data(15,0).sextTo(dataWidth)),
                                                     io.req.bits.rs1_data,
                                                     io.req.bits.rs1_data))
    rs2_data := Mux1H(UIntToOH(io_vs2_eew(1,0)), Seq(Mux(io_unsigned, io.req.bits.rs2_data(7,0),  io.req.bits.rs2_data(7,0).sextTo(dataWidth)),
                                                     Mux(io_unsigned, io.req.bits.rs2_data(15,0), io.req.bits.rs2_data(15,0).sextTo(dataWidth)),
                                                     io.req.bits.rs2_data,
                                                     io.req.bits.rs2_data))
    rs3_data := Mux1H(UIntToOH(io_vd_eew(1,0)), Seq(Mux(io_unsigned, io.req.bits.rs3_data(7,0),  io.req.bits.rs3_data(7,0).sextTo(dataWidth)),
                                                    Mux(io_unsigned, io.req.bits.rs3_data(15,0), io.req.bits.rs3_data(15,0).sextTo(dataWidth)),
                                                    io.req.bits.rs3_data,
                                                    io.req.bits.rs3_data))
  } else if (e16) {
    rs1_data := Mux1H(UIntToOH(io_vs1_eew(0)), Seq(Mux(io_unsigned, io.req.bits.rs1_data(7,0),  io.req.bits.rs1_data(7,0).sextTo(dataWidth)),
                                                   io.req.bits.rs1_data))
    rs2_data := Mux1H(UIntToOH(io_vs2_eew(0)), Seq(Mux(io_unsigned, io.req.bits.rs2_data(7,0),  io.req.bits.rs2_data(7,0).sextTo(dataWidth)),
                                                   io.req.bits.rs2_data))
    rs3_data := Mux1H(UIntToOH(io_vd_eew(0)), Seq(Mux(io_unsigned, io.req.bits.rs3_data(7,0),  io.req.bits.rs3_data(7,0).sextTo(dataWidth)),
                                                  io.req.bits.rs3_data))
  } else { // e8
    rs1_data := io.req.bits.rs1_data
    rs2_data := io.req.bits.rs2_data
    rs3_data := io.req.bits.rs3_data
  }

  in_req.bits.rs1_data := rs1_data
  in_req.bits.rs2_data := rs2_data
  in_req.bits.rs3_data := rs3_data
  when (io.req.bits.uop.uopc.isOneOf(uopVMADD, uopVNMSUB)) {
    in_req.bits.rs2_data := rs3_data
    in_req.bits.rs3_data := rs2_data
  }
  when (io.req.bits.uop.uopc.isOneOf(uopVNMSAC, uopVNMSUB)) {
    in_req.bits.rs1_data := ~rs1_data + 1.U
  }
  when (io.req.bits.uop.uopc.isOneOf(uopVSADDU, uopVSADD, uopVAADDU, uopVAADD, uopVSSUBU, uopVSSUB, uopVASUBU, uopVASUB)) {
    in_req.bits.rs3_data := rs1_data
  }
  val DC2 = BitPat.dontCare(2)
  val table: List[(BitPat, List[BitPat])] = List(
    //                          cmdHi   negAcc accSigned   srType (0, 1, SEW-1, RS1)
    //                          |  lhsSigned|  |  lhsOne   |    doClip
    //                          |  |  rhsSigned|  doRO  |    |
    //                          |  |  |  doAcc | |  |  roSigned|
    //                          |  |  |  |  |  |  |  |  |  |    |
    BitPat(uopVMUL)     -> List(N, Y, Y, N, X, X, N, N, X, DC2, N)
   ,BitPat(uopVMULH)    -> List(Y, Y, Y, N, X, X, N, N, X, DC2, N)
   ,BitPat(uopVMULHU)   -> List(Y, N, N, N, X, X, N, N, X, DC2, N)
   ,BitPat(uopVMULHSU)  -> List(Y, Y, N, N, X, X, N, N, X, DC2, N)
   ,BitPat(uopVWMULU)   -> List(N, N, N, N, X, X, N, N, X, DC2, N)
   ,BitPat(uopVWMULSU)  -> List(N, Y, N, N, X, X, N, N, X, DC2, N)
   ,BitPat(uopVMACC)    -> List(N, Y, Y, Y, N, Y, N, N, X, DC2, N)
   ,BitPat(uopVNMSAC)   -> List(N, Y, Y, Y, N, Y, N, N, X, DC2, N)
   ,BitPat(uopVMADD)    -> List(N, Y, Y, Y, N, Y, N, N, X, DC2, N)
   ,BitPat(uopVNMSUB)   -> List(N, Y, Y, Y, N, Y, N, N, X, DC2, N)
   ,BitPat(uopVWMACCU)  -> List(N, N, N, Y, N, N, N, N, X, DC2, N)
   ,BitPat(uopVWMACCSU) -> List(N, Y, N, Y, N, Y, N, N, X, DC2, N)
   ,BitPat(uopVWMACCUS) -> List(N, N, Y, Y, N, Y, N, N, X, DC2, N)
   ,BitPat(uopVSADDU)   -> List(N, X, N, Y, N, N, Y, Y, N, U_0, Y) // vs2 + rs1
   ,BitPat(uopVSADD)    -> List(N, X, Y, Y, N, Y, Y, Y, Y, U_0, Y)
   ,BitPat(uopVSSUBU)   -> List(N, X, N, Y, Y, N, Y, Y, N, U_0, Y)
   ,BitPat(uopVSSUB)    -> List(N, X, Y, Y, Y, Y, Y, Y, Y, U_0, Y)
   ,BitPat(uopVAADDU)   -> List(N, X, N, Y, N, N, Y, Y, N, U_1, N)
   ,BitPat(uopVAADD)    -> List(N, X, Y, Y, N, Y, Y, Y, Y, U_1, N)
   ,BitPat(uopVASUBU)   -> List(N, X, N, Y, Y, N, Y, Y, N, U_1, N)
   ,BitPat(uopVASUB)    -> List(N, X, Y, Y, Y, Y, Y, Y, Y, U_1, N)
   ,BitPat(uopVSMUL)    -> List(N, Y, Y, N, X, X, N, Y, Y, U_2, Y)
   ,BitPat(uopVSSRL)    -> List(N, X, N, N, X, X, Y, Y, N, U_3, N)
   ,BitPat(uopVSSRA)    -> List(N, X, Y, N, X, X, Y, Y, Y, U_3, N)
   ,BitPat(uopVNCLIPU)  -> List(N, X, N, N, X, X, Y, Y, N, U_3, Y)
   ,BitPat(uopVNCLIP)   -> List(N, X, Y, N, X, X, Y, Y, Y, U_3, Y)
  )
  val cs = Wire(new FixMulAccCtrlSigs()).decoder(in.bits.uop.uopc, table)

  val lhs = Mux(cs.lhsOne, 1.S, Cat(cs.lhsSigned && in.bits.rs1_data(dataWidth-1), in.bits.rs1_data).asSInt)
  val rhs = Cat(cs.rhsSigned && in.bits.rs2_data(dataWidth-1), in.bits.rs2_data).asSInt
  val sext_rs3 = Cat(cs.accSigned && in.bits.rs3_data(dataWidth-1), in.bits.rs3_data)
  val sub_rs3  = (~sext_rs3 + 1.U(1.W)).asSInt
  val acc = Mux(cs.doAcc, Mux(cs.negAcc, sub_rs3, sext_rs3.asSInt), 0.S)
  //when (in.valid && cs.doAcc) { assert(in.bits.uop.frs3_en, "Incorrect frs3_en") }
  val macc = lhs * rhs +& acc
  val macc_msk = ~(0.U((dataWidth*2+3).W))
  val vd_eew = in.bits.uop.vd_eew
  val unsigned = in.bits.uop.rt(RD, isUnsignedV)
  if (e64) {
    u_max := Cat(0.U(1.W), Mux1H(UIntToOH(vd_eew(1,0)), Seq(0xFF.U(dataWidth.W),
                                                            0xFFFF.U(dataWidth.W),
                                                            0xFFFFFFFFL.U(dataWidth.W),
                                                            Fill(dataWidth, 1.U(1.W)))))
    s_max := Mux1H(UIntToOH(vd_eew(1,0)), Seq(0x7F.U(dataWidth.W),
                                              0x7FFF.U(dataWidth.W),
                                              0x7FFFFFFF.U(dataWidth.W),
                                              0x7FFFFFFFFFFFFFFFL.U(dataWidth.W)))
    s_min := Mux1H(UIntToOH(vd_eew(1,0)), Seq(Cat(Fill(dataWidth-8,  1.U(1.W)), 0x80.U(8.W)),
                                              Cat(Fill(dataWidth-16, 1.U(1.W)), 0x8000.U(16.W)),
                                              Cat(Fill(dataWidth-32, 1.U(1.W)), 0x80000000L.U(32.W)),
                                              Cat(1.U(1.W), Fill(dataWidth-1, 0.U(1.W)))))
    hi  := Mux1H(UIntToOH(vd_eew(1,0)), Seq(Cat(Mux(unsigned, 0.U((dataWidth-8).W),  Fill(dataWidth-8,  macc(15))), macc(15,  8)),
                                            Cat(Mux(unsigned, 0.U((dataWidth-16).W), Fill(dataWidth-16, macc(31))), macc(31, 16)),
                                            Cat(Mux(unsigned, 0.U((dataWidth-32).W), Fill(dataWidth-32, macc(63))), macc(63, 32)),
                                            macc(127, 64)))
  } else if (e32) {
    u_max := Cat(0.U(1.W), Mux1H(UIntToOH(vd_eew(1,0)), Seq(0xFF.U(dataWidth.W),
                                                            0xFFFF.U(dataWidth.W),
                                                            Fill(dataWidth, 1.U(1.W)),
                                                            Fill(dataWidth, 1.U(1.W)))))
    s_max := Mux1H(UIntToOH(vd_eew(1,0)), Seq(0x7F.U(dataWidth.W),
                                              0x7FFF.U(dataWidth.W),
                                              0x7FFFFFFF.U(dataWidth.W),
                                              0x7FFFFFFF.U(dataWidth.W)))
    s_min := Mux1H(UIntToOH(vd_eew(1,0)), Seq(Cat(Fill(dataWidth-8,  1.U(1.W)), 0x80.U(8.W)),
                                              Cat(Fill(dataWidth-16, 1.U(1.W)), 0x8000.U(16.W)),
                                              Cat(1.U(1.W), Fill(dataWidth-1, 0.U(1.W))),
                                              Cat(1.U(1.W), Fill(dataWidth-1, 0.U(1.W)))))
    hi  := Mux1H(UIntToOH(vd_eew(1,0)), Seq(Cat(Mux(unsigned, 0.U((dataWidth-8).W),  Fill(dataWidth-8,  macc(15))), macc(15,  8)),
                                            Cat(Mux(unsigned, 0.U((dataWidth-16).W), Fill(dataWidth-16, macc(31))), macc(31, 16)),
                                            macc(63, 32),
                                            macc(63, 32)))
  } else if (e16) {
    u_max := Cat(0.U(1.W), Mux1H(UIntToOH(vd_eew(0)), Seq(0xFF.U(dataWidth.W),
                                                          Fill(dataWidth, 1.U(1.W)))))
    s_max := Mux1H(UIntToOH(vd_eew(0)), Seq(0x7F.U(dataWidth.W),
                                            0x7FFF.U(dataWidth.W)))
    s_min := Mux1H(UIntToOH(vd_eew(0)), Seq(Cat(Fill(dataWidth-8,  1.U(1.W)), 0x80.U(8.W)),
                                            Cat(1.U(1.W), Fill(dataWidth-1, 0.U(1.W)))))
    hi  := Mux1H(UIntToOH(vd_eew(0)), Seq(Cat(Mux(unsigned, 0.U((dataWidth-8).W),  Fill(dataWidth-8,  macc(15))), macc(15,  8)),
                                          macc(32, 16)))
  } else { // e8
    u_max := Fill(dataWidth, 1.U(1.W))
    s_max := 0x7F.U(dataWidth.W)
    s_min := Cat(1.U(1.W), Fill(dataWidth-1, 0.U(1.W)))
    hi  := macc(16, 8)
  }
  val sramt = Mux(!cs.doRO, 0.U,
              Mux1H(UIntToOH(cs.srType), Seq(0.U(6.W),
                                         1.U(6.W),
                                         ((8.U << vd_eew) - 1.U)(5,0),
                                         Mux1H(UIntToOH(vd_eew(1,0)), Seq(in.bits.rs1_data(2,0),
                                                                          in.bits.rs1_data(3,0),
                                                                          in.bits.rs1_data(4,0),
                                                                          in.bits.rs1_data(5,0))))))
  val srres = macc >> sramt
  val sroff = (macc.asUInt & ~(macc_msk << sramt))(dataWidth-1, 0)
  val sroff_msb = Mux(sramt === 0.U, 0.U, macc(sramt-1.U))
  val sroff_rem = Mux(sramt <=  1.U, 0.U, sroff & ~(macc_msk << (sramt-1.U)))
  val roinc = Mux1H(UIntToOH(io.vxrm(1,0)), Seq(sroff_msb,
                                                sroff_msb & (sroff_rem.orR | srres(0)),
                                                0.U(1.W),
                                                ~srres(0) & sroff.orR))
  val rores = srres + Cat(0.U, roinc).asSInt
  val uclip_max = rores > u_max.asSInt
  val uclip_min = rores < 0.S
  val uclip = Mux(uclip_max, u_max, Mux(uclip_min, 0.U, rores.asUInt))
  val sclip_max = rores > s_max.asSInt
  val sclip_min = rores < s_min.asSInt
  val sclip = Mux(sclip_max, s_max, Mux(sclip_min, s_min, rores.asUInt))
  val muxed = Mux(cs.cmdHi,   hi,
              Mux(cs.doClip,  Mux(cs.roSigned, sclip(dataWidth-1,0), uclip(dataWidth-1,0)),
              Mux(cs.doRO,    rores(dataWidth-1,0),
                              macc(dataWidth-1, 0))))

  val out  = WireInit(in)
  out.bits.uop.vxsat := cs.doClip && Mux(cs.roSigned, sclip_max || sclip_min, uclip_max || uclip_min)
  val resp = Pipe(out, numStages-1)
  io.resp.valid := resp.valid
  io.resp.bits.uop.vxsat := resp.bits.uop.vxsat
  val active = (out.bits.uop.v_unmasked || out.bits.rvm_data(0)) && out.bits.uop.v_eidx < out.bits.uop.vconfig.vl
  io.resp.bits.data := Pipe(out.valid, Mux(active, muxed, stale_rs3_data), numStages-1).bits
}

class VecFixUnit(numStages: Int, dataWidth: Int)(implicit p: Parameters)
  extends PipelinedFunctionalUnit(
    numStages = numStages,
    numBypassStages = 0,
    earliestBypassStage = 0,
    dataWidth = dataWidth,
    isFixMulAcc = true)
{
  // FIXME: optimize me by merge fix-point decoders among different FixMulAcc
  // FIXME: mute corresponding FixMulAcc by mask to save power
  val uop = io.req.bits.uop
  val rs1_data = io.req.bits.rs1_data
  val rs2_data = io.req.bits.rs2_data
  val rs3_data = io.req.bits.rs3_data
  val rvm_data = io.req.bits.rvm_data
  val xma = (0 until vLenb).map(e => {
      val dw = if (e < vLen/64) {64}
               else if (e < vLen/32) {32}
               else if (e < vLen/16) {16}
               else {8}
      Module(new FixMulAcc(numStages, dw))
  })
  for (e <- 0 until vLenb) {

    if (e < vLen/64) {
      xma(e).io.req.valid := io.req.valid
      xma(e).io.req.bits.rs1_data := Mux1H(UIntToOH(uop.vs1_eew), Seq(rs1_data(8*e+7, 8*e), rs1_data(16*e+15, 16*e), rs1_data(32*e+31, 32*e), rs1_data(64*e+63, 64*e)))
      xma(e).io.req.bits.rs2_data := Mux1H(UIntToOH(uop.vs2_eew), Seq(rs2_data(8*e+7, 8*e), rs2_data(16*e+15, 16*e), rs2_data(32*e+31, 32*e), rs2_data(64*e+63, 64*e)))
      xma(e).io.req.bits.rs3_data := Mux1H(UIntToOH(uop.vd_eew),  Seq(rs3_data(8*e+7, 8*e), rs3_data(16*e+15, 16*e), rs3_data(32*e+31, 32*e), rs3_data(64*e+63, 64*e)))
    } else if (e < vLen/32) {
      xma(e).io.req.valid := io.req.valid && (uop.vd_eew < 3.U)
      xma(e).io.req.bits.rs1_data := Mux1H(UIntToOH(uop.vs1_eew), Seq(rs1_data(8*e+7, 8*e), rs1_data(16*e+15, 16*e), rs1_data(32*e+31, 32*e), 0.U))
      xma(e).io.req.bits.rs2_data := Mux1H(UIntToOH(uop.vs2_eew), Seq(rs2_data(8*e+7, 8*e), rs2_data(16*e+15, 16*e), rs2_data(32*e+31, 32*e), 0.U))
      xma(e).io.req.bits.rs3_data := Mux1H(UIntToOH(uop.vd_eew),  Seq(rs3_data(8*e+7, 8*e), rs3_data(16*e+15, 16*e), rs3_data(32*e+31, 32*e), 0.U))
    } else if (e < vLen/16) {
      xma(e).io.req.valid := io.req.valid && (uop.vd_eew < 2.U)
      xma(e).io.req.bits.rs1_data := Mux1H(UIntToOH(uop.vs1_eew(0)), Seq(rs1_data(8*e+7, 8*e), rs1_data(16*e+15, 16*e)))
      xma(e).io.req.bits.rs2_data := Mux1H(UIntToOH(uop.vs2_eew(0)), Seq(rs2_data(8*e+7, 8*e), rs2_data(16*e+15, 16*e)))
      xma(e).io.req.bits.rs3_data := Mux1H(UIntToOH(uop.vd_eew(0)),  Seq(rs3_data(8*e+7, 8*e), rs3_data(16*e+15, 16*e)))
    } else {
      xma(e).io.req.valid := io.req.valid && (uop.vd_eew === 0.U)
      xma(e).io.req.bits.rs1_data := rs1_data(8*e+7, 8*e)
      xma(e).io.req.bits.rs2_data := rs2_data(8*e+7, 8*e)
      xma(e).io.req.bits.rs3_data := rs3_data(8*e+7, 8*e)
    }
    xma(e).io.vxrm := io.vxrm
    xma(e).io.req.bits.uop        := uop
    xma(e).io.req.bits.uop.v_eidx := uop.v_eidx + e.U
    xma(e).io.req.bits.rvm_data   := rvm_data
    xma(e).io.req.bits.pred_data  := io.req.bits.pred_data
    xma(e).io.req.bits.kill       := io.req.bits.kill
    xma(e).io.brupdate            := io.brupdate
    xma(e).io.resp.ready          := io.resp.ready
  }
  io.resp.valid := xma(0).io.resp.valid
  io.resp.bits.uop.vxsat := xma.map(m => m.io.resp.valid && m.io.resp.bits.uop.vxsat).reduce(_ || _)
  io.resp.bits.data := Mux1H(UIntToOH(uop.vd_eew), Seq(Cat(xma.slice(0, vLen/ 8).map(_.io.resp.bits.data( 7,0)).reverse),
                                                       Cat(xma.slice(0, vLen/16).map(_.io.resp.bits.data(15,0)).reverse),
                                                       Cat(xma.slice(0, vLen/32).map(_.io.resp.bits.data(31,0)).reverse),
                                                       Cat(xma.slice(0, vLen/64).map(_.io.resp.bits.data(63,0)).reverse)))
}

/**
 * frsqrt7/frec7 unit returns an estimate results accurate to 7 bits.
 *
 * @param isPipelined is the functional unit pipelined
 * @param numStages number of stages for the functional unit
 * @param numBypassStages number of bypass stages
 * @param dataWidth width of the data out of the functional unit
 */

class FR7Unit(latency: Int)(implicit p: Parameters) 
  extends PipelinedFunctionalUnit(
  numStages = latency,
  numBypassStages = 0,
  earliestBypassStage = 0,
  dataWidth = 65,
  needsFcsr = true
) with tile.HasFPUParameters
{
  val io_req = io.req.bits
  val vsew = io_req.uop.vconfig.vtype.vsew
  val req = Wire(new tile.FPInput)
  val tag = Mux(vsew === 3.U, D, Mux(vsew === 2.U, S, H))
  val fp_ctrl = Wire(new FPUCtrlSigs())
  fp_ctrl.typeTagIn := tag
  fp_ctrl.typeTagOut:= tag
  fp_ctrl.ldst      := DontCare
  fp_ctrl.wen       := DontCare
  fp_ctrl.ren1      := DontCare
  fp_ctrl.ren2      := DontCare
  fp_ctrl.ren3      := DontCare
  fp_ctrl.swap12    := DontCare
  fp_ctrl.swap23    := DontCare
  fp_ctrl.fromint   := DontCare
  fp_ctrl.toint     := DontCare
  fp_ctrl.fastpipe  := DontCare
  fp_ctrl.fma       := DontCare
  fp_ctrl.div       := DontCare
  fp_ctrl.sqrt      := DontCare
  fp_ctrl.wflags    := DontCare

  req <> fp_ctrl
  req.in1    := unbox(recode(io_req.rs2_data, tag), tag, None)
  req.in2    := io_req.rs2_data
  req.in3    := DontCare
  req.rm     := io.fcsr_rm
  req.typ    := Mux(io_req.uop.uopc === uopVFRSQRT7, 0.U, 1.U)
  req.fmt    := DontCare
  req.fmaCmd := DontCare

  val fr7 = Module(new tile.FR7(latency))
  fr7.io.in.valid := io.req.valid
  fr7.io.in.bits := req
  fr7.io.active  := true.B

  io.resp.bits.data              := fr7.io.out.bits.data
  io.resp.bits.fflags.valid      := fr7.io.out.valid
  io.resp.bits.fflags.bits.uop   := io.resp.bits.uop
  io.resp.bits.fflags.bits.flags := fr7.io.out.bits.exc
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
  val is_vmaskInsn_last_split = uop.v_eidx === (vmaskInsn_vl-1.U)
  val vmaskInsn_mask = boom.util.MaskGen(0.U, vl(5,0), 64)
  val vmaskInsn_rs2_data = Mux(is_vmaskInsn_last_split & (~is_multiple_of_64), (rs2_data & vmaskInsn_mask), rs2_data)

  // operand 1 select
  var op1_data: UInt = null
  op1_data = Mux(uop.ctrl.op1_sel.asUInt === OP1_RS1 , rs1_data,
             Mux(uop.ctrl.op1_sel.asUInt === OP1_VS2 , vmaskInsn_rs2_data,
                 0.U))

  val init_popc = Mux(uop.v_eidx === 0.U, 0.U, RegNext(vmaskUnit.io.out))
  val init_first_idx = Mux(uop.v_eidx === 0.U, 0.U, 64.U)

  // operand 2 select
  val op2_data = WireInit(0.U(xLen.W))
  op2_data:= Mux(uop.ctrl.op2_sel === OP2_RS2 , rs2_data,
             Mux(uop.uopc.isOneOf(uopVPOPC), init_popc,
             Mux(uop.uopc.isOneOf(uopVFIRST), init_first_idx,
             Mux(uop.ctrl.op2_sel === OP2_VS1,  rs1_data, 0.U))))

  vmaskUnit.io.in := op1_data.asUInt
  vmaskUnit.io.in_addend := op2_data.asUInt
  vmaskUnit.io.fn  := uop.ctrl.op_fcn

  val is_0_op_num = (op1_data === 0.U) || (is_vmaskInsn_last_split & ~is_multiple_of_64 & ((vmaskInsn_mask & op1_data) === 0.U))
  val has_find_1_r = RegInit(false.B)
  val firstIdx_r = RegInit(0.U(xLen.W))
  when(is_vmaskInsn_last_split) {
    has_find_1_r := false.B
    firstIdx_r := 0.U(xLen.W)
  }.elsewhen(~is_0_op_num & ~has_find_1_r){
    has_find_1_r := true.B
    firstIdx_r := vmaskUnit.io.out
  }.elsewhen(uop.v_eidx === 0.U){
    has_find_1_r := false.B
    firstIdx_r := 0.U(xLen.W)
  }

  val firstIdx_result = Mux(is_vmaskInsn_last_split & is_0_op_num & ~has_find_1_r , ~0.U(xLen.W),
    Mux(is_vmaskInsn_last_split & ~is_0_op_num & ~has_find_1_r, vmaskUnit.io.out,
    Mux(is_vmaskInsn_last_split, firstIdx_r, 0.U(xLen.U))))

  val sof_result = Mux(has_find_1_r,
    Mux(is_vmaskInsn_last_split & ~is_multiple_of_64, ~vmaskInsn_mask & op1_data, 0.U(xLen.W)),
    Mux(is_vmaskInsn_last_split & ~is_multiple_of_64, ~vmaskInsn_mask & op1_data | vmaskUnit.io.out, Mux(is_0_op_num, 0.U(xLen.W), vmaskUnit.io.out)))

  val sbf_result = Mux(has_find_1_r,
    Mux(is_vmaskInsn_last_split & ~is_multiple_of_64, ~vmaskInsn_mask & op1_data, 0.U(xLen.W)),
    Mux(is_vmaskInsn_last_split & ~is_multiple_of_64, ~vmaskInsn_mask & op1_data | vmaskUnit.io.out, Mux(is_0_op_num, ~0.U(xLen.W), vmaskUnit.io.out)))

  val sif_result = Mux(has_find_1_r,
    Mux(is_vmaskInsn_last_split & ~is_multiple_of_64, ~vmaskInsn_mask & op1_data, 0.U(xLen.W)),
    Mux(is_vmaskInsn_last_split & ~is_multiple_of_64, ~vmaskInsn_mask & op1_data | vmaskUnit.io.out, Mux(is_0_op_num, ~0.U(xLen.W), vmaskUnit.io.out)))

  val is_masked  = !uop.v_unmasked
  val v_inactive = uop.is_rvv && !uop.v_active

  val is_viota_m = uop.uopc.isOneOf(uopVIOTA)
  val is_viotaInsn_last = is_viota_m && (uop.v_eidx === vl)
  val iota_r = RegInit(0.U(eLen.W))
  when(is_viota_m){
    when(is_viotaInsn_last){
      iota_r := 0.U
    }.elsewhen(is_masked & rs1_data(uop.v_eidx(log2Ceil(eLen)-1,0))) {
      iota_r := iota_r + rs2_data(uop.v_eidx(log2Ceil(eLen)-1,0))
    }.elsewhen(is_masked & (~rs1_data(uop.v_eidx(log2Ceil(eLen)-1,0)))) {
      iota_r := iota_r
    }.otherwise{
      iota_r := iota_r + rs2_data(uop.v_eidx(log2Ceil(eLen)-1,0))
    }
  }

  val iota_result = WireInit(0.U(eLen.W))
  when(is_masked & (~rs1_data(uop.v_eidx(log2Ceil(eLen)-1,0)))) {
    iota_result := io.req.bits.rs3_data
  }.elsewhen(uop.v_eidx === 0.U){
    iota_result := 0.U
  }.elsewhen(is_masked & rs1_data(uop.v_eidx(log2Ceil(eLen)-1,0))) {
    iota_result := iota_r
  }.otherwise{
    iota_result := iota_r
  }

  val vmaskUnit_out = Mux(uop.uopc.isOneOf(uopVFIRST), firstIdx_result,
    Mux(uop.uopc.isOneOf(uopVMSOF), sof_result,
    Mux(uop.uopc.isOneOf(uopVMSBF), Mux(is_masked, sbf_result & rs1_data, sbf_result),
    Mux(uop.uopc.isOneOf(uopVMSIF), Mux(is_masked, sif_result & rs1_data, sif_result),
    Mux(uop.uopc.isOneOf(uopVIOTA), Mux(v_inactive, io.req.bits.rs3_data(0), iota_result),
    vmaskUnit.io.out)))))

  // vl => last
  io.resp.bits.data := vmaskUnit_out
}

/**
 * VMX functional unit with back-pressure machanism.
 *
 * @param dataWidth data to be passed into the functional unit
 */
// implemented with BranchKillableQueue
class VMXUnit(dataWidth: Int)(implicit p: Parameters) extends FunctionalUnit(
    isPipelined = false,
    numStages = 4,
    numBypassStages = 0,
    dataWidth = dataWidth)
{
  require (numStages > 1)
  // buffer up results since we share write-port on integer regfile.
  val queue = Module(new BranchKillableQueue(new ExeUnitResp(dataWidth), entries = numStages))
  // enque
  io.req.ready                   := queue.io.enq.ready && (queue.io.count < (numStages-1).U)
  queue.io.enq.valid             := io.req.valid && !IsKilledByBranch(io.brupdate, io.req.bits.uop) && !io.req.bits.kill
  queue.io.enq.bits.uop          := io.req.bits.uop
  queue.io.enq.bits.data         := io.req.bits.rs3_data
  queue.io.enq.bits.predicated   := false.B
  queue.io.enq.bits.fflags.valid := false.B
  queue.io.enq.bits.fflags.bits  := DontCare
  queue.io.brupdate              := io.brupdate
  queue.io.flush                 := io.req.bits.kill
  // deque
  queue.io.deq.ready             := io.resp.ready
  io.resp.valid                  := queue.io.deq.valid && !IsKilledByBranch(io.brupdate, queue.io.deq.bits.uop)
  io.resp.bits.uop               := queue.io.deq.bits.uop
  io.resp.bits.uop.br_mask       := GetNewBrMask(io.brupdate, queue.io.deq.bits.uop)
  io.resp.bits.data              := queue.io.deq.bits.data
  io.resp.bits.predicated        := queue.io.deq.bits.predicated
  io.resp.bits.fflags            := queue.io.deq.bits.fflags
}

/**
 * Divide functional unit.
 *
 * @param dataWidth data to be passed into the functional unit
 */
class SRT4DivUnit(dataWidth: Int)(implicit p: Parameters) extends IterativeFunctionalUnit(dataWidth)
{
  // use SRT4Divider in Xiangshan core
  val div = Module(new SRT4DividerDataModule(len = dataWidth))

  // request
  io.req.ready      := div.io.in_ready
  div.io.valid      := io.req.valid && !this.do_kill
  div.io.src(0)     := io.req.bits.rs1_data
  div.io.src(1)     := io.req.bits.rs2_data
  div.io.sign       := io.req.bits.uop.uopc.isOneOf(uopDIV, uopREM, uopDIVW, uopREMW, uopVDIV, uopVREM)
  div.io.kill_w     := this.do_kill
  div.io.kill_r     := this.do_kill && !div.io.in_ready
  div.io.isHi       := io.req.bits.uop.uopc.isOneOf(uopREM, uopREMU, uopREMW, uopREMUW, uopVREM, uopVREMU)
  div.io.isW        := !io.req.bits.uop.ctrl.fcn_dw
  if(usingVector) {
    when(io.req.bits.uop.is_rvv && !io.req.bits.uop.v_active) {
      div.io.src(0) := io.req.bits.rs3_data
      div.io.src(1) := 1.U
    } .elsewhen(io.req.bits.uop.is_rvv) {
      div.io.src(0) := io.req.bits.rs2_data
      div.io.src(1) := io.req.bits.rs1_data
    }
  }
  // response
  div.io.out_ready  := io.resp.ready
  io.resp.valid     := div.io.out_valid && !this.do_kill
  io.resp.bits.data := div.io.out_data
}

/**
 * Vector ALU Wrapper
 *
 * @param numStages how many pipeline stages does the functional unit have
 * @param dataWidth width of the data being operated on in the functional unit
 */
@chiselName
class VecALUUnit(
  numStages: Int = 1,
  dataWidth: Int)
(implicit p: Parameters)
  extends PipelinedFunctionalUnit(
    numStages = numStages,
    numBypassStages = numStages,
    isAluUnit = false,
    earliestBypassStage = 0,
    dataWidth = dataWidth,
    isCsrUnit = false,
    isJmpUnit = false)
  with boom.ifu.HasBoomFrontendParameters
{
  val uop = io.req.bits.uop

  // immediate generation
  val imm_xprlen = ImmGen(uop.imm_packed, uop.ctrl.imm_sel)
  val rs1_data = io.req.bits.rs1_data
  val rs2_data = io.req.bits.rs2_data
  val rs3_data = io.req.bits.rs3_data
  val body, prestart, tail, mask, inactive = Wire(UInt(vLenb.W))
  val vl = uop.vconfig.vl
  prestart := 0.U // FIXME: Cat((0 until vLen/8).map(b => uop.v_eidx + b.U < csr_vstart).reverse)
  //body     := Cat((0 until vLen/8).map(b => uop.v_eidx + b.U >= csr_vstart && uop.v_eidx + b.U < vl).reverse)
  body     := Cat((0 until vLen/8).map(b => uop.v_eidx + b.U >= 0.U && uop.v_eidx + b.U < vl).reverse)
  mask     := Mux(uop.v_unmasked, ~(0.U(vLenb.W)),
              Cat((0 until vLen/8).map(b => io.req.bits.rvm_data(b)).reverse))
  tail     := Cat((0 until vLen/8).map(b => uop.v_eidx + b.U >= vl).reverse)
  inactive := prestart | body & ~mask | tail
  val byte_inactive = Mux1H(UIntToOH(uop.vd_eew(1,0)),
                            Seq(inactive,
                                Cat((0 until vLenb/2).map(e => Fill(2, inactive(e))).reverse),
                                Cat((0 until vLenb/4).map(e => Fill(4, inactive(e))).reverse),
                                Cat((0 until vLenb/8).map(e => Fill(8, inactive(e))).reverse)))

  // operand 1 select
  val op1_data = WireInit(0.U(vLen.W))
  op1_data := Mux1H(Seq((uop.ctrl.op1_sel === OP1_RS1).asBool     -> io.req.bits.rs1_data,
                        (uop.ctrl.op1_sel === OP1_VS2).asBool     -> io.req.bits.rs2_data,
                        (uop.ctrl.op1_sel === OP1_INV_VS2).asBool -> ~io.req.bits.rs2_data,
                        (uop.ctrl.op1_sel === OP1_ZERO).asBool    -> 0.U))

  // operand 2 select
  val op2_data = WireInit(0.U(vLen.W))
  op2_data := Mux1H(Seq((uop.ctrl.op2_sel === OP2_IMMC).asBool    -> uop.prs1(4,0),
                        (uop.ctrl.op2_sel === OP2_RS2).asBool     -> io.req.bits.rs2_data,
                        (uop.ctrl.op2_sel === OP2_VS1).asBool     -> io.req.bits.rs1_data,
                        (uop.ctrl.op2_sel === OP2_INV_VS1).asBool -> ~io.req.bits.rs1_data,
                        (uop.ctrl.op2_sel === OP2_ZERO).asBool    -> 0.U))

  val op1_eew = Mux(uop.ctrl.op1_sel === OP1_RS1, uop.vs1_eew, uop.vs2_eew)
  val op2_eew = Mux(uop.ctrl.op2_sel === OP2_RS2, uop.vs2_eew, uop.vs1_eew)
  val e64_adder_out       = Wire(Vec(numELENinVLEN, UInt(64.W)))
  val e64_cmp_out, e64_co = Wire(Vec(numELENinVLEN, Bool()))
  val e32_adder_out       = Wire(Vec(numELENinVLEN*2, UInt(32.W)))
  val e32_cmp_out, e32_co = Wire(Vec(numELENinVLEN*2, Bool()))
  val e16_adder_out       = Wire(Vec(numELENinVLEN*4, UInt(16.W)))
  val e16_cmp_out, e16_co = Wire(Vec(numELENinVLEN*4, Bool()))
  val e8_adder_out        = Wire(Vec(numELENinVLEN*8, UInt(8.W)))
  val e8_cmp_out, e8_co   = Wire(Vec(numELENinVLEN*8, Bool()))
  for (e <- 0 until vLenb) {
    // FIXME: parameterize data width of ALU to save area
    val alu = Module(new freechips.rocketchip.rocket.ALU(withCarryIO = true))
    // input
    alu.io.fn  := uop.ctrl.op_fcn
    alu.io.dw  := uop.ctrl.fcn_dw
    if (e < numELENinVLEN) {
      // e64 ALU
      alu.io.in1 := Mux1H(UIntToOH(op1_eew), Seq(op1_data(8*e+7, 8*e), op1_data(16*e+15, 16*e), op1_data(32*e+31, 32*e), op1_data(64*e+63, 64*e)))
      alu.io.in2 := Mux1H(UIntToOH(op2_eew), Seq(op2_data(8*e+7, 8*e), op2_data(16*e+15, 16*e), op2_data(32*e+31, 32*e), op2_data(64*e+63, 64*e)))
      alu.io.ci  := false.B // FIXME
    } else if (e < numELENinVLEN*2) {
      // e32 ALU
      alu.io.in1 := Mux1H(UIntToOH(op1_eew), Seq(op1_data(8*e+7, 8*e), op1_data(16*e+15, 16*e), op1_data(32*e+31, 32*e), 0.U))
      alu.io.in2 := Mux1H(UIntToOH(op2_eew), Seq(op2_data(8*e+7, 8*e), op2_data(16*e+15, 16*e), op2_data(32*e+31, 32*e), 0.U))
      alu.io.ci  := false.B // FIXME
    } else if (e < numELENinVLEN*4) {
      // e16 ALU
      alu.io.in1 := Mux(op1_eew(0), op1_data(16*e+15, 16*e), op1_data(8*e+7, 8*e))
      alu.io.in2 := Mux(op2_eew(0), op2_data(16*e+15, 16*e), op2_data(8*e+7, 8*e))
      alu.io.ci  := false.B // FIXME
    } else {
      // e8 ALU
      alu.io.in1 := op1_data(8*e+7, 8*e)
      alu.io.in2 := op2_data(8*e+7, 8*e)
      alu.io.ci  := false.B // FIXME
    }

    // output
    if (e < numELENinVLEN) {
      e64_adder_out(e) := alu.io.adder_out
      e64_cmp_out(e)   := alu.io.cmp_out
      e64_co(e)        := alu.io.co
    }
    if (e < numELENinVLEN*2) {
      e32_adder_out(e) := alu.io.adder_out
      e32_cmp_out(e)   := alu.io.cmp_out
      e32_co(e)        := alu.io.co
    }
    if (e < numELENinVLEN*4) {
      e16_adder_out(e) := alu.io.adder_out
      e16_cmp_out(e)   := alu.io.cmp_out
      e16_co(e)        := alu.io.co
    }
    e8_adder_out(e) := alu.io.adder_out
    e8_cmp_out(e)   := alu.io.cmp_out
    e8_co(e)        := alu.io.co
  }

  // Did I just get killed by the previous cycle's branch,
  // or by a flush pipeline?
  val killed = WireInit(false.B)
  when (io.req.bits.kill || IsKilledByBranch(io.brupdate, uop)) {
    killed := true.B
  }

  val r_val  = RegInit(VecInit(Seq.fill(numStages) { false.B }))
  val r_data = Reg(Vec(numStages, UInt(vLen.W)))
  val r_pred = Reg(Vec(numStages, Bool()))
  val alu_out = WireInit(0.U(vLen.W))
  val v_eidx = uop.v_eidx

  //val v_inactive = uop.is_rvv && !uop.v_active
  //val v_tail = (v_eidx >= vl)

  //val byteWidth = 3.U
  //val vsew64bit = 3.U
  //val is_multiple_of_64 = vl(5,0) === 0.U
  //val vmlogic_vl = vl(5,0).orR +& (vl>>(byteWidth +& vsew64bit))
  //val is_vmlogic_last_split = v_eidx === (vmlogic_vl-1.U)

  //val vmlogic_insn = uop.ctrl.is_vmlogic
  //val vmlogic_mask = boom.util.MaskGen(0.U, vl(5,0), 64)
  //val vmlogic_alu_result = Mux(uop.uopc.isOneOf(uopVMNAND, uopVMNOR, uopVMXNOR), ~alu.io.out, alu.io.out)
  //val vmlogic_last_result = (vmlogic_alu_result & vmlogic_mask) | (io.req.bits.rs3_data & (~vmlogic_mask))
  //val vmlogic_result = Mux(is_vmlogic_last_split & (~is_multiple_of_64), vmlogic_last_result, vmlogic_alu_result)

  //val vmscmp = uop.ctrl.is_vmscmp
  //when (io.req.valid && vmscmp) { assert(uop.rt(RD, isMaskVD), "Problematic vmcompare") }
  //val vadc   = uop.uopc === uopVADC
  //val vsbc   = uop.uopc === uopVSBC
  //val vmadc  = uop.uopc === uopVMADC
  //val vmsbc  = uop.uopc === uopVMSBC
  //when (io.req.valid && (vadc || vsbc)) { assert(!uop.v_unmasked, "Problematic vadc/vsbc") }
  //val alu_co = Mux1H(UIntToOH(uop.vconfig.vtype.vsew(1,0)), Seq(alu.io.out(8), alu.io.out(16), alu.io.out(32), alu.io.co))

  alu_out := Mux1H(UIntToOH(uop.vd_eew), Seq(e8_adder_out.asUInt,
                                             e16_adder_out.asUInt,
                                             e32_adder_out.asUInt,
                                             e64_adder_out.asUInt))

  r_val (0) := io.req.valid
  r_data(0) := Cat((0 until vLenb).map(b => Mux(byte_inactive(b), rs3_data(b*8+7, b*8), alu_out(b*8+7, b*8))).reverse)
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
  io.bypass(0).bits.data := alu_out
  for (i <- 1 until numStages) {
    io.bypass(i).valid := r_val(i-1)
    io.bypass(i).bits.data := r_data(i-1)
  }

  // Exceptions
  io.resp.bits.fflags.valid := false.B
  //io.brinfo := DontCare
}

/**
 * Functional unit to wrap lower level FPU
 *
 * Currently, bypassing is unsupported!
 * All FP instructions are padded out to the max latency unit for easy
 * write-port scheduling.
 */
class VecFPUUnit(dataWidth: Int)(implicit p: Parameters)
  extends PipelinedFunctionalUnit(
    numStages = p(tile.TileKey).core.fpu.get.dfmaLatency,
    numBypassStages = 0,
    earliestBypassStage = 0,
    dataWidth = dataWidth,
    needsFcsr = true)
{
  val fpu = Module(new VecFPU)
  fpu.io.req.valid         := io.req.valid
  fpu.io.req.bits.uop      := io.req.bits.uop
  fpu.io.req.bits.rs1_data := io.req.bits.rs1_data
  fpu.io.req.bits.rs2_data := io.req.bits.rs2_data
  fpu.io.req.bits.rs3_data := io.req.bits.rs3_data
  fpu.io.req.bits.rvm_data := io.req.bits.rvm_data
  fpu.io.req.bits.fcsr_rm  := io.fcsr_rm

  io.resp.bits.data              := fpu.io.resp.bits.data
  io.resp.bits.fflags.valid      := fpu.io.resp.bits.fflags.valid
  io.resp.bits.fflags.bits.uop   := io.resp.bits.uop
  io.resp.bits.fflags.bits.flags := fpu.io.resp.bits.fflags.bits.flags // kill me now
}

