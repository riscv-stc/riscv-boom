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
import freechips.rocketchip.rocket.{DecodeLogic,PipelinedMultiplier,BP,BreakpointUnit,Causes,CSR,VConfig,VType,MConfig,MType}

//import FUConstants._
import boom.common._
import boom.common.MicroOpcodes._
import boom.ifu._
import boom.util._

/**t
 * Functional unit constants
 */
object FUConstants extends Enumeration
{
  // bit mask, since a given execution pipeline may support multiple functional units
  val FU_ALU_ID = Value
  val FU_JMP_ID = Value
  val FU_MEM_ID = Value
  val FU_MUL_ID = Value
  val FU_DIV_ID = Value
  val FU_CSR_ID = Value
  val FU_FPU_ID = Value
  val FU_FDV_ID = Value
  val FU_I2F_ID = Value
  val FU_F2I_ID = Value
  //val FU_VMX_ID = Value // vec load /store index vec store data
  val FU_MAC_ID = Value
  val FU_FR7_ID = Value // vfrsqrt7 / vfrec7
  val FU_VMASKU_ID = Value
  val FU_VRP_ID = Value  // reduction/permutation
  val FU_MT_ID  = Value
  // TODO: Function units can be grouped to save width, as one issue queue only relates to several of them, not all of them
  val FU_GEMM_ID   = Value
  val FU_MMUL_ID   = Value
  val FU_XCLR_ID   = Value
  val FU_HSLICE_ID = Value
  val FU_VSLICE_ID = Value
  val FU_MFCVT_ID  = Value
  val FUC_SZ_ENUM  = Value

  val FUC_SZ    = FUC_SZ_ENUM.id
  val FU_X      = BitPat.dontCare(FUC_SZ)
  val FU_ALU    = (1<<FU_ALU_ID.id).U(FUC_SZ.W)
  val FU_JMP    = (1<<FU_JMP_ID.id).U(FUC_SZ.W)
  val FU_MEM    = (1<<FU_MEM_ID.id).U(FUC_SZ.W)
  val FU_MUL    = (1<<FU_MUL_ID.id).U(FUC_SZ.W)
  val FU_DIV    = (1<<FU_DIV_ID.id).U(FUC_SZ.W)
  val FU_CSR    = (1<<FU_CSR_ID.id).U(FUC_SZ.W)
  val FU_FPU    = (1<<FU_FPU_ID.id).U(FUC_SZ.W)
  val FU_FDV    = (1<<FU_FDV_ID.id).U(FUC_SZ.W)
  val FU_I2F    = (1<<FU_I2F_ID.id).U(FUC_SZ.W)
  val FU_F2I    = (1<<FU_F2I_ID.id).U(FUC_SZ.W)
  //val FU_VMX    = (1<<FU_VMX_ID.id).U(FUC_SZ.W)
  val FU_MAC    = (1<<FU_MAC_ID.id).U(FUC_SZ.W)
  val FU_FR7    = (1<<FU_FR7_ID.id).U(FUC_SZ.W)
  val FU_VMASKU = (1<<FU_VMASKU_ID.id).U(FUC_SZ.W)
  val FU_VRP    = (1<<FU_VRP_ID.id).U(FUC_SZ.W)
  val FU_MT     = (1<<FU_MT_ID.id).U(FUC_SZ.W)
  val FU_GEMM   = (1<<FU_GEMM_ID.id).U(FUC_SZ.W)
  val FU_MMUL   = (1<<FU_MMUL_ID.id).U(FUC_SZ.W)
  val FU_XCLR   = (1<<FU_XCLR_ID.id).U(FUC_SZ.W)
  val FU_HSLICE = (1<<FU_HSLICE_ID.id).U(FUC_SZ.W)
  val FU_VSLICE = (1<<FU_VSLICE_ID.id).U(FUC_SZ.W)
  val FU_MFCVT  = (1<<FU_MFCVT_ID.id).U(FUC_SZ.W)

  // FP stores generate data through FP F2I, and generate address through MemAddrCalc
  def FU_F2IMEM = ((1<<FU_MEM_ID.id) | (1<<FU_F2I_ID.id)).U(FUC_SZ.W)
  // VEC load / store, vs3 read by ALU of Vec Exe Unit
  //def FU_MEMV   = ((1<<FU_MEM_ID.id) | (1<<FU_VMX_ID.id)).U(FUC_SZ.W)
  def FU_IVRP   = ((1<<FU_ALU_ID.id) | (1<<FU_VRP_ID.id)).U(FUC_SZ.W)
  def FU_FVRP   = ((1<<FU_FPU_ID.id) | (1<<FU_VRP_ID.id)).U(FUC_SZ.W)
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
  // val vmx:  Boolean = false,
  val vector: Boolean = false,
  val matrix: Boolean = false)
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
  val vmask = if (usingVector) UInt(vLenb.W) else UInt(0.W)
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
  val needsVxrm: Boolean = false,
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
    val mconfig = if (usingMatrix & isCsrUnit) Input(new MConfig) else null

    // only used by the Fixed unit
    val vxrm    = if (needsVxrm) Input(UInt(2.W)) else null
    val fixCtrl = if (isFixMulAcc) Input(new FixMulAccCtrlSigs()) else null

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
  needsVxrm: Boolean = false,
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
    needsVxrm = needsVxrm,
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
  } else if(usingMatrix && isCsrUnit) {
    val msetr    = uop.uopc.isOneOf(uopMSETTYPE, uopMSETTILEM, uopMSETTILEK, uopMSETTILEN, uopMSETTSIDXI)
    val mseti    = uop.uopc.isOneOf(uopMSETTYPEI, uopMSETTILEMI, uopMSETTILEKI, uopMSETTILENI, uopMSETTSIDXI)
    val mset     = msetr | mseti
    val msettype = uop.uopc.isOneOf(uopMSETTYPE,  uopMSETTYPEI)
    val msetm    = uop.uopc.isOneOf(uopMSETTILEM, uopMSETTILEMI)
    val msetn    = uop.uopc.isOneOf(uopMSETTILEN, uopMSETTILENI)
    val useMax   = uop.ldst =/= 0.U && uop.lrs1 === 0.U
    val msetdata = Mux(mseti, imm_xprlen(27,15), io.req.bits.rs1_data)
    val tilemMax = numTrTileRows.U
    val tilenMax = vLenb.U >> io.mconfig.mtype.msew
    val tilekMax = tilemMax.min(tilenMax)
    val msettile = Mux(msetm, tilemMax,
                   Mux(msetn, tilenMax, tilekMax)).min(Mux(useMax, tilemMax+tilenMax, msetdata))
    op1_data = Mux(msettype, msetdata, 
               Mux(mset,     msettile,
               Mux(uop.ctrl.op1_sel.asUInt === OP1_RS1 , io.req.bits.rs1_data, 0.U)))
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
    //val v_ls_ew = Mux(usingVector.B & uop.is_rvv, uop.v_ls_ew, 0.U)
    //val uop_sew = Mux(usingVector.B & uop.is_rvv, uop.vconfig.vtype.vsew, 0.U)
    // unit stride load/store
    //val vec_us_ls = usingVector.B & uop.is_rvv & uop.uopc.isOneOf(uopVL, uopVLFF, uopVSA) // or uopVLFF
    //val vec_cs_ls = usingVector.B & uop.is_rvv & uop.uopc.isOneOf(uopVLS, uopVSSA)
    //val vec_idx_ls = usingVector.B & uop.is_rvv & uop.uopc.isOneOf(uopVLUX, uopVSUXA, uopVLOX, uopVSOXA)
    // TODO: optimize multiplications here
    //op2 := Mux(vec_us_ls, ((uop.v_eidx * uop.v_seg_nf + uop.v_seg_f) << v_ls_ew).asSInt,
           //Mux(vec_cs_ls, io.req.bits.rs2_data.asSInt * uop.v_eidx.asUInt + Cat(0.U(1.W), uop.v_seg_f << v_ls_ew).asSInt,
           //Mux(vec_idx_ls, uop.v_xls_offset.asSInt + Cat(0.U(1.W), uop.v_seg_f << uop_sew).asSInt,
           //uop.imm_packed(19,8).asSInt)))
  }

  // perform address calculation
  val sum = (op1 + op2).asUInt
  val ea_sign = Mux(sum(vaddrBits-1), ~sum(63,vaddrBits) === 0.U,
                                       sum(63,vaddrBits) =/= 0.U)
  val effective_address = Cat(ea_sign, sum(vaddrBits-1,0)).asUInt

  val store_data = io.req.bits.rs2_data

  io.resp.bits.addr := Mux(uop.is_vm_ext, op1.asUInt, effective_address)
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
    assert (!(uop.is_rvv && io.req.valid && !uop.uopc.isOneOf(uopVL, uopVLM, uopVLFF, uopVSA, uopVSMA, uopVLS, uopVSSA, uopVLUX, uopVSUXA, uopVLOX, uopVSOXA)))
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
      //assert(io_req.uop.v_active, "unexpected inactive split")
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
abstract class IterativeFunctionalUnit(dataWidth: Int, needsFcsr: Boolean = false)(implicit p: Parameters)
  extends FunctionalUnit(
    isPipelined = false,
    numStages = 1,
    numBypassStages = 0,
    dataWidth = dataWidth,
    needsFcsr = needsFcsr)
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
  io.resp.bits.vmask := Fill(vLenb, 1.U(1.W))
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
    when(io.req.bits.uop.is_rvv) { //&& !io.req.bits.uop.v_active) {
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
    isFixMulAcc = true,
    needsVxrm = true) with ShouldBeRetimed
{
  val e8 = (dataWidth == 8)
  val e16= (dataWidth == 16)
  val e32= (dataWidth == 32)
  val e64= (dataWidth == 64)
  val in_req = WireInit(io.req)
  val in = Pipe(in_req.valid, in_req.bits)
  val cs = Pipe(in_req.valid, io.fixCtrl).bits
  val uop = io.req.bits.uop
  val rs1_data = io.req.bits.rs1_data
  val rs2_data = io.req.bits.rs2_data
  val rs3_data = io.req.bits.rs3_data
  val u_max = Wire(UInt((dataWidth+1).W))
  val hi, s_max, s_min = Wire(UInt(dataWidth.W))
  val stale_rs3_data = Pipe(in_req.valid, rs3_data).bits

  when (uop.uopc.isOneOf(uopVMADD, uopVNMSUB)) {
    in_req.bits.rs2_data := rs3_data
    in_req.bits.rs3_data := rs2_data
  }
  when (uop.uopc.isOneOf(uopVNMSAC, uopVNMSUB)) {
    in_req.bits.rs1_data := ~rs1_data + 1.U
  }
  when (uop.uopc.isOneOf(uopVSADDU, uopVSADD, uopVAADDU, uopVAADD, uopVSSUBU, uopVSSUB, uopVASUBU, uopVASUB)) {
    in_req.bits.rs3_data := rs1_data
  }

  val lhs = Mux(cs.lhsOne, 1.S, Cat(cs.lhsSigned && in.bits.rs1_data(dataWidth-1), in.bits.rs1_data).asSInt)
  val rhs = Cat(cs.rhsSigned && in.bits.rs2_data(dataWidth-1), in.bits.rs2_data).asSInt
  val sext_rs3 = Cat(cs.accSigned && in.bits.rs3_data(dataWidth-1), in.bits.rs3_data)
  val sub_rs3  = (~sext_rs3 + 1.U(1.W)).asSInt
  val acc = Mux(cs.doAcc, Mux(cs.negAcc, sub_rs3, sext_rs3.asSInt), 0.S)
  //when (in.valid && cs.doAcc) { assert(in.bits.uop.frs3_en, "Incorrect frs3_en") }
  val macc = lhs * rhs +& acc
  val macc_msk = ~(0.U((dataWidth*2+3).W))
  val vd_eew = in.bits.uop.vd_eew
  val vs2_eew = in.bits.uop.vs2_eew
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
              Mux1H(UIntToOH(cs.srType),
                Seq(0.U(6.W),
                    1.U(6.W),
                    ((8.U << vd_eew) - 1.U)(5,0),
                    Mux1H(UIntToOH(Mux(cs.doClip, vs2_eew(1,0), vd_eew(1,0))),
                      Seq(in.bits.rs1_data(2,0),
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
  val active = (out.bits.uop.v_unmasked || out.bits.rvm_data(0)) && out.bits.uop.v_eidx >= out.bits.uop.vstart && out.bits.uop.v_eidx < out.bits.uop.vconfig.vl
  io.resp.bits.data := Pipe(out.valid, Mux(active, muxed, stale_rs3_data), numStages-1).bits
}

class VecFixUnit(numStages: Int, dataWidth: Int)(implicit p: Parameters)
  extends PipelinedFunctionalUnit(
    numStages = numStages,
    numBypassStages = 0,
    earliestBypassStage = 0,
    dataWidth = dataWidth,
    needsVxrm = true)
{
  // FIXME: optimize me by merge fix-point decoders among different FixMulAcc
  // FIXME: mute corresponding FixMulAcc by mask to save power
  val uop = io.req.bits.uop
  val rs1_data = io.req.bits.rs1_data
  val rs2_data = io.req.bits.rs2_data
  val rs3_data = io.req.bits.rs3_data
  val rvm_data = io.req.bits.rvm_data
  val rs1Unsigned = uop.rt(RS1, isUnsignedV) || uop.rt(RS1, isIntU)
  val rs2Unsigned = uop.rt(RS2, isUnsignedV)
  val rdUnsigned  = uop.rt(RD, isUnsignedV)

  val e64Out = Wire(Vec(numELENinVLEN, UInt(64.W)))
  val e32Out = Wire(Vec(numELENinVLEN*2, UInt(32.W)))
  val e16Out = Wire(Vec(numELENinVLEN*4, UInt(16.W)))
  val e8Out  = Wire(Vec(numELENinVLEN*8, UInt(8.W)))
  val vxsatOut = Wire(Vec(vLenb, Bool()))

  val DC2 = BitPat.dontCare(2)
  val table: List[(BitPat, List[BitPat])] = List(
    //                          cmdHi   negAcc accSigned   srType (0, 1, SEW-1, RS1)
    //                          |  lhsSigned|  |  lhsOne   |    doClip
    //                          |  |  rhsSigned|  doRO     |    |
    //                          |  |  |  doAcc |  |  |  roSigned|
    //                          |  |  |  |  |  |  |  |  |  |    |
    BitPat(uopVMUL)     -> List(N, Y, Y, N, X, X, N, N, X, DC2, N)
   ,BitPat(uopVMULH)    -> List(Y, Y, Y, N, X, X, N, N, X, DC2, N)
   ,BitPat(uopVMULHU)   -> List(Y, N, N, N, X, X, N, N, X, DC2, N)
   ,BitPat(uopVMULHSU)  -> List(Y, N, Y, N, X, X, N, N, X, DC2, N)
   ,BitPat(uopVWMULU)   -> List(N, N, N, N, X, X, N, N, X, DC2, N)
   ,BitPat(uopVWMULSU)  -> List(N, N, Y, N, X, X, N, N, X, DC2, N)
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
  val cs = Wire(new FixMulAccCtrlSigs()).decoder(uop.uopc, table)

  for(e <- 0 until vLenb) {
    // instants
    val xma = if(e < numELENinVLEN)        Module(new FixMulAcc(numStages, eLen))
              else if(e < numELENinVLEN*2) Module(new FixMulAcc(numStages, eLen >> 1))
              else if(e < numELENinVLEN*4) Module(new FixMulAcc(numStages, eLen >> 2))
              else                         Module(new FixMulAcc(numStages, eLen >> 3))

    xma.io.vxrm                := io.vxrm
    xma.io.req.bits.uop        := uop
    xma.io.req.valid           := io.req.valid
    xma.io.req.bits.uop.v_eidx := uop.v_eidx + e.U
    xma.io.req.bits.rvm_data   := rvm_data(e)
    xma.io.req.bits.pred_data  := io.req.bits.pred_data
    xma.io.req.bits.kill       := io.req.bits.kill
    xma.io.brupdate            := io.brupdate
    xma.io.resp.ready          := io.resp.ready
    xma.io.fixCtrl             := cs
    // inputs
    if(e < numELENinVLEN) {
      xma.io.req.bits.rs1_data := Mux(rs1Unsigned, Mux1H(UIntToOH(uop.vs1_eew),
                                                         Seq(rs1_data(8*e+7, 8*e), rs1_data(16*e+15, 16*e), rs1_data(32*e+31, 32*e), rs1_data(64*e+63, 64*e))),
                                                   Mux1H(UIntToOH(uop.vs1_eew),
                                                         Seq(rs1_data( 8*e+7,   8*e).sextTo(eLen),
                                                             rs1_data(16*e+15, 16*e).sextTo(eLen),
                                                             rs1_data(32*e+31, 32*e).sextTo(eLen),
                                                             rs1_data(64*e+63, 64*e))))
      xma.io.req.bits.rs2_data := Mux(rs2Unsigned, Mux1H(UIntToOH(uop.vs2_eew),
                                                         Seq(rs2_data(8*e+7, 8*e), rs2_data(16*e+15, 16*e), rs2_data(32*e+31, 32*e), rs2_data(64*e+63, 64*e))),
                                                   Mux1H(UIntToOH(uop.vs2_eew),
                                                         Seq(rs2_data( 8*e+7,   8*e).sextTo(eLen),
                                                             rs2_data(16*e+15, 16*e).sextTo(eLen),
                                                             rs2_data(32*e+31, 32*e).sextTo(eLen),
                                                             rs2_data(64*e+63, 64*e))))
      xma.io.req.bits.rs3_data := Mux(rdUnsigned,  Mux1H(UIntToOH(uop.vd_eew),
                                                         Seq(rs3_data(8*e+7, 8*e), rs3_data(16*e+15, 16*e), rs3_data(32*e+31, 32*e), rs3_data(64*e+63, 64*e))),
                                                   Mux1H(UIntToOH(uop.vd_eew),
                                                         Seq(rs3_data( 8*e+7,   8*e).sextTo(eLen),
                                                             rs3_data(16*e+15, 16*e).sextTo(eLen),
                                                             rs3_data(32*e+31, 32*e).sextTo(eLen),
                                                             rs3_data(64*e+63, 64*e))))
    } else if(e < numELENinVLEN*2) {
      xma.io.req.bits.rs1_data := Mux(rs1Unsigned, Mux1H(UIntToOH(uop.vs1_eew),
                                                         Seq(rs1_data(8*e+7, 8*e), rs1_data(16*e+15, 16*e), rs1_data(32*e+31, 32*e), 0.U)),
                                                   Mux1H(UIntToOH(uop.vs1_eew),
                                                         Seq(rs1_data( 8*e+7,   8*e).sextTo(eLen >> 1),
                                                             rs1_data(16*e+15, 16*e).sextTo(eLen >> 1),
                                                             rs1_data(32*e+31, 32*e),
                                                             0.U)))
      xma.io.req.bits.rs2_data := Mux(rs2Unsigned, Mux1H(UIntToOH(uop.vs2_eew),
                                                         Seq(rs2_data(8*e+7, 8*e), rs2_data(16*e+15, 16*e), rs2_data(32*e+31, 32*e), 0.U)),
                                                   Mux1H(UIntToOH(uop.vs2_eew),
                                                         Seq(rs2_data( 8*e+7,   8*e).sextTo(eLen >> 1),
                                                             rs2_data(16*e+15, 16*e).sextTo(eLen >> 1),
                                                             rs2_data(32*e+31, 32*e),
                                                             0.U)))
      xma.io.req.bits.rs3_data := Mux(rdUnsigned,  Mux1H(UIntToOH(uop.vd_eew),
                                                         Seq(rs3_data(8*e+7, 8*e), rs3_data(16*e+15, 16*e), rs3_data(32*e+31, 32*e), 0.U)),
                                                   Mux1H(UIntToOH(uop.vd_eew),
                                                         Seq(rs3_data( 8*e+7,   8*e).sextTo(eLen >> 1),
                                                             rs3_data(16*e+15, 16*e).sextTo(eLen >> 1),
                                                             rs3_data(32*e+31, 32*e),
                                                             0.U)))
    } else if(e < numELENinVLEN*4) {
      xma.io.req.bits.rs1_data := Mux(rs1Unsigned, Mux(uop.vs1_eew(0), rs1_data(16*e+15, 16*e), rs1_data(8*e+7, 8*e)),
                                                   Mux(uop.vs1_eew(0), rs1_data(16*e+15, 16*e), rs1_data(8*e+7, 8*e).sextTo(eLen >> 2)))
      xma.io.req.bits.rs2_data := Mux(rs2Unsigned, Mux(uop.vs2_eew(0), rs2_data(16*e+15, 16*e), rs2_data(8*e+7, 8*e)),
                                                   Mux(uop.vs2_eew(0), rs2_data(16*e+15, 16*e), rs2_data(8*e+7, 8*e).sextTo(eLen >> 2)))
      xma.io.req.bits.rs3_data := Mux(rdUnsigned,  Mux(uop.vd_eew(0),  rs3_data(16*e+15, 16*e), rs3_data(8*e+7, 8*e)),
                                                   Mux(uop.vd_eew(0),  rs3_data(16*e+15, 16*e), rs3_data(8*e+7, 8*e).sextTo(eLen >> 2)))
    } else {
      xma.io.req.bits.rs1_data := rs1_data(8*e+7, 8*e)
      xma.io.req.bits.rs2_data := rs2_data(8*e+7, 8*e)
      xma.io.req.bits.rs3_data := rs3_data(8*e+7, 8*e)
    }
    // outputs
    if(e < numELENinVLEN) {
      e64Out(e) := xma.io.resp.bits.data
    }
    if(e < numELENinVLEN*2) {
      e32Out(e) := xma.io.resp.bits.data
    }
    if(e < numELENinVLEN*4) {
      e16Out(e) := xma.io.resp.bits.data
    }
    e8Out(e)    := xma.io.resp.bits.data
    vxsatOut(e) := xma.io.resp.valid && xma.io.resp.bits.uop.vxsat
  }

  val respUop = io.resp.bits.uop
  val isNarrowOdd = respUop.rt(RS2, isWidenV) && (respUop.v_eidx((vLenbSz - 1).U - respUop.vd_eew) === 1.U)
  val alu_out = Mux1H(UIntToOH(respUop.vd_eew),
    Seq(e8Out.asUInt, e16Out.asUInt, e32Out.asUInt, e64Out.asUInt))

  io.resp.bits.uop.vxsat := vxsatOut.orR
  io.resp.bits.data :=  Mux(isNarrowOdd, alu_out << (vLen / 2), alu_out)
  io.resp.bits.vmask := Mux(isNarrowOdd, Fill(vLenb / 2, 1.U(1.W)) ## Fill(vLenb / 2, 0.U(1.W)), Fill(vLenb, 1.U(1.W)))
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
  io.resp.bits.vmask             := Fill(vLenb, 1.U(1.W))
  io.resp.bits.fflags.valid      := fr7.io.out.valid
  io.resp.bits.fflags.bits.uop   := io.resp.bits.uop
  io.resp.bits.fflags.bits.flags := fr7.io.out.bits.exc
}


class VecFR7Unit(latency: Int, dataWidth: Int)(implicit p: Parameters)
  extends PipelinedFunctionalUnit(
    numStages = latency,
    numBypassStages = 0,
    earliestBypassStage = 0,
    dataWidth = dataWidth,
    needsFcsr = true
  ) with tile.HasFPUParameters
{
  val io_req = io.req.bits
  val vsew = io_req.uop.vconfig.vtype.vsew
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

  // FUE: function-unit-enable
  class fre extends Bundle {
    val dfr7 = Bool()
    val sfr7 = Bool()
    val hfr7 = Bool()
  }

  val reqfre = Wire(new fre);
  reqfre.dfr7 := (tag === D)
  reqfre.sfr7 := (tag === S)
  reqfre.hfr7 := (tag === H)

  val reqpipe  = Pipe(io.req.valid, io_req, latency)
  val frepipe = Pipe(io.req.valid, reqfre, latency).bits

  def fr7Input(minT: Option[tile.FType], esel: Int): tile.FPInput = {
    val req = Wire(new tile.FPInput(true))
    val rs1_edata, rs2_edata = WireInit(0.U(65.W))
    if (minT == Some(tile.FType.D)) {
      rs1_edata := unbox(recode(io_req.rs2_data(esel * 64 + 63, esel * 64), tag), tag, None)
      rs2_edata := io_req.rs2_data(esel * 64 + 63, esel * 64)
    } else if (minT == Some(tile.FType.S)) {
      rs1_edata := unbox(recode(io_req.rs2_data(esel * 32 + 31, esel * 32), tag), tag, None)
      rs2_edata :=io_req.rs2_data(esel * 32 + 31, esel * 32)
    } else { //FType.H
      rs1_edata := unbox(recode(io_req.rs2_data(esel * 16 + 15, esel * 16), tag), tag, None)
      rs2_edata := io_req.rs2_data(esel * 16 + 15, esel * 16)
    }
    req <> fp_ctrl
    req.in1 := rs1_edata
    req.in2 := rs2_edata
    req.in3 := DontCare
    req.rm := io.fcsr_rm
    req.typ := Mux(io_req.uop.uopc === uopVFRSQRT7, 0.U, 1.U)
    req.fmt := DontCare
    req.fmaCmd := DontCare
    req
  }

  val dfr7 = (0 until vLen / 64).map(i => Module(new tile.FR7(latency, supportD = true, supportS = false)))
  for (i <- 0 until vLen / 64) {
    val active = (io.req.bits.uop.v_unmasked || io.req.bits.rvm_data(i)) && ((io.req.bits.uop.v_eidx + i.U) < io.req.bits.uop.vconfig.vl) && ((io.req.bits.uop.v_eidx + i.U) >= io.req.bits.uop.vstart)
    dfr7(i).io.in.bits  := fr7Input(Some(tile.FType.D), i)
    dfr7(i).io.in.valid := active && io.req.valid && reqfre.dfr7
    dfr7(i).io.active   := active
  }
  val sfr7 = (0 until vLen / 32).map(i => Module(new tile.FR7(latency, supportD = false, supportS = true)))
  for (i <- 0 until vLen / 32) {
    val active = (io.req.bits.uop.v_unmasked || io.req.bits.rvm_data(i)) && ((io.req.bits.uop.v_eidx + i.U) < io.req.bits.uop.vconfig.vl) && ((io.req.bits.uop.v_eidx + i.U) >= io.req.bits.uop.vstart)
    sfr7(i).io.in.bits  := fr7Input(Some(tile.FType.S), i)
    sfr7(i).io.in.valid := active && io.req.valid && reqfre.sfr7
    sfr7(i).io.active   := active
  }
  val hfr7 = (0 until vLen / 16).map(i => Module(new tile.FR7(latency, supportD = false, supportS = false)))
  for (i <- 0 until vLen / 16) {
    val active = (io.req.bits.uop.v_unmasked || io.req.bits.rvm_data(i)) && ((io.req.bits.uop.v_eidx + i.U) < io.req.bits.uop.vconfig.vl) && ((io.req.bits.uop.v_eidx + i.U) >= io.req.bits.uop.vstart)
    hfr7(i).io.in.bits  := fr7Input(Some(tile.FType.H), i)
    hfr7(i).io.in.valid := active && io.req.valid && reqfre.hfr7
    hfr7(i).io.active   := active
  }
  io.resp.valid := reqpipe.valid
  val fr7_out_data =
    Mux(frepipe.dfr7, Cat(dfr7.zipWithIndex.map{case(m,i) => Mux(m.io.out.valid, m.io.out.bits.data(63,0), reqpipe.bits.rs3_data(i*64+63, i*64))}.reverse),
      Mux(frepipe.sfr7, Cat(sfr7.zipWithIndex.map{case(m,i) => Mux(m.io.out.valid, m.io.out.bits.data(31,0), reqpipe.bits.rs3_data(i*32+31, i*32))}.reverse),
        Cat(hfr7.zipWithIndex.map{case(m,i) => Mux(m.io.out.valid, m.io.out.bits.data(15,0), reqpipe.bits.rs3_data(i*16+15, i*16))}.reverse)))

  io.resp.bits.data := fr7_out_data
  io.resp.bits.vmask := Fill(vLenb, 1.U(1.W))
  io.resp.bits.fflags.valid := io.resp.valid
  io.resp.bits.fflags.bits.flags :=
    Mux(frepipe.dfr7, dfr7.map(m => Mux(m.io.out.valid, m.io.out.bits.exc, 0.U)).reduce(_ | _),
      Mux(frepipe.sfr7, sfr7.map(m => Mux(m.io.out.valid, m.io.out.bits.exc, 0.U)).reduce(_ | _),
        hfr7.map(m => Mux(m.io.out.valid, m.io.out.bits.exc, 0.U)).reduce(_ | _)))

  io.resp.bits.predicated := DontCare
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
    val isSigned = !io.req.bits.uop.rt(RS2, isUnsignedV)
    when(io.req.bits.uop.is_rvv) {
      div.io.src(0) := Mux(isSigned, Mux1H(UIntToOH(io.req.bits.uop.vd_eew),
                                           Seq(io.req.bits.rs2_data( 7, 0).sextTo(eLen),
                                               io.req.bits.rs2_data(15, 0).sextTo(eLen),
                                               io.req.bits.rs2_data(31, 0).sextTo(eLen),
                                               io.req.bits.rs2_data(63, 0))),
                                     Mux1H(UIntToOH(io.req.bits.uop.vd_eew),
                                           Seq(io.req.bits.rs2_data(7, 0), io.req.bits.rs2_data(15, 0), io.req.bits.rs2_data(31, 0), io.req.bits.rs2_data(63, 0))))
      div.io.src(1) := Mux(isSigned, Mux1H(UIntToOH(io.req.bits.uop.vd_eew),
                                           Seq(io.req.bits.rs1_data( 7, 0).sextTo(eLen),
                                               io.req.bits.rs1_data(15, 0).sextTo(eLen),
                                               io.req.bits.rs1_data(31, 0).sextTo(eLen),
                                               io.req.bits.rs1_data(63, 0))),
                                     Mux1H(UIntToOH(io.req.bits.uop.vd_eew),
                                           Seq(io.req.bits.rs1_data(7, 0), io.req.bits.rs1_data(15, 0), io.req.bits.rs1_data(31, 0), io.req.bits.rs1_data(63, 0))))
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
  val withCarry = uop.uopc.isOneOf(uopVADC, uopVSBC, uopVMADC, uopVMSBC)
  val isVMADC   = uop.uopc.isOneOf(uopVMADC, uopVMSBC)
  val isMerge   = uop.uopc.isOneOf(uopMERGE)
  val isShift   = uop.uopc.isOneOf(uopVSLL, uopVSRL, uopVSRA)
  val isVMask   = uop.uopc.isOneOf(uopVMAND, uopVMNAND, uopVMANDNOT, uopVMXOR, uopVMOR, uopVMNOR, uopVMORNOT, uopVMXNOR)
  val isInvert  = uop.uopc.isOneOf(uopVMNAND, uopVMNOR, uopVMXNOR)
  val isScalarMove = uop.uopc.isOneOf(uopVMV_S_X, uopVFMV_S_F)

  // immediate generation
  val imm_xprlen = ImmGen(uop.imm_packed, uop.ctrl.imm_sel)
  val rs1_data = io.req.bits.rs1_data
  val rs2_data = io.req.bits.rs2_data
  val rs3_data = io.req.bits.rs3_data
  val rvm_data = io.req.bits.rvm_data
  val vl       = Mux(isVMask, uop.vconfig.vl >> 3.U, uop.vconfig.vl)
  val vstart   = Mux(isVMask, uop.vstart >> 3.U, uop.vstart)
  val body, prestart, tail, mask, inactive = Wire(UInt(vLenb.W))
  prestart := Cat((0 until vLenb).map(b => uop.v_eidx + b.U < vstart).reverse)
  body     := Cat((0 until vLenb).map(b => uop.v_eidx + b.U >= uop.vstart && uop.v_eidx + b.U < vl).reverse)
  mask     := Mux(uop.v_unmasked || withCarry || isMerge, ~(0.U(vLenb.W)),
              Cat((0 until vLenb).map(b => rvm_data(b)).reverse))
  tail     := Cat((0 until vLenb).map(b => uop.v_eidx + b.U >= vl).reverse)
  inactive := Mux(isScalarMove, Cat(Fill(vLenb-1, 1.U(1.W)), vstart >= vl), prestart | body & ~mask | tail)
  val byteInactive = Mux1H(UIntToOH(uop.vd_eew(1,0)),
                           Seq(inactive,
                               FillInterleaved(2, inactive(vLenb/2-1, 0)),
                               FillInterleaved(4, inactive(vLenb/4-1, 0)),
                               FillInterleaved(8, inactive(vLenb/8-1, 0))))
  val shiftMask   = Fill(eLen, !isShift) | Mux1H(UIntToOH(uop.vs2_eew), Seq("h7".U, "hf".U, "h1f".U, "h3f".U))
  val bitPreStart = FillInterleaved(8, prestart)
  val bitTail     = FillInterleaved(8, tail)
  val bitPreMask  = Mux1H(UIntToOH(uop.vstart(2, 0)),
                          Seq(0.U(8.W), 1.U, 3.U, 7.U, "hf".U, "h1f".U, "h3f".U, "h7f".U))
  val bitTailMask = Cat(Fill(vLen-8, 1.U(1.W)),
                        Mux1H(UIntToOH(uop.vconfig.vl(2, 0)),
                              Seq("hff".U, "hfe".U, "hfc".U, "hf8".U, "hf0".U, "he0".U, "hc0".U, "h80".U)))
  val bitInactive = bitPreStart | bitPreMask << Cat(vstart, 0.U(3.W)) |
                    bitTail & (bitTailMask << Cat(uop.vconfig.vl(vLenSz-1, 3), 0.U(3.W)))(vLen-1, 0)

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
  val isRs1UnSigned = uop.rt(RS1, isUnsignedV) || uop.rt(RS1, isIntU)
  val isRs2UnSigned = uop.rt(RS2, isUnsignedV) || uop.rt(RS2, isIntU)
  val op1Signed = Mux(uop.ctrl.op1_sel === OP1_RS1, !isRs1UnSigned, !isRs2UnSigned)
  val op2Signed = Mux(uop.ctrl.op2_sel === OP2_RS2, !isRs2UnSigned, !isRs1UnSigned)
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
    val alu = if(e < numELENinVLEN)
                Module(new freechips.rocketchip.rocket.ALU(withCarryIO = true, vector = true, dataWidth = eLen))
              else if(e < numELENinVLEN*2)
                Module(new freechips.rocketchip.rocket.ALU(withCarryIO = true, vector = true, dataWidth = eLen >> 1))
              else if(e < numELENinVLEN*4)
                Module(new freechips.rocketchip.rocket.ALU(withCarryIO = true, vector = true, dataWidth = eLen >> 2))
              else
                Module(new freechips.rocketchip.rocket.ALU(withCarryIO = true, vector = true, dataWidth = eLen >> 3))
    // input
    alu.io.fn  := uop.ctrl.op_fcn
    alu.io.dw  := uop.ctrl.fcn_dw
    if (e < numELENinVLEN) {
      // e64 ALU
      alu.io.in1 := Mux(isMerge,  0.U,
                    Mux(op1Signed, Mux1H(UIntToOH(op1_eew),
                                         Seq(op1_data( 8*e+7,   8*e).sextTo(eLen),
                                             op1_data(16*e+15, 16*e).sextTo(eLen),
                                             op1_data(32*e+31, 32*e).sextTo(eLen),
                                             op1_data(64*e+63, 64*e))),
                                   Mux1H(UIntToOH(op1_eew),
                                         Seq(op1_data(8*e+7, 8*e), op1_data(16*e+15, 16*e), op1_data(32*e+31, 32*e), op1_data(64*e+63, 64*e)))))
      //alu.io.in1 := Mux1H(UIntToOH(op1_eew), Seq(op1_data(8*e+7, 8*e), op1_data(16*e+15, 16*e), op1_data(32*e+31, 32*e), op1_data(64*e+63, 64*e)))
      alu.io.in2 := Mux(isMerge,   Mux1H(UIntToOH(op2_eew),
                                         Seq(op1_data( 8*e+7,   8*e) & Fill( 8, !rvm_data(e)) | op2_data( 8*e+7,   8*e) & Fill( 8, rvm_data(e)),
                                             op1_data(16*e+15, 16*e) & Fill(16, !rvm_data(e)) | op2_data(16*e+15, 16*e) & Fill(16, rvm_data(e)),
                                             op1_data(32*e+31, 32*e) & Fill(32, !rvm_data(e)) | op2_data(32*e+31, 32*e) & Fill(32, rvm_data(e)),
                                             op1_data(64*e+63, 64*e) & Fill(64, !rvm_data(e)) | op2_data(64*e+63, 64*e) & Fill(64, rvm_data(e)))),
                    Mux(op2Signed, Mux1H(UIntToOH(op2_eew),
                                         Seq(op2_data( 8*e+7,   8*e).sextTo(eLen),
                                             op2_data(16*e+15, 16*e).sextTo(eLen),
                                             op2_data(32*e+31, 32*e).sextTo(eLen),
                                             op2_data(64*e+63, 64*e))),
                                   Mux1H(UIntToOH(op2_eew),
                                         Seq(op2_data( 8*e+ 7,  8*e) & shiftMask,
                                             op2_data(16*e+15, 16*e) & shiftMask,
                                             op2_data(32*e+31, 32*e) & shiftMask,
                                             op2_data(64*e+63, 64*e) & shiftMask))))
      alu.io.ci  := Mux(withCarry && !uop.v_unmasked, rvm_data(e), false.B) // FIXME
    } else if (e < numELENinVLEN*2) {
      // e32 ALU
      alu.io.in1 := Mux(isMerge,   0.U,
                    Mux(op1Signed, Mux1H(UIntToOH(op1_eew),
                                         Seq(op1_data( 8*e+7,   8*e).sextTo(eLen >> 1),
                                             op1_data(16*e+15, 16*e).sextTo(eLen >> 1),
                                             op1_data(32*e+31, 32*e),
                                             0.U)),
                                   Mux1H(UIntToOH(op1_eew),
                                         Seq(op1_data(8*e+7, 8*e),op1_data(16*e+15, 16*e), op1_data(32*e+31, 32*e), 0.U))))
      //alu.io.in1 := Mux1H(UIntToOH(op1_eew), Seq(op1_data(8*e+7, 8*e), op1_data(16*e+15, 16*e), op1_data(32*e+31, 32*e), 0.U))
      alu.io.in2 := Mux(isMerge,   Mux1H(UIntToOH(op2_eew),
                                         Seq(op1_data( 8*e+7,   8*e) & Fill( 8, !rvm_data(e)) | op2_data( 8*e+7,   8*e) & Fill( 8, rvm_data(e)),
                                             op1_data(16*e+15, 16*e) & Fill(16, !rvm_data(e)) | op2_data(16*e+15, 16*e) & Fill(16, rvm_data(e)),
                                             op1_data(32*e+31, 32*e) & Fill(32, !rvm_data(e)) | op2_data(32*e+31, 32*e) & Fill(32, rvm_data(e)),
                                             0.U)),
                    Mux(op2Signed, Mux1H(UIntToOH(op2_eew),
                                         Seq(op2_data( 8*e+7,   8*e).sextTo(eLen >> 1),
                                             op2_data(16*e+15, 16*e).sextTo(eLen >> 1),
                                             op2_data(32*e+31, 32*e),
                                             0.U)),
                                   Mux1H(UIntToOH(op2_eew),
                                         Seq(op2_data( 8*e+7,   8*e) & shiftMask,
                                             op2_data(16*e+15, 16*e) & shiftMask,
                                             op2_data(32*e+31, 32*e) & shiftMask, 0.U))))
      alu.io.ci  := Mux(withCarry && !uop.v_unmasked, rvm_data(e), false.B) // FIXME
    } else if (e < numELENinVLEN*4) {
      // e16 ALU
      alu.io.in1 := Mux(isMerge,   0.U,
                    Mux(op1Signed, Mux(op1_eew(0), op1_data(16*e+15, 16*e), op1_data(8*e+7, 8*e).sextTo(eLen >> 2)),
                                   Mux(op1_eew(0), op1_data(16*e+15, 16*e), op1_data(8*e+7, 8*e))))
      alu.io.in2 := Mux(isMerge,   Mux(op2_eew(0), op1_data(16*e+15, 16*e) & Fill(16, !rvm_data(e)) | op2_data(16*e+15, 16*e) & Fill(16, rvm_data(e)),
                                                   op1_data( 8*e+7,   8*e) & Fill( 8, !rvm_data(e)) | op2_data( 8*e+7,   8*e) & Fill( 8, rvm_data(e))),
                    Mux(op2Signed, Mux(op2_eew(0), op2_data(16*e+15, 16*e), op2_data(8*e+7, 8*e).sextTo(eLen >> 2)),
                                   Mux(op2_eew(0), op2_data(16*e+15, 16*e) & shiftMask, op2_data(8*e+7, 8*e) & shiftMask)))
      alu.io.ci  := Mux(withCarry && !uop.v_unmasked, rvm_data(e), false.B) // FIXME
    } else {
      // e8 ALU
      alu.io.in1 := Mux(isMerge, 0.U, op1_data(8*e+7, 8*e))
      alu.io.in2 := Mux(isMerge, op1_data(8*e+7, 8*e) & Fill(8, !rvm_data(e)) | op2_data(8*e+7, 8*e) & Fill(8, rvm_data(e)),
                                 op2_data(8*e+7, 8*e) & shiftMask)
      alu.io.ci  := Mux(withCarry && !uop.v_unmasked, rvm_data(e), false.B) // FIXME
    }

    // output
    if (e < numELENinVLEN) {
      e64_adder_out(e) := Mux(isInvert, ~alu.io.out, alu.io.out)
      e64_cmp_out(e)   := alu.io.cmp_out
      e64_co(e)        := alu.io.co
    }
    if (e < numELENinVLEN*2) {
      e32_adder_out(e) := Mux(isInvert, ~alu.io.out, alu.io.out)
      e32_cmp_out(e)   := alu.io.cmp_out
      e32_co(e)        := alu.io.co
    }
    if (e < numELENinVLEN*4) {
      e16_adder_out(e) := Mux(isInvert, ~alu.io.out, alu.io.out)
      e16_cmp_out(e)   := alu.io.cmp_out
      e16_co(e)        := alu.io.co
    }
    e8_adder_out(e) := Mux(isInvert, ~alu.io.out, alu.io.out)
    e8_cmp_out(e)   := alu.io.cmp_out
    e8_co(e)        := alu.io.co
  }

  // Did I just get killed by the previous cycle's branch,
  // or by a flush pipeline?
  val killed = WireInit(false.B)
  when (io.req.bits.kill || IsKilledByBranch(io.brupdate, uop)) {
    killed := true.B
  }

  val r_val   = RegInit(VecInit(Seq.fill(numStages) { false.B }))
  val r_data  = Reg(Vec(numStages, UInt(vLen.W)))
  val r_mask  = Reg(Vec(numStages, UInt(vLenb.W))) 
  val r_pred  = Reg(Vec(numStages, Bool()))
  val alu_out = WireInit(0.U(vLen.W))

  alu_out := Mux(isVMADC, Mux1H(UIntToOH(uop.vd_eew),
                                Seq(e8_co.asUInt, e16_co.asUInt, e32_co.asUInt, e64_co.asUInt)),
             Mux(uop.rt(RD, isMaskVD), Mux1H(UIntToOH(uop.vd_eew),
                                Seq(e8_cmp_out.asUInt, e16_cmp_out.asUInt, e32_cmp_out.asUInt, e64_cmp_out.asUInt)),
                          Mux1H(UIntToOH(uop.vd_eew),
                                Seq(e8_adder_out.asUInt, e16_adder_out.asUInt, e32_adder_out.asUInt,e64_adder_out.asUInt))))

  val isNarrowOdd = uop.rt(RS2, isWidenV) && uop.v_eidx((vLenbSz-1).U-uop.vd_eew) === 1.U
  r_val (0) := io.req.valid && !killed
  r_data(0) := Mux(isVMask,              rs3_data & bitInactive | alu_out & ~bitInactive,
               Mux(uop.rt(RD, isMaskVD), rs3_data & Cat(Fill(vLen-vLenb, 1.U(1.W)), inactive) | alu_out & ~inactive,
               Mux(isNarrowOdd,          Cat((0 until vLenb/2).map(b => Mux(byteInactive(b), rs3_data(b*8+7, b*8), alu_out(b*8+7, b*8))).reverse) ## Fill(vLen/2, 0.U(1.W)),
                                         Cat((0 until vLenb).map(b => Mux(byteInactive(b), rs3_data(b*8+7, b*8), alu_out(b*8+7, b*8))).reverse))))
  r_mask(0) := Mux(isNarrowOdd, Fill(vLenb/2, 1.U(1.W)) ## Fill(vLenb/2, 0.U(1.W)), Fill(vLenb, 1.U(1.W)))
  r_pred(0) := uop.is_sfb_shadow && io.req.bits.pred_data
  for (i <- 1 until numStages) {
    r_val(i)  := r_val(i-1)
    r_data(i) := r_data(i-1)
    r_mask(i) := r_mask(i-1)
    r_pred(i) := r_pred(i-1)
  }
  io.resp.bits.data  := r_data(numStages-1)
  io.resp.bits.vmask := r_mask(numStages-1)
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

  val isNarrowOdd = io.resp.bits.uop.rt(RS2, isWidenV) && (io.resp.bits.uop.v_eidx((vLenbSz - 1).U - io.resp.bits.uop.vd_eew) === 1.U)

  io.resp.bits.data              := Mux(isNarrowOdd, fpu.io.resp.bits.data << (vLen / 2),fpu.io.resp.bits.data)
  io.resp.bits.fflags.valid      := fpu.io.resp.bits.fflags.valid
  io.resp.bits.fflags.bits.uop   := io.resp.bits.uop
  io.resp.bits.fflags.bits.flags := fpu.io.resp.bits.fflags.bits.flags
  io.resp.bits.vmask := Mux(isNarrowOdd, Fill(vLenb / 2, 1.U(1.W)) ## Fill(vLenb / 2, 0.U(1.W)), Fill(vLenb, 1.U(1.W)))


}

// Vector Reduction-Permutation Assist
class VecRPAssist()(implicit p: Parameters) extends BoomModule {
  val io = IO(new Bundle {
    val brupdate = Input(new BrUpdateInfo())
    val exreq = Flipped(DecoupledIO(new FuncUnitReq(vLen)))
    val fbreq = DecoupledIO(new FuncUnitReq(vLen))
    val fbrsp = Flipped(DecoupledIO(new FuncUnitResp(vLen)))
    val busy  = Output(Bool())
  })
  val v1buf       = Reg(UInt(vLen.W))
  val v2buf       = Reg(Vec(8, UInt(vLen.W)))
  val v2buf_wp    = VRegSel(io.exreq.bits.uop.v_eidx, io.exreq.bits.uop.vs2_eew, eLenSelSz)
  //val v2bufvld    = RegInit(Vec(0.U(8.W).asBools))
  val vdbuf       = Reg(Vec(8, UInt(vLen.W)))
  val vdbuf_wp    = VRegSel(io.exreq.bits.uop.v_eidx, io.exreq.bits.uop.vd_eew, eLenSelSz)
  //val vdbufvld    = RegInit(Vec(0.U(8.W).asBools))
  //val vdmux       = Wire(Vec(vLenb, UInt(8.W)))
  //val vdmux_sel   = Wire(Vec(vLenb, UInt(vLen.W)))
  val vmbuf       = Reg(Vec(8, UInt(vLenb.W)))
  val progress    = RegInit(0.U((vLenSz+1).W))
  val uop_v       = RegInit(false.B)
  val uop         = Reg(new MicroOp)
  val s_idle :: s_fill :: s_work :: Nil = Enum(3)
  val state       = RegInit(s_idle)
  val is_idle     = (state === s_idle)
  val filling     = (state === s_fill)
  val working     = (state === s_work)
  val is_last     = WireInit(false.B)
  val vlen_ecnt   = vLenb.U >> uop.vd_eew
  // unordered reduce phase1: compress between vreg: v2buf[]
  // unordered reduce phase2: compress within vreg: fbrsp
  val ured_ph1_prgrs = nrVecGroup(uop.vs2_emul) << uop.rt(RD, isWidenV).asUInt
  val ured_ph2_prgrs = (vLenSz-3).U - uop.vd_eew //+ uop.rt(RD, isWidenV).asUInt - 1.U
  val is_vrgather = uop.uopc === uopVRGATHER
  val is_vcompress= uop.uopc === uopVCOMPRESS
  val is_slide1up = uop.uopc === uopVSLIDE1UP
  val is_slide1dn = uop.uopc === uopVSLIDE1DOWN
  val is_slideup  = uop.uopc === uopVSLIDEUP
  val is_slidedn  = uop.uopc === uopVSLIDEDOWN
  def v2red_masked(u: MicroOp, vm: UInt, data: UInt): UInt = {
    val ret = Wire(UInt(vLen.W))
    val p = VRegSel(u.v_eidx, u.vs2_eew, eLenSelSz)
    val e8red_identity: UInt = Mux1H(Seq(
      u.uopc.isOneOf(uopVMAX) -> 0x80.U(8.W),
      u.uopc.isOneOf(uopVMIN) -> 0x7F.U(8.W),
      u.uopc.isOneOf(uopVAND, uopVMINU) -> ~(0.U(8.W))
      // omit default clause because Mux1H returns zero when nothing matches
      //u.uopc.isOneOf(uopVADD, uopVMAXU, uopVOR, uopVXOR) -> 0.U(8.W)
    ))
    val e16red_identity: UInt = Mux1H(Seq(
      u.uopc.isOneOf(uopVMAX) -> 0x8000.U(16.W),
      u.uopc.isOneOf(uopVMIN) -> 0x7FFF.U(16.W),
      u.uopc.isOneOf(uopVFMAX)-> 0xFC00.U(16.W),
      u.uopc.isOneOf(uopVFMIN)-> 0x7C00.U(16.W),
      u.uopc.isOneOf(uopVAND, uopVMINU) -> ~(0.U(16.W))
      //u.uopc.isOneOf(uopVADD, uopVMAXU, uopVOR, uopVXOR, uopVFADD) -> 0.U(16.W)
    ))
    val e32red_identity: UInt = Mux1H(Seq(
      u.uopc.isOneOf(uopVMAX) -> 0x80000000L.U(32.W),
      u.uopc.isOneOf(uopVMIN) -> 0x7FFFFFFFL.U(32.W),
      u.uopc.isOneOf(uopVFMAX)-> 0xFF800000L.U(32.W),
      u.uopc.isOneOf(uopVFMIN)-> 0x7F800000L.U(32.W),
      u.uopc.isOneOf(uopVAND, uopVMINU) -> ~(0.U(32.W))
      //u.uopc.isOneOf(uopVADD, uopVMAXU, uopVOR, uopVXOR, uopVFADD) -> 0.U(32.W)
    ))
    require(eLen==64)
    val e64red_identity: UInt = Mux1H(Seq(
        u.uopc.isOneOf(uopVMAX) -> Cat(1.U(1.W), 0.U((eLen-1).W)), // signed low limit
        u.uopc.isOneOf(uopVMIN) -> Cat(0.U(1.W), Fill(eLen-1, 1.U(1.W))), // signed high limit
        u.uopc.isOneOf(uopVFMAX)-> Cat(0xFFF.U(12.W), Fill(eLen-12, 0.U(1.W))), // -INF
        u.uopc.isOneOf(uopVFMIN)-> Cat(0x7FF.U(12.W), Fill(eLen-12, 0.U(1.W))), // +INF
        u.uopc.isOneOf(uopVAND, uopVMINU) -> ~(0.U(eLen.W))
        // omit default clause because Mux1H returns zero when nothing matches
        //u.uopc.isOneOf(uopVADD, uopVMAXU, uopVOR, uopVXOR, uopVFADD) -> 0.U(eLen.W)
    ))
    ret := Mux1H(Seq(
      (u.vs2_eew(1,0) === 0.U) -> Cat((0 until vLen/8).map(i => {
        val eidx = Cat(p(2,0), i.U((vLenSz-3).W))
        val actv = (u.v_unmasked || vm(i)) && eidx < u.vconfig.vl
        Mux(actv, data(i*8+7, i*8), e8red_identity(7,0))
      }).reverse),
      (u.vs2_eew(1,0) === 1.U) -> Cat((0 until vLen/16).map(i => {
        val eidx = Cat(p(2,0), i.U((vLenSz-4).W))
        val actv = (u.v_unmasked || vm(i)) && eidx < u.vconfig.vl
        Mux(actv, data(i*16+15, i*16), e16red_identity(15,0))
      }).reverse),
      (u.vs2_eew(1,0) === 2.U) -> Cat((0 until vLen/32).map(i => {
        val eidx = Cat(p(2,0), i.U((vLenSz-5).W))
        val actv = (u.v_unmasked || vm(i)) && eidx < u.vconfig.vl
        Mux(actv, data(i*32+31, i*32), e32red_identity(31,0))
      }).reverse),
      (u.vs2_eew(1,0) === 3.U) -> Cat((0 until vLen/64).map(i => {
        val eidx = Cat(p(2,0), i.U((vLenSz-6).W))
        val actv = (u.v_unmasked || vm(i)) && eidx < u.vconfig.vl
        Mux(actv, data(i*64+63, i*64), e64red_identity(63,0))
      }).reverse)
    ))
    ret
  }

  when (io.exreq.valid && !working) {
    vmbuf(v2buf_wp) := io.exreq.bits.rvm_data
    v2buf(v2buf_wp) := Mux(io.exreq.bits.uop.is_reduce,
                           v2red_masked(io.exreq.bits.uop, io.exreq.bits.rvm_data, io.exreq.bits.rs2_data),
                           io.exreq.bits.rs2_data)
    vdbuf(vdbuf_wp) := io.exreq.bits.rs3_data
  }

  when (io.exreq.valid && is_idle) {
    v1buf := io.exreq.bits.rs1_data
  }

  switch(state) {
    is (s_idle) {
      when (io.exreq.valid) {
        state     := Mux(nrVecGroup(io.exreq.bits.uop.vs2_emul) > 1.U, s_fill, s_work)
        uop_v     := true.B
        uop       := io.exreq.bits.uop
        progress  := Mux(io.exreq.bits.uop.is_ureduce, 1.U, 0.U)
      }
    }
    is (s_fill) {
      when (io.exreq.valid && v2buf_wp +& 1.U === nrVecGroup(uop.vs2_emul)) {
        state := s_work
      }
    }
    is (s_work) {
      when (io.fbreq.fire) {
        is_last := Mux(uop.is_ureduce, ured_ph1_prgrs + ured_ph2_prgrs === progress,
                   Mux(uop.is_oreduce, uop.vconfig.vl <= progress + 1.U,
                   Mux(is_vrgather,    nrVecGroup(uop.vs1_emul) === progress,
                                       nrVecGroup(uop.vd_emul) === progress)))
        when (is_last) {
          state := s_idle
        }
        progress := progress + 1.U
      }
    }
  }

  when ((IsKilledByBranch(io.brupdate, io.exreq.bits.uop) ||
         IsKilledByBranch(io.brupdate, io.fbrsp.bits.uop) ||
         IsKilledByBranch(io.brupdate, uop) ||
         io.exreq.bits.kill) && uop_v) {
    state := s_idle
    uop_v := false.B
  }

  when (uop_v) {
    uop.br_mask := GetNewBrMask(io.brupdate, uop)
  }

  io.exreq.ready := true.B
  io.fbrsp.ready := true.B
  io.fbreq.valid := working && (uop.is_ureduce && (io.fbrsp.valid || progress === 1.U) ||
                                uop.is_oreduce && (io.fbrsp.valid || progress === 0.U))
  io.fbreq.bits.uop := uop
  io.fbreq.bits.uop.v_split_last := is_last
  when (uop.is_ureduce) {
    io.fbreq.bits.uop.vs1_eew       := Mux(uop.rt(RD, isWidenV) && uop.fp_val && progress < ured_ph1_prgrs, uop.vs2_eew, uop.vs1_eew)
    io.fbreq.bits.uop.vs2_eew       := Mux(uop.rt(RD, isWidenV) && uop.fp_val && progress === 1.U, uop.vs2_eew, uop.vd_eew)
    io.fbreq.bits.uop.lrs1_rtype    := Mux(uop.rt(RD, isWidenV) && uop.fp_val && progress < ured_ph1_prgrs, uop.lrs2_rtype, uop.lrs1_rtype)
    io.fbreq.bits.uop.lrs2_rtype    := Mux(uop.rt(RD, isWidenV) && uop.fp_val && progress === 1.U, uop.lrs2_rtype, uop.dst_rtype)
    io.fbreq.bits.uop.v_unmasked    := true.B // treat unordered REDops as unmasked
    io.fbreq.bits.uop.v_eidx        := 0.U
    io.fbreq.bits.uop.vconfig.vl    := Mux(progress < ured_ph1_prgrs, vlen_ecnt,
                                       Mux(is_last, Mux(uop.vconfig.vl === 0.U, 0.U, 1.U), vlen_ecnt >> (1.U + progress - ured_ph1_prgrs)))
    io.fbreq.bits.uop.v_split_ecnt  := vlen_ecnt
  } .elsewhen (uop.is_oreduce) {
    io.fbreq.bits.uop.v_unmasked    := true.B // treat ordered REDops as unmasked
    io.fbreq.bits.uop.v_eidx        := 0.U
    io.fbreq.bits.uop.vconfig.vl    := Mux(uop.vconfig.vl === 0.U, 0.U, 1.U)
    io.fbreq.bits.uop.v_split_ecnt  := vlen_ecnt
  }
  when(uop.is_reduce && is_last) {
    io.fbreq.bits.uop.fu_code       := uop.fu_code & (~FU_VRP)
  }
  val v1uredmux = Mux1H(Seq(
    (uop.vd_eew(1,0) === 0.U) -> // assert(!uop.rt(RD, isWidenV))
      Mux(is_last, Cat(0.U((vLen-8).W), v1buf(7, 0)),
      Mux(progress === 1.U && ured_ph1_prgrs === 1.U, Cat(0.U((vLen/2).W),  Cat((0 until vLen/16).map(i => v2buf(0)(i*16+15, i*16+8)).reverse)),
      Mux(progress < ured_ph1_prgrs, v2buf(progress), Cat(0.U((vLen/2).W),  Cat((0 until vLen/16).map(i => io.fbrsp.bits.data(i*16+15, i*16+8)).reverse))))),
    (uop.vd_eew(1,0) === 1.U) -> {
      val v2m = v2buf(progress >> 1.U)
      Mux(is_last, Cat(0.U((vLen-16).W), v1buf(15,0)),
      Mux(progress === 1.U && ured_ph1_prgrs === 1.U, Cat(0.U((vLen/2).W),  Cat((0 until vLen/32).map(i => v2buf(0)(i*32+31, i*32+16)).reverse)),
      Mux(progress < ured_ph1_prgrs, Mux(uop.rt(RD, isWidenV), Cat((0 until vLen/16).map(i => Cext(Mux(progress(0), v2m(vLen/2+i*8+7, vLen/2+i*8), v2m(i*8+7, i*8)), uop.rt(RD, isUnsignedV), 16)).reverse),
                                                               v2buf(progress)),
                                     Cat(0.U((vLen/2).W),  Cat((0 until vLen/32).map(i => io.fbrsp.bits.data(i*32+31, i*32+16)).reverse)))))
    },
    (uop.vd_eew(1,0) === 2.U) -> {
      val v2m = v2buf(progress >> 1.U)
      Mux(is_last, Cat(0.U((vLen-32).W), v1buf(31, 0)),
      Mux(progress === 1.U && ured_ph1_prgrs === 1.U, Cat(0.U((vLen/2).W),  Cat((0 until vLen/64).map(i => v2buf(0)(i*64+63, i*64+32)).reverse)),
      Mux(progress < ured_ph1_prgrs, Mux(uop.rt(RD, isWidenV), Mux(!uop.fp_val, Cat((0 until vLen/32).map(i => Cext(Mux(progress(0), v2m(vLen/2+i*16+15, vLen/2+i*16), v2m(i*16+15, i*16)), uop.rt(RD, isUnsignedV), 32)).reverse),
                                                                                Cat(0.U((vLen/2).W), Cat((0 until vLen/32).map(i => Mux(progress(0), v2m(vLen/2+i*16+15, vLen/2+i*16), v2m(i*16+15, i*16))).reverse))),
                                                               v2buf(progress)),
                                     Cat(0.U((vLen/2).W),  Cat((0 until vLen/64).map(i => io.fbrsp.bits.data(i*64+63, i*64+32)).reverse)))))
    },
    (uop.vd_eew(1,0) === 3.U) -> {
      val v2m = v2buf(progress >> 1.U)
      Mux(is_last, Cat(0.U((vLen-64).W), v1buf(63, 0)),
      Mux(progress === 1.U && ured_ph1_prgrs === 1.U, Cat(0.U((vLen/2).W),  Cat((0 until vLen/128).map(i => v2buf(0)(i*128+127, i*128+64)).reverse)),
      Mux(progress < ured_ph1_prgrs, Mux(uop.rt(RD, isWidenV), Mux(!uop.fp_val, Cat((0 until vLen/64).map(i => Cext(Mux(progress(0), v2m(vLen/2+i*32+31, vLen/2+i*32), v2m(i*32+31, i*32)), uop.rt(RD, isUnsignedV), 64)).reverse),
                                                                                Cat(0.U((vLen/2).W), Cat((0 until vLen/64).map(i => Mux(progress(0), v2m(vLen/2+i*32+31, vLen/2+i*32), v2m(i*32+31, i*32))).reverse))),
                                                               v2buf(progress)),
                                     Cat(0.U((vLen/2).W),  Cat((0 until vLen/128).map(i => io.fbrsp.bits.data(i*128+127, i*128+64)).reverse)))))
    }
  ))
  val v2uredmux = Mux1H(Seq(
    (uop.vd_eew(1,0) === 0.U) ->
      Mux(progress === 1.U, Mux(ured_ph1_prgrs === 1.U, Cat(0.U((vLen/2).W), Cat((0 until vLen/16).map(i => v2buf(0)(i*16+7, i*16)).reverse)), v2buf(0)),
      Mux(progress < ured_ph1_prgrs,  io.fbrsp.bits.data,
      Cat(0.U((vLen/2).W),  Cat((0 until vLen/16).map(i => io.fbrsp.bits.data(i*16+7, i*16)).reverse)))),
    (uop.vd_eew(1,0) === 1.U) -> {
      val v2m = v2buf(0.U)
      Mux(progress === 1.U, Mux(uop.rt(RD, isWidenV), Cat((0 until vLen/16).map(i => Cext(v2m(i*8+7, i*8), uop.rt(RD, isUnsignedV), 16)).reverse),
                            Mux(ured_ph1_prgrs === 1.U, Cat(0.U((vLen/2).W), Cat((0 until vLen/32).map(i => v2buf(0)(i*32+15, i*32)).reverse)), v2buf(0))),
      Mux(progress < ured_ph1_prgrs, io.fbrsp.bits.data,
      Cat(0.U((vLen/2).W),  Cat((0 until vLen/32).map(i => io.fbrsp.bits.data(i*32+15, i*32)).reverse))))
    },
    (uop.vd_eew(1,0) === 2.U) -> {
      val v2m = v2buf(0.U)
      Mux(progress === 1.U, Mux(uop.rt(RD, isWidenV), Mux(!uop.fp_val, Cat((0 until vLen/32).map(i => Cext(v2m(i*16+15, i*16), uop.rt(RD, isUnsignedV), 32)).reverse),
                                                                       Cat(0.U((vLen/2).W), Cat((0 until vLen/32).map(i => v2m(i*16+15, i*16)).reverse))),
                            Mux(ured_ph1_prgrs === 1.U, Cat(0.U((vLen/2).W), Cat((0 until vLen/64).map(i => v2buf(0)(i*64+31, i*64)).reverse)), v2buf(0))),
      Mux(progress < ured_ph1_prgrs, io.fbrsp.bits.data,
      Cat(0.U((vLen/2).W),  Cat((0 until vLen/64).map(i => io.fbrsp.bits.data(i*64+31, i*64)).reverse))))
    },
    (uop.vd_eew(1,0) === 3.U) -> {
      val v2m = v2buf(0.U)
      Mux(progress === 1.U, Mux(uop.rt(RD, isWidenV), Mux(!uop.fp_val, Cat((0 until vLen/64).map(i => Cext(v2m(i*32+31, i*32), uop.rt(RD, isUnsignedV), 64)).reverse),
                                                                       Cat(0.U((vLen/2).W), Cat((0 until vLen/64).map(i => v2m(i*32+31, i*32)).reverse))),
                            Mux(ured_ph1_prgrs === 1.U, Cat(0.U((vLen/2).W), Cat((0 until vLen/128).map(i => v2buf(0)(i*128+63, i*128)).reverse)), v2buf(0))),
      Mux(progress < ured_ph1_prgrs, io.fbrsp.bits.data,
      Cat(0.U((vLen/2).W),  Cat((0 until vLen/128).map(i => io.fbrsp.bits.data(i*128+63, i*128)).reverse))))
    }
  ))
  val v1oredmux = Mux1H(Seq(
    (uop.vd_eew(1,0) === 1.U) -> Cat(0.U((vLen-16).W), Mux(progress === 0.U, v1buf(15, 0), io.fbrsp.bits.data(15, 0))),
    (uop.vd_eew(1,0) === 2.U) -> Cat(0.U((vLen-32).W), Mux(progress === 0.U, v1buf(31, 0), io.fbrsp.bits.data(31, 0))),
    (uop.vd_eew(1,0) === 3.U) -> Cat(0.U((vLen-64).W), Mux(progress === 0.U, v1buf(63, 0), io.fbrsp.bits.data(63, 0)))
  ))
  val v2oredmux = Mux1H(Seq(
    (uop.vs2_eew(1,0) === 1.U) -> Cat(0.U((vLen-16).W), v2buf((progress>>6.U)(2,0))>>Cat(progress(5,0),0.U(4.W))),
    (uop.vs2_eew(1,0) === 2.U) -> Cat(0.U((vLen-32).W), v2buf((progress>>5.U)(2,0))>>Cat(progress(4,0),0.U(5.W))),
    (uop.vs2_eew(1,0) === 3.U) -> Cat(0.U((vLen-64).W), v2buf((progress>>4.U)(2,0))>>Cat(progress(3,0),0.U(6.W)))
  ))
  io.fbreq.bits.rs1_data := Mux(uop.is_ureduce, v1uredmux,
                            Mux(uop.is_oreduce, v1oredmux, v1buf))
  io.fbreq.bits.rs2_data := Mux(uop.is_ureduce, v2uredmux,
                            Mux(uop.is_oreduce, v2oredmux, v2buf(0)))
  io.fbreq.bits.rs3_data := Mux(uop.is_reduce,  vdbuf(0),  vdbuf(progress))
  io.busy := !is_idle
}
/**
 * Vector Divide functional units wrapper.
 *
 * @param dataWidth data to be passed into the functional unit
 */
class VecSRT4DivUnit(dataWidth: Int)(implicit p: Parameters) extends IterativeFunctionalUnit(dataWidth)
{
  val uop = io.req.bits.uop
  val rs1_data = io.req.bits.rs1_data
  val rs2_data = io.req.bits.rs2_data
  val rs3_data = io.req.bits.rs3_data
  val rvm_data = io.req.bits.rvm_data
  val isSigned = !uop.rt(RS2, isUnsignedV)
  val body, prestart, tail, mask, inactive = Wire(UInt(vLenb.W))
  val vl = uop.vconfig.vl
  prestart := Cat((0 until vLenb).map(b => uop.v_eidx + b.U < uop.vstart).reverse)
  body     := Cat((0 until vLenb).map(b => uop.v_eidx + b.U >= uop.vstart && uop.v_eidx + b.U < vl).reverse)
  mask     := Mux(uop.v_unmasked, ~(0.U(vLenb.W)), Cat((0 until vLenb).map(b => rvm_data(b)).reverse))
  tail     := Cat((0 until vLenb).map(b => uop.v_eidx + b.U >= vl).reverse)
  inactive := prestart | body & ~mask | tail

  val divValid = Wire(Vec(vLenb, Bool()))
  val e64Out   = Wire(Vec(numELENinVLEN, UInt(64.W)))
  val e32Out   = Wire(Vec(numELENinVLEN*2, UInt(32.W)))
  val e16Out   = Wire(Vec(numELENinVLEN*4, UInt(16.W)))
  val e8Out    = Wire(Vec(numELENinVLEN*8, UInt(8.W)))

  val s_idle :: s_work :: s_done :: Nil = Enum(3)
  val state = RegInit(s_idle)
  switch(state) {
    is(s_idle) {
      when(io.req.valid && !this.do_kill) {
        state := s_work
      }
    }
    is(s_work) {
      when(divValid.andR && io.resp.ready) {
        state := s_idle
      } .elsewhen(divValid.andR) {
        state := s_done
      }
    }
    is(s_done) {
      when(io.resp.ready) { state := s_idle }
    }
  }
  when(this.do_kill) { state := s_idle }

  for(e <- 0 until vLenb) {
    val div = if(e < numELENinVLEN)        Module(new SRT4DividerDataModule(len = eLen))
              else if(e < numELENinVLEN*2) Module(new SRT4DividerDataModule(len = eLen >> 1))
              else if(e < numELENinVLEN*4) Module(new SRT4DividerDataModule(len = eLen >> 2))
              else                         Module(new SRT4DividerDataModule(len = eLen >> 3))
    div.io.valid  := io.req.valid && !this.do_kill
    div.io.sign   := uop.uopc.isOneOf(uopVDIV, uopVREM)
    div.io.isHi   := Mux(inactive(e), false.B, uop.uopc.isOneOf(uopVREM, uopVREMU))
    div.io.isW    := !uop.ctrl.fcn_dw
    div.io.kill_w := this.do_kill
    div.io.kill_r := this.do_kill && !div.io.in_ready
    divValid(e)   := div.io.out_valid
    div.io.out_ready := (divValid.andR && io.resp.ready)

    if(e < numELENinVLEN) {
      div.io.src(0) := Mux(inactive(e), Mux(isSigned, Mux1H(UIntToOH(uop.vd_eew),
                                                            Seq(rs3_data( 8*e+7,   8*e).sextTo(eLen),
                                                                rs3_data(16*e+15, 16*e).sextTo(eLen),
                                                                rs3_data(32*e+31, 32*e).sextTo(eLen),
                                                                rs3_data(64*e+63, 64*e))),
                                                      Mux1H(UIntToOH(uop.vd_eew),
                                                            Seq(rs3_data(8*e+7, 8*e), rs3_data(16*e+15, 16*e), rs3_data(32*e+31, 32*e), rs3_data(64*e+63, 64*e)))),
                                        Mux(isSigned, Mux1H(UIntToOH(uop.vd_eew),
                                                            Seq(rs2_data( 8*e+7,   8*e).sextTo(eLen),
                                                                rs2_data(16*e+15, 16*e).sextTo(eLen),
                                                                rs2_data(32*e+31, 32*e).sextTo(eLen),
                                                                rs2_data(64*e+63, 64*e))),
                                                      Mux1H(UIntToOH(uop.vd_eew),
                                                            Seq(rs2_data(8*e+7, 8*e), rs2_data(16*e+15, 16*e), rs2_data(32*e+31, 32*e), rs2_data(64*e+63, 64*e)))))
      div.io.src(1) := Mux(inactive(e), 1.U,
                                        Mux(isSigned, Mux1H(UIntToOH(uop.vd_eew),
                                                            Seq(rs1_data( 8*e+7,   8*e).sextTo(eLen),
                                                                rs1_data(16*e+15, 16*e).sextTo(eLen),
                                                                rs1_data(32*e+31, 32*e).sextTo(eLen),
                                                                rs1_data(64*e+63, 64*e))),
                                                      Mux1H(UIntToOH(uop.vd_eew),
                                                            Seq(rs1_data(8*e+7, 8*e), rs1_data(16*e+15, 16*e), rs1_data(32*e+31, 32*e), rs1_data(64*e+63, 64*e)))))
    } else if(e < numELENinVLEN*2) {
      div.io.src(0) := Mux(inactive(e), Mux(isSigned, Mux1H(UIntToOH(uop.vd_eew),
                                                            Seq(rs3_data( 8*e+7,   8*e).sextTo(eLen >> 1),
                                                                rs3_data(16*e+15, 16*e).sextTo(eLen >> 1),
                                                                rs3_data(32*e+31, 32*e),
                                                                0.U)),
                                                      Mux1H(UIntToOH(uop.vd_eew),
                                                            Seq(rs3_data(8*e+7, 8*e), rs3_data(16*e+15, 16*e), rs3_data(32*e+31, 32*e), 0.U))),
                                        Mux(isSigned, Mux1H(UIntToOH(uop.vd_eew),
                                                            Seq(rs2_data( 8*e+7,   8*e).sextTo(eLen >> 1),
                                                                rs2_data(16*e+15, 16*e).sextTo(eLen >> 1),
                                                                rs2_data(32*e+31, 32*e),
                                                                0.U)),
                                                      Mux1H(UIntToOH(uop.vd_eew),
                                                            Seq(rs2_data(8*e+7, 8*e), rs2_data(16*e+15, 16*e), rs2_data(32*e+31, 32*e), 0.U))))
      div.io.src(1) := Mux(inactive(e), 1.U,
                                        Mux(isSigned, Mux1H(UIntToOH(uop.vd_eew),
                                                            Seq(rs1_data( 8*e+7,   8*e).sextTo(eLen >> 1),
                                                                rs1_data(16*e+15, 16*e).sextTo(eLen >> 1),
                                                                rs1_data(32*e+31, 32*e),
                                                                1.U)),
                                                      Mux1H(UIntToOH(uop.vd_eew),
                                                            Seq(rs1_data(8*e+7, 8*e), rs1_data(16*e+15, 16*e), rs1_data(32*e+31, 32*e), 1.U))))
    } else if(e < numELENinVLEN*4) {
      div.io.src(0) := Mux(inactive(e), Mux(isSigned, Mux(uop.vd_eew(0), rs3_data(16*e+15, 16*e), rs3_data(8*e+7, 8*e).sextTo(eLen >> 2)),
                                                      Mux(uop.vd_eew(0), rs3_data(16*e+15, 16*e), rs3_data(8*e+7, 8*e))),
                                        Mux(isSigned, Mux(uop.vd_eew(0), rs2_data(16*e+15, 16*e), rs2_data(8*e+7, 8*e).sextTo(eLen >> 2)),
                                                      Mux(uop.vd_eew(0), rs2_data(16*e+15, 16*e), rs2_data(8*e+7, 8*e))))
      div.io.src(1) := Mux(inactive(e), 1.U,
                                        Mux(isSigned, Mux(uop.vd_eew(0), rs1_data(16*e+15, 16*e), rs1_data(8*e+7, 8*e).sextTo(eLen >> 2)),
                                                      Mux(uop.vd_eew(0), rs1_data(16*e+15, 16*e), rs1_data(8*e+7, 8*e))))
    } else {
      div.io.src(0) := Mux(inactive(e), rs3_data(8*e+7, 8*e), rs2_data(8*e+7, 8*e))
      div.io.src(1) := Mux(inactive(e), 1.U, rs1_data(8*e+7, 8*e))
    }

    // output
    if (e < numELENinVLEN) {
      e64Out(e) := div.io.out_data
    }
    if(e < numELENinVLEN*2) {
      e32Out(e) := div.io.out_data
    }
    if(e < numELENinVLEN*4) {
      e16Out(e) := div.io.out_data
    }
    e8Out(e) := div.io.out_data
  }

  // output
  io.req.ready  := (state === s_idle)
  io.resp.valid := divValid.andR && !this.do_kill
  io.resp.bits.vmask := Fill(vLenb, 1.U(1.W))
  io.resp.bits.data := Mux1H(UIntToOH(io.resp.bits.uop.vd_eew),
                             Seq(e8Out.asUInt, e16Out.asUInt, e32Out.asUInt, e64Out.asUInt))
}

/**
 * Vector VMASK_UNIT Wrapper
 *
 * @param numStages how many pipeline stages does the functional unit have
 * @param dataWidth width of the data being operated on in the functional unit
 */
@chiselName
class VecMaskUnit(
  numStages: Int = 2,
  dataWidth: Int)
(implicit p: Parameters)
  extends PipelinedFunctionalUnit(
    numStages = numStages,
    numBypassStages = 0,
    earliestBypassStage = 0,
    dataWidth = dataWidth)
  with boom.ifu.HasBoomFrontendParameters
{
  val uop = io.req.bits.uop
  val rs1_data = io.req.bits.rs1_data      // rvm data in vpopc/vfirst/vmsbf/vmsof/vmsif instructions
  val rs2_data = io.req.bits.rs2_data
  val rs3_data = io.req.bits.rs3_data
  val rvm_data = io.req.bits.rvm_data

  val vmaskOut = Wire(Vec(numELENinVLEN, UInt(eLen.W)))
  val indexOut = Wire(Vec(numELENinVLEN, UInt(vLenSz.W)))
  val indexVal = Wire(Vec(numELENinVLEN, Bool()))

  // ---------------------------------------------------
  // vpopc, vfirst, vmsbf, vmsof, vmsif
  // ---------------------------------------------------
  val tailBits   = (Fill(vLen, 1.U(1.W)) << uop.vconfig.vl)(vLen-1, 0)
  val activeBits = Mux(uop.v_unmasked, ~tailBits, rs1_data & ~tailBits)
  for (e <- 0 until numELENinVLEN) {
    val vmaskUnit = Module(new VMaskUnit(offset = eLen*e))
    vmaskUnit.io.in1  := rs2_data(64*e+63, 64*e)
    vmaskUnit.io.in2  := rs3_data(64*e+63, 64*e)
    vmaskUnit.io.mask := activeBits(64*e+63, 64*e)
    vmaskUnit.io.fn   := uop.ctrl.op_fcn

    vmaskOut(e) := vmaskUnit.io.out
    indexOut(e) := vmaskUnit.io.index.bits
    indexVal(e) := vmaskUnit.io.index.valid
  }
  // special vmask output
  val hasNoSets    = (rs2_data & activeBits).orR === 0.U
  val vmaskOutTail = rs3_data & ~activeBits
  val vmaskOutSpc  = Mux(uop.uopc === uopVMSOF, vmaskOutTail, rs3_data | activeBits)

  // stage1
  val vpopcSt1 = Reg(Vec(numELENinVLEN/2, UInt(vLenSz.W)))
  for(e <- 0 until numELENinVLEN/2) {
    vpopcSt1(e) := (0 until 2).map(j => indexOut(e*2+j)).reduce(_ + _)
  }
  val indexOutSt1  = RegNext(indexOut)
  val indexValSt1  = RegNext(indexVal)
  val vfirstIdxMux = PriorityEncoder(indexValSt1)
  val vfirstIdxSt1 = Mux(indexValSt1.orR, indexOutSt1(vfirstIdxMux), Fill(xLen, 1.U(1.W)))
  val vmaskOutNorm = Reg(Vec(numELENinVLEN, UInt(eLen.W)))
  vmaskOutNorm(0) := vmaskOut(0)
  for(e <- 1 until numELENinVLEN) {
    vmaskOutNorm(e) := Mux((0 until e).map(j => indexVal(j)).orR, 0.U, vmaskOut(e))
  }
  val vmaskOutSt1 = Mux(RegNext(hasNoSets), RegNext(vmaskOutSpc), vmaskOutNorm.asUInt | RegNext(vmaskOutTail))

  // stage2
  val vpopcSt2 = Reg(UInt(xLen.W))
  vpopcSt2 := vpopcSt1.reduce(_ + _)

  val vfirstIdxSt2 = RegNext(vfirstIdxSt1)
  val vmaskOutSt2  = RegNext(vmaskOutSt1)

  // ---------------------------------------------------
  // viota, vid
  // ---------------------------------------------------
  val vstart = Mux(uop.uopc === uopVID, uop.vstart, 0.U)
  val body   = Cat((0 until vLen/8).map(b => uop.v_eidx + b.U >= vstart && uop.v_eidx + b.U < uop.vconfig.vl).reverse)
  val maskIn = Mux(uop.v_unmasked, body, body & rvm_data)
  val e64Out = Wire(Vec(numELENinVLEN,   UInt(64.W)))
  val e32Out = Wire(Vec(numELENinVLEN*2, UInt(32.W)))
  val e16Out = Wire(Vec(numELENinVLEN*4, UInt(16.W)))
  val e8Out  = Wire(Vec(numELENinVLEN*8, UInt( 8.W)))
  val baseIdx = RegInit(0.U(eLen.W))
  when(io.req.valid && uop.v_split_last) {
    baseIdx := 0.U
  } .elsewhen(io.req.valid && uop.uopc === uopVID) {
    baseIdx := baseIdx + uop.v_split_ecnt
  } .elsewhen(io.req.valid && uop.uopc === uopVIOTA) {
    baseIdx := Mux1H(UIntToOH(uop.vd_eew),
                     Seq(e8Out(vLenb-1) + (maskIn(vLenb-1) & rs2_data(vLenb-1)),
                         e16Out(numELENinVLEN*4-1) + (maskIn(numELENinVLEN*4-1) & rs2_data(numELENinVLEN*4-1)),
                         e32Out(numELENinVLEN*2-1) + (maskIn(numELENinVLEN*2-1) & rs2_data(numELENinVLEN*2-1)),
                         e64Out(numELENinVLEN-1)   + (maskIn(numELENinVLEN-1)   & rs2_data(numELENinVLEN-1))))
  }
  for (e <- 0 until vLenb) {
    // instants
    val viotaUnit = if(e < numELENinVLEN)
                      Module(new VIotaUnit(maskWidth = e%eLen+1, offset = e, dataWidth = eLen))
                    else if(e < numELENinVLEN*2)
                      Module(new VIotaUnit(maskWidth = e%eLen+1, offset = e, dataWidth = eLen >> 1))
                    else if(e < numELENinVLEN*4)
                      Module(new VIotaUnit(maskWidth = e%eLen+1, offset = e, dataWidth = eLen >> 2))
                    else
                      Module(new VIotaUnit(maskWidth = e%eLen+1, offset = e, dataWidth = eLen >> 3))
    // inputs
    viotaUnit.io.fn := uop.ctrl.op_fcn
    if(e < numELENinVLEN) {
      viotaUnit.io.in   := rs2_data(e, 0)
      viotaUnit.io.mask := maskIn(e, 0)
      viotaUnit.io.base := baseIdx
    } else if(e < numELENinVLEN*2) {
      viotaUnit.io.in   := rs2_data(e, 0)
      viotaUnit.io.mask := maskIn(e, 0)
      viotaUnit.io.base := baseIdx
    } else if(e < numELENinVLEN*4) {
      viotaUnit.io.in   := rs2_data(e, 0)
      viotaUnit.io.mask := maskIn(e, 0)
      viotaUnit.io.base := baseIdx
    } else {
      viotaUnit.io.in   := rs2_data(e, 0)
      viotaUnit.io.mask := maskIn(e, 0)
      viotaUnit.io.base := baseIdx
    }

    // output
    if(e < numELENinVLEN) {
      e64Out(e) := viotaUnit.io.out
    }
    if(e < numELENinVLEN*2) {
      e32Out(e) := viotaUnit.io.out
    }
    if(e < numELENinVLEN*4) {
      e16Out(e) := viotaUnit.io.out
    }
    e8Out(e) := viotaUnit.io.out
  }

  // stage1
  val e64OutSt1  = RegNext(e64Out)
  val e32OutSt1  = RegNext(e32Out)
  val e16OutSt1  = RegNext(e16Out)
  val e8OutSt1   = RegNext(e8Out)
  val maskInSt1  = RegNext(maskIn)
  val rs3DataSt1 = RegNext(rs3_data)

  val e64OutMux = Wire(Vec(numELENinVLEN,   UInt(64.W)))
  val e32OutMux = Wire(Vec(numELENinVLEN*2, UInt(32.W)))
  val e16OutMux = Wire(Vec(numELENinVLEN*4, UInt(16.W)))
  val e8OutMux  = Wire(Vec(numELENinVLEN*8, UInt(8.W)))
  for(e <- 0 until vLenb) {
    if(e < numELENinVLEN) {
      e64OutMux(e) := Mux(maskInSt1(e), e64OutSt1(e), rs3DataSt1(64*e+63, 64*e))
    }
    if(e < numELENinVLEN*2) {
      e32OutMux(e) := Mux(maskInSt1(e), e32OutSt1(e), rs3DataSt1(32*e+31, 32*e))
    }
    if(e < numELENinVLEN*4) {
      e16OutMux(e) := Mux(maskInSt1(e), e16OutSt1(e), rs3DataSt1(16*e+15, 16*e))
      e8OutMux(e)  := Mux(maskInSt1(e), e8OutSt1(e),  rs3DataSt1( 8*e+7,   8*e))
    } else {
      e8OutMux(e)  := Mux(maskInSt1(e), Mux(RegNext(uop.uopc === uopVID), e8OutSt1(e),
                        e8OutSt1(e)), rs3DataSt1(8*e+7, 8*e))
    }
  }

  // stage2
  val e64OutSt2 = RegNext(e64OutMux)
  val e32OutSt2 = RegNext(e32OutMux)
  val e16OutSt2 = RegNext(e16OutMux)
  val e8OutSt2  = RegNext(e8OutMux)

  val respUop = Pipe(io.req.valid, io.req.bits.uop, 2).bits // numStages default is 2
  val viotaOut = Mux1H(UIntToOH(respUop.vd_eew),
                       Seq(e8OutSt2.asUInt, e16OutSt2.asUInt, e32OutSt2.asUInt, e64OutSt2.asUInt))

  // ----------------------------
  // output mux
  // ----------------------------
  val out = Mux(respUop.uopc === uopVPOPC,  vpopcSt2,
            Mux(respUop.uopc === uopVFIRST, vfirstIdxSt2,
            Mux(respUop.uopc.isOneOf(uopVID, uopVIOTA), viotaOut,
            Mux(respUop.uopc.isOneOf(uopVMSBF, uopVMSIF, uopVMSOF), vmaskOutSt2, 0.U))))

  io.resp.bits.data         := Pipe(true.B, out, numStages - 2).bits
  io.resp.bits.vmask        := Fill(vLenb, 1.U(1.W))
  io.resp.bits.predicated   := false.B
  io.resp.bits.fflags.valid := false.B
}

/**
 * Vector Int to FP conversion functional unit
 *
 * @param latency the amount of stages to delay by
 */
class VecIntToFPUnit(dataWidth: Int, latency: Int)(implicit p: Parameters)
  extends PipelinedFunctionalUnit(
    numStages = latency,
    numBypassStages = 0,
    earliestBypassStage = 0,
    dataWidth = dataWidth,
    needsFcsr = true)
  with tile.HasFPUParameters
{
  val uop = io.req.bits.uop
  val rs1_data = io.req.bits.rs1_data
  val rs2_data = io.req.bits.rs2_data
  val rs3_data = io.req.bits.rs3_data
  val rvm_data = io.req.bits.rvm_data
  val body, prestart, tail, mask, inactive = Wire(UInt(vLenb.W))
  val vl = uop.vconfig.vl
  prestart := Cat((0 until vLen/16).map(b => uop.v_eidx + b.U < uop.vstart).reverse)
  body     := Cat((0 until vLen/16).map(b => uop.v_eidx + b.U >= uop.vstart && uop.v_eidx + b.U < vl).reverse)
  mask     := Mux(uop.v_unmasked, ~(0.U(vLenb.W)), Cat((0 until vLen/16).map(b => rvm_data(b)).reverse))
  tail     := Cat((0 until vLen/16).map(b => uop.v_eidx + b.U >= vl).reverse)
  inactive := prestart | body & ~mask | tail
  val outInactive = Pipe(io.req.valid, inactive, latency).bits
  val outRs3Data  = Pipe(io.req.valid, rs3_data, latency).bits
  val isUnsigned  = uop.rt(RS2, isUnsignedV)

  val fp_decoder = Module(new UOPCodeFPUDecoder(vector = true)) // TODO use a simpler decoder
  fp_decoder.io.uopc := uop.uopc
  val fp_ctrl = WireInit(fp_decoder.io.sigs)
  val vsew    = uop.vconfig.vtype.vsew
  val vd_fmt  = Mux(uop.vd_eew  === 3.U, D, Mux(uop.vd_eew  === 2.U, S, H))
  val vs2_fmt = Mux(uop.vs2_eew === 3.U, D, Mux(uop.vs2_eew === 2.U, S, H))
  fp_ctrl.typeTagIn  := vs2_fmt
  fp_ctrl.typeTagOut := vd_fmt
  when (io.req.valid) {
    assert(uop.fp_val, "unexpected fp_val")
    assert(vsew <= 3.U, "unsupported vsew")
    assert(uop.vd_eew >= 1.U && uop.vd_eew <= 3.U, "unsupported vd_eew")
  }

  val tag  = fp_ctrl.typeTagIn
  val typ1 = Mux(tag === D, 1.U(1.W), 0.U(1.W))
  val req  = Wire(new tile.FPInput)
  req <> fp_ctrl
  req.rm  := io.fcsr_rm
  req.in1 := DontCare
  req.in2 := DontCare
  req.in3 := DontCare
  when(fp_ctrl.wflags) {
    req.typeTagIn := fp_ctrl.typeTagOut      // IntToFP typeTagIn, based on float width, not integer
  }
  req.typ    := ImmGenTypRVV(typ1, uop.imm_packed)
  req.fmt    := Mux(tag === H, 2.U, Mux(tag === S, 0.U, 1.U)) // TODO support Zfh and avoid special-case below
  req.fmaCmd := DontCare

  val e64DataOut = Wire(Vec(numELENinVLEN,   UInt(64.W)))
  val e32DataOut = Wire(Vec(numELENinVLEN*2, UInt(32.W)))
  val e16DataOut = Wire(Vec(numELENinVLEN*4, UInt(16.W)))
  val e64FlagOut = Wire(Vec(numELENinVLEN,   Bits(tile.FPConstants.FLAGS_SZ.W)))
  val e32FlagOut = Wire(Vec(numELENinVLEN*2, Bits(tile.FPConstants.FLAGS_SZ.W)))
  val e16FlagOut = Wire(Vec(numELENinVLEN*4, Bits(tile.FPConstants.FLAGS_SZ.W)))

  for (e <- 0 until vLen/16) {
    // instants
    val ifpu = if(e < numELENinVLEN)        Module(new tile.IntToFP(latency, vector = true, supportD = true))
               else if(e < numELENinVLEN*2) Module(new tile.IntToFP(latency, vector = true, supportD = false, supportS = true ))
               else                         Module(new tile.IntToFP(latency, vector = true, supportD = false, supportS = false))
    // inputs
    ifpu.io.in.valid := io.req.valid
    ifpu.io.in.bits  := req
    if(e < numELENinVLEN) {
      ifpu.io.in.bits.in1 := Mux(isUnsigned, Mux1H(UIntToOH(uop.vs2_eew),
                                                   Seq(rs2_data(8*e+7, 8*e), rs2_data(16*e+15, 16*e), rs2_data(32*e+31, 32*e), rs2_data(64*e+63, 64*e))),
                                             Mux1H(UIntToOH(uop.vs2_eew),
                                                   Seq(rs2_data( 8*e+7,   8*e).sextTo(xLen),
                                                       rs2_data(16*e+15, 16*e).sextTo(xLen),
                                                       rs2_data(32*e+31, 32*e).sextTo(xLen),
                                                       rs2_data(64*e+63, 64*e))))
    } else if(e < numELENinVLEN*2) {
      ifpu.io.in.bits.in1 := Mux(isUnsigned, Mux1H(UIntToOH(uop.vs2_eew),
                                                   Seq(rs2_data(8*e+7, 8*e), rs2_data(16*e+15, 16*e), rs2_data(32*e+31, 32*e), 0.U)),
                                             Mux1H(UIntToOH(uop.vs2_eew),
                                                   Seq(rs2_data( 8*e+7,   8*e).sextTo(xLen),
                                                       rs2_data(16*e+15, 16*e).sextTo(xLen),
                                                       rs2_data(32*e+31, 32*e).sextTo(xLen),
                                                       0.U)))
    } else {
      ifpu.io.in.bits.in1 := Mux(isUnsigned, Mux(uop.vs2_eew(0), rs2_data(16*e+15, 16*e), rs2_data(8*e+7, 8*e)),
                                             Mux(uop.vs2_eew(0), rs2_data(16*e+15, 16*e).sextTo(xLen), rs2_data(8*e+7, 8*e).sextTo(xLen)))
    }
    // outputs
    if(e < numELENinVLEN) {
      e64DataOut(e) := Mux(outInactive(e), outRs3Data(64*e+63, 64*e), ieee(box(ifpu.io.out.bits.data, D)))
      e64FlagOut(e) := Mux(outInactive(e), 0.U, ifpu.io.out.bits.exc)
    }
    if(e < numELENinVLEN*2) {
      e32DataOut(e) := Mux(outInactive(e), outRs3Data(32*e+31, 32*e), ieee(box(ifpu.io.out.bits.data, S)))
      e32FlagOut(e) := Mux(outInactive(e), 0.U, ifpu.io.out.bits.exc)
    }
    e16DataOut(e) := Mux(outInactive(e), outRs3Data(16*e+15, 16*e), ieee(box(ifpu.io.out.bits.data, H)))
    e16FlagOut(e) := Mux(outInactive(e), 0.U, ifpu.io.out.bits.exc)
  }
  val isNarrowOdd = io.resp.bits.uop.rt(RS2, isWidenV) && (io.resp.bits.uop.v_eidx((vLenbSz - 1).U - io.resp.bits.uop.vd_eew) === 1.U)
  val resp_data = Mux1H(UIntToOH(io.resp.bits.uop.vd_eew), Seq(0.U, e16DataOut.asUInt, e32DataOut.asUInt, e64DataOut.asUInt))
  io.resp.bits.data :=  Mux(isNarrowOdd, resp_data << (vLen / 2), resp_data)
  io.resp.bits.vmask :=  Mux(isNarrowOdd, Fill(vLenb / 2, 1.U(1.W)) ## Fill(vLenb / 2, 0.U(1.W)), Fill(vLenb, 1.U(1.W)))
  io.resp.bits.fflags.valid      := io.resp.valid
  io.resp.bits.fflags.bits.uop   := io.resp.bits.uop
  val resp_flag = Mux1H(UIntToOH(io.resp.bits.uop.vd_eew),
                            Seq(0.U,
                                e16FlagOut.reduce(_ | _),
                                e32FlagOut.reduce(_ | _),
                                e64FlagOut.reduce(_ | _)))
  io.resp.bits.fflags.bits.flags := resp_flag
}
