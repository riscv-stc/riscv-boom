//******************************************************************************
// Copyright (c) 2011 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// RISCV Processor Constants
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.common.constants

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.Parameters
import freechips.rocketchip.util.Str
import freechips.rocketchip.rocket.RVCExpander


/**
 * Mixin indicating the debug flags that can be set for viewing different
 * debug printf's
 */
trait BOOMDebugConstants
{
  val DEBUG_PRINTF        = false // use the Chisel printf functionality
  val COMMIT_LOG_PRINTF   = false // dump commit state, for comparision against ISA sim
  val MEMTRACE_PRINTF     = false // dump trace of memory accesses to L1D for debugging
  val O3PIPEVIEW_PRINTF   = true // dump trace for O3PipeView from gem5
  val O3_CYCLE_TIME       = (1000)// "cycle" time expected by o3pipeview.py

  val DROMAJO_COSIM_ENABLE = false // enable dromajo cosim

  // When enabling DEBUG_PRINTF, the vertical whitespace can be padded out
  // such that viewing the *.out file in vim can line up veritically to
  // enable ctrl+f/ctrl+b to advance the *.out file one cycle without
  // moving the structures.
  val debugScreenheight  = 79

  // turn off stuff to dramatically reduce Chisel node count
  val DEBUG_PRINTF_LSU    = true && DEBUG_PRINTF
  val DEBUG_PRINTF_ROB    = true && DEBUG_PRINTF
  val DEBUG_PRINTF_TAGE   = true && DEBUG_PRINTF
  val DEBUG_PRINTF_FTQ    = true && DEBUG_PRINTF
  val DEBUG_PRINTF_IQ     = true && DEBUG_PRINTF

  if (O3PIPEVIEW_PRINTF) require (!DEBUG_PRINTF && !COMMIT_LOG_PRINTF)
}

/**
 * Mixin for issue queue types
 */
trait IQType
{
  val IQT_SZ  = 4
  val IQT_INT = 1.U(IQT_SZ.W)
  val IQT_MEM = 2.U(IQT_SZ.W)
  val IQT_FP  = 4.U(IQT_SZ.W)
  val IQT_VEC = 8.U(IQT_SZ.W)

  val IQT_MFP = 6.U(IQT_SZ.W)
  val IQT_MVEC= 0xA.U(IQT_SZ.W)
  val IQT_IVEC= 0x9.U(IQT_SZ.W)
  val IQT_FVEC= 0xC.U(IQT_SZ.W)
}


/**
 * Mixin for scalar operation constants
 */
trait ScalarOpConstants
{
  val X = BitPat("b?")
  val Y = BitPat("b1")
  val N = BitPat("b0")
  val U_0 = 0.U(2.W)
  val U_1 = 1.U(2.W)
  val U_2 = 2.U(2.W)
  val U_3 = 3.U(2.W)

  //************************************
  // Extra Constants

  // Which branch predictor predicted us
  val BSRC_SZ = 2
  val BSRC_1 = 0.U(BSRC_SZ.W) // 1-cycle branch pred
  val BSRC_2 = 1.U(BSRC_SZ.W) // 2-cycle branch pred
  val BSRC_3 = 2.U(BSRC_SZ.W) // 3-cycle branch pred
  val BSRC_C = 3.U(BSRC_SZ.W) // core branch resolution

  //************************************
  // Control Signals

  // CFI types
  val CFI_SZ   = 3
  val CFI_X    = 0.U(CFI_SZ.W) // Not a CFI instruction
  val CFI_BR   = 1.U(CFI_SZ.W) // Branch
  val CFI_JAL  = 2.U(CFI_SZ.W) // JAL
  val CFI_JALR = 3.U(CFI_SZ.W) // JALR

  // PC Select Signal
  val PC_PLUS4 = 0.U(2.W)  // PC + 4
  val PC_BRJMP = 1.U(2.W)  // brjmp_target
  val PC_JALR  = 2.U(2.W)  // jump_reg_target

  // Branch Type
  val BR_N   = 0.U(4.W)  // Next
  val BR_NE  = 1.U(4.W)  // Branch on NotEqual
  val BR_EQ  = 2.U(4.W)  // Branch on Equal
  val BR_GE  = 3.U(4.W)  // Branch on Greater/Equal
  val BR_GEU = 4.U(4.W)  // Branch on Greater/Equal Unsigned
  val BR_LT  = 5.U(4.W)  // Branch on Less Than
  val BR_LTU = 6.U(4.W)  // Branch on Less Than Unsigned
  val BR_J   = 7.U(4.W)  // Jump
  val BR_JR  = 8.U(4.W)  // Jump Register

  // RS1 Operand Select Signal
  val OP1_RS1 = 0.U(3.W) // Register Source #1
  val OP1_ZERO= 1.U(3.W)
  val OP1_PC  = 2.U(3.W)
  val OP1_VS2 = 3.U(3.W) // for vector
  val OP1_INV_VS2 = 4.U(3.W) // for vector mask insn
  val OP1_X   = BitPat("b???")

  // RS2 Operand Select Signal
  val OP2_RS2 = 0.U(3.W) // Register Source #2
  val OP2_IMM = 1.U(3.W) // immediate
  val OP2_ZERO= 2.U(3.W) // constant 0
  val OP2_NEXT= 3.U(3.W) // constant 2/4 (for PC+2/4)
  val OP2_IMMC= 4.U(3.W) // for CSR imm found in RS1
  val OP2_VL  = 5.U(3.W) // for vset{i}vl{i}
  val OP2_VS1 = 6.U(3.W) // for vector
  val OP2_INV_VS1 = 7.U(3.W) // for vector mask insn
  val OP2_X   = BitPat("b???")

  // Register File Write Enable Signal
  val REN_0   = false.B
  val REN_1   = true.B

  // Is 32b Word or 64b Doubldword?
  val SZ_DW = 1
  val DW_X   = true.B // Bool(xLen==64)
  val DW_32  = false.B
  val DW_64  = true.B
  val DW_XPR = true.B // Bool(xLen==64)

  // Memory Enable Signal
  val MEN_0   = false.B
  val MEN_1   = true.B
  val MEN_X   = false.B

  // Immediate Extend Select
  val IS_I   = 0.U(3.W)  // I-Type  (LD,ALU)
  val IS_S   = 1.U(3.W)  // S-Type  (ST)
  val IS_B   = 2.U(3.W)  // SB-Type (BR)
  val IS_U   = 3.U(3.W)  // U-Type  (LUI/AUIPC)
  val IS_J   = 4.U(3.W)  // UJ-Type (J/JAL)
  val IS_X   = BitPat("b???")

  val RD  = 0.U(2.W)
  val RS1 = 1.U(2.W)
  val RS2 = 2.U(2.W)
  val RS3 = 3.U(2.W)
  // Decode Stage Control Signals
  val RT_FIX   = 0x00.U(5.W)
  val RT_FLT   = 0x01.U(5.W)
  val RT_VI    = 0x02.U(5.W) // RS1 as simm5 (vop.vi), TODO: move to OP selection
  val RT_FIXU  = 0x04.U(5.W) // RS1 used as unsigned for vector
  val RT_VIU   = 0x06.U(5.W) // RS1 as uimm5 (vop.vi), TODO: move to OP selection
  val RT_X     = 0x07.U(5.W) // not-a-register (but shouldn't get a busy-bit, etc.)
  val RT_PERM  = 0x0F.U(5.W) // special mark for permutation VADD
  val RT_VEC   = 0x10.U(5.W) // vector, signed width vsew
  val RT_VN    = 0x11.U(5.W) // vector, signed width vsew/2
  val RT_VW    = 0x12.U(5.W) // vector, signed width vsew*2 (for v*ext, maybe *4 *8)
  val RT_VF    = 0x13.U(5.W) // vector, ieee FP format, currently not used
  val RT_VU    = 0x14.U(5.W) // vector, unsigned width vsew
  val RT_VNU   = 0x15.U(5.W) // vector, unsigned width vsew/2
  val RT_VWU   = 0x16.U(5.W) // vector, unsigned width vsew*2 (for v*ext, maybe *4 *8)
  val RT_VM    = 0x17.U(5.W) // VD is mask bit
  val RT_VRED  = 0x18.U(5.W) // vector, signed reduction VD/VS1 with vsew
  val RT_VRW   = 0x1A.U(5.W) // vector, signed reduction VD/VS1 with vsew*2
  val RT_VRU   = 0x1C.U(5.W) // vector, unsigned reduction VD/VS1 with vsew
  val RT_VRWU  = 0x1E.U(5.W) // vector, unsigned reduction VD/VS1 with vsew*2
                             // TODO rename RT_NAR

  def isInt      (rt: UInt): Bool = (rt === RT_FIX || rt === RT_FIXU)
  def isNotInt   (rt: UInt): Bool = !isInt(rt)
  def isFloat    (rt: UInt): Bool = (rt === RT_FLT)
  def isVector   (rt: UInt): Bool = rt(4)
  def isWidenV   (rt: UInt): Bool = (rt === RT_VW || rt === RT_VWU || rt === RT_VRW || rt === RT_VRWU)
  def isNarrowV  (rt: UInt): Bool = (rt === RT_VN || rt === RT_VNU)
  def isUnsignedV(rt: UInt): Bool = (rt === RT_VU || rt === RT_VWU || rt === RT_VNU || rt === RT_VIU || rt === RT_VRU || rt === RT_VRWU)
  def isReduceV  (rt: UInt): Bool = (rt(4,3) === 3.U)
  def isMaskVD   (rt: UInt): Bool = (rt === RT_VM)
  def isNotReg   (rt: UInt): Bool = (rt === RT_X  || rt === RT_VI  || rt === RT_VIU || rt === RT_PERM)
  def isSomeReg  (rt: UInt): Bool = !isNotReg(rt)
  def isRvvSImm5 (rt: UInt): Bool = (rt === RT_VI)
  def isRvvUImm5 (rt: UInt): Bool = (rt === RT_VIU)
  def isRvvImm5  (rt: UInt): Bool = (rt === RT_VI || rt === RT_VIU)
  //def isMaskV    (rt: UInt): Bool = (rt === RT_VM)

  val uopX    = BitPat.dontCare(boom.common.MicroOpcodes.UOPC_SZ)
  // The Bubble Instruction (Machine generated NOP)
  // Insert (XOR x0,x0,x0) which is different from software compiler
  // generated NOPs which are (ADDI x0, x0, 0).
  // Reasoning for this is to let visualizers and stat-trackers differentiate
  // between software NOPs and machine-generated Bubbles in the pipeline.
  val BUBBLE  = (0x4033).U(32.W)

  def NullMicroOp(usingVector: Boolean = false)(implicit p: Parameters): boom.common.MicroOp = {
    val uop = Wire(new boom.common.MicroOp)
    uop            := DontCare // Overridden in the following lines
    // maybe not required, but helps on asserts that try to catch spurious behavior
    uop.uopc       := boom.common.MicroOpcodes.uopNOP
    uop.bypassable := false.B
    uop.fp_val     := false.B
    uop.uses_stq   := false.B
    uop.uses_ldq   := false.B
    uop.pdst       := 0.U
    uop.prs1       := 0.U
    uop.prs2       := 0.U
    uop.prs3       := 0.U
    if (usingVector) {
      uop.prvm        := 0.U
    }
    uop.dst_rtype  := RT_X

    val cs = Wire(new boom.common.CtrlSignals())
    cs             := DontCare // Overridden in the following lines
    cs.br_type     := BR_N
    cs.csr_cmd     := freechips.rocketchip.rocket.CSR.N
    cs.is_load     := false.B
    cs.is_sta      := false.B
    cs.is_std      := false.B

    uop.ctrl := cs
    uop
  }
}

/**
 * Mixin for RISCV constants
 */
trait RISCVConstants
{
  // abstract out instruction decode magic numbers
  val RD_MSB  = 11
  val RD_LSB  = 7
  val RS1_MSB = 19
  val RS1_LSB = 15
  val RS2_MSB = 24
  val RS2_LSB = 20
  val RS3_MSB = 31
  val RS3_LSB = 27
  val VM_BIT  = 25
  val NF_MSB  = 31
  val NF_LSB  = 29

  val CSR_ADDR_MSB = 31
  val CSR_ADDR_LSB = 20
  val CSR_ADDR_SZ = 12

  // location of the fifth bit in the shamt (for checking for illegal ops for SRAIW,etc.)
  val SHAMT_5_BIT = 25
  val LONGEST_IMM_SZ = 20
  val X0 = 0.U
  val RA = 1.U // return address register

  // memory consistency model
  // The C/C++ atomics MCM requires that two loads to the same address maintain program order.
  // The Cortex A9 does NOT enforce load/load ordering (which leads to buggy behavior).
  val MCM_ORDER_DEPENDENT_LOADS = true

  val jal_opc = (0x6f).U
  val jalr_opc = (0x67).U

  def GetUop(inst: UInt): UInt = inst(6,0)
  def GetRd (inst: UInt): UInt = inst(RD_MSB,RD_LSB)
  def GetRs1(inst: UInt): UInt = inst(RS1_MSB,RS1_LSB)

  def ExpandRVC(inst: UInt)(implicit p: Parameters): UInt = {
    val rvc_exp = Module(new RVCExpander)
    rvc_exp.io.in := inst
    Mux(rvc_exp.io.rvc, rvc_exp.io.out.bits, inst)
  }

  // Note: Accepts only EXPANDED rvc instructions
  def ComputeBranchTarget(pc: UInt, inst: UInt, xlen: Int)(implicit p: Parameters): UInt = {
    val b_imm32 = Cat(Fill(20,inst(31)), inst(7), inst(30,25), inst(11,8), 0.U(1.W))
    ((pc.asSInt + b_imm32.asSInt).asSInt & (-2).S).asUInt
  }

  // Note: Accepts only EXPANDED rvc instructions
  def ComputeJALTarget(pc: UInt, inst: UInt, xlen: Int)(implicit p: Parameters): UInt = {
    val j_imm32 = Cat(Fill(12,inst(31)), inst(19,12), inst(20), inst(30,25), inst(24,21), 0.U(1.W))
    ((pc.asSInt + j_imm32.asSInt).asSInt & (-2).S).asUInt
  }

  // Note: Accepts only EXPANDED rvc instructions
  def GetCfiType(inst: UInt)(implicit p: Parameters): UInt = {
    val bdecode = Module(new boom.exu.BranchDecode)
    bdecode.io.inst := inst
    bdecode.io.pc := 0.U
    bdecode.io.out.cfi_type
  }
}

/**
 * Mixin for exception cause constants
 */
trait ExcCauseConstants
{
  // a memory disambigious misspeculation occurred
  val MINI_EXCEPTION_MEM_ORDERING = 16.U

  require (!freechips.rocketchip.rocket.Causes.all.contains(16))
}
