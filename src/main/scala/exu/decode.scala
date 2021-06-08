//******************************************************************************
// Copyright (c) 2015 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

package boom.exu

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.Parameters
import freechips.rocketchip.rocket.Instructions._
import freechips.rocketchip.rocket.RVCExpander
import freechips.rocketchip.rocket.{CSR,Causes,VConfig,VType}
import freechips.rocketchip.util.{uintToBitPat,UIntIsOneOf}

import FUConstants._
import boom.common._
import boom.common.MicroOpcodes._
import boom.util._

// scalastyle:off
/**
 * Abstract trait giving defaults and other relevant values to different Decode constants/
 */
abstract trait DecodeConstants
  extends freechips.rocketchip.rocket.constants.ScalarOpConstants
  with freechips.rocketchip.rocket.constants.MemoryOpConstants
{
  val xpr64 = Y // TODO inform this from xLen
  val DC2 = BitPat.dontCare(2) // Makes the listing below more readable
  def decode_default: List[BitPat] =
            //                                                                  frs3_en                        wakeup_delay
            //     is val inst?                                                 |  imm sel                     |    bypassable (aka, known/fixed latency)
            //     |  is fp inst?                                               |  |     uses_ldq              |    |  is_br              is vector instruction
            //     |  |  is single-prec?                        rs1 regtype     |  |     |  uses_stq           |    |  |                  |  can be split
            //     |  |  |  micro-code                          |       rs2 type|  |     |  |  is_amo          |    |  |                  |  |  use vm?
            //     |  |  |  |         iq-type  func unit        |       |       |  |     |  |  |  is_fence     |    |  |                  |  |  |  ew of ls vector
            //     |  |  |  |         |        |                |       |       |  |     |  |  |  |  is_fencei |    |  |  is breakpoint or ecall?  |
            //     |  |  |  |         |        |        dst     |       |       |  |     |  |  |  |  |  mem    |    |  |  |  is unique? (clear pipeline for it)
            //     |  |  |  |         |        |        regtype |       |       |  |     |  |  |  |  |  cmd    |    |  |  |  |  flush on commit |  |
            //     |  |  |  |         |        |        |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  csr cmd   |  |  |
            //     |  |  |  |         |        |        |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  |      |  |  |  |
              List(N, N, X, uopX    , IQT_INT, FU_X   , RT_X  , DC2    ,DC2    ,X, IS_X, X, X, X, X, N, M_X,   DC2, X, X, N, N, X, CSR.X, N, N, N, 0.U)

  val table: Array[(BitPat, List[BitPat])]
}
// scalastyle:on

/**
 * Decoded control signals
 */
class CtrlSigs extends Bundle
{
  val legal           = Bool()
  val fp_val          = Bool()
  val fp_single       = Bool()
  val uopc            = UInt(UOPC_SZ.W)
  val iq_type         = UInt(IQT_SZ.W)
  val fu_code         = UInt(FUC_SZ.W)
  val dst_type        = UInt(RT_X.getWidth.W)
  val rs1_type        = UInt(RT_X.getWidth.W)
  val rs2_type        = UInt(RT_X.getWidth.W)
  val frs3_en         = Bool()
  val imm_sel         = UInt(IS_X.getWidth.W)
  val uses_ldq        = Bool()
  val uses_stq        = Bool()
  val is_amo          = Bool()
  val is_fence        = Bool()
  val is_fencei       = Bool()
  val mem_cmd         = UInt(freechips.rocketchip.rocket.M_SZ.W)
  val wakeup_delay    = UInt(2.W)
  val bypassable      = Bool()
  val is_br           = Bool()
  val is_sys_pc2epc   = Bool()
  val inst_unique     = Bool()
  val flush_on_commit = Bool()
  val csr_cmd         = UInt(freechips.rocketchip.rocket.CSR.SZ.W)
  val is_rvv          = Bool()
  val can_be_split    = Bool()
  val uses_vm         = Bool()
  val v_ls_ew         = UInt(2.W)
  val rocc            = Bool()

  def decode(inst: UInt, table: Iterable[(BitPat, List[BitPat])]) = {
    val decoder = freechips.rocketchip.rocket.DecodeLogic(inst, XDecode.decode_default, table)
    val sigs =
      Seq(legal, fp_val, fp_single, uopc, iq_type, fu_code, dst_type, rs1_type,
          rs2_type, frs3_en, imm_sel, uses_ldq, uses_stq, is_amo,
          is_fence, is_fencei, mem_cmd, wakeup_delay, bypassable,
          is_br, is_sys_pc2epc, inst_unique, flush_on_commit, csr_cmd,
          is_rvv, can_be_split, uses_vm, v_ls_ew)
      sigs zip decoder map {case(s,d) => s := d}
      rocc := false.B
      this
  }
}

// scalastyle:off
/**
 * Decode constants for RV32
 */
object X32Decode extends DecodeConstants
{
            //                                                                  frs3_en                        wakeup_delay
            //     is val inst?                                                 |  imm sel                     |    bypassable (aka, known/fixed latency)
            //     |  is fp inst?                                               |  |     uses_ldq              |    |  is_br
            //     |  |  is single-prec?                        rs1 regtype     |  |     |  uses_stq           |    |  |
            //     |  |  |  micro-code                          |       rs2 type|  |     |  |  is_amo          |    |  |
            //     |  |  |  |         iq-type  func unit        |       |       |  |     |  |  |  is_fence     |    |  |
            //     |  |  |  |         |        |                |       |       |  |     |  |  |  |  is_fencei |    |  |  is breakpoint or ecall?
            //     |  |  |  |         |        |        dst     |       |       |  |     |  |  |  |  |  mem    |    |  |  |  is unique? (clear pipeline for it)
            //     |  |  |  |         |        |        regtype |       |       |  |     |  |  |  |  |  cmd    |    |  |  |  |  flush on commit
            //     |  |  |  |         |        |        |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  csr cmd
  val table: Array[(BitPat, List[BitPat])] = Array(//   |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  |
  SLLI_RV32-> List(Y, N, X, uopSLLI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N, N, N, N, 0.U),
  SRLI_RV32-> List(Y, N, X, uopSRLI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N, N, N, N, 0.U),
  SRAI_RV32-> List(Y, N, X, uopSRAI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N, N, N, N, 0.U)
  )
}

/**
 * Decode constants for RV64
 */
object X64Decode extends DecodeConstants
{
           //                                                                  frs3_en                        wakeup_delay
           //     is val inst?                                                 |  imm sel                     |    bypassable (aka, known/fixed latency)
           //     |  is fp inst?                                               |  |     uses_ldq              |    |  is_br
           //     |  |  is single-prec?                        rs1 regtype     |  |     |  uses_stq           |    |  |
           //     |  |  |  micro-code                          |       rs2 type|  |     |  |  is_amo          |    |  |
           //     |  |  |  |         iq-type  func unit        |       |       |  |     |  |  |  is_fence     |    |  |
           //     |  |  |  |         |        |                |       |       |  |     |  |  |  |  is_fencei |    |  |  is breakpoint or ecall?
           //     |  |  |  |         |        |        dst     |       |       |  |     |  |  |  |  |  mem    |    |  |  |  is unique? (clear pipeline for it)
           //     |  |  |  |         |        |        regtype |       |       |  |     |  |  |  |  |  cmd    |    |  |  |  |  flush on commit
           //     |  |  |  |         |        |        |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  csr cmd
  val table: Array[(BitPat, List[BitPat])] = Array(//  |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  |
  LD      -> List(Y, N, X, uopLD   , IQT_MEM, FU_MEM , RT_FIX, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, 3.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  LWU     -> List(Y, N, X, uopLD   , IQT_MEM, FU_MEM , RT_FIX, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, 3.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  SD      -> List(Y, N, X, uopSTA  , IQT_MEM, FU_MEM , RT_X  , RT_FIX, RT_FIX, N, IS_S, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),

  SLLI    -> List(Y, N, X, uopSLLI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N, N, N, N, 0.U),
  SRLI    -> List(Y, N, X, uopSRLI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N, N, N, N, 0.U),
  SRAI    -> List(Y, N, X, uopSRAI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N, N, N, N, 0.U),

  ADDIW   -> List(Y, N, X, uopADDIW, IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N, N, N, N, 0.U),
  SLLIW   -> List(Y, N, X, uopSLLIW, IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N, N, N, N, 0.U),
  SRAIW   -> List(Y, N, X, uopSRAIW, IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N, N, N, N, 0.U),
  SRLIW   -> List(Y, N, X, uopSRLIW, IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N, N, N, N, 0.U),

  ADDW    -> List(Y, N, X, uopADDW , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N, N, N, N, 0.U),
  SUBW    -> List(Y, N, X, uopSUBW , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N, N, N, N, 0.U),
  SLLW    -> List(Y, N, X, uopSLLW , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N, N, N, N, 0.U),
  SRAW    -> List(Y, N, X, uopSRAW , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N, N, N, N, 0.U),
  SRLW    -> List(Y, N, X, uopSRLW , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N, N, N, N, 0.U)
  )
}

/**
 * Overall Decode constants
 */
object XDecode extends DecodeConstants
{
           //                                                                  frs3_en                        wakeup_delay
           //     is val inst?                                                 |  imm sel                     |    bypassable (aka, known/fixed latency)
           //     |  is fp inst?                                               |  |     uses_ldq              |    |  is_br
           //     |  |  is single-prec?                        rs1 regtype     |  |     |  uses_stq           |    |  |
           //     |  |  |  micro-code                          |       rs2 type|  |     |  |  is_amo          |    |  |
           //     |  |  |  |         iq-type  func unit        |       |       |  |     |  |  |  is_fence     |    |  |
           //     |  |  |  |         |        |                |       |       |  |     |  |  |  |  is_fencei |    |  |  is breakpoint or ecall?
           //     |  |  |  |         |        |        dst     |       |       |  |     |  |  |  |  |  mem    |    |  |  |  is unique? (clear pipeline for it)
           //     |  |  |  |         |        |        regtype |       |       |  |     |  |  |  |  |  cmd    |    |  |  |  |  flush on commit
           //     |  |  |  |         |        |        |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  csr cmd
  val table: Array[(BitPat, List[BitPat])] = Array(//  |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  |
  LW      -> List(Y, N, X, uopLD   , IQT_MEM, FU_MEM , RT_FIX, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, 3.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  LH      -> List(Y, N, X, uopLD   , IQT_MEM, FU_MEM , RT_FIX, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, 3.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  LHU     -> List(Y, N, X, uopLD   , IQT_MEM, FU_MEM , RT_FIX, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, 3.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  LB      -> List(Y, N, X, uopLD   , IQT_MEM, FU_MEM , RT_FIX, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, 3.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  LBU     -> List(Y, N, X, uopLD   , IQT_MEM, FU_MEM , RT_FIX, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, 3.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),

  SW      -> List(Y, N, X, uopSTA  , IQT_MEM, FU_MEM , RT_X  , RT_FIX, RT_FIX, N, IS_S, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  SH      -> List(Y, N, X, uopSTA  , IQT_MEM, FU_MEM , RT_X  , RT_FIX, RT_FIX, N, IS_S, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  SB      -> List(Y, N, X, uopSTA  , IQT_MEM, FU_MEM , RT_X  , RT_FIX, RT_FIX, N, IS_S, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),

  LUI     -> List(Y, N, X, uopLUI  , IQT_INT, FU_ALU , RT_FIX, RT_X  , RT_X  , N, IS_U, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N, N, N, N, 0.U),

  ADDI    -> List(Y, N, X, uopADDI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N, N, N, N, 0.U),
  ANDI    -> List(Y, N, X, uopANDI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N, N, N, N, 0.U),
  ORI     -> List(Y, N, X, uopORI  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N, N, N, N, 0.U),
  XORI    -> List(Y, N, X, uopXORI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N, N, N, N, 0.U),
  SLTI    -> List(Y, N, X, uopSLTI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N, N, N, N, 0.U),
  SLTIU   -> List(Y, N, X, uopSLTIU, IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N, N, N, N, 0.U),

  SLL     -> List(Y, N, X, uopSLL  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N, N, N, N, 0.U),
  ADD     -> List(Y, N, X, uopADD  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N, N, N, N, 0.U),
  SUB     -> List(Y, N, X, uopSUB  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N, N, N, N, 0.U),
  SLT     -> List(Y, N, X, uopSLT  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N, N, N, N, 0.U),
  SLTU    -> List(Y, N, X, uopSLTU , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N, N, N, N, 0.U),
  AND     -> List(Y, N, X, uopAND  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N, N, N, N, 0.U),
  OR      -> List(Y, N, X, uopOR   , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N, N, N, N, 0.U),
  XOR     -> List(Y, N, X, uopXOR  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N, N, N, N, 0.U),
  SRA     -> List(Y, N, X, uopSRA  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N, N, N, N, 0.U),
  SRL     -> List(Y, N, X, uopSRL  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N, N, N, N, 0.U),

  MUL     -> List(Y, N, X, uopMUL  , IQT_INT, FU_MUL , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  MULH    -> List(Y, N, X, uopMULH , IQT_INT, FU_MUL , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  MULHU   -> List(Y, N, X, uopMULHU, IQT_INT, FU_MUL , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  MULHSU  -> List(Y, N, X, uopMULHSU,IQT_INT, FU_MUL , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  MULW    -> List(Y, N, X, uopMULW , IQT_INT, FU_MUL , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),

  DIV     -> List(Y, N, X, uopDIV  , IQT_INT, FU_DIV , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  DIVU    -> List(Y, N, X, uopDIVU , IQT_INT, FU_DIV , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  REM     -> List(Y, N, X, uopREM  , IQT_INT, FU_DIV , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  REMU    -> List(Y, N, X, uopREMU , IQT_INT, FU_DIV , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  DIVW    -> List(Y, N, X, uopDIVW , IQT_INT, FU_DIV , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  DIVUW   -> List(Y, N, X, uopDIVUW, IQT_INT, FU_DIV , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  REMW    -> List(Y, N, X, uopREMW , IQT_INT, FU_DIV , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  REMUW   -> List(Y, N, X, uopREMUW, IQT_INT, FU_DIV , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),

  AUIPC   -> List(Y, N, X, uopAUIPC, IQT_INT, FU_JMP , RT_FIX, RT_X  , RT_X  , N, IS_U, N, N, N, N, N, M_X  , 1.U, N, N, N, N, N, CSR.N, N, N, N, 0.U), // use BRU for the PC read
  JAL     -> List(Y, N, X, uopJAL  , IQT_INT, FU_JMP , RT_FIX, RT_X  , RT_X  , N, IS_J, N, N, N, N, N, M_X  , 1.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  JALR    -> List(Y, N, X, uopJALR , IQT_INT, FU_JMP , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  BEQ     -> List(Y, N, X, uopBEQ  , IQT_INT, FU_ALU , RT_X  , RT_FIX, RT_FIX, N, IS_B, N, N, N, N, N, M_X  , 0.U, N, Y, N, N, N, CSR.N, N, N, N, 0.U),
  BNE     -> List(Y, N, X, uopBNE  , IQT_INT, FU_ALU , RT_X  , RT_FIX, RT_FIX, N, IS_B, N, N, N, N, N, M_X  , 0.U, N, Y, N, N, N, CSR.N, N, N, N, 0.U),
  BGE     -> List(Y, N, X, uopBGE  , IQT_INT, FU_ALU , RT_X  , RT_FIX, RT_FIX, N, IS_B, N, N, N, N, N, M_X  , 0.U, N, Y, N, N, N, CSR.N, N, N, N, 0.U),
  BGEU    -> List(Y, N, X, uopBGEU , IQT_INT, FU_ALU , RT_X  , RT_FIX, RT_FIX, N, IS_B, N, N, N, N, N, M_X  , 0.U, N, Y, N, N, N, CSR.N, N, N, N, 0.U),
  BLT     -> List(Y, N, X, uopBLT  , IQT_INT, FU_ALU , RT_X  , RT_FIX, RT_FIX, N, IS_B, N, N, N, N, N, M_X  , 0.U, N, Y, N, N, N, CSR.N, N, N, N, 0.U),
  BLTU    -> List(Y, N, X, uopBLTU , IQT_INT, FU_ALU , RT_X  , RT_FIX, RT_FIX, N, IS_B, N, N, N, N, N, M_X  , 0.U, N, Y, N, N, N, CSR.N, N, N, N, 0.U),

  // I-type, the immediate12 holds the CSR register.
  CSRRW   -> List(Y, N, X, uopCSRRW, IQT_INT, FU_CSR , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, Y, Y, CSR.W, N, N, N, 0.U),
  CSRRS   -> List(Y, N, X, uopCSRRS, IQT_INT, FU_CSR , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, Y, Y, CSR.S, N, N, N, 0.U),
  CSRRC   -> List(Y, N, X, uopCSRRC, IQT_INT, FU_CSR , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, Y, Y, CSR.C, N, N, N, 0.U),

  CSRRWI  -> List(Y, N, X, uopCSRRWI,IQT_INT, FU_CSR , RT_FIX, RT_X  , RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, Y, Y, CSR.W, N, N, N, 0.U),
  CSRRSI  -> List(Y, N, X, uopCSRRSI,IQT_INT, FU_CSR , RT_FIX, RT_X  , RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, Y, Y, CSR.S, N, N, N, 0.U),
  CSRRCI  -> List(Y, N, X, uopCSRRCI,IQT_INT, FU_CSR , RT_FIX, RT_X  , RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, Y, Y, CSR.C, N, N, N, 0.U),

  SFENCE_VMA->List(Y,N, X, uopSFENCE,IQT_MEM, FU_MEM , RT_X  , RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N,M_SFENCE,0.U,N, N, N, Y, Y, CSR.N, N, N, N, 0.U),
  SCALL   -> List(Y, N, X, uopERET  ,IQT_INT, FU_CSR , RT_X  , RT_X  , RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, Y, Y, Y, CSR.I, N, N, N, 0.U),
  SBREAK  -> List(Y, N, X, uopERET  ,IQT_INT, FU_CSR , RT_X  , RT_X  , RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, Y, Y, Y, CSR.I, N, N, N, 0.U),
  SRET    -> List(Y, N, X, uopERET  ,IQT_INT, FU_CSR , RT_X  , RT_X  , RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, Y, Y, CSR.I, N, N, N, 0.U),
  MRET    -> List(Y, N, X, uopERET  ,IQT_INT, FU_CSR , RT_X  , RT_X  , RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, Y, Y, CSR.I, N, N, N, 0.U),
  DRET    -> List(Y, N, X, uopERET  ,IQT_INT, FU_CSR , RT_X  , RT_X  , RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, Y, Y, CSR.I, N, N, N, 0.U),

  WFI     -> List(Y, N, X, uopWFI   ,IQT_INT, FU_CSR , RT_X  , RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, Y, Y, CSR.I, N, N, N, 0.U),

  FENCE_I -> List(Y, N, X, uopNOP  , IQT_INT, FU_X   , RT_X  , RT_X  , RT_X  , N, IS_X, N, N, N, N, Y, M_X  , 0.U, N, N, N, Y, Y, CSR.N, N, N, N, 0.U),
  FENCE   -> List(Y, N, X, uopFENCE, IQT_INT, FU_MEM , RT_X  , RT_X  , RT_X  , N, IS_X, N, Y, N, Y, N, M_X  , 0.U, N, N, N, Y, Y, CSR.N, N, N, N, 0.U), // TODO PERF make fence higher performance
                                                                                                                                                       // currently serializes pipeline

           //                                                                  frs3_en                           wakeup_delay
           //     is val inst?                                                 |  imm sel                        |   bypassable (aka, known/fixed latency)
           //     |  is fp inst?                                               |  |     uses_ldq                 |   |  is_br
           //     |  |  is single-prec?                        rs1 regtype     |  |     |  uses_stq              |   |  |
           //     |  |  |  micro-code                          |       rs2 type|  |     |  |  is_amo             |   |  |
           //     |  |  |  |          iq-type  func unit       |       |       |  |     |  |  |  is_fence        |   |  |
           //     |  |  |  |          |        |               |       |       |  |     |  |  |  |  is_fencei    |   |  |  is breakpoint or ecall?
           //     |  |  |  |          |        |       dst     |       |       |  |     |  |  |  |  |  mem       |   |  |  |  is unique? (clear pipeline for it)
           //     |  |  |  |          |        |       regtype |       |       |  |     |  |  |  |  |  cmd       |   |  |  |  |  flush on commit
           //     |  |  |  |          |        |       |       |       |       |  |     |  |  |  |  |  |         |   |  |  |  |  |  csr cmd
  // A-type       |  |  |  |          |        |       |       |       |       |  |     |  |  |  |  |  |         |   |  |  |  |  |  |
  AMOADD_W-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_ADD, 0.U,N, N, N, Y, Y, CSR.N, N, N, N, 0.U), // TODO make AMOs higherperformance
  AMOXOR_W-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_XOR, 0.U,N, N, N, Y, Y, CSR.N, N, N, N, 0.U),
  AMOSWAP_W->List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_SWAP,0.U,N, N, N, Y, Y, CSR.N, N, N, N, 0.U),
  AMOAND_W-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_AND, 0.U,N, N, N, Y, Y, CSR.N, N, N, N, 0.U),
  AMOOR_W -> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_OR,  0.U,N, N, N, Y, Y, CSR.N, N, N, N, 0.U),
  AMOMIN_W-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_MIN, 0.U,N, N, N, Y, Y, CSR.N, N, N, N, 0.U),
  AMOMINU_W->List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_MINU,0.U,N, N, N, Y, Y, CSR.N, N, N, N, 0.U),
  AMOMAX_W-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_MAX, 0.U,N, N, N, Y, Y, CSR.N, N, N, N, 0.U),
  AMOMAXU_W->List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_MAXU,0.U,N, N, N, Y, Y, CSR.N, N, N, N, 0.U),

  AMOADD_D-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_ADD, 0.U,N, N, N, Y, Y, CSR.N, N, N, N, 0.U),
  AMOXOR_D-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_XOR, 0.U,N, N, N, Y, Y, CSR.N, N, N, N, 0.U),
  AMOSWAP_D->List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_SWAP,0.U,N, N, N, Y, Y, CSR.N, N, N, N, 0.U),
  AMOAND_D-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_AND, 0.U,N, N, N, Y, Y, CSR.N, N, N, N, 0.U),
  AMOOR_D -> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_OR,  0.U,N, N, N, Y, Y, CSR.N, N, N, N, 0.U),
  AMOMIN_D-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_MIN, 0.U,N, N, N, Y, Y, CSR.N, N, N, N, 0.U),
  AMOMINU_D->List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_MINU,0.U,N, N, N, Y, Y, CSR.N, N, N, N, 0.U),
  AMOMAX_D-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_MAX, 0.U,N, N, N, Y, Y, CSR.N, N, N, N, 0.U),
  AMOMAXU_D->List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_MAXU,0.U,N, N, N, Y, Y, CSR.N, N, N, N, 0.U),

  LR_W    -> List(Y, N, X, uopLD    , IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_X  , N, IS_X, Y, N, N, N, N, M_XLR   , 0.U,N, N, N, Y, Y, CSR.N, N, N, N, 0.U),
  LR_D    -> List(Y, N, X, uopLD    , IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_X  , N, IS_X, Y, N, N, N, N, M_XLR   , 0.U,N, N, N, Y, Y, CSR.N, N, N, N, 0.U),
  SC_W    -> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XSC   , 0.U,N, N, N, Y, Y, CSR.N, N, N, N, 0.U),
  SC_D    -> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XSC   , 0.U,N, N, N, Y, Y, CSR.N, N, N, N, 0.U)
  )
}

/**
 * FP Decode constants
 */
object FDecode extends DecodeConstants
{
  val table: Array[(BitPat, List[BitPat])] = Array(
            //                                                                  frs3_en                        wakeup_delay
            //                                                                  |  imm sel                     |    bypassable (aka, known/fixed latency)
            //                                                                  |  |     uses_ldq              |    |  is_br
            //    is val inst?                                  rs1 regtype     |  |     |  uses_stq           |    |  |
            //    |  is fp inst?                                |       rs2 type|  |     |  |  is_amo          |    |  |
            //    |  |  is dst single-prec?                     |       |       |  |     |  |  |  is_fence     |    |  |
            //    |  |  |  micro-opcode                         |       |       |  |     |  |  |  |  is_fencei |    |  |  is breakpoint or ecall
            //    |  |  |  |           iq_type  func    dst     |       |       |  |     |  |  |  |  |  mem    |    |  |  |  is unique? (clear pipeline for it)
            //    |  |  |  |           |        unit    regtype |       |       |  |     |  |  |  |  |  cmd    |    |  |  |  |  flush on commit
            //    |  |  |  |           |        |       |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  csr cmd
  FLW     -> List(Y, Y, Y, uopLD     , IQT_MEM, FU_MEM, RT_FLT, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FLD     -> List(Y, Y, N, uopLD     , IQT_MEM, FU_MEM, RT_FLT, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FSW     -> List(Y, Y, Y, uopSTA    , IQT_MFP,FU_F2IMEM,RT_X , RT_FIX, RT_FLT, N, IS_S, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U), // sort of a lie; broken into two micro-ops
  FSD     -> List(Y, Y, N, uopSTA    , IQT_MFP,FU_F2IMEM,RT_X , RT_FIX, RT_FLT, N, IS_S, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),

  FCLASS_S-> List(Y, Y, Y, uopFCLASS_S,IQT_FP , FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FCLASS_D-> List(Y, Y, N, uopFCLASS_D,IQT_FP , FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),

  FMV_S_X -> List(Y, Y, Y, uopFMV_S_X, IQT_INT, FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FMV_D_X -> List(Y, Y, N, uopFMV_D_X, IQT_INT, FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FMV_X_S -> List(Y, Y, Y, uopFMV_X_S, IQT_FP , FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FMV_X_D -> List(Y, Y, N, uopFMV_X_D, IQT_FP , FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),

  FSGNJ_S -> List(Y, Y, Y, uopFSGNJ_S, IQT_FP , FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FSGNJ_D -> List(Y, Y, N, uopFSGNJ_D, IQT_FP , FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FSGNJX_S-> List(Y, Y, Y, uopFSGNJ_S, IQT_FP , FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FSGNJX_D-> List(Y, Y, N, uopFSGNJ_D, IQT_FP , FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FSGNJN_S-> List(Y, Y, Y, uopFSGNJ_S, IQT_FP , FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FSGNJN_D-> List(Y, Y, N, uopFSGNJ_D, IQT_FP , FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),

  // FP to FP
  FCVT_S_D-> List(Y, Y, Y, uopFCVT_S_D,IQT_FP , FU_FPU, RT_FLT, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FCVT_D_S-> List(Y, Y, N, uopFCVT_D_S,IQT_FP , FU_FPU, RT_FLT, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),

  // Int to FP
  FCVT_S_W-> List(Y, Y, Y, uopFCVT_S_X, IQT_INT,FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FCVT_S_WU->List(Y, Y, Y, uopFCVT_S_X, IQT_INT,FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FCVT_S_L-> List(Y, Y, Y, uopFCVT_S_X, IQT_INT,FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FCVT_S_LU->List(Y, Y, Y, uopFCVT_S_X, IQT_INT,FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),

  FCVT_D_W-> List(Y, Y, N, uopFCVT_D_X, IQT_INT,FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FCVT_D_WU->List(Y, Y, N, uopFCVT_D_X, IQT_INT,FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FCVT_D_L-> List(Y, Y, N, uopFCVT_D_X, IQT_INT,FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FCVT_D_LU->List(Y, Y, N, uopFCVT_D_X, IQT_INT,FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),

  // FP to Int
  FCVT_W_S-> List(Y, Y, Y, uopFCVT_X_S, IQT_FP, FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FCVT_WU_S->List(Y, Y, Y, uopFCVT_X_S, IQT_FP, FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FCVT_L_S-> List(Y, Y, Y, uopFCVT_X_S, IQT_FP, FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FCVT_LU_S->List(Y, Y, Y, uopFCVT_X_S, IQT_FP, FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),

  FCVT_W_D-> List(Y, Y, N, uopFCVT_X_D, IQT_FP, FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FCVT_WU_D->List(Y, Y, N, uopFCVT_X_D, IQT_FP, FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FCVT_L_D-> List(Y, Y, N, uopFCVT_X_D, IQT_FP, FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FCVT_LU_D->List(Y, Y, N, uopFCVT_X_D, IQT_FP, FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),

  // "fp_single" is used for wb_data formatting (and debugging)
  FEQ_S    ->List(Y, Y, Y, uopCMPR_S , IQT_FP,  FU_F2I, RT_FIX, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FLT_S    ->List(Y, Y, Y, uopCMPR_S , IQT_FP,  FU_F2I, RT_FIX, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FLE_S    ->List(Y, Y, Y, uopCMPR_S , IQT_FP,  FU_F2I, RT_FIX, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),

  FEQ_D    ->List(Y, Y, N, uopCMPR_D , IQT_FP,  FU_F2I, RT_FIX, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FLT_D    ->List(Y, Y, N, uopCMPR_D , IQT_FP,  FU_F2I, RT_FIX, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FLE_D    ->List(Y, Y, N, uopCMPR_D , IQT_FP,  FU_F2I, RT_FIX, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),

  FMIN_S   ->List(Y, Y, Y,uopFMINMAX_S,IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FMAX_S   ->List(Y, Y, Y,uopFMINMAX_S,IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FMIN_D   ->List(Y, Y, N,uopFMINMAX_D,IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FMAX_D   ->List(Y, Y, N,uopFMINMAX_D,IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),

  FADD_S   ->List(Y, Y, Y, uopFADD_S , IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FSUB_S   ->List(Y, Y, Y, uopFSUB_S , IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FMUL_S   ->List(Y, Y, Y, uopFMUL_S , IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FADD_D   ->List(Y, Y, N, uopFADD_D , IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FSUB_D   ->List(Y, Y, N, uopFSUB_D , IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FMUL_D   ->List(Y, Y, N, uopFMUL_D , IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),

  FMADD_S  ->List(Y, Y, Y, uopFMADD_S, IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FMSUB_S  ->List(Y, Y, Y, uopFMSUB_S, IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FNMADD_S ->List(Y, Y, Y, uopFNMADD_S,IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FNMSUB_S ->List(Y, Y, Y, uopFNMSUB_S,IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FMADD_D  ->List(Y, Y, N, uopFMADD_D, IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FMSUB_D  ->List(Y, Y, N, uopFMSUB_D, IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FNMADD_D ->List(Y, Y, N, uopFNMADD_D,IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FNMSUB_D ->List(Y, Y, N, uopFNMSUB_D,IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U)
  )
}

/**
 * FP Divide SquareRoot Constants
 */
object FDivSqrtDecode extends DecodeConstants
{
  val table: Array[(BitPat, List[BitPat])] = Array(
            //                                                                  frs3_en                        wakeup_delay
            //                                                                  |  imm sel                     |    bypassable (aka, known/fixed latency)
            //                                                                  |  |     uses_ldq              |    |  is_br
            //     is val inst?                                 rs1 regtype     |  |     |  uses_stq           |    |  |
            //     |  is fp inst?                               |       rs2 type|  |     |  |  is_amo          |    |  |
            //     |  |  is dst single-prec?                    |       |       |  |     |  |  |  is_fence     |    |  |
            //     |  |  |  micro-opcode                        |       |       |  |     |  |  |  |  is_fencei |    |  |  is breakpoint or ecall
            //     |  |  |  |           iq-type func    dst     |       |       |  |     |  |  |  |  |  mem    |    |  |  |  is unique? (clear pipeline for it)
            //     |  |  |  |           |       unit    regtype |       |       |  |     |  |  |  |  |  cmd    |    |  |  |  |  flush on commit
            //     |  |  |  |           |       |       |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  csr cmd
  FDIV_S    ->List(Y, Y, Y, uopFDIV_S , IQT_FP, FU_FDV, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FDIV_D    ->List(Y, Y, N, uopFDIV_D , IQT_FP, FU_FDV, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FSQRT_S   ->List(Y, Y, Y, uopFSQRT_S, IQT_FP, FU_FDV, RT_FLT, RT_FLT, RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
  FSQRT_D   ->List(Y, Y, N, uopFSQRT_D, IQT_FP, FU_FDV, RT_FLT, RT_FLT, RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U)
  )
}

/**
 * Vector Extension
 */
// chisel complaints on single giant table, so we use multiple objects
object VectorCfgDecode extends DecodeConstants
{
  val table: Array[(BitPat, List[BitPat])] = Array(
              //                                                                       frs3_en                        wakeup_delay
              //     is val inst?                                                      |  imm sel                     |    bypassable (aka, known/fixed latency)
              //     |  is fp inst?                                                    |  |     uses_ldq              |    |  is_br              is vector instruction
              //     |  |  is single-prec?                             rs1 regtype     |  |     |  uses_stq           |    |  |                  |  can be split
              //     |  |  |  micro-code                               |       rs2 type|  |     |  |  is_amo          |    |  |                  |  |  use vm?
              //     |  |  |  |               iq-type  func unit       |       |       |  |     |  |  |  is_fence     |    |  |                  |  |  |  ew of ls vector
              //     |  |  |  |               |        |               |       |       |  |     |  |  |  |  is_fencei |    |  |  is breakpoint or ecall?  |
              //     |  |  |  |               |        |       dst     |       |       |  |     |  |  |  |  |  mem    |    |  |  |  is unique? (clear pipeline for it)
              //     |  |  |  |               |        |       regtype |       |       |  |     |  |  |  |  |  cmd    |    |  |  |  |  flush on commit |  |
              //     |  |  |  |               |        |       |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  csr cmd   |  |  |
              //     |  |  |  |               |        |       |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  |      |  |  |  |
  VSETVLI     ->List(Y, N, X, uopVSETVLI,     IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, Y, Y, CSR.W, Y, N, N, 0.U)  // TODO optimize us
 ,VSETIVLI    ->List(Y, N, X, uopVSETIVLI,    IQT_INT, FU_CSR, RT_FIX, RT_X  , RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, Y, Y, CSR.W, Y, N, N, 0.U)
 ,VSETVL      ->List(Y, N, X, uopVSETVL,      IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, Y, Y, CSR.W, Y, N, N, 0.U)
 )
}

object VectorLSDecode extends DecodeConstants
{
  val table: Array[(BitPat, List[BitPat])] = Array(
  VLE8_V      ->List(Y, N, X, uopVL,          IQT_MEM ,FU_MEM, RT_VEC, RT_FIX, RT_X  , N, IS_X, Y, N, N, N, N, M_XRD, 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VLE16_V     ->List(Y, N, X, uopVL,          IQT_MEM ,FU_MEM, RT_VEC, RT_FIX, RT_X  , N, IS_X, Y, N, N, N, N, M_XRD, 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 1.U)
 ,VLE32_V     ->List(Y, N, X, uopVL,          IQT_MEM ,FU_MEM, RT_VEC, RT_FIX, RT_X  , N, IS_X, Y, N, N, N, N, M_XRD, 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 2.U)
 ,VLE64_V     ->List(Y, N, X, uopVL,          IQT_MEM ,FU_MEM, RT_VEC, RT_FIX, RT_X  , N, IS_X, Y, N, N, N, N, M_XRD, 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 3.U)
 ,VSE8_V      ->List(Y, N, X, uopVSA,         IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_X  , Y, IS_X, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VSE16_V     ->List(Y, N, X, uopVSA,         IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_X  , Y, IS_X, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 1.U)
 ,VSE32_V     ->List(Y, N, X, uopVSA,         IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_X  , Y, IS_X, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 2.U)
 ,VSE64_V     ->List(Y, N, X, uopVSA,         IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_X  , Y, IS_X, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 3.U)
 ,VLSE8_V     ->List(Y, N, X, uopVLS,         IQT_MEM ,FU_MEM, RT_VEC, RT_FIX, RT_FIX, N, IS_X, Y, N, N, N, N, M_XRD, 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VLSE16_V    ->List(Y, N, X, uopVLS,         IQT_MEM ,FU_MEM, RT_VEC, RT_FIX, RT_FIX, N, IS_X, Y, N, N, N, N, M_XRD, 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 1.U)
 ,VLSE32_V    ->List(Y, N, X, uopVLS,         IQT_MEM ,FU_MEM, RT_VEC, RT_FIX, RT_FIX, N, IS_X, Y, N, N, N, N, M_XRD, 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 2.U)
 ,VLSE64_V    ->List(Y, N, X, uopVLS,         IQT_MEM ,FU_MEM, RT_VEC, RT_FIX, RT_FIX, N, IS_X, Y, N, N, N, N, M_XRD, 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 3.U)
 ,VSSE8_V     ->List(Y, N, X, uopVSSA,        IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_FIX, Y, IS_X, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VSSE16_V    ->List(Y, N, X, uopVSSA,        IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_FIX, Y, IS_X, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 1.U)
 ,VSSE32_V    ->List(Y, N, X, uopVSSA,        IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_FIX, Y, IS_X, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 2.U)
 ,VSSE64_V    ->List(Y, N, X, uopVSSA,        IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_FIX, Y, IS_X, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 3.U)
 ,VLUXEI8_V   ->List(Y, N, X, uopVLUX,        IQT_MVEC,FU_MEMV,RT_VEC, RT_FIX, RT_VEC, N, IS_X, Y, N, N, N, N, M_XRD, 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VLUXEI16_V  ->List(Y, N, X, uopVLUX,        IQT_MVEC,FU_MEMV,RT_VEC, RT_FIX, RT_VEC, N, IS_X, Y, N, N, N, N, M_XRD, 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 1.U)
 ,VLUXEI32_V  ->List(Y, N, X, uopVLUX,        IQT_MVEC,FU_MEMV,RT_VEC, RT_FIX, RT_VEC, N, IS_X, Y, N, N, N, N, M_XRD, 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 2.U)
 ,VLUXEI64_V  ->List(Y, N, X, uopVLUX,        IQT_MVEC,FU_MEMV,RT_VEC, RT_FIX, RT_VEC, N, IS_X, Y, N, N, N, N, M_XRD, 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 3.U)
 ,VSUXEI8_V   ->List(Y, N, X, uopVSUXA,       IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_VEC, Y, IS_X, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VSUXEI16_V  ->List(Y, N, X, uopVSUXA,       IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_VEC, Y, IS_X, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 1.U)
 ,VSUXEI32_V  ->List(Y, N, X, uopVSUXA,       IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_VEC, Y, IS_X, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 2.U)
 ,VSUXEI64_V  ->List(Y, N, X, uopVSUXA,       IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_VEC, Y, IS_X, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 3.U)
 ,VLOXEI8_V   ->List(Y, N, X, uopVLOX,        IQT_MVEC,FU_MEMV,RT_VEC, RT_FIX, RT_VEC, N, IS_X, Y, N, N, N, N, M_XRD, 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VLOXEI16_V  ->List(Y, N, X, uopVLOX,        IQT_MVEC,FU_MEMV,RT_VEC, RT_FIX, RT_VEC, N, IS_X, Y, N, N, N, N, M_XRD, 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 1.U)
 ,VLOXEI32_V  ->List(Y, N, X, uopVLOX,        IQT_MVEC,FU_MEMV,RT_VEC, RT_FIX, RT_VEC, N, IS_X, Y, N, N, N, N, M_XRD, 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 2.U)
 ,VLOXEI64_V  ->List(Y, N, X, uopVLOX,        IQT_MVEC,FU_MEMV,RT_VEC, RT_FIX, RT_VEC, N, IS_X, Y, N, N, N, N, M_XRD, 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 3.U)
 ,VSOXEI8_V   ->List(Y, N, X, uopVSOXA,       IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_VEC, Y, IS_X, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VSOXEI16_V  ->List(Y, N, X, uopVSOXA,       IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_VEC, Y, IS_X, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 1.U)
 ,VSOXEI32_V  ->List(Y, N, X, uopVSOXA,       IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_VEC, Y, IS_X, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 2.U)
 ,VSOXEI64_V  ->List(Y, N, X, uopVSOXA,       IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_VEC, Y, IS_X, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 3.U)
 )
}

object VectorIntDecode extends DecodeConstants
{
  val table: Array[(BitPat, List[BitPat])] = Array(
              //                                                                       frs3_en                        wakeup_delay
              //     is val inst?                                                      |  imm sel                     |    bypassable (aka, known/fixed latency)
              //     |  is fp inst?                                                    |  |     uses_ldq              |    |  is_br              is vector instruction
              //     |  |  is single-prec?                             rs1 regtype     |  |     |  uses_stq           |    |  |                  |  can be split
              //     |  |  |  micro-code                               |       rs2 type|  |     |  |  is_amo          |    |  |                  |  |  use vm?
              //     |  |  |  |               iq-type  func unit       |       |       |  |     |  |  |  is_fence     |    |  |                  |  |  |  ew of ls vector
              //     |  |  |  |               |        |               |       |       |  |     |  |  |  |  is_fencei |    |  |  is breakpoint or ecall?  |
              //     |  |  |  |               |        |       dst     |       |       |  |     |  |  |  |  |  mem    |    |  |  |  is unique? (clear pipeline for it)
              //     |  |  |  |               |        |       regtype |       |       |  |     |  |  |  |  |  cmd    |    |  |  |  |  flush on commit |  |
              //     |  |  |  |               |        |       |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  csr cmd   |  |  |
              //     |  |  |  |               |        |       |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  |      |  |  |  |
  VADD_VV     ->List(Y, N, X, uopVADD,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VADD_VX     ->List(Y, N, X, uopVADD,        IQT_IVEC,FU_ALU ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VADD_VI     ->List(Y, N, X, uopVADD,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VI , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VSUB_VV     ->List(Y, N, X, uopVSUB,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VSUB_VX     ->List(Y, N, X, uopVSUB,        IQT_IVEC,FU_ALU ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VRSUB_VX    ->List(Y, N, X, uopVRSUB,       IQT_IVEC,FU_ALU ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VRSUB_VI    ->List(Y, N, X, uopVRSUB,       IQT_VEC ,FU_ALU ,RT_VEC, RT_VI , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VWADDU_VV   ->List(Y, N, X, uopVADD,        IQT_VEC ,FU_ALU ,RT_VW , RT_VU , RT_VU , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VWADDU_VX   ->List(Y, N, X, uopVADD,        IQT_IVEC,FU_ALU ,RT_VW , RT_FIXU,RT_VU , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VWSUBU_VV   ->List(Y, N, X, uopVSUB,        IQT_VEC ,FU_ALU ,RT_VW , RT_VU , RT_VU , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VWSUBU_VX   ->List(Y, N, X, uopVSUB,        IQT_IVEC,FU_ALU ,RT_VW , RT_FIXU,RT_VU , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VWADD_VV    ->List(Y, N, X, uopVADD,        IQT_VEC ,FU_ALU ,RT_VW , RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VWADD_VX    ->List(Y, N, X, uopVADD,        IQT_IVEC,FU_ALU ,RT_VW , RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VWSUB_VV    ->List(Y, N, X, uopVSUB,        IQT_VEC ,FU_ALU ,RT_VW , RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VWSUB_VX    ->List(Y, N, X, uopVSUB,        IQT_IVEC,FU_ALU ,RT_VW , RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VWADDU_WV   ->List(Y, N, X, uopVADD,        IQT_VEC ,FU_ALU ,RT_VW , RT_VU , RT_VWU, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VWADDU_WX   ->List(Y, N, X, uopVADD,        IQT_IVEC,FU_ALU ,RT_VW , RT_FIXU,RT_VWU, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VWSUBU_WV   ->List(Y, N, X, uopVSUB,        IQT_VEC ,FU_ALU ,RT_VW , RT_VU , RT_VWU, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VWSUBU_WX   ->List(Y, N, X, uopVSUB,        IQT_IVEC,FU_ALU ,RT_VW , RT_FIXU,RT_VWU, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VWADD_WV    ->List(Y, N, X, uopVADD,        IQT_VEC ,FU_ALU ,RT_VW , RT_VEC, RT_VW , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VWADD_WX    ->List(Y, N, X, uopVADD,        IQT_IVEC,FU_ALU ,RT_VW , RT_FIX, RT_VW , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VWSUB_WV    ->List(Y, N, X, uopVSUB,        IQT_VEC ,FU_ALU ,RT_VW , RT_VEC, RT_VW , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VWSUB_WX    ->List(Y, N, X, uopVSUB,        IQT_IVEC,FU_ALU ,RT_VW , RT_FIX, RT_VW , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VZEXT_VF2   ->List(Y, N, X, uopVEXT2,       IQT_VEC ,FU_ALU ,RT_VW , RT_X  , RT_VNU, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VSEXT_VF2   ->List(Y, N, X, uopVEXT2,       IQT_VEC ,FU_ALU ,RT_VW , RT_X  , RT_VN , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VZEXT_VF4   ->List(Y, N, X, uopVEXT4,       IQT_VEC ,FU_ALU ,RT_VW , RT_X  , RT_VNU, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VSEXT_VF4   ->List(Y, N, X, uopVEXT4,       IQT_VEC ,FU_ALU ,RT_VW , RT_X  , RT_VN , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VZEXT_VF8   ->List(Y, N, X, uopVEXT8,       IQT_VEC ,FU_ALU ,RT_VW , RT_X  , RT_VNU, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VSEXT_VF8   ->List(Y, N, X, uopVEXT8,       IQT_VEC ,FU_ALU ,RT_VW , RT_X  , RT_VN , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VADC_VVM    ->List(Y, N, X, uopVADC,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VADC_VXM    ->List(Y, N, X, uopVADC,        IQT_IVEC,FU_ALU ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VADC_VIM    ->List(Y, N, X, uopVADC,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VI , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VMADC_VVM   ->List(Y, N, X, uopVMADC,       IQT_VEC ,FU_ALU ,RT_VM , RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VMADC_VXM   ->List(Y, N, X, uopVMADC,       IQT_IVEC,FU_ALU ,RT_VM , RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VMADC_VIM   ->List(Y, N, X, uopVMADC,       IQT_VEC ,FU_ALU ,RT_VM , RT_VI , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VSBC_VVM    ->List(Y, N, X, uopVSBC,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VSBC_VXM    ->List(Y, N, X, uopVSBC,        IQT_IVEC,FU_ALU ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VMSBC_VVM   ->List(Y, N, X, uopVMSBC,       IQT_VEC ,FU_ALU ,RT_VM , RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VMSBC_VXM   ->List(Y, N, X, uopVMSBC,       IQT_IVEC,FU_ALU ,RT_VM , RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VAND_VV     ->List(Y, N, X, uopVAND,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VAND_VX     ->List(Y, N, X, uopVAND,        IQT_IVEC,FU_ALU ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VAND_VI     ->List(Y, N, X, uopVAND,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VI , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VOR_VV      ->List(Y, N, X, uopVOR,         IQT_VEC ,FU_ALU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VOR_VX      ->List(Y, N, X, uopVOR,         IQT_IVEC,FU_ALU ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VOR_VI      ->List(Y, N, X, uopVOR,         IQT_VEC ,FU_ALU ,RT_VEC, RT_VI , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VXOR_VV     ->List(Y, N, X, uopVXOR,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VXOR_VX     ->List(Y, N, X, uopVXOR,        IQT_IVEC,FU_ALU ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VXOR_VI     ->List(Y, N, X, uopVXOR,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VI , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VSLL_VV     ->List(Y, N, X, uopVSLL,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VU , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VSLL_VX     ->List(Y, N, X, uopVSLL,        IQT_IVEC,FU_ALU ,RT_VEC, RT_FIXU,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VSLL_VI     ->List(Y, N, X, uopVSLL,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VIU, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VSRL_VV     ->List(Y, N, X, uopVSRL,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VU , RT_VU , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VSRL_VX     ->List(Y, N, X, uopVSRL,        IQT_IVEC,FU_ALU ,RT_VEC, RT_FIXU,RT_VU , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VSRL_VI     ->List(Y, N, X, uopVSRL,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VIU, RT_VU , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VSRA_VV     ->List(Y, N, X, uopVSRA,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VU , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VSRA_VX     ->List(Y, N, X, uopVSRA,        IQT_IVEC,FU_ALU ,RT_VEC, RT_FIXU,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VSRA_VI     ->List(Y, N, X, uopVSRA,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VIU, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VNSRL_WV    ->List(Y, N, X, uopVSRL,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VU , RT_VWU, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VNSRL_WX    ->List(Y, N, X, uopVSRL,        IQT_IVEC,FU_ALU ,RT_VEC, RT_FIXU,RT_VWU, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VNSRL_WI    ->List(Y, N, X, uopVSRL,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VIU, RT_VWU, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VNSRA_WV    ->List(Y, N, X, uopVSRA,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VU , RT_VW , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VNSRA_WX    ->List(Y, N, X, uopVSRA,        IQT_IVEC,FU_ALU ,RT_VEC, RT_FIXU,RT_VW , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VNSRA_WI    ->List(Y, N, X, uopVSRA,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VIU, RT_VW , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VMINU_VV    ->List(Y, N, X, uopVMINU,       IQT_VEC ,FU_ALU ,RT_VEC, RT_VU , RT_VU , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VMINU_VX    ->List(Y, N, X, uopVMINU,       IQT_IVEC,FU_ALU ,RT_VEC, RT_FIXU,RT_VU , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VMIN_VV     ->List(Y, N, X, uopVMIN,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VMIN_VX     ->List(Y, N, X, uopVMIN,        IQT_IVEC,FU_ALU ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VMAXU_VV    ->List(Y, N, X, uopVMAXU,       IQT_VEC ,FU_ALU ,RT_VEC, RT_VU , RT_VU , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VMAXU_VX    ->List(Y, N, X, uopVMAXU,       IQT_IVEC,FU_ALU ,RT_VEC, RT_FIXU,RT_VU , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VMAX_VV     ->List(Y, N, X, uopVMAX,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VMAX_VX     ->List(Y, N, X, uopVMAX,        IQT_IVEC,FU_ALU ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VMUL_VV     ->List(Y, N, X, uopVMUL,        IQT_VEC ,FU_MAC ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VMUL_VX     ->List(Y, N, X, uopVMUL,        IQT_IVEC,FU_MAC ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VMULH_VV    ->List(Y, N, X, uopVMULH,       IQT_VEC ,FU_MAC ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VMULH_VX    ->List(Y, N, X, uopVMULH,       IQT_IVEC,FU_MAC ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VMULHU_VV   ->List(Y, N, X, uopVMULHU,      IQT_VEC ,FU_MAC ,RT_VEC, RT_VU , RT_VU , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VMULHU_VX   ->List(Y, N, X, uopVMULHU,      IQT_IVEC,FU_MAC ,RT_VEC, RT_FIXU,RT_VU , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VMULHSU_VV  ->List(Y, N, X, uopVMULHSU,     IQT_VEC ,FU_MAC ,RT_VEC, RT_VU , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VMULHSU_VX  ->List(Y, N, X, uopVMULHSU,     IQT_IVEC,FU_MAC ,RT_VEC, RT_FIXU,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VDIVU_VV    ->List(Y, N, X, uopVDIVU,       IQT_VEC ,FU_DIV ,RT_VEC, RT_VU , RT_VU , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VDIVU_VX    ->List(Y, N, X, uopVDIVU,       IQT_IVEC,FU_DIV ,RT_VEC, RT_FIXU,RT_VU , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VDIV_VV     ->List(Y, N, X, uopVDIV,        IQT_VEC ,FU_DIV ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VDIV_VX     ->List(Y, N, X, uopVDIV,        IQT_IVEC,FU_DIV ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VREMU_VV    ->List(Y, N, X, uopVREMU,       IQT_VEC ,FU_DIV ,RT_VEC, RT_VU , RT_VU , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VREMU_VX    ->List(Y, N, X, uopVREMU,       IQT_IVEC,FU_DIV ,RT_VEC, RT_FIXU,RT_VU , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VREM_VV     ->List(Y, N, X, uopVREM,        IQT_VEC ,FU_DIV ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VREM_VX     ->List(Y, N, X, uopVREM,        IQT_IVEC,FU_DIV ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VWMUL_VV    ->List(Y, N, X, uopVMUL,        IQT_VEC ,FU_MAC ,RT_VW , RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VWMUL_VX    ->List(Y, N, X, uopVMUL,        IQT_IVEC,FU_MAC ,RT_VW , RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VWMULU_VV   ->List(Y, N, X, uopVWMULU,      IQT_VEC ,FU_MAC ,RT_VWU, RT_VU , RT_VU , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VWMULU_VX   ->List(Y, N, X, uopVWMULU,      IQT_IVEC,FU_MAC ,RT_VWU, RT_FIXU,RT_VU , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VWMULSU_VV  ->List(Y, N, X, uopVWMULSU,     IQT_VEC ,FU_MAC ,RT_VW , RT_VU , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VWMULSU_VX  ->List(Y, N, X, uopVWMULSU,     IQT_IVEC,FU_MAC ,RT_VW , RT_FIXU,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VMACC_VV    ->List(Y, N, X, uopVMACC,       IQT_VEC ,FU_MAC ,RT_VEC, RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VMACC_VX    ->List(Y, N, X, uopVMACC,       IQT_IVEC,FU_MAC ,RT_VEC, RT_FIX, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VNMSAC_VV   ->List(Y, N, X, uopVNMSAC,      IQT_VEC ,FU_MAC ,RT_VEC, RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VNMSAC_VX   ->List(Y, N, X, uopVNMSAC,      IQT_IVEC,FU_MAC ,RT_VEC, RT_FIX, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VMADD_VV    ->List(Y, N, X, uopVMADD,       IQT_VEC ,FU_MAC ,RT_VEC, RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VMADD_VX    ->List(Y, N, X, uopVMADD,       IQT_IVEC,FU_MAC ,RT_VEC, RT_FIX, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VNMSUB_VV   ->List(Y, N, X, uopVNMSUB,      IQT_VEC ,FU_MAC ,RT_VEC, RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VNMSUB_VX   ->List(Y, N, X, uopVNMSUB,      IQT_IVEC,FU_MAC ,RT_VEC, RT_FIX, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VWMACCU_VV  ->List(Y, N, X, uopVWMACCU,     IQT_VEC ,FU_MAC ,RT_VWU, RT_VU , RT_VU , Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VWMACCU_VX  ->List(Y, N, X, uopVWMACCU,     IQT_IVEC,FU_MAC ,RT_VWU, RT_FIXU,RT_VU , Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VWMACC_VV   ->List(Y, N, X, uopVMACC,       IQT_VEC ,FU_MAC ,RT_VW , RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VWMACC_VX   ->List(Y, N, X, uopVMACC,       IQT_IVEC,FU_MAC ,RT_VW , RT_FIX, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VWMACCSU_VV ->List(Y, N, X, uopVWMACCSU,    IQT_VEC ,FU_MAC ,RT_VW , RT_VEC, RT_VU , Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VWMACCSU_VX ->List(Y, N, X, uopVWMACCSU,    IQT_IVEC,FU_MAC ,RT_VW , RT_FIX, RT_VU , Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VWMACCUS_VX ->List(Y, N, X, uopVWMACCUS,    IQT_IVEC,FU_MAC ,RT_VW , RT_FIXU,RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VMAND_MM    ->List(Y, Y, X, uopVMAND,       IQT_VEC ,FU_ALU ,RT_VM,  RT_VM,  RT_VM,  N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, N, 0.U)
 ,VMNAND_MM   ->List(Y, Y, X, uopVMNAND,      IQT_VEC ,FU_ALU ,RT_VM,  RT_VM,  RT_VM,  N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, N, 0.U)
 ,VMANDNOT_MM ->List(Y, Y, X, uopVMANDNOT,    IQT_VEC ,FU_ALU ,RT_VM,  RT_VM,  RT_VM,  N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, N, 0.U)
 ,VMXOR_MM    ->List(Y, Y, X, uopVMXOR,       IQT_VEC ,FU_ALU ,RT_VM,  RT_VM,  RT_VM,  N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, N, 0.U)
 ,VMOR_MM     ->List(Y, Y, X, uopVMOR,        IQT_VEC ,FU_ALU ,RT_VM,  RT_VM,  RT_VM,  N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, N, 0.U)
 ,VMNOR_MM    ->List(Y, Y, X, uopVMNOR,       IQT_VEC ,FU_ALU ,RT_VM,  RT_VM,  RT_VM,  N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, N, 0.U)
 ,VMORNOT_MM  ->List(Y, Y, X, uopVMORNOT,     IQT_VEC ,FU_ALU ,RT_VM,  RT_VM,  RT_VM,  N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, N, 0.U)
 ,VMXNOR_MM   ->List(Y, Y, X, uopVMXNOR,      IQT_VEC ,FU_ALU ,RT_VM,  RT_VM,  RT_VM,  N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, N, 0.U)
 ,VMV_V_V     ->List(Y, N, X, uopVMV_V,       IQT_VEC ,FU_ALU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VMV_V_X     ->List(Y, N, X, uopVMV_V,       IQT_IVEC,FU_ALU ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VMV_V_I     ->List(Y, N, X, uopVMV_V,       IQT_VEC, FU_ALU ,RT_VEC, RT_VI,  RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VMERGE_VVM  ->List(Y, N, X, uopMERGE,       IQT_VEC ,FU_ALU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VMERGE_VXM  ->List(Y, N, X, uopMERGE,       IQT_IVEC,FU_ALU ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VMERGE_VIM  ->List(Y, N, X, uopMERGE,       IQT_VEC ,FU_ALU ,RT_VEC, RT_VI , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 )
}

object VectorFPDecode extends DecodeConstants
{
  val table: Array[(BitPat, List[BitPat])] = Array (
                   //                                                                       frs3_en                        wakeup_delay
                   //     is val inst?                                                      |  imm sel                     |    bypassable (aka, known/fixed latency)
                   //     |  is fp inst?                                                    |  |     uses_ldq              |    |  is_br              is vector instruction
                   //     |  |  is single-prec?                             rs1 regtype     |  |     |  uses_stq           |    |  |                  |  can be split
                   //     |  |  |  micro-code                               |       rs2 type|  |     |  |  is_amo          |    |  |                  |  |  use vm?
                   //     |  |  |  |               iq-type  func unit       |       |       |  |     |  |  |  is_fence     |    |  |                  |  |  |  ew of ls vector
                   //     |  |  |  |               |        |               |       |       |  |     |  |  |  |  is_fencei |    |  |  is breakpoint or ecall?  |
                   //     |  |  |  |               |        |       dst     |       |       |  |     |  |  |  |  |  mem    |    |  |  |  is unique? (clear pipeline for it)
                   //     |  |  |  |               |        |       regtype |       |       |  |     |  |  |  |  |  cmd    |    |  |  |  |  flush on commit |  |
                   //     |  |  |  |               |        |       |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  csr cmd   |  |  |
                   //     |  |  |  |               |        |       |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  |      |  |  |  |
  VFADD_VV         ->List(Y, Y, X, uopVFADD,       IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFADD_VF         ->List(Y, Y, X, uopVFADD,       IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFSUB_VV         ->List(Y, Y, X, uopVFSUB,       IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFSUB_VF         ->List(Y, Y, X, uopVFSUB,       IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFRSUB_VF        ->List(Y, Y, X, uopVFRSUB,      IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFWADD_VV        ->List(Y, Y, X, uopVFADD,       IQT_VEC ,FU_FPU ,RT_VW , RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFWADD_VF        ->List(Y, Y, X, uopVFADD,       IQT_FVEC,FU_FPU ,RT_VW , RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFWADD_WV        ->List(Y, Y, X, uopVFADD,       IQT_VEC ,FU_FPU ,RT_VW , RT_VEC, RT_VW , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFWADD_WF        ->List(Y, Y, X, uopVFADD,       IQT_FVEC,FU_FPU ,RT_VW , RT_FLT, RT_VW , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFWSUB_VV        ->List(Y, Y, X, uopVFSUB,       IQT_VEC ,FU_FPU ,RT_VW , RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFWSUB_VF        ->List(Y, Y, X, uopVFSUB,       IQT_FVEC,FU_FPU ,RT_VW , RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFWSUB_WV        ->List(Y, Y, X, uopVFSUB,       IQT_VEC ,FU_FPU ,RT_VW , RT_VEC, RT_VW , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFWSUB_WF        ->List(Y, Y, X, uopVFSUB,       IQT_FVEC,FU_FPU ,RT_VW , RT_FLT, RT_VW , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFMUL_VV         ->List(Y, Y, X, uopVFMUL,       IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFMUL_VF         ->List(Y, Y, X, uopVFMUL,       IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFDIV_VV         ->List(Y, Y, X, uopVFDIV,       IQT_VEC ,FU_FDV ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFDIV_VF         ->List(Y, Y, X, uopVFDIV,       IQT_FVEC,FU_FDV ,RT_VEC, RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFRDIV_VF        ->List(Y, Y, X, uopVFRDIV,      IQT_FVEC,FU_FDV ,RT_VEC, RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFWMUL_VV        ->List(Y, Y, X, uopVFMUL,       IQT_VEC ,FU_FPU ,RT_VW , RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFWMUL_VF        ->List(Y, Y, X, uopVFMUL,       IQT_FVEC,FU_FPU ,RT_VW , RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFMACC_VV        ->List(Y, Y, X, uopVFMACC,      IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFMACC_VF        ->List(Y, Y, X, uopVFMACC,      IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFNMACC_VV       ->List(Y, Y, X, uopVFNMACC,     IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFNMACC_VF       ->List(Y, Y, X, uopVFNMACC,     IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFMSAC_VV        ->List(Y, Y, X, uopVFMSAC,      IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFMSAC_VF        ->List(Y, Y, X, uopVFMSAC,      IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFNMSAC_VV       ->List(Y, Y, X, uopVFNMSAC,     IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFNMSAC_VF       ->List(Y, Y, X, uopVFNMSAC,     IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFMADD_VV        ->List(Y, Y, X, uopVFMADD,      IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFMADD_VF        ->List(Y, Y, X, uopVFMADD,      IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFNMADD_VV       ->List(Y, Y, X, uopVFNMADD,     IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFNMADD_VF       ->List(Y, Y, X, uopVFNMADD,     IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFMSUB_VV        ->List(Y, Y, X, uopVFMSUB,      IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFMSUB_VF        ->List(Y, Y, X, uopVFMSUB,      IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFNMSUB_VV       ->List(Y, Y, X, uopVFNMSUB,     IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFNMSUB_VF       ->List(Y, Y, X, uopVFNMSUB,     IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFWMACC_VV       ->List(Y, Y, X, uopVFMACC,      IQT_VEC ,FU_FPU ,RT_VW , RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFWMACC_VF       ->List(Y, Y, X, uopVFMACC,      IQT_FVEC,FU_FPU ,RT_VW , RT_FLT, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFWNMACC_VV      ->List(Y, Y, X, uopVFNMACC,     IQT_VEC ,FU_FPU ,RT_VW , RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFWNMACC_VF      ->List(Y, Y, X, uopVFNMACC,     IQT_FVEC,FU_FPU ,RT_VW , RT_FLT, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFWMSAC_VV       ->List(Y, Y, X, uopVFMSAC,      IQT_VEC ,FU_FPU ,RT_VW , RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFWMSAC_VF       ->List(Y, Y, X, uopVFMSAC,      IQT_FVEC,FU_FPU ,RT_VW , RT_FLT, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFWNMSAC_VV      ->List(Y, Y, X, uopVFNMSAC,     IQT_VEC ,FU_FPU ,RT_VW , RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFWNMSAC_VF      ->List(Y, Y, X, uopVFNMSAC,     IQT_FVEC,FU_FPU ,RT_VW , RT_FLT, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFSQRT_V         ->List(Y, Y, X, uopVFSQRT,      IQT_VEC ,FU_FDV ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFMIN_VV         ->List(Y, Y, X, uopVFMIN,       IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFMIN_VF         ->List(Y, Y, X, uopVFMIN,       IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFMAX_VV         ->List(Y, Y, X, uopVFMAX,       IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFMAX_VF         ->List(Y, Y, X, uopVFMAX,       IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFSGNJ_VV        ->List(Y, Y, X, uopVFSGNJ,      IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFSGNJ_VF        ->List(Y, Y, X, uopVFSGNJ,      IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFSGNJN_VV       ->List(Y, Y, X, uopVFSGNJ,      IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFSGNJN_VF       ->List(Y, Y, X, uopVFSGNJ,      IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFSGNJX_VV       ->List(Y, Y, X, uopVFSGNJ,      IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFSGNJX_VF       ->List(Y, Y, X, uopVFSGNJ,      IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFCLASS_V        ->List(Y, Y, X, uopVFCLASS,     IQT_VEC ,FU_F2I ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFMV_V_F         ->List(Y, Y, X, uopVFMV_V_F,    IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFCVT_XU_F_V     ->List(Y, Y, X, uopVFCVT_F2I,   IQT_VEC ,FU_F2I ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFCVT_X_F_V      ->List(Y, Y, X, uopVFCVT_F2I,   IQT_VEC ,FU_F2I ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFCVT_F_XU_V     ->List(Y, Y, X, uopVFCVT_I2F,   IQT_VEC ,FU_I2F ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFCVT_F_X_V      ->List(Y, Y, X, uopVFCVT_I2F,   IQT_VEC ,FU_I2F ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFCVT_RTZ_XU_F_V ->List(Y, Y, X, uopVFCVT_F2I,   IQT_VEC ,FU_F2I ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFCVT_RTZ_X_F_V  ->List(Y, Y, X, uopVFCVT_F2I,   IQT_VEC ,FU_F2I ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFWCVT_XU_F_V    ->List(Y, Y, X, uopVFCVT_F2I,   IQT_VEC ,FU_F2I ,RT_VW , RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFWCVT_X_F_V     ->List(Y, Y, X, uopVFCVT_F2I,   IQT_VEC ,FU_F2I ,RT_VW , RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFWCVT_F_XU_V    ->List(Y, Y, X, uopVFCVT_I2F,   IQT_VEC ,FU_I2F ,RT_VW , RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFWCVT_F_X_V     ->List(Y, Y, X, uopVFCVT_I2F,   IQT_VEC ,FU_I2F ,RT_VW , RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFWCVT_F_F_V     ->List(Y, Y, X, uopVFCVT_F2F,   IQT_VEC ,FU_FPU ,RT_VW , RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFWCVT_RTZ_XU_F_V->List(Y, Y, X, uopVFCVT_F2I,   IQT_VEC ,FU_F2I ,RT_VW , RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFWCVT_RTZ_X_F_V ->List(Y, Y, X, uopVFCVT_F2I,   IQT_VEC ,FU_F2I ,RT_VW , RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFNCVT_XU_F_W    ->List(Y, Y, X, uopVFCVT_F2I,   IQT_VEC ,FU_F2I ,RT_VEC, RT_VEC, RT_VW , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFNCVT_X_F_W     ->List(Y, Y, X, uopVFCVT_F2I,   IQT_VEC ,FU_F2I ,RT_VEC, RT_VEC, RT_VW , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFNCVT_F_XU_W    ->List(Y, Y, X, uopVFCVT_I2F,   IQT_VEC ,FU_I2F ,RT_VEC, RT_VEC, RT_VW , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFNCVT_F_X_W     ->List(Y, Y, X, uopVFCVT_I2F,   IQT_VEC ,FU_I2F ,RT_VEC, RT_VEC, RT_VW , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFNCVT_F_F_W     ->List(Y, Y, X, uopVFCVT_F2F,   IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VW , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFNCVT_ROD_F_F_W ->List(Y, Y, X, uopVFCVT_F2F,   IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VW , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFNCVT_RTZ_XU_F_W->List(Y, Y, X, uopVFCVT_F2I,   IQT_VEC ,FU_F2I ,RT_VEC, RT_VEC, RT_VW , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFNCVT_RTZ_X_F_W ->List(Y, Y, X, uopVFCVT_F2I,   IQT_VEC ,FU_F2I ,RT_VEC, RT_VEC, RT_VW , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 )
}

object VectorRedDecode extends DecodeConstants
{
  val table: Array[(BitPat, List[BitPat])] = Array (
              //                                                                       frs3_en                        wakeup_delay
              //     is val inst?                                                      |  imm sel                     |    bypassable (aka, known/fixed latency)
              //     |  is fp inst?                                                    |  |     uses_ldq              |    |  is_br              is vector instruction
              //     |  |  is single-prec?                             rs1 regtype     |  |     |  uses_stq           |    |  |                  |  can be split
              //     |  |  |  micro-code                               |       rs2 type|  |     |  |  is_amo          |    |  |                  |  |  use vm?
              //     |  |  |  |               iq-type  func unit       |       |       |  |     |  |  |  is_fence     |    |  |                  |  |  |  ew of ls vector
              //     |  |  |  |               |        |               |       |       |  |     |  |  |  |  is_fencei |    |  |  is breakpoint or ecall?  |
              //     |  |  |  |               |        |       dst     |       |       |  |     |  |  |  |  |  mem    |    |  |  |  is unique? (clear pipeline for it)
              //     |  |  |  |               |        |       regtype |       |       |  |     |  |  |  |  |  cmd    |    |  |  |  |  flush on commit |  |
              //     |  |  |  |               |        |       |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  csr cmd   |  |  |
              //     |  |  |  |               |        |       |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  |      |  |  |  |
  VREDSUM_VS  ->List(Y, N, X, uopVADD,        IQT_VEC ,FU_ALU ,RT_VRED,RT_VRED,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VREDMAXU_VS ->List(Y, N, X, uopVMAXU,       IQT_VEC ,FU_ALU ,RT_VRED,RT_VRED,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VREDMAX_VS  ->List(Y, N, X, uopVMAX,        IQT_VEC ,FU_ALU ,RT_VRED,RT_VRED,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VREDMINU_VS ->List(Y, N, X, uopVMINU,       IQT_VEC ,FU_ALU ,RT_VRED,RT_VRED,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VREDMIN_VS  ->List(Y, N, X, uopVMIN,        IQT_VEC ,FU_ALU ,RT_VRED,RT_VRED,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VREDAND_VS  ->List(Y, N, X, uopVAND,        IQT_VEC ,FU_ALU ,RT_VRED,RT_VRED,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VREDOR_VS   ->List(Y, N, X, uopVOR,         IQT_VEC ,FU_ALU ,RT_VRED,RT_VRED,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VREDXOR_VS  ->List(Y, N, X, uopVXOR,        IQT_VEC ,FU_ALU ,RT_VRED,RT_VRED,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VWREDSUMU_VS->List(Y, N, X, uopVADD,        IQT_VEC ,FU_ALU ,RT_VRWU,RT_VRWU,RT_VU , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VWREDSUM_VS ->List(Y, N, X, uopVADD,        IQT_VEC ,FU_ALU ,RT_VRW ,RT_VRW ,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFREDOSUM_VS->List(Y, Y, X, uopVFADD,       IQT_VEC ,FU_FPU ,RT_VRED,RT_VRED,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFREDSUM_VS ->List(Y, Y, X, uopVFADD,       IQT_VEC ,FU_FPU ,RT_VRED,RT_VRED,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFREDMIN_VS ->List(Y, Y, X, uopVFMIN,       IQT_VEC ,FU_FPU ,RT_VRED,RT_VRED,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFREDMAX_VS ->List(Y, Y, X, uopVFMAX,       IQT_VEC ,FU_FPU ,RT_VRED,RT_VRED,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFWREDOSUM_VS->List(Y,Y, X, uopVFADD,       IQT_VEC ,FU_FPU ,RT_VRW ,RT_VRW ,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 ,VFWREDSUM_VS->List(Y, Y, X, uopVFADD,       IQT_VEC ,FU_FPU ,RT_VRW ,RT_VRW ,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, Y, Y, Y, 0.U)
 )
}
//scalastyle:on
// TODO:
// 1. make vsetvli and vsetivli not unique, by tagging at Decode stage, with a flop control whether using CSR.vconfig all decoder.vconfig

/**
 * RoCC initial decode
 */
object RoCCDecode extends DecodeConstants
{
  // Note: We use FU_CSR since CSR instructions cannot co-execute with RoCC instructions
                       //                                                                   frs3_en                        wakeup_delay
                       //     is val inst?                                                  |  imm sel                     |    bypassable (aka, known/fixed latency)
                       //     |  is fp inst?                                                |  |     uses_ldq              |    |  is_br
                       //     |  |  is single-prec                          rs1 regtype     |  |     |  uses_stq           |    |  |
                       //     |  |  |                                       |       rs2 type|  |     |  |  is_amo          |    |  |
                       //     |  |  |  micro-code           func unit       |       |       |  |     |  |  |  is_fence     |    |  |
                       //     |  |  |  |           iq-type  |               |       |       |  |     |  |  |  |  is_fencei |    |  |  is breakpoint or ecall?
                       //     |  |  |  |           |        |       dst     |       |       |  |     |  |  |  |  |  mem    |    |  |  |  is unique? (clear pipeline for it)
                       //     |  |  |  |           |        |       regtype |       |       |  |     |  |  |  |  |  cmd    |    |  |  |  |  flush on commit
                       //     |  |  |  |           |        |       |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  csr cmd
                       //     |  |  |  |           |        |       |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  |
  val table: Array[(BitPat, List[BitPat])] = Array(//       |       |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  |
    CUSTOM0            ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
    CUSTOM0_RS1        ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_FIX, RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
    CUSTOM0_RS1_RS2    ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
    CUSTOM0_RD         ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
    CUSTOM0_RD_RS1     ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
    CUSTOM0_RD_RS1_RS2 ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
    CUSTOM1            ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
    CUSTOM1_RS1        ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_FIX, RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
    CUSTOM1_RS1_RS2    ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
    CUSTOM1_RD         ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
    CUSTOM1_RD_RS1     ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
    CUSTOM1_RD_RS1_RS2 ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
    CUSTOM2            ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
    CUSTOM2_RS1        ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_FIX, RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
    CUSTOM2_RS1_RS2    ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
    CUSTOM2_RD         ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
    CUSTOM2_RD_RS1     ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
    CUSTOM2_RD_RS1_RS2 ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
    CUSTOM3            ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
    CUSTOM3_RS1        ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_FIX, RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
    CUSTOM3_RS1_RS2    ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
    CUSTOM3_RD         ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
    CUSTOM3_RD_RS1     ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U),
    CUSTOM3_RD_RS1_RS2 ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N, N, N, N, 0.U)
  )
}





/**
 * IO bundle for the Decode unit
 */
class DecodeUnitIo(implicit p: Parameters) extends BoomBundle
{
  val enq = new Bundle { val uop = Input(new MicroOp()) }
  val enq_stall = Output(Bool())
  val deq = new Bundle { val uop = Output(new MicroOp()) }
  val deq_fire = Input(Bool())
  val kill = Input(Bool())

  // from CSRFile
  val status = Input(new freechips.rocketchip.rocket.MStatus())
  val csr_decode = Flipped(new freechips.rocketchip.rocket.CSRDecodeIO)
  val csr_vconfig = Input(new VConfig)
  val interrupt = Input(Bool())
  val interrupt_cause = Input(UInt(xLen.W))
}

/**
 * Decode unit that takes in a single instruction and generates a MicroOp.
 */
class DecodeUnit(implicit p: Parameters) extends BoomModule
  with freechips.rocketchip.rocket.constants.MemoryOpConstants
{
  val io = IO(new DecodeUnitIo)

  val uop = Wire(new MicroOp())
  uop := io.enq.uop

  var decode_table = XDecode.table
  if (usingFPU) decode_table ++= FDecode.table
  if (usingFPU && usingFDivSqrt) decode_table ++= FDivSqrtDecode.table
  if (usingVector) decode_table ++= VectorCfgDecode.table
  if (usingVector) decode_table ++= VectorLSDecode.table
  if (usingVector) decode_table ++= VectorIntDecode.table
  if (usingVector) decode_table ++= VectorFPDecode.table
  //if (usingVector) decode_table ++= VectorRedDecode.table
  // if (usingRoCC) decode_table ++= RoCCDecode.table
  decode_table ++= (if (xLen == 64) X64Decode.table else X32Decode.table)

  val inst = uop.inst

  val cs = Wire(new CtrlSigs()).decode(inst, decode_table)

  // Exception Handling
  val vsetvl = cs.uopc.isOneOf(uopVSETVL, uopVSETVLI, uopVSETIVLI)
  io.csr_decode.csr := Mux(vsetvl, 0.U, inst(31,20))
  val csr_en = cs.csr_cmd.isOneOf(CSR.S, CSR.C, CSR.W) && !vsetvl
  val csr_ren = cs.csr_cmd.isOneOf(CSR.S, CSR.C) && uop.lrs1 === 0.U
  val system_insn = cs.csr_cmd === CSR.I
  val sfence = cs.uopc === uopSFENCE

  val cs_legal = cs.legal
//   dontTouch(cs_legal)

  val id_illegal_insn = !cs_legal ||
    cs.fp_val && io.csr_decode.fp_illegal || // TODO check for illegal rm mode: (io.fpu.illegal_rm)
    cs.is_rvv && io.csr_decode.vector_illegal ||
    cs.rocc && io.csr_decode.rocc_illegal ||
    cs.is_amo && !io.status.isa('a'-'a')  ||
    (cs.fp_val && !cs.fp_single) && !io.status.isa('d'-'a') ||
    csr_en && (io.csr_decode.read_illegal || !csr_ren && io.csr_decode.write_illegal) ||
    ((sfence || system_insn) && io.csr_decode.system_illegal)

//     cs.div && !csr.io.status.isa('m'-'a') || TODO check for illegal div instructions

  def checkExceptions(x: Seq[(Bool, UInt)]) =
    (x.map(_._1).reduce(_||_), PriorityMux(x))

  val (xcpt_valid, xcpt_cause) = checkExceptions(List(
    (io.interrupt && !io.enq.uop.is_sfb, io.interrupt_cause),  // Disallow interrupts while we are handling a SFB
    (uop.bp_debug_if,                    (CSR.debugTriggerCause).U),
    (uop.bp_xcpt_if,                     (Causes.breakpoint).U),
    (uop.xcpt_pf_if,                     (Causes.fetch_page_fault).U),
    (uop.xcpt_ae_if,                     (Causes.fetch_access).U),
    (id_illegal_insn,                    (Causes.illegal_instruction).U)))

  uop.exception := xcpt_valid
  uop.exc_cause := xcpt_cause

  //-------------------------------------------------------------

  uop.uopc       := cs.uopc
  uop.iq_type    := cs.iq_type
  uop.fu_code    := cs.fu_code

  // x-registers placed in 0-31, f-registers placed in 32-63.
  // This allows us to straight-up compare register specifiers and not need to
  // verify the rtypes (e.g., bypassing in rename).
  uop.ldst       := inst(RD_MSB,RD_LSB)
  uop.lrs1       := inst(RS1_MSB,RS1_LSB)
  uop.lrs2       := inst(RS2_MSB,RS2_LSB)
  uop.lrs3       := inst(RS3_MSB,RS3_LSB)

  uop.ldst_val   := isSomeReg(cs.dst_type) && !(uop.ldst === 0.U && uop.rt(RD, isInt))
  uop.dst_rtype  := cs.dst_type
  uop.lrs1_rtype := cs.rs1_type
  uop.lrs2_rtype := cs.rs2_type
  uop.frs3_en    := cs.frs3_en

  uop.ldst_is_rs1 := uop.is_sfb_shadow
  // SFB optimization
  when (uop.is_sfb_shadow && isNotReg(cs.rs2_type)) {
    uop.lrs2_rtype  := RT_FIX
    uop.lrs2        := inst(RD_MSB,RD_LSB)
    uop.ldst_is_rs1 := false.B
  } .elsewhen (uop.is_sfb_shadow && cs.uopc === uopADD && inst(RS1_MSB,RS1_LSB) === 0.U) {
    uop.uopc        := uopMOV
    uop.lrs1        := inst(RD_MSB, RD_LSB)
    uop.ldst_is_rs1 := true.B
  }
  when (uop.is_sfb_br) {
    uop.fu_code := FU_JMP
  }


  uop.fp_val     := cs.fp_val
  uop.fp_single  := cs.fp_single // TODO use this signal instead of the FPU decode's table signal?

  uop.mem_cmd    := cs.mem_cmd
  uop.mem_size   := Mux(cs.mem_cmd.isOneOf(M_SFENCE, M_FLUSH_ALL), Cat(uop.lrs2 =/= 0.U, uop.lrs1 =/= 0.U), inst(13,12))
  uop.mem_signed := !inst(14)
  uop.uses_ldq   := cs.uses_ldq
  uop.uses_stq   := cs.uses_stq
  uop.is_amo     := cs.is_amo
  uop.is_fence   := cs.is_fence
  uop.is_fencei  := cs.is_fencei
  uop.is_sys_pc2epc   := cs.is_sys_pc2epc
  uop.is_unique  := cs.inst_unique
  uop.flush_on_commit := cs.flush_on_commit || (csr_en && !csr_ren && io.csr_decode.write_flush)

  uop.bypassable   := cs.bypassable

  //-------------------------------------------------------------
  // immediates

  // repackage the immediate, and then pass the fewest number of bits around
  val di24_20 = Mux(cs.imm_sel === IS_B || cs.imm_sel === IS_S, inst(11,7), inst(24,20))
  uop.imm_packed := Cat(inst(31,25), di24_20, inst(19,12))

  //-------------------------------------------------------------

  uop.is_br          := cs.is_br
  uop.is_jal         := (uop.uopc === uopJAL)
  uop.is_jalr        := (uop.uopc === uopJALR)
  // uop.is_jump        := cs.is_jal || (uop.uopc === uopJALR)
  // uop.is_ret         := (uop.uopc === uopJALR) &&
  //                       (uop.ldst === X0) &&
  //                       (uop.lrs1 === RA)
  // uop.is_call        := (uop.uopc === uopJALR || uop.uopc === uopJAL) &&
  //                       (uop.ldst === RA)

  //-------------------------------------------------------------
  // vector stuff
  //
  if (usingVector) {
    val is_v_ls = cs.is_rvv & (cs.uses_stq | cs.uses_ldq)
    val is_v_ls_stride = cs.uopc.isOneOf(uopVLS, uopVSSA)
    val is_v_ls_index = cs.uopc.isOneOf(uopVLUX, uopVSUXA, uopVLOX, uopVSOXA)
    val vseg_nf = inst(NF_MSB, NF_LSB)
    val is_v_ls_seg = is_v_ls && (vseg_nf =/= 0.U) // FIXME: exclude whole register load/store
    val vstart  = RegInit(0.U((vLenSz+1).W))
    val vseg_finc = RegInit(0.U(3.W))
    val vseg_gidx = RegInit(0.U(3.W))
    val vseg_elem = RegInit(0.U(vLenSz.W))
    val vlmax = io.csr_vconfig.vtype.vlMax
    val vsew = io.csr_vconfig.vtype.vsew
    val vlmul = io.csr_vconfig.vtype.vlmul_mag
    // excluding negative vlmul at the moment
    // TODO: support negative vlmul
    assert(!io.csr_vconfig.vtype.vlmul_sign)
    val vd_wfactor = Mux(uop.rt(RD,  isWidenV ), 1.U, 0.U)
    val vd_nfactor = Mux(uop.rt(RD,  isNarrowV), 1.U, 0.U)
    val vs2_wfactor= Mux(uop.rt(RS2, isWidenV ), 1.U, 0.U)
    val vs2_nfactor= Mux(uop.uopc === uopVEXT8 , 3.U,
                     Mux(uop.uopc === uopVEXT4 , 2.U,
                     Mux(uop.rt(RS2, isNarrowV), 1.U, 0.U))) // uopVEXT2 is included
    val vd_sew     = Mux(uop.rt(RD,  isWidenV ), vsew + vd_wfactor,
                     Mux(uop.rt(RD,  isNarrowV), vsew - vd_nfactor, vsew))
    val vs2_sew    = Mux(is_v_ls_index, Cat(0.U(1.W), cs.v_ls_ew),
                     Mux(uop.rt(RS2, isWidenV ), vsew + vs2_wfactor,
                     Mux(uop.rt(RS2, isNarrowV), vsew - vs2_nfactor,vsew)))
    val vd_inc     = vstart >> (vLenSz.U - 3.U - vd_sew)
    val vs2_inc    = vstart >> (vLenSz.U - 3.U - vs2_sew)
    val vs1_inc    = vstart >> (vLenSz.U - 3.U - vsew)
    when (io.deq_fire && cs.is_rvv) {
      assert(vsew <= 3.U, "Unsupported vsew")
      //assert(vsew >= vd_nfactor  && vsew + vd_wfactor  <= 3.U, "Unsupported vd_sew")
      //assert(vsew >= vs2_nfactor && vsew + vs2_wfactor <= 3.U, "Unsupported vs2_sew")
    }


    // vmasklogic 
    val vmlogic_insn = cs.uopc.isOneOf(uopVMAND, uopVMNAND, uopVMANDNOT, uopVMXOR, uopVMOR, uopVMNOR, uopVMORNOT, uopVMXNOR)
    val byteWidth = 3.U
    val vsew64bit = 3.U
    val vmlogic_split_ecnt = vLen.U >> (vsew +& byteWidth +& vsew64bit - io.csr_vconfig.vtype.vlmul_sign)
    // vsew => element num
    val vmlogic_tolal_ecnt = vlmax >> vsew64bit


    //val eLen_ecnt = eLen.U >> (vsew+3.U)
    //val vLen_ecnt = vLen.U >> (vd_sew+3.U)
    val vLen_ecnt = Mux(vs2_sew > vd_sew, vLen.U >> (vs2_sew+3.U), vLen.U >> (vd_sew+3.U))
    val split_ecnt = Mux(is_v_ls, 1.U, Mux(vmlogic_insn, vmlogic_split_ecnt, vLen_ecnt))
    // for store, we can skip inactive locations; otherwise, we have to visit every element
    val total_ecnt = Mux(cs.uses_stq, io.csr_vconfig.vl, Mux(vmlogic_insn, vmlogic_tolal_ecnt, vlmax))
    val split_last = vstart + split_ecnt === total_ecnt
    when (io.kill) {
      vstart    := 0.U
      vseg_finc := 0.U
      vseg_elem := 0.U
      vseg_gidx := 0.U
    } .elsewhen (~cs.can_be_split | split_last & io.deq_fire) {
      vstart    := 0.U
      vseg_finc := 0.U
      vseg_elem := 0.U
      vseg_gidx := 0.U
    } .elsewhen (cs.can_be_split & ~split_last & io.deq_fire) {
      vstart    := vstart + split_ecnt
      when (is_v_ls_seg) {
        vseg_finc := Mux(vseg_finc === vseg_nf, 0.U, vseg_finc + 1.U)
        vseg_elem := vseg_elem + Mux(vseg_finc === vseg_nf, 1.U, 0.U)
        vseg_gidx := vseg_gidx + Mux((vseg_finc === vseg_nf) && (vseg_elem + 1.U === vLen_ecnt), 1.U, 0.U)
      }
    }
    val vseg_vd_inc  = (vseg_finc << vlmul) + vseg_gidx
    val vseg_vs2_inc = vseg_elem >> (vLenSz.U - 3.U - vs2_sew)

    uop.is_rvv      := cs.is_rvv
    uop.v_ls_ew     := Mux(is_v_ls_index, vsew, cs.v_ls_ew)
    when (is_v_ls) {
      uop.mem_size  := uop.v_ls_ew
      uop.mem_signed:= false.B
    }
    uop.v_unmasked  := !cs.uses_vm || inst(VM_BIT)
    uop.vxsat       := false.B
    uop.vconfig     := io.csr_vconfig
    uop.vconfig.vtype.reserved := DontCare
    uop.vstart      := vstart
    uop.voffset     := 0.U
    uop.v_is_split  := cs.can_be_split
    uop.v_split_ecnt:= split_ecnt
    uop.vconfig.vtype.vsew := 3.U
    when (io.deq_fire && cs.can_be_split) {
      assert(cs.is_rvv, "can_be_split applies only to vector instructions.")
    }
    uop.v_is_first  := (vstart === 0.U)
    uop.v_is_last   := split_last
    val ren_mask = ~(Fill(vLenSz,1.U) << (7.U - vd_sew))
    uop.v_re_alloc  := (vstart & ren_mask(vLenSz,0)) === 0.U

    when (cs.is_rvv) {
      uop.ldst := inst(RD_MSB,RD_LSB)   + vd_inc
      when (uop.rt(RS1, isVector)) {
        uop.lrs1 := inst(RS1_MSB,RS1_LSB) + vs1_inc
      }
      uop.lrs2 := inst(RS2_MSB,RS2_LSB) + vs2_inc
      uop.lrs3 := uop.ldst
      uop.frs3_en := cs.uses_vm
    }

    when (cs.is_rvv && !uop.v_unmasked) {
      when (is_v_ls) {
        uop.iq_type := IQT_MVEC
        uop.fu_code := FU_MEMV
      }
      uop.frs3_en := true.B
    }

    uop.v_xls_offset := 0.U
    when (cs.is_rvv && is_v_ls_index) {
      uop.v_unmasked := false.B // force indexed load/store wait for vmupdate
    }

    uop.v_seg_ls := false.B
    uop.v_seg_f := 0.U
    uop.v_seg_e := 0.U
    when (cs.is_rvv && is_v_ls_seg) {
      uop.ldst := inst(RD_MSB,RD_LSB) + vseg_vd_inc
      uop.lrs2 := inst(RS2_MSB,RS2_LSB) + vseg_vs2_inc
      uop.lrs3 := uop.ldst
      uop.v_re_alloc := (vseg_elem & ren_mask(vLenSz,0)) === 0.U
      uop.v_seg_ls   := true.B
      uop.v_seg_f    := vseg_finc
      uop.v_seg_e    := vseg_elem
    }

    // handle load tail: dispatch to vector pipe
    // masked load / store, send to vector pipe
    when (cs.is_rvv && cs.uses_ldq && vstart >= io.csr_vconfig.vl) {
      uop.iq_type := IQT_VEC
      uop.fu_code := FU_VMX
      uop.uses_ldq := false.B
      uop.frs3_en := true.B
    }

    val red_op  = cs.is_rvv && isReduceV(cs.dst_type)
    val red_act = RegInit(false.B)

    when (io.kill) {
      red_act := false.B
    } .elsewhen (red_op && io.deq_fire && red_act && split_last) {
      red_act := false.B
    } .elsewhen (red_op && io.deq_fire && !red_act) {
      red_act := true.B
    }

    io.enq_stall := cs.can_be_split && !uop.v_is_last

    when (cs.is_rvv && red_op) {
      // keep vd during reduction
      uop.ldst := inst(RD_MSB,RD_LSB)
      uop.lrs3 := uop.ldst
      when (!red_act) {
        // insert an undisturbing move before actual reduction
        vstart          := vstart
        uop.uopc        := uopVADD
        uop.v_unmasked  := false.B
        uop.dst_rtype   := RT_VEC
        // keep lrs1_rtype to distinguish this inserted mov and actual reduction
        //uop.lrs1_rtype  := RT_VEC
        uop.vconfig.vl  := 0.U // make all elements inactive
        io.enq_stall    := true.B
      }
    }

  } // if usingvector
  io.deq.uop := uop

  //assert(!id_illegal_insn)
}

/**
 * Smaller Decode unit for the Frontend to decode different
 * branches.
 * Accepts EXPANDED RVC instructions
  */

class BranchDecodeSignals(implicit p: Parameters) extends BoomBundle
{
  val is_ret   = Bool()
  val is_call  = Bool()
  val target   = UInt(vaddrBitsExtended.W)
  val cfi_type = UInt(CFI_SZ.W)


  // Is this branch a short forwards jump?
  val sfb_offset = Valid(UInt(log2Ceil(icBlockBytes).W))
  // Is this instruction allowed to be inside a sfb?
  val shadowable = Bool()
}

class BranchDecode(implicit p: Parameters) extends BoomModule
{
  val io = IO(new Bundle {
    val inst    = Input(UInt(32.W))
    val pc      = Input(UInt(vaddrBitsExtended.W))

    val out = Output(new BranchDecodeSignals)
  })

  val bpd_csignals =
    freechips.rocketchip.rocket.DecodeLogic(io.inst,
                  List[BitPat](N, N, N, N, X),
////                               is br?
////                               |  is jal?
////                               |  |  is jalr?
////                               |  |  |
////                               |  |  |  shadowable
////                               |  |  |  |  has_rs2
////                               |  |  |  |  |
            Array[(BitPat, List[BitPat])](
               JAL         -> List(N, Y, N, N, X),
               JALR        -> List(N, N, Y, N, X),
               BEQ         -> List(Y, N, N, N, X),
               BNE         -> List(Y, N, N, N, X),
               BGE         -> List(Y, N, N, N, X),
               BGEU        -> List(Y, N, N, N, X),
               BLT         -> List(Y, N, N, N, X),
               BLTU        -> List(Y, N, N, N, X),

               SLLI        -> List(N, N, N, Y, N),
               SRLI        -> List(N, N, N, Y, N),
               SRAI        -> List(N, N, N, Y, N),

               ADDIW       -> List(N, N, N, Y, N),
               SLLIW       -> List(N, N, N, Y, N),
               SRAIW       -> List(N, N, N, Y, N),
               SRLIW       -> List(N, N, N, Y, N),

               ADDW        -> List(N, N, N, Y, Y),
               SUBW        -> List(N, N, N, Y, Y),
               SLLW        -> List(N, N, N, Y, Y),
               SRAW        -> List(N, N, N, Y, Y),
               SRLW        -> List(N, N, N, Y, Y),

               LUI         -> List(N, N, N, Y, N),

               ADDI        -> List(N, N, N, Y, N),
               ANDI        -> List(N, N, N, Y, N),
               ORI         -> List(N, N, N, Y, N),
               XORI        -> List(N, N, N, Y, N),
               SLTI        -> List(N, N, N, Y, N),
               SLTIU       -> List(N, N, N, Y, N),

               SLL         -> List(N, N, N, Y, Y),
               ADD         -> List(N, N, N, Y, Y),
               SUB         -> List(N, N, N, Y, Y),
               SLT         -> List(N, N, N, Y, Y),
               SLTU        -> List(N, N, N, Y, Y),
               AND         -> List(N, N, N, Y, Y),
               OR          -> List(N, N, N, Y, Y),
               XOR         -> List(N, N, N, Y, Y),
               SRA         -> List(N, N, N, Y, Y),
               SRL         -> List(N, N, N, Y, Y)
            ))

  val (cs_is_br: Bool) :: (cs_is_jal: Bool) :: (cs_is_jalr:Bool) :: (cs_is_shadowable:Bool) :: (cs_has_rs2) :: Nil = bpd_csignals

  io.out.is_call := (cs_is_jal || cs_is_jalr) && GetRd(io.inst) === RA
  io.out.is_ret  := cs_is_jalr && GetRs1(io.inst) === BitPat("b00?01") && GetRd(io.inst) === X0

  io.out.target := Mux(cs_is_br, ComputeBranchTarget(io.pc, io.inst, xLen),
                                 ComputeJALTarget(io.pc, io.inst, xLen))
  io.out.cfi_type :=
    Mux(cs_is_jalr,
      CFI_JALR,
    Mux(cs_is_jal,
      CFI_JAL,
    Mux(cs_is_br,
      CFI_BR,
      CFI_X)))

  val br_offset = Cat(io.inst(7), io.inst(30,25), io.inst(11,8), 0.U(1.W))
  // Is a sfb if it points forwards (offset is positive)
  io.out.sfb_offset.valid := cs_is_br && !io.inst(31) && br_offset =/= 0.U && (br_offset >> log2Ceil(icBlockBytes)) === 0.U
  io.out.sfb_offset.bits  := br_offset
  io.out.shadowable := cs_is_shadowable && (
    !cs_has_rs2 ||
    (GetRs1(io.inst) === GetRd(io.inst)) ||
    (io.inst === ADD && GetRs1(io.inst) === X0)
  )
}

/**
 * Track the current "branch mask", and give out the branch mask to each micro-op in Decode
 * (each micro-op in the machine has a branch mask which says which branches it
 * is being speculated under).
 *
 * @param pl_width pipeline width for the processor
 */
class BranchMaskGenerationLogic(val pl_width: Int)(implicit p: Parameters) extends BoomModule
{
  val io = IO(new Bundle {
    // guess if the uop is a branch (we'll catch this later)
    val is_branch = Input(Vec(pl_width, Bool()))
    // lock in that it's actually a branch and will fire, so we update
    // the branch_masks.
    val will_fire = Input(Vec(pl_width, Bool()))

    // give out tag immediately (needed in rename)
    // mask can come later in the cycle
    val br_tag    = Output(Vec(pl_width, UInt(brTagSz.W)))
    val br_mask   = Output(Vec(pl_width, UInt(maxBrCount.W)))

     // tell decoders the branch mask has filled up, but on the granularity
     // of an individual micro-op (so some micro-ops can go through)
    val is_full   = Output(Vec(pl_width, Bool()))

    val brupdate         = Input(new BrUpdateInfo())
    val flush_pipeline = Input(Bool())

    val debug_branch_mask = Output(UInt(maxBrCount.W))
  })

  val branch_mask = RegInit(0.U(maxBrCount.W))

  //-------------------------------------------------------------
  // Give out the branch tag to each branch micro-op

  var allocate_mask = branch_mask
  val tag_masks = Wire(Vec(pl_width, UInt(maxBrCount.W)))

  for (w <- 0 until pl_width) {
    // TODO this is a loss of performance as we're blocking branches based on potentially fake branches
    io.is_full(w) := (allocate_mask === ~(0.U(maxBrCount.W))) && io.is_branch(w)

    // find br_tag and compute next br_mask
    val new_br_tag = Wire(UInt(brTagSz.W))
    new_br_tag := 0.U
    tag_masks(w) := 0.U

    for (i <- maxBrCount-1 to 0 by -1) {
      when (~allocate_mask(i)) {
        new_br_tag := i.U
        tag_masks(w) := (1.U << i.U)
      }
    }

    io.br_tag(w) := new_br_tag
    allocate_mask = Mux(io.is_branch(w), tag_masks(w) | allocate_mask, allocate_mask)
  }

  //-------------------------------------------------------------
  // Give out the branch mask to each micro-op
  // (kill off the bits that corresponded to branches that aren't going to fire)

  var curr_mask = branch_mask
  for (w <- 0 until pl_width) {
    io.br_mask(w) := GetNewBrMask(io.brupdate, curr_mask)
    curr_mask = Mux(io.will_fire(w), tag_masks(w) | curr_mask, curr_mask)
  }

  //-------------------------------------------------------------
  // Update the current branch_mask

  when (io.flush_pipeline) {
    branch_mask := 0.U
  } .otherwise {
    val mask = Mux(io.brupdate.b2.mispredict,
      io.brupdate.b2.uop.br_mask,
      ~(0.U(maxBrCount.W)))
    branch_mask := GetNewBrMask(io.brupdate, curr_mask) & mask
  }

  io.debug_branch_mask := branch_mask
}
