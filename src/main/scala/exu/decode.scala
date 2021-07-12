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
  val DC2   = BitPat.dontCare(2) // Makes the listing below more readable
  val RT_DC = BitPat.dontCare(RT_X.getWidth)
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
              List(N, N, X, uopX    , IQT_INT, FU_X   , RT_X  , RT_DC  ,RT_DC  ,X, IS_X, X, X, X, X, N, M_X,   DC2, X, X, N, N, X, CSR.X, N, N, N, DC2)

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
    //val decoder = freechips.rocketchip.rocket.DecodeLogic(inst, XDecode.decode_default, table)
    val decoder = BoomDecoder(inst, XDecode.decode_default, table)
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
  SLLI_RV32-> List(Y, N, X, uopSLLI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_1, Y, N, N, N, N, CSR.N, N, N, N, DC2),
  SRLI_RV32-> List(Y, N, X, uopSRLI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_1, Y, N, N, N, N, CSR.N, N, N, N, DC2),
  SRAI_RV32-> List(Y, N, X, uopSRAI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_1, Y, N, N, N, N, CSR.N, N, N, N, DC2)
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
  LD      -> List(Y, N, X, uopLD   , IQT_MEM, FU_MEM , RT_FIX, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, U_3, N, N, N, N, N, CSR.N, N, N, N, DC2),
  LWU     -> List(Y, N, X, uopLD   , IQT_MEM, FU_MEM , RT_FIX, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, U_3, N, N, N, N, N, CSR.N, N, N, N, DC2),
  SD      -> List(Y, N, X, uopSTA  , IQT_MEM, FU_MEM , RT_X  , RT_FIX, RT_FIX, N, IS_S, N, Y, N, N, N, M_XWR, U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),

  SLLI    -> List(Y, N, X, uopSLLI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_1, Y, N, N, N, N, CSR.N, N, N, N, DC2),
  SRLI    -> List(Y, N, X, uopSRLI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_1, Y, N, N, N, N, CSR.N, N, N, N, DC2),
  SRAI    -> List(Y, N, X, uopSRAI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_1, Y, N, N, N, N, CSR.N, N, N, N, DC2),

  ADDIW   -> List(Y, N, X, uopADDIW, IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_1, Y, N, N, N, N, CSR.N, N, N, N, DC2),
  SLLIW   -> List(Y, N, X, uopSLLIW, IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_1, Y, N, N, N, N, CSR.N, N, N, N, DC2),
  SRAIW   -> List(Y, N, X, uopSRAIW, IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_1, Y, N, N, N, N, CSR.N, N, N, N, DC2),
  SRLIW   -> List(Y, N, X, uopSRLIW, IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_1, Y, N, N, N, N, CSR.N, N, N, N, DC2),

  ADDW    -> List(Y, N, X, uopADDW , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , U_1, Y, N, N, N, N, CSR.N, N, N, N, DC2),
  SUBW    -> List(Y, N, X, uopSUBW , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , U_1, Y, N, N, N, N, CSR.N, N, N, N, DC2),
  SLLW    -> List(Y, N, X, uopSLLW , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , U_1, Y, N, N, N, N, CSR.N, N, N, N, DC2),
  SRAW    -> List(Y, N, X, uopSRAW , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , U_1, Y, N, N, N, N, CSR.N, N, N, N, DC2),
  SRLW    -> List(Y, N, X, uopSRLW , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , U_1, Y, N, N, N, N, CSR.N, N, N, N, DC2)
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
  LW      -> List(Y, N, X, uopLD   , IQT_MEM, FU_MEM , RT_FIX, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, U_3, N, N, N, N, N, CSR.N, N, N, N, DC2),
  LH      -> List(Y, N, X, uopLD   , IQT_MEM, FU_MEM , RT_FIX, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, U_3, N, N, N, N, N, CSR.N, N, N, N, DC2),
  LHU     -> List(Y, N, X, uopLD   , IQT_MEM, FU_MEM , RT_FIX, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, U_3, N, N, N, N, N, CSR.N, N, N, N, DC2),
  LB      -> List(Y, N, X, uopLD   , IQT_MEM, FU_MEM , RT_FIX, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, U_3, N, N, N, N, N, CSR.N, N, N, N, DC2),
  LBU     -> List(Y, N, X, uopLD   , IQT_MEM, FU_MEM , RT_FIX, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, U_3, N, N, N, N, N, CSR.N, N, N, N, DC2),

  SW      -> List(Y, N, X, uopSTA  , IQT_MEM, FU_MEM , RT_X  , RT_FIX, RT_FIX, N, IS_S, N, Y, N, N, N, M_XWR, U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  SH      -> List(Y, N, X, uopSTA  , IQT_MEM, FU_MEM , RT_X  , RT_FIX, RT_FIX, N, IS_S, N, Y, N, N, N, M_XWR, U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  SB      -> List(Y, N, X, uopSTA  , IQT_MEM, FU_MEM , RT_X  , RT_FIX, RT_FIX, N, IS_S, N, Y, N, N, N, M_XWR, U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),

  LUI     -> List(Y, N, X, uopLUI  , IQT_INT, FU_ALU , RT_FIX, RT_X  , RT_X  , N, IS_U, N, N, N, N, N, M_X  , U_1, Y, N, N, N, N, CSR.N, N, N, N, DC2),

  ADDI    -> List(Y, N, X, uopADDI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_1, Y, N, N, N, N, CSR.N, N, N, N, DC2),
  ANDI    -> List(Y, N, X, uopANDI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_1, Y, N, N, N, N, CSR.N, N, N, N, DC2),
  ORI     -> List(Y, N, X, uopORI  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_1, Y, N, N, N, N, CSR.N, N, N, N, DC2),
  XORI    -> List(Y, N, X, uopXORI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_1, Y, N, N, N, N, CSR.N, N, N, N, DC2),
  SLTI    -> List(Y, N, X, uopSLTI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_1, Y, N, N, N, N, CSR.N, N, N, N, DC2),
  SLTIU   -> List(Y, N, X, uopSLTIU, IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_1, Y, N, N, N, N, CSR.N, N, N, N, DC2),

  SLL     -> List(Y, N, X, uopSLL  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , U_1, Y, N, N, N, N, CSR.N, N, N, N, DC2),
  ADD     -> List(Y, N, X, uopADD  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , U_1, Y, N, N, N, N, CSR.N, N, N, N, DC2),
  SUB     -> List(Y, N, X, uopSUB  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , U_1, Y, N, N, N, N, CSR.N, N, N, N, DC2),
  SLT     -> List(Y, N, X, uopSLT  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , U_1, Y, N, N, N, N, CSR.N, N, N, N, DC2),
  SLTU    -> List(Y, N, X, uopSLTU , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , U_1, Y, N, N, N, N, CSR.N, N, N, N, DC2),
  AND     -> List(Y, N, X, uopAND  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , U_1, Y, N, N, N, N, CSR.N, N, N, N, DC2),
  OR      -> List(Y, N, X, uopOR   , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , U_1, Y, N, N, N, N, CSR.N, N, N, N, DC2),
  XOR     -> List(Y, N, X, uopXOR  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , U_1, Y, N, N, N, N, CSR.N, N, N, N, DC2),
  SRA     -> List(Y, N, X, uopSRA  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , U_1, Y, N, N, N, N, CSR.N, N, N, N, DC2),
  SRL     -> List(Y, N, X, uopSRL  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , U_1, Y, N, N, N, N, CSR.N, N, N, N, DC2),

  MUL     -> List(Y, N, X, uopMUL  , IQT_INT, FU_MUL , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  MULH    -> List(Y, N, X, uopMULH , IQT_INT, FU_MUL , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  MULHU   -> List(Y, N, X, uopMULHU, IQT_INT, FU_MUL , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  MULHSU  -> List(Y, N, X, uopMULHSU,IQT_INT, FU_MUL , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  MULW    -> List(Y, N, X, uopMULW , IQT_INT, FU_MUL , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),

  DIV     -> List(Y, N, X, uopDIV  , IQT_INT, FU_DIV , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  DIVU    -> List(Y, N, X, uopDIVU , IQT_INT, FU_DIV , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  REM     -> List(Y, N, X, uopREM  , IQT_INT, FU_DIV , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  REMU    -> List(Y, N, X, uopREMU , IQT_INT, FU_DIV , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  DIVW    -> List(Y, N, X, uopDIVW , IQT_INT, FU_DIV , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  DIVUW   -> List(Y, N, X, uopDIVUW, IQT_INT, FU_DIV , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  REMW    -> List(Y, N, X, uopREMW , IQT_INT, FU_DIV , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  REMUW   -> List(Y, N, X, uopREMUW, IQT_INT, FU_DIV , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),

  AUIPC   -> List(Y, N, X, uopAUIPC, IQT_INT, FU_JMP , RT_FIX, RT_X  , RT_X  , N, IS_U, N, N, N, N, N, M_X  , U_1, N, N, N, N, N, CSR.N, N, N, N, DC2), // use BRU for the PC read
  JAL     -> List(Y, N, X, uopJAL  , IQT_INT, FU_JMP , RT_FIX, RT_X  , RT_X  , N, IS_J, N, N, N, N, N, M_X  , U_1, N, N, N, N, N, CSR.N, N, N, N, DC2),
  JALR    -> List(Y, N, X, uopJALR , IQT_INT, FU_JMP , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_1, N, N, N, N, N, CSR.N, N, N, N, DC2),
  BEQ     -> List(Y, N, X, uopBEQ  , IQT_INT, FU_ALU , RT_X  , RT_FIX, RT_FIX, N, IS_B, N, N, N, N, N, M_X  , U_0, N, Y, N, N, N, CSR.N, N, N, N, DC2),
  BNE     -> List(Y, N, X, uopBNE  , IQT_INT, FU_ALU , RT_X  , RT_FIX, RT_FIX, N, IS_B, N, N, N, N, N, M_X  , U_0, N, Y, N, N, N, CSR.N, N, N, N, DC2),
  BGE     -> List(Y, N, X, uopBGE  , IQT_INT, FU_ALU , RT_X  , RT_FIX, RT_FIX, N, IS_B, N, N, N, N, N, M_X  , U_0, N, Y, N, N, N, CSR.N, N, N, N, DC2),
  BGEU    -> List(Y, N, X, uopBGEU , IQT_INT, FU_ALU , RT_X  , RT_FIX, RT_FIX, N, IS_B, N, N, N, N, N, M_X  , U_0, N, Y, N, N, N, CSR.N, N, N, N, DC2),
  BLT     -> List(Y, N, X, uopBLT  , IQT_INT, FU_ALU , RT_X  , RT_FIX, RT_FIX, N, IS_B, N, N, N, N, N, M_X  , U_0, N, Y, N, N, N, CSR.N, N, N, N, DC2),
  BLTU    -> List(Y, N, X, uopBLTU , IQT_INT, FU_ALU , RT_X  , RT_FIX, RT_FIX, N, IS_B, N, N, N, N, N, M_X  , U_0, N, Y, N, N, N, CSR.N, N, N, N, DC2),

  // I-type, the immediate12 holds the CSR register.
  CSRRW   -> List(Y, N, X, uopCSRRW, IQT_INT, FU_CSR , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, Y, Y, CSR.W, N, N, N, DC2),
  CSRRS   -> List(Y, N, X, uopCSRRS, IQT_INT, FU_CSR , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, Y, Y, CSR.S, N, N, N, DC2),
  CSRRC   -> List(Y, N, X, uopCSRRC, IQT_INT, FU_CSR , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, Y, Y, CSR.C, N, N, N, DC2),

  CSRRWI  -> List(Y, N, X, uopCSRRWI,IQT_INT, FU_CSR , RT_FIX, RT_X  , RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, Y, Y, CSR.W, N, N, N, DC2),
  CSRRSI  -> List(Y, N, X, uopCSRRSI,IQT_INT, FU_CSR , RT_FIX, RT_X  , RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, Y, Y, CSR.S, N, N, N, DC2),
  CSRRCI  -> List(Y, N, X, uopCSRRCI,IQT_INT, FU_CSR , RT_FIX, RT_X  , RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, Y, Y, CSR.C, N, N, N, DC2),

  SFENCE_VMA->List(Y,N, X, uopSFENCE,IQT_MEM, FU_MEM , RT_X  , RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N,M_SFENCE,U_0,N, N, N, Y, Y, CSR.N, N, N, N, DC2),
  SCALL   -> List(Y, N, X, uopERET  ,IQT_INT, FU_CSR , RT_X  , RT_X  , RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, Y, Y, Y, CSR.I, N, N, N, DC2),
  SBREAK  -> List(Y, N, X, uopERET  ,IQT_INT, FU_CSR , RT_X  , RT_X  , RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, Y, Y, Y, CSR.I, N, N, N, DC2),
  SRET    -> List(Y, N, X, uopERET  ,IQT_INT, FU_CSR , RT_X  , RT_X  , RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, Y, Y, CSR.I, N, N, N, DC2),
  MRET    -> List(Y, N, X, uopERET  ,IQT_INT, FU_CSR , RT_X  , RT_X  , RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, Y, Y, CSR.I, N, N, N, DC2),
  DRET    -> List(Y, N, X, uopERET  ,IQT_INT, FU_CSR , RT_X  , RT_X  , RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, Y, Y, CSR.I, N, N, N, DC2),

  WFI     -> List(Y, N, X, uopWFI   ,IQT_INT, FU_CSR , RT_X  , RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, Y, Y, CSR.I, N, N, N, DC2),

  FENCE_I -> List(Y, N, X, uopNOP  , IQT_INT, FU_X   , RT_X  , RT_X  , RT_X  , N, IS_X, N, N, N, N, Y, M_X  , U_0, N, N, N, Y, Y, CSR.N, N, N, N, DC2),
  FENCE   -> List(Y, N, X, uopFENCE, IQT_INT, FU_MEM , RT_X  , RT_X  , RT_X  , N, IS_X, N, Y, N, Y, N, M_X  , U_0, N, N, N, Y, Y, CSR.N, N, N, N, DC2), // TODO PERF make fence higher performance
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
  AMOADD_W-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_ADD, U_0,N, N, N, Y, Y, CSR.N, N, N, N, DC2), // TODO make AMOs higherperformance
  AMOXOR_W-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_XOR, U_0,N, N, N, Y, Y, CSR.N, N, N, N, DC2),
  AMOSWAP_W->List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_SWAP,U_0,N, N, N, Y, Y, CSR.N, N, N, N, DC2),
  AMOAND_W-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_AND, U_0,N, N, N, Y, Y, CSR.N, N, N, N, DC2),
  AMOOR_W -> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_OR,  U_0,N, N, N, Y, Y, CSR.N, N, N, N, DC2),
  AMOMIN_W-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_MIN, U_0,N, N, N, Y, Y, CSR.N, N, N, N, DC2),
  AMOMINU_W->List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_MINU,U_0,N, N, N, Y, Y, CSR.N, N, N, N, DC2),
  AMOMAX_W-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_MAX, U_0,N, N, N, Y, Y, CSR.N, N, N, N, DC2),
  AMOMAXU_W->List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_MAXU,U_0,N, N, N, Y, Y, CSR.N, N, N, N, DC2),

  AMOADD_D-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_ADD, U_0,N, N, N, Y, Y, CSR.N, N, N, N, DC2),
  AMOXOR_D-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_XOR, U_0,N, N, N, Y, Y, CSR.N, N, N, N, DC2),
  AMOSWAP_D->List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_SWAP,U_0,N, N, N, Y, Y, CSR.N, N, N, N, DC2),
  AMOAND_D-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_AND, U_0,N, N, N, Y, Y, CSR.N, N, N, N, DC2),
  AMOOR_D -> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_OR,  U_0,N, N, N, Y, Y, CSR.N, N, N, N, DC2),
  AMOMIN_D-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_MIN, U_0,N, N, N, Y, Y, CSR.N, N, N, N, DC2),
  AMOMINU_D->List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_MINU,U_0,N, N, N, Y, Y, CSR.N, N, N, N, DC2),
  AMOMAX_D-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_MAX, U_0,N, N, N, Y, Y, CSR.N, N, N, N, DC2),
  AMOMAXU_D->List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_MAXU,U_0,N, N, N, Y, Y, CSR.N, N, N, N, DC2),

  LR_W    -> List(Y, N, X, uopLD    , IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_X  , N, IS_X, Y, N, N, N, N, M_XLR   , U_0,N, N, N, Y, Y, CSR.N, N, N, N, DC2),
  LR_D    -> List(Y, N, X, uopLD    , IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_X  , N, IS_X, Y, N, N, N, N, M_XLR   , U_0,N, N, N, Y, Y, CSR.N, N, N, N, DC2),
  SC_W    -> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XSC   , U_0,N, N, N, Y, Y, CSR.N, N, N, N, DC2),
  SC_D    -> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XSC   , U_0,N, N, N, Y, Y, CSR.N, N, N, N, DC2)
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
  FLW     -> List(Y, Y, Y, uopLD     , IQT_MEM, FU_MEM, RT_FLT, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FLD     -> List(Y, Y, N, uopLD     , IQT_MEM, FU_MEM, RT_FLT, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FSW     -> List(Y, Y, Y, uopSTA    , IQT_MFP,FU_F2IMEM,RT_X , RT_FIX, RT_FLT, N, IS_S, N, Y, N, N, N, M_XWR, U_0, N, N, N, N, N, CSR.N, N, N, N, DC2), // sort of a lie; broken into two micro-ops
  FSD     -> List(Y, Y, N, uopSTA    , IQT_MFP,FU_F2IMEM,RT_X , RT_FIX, RT_FLT, N, IS_S, N, Y, N, N, N, M_XWR, U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),

  FCLASS_S-> List(Y, Y, Y, uopFCLASS_S,IQT_FP , FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FCLASS_D-> List(Y, Y, N, uopFCLASS_D,IQT_FP , FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),

  FMV_S_X -> List(Y, Y, Y, uopFMV_S_X, IQT_INT, FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FMV_D_X -> List(Y, Y, N, uopFMV_D_X, IQT_INT, FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FMV_X_S -> List(Y, Y, Y, uopFMV_X_S, IQT_FP , FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FMV_X_D -> List(Y, Y, N, uopFMV_X_D, IQT_FP , FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),

  FSGNJ_S -> List(Y, Y, Y, uopFSGNJ_S, IQT_FP , FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FSGNJ_D -> List(Y, Y, N, uopFSGNJ_D, IQT_FP , FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FSGNJX_S-> List(Y, Y, Y, uopFSGNJ_S, IQT_FP , FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FSGNJX_D-> List(Y, Y, N, uopFSGNJ_D, IQT_FP , FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FSGNJN_S-> List(Y, Y, Y, uopFSGNJ_S, IQT_FP , FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FSGNJN_D-> List(Y, Y, N, uopFSGNJ_D, IQT_FP , FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),

  // FP to FP
  FCVT_S_D-> List(Y, Y, Y, uopFCVT_S_D,IQT_FP , FU_FPU, RT_FLT, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FCVT_D_S-> List(Y, Y, N, uopFCVT_D_S,IQT_FP , FU_FPU, RT_FLT, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),

  // Int to FP
  FCVT_S_W-> List(Y, Y, Y, uopFCVT_S_X, IQT_INT,FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FCVT_S_WU->List(Y, Y, Y, uopFCVT_S_X, IQT_INT,FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FCVT_S_L-> List(Y, Y, Y, uopFCVT_S_X, IQT_INT,FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FCVT_S_LU->List(Y, Y, Y, uopFCVT_S_X, IQT_INT,FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),

  FCVT_D_W-> List(Y, Y, N, uopFCVT_D_X, IQT_INT,FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FCVT_D_WU->List(Y, Y, N, uopFCVT_D_X, IQT_INT,FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FCVT_D_L-> List(Y, Y, N, uopFCVT_D_X, IQT_INT,FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FCVT_D_LU->List(Y, Y, N, uopFCVT_D_X, IQT_INT,FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),

  // FP to Int
  FCVT_W_S-> List(Y, Y, Y, uopFCVT_X_S, IQT_FP, FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FCVT_WU_S->List(Y, Y, Y, uopFCVT_X_S, IQT_FP, FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FCVT_L_S-> List(Y, Y, Y, uopFCVT_X_S, IQT_FP, FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FCVT_LU_S->List(Y, Y, Y, uopFCVT_X_S, IQT_FP, FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),

  FCVT_W_D-> List(Y, Y, N, uopFCVT_X_D, IQT_FP, FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FCVT_WU_D->List(Y, Y, N, uopFCVT_X_D, IQT_FP, FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FCVT_L_D-> List(Y, Y, N, uopFCVT_X_D, IQT_FP, FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FCVT_LU_D->List(Y, Y, N, uopFCVT_X_D, IQT_FP, FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),

  // "fp_single" is used for wb_data formatting (and debugging)
  FEQ_S    ->List(Y, Y, Y, uopCMPR_S , IQT_FP,  FU_F2I, RT_FIX, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FLT_S    ->List(Y, Y, Y, uopCMPR_S , IQT_FP,  FU_F2I, RT_FIX, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FLE_S    ->List(Y, Y, Y, uopCMPR_S , IQT_FP,  FU_F2I, RT_FIX, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),

  FEQ_D    ->List(Y, Y, N, uopCMPR_D , IQT_FP,  FU_F2I, RT_FIX, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FLT_D    ->List(Y, Y, N, uopCMPR_D , IQT_FP,  FU_F2I, RT_FIX, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FLE_D    ->List(Y, Y, N, uopCMPR_D , IQT_FP,  FU_F2I, RT_FIX, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),

  FMIN_S   ->List(Y, Y, Y,uopFMINMAX_S,IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FMAX_S   ->List(Y, Y, Y,uopFMINMAX_S,IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FMIN_D   ->List(Y, Y, N,uopFMINMAX_D,IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FMAX_D   ->List(Y, Y, N,uopFMINMAX_D,IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),

  FADD_S   ->List(Y, Y, Y, uopFADD_S , IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FSUB_S   ->List(Y, Y, Y, uopFSUB_S , IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FMUL_S   ->List(Y, Y, Y, uopFMUL_S , IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FADD_D   ->List(Y, Y, N, uopFADD_D , IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FSUB_D   ->List(Y, Y, N, uopFSUB_D , IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FMUL_D   ->List(Y, Y, N, uopFMUL_D , IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),

  FMADD_S  ->List(Y, Y, Y, uopFMADD_S, IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FMSUB_S  ->List(Y, Y, Y, uopFMSUB_S, IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FNMADD_S ->List(Y, Y, Y, uopFNMADD_S,IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FNMSUB_S ->List(Y, Y, Y, uopFNMSUB_S,IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FMADD_D  ->List(Y, Y, N, uopFMADD_D, IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FMSUB_D  ->List(Y, Y, N, uopFMSUB_D, IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FNMADD_D ->List(Y, Y, N, uopFNMADD_D,IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FNMSUB_D ->List(Y, Y, N, uopFNMSUB_D,IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2)
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
  FDIV_S    ->List(Y, Y, Y, uopFDIV_S , IQT_FP, FU_FDV, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FDIV_D    ->List(Y, Y, N, uopFDIV_D , IQT_FP, FU_FDV, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FSQRT_S   ->List(Y, Y, Y, uopFSQRT_S, IQT_FP, FU_FDV, RT_FLT, RT_FLT, RT_X  , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
  FSQRT_D   ->List(Y, Y, N, uopFSQRT_D, IQT_FP, FU_FDV, RT_FLT, RT_FLT, RT_X  , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2)
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
  VSETVLI     ->List(Y, N, X, uopVSETVLI,     IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, Y, Y, CSR.W, Y, N, X, DC2)  // TODO optimize us
 ,VSETIVLI    ->List(Y, N, X, uopVSETIVLI,    IQT_INT, FU_CSR, RT_FIX, RT_X  , RT_X  , N, IS_I, N, N, N, N, N, M_X  , U_0, N, N, N, Y, Y, CSR.W, Y, N, X, DC2)
 ,VSETVL      ->List(Y, N, X, uopVSETVL,      IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, Y, Y, CSR.W, Y, N, X, DC2)
 )
}

object VectorLSDecode extends DecodeConstants
{
  val table: Array[(BitPat, List[BitPat])] = Array(
  VLE8_V      ->List(Y, N, X, uopVL,          IQT_MEM ,FU_MEM, RT_VEC, RT_FIX, RT_X  , N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_0)
 ,VLE16_V     ->List(Y, N, X, uopVL,          IQT_MEM ,FU_MEM, RT_VEC, RT_FIX, RT_X  , N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_1)
 ,VLE32_V     ->List(Y, N, X, uopVL,          IQT_MEM ,FU_MEM, RT_VEC, RT_FIX, RT_X  , N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_2)
 ,VLE64_V     ->List(Y, N, X, uopVL,          IQT_MEM ,FU_MEM, RT_VEC, RT_FIX, RT_X  , N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_3)
 ,VLE8FF_V    ->List(Y, N, X, uopVLFF,        IQT_MEM ,FU_MEM, RT_VEC, RT_FIX, RT_X  , N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_0)
 ,VLE16FF_V   ->List(Y, N, X, uopVLFF,        IQT_MEM ,FU_MEM, RT_VEC, RT_FIX, RT_X  , N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_1)
 ,VLE32FF_V   ->List(Y, N, X, uopVLFF,        IQT_MEM ,FU_MEM, RT_VEC, RT_FIX, RT_X  , N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_2)
 ,VLE64FF_V   ->List(Y, N, X, uopVLFF,        IQT_MEM ,FU_MEM, RT_VEC, RT_FIX, RT_X  , N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_3)
 ,VSE8_V      ->List(Y, N, X, uopVSA,         IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_X  , Y, IS_X, N, Y, N, N, N, M_XWR, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_0)
 ,VSE16_V     ->List(Y, N, X, uopVSA,         IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_X  , Y, IS_X, N, Y, N, N, N, M_XWR, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_1)
 ,VSE32_V     ->List(Y, N, X, uopVSA,         IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_X  , Y, IS_X, N, Y, N, N, N, M_XWR, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_2)
 ,VSE64_V     ->List(Y, N, X, uopVSA,         IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_X  , Y, IS_X, N, Y, N, N, N, M_XWR, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_3)
 ,VLM_V       ->List(Y, N, X, uopVLM,         IQT_MEM, FU_MEM, RT_VEC, RT_FIX, RT_X,   N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, N, U_0)
 ,VSM_V       ->List(Y, N, X, uopVSMA,        IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_X  , Y, IS_X, N, Y, N, N, N, M_XWR, DC2, N, N, N, N, N, CSR.N, Y, Y, N, U_0)
 ,VLSE8_V     ->List(Y, N, X, uopVLS,         IQT_MEM ,FU_MEM, RT_VEC, RT_FIX, RT_FIX, N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_0)
 ,VLSE16_V    ->List(Y, N, X, uopVLS,         IQT_MEM ,FU_MEM, RT_VEC, RT_FIX, RT_FIX, N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_1)
 ,VLSE32_V    ->List(Y, N, X, uopVLS,         IQT_MEM ,FU_MEM, RT_VEC, RT_FIX, RT_FIX, N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_2)
 ,VLSE64_V    ->List(Y, N, X, uopVLS,         IQT_MEM ,FU_MEM, RT_VEC, RT_FIX, RT_FIX, N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_3)
 ,VSSE8_V     ->List(Y, N, X, uopVSSA,        IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_FIX, Y, IS_X, N, Y, N, N, N, M_XWR, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_0)
 ,VSSE16_V    ->List(Y, N, X, uopVSSA,        IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_FIX, Y, IS_X, N, Y, N, N, N, M_XWR, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_1)
 ,VSSE32_V    ->List(Y, N, X, uopVSSA,        IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_FIX, Y, IS_X, N, Y, N, N, N, M_XWR, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_2)
 ,VSSE64_V    ->List(Y, N, X, uopVSSA,        IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_FIX, Y, IS_X, N, Y, N, N, N, M_XWR, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_3)
 ,VLUXEI8_V   ->List(Y, N, X, uopVLUX,        IQT_MVEC,FU_MEMV,RT_VEC, RT_FIX, RT_VU , N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_0)
 ,VLUXEI16_V  ->List(Y, N, X, uopVLUX,        IQT_MVEC,FU_MEMV,RT_VEC, RT_FIX, RT_VU , N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_1)
 ,VLUXEI32_V  ->List(Y, N, X, uopVLUX,        IQT_MVEC,FU_MEMV,RT_VEC, RT_FIX, RT_VU , N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_2)
 ,VLUXEI64_V  ->List(Y, N, X, uopVLUX,        IQT_MVEC,FU_MEMV,RT_VEC, RT_FIX, RT_VU , N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_3)
 ,VSUXEI8_V   ->List(Y, N, X, uopVSUXA,       IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_VU , Y, IS_X, N, Y, N, N, N, M_XWR, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_0)
 ,VSUXEI16_V  ->List(Y, N, X, uopVSUXA,       IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_VU , Y, IS_X, N, Y, N, N, N, M_XWR, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_1)
 ,VSUXEI32_V  ->List(Y, N, X, uopVSUXA,       IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_VU , Y, IS_X, N, Y, N, N, N, M_XWR, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_2)
 ,VSUXEI64_V  ->List(Y, N, X, uopVSUXA,       IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_VU , Y, IS_X, N, Y, N, N, N, M_XWR, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_3)
 ,VLOXEI8_V   ->List(Y, N, X, uopVLOX,        IQT_MVEC,FU_MEMV,RT_VEC, RT_FIX, RT_VU , N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_0)
 ,VLOXEI16_V  ->List(Y, N, X, uopVLOX,        IQT_MVEC,FU_MEMV,RT_VEC, RT_FIX, RT_VU , N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_1)
 ,VLOXEI32_V  ->List(Y, N, X, uopVLOX,        IQT_MVEC,FU_MEMV,RT_VEC, RT_FIX, RT_VU , N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_2)
 ,VLOXEI64_V  ->List(Y, N, X, uopVLOX,        IQT_MVEC,FU_MEMV,RT_VEC, RT_FIX, RT_VU , N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_3)
 ,VSOXEI8_V   ->List(Y, N, X, uopVSOXA,       IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_VU , Y, IS_X, N, Y, N, N, N, M_XWR, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_0)
 ,VSOXEI16_V  ->List(Y, N, X, uopVSOXA,       IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_VU , Y, IS_X, N, Y, N, N, N, M_XWR, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_1)
 ,VSOXEI32_V  ->List(Y, N, X, uopVSOXA,       IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_VU , Y, IS_X, N, Y, N, N, N, M_XWR, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_2)
 ,VSOXEI64_V  ->List(Y, N, X, uopVSOXA,       IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_VU , Y, IS_X, N, Y, N, N, N, M_XWR, DC2, N, N, N, N, N, CSR.N, Y, Y, Y, U_3)
 ,VL1RE8_V    ->List(Y, N, X, uopVLR,         IQT_MEM, FU_MEM, RT_VEC, RT_FIX, RT_X,   N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, N, U_0)
 ,VL1RE16_V   ->List(Y, N, X, uopVLR,         IQT_MEM, FU_MEM, RT_VEC, RT_FIX, RT_X,   N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, N, U_1)
 ,VL1RE32_V   ->List(Y, N, X, uopVLR,         IQT_MEM, FU_MEM, RT_VEC, RT_FIX, RT_X,   N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, N, U_2)
 ,VL1RE64_V   ->List(Y, N, X, uopVLR,         IQT_MEM, FU_MEM, RT_VEC, RT_FIX, RT_X,   N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, N, U_3)
 ,VL2RE8_V    ->List(Y, N, X, uopVLR,         IQT_MEM, FU_MEM, RT_VEC, RT_FIX, RT_X,   N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, N, U_0)
 ,VL2RE16_V   ->List(Y, N, X, uopVLR,         IQT_MEM, FU_MEM, RT_VEC, RT_FIX, RT_X,   N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, N, U_1)
 ,VL2RE32_V   ->List(Y, N, X, uopVLR,         IQT_MEM, FU_MEM, RT_VEC, RT_FIX, RT_X,   N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, N, U_2)
 ,VL2RE64_V   ->List(Y, N, X, uopVLR,         IQT_MEM, FU_MEM, RT_VEC, RT_FIX, RT_X,   N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, N, U_3)
 ,VL4RE8_V    ->List(Y, N, X, uopVLR,         IQT_MEM, FU_MEM, RT_VEC, RT_FIX, RT_X,   N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, N, U_0)
 ,VL4RE16_V   ->List(Y, N, X, uopVLR,         IQT_MEM, FU_MEM, RT_VEC, RT_FIX, RT_X,   N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, N, U_1)
 ,VL4RE32_V   ->List(Y, N, X, uopVLR,         IQT_MEM, FU_MEM, RT_VEC, RT_FIX, RT_X,   N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, N, U_2)
 ,VL4RE64_V   ->List(Y, N, X, uopVLR,         IQT_MEM, FU_MEM, RT_VEC, RT_FIX, RT_X,   N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, N, U_3)
 ,VL8RE8_V    ->List(Y, N, X, uopVLR,         IQT_MEM, FU_MEM, RT_VEC, RT_FIX, RT_X,   N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, N, U_0)
 ,VL8RE16_V   ->List(Y, N, X, uopVLR,         IQT_MEM, FU_MEM, RT_VEC, RT_FIX, RT_X,   N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, N, U_1)
 ,VL8RE32_V   ->List(Y, N, X, uopVLR,         IQT_MEM, FU_MEM, RT_VEC, RT_FIX, RT_X,   N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, N, U_2)
 ,VL8RE64_V   ->List(Y, N, X, uopVLR,         IQT_MEM, FU_MEM, RT_VEC, RT_FIX, RT_X,   N, IS_X, Y, N, N, N, N, M_XRD, DC2, N, N, N, N, N, CSR.N, Y, Y, N, U_3)
 ,VS1R_V      ->List(Y, N, X, uopVSR,         IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_X  , Y, IS_X, N, Y, N, N, N, M_XWR, DC2, N, N, N, N, N, CSR.N, Y, Y, N, U_0)
 ,VS2R_V      ->List(Y, N, X, uopVSR,         IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_X  , Y, IS_X, N, Y, N, N, N, M_XWR, DC2, N, N, N, N, N, CSR.N, Y, Y, N, U_0)
 ,VS4R_V      ->List(Y, N, X, uopVSR,         IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_X  , Y, IS_X, N, Y, N, N, N, M_XWR, DC2, N, N, N, N, N, CSR.N, Y, Y, N, U_0)
 ,VS8R_V      ->List(Y, N, X, uopVSR,         IQT_MVEC,FU_MEMV,RT_X  , RT_FIX, RT_X  , Y, IS_X, N, Y, N, N, N, M_XWR, DC2, N, N, N, N, N, CSR.N, Y, Y, N, U_0)
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
  VADD_VV     ->List(Y, N, X, uopVADD,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VADD_VX     ->List(Y, N, X, uopVADD,        IQT_IVEC,FU_ALU ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VADD_VI     ->List(Y, N, X, uopVADD,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VI , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VSUB_VV     ->List(Y, N, X, uopVSUB,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VSUB_VX     ->List(Y, N, X, uopVSUB,        IQT_IVEC,FU_ALU ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VRSUB_VX    ->List(Y, N, X, uopVRSUB,       IQT_IVEC,FU_ALU ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VRSUB_VI    ->List(Y, N, X, uopVRSUB,       IQT_VEC ,FU_ALU ,RT_VEC, RT_VI , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VWADDU_VV   ->List(Y, N, X, uopVADD,        IQT_VEC ,FU_ALU ,RT_VW , RT_VU , RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VWADDU_VX   ->List(Y, N, X, uopVADD,        IQT_IVEC,FU_ALU ,RT_VW , RT_FIXU,RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VWSUBU_VV   ->List(Y, N, X, uopVSUB,        IQT_VEC ,FU_ALU ,RT_VW , RT_VU , RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VWSUBU_VX   ->List(Y, N, X, uopVSUB,        IQT_IVEC,FU_ALU ,RT_VW , RT_FIXU,RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VWADD_VV    ->List(Y, N, X, uopVADD,        IQT_VEC ,FU_ALU ,RT_VW , RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VWADD_VX    ->List(Y, N, X, uopVADD,        IQT_IVEC,FU_ALU ,RT_VW , RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VWSUB_VV    ->List(Y, N, X, uopVSUB,        IQT_VEC ,FU_ALU ,RT_VW , RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VWSUB_VX    ->List(Y, N, X, uopVSUB,        IQT_IVEC,FU_ALU ,RT_VW , RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VWADDU_WV   ->List(Y, N, X, uopVADD,        IQT_VEC ,FU_ALU ,RT_VW , RT_VU , RT_VWU, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VWADDU_WX   ->List(Y, N, X, uopVADD,        IQT_IVEC,FU_ALU ,RT_VW , RT_FIXU,RT_VWU, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VWSUBU_WV   ->List(Y, N, X, uopVSUB,        IQT_VEC ,FU_ALU ,RT_VW , RT_VU , RT_VWU, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VWSUBU_WX   ->List(Y, N, X, uopVSUB,        IQT_IVEC,FU_ALU ,RT_VW , RT_FIXU,RT_VWU, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VWADD_WV    ->List(Y, N, X, uopVADD,        IQT_VEC ,FU_ALU ,RT_VW , RT_VEC, RT_VW , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VWADD_WX    ->List(Y, N, X, uopVADD,        IQT_IVEC,FU_ALU ,RT_VW , RT_FIX, RT_VW , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VWSUB_WV    ->List(Y, N, X, uopVSUB,        IQT_VEC ,FU_ALU ,RT_VW , RT_VEC, RT_VW , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VWSUB_WX    ->List(Y, N, X, uopVSUB,        IQT_IVEC,FU_ALU ,RT_VW , RT_FIX, RT_VW , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VZEXT_VF2   ->List(Y, N, X, uopVEXT2,       IQT_VEC ,FU_ALU ,RT_VW , RT_X  , RT_VNU, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VSEXT_VF2   ->List(Y, N, X, uopVEXT2,       IQT_VEC ,FU_ALU ,RT_VW , RT_X  , RT_VN , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VZEXT_VF4   ->List(Y, N, X, uopVEXT4,       IQT_VEC ,FU_ALU ,RT_VW , RT_X  , RT_VNU, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VSEXT_VF4   ->List(Y, N, X, uopVEXT4,       IQT_VEC ,FU_ALU ,RT_VW , RT_X  , RT_VN , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VZEXT_VF8   ->List(Y, N, X, uopVEXT8,       IQT_VEC ,FU_ALU ,RT_VW , RT_X  , RT_VNU, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VSEXT_VF8   ->List(Y, N, X, uopVEXT8,       IQT_VEC ,FU_ALU ,RT_VW , RT_X  , RT_VN , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VADC_VVM    ->List(Y, N, X, uopVADC,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VU , RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VADC_VXM    ->List(Y, N, X, uopVADC,        IQT_IVEC,FU_ALU ,RT_VEC, RT_FIXU,RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VADC_VIM    ->List(Y, N, X, uopVADC,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VIU, RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMADC_VVM   ->List(Y, N, X, uopVMADC,       IQT_VEC ,FU_ALU ,RT_VM , RT_VU , RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMADC_VXM   ->List(Y, N, X, uopVMADC,       IQT_IVEC,FU_ALU ,RT_VM , RT_FIXU,RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMADC_VIM   ->List(Y, N, X, uopVMADC,       IQT_VEC ,FU_ALU ,RT_VM , RT_VIU, RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VSBC_VVM    ->List(Y, N, X, uopVSBC,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VU , RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VSBC_VXM    ->List(Y, N, X, uopVSBC,        IQT_IVEC,FU_ALU ,RT_VEC, RT_FIXU,RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMSBC_VVM   ->List(Y, N, X, uopVMSBC,       IQT_VEC ,FU_ALU ,RT_VM , RT_VU , RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMSBC_VXM   ->List(Y, N, X, uopVMSBC,       IQT_IVEC,FU_ALU ,RT_VM , RT_FIXU,RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VAND_VV     ->List(Y, N, X, uopVAND,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VAND_VX     ->List(Y, N, X, uopVAND,        IQT_IVEC,FU_ALU ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VAND_VI     ->List(Y, N, X, uopVAND,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VI , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VOR_VV      ->List(Y, N, X, uopVOR,         IQT_VEC ,FU_ALU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VOR_VX      ->List(Y, N, X, uopVOR,         IQT_IVEC,FU_ALU ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VOR_VI      ->List(Y, N, X, uopVOR,         IQT_VEC ,FU_ALU ,RT_VEC, RT_VI , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VXOR_VV     ->List(Y, N, X, uopVXOR,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VXOR_VX     ->List(Y, N, X, uopVXOR,        IQT_IVEC,FU_ALU ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VXOR_VI     ->List(Y, N, X, uopVXOR,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VI , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VSLL_VV     ->List(Y, N, X, uopVSLL,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VU , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VSLL_VX     ->List(Y, N, X, uopVSLL,        IQT_IVEC,FU_ALU ,RT_VEC, RT_FIXU,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VSLL_VI     ->List(Y, N, X, uopVSLL,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VIU, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VSRL_VV     ->List(Y, N, X, uopVSRL,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VU , RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VSRL_VX     ->List(Y, N, X, uopVSRL,        IQT_IVEC,FU_ALU ,RT_VEC, RT_FIXU,RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VSRL_VI     ->List(Y, N, X, uopVSRL,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VIU, RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VSRA_VV     ->List(Y, N, X, uopVSRA,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VU , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VSRA_VX     ->List(Y, N, X, uopVSRA,        IQT_IVEC,FU_ALU ,RT_VEC, RT_FIXU,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VSRA_VI     ->List(Y, N, X, uopVSRA,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VIU, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VNSRL_WV    ->List(Y, N, X, uopVSRL,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VU , RT_VWU, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VNSRL_WX    ->List(Y, N, X, uopVSRL,        IQT_IVEC,FU_ALU ,RT_VEC, RT_FIXU,RT_VWU, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VNSRL_WI    ->List(Y, N, X, uopVSRL,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VIU, RT_VWU, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VNSRA_WV    ->List(Y, N, X, uopVSRA,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VU , RT_VW , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VNSRA_WX    ->List(Y, N, X, uopVSRA,        IQT_IVEC,FU_ALU ,RT_VEC, RT_FIXU,RT_VW , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VNSRA_WI    ->List(Y, N, X, uopVSRA,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VIU, RT_VW , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMSEQ_VV    ->List(Y, N, X, uopVMSEQ,       IQT_VEC ,FU_ALU ,RT_VM , RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMSEQ_VX    ->List(Y, N, X, uopVMSEQ,       IQT_IVEC,FU_ALU ,RT_VM , RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMSEQ_VI    ->List(Y, N, X, uopVMSEQ,       IQT_VEC ,FU_ALU ,RT_VM , RT_VI , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMSNE_VV    ->List(Y, N, X, uopVMSNE,       IQT_VEC ,FU_ALU ,RT_VM , RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMSNE_VX    ->List(Y, N, X, uopVMSNE,       IQT_IVEC,FU_ALU ,RT_VM , RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMSNE_VI    ->List(Y, N, X, uopVMSNE,       IQT_VEC ,FU_ALU ,RT_VM , RT_VI , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMSLTU_VV   ->List(Y, N, X, uopVMSLTU,      IQT_VEC ,FU_ALU ,RT_VM , RT_VU , RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMSLTU_VX   ->List(Y, N, X, uopVMSLTU,      IQT_IVEC,FU_ALU ,RT_VM , RT_FIXU,RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMSLT_VV    ->List(Y, N, X, uopVMSLT,       IQT_VEC ,FU_ALU ,RT_VM , RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMSLT_VX    ->List(Y, N, X, uopVMSLT,       IQT_IVEC,FU_ALU ,RT_VM , RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMSLEU_VV   ->List(Y, N, X, uopVMSLEU,      IQT_VEC ,FU_ALU ,RT_VM , RT_VU , RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMSLEU_VX   ->List(Y, N, X, uopVMSLEU,      IQT_IVEC,FU_ALU ,RT_VM , RT_FIXU,RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMSLEU_VI   ->List(Y, N, X, uopVMSLEU,      IQT_VEC ,FU_ALU ,RT_VM , RT_VIU, RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMSLE_VV    ->List(Y, N, X, uopVMSLE,       IQT_VEC ,FU_ALU ,RT_VM , RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMSLE_VX    ->List(Y, N, X, uopVMSLE,       IQT_IVEC,FU_ALU ,RT_VM , RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMSLE_VI    ->List(Y, N, X, uopVMSLE,       IQT_VEC ,FU_ALU ,RT_VM , RT_VI , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMSGTU_VX   ->List(Y, N, X, uopVMSGTU,      IQT_IVEC,FU_ALU ,RT_VM , RT_FIXU,RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMSGTU_VI   ->List(Y, N, X, uopVMSGTU,      IQT_VEC ,FU_ALU ,RT_VM , RT_VIU, RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMSGT_VX    ->List(Y, N, X, uopVMSGT,       IQT_IVEC,FU_ALU ,RT_VM , RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMSGT_VI    ->List(Y, N, X, uopVMSGT,       IQT_VEC ,FU_ALU ,RT_VM , RT_VI,  RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMINU_VV    ->List(Y, N, X, uopVMINU,       IQT_VEC ,FU_ALU ,RT_VEC, RT_VU , RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMINU_VX    ->List(Y, N, X, uopVMINU,       IQT_IVEC,FU_ALU ,RT_VEC, RT_FIXU,RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMIN_VV     ->List(Y, N, X, uopVMIN,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMIN_VX     ->List(Y, N, X, uopVMIN,        IQT_IVEC,FU_ALU ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMAXU_VV    ->List(Y, N, X, uopVMAXU,       IQT_VEC ,FU_ALU ,RT_VEC, RT_VU , RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMAXU_VX    ->List(Y, N, X, uopVMAXU,       IQT_IVEC,FU_ALU ,RT_VEC, RT_FIXU,RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMAX_VV     ->List(Y, N, X, uopVMAX,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMAX_VX     ->List(Y, N, X, uopVMAX,        IQT_IVEC,FU_ALU ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMUL_VV     ->List(Y, N, X, uopVMUL,        IQT_VEC ,FU_MAC ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMUL_VX     ->List(Y, N, X, uopVMUL,        IQT_IVEC,FU_MAC ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMULH_VV    ->List(Y, N, X, uopVMULH,       IQT_VEC ,FU_MAC ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMULH_VX    ->List(Y, N, X, uopVMULH,       IQT_IVEC,FU_MAC ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMULHU_VV   ->List(Y, N, X, uopVMULHU,      IQT_VEC ,FU_MAC ,RT_VEC, RT_VU , RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMULHU_VX   ->List(Y, N, X, uopVMULHU,      IQT_IVEC,FU_MAC ,RT_VEC, RT_FIXU,RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMULHSU_VV  ->List(Y, N, X, uopVMULHSU,     IQT_VEC ,FU_MAC ,RT_VEC, RT_VU , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMULHSU_VX  ->List(Y, N, X, uopVMULHSU,     IQT_IVEC,FU_MAC ,RT_VEC, RT_FIXU,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VDIVU_VV    ->List(Y, N, X, uopVDIVU,       IQT_VEC ,FU_DIV ,RT_VEC, RT_VU , RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VDIVU_VX    ->List(Y, N, X, uopVDIVU,       IQT_IVEC,FU_DIV ,RT_VEC, RT_FIXU,RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VDIV_VV     ->List(Y, N, X, uopVDIV,        IQT_VEC ,FU_DIV ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VDIV_VX     ->List(Y, N, X, uopVDIV,        IQT_IVEC,FU_DIV ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VREMU_VV    ->List(Y, N, X, uopVREMU,       IQT_VEC ,FU_DIV ,RT_VEC, RT_VU , RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VREMU_VX    ->List(Y, N, X, uopVREMU,       IQT_IVEC,FU_DIV ,RT_VEC, RT_FIXU,RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VREM_VV     ->List(Y, N, X, uopVREM,        IQT_VEC ,FU_DIV ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VREM_VX     ->List(Y, N, X, uopVREM,        IQT_IVEC,FU_DIV ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VWMUL_VV    ->List(Y, N, X, uopVMUL,        IQT_VEC ,FU_MAC ,RT_VW , RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VWMUL_VX    ->List(Y, N, X, uopVMUL,        IQT_IVEC,FU_MAC ,RT_VW , RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VWMULU_VV   ->List(Y, N, X, uopVWMULU,      IQT_VEC ,FU_MAC ,RT_VWU, RT_VU , RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VWMULU_VX   ->List(Y, N, X, uopVWMULU,      IQT_IVEC,FU_MAC ,RT_VWU, RT_FIXU,RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VWMULSU_VV  ->List(Y, N, X, uopVWMULSU,     IQT_VEC ,FU_MAC ,RT_VW , RT_VU , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VWMULSU_VX  ->List(Y, N, X, uopVWMULSU,     IQT_IVEC,FU_MAC ,RT_VW , RT_FIXU,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMACC_VV    ->List(Y, N, X, uopVMACC,       IQT_VEC ,FU_MAC ,RT_VEC, RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMACC_VX    ->List(Y, N, X, uopVMACC,       IQT_IVEC,FU_MAC ,RT_VEC, RT_FIX, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VNMSAC_VV   ->List(Y, N, X, uopVNMSAC,      IQT_VEC ,FU_MAC ,RT_VEC, RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VNMSAC_VX   ->List(Y, N, X, uopVNMSAC,      IQT_IVEC,FU_MAC ,RT_VEC, RT_FIX, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMADD_VV    ->List(Y, N, X, uopVMADD,       IQT_VEC ,FU_MAC ,RT_VEC, RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMADD_VX    ->List(Y, N, X, uopVMADD,       IQT_IVEC,FU_MAC ,RT_VEC, RT_FIX, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VNMSUB_VV   ->List(Y, N, X, uopVNMSUB,      IQT_VEC ,FU_MAC ,RT_VEC, RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VNMSUB_VX   ->List(Y, N, X, uopVNMSUB,      IQT_IVEC,FU_MAC ,RT_VEC, RT_FIX, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VWMACCU_VV  ->List(Y, N, X, uopVWMACCU,     IQT_VEC ,FU_MAC ,RT_VWU, RT_VU , RT_VU , Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VWMACCU_VX  ->List(Y, N, X, uopVWMACCU,     IQT_IVEC,FU_MAC ,RT_VWU, RT_FIXU,RT_VU , Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VWMACC_VV   ->List(Y, N, X, uopVMACC,       IQT_VEC ,FU_MAC ,RT_VW , RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VWMACC_VX   ->List(Y, N, X, uopVMACC,       IQT_IVEC,FU_MAC ,RT_VW , RT_FIX, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VWMACCSU_VV ->List(Y, N, X, uopVWMACCSU,    IQT_VEC ,FU_MAC ,RT_VW , RT_VEC, RT_VU , Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VWMACCSU_VX ->List(Y, N, X, uopVWMACCSU,    IQT_IVEC,FU_MAC ,RT_VW , RT_FIX, RT_VU , Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VWMACCUS_VX ->List(Y, N, X, uopVWMACCUS,    IQT_IVEC,FU_MAC ,RT_VW , RT_FIXU,RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMAND_MM    ->List(Y, N, X, uopVMAND,       IQT_VEC ,FU_ALU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, N, DC2)
 ,VMNAND_MM   ->List(Y, N, X, uopVMNAND,      IQT_VEC ,FU_ALU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, N, DC2)
 ,VMANDNOT_MM ->List(Y, N, X, uopVMANDNOT,    IQT_VEC ,FU_ALU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, N, DC2)
 ,VMXOR_MM    ->List(Y, N, X, uopVMXOR,       IQT_VEC ,FU_ALU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, N, DC2)
 ,VMOR_MM     ->List(Y, N, X, uopVMOR,        IQT_VEC ,FU_ALU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, N, DC2)
 ,VMNOR_MM    ->List(Y, N, X, uopVMNOR,       IQT_VEC ,FU_ALU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, N, DC2)
 ,VMORNOT_MM  ->List(Y, N, X, uopVMORNOT,     IQT_VEC ,FU_ALU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, N, DC2)
 ,VMXNOR_MM   ->List(Y, N, X, uopVMXNOR,      IQT_VEC ,FU_ALU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, N, DC2)
 ,VPOPC_M     ->List(Y, Y, X, uopVPOPC,       IQT_VEC ,FU_VMASKU,RT_FIX, RT_X, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFIRST_M    ->List(Y, Y, X, uopVFIRST,      IQT_VEC ,FU_VMASKU,RT_FIX, RT_X, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMSBF_M     ->List(Y, Y, X, uopVMSBF,       IQT_VEC ,FU_VMASKU,RT_VEC, RT_X, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMSIF_M     ->List(Y, Y, X, uopVMSIF,       IQT_VEC ,FU_VMASKU,RT_VEC, RT_X, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMSOF_M     ->List(Y, Y, X, uopVMSOF,       IQT_VEC ,FU_VMASKU,RT_VEC, RT_X, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VIOTA_M     ->List(Y, Y, X, uopVIOTA,       IQT_VEC ,FU_VMASKU,RT_VU,  RT_X, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VID_V       ->List(Y, Y, X, uopVID,         IQT_VEC ,FU_ALU   ,RT_VU,  RT_X, RT_X,   N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMV_V_V     ->List(Y, N, X, uopVMV_V,       IQT_VEC ,FU_ALU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMV_V_X     ->List(Y, N, X, uopVMV_V,       IQT_IVEC,FU_ALU ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMV_V_I     ->List(Y, N, X, uopVMV_V,       IQT_VEC, FU_ALU ,RT_VEC, RT_VI,  RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMV_X_S     ->List(Y, N, X, uopVMV_X_S,     IQT_VEC, FU_ALU ,RT_FIX, RT_X,   RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMV_S_X     ->List(Y, N, X, uopVMV_S_X,     IQT_IVEC,FU_ALU ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMV1R_V     ->List(Y, N, X, uopVMVR,        IQT_VEC ,FU_ALU ,RT_VEC, RT_X,   RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMV2R_V     ->List(Y, N, X, uopVMVR,        IQT_VEC ,FU_ALU ,RT_VEC, RT_X,   RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMV4R_V     ->List(Y, N, X, uopVMVR,        IQT_VEC ,FU_ALU ,RT_VEC, RT_X,   RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMV8R_V     ->List(Y, N, X, uopVMVR,        IQT_VEC ,FU_ALU ,RT_VEC, RT_X,   RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMERGE_VVM  ->List(Y, N, X, uopMERGE,       IQT_VEC ,FU_ALU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMERGE_VXM  ->List(Y, N, X, uopMERGE,       IQT_IVEC,FU_ALU ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMERGE_VIM  ->List(Y, N, X, uopMERGE,       IQT_VEC ,FU_ALU ,RT_VEC, RT_VI , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 )
}

object VectorFixDecode extends DecodeConstants
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
  VSADDU_VV   ->List(Y, N, X, uopVSADDU,      IQT_VEC ,FU_MAC ,RT_VU , RT_VU , RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VSADDU_VX   ->List(Y, N, X, uopVSADDU,      IQT_IVEC,FU_MAC ,RT_VU , RT_FIXU,RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VSADDU_VI   ->List(Y, N, X, uopVSADDU,      IQT_VEC ,FU_MAC ,RT_VU , RT_VIU, RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VSADD_VV    ->List(Y, N, X, uopVSADD,       IQT_VEC ,FU_MAC ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VSADD_VX    ->List(Y, N, X, uopVSADD,       IQT_IVEC,FU_MAC ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VSADD_VI    ->List(Y, N, X, uopVSADD,       IQT_VEC ,FU_MAC ,RT_VEC, RT_VI , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VSSUBU_VV   ->List(Y, N, X, uopVSSUBU,      IQT_VEC ,FU_MAC ,RT_VU , RT_VU , RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VSSUBU_VX   ->List(Y, N, X, uopVSSUBU,      IQT_IVEC,FU_MAC ,RT_VU , RT_FIXU,RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VSSUB_VV    ->List(Y, N, X, uopVSSUB,       IQT_VEC ,FU_MAC ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VSSUB_VX    ->List(Y, N, X, uopVSSUB,       IQT_IVEC,FU_MAC ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VAADDU_VV   ->List(Y, N, X, uopVAADDU,      IQT_VEC ,FU_MAC ,RT_VU , RT_VU , RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VAADDU_VX   ->List(Y, N, X, uopVAADDU,      IQT_IVEC,FU_MAC ,RT_VU , RT_FIXU,RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VAADD_VV    ->List(Y, N, X, uopVAADD,       IQT_VEC ,FU_MAC ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VAADD_VX    ->List(Y, N, X, uopVAADD,       IQT_IVEC,FU_MAC ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VASUBU_VV   ->List(Y, N, X, uopVASUBU,      IQT_VEC ,FU_MAC ,RT_VU , RT_VU , RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VASUBU_VX   ->List(Y, N, X, uopVASUBU,      IQT_IVEC,FU_MAC ,RT_VU , RT_FIXU,RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VASUB_VV    ->List(Y, N, X, uopVASUB,       IQT_VEC ,FU_MAC ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VASUB_VX    ->List(Y, N, X, uopVASUB,       IQT_IVEC,FU_MAC ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VSMUL_VV    ->List(Y, N, X, uopVSMUL,       IQT_VEC ,FU_MAC ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VSMUL_VX    ->List(Y, N, X, uopVSMUL,       IQT_IVEC,FU_MAC ,RT_VEC, RT_FIX, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VSSRL_VV    ->List(Y, N, X, uopVSSRL,       IQT_VEC ,FU_MAC ,RT_VU , RT_VU , RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VSSRL_VX    ->List(Y, N, X, uopVSSRL,       IQT_IVEC,FU_MAC ,RT_VU , RT_FIXU,RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VSSRL_VI    ->List(Y, N, X, uopVSSRL,       IQT_VEC ,FU_MAC ,RT_VU , RT_VIU, RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VSSRA_VV    ->List(Y, N, X, uopVSSRA,       IQT_VEC ,FU_MAC ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VSSRA_VX    ->List(Y, N, X, uopVSSRA,       IQT_IVEC,FU_MAC ,RT_VEC, RT_FIXU,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VSSRA_VI    ->List(Y, N, X, uopVSSRA,       IQT_VEC ,FU_MAC ,RT_VEC, RT_VIU, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VNCLIPU_WV  ->List(Y, N, X, uopVNCLIPU,     IQT_VEC ,FU_MAC ,RT_VU , RT_VU , RT_VWU, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VNCLIPU_WX  ->List(Y, N, X, uopVNCLIPU,     IQT_IVEC,FU_MAC ,RT_VU , RT_FIXU,RT_VWU, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VNCLIPU_WI  ->List(Y, N, X, uopVNCLIPU,     IQT_VEC ,FU_MAC ,RT_VU , RT_VIU, RT_VWU, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VNCLIP_WV   ->List(Y, N, X, uopVNCLIP,      IQT_VEC ,FU_MAC ,RT_VEC, RT_VEC, RT_VW , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VNCLIP_WX   ->List(Y, N, X, uopVNCLIP,      IQT_IVEC,FU_MAC ,RT_VEC, RT_FIXU,RT_VW , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VNCLIP_WI   ->List(Y, N, X, uopVNCLIP,      IQT_VEC ,FU_MAC ,RT_VEC, RT_VIU, RT_VW , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
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
  VFADD_VV         ->List(Y, Y, X, uopVFADD,       IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFADD_VF         ->List(Y, Y, X, uopVFADD,       IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFSUB_VV         ->List(Y, Y, X, uopVFSUB,       IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFSUB_VF         ->List(Y, Y, X, uopVFSUB,       IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFRSUB_VF        ->List(Y, Y, X, uopVFRSUB,      IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFWADD_VV        ->List(Y, Y, X, uopVFADD,       IQT_VEC ,FU_FPU ,RT_VW , RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFWADD_VF        ->List(Y, Y, X, uopVFADD,       IQT_FVEC,FU_FPU ,RT_VW , RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFWADD_WV        ->List(Y, Y, X, uopVFADD,       IQT_VEC ,FU_FPU ,RT_VW , RT_VEC, RT_VW , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFWADD_WF        ->List(Y, Y, X, uopVFADD,       IQT_FVEC,FU_FPU ,RT_VW , RT_FLT, RT_VW , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFWSUB_VV        ->List(Y, Y, X, uopVFSUB,       IQT_VEC ,FU_FPU ,RT_VW , RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFWSUB_VF        ->List(Y, Y, X, uopVFSUB,       IQT_FVEC,FU_FPU ,RT_VW , RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFWSUB_WV        ->List(Y, Y, X, uopVFSUB,       IQT_VEC ,FU_FPU ,RT_VW , RT_VEC, RT_VW , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFWSUB_WF        ->List(Y, Y, X, uopVFSUB,       IQT_FVEC,FU_FPU ,RT_VW , RT_FLT, RT_VW , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFMUL_VV         ->List(Y, Y, X, uopVFMUL,       IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFMUL_VF         ->List(Y, Y, X, uopVFMUL,       IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFDIV_VV         ->List(Y, Y, X, uopVFDIV,       IQT_VEC ,FU_FDV ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFDIV_VF         ->List(Y, Y, X, uopVFDIV,       IQT_FVEC,FU_FDV ,RT_VEC, RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFRDIV_VF        ->List(Y, Y, X, uopVFRDIV,      IQT_FVEC,FU_FDV ,RT_VEC, RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFWMUL_VV        ->List(Y, Y, X, uopVFMUL,       IQT_VEC ,FU_FPU ,RT_VW , RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFWMUL_VF        ->List(Y, Y, X, uopVFMUL,       IQT_FVEC,FU_FPU ,RT_VW , RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFMACC_VV        ->List(Y, Y, X, uopVFMACC,      IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFMACC_VF        ->List(Y, Y, X, uopVFMACC,      IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFNMACC_VV       ->List(Y, Y, X, uopVFNMACC,     IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFNMACC_VF       ->List(Y, Y, X, uopVFNMACC,     IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFMSAC_VV        ->List(Y, Y, X, uopVFMSAC,      IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFMSAC_VF        ->List(Y, Y, X, uopVFMSAC,      IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFNMSAC_VV       ->List(Y, Y, X, uopVFNMSAC,     IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFNMSAC_VF       ->List(Y, Y, X, uopVFNMSAC,     IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFMADD_VV        ->List(Y, Y, X, uopVFMADD,      IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFMADD_VF        ->List(Y, Y, X, uopVFMADD,      IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFNMADD_VV       ->List(Y, Y, X, uopVFNMADD,     IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFNMADD_VF       ->List(Y, Y, X, uopVFNMADD,     IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFMSUB_VV        ->List(Y, Y, X, uopVFMSUB,      IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFMSUB_VF        ->List(Y, Y, X, uopVFMSUB,      IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFNMSUB_VV       ->List(Y, Y, X, uopVFNMSUB,     IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFNMSUB_VF       ->List(Y, Y, X, uopVFNMSUB,     IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFWMACC_VV       ->List(Y, Y, X, uopVFMACC,      IQT_VEC ,FU_FPU ,RT_VW , RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFWMACC_VF       ->List(Y, Y, X, uopVFMACC,      IQT_FVEC,FU_FPU ,RT_VW , RT_FLT, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFWNMACC_VV      ->List(Y, Y, X, uopVFNMACC,     IQT_VEC ,FU_FPU ,RT_VW , RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFWNMACC_VF      ->List(Y, Y, X, uopVFNMACC,     IQT_FVEC,FU_FPU ,RT_VW , RT_FLT, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFWMSAC_VV       ->List(Y, Y, X, uopVFMSAC,      IQT_VEC ,FU_FPU ,RT_VW , RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFWMSAC_VF       ->List(Y, Y, X, uopVFMSAC,      IQT_FVEC,FU_FPU ,RT_VW , RT_FLT, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFWNMSAC_VV      ->List(Y, Y, X, uopVFNMSAC,     IQT_VEC ,FU_FPU ,RT_VW , RT_VEC, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFWNMSAC_VF      ->List(Y, Y, X, uopVFNMSAC,     IQT_FVEC,FU_FPU ,RT_VW , RT_FLT, RT_VEC, Y, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFSQRT_V         ->List(Y, Y, X, uopVFSQRT,      IQT_VEC ,FU_FDV ,RT_VEC, RT_X  , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFRSQRT7_V       ->List(Y, Y, X, uopVFRSQRT7,    IQT_VEC ,FU_FR7 ,RT_VEC, RT_X  , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFREC7_V         ->List(Y, Y, X, uopVFREC7,      IQT_VEC ,FU_FR7 ,RT_VEC, RT_X  , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFMIN_VV         ->List(Y, Y, X, uopVFMIN,       IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFMIN_VF         ->List(Y, Y, X, uopVFMIN,       IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFMAX_VV         ->List(Y, Y, X, uopVFMAX,       IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFMAX_VF         ->List(Y, Y, X, uopVFMAX,       IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFSGNJ_VV        ->List(Y, Y, X, uopVFSGNJ,      IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFSGNJ_VF        ->List(Y, Y, X, uopVFSGNJ,      IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFSGNJN_VV       ->List(Y, Y, X, uopVFSGNJ,      IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFSGNJN_VF       ->List(Y, Y, X, uopVFSGNJ,      IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFSGNJX_VV       ->List(Y, Y, X, uopVFSGNJ,      IQT_VEC ,FU_FPU ,RT_VEC, RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFSGNJX_VF       ->List(Y, Y, X, uopVFSGNJ,      IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMFEQ_VV         ->List(Y, Y, X, uopVMFEQ,       IQT_VEC ,FU_F2I ,RT_VM , RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMFEQ_VF         ->List(Y, Y, X, uopVMFEQ,       IQT_FVEC,FU_F2I ,RT_VM , RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMFNE_VV         ->List(Y, Y, X, uopVMFNE,       IQT_VEC ,FU_F2I ,RT_VM , RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMFNE_VF         ->List(Y, Y, X, uopVMFNE,       IQT_FVEC,FU_F2I ,RT_VM , RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMFLT_VV         ->List(Y, Y, X, uopVMFLT,       IQT_VEC ,FU_F2I ,RT_VM , RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMFLT_VF         ->List(Y, Y, X, uopVMFLT,       IQT_FVEC,FU_F2I ,RT_VM , RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMFLE_VV         ->List(Y, Y, X, uopVMFLE,       IQT_VEC ,FU_F2I ,RT_VM , RT_VEC, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMFLE_VF         ->List(Y, Y, X, uopVMFLE,       IQT_FVEC,FU_F2I ,RT_VM , RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMFGT_VF         ->List(Y, Y, X, uopVMFGT,       IQT_FVEC,FU_F2I ,RT_VM , RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VMFGE_VF         ->List(Y, Y, X, uopVMFGE,       IQT_FVEC,FU_F2I ,RT_VM , RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFCLASS_V        ->List(Y, Y, X, uopVFCLASS,     IQT_VEC ,FU_F2I ,RT_VEC, RT_X  , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFMV_V_F         ->List(Y, Y, X, uopVFMV_V_F,    IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFMV_F_S         ->List(Y, Y, X, uopVFMV_F_S,    IQT_VEC ,FU_ALU ,RT_FLT, RT_X  , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, N, Y, DC2)
 ,VFMV_S_F         ->List(Y, Y, X, uopVFMV_S_F,    IQT_FVEC,FU_FPU ,RT_VEC, RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFCVT_XU_F_V     ->List(Y, Y, X, uopVFCVT_F2I,   IQT_VEC ,FU_F2I ,RT_VEC, RT_X  , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFCVT_X_F_V      ->List(Y, Y, X, uopVFCVT_F2I,   IQT_VEC ,FU_F2I ,RT_VEC, RT_X  , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFCVT_F_XU_V     ->List(Y, Y, X, uopVFCVT_I2F,   IQT_VEC ,FU_I2F ,RT_VEC, RT_X  , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFCVT_F_X_V      ->List(Y, Y, X, uopVFCVT_I2F,   IQT_VEC ,FU_I2F ,RT_VEC, RT_X  , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFCVT_RTZ_XU_F_V ->List(Y, Y, X, uopVFCVT_F2I,   IQT_VEC ,FU_F2I ,RT_VEC, RT_X  , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFCVT_RTZ_X_F_V  ->List(Y, Y, X, uopVFCVT_F2I,   IQT_VEC ,FU_F2I ,RT_VEC, RT_X  , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFWCVT_XU_F_V    ->List(Y, Y, X, uopVFCVT_F2I,   IQT_VEC ,FU_F2I ,RT_VW , RT_X  , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFWCVT_X_F_V     ->List(Y, Y, X, uopVFCVT_F2I,   IQT_VEC ,FU_F2I ,RT_VW , RT_X  , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFWCVT_F_XU_V    ->List(Y, Y, X, uopVFCVT_I2F,   IQT_VEC ,FU_I2F ,RT_VW , RT_X  , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFWCVT_F_X_V     ->List(Y, Y, X, uopVFCVT_I2F,   IQT_VEC ,FU_I2F ,RT_VW , RT_X  , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFWCVT_F_F_V     ->List(Y, Y, X, uopVFCVT_F2F,   IQT_VEC ,FU_FPU ,RT_VW , RT_X  , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFWCVT_RTZ_XU_F_V->List(Y, Y, X, uopVFCVT_F2I,   IQT_VEC ,FU_F2I ,RT_VW , RT_X  , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFWCVT_RTZ_X_F_V ->List(Y, Y, X, uopVFCVT_F2I,   IQT_VEC ,FU_F2I ,RT_VW , RT_X  , RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFNCVT_XU_F_W    ->List(Y, Y, X, uopVFCVT_F2I,   IQT_VEC ,FU_F2I ,RT_VEC, RT_X  , RT_VW , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFNCVT_X_F_W     ->List(Y, Y, X, uopVFCVT_F2I,   IQT_VEC ,FU_F2I ,RT_VEC, RT_X  , RT_VW , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFNCVT_F_XU_W    ->List(Y, Y, X, uopVFCVT_I2F,   IQT_VEC ,FU_I2F ,RT_VEC, RT_X  , RT_VW , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFNCVT_F_X_W     ->List(Y, Y, X, uopVFCVT_I2F,   IQT_VEC ,FU_I2F ,RT_VEC, RT_X  , RT_VW , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFNCVT_F_F_W     ->List(Y, Y, X, uopVFCVT_F2F,   IQT_VEC ,FU_FPU ,RT_VEC, RT_X  , RT_VW , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFNCVT_ROD_F_F_W ->List(Y, Y, X, uopVFCVT_F2F,   IQT_VEC ,FU_FPU ,RT_VEC, RT_X  , RT_VW , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFNCVT_RTZ_XU_F_W->List(Y, Y, X, uopVFCVT_F2I,   IQT_VEC ,FU_F2I ,RT_VEC, RT_X  , RT_VW , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFNCVT_RTZ_X_F_W ->List(Y, Y, X, uopVFCVT_F2I,   IQT_VEC ,FU_F2I ,RT_VEC, RT_X  , RT_VW , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFMERGE_VFM      ->List(Y, Y, X, uopMERGE,       IQT_FVEC,FU_ALU ,RT_VEC, RT_FLT, RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
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
  VREDSUM_VS  ->List(Y, N, X, uopVADD,        IQT_VEC ,FU_ALU ,RT_VRED,RT_VRED,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VREDMAXU_VS ->List(Y, N, X, uopVMAXU,       IQT_VEC ,FU_ALU ,RT_VRU ,RT_VRU ,RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VREDMAX_VS  ->List(Y, N, X, uopVMAX,        IQT_VEC ,FU_ALU ,RT_VRED,RT_VRED,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VREDMINU_VS ->List(Y, N, X, uopVMINU,       IQT_VEC ,FU_ALU ,RT_VRU ,RT_VRU ,RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VREDMIN_VS  ->List(Y, N, X, uopVMIN,        IQT_VEC ,FU_ALU ,RT_VRED,RT_VRED,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VREDAND_VS  ->List(Y, N, X, uopVAND,        IQT_VEC ,FU_ALU ,RT_VRED,RT_VRED,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VREDOR_VS   ->List(Y, N, X, uopVOR,         IQT_VEC ,FU_ALU ,RT_VRED,RT_VRED,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VREDXOR_VS  ->List(Y, N, X, uopVXOR,        IQT_VEC ,FU_ALU ,RT_VRED,RT_VRED,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VWREDSUMU_VS->List(Y, N, X, uopVADD,        IQT_VEC ,FU_ALU ,RT_VRWU,RT_VRWU,RT_VU , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VWREDSUM_VS ->List(Y, N, X, uopVADD,        IQT_VEC ,FU_ALU ,RT_VRW ,RT_VRW ,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFREDOSUM_VS->List(Y, Y, X, uopVFADD,       IQT_VEC ,FU_FPU ,RT_VRED,RT_VRED,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFREDSUM_VS ->List(Y, Y, X, uopVFADD,       IQT_VEC ,FU_FPU ,RT_VRED,RT_VRED,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFREDMIN_VS ->List(Y, Y, X, uopVFMIN,       IQT_VEC ,FU_FPU ,RT_VRED,RT_VRED,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFREDMAX_VS ->List(Y, Y, X, uopVFMAX,       IQT_VEC ,FU_FPU ,RT_VRED,RT_VRED,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFWREDOSUM_VS->List(Y,Y, X, uopVFADD,       IQT_VEC ,FU_FPU ,RT_VRW ,RT_VRW ,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
 ,VFWREDSUM_VS->List(Y, Y, X, uopVFADD,       IQT_VEC ,FU_FPU ,RT_VRW ,RT_VRW ,RT_VEC, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, Y, Y, Y, DC2)
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
    CUSTOM0            ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
    CUSTOM0_RS1        ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_FIX, RT_X  , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
    CUSTOM0_RS1_RS2    ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
    CUSTOM0_RD         ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
    CUSTOM0_RD_RS1     ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_X  , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
    CUSTOM0_RD_RS1_RS2 ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
    CUSTOM1            ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
    CUSTOM1_RS1        ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_FIX, RT_X  , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
    CUSTOM1_RS1_RS2    ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
    CUSTOM1_RD         ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
    CUSTOM1_RD_RS1     ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_X  , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
    CUSTOM1_RD_RS1_RS2 ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
    CUSTOM2            ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
    CUSTOM2_RS1        ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_FIX, RT_X  , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
    CUSTOM2_RS1_RS2    ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
    CUSTOM2_RD         ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
    CUSTOM2_RD_RS1     ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_X  , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
    CUSTOM2_RD_RS1_RS2 ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
    CUSTOM3            ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
    CUSTOM3_RS1        ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_FIX, RT_X  , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
    CUSTOM3_RS1_RS2    ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
    CUSTOM3_RD         ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
    CUSTOM3_RD_RS1     ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_X  , N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2),
    CUSTOM3_RD_RS1_RS2 ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , U_0, N, N, N, N, N, CSR.N, N, N, N, DC2)
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
  if (usingRoCC) decode_table ++= RoCCDecode.table
  decode_table ++= (if (xLen == 64) X64Decode.table else X32Decode.table)
  if (usingVector) decode_table ++= VectorLSDecode.table
  if (usingVector) decode_table ++= VectorCfgDecode.table
  if (usingVector) decode_table ++= VectorIntDecode.table
  if (usingVector) decode_table ++= VectorFixDecode.table
  if (usingVector) decode_table ++= VectorFPDecode.table
  if (usingVector) decode_table ++= VectorRedDecode.table

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
    val ls_vconfig = WireInit(io.csr_vconfig)
    ls_vconfig.vtype.vsew := cs.v_ls_ew
    val ls_vlmax = ls_vconfig.vtype.vlMax
    val is_v_ls = cs.is_rvv & (cs.uses_stq | cs.uses_ldq)
    val is_v_ls_ustride = cs.uopc.isOneOf(uopVL, uopVLFF, uopVSA)
    val is_v_ls_stride = cs.uopc.isOneOf(uopVLS, uopVSSA)
    val isVMVR: Bool = cs.uopc.isOneOf(uopVMVR)
    val is_v_ls_index = cs.uopc.isOneOf(uopVLUX, uopVSUXA, uopVLOX, uopVSOXA)
    val is_v_mask_ls = cs.uopc.isOneOf(uopVLM, uopVSMA)
    val is_viota_m = cs.uopc.isOneOf(uopVIOTA)
    val is_v_reg_ls = cs.uopc.isOneOf(uopVLR, uopVSR)
    val vseg_nf = inst(NF_MSB, NF_LSB)
    val is_v_ls_seg = is_v_ls && (vseg_nf =/= 0.U) && !is_v_reg_ls
    val vstart  = RegInit(0.U((vLenSz+1).W))
    val vMVRCounter = RegInit(0.U(3.W))
    val vseg_finc = RegInit(0.U(3.W))
    val vseg_gidx = RegInit(0.U(3.W))

    val vreg_nf = WireDefault(UInt((NF_MSB - NF_LSB + 2).W), vseg_nf)
    val vlmax = Mux(is_v_mask_ls, vLenb.U,
                Mux(is_v_reg_ls, vLenb.U << Log2(vreg_nf + 1.U) >> cs.v_ls_ew,
                Mux(is_v_ls_ustride || is_v_ls_stride, ls_vlmax, io.csr_vconfig.vtype.vlMax)))
    val vsew = Mux(is_v_reg_ls, cs.v_ls_ew, io.csr_vconfig.vtype.vsew)
    val vlmul_sign = io.csr_vconfig.vtype.vlmul_sign
    val vlmul = Mux(vlmul_sign, 0.U(2.W), io.csr_vconfig.vtype.vlmul_mag)
    val vd_wfactor = Mux(uop.rt(RD,  isWidenV ), 1.U, 0.U)
    val vd_nfactor = Mux(uop.rt(RD,  isNarrowV), 1.U, 0.U)
    val vs2_wfactor= Mux(uop.rt(RS2, isWidenV ), 1.U, 0.U)
    val vs2_nfactor= Mux(uop.uopc === uopVEXT8 , 3.U,
                     Mux(uop.uopc === uopVEXT4 , 2.U,
                     Mux(uop.rt(RS2, isNarrowV), 1.U, 0.U))) // uopVEXT2 is included
    val vd_sew     = Mux(is_v_ls && !is_v_ls_index, cs.v_ls_ew,
                     Mux(uop.rt(RD,  isWidenV ), vsew + vd_wfactor,
                     Mux(uop.rt(RD,  isNarrowV), vsew - vd_nfactor, vsew)))
    val vs2_sew    = Mux(is_v_ls_index, Cat(0.U(1.W), cs.v_ls_ew),
                     Mux(uop.rt(RS2, isWidenV ), vsew + vs2_wfactor,
                     Mux(uop.rt(RS2, isNarrowV), vsew - vs2_nfactor,vsew)))
    val vd_inc     = Mux(isVMVR, vMVRCounter,
                     vstart >> (vLenSz.U - 3.U - vd_sew))
    val vs2_inc    = Mux(is_viota_m, 0.U,
                     Mux(isVMVR, vMVRCounter,
                     vstart >> (vLenSz.U - 3.U - vs2_sew)))
    val vs1_inc    = Mux(is_viota_m, 0.U, vstart >> (vLenSz.U - 3.U - vsew))
    when (io.deq_fire && cs.is_rvv) {
      assert(vsew <= 3.U, "Unsupported vsew")
      //assert(vsew >= vd_nfactor  && vsew + vd_wfactor  <= 3.U, "Unsupported vd_sew")
      //assert(vsew >= vs2_nfactor && vsew + vs2_wfactor <= 3.U, "Unsupported vs2_sew")
    }

    // vmasklogic
    val vmlogic_insn = cs.uopc.isOneOf(uopVMAND, uopVMNAND, uopVMANDNOT, uopVMXOR, uopVMOR, uopVMNOR, uopVMORNOT, uopVMXNOR)
    val byteWidth = 3.U
    val vsew64bit = 3.U
    val vmlogic_split_ecnt = vLen.U >> (byteWidth +& vsew64bit)
    // vsew => element num
    val vmlogic_tolal_ecnt = vLen.U >> (byteWidth +& vsew64bit)

    val is_vmask_cnt_m = cs.uopc.isOneOf(uopVPOPC, uopVFIRST)
    val is_vmask_set_m = cs.uopc.isOneOf(uopVMSOF, uopVMSBF, uopVMSIF)
    val is_v_mask_insn = vmlogic_insn || is_vmask_cnt_m || is_vmask_set_m

    //val eLen_ecnt = eLen.U >> (vsew+3.U)
    val vLen_ecnt  = Mux(!is_v_ls && vs2_sew > vd_sew, vLen.U >> (vs2_sew+3.U), vLen.U >> (vd_sew+3.U))
    val split_ecnt = Mux(is_v_ls, 1.U,
      Mux(is_v_mask_insn, vmlogic_split_ecnt, vLen_ecnt))
    // for store, we can skip inactive locations; otherwise, we have to visit every element
    // for fractional lmul, we need visit at least one entire vreg
    val total_ecnt = Mux(is_v_reg_ls, vlmax,
      Mux(cs.uses_stq, Mux(is_v_mask_ls, (io.csr_vconfig.vl + 7.U) >> 3.U, io.csr_vconfig.vl),
      Mux(vlmul_sign && !is_v_mask_ls, vLen_ecnt, Mux(is_v_mask_insn, vmlogic_tolal_ecnt, vlmax))))
    val vseg_flast = vseg_finc === vseg_nf
    val vMVRCount: UInt = inst(RS1_MSB,RS1_LSB)
    val elem_last  = Mux(isVMVR, vMVRCounter === vMVRCount, (vstart + split_ecnt) >= total_ecnt)
    val split_last = elem_last && Mux(is_v_ls_seg, vseg_flast, true.B)
    when (io.kill) {
      vstart    := 0.U
      vseg_finc := 0.U
      vseg_gidx := 0.U
      vMVRCounter := 0.U
    } .elsewhen (~cs.can_be_split | split_last & io.deq_fire) {
      vstart    := 0.U
      vseg_finc := 0.U
      vseg_gidx := 0.U
      vMVRCounter := 0.U
    } .elsewhen (cs.can_be_split & ~split_last & io.deq_fire) {
      when (is_v_ls_seg) {
        vseg_finc := Mux(vseg_flast, 0.U, vseg_finc + 1.U)
        vstart    := vstart    + Mux(vseg_flast, 1.U, 0.U)
        vseg_gidx := vseg_gidx + Mux((vseg_flast) && uop.v_re_alloc, 1.U, 0.U)
      }.elsewhen(isVMVR){
        vstart := 0.U
        vMVRCounter := vMVRCounter + 1.U
      } .otherwise {
        vstart    := vstart + split_ecnt
      }
    }
    val vseg_vd_inc  = (vseg_finc << vlmul) + vseg_gidx

    uop.is_rvv      := cs.is_rvv
    uop.v_ls_ew     := cs.v_ls_ew
    when (is_v_ls) {
      uop.mem_size  := Mux(is_v_ls_index, vsew, cs.v_ls_ew)
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
    uop.vconfig.vtype.vsew := Mux(is_v_mask_insn, 3.U, vsew)
    when (io.deq_fire && cs.can_be_split) {
      assert(cs.is_rvv, "can_be_split applies only to vector instructions.")
    }
    uop.v_is_first  := (vstart === 0.U)
    uop.v_is_last   := elem_last
    val ren_mask = ~(Fill(vLenSz,1.U) << (7.U - vd_sew))
    val vs1_mask = ~(Fill(vLenSz,1.U) << (7.U - vsew))
    val vs2_mask = ~(Fill(vLenSz,1.U) << (7.U - vs2_sew))
    uop.v_re_alloc  := Mux(uop.rt(RD, isMaskVD), vstart === 0.U, (vstart & ren_mask(vLenSz,0)) === 0.U)
    uop.v_re_vs1    := (vstart & vs1_mask(vLenSz,0)) === 0.U
    uop.v_re_vs2    := (vstart & vs2_mask(vLenSz,0)) === 0.U

    when (cs.is_rvv) {
      when (!uop.rt(RD, isMaskVD)) {
        uop.ldst := inst(RD_MSB,RD_LSB) + vd_inc
      }
      when (uop.rt(RS1, isVector)) {
        uop.lrs1 := inst(RS1_MSB,RS1_LSB) + vs1_inc
      }
      when (uop.rt(RS2, isVector)) {
        uop.lrs2 := inst(RS2_MSB,RS2_LSB) + vs2_inc
      }
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

    uop.v_idx_ls     := is_v_ls_index

    when (cs.is_rvv && cs.uopc === uopVLM) {
      uop.uopc := uopVL
      // make elements >= ceil(vl/8) inactive
      uop.vconfig.vl := (io.csr_vconfig.vl + 7.U) >> 3.U
    }
    when (cs.is_rvv && cs.uopc === uopVSMA) {
      uop.uopc := uopVSA
    }
    when (cs.is_rvv && cs.uopc === uopVLR) {
      uop.uopc := uopVL
    }
    when (cs.is_rvv && cs.uopc === uopVSR) {
      uop.uopc := uopVSA
    }

    uop.v_xls_offset := 0.U

    uop.v_seg_ls  := false.B
    uop.v_seg_f   := 0.U
    uop.v_seg_nf  := 1.U
    when (cs.is_rvv && is_v_ls_seg) {
      uop.ldst := inst(RD_MSB,RD_LSB) + vseg_vd_inc
      uop.lrs3 := uop.ldst
      uop.v_seg_ls   := true.B
      uop.v_seg_f    := vseg_finc
      uop.v_seg_nf   := 1.U(4.W) + vseg_nf
    }

    // handle load tail: dispatch to vector pipe
    // masked load / store, send to vector pipe
    when (cs.is_rvv && cs.uses_ldq && !is_v_reg_ls && vstart >= io.csr_vconfig.vl) {
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

    io.enq_stall := cs.can_be_split && !split_last

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
