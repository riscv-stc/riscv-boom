//******************************************************************************
// Copyright (c) 2015 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

package boom.exu

import Chisel.UInt
import chisel3._
import chisel3.util._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.rocket.Instructions._
import freechips.rocketchip.rocket.RVCExpander
import freechips.rocketchip.rocket.{CSR, Causes, VConfig, VType, MConfig, MType}
import freechips.rocketchip.util.{UIntIsOneOf, rightOR, uintToBitPat}
import FUConstants._
import boom.common._
import boom.common.MicroOpcodes._
import boom.util._
import freechips.rocketchip.util._

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
  //                                                       frs3_en                      wakeup_delay
  //     is val inst?                                      |  imm sel                   |    bypassable (aka, known/fixed latency)
  //     |  is fp inst?                                    |  |     uses_ldq            |    |  is_br              is vector instruction
  //     |  |  is single-prec?               rs1 regtype   |  |     |  uses_stq         |    |  |                  |  can be split  allow_vd_is_v0
  //     |  |  |  micro-code                 |      rs2 type  |     |  |  is_amo        |    |  |                  |  |  use vm?    |  not_use_vtype
  //     |  |  |  |     iq-type  func unit   |      |      |  |     |  |  |  is_fence   |    |  |                  |  |  |  ew of ls vector
  //     |  |  |  |     |        |           |      |      |  |     |  |  |  |  is_fencei    |  |  is breakpoint or ecall?  |       |  |  vd_unequal_vs1
  //     |  |  |  |     |        |     dst   |      |      |  |     |  |  |  |  |  mem  |    |  |  |  is unique? (clear pipeline for it)  |  vd_unequal_vs2
  //     |  |  |  |     |        |     regtype      |      |  |     |  |  |  |  |  cmd  |    |  |  |  |  flush on commit |  |       |  |  |  |  is matrix inst?
  //     |  |  |  |     |        |     |     |      |      |  |     |  |  |  |  |  |    |    |  |  |  |  |  csr cmd   |  |  |    vstart_is_zero |
  //     |  |  |  |     |        |     |     |      |      |  |     |  |  |  |  |  |    |    |  |  |  |  |  |      |  |  |  |    |  |  |  |  |  |
    List(N, N, X, uopX, IQT_INT, FU_X, RT_X, RT_DC, RT_DC, X, IS_X, X, X, X, X, N, M_X, DC2, X, X, N, N, X, CSR.X, N, N, N, DC2, X, X, X, X, X, N)

  val table: Array[(BitPat, List[BitPat])]
}

object DecoderCSVReader {
  def apply(csv: String, category: String, preContext: String, postContext: String): Array[(BitPat, List[BitPat])] = {
    import scala.reflect.runtime.universe
    import scala.tools.reflect.ToolBox
    import scala.io.Source

    println(s"Reading $category instructions from resources$csv")
    val stream= getClass.getResourceAsStream(csv)
    val lines = Source.fromInputStream(stream).mkString
    val src   = lines.split("\n")
                     .filter(_.startsWith(category))
                     .map(_.split(" +"))
                     .map(row => row(1) + " -> List(" + row.drop(2).mkString(",") + ")")
                     .mkString(preContext, ",\n", postContext)
    val tb    = universe.runtimeMirror(getClass.getClassLoader).mkToolBox()
    val ret: Array[(BitPat, List[BitPat])] = tb.eval(tb.parse(src)).asInstanceOf[Array[(BitPat, List[BitPat])]]
    ret
  }
}

/**
 * Decoded control signals
 */
class CtrlSigs extends Bundle with DecodeConstants
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
  val vstart_is_zero  = Bool()
  val allow_vd_is_v0  = Bool()
  val not_use_vtype   = Bool()
  val vd_unequal_vs1  = Bool()
  val vd_unequal_vs2  = Bool()
  val is_rvm          = Bool()

  def decode(inst: UInt, table: Iterable[(BitPat, List[BitPat])]) = {
    //val decoder = freechips.rocketchip.rocket.DecodeLogic(inst, decode_default, table)
    val decoder = BoomDecoder(inst, decode_default, table)
    val sigs =
      Seq(legal, fp_val, fp_single, uopc, iq_type, fu_code, dst_type, rs1_type,
          rs2_type, frs3_en, imm_sel, uses_ldq, uses_stq, is_amo,
          is_fence, is_fencei, mem_cmd, wakeup_delay, bypassable,
          is_br, is_sys_pc2epc, inst_unique, flush_on_commit, csr_cmd,
          is_rvv, can_be_split, uses_vm, v_ls_ew, vstart_is_zero,
          allow_vd_is_v0, not_use_vtype, vd_unequal_vs1, vd_unequal_vs2,
          is_rvm)
      sigs zip decoder map {case(s,d) => s := d}
      rocc := false.B
      this
  }

  val table: Array[(BitPat, List[BitPat])] = null
}

// TODO:
// 1. make vsetvli and vsetivli not unique, by tagging at Decode stage, with a flop control whether using CSR.vconfig all decoder.vconfig

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
  val csr_vstart = Input(UInt(vLenSz.W))
  val interrupt = Input(Bool())
  val interrupt_cause = Input(UInt(xLen.W))

  val csr_mconfig = Input(new MConfig)
  val csr_tilem = Input(UInt(xLen.W))
  val csr_tilek = Input(UInt(xLen.W))
  val csr_tilen = Input(UInt(xLen.W))
  val csr_tsidx = Input(UInt(xLen.W))
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

  val definition: String = "/csv/decode-inst.csv"
  val preContext: String = s"""
  |{
  |  import chisel3._
  |  import chisel3.util._
  |  import freechips.rocketchip.rocket.CSR
  |  import freechips.rocketchip.rocket.Instructions._
  |  import freechips.rocketchip.util.uintToBitPat
  |  import boom.common._
  |  import boom.common.MicroOpcodes._
  |  import boom.exu.FUConstants._
  |  object DecoderCSVHelper extends boom.exu.DecodeConstants
  |  {
  |    val table: Array[(BitPat, List[BitPat])] = Array(
  |""".stripMargin
  val postContext: String = s"""
  |    )
  |  }
  |  val ret: Array[(BitPat, List[BitPat])] = DecoderCSVHelper.table
  |  ret
  |}""".stripMargin
  val xLenISA: String = if (xLen == 64) "X64" else "X32"
  def getTable(category: String): Array[(BitPat, List[BitPat])] = {
    DecoderCSVReader(definition, category, preContext, postContext)
  }

  var decode_table = getTable("XInt")
  decode_table ++= getTable(xLenISA)
  if (usingFPU) {
    decode_table ++= getTable("Float")
    if (usingFDivSqrt) decode_table ++= getTable("FDivSqrt")
    if (usingzfhExt) decode_table ++= getTable("zfhExt")
  }
  if (usingRoCC) decode_table ++= getTable("RoCC")
  if (usingVector) {
    decode_table ++= getTable("VectorCfg")
    decode_table ++= getTable("VectorMem")
    decode_table ++= getTable("VectorInt")
    decode_table ++= getTable("VectorFix")
    decode_table ++= getTable("VectorFloat")
    decode_table ++= getTable("VectorReduction")
    decode_table ++= getTable("VectorMask")
    decode_table ++= getTable("VectorPerm")
  }
  if(usingMatrix) decode_table ++= getTable("Matrix")

  val inst = uop.inst

  val cs = Wire(new CtrlSigs()).decode(inst, decode_table)

  // Exception Handling
  val vsetvl = cs.uopc.isOneOf(uopVSETVL, uopVSETVLI, uopVSETIVLI)
  val mconfig = uop.is_rvm && cs.inst_unique
  io.csr_decode.csr := Mux(vsetvl | mconfig, 0.U, inst(31,20))
  val csr_en = cs.csr_cmd.isOneOf(CSR.S, CSR.C, CSR.W) && !vsetvl && !mconfig
  val csr_ren = cs.csr_cmd.isOneOf(CSR.S, CSR.C) && uop.lrs1 === 0.U
  val system_insn = cs.csr_cmd === CSR.I
  val sfence = cs.uopc === uopSFENCE

  val cs_legal = cs.legal
//   dontTouch(cs_legal)
  val illegal_vector_case = if(usingVector) Wire(Bool()) else false.B
  val illegal_matrix_case = if(usingMatrix) Wire(Bool()) else false.B
  val id_illegal_insn = !cs_legal ||
    cs.fp_val && io.csr_decode.fp_illegal || // TODO check for illegal rm mode: (io.fpu.illegal_rm)
    cs.is_rvv && (io.csr_decode.vector_illegal || illegal_vector_case) ||
    cs.is_rvm && (io.csr_decode.matrix_illegal || illegal_matrix_case) ||
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
  uop.ctrl.csr_cmd := cs.csr_cmd

  // x-registers placed in 0-31, f-registers placed in 32-63.
  // This allows us to straight-up compare register specifiers and not need to
  // verify the rtypes (e.g., bypassing in rename).
  val instRD      = inst(RD_MSB,RD_LSB)
  val instRS1     = inst(RS1_MSB,RS1_LSB)
  val instRS2     = inst(RS2_MSB,RS2_LSB)
  val instRS3     = inst(RS3_MSB,RS3_LSB)
  uop.ldst       := instRD
  uop.lrs1       := instRS1
  uop.lrs2       := instRS2
  uop.lrs3       := instRS3

  uop.ldst_val   := isSomeReg(cs.dst_type) && !(uop.ldst === 0.U && uop.rt(RD, isInt)) && Mux(cs.is_rvv || cs.is_rvm, !cs.uses_stq, true.B)
  uop.dst_rtype  := cs.dst_type
  uop.lrs1_rtype := cs.rs1_type
  uop.lrs2_rtype := cs.rs2_type
  uop.frs3_en    := cs.frs3_en

  uop.ldst_is_rs1 := uop.is_sfb_shadow
  // SFB optimization
  when (uop.is_sfb_shadow && isNotReg(cs.rs2_type)) {
    uop.lrs2_rtype  := RT_FIX
    uop.lrs2        := instRD
    uop.ldst_is_rs1 := false.B
  } .elsewhen (uop.is_sfb_shadow && cs.uopc === uopADD && instRS1 === 0.U) {
    uop.uopc        := uopMOV
    uop.lrs1        := instRD
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
  uop.is_unique  := cs.inst_unique || (cs.uopc === uopVSETVLI) && inst(19,15) === 0.U && inst(11,7)  === 0.U
  uop.flush_on_commit := cs.flush_on_commit || (csr_en && !csr_ren && io.csr_decode.write_flush) || (cs.uopc === uopVSETVLI) && inst(19,15) === 0.U && inst(11,7)  === 0.U
  uop.is_vsetivli := (cs.uopc === uopVSETIVLI)
  uop.is_vsetvli := (cs.uopc === uopVSETVLI)
  uop.vl_ready   := Mux(cs.not_use_vtype, true.B, io.enq.uop.vl_ready)
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
    val csr_vsew       = io.csr_vconfig.vtype.vsew
    val csr_vlmax      = io.csr_vconfig.vtype.vlMax
    val is_v_ls        = cs.is_rvv & (cs.uses_stq | cs.uses_ldq)
    val isVMVR         = cs.uopc.isOneOf(uopVMVR)
    val is_v_ls_index  = cs.uopc.isOneOf(uopVLUX, uopVSUXA, uopVLOX, uopVSOXA)
    val is_v_mask_ld   = cs.uopc.isOneOf(uopVLM)
    val is_v_mask_st   = cs.uopc.isOneOf(uopVSMA)
    val is_v_reg_ls    = cs.uopc.isOneOf(uopVLR, uopVSR)
    val is_viota_m     = cs.uopc.isOneOf(uopVIOTA)
    val vmlogic_insn   = cs.uopc.isOneOf(uopVMAND, uopVMNAND, uopVMANDNOT, uopVMXOR, uopVMOR, uopVMNOR, uopVMORNOT, uopVMXNOR)
    val is_vmask_cnt_m = cs.uopc.isOneOf(uopVPOPC, uopVFIRST)
    val is_vmask_set_m = cs.uopc.isOneOf(uopVMSOF, uopVMSBF, uopVMSIF)
    val is_v_mask_insn = vmlogic_insn || is_vmask_cnt_m || is_vmask_set_m
    val isScalarMove   = cs.uopc.isOneOf(uopVMV_X_S, uopVMV_S_X, uopVFMV_F_S, uopVFMV_S_F)
    
    val vseg_nf = inst(NF_MSB, NF_LSB)
    val is_v_ls_seg = is_v_ls && (vseg_nf =/= 0.U) && !is_v_reg_ls

    val vreg_nf = WireDefault(UInt((NF_MSB - NF_LSB + 2).W), vseg_nf)
    val vsew = Mux(is_v_reg_ls, cs.v_ls_ew, csr_vsew)
    val vlmul_sign = io.csr_vconfig.vtype.vlmul_sign
    val vlmul_mag  = io.csr_vconfig.vtype.vlmul_mag
    val vlmul = Mux(vlmul_sign, 0.U(2.W), vlmul_mag)
    val vd_wfactor = Mux(uop.rt(RD,  isWidenV ), 1.U, 0.U)
    val vd_nfactor = Mux(uop.rt(RD,  isNarrowV), 1.U, 0.U)
    val vs2_wfactor= Mux(uop.rt(RS2, isWidenV ), 1.U, 0.U)
    val vs2_nfactor= Mux(uop.uopc === uopVEXT8 , 3.U,
                     Mux(uop.uopc === uopVEXT4 , 2.U,
                     Mux(uop.rt(RS2, isNarrowV), 1.U, 0.U))) // uopVEXT2 is included
    val vd_sew     = Mux(is_v_ls && !is_v_ls_index, cs.v_ls_ew,
                     Mux(is_v_mask_insn, 3.U,
                     Mux(uop.rt(RD,  isWidenV ), csr_vsew + vd_wfactor,
                     Mux(uop.rt(RD,  isNarrowV), csr_vsew - vd_nfactor, csr_vsew))))
    val vs1_sew    = Mux(cs.uopc === uopVRGATHEREI16, 1.U,
                     Mux(is_v_mask_insn, 3.U,
                     Mux(uop.rt(RS1, isWidenV ), csr_vsew + vd_wfactor, csr_vsew)))
    val vs2_sew    = Mux(is_v_ls_index, Cat(0.U(1.W), cs.v_ls_ew),
                     Mux(is_v_mask_insn, 3.U,
                     Mux(uop.rt(RS2, isWidenV ), csr_vsew + vs2_wfactor,
                     Mux(uop.rt(RS2, isNarrowV), csr_vsew - vs2_nfactor, csr_vsew))))
    val vlmul_value = Mux(is_v_mask_insn, 0.U, Cat(vlmul_sign, vlmul_mag))
    val vmvr_emul  = Mux(instRS1(2), 3.U, Mux(instRS1(1), 2.U, Mux(instRS1(0), 1.U, 0.U)))
    val vd_emul    = Mux(isVMVR, vmvr_emul,
                     Mux(isScalarMove || is_v_mask_ld || vmlogic_insn || uop.is_reduce || uop.rt(RD, isMaskVD) || !uop.rt(RD, isVector), 0.U,
                     Mux(uop.rt(RD, isWidenV), vlmul_value + vd_wfactor,
                     Mux(uop.rt(RD, isNarrowV), vlmul_value - vd_nfactor,
                     Mux(is_v_reg_ls, Log2(vreg_nf + 1.U),
                     Mux(is_v_ls && !is_v_ls_index, cs.v_ls_ew - vsew + vlmul_value, vlmul_value))))))
    val vs1_emul   = Mux(cs.uopc === uopVRGATHEREI16, vlmul_value + 1.U - vsew,
                     Mux(vmlogic_insn || uop.is_reduce || !uop.rt(RS1, isVector), 0.U,
                     Mux(uop.rt(RS1, isWidenV), vlmul_value + 1.U, vlmul_value)))
    val vs2_emul   = Mux(isVMVR, vmvr_emul,
                     Mux(is_v_ls_index, cs.v_ls_ew - vsew + vlmul_value,
                     Mux(uop.rt(RS2, isWidenV), vlmul_value + vs2_wfactor,
                     Mux(uop.rt(RS2, isNarrowV), vlmul_value - vs2_nfactor,
                     Mux(isScalarMove || vmlogic_insn || is_viota_m || !uop.rt(RS2, isVector), 0.U, vlmul_value)))))
    when (io.deq_fire && cs.is_rvv) {
      assert(vsew <= 3.U, "Unsupported vsew")
      //assert(vsew >= vd_nfactor  && vsew + vd_wfactor  <= 3.U, "Unsupported vd_sew")
      //assert(vsew >= vs2_nfactor && vsew + vs2_wfactor <= 3.U, "Unsupported vs2_sew")
    }

    val vLen_ecnt  = Mux(!is_v_ls && vs2_sew > vd_sew && !vlmul_sign, vLen.U >> (vs2_sew+3.U), vLen.U >> (vd_sew+3.U))
    // for store, we can skip inactive locations; otherwise, we have to visit every element
    // for fractional lmul, we need visit at least one entire vreg
    // for undisturbing move before reduction, we need visit at most one vreg
    val total_ecnt = Mux(is_v_mask_ld, vLenb.U,
                     Mux(is_v_reg_ls, vLenb.U << Log2(vreg_nf + 1.U) >> cs.v_ls_ew,
                     Mux(cs.uses_stq, Mux(is_v_mask_st, (io.csr_vconfig.vl + 7.U) >> 3.U, io.csr_vconfig.vl),
                     Mux(isVMVR, vLen_ecnt << vmvr_emul,
                     Mux(isScalarMove || is_v_mask_insn || vlmul_sign || csr_vlmax < vLen_ecnt, vLen_ecnt, csr_vlmax)))))
    uop.is_rvv      := cs.is_rvv
    uop.v_ls_ew     := cs.v_ls_ew
    when (is_v_ls) {
      uop.mem_size  := Mux(is_v_ls_index, csr_vsew, cs.v_ls_ew)
      uop.mem_signed:= false.B
    }
    uop.v_unmasked  := !cs.uses_vm || inst(VM_BIT)
    uop.vxsat       := false.B
    uop.vconfig     := io.csr_vconfig
    uop.vconfig.vtype.reserved := DontCare
    uop.v_eidx      := 0.U // io.csr_vstart
    //uop.v_eofs      := 0.U
    uop.v_is_split  := cs.can_be_split
    //uop.can_be_split  := cs.can_be_split
    uop.v_split_ecnt:= total_ecnt
    uop.vconfig.vtype.vsew := Mux(is_v_mask_insn, 3.U, vsew)
    //when (io.deq_fire && cs.can_be_split) {
      //assert(cs.is_rvv, "can_be_split applies only to vector instructions.")
    //}
    uop.vs1_eew     := vs1_sew
    uop.vs2_eew     := vs2_sew
    uop.vd_eew      := vd_sew
    uop.vs1_emul    := vs1_emul
    uop.vs2_emul    := vs2_emul
    //uop.vd_emul     := vd_emul
    if(usingMatrix) {
      uop.vd_emul := Mux(cs.uopc.isOneOf(uopMQMV_V), 2.U, Mux(cs.uopc.isOneOf(uopMWMV_V), 1.U, vd_emul))
    }
    //when (cs.is_rvv && !uop.v_unmasked) {
      //when (is_v_ls) {
        //uop.iq_type := IQT_MVEC
        //uop.fu_code := FU_MEMV
      //}
      //uop.frs3_en := true.B
    //}

    uop.v_seg_nf     := Mux(is_v_ls_seg, 1.U(4.W) + vseg_nf, 1.U)
    uop.v_eidx       := 0.U

    when (cs.is_rvv && is_v_mask_ld) {
      //uop.uopc := uopVL
      // make elements >= ceil(vl/8) inactive
      uop.vconfig.vl := (io.csr_vconfig.vl + 7.U) >> 3.U
    }
    when (cs.is_rvv && is_v_mask_st) {
      uop.vconfig.vl := total_ecnt
    }
    when (cs.is_rvv && cs.uopc.isOneOf(uopVLR, uopVMVR)) {
      uop.uopc := Mux(cs.uopc.isOneOf(uopVLR), uopVL, uopVMVR)
      uop.vconfig.vl := total_ecnt
      uop.vl_ready := true.B
    }
    when (cs.is_rvv && cs.uopc === uopVSR) {
      uop.uopc := uopVSA
      uop.vconfig.vl := total_ecnt
      uop.vl_ready := true.B
    }

    // vstart control
    uop.vstart    := 0.U
    uop.vstartSrc := VSTART_ZERO

    //io.enq_stall := cs.can_be_split && !split_last
    io.enq_stall := false.B

    /**
     * vector illegal instruction handler
     */
    // reduction, vpopc, vfirst,vmsbf,vmsif,vmsof,viota,vcompress must execute with vstart=0,otherwise illegal
    val illegal_vstart_not_zero = cs.vstart_is_zero && (io.csr_vstart =/= 0.U)
    //The destination register cannot overlap the source register
    val illegal_vd_unequal_vs1 = cs.vd_unequal_vs1 && (uop.ldst === uop.lrs1)
    val illegal_vd_unequal_vs2 = (cs.vd_unequal_vs2 || cs.uses_ldq && is_v_ls_seg && is_v_ls_index) && (uop.ldst === uop.lrs2)
    //vadc, vsbc, or a masked instruction(except comparison, reduction) , vd overlap v0 will raise illegal exception
    val illegal_vd_overlap_v0 = (cs.uopc.isOneOf(uopVADC, uopVSBC) || cs.uses_vm && !inst(VM_BIT) && !cs.allow_vd_is_v0) &&
                                (uop.ldst === 0.U) && (uop.dst_rtype === RT_VEC)

    //basic SEW and LMUL configuration illegal is judged by CSR module and will set vill
    //vill and instruction depend on vtype, V-FP sew = 8 but not INT_TO_FP instruction
    val illegal_vtype_configure = cs.is_rvv && io.csr_vconfig.vtype.vill && !cs.not_use_vtype
    // illegal floating vsew settings
    val illegal_vsew_vfloat = cs.is_rvv && cs.fp_val && Mux(cs.uopc === uopVFCVT_I2F, vd_sew  === 0.U,
                                                        Mux(cs.uopc === uopVFCVT_F2I, vs2_sew === 0.U,
                                                                                      vs1_sew === 0.U || vs2_sew === 0.U && vd_sew === 0.U))
    //register EEW > 64
    val illegal_reg_sew = !cs.not_use_vtype && (vd_sew(2) || vs2_sew(2) || vs1_sew(2))
    //vd/vs2 EMUL should be illegal
    val vd_emul_legal = Mux(vd_emul(2), vd_emul(1,0) =/= 0.U && ~vd_emul(1,0) < (3.U - vd_sew), true.B)
    val vs1_emul_legal = Mux(vs1_emul(2), vs1_emul(1,0) =/= 0.U && ~vs1_emul(1,0) < 3.U - vs1_sew, true.B)
    val vs2_emul_legal = Mux(vs2_emul(2), vs2_emul(1,0) =/= 0.U && ~vs2_emul(1,0) < 3.U - vs2_sew, true.B)
    val illegal_reg_emul = !vd_emul_legal || !vs1_emul_legal || !vs2_emul_legal

    //reg_num should be multiple of emul, low bit or reg_num !=0 will raise illegal
    val illegal_dst_multiple_emul: Bool = Mux(vd_emul(2), false.B, (((rightOR(UIntToOH(vd_emul(1,0))) >> 1.U).asUInt
                                    & instRD) =/= 0.U) && (uop.dst_rtype === RT_VEC))
    val illegal_vs2_multiple_emul = Mux(vs2_emul(2), false.B, (((rightOR(UIntToOH(vs2_emul(1,0))) >> 1.U).asUInt
                                    & instRS2) =/= 0.U) && (uop.lrs2_rtype === RT_VEC))
    val illegal_vs1_multiple_emul = Mux(vs1_emul(2), false.B, (((rightOR(UIntToOH(vs1_emul(1,0))) >> 1.U).asUInt
                                    & instRS1) =/= 0.U) && (uop.lrs1_rtype === RT_VEC))

    val illegal_regnum_multiple_emul = illegal_dst_multiple_emul || illegal_vs2_multiple_emul || illegal_vs1_multiple_emul

    val illegal_vs2_overlap_vd_lowpart = (uop.ldst === (uop.lrs2 + vs2_emul(1,0))) && uop.rt(RD, isNarrowV)

    illegal_vector_case :=  !vsetvl && (illegal_vstart_not_zero ||
                            illegal_vd_unequal_vs1              ||
                            illegal_vd_unequal_vs2              ||
                            illegal_vd_overlap_v0               ||
                            illegal_vtype_configure             ||
                            illegal_vsew_vfloat                 ||
                            illegal_reg_sew                     ||
                            illegal_reg_emul                    ||
                            illegal_regnum_multiple_emul        ||
                            illegal_vs2_overlap_vd_lowpart)
  } // if usingvector

  // matrix stuff
  if (usingMatrix) {
    val csr_msew = io.csr_mconfig.mtype.msew
    val csr_tilem = io.csr_tilem
    val csr_tilen = io.csr_tilen
    val csr_tilek = io.csr_tilek
    val csr_tsidx = io.csr_tsidx
    val csr_mltr = io.csr_mconfig.mtype.mltr
    val csr_mrtr = io.csr_mconfig.mtype.mrtr

    val is_mls = cs.is_rvm & (cs.uses_ldq | cs.uses_stq)
    val mslice_tt0 = uop.inst(28).asBool()     //1: col,  0: row
    val mslice_dim = uop.inst(27,26)

    val slice_cnt_tilem = (mslice_dim === 1.U &&  csr_mltr) || (mslice_dim === 0.U)
    val slice_cnt_tilen = (mslice_dim === 2.U &&  csr_mrtr)
    val slice_cnt_tilek = (mslice_dim === 1.U && !csr_mltr) || (mslice_dim === 2.U && !csr_mrtr)
    val slice_len_tilem = (mslice_dim === 1.U && !csr_mltr)
    val slice_len_tilen = (mslice_dim === 2.U && !csr_mrtr) || (mslice_dim === 0.U)
    val slice_len_tilek = (mslice_dim === 1.U &&  csr_mltr) || (mslice_dim === 2.U && csr_mrtr)

    val sel_slice_cnt = Mux(slice_cnt_tilem, csr_tilem,
                        Mux(slice_cnt_tilen, csr_tilen, csr_tilek))
    val sel_slice_len = Mux(slice_len_tilem, csr_tilem,
                        Mux(slice_len_tilen, csr_tilen, csr_tilek))

    val msew = Mux(is_mls, cs.v_ls_ew, csr_msew)
    val is_mmv = cs.uopc.isOneOf(uopMMV_T,uopMMV_V,uopMWMV_T,uopMWMV_V,uopMQMV_T,uopMQMV_V)
    val mqwiden = cs.uopc.isOneOf(uopMQMUL,uopMQOPA,uopMQMV_T,uopMQMV_V)
    val mwwiden = cs.uopc.isOneOf(uopMWMUL,uopMWOPA,uopMFWOPA,uopMWMV_T,uopMWMV_V)
    val td_mwfactor = Mux(mqwiden, 2.U, Mux(mwwiden, 1.U, 0.U))
    val td_mnfactor = Mux(cs.uopc === uopMFNCVT, 1.U, 0.U)
    val ts1_mwfactor = Mux(mqwiden && is_mmv, 2.U, Mux(mwwiden && is_mmv, 1.U, 0.U))

    val ts1_eew = msew + ts1_mwfactor
    val ts2_eew = msew
    val td_eew  = Mux(cs.uopc === uopMFNCVT, msew - td_mnfactor, msew + td_mwfactor)
    when (io.deq_fire && cs.is_rvm) {
      assert(msew <= 3.U, "Unsupported msew")
    }

    when (is_mls) {
      uop.mem_size   := cs.v_ls_ew
      uop.mem_signed := false.B
    }

    uop.m_is_split    := cs.can_be_split
    uop.m_slice_cnt   := Mux(is_mls, sel_slice_cnt, csr_tilem)
    uop.m_slice_len   := Mux(is_mls, sel_slice_len, csr_tilek)
    uop.m_tilen       := csr_tilen
    uop.m_sidx        := 0.U
    uop.ts1_eew       := ts1_eew
    uop.ts2_eew       := ts2_eew
    uop.td_eew        := td_eew
    uop.is_rvm        := cs.is_rvm
    uop.m_ls_ew       := cs.v_ls_ew
    uop.mconfig       := io.csr_mconfig

    uop.m_split_first := true.B    //remove split after dispatch
    uop.m_split_last  := true.B
    uop.isHSlice      := !mslice_tt0
    uop.mslice_dim    := mslice_dim
    when (cs.is_rvm && cs.uopc.isOneOf(uopMMV_V, uopMWMV_V, uopMQMV_V)) {
      //uop.dst_rtype := Mux(uop.inst(29).asBool(), RT_TR, RT_ACC)
      uop.lrs1_rtype := Mux(uop.inst(29).asBool(), RT_TR, RT_ACC)
      uop.fu_code   := Mux(uop.inst(28).asBool(), FU_VSLICE, FU_HSLICE)
    }

    //matrix illegal instruction handler

    val illegal_msew = cs.is_rvm && !cs.inst_unique && (td_eew > 2.U || ts1_eew > 2.U || ts2_eew > 2.U)
    illegal_matrix_case := illegal_msew

  } // if usingMatrix
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

class VlWakeupResp(implicit p: Parameters) extends BoomBundle
{
  val vcq_idx = UInt(vcqSz.W)
  val vl = UInt((maxVLMax.log2 + 1).W)
  val vconfig_tag = UInt(vconfigTagSz.W)
  val vconfig_mask = UInt(maxVconfigCount.W)
}

class VconfigDecodeSignals(implicit p: Parameters) extends BoomBundle
{
  val vl_ready = Bool()
  val vconfig    = new VConfig
}

/**
 * Track the current "vconfig mask", and give out the vconfig mask to each micro-op in Decode
 * (each micro-op in the machine has a vconfig mask which says which vconfig it
 * is being speculated under).
 *
 * @param pl_width pipeline width for the processor
 */
class VconfigMaskGenerationLogic(val pl_width: Int) (implicit p: Parameters) extends BoomModule
{
  val io = IO(new Bundle {
    // guess if the uop is a vsetvli/vsetivli (we'll catch this later)
    val is_vconfig = Input(Vec(pl_width, Bool()))
    // lock in that it's actually a vconfig and will fire, so we update
    // the vconfig_masks.
    val will_fire = Input(Vec(pl_width, Bool()))

    // give out tag immediately (needed in rename)
    // mask can come later in the cycle
    val vconfig_tag    = Output(Vec(pl_width, UInt(vconfigTagSz.W)))
    val vconfig_mask   = Output(Vec(pl_width, UInt(maxVconfigCount.W)))

    // tell decoders the vconfig mask has filled up, but on the granularity
    // of an individual micro-op (so some micro-ops can go through)
    val is_full   = Output(Vec(pl_width, Bool()))

    //deadallocate the committed vconfig
    val vconfig_mask_update  = Input(UInt(maxVconfigCount.W))
    val flush_pipeline = Input(Bool())

    val debug_vconfig_mask = Output(UInt(maxVconfigCount.W))
  })

  val vconfig_mask = RegInit(0.U(maxVconfigCount.W))

  //-------------------------------------------------------------
  // Give out the branch tag to each speculative vconfig micro-op

  var allocate_mask = vconfig_mask
  val update_vtag = (io.is_vconfig zip io.will_fire).map{case(v,w) => v && w}.reduce(_||_)
  val tag_masks = Wire(Vec(pl_width, UInt(maxVconfigCount.W)))
  for (w <- 0 until pl_width) {
    io.is_full(w) := (allocate_mask === ~(0.U(maxVconfigCount.W))) && io.is_vconfig(w)

    // find vconfig_tag and compute next vconfig_mask
    val new_vconfig_tag = RegInit(0.U(vconfigTagSz.W))
    //new_vconfig_tag := 0.U
    tag_masks(w) := 0.U

    for (i <- maxVconfigCount - 1 to 0 by -1) {
      when(~allocate_mask(i)) {
        tag_masks(w) := (1.U << i.U)
        when(update_vtag) {
          new_vconfig_tag := new_vconfig_tag + 1.U
        }
      }
    }

    io.vconfig_tag(w) := new_vconfig_tag
    allocate_mask = Mux(io.is_vconfig(w), tag_masks(w) | allocate_mask, allocate_mask)
  }
  //-------------------------------------------------------------
  // Give out the branch mask to each micro-op
  // (kill off the bits that corresponded to branches that aren't going to fire)

  var curr_mask = vconfig_mask
  for (w <- 0 until pl_width) {
    io.vconfig_mask(w) := ~io.vconfig_mask_update & curr_mask
    curr_mask = Mux(io.will_fire(w), tag_masks(w) | curr_mask, curr_mask)
  }
  //-------------------------------------------------------------
  // Update the current vconfig_mask

  when (io.flush_pipeline) {
    vconfig_mask := 0.U
  } .otherwise {
    vconfig_mask := ~io.vconfig_mask_update & curr_mask
  }

  io.debug_vconfig_mask := vconfig_mask
}

case class VcqParameters(
                          nEntries: Int = 4
                        )

/**
 * Queue to store the vconfig info that are inflight in the processor.
 *
 * @param num_entries # of entries in the VCQ
 */
class VconfigQueue(implicit p: Parameters) extends BoomModule
  with HasBoomCoreParameters {
  val num_entries = vcqSz
  private val idx_sz = log2Ceil(num_entries)

  val io = IO(new BoomBundle {
    //Enqueue one entry when decode a vconfig instruction.
    val enq = Flipped(Decoupled(new VconfigDecodeSignals()))
    val enq_idx = Output(UInt(num_entries.W))
    val deq   = Input(Bool())
    val flush = Input(Bool())

    val get_vconfig = Output(new VconfigDecodeSignals())
    val empty = Output(Bool())

    val update_vl_idx = Input(UInt(num_entries.W))
    val update_vl = Flipped(Decoupled(new VconfigDecodeSignals()))

    val vcq_Wcsr = Output(new VconfigDecodeSignals())
  })

  val ram = Reg(Vec(num_entries, new VconfigDecodeSignals()))
  ram.suggestName("vconfig_table")

  val enq_ptr = RegInit(0.U(num_entries.W))
  val deq_ptr = RegInit(0.U(num_entries.W))
  val maybe_full = RegInit(false.B)

  val ptr_match = enq_ptr === deq_ptr
  val empty = ptr_match && !maybe_full
  val full = ptr_match && maybe_full
  val do_enq = WireDefault(io.enq.fire())
  val do_deq = WireDefault(io.deq)

  def inc(ptr: UInt) = {
    val n = ptr.getWidth
    Cat(ptr(n-2,0), ptr(n-1))
  }
  def dec(ptr: UInt) = {
    val n = ptr.getWidth
    Cat(ptr(0), ptr(n-1,1))
  }

  when(do_enq) {
    ram(enq_ptr) := io.enq.bits
    enq_ptr := inc(enq_ptr)
  }
  io.enq_idx := enq_ptr

  when(do_deq && !empty) {
    deq_ptr := inc(deq_ptr)
    io.vcq_Wcsr := ram(deq_ptr)
  }
  when(io.update_vl.valid) {
    ram(io.update_vl_idx).vconfig.vl := io.update_vl.bits.vconfig.vl
    ram(io.update_vl_idx).vl_ready := io.update_vl.bits.vl_ready
    // ram(io.update_vl_idx).vconfig.vtype := io.update_vl.bits.vconfig.vtype
  }
  when(do_enq =/= do_deq) {
    maybe_full := do_enq
  }
  when(io.flush) {
    enq_ptr := 1.U
    deq_ptr := 1.U
    maybe_full := false.B
  }

  io.enq_idx := enq_ptr
  io.enq.ready := !full
  io.get_vconfig := ram(dec(enq_ptr))
  io.empty := empty
}
