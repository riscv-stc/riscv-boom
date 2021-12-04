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
import freechips.rocketchip.rocket.{CSR, Causes, VConfig, VType}
import freechips.rocketchip.util.{UIntIsOneOf, rightOR, uintToBitPat}
import FUConstants._
import boom.common._
import boom.common.MicroOpcodes._
import boom.util._

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
  //     |  |  |  |     |        |     regtype      |      |  |     |  |  |  |  |  cmd  |    |  |  |  |  flush on commit |  |       |  |  |  |
  //     |  |  |  |     |        |     |     |      |      |  |     |  |  |  |  |  |    |    |  |  |  |  |  csr cmd   |  |  |    vstart_is_zero
  //     |  |  |  |     |        |     |     |      |      |  |     |  |  |  |  |  |    |    |  |  |  |  |  |      |  |  |  |    |  |  |  |  |
    List(N, N, X, uopX, IQT_INT, FU_X, RT_X, RT_DC, RT_DC, X, IS_X, X, X, X, X, N, M_X, DC2, X, X, N, N, X, CSR.X, N, N, N, DC2, X, X, X, X, X)

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

  def decode(inst: UInt, table: Iterable[(BitPat, List[BitPat])]) = {
    //val decoder = freechips.rocketchip.rocket.DecodeLogic(inst, decode_default, table)
    val decoder = BoomDecoder(inst, decode_default, table)
    val sigs =
      Seq(legal, fp_val, fp_single, uopc, iq_type, fu_code, dst_type, rs1_type,
          rs2_type, frs3_en, imm_sel, uses_ldq, uses_stq, is_amo,
          is_fence, is_fencei, mem_cmd, wakeup_delay, bypassable,
          is_br, is_sys_pc2epc, inst_unique, flush_on_commit, csr_cmd,
          is_rvv, can_be_split, uses_vm, v_ls_ew, vstart_is_zero,
          allow_vd_is_v0, not_use_vtype, vd_unequal_vs1, vd_unequal_vs2)
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
  val illegal_vector_case = Wire(Bool())
  val id_illegal_insn = !cs_legal ||
    cs.fp_val && io.csr_decode.fp_illegal || // TODO check for illegal rm mode: (io.fpu.illegal_rm)
    cs.is_rvv && (io.csr_decode.vector_illegal || illegal_vector_case) ||
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

  uop.ldst_val   := isSomeReg(cs.dst_type) && !(uop.ldst === 0.U && uop.rt(RD, isInt))
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
    val csr_vsew  = io.csr_vconfig.vtype.vsew
    val csr_vlmax = io.csr_vconfig.vtype.vlMax
    val is_v_ls = cs.is_rvv & (cs.uses_stq | cs.uses_ldq)
    val isVMVR: Bool = cs.uopc.isOneOf(uopVMVR)
    val is_v_ls_index = cs.uopc.isOneOf(uopVLUX, uopVSUXA, uopVLOX, uopVSOXA)
    val is_v_mask_ld = cs.uopc.isOneOf(uopVLM)
    val is_v_mask_st = cs.uopc.isOneOf(uopVSMA)
    val is_v_reg_ls = cs.uopc.isOneOf(uopVLR, uopVSR)
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
                     Mux(uop.rt(RD,  isWidenV ), csr_vsew + vd_wfactor,
                     Mux(uop.rt(RD,  isNarrowV), csr_vsew - vd_nfactor, csr_vsew)))
    val vs1_sew    = Mux(cs.uopc === uopVRGATHEREI16, 1.U,
                     Mux(uop.rt(RS1, isWidenV ), csr_vsew + vd_wfactor, csr_vsew))
    val vs2_sew    = Mux(is_v_ls_index, Cat(0.U(1.W), cs.v_ls_ew),
                     Mux(uop.rt(RS2, isWidenV ), csr_vsew + vs2_wfactor,
                     Mux(uop.rt(RS2, isNarrowV), csr_vsew - vs2_nfactor, csr_vsew)))
    val vlmul_value =Cat(vlmul_sign, vlmul_mag)
    val vd_emul    = Mux(isVMVR, instRS1(2,0),
                     Mux(uop.rt(RD, isReduceV) || uop.rt(RD, isMaskVD), 0.U,
                     Mux(uop.rt(RD, isWidenV), vlmul_value + vd_wfactor,
                     Mux(uop.rt(RD, isNarrowV), vlmul_value - vd_nfactor,
                     Mux(is_v_ls && !is_v_ls_index, cs.v_ls_ew - vsew + vlmul_value,
                     Mux(cs.allow_vd_is_v0, 0.U, vlmul_value))))))
    val vs1_emul   = Mux(cs.uopc === uopVRGATHEREI16, vlmul_value + 1.U - vsew,
                     Mux(uop.rt(RS1, isReduceV), 0.U,
                     Mux(uop.rt(RS1, isWidenV ), vlmul_value + 1.U, vlmul_value)))
    val vs2_emul   = Mux(uop.rt(RS2, isWidenV), vlmul_value + vs2_wfactor,
                     Mux(uop.rt(RS2, isNarrowV), vlmul_value - vs2_nfactor, vlmul_value))
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

    val vLen_ecnt  = Mux(!is_v_ls && vs2_sew > vd_sew, vLen.U >> (vs2_sew+3.U), vLen.U >> (vd_sew+3.U))
    val split_ecnt = Mux(is_v_mask_insn, vmlogic_split_ecnt, vLen_ecnt)
    // for store, we can skip inactive locations; otherwise, we have to visit every element
    // for fractional lmul, we need visit at least one entire vreg
    // for undisturbing move before reduction, we need visit at most one vreg
    val total_ecnt = Mux(is_v_mask_ld, vLenb.U,
                     Mux(is_v_reg_ls, vLenb.U << Log2(vreg_nf + 1.U) >> cs.v_ls_ew,
                     Mux(cs.uses_stq, Mux(is_v_mask_st, (io.csr_vconfig.vl + 7.U) >> 3.U, io.csr_vconfig.vl),
                     Mux(is_v_mask_insn, vmlogic_tolal_ecnt,
                     Mux(vlmul_sign || csr_vlmax < vLen_ecnt, vLen_ecnt, csr_vlmax)))))
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
    uop.vd_emul     := vd_emul

    //when (cs.is_rvv && !uop.v_unmasked) {
      //when (is_v_ls) {
        //uop.iq_type := IQT_MVEC
        //uop.fu_code := FU_MEMV
      //}
      //uop.frs3_en := true.B
    //}

    uop.v_seg_nf     := Mux(is_v_ls_seg, 1.U(4.W) + vseg_nf, 1.U)
    uop.v_idx_ls     := is_v_ls_index
    uop.v_eidx       := 0.U

    when (cs.is_rvv && is_v_mask_ld) {
      uop.uopc := uopVL
      // make elements >= ceil(vl/8) inactive
      uop.vconfig.vl := (io.csr_vconfig.vl + 7.U) >> 3.U
    }
    when (cs.is_rvv && is_v_mask_st) {
      uop.uopc := uopVSA
    }
    ////when (cs.is_rvv && cs.uopc === uopVLR) {
      //uop.uopc := uopVL
    //}
    //when (cs.is_rvv && cs.uopc === uopVSR) {
      //uop.uopc := uopVSA
    //}

    uop.v_xls_offset := 0.U
    // vstart control
    uop.vstart    := 0.U
    uop.vstartSrc := VSTART_ZERO
    uop.vl_mov    := false.B

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
    val illegal_vtype_configure = cs.is_rvv && ((io.csr_vconfig.vtype.vill && !cs.not_use_vtype ||
                                                (csr_vsew === 0.U) && cs.fp_val && cs.uopc =/= uopVFCVT_I2F))
    //register EEW > 64
    val illegal_reg_sew = cs.not_use_vtype && (vd_sew(2) || vs2_sew(2) || vs1_sew(2))
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
                                illegal_vd_unequal_vs1  ||
                                illegal_vd_unequal_vs2  ||
                                illegal_vd_overlap_v0   ||
                               // illegal_vtype_configure ||
                                illegal_reg_sew         ||
                                illegal_reg_emul        ||
                                illegal_regnum_multiple_emul ||
                                illegal_vs2_overlap_vd_lowpart)
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
