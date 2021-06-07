//******************************************************************************
// Copyright (c) 2015 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

package boom.exu

import chisel3._
import chisel3.util._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.tile.FPConstants._
import freechips.rocketchip.tile.{FPUCtrlSigs, HasFPUParameters}
import freechips.rocketchip.tile
import freechips.rocketchip.rocket
import freechips.rocketchip.util.{uintToBitPat, UIntIsOneOf}
import FUConstants._
import boom.common._
import boom.common.MicroOpcodes._
import boom.util.{ImmGenRm, ImmGenRmVSGN, ImmGenTyp, ImmGenTypRVV, CheckF2IRm, CheckF2FRm}

/**
 * FP Decoder for the FPU
 *
 * TODO get rid of this decoder and move into the Decode stage? Or the RRd stage?
 * most of these signals are already created, just need to be translated
 * to the Rocket FPU-speak
 */
class UOPCodeFPUDecoder(vector: Boolean = false)(implicit p: Parameters) extends BoomModule with HasFPUParameters
{
  val io = IO(new Bundle {
    val uopc = Input(Bits(UOPC_SZ.W))
    val sigs = Output(new FPUCtrlSigs())
  })

  // TODO change N,Y,X to BitPat("b1"), BitPat("b0"), and BitPat("b?")
  val N = false.B
  val Y = true.B
  val X = false.B

  val default: List[BitPat] = List(X,X,X,X,X, X,X,X,X,X,X,X, X,X,X,X)

  val f_table: Array[(BitPat, List[BitPat])] =
    // Note: not all of these signals are used or necessary, but we're
    // constrained by the need to fit the rocket.FPU units' ctrl signals.
    //                                     swap12         fma
    //                                     | swap32       | div
    //                                     | | typeTagIn  | | sqrt
    //                          ldst       | | | typeTagOut | | wflags
    //                          | wen      | | | | from_int | | |
    //                          | | ren1   | | | | | to_int | | |
    //                          | | | ren2 | | | | | | fastpipe |
    //                          | | | | ren3 | | | | | |  | | | |
    //                          | | | | |  | | | | | | |  | | | |
    Array(
    BitPat(uopFCLASS_S) -> List(X,X,Y,N,N, N,X,S,S,N,Y,N, N,N,N,N),
    BitPat(uopFMV_S_X)  -> List(X,X,N,N,N, X,X,S,D,Y,N,N, N,N,N,N),
    BitPat(uopFMV_X_S)  -> List(X,X,Y,N,N, N,X,D,S,N,Y,N, N,N,N,N),

    BitPat(uopFCVT_S_X) -> List(X,X,N,N,N, X,X,S,S,Y,N,N, N,N,N,Y),

    BitPat(uopFCVT_X_S) -> List(X,X,Y,N,N, N,X,S,S,N,Y,N, N,N,N,Y),

    BitPat(uopCMPR_S)   -> List(X,X,Y,Y,N, N,N,S,S,N,Y,N, N,N,N,Y),

    BitPat(uopFSGNJ_S)  -> List(X,X,Y,Y,N, N,N,S,S,N,N,Y, N,N,N,N),

    BitPat(uopFMINMAX_S)-> List(X,X,Y,Y,N, N,N,S,S,N,N,Y, N,N,N,Y),

    BitPat(uopFADD_S)   -> List(X,X,Y,Y,N, N,Y,S,S,N,N,N, Y,N,N,Y),
    BitPat(uopFSUB_S)   -> List(X,X,Y,Y,N, N,Y,S,S,N,N,N, Y,N,N,Y),
    BitPat(uopFMUL_S)   -> List(X,X,Y,Y,N, N,N,S,S,N,N,N, Y,N,N,Y),
    BitPat(uopFMADD_S)  -> List(X,X,Y,Y,Y, N,N,S,S,N,N,N, Y,N,N,Y),
    BitPat(uopFMSUB_S)  -> List(X,X,Y,Y,Y, N,N,S,S,N,N,N, Y,N,N,Y),
    BitPat(uopFNMADD_S) -> List(X,X,Y,Y,Y, N,N,S,S,N,N,N, Y,N,N,Y),
    BitPat(uopFNMSUB_S) -> List(X,X,Y,Y,Y, N,N,S,S,N,N,N, Y,N,N,Y)
    )

  val d_table: Array[(BitPat, List[BitPat])] =
    Array(
    BitPat(uopFCLASS_D) -> List(X,X,Y,N,N, N,X,D,D,N,Y,N, N,N,N,N),
    BitPat(uopFMV_D_X)  -> List(X,X,N,N,N, X,X,D,D,Y,N,N, N,N,N,N),
    BitPat(uopFMV_X_D)  -> List(X,X,Y,N,N, N,X,D,D,N,Y,N, N,N,N,N),
    BitPat(uopFCVT_S_D) -> List(X,X,Y,N,N, N,X,D,S,N,N,Y, N,N,N,Y),
    BitPat(uopFCVT_D_S) -> List(X,X,Y,N,N, N,X,S,D,N,N,Y, N,N,N,Y),

    BitPat(uopFCVT_D_X) -> List(X,X,N,N,N, X,X,D,D,Y,N,N, N,N,N,Y),

    BitPat(uopFCVT_X_D) -> List(X,X,Y,N,N, N,X,D,D,N,Y,N, N,N,N,Y),

    BitPat(uopCMPR_D)   -> List(X,X,Y,Y,N, N,N,D,D,N,Y,N, N,N,N,Y),

    BitPat(uopFSGNJ_D)  -> List(X,X,Y,Y,N, N,N,D,D,N,N,Y, N,N,N,N),

    BitPat(uopFMINMAX_D)-> List(X,X,Y,Y,N, N,N,D,D,N,N,Y, N,N,N,Y),

    BitPat(uopFADD_D)   -> List(X,X,Y,Y,N, N,Y,D,D,N,N,N, Y,N,N,Y),
    BitPat(uopFSUB_D)   -> List(X,X,Y,Y,N, N,Y,D,D,N,N,N, Y,N,N,Y),
    BitPat(uopFMUL_D)   -> List(X,X,Y,Y,N, N,N,D,D,N,N,N, Y,N,N,Y),

    BitPat(uopFMADD_D)  -> List(X,X,Y,Y,Y, N,N,D,D,N,N,N, Y,N,N,Y),
    BitPat(uopFMSUB_D)  -> List(X,X,Y,Y,Y, N,N,D,D,N,N,N, Y,N,N,Y),
    BitPat(uopFNMADD_D) -> List(X,X,Y,Y,Y, N,N,D,D,N,N,N, Y,N,N,Y),
    BitPat(uopFNMSUB_D) -> List(X,X,Y,Y,Y, N,N,D,D,N,N,N, Y,N,N,Y)
    )

    //                                     swap12         fma
    //                                     | swap32       | div
    //                                     | | typeTagIn  | | sqrt
    //                          ldst       | | | typeTagOut | | wflags
    //                          | wen      | | | | from_int | | |
    //                          | | ren1   | | | | | to_int | | |
    //                          | | | ren2 | | | | | | fastpipe |
    //                          | | | | ren3 | | | | | |  | | | |
    //                          | | | | |  | | | | | | |  | | | |
  val v_table: Array[(BitPat, List[BitPat])] =
    Array(
    BitPat(uopVFADD)    -> List(X,X,Y,Y,N, N,Y,D,D,N,N,N, Y,N,N,Y)
   ,BitPat(uopVFSUB)    -> List(X,X,Y,Y,N, N,Y,D,D,N,N,N, Y,N,N,Y)
   ,BitPat(uopVFRSUB)   -> List(X,X,Y,Y,N, N,Y,D,D,N,N,N, Y,N,N,Y)
   ,BitPat(uopVFMUL)    -> List(X,X,Y,Y,N, N,N,D,D,N,N,N, Y,N,N,Y)
   ,BitPat(uopVFMACC)   -> List(X,X,Y,Y,Y, N,N,D,D,N,N,N, Y,N,N,Y)
   ,BitPat(uopVFNMACC)  -> List(X,X,Y,Y,Y, N,N,D,D,N,N,N, Y,N,N,Y)
   ,BitPat(uopVFMSAC)   -> List(X,X,Y,Y,Y, N,N,D,D,N,N,N, Y,N,N,Y)
   ,BitPat(uopVFNMSAC)  -> List(X,X,Y,Y,Y, N,N,D,D,N,N,N, Y,N,N,Y)
   ,BitPat(uopVFMADD)   -> List(X,X,Y,Y,Y, N,N,D,D,N,N,N, Y,N,N,Y)
   ,BitPat(uopVFNMADD)  -> List(X,X,Y,Y,Y, N,N,D,D,N,N,N, Y,N,N,Y)
   ,BitPat(uopVFMSUB)   -> List(X,X,Y,Y,Y, N,N,D,D,N,N,N, Y,N,N,Y)
   ,BitPat(uopVFNMSUB)  -> List(X,X,Y,Y,Y, N,N,D,D,N,N,N, Y,N,N,Y)
   ,BitPat(uopVFMIN)    -> List(X,X,Y,Y,N, N,N,D,D,N,N,Y, N,N,N,Y)
   ,BitPat(uopVFMAX)    -> List(X,X,Y,Y,N, N,N,D,D,N,N,Y, N,N,N,Y)
   ,BitPat(uopVFMV_V_F) -> List(X,X,Y,N,N, N,Y,D,D,N,N,N, Y,N,N,N)
   ,BitPat(uopVFSGNJ)   -> List(X,X,Y,Y,N, Y,N,D,D,N,N,Y, N,N,N,N)
   ,BitPat(uopVFCLASS)  -> List(X,X,Y,N,N, Y,X,D,D,N,Y,N, N,N,N,N)
   ,BitPat(uopVFCVT_F2I)-> List(X,X,Y,N,N, Y,X,D,D,N,Y,N, N,N,N,Y)
   ,BitPat(uopVFCVT_I2F)-> List(X,X,N,N,N, Y,X,D,D,Y,N,N, N,N,N,Y)
   ,BitPat(uopVFCVT_F2F)-> List(X,X,Y,N,N, Y,X,D,D,N,N,Y, N,N,N,Y)
    )

//   val insns = fLen match {
//      case 32 => f_table
//      case 64 => f_table ++ d_table
//   }
  val insns = if (vector) v_table else f_table ++ d_table
  val decoder = rocket.DecodeLogic(io.uopc, default, insns)

  val s = io.sigs
  val sigs = Seq(s.ldst, s.wen, s.ren1, s.ren2, s.ren3, s.swap12,
                 s.swap23, s.typeTagIn, s.typeTagOut, s.fromint, s.toint, s.fastpipe, s.fma,
                 s.div, s.sqrt, s.wflags)
  sigs zip decoder map {case(s,d) => s := d}
}

/**
 * FP fused multiple add decoder for the FPU
 */
class FMADecoder(vector: Boolean = false) extends Module
{
  val io = IO(new Bundle {
    val uopc = Input(UInt(UOPC_SZ.W))
    val cmd = Output(UInt(2.W))
  })

  val default: List[BitPat] = List(BitPat("b??"))
  val table: Array[(BitPat, List[BitPat])] = if (vector) {
    Array(
      BitPat(uopVFADD)    -> List(BitPat("b00"))
     ,BitPat(uopVFSUB)    -> List(BitPat("b10"))
     ,BitPat(uopVFRSUB)   -> List(BitPat("b01"))
     ,BitPat(uopVFMUL)    -> List(BitPat("b00"))
     ,BitPat(uopVFMACC)   -> List(BitPat("b00"))
     ,BitPat(uopVFNMACC)  -> List(BitPat("b11"))
     ,BitPat(uopVFMSAC)   -> List(BitPat("b01"))
     ,BitPat(uopVFNMSAC)  -> List(BitPat("b10"))
     ,BitPat(uopVFMADD)   -> List(BitPat("b00"))
     ,BitPat(uopVFNMADD)  -> List(BitPat("b11"))
     ,BitPat(uopVFMSUB)   -> List(BitPat("b01"))
     ,BitPat(uopVFNMSUB)  -> List(BitPat("b10"))
     ,BitPat(uopVFMV_V_F) -> List(BitPat("b00"))
    )
  } else {
    Array(
      BitPat(uopFADD_S)   -> List(BitPat("b00")),
      BitPat(uopFSUB_S)   -> List(BitPat("b01")),
      BitPat(uopFMUL_S)   -> List(BitPat("b00")),
      BitPat(uopFMADD_S)  -> List(BitPat("b00")),
      BitPat(uopFMSUB_S)  -> List(BitPat("b01")),
      BitPat(uopFNMADD_S) -> List(BitPat("b11")),
      BitPat(uopFNMSUB_S) -> List(BitPat("b10")),
      BitPat(uopFADD_D)   -> List(BitPat("b00")),
      BitPat(uopFSUB_D)   -> List(BitPat("b01")),
      BitPat(uopFMUL_D)   -> List(BitPat("b00")),
      BitPat(uopFMADD_D)  -> List(BitPat("b00")),
      BitPat(uopFMSUB_D)  -> List(BitPat("b01")),
      BitPat(uopFNMADD_D) -> List(BitPat("b11")),
      BitPat(uopFNMSUB_D) -> List(BitPat("b10"))
      )
  }

  val decoder = rocket.DecodeLogic(io.uopc, default, table)

  val (cmd: UInt) :: Nil = decoder
  io.cmd := cmd
}

/**
 * Bundle representing data to be sent to the FPU
 */
class FpuReq()(implicit p: Parameters) extends BoomBundle
{
  val uop      = new MicroOp()
  val rs1_data = Bits(65.W)
  val rs2_data = Bits(65.W)
  val rs3_data = Bits(65.W)
  val fcsr_rm  = Bits(tile.FPConstants.RM_SZ.W)
}

/**
 * FPU unit that wraps the RocketChip FPU units (which in turn wrap hardfloat)
 */
class FPU(vector: Boolean = false)(implicit p: Parameters) extends BoomModule with tile.HasFPUParameters
{
  val io = IO(new Bundle {
    val req = Flipped(new ValidIO(new FpuReq))
    val resp = new ValidIO(new ExeUnitResp(65))
  })

  // all FP units are padded out to the same latency for easy scheduling of the write port
  val fpu_latency = dfmaLatency
  val io_req = io.req.bits

  val fp_decoder = Module(new UOPCodeFPUDecoder(vector))
  fp_decoder.io.uopc := io_req.uop.uopc
  val fp_ctrl = WireInit(fp_decoder.io.sigs)
  //val fp_rm = Mux(ImmGenRm(io_req.uop.imm_packed) === 7.U || io_req.uop.is_rvv, io_req.fcsr_rm, ImmGenRm(io_req.uop.imm_packed))
  val fp_rm = Mux(ImmGenRm(io_req.uop.imm_packed) === 7.U, io_req.fcsr_rm, ImmGenRm(io_req.uop.imm_packed))
  val rs1_data = WireInit(io_req.rs1_data)
  val rs2_data = WireInit(io_req.rs2_data)
  val rs3_data = WireInit(io_req.rs3_data)
  val vd_widen = io_req.uop.rt(RD , isWidenV)
  val vs2_widen= io_req.uop.rt(RS2, isWidenV)
  if (vector) {
    val vsew = io_req.uop.vconfig.vtype.vsew
    val vd_sew  = Mux(vd_widen,  vsew+1.U, vsew)
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
    rs1_data := recode(io_req.rs1_data, vs1_fmt)
    rs2_data := recode(io_req.rs2_data, vs2_fmt)
    rs3_data := recode(io_req.rs3_data, vd_fmt)
    when (io_req.uop.is_rvv) {
      fp_ctrl.typeTagIn := Mux(fp_ctrl.swap12, vs2_fmt, vs1_fmt)
      fp_ctrl.typeTagOut:= vd_fmt
    }
  }

  // FIXME: S->H widening operation must be fixed
  def fuInput(minT: Option[tile.FType], vector: Boolean = false): tile.FPInput = {
    val req     = Wire(new tile.FPInput)
    val tagIn   = fp_ctrl.typeTagIn
    val tag     = fp_ctrl.typeTagOut
    val vs2_tag = Mux(vector.B && (vd_widen ^ vs2_widen), tagIn, tag)
    req <> fp_ctrl
    req.rm := Mux(~io_req.uop.is_rvv, fp_rm, 
              Mux(io_req.uop.fu_code_is(FU_F2I) && CheckF2IRm(io_req.uop.imm_packed), 1.U, 
              Mux(io_req.uop.uopc === uopVFCLASS, 1.U,
              Mux(io_req.uop.uopc === uopVFSGNJ,  ImmGenRmVSGN(io_req.uop.imm_packed),
              Mux(io_req.uop.uopc === uopVFCVT_F2F && CheckF2FRm(io_req.uop.imm_packed), 6.U,
              io_req.fcsr_rm)))))
    val unbox_rs1 = Mux(vector.B && vd_widen,               unbox(rs1_data, tagIn,   None), unbox(rs1_data, tagIn, minT))
    val unbox_rs2 = Mux(vector.B && (vd_widen ^ vs2_widen), unbox(rs2_data, vs2_tag, None), unbox(rs2_data, vs2_tag, minT))
    val unbox_rs3 = unbox(rs3_data, tag,      minT)
    req.in1 := unbox_rs1
    req.in2 := unbox_rs2
    req.in3 := unbox_rs3
    // e.g. vfcvt.x.f.v   vd, vs2, vm
    // e.g. vfsgnj.vv vd, vs2, vs1, vm
    when (fp_ctrl.swap12) { 
      req.in1 := unbox_rs2
      req.in2 := unbox_rs1
    }
    when (fp_ctrl.swap23) { req.in3 := unbox_rs2 }
    //req.typ := ImmGenTyp(io_req.uop.imm_packed)
    val typ1 = Mux(tag === D, 1.U(1.W), 0.U(1.W))
    req.typ := Mux(io_req.uop.is_rvv, ImmGenTypRVV(typ1, io_req.uop.imm_packed), ImmGenTyp(io_req.uop.imm_packed)) // typ of F2I and I2F
    req.fmt := Mux(tag === H, 2.U, Mux(tag === S, 0.U, 1.U)) // TODO support Zfh and avoid special-case below
    when (io_req.uop.uopc === uopFMV_X_S) {
      req.fmt := 0.U
    } .elsewhen (io_req.uop.uopc.isOneOf(uopVFMADD,uopVFNMADD,uopVFMSUB,uopVFNMSUB)) {
        req.in2 := unbox_rs3
        req.in3 := unbox_rs2
    }

    val fma_decoder = Module(new FMADecoder(vector))
    fma_decoder.io.uopc := io_req.uop.uopc
    req.fmaCmd := fma_decoder.io.cmd // ex_reg_inst(3,2) | (!fp_ctrl.ren3 && ex_reg_inst(27))
    req
  }

  val dfma = Module(new tile.FPUFMAPipe(latency = fpu_latency, t = tile.FType.D))
  dfma.io.in.bits := fuInput(Some(dfma.t), vector)

  val sfma = Module(new tile.FPUFMAPipe(latency = fpu_latency, t = tile.FType.S))
  sfma.io.in.bits := fuInput(Some(sfma.t), vector)

  val hfma = Module(new tile.FPUFMAPipe(latency = fpu_latency, t = tile.FType.H))
  hfma.io.in.bits := fuInput(Some(hfma.t), vector)

  val fpiu = Module(new tile.FPToInt)
  fpiu.io.in.bits := fuInput(None, vector)

  val fpiu_out = Pipe(RegNext(fpiu.io.in.valid && !fp_ctrl.fastpipe),
                              fpiu.io.out.bits, fpu_latency-1)
  val fpiu_result  = Wire(new tile.FPResult)
  fpiu_result.data := fpiu_out.bits.toint
  fpiu_result.exc  := fpiu_out.bits.exc

  val fpmu = Module(new tile.FPToFP(fpu_latency)) // latency 2 for rocket
  fpmu.io.in.bits := fpiu.io.in.bits
  fpmu.io.lt := fpiu.io.out.bits.lt
  val fpmu_double = Pipe(io.req.valid && fp_ctrl.fastpipe,
                         (if (vector) fp_ctrl.typeTagOut else fp_ctrl.typeTagOut === D),
                         fpu_latency).bits

  if (vector) {
    dfma.io.in.valid := io.req.valid && fp_ctrl.fma && (fp_ctrl.typeTagOut === D) //&& (!io_req.uop.is_rvv || io_req.uop.v_active)
    sfma.io.in.valid := io.req.valid && fp_ctrl.fma && (fp_ctrl.typeTagOut === S) //&& (!io_req.uop.is_rvv || io_req.uop.v_active)
    hfma.io.in.valid := io.req.valid && fp_ctrl.fma && (fp_ctrl.typeTagOut === H) //&& (!io_req.uop.is_rvv || io_req.uop.v_active)
    fpiu.io.in.valid := io.req.valid && (fp_ctrl.toint || (fp_ctrl.fastpipe && fp_ctrl.wflags)) //&& (!io_req.uop.is_rvv || io_req.uop.v_active)
    fpmu.io.in.valid := io.req.valid && fp_ctrl.fastpipe //&& (!io_req.uop.is_rvv || io_req.uop.v_active)
    // inactive elements are handled through vector integer path
//  when (io_req.uop.is_rvv && !io_req.uop.v_active) {
//    fpmu.io.in.bits.in1 := fpiu.io.in.bits.in3
//  }
  } else {
    dfma.io.in.valid := io.req.valid && fp_ctrl.fma && (fp_ctrl.typeTagOut === D)
    sfma.io.in.valid := io.req.valid && fp_ctrl.fma && (fp_ctrl.typeTagOut === S)
    hfma.io.in.valid := io.req.valid && fp_ctrl.fma && (fp_ctrl.typeTagOut === H)
    fpiu.io.in.valid := io.req.valid && (fp_ctrl.toint || (fp_ctrl.fastpipe && fp_ctrl.wflags))
    fpmu.io.in.valid := io.req.valid && fp_ctrl.fastpipe
  }

  // Response (all FP units have been padded out to the same latency)
  io.resp.valid := fpiu_out.valid ||
                   fpmu.io.out.valid ||
                   hfma.io.out.valid ||
                   sfma.io.out.valid ||
                   dfma.io.out.valid

  val fpu_out_data =
      Mux(dfma.io.out.valid, box(dfma.io.out.bits.data, D),
      Mux(sfma.io.out.valid, box(sfma.io.out.bits.data, S),
      Mux(hfma.io.out.valid, box(hfma.io.out.bits.data, H),
      Mux(fpiu_out.valid,    fpiu_result.data,
                             box(fpmu.io.out.bits.data, fpmu_double)))))

  val fpu_out_exc =
    Mux(dfma.io.out.valid, dfma.io.out.bits.exc,
    Mux(sfma.io.out.valid, sfma.io.out.bits.exc,
    Mux(hfma.io.out.valid, hfma.io.out.bits.exc,
    Mux(fpiu_out.valid,    fpiu_result.exc,
                           fpmu.io.out.bits.exc))))

  if (vector) {
    io.resp.bits.data := Mux(fpiu_out.valid, fpu_out_data, ieee(fpu_out_data))   // fpiu_out outputs integer numbers
  } else {
    io.resp.bits.data := fpu_out_data
  }
  io.resp.bits.fflags.valid      := io.resp.valid
  io.resp.bits.fflags.bits.flags := fpu_out_exc
}
