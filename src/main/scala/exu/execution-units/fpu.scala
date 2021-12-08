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
import freechips.rocketchip.util.{uintToBitPat, UIntIsOneOf, ShouldBeRetimed}
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

  val h_table: Array[(BitPat, List[BitPat])] =
    Array(
    BitPat(uopFCLASS_H) -> List(X,X,Y,N,N, N,X,H,H,N,Y,N, N,N,N,N),
    BitPat(uopFMV_H_X)  -> List(X,X,N,N,N, X,X,H,D,Y,N,N, N,N,N,N),
    BitPat(uopFMV_X_H)  -> List(X,X,Y,N,N, N,X,D,H,N,Y,N, N,N,N,N),

    BitPat(uopFCVT_H_S) -> List(X,X,Y,N,N, N,X,S,H,N,N,Y, N,N,N,Y),
    BitPat(uopFCVT_H_D) -> List(X,X,Y,N,N, N,X,D,H,N,N,Y, N,N,N,Y),
    BitPat(uopFCVT_S_H) -> List(X,X,Y,N,N, N,X,H,S,N,N,Y, N,N,N,Y),
    BitPat(uopFCVT_D_H) -> List(X,X,Y,N,N, N,X,H,D,N,N,Y, N,N,N,Y),

    BitPat(uopFCVT_H_W) -> List(X,X,N,N,N, X,X,S,H,Y,N,N, N,N,N,Y),
    BitPat(uopFCVT_H_L) -> List(X,X,N,N,N, X,X,D,H,Y,N,N, N,N,N,Y),

    BitPat(uopFCVT_W_H) -> List(X,X,Y,N,N, N,X,H,S,N,Y,N, N,N,N,Y),
    BitPat(uopFCVT_L_H) -> List(X,X,Y,N,N, N,X,H,D,N,Y,N, N,N,N,Y),

    BitPat(uopCMPR_H)   -> List(X,X,Y,Y,N, N,N,H,H,N,Y,N, N,N,N,Y),

    BitPat(uopFSGNJ_H)  -> List(X,X,Y,Y,N, N,N,H,H,N,N,Y, N,N,N,N),

    BitPat(uopFMINMAX_H)-> List(X,X,Y,Y,N, N,N,H,H,N,N,Y, N,N,N,Y),

    BitPat(uopFADD_H)   -> List(X,X,Y,Y,N, N,Y,H,H,N,N,N, Y,N,N,Y),
    BitPat(uopFSUB_H)   -> List(X,X,Y,Y,N, N,Y,H,H,N,N,N, Y,N,N,Y),
    BitPat(uopFMUL_H)   -> List(X,X,Y,Y,N, N,N,H,H,N,N,N, Y,N,N,Y),

    BitPat(uopFMADD_H)  -> List(X,X,Y,Y,Y, N,N,H,H,N,N,N, Y,N,N,Y),
    BitPat(uopFMSUB_H)  -> List(X,X,Y,Y,Y, N,N,H,H,N,N,N, Y,N,N,Y),
    BitPat(uopFNMADD_H) -> List(X,X,Y,Y,Y, N,N,H,H,N,N,N, Y,N,N,Y),
    BitPat(uopFNMSUB_H) -> List(X,X,Y,Y,Y, N,N,H,H,N,N,N, Y,N,N,Y)
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
   ,BitPat(uopVMFEQ)    -> List(X,X,Y,Y,N, N,N,D,D,N,Y,N, N,N,N,Y)
   ,BitPat(uopVMFNE)    -> List(X,X,Y,Y,N, N,N,D,D,N,Y,N, N,N,N,Y)
   ,BitPat(uopVMFLT)    -> List(X,X,Y,Y,N, Y,N,D,D,N,Y,N, N,N,N,Y)
   ,BitPat(uopVMFLE)    -> List(X,X,Y,Y,N, Y,N,D,D,N,Y,N, N,N,N,Y)
   ,BitPat(uopVMFGT)    -> List(X,X,Y,Y,N, N,N,D,D,N,Y,N, N,N,N,Y)
   ,BitPat(uopVMFGE)    -> List(X,X,Y,Y,N, N,N,D,D,N,Y,N, N,N,N,Y)
   ,BitPat(uopVFCLASS)  -> List(X,X,Y,N,N, Y,X,D,D,N,Y,N, N,N,N,N)
   ,BitPat(uopVFCVT_F2I)-> List(X,X,Y,N,N, Y,X,D,D,N,Y,N, N,N,N,Y)
   ,BitPat(uopVFCVT_I2F)-> List(X,X,N,N,N, Y,X,D,D,Y,N,N, N,N,N,Y)
   ,BitPat(uopVFCVT_F2F)-> List(X,X,Y,N,N, Y,X,D,D,N,N,Y, N,N,N,Y)
   ,BitPat(uopVFMV_S_F) -> List(X,X,Y,N,N, N,Y,D,D,N,N,N, Y,N,N,N)
    )

//   val insns = fLen match {
//      case 32 => f_table
//      case 64 => f_table ++ d_table
//   }
  val insns = if (vector) v_table else f_table ++ d_table ++ h_table
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
     ,BitPat(uopVFMV_F_S) -> List(BitPat("b00"))
     ,BitPat(uopVFMV_S_F) -> List(BitPat("b00"))
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
class FpuReq(val vector: Boolean = false)(implicit p: Parameters) extends BoomBundle
{
  val uop      = new MicroOp()
  val rs1_data = if (vector) Bits(vLen.W) else Bits(65.W)
  val rs2_data = if (vector) Bits(vLen.W) else Bits(65.W)
  val rs3_data = if (vector) Bits(vLen.W) else Bits(65.W)
  val rvm_data = if (vector) Bits((vLen/16).W) else null
  val fcsr_rm  = Bits(tile.FPConstants.RM_SZ.W)
}

/**
 * FPU unit that wraps the RocketChip FPU units (which in turn wrap hardfloat)
 */
class FPU(vector: Boolean = false)(implicit p: Parameters) extends BoomModule with tile.HasFPUParameters
{
  val io = IO(new Bundle {
    val req = Flipped(new ValidIO(new FpuReq(false)))
    val resp = new ValidIO(new ExeUnitResp(65))
  })

  // all FP units are padded out to the same latency for easy scheduling of the write port
  val fpu_latency = dfmaLatency
  val io_req = io.req.bits

  val fp_decoder = Module(new UOPCodeFPUDecoder(vector))
  fp_decoder.io.uopc := io_req.uop.uopc
  val fp_ctrl = WireInit(fp_decoder.io.sigs)
  val fp_rm = if (usingVector) Mux(ImmGenRm(io_req.uop.imm_packed) === 7.U || io_req.uop.is_rvv, io_req.fcsr_rm, ImmGenRm(io_req.uop.imm_packed))
              else             Mux(ImmGenRm(io_req.uop.imm_packed) === 7.U, io_req.fcsr_rm, ImmGenRm(io_req.uop.imm_packed))
  val rs1_data = WireInit(io_req.rs1_data)
  val rs2_data = WireInit(io_req.rs2_data)
  val rs3_data = WireInit(io_req.rs3_data)
  val vd_widen = io_req.uop.rt(RD , isWidenV)
  val vs1_widen= io_req.uop.rt(RS1, isWidenV)
  val vs2_widen= io_req.uop.rt(RS2, isWidenV)
  if (vector) {
    val vsew = io_req.uop.vconfig.vtype.vsew
    val vd_sew  = Mux(vd_widen,  vsew+1.U, vsew)
    val vs1_sew = Mux(vs1_widen, vsew+1.U, vsew)
    val vs2_sew = Mux(vs2_widen, vsew+1.U, vsew)
    val vd_fmt  = Mux(vd_sew  === 3.U, D, Mux(vd_sew  === 2.U, S, H))
    val vs1_fmt = Mux(vs1_sew === 3.U, D, Mux(vs1_sew === 2.U, S, H))
    val vs2_fmt = Mux(vs2_sew === 3.U, D, Mux(vs2_sew === 2.U, S, H))
    when (io.req.valid && io_req.uop.is_rvv) {
      assert(io_req.uop.fp_val, "unexpected fp_val")
      assert(io_req.uop.v_active, "unexpected inactive split")
      assert(vsew <= 3.U, "unsupported vsew")
      assert(vd_sew >= 1.U && vd_sew <= 3.U, "unsupported vd_sew")
    }
    rs1_data := recode(io_req.rs1_data, vs1_fmt)
    rs2_data := recode(io_req.rs2_data, vs2_fmt)
    rs3_data := recode(io_req.rs3_data, vd_fmt)
    when (io_req.uop.is_rvv) {
      fp_ctrl.typeTagIn := Mux(fp_ctrl.swap12 || vs1_widen && !vs2_widen, vs2_fmt, vs1_fmt)
      fp_ctrl.typeTagOut:= vd_fmt
    }
  }

  // FIXME: S->H widening operation must be fixed
  def fuInput(minT: Option[tile.FType], vector: Boolean = false): tile.FPInput = {
    val req     = Wire(new tile.FPInput)
    val tagIn   = fp_ctrl.typeTagIn
    val tag     = fp_ctrl.typeTagOut
    val vs1_tag = Mux(vector.B && (vd_widen ^ vs1_widen), tagIn, tag)
    val vs2_tag = Mux(vector.B && (vd_widen ^ vs2_widen), tagIn, tag)
    req <> fp_ctrl
    val unbox_rs1 = Mux(vector.B && (vd_widen ^ vs1_widen), unbox(rs1_data, vs1_tag, None), unbox(rs1_data, tag, minT))
    val unbox_rs2 = Mux(vector.B && (vd_widen ^ vs2_widen), unbox(rs2_data, vs2_tag, None), unbox(rs2_data, tag, minT))
    val unbox_rs3 = unbox(rs3_data, tag,      minT)
    if(vector) {
      req.rm := Mux(~io_req.uop.is_rvv, fp_rm, 
                Mux(io_req.uop.uopc.isOneOf(uopVFCLASS, uopVFMAX, uopVMFLT, uopVMFGT), 1.U,
                Mux(io_req.uop.uopc.isOneOf(uopVFMIN, uopVMFLE, uopVMFGE), 0.U,
                Mux(io_req.uop.uopc.isOneOf(uopVMFEQ, uopVMFNE), 2.U,
                Mux(io_req.uop.uopc === uopVFSGNJ,  ImmGenRmVSGN(io_req.uop.imm_packed),
                Mux(io_req.uop.uopc === uopVFCVT_F2F && CheckF2FRm(io_req.uop.imm_packed), 6.U,
                Mux(io_req.uop.fu_code_is(FU_F2I) && CheckF2IRm(io_req.uop.imm_packed), 1.U, 
                io_req.fcsr_rm)))))))
      req.in1 := unbox_rs1
      req.in2 := unbox_rs2
      req.in3 := unbox_rs3
    } else {
      req.rm := fp_rm
      req.in1 := unbox(io_req.rs1_data, tagIn, minT)
      req.in2 := unbox(io_req.rs2_data, tagIn, minT)
      req.in3 := unbox(io_req.rs3_data, tagIn, minT)
    }
    // e.g. vfcvt.x.f.v   vd, vs2, vm
    // e.g. vfsgnj.vv vd, vs2, vs1, vm
    when (fp_ctrl.swap12) { 
      req.in1 := unbox_rs2
      req.in2 := unbox_rs1
    }
    when (fp_ctrl.swap23) { req.in3 := req.in2 }
    //req.typ := ImmGenTyp(io_req.uop.imm_packed)
    val typ1 = Mux(tag === D, 1.U(1.W), 0.U(1.W))
    // typ of F2I and I2F
    if (vector) {
      req.typ := ImmGenTypRVV(typ1, io_req.uop.imm_packed)
    } else {
      req.typ := ImmGenTyp(io_req.uop.imm_packed)
    }
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
  val fpiu_invert = Pipe(io.req.valid, io_req.uop.uopc === uopVMFNE, fpu_latency).bits
  val fpiu_result  = Wire(new tile.FPResult)
  fpiu_result.data := Mux(fpiu_invert, ~fpiu_out.bits.toint, fpiu_out.bits.toint)
  fpiu_result.exc  := fpiu_out.bits.exc

  val fpmu = Module(new tile.FPToFP(fpu_latency)) // latency 2 for rocket
  fpmu.io.in.bits := fpiu.io.in.bits
  fpmu.io.lt := fpiu.io.out.bits.lt
  val fpmu_dtype = Pipe(io.req.valid && fp_ctrl.fastpipe, fp_ctrl.typeTagOut, fpu_latency).bits

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
                             box(fpmu.io.out.bits.data, fpmu_dtype)))))

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

/**
 * FPU unit that wraps the RocketChip FPU units (which in turn wrap hardfloat)
 */
class VecFPU()(implicit p: Parameters) extends BoomModule with tile.HasFPUParameters with ShouldBeRetimed
{
  val io = IO(new Bundle {
    val req = Flipped(new ValidIO(new FpuReq(true)))
    val resp = new ValidIO(new ExeUnitResp(vLen))
  })

  // all FP units are padded out to the same latency for easy scheduling of the write port
  val fpu_latency = dfmaLatency
  val io_req = io.req.bits

  val fp_decoder = Module(new UOPCodeFPUDecoder(vector = true))
  fp_decoder.io.uopc := io_req.uop.uopc
  val fp_ctrl = WireInit(fp_decoder.io.sigs)
  val fp_rm = io_req.fcsr_rm
  val rs1_data = WireInit(io_req.rs1_data)
  val rs2_data = WireInit(io_req.rs2_data)
  val rs3_data = WireInit(io_req.rs3_data)
  val vd_widen = io_req.uop.rt(RD , isWidenV)
  val vs1_widen= io_req.uop.rt(RS1, isWidenV)
  val vs2_widen= io_req.uop.rt(RS2, isWidenV)
  val vd_sew  = io_req.uop.vd_eew
  val vs1_sew = io_req.uop.vs1_eew
  val vs2_sew = io_req.uop.vs2_eew
  val vd_fmt  = Mux(vd_sew  === 3.U, D, Mux(vd_sew  === 2.U, S, H))
  val vs1_fmt = Mux(vs1_sew === 3.U, D, Mux(vs1_sew === 2.U, S, H))
  val vs2_fmt = Mux(vs2_sew === 3.U, D, Mux(vs2_sew === 2.U, S, H))
  when (io.req.valid && io_req.uop.is_rvv) {
    assert(io_req.uop.fp_val, "unexpected fp_val")
    assert(vd_sew >= 1.U && vd_sew <= 3.U, "unsupported vd_sew")
  }
  when (io_req.uop.is_rvv) {
    fp_ctrl.typeTagIn := Mux(fp_ctrl.swap12 || vs1_widen && !vs2_widen, vs2_fmt, vs1_fmt)
    fp_ctrl.typeTagOut:= vd_fmt
  }

  // FIXME: S->H widening operation must be fixed
  def fuInput(minT: Option[tile.FType], esel: Int): tile.FPInput = {
    val req     = Wire(new tile.FPInput)
    val tagIn   = fp_ctrl.typeTagIn
    val tag     = fp_ctrl.typeTagOut
    val vs1_tag = Mux((vd_widen ^ vs1_widen), tagIn, tag)
    val vs2_tag = Mux((vd_widen ^ vs2_widen), tagIn, tag)
    req <> fp_ctrl
    val rs1_edata, rs2_edata, rs3_edata = WireInit(0.U(65.W))
    if (minT == Some(tile.FType.D)) {
      rs1_edata := recode(rs1_data(esel*64+63, esel*64), vs1_fmt)
      rs2_edata := recode(rs2_data(esel*64+63, esel*64), vs2_fmt)
      rs3_edata := recode(rs3_data(esel*64+63, esel*64), vd_fmt)
    } else if (minT == Some(tile.FType.S)) {
      rs1_edata := recode(rs1_data(esel*32+31, esel*32), vs1_fmt)
      rs2_edata := recode(rs2_data(esel*32+31, esel*32), vs2_fmt)
      rs3_edata := recode(rs3_data(esel*32+31, esel*32), vd_fmt)
    } else if (minT == Some(tile.FType.H)) {
      rs1_edata := recode(rs1_data(esel*16+15, esel*16), vs1_fmt)
      rs2_edata := recode(rs2_data(esel*16+15, esel*16), vs2_fmt)
      rs3_edata := recode(rs3_data(esel*16+15, esel*16), vd_fmt)
    } else {
      if (esel < 16) {
        rs1_edata := recode(Mux(vs1_sew === 3.U, rs1_data(esel*64+63, esel*64),
                            Mux(vs1_sew === 2.U, rs1_data(esel*32+31, esel*32),
                                                 rs1_data(esel*16+15, esel*16))), vs1_fmt)
        rs2_edata := recode(Mux(vs2_sew === 3.U, rs2_data(esel*64+63, esel*64),
                            Mux(vs2_sew === 2.U, rs2_data(esel*32+31, esel*32),
                                                 rs2_data(esel*16+15, esel*16))), vs2_fmt)
        rs3_edata := recode(Mux(vd_sew === 3.U,  rs3_data(esel*64+63, esel*64),
                            Mux(vd_sew === 2.U,  rs3_data(esel*32+31, esel*32),
                                                 rs3_data(esel*16+15, esel*16))), vd_fmt)
      } else if (esel < 32) {
        rs1_edata := recode(Mux(vs1_sew === 2.U, rs1_data(esel*32+31, esel*32),
                                                 rs1_data(esel*16+15, esel*16)), vs1_fmt)
        rs2_edata := recode(Mux(vs2_sew === 2.U, rs2_data(esel*32+31, esel*32),
                                                 rs2_data(esel*16+15, esel*16)), vs2_fmt)
        rs3_edata := recode(Mux(vd_sew === 2.U,  rs3_data(esel*32+31, esel*32),
                                                 rs3_data(esel*16+15, esel*16)), vd_fmt)
      } else {
        rs1_edata := recode(rs1_data(esel*16+15, esel*16), vs1_fmt)
        rs2_edata := recode(rs2_data(esel*16+15, esel*16), vs2_fmt)
        rs3_edata := recode(rs3_data(esel*16+15, esel*16), vd_fmt)
      }
    }
    val unbox_rs1 = Mux(vd_widen^vs1_widen, unbox(rs1_edata, vs1_tag, None), unbox(rs1_edata, tag, minT))
    val unbox_rs2 = Mux(vd_widen^vs2_widen, unbox(rs2_edata, vs2_tag, None), unbox(rs2_edata, tag, minT))
    val unbox_rs3 = unbox(rs3_edata, tag, minT)
    req.rm := Mux(io_req.uop.uopc.isOneOf(uopVFCLASS, uopVFMAX, uopVMFLT, uopVMFGT), 1.U,
              Mux(io_req.uop.uopc.isOneOf(uopVFMIN, uopVMFLE, uopVMFGE), 0.U,
              Mux(io_req.uop.uopc.isOneOf(uopVMFEQ, uopVMFNE), 2.U,
              Mux(io_req.uop.uopc === uopVFSGNJ,  ImmGenRmVSGN(io_req.uop.imm_packed),
              Mux(io_req.uop.uopc === uopVFCVT_F2F && CheckF2FRm(io_req.uop.imm_packed), 6.U,
              Mux(io_req.uop.fu_code_is(FU_F2I) && CheckF2IRm(io_req.uop.imm_packed), 1.U,
              io_req.fcsr_rm))))))
    req.in1 := unbox_rs1
    req.in2 := unbox_rs2
    req.in3 := unbox_rs3
    // e.g. vfcvt.x.f.v   vd, vs2, vm
    // e.g. vfsgnj.vv vd, vs2, vs1, vm
    when (fp_ctrl.swap12) { 
      req.in1 := unbox_rs2
      req.in2 := unbox_rs1
    }
    when (fp_ctrl.swap23) { req.in3 := req.in2 }
    //req.typ := ImmGenTyp(io_req.uop.imm_packed)
    val typ1 = Mux(tag === D, 1.U(1.W), 0.U(1.W))
    // typ of F2I and I2F
    req.typ := ImmGenTypRVV(typ1, io_req.uop.imm_packed)
    req.fmt := Mux(tag === H, 2.U, Mux(tag === S, 0.U, 1.U)) // TODO support Zfh and avoid special-case below
    when (io_req.uop.uopc === uopFMV_X_S) {
      req.fmt := 0.U
    } .elsewhen (io_req.uop.uopc.isOneOf(uopVFMADD,uopVFNMADD,uopVFMSUB,uopVFNMSUB)) {
        req.in2 := unbox_rs3
        req.in3 := unbox_rs2
    }

    val fma_decoder = Module(new FMADecoder(vector = true))
    fma_decoder.io.uopc := io_req.uop.uopc
    req.fmaCmd := fma_decoder.io.cmd // ex_reg_inst(3,2) | (!fp_ctrl.ren3 && ex_reg_inst(27))
    req
  }

  // FUE: function-unit-enable
  class fue extends Bundle {
    val dfma = Bool()
    val sfma = Bool()
    val hfma = Bool()
    val fpiu = Bool()
    val fpmu = Bool()
  }

  val reqfue = Wire(new fue);
  reqfue.dfma := fp_ctrl.fma && (fp_ctrl.typeTagOut === D)
  reqfue.sfma := fp_ctrl.fma && (fp_ctrl.typeTagOut === S)
  reqfue.hfma := fp_ctrl.fma && (fp_ctrl.typeTagOut === H)
  reqfue.fpiu := (fp_ctrl.toint || (fp_ctrl.fastpipe && fp_ctrl.wflags))
  reqfue.fpmu := fp_ctrl.fastpipe
  val reqpipe = Pipe(io.req.valid, io_req, fpu_latency)
  val fuepipe = Pipe(io.req.valid, reqfue, fpu_latency).bits

  val dfma = (0 until vLen/64).map(i => Module(new tile.FPUFMAPipe(latency = fpu_latency, t = tile.FType.D)))
  for (i <- 0 until vLen/64) {
    val active = (io.req.bits.uop.v_unmasked || io.req.bits.rvm_data(i)) && ((io.req.bits.uop.v_eidx + i.U) < io.req.bits.uop.vconfig.vl) && ((io.req.bits.uop.v_eidx + i.U) >= io.req.bits.uop.vstart)
    dfma(i).io.in.bits := fuInput(Some(tile.FType.D), i)
    dfma(i).io.in.valid := active && io.req.valid && reqfue.dfma
  }

  val sfma = (0 until vLen/32).map(i => Module(new tile.FPUFMAPipe(latency = fpu_latency, t = tile.FType.S)))
  for (i <- 0 until vLen/32) {
    val active = (io.req.bits.uop.v_unmasked || io.req.bits.rvm_data(i)) && ((io.req.bits.uop.v_eidx + i.U) < io.req.bits.uop.vconfig.vl) && ((io.req.bits.uop.v_eidx + i.U) >= io.req.bits.uop.vstart)
    sfma(i).io.in.bits := fuInput(Some(tile.FType.S), i)
    sfma(i).io.in.valid := active && io.req.valid && reqfue.sfma
  }

  val hfma = (0 until vLen/16).map(i => Module(new tile.FPUFMAPipe(latency = fpu_latency, t = tile.FType.H)))

  val fpiu = (0 until vLen/16).map(i => Module(new tile.FPToInt))
  val fpiu_result = Wire(Vec(vLen/16, new tile.FPResult))
  val fpiu_out = fpiu.map(m => Pipe(RegNext(m.io.in.valid && !fp_ctrl.fastpipe), m.io.out.bits, fpu_latency-1))
  val fpiu_invert = reqpipe.bits.uop.uopc === uopVMFNE
  val fpiu_dtype = Pipe(io.req.valid && !fp_ctrl.fastpipe, fp_ctrl.typeTagOut, fpu_latency).bits
  val pipe_fastpipe = Pipe(io.req.valid, fp_ctrl.fastpipe, fpu_latency).bits
  val fpiu_out_valid = fpiu.map(m => m.io.out.valid).reduce(_ | _) && !pipe_fastpipe

  val fpmu = (0 until vLen/16).map(i => Module(new tile.FPToFP(fpu_latency)))
  val fpmu_dtype = Pipe(io.req.valid && fp_ctrl.fastpipe, fp_ctrl.typeTagOut, fpu_latency).bits
  for (i <- 0 until vLen/16) {
    val active = (io.req.bits.uop.v_unmasked || io.req.bits.rvm_data(i)) && ((io.req.bits.uop.v_eidx + i.U) < io.req.bits.uop.vconfig.vl)  && ((io.req.bits.uop.v_eidx + i.U) >= io.req.bits.uop.vstart)

    hfma(i).io.in.bits  := fuInput(Some(tile.FType.H), i)
    hfma(i).io.in.valid := active && io.req.valid && reqfue.hfma

    fpiu(i).io.in.bits  := fuInput(None, i)
    fpiu(i).io.in.valid := active && io.req.valid && reqfue.fpiu &&
                           Mux(fp_ctrl.typeTagOut === D, (i < vLen/64).B,
                           Mux(fp_ctrl.typeTagOut === S, (i < vLen/32).B, true.B))
    fpiu_result(i).data := Mux(fpiu_invert, ~fpiu_out(i).bits.toint, fpiu_out(i).bits.toint)
    fpiu_result(i).exc  := fpiu_out(i).bits.exc

    fpmu(i).io.lt       := fpiu(i).io.out.bits.lt
    fpmu(i).io.in.bits  := fuInput(None, i)
    fpmu(i).io.in.valid := active && io.req.valid && reqfue.fpmu &&
                           Mux(fp_ctrl.typeTagOut === D, (i < vLen/64).B,
                           Mux(fp_ctrl.typeTagOut === S, (i < vLen/32).B, true.B))
  }

  // Response (all FP units have been padded out to the same latency)
  io.resp.valid := reqpipe.valid
  val fpu_out_data =
      Mux(fuepipe.dfma, Cat(dfma.zipWithIndex.map{case(m,i) => Mux(m.io.out.valid, ieee(box(m.io.out.bits.data, D))(63,0), reqpipe.bits.rs3_data(i*64+63,i*64))}.reverse),
      Mux(fuepipe.sfma, Cat(sfma.zipWithIndex.map{case(m,i) => Mux(m.io.out.valid, ieee(box(m.io.out.bits.data, S))(31,0), reqpipe.bits.rs3_data(i*32+31,i*32))}.reverse),
      Mux(fuepipe.hfma, Cat(hfma.zipWithIndex.map{case(m,i) => Mux(m.io.out.valid, ieee(box(m.io.out.bits.data, H))(15,0), reqpipe.bits.rs3_data(i*16+15,i*16))}.reverse),
      Mux(fuepipe.fpiu && (fpiu_dtype === D) && fpiu_out_valid, Cat(fpiu_result.slice(0, vLen/64).zipWithIndex.map{case(r,i) => Mux(fpiu(i).io.out.valid, r.data(63,0), reqpipe.bits.rs3_data(i*64+63,i*64))}.reverse),
      Mux(fuepipe.fpiu && (fpiu_dtype === S) && fpiu_out_valid, Cat(fpiu_result.slice(0, vLen/32).zipWithIndex.map{case(r,i) => Mux(fpiu(i).io.out.valid, r.data(31,0), reqpipe.bits.rs3_data(i*32+31,i*32))}.reverse),
      Mux(fuepipe.fpiu && (fpiu_dtype === H) && fpiu_out_valid, Cat(fpiu_result.slice(0, vLen/16).zipWithIndex.map{case(r,i) => Mux(fpiu(i).io.out.valid, r.data(15,0), reqpipe.bits.rs3_data(i*16+15,i*16))}.reverse),
      Mux(fuepipe.fpmu && fpmu_dtype === D, Cat(fpmu.slice(0, vLen/64).zipWithIndex.map{case(m,i) => Mux(m.io.out.valid, ieee(box(m.io.out.bits.data, D))(63,0), reqpipe.bits.rs3_data(i*64+63,i*64))}.reverse),
      Mux(fuepipe.fpmu && fpmu_dtype === S, Cat(fpmu.slice(0, vLen/32).zipWithIndex.map{case(m,i) => Mux(m.io.out.valid, ieee(box(m.io.out.bits.data, S))(31,0), reqpipe.bits.rs3_data(i*32+31,i*32))}.reverse),
                                            Cat(fpmu.zipWithIndex.map{                  case(m,i) => Mux(m.io.out.valid, ieee(box(m.io.out.bits.data, H))(15,0), reqpipe.bits.rs3_data(i*16+15,i*16))}.reverse)))))))))

  val fpu_out_exc =
      Mux(fuepipe.dfma, dfma.map(m => Mux(m.io.out.valid, m.io.out.bits.exc, 0.U)).reduce(_ | _),
      Mux(fuepipe.sfma, sfma.map(m => Mux(m.io.out.valid, m.io.out.bits.exc, 0.U)).reduce(_ | _),
      Mux(fuepipe.hfma, hfma.map(m => Mux(m.io.out.valid, m.io.out.bits.exc, 0.U)).reduce(_ | _),
      Mux(fuepipe.fpiu, (fpiu_out zip fpiu_result).map{case(o,r)=>Mux(o.valid, r.exc, 0.U)}.reduce(_ | _),
                        fpmu.map(m => Mux(m.io.out.valid, m.io.out.bits.exc, 0.U)).reduce(_ | _)))))

  // FIXME: handle masked ops
  io.resp.bits.data              := fpu_out_data
  io.resp.bits.fflags.valid      := io.resp.valid
  io.resp.bits.fflags.bits.flags := fpu_out_exc
}
