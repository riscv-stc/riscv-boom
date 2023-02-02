//******************************************************************************
// Copyright (c) 2016 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// FDiv/FSqrt Unit
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.exu

import chisel3._
import chisel3.util._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.tile.FPConstants._
import freechips.rocketchip.tile
import boom.common._
import boom.common.MicroOpcodes._
import boom.util._
import freechips.rocketchip.tile.HasFPUParameters
import freechips.rocketchip.util.uintToBitPat
import freechips.rocketchip.util._

/**
 * Decoder for FPU divide and square root signals
 */
class UOPCodeFDivDecoder(vector: Boolean = false)(implicit p: Parameters) extends BoomModule
  with HasFPUParameters
{
  val io = IO(new Bundle {
    val uopc = Input(Bits(UOPC_SZ.W))
    val sigs = Output(new tile.FPUCtrlSigs())
  })

  val N = BitPat("b0")
  val Y = BitPat("b1")
  val X = BitPat("b?")

  val default: List[BitPat] = List(X,X,X,X,X, X,X,X,X,X,X,X, X,X,X,X)

    //                                      swap12         fma
    //                                      | swap32       | div
    //                                      | | typeTagIn  | | sqrt
    //                           ldst       | | | typeTagOut | | wflags
    //                           | wen      | | | | from_int | | |
    //                           | | ren1   | | | | | to_int | | |
    //                           | | | ren2 | | | | | | fast | | |
    //                           | | | | ren3 | | | | | |  | | | |
    //                           | | | | |  | | | | | | |  | | | |
  val f_table: Array[(BitPat, List[BitPat])] = 
    Array(
      BitPat(uopFDIV_S)  -> List(X,X,Y,Y,X, X,X,S,S,X,X,X, X,Y,N,Y),
      BitPat(uopFDIV_D)  -> List(X,X,Y,Y,X, X,X,D,D,X,X,X, X,Y,N,Y),
      BitPat(uopFSQRT_S) -> List(X,X,Y,N,X, X,X,S,S,X,X,X, X,N,Y,Y),
      BitPat(uopFSQRT_D) -> List(X,X,Y,N,X, X,X,D,D,X,X,X, X,N,Y,Y)
    )
  
    //                                      swap12         fma
    //                                      | swap32       | div
    //                                      | | typeTagIn  | | sqrt
    //                           ldst       | | | typeTagOut | | wflags
    //                           | wen      | | | | from_int | | |
    //                           | | ren1   | | | | | to_int | | |
    //                           | | | ren2 | | | | | | fast | | |
    //                           | | | | ren3 | | | | | |  | | | |
    //                           | | | | |  | | | | | | |  | | | |
  val v_table: Array[(BitPat, List[BitPat])] = 
    Array(
      BitPat(uopVFDIV)   -> List(X,X,Y,Y,X, Y,X,D,D,X,X,X, X,Y,N,Y)
     ,BitPat(uopVFRDIV)  -> List(X,X,Y,Y,X, N,X,D,D,X,X,X, X,Y,N,Y)
     ,BitPat(uopVFSQRT)  -> List(X,X,Y,N,X, Y,X,D,D,X,X,X, X,N,Y,Y)
    )
  
  val insns = if (vector) v_table else f_table

  val decoder = freechips.rocketchip.rocket.DecodeLogic(io.uopc, default, insns)

  val s = io.sigs
  val sigs = Seq(s.ldst, s.wen, s.ren1, s.ren2, s.ren3, s.swap12,
                 s.swap23, s.typeTagIn, s.typeTagOut, s.fromint, s.toint, s.fastpipe, s.fma,
                 s.div, s.sqrt, s.wflags)
  sigs zip decoder map {case(s,d) => s := d}
}

/**
 * fdiv/fsqrt is douple-precision. Must upconvert inputs and downconvert outputs
 * as necessary.  Must wait till killed uop finishes before we're ready again.
 * fdiv/fsqrt unit uses an unstable FIFO interface, and thus we must spend a
 * cycle buffering up an uop to provide slack between the issue queue and the
 * fdiv/fsqrt unit.  FDivUnit inherents directly from FunctionalUnit, because
 * UnpipelinedFunctionalUnit can only handle 1 inflight uop, whereas FDivUnit
 * contains up to 2 inflight uops due to the need to buffer the input as the
 * fdiv unit uses an unstable FIFO interface.
 * TODO extend UnpipelinedFunctionalUnit to handle a >1 uops inflight.
 *
 * @param isPipelined is the functional unit pipelined
 * @param numStages number of stages for the functional unit
 * @param numBypassStages number of bypass stages
 * @param dataWidth width of the data out of the functional unit
 */
class FDivSqrtUnit(vector: Boolean = false)(implicit p: Parameters)
  extends FunctionalUnit(
    isPipelined = false,
    numStages = 1,
    numBypassStages = 0,
    dataWidth = 65,
    needsFcsr = true)
  with tile.HasFPUParameters
{
  //--------------------------------------
  // buffer inputs and upconvert as needed

  // provide a one-entry queue to store incoming uops while waiting for the fdiv/fsqrt unit to become available.
  val r_buffer_val = RegInit(false.B)
  val r_buffer_req = Reg(new FuncUnitReq(dataWidth=65))
  val r_buffer_fin = Reg(new tile.FPInput)

  val fdiv_decoder = Module(new UOPCodeFDivDecoder(vector))
  fdiv_decoder.io.uopc := io.req.bits.uop.uopc

  // handle branch kill on queued entry
  r_buffer_val := !IsKilledByBranch(io.brupdate, r_buffer_req.uop) && !io.req.bits.kill && r_buffer_val
  r_buffer_req.uop.br_mask := GetNewBrMask(io.brupdate, r_buffer_req.uop)

  // handle incoming uop, including upconversion as needed, and push back if our input queue is already occupied
  io.req.ready := !r_buffer_val

  def upconvert_s2d(x: UInt) = {
    val s2d = Module(new hardfloat.RecFNToRecFN(inExpWidth = 8, inSigWidth = 24, outExpWidth = 11, outSigWidth = 53))
    s2d.io.in := x
    s2d.io.roundingMode := 0.U
    s2d.io.detectTininess := DontCare
    s2d.io.out
  }

  def upconvert_h2d(x: UInt) = {
    val h2d = Module(new hardfloat.RecFNToRecFN(inExpWidth = 5, inSigWidth = 11, outExpWidth = 11, outSigWidth = 53))
    h2d.io.in := x
    h2d.io.roundingMode := 0.U
    h2d.io.detectTininess := DontCare
    h2d.io.out
  }

  val io_req = io.req.bits
  val fdiv_ctrl = WireInit(fdiv_decoder.io.sigs)
  val rs1_data = WireInit(io_req.rs1_data)
  val rs2_data = WireInit(io_req.rs2_data)
  if(vector) {
    val vsew = io_req.uop.vconfig.vtype.vsew
    assert(!io.req.valid || !io_req.uop.is_rvv || (vsew >= 1.U && vsew <= 3.U), s"unsupported vsew: $vsew")
    assert(!io.req.valid || io_req.uop.is_rvv && io_req.uop.fp_val, "unsupported data type")
    val fmt = Mux(vsew === 3.U, D, Mux(vsew === 2.U, S, H))
    rs1_data := recode(io_req.rs1_data, fmt)
    rs2_data := recode(io_req.rs2_data, fmt)
    when(fdiv_ctrl.swap12) {
      rs1_data := recode(io_req.rs2_data, fmt)
      rs2_data := recode(io_req.rs1_data, fmt)
    }
    fdiv_ctrl.typeTagIn := Mux(vsew === 3.U, D, Mux(vsew === 2.U, S, H))
    fdiv_ctrl.typeTagOut:= Mux(vsew === 3.U, D, Mux(vsew === 2.U, S, H))
    //when (!io_req.uop.v_active) {
    when (io_req.rvm_data(0)) {
      fdiv_ctrl.ren2 := false.B
      fdiv_ctrl.ren3 := false.B
    }
  }

  val tag = fdiv_ctrl.typeTagIn
  val in1_upconvert_s2d = upconvert_s2d(unbox(rs1_data, tag, Some(tile.FType.S)))
  val in2_upconvert_s2d = upconvert_s2d(unbox(rs2_data, tag, Some(tile.FType.S)))
  // aiming to support scalar half-precision floating point instructions
  // if(vector) { }
  val in1_upconvert_h2d = upconvert_h2d(unbox(rs1_data, tag, Some(tile.FType.H)))
  val in2_upconvert_h2d = upconvert_h2d(unbox(rs2_data, tag, Some(tile.FType.H)))

  when (io.req.valid && !IsKilledByBranch(io.brupdate, io.req.bits.uop) && !io.req.bits.kill) {
    r_buffer_val := true.B
    r_buffer_req := io.req.bits
    r_buffer_req.uop.br_mask := GetNewBrMask(io.brupdate, io.req.bits.uop)
    r_buffer_fin <> fdiv_ctrl

    r_buffer_fin.rm := io.fcsr_rm
    r_buffer_fin.typ := 0.U // unused for fdivsqrt
    r_buffer_fin.in1 := unbox(rs1_data, tag, Some(tile.FType.D))
    r_buffer_fin.in2 := unbox(rs2_data, tag, Some(tile.FType.D))
    r_buffer_fin.in3 := io_req.rs3_data
    when (tag === S) {
      r_buffer_fin.in1 := in1_upconvert_s2d
      r_buffer_fin.in2 := in2_upconvert_s2d
    } .elsewhen(tag === H) {
      r_buffer_fin.in1 := in1_upconvert_h2d
      r_buffer_fin.in2 := in2_upconvert_h2d
    }
  }

  // assert (!(r_buffer_val && io.req.valid), "[fdiv] a request is incoming while the buffer is already full.")

  //-----------
  // fdiv/fsqrt

  val divsqrt = Module(new hardfloat.DivSqrtRecF64)

  val r_divsqrt_val = RegInit(false.B)  // inflight uop?
  val r_divsqrt_killed = Reg(Bool())           // has inflight uop been killed?
  val r_divsqrt_fin = Reg(new tile.FPInput)
  val r_divsqrt_uop = Reg(new MicroOp)

  // Need to buffer output until RF writeport is available.
  val output_buffer_available = Wire(Bool())

  val may_fire_input =
    r_buffer_val &&
    (r_buffer_fin.div || r_buffer_fin.sqrt) &&
    !r_divsqrt_val &&
    output_buffer_available

  val divsqrt_ready = Mux(divsqrt.io.sqrtOp, divsqrt.io.inReady_sqrt, divsqrt.io.inReady_div)
  divsqrt.io.inValid := may_fire_input // must be setup early
  divsqrt.io.sqrtOp := r_buffer_fin.sqrt
  divsqrt.io.a := r_buffer_fin.in1
  divsqrt.io.b := Mux(divsqrt.io.sqrtOp, r_buffer_fin.in1, r_buffer_fin.in2)
  divsqrt.io.roundingMode := r_buffer_fin.rm
  divsqrt.io.detectTininess := DontCare

  r_divsqrt_killed := r_divsqrt_killed || IsKilledByBranch(io.brupdate, r_divsqrt_uop) || io.req.bits.kill
  r_divsqrt_uop.br_mask := GetNewBrMask(io.brupdate, r_divsqrt_uop)

  when (may_fire_input && divsqrt_ready) {
    // Remove entry from the input buffer.
    // We don't have time to kill divsqrt request so must track if killed on entry.
    r_buffer_val := false.B
    r_divsqrt_val := true.B
    r_divsqrt_fin := r_buffer_fin
    r_divsqrt_uop := r_buffer_req.uop
    r_divsqrt_killed := IsKilledByBranch(io.brupdate, r_buffer_req.uop) || io.req.bits.kill
    r_divsqrt_uop.br_mask := GetNewBrMask(io.brupdate, r_buffer_req.uop)
  }

  //-----------------------------------------
  // buffer output and down-convert as needed

  val r_out_val = RegInit(false.B)
  val r_out_uop = Reg(new MicroOp)
  val r_out_flags_double = Reg(Bits())
  val r_out_wdata_double = Reg(Bits())

  output_buffer_available := !r_out_val

  r_out_uop.br_mask := GetNewBrMask(io.brupdate, r_out_uop)

  when (io.resp.ready || IsKilledByBranch(io.brupdate, r_out_uop) || io.req.bits.kill) {
    r_out_val := false.B
  }
  when (divsqrt.io.outValid_div || divsqrt.io.outValid_sqrt) {
    r_divsqrt_val := false.B

    r_out_val := !r_divsqrt_killed && !IsKilledByBranch(io.brupdate, r_divsqrt_uop) && !io.req.bits.kill
    r_out_uop := r_divsqrt_uop
    r_out_uop.br_mask := GetNewBrMask(io.brupdate, r_divsqrt_uop)
    r_out_wdata_double := sanitizeNaN(divsqrt.io.out, tile.FType.D)
    r_out_flags_double := divsqrt.io.exceptionFlags

    assert (r_divsqrt_val, "[fdiv] a response is being generated for no request.")
  }

  assert (!(r_out_val && (divsqrt.io.outValid_div || divsqrt.io.outValid_sqrt)),
    "[fdiv] Buffered output being overwritten by another output from the fdiv/fsqrt unit.")

  val downvert_d2s = Module(new hardfloat.RecFNToRecFN(
    inExpWidth = 11, inSigWidth = 53, outExpWidth = 8, outSigWidth = 24))
  downvert_d2s.io.in := r_out_wdata_double
  downvert_d2s.io.roundingMode := r_divsqrt_fin.rm
  downvert_d2s.io.detectTininess := DontCare

  val downvert_d2h = Module(new hardfloat.RecFNToRecFN(
    inExpWidth = 11, inSigWidth = 53, outExpWidth = 5, outSigWidth = 11))
  downvert_d2h.io.in := r_out_wdata_double
  downvert_d2h.io.roundingMode := r_divsqrt_fin.rm
  downvert_d2h.io.detectTininess := DontCare

  val out_flags = r_out_flags_double | Mux(r_divsqrt_fin.typeTagIn === S, downvert_d2s.io.exceptionFlags, 
                                       Mux(r_divsqrt_fin.typeTagIn === H, downvert_d2h.io.exceptionFlags, 0.U) )

  io.resp.valid := r_out_val && !IsKilledByBranch(io.brupdate, r_out_uop)
  io.resp.bits.uop := r_out_uop
  val fdiv_out_data =
    Mux(r_divsqrt_fin.typeTagIn === S, box(downvert_d2s.io.out, S),
    Mux(r_divsqrt_fin.typeTagIn === H, box(downvert_d2h.io.out.sextTo(xLen), H), 
        box(r_out_wdata_double, D) ) )

  if(vector) {
    // FIXME
    io.resp.bits.data := Mux(r_buffer_req.rvm_data(0), ieee(fdiv_out_data), r_divsqrt_fin.in3)
    io.resp.bits.fflags.bits.flags := Mux(r_buffer_req.rvm_data(0), out_flags, 0.U)
  } else {
    io.resp.bits.data := fdiv_out_data
    io.resp.bits.fflags.bits.flags := out_flags
  }
  io.resp.bits.fflags.valid := io.resp.valid
  io.resp.bits.fflags.bits.uop := r_out_uop
  io.resp.bits.fflags.bits.uop.br_mask := GetNewBrMask(io.brupdate, r_out_uop)
}


/**
 * Vector Floating Divide and Sqrt functional units wrapper.
 *
 * @param dataWidth data to be passed into the functional unit
 */
class VecFDivSqrtUnit(dataWidth: Int)(implicit p: Parameters) extends IterativeFunctionalUnit(dataWidth, needsFcsr = true)
{
  val uop = io.req.bits.uop
  val rs1_data = io.req.bits.rs1_data
  val rs2_data = io.req.bits.rs2_data
  val rs3_data = WireDefault(io.req.bits.rs3_data)
  val old_data = WireDefault(io.req.bits.rs3_data)
  val rvm_data = io.req.bits.rvm_data
  val body, prestart, tail, mask, inactive = Wire(UInt(vLenb.W))
  val vl = uop.vconfig.vl
  prestart := Cat((0 until vLen/16).map(b => uop.v_eidx + b.U < uop.vstart).reverse)
  body     := Cat((0 until vLen/16).map(b => uop.v_eidx + b.U >= uop.vstart && uop.v_eidx + b.U < vl).reverse)
  mask     := Mux(uop.v_unmasked, ~(0.U(vLenb.W)), Cat((0 until vLen/16).map(b => rvm_data(b)).reverse))
  tail     := Cat((0 until vLen/16).map(b => uop.v_eidx + b.U >= vl).reverse)
  inactive := prestart | body & ~mask | tail
  val unmask = body & ~mask
  val tail_bitInactive = Mux1H(UIntToOH(uop.vd_eew(1,0)),
                           Seq(FillInterleaved(8, tail),
                               FillInterleaved(16, tail(vLenb/2-1, 0)),
                               FillInterleaved(32, tail(vLenb/4-1, 0)),
                               FillInterleaved(64, tail(vLenb/8-1, 0))))
  val unmask_bitInactive = Mux1H(UIntToOH(uop.vd_eew(1,0)),
                           Seq(FillInterleaved(8, unmask),
                               FillInterleaved(16, unmask(vLenb/2-1, 0)),
                               FillInterleaved(32, unmask(vLenb/4-1, 0)),
                               FillInterleaved(64, unmask(vLenb/8-1, 0))))
  rs3_data := Mux1H(UIntToOH(Cat(uop.vconfig.vtype.vta,uop.vconfig.vtype.vma)),
              Seq(old_data,
                  old_data | unmask_bitInactive,
                  old_data | tail_bitInactive,
                  old_data | tail_bitInactive | unmask_bitInactive))

  val fdivValid  = Wire(Vec(vLen/16, Bool()))
  val e64DataOut = Wire(Vec(numELENinVLEN,   UInt(64.W)))
  val e32DataOut = Wire(Vec(numELENinVLEN*2, UInt(32.W)))
  val e16DataOut = Wire(Vec(numELENinVLEN*4, UInt(16.W)))
  val e64FlagOut = Wire(Vec(numELENinVLEN,   Bits(tile.FPConstants.FLAGS_SZ.W)))
  val e32FlagOut = Wire(Vec(numELENinVLEN*2, Bits(tile.FPConstants.FLAGS_SZ.W)))
  val e16FlagOut = Wire(Vec(numELENinVLEN*4, Bits(tile.FPConstants.FLAGS_SZ.W)))

  Bits(tile.FPConstants.FLAGS_SZ.W)

  val s_idle :: s_work :: s_done :: Nil = Enum(3)
  val state = RegInit(s_idle)
  switch(state) {
    is(s_idle) {
      when(io.req.valid && !this.do_kill) {
        state := s_work
      }
    }
    is(s_work) {
      when(fdivValid.andR && io.resp.ready) {
        state := s_idle
      } .elsewhen(fdivValid.andR) {
        state := s_done
      }
    }
    is(s_done) {
      when(io.resp.ready) { state := s_idle }
    }
  }
  when(this.do_kill) { state := s_idle }

  for(e <- 0 until vLen/16) {
    val fdiv = Module(new FDivSqrtUnit(vector = true))
    // inputs
    fdiv.io.req.valid          := io.req.valid && !this.do_kill
    fdiv.io.fcsr_rm            := io.fcsr_rm
    fdiv.io.req.bits.uop       := io.req.bits.uop
    fdiv.io.req.bits.pred_data := io.req.bits.pred_data
    fdiv.io.req.bits.kill      := io.req.bits.kill
    fdiv.io.brupdate           := io.brupdate
    fdiv.io.resp.ready         := (fdivValid.andR && io.resp.ready)
    if(e < numELENinVLEN) {
      fdiv.io.req.bits.rs1_data  := Mux1H(UIntToOH(uop.vd_eew),
                                        Seq(0.U, rs1_data(16*e+15, 16*e), rs1_data(32*e+31, 32*e), rs1_data(64*e+63, 64*e)))
      fdiv.io.req.bits.rs2_data  := Mux1H(UIntToOH(uop.vd_eew),
                                          Seq(0.U, rs2_data(16*e+15, 16*e), rs2_data(32*e+31, 32*e), rs2_data(64*e+63, 64*e)))
      fdiv.io.req.bits.rs3_data  := Mux1H(UIntToOH(uop.vd_eew),
                                          Seq(0.U, rs3_data(16*e+15, 16*e), rs3_data(32*e+31, 32*e), rs3_data(64*e+63, 64*e)))
    } else if(e < numELENinVLEN*2) {
      fdiv.io.req.bits.rs1_data  := Mux(uop.vd_eew(1), rs1_data(32*e+31, 32*e), rs1_data(16*e+15, 16*e))
      fdiv.io.req.bits.rs2_data  := Mux(uop.vd_eew(1), rs2_data(32*e+31, 32*e), rs2_data(16*e+15, 16*e))
      fdiv.io.req.bits.rs3_data  := Mux(uop.vd_eew(1), rs3_data(32*e+31, 32*e), rs3_data(16*e+15, 16*e))
    } else {
      fdiv.io.req.bits.rs1_data  := rs1_data(16*e+15, 16*e)
      fdiv.io.req.bits.rs2_data  := rs2_data(16*e+15, 16*e)
      fdiv.io.req.bits.rs3_data  := rs3_data(16*e+15, 16*e)
    }
    //fdiv.io.req.bits.uop.v_active := !inactive(e)
    fdiv.io.req.bits.rvm_data    := !inactive(e)
    // output
    fdivValid(e) := fdiv.io.resp.valid
    if (e < numELENinVLEN) {
      e64DataOut(e) := fdiv.io.resp.bits.data
      e64FlagOut(e) := fdiv.io.resp.bits.fflags.bits.flags
    }
    if(e < numELENinVLEN*2) {
      e32DataOut(e) := fdiv.io.resp.bits.data
      e32FlagOut(e) := fdiv.io.resp.bits.fflags.bits.flags
    }
    e16DataOut(e) := fdiv.io.resp.bits.data
    e16FlagOut(e) := fdiv.io.resp.bits.fflags.bits.flags
  }

  // output
  io.req.ready     := (state === s_idle)
  io.resp.valid    := fdivValid.andR && !this.do_kill
  io.resp.bits.data := Mux1H(UIntToOH(io.resp.bits.uop.vd_eew),
                             Seq(0.U, e16DataOut.asUInt, e32DataOut.asUInt, e64DataOut.asUInt))
  io.resp.bits.fflags.valid := io.resp.valid
  io.resp.bits.fflags.bits.flags := Mux1H(UIntToOH(io.resp.bits.uop.vd_eew),
                                          Seq(0.U, 
                                              e16FlagOut.reduce(_ | _),
                                              e32FlagOut.reduce(_ | _),
                                              e64FlagOut.reduce(_ | _)))
}
