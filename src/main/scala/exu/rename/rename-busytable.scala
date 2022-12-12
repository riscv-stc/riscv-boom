//******************************************************************************
// Copyright (c) 2015 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Rename BusyTable
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.exu

import chisel3._
import chisel3.util._
import boom.common._
import boom.util._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.util.{UIntIsOneOf, rightOR, uintToBitPat}
import boom.common.MicroOpcodes._

class BusyResp extends Bundle
{
  val prs1_busy = Bool()
  val prs2_busy = Bool()
  val prs3_busy = Bool()
}

class RenameBusyTable(
  val plWidth: Int,
  val numPregs: Int,
  val numWbPorts: Int,
  val bypass: Boolean,
  val float: Boolean)
  (implicit p: Parameters) extends BoomModule
{
  val pregSz = log2Ceil(numPregs)

  val io = IO(new BoomBundle()(p) {
    val ren_uops = Input(Vec(plWidth, new MicroOp))
    val rebusy_reqs = Input(Vec(plWidth, Bool()))
    val busy_resps  = Output(Vec(plWidth, new BusyResp))

    val wb_pdsts    = Input(Vec(numWbPorts, UInt(pregSz.W)))
    val wb_valids   = Input(Vec(numWbPorts, Bool()))

    val debug = new Bundle {
      val busytable = Output(Bits(numPregs.W))
    }
  })

  val busy_table = RegInit(0.U(numPregs.W))
  val busy_table_wb = Wire(UInt(numPregs.W))
  val busy_table_next = Wire(UInt(numPregs.W))

  // Unbusy written back registers.
  busy_table_wb := busy_table & ~(io.wb_pdsts zip io.wb_valids)
    .map {case (pdst, valid) => UIntToOH(pdst) & Fill(numPregs, valid.asUInt)}.reduce(_|_)
  // Rebusy newly allocated registers.
  busy_table_next := busy_table_wb | (io.ren_uops zip io.rebusy_reqs)
    .map {case (uop, req) => UIntToOH(uop.pdst) & Fill(numPregs, req.asUInt)}.reduce(_|_)

  // Read the busy table.
  for (i <- 0 until plWidth) {
    val prs1_was_bypassed = (0 until i).map(j =>
      io.ren_uops(i).lrs1 === io.ren_uops(j).ldst && io.rebusy_reqs(j)).foldLeft(false.B)(_||_)
    val prs2_was_bypassed = (0 until i).map(j =>
      io.ren_uops(i).lrs2 === io.ren_uops(j).ldst && io.rebusy_reqs(j)).foldLeft(false.B)(_||_)
    val prs3_was_bypassed = (0 until i).map(j =>
      io.ren_uops(i).lrs3 === io.ren_uops(j).ldst && io.rebusy_reqs(j)).foldLeft(false.B)(_||_)

    io.busy_resps(i).prs1_busy := busy_table(io.ren_uops(i).prs1) || prs1_was_bypassed && bypass.B
    io.busy_resps(i).prs2_busy := busy_table(io.ren_uops(i).prs2) || prs2_was_bypassed && bypass.B
    io.busy_resps(i).prs3_busy := busy_table(io.ren_uops(i).prs3) || prs3_was_bypassed && bypass.B
    if (!float) io.busy_resps(i).prs3_busy := false.B
  }

  busy_table := busy_table_next
  io.debug.busytable := busy_table
}

class VecRenameBusyTable(
  val plWidth: Int,
  val numPregs: Int,
  val numWbPorts: Int)
  (implicit p: Parameters) extends BoomModule
{
  val pregSz = log2Ceil(numPregs)

  val io = IO(new BoomBundle()(p) {
    val ren_uops = Input(Vec(plWidth, new MicroOp))
    val rebusy_reqs = Input(Vec(plWidth, Bool()))
    val vbusy_status = Output(UInt(numPregs.W))

    val wb_pdsts    = Input(Vec(numWbPorts, UInt(pregSz.W)))
    val wb_valids   = Input(Vec(numWbPorts, Bool()))
    val wb_bits     = Input(Vec(numWbPorts, UInt(vLenb.W)))

    val clr_valids  = Input(Vec(8, Bool()))
    val clr_pdsts   = Input(Vec(8, UInt(pregSz.W)))

    val debug = new Bundle {
      val busytable = Output(Vec(numPregs, UInt(vLenb.W)))
    }
  })

    val busy_table = RegInit(VecInit(Seq.fill(numPregs){0.U(vLenb.W)}))
    val busy_table_wb = Wire(Vec(numPregs, UInt(vLenb.W)))
    val busy_table_next = Wire(Vec(numPregs, UInt(vLenb.W)))

    for (r <- 0 until numPregs) {
      busy_table_wb(r) := busy_table(r) & ~(io.wb_pdsts zip io.wb_valids zip io.wb_bits).map {case ((pdst, valid), bits) =>
        bits & Fill(vLenb, (r.U === pdst && valid).asUInt) }.reduce(_|_) &
        ~(io.clr_pdsts zip io.clr_valids).map { case(pdst, valid) => Fill(vLenb, r.U === pdst && valid).asUInt }.reduce(_|_)
      busy_table_next(r) := (io.ren_uops zip io.rebusy_reqs).map {case (uop, rbreq) =>
        Fill(vLenb, (uop.pvd.map(vd => vd.valid && vd.bits === r.U).reduce(_ || _) && rbreq).asUInt)
      }.reduce(_|_) | busy_table_wb(r)
    }

    busy_table := busy_table_next
    io.vbusy_status := Cat((0 until numPregs).map(p => busy_table(p).orR).reverse)
    // for better timing, consider following
    //io.vbusy_status := RegNext(Cat((0 until numPregs).map(p => busy_table(p).orR)))
    io.debug.busytable := busy_table
}

class MatBusyResp(val nBits: Int) extends Bundle
{
  val prs1_busy = UInt(nBits.W)
  val prs2_busy = UInt(nBits.W)
  val prs3_busy = UInt(nBits.W)
}

class MatRenameBusyTable(
                       val plWidth: Int,
                       val numPregs: Int,
                       val bypass: Boolean,
                       val numWbPorts: Int)
                     (implicit p: Parameters) extends BoomModule {
  val pregSz = log2Ceil(numPregs)

  val io = IO(new BoomBundle()(p) {
    val ren_uops = Input(Vec(plWidth, new MicroOp))
    val rebusy_reqs = Input(Vec(plWidth, Bool()))
    val busy_resps = Output(Vec(plWidth, new MatBusyResp(vLenb + 1))) //the first bit indicate slice direction
    // whole tile register busy status
    val tbusy_status = Output(UInt(numPregs.W))

    val wb_pdsts = Input(Vec(numWbPorts, UInt(pregSz.W)))
    val wb_valids = Input(Vec(numWbPorts, Bool()))
    val wb_bits = Input(Vec(numWbPorts, UInt(vLenb.W)))   //write back without direction bit?

    val matrix_iss_valid = Input(Vec(matWidth,Bool()))
    val matrix_iss_uop = Input(Vec(matWidth,new MicroOp()))

    val mem_iss_valid = Input(Vec(memWidth,Bool()))
    val mem_iss_uop = Input(Vec(memWidth,new MicroOp()))

    val wake_issue_prs = Output(Vec(memWidth + matWidth,UInt((vLenb+1).W)))
    val wake_issue_data = Output(Vec(memWidth + matWidth,UInt((vLenb+1).W)))
    val wake_issue_valid = Output(Vec(memWidth + matWidth,Bool()))
    val wake_issue_rs_type = Output(Vec(memWidth + matWidth,UInt(RT_X.getWidth.W)))

    val debug = new Bundle {
      val busytable = Output(Vec(numPregs, UInt((vLenb + 1).W)))
    }
  })

  val busy_table = RegInit(VecInit(Seq.fill(numPregs) {0.U((vLenb+1).W)}))
  val busy_table_wb = Wire(Vec(numPregs, UInt((vLenb+1).W)))
  val busy_table_next = Wire(Vec(numPregs, UInt((vLenb+1).W)))
  
  for (r <- 0 until numPregs) {
    // Unbusy written back registers.
    busy_table_wb(r) := busy_table(r) & ~(io.wb_pdsts zip io.wb_valids zip io.wb_bits)
      .map { case ((pdst, valid), bits) => bits & Fill(vLenb, (r.U === pdst && valid).asUInt()) }.reduce(_ | _)
    // Rebusy newly allocated registers.
    busy_table_next(r) := busy_table_wb(r) | (io.ren_uops zip io.rebusy_reqs)
      .map { case (uop, req) => Cat(Mux((r.U === uop.pdst) && req, uop.isHSlice, busy_table(r)(vLenb)) ,
        Fill(vLenb, ((r.U === uop.pdst) && req).asUInt()) &
          Mux(uop.dst_rtype === RT_ACC && !uop.m_is_split, Fill(vLenb, 1.U(1.W)), MaskGen(uop.m_sidx, uop.m_slice_cnt, vLenb))) }.reduce(_ | _)

           
    // Read the busy table.
    for (i <- 0 until plWidth) {                                                                                                                                                                                                                                                                                                                                                                                                                                                          
      val prs1_was_bypassed = (0 until i).map(j =>
        (io.ren_uops(i).lrs1 === io.ren_uops(j).ldst) && (io.ren_uops(i).m_sidx === io.ren_uops(j).m_sidx)
         && (io.ren_uops(i).isHSlice === io.ren_uops(j).isHSlice) && io.rebusy_reqs(j)).foldLeft(false.B)(_ || _)
      val prs2_was_bypassed = (0 until i).map(j =>
        io.ren_uops(i).lrs2 === io.ren_uops(j).ldst && (io.ren_uops(i).m_sidx === io.ren_uops(j).m_sidx)
         && (io.ren_uops(i).isHSlice === io.ren_uops(j).isHSlice) && io.rebusy_reqs(j)).foldLeft(false.B)(_ || _)
      val prs3_was_bypassed = (0 until i).map(j =>
        io.ren_uops(i).lrs3 === io.ren_uops(j).ldst && (io.ren_uops(i).m_sidx === io.ren_uops(j).m_sidx)
         && (io.ren_uops(i).isHSlice === io.ren_uops(j).isHSlice) && io.rebusy_reqs(j)).foldLeft(false.B)(_ || _)

      io.busy_resps(i).prs1_busy := busy_table(io.ren_uops(i).prs1) | Fill(vLenb, (prs1_was_bypassed && bypass.B).asUInt)
      io.busy_resps(i).prs2_busy := busy_table(io.ren_uops(i).prs2) | Fill(vLenb, (prs2_was_bypassed && bypass.B).asUInt)
      io.busy_resps(i).prs3_busy := busy_table(io.ren_uops(i).prs3) | Fill(vLenb, (prs3_was_bypassed && bypass.B).asUInt)
    }
    busy_table := busy_table_next
    io.debug.busytable := busy_table
  }

  for ( i<- 0 until (memWidth + matWidth)) {
    io.wake_issue_valid(i) := false.B
   }

  val matrix_value = Wire(Vec(matWidth, (UInt((vLenb+1).W))))
  val matrix_wake_pdst = Wire(Vec(matWidth,(UInt(pregSz.W))))
  val matrix_wake_type = Wire(Vec(matWidth,UInt(RT_X.getWidth.W)))

  for ( i <- 0 until matWidth) {
    matrix_value(i) := 0.U
    matrix_wake_pdst(i)  := 0.U
    matrix_wake_type(i) := 0.U
    when(~(io.matrix_iss_uop(i).dst_rtype === RT_ACC && !io.matrix_iss_uop(i).m_is_split) && (io.matrix_iss_valid(i))) {
        matrix_value(i)  :=  busy_table_next(io.matrix_iss_uop(i).pdst) & Cat(busy_table_next(io.matrix_iss_uop(i).pdst)(vLenb),Fill(vLenb, 1.U(1.W)) &  MaskGen(0.U, io.matrix_iss_uop(i).m_slice_cnt, vLenb))
        matrix_wake_pdst(i) := io.matrix_iss_uop(i).pdst
        matrix_wake_type(i) := io.matrix_iss_uop(i).dst_rtype
        busy_table(io.matrix_iss_uop(i).pdst) := matrix_value(i)
        io.wake_issue_valid(i) := true.B
        io.wake_issue_prs(i) := matrix_wake_pdst(i)
        io.wake_issue_data(i) := matrix_value(i)
        io.wake_issue_rs_type(i) := matrix_wake_type(i)
    }
  }

  val mem_value = Wire(Vec(memWidth, (UInt((vLenb+1).W))))
  val mem_wake_pdst = Wire(Vec(memWidth, (UInt(pregSz.W))))
  val mem_wake_type = Wire(Vec(memWidth,UInt(RT_X.getWidth.W)))
  for (j <-  0 until memWidth) {
    mem_value(j) := 0.U
    mem_wake_pdst(j) := 0.U
    mem_wake_type(j) := 0.U
    when(~(io.mem_iss_uop(j).dst_rtype === RT_ACC && !io.mem_iss_uop(j).m_is_split) && (io.mem_iss_valid(j))) {
        mem_value(j)  := busy_table_next(io.mem_iss_uop(j).pdst) & Cat(busy_table_next(io.mem_iss_uop(j).pdst)(vLenb),Fill(vLenb, 1.U(1.W)) &  MaskGen(0.U, io.mem_iss_uop(j).m_slice_cnt, vLenb))
        mem_wake_pdst(j) := io.mem_iss_uop(j).pdst
        mem_wake_type(j) := io.mem_iss_uop(j).dst_rtype
        busy_table(io.mem_iss_uop(j).pdst) := mem_value(j)
        io.wake_issue_valid(matWidth + j) := true.B
        io.wake_issue_prs(matWidth + j) := mem_wake_pdst(j)
        io.wake_issue_data(matWidth + j):= mem_value(j)
        io.wake_issue_rs_type(matWidth + j):= mem_wake_type(j)
    }
  }
  io.tbusy_status := Cat((0 until numPregs).map(p => busy_table(p)(vLenb-1, 0).orR).reverse)
}


