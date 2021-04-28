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

class BusyResp extends Bundle
{
  val prs1_busy = Bool()
  val prs2_busy = Bool()
  val prs3_busy = Bool()
}

class VecBusyResp(val nBits: Int) extends Bundle {
  val prs1_busy = UInt(nBits.W)
  val prs2_busy = UInt(nBits.W)
  val prs3_busy = UInt(nBits.W)
  val prvm_busy = UInt(nBits.W)
}

class RenameBusyTable(
  val plWidth: Int,
  val numPregs: Int,
  val numWbPorts: Int,
  val bypass: Boolean,
  val float: Boolean, val vector: Boolean)
  (implicit p: Parameters) extends BoomModule
{
  val pregSz = log2Ceil(numPregs)

  val io = IO(new BoomBundle()(p) {
    val ren_uops = Input(Vec(plWidth, new MicroOp))
    val busy_resps = if (!vector) Output(Vec(plWidth, new BusyResp)) else null
    val vbusy_resps = if (vector) Output(Vec(plWidth, new VecBusyResp(vLenb))) else null
    val rebusy_reqs = Input(Vec(plWidth, Bool()))

    val wb_pdsts = Input(Vec(numWbPorts, UInt(pregSz.W)))
    val wb_valids = Input(Vec(numWbPorts, Bool()))
    val wb_bits = if (vector) Input(Vec(numWbPorts, UInt(vLenb.W))) else null

    val debug = new Bundle {
      val busytable = if(vector) Output(Vec(numPregs, UInt(vLenb.W))) else Output(Bits(numPregs.W)) 
    }
  })

  if (vector) {
    val busy_table = RegInit(VecInit(Seq.fill(numPregs){0.U(vLenb.W)}))
    val busy_table_wb = Wire(Vec(numPregs, UInt(vLenb.W)))
    val busy_table_next = Wire(Vec(numPregs, UInt(vLenb.W)))

    for (r <- 0 until numPregs) {
      busy_table_wb(r) := busy_table(r) & ~(io.wb_pdsts zip io.wb_valids zip io.wb_bits)
        .map {case ((pdst, valid), bits) => bits & Fill(vLenb, (r.U === pdst && valid).asUInt)}.reduce(_|_)
      busy_table_next(r) := busy_table_wb(r) | (io.ren_uops zip io.rebusy_reqs)
        .map {case (uop, req) => Fill(vLenb, ((r.U === uop.pdst) && req).asUInt)}.reduce(_|_)
    }

    for (i <- 0 until plWidth) {
      val prs1_was_bypassed = (0 until i).map(j =>
        io.ren_uops(i).lrs1 === io.ren_uops(j).ldst && io.rebusy_reqs(j)).foldLeft(false.B)(_||_)
      val prs2_was_bypassed = (0 until i).map(j =>
        io.ren_uops(i).lrs2 === io.ren_uops(j).ldst && io.rebusy_reqs(j)).foldLeft(false.B)(_||_)
      val prs3_was_bypassed = (0 until i).map(j =>
        io.ren_uops(i).lrs3 === io.ren_uops(j).ldst && io.rebusy_reqs(j)).foldLeft(false.B)(_||_)
      val prvm_was_bypassed = (0 until i).map(j =>
        0.U === io.ren_uops(j).ldst && io.rebusy_reqs(j)).foldLeft(false.B)(_||_)

      io.vbusy_resps(i).prs1_busy := busy_table(io.ren_uops(i).prs1) | Fill(vLenb, (prs1_was_bypassed && bypass.B).asUInt)
      io.vbusy_resps(i).prs2_busy := busy_table(io.ren_uops(i).prs2) | Fill(vLenb, (prs2_was_bypassed && bypass.B).asUInt)
      io.vbusy_resps(i).prs3_busy := busy_table(io.ren_uops(i).prs3) | Fill(vLenb, (prs3_was_bypassed && bypass.B).asUInt)
      io.vbusy_resps(i).prvm_busy := busy_table(io.ren_uops(i).prvm) | Fill(vLenb, (prvm_was_bypassed && bypass.B).asUInt)
    }

    busy_table := busy_table_next
    io.debug.busytable := busy_table
  } else {
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
      if (!float && !vector) io.busy_resps(i).prs3_busy := false.B
    }

    busy_table := busy_table_next
    io.debug.busytable := busy_table
  }
}
