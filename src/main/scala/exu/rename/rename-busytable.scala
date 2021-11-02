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

    val debug = new Bundle {
      val busytable = Output(Vec(numPregs, UInt(vLenb.W)))
    }
  })

    val busy_table = RegInit(VecInit(Seq.fill(numPregs){0.U(vLenb.W)}))
    val busy_table_wb = Wire(Vec(numPregs, UInt(vLenb.W)))
    val busy_table_next = Wire(Vec(numPregs, UInt(vLenb.W)))

    for (r <- 0 until numPregs) {
      busy_table_wb(r) := busy_table(r) & ~(io.wb_pdsts zip io.wb_valids zip io.wb_bits).map {case ((pdst, valid), bits) =>
        bits & Fill(vLenb, (r.U === pdst && valid).asUInt)
      }.reduce(_|_)
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
