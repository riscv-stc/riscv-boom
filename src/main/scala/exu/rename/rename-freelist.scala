//******************************************************************************
// Copyright (c) 2015 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Rename FreeList
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.exu

import chisel3._
import chisel3.util._
import boom.common._
import boom.util._
import freechips.rocketchip.config.Parameters

class RenameFreeList(
  val plWidth: Int,
  val numPregs: Int,
  val numLregs: Int)
  (implicit p: Parameters) extends BoomModule
{
  private val pregSz = log2Ceil(numPregs)
  private val n = numPregs

  val io = IO(new BoomBundle()(p) {
    // Physical register requests.
    val reqs          = Input(Vec(plWidth, Valid(new MicroOp)))
    val alloc_pregs   = Output(Vec(plWidth, Valid(UInt(pregSz.W))))

    // Pregs returned by the ROB.
    val dealloc_pregs = Input(Vec(plWidth, Valid(UInt(pregSz.W))))

    // Branch info for starting new allocation lists.
    val ren_br_tags   = Input(Vec(plWidth, Valid(UInt(brTagSz.W))))

    // Mispredict info for recovering speculatively allocated registers.
    val brupdate        = Input(new BrUpdateInfo)

    val debug = new Bundle {
      val pipeline_empty = Input(Bool())
      val freelist = Output(Bits(numPregs.W))
      val isprlist = Output(Bits(numPregs.W))
    }
  })
  // The free list register array and its branch allocation lists.
  val free_list = RegInit(UInt(numPregs.W), ~(1.U(numPregs.W)))
  val br_alloc_lists = Reg(Vec(maxBrCount, UInt(numPregs.W)))

  // Select pregs from the free list.
  val sels = Wire(Vec(plWidth, UInt(numPregs.W)))
  sels := SelectFirstN(free_list, plWidth)
  val sel_fire = Wire(Vec(plWidth, Bool()))

  // Allocations seen by branches in each pipeline slot.
  val allocs = io.alloc_pregs.map(a => UIntToOH(a.bits))
  val alloc_masks = (allocs zip io.reqs).scanRight(0.U(n.W)) { case ((a,r),m) => m | a & Fill(n,r.valid) }

  // Masks that modify the freelist array.
  val sel_mask = (sels zip sel_fire).map { case (s,f) => s & Fill(n,f) } reduce(_|_)
  val br_deallocs = br_alloc_lists(io.brupdate.b2.uop.br_tag) & Fill(n, io.brupdate.b2.mispredict)
  val dealloc_mask = io.dealloc_pregs.map(dreq => {
    UIntToOH(dreq.bits)(numPregs-1,0) & Fill(n,dreq.valid)
  }).reduce(_|_) | br_deallocs

  val br_slots = VecInit(io.ren_br_tags.map(tag => tag.valid)).asUInt
  // Create branch allocation lists.
  for (i <- 0 until maxBrCount) {
    val list_req = VecInit(io.ren_br_tags.map(tag => UIntToOH(tag.bits)(i))).asUInt & br_slots
    val new_list = list_req.orR
    br_alloc_lists(i) := Mux(new_list, Mux1H(list_req, alloc_masks.slice(1, plWidth+1)),
                                       br_alloc_lists(i) & ~br_deallocs | alloc_masks(0))
  }

  // Update the free list.
  free_list := (free_list & ~sel_mask | dealloc_mask) & ~(1.U(numPregs.W))

  // Pipeline logic | hookup outputs.
  for (w <- 0 until plWidth) {
    val can_sel = sels(w).orR
    val r_valid = RegInit(false.B)
    val r_sel   = RegEnable(PriorityEncoder(sels(w)), sel_fire(w))

    r_valid := r_valid && !io.reqs(w).valid || can_sel
    sel_fire(w) := (!r_valid || io.reqs(w).valid) && can_sel

    io.alloc_pregs(w).bits  := r_sel
    io.alloc_pregs(w).valid := r_valid
  }

  io.debug.freelist := free_list //| io.alloc_pregs.map(p => UIntToOH(p.bits) & Fill(n, p.valid)).reduce(_|_)
  io.debug.isprlist := 0.U  // TODO track commit free list.

  assert (!(io.debug.freelist & dealloc_mask).orR, "[freelist] Returning a free physical register.")
  assert (!io.debug.pipeline_empty || PopCount(io.debug.freelist) >= (numPregs - numLregs - 1).U,
    "[freelist] Leaking physical registers.")
}

class VecRenameFreeList(
  val plWidth: Int,
  val numPregs: Int,
  val numLregs: Int)
  (implicit p: Parameters) extends BoomModule
{
  private val pregSz = log2Ceil(numPregs)
  private val n = numPregs

  val io = IO(new BoomBundle()(p) {
    // Physical register requests.
    val reqs          = Input(Vec(plWidth, Valid(new MicroOp)))
    val alloc_pregs   = Output(Vec(plWidth, Vec(8, Valid(UInt(pregSz.W)))))

    // Pregs returned by the ROB.
    val dealloc_pregs = Input(Vec(plWidth, Vec(8, Valid(UInt(pregSz.W)))))

    // Branch info for starting new allocation lists.
    val ren_br_tags   = Input(Vec(plWidth, Valid(UInt(brTagSz.W))))

    // Mispredict info for recovering speculatively allocated registers.
    val brupdate        = Input(new BrUpdateInfo)

    val debug = new Bundle {
      val pipeline_empty = Input(Bool())
      val freelist = Output(Bits(numPregs.W))
      val isprlist = Output(Bits(numPregs.W))
    }
  })
  // The free list register array and its branch allocation lists.
  val free_list = RegInit(UInt(numPregs.W), ~(1.U(numPregs.W)))
  val br_alloc_lists = Reg(Vec(maxBrCount, UInt(numPregs.W)))

  // Select pregs from the free list.
  val vec_sels = Wire(Vec(plWidth, Vec(8, UInt(numPregs.W))))
  val flat_sels = SelectFirstN(free_list, plWidth*8)
  for (w <- 0 until plWidth) {
    (vec_sels(w) zip flat_sels.slice(8*w, 8*w+8)).map { case(v,f) => v := f }
  }
  val sel_fire = Wire(Vec(plWidth, UInt(8.W)))

  val allocs = io.alloc_pregs.map(a => a.map(ar => Fill(n, ar.valid) & UIntToOH(ar.bits, n)).reduce(_ | _))
  val alloc_masks = (allocs zip io.reqs).scanRight(0.U(n.W)) { case ((a,r),m) => m | a & Fill(n,r.valid) }

  // Masks that modify the freelist array.
  val sel_mask = (vec_sels zip sel_fire).map { case (vs,sf) =>
                   (vs zip sf.asBools).map { case(bm,f) => bm & Fill(n,f) }.reduce(_ | _)
                 }.reduce(_ | _)
  val br_deallocs = br_alloc_lists(io.brupdate.b2.uop.br_tag) & Fill(n, io.brupdate.b2.mispredict)
  val dealloc_mask = io.dealloc_pregs.map(dreq => {
    dreq.map(dr => UIntToOH(dr.bits, n) & Fill(n, dr.valid)).reduce(_ | _)
  }).reduce(_|_) | br_deallocs

  val br_slots = VecInit(io.ren_br_tags.map(tag => tag.valid)).asUInt
  // Create branch allocation lists.
  for (i <- 0 until maxBrCount) {
    val list_req = VecInit(io.ren_br_tags.map(tag => UIntToOH(tag.bits)(i))).asUInt & br_slots
    val new_list = list_req.orR
    br_alloc_lists(i) := Mux(new_list, Mux1H(list_req, alloc_masks.slice(1, plWidth+1)),
                                       br_alloc_lists(i) & ~br_deallocs | alloc_masks(0))
  }

  // Update the free list.
  free_list := (free_list & ~sel_mask | dealloc_mask) & ~(1.U(numPregs.W))

  // Pipeline logic | hookup outputs.
  for (w <- 0 until plWidth) {
    val vgrp = lvdGroup(io.reqs(w).bits.ldst, io.reqs(w).bits.vd_emul, io.reqs(w).bits.v_seg_nf)
    val can_sel = vec_sels(w).zipWithIndex.map { case (vs, i) => vs.orR || !vgrp(i).valid }.reduce(_ && _)
    val r_valid = RegInit(false.B)

    r_valid := r_valid && !io.reqs(w).valid || can_sel
    sel_fire(w) := Fill(8, ((!r_valid || io.reqs(w).valid) && can_sel).asUInt)

    io.alloc_pregs(w).zipWithIndex.map { case(a,i) =>
      a.bits   := RegEnable(PriorityEncoder(vec_sels(w)(i)), sel_fire(w)(i))
      a.valid  := r_valid && vgrp(i).valid
    }
  }

  io.debug.freelist := free_list //| io.alloc_pregs.map(p => UIntToOH(p.bits) & Fill(n, p.valid)).reduce(_|_)
  io.debug.isprlist := 0.U  // TODO track commit free list.

  assert (!(io.debug.freelist & dealloc_mask).orR, "[freelist] Returning a free physical register.")
  assert (!io.debug.pipeline_empty || PopCount(io.debug.freelist) >= (numPregs - numLregs - 1).U,
    "[freelist] Leaking physical registers.")
}
