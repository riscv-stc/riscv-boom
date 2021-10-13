//******************************************************************************
// Copyright (c) 2012 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// RISCV Processor Datapath: Rename Logic
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// Supports 1-cycle and 2-cycle latencies. (aka, passthrough versus registers between ren1 and ren2).
//    - ren1: read the map tables and allocate a new physical register from the freelist.
//    - ren2: read the busy table for the physical operands.
//
// Ren1 data is provided as an output to be fed directly into the ROB.

package boom.exu

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.Parameters
import freechips.rocketchip.util._
import freechips.rocketchip.tile.{TileKey, XLen}

import boom.common._
import boom.util._

/**
 * IO bundle to interface with the Register Rename logic
 *
 * @param plWidth pipeline width
 * @param numIntPregs number of int physical registers
 * @param numFpPregs number of FP physical registers
 * @param numWbPorts number of int writeback ports
 * @param numWbPorts number of FP writeback ports
 */
class RenameStageIO(
  val plWidth: Int,
  val numPhysRegs: Int,
  val numWbPorts: Int)
  (implicit p: Parameters) extends BoomBundle


/**
 * IO bundle to debug the rename stage
 */
class DebugRenameStageIO(val numPhysRegs: Int, val vector: Boolean = false)(implicit p: Parameters) extends BoomBundle
{
  val freelist  = Bits(numPhysRegs.W)
  val isprlist  = Bits(numPhysRegs.W)
  val busytable = if (vector) Vec(numPhysRegs, UInt(vLenb.W)) else UInt(numPhysRegs.W)
}

abstract class AbstractRenameStage(
  plWidth: Int,
  numPhysRegs: Int,
  numWbPorts: Int,
  val vector: Boolean = false)
  (implicit p: Parameters) extends BoomModule
{
  val io = IO(new Bundle {
    val ren_stalls = Output(Vec(plWidth, Bool()))

    val kill = Input(Bool())

    val dec_fire  = Input(Vec(plWidth, Bool())) // will commit state updates
    val dec_uops  = Input(Vec(plWidth, new MicroOp()))

    // physical specifiers available AND busy/ready status available.
    val ren2_mask = Vec(plWidth, Output(Bool())) // mask of valid instructions
    val ren2_uops = Vec(plWidth, Output(new MicroOp()))

    // branch resolution (execute)
    val brupdate = Input(new BrUpdateInfo())

    val dis_fire  = Input(Vec(coreWidth, Bool()))
    val dis_ready = Input(Bool())

    // wakeup ports
    val wakeups = Flipped(Vec(numWbPorts, Valid(new ExeUnitResp(xLen))))

    // commit stage
    val com_valids = Input(Vec(plWidth, Bool()))
    val com_uops = Input(Vec(plWidth, new MicroOp()))
    val rbk_valids = Input(Vec(plWidth, Bool()))
    val rollback = Input(Bool())
    val vbusy_status = if (vector) Output(UInt(numPhysRegs.W)) else Output(UInt(0.W))

    val debug_rob_empty = Input(Bool())
    val debug = Output(new DebugRenameStageIO(numPhysRegs, vector))
  })

  def BypassAllocations(uop: MicroOp, older_uops: Seq[MicroOp], alloc_reqs: Seq[Bool]): MicroOp

  //-------------------------------------------------------------
  // Pipeline State & Wires

  // Stage 1
  val ren1_fire       = Wire(Vec(plWidth, Bool()))
  val ren1_uops       = Wire(Vec(plWidth, new MicroOp))


  // Stage 2
  val ren2_fire       = io.dis_fire
  val ren2_ready      = io.dis_ready
  val ren2_valids     = Wire(Vec(plWidth, Bool()))
  val ren2_uops       = Wire(Vec(plWidth, new MicroOp))
  val ren2_alloc_reqs = Wire(Vec(plWidth, Bool()))


  //-------------------------------------------------------------
  // pipeline registers

  for (w <- 0 until plWidth) {
    ren1_fire(w)          := io.dec_fire(w)
    ren1_uops(w)          := io.dec_uops(w)
  }

  for (w <- 0 until plWidth) {
    val r_valid  = RegInit(false.B)
    val r_uop    = Reg(new MicroOp)
    val next_uop = Wire(new MicroOp)

    next_uop := r_uop

    when (io.kill) {
      r_valid := false.B
    } .elsewhen (ren2_ready) {
      r_valid := ren1_fire(w)
      next_uop := ren1_uops(w)
    } .otherwise {
      r_valid := r_valid && !ren2_fire(w) // clear bit if uop gets dispatched
      next_uop := r_uop
    }

    r_uop := GetNewUopAndBrMask(BypassAllocations(next_uop, ren2_uops, ren2_alloc_reqs), io.brupdate)

    ren2_valids(w) := r_valid
    ren2_uops(w)   := r_uop
  }

  //-------------------------------------------------------------
  // Outputs

  io.ren2_mask := ren2_valids


}

class RenameBypass(
  val oldWidth: Int,
  val float: Boolean = false)
(implicit p: Parameters) extends BoomModule {
  val io = IO(new Bundle{
    val i_uop = Input(new MicroOp)
    val older_uops = Input(Vec(oldWidth, new MicroOp()))
    val alloc_reqs = Input(Vec(oldWidth, Bool()))
    val o_uop = Output(new MicroOp)
  })
  val bypassed_uop = Wire(new MicroOp)
  val older_uops = io.older_uops
  val alloc_reqs = io.alloc_reqs
  val uop = io.i_uop
  bypassed_uop := uop

  val bypass_hits_rs1 = (older_uops zip alloc_reqs) map { case (o,a) => a && o.ldst === uop.lrs1 }
  val bypass_hits_rs2 = (older_uops zip alloc_reqs) map { case (o,a) => a && o.ldst === uop.lrs2 }
  val bypass_hits_dst = (older_uops zip alloc_reqs) map { case (o,a) => a && o.ldst === uop.ldst }
  val bypass_hits_rs3 = (older_uops zip alloc_reqs) map { case (o,a) => a && o.ldst === uop.lrs3 && float.B}

  val bypass_sel_rs1 = PriorityEncoderOH(bypass_hits_rs1.reverse).reverse
  val bypass_sel_rs2 = PriorityEncoderOH(bypass_hits_rs2.reverse).reverse
  val bypass_sel_rs3 = PriorityEncoderOH(bypass_hits_rs3.reverse).reverse
  val bypass_sel_dst = PriorityEncoderOH(bypass_hits_dst.reverse).reverse

  val do_bypass_rs1 = bypass_hits_rs1.reduce(_||_)
  val do_bypass_rs2 = bypass_hits_rs2.reduce(_||_)
  val do_bypass_rs3 = bypass_hits_rs3.reduce(_||_)
  val do_bypass_dst = bypass_hits_dst.reduce(_||_)


  val bypass_pdsts = older_uops.map(_.pdst)
  when (do_bypass_rs1) { bypassed_uop.prs1       := Mux1H(bypass_sel_rs1, bypass_pdsts) }
  when (do_bypass_rs2) { bypassed_uop.prs2       := Mux1H(bypass_sel_rs2, bypass_pdsts) }
  when (do_bypass_rs3) { bypassed_uop.prs3       := Mux1H(bypass_sel_rs3, bypass_pdsts) }
  when (do_bypass_dst) { bypassed_uop.stale_pdst := Mux1H(bypass_sel_dst, bypass_pdsts) }

  bypassed_uop.prs1_busy := uop.prs1_busy | do_bypass_rs1
  bypassed_uop.prs2_busy := uop.prs2_busy | do_bypass_rs2
  bypassed_uop.prs3_busy := uop.prs3_busy | do_bypass_rs3

  if (!float) {
    bypassed_uop.prs3      := DontCare
    bypassed_uop.prs3_busy := 0.U
  }

  io.o_uop := bypassed_uop
}

/**
 * Rename stage that connets the map table, free list, and busy table.
 * Can be used in both the FP pipeline and the normal execute pipeline.
 *
 * @param plWidth pipeline width
 * @param numWbPorts number of int writeback ports
 * @param numWbPorts number of FP writeback ports
 */
class RenameStage(
  plWidth: Int,
  numPhysRegs: Int,
  numWbPorts: Int,
  float: Boolean)(implicit p: Parameters)
extends AbstractRenameStage(
  plWidth,
  numPhysRegs,
  numWbPorts)(p)
{
  val pregSz = log2Ceil(numPhysRegs)
  val rtype: UInt => Bool = if (float) isFloat else isInt

  //-------------------------------------------------------------
  // Helper Functions

  def BypassAllocations(uop: MicroOp, older_uops: Seq[MicroOp], alloc_reqs: Seq[Bool]): MicroOp = {
    require(older_uops.length == alloc_reqs.length)
    val bypassed_uop = Wire(new MicroOp)
    val bypLogic = Module(new RenameBypass(older_uops.length, float))
    bypLogic.io.i_uop := uop
    bypLogic.io.older_uops := older_uops
    bypLogic.io.alloc_reqs := alloc_reqs
    bypassed_uop := bypLogic.io.o_uop
    bypassed_uop
  }

  //-------------------------------------------------------------
  // Rename Structures

  val maptable = Module(new RenameMapTable(
    plWidth,
    32,
    numPhysRegs,
    false,
    float))
  val freelist = Module(new RenameFreeList(
    plWidth,
    numPhysRegs,
    if (float) 32 else 31))
  val busytable = Module(new RenameBusyTable(
    plWidth,
    numPhysRegs,
    numWbPorts,
    false,
    float))

  val ren2_br_tags    = Wire(Vec(plWidth, Valid(UInt(brTagSz.W))))

  // Commit/Rollback
  val com_valids      = Wire(Vec(plWidth, Bool()))
  val rbk_valids      = Wire(Vec(plWidth, Bool()))

  for (w <- 0 until plWidth) {
    ren2_alloc_reqs(w) := ren2_uops(w).ldst_val && ren2_uops(w).rt(RD, rtype) && ren2_fire(w)
    ren2_br_tags(w).valid := ren2_fire(w) && ren2_uops(w).allocate_brtag

    com_valids(w) := io.com_uops(w).ldst_val && io.com_uops(w).rt(RD, rtype) && io.com_valids(w)
    rbk_valids(w) := io.com_uops(w).ldst_val && io.com_uops(w).rt(RD, rtype) && io.rbk_valids(w)
    ren2_br_tags(w).bits  := ren2_uops(w).br_tag
  }

  //-------------------------------------------------------------
  // Rename Table

  // Maptable inputs.
  val map_reqs   = Wire(Vec(plWidth, new MapReq(lregSz)))
  val remap_reqs = Wire(Vec(plWidth, new RemapReq(lregSz, pregSz)))

  // Generate maptable requests.
  for ((((ren1,ren2),com),w) <- ren1_uops zip ren2_uops zip io.com_uops.reverse zipWithIndex) {
    map_reqs(w).lrs1 := ren1.lrs1
    map_reqs(w).lrs2 := ren1.lrs2
    map_reqs(w).lrs3 := ren1.lrs3
    map_reqs(w).ldst := ren1.ldst

    remap_reqs(w).ldst := Mux(io.rollback, com.ldst      , ren2.ldst)
    remap_reqs(w).pdst := Mux(io.rollback, com.stale_pdst, ren2.pdst)
  }
  ren2_alloc_reqs zip rbk_valids.reverse zip remap_reqs map {
    case ((a,r),rr) => rr.valid := a || r}

  // Hook up inputs.
  maptable.io.map_reqs    := map_reqs
  maptable.io.remap_reqs  := remap_reqs
  maptable.io.ren_br_tags := ren2_br_tags
  maptable.io.brupdate    := io.brupdate
  maptable.io.rollback    := io.rollback

  // Maptable outputs.
  for ((uop, w) <- ren1_uops.zipWithIndex) {
    val mappings = maptable.io.map_resps(w)

    uop.prs1       := mappings.prs1
    uop.prs2       := mappings.prs2
    uop.prs3       := mappings.prs3
    uop.stale_pdst := mappings.stale_pdst
  }

  //-------------------------------------------------------------
  // Free List

  // Freelist inputs.
  for (w <- 0 until plWidth) {
    freelist.io.reqs(w).valid := ren2_alloc_reqs(w)
    freelist.io.reqs(w).bits  := ren2_uops(w)
  }
  freelist.io.dealloc_pregs zip com_valids zip rbk_valids map
    {case ((d,c),r) => d.valid := c || r}
  freelist.io.dealloc_pregs zip io.com_uops map
    {case (d,c) => d.bits := Mux(io.rollback, c.pdst, c.stale_pdst)}
  assert (ren2_alloc_reqs zip freelist.io.alloc_pregs map {case (r,p) => !r || p.bits =/= 0.U} reduce (_&&_),
         "[rename-stage] A uop is trying to allocate the zero physical register.")
  freelist.io.ren_br_tags := ren2_br_tags
  freelist.io.brupdate := io.brupdate
  freelist.io.debug.pipeline_empty := io.debug_rob_empty

  //val prs1, prs2, prs3, stale_pdst, pdst, prvm = Reg(Vec(plWidth, UInt(maxPregSz.W)))
  // Freelist outputs.
  for ((uop, w) <- ren2_uops.zipWithIndex) {
    val preg = freelist.io.alloc_pregs(w).bits
    uop.pdst := Mux(uop.ldst =/= 0.U || float.B, preg, 0.U)
  }

  //-------------------------------------------------------------
  // Busy Table

  busytable.io.ren_uops := ren2_uops  // expects pdst to be set up.
  busytable.io.rebusy_reqs := ren2_alloc_reqs
  busytable.io.wb_valids := io.wakeups.map(_.valid)
  busytable.io.wb_pdsts := io.wakeups.map(_.bits.uop.pdst)

  assert (!(io.wakeups.map(x => x.valid && !x.bits.uop.rt(RD, rtype)).reduce(_||_)),
   "[rename] Wakeup has wrong rtype.")

  for ((uop, w) <- ren2_uops.zipWithIndex) {
    val busy = busytable.io.busy_resps(w)
    uop.prs1_busy := uop.rt(RS1, rtype) && busy.prs1_busy
    uop.prs2_busy := uop.rt(RS2, rtype) && busy.prs2_busy
    uop.prs3_busy := uop.frs3_en && busy.prs3_busy

    val valid = ren2_valids(w)
    assert (!(valid && busy.prs1_busy && !float.B && uop.lrs1 === 0.U), "[rename] x0 is busy??")
    assert (!(valid && busy.prs2_busy && !float.B && uop.lrs2 === 0.U), "[rename] x0 is busy??")
  }

  //-------------------------------------------------------------
  // Outputs

  for (w <- 0 until plWidth) {
    val can_allocate = freelist.io.alloc_pregs(w).valid

    // Push back against Decode stage if Rename1 can't proceed.
    io.ren_stalls(w) := (ren2_uops(w).rt(RD, rtype)) && !can_allocate

    val bypassed_uop = Wire(new MicroOp)
    if (w > 0) bypassed_uop := BypassAllocations(ren2_uops(w), ren2_uops.slice(0,w), ren2_alloc_reqs.slice(0,w))
    else       bypassed_uop := ren2_uops(w)

    io.ren2_uops(w) := GetNewUopAndBrMask(bypassed_uop, io.brupdate)
  }

  //-------------------------------------------------------------
  // Debug signals

  io.debug.freelist  := freelist.io.debug.freelist
  io.debug.isprlist  := freelist.io.debug.isprlist
  io.debug.busytable := busytable.io.debug.busytable
}

class PredRenameStage(
  plWidth: Int,
  numPhysRegs: Int,
  numWbPorts: Int)
  (implicit p: Parameters) extends AbstractRenameStage(plWidth, numPhysRegs, numWbPorts)(p)
{

  def BypassAllocations(uop: MicroOp, older_uops: Seq[MicroOp], alloc_reqs: Seq[Bool]): MicroOp = {
    uop
  }

  ren2_alloc_reqs := DontCare

  val busy_table = RegInit(VecInit(0.U(ftqSz.W).asBools))
  val to_busy = WireInit(VecInit(0.U(ftqSz.W).asBools))
  val unbusy = WireInit(VecInit(0.U(ftqSz.W).asBools))

  val current_ftq_idx = Reg(UInt(log2Ceil(ftqSz).W))
  var next_ftq_idx = current_ftq_idx

  for (w <- 0 until plWidth) {
    io.ren2_uops(w) := ren2_uops(w)

    val is_sfb_br = ren2_uops(w).is_sfb_br && ren2_fire(w)
    val is_sfb_shadow = ren2_uops(w).is_sfb_shadow && ren2_fire(w)

    val ftq_idx = ren2_uops(w).ftq_idx
    when (is_sfb_br) {
      io.ren2_uops(w).pdst := ftq_idx
      to_busy(ftq_idx) := true.B
    }
    next_ftq_idx = Mux(is_sfb_br, ftq_idx, next_ftq_idx)

    when (is_sfb_shadow) {
      io.ren2_uops(w).ppred := next_ftq_idx
      io.ren2_uops(w).ppred_busy := (busy_table(next_ftq_idx) || to_busy(next_ftq_idx)) && !unbusy(next_ftq_idx)
    }
  }

  for (w <- 0 until numWbPorts) {
    when (io.wakeups(w).valid) {
      unbusy(io.wakeups(w).bits.uop.pdst) := true.B
    }
  }

  current_ftq_idx := next_ftq_idx

  busy_table := ((busy_table.asUInt | to_busy.asUInt) & ~unbusy.asUInt).asBools
}

class VecRenameBypass(
  val oldWidth: Int)
(implicit p: Parameters) extends BoomModule {
  val io = IO(new Bundle{
    val i_uop = Input(new MicroOp)
    val older_uops = Input(Vec(oldWidth, new MicroOp()))
    val alloc_reqs = Input(Vec(oldWidth, Bool()))
    val o_uop = Output(new MicroOp)
  })
  val bypassed_uop = Wire(new MicroOp)
  val older_uops = io.older_uops
  val alloc_reqs = io.alloc_reqs
  val uop = io.i_uop
  bypassed_uop := uop

  def idIsActive(emul: UInt, nf: UInt, idInGroup: UInt): Bool = {
    val grpCount = nf << Mux(emul(2), 0.U, emul(1,0))
    val ret: Bool = idInGroup < grpCount
    ret
  }
  def grpCount(emul: UInt, nf: UInt): UInt = {
    val ret = nf << Mux(emul(2), 0.U, emul(1,0))
    ret
  }

  def isInGroup(rs: UInt, grpRS: UInt, grpEMUL: UInt = 0.U, grpNF: UInt = 1.U): Bool = {
    val ret: Bool = (rs >= grpRS) && (rs < grpRS + grpCount(grpEMUL, grpNF))
    ret
  }

  val bypass_hits_rs1 = (older_uops zip alloc_reqs) map { case (o,a) => {
      // returns a vector of Bool, one for each reg in a group
      Cat((0 until 8).map(r => a &&                                                       // it requests alloc
                               r.U < grpCount(uop.vs1_emul, 1.U) &&                       // vs1 is active in the group
                               uop.rt(RS1, isVector) &&                                   // uop.vs1 is Vector
                               o.rt(RD, isVector) &&                                      // older uop vd is vector
                               o.ldst_val &&                                              // older uop updates vd
                               isInGroup(uop.lrs1 + r.U, o.ldst, o.vd_emul, o.v_seg_nf))) // vs1 matches one in the older dst group
  }}
  val bypass_hits_rs2 = (older_uops zip alloc_reqs) map { case (o,a) => {
      Cat((0 until 8).map(r => a &&
                               r.U < grpCount(uop.vs2_emul, 1.U) &&
                               uop.rt(RS2, isVector) &&
                               o.rt(RD, isVector) &&
                               o.ldst_val &&
                               isInGroup(uop.lrs2 + r.U, o.ldst, o.vd_emul, o.v_seg_nf)))
  }}
  val bypass_hits_dst = (older_uops zip alloc_reqs) map { case (o,a) => {
      Cat((0 until 8).map(r => a &&
                               r.U < grpCount(uop.vd_emul, uop.v_seg_nf) &&
                               uop.rt(RD, isVector) &&
                               o.rt(RD, isVector) &&
                               o.ldst_val &&
                               isInGroup(uop.ldst + r.U, o.ldst, o.vd_emul, o.v_seg_nf)))
  }}
  val bypass_hits_vs0 = (older_uops zip alloc_reqs) map { case (o,a) => a && o.ldst === 0.U }

  val bypass_sel_rs1 = (0 until 8).map(r => PriorityEncoderOH(bypass_hits_rs1.map(b => b(r)).reverse).reverse)
  val bypass_sel_rs2 = (0 until 8).map(r => PriorityEncoderOH(bypass_hits_rs2.map(b => b(r)).reverse).reverse)
  val bypass_sel_dst = (0 until 8).map(r => PriorityEncoderOH(bypass_hits_dst.map(b => b(r)).reverse).reverse)
  val bypass_sel_vs0 = PriorityEncoderOH(bypass_hits_vs0.reverse).reverse

  val do_bypass_rs1 = (0 until 8).map(r => bypass_sel_rs1(r).reduce(_ || _))
  val do_bypass_rs2 = (0 until 8).map(r => bypass_sel_rs2(r).reduce(_ || _))
  val do_bypass_dst = (0 until 8).map(r => bypass_sel_dst(r).reduce(_ || _))
  val do_bypass_vs0 = bypass_hits_vs0.reduce(_||_)

  val bypass_pvd = older_uops.map(_.pvd)
  for (i <- 0 until 8) {
    when (do_bypass_rs1(i)) { bypassed_uop.pvs1(i).bits      := Mux1H(bypass_sel_rs1(i), bypass_pvd.map(b => b(i).bits)) }
    when (do_bypass_rs2(i)) { bypassed_uop.pvs2(i).bits      := Mux1H(bypass_sel_rs2(i), bypass_pvd.map(b => b(i).bits)) }
    when (do_bypass_dst(i)) { bypassed_uop.stale_pvd(i).bits := Mux1H(bypass_sel_dst(i), bypass_pvd.map(b => b(i).bits)) }
  }
  when (do_bypass_vs0)    { bypassed_uop.pvm               := Mux1H(bypass_sel_vs0, bypass_pvd.map(b => b(0).bits)) }

  bypassed_uop.prs1_busy := DontCare
  bypassed_uop.prs2_busy := DontCare
  bypassed_uop.prs3_busy := DontCare
  bypassed_uop.prs3      := DontCare
  io.o_uop := bypassed_uop
  bypassed_uop.prs3      := DontCare
}

/**
 * Rename stage that connets the map table, free list, and busy table.
 * Can be used in both the FP pipeline and the normal execute pipeline.
 *
 * @param plWidth pipeline width
 * @param numWbPorts number of int writeback ports
 * @param numWbPorts number of FP writeback ports
 */
class VecRenameStage(
  plWidth: Int,
  numPhysRegs: Int,
  numWbPorts: Int)(implicit p: Parameters)
extends AbstractRenameStage(
  plWidth,
  numPhysRegs,
  numWbPorts,
  true)(p)
{
  val pregSz = log2Ceil(numPhysRegs)
  val rtype: UInt => Bool = isVector

  //-------------------------------------------------------------
  // Helper Functions

  def BypassAllocations(uop: MicroOp, older_uops: Seq[MicroOp], alloc_reqs: Seq[Bool]): MicroOp = {
    require(older_uops.length == alloc_reqs.length)
    val bypassed_uop = Wire(new MicroOp)
    val bypLogic = Module(new VecRenameBypass(older_uops.length))
    bypLogic.io.i_uop := uop
    bypLogic.io.older_uops := older_uops
    bypLogic.io.alloc_reqs := alloc_reqs
    bypassed_uop := bypLogic.io.o_uop
    bypassed_uop
  }

  //-------------------------------------------------------------
  // Rename Structures

  val maptable = Module(new VecRenameMapTable(
    plWidth,
    32,
    numPhysRegs,
    false))
  val freelist = Module(new VecRenameFreeList(
    plWidth,
    numPhysRegs,
    32))
  val busytable = Module(new VecRenameBusyTable(
    plWidth,
    numPhysRegs,
    numWbPorts))

  val ren2_br_tags    = Wire(Vec(plWidth, Valid(UInt(brTagSz.W))))

  // Commit/Rollback
  val com_valids      = Wire(Vec(plWidth, Bool()))
  val rbk_valids      = Wire(Vec(plWidth, Bool()))

  for (w <- 0 until plWidth) {
    ren2_alloc_reqs(w) := ren2_uops(w).ldst_val && ren2_uops(w).rt(RD, rtype) && ren2_fire(w)
    ren2_br_tags(w).valid := ren2_fire(w) && ren2_uops(w).allocate_brtag

    com_valids(w) := io.com_uops(w).ldst_val && io.com_uops(w).rt(RD, rtype) && io.com_valids(w)
    rbk_valids(w) := io.com_uops(w).ldst_val && io.com_uops(w).rt(RD, rtype) && io.rbk_valids(w)
    ren2_br_tags(w).bits  := ren2_uops(w).br_tag
  }

  //-------------------------------------------------------------
  // Rename Table

  // Maptable inputs.
  val map_reqs   = Wire(Vec(plWidth, new VecMapReq(lregSz)))
  val remap_reqs = Wire(Vec(plWidth, new VecRemapReq(lregSz, pregSz)))

  // Generate maptable requests.
  for ((((ren1,ren2),com),w) <- ren1_uops zip ren2_uops zip io.com_uops.reverse zipWithIndex) {
    map_reqs(w).lrs1 := ren1.lrs1
    map_reqs(w).lrs2 := ren1.lrs2
    map_reqs(w).lrs3 := ren1.lrs3
    map_reqs(w).ldst := ren1.ldst
    map_reqs(w).vs1_emul := ren1.vs1_emul
    map_reqs(w).vs2_emul := ren1.vs2_emul
    map_reqs(w).vd_emul  := ren1.vd_emul
    map_reqs(w).v_seg_nf := ren1.v_seg_nf

    remap_reqs(w).ldst := Mux(io.rollback, com.ldst, ren2.ldst)
    for (i <- 0 until 8) {
      remap_reqs(w).pdst(i) := Mux(io.rollback, com.stale_pvd(i).bits, ren2.pvd(i).bits )
    }
    remap_reqs(w).vd_emul  := Mux(io.rollback, com.vd_emul,  ren2.vd_emul)
    remap_reqs(w).v_seg_nf := Mux(io.rollback, com.v_seg_nf, ren2.v_seg_nf)
  }
  ren2_alloc_reqs zip rbk_valids.reverse zip remap_reqs map {
    case ((a,r),rr) => rr.valid := a || r}

  // Hook up inputs.
  maptable.io.map_reqs    := map_reqs
  maptable.io.remap_reqs  := remap_reqs
  maptable.io.ren_br_tags := ren2_br_tags
  maptable.io.brupdate    := io.brupdate
  maptable.io.rollback    := io.rollback

  // Maptable outputs.
  for ((uop, w) <- ren1_uops.zipWithIndex) {
    val mappings = maptable.io.map_resps(w)
    uop.pvs1       := mappings.prs1
    uop.pvs2       := mappings.prs2
    uop.stale_pvd  := mappings.stale_pdst
    uop.pvm        := mappings.prvm
  }

  //-------------------------------------------------------------
  // Free List

  // Freelist inputs.
  for (w <- 0 until plWidth) {
    freelist.io.reqs(w).valid := ren2_alloc_reqs(w)
    freelist.io.reqs(w).bits  := ren2_uops(w)
  }
  (freelist.io.dealloc_pregs zip io.com_uops zip com_valids zip rbk_valids).map { case(((da, uop), c), r) => {
    da.zipWithIndex.map { case (d, i) => {
      d.valid := (c || r) && uop.stale_pvd(i).valid
      d.bits  := Mux(io.rollback, uop.pvd(i).bits, uop.stale_pvd(i).bits)
    }}
  }}
  freelist.io.ren_br_tags := ren2_br_tags
  freelist.io.brupdate := io.brupdate
  freelist.io.debug.pipeline_empty := io.debug_rob_empty

  //val prs1, prs2, prs3, stale_pdst, pdst, prvm = Reg(Vec(plWidth, UInt(maxPregSz.W)))
  // Freelist outputs.
  for ((uop, w) <- ren2_uops.zipWithIndex) {
    uop.pvd := freelist.io.alloc_pregs(w)
  }

  //-------------------------------------------------------------
  // Busy Table

  busytable.io.ren_uops := ren2_uops  // expects pdst to be set up.
  busytable.io.rebusy_reqs := ren2_alloc_reqs
  busytable.io.wb_valids := io.wakeups.map(_.valid)
  busytable.io.wb_pdsts := io.wakeups.map(_.bits.uop.pdst)
  io.vbusy_status := busytable.io.vbusy_status
  for ((bs, wk) <- busytable.io.wb_bits zip io.wakeups) {
    val v_eidx  = wk.bits.uop.v_eidx
    val ecnt    = wk.bits.uop.v_split_ecnt
    val vsew    = wk.bits.uop.vconfig.vtype.vsew(1,0)
    val lsb     = (v_eidx << vsew)(vLenSz-1, 0)
    val len     = ecnt << vsew
    val rmask   = boom.util.MaskGen(v_eidx, len, vLen)
    bs := rmask
  }

  assert (!(io.wakeups.map(x => x.valid && !x.bits.uop.rt(RD, rtype)).reduce(_||_)),
   "[rename] Wakeup has wrong rtype.")

  //-------------------------------------------------------------
  // Outputs

  for (w <- 0 until plWidth) {
    val can_allocate = freelist.io.alloc_pregs(w).map(_.valid).reduce(_ || _)

    // Push back against Decode stage if Rename1 can't proceed.
    io.ren_stalls(w) := (ren2_uops(w).rt(RD, rtype)) && !can_allocate

    val bypassed_uop = Wire(new MicroOp)
    if (w > 0) bypassed_uop := BypassAllocations(ren2_uops(w), ren2_uops.slice(0,w), ren2_alloc_reqs.slice(0,w))
    else       bypassed_uop := ren2_uops(w)

    io.ren2_uops(w) := GetNewUopAndBrMask(bypassed_uop, io.brupdate)
  }

  //-------------------------------------------------------------
  // Debug signals

  io.debug.freelist  := freelist.io.debug.freelist
  io.debug.isprlist  := freelist.io.debug.isprlist
  io.debug.busytable := busytable.io.debug.busytable
}

import freechips.rocketchip.rocket._
import freechips.rocketchip.config._
import freechips.rocketchip.unittest._
import boom.ifu._
import boom.lsu._

class VecRenameUT(timeout: Int = 1000)(implicit p: Parameters) extends UnitTest(timeout)
{
  def NullBrUpdateInfo: BrUpdateInfo = {
    val ret = Wire(new BrUpdateInfo)
    ret.b1.resolve_mask     := 0.U
    ret.b1.mispredict_mask  := 0.U
    ret.b2.uop              := NullMicroOp(true)
    ret.b2.valid            := false.B
    ret.b2.mispredict       := false.B
    ret.b2.taken            := false.B
    ret.b2.cfi_type         := 0.U
    ret.b2.pc_sel           := 0.U
    ret.b2.jalr_target      := 0.U
    ret.b2.target_offset    := 0.S
    ret
  }

  def NullWakeup: Valid[ExeUnitResp] = {
    val ret = Wire(new Valid(new ExeUnitResp(p(XLen))))
    ret.valid                   := false.B
    ret.bits.uop                := NullMicroOp(true)
    ret.bits.data               := 0.U
    ret.bits.predicated         := false.B
    ret.bits.fflags.valid       := false.B
    ret.bits.fflags.bits.uop    := NullMicroOp(true)
    ret.bits.fflags.bits.flags  := 0.U
    ret
  }

  val plWidth: Int = 2
  val numPregs: Int = 64
  val numWbPorts: Int = 1

  val dut = Module(new VecRenameStage(plWidth, numPregs, numWbPorts))
  //val dec_fire = RegInit(VecInit.tabulate(plWidth)(x: Int => false.B))
  //val dec_uops = RegInit(VecInit.tabulate(plWidth)(x: Int => NullMicroOp(true)))
  //val brupdate = Wire(new BrUpdateInfo())
  //val dis_fire = RegInit(VecInit.tabulate(plWidth)(x: Int => false.B))
  //val dis_ready = RegInit(false.B)
  dut.io.kill := false.B
  dut.io.dec_fire.map(x => x := false.B)
  dut.io.dec_uops.map(x => x := NullMicroOp(true))
  dut.io.brupdate := NullBrUpdateInfo
  dut.io.wakeups(0) := NullWakeup
  dut.io.dis_fire.map(x => x := false.B)
  dut.io.dis_ready := false.B
  dut.io.com_valids.map(x => x := false.B)
  dut.io.com_uops.map(x => x := NullMicroOp(true))
  dut.io.rbk_valids.map(x => x := false.B)
  dut.io.rollback := false.B
  dut.io.debug_rob_empty := false.B
}

class WithVecRenameUT extends Config((site, here, up) => {
  case UnitTests => (q: Parameters) => {
    implicit val p = BoomTestUtils.getBoomParameters("StcBoomConfig", "chipyard")
    Seq(
      Module(new VecRenameUT(10000))
    )
  }
})

class VecRenameUTConfig extends Config(new WithVecRenameUT)
