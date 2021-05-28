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
  val float: Boolean = false,
  val vector: Boolean = false)
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

  if (vector) require(usingVector)

  val bypass_hits_rs1 = (older_uops zip alloc_reqs) map { case (r,a) => a && r.ldst === uop.lrs1 }
  val bypass_hits_rs2 = (older_uops zip alloc_reqs) map { case (r,a) => a && r.ldst === uop.lrs2 }
  val bypass_hits_rs3 = (older_uops zip alloc_reqs) map { case (r,a) => a && r.ldst === uop.lrs3 }
  val bypass_hits_vs0 = (older_uops zip alloc_reqs) map { case (r,a) => a && r.ldst === 0.U && vector.B }
  val bypass_hits_dst = (older_uops zip alloc_reqs) map { case (r,a) => a && r.ldst === uop.ldst }

  val bypass_sel_rs1 = PriorityEncoderOH(bypass_hits_rs1.reverse).reverse
  val bypass_sel_rs2 = PriorityEncoderOH(bypass_hits_rs2.reverse).reverse
  val bypass_sel_rs3 = PriorityEncoderOH(bypass_hits_rs3.reverse).reverse
  val bypass_sel_vs0 = PriorityEncoderOH(bypass_hits_vs0.reverse).reverse
  val bypass_sel_dst = PriorityEncoderOH(bypass_hits_dst.reverse).reverse

  val do_bypass_rs1 = bypass_hits_rs1.reduce(_||_)
  val do_bypass_rs2 = bypass_hits_rs2.reduce(_||_)
  val do_bypass_rs3 = bypass_hits_rs3.reduce(_||_)
  val do_bypass_vs0 = bypass_hits_vs0.reduce(_||_)
  val do_bypass_dst = bypass_hits_dst.reduce(_||_)

  val bypass_pdsts = older_uops.map(_.pdst)

  when (do_bypass_rs1) { bypassed_uop.prs1       := Mux1H(bypass_sel_rs1, bypass_pdsts) }
  when (do_bypass_rs2) { bypassed_uop.prs2       := Mux1H(bypass_sel_rs2, bypass_pdsts) }
  when (do_bypass_rs3) { bypassed_uop.prs3       := Mux1H(bypass_sel_rs3, bypass_pdsts) }
  when (do_bypass_dst) { bypassed_uop.stale_pdst := Mux1H(bypass_sel_dst, bypass_pdsts) }

  if (usingVector) {
    if (vector) {
      bypassed_uop.prs1_busy := uop.prs1_busy | Fill(vLenb, do_bypass_rs1.asUInt)
      bypassed_uop.prs2_busy := uop.prs2_busy | Fill(vLenb, do_bypass_rs2.asUInt)
      bypassed_uop.prs3_busy := uop.prs3_busy | Fill(vLenb, do_bypass_rs3.asUInt)
      when (do_bypass_vs0) { bypassed_uop.prvm := Mux1H(bypass_sel_vs0, bypass_pdsts) }
      bypassed_uop.prvm_busy := uop.prvm_busy | Fill(vLenb, do_bypass_vs0.asUInt)
    } else {
      bypassed_uop.prs1_busy := Cat(0.U, (uop.prs1_busy(0) || do_bypass_rs1).asUInt)
      bypassed_uop.prs2_busy := Cat(0.U, (uop.prs2_busy(0) || do_bypass_rs2).asUInt)
      bypassed_uop.prs3_busy := Cat(0.U, (uop.prs3_busy(0) || do_bypass_rs3).asUInt)
      bypassed_uop.prvm_busy := 0.U
    }
  } else {
    bypassed_uop.prs1_busy := uop.prs1_busy | do_bypass_rs1
    bypassed_uop.prs2_busy := uop.prs2_busy | do_bypass_rs2
    bypassed_uop.prs3_busy := uop.prs3_busy | do_bypass_rs3
  }

  if (!float && !vector) {
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
  float: Boolean, vector: Boolean=false)
(implicit p: Parameters) extends AbstractRenameStage(plWidth, numPhysRegs, numWbPorts, vector)(p)
{
  require(!(float & vector))
  val pregSz = log2Ceil(numPhysRegs)
  val rtype: UInt => Bool = if (float) isFloat else if (vector) isVector else isInt

  //-------------------------------------------------------------
  // Helper Functions

  def BypassAllocations(uop: MicroOp, older_uops: Seq[MicroOp], alloc_reqs: Seq[Bool]): MicroOp = {
    require(older_uops.length == alloc_reqs.length)
    val bypassed_uop = Wire(new MicroOp)
    val bypLogic = Module(new RenameBypass(older_uops.length, float, vector))
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
    float, vector))
  val freelist = Module(new RenameFreeList(
    plWidth,
    numPhysRegs,
    if (float | vector) 32 else 31))
  val busytable = Module(new RenameBusyTable(
    plWidth,
    numPhysRegs,
    numWbPorts,
    false,
    float, vector))



  val ren2_br_tags    = Wire(Vec(plWidth, Valid(UInt(brTagSz.W))))

  // Commit/Rollback
  val com_valids      = Wire(Vec(plWidth, Bool()))
  val rbk_valids      = Wire(Vec(plWidth, Bool()))

  for (w <- 0 until plWidth) {
    // NOTE: for reduction vd should be renamed only when vstart is 0
    ren2_alloc_reqs(w)    := ren2_uops(w).ldst_val && ren2_uops(w).rt(RD, rtype) && ren2_fire(w) &&
                             (~ren2_uops(w).v_is_split ||
                              (ren2_uops(w).v_re_alloc && !ren2_uops(w).rt(RD, isReduceV)))
    ren2_br_tags(w).valid := ren2_fire(w) && ren2_uops(w).allocate_brtag

    if (usingVector && vector) {
      val com_red     = io.com_uops(w).rt(RS1, isReduceV)
      val com_red_act = io.com_uops(w).rt(RD, isReduceV)
      val com_red_mov = com_red && !com_red_act
      com_valids(w) := io.com_uops(w).ldst_val && io.com_uops(w).rt(RD, rtype) && io.com_valids(w) &&
                       (!io.com_uops(w).v_is_split || io.com_uops(w).v_is_last && !com_red_mov)
      rbk_valids(w) := io.com_uops(w).ldst_val && io.com_uops(w).rt(RD, rtype) && io.rbk_valids(w) &&
                       (!io.com_uops(w).v_is_split || io.com_uops(w).v_is_first && !com_red_act)
    } else {
      com_valids(w) := io.com_uops(w).ldst_val && io.com_uops(w).rt(RD, rtype) && io.com_valids(w)
      rbk_valids(w) := io.com_uops(w).ldst_val && io.com_uops(w).rt(RD, rtype) && io.rbk_valids(w)
    }
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
  maptable.io.brupdate      := io.brupdate
  maptable.io.rollback    := io.rollback

  // Maptable outputs.
  for ((uop, w) <- ren1_uops.zipWithIndex) {
    val mappings = maptable.io.map_resps(w)

    uop.prs1       := mappings.prs1
    uop.prs2       := mappings.prs2
    uop.prs3       := mappings.prs3 // scalar integer will not use rs3
    uop.stale_pdst := mappings.stale_pdst
    if (vector) uop.prvm := mappings.prvm
  }



  //-------------------------------------------------------------
  // Free List

  // Freelist inputs.
  freelist.io.reqs := ren2_alloc_reqs
  freelist.io.dealloc_pregs zip com_valids zip rbk_valids map
    {case ((d,c),r) => d.valid := c || r}
  freelist.io.dealloc_pregs zip io.com_uops map
    {case (d,c) => d.bits := Mux(io.rollback, c.pdst, c.stale_pdst)}
  freelist.io.ren_br_tags := ren2_br_tags
  freelist.io.brupdate := io.brupdate
  freelist.io.debug.pipeline_empty := io.debug_rob_empty

  assert (ren2_alloc_reqs zip freelist.io.alloc_pregs map {case (r,p) => !r || p.bits =/= 0.U} reduce (_&&_),
           "[rename-stage] A uop is trying to allocate the zero physical register.")

  val prs1, prs2, prs3, stale_pdst, pdst, prvm = Reg(Vec(plWidth, UInt(maxPregSz.W)))
  // Freelist outputs.
  for ((uop, w) <- ren2_uops.zipWithIndex) {
    val preg = freelist.io.alloc_pregs(w).bits
    uop.pdst := Mux(uop.ldst =/= 0.U || float.B || vector.B, preg, 0.U)

    if (usingVector) {
      // record physical names of first split
      // for reduction, vs1, vd, vm should be consistent through vrgroup
      // NOTE: for reduction vd may overlap vs1, vs2, or vm
      // NOTE: for reduction we need read VS2 map through this, but skip vs1 and do not re-alloc vd
      when (uop.v_is_split && uop.v_re_alloc) {
        when(uop.rt(RS1, rtype) && !uop.rt(RD, isReduceV)) { prs1(w) := io.ren2_uops(w).prs1 }
        when(uop.rt(RS2, rtype) && !uop.rt(RD, isReduceV)) { prs2(w) := io.ren2_uops(w).prs2 }
        // TODO: check the essentiality of following when block, since decode may already handle this part
        when(uop.is_rvv && uop.uses_ldq) {
          prs3(w) := io.ren2_uops(w).stale_pdst
        } .elsewhen(uop.frs3_en && (float.B || vector.B) && !uop.rt(RD, isReduceV)) {
          prs3(w) := io.ren2_uops(w).prs3
        }
        when(uop.ldst_val && uop.rt(RD, rtype) && !uop.rt(RD, isReduceV)) { stale_pdst(w) := io.ren2_uops(w).stale_pdst }
        when(uop.ldst_val && uop.rt(RD, rtype) && !uop.rt(RD, isReduceV)) { pdst(w) := io.ren2_uops(w).pdst }
        when(~uop.v_unmasked                   && !uop.rt(RD, isReduceV)) { prvm(w) := io.ren2_uops(w).prvm }
        when(uop.rt(RD, isReduceV)) {
          when(uop.rt(RS1, rtype)) { uop.prs1 := prs1(w) }
          when(uop.rt(RS2, rtype) && uop.v_is_first) { uop.prs2 := prs2(w) }
          when(uop.frs3_en && vector.B) { uop.prs3 := prs3(w) }
          when(uop.rt(RD,  rtype)) { uop.stale_pdst := stale_pdst(w) }
          when(uop.rt(RD,  rtype)) { uop.pdst := pdst(w) }
          when(~uop.v_unmasked)    { uop.prvm := prvm(w) }
        }
        // vd may overlap vs2, latch vs2 on reduction mov
        when (uop.rt(RS1, isReduceV)) {
          when(uop.rt(RS2, rtype) && (!uop.rt(RD, isReduceV) || !uop.v_is_first)) { prs2(w) := uop.prs2 }
        }
      }

      // recover physical names for splits other than the first
      when (uop.v_is_split && !uop.v_re_alloc) {
        when(uop.rt(RS1, rtype)) { uop.prs1 := prs1(w) }
        when(uop.rt(RS2, rtype)) { uop.prs2 := prs2(w) }
        when(uop.frs3_en && (float.B || vector.B)) { uop.prs3 := prs3(w) }
        when(uop.ldst_val && uop.rt(RD, rtype)) { uop.stale_pdst := stale_pdst(w) }
        when(uop.ldst_val && uop.rt(RD, rtype)) { uop.pdst := pdst(w) }
        when(~uop.v_unmasked) { uop.prvm := prvm(w) }
      }
    }
  }

  //-------------------------------------------------------------
  // Busy Table

  busytable.io.ren_uops := ren2_uops  // expects pdst to be set up.
  busytable.io.rebusy_reqs := ren2_alloc_reqs
  busytable.io.wb_valids := io.wakeups.map(_.valid)
  busytable.io.wb_pdsts := io.wakeups.map(_.bits.uop.pdst)
  if (vector) {
    for ((bs, wk) <- busytable.io.wb_bits zip io.wakeups) {
      val vstart  = wk.bits.uop.vstart
      val ecnt    = wk.bits.uop.v_split_ecnt
      val (rsel, rmsk) = VRegSel(vstart, wk.bits.uop.vd_eew, ecnt, eLenb, eLenSelSz)
      bs := rmsk << Cat(rsel, 0.U(3.W))
      // only the final split clears busy status for recductions
      when (wk.bits.uop.rt(RS1, isReduceV)) {
        val is_act = wk.bits.uop.rt(RD, isReduceV)
        val is_fin = (vstart + 1.U === wk.bits.uop.vconfig.vl)
        bs := Fill(vLenb, (is_act && is_fin).asUInt)
      }
    }
  }

  assert (!(io.wakeups.map(x => x.valid && !x.bits.uop.rt(RD, rtype)).reduce(_||_)),
   "[rename] Wakeup has wrong rtype.")

  for ((uop, w) <- ren2_uops.zipWithIndex) {
    if (usingVector) {
      if (vector) {
        val vbusy = busytable.io.vbusy_resps(w)
        uop.prs1_busy := vbusy.prs1_busy & Fill(vLenb, (uop.rt(RS1, rtype)).asUInt) //& (rmsk << Cat(rsel, 0.U(3.W)))
        uop.prs2_busy := vbusy.prs2_busy & Fill(vLenb, (uop.rt(RS2, rtype)).asUInt) //& (rmsk << Cat(rsel, 0.U(3.W)))
        uop.prs3_busy := vbusy.prs3_busy & Fill(vLenb, uop.frs3_en.asUInt) //& (rmsk << Cat(rsel, 0.U(3.W)))
        uop.prvm_busy := vbusy.prvm_busy & Fill(vLenb, (!uop.is_rvv || !uop.v_unmasked).asUInt)
      } else {
        val busy = busytable.io.busy_resps(w)
        uop.prs1_busy := Cat(0.U, (uop.rt(RS1, rtype) && busy.prs1_busy).asUInt)
        uop.prs2_busy := Cat(0.U, (uop.rt(RS2, rtype) && busy.prs2_busy).asUInt)
        uop.prs3_busy := Cat(0.U, (uop.frs3_en && busy.prs3_busy).asUInt)

        val valid = ren2_valids(w)
        assert (!(valid && busy.prs1_busy(0) && (!vector.B && !float.B) && uop.lrs1 === 0.U), "[rename] x0 is busy??")
        assert (!(valid && busy.prs2_busy(0) && (!vector.B && !float.B) && uop.lrs2 === 0.U), "[rename] x0 is busy??")
      }
    } else {
      val busy = busytable.io.busy_resps(w)
      uop.prs1_busy := uop.rt(RS1, rtype) && busy.prs1_busy
      uop.prs2_busy := uop.rt(RS2, rtype) && busy.prs2_busy
      uop.prs3_busy := uop.frs3_en && busy.prs3_busy

      val valid = ren2_valids(w)
      assert (!(valid && busy.prs1_busy && (!vector.B && !float.B) && uop.lrs1 === 0.U), "[rename] x0 is busy??")
      assert (!(valid && busy.prs2_busy && (!vector.B && !float.B) && uop.lrs2 === 0.U), "[rename] x0 is busy??")
    }
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
