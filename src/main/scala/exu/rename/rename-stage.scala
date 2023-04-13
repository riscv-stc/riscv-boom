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
import freechips.rocketchip.rocket.{CSRs, CSR}

import boom.common._
import boom.util._
import boom.common.MicroOpcodes._
import FUConstants._

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
class DebugRenameStageIO(val numPhysRegs: Int, val vector: Boolean = false, val matrix: Boolean = false)(implicit p: Parameters) extends BoomBundle
{
  val freelist  = Bits(numPhysRegs.W)
  val isprlist  = Bits(numPhysRegs.W)
  val busytable = if (vector) Vec(numPhysRegs, UInt(vLenb.W))
                  else if(matrix) Vec(numPhysRegs, UInt((vLenb+1).W))
                  else UInt(numPhysRegs.W)
}

abstract class AbstractRenameStage(
  plWidth: Int,
  numPhysRegs: Int,
  numWbPorts: Int,
  val vector: Boolean = false,
  val matrix: Boolean = false)
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
    //val dis_fire_first = if (vector) Input(Vec(coreWidth, Bool())) else null

    // wakeup ports
    val wakeups   = if (vector) Flipped(Vec(numWbPorts, Valid(new ExeUnitResp(vLen)))) 
                    else        Flipped(Vec(numWbPorts, Valid(new ExeUnitResp(xLen))))
    val vl_wakeup = if (usingVector) Flipped(Valid(new VlWakeupResp())) else null
    
    val mtype_wakeup  = Flipped(Valid(new MtypeWakeupResp()))
    val tile_m_wakeup = Flipped(Valid(new MtileWakeupResp()))
    val tile_n_wakeup = Flipped(Valid(new MtileWakeupResp()))
    val tile_k_wakeup = Flipped(Valid(new MtileWakeupResp()))
    val moutsh_wakeup = Flipped(Valid(new OutputShapeWakeupResp()))
    val minsh_wakeup  = Flipped(Valid(new InputShapeWakeupResp()))
    val msk_wakeup    = Flipped(Valid(new KernelPositionWakeupResp()))

    val matrix_iss_valid = if (matrix) Input(Vec(matWidth,Bool())) else null
    val matrix_iss_uop = if (matrix) Input(Vec(matWidth,new MicroOp())) else null
    val mem_iss_valid = if (matrix) Input(Vec(memWidth,Bool())) else null
    val mem_iss_uop = if (matrix) Input(Vec(memWidth,new MicroOp())) else null
    val wake_issue_prs = Output(Vec(2,Vec(memWidth + matWidth,UInt((vLenb+1).W))))
    val wake_issue_data = Output(Vec(2,Vec(memWidth + matWidth,UInt((vLenb+1).W))))
    val wake_issue_valid = Output(Vec(2,Vec(memWidth + matWidth,Bool())))
    val wake_issue_rs_type = Output(Vec(2,Vec(memWidth + matWidth,UInt(RT_X.getWidth.W))))

    // commit stage
    val com_valids = Input(Vec(plWidth, Bool()))
    val com_uops   = Input(Vec(plWidth, new MicroOp()))
    val rbk_valids = Input(Vec(plWidth, Bool()))
    val vl_xcpt    = if (vector) Input(Vec(plWidth, Bool())) else null
    val rollback   = Input(Bool())
    val vbusy_status    = if (vector) Output(UInt(numPhysRegs.W)) else Output(UInt(0.W))
    val tr_busy_status  = if (matrix) Output(UInt(numPhysRegs.W)) else null
    val acc_busy_status = if (matrix) Output(UInt(numPhysRegs.W)) else null

    val debug_rob_empty = Input(Bool())
    val debug = Output(new DebugRenameStageIO(numPhysRegs, vector, matrix))
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

    // vl_wakeup, ren2_uops may stall
    if (usingVector) {
      when(io.vl_wakeup.valid && r_valid && !r_uop.vl_ready && !ren2_ready &&
          (io.vl_wakeup.bits.vconfig_tag + 1.U) === r_uop.vconfig_tag) {
        r_uop.vl_ready   := true.B
        r_uop.vconfig.vl := Mux(r_uop.uopc.isOneOf(uopVSMA, uopVLM), (io.vl_wakeup.bits.vl + 7.U) >> 3.U,
                                                                      io.vl_wakeup.bits.vl)
      }
    }

    if (usingMatrix) {
      when (io.moutsh_wakeup.valid && r_valid && !r_uop.moutsh_ready && !ren2_ready &&
        (io.moutsh_wakeup.bits.moutsh_tag + 1.U) === r_uop.moutsh_tag) {
        r_uop.moutsh_ready := true.B
        r_uop.moutsh := io.moutsh_wakeup.bits.moutsh
        r_uop.mstdi := io.moutsh_wakeup.bits.mstdi
      }

      when(io.minsh_wakeup.valid && r_valid && !r_uop.minsh_ready && !ren2_ready &&
        (io.minsh_wakeup.bits.minsh_tag + 1.U) === r_uop.minsh_tag) {
        r_uop.minsh_ready := true.B
        r_uop.minsh := io.minsh_wakeup.bits.minsh
        r_uop.mpad := io.minsh_wakeup.bits.mpad
      }

      when(io.msk_wakeup.valid && r_valid && !r_uop.msk_ready && !ren2_ready &&
        (io.msk_wakeup.bits.msk_tag + 1.U) === r_uop.msk_tag) {
        r_uop.msk_ready := true.B
        r_uop.minsk := io.msk_wakeup.bits.minsk
        r_uop.moutsk := io.msk_wakeup.bits.moutsk
      }
    }

   // if (matrix) {
        when(io.mtype_wakeup.valid && r_valid && !r_uop.mtype_ready && !ren2_ready &&
          (io.mtype_wakeup.bits.mconfig_tag + 1.U) === r_uop.mconfig_tag) {
            r_uop.mtype_ready := true.B
            r_uop.mconfig := io.mtype_wakeup.bits.mconfig
        }
        when(io.tile_m_wakeup.valid && r_valid && !r_uop.tile_m_ready && !ren2_ready &&
            (io.tile_m_wakeup.bits.tile_tag + 1.U) === r_uop.tile_m_tag) {
              r_uop.tile_m_ready := true.B
              r_uop.tile_m := io.tile_m_wakeup.bits.tile_len
        }
        when(io.tile_n_wakeup.valid && r_valid && !r_uop.tile_n_ready && !ren2_ready &&
            (io.tile_n_wakeup.bits.tile_tag + 1.U) === r_uop.tile_n_tag) {
              r_uop.tile_n_ready := true.B
              r_uop.tile_n := io.tile_n_wakeup.bits.tile_len
        }
        when(io.tile_k_wakeup.valid && r_valid && !r_uop.tile_k_ready && !ren2_ready &&
            (io.tile_k_wakeup.bits.tile_tag + 1.U) === r_uop.tile_k_tag) {
              r_uop.tile_k_ready := true.B
              r_uop.tile_k := io.tile_k_wakeup.bits.tile_len
        }
        val  m_ok = (io.tile_m_wakeup.valid && r_valid && !r_uop.tile_m_ready && !ren2_ready &&
            (io.tile_m_wakeup.bits.tile_tag + 1.U) === r_uop.tile_m_tag)
        val  n_ok = (io.tile_n_wakeup.valid && r_valid && !r_uop.tile_n_ready && !ren2_ready &&
            (io.tile_n_wakeup.bits.tile_tag + 1.U) === r_uop.tile_n_tag)
        val k_ok = (io.tile_k_wakeup.valid && r_valid && !r_uop.tile_k_ready && !ren2_ready &&
            (io.tile_k_wakeup.bits.tile_tag + 1.U) === r_uop.tile_k_tag)
        when(k_ok || n_ok || m_ok) {
//          val is_mls = r_uop.uopc.isOneOf(uopMLE,uopMSE)
          val is_mls = r_uop.is_rvm && (r_uop.uses_ldq || r_uop.uses_stq)
          val is_mopa = r_uop.uopc.isOneOf(uopMMA, uopMWMA, uopMQMA)
          val is_mmv =  r_uop.uopc.isOneOf(uopMMV_T,uopMMV_V,uopMWMV_T,uopMWMV_V,uopMQMV_T,uopMQMV_V)
          val transposed = r_uop.transposed
          val mslice_dim = r_uop.mslice_dim
          val is_unfold = r_uop.ctrl.is_unfold

          val slice_cnt_tilem = (mslice_dim === 1.U && !transposed) || (mslice_dim === 0.U && !transposed)
          val slice_cnt_tilen = (mslice_dim === 2.U &&  transposed) || (mslice_dim === 0.U &&  transposed)
          val slice_cnt_tilek = (mslice_dim === 1.U &&  transposed) || (mslice_dim === 2.U && !transposed)
          val slice_len_tilem = (mslice_dim === 1.U &&  transposed) || (mslice_dim === 0.U &&  transposed)
          val slice_len_tilen = (mslice_dim === 2.U && !transposed) || (mslice_dim === 0.U && !transposed)
          val slice_len_tilek = (mslice_dim === 1.U && !transposed) || (mslice_dim === 2.U &&  transposed)

//          val slice_cnt_tilem = Mux(is_unfold, (mslice_dim === 0.U) || (mslice_dim === 2.U),
//            (mslice_dim === 1.U && !transposed) || (mslice_dim === 0.U && !transposed)) // A  || C
//          val slice_cnt_tilen = Mux(is_unfold, false.B,
//            (mslice_dim === 2.U && transposed) || (mslice_dim === 0.U && transposed)) // BT || CT
//          val slice_cnt_tilek = Mux(is_unfold, (mslice_dim === 1.U),
//            (mslice_dim === 1.U && transposed) || (mslice_dim === 2.U && !transposed)) // AT || B
//          val slice_len_tilem = Mux(is_unfold, false.B,
//            (mslice_dim === 1.U && transposed) || (mslice_dim === 0.U && transposed)) // AT || CT
//          val slice_len_tilen = Mux(is_unfold, (mslice_dim === 1.U) || (mslice_dim === 2.U),
//            (mslice_dim === 2.U && !transposed) || (mslice_dim === 0.U && !transposed)) // B  || C
//          val slice_len_tilek = Mux(is_unfold, (mslice_dim === 0.U),
//            (mslice_dim === 1.U && !transposed) || (mslice_dim === 2.U && transposed)) // A  || BT

          val sel_m = Mux(m_ok, io.tile_m_wakeup.bits.tile_len, r_uop.tile_m)
          val sel_n = Mux(n_ok, io.tile_n_wakeup.bits.tile_len, r_uop.tile_n)
          val sel_k = Mux(k_ok, io.tile_k_wakeup.bits.tile_len, r_uop.tile_k)
          val sel_slice_cnt = Mux(slice_cnt_tilem, sel_m,
                              Mux(slice_cnt_tilen, sel_n, sel_k))
          val sel_slice_len = Mux(slice_len_tilem, sel_m,
                              Mux(slice_len_tilen, sel_n, sel_k))
          r_uop.m_slice_cnt   := Mux(is_mls,  sel_slice_cnt,
                                    Mux(is_mopa, sel_k, sel_m))
          r_uop.m_slice_len   := Mux(is_mls, sel_slice_len,
                                    Mux(is_mmv && mslice_dim === 2.U, sel_m,
                                    Mux(is_mmv && mslice_dim === 3.U, sel_n, sel_k)))
      }
//    }

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

  def grpCount(emul: UInt, nf: UInt): UInt = {
    val ret = nf << Mux(emul(2), 0.U, emul(1,0))
    ret
  }

  def matchGroup(rs: UInt, grpRS: UInt, grpEMUL: UInt = 0.U, grpNF: UInt = 1.U): Vec[Bool] = {
    val gCount = grpCount(grpEMUL, grpNF)
    val ret = Wire(Vec(8, Bool()))
    (0 until 8).map(i => ret(i) := (rs === grpRS + i.U) && i.U < gCount)
    ret
  }

  for (i <- 0 until 8) {
    val bypass_hits_rs1 = (older_uops zip alloc_reqs) map { case (o,a) =>
      (Fill(8, (a && i.U < grpCount(uop.vs1_emul, 1.U) && uop.rt(RS1, isVector) &&
               o.rt(RD, isVector) && o.ldst_val).asUInt) &
       matchGroup(uop.lrs1 + i.U, o.ldst, o.vd_emul, o.v_seg_nf).asUInt).asBools
    }
    val bypass_hits_rs2 = (older_uops zip alloc_reqs) map { case (o,a) =>
      (Fill(8, (a && i.U < grpCount(uop.vs2_emul, 1.U) && uop.rt(RS2, isVector) &&
               o.rt(RD, isVector) && o.ldst_val).asUInt) &
       matchGroup(uop.lrs2 + i.U, o.ldst, o.vd_emul, o.v_seg_nf).asUInt).asBools
    }
    // FIXME: segment load/store
    val bypass_hits_dst = (older_uops zip alloc_reqs) map { case (o,a) =>
      (Fill(8, (a && i.U < grpCount(uop.vd_emul, 1.U) && uop.rt(RD, isVector) &&
               o.rt(RD, isVector) && o.ldst_val).asUInt) &
       matchGroup(uop.ldst + i.U, o.ldst, o.vd_emul, o.v_seg_nf).asUInt).asBools
    }

    bypass_hits_rs1.map(x => assert(PopCount(x) <= 1.U))
    bypass_hits_rs2.map(x => assert(PopCount(x) <= 1.U))
    bypass_hits_dst.map(x => assert(PopCount(x) <= 1.U))
    val bypass_sel_rs1 = bypass_hits_rs1
    val bypass_sel_rs2 = bypass_hits_rs2
    val bypass_sel_dst = bypass_hits_dst

    val do_bypass_rs1 = PriorityEncoderOH(bypass_hits_rs1.map(x => x.orR).reverse).reverse
    val do_bypass_rs2 = PriorityEncoderOH(bypass_hits_rs2.map(x => x.orR).reverse).reverse
    val do_bypass_dst = PriorityEncoderOH(bypass_hits_dst.map(x => x.orR).reverse).reverse

    val bypass_pvd = older_uops.map(_.pvd)
    when (do_bypass_rs1.reduce(_ || _)) {
      bypassed_uop.pvs1(i).bits      := Mux1H(do_bypass_rs1, (bypass_sel_rs1 zip bypass_pvd).map{case(s,p) => Mux1H(s, p.map(_.bits))})
    }
    when (do_bypass_rs2.reduce(_ || _)) {
      bypassed_uop.pvs2(i).bits      := Mux1H(do_bypass_rs2, (bypass_sel_rs2 zip bypass_pvd).map{case(s,p) => Mux1H(s, p.map(_.bits))})
    }
    when (do_bypass_dst.reduce(_ || _)) {
      bypassed_uop.stale_pvd(i).bits := Mux1H(do_bypass_dst, (bypass_sel_dst zip bypass_pvd).map{case(s,p) => Mux1H(s, p.map(_.bits))})
    }
  }

  val bypass_hits_vs0 = (older_uops zip alloc_reqs) map { case (o,a) => a && o.ldst === 0.U }
  val bypass_sel_vs0 = PriorityEncoderOH(bypass_hits_vs0.reverse).reverse
  val do_bypass_vs0 = bypass_hits_vs0.reduce(_||_)
  val bypass_pvd = older_uops.map(_.pvd)
  when (do_bypass_vs0)    { bypassed_uop.pvm := Mux1H(bypass_sel_vs0, bypass_pvd.map(b => b(0).bits)) }

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

    val com_uop = io.com_uops(w)
    com_valids(w) := io.com_valids(w) && com_uop.ldst_val && com_uop.rt(RD, rtype) 
    rbk_valids(w) := io.rbk_valids(w) && com_uop.ldst_val && com_uop.rt(RD, rtype)
    ren2_br_tags(w).bits := ren2_uops(w).br_tag
  }

  //-------------------------------------------------------------
  // Rename Table

  // Maptable inputs.
  val map_reqs    = Wire(Vec(plWidth, new VecMapReq(lregSz)))
  val remap_reqs  = Wire(Vec(plWidth, new VecRemapReq(lregSz, pregSz)))
  val vstart_reqs = Wire(Vec(plWidth, new VecVstartReq()))

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

    // reset vstart source request
    val vstartCSRW = ren2.ctrl.csr_cmd.isOneOf(CSR.S, CSR.C, CSR.W) && ren2.inst(31,20) === CSRs.vstart.asUInt
    vstart_reqs(w).valid := Mux(io.rollback, true.B, ren2_fire(w) & (ren2.is_rvv | vstartCSRW))
    vstart_reqs(w).src   := Mux(io.rollback || vstartCSRW, VSTART_CSR, VSTART_ZERO)
    // vstart control
    ren2.vstartSrc  := maptable.io.vstart_resps(w)
  }
  ren2_alloc_reqs zip rbk_valids.reverse zip io.vl_xcpt zip remap_reqs map {
    case (((a,r),x),rr) => rr.valid := a || (r && !x)}

  // Hook up inputs.
  maptable.io.map_reqs    := map_reqs
  maptable.io.remap_reqs  := remap_reqs
  maptable.io.vstart_reqs := vstart_reqs
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
    when(!uop.rt(RS1, rtype)) {uop.pvs1.map(_.valid := false.B)}
    when(!uop.rt(RS2, rtype)) {uop.pvs2.map(_.valid := false.B)}
    when(!uop.rt(RD , rtype)) {uop.pvd.map( _.valid := false.B)}
    when(!uop.rt(RD , rtype)) {uop.stale_pvd.map( _.valid := false.B)}
  }

  //-------------------------------------------------------------
  // Free List

  // Freelist inputs.
  for (w <- 0 until plWidth) {
    freelist.io.reqs(w).valid := ren2_alloc_reqs(w)
    freelist.io.reqs(w).bits  := ren2_uops(w)
    freelist.io.stall(w) := (ren2_uops(w).rt(RD, rtype)) && !freelist.io.can_allocate(w)
  }
  (freelist.io.dealloc_pregs zip io.com_uops zip com_valids zip rbk_valids zip io.vl_xcpt).map { case((((da, uop), c), r), x) => {
    da.zipWithIndex.map { case (d, i) => {
      d.valid := (c || r) && uop.stale_pvd(i).valid
      d.bits  := Mux(io.rollback && !x, uop.pvd(i).bits, uop.stale_pvd(i).bits)
    }}
  }}
  freelist.io.ren_br_tags := ren2_br_tags
  freelist.io.brupdate := io.brupdate
  freelist.io.debug.pipeline_empty := io.debug_rob_empty

  // Freelist outputs.
  for ((uop, w) <- ren2_uops.zipWithIndex) {
    val alloc_req, alloc_assign = Wire(Valid(new MicroOp))
    alloc_req.valid := ren2_alloc_reqs(w)
    alloc_req.bits  := ren2_uops(w)
    alloc_assign.valid := ren2_uops(w).ldst_val && ren2_uops(w).rt(RD, rtype) && ren2_valids(w)
    alloc_assign.bits  := ren2_uops(w)
    val alloc_grp = lvdGroup(alloc_assign)
    for (i <- 0 until 8) {
      uop.pvd(i).valid := freelist.io.alloc_pregs(w)(i).valid && alloc_grp(i).valid
      uop.pvd(i).bits  := freelist.io.alloc_pregs(w)(i).bits
    }
  }

  //-------------------------------------------------------------
  // Busy Table

  busytable.io.ren_uops := ren2_uops  // expects pdst to be set up.
  busytable.io.wb_valids := VecInit(io.wakeups.map(wk => wk.valid))
  busytable.io.wb_pdsts := VecInit(io.wakeups.map(_.bits.uop.pdst))
  io.vbusy_status := busytable.io.vbusy_status
  for (w <- 0 until plWidth) {
    busytable.io.rebusy_reqs(w) := ren2_uops(w).ldst_val && ren2_uops(w).rt(RD, rtype) //&& io.dis_fire_first(w)
  }
  for ((bs, wk) <- busytable.io.wb_bits zip io.wakeups) { 
    bs := Mux(wk.bits.uop.is_rvm, Fill(vLenb, 1.U(1.W)), wk.bits.vmask)
  }
  
  // clear busy bits when:
  // 1. commit a vlexff instuction that raises an exception at element index > 0
  // 2. a vload instruction raises an exception, record exception element index in vstart csr and commit pvd
  val vlClrValids = (com_valids zip rbk_valids zip io.vl_xcpt zip io.com_uops).map { case(((c, r), x), uop) => 
    (r & x) | (c & uop.uopc.isOneOf(uopVLFF) & uop.exception)
  }
  val vlClear  = vlClrValids.reduce(_||_)
  val vlClrUop = Mux1H(vlClrValids, io.com_uops)
  for(i <- 0 until 8) {
    busytable.io.clr_valids(i) := vlClear && vlClrUop.pvd(i).valid
    busytable.io.clr_pdsts(i)  := vlClrUop.pvd(i).bits
  }

  assert (!(io.wakeups.map(x => x.valid && !x.bits.uop.rt(RD, rtype)).reduce(_||_)),
   "[rename] Wakeup has wrong rtype.")

  //-------------------------------------------------------------
  // Outputs

  for (w <- 0 until plWidth) {
    //val can_allocate = freelist.io.alloc_pregs(w).map(_.valid).reduce(_ && _)
    val can_allocate = freelist.io.can_allocate(w)

    // Push back against Decode stage if Rename1 can't proceed.
    io.ren_stalls(w) := (ren2_uops(w).rt(RD, rtype)) && !can_allocate

    val bypassed_uop = Wire(new MicroOp)
    if (w > 0) bypassed_uop := BypassAllocations(ren2_uops(w), ren2_uops.slice(0, w), ren2_alloc_reqs.slice(0, w))
    else bypassed_uop := ren2_uops(w)

      io.ren2_uops(w) := GetNewUopAndBrMask(bypassed_uop, io.brupdate)
    }
  //-------------------------------------------------------------
  // Debug signals

  io.debug.freelist  := freelist.io.debug.freelist
  io.debug.isprlist  := freelist.io.debug.isprlist
  io.debug.busytable := busytable.io.debug.busytable
}


class MatRenameBypass(
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

  val bypass_hits_rs1 = (older_uops zip alloc_reqs) map { case (o,a) => a && o.ldst === uop.lrs1 && o.dst_rtype === uop.lrs1_rtype}
  val bypass_hits_rs2 = (older_uops zip alloc_reqs) map { case (o,a) => a && o.ldst === uop.lrs2 && o.dst_rtype === uop.lrs2_rtype}
  val bypass_hits_dst = (older_uops zip alloc_reqs) map { case (o,a) => a && o.ldst === uop.ldst && o.dst_rtype === uop.dst_rtype}

  val bypass_sel_rs1 = PriorityEncoderOH(bypass_hits_rs1.reverse).reverse
  val bypass_sel_rs2 = PriorityEncoderOH(bypass_hits_rs2.reverse).reverse
  val bypass_sel_dst = PriorityEncoderOH(bypass_hits_dst.reverse).reverse

  val do_bypass_rs1 = bypass_hits_rs1.reduce(_||_)
  val do_bypass_rs2 = bypass_hits_rs2.reduce(_||_)
  val do_bypass_dst = bypass_hits_dst.reduce(_||_)

  val bypass_pdsts = older_uops.map(_.pdst)
  when (do_bypass_rs1) { bypassed_uop.prs1       := Mux1H(bypass_sel_rs1, bypass_pdsts) }
  when (do_bypass_rs2) { bypassed_uop.prs2       := Mux1H(bypass_sel_rs2, bypass_pdsts) }
  when (do_bypass_dst) { bypassed_uop.prs3       := Mux1H(bypass_sel_dst, bypass_pdsts) }
  when (do_bypass_dst) { bypassed_uop.stale_pdst := Mux1H(bypass_sel_dst, bypass_pdsts) }

  bypassed_uop.pts1_busy := uop.pts1_busy | Fill(vLenb, do_bypass_rs1)
  bypassed_uop.pts2_busy := uop.pts2_busy | Fill(vLenb, do_bypass_rs2)
  bypassed_uop.pts3_busy := uop.pts3_busy | Fill(vLenb, do_bypass_dst)

  io.o_uop := bypassed_uop
}

/**
 * Rename stage that connets the map table, free list, and busy table.
 * Can be used in both the FP pipeline and the normal execute pipeline.
 *
 * @param plWidth pipeline width
 * @param numWbPorts number of Mat writeback ports
 */
class MatRenameStage(
                   plWidth: Int,
                   numTrPhysRegs: Int,
                   numAccPhysRegs: Int,
                   numWbPorts: Int)(implicit p: Parameters)
  extends AbstractRenameStage(
    plWidth,
    numTrPhysRegs.max(numAccPhysRegs),
    numWbPorts, false, true)(p)
{
  val trpregSz = log2Ceil(numTrPhysRegs)
  val accpregSz = log2Ceil(numAccPhysRegs)
  val rtype: UInt => Bool = isMatrix

  //-------------------------------------------------------------
  // Helper Functions

  def BypassAllocations(uop: MicroOp, older_uops: Seq[MicroOp], alloc_reqs: Seq[Bool]): MicroOp = {
    require(older_uops.length == alloc_reqs.length)
    val bypassed_uop = Wire(new MicroOp)
    val bypLogic = Module(new MatRenameBypass(older_uops.length))
    bypLogic.io.i_uop := uop
    bypLogic.io.older_uops := older_uops
    bypLogic.io.alloc_reqs := alloc_reqs
    bypassed_uop := bypLogic.io.o_uop
    bypassed_uop
  }

  //-------------------------------------------------------------
  // Rename Structures

  val trmaptable = Module(new MatRenameMapTable(
    plWidth,
    8,
    numTrPhysRegs,
    false))
  val trfreelist = Module(new MatRenameFreeList(
    plWidth,
    numTrPhysRegs,
    8))
  val trbusytable = Module(new MatRenameBusyTable(
    plWidth,
    numTrPhysRegs,
    false,
    numWbPorts))

  val accmaptable = Module(new MatRenameMapTable(
    plWidth,
    2,
    numAccPhysRegs,
    false))
  val accfreelist = Module(new MatRenameFreeList(
    plWidth,
    numAccPhysRegs,
    2))
  val accbusytable = Module(new MatRenameBusyTable(
    plWidth,
    numAccPhysRegs,
    false,
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
  val map_reqs   = Wire(Vec(plWidth, new MapReq(lregSz)))
  val trremap_reqs = Wire(Vec(plWidth, new RemapReq(lregSz, trpregSz)))
  val accremap_reqs = Wire(Vec(plWidth, new RemapReq(lregSz, accpregSz)))

  // Generate maptable requests.
  for ((((ren1,ren2),com),w) <- ren1_uops zip ren2_uops zip io.com_uops.reverse zipWithIndex) {
    map_reqs(w).lrs1 := ren1.lrs1
    map_reqs(w).lrs2 := ren1.lrs2
    map_reqs(w).lrs3 := ren1.lrs3
    map_reqs(w).ldst := ren1.ldst

    trremap_reqs(w).ldst  := Mux(io.rollback, com.ldst      , ren2.ldst)
    trremap_reqs(w).pdst  := Mux(io.rollback, com.stale_pdst, ren2.pdst)
    accremap_reqs(w).ldst := Mux(io.rollback, com.ldst      , ren2.ldst)
    accremap_reqs(w).pdst := Mux(io.rollback, com.stale_pdst, ren2.pdst)
  }

  ren2_alloc_reqs zip ren2_uops zip rbk_valids.reverse zip io.com_uops.reverse zip trremap_reqs map {
    case ((((a, a_uop), r), r_uop), tr) => tr.valid := (a && a_uop.dst_rtype === RT_TR) || (r && r_uop.dst_rtype === RT_TR)
  }

  ren2_alloc_reqs zip ren2_uops zip rbk_valids.reverse zip io.com_uops.reverse zip accremap_reqs map {
    case ((((a, a_uop), r), r_uop), acc) => acc.valid := (a && a_uop.dst_rtype === RT_ACC) || (r && r_uop.dst_rtype === RT_ACC)
  }

  // Hook up inputs.
  trmaptable.io.map_reqs    := map_reqs
  trmaptable.io.remap_reqs  := trremap_reqs
  trmaptable.io.ren_br_tags := ren2_br_tags
  trmaptable.io.brupdate    := io.brupdate
  trmaptable.io.rollback    := io.rollback

  accmaptable.io.map_reqs    := map_reqs
  accmaptable.io.remap_reqs  := accremap_reqs
  accmaptable.io.ren_br_tags := ren2_br_tags
  accmaptable.io.brupdate    := io.brupdate
  accmaptable.io.rollback    := io.rollback

  // Maptable outputs.
  for ((uop, w) <- ren1_uops.zipWithIndex) {
    val trmappings  = trmaptable.io.map_resps(w)
    val accmappings = accmaptable.io.map_resps(w)

    uop.prs1       := Mux(uop.lrs1_rtype === RT_TR, trmappings.prs1, accmappings.prs1)
    uop.prs2       := Mux(uop.lrs2_rtype === RT_TR, trmappings.prs2, accmappings.prs2)
    //uop.prs3       := Mux(uop.dst_rtype  === RT_TR, trmappings.prs3, accmappings.prs3)
    uop.prs3       := Mux(uop.dst_rtype  === RT_TR, trmappings.stale_pdst, accmappings.stale_pdst)
    uop.stale_pdst := Mux(uop.dst_rtype  === RT_TR, trmappings.stale_pdst, accmappings.stale_pdst)
  }

  //-------------------------------------------------------------
  // Free List

  // Freelist inputs.
  for (w <- 0 until plWidth) {
    trfreelist.io.reqs(w).valid := ren2_alloc_reqs(w) && ren2_uops(w).dst_rtype === RT_TR
    trfreelist.io.reqs(w).bits  := ren2_uops(w)
    accfreelist.io.reqs(w).valid := ren2_alloc_reqs(w)&& ren2_uops(w).dst_rtype === RT_ACC
    accfreelist.io.reqs(w).bits  := ren2_uops(w)
  }
  trfreelist.io.dealloc_pregs zip com_valids zip rbk_valids zip io.com_uops map
    {case (((d,c),r),u) => d.valid := (c || r) && u.dst_rtype === RT_TR}
  trfreelist.io.dealloc_pregs zip io.com_uops map
    {case (d,c) => d.bits := Mux(io.rollback, c.pdst, c.stale_pdst)}
  trfreelist.io.ren_br_tags := ren2_br_tags
  trfreelist.io.brupdate := io.brupdate
  trfreelist.io.debug.pipeline_empty := io.debug_rob_empty

  accfreelist.io.dealloc_pregs zip com_valids zip rbk_valids zip io.com_uops map
    {case (((d,c),r),u) => d.valid := (c || r) && u.dst_rtype === RT_ACC}
  accfreelist.io.dealloc_pregs zip io.com_uops map
    {case (d,c) => d.bits := Mux(io.rollback, c.pdst, c.stale_pdst)}
  accfreelist.io.ren_br_tags := ren2_br_tags
  accfreelist.io.brupdate := io.brupdate
  accfreelist.io.debug.pipeline_empty := io.debug_rob_empty

  //val prs1, prs2, prs3, stale_pdst, pdst, prvm = Reg(Vec(plWidth, UInt(maxPregSz.W)))
  // Freelist outputs.
  for ((uop, w) <- ren2_uops.zipWithIndex) {
    val trdst  = Mux(uop.dst_rtype === RT_TR  && trfreelist.io.alloc_pregs(w).valid,  trfreelist.io.alloc_pregs(w).bits,  0.U)
    val accdst = Mux(uop.dst_rtype === RT_ACC && accfreelist.io.alloc_pregs(w).valid, accfreelist.io.alloc_pregs(w).bits, 0.U)
    uop.pdst := trdst | accdst
  }

  //-------------------------------------------------------------
  // Busy Table
  for (w <- 0 until plWidth) {
    trbusytable.io.rebusy_reqs(w)  := ren2_alloc_reqs(w) && ren2_uops(w).dst_rtype === RT_TR
    accbusytable.io.rebusy_reqs(w) := ren2_alloc_reqs(w) && ren2_uops(w).dst_rtype === RT_ACC
  }
  for (i <- 0 until matWidth) {
    trbusytable.io.matrix_iss_valid(i)  := io.matrix_iss_valid(i)  && (io.matrix_iss_uop(i).dst_rtype ===  RT_TR)
    trbusytable.io.matrix_iss_uop(i)  := io.matrix_iss_uop(i)
    accbusytable.io.matrix_iss_valid(i)  := io.matrix_iss_valid(i)  && (io.matrix_iss_uop(i).dst_rtype ===  RT_ACC)
  accbusytable.io.matrix_iss_uop(i)  := io.matrix_iss_uop(i)
  }
  for (j <- 0 until memWidth) {
    trbusytable.io.mem_iss_valid(j)  := io.mem_iss_valid(j) && (io.mem_iss_uop(j).dst_rtype ===  RT_TR)
    trbusytable.io.mem_iss_uop(j)  := io.mem_iss_uop(j)
    accbusytable.io.mem_iss_valid(j) := io.mem_iss_valid(j)  && (io.mem_iss_uop(j).dst_rtype ===  RT_ACC)
    accbusytable.io.mem_iss_uop(j)  := io.mem_iss_uop(j)
  }

  trbusytable.io.ren_uops  := ren2_uops  // expects pdst to be set up.
  trbusytable.io.wb_valids := io.wakeups.map(x => x.valid && x.bits.uop.dst_rtype === RT_TR && Mux(x.bits.uop.uses_ldq, x.bits.uop.m_slice_done, true.B))
  trbusytable.io.wb_pdsts  := io.wakeups.map(_.bits.uop.pdst)
  trbusytable.io.wb_bits   := io.wakeups.map(w => UIntToOH(w.bits.uop.m_sidx))
  io.wake_issue_valid(0) := trbusytable.io.wake_issue_valid
  io.wake_issue_data(0) := trbusytable.io.wake_issue_data
  io.wake_issue_prs(0) := trbusytable.io.wake_issue_prs
  io.wake_issue_rs_type(0) := trbusytable.io.wake_issue_rs_type

  accbusytable.io.ren_uops  := ren2_uops  // expects pdst to be set up.
  accbusytable.io.wb_valids := io.wakeups.map(x => x.valid && x.bits.uop.dst_rtype === RT_ACC && Mux(x.bits.uop.uses_ldq, x.bits.uop.m_slice_done, true.B))
  accbusytable.io.wb_pdsts  := io.wakeups.map(_.bits.uop.pdst)
  accbusytable.io.wb_bits   := io.wakeups.map(w => Mux(w.bits.uop.m_is_split, UIntToOH(w.bits.uop.m_sidx), Fill(vLenb, 1.U(1.W))))
  io.wake_issue_valid(1) := accbusytable.io.wake_issue_valid
  io.wake_issue_data(1) := accbusytable.io.wake_issue_data
  io.wake_issue_prs(1) := accbusytable.io.wake_issue_prs
  io.wake_issue_rs_type(1) := accbusytable.io.wake_issue_rs_type

  assert (!(io.wakeups.map(x => x.valid && !x.bits.uop.rt(RD, rtype)).reduce(_||_)),
    "[rename] Wakeup has wrong rtype.")

  //Trtile/ACC Register busy state
  for ((uop, w) <- ren2_uops.zipWithIndex) {
    val trbusy = trbusytable.io.busy_resps(w)
    val accbusy = accbusytable.io.busy_resps(w)

    when(uop.lrs1_rtype === RT_TR) {
      uop.pts1_busy    := trbusy.prs1_busy
      uop.pts1DirCross := Mux(uop.uopc.isOneOf(uopMMA, uopMWMA, uopMQMA), trbusy.prs1_busy(vLenb), uop.isHSlice =/= trbusy.prs1_busy(vLenb))
    } .elsewhen(uop.lrs1_rtype === RT_ACC) {
      uop.pts1_busy    := accbusy.prs1_busy
      uop.pts1DirCross := uop.isHSlice =/= accbusy.prs1_busy(vLenb)
    }

    when(uop.lrs2_rtype === RT_TR) {
      uop.pts2_busy    := trbusy.prs2_busy
      uop.pts2DirCross := Mux(uop.uopc.isOneOf(uopMMA, uopMWMA, uopMQMA), !trbusy.prs2_busy(vLenb), uop.isHSlice =/= trbusy.prs2_busy(vLenb))
    } .elsewhen(uop.lrs2_rtype === RT_ACC) {
      uop.pts2_busy    := accbusy.prs2_busy
      uop.pts2DirCross := uop.isHSlice =/= accbusy.prs2_busy(vLenb)
    }
    
    when(uop.dst_rtype === RT_ACC) {
      uop.pts3_busy := accbusy.prs3_busy
    }

    val valid = ren2_valids(w)
  }
  io.tr_busy_status  := trbusytable.io.tbusy_status
  io.acc_busy_status := accbusytable.io.tbusy_status

  //-------------------------------------------------------------
  // Outputs

  for (w <- 0 until plWidth) {

    // Push back against Decode stage if Rename1 can't proceed.
    io.ren_stalls(w) := !trfreelist.io.alloc_pregs(w).valid &&  ren2_uops(w).dst_rtype === RT_TR ||
                        !accfreelist.io.alloc_pregs(w).valid && ren2_uops(w).dst_rtype === RT_ACC

    val bypassed_uop = Wire(new MicroOp)
    if (w > 0) bypassed_uop := BypassAllocations(ren2_uops(w), ren2_uops.slice(0,w), ren2_alloc_reqs.slice(0,w))
    else       bypassed_uop := ren2_uops(w)

    io.ren2_uops(w) := GetNewUopAndBrMask(bypassed_uop, io.brupdate)
  }

  //-------------------------------------------------------------
  // Debug signals

  io.debug.freelist  := trfreelist.io.debug.freelist
  io.debug.isprlist  := trfreelist.io.debug.isprlist
  io.debug.busytable := trbusytable.io.debug.busytable
}


import freechips.rocketchip.rocket._
import freechips.rocketchip.config._
import freechips.rocketchip.unittest._
import freechips.rocketchip.tile._
import freechips.rocketchip.subsystem._
import freechips.rocketchip.tilelink.LFSR64
import boom.ifu._
import boom.lsu._

class VecRenameUT(timeout: Int = 10000)(implicit p: Parameters)
  extends UnitTest(timeout)
{
  val tileParams = p(TilesLocated(InSubsystem))(0).tileParams
  val coreParames = tileParams.core.asInstanceOf[BoomCoreParams]
  val coreWidth = coreParames.decodeWidth
  val numVecPhysRegs = coreParames.numVecPhysRegisters
  val vpregSz = log2Ceil(numVecPhysRegs)

  val cycle = Reg(UInt(32.W))
  when (io.start) {
    cycle := 0.U
  } .otherwise {
    cycle := cycle + 1.U
  }

  val active = RegInit(false.B)
  when (io.start) {
    active := true.B
  }

  val dut = Module(new VecRenameStage(coreWidth, numVecPhysRegs, 1))

  //val dec_fire = LCG(coreWidth, active)
  val dec_uops = WireInit(VecInit.tabulate(coreWidth)(x => {
    val ret = Wire(new MicroOp)
    ret := NullMicroOp(true)
    ret.vconfig := BoomTestUtils.NullVConfig
    ret
  }))
  def lreg_mask(emul: UInt): UInt = {
    val ret = Wire(UInt(5.W))
    ret := Mux(emul === 3.U, ~(7.U(5.W)),
           Mux(emul === 2.U, ~(3.U(5.W)),
           Mux(emul === 1.U, ~(1.U(5.W)), ~(0.U(5.W)))))
    ret
  }
  dec_uops.map(uop => {
      val vd_emul  = Cat(LCG(2, active)===3.U, LCG(2, active))
      val vs1_emul = Cat(LCG(2, active)===3.U, LCG(2, active))
      val vs2_emul = Cat(LCG(2, active)===3.U, LCG(2, active))
      uop.ldst_val    := LCG(1, active)
      uop.ldst        := LCG(5, active) & lreg_mask(vd_emul)
      uop.lrs1        := LCG(5, active) & lreg_mask(vs1_emul)
      uop.lrs2        := LCG(5, active) & lreg_mask(vs2_emul)
      uop.dst_rtype   := LCG(5, active)
      uop.lrs1_rtype  := LCG(5, active)
      uop.lrs2_rtype  := LCG(5, active)
      uop.frs3_en     := false.B
      uop.v_unmasked  := LCG(1, active)
      uop.vd_eew      := Cat(0.U(1.W),LCG(2, active))
      uop.vs1_eew     := Cat(0.U(1.W),LCG(2, active))
      uop.vs2_eew     := Cat(0.U(1.W),LCG(2, active))
      uop.vd_emul     := vd_emul
      uop.vs1_emul    := vs1_emul
      uop.vs2_emul    := vs2_emul
      uop.v_seg_nf    := 1.U
  })
  val dec_queue_enq = Wire(DecoupledIO(Vec(coreWidth, new MicroOp)))
  val dec_queue = Queue(dec_queue_enq, 2)
  val dec_done  = RegInit(0.U(coreWidth.W))
  val dis_done  = RegInit(0.U(coreWidth.W))
  val ren_stalls = Cat(dut.io.ren_stalls.scanLeft(false.B){case(s,acc) => s || acc}.tail.reverse)
  val dis_ready = LCG(1, active)(0)
  val dec_fire  = ~dec_done & Fill(coreWidth, (dec_queue.valid && dis_ready).asUInt) & ~ren_stalls
  val dis_fire  = dut.io.ren2_mask.asUInt & Fill(2, dis_ready) & ~dis_done & ~ren_stalls
  dec_queue_enq.valid := LCG(1, active)
  dec_queue_enq.bits  := dec_uops
  dec_queue.ready     := (dec_done | dec_fire).andR
  when ((dec_done | dec_fire).andR) {
    dec_done := 0.U
  }.otherwise {
    dec_done := dec_fire
  }
  when ((dis_done | dis_fire).andR) {
    dis_done := 0.U
  }.otherwise {
    dis_done := dis_fire
  }

  val brupdate = RegInit(BoomTestUtils.NullBrUpdateInfo)
  val dis_uops = Wire(Vec(coreWidth, Valid(new MicroOp)))
  val com = Pipe(
    dis_fire.orR,
    dis_uops,
    5)
  for (i <- 0 until coreWidth) {
    dis_uops(i).valid := dis_fire(i)
    dis_uops(i).bits  := dut.io.ren2_uops(i)
    dut.io.com_valids(i) := com.valid && com.bits(i).valid && com.bits(i).bits.ldst_val && com.bits(i).bits.dst_rtype(4)
    dut.io.com_uops(i)   := com.bits(i).bits
    //dut.io.dis_fire_first(i) := true.B
  }

  dut.io.kill       := false.B
  dut.io.dec_fire   := dec_fire.asBools
  dut.io.dec_uops   := dec_queue.bits
  dut.io.brupdate   := brupdate
  dut.io.wakeups(0) := BoomTestUtils.NullWakeup
  dut.io.dis_fire   := dis_fire.asBools
  dut.io.dis_ready  := dis_ready
  dut.io.rbk_valids.map(x => x := false.B)
  dut.io.rollback := false.B
  dut.io.debug_rob_empty := false.B

  for (w <- 0 until coreWidth) {
    val f = dis_fire(w)
    val u = dut.io.ren2_uops(w)
    when(f && u.ldst_val && u.dst_rtype(4)) {
      printf("dis(%d) vd[%x,m%x,nf%x,p(%x,%x,%x,%x,%x,%x,%x,%x)]<-vs2[%x,m%x,p(%x,%x,%x,%x,%x,%x,%x,%x)],vs1[%x,m%x,p(%x,%x,%x,%x,%x,%x,%x,%x)]\n",
        w.U,
        u.ldst, 1.U << Mux(u.vd_emul(2), 0.U, u.vd_emul(1,0)), u.v_seg_nf,
                Mux(u.pvd(0).valid, u.pvd(0).bits, 0.U),
                Mux(u.pvd(1).valid, u.pvd(1).bits, 0.U),
                Mux(u.pvd(2).valid, u.pvd(2).bits, 0.U),
                Mux(u.pvd(3).valid, u.pvd(3).bits, 0.U),
                Mux(u.pvd(4).valid, u.pvd(4).bits, 0.U),
                Mux(u.pvd(5).valid, u.pvd(5).bits, 0.U),
                Mux(u.pvd(6).valid, u.pvd(6).bits, 0.U),
                Mux(u.pvd(7).valid, u.pvd(7).bits, 0.U),
        u.lrs2, 1.U << Mux(u.vs2_emul(2), 0.U, u.vs2_emul(1,0)),
                Mux(u.pvs2(0).valid, u.pvs2(0).bits, 0.U),
                Mux(u.pvs2(1).valid, u.pvs2(1).bits, 0.U),
                Mux(u.pvs2(2).valid, u.pvs2(2).bits, 0.U),
                Mux(u.pvs2(3).valid, u.pvs2(3).bits, 0.U),
                Mux(u.pvs2(4).valid, u.pvs2(4).bits, 0.U),
                Mux(u.pvs2(5).valid, u.pvs2(5).bits, 0.U),
                Mux(u.pvs2(6).valid, u.pvs2(6).bits, 0.U),
                Mux(u.pvs2(7).valid, u.pvs2(7).bits, 0.U),
        u.lrs1, 1.U << Mux(u.vs1_emul(2), 0.U, u.vs1_emul(1,0)),
                Mux(u.pvs1(0).valid, u.pvs1(0).bits, 0.U),
                Mux(u.pvs1(1).valid, u.pvs1(1).bits, 0.U),
                Mux(u.pvs1(2).valid, u.pvs1(2).bits, 0.U),
                Mux(u.pvs1(3).valid, u.pvs1(3).bits, 0.U),
                Mux(u.pvs1(4).valid, u.pvs1(4).bits, 0.U),
                Mux(u.pvs1(5).valid, u.pvs1(5).bits, 0.U),
                Mux(u.pvs1(6).valid, u.pvs1(6).bits, 0.U),
                Mux(u.pvs1(7).valid, u.pvs1(7).bits, 0.U))
    }
  }
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
