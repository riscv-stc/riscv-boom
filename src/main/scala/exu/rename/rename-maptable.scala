//******************************************************************************
// Copyright (c) 2015 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Rename Map Table
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.exu

import chisel3._
import chisel3.util._
import boom.common._
import boom.util._
import freechips.rocketchip.config.Parameters

class MapReq(val lregSz: Int) extends Bundle
{
  val lrs1 = UInt(lregSz.W)
  val lrs2 = UInt(lregSz.W)
  val lrs3 = UInt(lregSz.W)
  val ldst = UInt(lregSz.W)
}

class MapResp(val pregSz: Int) extends Bundle
{
  val prs1 = UInt(pregSz.W)
  val prs2 = UInt(pregSz.W)
  val prs3 = UInt(pregSz.W)
  val stale_pdst = UInt(pregSz.W)
}

class RemapReq(val lregSz: Int, val pregSz: Int) extends Bundle
{
  val ldst = UInt(lregSz.W)
  val pdst = UInt(pregSz.W)
  val valid = Bool()
}

class RenameMapTable(
  val plWidth: Int,
  val numLregs: Int,
  val numPregs: Int,
  val bypass: Boolean,
  val float: Boolean)
  (implicit p: Parameters) extends BoomModule
{
  val pregSz = log2Ceil(numPregs)

  val io = IO(new BoomBundle()(p) {
    // Logical sources -> physical sources.
    val map_reqs    = Input(Vec(plWidth, new MapReq(lregSz)))
    val map_resps   = Output(Vec(plWidth, new MapResp(pregSz)))

    // Remapping an ldst to a newly allocated pdst?
    val remap_reqs  = Input(Vec(plWidth, new RemapReq(lregSz, pregSz)))

    // Dispatching branches: need to take snapshots of table state.
    val ren_br_tags = Input(Vec(plWidth, Valid(UInt(brTagSz.W))))

    // Signals for restoring state following misspeculation.
    val brupdate    = Input(new BrUpdateInfo)
    val rollback    = Input(Bool())
  })

  // The map table register array and its branch snapshots.
  val map_table = RegInit(VecInit(Seq.fill(numLregs){0.U(pregSz.W)}))
  val br_snapshots = Reg(Vec(maxBrCount, Vec(numLregs, UInt(pregSz.W))))

  // The intermediate states of the map table following modification by each pipeline slot.
  val remap_table = Wire(Vec(plWidth+1, Vec(numLregs, UInt(pregSz.W))))

  def lvdBitmap(lvd: UInt, emul: UInt, nf: UInt): UInt = {
    val ret = UIntToOH(lvd, numLregs)
    ret
  }

  // Uops requesting changes to the map table.
  val remap_pdsts = io.remap_reqs map (_.pdst)
  val remap_ldsts_oh = io.remap_reqs map (req => lvdBitmap(req.ldst, 0.U, 1.U) & Fill(numLregs, req.valid.asUInt))

  // Figure out the new mappings seen by each pipeline slot.
  for (i <- 0 until numLregs) {
    if (i == 0 && !float) {
      for (j <- 0 until plWidth+1) {
        remap_table(j)(i) := 0.U
      }
    } else {
      val remapped_row = (remap_ldsts_oh.map(ldst => ldst(i)) zip remap_pdsts)
        .scanLeft(map_table(i)) {case (pdst, (ldst, new_pdst)) => Mux(ldst, new_pdst.asUInt, pdst)}
      for (j <- 0 until plWidth+1) {
        remap_table(j)(i) := remapped_row(j)
      }
    }
  }

  // Create snapshots of new mappings.
  for (i <- 0 until plWidth) {
    when (io.ren_br_tags(i).valid) {
      br_snapshots(io.ren_br_tags(i).bits) := remap_table(i+1)
    }
  }

  when (io.brupdate.b2.mispredict) {
    // Restore the map table to a branch snapshot.
    map_table := br_snapshots(io.brupdate.b2.uop.br_tag)
  } .otherwise {
    // Update mappings.
    map_table := remap_table(plWidth)
  }

  // Read out mappings.
  for (i <- 0 until plWidth) {
    val mreq_lrs_es = Seq(io.map_reqs(i).lrs1,  io.map_reqs(i).lrs2, io.map_reqs(i).lrs3,  io.map_reqs(i).ldst)
    val mrsp_prs_es = Seq(io.map_resps(i).prs1, io.map_resps(i).prs2,io.map_resps(i).prs3, io.map_resps(i).stale_pdst)
    (mreq_lrs_es zip mrsp_prs_es).map{case (mreq_lrs, mrsp_prs) => {
      mrsp_prs := (0 until i).foldLeft(map_table(mreq_lrs)) ((prev,curr) =>
        Mux(bypass.B && io.remap_reqs(curr).valid && io.remap_reqs(curr).ldst === mreq_lrs, io.remap_reqs(curr).pdst, prev))
    }}
    if (!float) io.map_resps(i).prs3 := DontCare
  }

  // Don't flag the creation of duplicate 'p0' mappings during rollback.
  // These cases may occur soon after reset, as all maptable entries are initialized to 'p0'.
  io.remap_reqs map (req => (req.pdst, req.valid)) foreach {case (pd,r) =>
    assert(!r || !map_table.contains(pd.asUInt) || pd.asUInt === 0.U && io.rollback,
           "[maptable] Trying to write a duplicate mapping.")
  }
}

class VecMapReq(val lregSz: Int) extends Bundle
{
  val lrs1 = UInt(lregSz.W)
  val lrs2 = UInt(lregSz.W)
  val lrs3 = UInt(lregSz.W)
  val ldst = UInt(lregSz.W)
  val vs1_emul = UInt(3.W)
  val vs2_emul = UInt(3.W)
  val vd_emul  = UInt(3.W)
  val v_seg_nf = UInt(4.W)
}

class VecMapResp(val pregSz: Int) extends Bundle
{
  val prs1 = Vec(8, Valid(UInt(pregSz.W)))
  val prs2 = Vec(8, Valid(UInt(pregSz.W)))
  val prs3 = UInt(pregSz.W)
  val stale_pdst = Vec(8, Valid(UInt(pregSz.W)))
  val prvm = UInt(pregSz.W)
}

class VecRemapReq(val lregSz: Int, val pregSz: Int) extends Bundle
{
  val ldst = UInt(lregSz.W)
  val pdst = Vec(8, UInt(pregSz.W))
  val vd_emul = UInt(3.W)
  val v_seg_nf = UInt(4.W)
  val valid = Bool()
}

class VecVstartReq() extends Bundle
{
  val valid = Bool()
  val src   = UInt(1.W)
}

class VecRenameMapTable(
  val plWidth: Int,
  val numLregs: Int,
  val numPregs: Int,
  val bypass: Boolean)
  (implicit p: Parameters) extends BoomModule
{
  val pregSz = log2Ceil(numPregs)

  val io = IO(new BoomBundle()(p) {
    // Logical sources -> physical sources.
    val map_reqs    = Input(Vec(plWidth, new VecMapReq(lregSz)))
    val map_resps   = Output(Vec(plWidth, new VecMapResp(pregSz)))

    // Remapping an ldst to a newly allocated pdst?
    val remap_reqs  = Input(Vec(plWidth, new VecRemapReq(lregSz, pregSz)))

    // vstart source request
    val vstart_resps = Output(Vec(plWidth, UInt(1.W)))
    // reset vstart source
    val vstart_reqs  = Input(Vec(plWidth, new VecVstartReq()))

    // Dispatching branches: need to take snapshots of table state.
    val ren_br_tags = Input(Vec(plWidth, Valid(UInt(brTagSz.W))))

    // Signals for restoring state following misspeculation.
    val brupdate      = Input(new BrUpdateInfo)
    val rollback    = Input(Bool())
  })

  // The map table register array and its branch snapshots.
  val map_table = RegInit(VecInit(Seq.fill(numLregs){0.U(pregSz.W)}))
  val br_snapshots = Reg(Vec(maxBrCount, Vec(numLregs, UInt(pregSz.W))))

  // The intermediate states of the map table following modification by each pipeline slot.
  val remap_table = Wire(Vec(plWidth+1, Vec(numLregs, UInt(pregSz.W))))

  def lvdBitmap(lvd: UInt, emul: UInt, nf: UInt): UInt = {
    val ret =  {
      val grp = lvdGroup(lvd, emul, nf)
      grp.map(r => Mux(r.valid, UIntToOH(r.bits, numLregs), 0.U(numLregs.W))).reduce(_ | _)
    }
    ret
  }

  // Uops requesting changes to the map table.
  val remap_pdsts = io.remap_reqs map (_.pdst)
  val remap_ldsts_oh = io.remap_reqs map (req => lvdBitmap(req.ldst, req.vd_emul, req.v_seg_nf) &
                                                 Fill(numLregs, req.valid.asUInt))

  def isInGroup(lrs: UInt, refvd: UInt, refemul: UInt, refnf: UInt): Bool = {
    val grp = lvdGroup(refvd, refemul, refnf)
    val ret = grp.map(r => r.valid && lrs === r.bits).reduce(_ || _)
    ret
  }

  // Figure out the new mappings seen by each pipeline slot.
  for (i <- 0 until numLregs) {
    val remapped_row = (remap_ldsts_oh.map(ldst => ldst(i)) zip io.remap_reqs)
      .scanLeft(map_table(i)) {case (pdst, (ldst, req)) =>
        Mux(ldst && isInGroup(i.U, req.ldst, req.vd_emul, req.v_seg_nf), req.pdst(i.U - req.ldst), pdst)
      }

    for (j <- 0 until plWidth+1) {
      remap_table(j)(i) := remapped_row(j)
    }
  }

  // Create snapshots of new mappings.
  for (i <- 0 until plWidth) {
    when (io.ren_br_tags(i).valid) {
      br_snapshots(io.ren_br_tags(i).bits) := remap_table(i+1)
    }
  }

  when (io.brupdate.b2.mispredict) {
    // Restore the map table to a branch snapshot.
    map_table := br_snapshots(io.brupdate.b2.uop.br_tag)
  } .otherwise {
    // Update mappings.
    map_table := remap_table(plWidth)
  }

  // Read out mappings.
  for (i <- 0 until plWidth) {
    val mreq_lrs_es = Seq(io.map_reqs(i).lrs1,     io.map_reqs(i).lrs2,      io.map_reqs(i).ldst)
    val mreq_emul_s = Seq(io.map_reqs(i).vs1_emul, io.map_reqs(i).vs2_emul,  io.map_reqs(i).vd_emul)
    val mreq_nf_s   = Seq(1.U,                     1.U,                      io.map_reqs(i).v_seg_nf)
    val mrsp_prs_es = Seq(io.map_resps(i).prs1,    io.map_resps(i).prs2,     io.map_resps(i).stale_pdst)
    (mreq_lrs_es zip mreq_emul_s zip mreq_nf_s zip mrsp_prs_es).map{case (((mreq_lrs, mreq_emul), mreq_nf), mrsp_prs) => {
      val mreq_lgrp = lvdGroup(mreq_lrs, mreq_emul, mreq_nf)
      (mreq_lgrp zip mrsp_prs).map{case (mreq_lreg, mrsp_preg) => {
        mrsp_preg.valid := mreq_lreg.valid
        mrsp_preg.bits  := (0 until i).foldLeft(map_table(mreq_lreg.bits))((prev, curr) => {
          val remap_lgrp = lvdGroup(io.remap_reqs(curr).ldst, io.remap_reqs(curr).vd_emul, io.remap_reqs(curr).v_seg_nf)
          val remap_hits = remap_lgrp.map(rmlrs => rmlrs.valid && mreq_lreg.valid && rmlrs.bits === mreq_lreg.bits)
          assert(PopCount(remap_hits) <= 1.U)
          val the_remap_pdst = Mux1H(remap_hits, io.remap_reqs(curr).pdst)
          Mux(bypass.B && io.remap_reqs(curr).valid && remap_hits.reduce(_ || _), the_remap_pdst, prev)
        })
      }}
    }}
    io.map_resps(i).prvm := (0 until i).foldLeft(map_table(0.U)) ((prev,curr) =>
      Mux(bypass.B && io.remap_reqs(curr).valid && io.remap_reqs(curr).ldst === 0.U, io.remap_reqs(curr).pdst(0), prev))
    io.map_resps(i).prs3 := DontCare
  }

  //-------------------------------------------------------------------------------
  // vstartReset handling
  //-------------------------------------------------------------------------------
  // vstart source table
  val vstartSrc = RegInit(VSTART_CSR)
  val vstartSrcSnaps = Reg(Vec(maxBrCount, Bool()))

  // The intermediate states of the vstart reset table following modification by each pipeline slot.
  val vstartSrcNext = io.vstart_reqs.scanLeft(vstartSrc) { case(src, req) => Mux(req.valid, req.src, src)}

  // Create snapshots of new mappings.
  for (i <- 0 until plWidth) {
    when (io.ren_br_tags(i).valid) {
      vstartSrcSnaps(io.ren_br_tags(i).bits) := vstartSrcNext(i+1)
    }
  }

  when (io.brupdate.b2.mispredict) {
    // Restore the vstart reset table to a branch snapshot.
    vstartSrc := vstartSrcSnaps(io.brupdate.b2.uop.br_tag)
  } .otherwise {
    // Update mappings.
    vstartSrc := vstartSrcNext(plWidth)
  }

  // Read out mappings.
  io.vstart_resps := vstartSrcNext.take(plWidth)

  // Don't flag the creation of duplicate 'p0' mappings during rollback.
  // These cases may occur soon after reset, as all maptable entries are initialized to 'p0'.
  /* uop_exception is set in these cases
  io.remap_reqs.map(req => {
    when(req.valid) {
      when(req.vd_emul === 1.U) { assert(req.ldst(0)   === 0.U) }
      when(req.vd_emul === 2.U) { assert(req.ldst(1,0) === 0.U) }
      when(req.vd_emul === 3.U) { assert(req.ldst(2,0) === 0.U) }
    }
  })
  */
  io.remap_reqs map {req =>
    val nr = req.v_seg_nf << Mux(req.vd_emul(2), 0.U(2.W), req.vd_emul(1,0))
    for (i <- 0 until 8) {
      val pd = req.pdst
      assert(!req.valid || i.U >= nr || !map_table.contains(pd(i)) || pd(i) === 0.U && io.rollback,
             "[maptable] Trying to write a duplicate mapping.")
    }
  }
}
