//******************************************************************************
// Copyright (c) 2015 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Vector Datapath Pipeline
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.exu

import Chisel.UInt
import chisel3._
import chisel3.util._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.rocket.MStatus
import freechips.rocketchip.tile.FPConstants
import freechips.rocketchip.util.UIntIsOneOf
import boom.exu.FUConstants._
import boom.common._
import boom.common.MicroOpcodes._
import boom.util._

/**
 * Top level datapath that wraps the floating point issue window, regfile, and arithmetic units.
 */
class MatPipeline(implicit p: Parameters) extends BoomModule {
  val matIssueParams = issueParams.find(_.iqType == IQT_MAT.litValue).get
  val dispatchWidth = matIssueParams.dispatchWidth
  val numWakeupPorts = matWidth * 4 + numVLdPorts * 2  // (MCLRACC; MOPA) + (VLSU to tr tile + VLSU to acc tile)

  val io = IO(new Bundle {
    // pipeline ctrl signals
    val brupdate = Input(new BrUpdateInfo())
    val flush_pipeline = Input(Bool())
    // CSR infos
    val fcsr_rm          = Input(UInt(width=FPConstants.RM_SZ.W))
    val status           = Input(new MStatus())
    val tilem            = Input(UInt(xLen.W))
    val tilen            = Input(UInt(xLen.W))
    val tilek            = Input(UInt(xLen.W))
    val wake_issue_prs = Input(Vec(2,Vec(memWidth + matWidth,UInt((vLenb+1).W))))
    val wake_issue_rs_type = Input(Vec(2,Vec(memWidth + matWidth,UInt(RT_X.getWidth.W)))) 
    val wake_issue_data = Input(Vec(2,Vec(memWidth + matWidth,UInt((vLenb+1).W))))
    val wake_issue_valid = Input(Vec(2,Vec(memWidth + matWidth,Bool())))
    val vbusy_status     = Input(UInt(numVecPhysRegs.W))

    // dispatched uops
    val dis_uops = Vec(dispatchWidth, Flipped(Decoupled(new MicroOp)))
    // vector pipeline related
    val toVec = Vec(matWidth, Decoupled(new ExeUnitResp(vLen)))
    // scalar pipeline related
    val intupdate = Input(Vec(intWidth, Valid(new ExeUnitResp(eLen))))
    val fpupdate = Input(Vec(fpWidth, Valid(new ExeUnitResp(eLen))))
    val lsu_tile_rport = Vec(memWidth, new TrTileRegReadPortIO())
    val vec_rport     = Vec(2, Flipped(new RegisterFileReadPortIO(vpregSz, vLen)))
    val lsu_tile_wbk = Vec(numVLdPorts * 2, Flipped(Decoupled(new ExeUnitResp(vLen))))
    val lsu_acc_rreq = Flipped(ValidIO(new AccReadReq()))
    val lsu_acc_rresp = ValidIO(new AccReadResp())
    // mset_wakeup, vsetvl related wakeup
    // val mset_wakeup        = Input(Valid(new MlWakeupResp()))  // TODO: msettype/msettile speculation optimization
    val wakeups = Vec(numWakeupPorts, Valid(new ExeUnitResp(vLen))) // wakeup issue_units
    val wakeup_bypass = Output(Vec(numWakeupPorts, Bool()))
    val vl_wakeup = Input(Valid(new VlWakeupResp()))

    // Clear tile registers.
    val trclr = Flipped(Vec(memWidth, Valid(UInt(log2Ceil(numMatTrPhysRegs).W))))

    val mtype_wakeup        = Input(Valid(new MtypeWakeupResp()))
    val tile_m_wakeup        = Input(Valid(new MtileWakeupResp()))
    val tile_n_wakeup        = Input(Valid(new MtileWakeupResp()))
    val tile_k_wakeup        = Input(Valid(new MtileWakeupResp()))
    val matrix_iss_valid = Output(Vec(matWidth,Bool()))
    val matrix_iss_uop = Output(Vec(matWidth,new MicroOp()))
    val debug_tsc_reg    = Input(UInt(width=xLen.W))
    val debug_wb_wdata   = Output(Vec(numWakeupPorts, UInt((vLen).W)))


    val perf = Output(new Bundle {
      val iss_valids = Vec(matIssueParam.issueWidth, Bool())
      val req_valids = Vec(matIssueParam.issueWidth, Bool())
      val iss_slots_empty = Bool()
      val iss_slots_full = Bool()
    })
  })

  //**********************************
  // construct all of the modules
  val issue_unit = Module(new IssueUnitCollapsing(matIssueParams, numWakeupPorts, vector = false, matrix = true))
  val exe_units = new boom.exu.ExecutionUnits(matrix = true)
  val trtileReg = Module(new TrTileReg(exe_units.numTrTileReadPorts + memWidth, numVLdPorts))
  val trtileReader = Module(new TileRegisterRead(
    matWidth,
    exe_units.withFilter(_.readsTrTile).map(_.supportedFuncUnits),
    exe_units.numTrTileReadPorts,
    exe_units.withFilter(_.readsTrTile).map(_ => 2),
    vLen))
  issue_unit.suggestName("mat_issue_unit")

  //*************************************************************
  // Issue window logic

  val iss_valids = Wire(Vec(exe_units.numTrTileReaders, Bool()))
  val iss_uops = Wire(Vec(exe_units.numTrTileReaders, new MicroOp()))

  issue_unit.io.tsc_reg := io.debug_tsc_reg
  issue_unit.io.brupdate := io.brupdate
  issue_unit.io.flush_pipeline := io.flush_pipeline
  issue_unit.io.intupdate := io.intupdate
  issue_unit.io.vbusy_status   := io.vbusy_status
  issue_unit.io.fpupdate  := io.fpupdate
  issue_unit.io.mtype_wakeup := io.mtype_wakeup
  issue_unit.io.tile_m_wakeup := io.tile_m_wakeup
  issue_unit.io.tile_n_wakeup := io.tile_n_wakeup
  issue_unit.io.tile_k_wakeup := io.tile_k_wakeup

  for (w <- 0 until memWidth) {
    issue_unit.io.spec_ld_wakeup(w).valid := false.B
    issue_unit.io.spec_ld_wakeup(w).bits := 0.U
  }
  issue_unit.io.ld_miss := false.B

  require(exe_units.numTotalBypassPorts == 0)

  //-------------------------------------------------------------
  // **** Dispatch Stage ****
  //-------------------------------------------------------------

  // Input (Dispatch)
  for (w <- 0 until dispatchWidth) {
    issue_unit.io.dis_uops(w) <> io.dis_uops(w)
  }

  //-------------------------------------------------------------
  // **** Issue Stage ****
  //-------------------------------------------------------------
  io.perf.iss_valids := issue_unit.io.iss_valids
  io.perf.req_valids := (0 until matWidth).map(i => exe_units(i).io.req.valid)
  io.perf.iss_slots_empty := issue_unit.io.perf.empty
  io.perf.iss_slots_full := issue_unit.io.perf.full

  // Output (Issue)
  for (i <- 0 until matWidth) {
    iss_valids(i) := issue_unit.io.iss_valids(i)
    iss_uops(i) := issue_unit.io.iss_uops(i)
    issue_unit.io.fu_types(i) := exe_units(i).io.fu_types
    require(exe_units(i).readsTrTile)
  }
  val wake_tile_r = Wire(Vec(matWidth,Bool()))
  wake_tile_r :=  issue_unit.io.wake_tile_r
  issue_unit.io.wake_issue_valid := io.wake_issue_valid
  issue_unit.io.wake_issue_data := io.wake_issue_data
  issue_unit.io.wake_issue_prs  := io.wake_issue_prs
  issue_unit.io.wake_issue_rs_type  := io.wake_issue_rs_type
  io.matrix_iss_valid := wake_tile_r
  io.matrix_iss_uop := iss_uops

  for (n <- 0 until numWakeupPorts) {
    io.wakeup_bypass(n) := false.B
  }

  // Wakeup
  for ((writeback, issue_wakeup) <- io.wakeups zip issue_unit.io.wakeup_ports) {
    issue_wakeup.valid := writeback.valid
    issue_wakeup.bits.pdst := writeback.bits.uop.pdst
    issue_wakeup.bits.poisoned := false.B
    issue_wakeup.bits.uop := writeback.bits.uop
  }
  issue_unit.io.pred_wakeup_port.valid := false.B
  issue_unit.io.pred_wakeup_port.bits := DontCare
  //issue_unit.io.vl_wakeup.valid        := false.B
  //issue_unit.io.vl_wakeup.bits         := DontCare
  issue_unit.io.vl_wakeup := io.vl_wakeup

  //-------------------------------------------------------------
  // **** Register Read Stage ****
  //-------------------------------------------------------------

  // Register Read <- Issue (rrd <- iss)
  trtileReader.io.iss_valids <> iss_valids
  trtileReader.io.iss_uops := iss_uops

  trtileReader.io.brupdate := io.brupdate
  trtileReader.io.kill := io.flush_pipeline

  // Only one port for vector load write back.
  for (i <- 0 until exe_units.numTrTileReadPorts) {
    trtileReader.io.tileReadPorts(i) <> trtileReg.io.readPorts(i)
  }
  for(w <- 0 until memWidth) {
    io.lsu_tile_rport(w) <> trtileReg.io.readPorts(exe_units.numTrTileReadPorts + w)
  }
  for(w <- 0 until 2) {
    io.vec_rport(w).addr := trtileReader.io.vec_rport(w).addr
    trtileReader.io.vec_rport(w).data := io.vec_rport(w).data
  }
  trtileReg.io.clearPorts := io.trclr
  //-------------------------------------------------------------
  // **** Execute Stage ****
  //-------------------------------------------------------------

  exe_units.map(_.io.brupdate := io.brupdate)

  for ((ex, w) <- exe_units.withFilter(_.readsTrTile).map(x => x).zipWithIndex) {
    ex.io.req <> trtileReader.io.exe_reqs(w)
    require(!ex.bypassable)
  }
  require(exe_units.numTotalBypassPorts == 0)

  //-------------------------------------------------------------
  // **** Writeback Stage ****
  //-------------------------------------------------------------
  (io.toVec zip exe_units.withFilter(_.writesLlVrf).map(_.io.ll_vresp)).foreach {
    case (toVec, vec) => toVec <> vec
  }

  // assign a write port for vlsu and vector pipeline each, no arbitration needed
  // Cut up critical path by delaying the write by a cycle.
  // Wakeup signal is sent on cycle S0, write is now delayed until end of S1,
  // but Issue happens on S1 and RegRead doesn't happen until S2 so we're safe.
  // TODO: wrap vlsu write with uops for tr_tile write control
  for (w <- 0 until numVLdPorts) {
    val lsuWbkBits = io.lsu_tile_wbk(w).bits
    trtileReg.io.writePorts(w).valid := io.lsu_tile_wbk(w).valid && lsuWbkBits.uop.rt(RD, isTrTile)
    trtileReg.io.writePorts(w).bits.msew := lsuWbkBits.uop.m_ls_ew
    trtileReg.io.writePorts(w).bits.tt := lsuWbkBits.uop.rt(RD, isTrTile).asUInt ## !lsuWbkBits.uop.isHSlice.asUInt
    trtileReg.io.writePorts(w).bits.addr := lsuWbkBits.uop.pdst
    trtileReg.io.writePorts(w).bits.index := lsuWbkBits.uop.m_sidx
    trtileReg.io.writePorts(w).bits.data := lsuWbkBits.data
    trtileReg.io.writePorts(w).bits.byteMask := lsuWbkBits.vmask
    trtileReg.io.writePorts(w).bits.quad := lsuWbkBits.uop.m_slice_quad

    exe_units.withFilter(_.writesAccTile).map(eu => {
      eu.io.mlsuWbk(w).valid := io.lsu_tile_wbk(w).valid && lsuWbkBits.uop.rt(RD, isAccTile)
      eu.io.mlsuWbk(w).bits := io.lsu_tile_wbk(w).bits
      eu.io.accReadReq := io.lsu_acc_rreq
      io.lsu_acc_rresp := eu.io.accReadResp
    })

    //-------------------------------------------------------------
    //-------------------------------------------------------------
    // **** Commit Stage ****
    //-------------------------------------------------------------
    //-------------------------------------------------------------

    //io.wakeups(0).valid := ll_wbarb.io.out.valid
    //io.wakeups(0).bits := ll_wbarb.io.out.bits
    //ll_wbarb.io.out.ready := true.B

    //vld write back clears busy table in rename but not busy bit in rob entry.
    io.wakeups(w) <> DontCare
    io.wakeups(w).valid := io.lsu_tile_wbk(w).valid && lsuWbkBits.uop.rt(RD, isTrTile)
    io.wakeups(w).bits  := io.lsu_tile_wbk(w).bits
  }

  for (w <- numVLdPorts until numVLdPorts * 2) {
    io.wakeups(w) := io.lsu_tile_wbk(w)
    io.wakeup_bypass(w) := true.B
  }

  // from MatExeUnit
  var w_cnt = numVLdPorts * 2
  for(eu <- exe_units) {
    io.wakeups(w_cnt)   := eu.io.mclrResp
    io.wakeups(w_cnt+1) := eu.io.mopaResp
    for (w <- 0 until numVLdPorts){
      io.wakeups(w_cnt+ 2+ w) := eu.io.mlsuResp(w)
    }
    w_cnt += 2 + numVLdPorts
  }

  for ((wdata, wakeup) <- io.debug_wb_wdata zip io.wakeups) {
    wdata := wakeup.bits.data
  }

  exe_units.withFilter(_.hasFcsr).map(_.io.fcsr_rm := io.fcsr_rm)
  exe_units.map(_.io.status := io.status)

  //-------------------------------------------------------------
  // **** Flush Pipeline ****
  //-------------------------------------------------------------
  // flush on exceptions, miniexeptions, and after some special instructions

  for (w <- 0 until exe_units.length) {
    exe_units(w).io.req.bits.kill := io.flush_pipeline
  }

  override def toString: String =
    (BoomCoreStringPrefix("===Matrix Pipeline===") + "\n"
    // + vregfile.toString
    + BoomCoreStringPrefix(
      "Num Wakeup Ports      : " + numWakeupPorts,
      "Num Bypass Ports      : " + exe_units.numTotalBypassPorts))
}
