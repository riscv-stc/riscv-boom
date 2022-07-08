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

class TileAccessInfo(implicit p: Parameters) extends BoomBundle {
  val ridx = UInt(vpregSz.W)
  val sidx = UInt(vLenSz.W)
  val tt   = UInt(2.W)
}

/**
 * Top level datapath that wraps the floating point issue window, regfile, and arithmetic units.
 */
class MatPipeline(implicit p: Parameters) extends BoomModule
{
  val matIssueParams = issueParams.find(_.iqType == IQT_MAT.litValue).get
  val dispatchWidth  = matIssueParams.dispatchWidth
  val numWakeupPorts = matWidth*2 + 1     // (MCLRACC; MOPA) + VLSU

  val io = IO(new Bundle {
    // pipeline ctrl signals
    val brupdate         = Input(new BrUpdateInfo())
    val flush_pipeline   = Input(Bool())
    // CSR infos
    val fcsr_rm          = Input(UInt(width=FPConstants.RM_SZ.W))
    val status           = Input(new MStatus())
    val tilem            = Input(UInt(xLen.W))
    val tilen            = Input(UInt(xLen.W))
    val tilek            = Input(UInt(xLen.W))
    // dispatched uops
    val dis_uops         = Vec(dispatchWidth, Flipped(Decoupled(new MicroOp)))
    // vector pipeline related
    val toVec            = Vec(matWidth, Decoupled(new ExeUnitResp(vLen)))
    // scalar pipeline related
    val intupdate        = Input(Vec(intWidth, Valid(new ExeUnitResp(eLen))))
    // mset_wakeup, vsetvl related wakeup
    // val mset_wakeup        = Input(Valid(new MlWakeupResp()))  // TODO: msettype/msettile speculation optimization
    val wakeups          = Vec(numWakeupPorts, Valid(new ExeUnitResp(vLen))) // wakeup issue_units

    val debug_tsc_reg    = Input(UInt(width=xLen.W))
    val debug_wb_wdata   = Output(Vec(numWakeupPorts, UInt((vLen).W)))
  })

  //**********************************
  // construct all of the modules
  val issue_unit   = Module(new IssueUnitCollapsing(matIssueParams, numWakeupPorts, vector = false, matrix = true))
  val exe_units    = new boom.exu.ExecutionUnits(matrix=true)
  val trtileReg    = Module(new TrTileReg(exe_units.numTrTileReadPorts+1, 1))
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
  val iss_uops   = Wire(Vec(exe_units.numTrTileReaders, new MicroOp()))

  issue_unit.io.tsc_reg := io.debug_tsc_reg
  issue_unit.io.brupdate := io.brupdate
  issue_unit.io.flush_pipeline := io.flush_pipeline
  issue_unit.io.intupdate := io.intupdate

  for (w <- 0 until memWidth) {
    issue_unit.io.spec_ld_wakeup(w).valid := false.B
    issue_unit.io.spec_ld_wakeup(w).bits  := 0.U
  }
  issue_unit.io.ld_miss := false.B

  require (exe_units.numTotalBypassPorts == 0)

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
  // Output (Issue)
  for (i <- 0 until matWidth) {
    iss_valids(i) := issue_unit.io.iss_valids(i)
    iss_uops(i)   := issue_unit.io.iss_uops(i)
    issue_unit.io.fu_types(i) := exe_units(i).io.fu_types
    require (exe_units(i).readsTrTile)
  }

  // Wakeup
  for ((writeback, issue_wakeup) <- io.wakeups zip issue_unit.io.wakeup_ports) {
    issue_wakeup.valid         := writeback.valid
    issue_wakeup.bits.pdst     := writeback.bits.uop.pdst
    issue_wakeup.bits.poisoned := false.B
    issue_wakeup.bits.uop      := writeback.bits.uop
  }
  issue_unit.io.pred_wakeup_port.valid := false.B
  issue_unit.io.pred_wakeup_port.bits  := DontCare
  issue_unit.io.vl_wakeup_port.valid := false.B
  issue_unit.io.vl_wakeup_port.bits := DontCare

  //-------------------------------------------------------------
  // **** Register Read Stage ****
  //-------------------------------------------------------------

  // Register Read <- Issue (rrd <- iss)
  trtileReader.io.iss_valids <> iss_valids
  trtileReader.io.iss_uops := iss_uops

  trtileReader.io.brupdate := io.brupdate
  trtileReader.io.kill     := io.flush_pipeline

  // Only one port for vector load write back.
  for(i <- 0 until exe_units.numTrTileReadPorts) {
    trtileReader.io.tileReadPorts(i) <> trtileReg.io.readPorts(i)
  }
  //FIXME:read port of vlsu
  trtileReg.io.readPorts(exe_units.numTrTileReadPorts) := DontCare
  //-------------------------------------------------------------
  // **** Execute Stage ****
  //-------------------------------------------------------------

  exe_units.map(_.io.brupdate := io.brupdate)

  for ((ex,w) <- exe_units.withFilter(_.readsTrTile).map(x=>x).zipWithIndex) {
    ex.io.req <> trtileReader.io.exe_reqs(w)
    require (!ex.bypassable)
  }
  require (exe_units.numTotalBypassPorts == 0)

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
  trtileReg.io.writePorts(0).valid          := DontCare
  trtileReg.io.writePorts(0).bits.msew      := 0.U
  trtileReg.io.writePorts(0).bits.tt        := 0.U
  trtileReg.io.writePorts(0).bits.addr      := 0.U
  trtileReg.io.writePorts(0).bits.index     := 0.U
  trtileReg.io.writePorts(0).bits.data      := 0.U
  trtileReg.io.writePorts(0).bits.byteMask  := 0.U
  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Commit Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  //io.wakeups(0).valid := ll_wbarb.io.out.valid
  //io.wakeups(0).bits := ll_wbarb.io.out.bits
  //ll_wbarb.io.out.ready := true.B

  var w_cnt = 0
  //vld write back clears busy table in rename but not busy bit in rob entry.
  io.wakeups(w_cnt) <> DontCare
  // from MatExeUnit
  w_cnt = 1
  for(eu <- exe_units) {
    io.wakeups(w_cnt)   := eu.io.mclrResp
    io.wakeups(w_cnt+1) := eu.io.mopaResp
    w_cnt += 2
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
