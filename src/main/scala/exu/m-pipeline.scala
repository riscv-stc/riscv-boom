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


class VLSUWriteBack(val dataWidth: Int)(implicit p: Parameters) extends BoomBundle
{
  val addr = UInt(vpregSz.W)
  val data = UInt(dataWidth.W)
  val byteMask = UInt((dataWidth/8).W)
}
/**
 * Top level datapath that wraps the floating point issue window, regfile, and arithmetic units.
 */
class MatPipeline(implicit p: Parameters) extends BoomModule
{
  val matIssueParams = issueParams.find(_.iqType == IQT_MAT.litValue).get
  val dispatchWidth  = matIssueParams.dispatchWidth
  val numWakeupPorts = matWidth

  val io = IO(new Bundle {
    // pipeline ctrl signals
    val brupdate         = Input(new BrUpdateInfo())
    val flush_pipeline   = Input(Bool())
    // CSR infos
    val fcsr_rm          = Input(UInt(width=FPConstants.RM_SZ.W))
    val vxrm             = Input(UInt(2.W))
    val status           = Input(new MStatus())
    // dispatched uops
    val mat_dis_uops     = Vec(dispatchWidth, Flipped(Decoupled(new MicroOp)))
    // vlsu related
    /** vld ops may write one vreg multiple times but be freed when all done. */
    val vlsuWritePort    = Flipped(ValidIO(new VLSUWriteBack(vLen)))
    val vlsuLoadWakeUp   = Flipped(ValidIO(UInt(vpregSz.W)))
    /** Send vrf data to vlsu for indexed or masked load store. */
    val toVlsuRr: ValidIO[FuncUnitReq] = ValidIO(new FuncUnitReq(vLen))
    val vlsuReadReq: DecoupledIO[UInt] = Flipped(Decoupled(UInt(vpregSz.W)))
    val vlsuReadResp     = ValidIO(UInt(vLen.W))
    // vector pipeline related
    val fromVec          = Vec(matWidth, Flipped(Decoupled(new ExeUnitResp(vLen))))
    val toVec            = Vec(matWidth, Decoupled(new ExeUnitResp(vLen)))
    // vl_wakeup, vsetvl related wakeup
    val vl_wakeup        = Input(Valid(new VlWakeupResp()))
    val wakeups          = Vec(numWakeupPorts, Valid(new ExeUnitResp(vLen))) // wakeup issue_units for mem, int and fp

    val debug_tsc_reg    = Input(UInt(width=xLen.W))
    val debug_wb_wdata   = Output(Vec(numWakeupPorts, UInt((vLen).W)))
  })

  //**********************************
  // construct all of the modules
  val mat_issue_unit = Module(new IssueUnitCollapsing(matIssueParams, numWakeupPorts, vector = false, matrix = true))
  val trtileReg      = Module(new TrTileReg(vLen, numMatTrPhysRegs, mLen, vLen, exe_units.numTrTileReadPorts+1, 2))
  val trtileReader   = Module(new TileRegisterRead(
                         matWidth, 
                         exe_units.withFilter(_.readsTrTile).map(_.supportedFuncUnits),
                         exe_units.numTrTileReadPorts,
                         0,
                         0,
                         vLen, float = false, vector = false, matrix = true))
  val exe_units      = new boom.exu.ExecutionUnits(matrix=true)
  mat_issue_unit.suggestName("mat_issue_unit")

  require (exe_units.count(_.readsVrf) == matWidth + 1)
  require (exe_units.numVrfWritePorts + vecMemWidth == numWakeupPorts,
    s"${exe_units.numVrfWritePorts} + ${memWidth} + ${vecMemWidth} + ${numWakeupPorts}")
  require (vecWidth >= memWidth)

  //*************************************************************
  // Issue window logic

  val iss_valids = Wire(Vec(exe_units.numTrTileReaders, Bool()))
  val iss_uops   = Wire(Vec(exe_units.numTrTileReaders, new MicroOp()))

  mat_issue_unit.map(_.io.vbusy_status := io.vbusy_status)
  viu.map(_.io.tsc_reg := io.debug_tsc_reg)
  viu.map(_.io.brupdate := io.brupdate)
  viu.map(_.io.flush_pipeline := io.flush_pipeline)
  viu.map(_.io.vl_wakeup_port := io.vl_wakeup)
  //viu.map(_.io.vecUpdate := vregister_read.io.vecUpdate)
  // Don't support ld-hit speculation to VEC window.
  for (w <- 0 until memWidth) {
    viu.map(_.io.spec_ld_wakeup(w).valid := false.B)
    viu.map(_.io.spec_ld_wakeup(w).bits := 0.U)
  }
  viu.map(_.io.ld_miss := false.B)

  require (exe_units.numTotalBypassPorts == 0)

  //-------------------------------------------------------------
  // **** Dispatch Stage ****
  //-------------------------------------------------------------

  // Input (Dispatch)
  for (w <- 0 until dispatchWidth) {
    mat_issue_unit.io.dis_uops(w) <> io.mat_dis_uops(w)
  }

  //-------------------------------------------------------------
  // **** Issue Stage ****
  //-------------------------------------------------------------
  // Output (Issue)
  for (i <- 0 until matWidth) {
    iss_valids(i) := mat_issue_unit.io.iss_valids(i)
    iss_uops(i) := mat_issue_unit.io.iss_uops(i)
    mat_issue_unit.io.fu_types(i) := exe_units(i).io.fu_types
    require (exe_units(i).readsTrTile)
  }

  // Wakeup
  mat_issue_unit.map(iu => {
    // matrix issue units are not using these wake up directly, so tie them up.
    iu.io.wakeup_ports.foreach{ wake =>
      wake.valid := false.B
      wake.bits := DontCare
    }
    iu.io.pred_wakeup_port.valid  := false.B
    iu.io.pred_wakeup_port.bits   := DontCare
  })

  //-------------------------------------------------------------
  // **** Register Read Stage ****
  //-------------------------------------------------------------

  // Register Read <- Issue (rrd <- iss)
  vregister_read.io.rf_read_ports <> VecInit(vregfile.io.read_ports.init)
  vregister_read.io.prf_read_ports map { port => port.data := false.B }

  vregister_read.io.iss_valids <> iss_valids
  vregister_read.io.iss_uops := iss_uops

  vregister_read.io.brupdate := io.brupdate
  vregister_read.io.kill := io.flush_pipeline
  //io.vmupdate := vregister_read.io.vmupdate

  // Only one port for vector load write back.
  // ts1
  trtileReg.io.readPorts(0).msew      := 
  trtileReg.io.readPorts(0).tilewidth := 
  trtileReg.io.readPorts(0).tt        := 
  trtileReg.io.readPorts(0).addr      := io.mseReadReq.bits.reqaddr.addr
  trtileReg.io.readPorts(0).index     := io.mseReadReq.bits.reqaddr.index
  // ts2
  trtileReg.io.readPorts(1).msew      := 
  trtileReg.io.readPorts(1).tilewidth := 
  trtileReg.io.readPorts(1).tt        := 
  trtileReg.io.readPorts(1).addr      := io.mseReadReq.bits.reqaddr.addr
  trtileReg.io.readPorts(1).index     := io.mseReadReq.bits.reqaddr.index
  // vlsu read store data
  trtileReg.io.readPorts(2).msew      := 
  trtileReg.io.readPorts(2).tilewidth := 
  trtileReg.io.readPorts(2).tt        := 
  trtileReg.io.readPorts(2).addr      := io.mseReadReq.bits.reqaddr.addr
  trtileReg.io.readPorts(2).index     := io.mseReadReq.bits.reqaddr.index
  vregfile.io.read_ports.last.addr := Mux(io.vlsuReadReq.valid, io.vlsuReadReq.bits, 0.U)
  io.vlsuReadResp.valid := RegNext(io.vlsuReadReq.valid)
  io.vlsuReadResp.bits := vregfile.io.read_ports.last.data
  //-------------------------------------------------------------
  // **** Execute Stage ****
  //-------------------------------------------------------------

  exe_units.map(_.io.brupdate := io.brupdate)

  for ((ex,w) <- exe_units.withFilter(_.readsVrf).map(x=>x).zipWithIndex) {
    ex.io.req <> vregister_read.io.exe_reqs(w)
    require (!ex.bypassable)
  }
  require (exe_units.numTotalBypassPorts == 0)

  //-------------------------------------------------------------
  // **** Writeback Stage ****
  //-------------------------------------------------------------
  (io.toVec zip exe_units.withFilter(_.hasAlu).map(_.io.iresp)).foreach {
    case (to_int, vec) => to_int <> vec
  }

  // assign a write port for vlsu and vector pipeline each, no arbitration needed
  // Cut up critical path by delaying the write by a cycle.
  // Wakeup signal is sent on cycle S0, write is now delayed until end of S1,
  // but Issue happens on S1 and RegRead doesn't happen until S2 so we're safe.
  trtileReg.io.writePorts(0).valid          := io.vlsuWritePort.valid
  trtileReg.io.writePorts(0).bits.msew      := io.vlsuWritePort.bits.addr
  trtileReg.io.writePorts(0).bits.tilewidth := io.vlsuWritePort.bits.data
  trtileReg.io.writePorts(0).bits.tt        := MaskExploder(io.vlsuWritePort.bits.byteMask, vLen)
  trtileReg.io.writePorts(0).bits.addr      := 
  trtileReg.io.writePorts(0).bits.index     := 
  trtileReg.io.writePorts(0).bits.data      := 
  // from vector pipeline
  trtileReg.io.writePorts(1).valid          := io.vlsuWritePort.valid
  trtileReg.io.writePorts(1).bits.msew      := io.vlsuWritePort.bits.addr
  trtileReg.io.writePorts(1).bits.tilewidth := io.vlsuWritePort.bits.data
  trtileReg.io.writePorts(1).bits.tt        := MaskExploder(io.vlsuWritePort.bits.byteMask, vLen)
  trtileReg.io.writePorts(1).bits.addr      := 
  trtileReg.io.writePorts(1).bits.index     := 
  trtileReg.io.writePorts(1).bits.data      := 
  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Commit Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  //io.wakeups(0).valid := ll_wbarb.io.out.valid
  //io.wakeups(0).bits := ll_wbarb.io.out.bits
  //ll_wbarb.io.out.ready := true.B

  w_cnt = 0
  //vld write back clears busy table in rename but not busy bit in rob entry.
  io.wakeups(w_cnt) <> DontCare
  io.wakeups(w_cnt).valid := io.vlsuLoadWakeUp.valid
  io.wakeups(w_cnt).bits.data := 0.U
  io.wakeups(w_cnt).bits.uop.is_rvm    := true.B
  io.wakeups(w_cnt).bits.uop.uses_ldq  := true.B
  io.wakeups(w_cnt).bits.uop.dst_rtype := RT_MAT
  io.wakeups(w_cnt).bits.uop.pdst      := io.vlsuLoadWakeUp.bits
  // from vector pipeline
  w_cnt = 1
  io.wakeups(w_cnt).valid := io.vlsuLoadWakeUp.valid
  io.wakeups(w_cnt).bits.data := 0.U
  io.wakeups(w_cnt).bits.uop.is_rvm    := true.B
  io.wakeups(w_cnt).bits.uop.uses_ldq  := true.B
  io.wakeups(w_cnt).bits.uop.dst_rtype := RT_MAT
  io.wakeups(w_cnt).bits.uop.pdst      := io.vlsuLoadWakeUp.bits
  // from mxu unit
  w_cnt = 2
  io.wakeups(w_cnt).valid := io.vlsuLoadWakeUp.valid
  io.wakeups(w_cnt).bits.data := 0.U
  io.wakeups(w_cnt).bits.uop.is_rvm    := true.B
  io.wakeups(w_cnt).bits.uop.uses_ldq  := true.B
  io.wakeups(w_cnt).bits.uop.dst_rtype := RT_MAT
  io.wakeups(w_cnt).bits.uop.pdst      := io.vlsuLoadWakeUp.bits

  for ((wdata, wakeup) <- io.debug_wb_wdata zip io.wakeups) {
    wdata := wakeup.bits.data
  }

  exe_units.withFilter(_.hasFcsr).map(_.io.fcsr_rm := io.fcsr_rm)
  exe_units.withFilter(_.hasVxrm).map(_.io.vxrm := io.vxrm)
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
