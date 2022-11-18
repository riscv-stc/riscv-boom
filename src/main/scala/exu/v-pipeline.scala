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
class VecPipeline(implicit p: Parameters) extends BoomModule
{
  val vecIssueParams = issueParams.find(_.iqType == IQT_VEC.litValue).get
  val dispatchWidth = vecIssueParams.dispatchWidth
  val numWakeupPorts = vecWidth + memWidth // internal wakeups

  val io = IO(new Bundle {
    val brupdate         = Input(new BrUpdateInfo())
    val flush_pipeline   = Input(Bool())
    val fcsr_rm          = Input(UInt(width=FPConstants.RM_SZ.W))
    val vxrm             = Input(UInt(2.W))
    val status           = Input(new MStatus())

    val dis_uops         = Vec(dispatchWidth, Flipped(Decoupled(new MicroOp)))
    val vbusy_status     = Input(UInt(numVecPhysRegs.W))

    val fromMat          = if (usingMatrix) Vec(matWidth, Flipped(Decoupled(new ExeUnitResp(vLen)))) else null
    val ll_wports        = Vec(numVLdPorts, Flipped(Decoupled(new ExeUnitResp(vLen))))
    val to_int           = Vec(vecWidth, Decoupled(new ExeUnitResp(eLen)))
    val to_fp            = Vec(vecWidth, Decoupled(new ExeUnitResp(eLen)))
    val intupdate        = Input(Vec(intWidth, Valid(new ExeUnitResp(eLen))))
    val fpupdate         = Input(Vec(fpWidth, Valid(new ExeUnitResp(eLen))))
    val lsu_vrf_rport    = Vec(memWidth, new RegisterFileReadPortIO(vpregSz, vLen))

    val vl_wakeup        = Input(Valid(new VlWakeupResp()))
    val wakeups          = Vec(numWakeupPorts, Valid(new ExeUnitResp(vLen))) // wakeup issue_units for mem, int and fp

    val debug_tsc_reg    = Input(UInt(width=xLen.W))
    val debug_wb_wdata   = Output(Vec(numWakeupPorts, UInt((eLen).W)))

    val perf = Output(new Bundle {
      val iss_valids      = Vec(vecIssueParams.issueWidth, Bool())
      val req_valids      = Vec(vecIssueParams.issueWidth, Bool())
      val iss_slots_empty = Bool()
      val iss_slots_full  = Bool()
      val div_busy        = Vec(vecIssueParams.issueWidth, Bool())
      val fdiv_busy       = Vec(vecIssueParams.issueWidth, Bool())
    })
  })

  //**********************************
  // construct all of the modules

  val exe_units  = new boom.exu.ExecutionUnits(fpu=false, vector=true)
  val issue_unit = Module(new IssueUnitCollapsing(
                         vecIssueParams,
                         numWakeupPorts, vector = true))
  issue_unit.suggestName("vec_issue_unit")

  val vregfile       = Module(new RegisterFileSynthesizable(numVecPhysRegs,
                         exe_units.numVrfReadPorts + memWidth,
                         exe_units.numVrfWritePorts + memWidth,
                         vLen,
                         // No bypassing for any VEC units, + memWidth for ll_wb
                         Seq.fill(exe_units.numVrfWritePorts + memWidth){ false },
                         vector = true))
  val vregister_read = Module(new RegisterRead(
                         vecWidth,
                         exe_units.withFilter(_.readsVrf).map(_.supportedFuncUnits),
                         exe_units.numVrfReadPorts,
                         exe_units.withFilter(_.readsVrf).map(x => 4),
                         0, // No bypass for VEC
                         0,
                         vLen, float = false, vector = true))

  require (exe_units.count(_.readsVrf) == vecWidth)
  require (exe_units.numVrfWritePorts + memWidth == numWakeupPorts)
  //require (vecWidth >= memWidth)

  //*************************************************************
  // Issue window logic

  val iss_valids = Wire(Vec(exe_units.numVrfReaders, Bool()))
  val iss_uops   = Wire(Vec(exe_units.numVrfReaders, new MicroOp()))

  issue_unit.io.vbusy_status   := io.vbusy_status
  issue_unit.io.tsc_reg        := io.debug_tsc_reg
  issue_unit.io.brupdate       := io.brupdate
  issue_unit.io.flush_pipeline := io.flush_pipeline
  issue_unit.io.intupdate      := io.intupdate
  issue_unit.io.fpupdate       := io.fpupdate
  issue_unit.io.vl_wakeup      := io.vl_wakeup
  // Don't support ld-hit speculation to VEC window.
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
  io.perf.iss_valids      := issue_unit.io.iss_valids
  io.perf.req_valids      := (0 until vecWidth).map(i => exe_units(i).io.req.valid)
  io.perf.iss_slots_empty := issue_unit.io.perf.empty
  io.perf.iss_slots_full  := issue_unit.io.perf.full
  io.perf.div_busy        := (0 until vecWidth).map(i => if(exe_units(i).hasDiv)  exe_units(i).io.perf.div_busy  else false.B)
  io.perf.fdiv_busy       := (0 until vecWidth).map(i => if(exe_units(i).hasFdiv) exe_units(i).io.perf.fdiv_busy else false.B)

  // Output (Issue)
  for (i <- 0 until vecWidth) {
    iss_valids(i) := issue_unit.io.iss_valids(i)
    iss_uops(i) := issue_unit.io.iss_uops(i)

    var fu_types = exe_units(i).io.fu_types
    if (exe_units(i).supportedFuncUnits.fdiv) {
      val fdiv_issued = iss_valids(i) && iss_uops(i).fu_code_is(FU_FDV)
      fu_types = fu_types & RegNext(~Mux(fdiv_issued, FU_FDV, 0.U))
    }
    issue_unit.io.fu_types(i) := fu_types & ~Fill(FUC_SZ, vregister_read.io.rrd_stall.asUInt)

    require (exe_units(i).readsVrf)
  }

  // Wakeup
  for ((writeback, issue_wakeup) <- io.wakeups zip issue_unit.io.wakeup_ports) {
    issue_wakeup.valid          := writeback.valid
    issue_wakeup.bits.pdst      := writeback.bits.uop.pdst
    issue_wakeup.bits.poisoned  := false.B
    issue_wakeup.bits.uop       := writeback.bits.uop
  }
  issue_unit.io.pred_wakeup_port.valid  := false.B
  issue_unit.io.pred_wakeup_port.bits   := DontCare


  //-------------------------------------------------------------
  // **** Register Read Stage ****
  //-------------------------------------------------------------

  // Register Read <- Issue (rrd <- iss)
  for (w <- 0 until exe_units.numVrfReadPorts) {
    vregister_read.io.rf_read_ports(w) <> vregfile.io.read_ports(w)
  }

  for (w <- 0 until memWidth) {
    io.lsu_vrf_rport(w) <> vregfile.io.read_ports(exe_units.numVrfReadPorts + w)
  }
  vregister_read.io.prf_read_ports map { port => port.data := false.B }

  vregister_read.io.iss_valids <> iss_valids
  vregister_read.io.iss_uops := iss_uops

  vregister_read.io.brupdate := io.brupdate
  vregister_read.io.kill := io.flush_pipeline

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

  (io.to_fp zip exe_units.withFilter(_.hasAlu).map(_.io.fresp)).foreach {
    case (to_fp, vec) => to_fp <> vec
  }

  (io.to_int zip exe_units.withFilter(_.hasAlu).map(_.io.iresp)).foreach {
    case (to_int, vec) => to_int <> vec
  }


  val ll_wbarb = Module(new Arbiter(new ExeUnitResp(vLen), matWidth+1))
  ll_wbarb.io.in(0) <> io.ll_wports(0)
  ll_wbarb.io.in(0).bits.data := io.ll_wports(0).bits.data
  for(w <- 0 until matWidth) {
    ll_wbarb.io.in(w+1) <> io.fromMat(w)
  }

  // Cut up critical path by delaying the write by a cycle.
  // Wakeup signal is sent on cycle S0, write is now delayed until end of S1,
  // but Issue happens on S1 and RegRead doesn't happen until S2 so we're safe.
  vregfile.io.write_ports(0) := RegNext(WritePort(ll_wbarb.io.out, vpregSz, vLen, isVector, true, true))
  assert (ll_wbarb.io.in(0).ready) // never backpressure the memory unit.

  var w_cnt = 1
  for (i <- 1 until numVLdPorts) {
    vregfile.io.write_ports(w_cnt) := RegNext(WritePort(io.ll_wports(i), vpregSz, vLen, isVector, true))
    w_cnt += 1
  }
  for (eu <- exe_units) {
    val eu_vresp = WireInit(eu.io.vresp)
    val eu_vresp_uop = eu_vresp.bits.uop
    when(eu_vresp_uop.rt(RD, isReduceV)) {
      eu_vresp.bits.uop.v_eidx := 0.U
    }
    if (eu.writesVrf) {
      eu_vresp.valid := eu.io.vresp.valid && eu_vresp_uop.rf_wen
      eu.io.vresp.ready := true.B
      vregfile.io.write_ports(w_cnt) := WritePort(eu_vresp, vpregSz, vLen, isVector, true)
      when(eu_vresp.valid && !(eu_vresp_uop.is_rvv && eu_vresp_uop.ctrl.is_sta)) {
        assert(eu.io.vresp.bits.uop.rf_wen, "rf_wen must be high here")
        assert(eu.io.vresp.bits.uop.rt(RD, isVector), "wb type must be vector")
      }
      w_cnt += 1
    }
  }
  require (w_cnt == vregfile.io.write_ports.length)

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Commit Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  io.wakeups(0).valid := ll_wbarb.io.out.valid
  io.wakeups(0).bits := ll_wbarb.io.out.bits
  ll_wbarb.io.out.ready := true.B

  w_cnt = 1
  for (eu <- exe_units) {
    if (eu.writesVrf) {
      val exe_resp = eu.io.vresp
      val wb_uop = eu.io.vresp.bits.uop
      val wport = io.wakeups(w_cnt)
      wport.valid := exe_resp.valid && wb_uop.rf_wen
      wport.bits := exe_resp.bits

      w_cnt += 1
    }
  }

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
    (BoomCoreStringPrefix("===Vec Pipeline===") + "\n"
    + vregfile.toString
    + BoomCoreStringPrefix(
      "Num Wakeup Ports      : " + numWakeupPorts,
      "Num Bypass Ports      : " + exe_units.numTotalBypassPorts))
}
