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

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.{Parameters}
import freechips.rocketchip.rocket.MStatus
import freechips.rocketchip.tile.FPConstants

import boom.exu.FUConstants._
import boom.common._
import boom.common.MicroOpcodes._
import boom.util.{BoomCoreStringPrefix}

/**
 * Top level datapath that wraps the floating point issue window, regfile, and arithmetic units.
 */
class VecPipeline(implicit p: Parameters) extends BoomModule
{
  val vIssueParams = issueParams.find(_.iqType == IQT_VEC.litValue).get
  val dispatchWidth = vIssueParams.dispatchWidth
  val issueWidth = vIssueParams.issueWidth
  val numWakeupPorts = issueWidth + memWidth // internal wakeups

  val io = IO(new Bundle {
    val brupdate         = Input(new BrUpdateInfo())
    val flush_pipeline   = Input(Bool())
    val fcsr_rm          = Input(UInt(width=FPConstants.RM_SZ.W))
    val status           = Input(new MStatus())

    val dis_uops         = Vec(dispatchWidth, Flipped(Decoupled(new MicroOp)))

    val ll_wports        = Flipped(Vec(memWidth, Decoupled(new ExeUnitResp(eLen))))
    val from_int         = Flipped(Decoupled(new ExeUnitResp(eLen)))
    val from_fp          = Flipped(Decoupled(new ExeUnitResp(eLen)))
//  val to_sdq           = Vec(memWidth, Decoupled(new ExeUnitResp(eLen)))
    val to_int           = Decoupled(new ExeUnitResp(eLen))
    val to_fp            = Decoupled(new ExeUnitResp(eLen))

    val wakeups          = Vec(numWakeupPorts, Valid(new ExeUnitResp(eLen))) // wakeup issue_units for mem, int and fp

    val debug_tsc_reg    = Input(UInt(width=xLen.W))
    val debug_wb_wdata   = Output(Vec(numWakeupPorts, UInt((eLen).W)))
  })

  //**********************************
  // construct all of the modules

  val exe_units      = new boom.exu.ExecutionUnits(fpu=false, vector=true)
  val issue_unit     = Module(new IssueUnitCollapsing(
                         vIssueParams,
                         numWakeupPorts))
  issue_unit.suggestName("v_issue_unit")
  val vregfile       = Module(new RegisterFileSynthesizable(numVecPhysRegs,
                         exe_units.numVrfReadPorts,
                         exe_units.numVrfWritePorts + memWidth,
                         eLen,
                         // No bypassing for any VEC units, + memWidth for ll_wb
                         Seq.fill(exe_units.numVrfWritePorts + memWidth){ false }))
  val vregister_read = Module(new RegisterRead(
                         issueWidth,
                         exe_units.withFilter(_.readsVrf).map(_.supportedFuncUnits),
                         exe_units.numVrfReadPorts,
                         exe_units.withFilter(_.readsVrf).map(x => 3),
                         0, // No bypass for VEC
                         0,
                         eLen))

  require (exe_units.count(_.readsVrf) == issueWidth)
  require (exe_units.numVrfWritePorts + memWidth == numWakeupPorts)

  //*************************************************************
  // Issue window logic

  val iss_valids = Wire(Vec(exe_units.numVrfReaders, Bool()))
  val iss_uops   = Wire(Vec(exe_units.numVrfReaders, new MicroOp()))

  issue_unit.io.tsc_reg := io.debug_tsc_reg
  issue_unit.io.brupdate := io.brupdate
  issue_unit.io.flush_pipeline := io.flush_pipeline
  // Don't support ld-hit speculation to VEC window.
  for (w <- 0 until memWidth) {
    issue_unit.io.spec_ld_wakeup(w).valid := false.B
    issue_unit.io.spec_ld_wakeup(w).bits := 0.U
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
  for (i <- 0 until issueWidth) {
    iss_valids(i) := issue_unit.io.iss_valids(i)
    iss_uops(i) := issue_unit.io.iss_uops(i)

    var fu_types = exe_units(i).io.fu_types
    if (exe_units(i).supportedFuncUnits.fdiv) {
      val fdiv_issued = iss_valids(i) && iss_uops(i).fu_code_is(FU_FDV)
      fu_types = fu_types & RegNext(~Mux(fdiv_issued, FU_FDV, 0.U))
    }
    issue_unit.io.fu_types(i) := fu_types

    require (exe_units(i).readsVrf)
  }

  // Wakeup
  for ((writeback, issue_wakeup) <- io.wakeups zip issue_unit.io.wakeup_ports) {
    issue_wakeup.valid := writeback.valid
    issue_wakeup.bits.pdst  := writeback.bits.uop.pdst
    issue_wakeup.bits.poisoned := false.B
  }
  issue_unit.io.pred_wakeup_port.valid := false.B
  issue_unit.io.pred_wakeup_port.bits := DontCare

  //-------------------------------------------------------------
  // **** Register Read Stage ****
  //-------------------------------------------------------------

  // Register Read <- Issue (rrd <- iss)
  vregister_read.io.rf_read_ports <> vregfile.io.read_ports
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

  val ll_wbarb = Module(new Arbiter(new ExeUnitResp(eLen), 3))


  ll_wbarb.io.in(0) <> io.ll_wports(0)
  ll_wbarb.io.in(0).bits.data := io.ll_wports(0).bits.data

  ll_wbarb.io.in(1) <> io.from_int
  ll_wbarb.io.in(2) <> io.from_fp


  // Cut up critical path by delaying the write by a cycle.
  // Wakeup signal is sent on cycle S0, write is now delayed until end of S1,
  // but Issue happens on S1 and RegRead doesn't happen until S2 so we're safe.
  vregfile.io.write_ports(0) := RegNext(WritePort(ll_wbarb.io.out, fpregSz, eLen, RT_VEC))

  assert (ll_wbarb.io.in(0).ready) // never backpressure the memory unit.
  when (io.from_int.valid) { assert (io.from_int.bits.uop.rf_wen && io.from_int.bits.uop.dst_rtype === RT_VEC) }
  when (io.from_fp.valid)  { assert (io.from_fp.bits.uop.rf_wen  && io.from_fp.bits.uop.dst_rtype  === RT_VEC) }

  var w_cnt = 1
  for (i <- 1 until memWidth) {
    vregfile.io.write_ports(w_cnt) := RegNext(WritePort(io.ll_wports(i), fpregSz, eLen, RT_VEC))
    w_cnt += 1
  }
  for (eu <- exe_units) {
    if (eu.writesVrf) {
      vregfile.io.write_ports(w_cnt).valid     := eu.io.vresp.valid && eu.io.vresp.bits.uop.rf_wen
      vregfile.io.write_ports(w_cnt).bits.addr := eu.io.vresp.bits.uop.pdst
      vregfile.io.write_ports(w_cnt).bits.data := eu.io.vresp.bits.data
      eu.io.vresp.ready                        := true.B
      when (eu.io.vresp.valid) {
        //assert(eu.io.vresp.ready, "No backpressuring the Vec EUs")
        assert(eu.io.vresp.bits.uop.rf_wen, "rf_wen must be high here")
        assert(eu.io.vresp.bits.uop.dst_rtype === RT_VEC, "wb type must be FLT for fpu")
      }
      w_cnt += 1
    }
  }
  require (w_cnt == vregfile.io.write_ports.length)

//io.to_sdq.map(_.valid := false.B) // FIXME
//io.to_sdq.map(_.bits  := DontCare)
  io.to_int.valid := false.B // FIXME
  io.to_int.bits  := DontCare
  io.to_fp.valid := false.B // FIXME
  io.to_fp.bits  := DontCare

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Commit Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  io.wakeups(0).valid := ll_wbarb.io.out.valid
  io.wakeups(0).bits := ll_wbarb.io.out.bits
  ll_wbarb.io.out.ready := true.B

  w_cnt = 1
  for (i <- 1 until memWidth) {
    io.wakeups(w_cnt) := io.ll_wports(i)
    io.wakeups(w_cnt).bits.data := io.ll_wports(i).bits.data
    w_cnt += 1
  }
  for (eu <- exe_units) {
    if (eu.writesVrf) {
      val exe_resp = eu.io.vresp
      val wb_uop = eu.io.vresp.bits.uop
      val wport = io.wakeups(w_cnt)
      wport.valid := exe_resp.valid && wb_uop.dst_rtype === RT_VEC
      wport.bits := exe_resp.bits

      w_cnt += 1

      //assert(!(exe_resp.valid && wb_uop.uses_ldq))
      //assert(!(exe_resp.valid && wb_uop.uses_stq))
      //assert(!(exe_resp.valid && wb_uop.is_amo))
    }
  }

  for ((wdata, wakeup) <- io.debug_wb_wdata zip io.wakeups) {
    wdata := wakeup.bits.data
  }

  //exe_units.map(if (_.hasFcsr) _.io.fcsr_rm := io.fcsr_rm)
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
