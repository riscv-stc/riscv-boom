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
  //val vmxIssueParams = issueParams.find(_.iqType == IQT_VMX.litValue).get
  //require(vecIssueParams.dispatchWidth == vmxIssueParams.dispatchWidth)
  val dispatchWidth = vecIssueParams.dispatchWidth
  val numWakeupPorts = vecWidth + memWidth // internal wakeups

  val io = IO(new Bundle {
    val brupdate         = Input(new BrUpdateInfo())
    val flush_pipeline   = Input(Bool())
    val fcsr_rm          = Input(UInt(width=FPConstants.RM_SZ.W))
    val vxrm             = Input(UInt(2.W))
    val status           = Input(new MStatus())

    val vec_dis_uops     = Vec(dispatchWidth, Flipped(Decoupled(new MicroOp)))
    //val vmx_dis_uops     = Vec(dispatchWidth, Flipped(Decoupled(new MicroOp)))
    val vbusy_status     = Input(UInt(numVecPhysRegs.W))

    val ll_wports        = Flipped(Decoupled(new ExeUnitResp(vLen)))
    val from_int         = Flipped(Decoupled(new ExeUnitResp(eLen)))
    val from_fp          = Flipped(Decoupled(new ExeUnitResp(eLen)))
//  val to_int_iss       = Decoupled(new ExeUnitResp(eLen))
//  val to_sdq           = Decoupled(new ExeUnitResp(eLen))
    val to_int           = Vec(vecWidth, Decoupled(new ExeUnitResp(eLen)))
    val to_fp            = Vec(vecWidth, Decoupled(new ExeUnitResp(eLen)))
    //val vmupdate         = Output(Vec(1, Valid(new MicroOp)))
    val intupdate        = Input(Vec(intWidth, Valid(new ExeUnitResp(eLen))))
    val fpupdate         = Input(Vec(fpWidth, Valid(new ExeUnitResp(eLen))))
    val lsu_vrf_rport    = new RegisterFileReadPortIO(vpregSz, vLen)
    //val lsu_vrf_wbk      = Flipped(new ExeUnitResp(vLen))

    val vl_wakeup        = Input(Valid(new VlWakeupResp()))
    val wakeups          = Vec(numWakeupPorts, Valid(new ExeUnitResp(eLen))) // wakeup issue_units for mem, int and fp

    val debug_tsc_reg    = Input(UInt(width=xLen.W))
    val debug_wb_wdata   = Output(Vec(numWakeupPorts, UInt((eLen).W)))

    val perf = Output(new Bundle {
      val vec_iss_valids      = Vec(vecIssueParams.issueWidth, Bool())
      val vec_req_valids      = Vec(vecIssueParams.issueWidth, Bool())
      val vec_iss_slots_empty = Bool()
      val vec_iss_slots_full  = Bool()
      //val vmx_iss_valids      = Vec(vmxIssueParams.issueWidth, Bool())
      //val vmx_req_valids      = Vec(vmxIssueParams.issueWidth, Bool())
      //val vmx_iss_slots_empty = Bool()
      //val vmx_iss_slots_full  = Bool()
      val div_busy            = Vec(vecIssueParams.issueWidth, Bool())
      val fdiv_busy           = Vec(vecIssueParams.issueWidth, Bool())
    })
  })

  //**********************************
  // construct all of the modules

  val exe_units      = new boom.exu.ExecutionUnits(fpu=false, vector=true)
  val vec_issue_unit = Module(new IssueUnitCollapsing(
                         vecIssueParams,
                         numWakeupPorts, vector = true))
  vec_issue_unit.suggestName("vec_issue_unit")
  //val vmx_issue_unit = Module(new IssueUnitCollapsing(
  //                       vmxIssueParams,
  //                       numWakeupPorts, vector = true))
  //vmx_issue_unit.suggestName("vmx_issue_unit")
  val viu = Seq(vec_issue_unit) //vmx_issue_unit
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
                         exe_units.withFilter(_.readsVrf).map(x => if (x.hasVMX) 2 else 4),
                         0, // No bypass for VEC
                         0,
                         vLen, float = false, vector = true))

  require (exe_units.count(_.readsVrf) == vecWidth)
  require (exe_units.numVrfWritePorts + memWidth == numWakeupPorts)
  require (vecWidth >= memWidth)

  //*************************************************************
  // Issue window logic

  val iss_valids = Wire(Vec(exe_units.numVrfReaders, Bool()))
  val iss_uops   = Wire(Vec(exe_units.numVrfReaders, new MicroOp()))

  viu.map(_.io.vbusy_status := io.vbusy_status)
  viu.map(_.io.tsc_reg := io.debug_tsc_reg)
  viu.map(_.io.brupdate := io.brupdate)
  viu.map(_.io.flush_pipeline := io.flush_pipeline)
  viu.map(_.io.intupdate := io.intupdate)
  viu.map(_.io.fpupdate  := io.fpupdate)
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
    vec_issue_unit.io.dis_uops(w) <> io.vec_dis_uops(w)
    //vmx_issue_unit.io.dis_uops(w) <> io.vmx_dis_uops(w)
  }

  //-------------------------------------------------------------
  // **** Issue Stage ****
  //-------------------------------------------------------------
  io.perf.vec_iss_valids := vec_issue_unit.io.iss_valids
  io.perf.vec_req_valids := (0 until vecWidth).map(i => exe_units(i).io.req.valid)
  io.perf.vec_iss_slots_empty := vec_issue_unit.io.perf.empty
  io.perf.vec_iss_slots_full  := vec_issue_unit.io.perf.full
  io.perf.div_busy  := (0 until vecWidth).map(i => if(exe_units(i).hasDiv)  exe_units(i).io.perf.div_busy  else false.B)
  io.perf.fdiv_busy := (0 until vecWidth).map(i => if(exe_units(i).hasFdiv) exe_units(i).io.perf.fdiv_busy else false.B)

  //io.perf.vmx_iss_valids := vmx_issue_unit.io.iss_valids
  //io.perf.vmx_req_valids := (vecWidth until vecWidth+1).map(i => exe_units(i).io.req.valid)
  //io.perf.vmx_iss_slots_empty := vmx_issue_unit.io.perf.empty
  //io.perf.vmx_iss_slots_full  := vmx_issue_unit.io.perf.full

  // Output (Issue)
  for (i <- 0 until vecWidth) {
    iss_valids(i) := vec_issue_unit.io.iss_valids(i)
    iss_uops(i) := vec_issue_unit.io.iss_uops(i)

    var fu_types = exe_units(i).io.fu_types
    if (exe_units(i).supportedFuncUnits.fdiv) {
      val fdiv_issued = iss_valids(i) && iss_uops(i).fu_code_is(FU_FDV)
      fu_types = fu_types & RegNext(~Mux(fdiv_issued, FU_FDV, 0.U))
    }
    vec_issue_unit.io.fu_types(i) := fu_types & ~Fill(FUC_SZ, vregister_read.io.rrd_stall.asUInt)

    require (exe_units(i).readsVrf)
  }
  //iss_valids(vecWidth) := vmx_issue_unit.io.iss_valids(0)
  //iss_uops(vecWidth)   := vmx_issue_unit.io.iss_uops(0)
  //vmx_issue_unit.io.fu_types(0) := exe_units.vmx_unit.io.fu_types

  // Wakeup
  viu.map(iu => {
    for ((writeback, issue_wakeup) <- io.wakeups zip iu.io.wakeup_ports) {
      issue_wakeup.valid          := writeback.valid
      issue_wakeup.bits.pdst      := writeback.bits.uop.pdst
      issue_wakeup.bits.poisoned  := false.B
      issue_wakeup.bits.uop       := writeback.bits.uop
    }
    iu.io.pred_wakeup_port.valid  := false.B
    iu.io.pred_wakeup_port.bits   := DontCare
  })

  //-------------------------------------------------------------
  // **** Register Read Stage ****
  //-------------------------------------------------------------

  // Register Read <- Issue (rrd <- iss)
  for (w <- 0 until exe_units.numVrfReadPorts) {
    vregister_read.io.rf_read_ports(w) <> vregfile.io.read_ports(w)
  }
  io.lsu_vrf_rport <> vregfile.io.read_ports(exe_units.numVrfReadPorts)

  vregister_read.io.prf_read_ports map { port => port.data := false.B }

  vregister_read.io.iss_valids <> iss_valids
  vregister_read.io.iss_uops := iss_uops

  vregister_read.io.brupdate := io.brupdate
  vregister_read.io.kill := io.flush_pipeline
  //io.vmupdate := vregister_read.io.vmupdate

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

  val ll_wbarb = Module(new Arbiter(new ExeUnitResp(vLen), 3))


  ll_wbarb.io.in(0) <> io.ll_wports
  ll_wbarb.io.in(0).bits.data := io.ll_wports.bits.data

  ll_wbarb.io.in(1) <> io.from_int
  ll_wbarb.io.in(1).bits.data := io.from_int.bits.data

  ll_wbarb.io.in(2) <> io.from_fp
  ll_wbarb.io.in(2).bits.data := io.from_fp.bits.data


  // Cut up critical path by delaying the write by a cycle.
  // Wakeup signal is sent on cycle S0, write is now delayed until end of S1,
  // but Issue happens on S1 and RegRead doesn't happen until S2 so we're safe.
  vregfile.io.write_ports(0) := RegNext(WritePort(ll_wbarb.io.out, vpregSz, vLen, isVector, true, true))

  assert (ll_wbarb.io.in(0).ready) // never backpressure the memory unit.
  when (io.from_int.valid) { assert (io.from_int.bits.uop.rf_wen && io.from_int.bits.uop.rt(RD, isVector)) }
  when (io.from_fp.valid)  { assert (io.from_fp.bits.uop.rf_wen  && io.from_fp.bits.uop.rt(RD, isVector)) }

  var w_cnt = 1
  for (i <- 1 until memWidth) {
    vregfile.io.write_ports(w_cnt) := RegNext(WritePort(io.ll_wports, vpregSz, vLen, isVector, true, true))
    w_cnt += 1
  }
  for (eu <- exe_units) {
    val eu_vresp = WireInit(eu.io.vresp)
    val eu_vresp_uop = eu_vresp.bits.uop
    when (eu_vresp_uop.rt(RD, isReduceV)) {
      eu_vresp.bits.uop.v_eidx := 0.U
    }
    if (eu.writesVrf) {
      //if (eu.hasVMX) {
        //eu_vresp.valid    := eu.io.vresp.valid && eu_vresp_uop.rf_wen && !eu_vresp_uop.ctrl.is_sta
        //eu.io.vresp.ready := Mux(eu_vresp_uop.ctrl.is_sta, io.to_sdq.ready, true.B)
        //when (eu.io.vresp.valid) { assert(eu_vresp_uop.is_rvv) }
      //} else {
        eu_vresp.valid    := eu.io.vresp.valid && eu_vresp_uop.rf_wen
        eu.io.vresp.ready := true.B
      //}
      vregfile.io.write_ports(w_cnt) := WritePort(eu_vresp, vpregSz, vLen, isVector, true)
      when (eu_vresp.valid && !(eu_vresp_uop.is_rvv && eu_vresp_uop.ctrl.is_sta)) {
        //assert(eu.io.vresp.ready, "No backpressuring the Vec EUs")
        assert(eu.io.vresp.bits.uop.rf_wen, "rf_wen must be high here")
        assert(eu.io.vresp.bits.uop.rt(RD, isVector), "wb type must be vector")
      }
      w_cnt += 1
    }
  }
  require (w_cnt == vregfile.io.write_ports.length)

  //val vmx_unit = exe_units.vmx_unit
  //val vmx_resp = vmx_unit.io.vresp
  //val vmx_resp_uop = vmx_unit.io.vresp.bits.uop

  //io.to_sdq.valid := vmx_resp.valid && vmx_resp_uop.is_rvv && vmx_resp_uop.ctrl.is_sta
  //io.to_sdq.bits  := vmx_resp.bits
  //io.to_sdq.bits.data := VDataSel(vmx_resp.bits.data, vmx_resp_uop.vd_eew, vmx_resp_uop.v_eidx, vLen, eLen)
  //vmx_resp.ready := Mux(io.to_sdq.valid, io.to_sdq.ready, true.B)
  //io.to_int.valid := false.B
  //io.to_int.bits  := DontCare
  //io.to_fp.valid := false.B
  //io.to_fp.bits  := DontCare

  //io.to_int_iss       = Decoupled(new ExeUnitResp(eLen))

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Commit Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  io.wakeups(0).valid := ll_wbarb.io.out.valid
  io.wakeups(0).bits := ll_wbarb.io.out.bits
  ll_wbarb.io.out.ready := true.B

  w_cnt = 1
  //for (i <- 1 until memWidth) {
  //io.wakeups(w_cnt) := io.ll_wports //(i)
  //io.wakeups(w_cnt).bits.data := io.ll_wports.bits.data //(i)
  //w_cnt += 1
  //}
  for (eu <- exe_units) {
    if (eu.writesVrf) {
      val exe_resp = eu.io.vresp
      val wb_uop = eu.io.vresp.bits.uop
      val wport = io.wakeups(w_cnt)
      //if (eu.hasVMX) {
        //wport.valid := exe_resp.valid && wb_uop.rf_wen && !(wb_uop.is_rvv && wb_uop.ctrl.is_sta)
      //} else {
        wport.valid := exe_resp.valid && wb_uop.rf_wen
      //}
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
