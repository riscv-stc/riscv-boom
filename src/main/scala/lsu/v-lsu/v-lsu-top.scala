package boom.vlsu

import chisel3._
import chisel3.util._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.rocket
import boom.common._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util.DontTouch

class VLSU(generalParams: VLSUGeneralParams,
           boomParams: BoomCoreParams
          )(implicit p: Parameters) extends LazyModule(){
  val vmshrs = generalParams.nLmshrs + generalParams.nSmshrs
  val nL2BeatBytes = generalParams.nL2DataBeatBytes
  val node = TLClientNode(Seq(TLMasterPortParameters.v1(
    Seq(TLMasterParameters.v1(
      name = s"l1vlsu",
      sourceId = IdRange(0, vmshrs),
      supportsProbe = TransferSizes.none
    ))
  )))(ValName(s"fromL1vlsu"))
  lazy val module = new VLSUImp(this, generalParams, boomParams)

}

/** This module is the TOP of VLSU. */
class VLSUImp(outer: VLSU,
              generalParams: VLSUGeneralParams,
              boomParams: BoomCoreParams) extends LazyModuleImp(outer)
  with DontTouch{
  val ap: VLSUArchitecturalParams =
    new VLSUArchitecturalParams(generalParams,  boomParams, outer.node.edges.out(0))
  val (tlOut, _) = outer.node.out(0)
  val io = IO(new VLSUTopBundle(ap))
  //________________________________________________________________________________//
  //--------------------------------Address-Gen-------------------------------------//
  val vldqHandler = Module(new VLdQueueHandler(ap))
  vldqHandler.io.vuopDis <> io.fromDis.vuopDis
  vldqHandler.io.vuopRR <> io.fromRr.vuop
  io.toDis.vLdQFull := vldqHandler.io.vLdQFull
  io.toDis.disVLdQIdx := vldqHandler.io.disVLdQIdx
  io.ldToRob <> vldqHandler.io.toRob
  vldqHandler.io.fromRob := io.fromRob
  io.wakeUpVReg <> vldqHandler.io.wakeUp
  vldqHandler.io.vrfReadResp := io.fromVrf.readResp

  val vstqHandler = Module(new VStQueueHandler(ap))
  vstqHandler.io.vuopDis <> io.fromDis.vuopDis
  vstqHandler.io.vuopRR <> io.fromRr.vuop
  io.toDis.vStQFull := vstqHandler.io.vStQFull
  io.toDis.disVStQIdx := vstqHandler.io.disVStQIdx
  io.stToRob <> vstqHandler.io.toRob
  vstqHandler.io.fromRob := io.fromRob
  //________________________________________________________________________________//
  //-------------------------------Address-Read-------------------------------------//
  val loadReqBuffer = Module(new LoadRequestBuffer(ap))
  loadReqBuffer.io.reqIncoming <> vldqHandler.io.vldReq
  loadReqBuffer.io.fromRob := io.fromRob

  val storeReqBuffer = Module(new StoreRequestBuffer(ap))
  storeReqBuffer.io.reqIncoming <> vstqHandler.io.vstReq
  storeReqBuffer.io.fromRob := io.fromRob
  // store req always win because its req also goes into addr-checker.
  io.toVrf.readReq.valid := storeReqBuffer.io.vrfReadReq.valid || vldqHandler.io.vrfReadReq.valid
  io.toVrf.readReq.bits := Mux(storeReqBuffer.io.vrfReadReq.valid, storeReqBuffer.io.vrfReadReq.bits, vldqHandler.io.vrfReadReq.bits)
  vldqHandler.io.vrfReadReq.ready := !storeReqBuffer.io.vrfReadReq.valid

  val vtlb = Module(new VTLB(ap))
  vtlb.io.req(0) <> loadReqBuffer.io.vtlbReq
  vtlb.io.req(1) <> storeReqBuffer.io.vtlbReq
  loadReqBuffer.io.vtlbResp := vtlb.io.resp(0)
  storeReqBuffer.io.vtlbResp := vtlb.io.resp(1)
  vstqHandler.io.fromVTLB := vtlb.io.resp(1)
  //________________________________________________________________________________//
  //-------------------------------Address-Match------------------------------------//
  val loadAddrChecker = Module(new VectorLoadAddressChecker(ap))
  loadAddrChecker.io.vtlbResp <> vtlb.io.resp(0)
  loadAddrChecker.io.reqIncoming <> loadReqBuffer.io.reqOutgoing

  val storeAddrChecker = Module(new VectorStoreAddressChecker(ap))
  storeAddrChecker.io.vtlbResp <> vtlb.io.resp(0)
  storeAddrChecker.io.reqIncoming <> storeReqBuffer.io.reqOutgoing
  storeReqBuffer.io.dataAccomplished := storeAddrChecker.io.storeResp
  vstqHandler.io.dataAccomplished := storeAddrChecker.io.storeResp
  val mshrFile = Module(new MSHRFile(ap))
  mshrFile.io.lmshrAllocateReq <> loadAddrChecker.io.mshrAllocate
  mshrFile.io.smshrAllocateReq <> storeAddrChecker.io.mshrAllocate
  tlOut.a <> mshrFile.io.tlOutA
  mshrFile.io.tlInD <> tlOut.d
  loadAddrChecker.io.lmshrStatus <> mshrFile.io.lmshrStatus
  storeAddrChecker.io.smshrStatus <> mshrFile.io.smshrStatus
  mshrFile.io.vrfReadResp := io.fromVrf.readResp
  //________________________________________________________________________________//
  //-------------------------------Data-Write-Back----------------------------------//
  val wbCtrl = Module(new WriteBackController(ap))
  wbCtrl.io <> DontCare
  wbCtrl.io.wbReqIncoming <> mshrFile.io.writeBackReq
  loadReqBuffer.io.wbBypassQuery <> wbCtrl.io.wbBypassQuery
  wbCtrl.io.wbBypassResp <> loadReqBuffer.io.wbBypassResp
  loadReqBuffer.io.reqWBDone <> wbCtrl.io.reqWBDone
  vldqHandler.io.finishAck <> wbCtrl.io.finishAck
  io.toVrf.write <> wbCtrl.io.toVRF
  // write stale data for untouched load.
  wbCtrl.io.wbReqFromQEntry <> vldqHandler.io.vrfWriteReq

}
