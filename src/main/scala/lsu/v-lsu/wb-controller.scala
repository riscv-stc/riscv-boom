package boom.vlsu

import chisel3._
import chisel3.util._
import freechips.rocketchip.tilelink.TLMessages._
import freechips.rocketchip.tilelink._
class WriteBackController(ap: VLSUArchitecturalParams) extends VLSUModules(ap){
  val io = IO(new Bundle {
    val wbReqIncoming: DecoupledIO[VLdWriteBackRequest] = Flipped(Decoupled(new VLdWriteBackRequest(ap)))
    val toVRF: ValidIO[VLSUWriteVRFReq] = Valid(new VLSUWriteVRFReq(ap))
    /** Query req buffer with memory addr that we gonna write back. */
    val wbBypassQuery: ValidIO[UInt] = Valid(UInt(ap.coreMaxAddrBits.W))
    /** Tell vldq handler which entry has partly finished. */
    val finishAck: ValidIO[VLdRequest] = Valid(new VLdRequest(ap))
    /** Tell req buffer which entry has finished. */
    val reqWBDone = Valid(UInt(ap.nVLdReqBufferIdxBits.W))

    /** Data path for queue entry writing stale data for undisturbed load. */
    val wbReqFromQEntry = Flipped(Decoupled(new VLSUWriteVRFReq(ap)))

    /** Vec input is to detect order fail. */
    val wbBypassResp: Vec[ValidIO[VLdRequest]] = Flipped(Vec(ap.nVLdReqBuffEntries, Valid(new VLdRequest(ap))))
  })

  val sIdle :: sNormalWB :: sBypassWB :: Nil = Enum(3)
  val state: UInt = RegInit(sIdle)
  io.toVRF := 0.U.asTypeOf(Valid(new VLSUWriteVRFReq(ap)))
  io.wbBypassQuery := 0.U.asTypeOf(Valid(UInt(ap.coreMaxAddrBits.W)))
  io.reqWBDone := 0.U.asTypeOf(Valid(UInt(ap.nVLdReqBufferIdxBits.W)))
  io.finishAck := 0.U.asTypeOf(Valid(new VLdRequest(ap)))
  val reg = RegInit(0.U.asTypeOf(Valid(new VLdWriteBackRequest(ap))))

  when(io.wbReqIncoming.fire()){
    reg.valid := true.B
    reg.bits := io.wbReqIncoming.bits
    state := sNormalWB
  }

  // Calculation of data to vrf here.
  val awaitingVLDReq = WireInit(0.U.asTypeOf(new VLdRequest(ap)))
  val awaitingVRFReq = WireInit(0.U.asTypeOf(new VLSUWriteVRFReq(ap)))
  val dataStretcherUnitStride = Module(new UnitStrideDataStretcher(ap))
  dataStretcherUnitStride.io.lineDataIn := reg.bits.data
  dataStretcherUnitStride.io.reqWB := awaitingVLDReq

  val dataStretcherElemental = Module(new ElementDataStretcher(ap))
  dataStretcherElemental.io.lineDataIn := reg.bits.data
  dataStretcherElemental.io.reqWB := awaitingVLDReq
  val bypassRespVlds: UInt = VecInit(io.wbBypassResp.map(_.valid)).asUInt()
  val hasBypassChance: Bool = bypassRespVlds.orR()
  val bypassWBIdx: UInt = OHToUInt(FindFirstOne(bypassRespVlds, ap.nVLdReqBuffEntries))

  awaitingVRFReq := Mux(awaitingVLDReq.style.isUnitStride,
    dataStretcherUnitStride.io.vrfWriteReq, dataStretcherElemental.io.vrfWriteReq)
  when(state === sNormalWB){
    //Write back to VRF.
    io.toVRF.valid := true.B
    awaitingVLDReq := reg.bits.req
    //Tell vldq handler we are finished.
    io.finishAck.valid := true.B
    io.finishAck.bits := reg.bits.req
    //Tell req-buffer we are finished.
    io.reqWBDone.valid := true.B
    io.reqWBDone.bits := reg.bits.req.reqBufferIdx
    state := sBypassWB
  }.elsewhen(state === sBypassWB){
    // Send query to req-buff handler, resp returns at same cycle.
    io.wbBypassQuery.valid := true.B
    io.wbBypassQuery.bits := reg.bits.req.address
    io.toVRF.valid := hasBypassChance
    awaitingVLDReq := io.wbBypassResp(bypassWBIdx).bits
    io.finishAck.valid := hasBypassChance
    io.finishAck.bits := io.wbBypassResp(bypassWBIdx).bits
    io.reqWBDone.valid := hasBypassChance
    io.reqWBDone.bits := bypassWBIdx
    when(!hasBypassChance){
      state := sIdle
    }
  }
  io.wbReqIncoming.ready := (state === sIdle)
  val idling = state === sIdle
  io.wbReqFromQEntry.ready := idling
  when(io.wbReqFromQEntry.fire()){
    io.toVRF.valid := true.B
  }
  io.toVRF.bits := Mux(io.wbReqFromQEntry.fire(), io.wbReqFromQEntry.bits, awaitingVRFReq)

}