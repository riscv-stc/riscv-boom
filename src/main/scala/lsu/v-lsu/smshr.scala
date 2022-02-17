package boom.vlsu

import chisel3._
import chisel3.util._
import freechips.rocketchip.tilelink.TLMessages._
import freechips.rocketchip.tilelink._
class SMSHR(ap: VLSUArchitecturalParams, idx: Int) extends VLSUModules(ap){
  val nLmshrs: Int = ap.nLmshrs
  val waitCycles = ap.storeWaitCycles
  val io = IO(new Bundle{
    val vStReq = Flipped(Valid(new VStRequest(ap)))
    val tlOutA = Decoupled(new TLBundleA(ap.l2BundleParams))
    val tlInD = Flipped(Decoupled(new TLBundleD(ap.l2BundleParams)))
    val status = Valid(new SMSHRStatus(ap))
    val fromVRF = Flipped(Valid(new VLSUReadVRFResp(ap)))
  })
  io.tlOutA.valid := false.B
  io.tlOutA.bits := 0.U.asTypeOf(new TLBundleA(ap.l2BundleParams))
  val sIdle :: sWaitMerge :: sWaitArb :: sWaitAck :: Nil = Enum(4)
  val state = RegInit(sIdle)
  val maskBuffer = RegInit(0.U(ap.cacheLineByteSize.W))
  val dataBuffer = RegInit(0.U(ap.cacheLineBits.W))
  val reg = RegInit(0.U.asTypeOf(Valid(new VStRequest(ap))))
  io.status.valid := state =/= sIdle
  io.status.bits.awaiting := state === sWaitMerge
  io.status.bits.addr := reg.bits.address
  assert(Mux(io.vStReq.valid, state === sIdle || state === sWaitMerge, true.B), "\nMust not allocate here when not idle or waitMerge.")
  val waitCount = RegInit(0.U(log2Ceil(waitCycles).W))
  val unitStrideFactory = Module(new UnitStrideStoreLineFactory(ap))
  unitStrideFactory.io.storeSnippet := io.vStReq.bits.regAccessCS.finishMaskSnippet
  unitStrideFactory.io.address := io.vStReq.bits.address
  unitStrideFactory.io.storeData := Mux(io.fromVRF.valid, io.fromVRF.bits.data, 0.U)
  unitStrideFactory.io.isHead := io.vStReq.bits.regAccessCS.snippetIsHead
  unitStrideFactory.io.isTail := io.vStReq.bits.regAccessCS.snippetIsTail
  val elementFactory = Module(new ElementStoreLineFactory(ap))
  elementFactory.io.storeSnippet := io.vStReq.bits.regAccessCS.finishMaskSnippet
  elementFactory.io.address := io.vStReq.bits.address
  elementFactory.io.storeData := Mux(io.fromVRF.valid, io.fromVRF.bits.data, 0.U)
  elementFactory.io.eew := io.vStReq.bits.style.dataEew
  val lineDataIncoming = Mux(io.vStReq.bits.style.isUnitStride, unitStrideFactory.io.lineData, elementFactory.io.lineData)
  val lineMaskIncoming = Mux(io.vStReq.bits.style.isUnitStride, unitStrideFactory.io.lineMask, elementFactory.io.lineMask)
  val dataMerger = Module(new LineDataMerger(ap))
  dataMerger.io.inData := lineDataIncoming
  dataMerger.io.inMask := lineMaskIncoming
  dataMerger.io.oldData := dataBuffer
  dataMerger.io.oldMask := maskBuffer

  when(state === sIdle && io.vStReq.valid){
    reg.valid := true.B
    reg.bits := io.vStReq.bits
    state := sWaitMerge
    dataBuffer := lineDataIncoming
    maskBuffer := lineMaskIncoming
  }.elsewhen(state === sWaitMerge){
    when(io.vStReq.valid){
      assert((io.vStReq.bits.address >> ap.offsetBits).asUInt() === (reg.bits.address >> ap.offsetBits).asUInt(),
        "\nLine addr must be same when merging.")
      dataBuffer := dataMerger.io.newData
      maskBuffer := dataMerger.io.newMask
    }
    waitCount := waitCount + 1.U
    when(waitCount === (waitCycles - 1).U){
      state := sWaitArb
    }
  }.elsewhen(state === sWaitArb){
    io.tlOutA.valid := true.B
    io.tlOutA.bits := ap.tlEdge.Put((nLmshrs + idx).U, lineAddress(reg.bits.address), log2Ceil(ap.cacheLineByteSize).U, dataBuffer, maskBuffer)._2
    when(io.tlOutA.fire()){
      state := sWaitAck
    }
  }.elsewhen(state === sWaitAck){
    when(io.tlInD.valid && (io.tlInD.bits.source === (nLmshrs + idx).U)){
      when(io.tlInD.bits.corrupt || io.tlInD.bits.denied){
        state := sWaitArb
      }.otherwise{
        state := sIdle
      }
    }
  }
  io.tlInD.ready := true.B
}
