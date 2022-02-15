package boom.vlsu

import chisel3._
import chisel3.util._
import boom.util.WrapInc

class VectorStoreAddressChecker(ap: VLSUArchitecturalParams) extends VLSUModules(ap){
  val io = IO(new Bundle{
    val smshrStatus = Flipped(Vec(ap.nSmshrs, Valid(new SMSHRStatus(ap))))
    val reqIncoming = Flipped(DecoupledIO(new VStRequest(ap)))
    val mshrAllocate = Vec(ap.nSmshrs, Valid(new VStRequest(ap)))
    val vtlbResp = Flipped(Valid(new VTLBResp(ap)))
    val storeResp = Valid(new VStRequest(ap))
  })
  io.mshrAllocate := 0.U.asTypeOf(Vec(ap.nSmshrs, Valid(new VStRequest(ap))))
  io.storeResp := 0.U.asTypeOf(Valid(new VStRequest(ap)))
  io.reqIncoming.ready := false.B
  val reg = RegInit(0.U.asTypeOf(Valid(new VStRequest(ap))))
  val allocate = WireInit(0.U.asTypeOf(Vec(ap.nSmshrs, Valid(new VStRequest(ap)))))
  io.mshrAllocate := allocate
  /** Indicates if we can merge store request. */
  val addrHits = VecInit(io.smshrStatus.map(r => r.valid && r.bits.awaiting &&
    (r.bits.addr >> ap.offsetBits).asUInt() === (io.reqIncoming.bits.address >> ap.offsetBits).asUInt())).asUInt()
  assert(PopCount(addrHits) <= 1.U, "more than one mshr merging same address.")
  val addrHit = addrHits.orR()
  val hitMshrIdx = OHToUInt(addrHits)
  val idleMshrs = VecInit(io.smshrStatus.map(r => !r.valid)).asUInt()
  val idleMshr = idleMshrs.orR()
  val mshrAllocateIdx: UInt = OHToUInt(FindFirstOne(idleMshrs, ap.nSmshrs))
  val awaitingMshrs = VecInit(io.smshrStatus.map(r => r.valid && r.bits.awaiting)).asUInt()
  val mshrEnough = PopCount(idleMshrs) > 1.U
  val canAllocate = addrHit || mshrEnough
  val blockBuffer = WireInit(false.B)
  val canProcess = (io.reqIncoming.valid && io.reqIncoming.bits.committed) && canAllocate && !blockBuffer
  val hitsS2 = VecInit(io.smshrStatus.map(r => r.valid && r.bits.awaiting &&
    (r.bits.addr >> ap.offsetBits).asUInt() === (reg.bits.address >> ap.offsetBits).asUInt())).asUInt()
  val hitS2 = hitsS2.orR()
  val hitIdxS2 = OHToUInt(hitsS2)
  val allocatedIdx = Mux(hitS2, hitIdxS2, mshrAllocateIdx)
  when(reg.valid){
    when(hitS2 || idleMshr){
      allocate(allocatedIdx).valid := true.B
      allocate(allocatedIdx).bits := reg.bits
      reg.valid := false.B
    }.otherwise{
      blockBuffer := true.B
    }
  }
  when(canProcess){
    reg.valid := true.B
    reg.bits := io.reqIncoming.bits
    io.storeResp.valid := io.reqIncoming.valid
    io.storeResp.bits := io.reqIncoming.bits
    io.reqIncoming.ready := true.B
  }.otherwise{
    io.reqIncoming.ready := false.B
  }
}