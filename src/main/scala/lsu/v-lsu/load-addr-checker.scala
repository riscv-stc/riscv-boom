package boom.vlsu

import chisel3._
import chisel3.util._
import boom.util.WrapInc
/** Check requesting addr among lmshr and scalar dcache.
 * If miss, allocate a mshr.
 * If hit lmshr, do nothing.
 * If hit scalar dcache, pass data to wb-controller. */
class VectorLoadAddressChecker(ap: VLSUArchitecturalParams) extends VLSUModules(ap){
  val io = IO(new Bundle {
    /** Receive processing address from lmshrs */
    val lmshrStatus: Vec[ValidIO[UInt]] = Flipped(Vec(ap.nLmshrs, Valid(UInt(ap.coreMaxAddrBits.W))))
    /** requests from req-buffer winner. */
    val reqIncoming: DecoupledIO[VLdRequest] = Flipped(DecoupledIO(new VLdRequest(ap)))
    /** allocate a idle mshr if not hit any. */
    val mshrAllocate: Vec[ValidIO[VLdRequest]] = Vec(ap.nLmshrs, Valid(new VLdRequest(ap)))
    /** hit or miss form vtlb. If miss, then do nothing. */
    val vtlbResp = Flipped(Valid(new VTLBResp(ap)))
  })

  io.mshrAllocate := 0.U.asTypeOf(Vec(ap.nLmshrs, Valid(new VLdRequest(ap))))
  val mshrVlds = io.lmshrStatus.map(_.valid)

  val reg = RegInit(0.U.asTypeOf(Valid(new VLdRequest(ap))))
  val idleLmshrs = VecInit(io.lmshrStatus.map(!_.valid)).asUInt()
  val idleLmshr = idleLmshrs.orR()
  val allocateMshrIdx: UInt = OHToUInt(FindFirstOne(idleLmshrs, ap.nLmshrs))
  when(reg.valid && idleLmshr){
    io.mshrAllocate(allocateMshrIdx).valid := true.B
    io.mshrAllocate(allocateMshrIdx).bits := reg.bits
    reg.valid := false.B
  }
  /** Indicates if new request is capable to process. */
  val newAddrSound = io.reqIncoming.valid && ((io.vtlbResp.valid && io.vtlbResp.bits.hit) || io.reqIncoming.bits.addressIsPhysical)
  val newAddr = Mux(io.reqIncoming.valid && io.reqIncoming.bits.addressIsPhysical, io.reqIncoming.bits.address,
                  Mux(io.reqIncoming.valid && io.vtlbResp.valid && io.vtlbResp.bits.hit, io.vtlbResp.bits.paddr, 0.U))
  val newAddrHitMshrs = VecInit(io.lmshrStatus.map { mshr => mshr.valid && (newAddr >> ap.offsetBits).asUInt() === (mshr.bits >> ap.offsetBits).asUInt()}).asUInt()
  val newAddrHitMshr = newAddrHitMshrs.orR()
  val newAddrHitReg = reg.valid && ((reg.bits.address >> ap.offsetBits).asUInt() === (newAddr >> ap.offsetBits).asUInt())
  val newAddrMissAll = !newAddrHitMshr && !newAddrHitReg
  val mshrEnough = PopCount(idleLmshrs) > 1.U
  /** 1. reg is valid, new addr hit mshr or reg, then ack, non-takein.
   *                   new addr miss all, and there is no idle mshr, then non-ack, non-takein.
   *  2. reg is idle,  new addr hit mshr, then ack, non-takein.
   *                   new addr miss all, then ack, takein.
   *  */
  val takeIn = newAddrMissAll && (!reg.valid || mshrEnough)
  /** back pressure. */
  val sendAck = !newAddrMissAll || !reg.valid || mshrEnough
  when(io.reqIncoming.valid && takeIn){
    reg.valid := true.B
    reg.bits := io.reqIncoming.bits
    reg.bits.address := newAddr
  }
  /** addr in reg should not hit any busy mshr. */
  io.reqIncoming.ready := io.reqIncoming.valid && sendAck && newAddrSound
}