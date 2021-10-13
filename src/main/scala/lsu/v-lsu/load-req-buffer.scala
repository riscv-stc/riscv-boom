package boom.vlsu

import chisel3._
import chisel3.util._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.rocket
import boom.common._

/** This module hold all element access for all vector store or load uop.
 * Random access is supported.
 * */
class LoadRequestBuffer(ap: VLSUArchitecturalParams) extends VLSUModules(ap){
  val io = IO(new Bundle {
    /** vld request from vld queue winner. */
    val reqIncoming: Vec[DecoupledIO[VLdRequest]] = Flipped(Vec(ap.coreWidth, Decoupled(new VLdRequest(ap))))
    //val vStReq = Flipped(Valid(new VStRequest(ap)))
    /** When a line is fetch back from L2,
     * the buffer will be scanned to pick all requests with same address. */
    val wbBypassQuery: ValidIO[UInt] = Flipped(Valid(UInt(ap.coreMaxAddrBits.W)))
    /** Tell wb controller which buffer entry is fit for bypass write back. */
    val wbBypassResp: Vec[ValidIO[VLdRequest]] =
      Vec(ap.nVLdReqBuffEntries, Valid(new VLdRequest(ap)))
    /** query addr checker to process data fetch. */
    val reqOutgoing: DecoupledIO[VLdRequest] = Decoupled(new VLdRequest(ap))
    /** query tlb to fetch paddr,  */
    val vtlbReq = DecoupledIO(new VTLBReq(ap))
    /** resp from vtlb, if miss, then this entry should re-issue. */
    val vtlbResp = Flipped(Valid(new VTLBResp(ap)))
    /** from wb controller telling us which buffer entry has finished. */
    val reqWBDone = Flipped(Valid(UInt(ap.nVLdReqBufferIdxBits.W)))
    /** from vld queue telling us which buffer entry need to retire due to commit. */
    val fromRob = new ROBVLSUIO(ap)
  })

  /** Instances of entries. */
  val ldEntries: Seq[LoadBufferEntry] =
    Seq.tabulate(ap.nVLdReqBuffEntries)(i => Module(new LoadBufferEntry(ap, i)))
  /** Input IO to entries. */
  val entryInputs: Vec[ValidIO[VLdRequest]] = WireInit(0.U.asTypeOf(Vec(ap.nVLdReqBuffEntries,Valid(new VLdRequest(ap)))))
  /** idle entries to allocate */
  val entryVlds: UInt = VecInit(ldEntries.map(_.io.status.valid)).asUInt()
  // Pick coreWidth idle entries to allocate.
  val idleEntryArbiter: IndexArbiter = Module(new IndexArbiter(ap.nVLdReqBuffEntries, ap.coreWidth))
  idleEntryArbiter.io.input := entryVlds
  val idleEntryIndexes: Vec[ValidIO[UInt]] = idleEntryArbiter.io.output
  /** From all idle entries, pick out one and convert it into index then pass back to queue. */
  val vLdReqArbitor = Module(new Arbiter(new VLdRequest(ap), ap.nVLdReqBuffEntries))
  ldEntries.zipWithIndex.foreach{ case (e, i) =>
    vLdReqArbitor.io.in(i) <> e.io.reqOutgoing
    e.io.wbBypassQuery <> io.wbBypassQuery
    io.wbBypassResp(i) <> e.io.wbBypassResp
    e.io.reqIncoming <> entryInputs(i)
    e.io.reqWBDone := io.reqWBDone
    e.io.fromVTlb := io.vtlbResp
    e.io.fromRob := io.fromRob
  }
  (io.reqIncoming.map(_.ready) zip idleEntryIndexes.map(_.valid)) foreach { case (r, v) =>
    r := v
  }
  //__________________________________________________________________________________//
  //--------------------------New Req Allocate----------------------------------------//
  var inputIndex: UInt = WireInit(0.U.asTypeOf(UInt(log2Ceil(ap.coreWidth).W)))
  for(w <- 0 until ap.coreWidth){
    val vld = io.reqIncoming(w).valid && idleEntryIndexes(inputIndex).valid
    when(vld){
      entryInputs(idleEntryIndexes(inputIndex).bits) := io.reqIncoming(w)
    }
    inputIndex = Mux(vld, inputIndex + 1.U, inputIndex)
  }
  //__________________________________________________________________________________//
  //--------------------------Requests Output-----------------------------------------//
  io.reqOutgoing <> vLdReqArbitor.io.out
  io.vtlbReq.valid := vLdReqArbitor.io.out.valid && !vLdReqArbitor.io.out.bits.addressIsPhysical
  io.vtlbReq.bits.vaddr := vLdReqArbitor.io.out.bits.address
  io.vtlbReq.bits.reqBufferIdx := vLdReqArbitor.io.out.bits.qEntryIdx
  io.vtlbReq.bits.isLoad := true.B
  io.vtlbReq.bits.queueIdx := vLdReqArbitor.io.out.bits.qEntryIdx
  io.vtlbReq.bits.snippet := vLdReqArbitor.io.out.bits.regAccessCS.finishMaskSnippet
  io.vtlbReq.bits.segmentIdx := vLdReqArbitor.io.out.bits.segmentIdx
}

class LoadBufferEntry(ap: VLSUArchitecturalParams, idx: Int) extends VLSUModules(ap){
  val io = IO(new Bundle{
    /** request from vld queue arb winner. */
    val reqIncoming: ValidIO[VLdRequest] = Flipped(Valid(new VLdRequest(ap)))
    /** expose us to buff for new request allocate. */
    val status: ValidIO[VLdRequest] = Valid(new VLdRequest(ap))
    /** request issue down to address checker. */
    val reqOutgoing: DecoupledIO[VLdRequest] = DecoupledIO(new VLdRequest(ap))
    /** address quest from wb controller for bypass wb. */
    val wbBypassQuery: ValidIO[UInt] = Flipped(Valid(UInt(ap.coreMaxAddrBits.W)))
    /** tell wb if querying address hit us. */
    val wbBypassResp: ValidIO[VLdRequest] = Valid(new VLdRequest(ap))
    /** from wb controller to tell us which entry has finished. */
    val reqWBDone = Flipped(Valid(UInt(ap.nVLdReqBufferIdxBits.W)))
    /** Resp from vtlb at same cycle. */
    val fromVTlb = Flipped(Valid(new VTLBResp(ap)))
    val fromRob = new ROBVLSUIO(ap)
  })
  io.wbBypassResp.valid := false.B
  io.wbBypassResp.bits := 0.U.asTypeOf(new VLdRequest(ap))
  val reg = RegInit(0.U.asTypeOf(Valid(new VLdRequest(ap))))
  reg.bits.reqBufferIdx := idx.U
  io.status := reg
  io.reqOutgoing.valid := (reg.valid && !reg.bits.executing && !reg.bits.done)
  io.reqOutgoing.bits := reg.bits
  io.reqOutgoing.bits.reqBufferIdx := idx.U
  when(reg.valid && io.reqOutgoing.fire() && (reg.bits.addressIsPhysical || io.fromVTlb.bits.hit)){
    reg.bits.executing := true.B
  }
  when(io.wbBypassQuery.valid && lineAddress(io.wbBypassQuery.bits) === lineAddress(reg.bits.address) &&
    !reg.bits.done && reg.valid){
    io.wbBypassResp.valid := true.B
    io.wbBypassResp.bits := reg.bits
  }
  when(io.reqIncoming.valid){
    assert(!reg.valid, s"Entry ${idx} should be idle when allocating new request.")
    reg.valid := true.B
    reg.bits := io.reqIncoming.bits
  }
  when(io.reqWBDone.valid && io.reqWBDone.bits === idx.U && reg.valid){
    reg.bits.done := true.B
    reg.bits.executing := false.B
  }
  when(VecInit(io.fromRob.retireEntries.map{i => i.valid && i.bits.isLoad && i.bits.qEntryIdx === reg.bits.qEntryIdx}).asUInt().orR()){
    assert(Mux(reg.valid, reg.bits.done, true.B))
    reg.valid := false.B
  }
}

