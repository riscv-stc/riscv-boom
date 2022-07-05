package boom.vlsu

import chisel3._
import chisel3.util._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.rocket
import boom.common._

class StoreRequestBuffer(ap: VLSUArchitecturalParams) extends VLSUModules(ap){
  val io = IO(new Bundle{
    val brUpdate = Input(new BranchUpdateInfo(ap))
    val reqIncoming = Flipped(Vec(ap.coreWidth, Decoupled(new VStRequest(ap))))
    val reqOutgoing = Decoupled(new VStRequest(ap))
    val vtlbReq = Decoupled(new VTLBReq(ap))
    val fromRob = new ROBVLSUIO(ap)
    val vtlbResp = Flipped(Valid(new VTLBResp(ap)))
    val dataAccomplished = Flipped(Valid(new VStRequest(ap)))
    val vrfReadReq = Valid(new VLSUReadVRFReq(ap))
  })

  val stEntries = Seq.tabulate(ap.nVStReqBuffEntries)(i => Module(new StoreBufferEntry(ap, i)))

  val entryInputs = WireInit(0.U.asTypeOf(Vec(ap.nVStReqBuffEntries, Valid(new VStRequest(ap)))))

  val entryVlds = VecInit(stEntries.map(_.io.status.valid)).asUInt()

  val idleEntryArbiter = Module(new IndexArbiter(ap.nVStReqBuffEntries, ap.coreWidth))
  idleEntryArbiter.io.input := entryVlds
  val idleEntryIdxes = idleEntryArbiter.io.output
  val vStReqArbiter = Module(new Arbiter(new VStRequest(ap), ap.nVStReqBuffEntries))
  val vTlbReqArbiter = Module(new Arbiter(new VTLBReq(ap), ap.nVStReqBuffEntries))
  stEntries.zipWithIndex.foreach{ case (e, i) =>
    vStReqArbiter.io.in(i) <> e.io.reqOutgoing
    vTlbReqArbiter.io.in(i) <> e.io.toVTlb
    e.io.reqIncoming <> entryInputs(i)
    e.io.dataAccomplished := io.dataAccomplished
    e.io.fromVTlb := io.vtlbResp
    e.io.fromRob := io.fromRob
    e.io.brUpdate := io.brUpdate
  }
  (io.reqIncoming.map(_.ready) zip idleEntryIdxes.map(_.valid)) foreach { case (r, v) =>
    r := v
  }
  //__________________________________________________________________________________//
  //--------------------------New Req Allocate----------------------------------------//
  var inputIdx = WireInit(0.U.asTypeOf(UInt(log2Ceil(ap.coreWidth).W)))
  for (w <- 0 until ap.coreWidth){
    val vld = io.reqIncoming(w).valid && idleEntryIdxes(inputIdx).valid
    when(vld){entryInputs(idleEntryIdxes(inputIdx).bits) := io.reqIncoming(w) }
    inputIdx = Mux(vld, inputIdx + 1.U, inputIdx)
  }
  //__________________________________________________________________________________//
  //--------------------------Requests Output-----------------------------------------//
  val arbWinnerReq = vStReqArbiter.io.out
  io.reqOutgoing <> arbWinnerReq
  io.vrfReadReq.valid       := arbWinnerReq.valid
  io.vrfReadReq.bits.addr   := Mux(arbWinnerReq.bits.isTile, arbWinnerReq.bits.ridx, arbWinnerReq.bits.regAccessCS.regIdx)
  io.vrfReadReq.bits.sidx   := arbWinnerReq.bits.sidx
  io.vrfReadReq.bits.isTile := arbWinnerReq.bits.isTile
  io.vtlbReq <> vTlbReqArbiter.io.out
}

class StoreBufferEntry(ap: VLSUArchitecturalParams, idx: Int) extends VLSUModules(ap){
  val io = IO(new Bundle{
    val brUpdate = Input(new BranchUpdateInfo(ap))
    val reqIncoming = Flipped(Valid(new VStRequest(ap)))
    val reqOutgoing = DecoupledIO(new VStRequest(ap))
    val toVTlb = DecoupledIO(new VTLBReq(ap))
    val fromVTlb = Flipped(Valid(new VTLBResp(ap)))
    /** expose to buffer handler. */
    val status = Valid(new VStRequest(ap))
    /** Finish signal from smshr. */
    val dataAccomplished = Flipped(Valid(new VStRequest(ap)))
    /** Commit signal from rob. */
    val fromRob = new ROBVLSUIO(ap)
  })
  io.reqOutgoing.valid := false.B
  io.reqOutgoing.bits := 0.U.asTypeOf(new VStRequest(ap))
  io.toVTlb.valid := false.B
  io.toVTlb.bits := 0.U.asTypeOf(new VTLBReq(ap))

  val reg = RegInit(0.U.asTypeOf(Valid(new VStRequest(ap))))
  val isKilledByBranch = IsKilledByBranch(io.brUpdate, reg.bits.brMask)
  io.status := reg
  io.reqOutgoing.valid := false.B
  io.reqOutgoing.bits := reg.bits
  when(io.reqIncoming.valid){
    assert(!reg.valid, s"Entry ${idx} should be idle when allocating new request.")
    reg.valid := true.B
    reg.bits := io.reqIncoming.bits
    reg.bits.brMask := GetNewBranchMask(io.brUpdate, io.reqIncoming.bits.brMask)
    reg.bits.reqBufferIdx := idx.U
  }
  when(VecInit(io.fromRob.retireEntries.map(r =>
    r.valid && r.bits.qEntryIdx === reg.bits.qEntryIdx && r.bits.isStore)).asUInt().orR()){
    reg.bits.committed := true.B
  }
  when(reg.valid && !isKilledByBranch){
    when(!reg.bits.addressIsPhysical){
      io.toVTlb.valid := true.B
      io.toVTlb.bits.vaddr := reg.bits.address
      io.toVTlb.bits.queueIdx := reg.bits.qEntryIdx
      io.toVTlb.bits.isLoad := false.B
      io.toVTlb.bits.reqBufferIdx := idx.U
      io.toVTlb.bits.snippet := reg.bits.regAccessCS.finishMaskSnippet
      io.toVTlb.bits.segmentIdx := reg.bits.segmentIdx
    }
    when(reg.bits.committed && !reg.bits.executing){
      io.reqOutgoing.valid := true.B
      io.reqOutgoing.bits := reg.bits
      when(io.reqOutgoing.fire()){
          reg.bits.executing := true.B
      }
    }
  }
  when(reg.valid && io.dataAccomplished.valid && (io.dataAccomplished.bits.reqBufferIdx === idx.U)){
    reg.valid := false.B
  }
  when(io.fromVTlb.valid && (io.fromVTlb.bits.reqBufferIdx === idx.U) && io.fromVTlb.bits.hit && !io.fromVTlb.bits.exception){
    reg.bits.address := io.fromVTlb.bits.paddr
    reg.bits.addressIsPhysical := true.B
  }
  when(reg.valid && isKilledByBranch){
    reg.valid := false.B
  }
}