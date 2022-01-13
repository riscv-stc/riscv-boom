package boom.vlsu

import chisel3._
import chisel3.util._
import freechips.rocketchip.config.Parameters
import boom.common._
import boom.exu.{CommitSignals, RegisterFileWritePort}
import boom.util.{IsOlder, WrapInc}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.rocket._
import freechips.rocketchip.tile._

/** This module handle address check and allocate idle or merge mshrs. */
class VStQueueHandler(ap: VLSUArchitecturalParams) extends VLSUModules(ap){
  val io = IO(new Bundle{
    /** Vector uop from dispatch stage. */
    val vuopDis: Vec[ValidIO[VLSMicroOP]] = Flipped(Vec(ap.coreWidth, Valid(new VLSMicroOP(ap))))
    /** Vector uop from register-read stage. */
    val vuopRR: Vec[ValidIO[VLSMicroOP]] = Flipped(Vec(ap.coreWidth, Valid(new VLSMicroOP(ap))))
    /** output winner of a vst queue entry req arb. */
    val vstReq: Vec[DecoupledIO[VStRequest]] = Vec(ap.coreWidth, Decoupled(new VStRequest(ap)))

    val disVStQIdx = Output(Vec(ap.coreWidth, Valid(new DispatchAck(ap))))
    val vStQFull = Output(Vec(ap.coreWidth, Bool()))
    val toRob = new VLSUROBIO(ap)
    val fromRob = new ROBVLSUIO(ap)
    val fromVTLB = Flipped(Valid(new VTLBResp(ap)))
    /** Finish signal from smshr. */
    val dataAccomplished = Flipped(Valid(new VStRequest(ap)))

  })

  val nEntries = ap.nVStQEntries
  val nIdxBits = ap.nVStQIndexBits
  val headPtr: UInt = RegInit(0.U(nIdxBits.W))
  /** points to entry to enq. */
  val tailPtr: UInt = RegInit(0.U(nIdxBits.W))
  val flushPtr: UInt = RegInit(0.U(nIdxBits.W))
  /** always points to oldest accomplished store. */
  val accomplishPtr: UInt = RegInit(0.U(nIdxBits.W))
  /** This records all un-committed non-unit-stride vuop,
   * vuop that is younger than the oldest should not go split */
  val nonUnitStrideOHs = RegInit(0.U.asTypeOf(Vec(nEntries, Bool())))
  val vstReqArb: RequestArbitrator[VStRequest] = Module(new RequestArbitrator(ap, new VStRequest(ap), false))

  val vuopDisInputs: Vec[ValidIO[VLSMicroOP]] =
    WireInit(0.U.asTypeOf(Vec(nEntries, Valid(new VLSMicroOP(ap)))))
  val vuopRRInputs: Vec[ValidIO[VLSMicroOP]] =
    WireInit(0.U.asTypeOf(Vec(nEntries, Valid(new VLSMicroOP(ap)))))

  val reqCandidates = WireInit(0.U.asTypeOf(Vec(nEntries, Decoupled(new VLdRequest(ap)))))

  val finishVec = WireInit(0.U.asTypeOf(Vec(nEntries, Bool())))
  val retireVec = WireInit(0.U.asTypeOf(Vec(nEntries, Bool())))
  val toRobVec = WireInit(0.U.asTypeOf(Vec(nEntries, Decoupled(UInt(ap.robAddrSz.W)))))
  val entries = Seq.tabulate(nEntries){ i =>
    val e = Module(new VStQEntry(ap, i))
    e.io.vuopDis := vuopDisInputs(i)
    e.io.vuopRR := vuopRRInputs(i)
    reqCandidates(i) <> e.io.uReq
    finishVec(i) := e.io.allDone
    vstReqArb.io.inputReq(i) <> e.io.uReq
    e.io.fromRob := io.fromRob
    e.io.fromVTLB := io.fromVTLB
    e.io.retire := retireVec(i)
    toRobVec(i) <> e.io.robAck
    e.io.dataAccomplished <> io.dataAccomplished
    e.io.headPtr := headPtr
    e.io.tailPtr := tailPtr
    e.io.nonUnitStrideOHs := nonUnitStrideOHs.asUInt()
    e
  }
  (vstReqArb.io.inputReq zip reqCandidates).foreach { case (a, req) => a <> req }
  //__________________________________________________________________________________//
  //--------------------------Dispatch Allocate---------------------------------------//
  val enqPtrVec = Wire(Vec(ap.coreWidth, UInt(nIdxBits.W)))
  for ( w <- 0 until ap.coreWidth){
    if (w == 0) enqPtrVec(w) := tailPtr
    else enqPtrVec(w) := WrapInc(enqPtrVec(w - 1), nEntries)
  }
  val storeVlds: Seq[Bool] = io.vuopDis.map(in => in.valid && in.bits.uCtrlSig.accessType.isStore)
  val storeVldCount: UInt = PopCount(storeVlds)
  io.disVStQIdx.foreach(idx => idx := 0.U.asTypeOf(Valid(new DispatchAck(ap))))
  for ( w <- 0 until ap.coreWidth){
    val preVlds: UInt = if (w == 0) 0.U else VecInit(storeVlds.slice(0, w)).asUInt()
    val preVldCount: UInt = PopCount(preVlds)
    val enqPtr: UInt = enqPtrVec(preVldCount)
    val vStQFull = WrapInc(enqPtr, ap.nVStQEntries) === headPtr
    io.vStQFull(w) := vStQFull
    val isVectorStore: Bool = io.vuopDis(w).valid && io.vuopDis(w).bits.uCtrlSig.accessType.isStore
    val nonUnitStride: Bool = isVectorStore &&
      (io.vuopDis(w).bits.uCtrlSig.accessStyle.isConstantStride || io.vuopDis(w).bits.uCtrlSig.accessStyle.isIndexed)
    when(isVectorStore){
      vuopDisInputs(enqPtr).valid := true.B
      vuopDisInputs(enqPtr).bits := io.vuopDis(w).bits
      when(nonUnitStride){
        nonUnitStrideOHs(enqPtr) := true.B
      }
      io.disVStQIdx(w).valid := true.B
      io.disVStQIdx(w).bits.qIdx := enqPtr
      io.disVStQIdx(w).bits.robIdx := io.vuopDis(w).bits.robIdx
    }
  }
  when(VecInit(storeVlds).asUInt().orR()){
    tailPtr := WrapInc(enqPtrVec(storeVldCount - 1.U), ap.nVStQEntries)
  }
  //__________________________________________________________________________________//
  //------------------------Register Read Wake up-------------------------------------//
  io.vuopRR.foreach{ rr =>
    when(rr.valid && rr.bits.uCtrlSig.accessType.isStore){
      vuopRRInputs(rr.bits.vStQIdx) := rr
    }
  }
  //__________________________________________________________________________________//
  //--------------------------Request Arbitration-------------------------------------//
  (io.vstReq zip vstReqArb.io.outputReq).foreach { case (outReq, arbWinner) =>
    outReq <> arbWinner
  }
  //__________________________________________________________________________________//
  //--------------------------Ack to rob when tlb finished----------------------------//
  val ackArbiter = Module(new RequestArbitrator(ap, UInt(ap.robAddrSz.W), false))
  ackArbiter.io.inputReq <> toRobVec
  (io.toRob.robIdx zip ackArbiter.io.outputReq).foreach { case (out, ack) =>
    out := ack
    ack.ready := true.B
  }

  //__________________________________________________________________________________//
  //--------------------------Dequeue on Finish---------------------------------------//
  // At each cycle, scan entry pointed by head, if all done, release its resource.
  val finished = finishVec(headPtr)
  when(nonUnitStrideOHs(headPtr) && finished){
    nonUnitStrideOHs(headPtr) := false.B
  }
  retireVec(headPtr) := finished
  headPtr := Mux(finished, WrapInc(headPtr, ap.nVStQEntries), headPtr)
}


class VStQEntry(ap: VLSUArchitecturalParams, id: Int) extends VLSUModules(ap){
  val io = IO(new Bundle{
    val entryId: UInt = Output(UInt(ap.nVStQIndexBits.W))
    val vuopDis = Flipped(Valid(new VLSMicroOP(ap)))
    val vuopRR = Flipped(Valid(new VLSMicroOP(ap)))
    val uReq = DecoupledIO(new VStRequest(ap))
    /** finish signal from store req buffer. */
    val dataAccomplished = Flipped(Valid(new VStRequest(ap)))
    val robAck = Decoupled(UInt(ap.robAddrSz.W))
    val fromRob = new ROBVLSUIO(ap)
    val allDone = Output(Bool())
    val retire = Input(Bool())
    val fromVTLB = Flipped(Valid(new VTLBResp(ap)))
    val headPtr = Input(UInt(ap.nVStQIndexBits.W))
    val tailPtr = Input(UInt(ap.nVStQIndexBits.W))
    val nonUnitStrideOHs = Input(UInt(ap.nVStQEntries.W))
  })
  io.uReq.valid := false.B
  io.uReq.bits := 0.U.asTypeOf(new VStRequest(ap))
  io.robAck.valid := false.B
  io.robAck.bits := 0.U.asTypeOf(UInt(ap.robAddrSz.W))
  io.allDone := false.B

  val reg = RegInit(0.U.asTypeOf(Valid(new VStQEntryBundle(ap))))
  io.entryId := id.U

  /** State descriptions:
   * [[sWaitRs]]: wait data and address.
   * [[sSplitting]]: splitting vuop into individual requests.
   * [[sWaitCommit]]: wait for commit to issue request to memory hierarchy.
   * [[sWaitResp]]: wait response from memory hierarchy.
   * [[sWaitRetire]]: wait for handler to release us after all done.
   * */
  val sIdle :: sWaitRs :: sSplitting :: sWaitCommitArb :: sWaitCommitResp :: sWaitDataResp :: sWaitRetire :: Nil = Enum(7)
  val state = RegInit(sIdle)

  val isUnitStride = reg.bits.style.isUnitStride

  val snippetInitializer = Module(new SnippetInitializer(ap))
  snippetInitializer.io.ctrl := io.vuopDis.bits.uCtrlSig.accessStyle

  val snippetVMAdjuster = Module(new SnippetVectorMaskAdjuster(ap))
  snippetVMAdjuster.io.eew := io.vuopRR.bits.uCtrlSig.accessStyle.eew
  snippetVMAdjuster.io.vlmul := io.vuopRR.bits.uCtrlSig.accessStyle.vlmul
  snippetVMAdjuster.io.isSegment := io.vuopRR.bits.uCtrlSig.accessStyle.isSegment
  snippetVMAdjuster.io.nf := io.vuopRR.bits.uCtrlSig.accessStyle.nf
  snippetVMAdjuster.io.vm := io.vuopRR.bits.vm

  /** If the oldest un-commit non-unit-stride is older than us, hang. */
  val oldestNonUnitStrideIdx: ValidIO[UInt] = PickOldest(io.nonUnitStrideOHs, io.headPtr, io.tailPtr, ap.nVStQEntries)
  val freeze = oldestNonUnitStrideIdx.valid && IsOlder(oldestNonUnitStrideIdx.bits, id.U, io.headPtr)
  val requestSplitter = Module(new RequestSplitter(ap, false, id))
  requestSplitter.io.reg := reg.bits
  io.uReq.bits := requestSplitter.io.uReq.bits

  when(state === sIdle){
    when(io.vuopDis.valid){
      reg.bits.addr := 0.U
      reg.bits.style := io.vuopDis.bits.uCtrlSig.accessStyle
      reg.bits.robIndex := io.vuopDis.bits.robIdx
      reg.bits.totalReq := snippetInitializer.io.totalRequest
      reg.bits.finishMasks := snippetInitializer.io.initSnippet
      reg.bits.tlbMasks := snippetInitializer.io.initSnippet
      reg.bits.pRegVec := io.vuopDis.bits.vpdst
      reg.bits.segmentCount := 0.U
      reg.bits.totalSegments := snippetInitializer.io.totalSegment
      state := sWaitRs
    }
  }.elsewhen(state === sWaitRs){
    when(io.vuopRR.valid){
      reg.bits.addr := io.vuopRR.bits.rs1
      reg.bits.preAddr := io.vuopRR.bits.rs1
      reg.bits.vs1 := io.vuopRR.bits.vs1
      reg.bits.vs2 := io.vuopRR.bits.vs2
      reg.bits.rs2 := io.vuopRR.bits.rs2
      /** indicates if addr is aligned to line address. */
      val offset: UInt = io.vuopRR.bits.rs1(ap.offsetBits - 1, 0)
      val alignedAddr: Bool = offset === 0.U
      val denseAccess = (reg.bits.style.isUnitStride && !reg.bits.style.isSegment) || reg.bits.style.isWholeAccess
      reg.bits.totalReq := Mux(alignedAddr && denseAccess, (ap.maxReqsInUnitStride - 1).U, reg.bits.totalReq)
      val adjustedSnippet: Vec[UInt] =
        VecInit((snippetVMAdjuster.io.adjustedSnippet zip reg.bits.finishMasks).map {case (vm, snippet) =>
          vm | snippet
        })
      reg.bits.finishMasks := adjustedSnippet
      reg.bits.tlbMasks := adjustedSnippet

      state := sSplitting
    }
  }.elsewhen(state === sSplitting){
    io.uReq.valid := requestSplitter.io.uReq.valid && !freeze
    io.uReq.bits := requestSplitter.io.uReq.bits
    when((io.uReq.valid && io.uReq.ready) || !io.uReq.valid){
      reg.bits.reqCount := reg.bits.reqCount + 1.U
      reg.bits.preAddr := requestSplitter.io.newAddr
      reg.bits.preSnippet := requestSplitter.io.newSnippet
    }
    when(reg.bits.reqCount === reg.bits.totalReq - 1.U){
      reg.bits.preAddr := reg.bits.addr
      reg.bits.preSnippet := 0.U
      reg.bits.segmentCount := reg.bits.segmentCount + 1.U
      reg.bits.reqCount := 0.U
      when(reg.bits.segmentCount === reg.bits.totalSegments){
        state := sWaitCommitArb
      }
    }
  }.elsewhen(state === sWaitCommitArb){
    when(reg.bits.tlbMasks.asUInt().andR()){
      io.robAck.valid := true.B
      io.robAck.bits := reg.bits.robIndex
    }.otherwise{
      io.robAck.valid := false.B
      io.robAck.bits := 0.U
    }
    when(io.robAck.fire()) {
      state := sWaitCommitResp
    }
  }.elsewhen(state === sWaitCommitResp){
    when(VecInit(io.fromRob.retireEntries.map(i => i.valid && i.bits.isStore && i.bits.qEntryIdx === id.U)).asUInt().orR()){
      reg.bits.committed := true.B
      state := sWaitDataResp
    }
  }.elsewhen(state === sWaitDataResp){
    when(io.dataAccomplished.valid && io.dataAccomplished.bits.qEntryIdx === id.U){
      reg.bits.finishMasks(io.dataAccomplished.bits.segmentIdx) := reg.bits.finishMasks(io.dataAccomplished.bits.segmentIdx) | io.dataAccomplished.bits.regAccessCS.finishMaskSnippet
    }
    val allFinished = WireInit(VecInit(reg.bits.finishMasks.map(_.andR())).asUInt().andR())
    when(allFinished){
      reg.bits.allSucceeded := true.B
      state := sWaitRetire
    }
  }.elsewhen(state === sWaitRetire){
    io.allDone := reg.bits.allSucceeded
    assert(io.allDone, "\n Must done all when waiting retire!\n")
    when(io.retire){
      state := sIdle
    }
  }
  when(io.fromVTLB.valid && state =/= sIdle && !io.fromVTLB.bits.isLoad &&
    io.fromVTLB.bits.hit && !io.fromVTLB.bits.exception && io.fromVTLB.bits.queueIdx === id.U){
    reg.bits.tlbMasks(io.fromVTLB.bits.segmentIdx) := reg.bits.tlbMasks(io.fromVTLB.bits.segmentIdx) | io.fromVTLB.bits.snippet
  }
}