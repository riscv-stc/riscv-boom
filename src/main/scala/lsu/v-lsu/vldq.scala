package boom.vlsu

import chisel3._
import chisel3.util._
import boom.util.{AgePriorityEncoder, IsOlder, WrapInc}
/** vuop should be kept in order from dispatch, they might be flushed after a entry. */
class VLdQueueHandler(ap: VLSUArchitecturalParams) extends VLSUModules(ap){
  val io = IO(new Bundle{
    /** Vector uop from dispatch stage. */
    val vuopDis: Vec[ValidIO[VLSMicroOP]] = Flipped(Vec(ap.coreWidth, Valid(new VLSMicroOP(ap))))
    /** Vector uop from register-read stage. */
    val vuopRR: Vec[ValidIO[VLSMicroOP]] = Flipped(Vec(ap.coreWidth, Valid(new VLSMicroOP(ap))))
    /** Winner req from a [[VLdQEntry]] */
    val vldReq: Vec[DecoupledIO[VLdRequest]] = Vec(ap.coreWidth, Decoupled(new VLdRequest(ap)))
    /** Finish ack from wb controller. */
    val finishAck: ValidIO[VLdRequest] = Flipped(Valid(new VLdRequest(ap)))
    /** Tell core pipeline index of newly allocated load queue entry. */
    val disVLdQIdx = Output(Vec(ap.coreWidth, Valid(new DispatchAck(ap))))
    /** Tell core stop dispatch when no entry left. */
    val vLdQFull = Output(Vec(ap.coreWidth, Bool()))
    /** Tell ROB that one vuop has done all elements. */
    val toRob = new VLSUROBIO(ap)
    /** Retire finished load queue entry. */
    val fromRob = new ROBVLSUIO(ap)

    val wakeUp = ValidIO(UInt(ap.vpregSz.W))

    /** For untouched load, we need to copy original data and write back to new reg. */
    val vrfReadReq = Decoupled(new VLSUReadVRFReq(ap))
    val vrfReadResp = Flipped(Valid(new VLSUReadVRFResp(ap)))
    val vrfWriteReq = Decoupled(new VLSUWriteVRFReq(ap))
  })

  val nEntries: Int = ap.nVLdQEntries
  val nIdxBits: Int = ap.nVLdQIndexBits
  /** Point to oldest vuop. */
  val headPtr: UInt = RegInit(0.U(nIdxBits.W))
  /** Point to youngest vuop. */
  val tailPtr: UInt = RegInit(0.U(nIdxBits.W))
  val flushPtr: UInt = RegInit(0.U(nIdxBits.W))
  val finishPtr: UInt = RegInit(0.U(nIdxBits.W))
  /** This records all un-committed non-unit-stride vuop,
   * vuop that is younger than the oldest should not go split */
  val nonUnitStrideOHs = RegInit(0.U.asTypeOf(Vec(nEntries, Bool())))
  io.toRob  := 0.U.asTypeOf(new VLSUROBIO(ap))

  /** Arb req from each [[entries]] */
  val vldReqArb = Module(new RequestArbitrator(ap, new VLdRequest(ap), true))
  /** Vec from io.dispatch to each entry according dispatching signals. */
  val vuopDisInputs: Vec[ValidIO[VLSMicroOP]] =
    WireInit(0.U.asTypeOf(Vec(nEntries, Valid(new VLSMicroOP(ap)))))
  /** Vec from register holding accessing address. */
  val vuopRRInputs = WireInit(0.U.asTypeOf(Vec(nEntries, Valid(new VLSMicroOP(ap)))))

  /** Vector from each entry output to be arbitrated. */
  val reqCandidates: Vec[DecoupledIO[VLdRequest]] = {
    WireInit(0.U.asTypeOf(Vec(nEntries, Decoupled(new VLdRequest(ap)))))
  }
  val toRobVec = WireInit(0.U.asTypeOf(Vec(nEntries, Decoupled(UInt(ap.robAddrSz.W)))))
  val finishVec = WireInit(0.U.asTypeOf(Vec(nEntries, Bool())))
  val wakeUpVec = WireInit(0.U.asTypeOf(Vec(nEntries, Decoupled(UInt(ap.vpregSz.W)))))
  val vrfReadArb = Module(new Arbiter(new VLSUReadVRFReq(ap), ap.nVLdQEntries))
  val vrfWriteArb = Module(new Arbiter(new VLSUWriteVRFReq(ap), ap.nVLdQEntries))
  val entries: Seq[VLdQEntry] = Seq.tabulate(nEntries){ i =>
    val e = Module(new VLdQEntry(ap, i))
    e.io.vuopDis := vuopDisInputs(i)
    e.io.vuopRR := vuopRRInputs(i)
    reqCandidates(i) <> e.io.uReq
    e.io.finishAck := io.finishAck
    toRobVec(i) <> e.io.robAck
    finishVec(i) := e.io.allDone
    wakeUpVec(i) <> e.io.wakeUp
    e.io.fromRob := io.fromRob
    e.io.headPtr := headPtr
    e.io.tailPtr := tailPtr
    e.io.nonUnitStrideOHs := nonUnitStrideOHs.asUInt()
    vrfReadArb.io.in(i) <> e.io.vrfReadReq
    e.io.vrfReadResp := io.vrfReadResp
    vrfWriteArb.io.in(i) <> e.io.vrfWriteReq
    e
  }
  io.vrfReadReq <> vrfReadArb.io.out
  io.vrfWriteReq <> vrfWriteArb.io.out
  // @todo: only one data path is not enough and round ronbin arbiter is also not good.
  (vldReqArb.io.inputReq zip reqCandidates).foreach {case (a, req) => a <> req}
  //__________________________________________________________________________________//
  //--------------------------Dispatch Allocate---------------------------------------//
  val enqPtrVec = Wire(Vec(ap.coreWidth, UInt(nIdxBits.W)))
  for ( w <- 0 until ap.coreWidth){
    if (w == 0) enqPtrVec(w) := tailPtr
    else enqPtrVec(w) := WrapInc(enqPtrVec(w - 1), nEntries)
  }
  val loadVlds: Seq[Bool] = io.vuopDis.map(in => in.valid && in.bits.uCtrlSig.accessType.isLoad)
  val loadVldCount: UInt = PopCount(loadVlds)
  io.disVLdQIdx.foreach(idx => idx := 0.U.asTypeOf(Valid(new DispatchAck(ap))))
  for ( w <- 0 until ap.coreWidth){
    val preVlds: UInt = if (w == 0) 0.U else VecInit(loadVlds.slice(0,w)).asUInt()
    val preVldCount: UInt = PopCount(preVlds)
    val enqPtr: UInt = enqPtrVec(preVldCount)
    val vLdQFull = WrapInc(enqPtr, ap.nVLdQEntries) === headPtr
    io.vLdQFull(w) := vLdQFull
    val isVectorLoad: Bool = io.vuopDis(w).valid && io.vuopDis(w).bits.uCtrlSig.accessType.isLoad
    assert(Mux(io.vuopDis(w).valid, io.vuopDis(w).bits.uCtrlSig.accessType.isStore ^ io.vuopDis(w).bits.uCtrlSig.accessType.isLoad, true.B),
      "vuop shold not be ld and st at same time.")
    val nonUnitStride: Bool = isVectorLoad &&
      (io.vuopDis(w).bits.uCtrlSig.accessStyle.isConstantStride || io.vuopDis(w).bits.uCtrlSig.accessStyle.isIndexed)
    when(isVectorLoad){
      vuopDisInputs(enqPtr).valid := true.B
      vuopDisInputs(enqPtr).bits := io.vuopDis(w).bits
      when(nonUnitStride){
        nonUnitStrideOHs(enqPtr) := true.B
      }
      io.disVLdQIdx(w).valid := true.B
      io.disVLdQIdx(w).bits.qIdx := enqPtr
      io.disVLdQIdx(w).bits.robIdx := io.vuopDis(w).bits.robIdx
    }
  }
  when(VecInit(loadVlds).asUInt().orR()){
    tailPtr := WrapInc(enqPtrVec(loadVldCount - 1.U), ap.nVLdQEntries)
  }
  //__________________________________________________________________________________//
  //------------------------Register Read Wake up-------------------------------------//
  // Each two index should not be same.
  io.vuopRR.foreach{ rr =>
    when(rr.valid && rr.bits.uCtrlSig.accessType.isLoad){
      vuopRRInputs(rr.bits.vLdQIdx) := rr
    }
  }
  //__________________________________________________________________________________//
  //--------------------------Request Arbitration-------------------------------------//
  (io.vldReq zip vldReqArb.io.outputReq).foreach { case (outReq, arbWinner) =>
    outReq <> arbWinner
  }
  //__________________________________________________________________________________//
  //----------------------------Wake up on partial finish-----------------------------//
  val wakeUpArbiter = Module(new Arbiter(UInt(ap.vpregSz.W), nEntries))
  io.wakeUp.valid := wakeUpArbiter.io.out.valid
  io.wakeUp.bits := wakeUpArbiter.io.out.bits
  wakeUpArbiter.io.out.ready := true.B
  (wakeUpVec zip wakeUpArbiter.io.in).foreach { case (entryOut, arbIn) =>
    arbIn <> entryOut
  }
  //__________________________________________________________________________________//
  //----------------------------Ack to rob when finished------------------------------//
  val ackArbiter = Module(new RequestArbitrator(ap, UInt(ap.robAddrSz.W), true))
  ackArbiter.io.inputReq <> toRobVec
  (io.toRob.robIdx zip ackArbiter.io.outputReq).foreach {case(out, ack) =>
    out := ack
    ack.ready := true.B
  }
  //__________________________________________________________________________________//
  //----------------------------Dequeue on Commit-------------------------------------//
  val finished = finishVec(headPtr)
  when(nonUnitStrideOHs(headPtr) && finished){
    nonUnitStrideOHs(headPtr) := false.B
  }
  headPtr := Mux(finished, WrapInc(headPtr, ap.nVLdQEntries), headPtr)
  //__________________________________________________________________________________//
  //------------------------------Flush Control---------------------------------------//

}

/** Hold vuop and split it into individual cache line load requests. */
class VLdQEntry(ap: VLSUArchitecturalParams, id: Int) extends VLSUModules(ap){
  val io = IO(new Bundle{
    val entryId: UInt = Output(UInt(ap.nVLdQIndexBits.W))
    /** Arrived vuop from register-read stage. */
    val vuopRR: ValidIO[VLSMicroOP] = Flipped(Valid(new VLSMicroOP(ap)))
    /** Arrived vuop from dispatch stage. */
    val vuopDis: ValidIO[VLSMicroOP] = Flipped(Valid(new VLSMicroOP(ap)))
    /** Output request for a cache line. */
    val uReq: DecoupledIO[VLdRequest] = Decoupled(new VLdRequest(ap))

    /** Write back controller tells vld queue finished access. */
    val finishAck: ValidIO[VLdRequest] = Flipped(Valid(new VLdRequest(ap)))
    /** tell rob that this entry has all finished. */
    val robAck = Decoupled(UInt(ap.robAddrSz.W))
    val allDone = Output(Bool())
    val fromRob = new ROBVLSUIO(ap)
    val headPtr = Input(UInt(ap.nVLdQIndexBits.W))
    val tailPtr = Input(UInt(ap.nVLdQIndexBits.W))
    val nonUnitStrideOHs = Input(UInt(ap.nVLdQEntries.W))
    /** Wake up core pipe line when single register is all done. */
    val wakeUp = DecoupledIO(UInt(ap.vpregSz.W))

    val vrfReadReq = Decoupled(new VLSUReadVRFReq(ap))
    val vrfReadResp = Flipped(Valid(new VLSUReadVRFResp(ap)))
    val vrfWriteReq = Decoupled(new VLSUWriteVRFReq(ap))
  })

  io.uReq.valid := false.B
  io.uReq.bits := 0.U.asTypeOf(new VLdRequest(ap))
  io.robAck.valid := false.B
  io.robAck.bits := 0.U
  io.wakeUp.valid := false.B
  io.wakeUp.bits := 0.U
  io.allDone := false.B
  io.vrfReadReq.valid := false.B
  io.vrfReadReq.bits := 0.U.asTypeOf(new VLSUReadVRFReq(ap))
  io.vrfWriteReq.valid := false.B
  io.vrfWriteReq.bits := 0.U.asTypeOf(new VLSUWriteVRFReq(ap))
  /** This register record which req buffer entry hold our request. */
  val reg: ValidIO[VLdQEntryBundle] = RegInit(0.U.asTypeOf(Valid(new VLdQEntryBundle(ap))))

  io.entryId := id.U

  val sIdle :: sWaitRs :: sCopyStale :: sSplitting :: sWaitData :: sAllDone :: sWaitRetire :: Nil = Enum(7)
  val state: UInt = RegInit(sIdle)

  val isUnitStride = reg.bits.style.isUnitStride
  val isSegment = reg.bits.style.isSegment

  //val snippetInitializer = Module(new SnippetInitializer(ap))
  //snippetInitializer.io.ctrl := io.vuopDis.bits.uCtrlSig.accessStyle

  val snippetVMAdjuster = Module(new SnippetVectorMaskAdjuster(ap))
  snippetVMAdjuster.io.ctrl := io.vuopRR.bits.uCtrlSig.accessStyle
  snippetVMAdjuster.io.vm := io.vuopRR.bits.vm

  val vlAdjust = Module(new SnippetInitializer(ap))
  vlAdjust.io.ctrl := io.vuopRR.bits.uCtrlSig.accessStyle

  /** If the oldest un-commit non-unit-stride is older than us, hang. */
  val oldestNonUnitStrideIdx: ValidIO[UInt] = PickOldest(io.nonUnitStrideOHs, io.headPtr, io.tailPtr, ap.nVLdQEntries)
  val freeze = oldestNonUnitStrideIdx.valid && IsOlder(AgePriorityEncoder(io.nonUnitStrideOHs.asBools(), io.headPtr),
    id.U, io.headPtr)

  val requestSplitter = Module(new RequestSplitter(ap, true, id))
  requestSplitter.io.reg := reg.bits
  io.uReq.bits := requestSplitter.io.uReq.bits
  // Claim a new entry when dispatch.
  when(state === sIdle){
    when(io.vuopDis.valid){
      // Dispatch claims us a entry, need init and wait for rs(address).
      reg.bits.addr := 0.U
      reg.bits.preAddr := 0.U
      reg.bits.rs2 := 0.U
      //reg.bits.totalReq := snippetInitializer.io.totalRequest
      reg.bits.robIndex := io.vuopDis.bits.robIdx
      //reg.bits.finishMasks := snippetInitializer.io.initSnippet
      //reg.bits.wakeUpVec := VecInit(snippetInitializer.io.wakeVecInit.asBools())
      reg.bits.orderFail := false.B
      reg.bits.allSucceeded := false.B
      reg.bits.pRegVec := io.vuopDis.bits.vpdst
      reg.bits.style := io.vuopDis.bits.uCtrlSig.accessStyle
      reg.bits.reqCount := 0.U
      reg.bits.segmentCount := Mux(io.vuopDis.bits.uCtrlSig.accessStyle.isIndexed, io.vuopDis.bits.uCtrlSig.accessStyle.fieldIdx, 0.U)
      //reg.bits.totalSegments := snippetInitializer.io.totalSegment
      state := sWaitRs
    }
  }.elsewhen(state === sWaitRs){// Data is ready, start split
    when(io.vuopRR.valid){
      reg.bits.addr := io.vuopRR.bits.rs1
      /** indicates if addr is aligned to line address. */
      val offset: UInt = io.vuopRR.bits.rs1(ap.offsetBits - 1, 0)
      val alignedAddr: Bool = offset === 0.U
      val isUnitStride = reg.bits.style.isUnitStride || reg.bits.style.isWholeAccess
      reg.bits.wakeUpVec := VecInit(vlAdjust.io.wakeVecInit.asBools())
      reg.bits.totalSegments := vlAdjust.io.totalSegment
      reg.bits.totalReq := Mux(alignedAddr && isUnitStride, (ap.maxReqsInUnitStride - 1).U, vlAdjust.io.totalRequest)
      reg.bits.preAddr := io.vuopRR.bits.rs1
      reg.bits.vs1 := io.vuopRR.bits.vs1
      reg.bits.vs2 := io.vuopRR.bits.vs2
      reg.bits.rs2 := io.vuopRR.bits.rs2
      reg.bits.staleRegIdxVec := io.vuopRR.bits.staleRegIdxes
      reg.bits.finishMasks :=
        (snippetVMAdjuster.io.adjustedSnippet zip vlAdjust.io.initSnippet).map {case (vm, snippet) =>
           vm | snippet
        }
      val needFetchTail = !io.vuopRR.bits.uCtrlSig.accessStyle.vta
      val needFetchMask = !io.vuopRR.bits.uCtrlSig.accessStyle.vma
      /** Fetch all if */
      val needCopyStale = needFetchTail || needFetchMask
      when(needCopyStale){
        state := sCopyStale
      }.otherwise{
        state := sSplitting
      }
    }
  }.elsewhen(state === sCopyStale){
    val fieldCount = RegInit(0.U(4.W))
    val totalFields = Mux(reg.bits.style.vlmul(2), 1.U, (1.U << reg.bits.style.vlmul(1,0)).asUInt())
    io.vrfReadReq.valid := true.B
    io.vrfReadReq.bits.addr := reg.bits.staleRegIdxVec(fieldCount)
    val staleDataReg = RegInit(0.U.asTypeOf(Valid(UInt(ap.vLen.W))))

    when(io.vrfReadResp.valid){
      staleDataReg.valid := true.B
      staleDataReg.bits := io.vrfReadResp.bits.data
    }
    io.vrfWriteReq.valid := staleDataReg.valid
    io.vrfWriteReq.bits.addr := reg.bits.staleRegIdxVec(fieldCount)
    io.vrfWriteReq.bits.data := staleDataReg.bits
    io.vrfWriteReq.bits.byteMask := Fill(ap.vLenb, 1.U(1.W))
    when(io.vrfWriteReq.fire()){
      fieldCount := fieldCount + 1.U
      staleDataReg.valid := false.B
    }
    val copyDone = fieldCount === totalFields
    when(copyDone){
      state := sSplitting
      fieldCount := 0.U
    }
  }.elsewhen(state === sSplitting){
    io.uReq.valid := requestSplitter.io.uReq.valid && !freeze
    io.uReq.bits := requestSplitter.io.uReq.bits
    /** Ready is true only when valid is true, but unnecessary request does not trigger valid. Both shift to next. */
    val nextSplit: Bool = (io.uReq.ready || !io.uReq.valid) && !freeze
    val nextSeg: Bool = reg.bits.reqCount === reg.bits.totalReq - 1.U
    reg.bits.reqCount := Mux(nextSplit, Mux(nextSeg, 0.U, reg.bits.reqCount + 1.U), reg.bits.reqCount)
    reg.bits.preAddr := Mux(nextSplit, Mux(nextSeg, reg.bits.addr, requestSplitter.io.newAddr), reg.bits.preAddr)
    reg.bits.preSnippet := Mux(nextSplit, Mux(nextSeg, 0.U, requestSplitter.io.newSnippet), reg.bits.preSnippet)
    reg.bits.segmentCount := Mux(nextSplit && nextSeg, reg.bits.segmentCount + 1.U, reg.bits.segmentCount)
    val splitDone: Bool = (reg.bits.segmentCount === (reg.bits.totalSegments - 1.U)) && nextSeg && nextSplit
    when(splitDone){
      state := sWaitData
    }
    when(io.finishAck.valid && io.finishAck.bits.qEntryIdx === id.U){
      reg.bits.finishMasks(io.finishAck.bits.segmentIdx) := reg.bits.finishMasks(io.finishAck.bits.segmentIdx) | io.finishAck.bits.regAccessCS.finishMaskSnippet
    }
  }.elsewhen(state === sWaitData){
    when(io.finishAck.valid && io.finishAck.bits.qEntryIdx === id.U){
      reg.bits.finishMasks(io.finishAck.bits.segmentIdx) := reg.bits.finishMasks(io.finishAck.bits.segmentIdx) | io.finishAck.bits.regAccessCS.finishMaskSnippet
    }
    /** indicates we have some vreg to wake up, 1 is valid. */
    val needWakeUpVec = VecInit((reg.bits.finishMasks zip reg.bits.wakeUpVec).map { case (finish, wake) =>
      wake && finish.andR()
    }).asUInt()
    val needWakeUp: Bool = needWakeUpVec.orR()
    /** indicates which reg in this group needs wakeup, not the reg addr. */
    val wakeUpGroupIdx: UInt = OHToUInt(FindFirstOne(needWakeUpVec, 8))
    /** find out real reg addr to wake up. */
    val wakeUpRegIdx: UInt = reg.bits.pRegVec(wakeUpGroupIdx)
    when(needWakeUp){
      io.wakeUp.valid := true.B
      io.wakeUp.bits := wakeUpRegIdx
      when(io.wakeUp.fire()){
        reg.bits.wakeUpVec(wakeUpGroupIdx) := false.B
      }
    }
    val allFinished = !reg.bits.wakeUpVec.asUInt().orR()
    when(allFinished){
      io.robAck.valid := true.B
      io.robAck.bits := reg.bits.robIndex
      reg.bits.allSucceeded := true.B
      when(io.robAck.fire()){
        state := sWaitRetire
      }
    }
  }.elsewhen(state === sWaitRetire){
    // We have tell ROB that we have all data write, which means vec pipe may use our data.
    // If there is a order fail, pipeline has to be flushed.
    when(VecInit(io.fromRob.retireEntries.map{i => i.valid && i.bits.isLoad && i.bits.qEntryIdx === id.U}).asUInt().orR()){
      state := sIdle
      io.allDone := reg.bits.allSucceeded
      reg.valid := false.B
    }
  }
}