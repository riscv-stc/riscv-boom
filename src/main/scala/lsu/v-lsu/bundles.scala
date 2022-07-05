package boom.vlsu

import chisel3._
import chisel3.util._
import freechips.rocketchip.util.{DontTouch, GenericParameterizedBundle}
import scala.math._
import boom.util.{MaskLower}

/** IO between Rob and v-lsu
 *  v-lsu tell rob when an uop is all done
 *  rob tell v-lsu the uop is committed.
 */
class VLSUROBIO(ap: VLSUArchitecturalParams) extends VLSUBundle(ap) {
  val robIdx: Vec[ValidIO[UInt]] = Vec(ap.coreWidth, Valid(UInt(ap.robAddrSz.W)))
}
/** Rob tells vlsu to retire vuop. */
class ROBVLSUIO(ap: VLSUArchitecturalParams) extends VLSUBundle(ap) {
  /** tell vldq entry index to retire. */
  val retireEntries = Flipped(Vec(ap.retireWidth, Valid(new VectorLoadStoreCommit(ap))))
}
class VectorLoadStoreCommit(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val isLoad = Bool()
  val isStore = Bool()
  /** index of  */
  val qEntryIdx = UInt(max(ap.nVStQIndexBits, ap.nVLdQIndexBits).W)
}

/** IO between dispatch and v-lsu
 * dispatch uop to v-lsu waiting for data or address to arrive.
 * All instruction arrives via this port follow their program order.
 */
class DispatchVLSUIO(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val vuopDis: Vec[ValidIO[VLSMicroOP]] = Flipped(Vec(ap.coreWidth, Valid(new VLSMicroOP(ap))))
}
/** This should be wrapped in validio, indicates successfully allocated in vlsu. */
class DispatchAck(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val w = max(ap.nVLdQIndexBits, ap.nVStQIndexBits)
  val qIdx = UInt(w.W)
  val robIdx = UInt(ap.robAddrSz.W)
}
class VLSUDispatchIO(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val disVLdQIdx = Output(Vec(ap.coreWidth, Valid(new DispatchAck(ap))))
  val disVStQIdx = Output(Vec(ap.coreWidth, Valid(new DispatchAck(ap))))
  val vLdQFull = Output(Vec(ap.coreWidth, Bool()))
  val vStQFull = Output(Vec(ap.coreWidth, Bool()))
}

/** Instruction arrives via this port can be out-of-order. */
class RRVLSUIO(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val vuop: Vec[ValidIO[VLSMicroOP]] = Flipped(Vec(ap.coreWidth, Valid(new VLSMicroOP(ap))))
}

/** Entry of an entire vector load uop. */
class VLdQEntryBundle(ap: VLSUArchitecturalParams) extends LoadStoreQueueEntryBundleBase(ap){
  /** 1 means need to wake up core pipeline. 0 means do nothing or done. */
  val wakeUpVec = Vec(8, Bool())

  val staleRegIdxVec = Vec(8, UInt(ap.vpregSz.W))
}

class VStQEntryBundle(ap: VLSUArchitecturalParams) extends LoadStoreQueueEntryBundleBase(ap){
}
class LoadStoreQueueEntryBundleBase(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val addr: UInt = UInt(ap.coreMaxAddrBits.W)
  val rs2: UInt = UInt(ap.xLen.W)
  val vs1: UInt = UInt(ap.vLen.W)
  /** For indexed load. */
  val vs2: UInt = UInt(ap.vLen.W)
  /** Indicates address of previous split of element for constant stride. */
  val preAddr: UInt = UInt(ap.coreMaxAddrBits.W)
  val preSnippet: UInt = UInt(ap.vLenb.W)
  val style = new VectorAccessStyle(ap)
  val robIndex: UInt = UInt(ap.robAddrSz.W)
  val finishMasks: Vec[UInt] = Vec(8, UInt(ap.vLenb.W))
  val allSucceeded: Bool = Bool()
  /** Max segments is 8 */
  val segmentCount: UInt = UInt(4.W)
  val totalSegments: UInt = UInt(4.W)
  /** total request number within one dest vreg. */
  val totalReq: UInt = UInt(ap.vLenb.W)
  /** variable request count within one dest vreg. */
  val reqCount: UInt = UInt(ap.vLenb.W)
  /** dstPReg[0] is dest for non-segment load. */
  val pRegVec: Vec[UInt] = Vec(8, UInt(ap.vpregSz.W))

  val orderFail: Bool = Bool()

  val committed: Bool = Bool()
  /** Indicates which byte has been translated to paddr. All one means can clear busy. */
  val tlbMasks: Vec[UInt] = Vec(8, UInt(ap.vLenb.W))

  val brMask: UInt = UInt(ap.maxBrCount.W)
  // appended for mle and mse instructions
  val ridx: UInt = UInt(ap.vpregSz.W)
  val sidx: UInt = UInt(ap.vLenSz.W)
  val tt:   UInt = UInt(2.W)
  val isTile = Bool()
}


/** hold information for a vector uop. */
class VLSMicroOP(ap: VLSUArchitecturalParams) extends VLSUBundle(ap) {
  val vLdQIdx: UInt = UInt(ap.nVLdQIndexBits.W)
  val vStQIdx: UInt = UInt(ap.nVStQIndexBits.W)
  val robIdx: UInt = UInt(ap.robAddrSz.W)
  val uCtrlSig: VLSUMicroControlSignal = new VLSUMicroControlSignal(ap)
  val vs1: UInt = UInt(ap.vLen.W)
  val vs2: UInt = UInt(ap.vLen.W)
  val rs1: UInt = UInt(ap.xLen.W)
  val rs2: UInt = UInt(ap.xLen.W)
  val vm: UInt = UInt(ap.vLen.W)
  val vpdst: Vec[UInt] = Vec(8, UInt(ap.vpregSz.W))
  val staleRegIdxes: Vec[UInt] = Vec(8, UInt(ap.vpregSz.W))
  val brMask: UInt = UInt(ap.maxBrCount.W)
  // appended for mle
  val ridx: UInt = UInt(ap.vpregSz.W)
  val sidx: UInt = UInt(ap.vLenSz.W)
  val tt:   UInt = UInt(2.W)
  val isTile = Bool()
}

class VecRequest(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val address: UInt = UInt(ap.addressBits.W)
  def lineAddress = address(ap.coreMaxAddrBits - 1, ap.offsetBits)
  val regAccessCS = new RegAccessControlSignal(ap)
  val style = new VectorAccessStyle(ap)
  val reqBufferIdx: UInt = UInt(max(ap.nVLdReqBufferIdxBits,ap.nVStReqBufferIdxBits).W)
  /** Accessing start point inside the cache line. */
  val lineStartIndex: UInt = UInt(log2Ceil(ap.cacheLineByteSize).W)

  /** Address is translated via TLB. */
  val addressIsPhysical: Bool = Bool()

  /** Indicates whether this request is under processing. */
  val executing: Bool = Bool()

  /** Indicates whether this request is finished. */
  val done: Bool = Bool()
  /** Indicates which segment reg is processing, 0 for non-segment. */
  val segmentIdx = UInt(3.W)

  /** This load request is observed as probed in L2 cache. */
  val observed: Bool = Bool()
  /** Proceeding */
  val orderFail: Bool = Bool()

  val qEntryIdx: UInt = UInt(max(ap.nVLdQIndexBits, ap.nVStQIndexBits).W)
  /** This store request is committed in ROB. Which means it can be issued to memory. */
  val committed: Bool = Bool()

  val brMask: UInt = UInt(ap.maxBrCount.W)

  // appended for mle
  val ridx: UInt = UInt(ap.vpregSz.W)
  val sidx: UInt = UInt(ap.vLenSz.W)
  val tt:   UInt = UInt(2.W)
  val isTile = Bool()

  def UnitStrideSnippetsCalculator(offset: UInt): (UInt, UInt, UInt) = {
    val offsetLeft = ap.cacheLineByteSize.U - offset
    val head = Wire(UInt(ap.vLenb.W))
    val tail = Wire(UInt(ap.vLenb.W))
    val body = Wire(UInt(ap.vLenb.W))
    head := (Fill(ap.cacheLineByteSize, 1.U(1.W)) >> offset).asUInt()
    tail := (Fill(ap.cacheLineByteSize, 1.U(1.W)) ## 0.U(ap.vLenb.W) >> offset).asUInt()(ap.vLenb - 1, 0)
    body := Fill(ap.vLenb, 1.U(1.W)) & (~(head | tail)).asUInt()
    (head, body, tail)
  }
  /** Extract index from vs2 according to voffset and eew. */
  def IndexExtractor(indexArray: UInt, IndexEew: UInt, vOffset: UInt): UInt = {
    val elementBytes: UInt = (1.U << IndexEew).asUInt()
    val vOffsetBits: UInt = (vOffset << IndexEew) ## 0.U(3.W)
    val indexRaw = indexArray >> vOffsetBits
    val indexByteSeq = Seq.tabulate(ap.vLenb)(i => i).map{i =>
      indexRaw(i*8+7, i*8)
    }
    val indexByteVec = VecInit(indexByteSeq)
    val elementMaxBytes: Int = ap.elementMaxBytes
    val elementByteMask = Wire(UInt(elementMaxBytes.W))
    elementByteMask := (1.U << elementBytes).asUInt() - 1.U
    val index = VecInit(elementByteMask.asBools().zipWithIndex.map{
      case (m, i) => Mux(m, indexByteVec(i), 0.U)
    }).asUInt()
    index
  }

  /** Check if the request is necessary according to init snippet. True is necessary.*/
  def RequestNecessaryCheck(reqSnippet: UInt, segmentCount: UInt, initSnippet: Vec[UInt]): (Bool, UInt) = {
    val oldSnippet = initSnippet(segmentCount)
    val necessary = ((reqSnippet ^ oldSnippet) & reqSnippet).orR()
    val newSnippet = reqSnippet & (~oldSnippet).asUInt()
    (necessary, newSnippet)
  }
  /** Access Example:
    * Snippet is for instructing write back stage after data returned,
    * even if the access data is misaligned to cache line address.
    * Each snippet is associated to one cache line.
    *          | line4  |  line3 |  line2 |  line1 |  line0 |
    * Memory:  |--------|--------|--------|--------|--------|
    * Vector:       |--------|--------|--------|--------|off|
    * accesses      |tail|-body--|--body--|--body--|head|   |
    * headSnippet:  |00000000|00000000|00000000|00001111|  base
    * bodySnippet:  |00000000|00000000|00001111|11110000|
    * bodySnippet:  |00000000|00001111|11110000|00000000|
    * bodySnippet:  |00001111|11110000|00000000|00000000|
    * tailSnippet:  |11110000|00000000|00000000|00000000|
    * Currently, we only support 0~1 body.
    */
  def UnitStride(addr: UInt,
                 reqCount: UInt,
                 dstPReg: UInt,
                 segmentCount: UInt,
                 isWholeAccess: Bool, isLoad: Boolean,
                 initialSnippet: Vec[UInt],
                 id: Int): (Bool, VecRequest) = {
    val out =
      if (isLoad) {
        val out = WireInit(0.U.asTypeOf(new VLdRequest(ap)))
        out.qEntryIdx := id.U
        out.observed := false.B
        out.orderFail := false.B
        out
      }
      else {
        val out = WireInit(0.U.asTypeOf(new VStRequest(ap)))
        out.qEntryIdx := id.U
        out.committed := false.B
        out
      }
    val isHead: Bool = reqCount === 0.U
    val isTail: Bool = reqCount === (ap.maxReqsInUnitStride - 1).U
    val offset: UInt = addr(ap.offsetBits - 1, 0)
    val (headSnippet, bodySnippet, tailSnippet) = UnitStrideSnippetsCalculator(offset)
    assert((tailSnippet | headSnippet | bodySnippet).andR(), "Wrong snippet result!")
    out.address := addr + (reqCount << ap.offsetBits).asUInt()
    out.segmentIdx := segmentCount(2,0)
    out.style.isUnitStride := true.B
    out.style.isIndexed := false.B
    out.style.isConstantStride := false.B
    out.style.isWholeAccess := isWholeAccess
    out.style.isSegment := false.B
    out.regAccessCS.snippetIsHead := isHead
    out.regAccessCS.snippetIsTail := isTail
    // eew is ineffective in unit stride
    out.style.dataEew := 0.U
    out.style.indexEew := 0.U
    out.regAccessCS.regIdx := dstPReg
    out.lineStartIndex := offset
    out.addressIsPhysical := false.B
    out.reqBufferIdx := 0.U
    val reqSnippet = Mux(isHead, headSnippet,
      Mux(isTail, tailSnippet,
        bodySnippet
      ))
    out.executing := false.B
    out.done := false.B
    val reqNecessary = RequestNecessaryCheck(reqSnippet, segmentCount, initialSnippet)
    out.regAccessCS.finishMaskSnippet := reqNecessary._2
    // appended for mle and mse
    out.ridx := ridx
    out.sidx := sidx
    out.tt   := tt
    (reqNecessary._1, out)
  }
  /** To avoid massive shift left, which is banned in chisel.
    * Here we use a recursive calculation of constant-stride
    * elements' address and snippet from previous split.
    */
  def ConstantStride(addr: UInt,
                     preAddr: UInt,
                     strideBytes: UInt,
                     dataEew: UInt,
                     dstPReg: UInt,
                     segmentCount: UInt,
                     reqCount: UInt,
                     preSnippet: UInt, isLoad: Boolean,
                     initialSnippet: Vec[UInt],
                     id: Int): (Bool, VecRequest, UInt, UInt) = {
    val out =
      if (isLoad) {
        val out = WireInit(0.U.asTypeOf(new VLdRequest(ap)))
        out.qEntryIdx := id.U
        out.observed := false.B
        out.orderFail := false.B
        out
      }
      else {
        val out = WireInit(0.U.asTypeOf(new VStRequest(ap)))
        out.qEntryIdx := id.U
        out.committed := false.B
        out
      }
    val offset = preAddr(ap.offsetBits - 1, 0)
    val elementAddr = Mux(reqCount=== 0.U , addr,preAddr + strideBytes)
    out.executing := false.B
    out.done := false.B
    out.style.isUnitStride := false.B
    out.style.isConstantStride := true.B
    out.style.isIndexed := false.B
    out.style.isSegment := false.B
    out.style.isWholeAccess := false.B
    val elementBytes: UInt = (1.U << dataEew(1,0)).asUInt()
    out.style.dataEew := dataEew
    out.style.indexEew := 0.U
    out.segmentIdx := segmentCount(2,0)
    out.regAccessCS.regIdx := dstPReg
    out.lineStartIndex := offset
    out.addressIsPhysical := false.B
    out.reqBufferIdx := 0.U
    out.address := elementAddr
    val initSnippet: UInt = (1.U << elementBytes).asUInt() - 1.U
    val elementSnippet = Mux(reqCount.orR(), (preSnippet << elementBytes).asUInt(), initSnippet)
    assert(PopCount(elementSnippet) === elementBytes, "1s in snippet should equal to eew bytes.")
    val reqNecessary = RequestNecessaryCheck(elementSnippet, segmentCount, initialSnippet)
    out.regAccessCS.finishMaskSnippet := reqNecessary._2
    (reqNecessary._1, out, elementAddr, elementSnippet)
  }
  def Indexed(addr: UInt,
              indexEew: UInt,
              dataEew: UInt,
              dstPReg: Vec[UInt],
              reqCount: UInt,
              indexRegIdx: UInt,
              preSnippet: UInt,
              indexArray: UInt, isLoad: Boolean,
              initialSnippet: Vec[UInt],
              id: Int): (Bool, VecRequest, UInt) = {
    val out =
      if (isLoad) {
        val out = WireInit(0.U.asTypeOf(new VLdRequest(ap)))
        out.qEntryIdx := id.U
        out.observed := false.B
        out.orderFail := false.B
        out
      }
      else {
        val out = WireInit(0.U.asTypeOf(new VStRequest(ap)))
        out.qEntryIdx := id.U
        out.committed := false.B
        out
      }
    val index: UInt = IndexExtractor(indexArray, indexEew, reqCount)
    val elementAddr: UInt = (addr.asSInt() + index.asSInt()).asUInt()
    val elementAddrOffset: UInt = elementAddr(ap.offsetBits - 1, 0)
    val dataElementBytes: UInt = (1.U << dataEew(1,0)).asUInt()
    val indexElementBytes: UInt = (1.U << indexEew(1,0)).asUInt()
    val nElementsPerDataReg: UInt = (ap.vLenb.U >> dataEew(1,0)).asUInt()
    val nElementsPerIndexReg: UInt = (ap.vLenb.U >> indexEew(1,0)).asUInt()

    val largerData = dataEew(1,0) > indexEew(1,0)
    val dataExpandRate = dataEew(1,0) - indexEew(1,0)
    val largerIndex = indexEew(1,0) > dataEew(1,0)
    val dataShrinkRate = indexEew(1,0) - dataEew(1,0)
    val equal = indexEew === dataEew
    val affectMultiReg = largerData

    val shrinkHalf = dataShrinkRate === 1.U
    val shrinkQuarter = dataShrinkRate === 2.U
    val shrinkEighth = dataShrinkRate === 3.U
    val expandDouble = dataExpandRate === 1.U
    val expandQuarter = dataExpandRate === 2.U
    val expandOctuple = dataExpandRate === 3.U

    val dataRegIdxShrink = Mux(shrinkHalf, indexRegIdx(2,1), Mux(shrinkQuarter, indexRegIdx(2), 0.U))

    val dataRegIdxExpandBase: UInt = Mux(expandOctuple, 0.U, Mux(expandDouble, indexRegIdx(1,0) << 1, indexRegIdx(0) << 2)).asUInt()
    val dataRegIdxExpandOffset: UInt = reqCount >> (log2Ceil(ap.vLenb).U - dataEew(1,0))
    val dataRegIdxExpand: UInt = dataRegIdxExpandBase + dataRegIdxExpandOffset
    /** Target reg idx in the group, not idx in vrf. */
    val dataRegIdx = Mux(equal, indexRegIdx, Mux(largerIndex, dataRegIdxShrink, dataRegIdxExpand))

    val initSnippetEqualExpand: UInt = (1.U << dataElementBytes).asUInt() - 1.U
    val initSnippetShrinkHalf: UInt = VecInit(UIntToOH(indexRegIdx(0), 2).asBools().map(b =>
      Mux(b, initSnippetEqualExpand | 0.U((ap.vLenb/2).W), 0.U((ap.vLenb/2).W)))).asUInt()
    val initSnippetShrinkQuarter: UInt = VecInit(UIntToOH(indexRegIdx(1,0)).asBools().map(b =>
      Mux(b, initSnippetEqualExpand | 0.U((ap.vLenb/4).W), 0.U((ap.vLenb/4).W)))).asUInt()
    val initSnippetShrinkEighth: UInt = VecInit(UIntToOH(indexRegIdx, 8).asBools().map(b =>
      Mux(b, initSnippetEqualExpand | 0.U((ap.vLenb/8).W), 0.U((ap.vLenb/8).W)))).asUInt()
    val initSnippetShrink: UInt = Mux(shrinkHalf, initSnippetShrinkHalf,
      Mux(shrinkQuarter, initSnippetShrinkQuarter, initSnippetShrinkEighth))

    val reqCountOffset: UInt = reqCount & MaskLower((ap.vLenb.U >> dataEew(1,0)).asUInt)
    val initSnippet: UInt = Mux(largerIndex, initSnippetShrink, initSnippetEqualExpand)
    val elementSnippet: UInt = Mux(reqCountOffset.orR(), (preSnippet << dataElementBytes).asUInt(), initSnippet)
    out.address := elementAddr
    out.executing := false.B
    out.done := false.B
    /** In indexed processing, this indicates regIdx of data. */
    out.segmentIdx := dataRegIdx
    out.style.isUnitStride := false.B
    out.style.isConstantStride := false.B
    out.style.isIndexed := true.B
    out.style.isSegment := false.B
    out.style.isWholeAccess := false.B
    out.style.indexEew := indexEew
    out.style.dataEew := dataEew
    out.regAccessCS.regIdx := dstPReg(dataRegIdx)
    out.addressIsPhysical := false.B
    out.lineStartIndex := elementAddrOffset
    out.addressIsPhysical := false.B
    out.reqBufferIdx := 0.U
    val reqNecessary = RequestNecessaryCheck(elementSnippet, dataRegIdx, initialSnippet)
    out.regAccessCS.finishMaskSnippet := reqNecessary._2
    (reqNecessary._1, out, elementSnippet)
  }
}

/** This bundle instructs where and how to access vector RF. */
class RegAccessControlSignal(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){

  val regIdx: UInt = UInt(ap.nVRegIndex.W)

  val snippetIsHead = Bool()
  val snippetIsTail = Bool()
  /** Indicates which part of a vector register. Is for masking after done. */
  val finishMaskSnippet: UInt = UInt(ap.vLenb.W)
}

/** Request bundle for a cache line size load. */
class VLdRequest(ap: VLSUArchitecturalParams) extends VecRequest(ap){

}

/** Request bundle for a cache line size store. */
class VStRequest(ap: VLSUArchitecturalParams) extends VecRequest(ap){
}

/** Top io bundle. */
class VLSUTopBundle(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){

  val brUpdate = Input(new BranchUpdateInfo(ap))
  /** VRFIO */
  val toVrf:   VLSUVRFIO = new VLSUVRFIO(ap)
  val fromVrf: VRFVLSUIO = new VRFVLSUIO(ap)

  /** Dispatch stage to VLSU. */
  val fromDis: DispatchVLSUIO = new DispatchVLSUIO(ap)
  val toDis: VLSUDispatchIO = new VLSUDispatchIO(ap)

  /** Register-read stage to VLSU. */
  val fromRr: RRVLSUIO = new RRVLSUIO(ap)

  /** VLSU to ROB IO. */
  val ldToRob: VLSUROBIO = new VLSUROBIO(ap)
  val stToRob: VLSUROBIO = new VLSUROBIO(ap)
  // val wakeUpVReg = ValidIO(UInt(ap.vpregSz.W))
  val wakeUpVReg = ValidIO(new WakeUpInfo(ap))
  val fromRob = new ROBVLSUIO(ap)
  val vrfBusyStatus = Input(UInt(ap.nVRegs.W))
}

abstract class VLSUBundle(ap: VLSUArchitecturalParams) extends GenericParameterizedBundle(ap)
  with HasVLSUArchParameters {
}

trait HasVLSUArchParameters {
}

class VLSUReadVRFReq(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val addr: UInt = UInt(ap.vpregSz.W)
  // appended for mle
  val sidx: UInt = UInt(ap.vLenSz.W)
  val tt  : UInt = UInt(2.W)
  val isTile = Bool()
}
class VLSUReadVRFResp(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val data: UInt = UInt(ap.vLen.W)
}
class VLSUWriteVRFReq(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val addr: UInt = UInt(ap.vpregSz.W)
  val byteMask: UInt = UInt(ap.vLenb.W)
  val data: UInt = UInt(ap.vLen.W)
  // appended for mle
  val sidx: UInt = UInt(ap.vLenSz.W)
  val tt  : UInt = UInt(2.W)
  val isTile = Bool()
}
class VLSUVRFIO(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val write = ValidIO(new VLSUWriteVRFReq(ap))
  val readReq = ValidIO(new VLSUReadVRFReq(ap))
}
class VRFVLSUIO(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val readResp = Flipped(ValidIO(new VLSUReadVRFResp(ap)))
}

abstract class VLSUModules(ap: VLSUArchitecturalParams) extends Module with DontTouch{
  def lineAddress(address: UInt): UInt = {
    (address >> ap.offsetBits) ## 0.U(ap.offsetBits.W)
  }
}

/** From uopc to this control signal, these signal can be passed in directly or decoded  */
class VLSUMicroControlSignal(ap: VLSUArchitecturalParams) extends VLSUBundle(ap) {
  val accessStyle: VectorAccessStyle = new VectorAccessStyle(ap)
  val accessType: VectorAccessType = new VectorAccessType
}
class VectorAccessStyle(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val isUnitStride: Bool = Bool()
  val isIndexed: Bool = Bool()
  val isConstantStride: Bool = Bool()
  val isSegment: Bool = Bool()
  val isWholeAccess: Bool = Bool()
  val dataEew: UInt = UInt(2.W)
  val indexEew: UInt = UInt(2.W)
  val vStart: UInt = UInt(ap.vLenSz.W)
  val vl: UInt = UInt(ap.vlMax.W)
  val vlmul: UInt = UInt(3.W)
  val indexLmul = UInt(3.W)
  val nf: UInt = UInt(3.W)
  /** 0 means need fetch old data for masked elements. */
  val vma: Bool = Bool()
  /** 0 means need fetch old data for tail elements. */
  val vta: Bool = Bool()
  /** indicates which of this vuop from split uop at dispatch stage. For indexed only.*/
  val fieldIdx: UInt = UInt(3.W)
}
class VectorAccessType extends Bundle{
  val isLoad: Bool = Bool()
  val isStore: Bool = Bool()
}
/** From MSHR to write back controller. */
class VLdWriteBackRequest(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val req = new VLdRequest(ap)
  val data = UInt(ap.cacheLineBits.W)
}

class VTLBReq(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val vaddr = UInt(ap.coreMaxAddrBits.W)
  val queueIdx = UInt(max(ap.nVLdQIndexBits,ap.nVStQIndexBits).W)
  val isLoad = Bool()
  val reqBufferIdx = UInt(ap.nVLdReqBufferIdxBits.W)
  val snippet = UInt(ap.vLenb.W)
  val segmentIdx = UInt(3.W)
}
class VTLBResp(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val paddr = UInt(ap.coreMaxAddrBits.W)
  val queueIdx = UInt(max(ap.nVLdQIndexBits,ap.nVStQIndexBits).W)
  val isLoad = Bool()
  val reqBufferIdx = UInt(ap.nVLdReqBufferIdxBits.W)
  val snippet = UInt(ap.vLenb.W)
  val hit = Bool()
  val exception = Bool()
  val segmentIdx = UInt(3.W)
}
class AddressCheckerResp(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val vldReqBufferIdx = UInt(ap.nVLdReqBufferIdxBits.W)
}

class SMSHRStatus(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val awaiting = Bool()
  val addr = UInt(ap.coreMaxAddrBits.W)
}

/** import from boom core pipeline. [[boom.exu.BrUpdateMasks]] */
class BranchUpdateMasks(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val resolveMask = UInt(ap.maxBrCount.W)
  val mispredictMask = UInt(ap.maxBrCount.W)
}
/** import [[boom.exu.BrResolutionInfo]] */
class BranchResolutionInfo(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val vLdqIdx = UInt(ap.nVLdQIndexBits.W)
  val vStqIdx = UInt(ap.nVStQIndexBits.W)
  val mispredicted = Bool()
}
/** import [[boom.exu.BrUpdateInfo]] */
class BranchUpdateInfo(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val b1 = new BranchUpdateMasks(ap)
  val b2 = new BranchResolutionInfo(ap)
}