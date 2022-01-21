package boom.vlsu

import chisel3._
import chisel3.util._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.util._
import boom.common._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink.{TLClientNode, TLMasterParameters, TLMasterPortParameters}

/* To prevent 'def' from generating too much unreadable verilog lines,
 * which ruins the whole readability of the calling module.
 * we wrap functions into modules.
 */


/** Find first one
 * Starting with entry zero, if a one if found, that one is passed through
 * any other following ones are gated.
 * @example:
 * 0100110 => 0000010
 */
class FindFirstOne(width: Int) extends Module {
  val io = IO(new Bundle {
    val input = Input(UInt(width.W))
    val result = Output(UInt(width.W))
  })
  val leftOrBits = leftOR(io.input)
  io.result := leftOrBits ^ (leftOrBits << 1)(leftOrBits.getWidth - 1, 0)
}

object FindFirstOne {
  def apply(input: UInt, width: Int): UInt = {
    val ffone = Module(new FindFirstOne(width))
    ffone.io.input := input
    ffone.io.result
  }
}

/** Combined logics to parse uop from core pipeline into vlsu uop.
 * Distract necessary information for vector load/store
 */
class UopParser(ap: VLSUArchitecturalParams) extends VLSUModules(ap){
  val io = IO(new Bundle{
    val input = Input(new VLSMicroOP(ap))
    val output = Output(new VLSMicroOP(ap))
  })

}
/** This module stretches data of a cache line into vlen width format according to
 * request snippet. */
class UnitStrideDataStretcher(ap: VLSUArchitecturalParams) extends VLSUModules(ap){
  val io = IO(new Bundle{
    val lineDataIn = Input(UInt(ap.cacheLineBits.W))
    val reqWB = Input(new VLdRequest(ap))
    val vrfWriteReq = Output(new VLSUWriteVRFReq(ap))
  })
  val offset: UInt = io.reqWB.address(ap.offsetBits - 1, 0)
  val alignedAddr = !offset.orR()
  val bodyChunks: Int = (ap.vLenb / ap.cacheLineByteSize) - 1
  require(bodyChunks < 2, "body too long to support.")
  io.vrfWriteReq.addr := io.reqWB.regAccessCS.regIdx
  val snippet: UInt = io.reqWB.regAccessCS.finishMaskSnippet
  io.vrfWriteReq.byteMask := snippet
  val isHead: Bool = io.reqWB.regAccessCS.snippetIsHead
  val isTail: Bool = io.reqWB.regAccessCS.snippetIsTail
  /** Number of bytes left from offset to end of cache line. */
  val offsetLeft: UInt = ap.cacheLineByteSize.U - offset
  val offsetBits = (offset << 3).asUInt()
  val headData: UInt = (io.lineDataIn >> offsetBits).asUInt()
  val rawData = io.lineDataIn ## 0.U(ap.vLen.W)
  val bodyData: UInt = ((rawData >> (bodyChunks * ap.cacheLineBits)) >> offsetBits).asUInt()
  val tailData: UInt = Mux(alignedAddr, bodyData, (rawData >> offsetBits)(ap.vLen - 1, 0))
  io.vrfWriteReq.data := Mux(isHead, headData, Mux(isTail, tailData, bodyData))
}

/** dedicated arbitrator for split vector access requests. Winner from last arb always wins. */
class RequestArbitrator[req <: Data](ap: VLSUArchitecturalParams, request: req, isLoad: Boolean) extends VLSUModules(ap){
  val outputWidth: Int = ap.coreWidth
  val inputWidth: Int = if (isLoad) ap.nVLdQEntries else ap.nVStQEntries
  val idxBits: Int = log2Ceil(inputWidth)
  val io = IO(new Bundle{
    val inputReq: Vec[DecoupledIO[req]] = Flipped(Vec(inputWidth, Decoupled(request)))
    val outputReq: Vec[DecoupledIO[req]] = Vec(outputWidth, Decoupled(request))
  })
  val winnersIdx: Vec[ValidIO[UInt]] = RegInit(0.U.asTypeOf(Vec(outputWidth, Valid(UInt(idxBits.W)))))
  val inputVlds = VecInit(io.inputReq.map(_.valid)).asUInt()

  var winners = WireInit(0.U(outputWidth.W))
  var players = inputVlds
  var winnerThisSlot = WireInit(0.U(outputWidth.W))
  var winnersLastCycle = WireInit(0.U(outputWidth.W))
  winnersLastCycle = RegNext(VecInit(io.outputReq.map(_.valid)).asUInt())
  val winnerIdxVec = Wire(Vec(outputWidth, UInt(idxBits.W)))
  for (w <- 0 until outputWidth){
    val winnerStillIn: UInt = FindFirstOne(winnersLastCycle & players, inputWidth)
    winnerThisSlot = Mux(winnerStillIn.orR(), winnerStillIn, FindFirstOne(players, inputWidth))
    val winnerIndex = OHToUInt(winnerThisSlot)
    winnerIdxVec(w) := winnerIndex
    io.outputReq(w).valid := winnerThisSlot.orR() && io.inputReq(winnerIndex).valid
    io.outputReq(w).bits := io.inputReq(winnerIndex).bits
    players = players & (~winnerThisSlot).asUInt()
    winners = winners | winnerThisSlot
  }
  io.inputReq.zipWithIndex.foreach{ case (qInput, i) =>
    val wonOH = VecInit(winnerIdxVec.map(_ === i.U)).asUInt()
    val wonIdx = OHToUInt(wonOH)
    qInput.ready := winners(i) && io.outputReq(wonIdx).ready
  }
}

/** return oldest entry according given width and head pointer. */
class PickOldest(width: Int) extends Module{
  val io = IO(new Bundle{
    val reqVec = Input(UInt(width.W))
    val headPtr = Input(UInt(log2Ceil(width).W))
    val tailPtr = Input(UInt(log2Ceil(width).W))
    val result = Valid(UInt(log2Ceil(width).W))
  })
  io.result := 0.U.asTypeOf(Valid(UInt(log2Ceil(width).W)))
  val head = io.headPtr
  val tail = io.tailPtr
  // if input is zero, then there is none to pick.
  io.result.valid := io.reqVec.orR()
  when(head > tail){
    io.result.bits := OHToUInt(FindFirstOne(io.reqVec & (~rightOR(io.headPtr)).asUInt(), width))
  }.elsewhen(head < tail){
    io.result.bits := OHToUInt(FindFirstOne(io.reqVec, width))
  }
}
object PickOldest{
  def apply(reqVec: UInt, headPtr: UInt, tailPtr: UInt, inputWidth: Int): Valid[UInt] = {
    val picker = Module(new PickOldest(inputWidth))
    picker.io.reqVec := reqVec
    picker.io.headPtr := headPtr
    picker.io.tailPtr := tailPtr
    picker.io.result
  }
}
/** This arbiter pick up a number of input valid, and transfer them into UInt index. */
class IndexArbiter(inputWidth: Int, outputWidth: Int) extends Module{
  val indexWidth = log2Ceil(inputWidth)
  val io = IO(new Bundle{
    val input = Input(UInt(inputWidth.W))
    val output = Output(Vec(outputWidth, Valid(UInt())))
  })

  val winnerIndexes = 0.U.asTypeOf(Vec(outputWidth, Valid(UInt(indexWidth.W))))
  var players = (~io.input).asUInt()
  for(i <- 0 until outputWidth){
    val winnerThisSlot = FindFirstOne(players, inputWidth)
    players = players & (~winnerThisSlot).asUInt()
    winnerIndexes(i).bits := OHToUInt(winnerThisSlot)
  }
  val idlesCounts = PopCount(~io.input)
  winnerIndexes.map(_.valid).zipWithIndex.foreach{ case (v, i) =>
    v := i.U < idlesCounts
  }
  io.output := winnerIndexes

}
/** First generate new data to write then merge new data with pre data. */
class UnitStrideStoreLineFactory(ap: VLSUArchitecturalParams) extends VLSUModules(ap){
  val io = IO(new Bundle{
    val isHead = Input(Bool())
    val isTail = Input(Bool())
    val storeSnippet = Input(UInt(ap.vLenb.W))
    val address = Input(UInt(ap.coreMaxAddrBits.W))
    val storeData = Input(UInt(ap.vLen.W))
    val lineData = Output(UInt(ap.cacheLineBits.W))
    val lineMask = Output(UInt(ap.cacheLineByteSize.W))
  })
  val isHead: Bool = io.isHead
  val isTail: Bool = io.isTail
  val hasBody: Boolean = (ap.vLen / ap.cacheLineBits) > 1
  val offset: UInt = io.address(ap.offsetBits - 1, 0)
  val alignedAddr = !offset.orR()
  val offsetLeft: UInt = ap.cacheLineByteSize.U - offset
  val vData: UInt = io.storeData
  val vMask: UInt = io.storeSnippet
  val headData = ((vData ## 0.U(ap.cacheLineBits.W)) >> (offsetLeft << 3))(ap.cacheLineBits - 1, 0)
  val bodyData: UInt = if (hasBody) (vData >> (offsetLeft << 3).asUInt())(ap.cacheLineBits - 1, 0) else 0.U
  val tailData: UInt = if (hasBody) Mux(alignedAddr, bodyData, ((vData >> (offsetLeft << 3).asUInt()) >> ap.cacheLineBits.U).asUInt()(ap.cacheLineBits - 1, 0))
                       else (vData >> (offsetLeft << 3).asUInt()).asUInt()(ap.cacheLineBits - 1, 0)
  val headMask: UInt = ((vMask ## 0.U(ap.cacheLineByteSize.W)) >> offsetLeft)(ap.cacheLineByteSize - 1, 0)
  val bodyMask: UInt = if (hasBody) Fill(ap.cacheLineByteSize, 1.U(1.W)) else 0.U
  val tailMask: UInt = if (hasBody) Mux(alignedAddr, bodyMask, ((vMask >> offsetLeft).asUInt() >> ap.cacheLineByteSize).asUInt()(ap.cacheLineByteSize - 1, 0))
                       else (vMask >> offsetLeft).asUInt()(ap.cacheLineByteSize - 1, 0)

  io.lineData := Mux(isHead, headData, Mux(isTail, tailData, bodyData))
  io.lineMask := Mux(isHead, headMask, Mux(isTail, tailMask, bodyMask))
}

/** Construct a line with single element snippet. */
class ElementStoreLineFactory(ap: VLSUArchitecturalParams) extends VLSUModules(ap){
  val io = IO(new Bundle{
    val storeSnippet = Input(UInt(ap.vLenb.W))
    val eew = Input(UInt(3.W))
    val address = Input(UInt(ap.coreMaxAddrBits.W))
    val storeData = Input(UInt(ap.vLen.W))
    val lineData = Output(UInt(ap.cacheLineBits.W))
    val lineMask = Output(UInt(ap.cacheLineByteSize.W))
  })
  val elementBytes: UInt = (1.U << io.eew(1, 0)).asUInt()
  val offset: UInt = io.address(ap.offsetBits - 1, 0)
  /** Indicates start byte in vreg that is to store. */
  val vStartIdx: UInt = OHToUInt(FindFirstOne(io.storeSnippet, ap.vLenb))
  val elementBitMask: UInt = (1.U << (elementBytes << 3.U).asUInt()).asUInt() - 1.U
  val elementData: UInt = (io.storeData >> (vStartIdx << 3.U).asUInt()).asUInt() & elementBitMask
  val offsetLeft: UInt = ap.cacheLineByteSize.U - offset
  val data: UInt = ((elementData ## 0.U(ap.cacheLineBits.W)) >> (offsetLeft << 3.U).asUInt()).asUInt()
  io.lineData := data
  val elementByteMask: UInt = (1.U << elementBytes).asUInt() - 1.U
  val mask: UInt = ((elementByteMask ## 0.U(ap.cacheLineByteSize.W)) >> offsetLeft).asUInt()
  io.lineMask := mask
}
/** in data would overlap old data. */
class LineDataMerger(ap: VLSUArchitecturalParams) extends VLSUModules(ap){
  val io = IO(new Bundle{
    /** newer incoming data that would merge into old data.*/
    val inData = Input(UInt(ap.cacheLineBits.W))
    val inMask = Input(UInt(ap.cacheLineByteSize.W))
    val oldData = Input(UInt(ap.cacheLineBits.W))
    val oldMask = Input(UInt(ap.cacheLineByteSize.W))
    val newData = Output(UInt(ap.cacheLineBits.W))
    val newMask = Output(UInt(ap.cacheLineByteSize.W))
  })
  val inMask = io.inMask.asBools()
  val oldMask = io.oldMask.asBools()
  io.newMask := io.inMask | io.oldMask
  val newData = VecInit((inMask zip oldMask).zipWithIndex.map {case ((in, old), i) =>
    val inByte = io.inData(i * 8 + 7, i * 8)
    val oldByte = io.oldData(i * 8 + 7, i * 8)
    Mux(in, inByte, Mux(old, oldByte, 0.U(8.W)))
  }).asUInt()
  io.newData := newData
}

/** Construct vlen data to write back according to incoming line data and vld req. */
class ElementDataStretcher(ap: VLSUArchitecturalParams) extends VLSUModules(ap){
  val io = IO(new Bundle{
    val reqWB = Input(new VLdRequest(ap))
    val lineDataIn = Input(UInt(ap.cacheLineBits.W))
    val vrfWriteReq = Output(new VLSUWriteVRFReq(ap))
  })
  val len = (1.U << io.reqWB.style.eew(1,0)).asUInt()
  val snippet = io.reqWB.regAccessCS.finishMaskSnippet
  io.vrfWriteReq.byteMask := snippet
  val offset = io.reqWB.address(ap.offsetBits - 1, 0)
  val vStartIdx = OHToUInt(FindFirstOne(snippet, ap.vLenb))
  val elementMask: UInt = (1.U << (len << 3.U).asUInt()).asUInt() - 1.U
  val elementData: UInt = (io.lineDataIn >> (offset << 3.U).asUInt()).asUInt() & elementMask
  /** attach vlen 0s on the right side, then right shift to correct place. */
  val data: UInt = ((elementData ## 0.U(ap.vLen.W)) >> ((ap.vLenb.U - vStartIdx) << 3.U).asUInt()).asUInt()
  io.vrfWriteReq.data := data
  io.vrfWriteReq.addr := io.reqWB.regAccessCS.regIdx

}

/** Construct snippet on first allocate into queue entry according vl, lmul, eew, nf and vStart */
class SnippetInitializer(ap: VLSUArchitecturalParams) extends VLSUModules(ap){
  val io = IO(new Bundle{
    val ctrl = Input(new VectorAccessStyle(ap))
    val initSnippet = Output(Vec(8, UInt(ap.vLenb.W)))
    val totalSegment = Output(UInt(4.W))
    val totalRequest = Output(UInt((log2Ceil(ap.vLenb) + 1).W))
  })
  val isSegment = io.ctrl.isSegment
  val elementBytes = (1.U << io.ctrl.eew(1, 0)).asUInt()
  /** 00 => 1, 01 => 2, 10 => 4, 11 => 8 */
  val lmul = io.ctrl.vlmul(1, 0)
  val lmulValue = (1.U << lmul).asUInt()
  def lmul2: Bool = WireInit(lmul === 1.U)
  def lmul4: Bool = WireInit(lmul === 2.U)
  def lmul8: Bool = WireInit(lmul === 3.U)
  /** lmul == 1 */
  def lmul1: Bool = WireInit(lmul === 0.U)
  /** lmul > 1 */
  val lmulLagerThanOne = !io.ctrl.vlmul(2) && lmul.orR()
  /** lmul < 1 */
  val lmulSmallerThanOne = io.ctrl.vlmul(2)
  /** lmul >= 1 */
  val lmulLargerEqualOne = !io.ctrl.vlmul(2)
  /** lmul <= 1 */
  val lmulSmallerEqualOne = lmul1 || lmulSmallerThanOne
  val nFields = io.ctrl.nf

  /** if register is valid in pure nf. */
  val nfVld: Int => Bool = (i: Int) => i.U < nFields
  val allOnes = Fill(ap.vLenb,1.U)
  /** if register is valid in lmul and nf combinations. */
  val segmentVld: Int => Bool = (i: Int) => i.U < (nFields * lmulValue)
  val shrinkRate = 4.U - lmul
  /** byte index of vstart */
  val startByte = io.ctrl.vStart * elementBytes
  /** Indicates if this vlen is valid under current vlmul. parameter is segment index. */
  val lmulVld: Int => Bool = (pregIdx: Int) => pregIdx.U < (1.U << lmul).asUInt()
  /** end byte index of vl */
  val endBytevl = (io.ctrl.vl * elementBytes).asUInt()
  /** Calculate end byte index of this vlen when is in one register group. parameter is segment index. */
  val endByteLmulX: Int => UInt = (pregIdx: Int) => WireInit(
    Mux(lmulLagerThanOne && lmulVld(pregIdx), (ap.vLenb.U * (pregIdx + 1).U).asUInt(),
      Mux(lmul1, ap.vLenb.U, (ap.vLenb.U >> shrinkRate).asUInt())))
  /** Compare two end byte index and pick smaller one. parameter is segment index. */
  val endByteIdx: Int => UInt = (pregIdx: Int) => WireInit(
    Mux(endByteLmulX(pregIdx) > endBytevl, endBytevl, endByteLmulX(pregIdx)))
  /** 0 means this byte needs to be accessed. parameter is byte index.*/
  val byteVldX: (Int, UInt) => Bool = (byteIdx: Int, endByteIdx: UInt) => WireInit(
    byteIdx.U < startByte || byteIdx.U >= endByteIdx)
  /** snippet when this reg is under same group with previous register. parameter is segment idx.*/
  val snippetX: Int => UInt = (pregIdx: Int) => WireInit(
    VecInit(Seq.tabulate(ap.vLenb)(byteIdx => byteIdx + pregIdx * ap.vLenb).map(byteIdx => byteVldX(byteIdx, endByteIdx(pregIdx)))).asUInt())

  /** Init Snippet output table, because of segment, latter register may duplicate snippet of previous register.
   *  |snippet|lmul1|lmul2|luml4|lmul8|
   *  |   0   |  0  |  0  |  0  |  0  |
   *  |   1   |  0  |  1  |  1  |  1  |
   *  |   2   |  0  |  0  |  2  |  2  |
   *  |   3   |  0  |  1  |  3  |  3  |
   *  |   4   |  0  |  0  |  0  |  4  |
   *  |   5   |  0  |  1  |  1  |  5  |
   *  |   6   |  0  |  0  |  2  |  6  |
   *  |   7   |  0  |  1  |  3  |  7  |
   *  output snippet can look up to this table when segment and lmul larger equal one, when it is accessed by nf.
   * */
  io.initSnippet.zipWithIndex.foreach { case (out, i) =>
    val snippet = snippetX(0)
    val idxMod4: Int = i % 4
    val idxMod2: Int = i % 2
    out := Mux(io.ctrl.isWholeAccess && nfVld(i), 0.U, Mux(!isSegment && lmulVld(i), snippetX(i),
      Mux(isSegment && lmul1 && segmentVld(i), snippet,
        Mux(isSegment && lmul2 && segmentVld(i), snippetX(idxMod2),
          Mux(isSegment && lmul4 && segmentVld(i), snippetX(idxMod4),
            allOnes)))))
  }
  //4 bits, up to 'h8.
  io.totalSegment := Mux(io.ctrl.isWholeAccess, nFields, Mux(isSegment, lmulValue * nFields, lmulValue))
  val denseAccess = (io.ctrl.isUnitStride && !isSegment) || io.ctrl.isWholeAccess
  io.totalRequest := Mux(denseAccess, ap.maxReqsInUnitStride.U, (ap.vLenb.U >> io.ctrl.eew).asUInt())
}

/** Construct active snippet for element access according to vm and lmul, nf. */
class SnippetVectorMaskAdjuster(ap: VLSUArchitecturalParams) extends VLSUModules(ap){
  val io = IO(new Bundle{
    val eew = Input(UInt(3.W))
    val vm = Input(UInt(ap.vLen.W))
    val vlmul = Input(UInt(3.W))
    val isSegment = Input(Bool())
    val nf = Input(UInt(3.W))
    val adjustedSnippet = Output(Vec(8, UInt(ap.vLenb.W)))
  })
  io.adjustedSnippet := 0.U.asTypeOf(Vec(8, UInt(ap.vLenb.W)))
  val elenB = (1.U << io.eew(1,0)).asUInt()
  val isSegment = io.isSegment
  val elen1 = io.eew === 0.U
  val elen2 = io.eew === 1.U
  val elen4 = io.eew === 2.U
  val elen8 = io.eew === 3.U
  val lmul = io.vlmul(1, 0)
  val lmulOHs = WireInit(0.U(8.W))
  lmulOHs := (1.U << lmul).asUInt() - 1.U
  val elementsPerRegElen1 = ap.vLenb
  val elementsPerRegElen2 = ap.vLenb / 2
  val elementsPerRegElen4 = ap.vLenb / 4
  val elementsPerRegElen8 = ap.vLenb / 8
  val lmul2: Bool = WireInit(lmul === 1.U)
  val lmul4: Bool = WireInit(lmul === 2.U)
  val lmul8: Bool = WireInit(lmul === 3.U)
  val lmul1: Bool = WireInit(lmul === 0.U)
  val nFields = io.nf + 1.U
  val lmulValue = (1.U << lmul).asUInt()
  val byteActiveX: (Int,Int) => Bool = (i: Int, j: Int) => ???
  val vm = io.vm
  /** Indicates if this vlen is valid under current vlmul. */
  val lmulVld: Int => Bool = (i: Int) => i.U < (1.U << lmul).asUInt()
  /** if register is valid in lmul and nf combinations. */
  val segmentVld: Int => Bool = (i: Int) => i.U < (nFields * lmulValue)
  /** This is normal look up to vm when index < lmul and non-segment. */
  val snippetX: Int => UInt = (X: Int) => WireInit(VecInit(Seq.tabulate(ap.vLenb)(i => i + X * ap.vLenb).map(x =>
    Mux(elen1, vm(x) , Mux(elen2, vm(x >> 1), Mux(elen4,  vm(x >> 2), vm(x >> 3)))))).asUInt())
  val allOnes = Fill(ap.vLenb,1.U)
  io.adjustedSnippet.zipWithIndex.foreach { case (out, i) =>
    val snippet = snippetX(0)
    val idxMod4: Int = i % 4
    val idxMod2: Int = i % 2
    out := Mux(!isSegment && lmulVld(i), snippetX(i),
      Mux(isSegment && lmul1 && segmentVld(i), snippet,
        Mux(isSegment && lmul2 && segmentVld(i), snippetX(idxMod2),
          Mux(isSegment && lmul4 && segmentVld(i), snippetX(idxMod4), 0.U
        ))))
  }
}

/** logics combinations to split request from load store queue entry. */
class RequestSplitter(ap: VLSUArchitecturalParams, isLoad: Boolean, id: Int) extends VLSUModules(ap){
  val io = IO(new Bundle{
    val reg = if (isLoad) Input(new VLdQEntryBundle(ap)) else Input(new VStQEntryBundle(ap))
    val uReq = if (isLoad) Valid(new VLdRequest(ap)) else Valid(new VStRequest(ap))
    val newAddr = Output(UInt(ap.coreMaxAddrBits.W))
    val newSnippet = Output(UInt(ap.vLenb.W))
  })
  val reg = io.reg
  val lmul = reg.style.vlmul(1,0)
  val lmulValue = (1.U << reg.style.vlmul(1,0)).asUInt()
  val lmulLargerEqualOne = !reg.style.vlmul(2)
  val lmulLargerThanOne = lmulLargerEqualOne && lmul =/= 0.U
  val segmentModLmul = WireInit(reg.segmentCount % lmulValue)

  io.newAddr := 0.U
  io.newSnippet := 0.U
  io.uReq := 0.U.asTypeOf(if (isLoad) Valid(new VLdRequest(ap)) else Valid(new VStRequest(ap)))
  when(reg.style.isUnitStride){
    val baseAddr = Mux(reg.style.isWholeAccess, reg.addr + (reg.segmentCount << log2Ceil(ap.vLenb).U).asUInt(),
      reg.addr + Mux(lmulLargerEqualOne, ap.vLenb.U * segmentModLmul, 0.U))
    val (reqNecessary, req) = io.uReq.bits.UnitStride(baseAddr, reg.reqCount, reg.pRegVec(reg.segmentCount),
      reg.segmentCount, reg.style.isWholeAccess, true,  reg.finishMasks, id)
    io.uReq.valid := reqNecessary
    io.uReq.bits := req
  }.elsewhen(reg.style.isConstantStride){
    /** when segmentCount === lmul, reset to base addr. */
    val newAddr: UInt = Mux(lmulLargerThanOne && segmentModLmul === 0.U, reg.addr, reg.preAddr)
    val (reqNecessary, req, addr, snippet) = io.uReq.bits.ConstantStride(reg.addr, newAddr, reg.rs2, reg.style.eew,
      reg.pRegVec(reg.segmentCount), reg.segmentCount, reg.reqCount, reg.preSnippet, true, reg.finishMasks, id)
    io.uReq.valid := reqNecessary
    io.uReq.bits := req
    io.newAddr := addr
    io.newSnippet := snippet
  }.elsewhen(reg.style.isIndexed){
    val (reqNecessary, req, snippet) = io.uReq.bits.Indexed(reg.addr, reg.style.eew, reg.pRegVec(reg.segmentCount),
      reg.reqCount, reg.segmentCount, reg.preSnippet, reg.vs2,true, reg.finishMasks, id)
    io.uReq.valid := reqNecessary
    io.uReq.bits := req
    io.newSnippet := snippet
  }
}