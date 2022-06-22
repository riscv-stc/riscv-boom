package boom.vlsu

import chisel3._
import chisel3.util._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.util._
import boom.common._
import freechips.rocketchip.subsystem.CacheBlockBytes
import freechips.rocketchip.tile.{HasNonDiplomaticTileParameters, XLen}
import freechips.rocketchip.tilelink._

/** Parameters that decided by architects. */
case class VLSUGeneralParams(nCacheLineSizeBytes: Int = 64,
                             vLen: Int = 1024,
                             nLmshrs: Int = 4,
                             nSmshrs: Int = 4,
                             nL2DataBeatBytes: Int = 32,
                             nVLdQEntries: Int = 8,
                             nVStQEntries: Int = 8,
                             nVLdReqBuffEntries: Int = 128,
                             nVStReqBuffEntries: Int = 128,
                             coreWidth: Int = 2,
                             storeWaitCycles: Int = 8
                            ){
  require(nL2DataBeatBytes < nCacheLineSizeBytes, "beat byte should small than cache line size, L2 data does not support equal.")
}

/** Parameters that decided by general parameters and CDE and diplomacy together.
 * Are used for actual hardware generating. ALL hardware in VLSU should fetch params here.
 */
class VLSUArchitecturalParams(gp: VLSUGeneralParams,
                              bp: BoomCoreParams,
                              edge: TLEdgeOut)(implicit p: Parameters) {
  val cacheLineByteSize: Int = gp.nCacheLineSizeBytes
  val cacheLineBits: Int = cacheLineByteSize * 8
  require( cacheLineByteSize == p(CacheBlockBytes))
  val storeWaitCycles: Int = gp.storeWaitCycles
  /** Bundle parameters for outward transactions of tilelink. */
  val l2BundleParams: TLBundleParameters = edge.bundle
  require(cacheLineBits == l2BundleParams.dataBits, "one cache line per cycle. Use witdhwidgt to adjust with bus.")
  val tlEdge = edge
  /** Number of vector physical registers. */
  val nVRegs: Int = bp.numVecPhysRegisters
  val elementMaxBytes: Int = 8
  require(nVRegs > 0, "VRF should larger than 0 in an vector lsu.")

  require(bp.vLen > 0, "Vector length should larger than 0 in an vector lsu.")

  /** Start index inside a register for a request. */
  val nVRegStartIndex: Int = log2Ceil(bp.vLen)
  /** Index of an individual VRF. */
  val nVRegIndex: Int = log2Ceil(nVRegs)

  val nVLdQIndexBits: Int = log2Ceil(gp.nVLdQEntries)
  val nVStQIndexBits: Int = log2Ceil(gp.nVStQEntries)

  val addressBits = l2BundleParams.addressBits
  val nVLdReqBuffEntries: Int = gp.nVLdReqBuffEntries
  val nVLdReqBufferIdxBits: Int = log2Ceil(nVLdReqBuffEntries)
  val nVStReqBuffEntries: Int = gp.nVStReqBuffEntries
  val nVStReqBufferIdxBits: Int = log2Ceil(nVStReqBuffEntries)
  val nVLdQEntries = gp.nVLdQEntries
  val nVStQEntries = gp.nVStQEntries

  val vLen = bp.vLen
  val vLenSz = log2Ceil(vLen)
  val xLen = p(XLen)
  val vLenb = vLen / 8
  /** Maximum elements number in one vector. which means eew = 8b, lmul = 8. */
  val vlMax = log2Ceil(vLen) + 1
  require(vlMax < 20, "chisel compiler does not support left shift more than 20 bits.")
  require(vLenb >= cacheLineByteSize)
  val vpregSz = log2Ceil(bp.numVecPhysRegisters)
  val coreWidth = bp.decodeWidth
  /** Indicates how many finished instructions can be retired from pipeline at each cycle. */
  val retireWidth = bp.decodeWidth
  val numRobRows = bp.numRobEntries / coreWidth
  val robAddrSz = log2Ceil(numRobRows) + log2Ceil(coreWidth)

  val coreMaxAddrBits = l2BundleParams.addressBits

  val maxReqsInUnitStride = (vLenb / cacheLineByteSize) + 1
  require(nVLdReqBuffEntries > (coreWidth * maxReqsInUnitStride * 8), "dead lock may happen due to insufficient buffer entries." )
  require(nVStReqBuffEntries > (coreWidth * maxReqsInUnitStride * 8), "dead lock may happen due to insufficient buffer entries." )

  require(maxReqsInUnitStride < 4)
  val lineOffsetMSB: Int = log2Ceil(cacheLineByteSize) - 1
  val offsetBits: Int = log2Ceil(cacheLineByteSize)

  val nLmshrs = gp.nLmshrs
  val nSmshrs = gp.nSmshrs

  val maxBrCount = bp.maxBrCount

}