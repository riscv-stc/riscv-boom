package boom.vlsu

import chisel3._
import chisel3.util._
import freechips.rocketchip.tilelink.TLMessages._
import freechips.rocketchip.tilelink._
class LMSHR(ap: VLSUArchitecturalParams, idx: Int) extends VLSUModules(ap){
  val io = IO(new Bundle{
    val vLdReq: ValidIO[VLdRequest] = Flipped(Valid(new VLdRequest(ap)))
    val tlOutA: DecoupledIO[TLBundleA] = Decoupled(new TLBundleA(ap.l2BundleParams))
    val tlInD: DecoupledIO[TLBundleD] = Flipped(Decoupled(new TLBundleD(ap.l2BundleParams)))
    val status: ValidIO[UInt] = Valid(UInt(ap.coreMaxAddrBits.W))
    val writeBackReq: DecoupledIO[VLdWriteBackRequest] = Decoupled(new VLdWriteBackRequest(ap))
  })

  require(io.tlOutA.bits.data.getWidth == ap.cacheLineBits, "one cache line at one cycle!\n")
  io.tlOutA.valid := false.B
  io.tlOutA.bits := 0.U.asTypeOf(new TLBundleA(ap.l2BundleParams))
  io.status := 0.U.asTypeOf(Valid(UInt(ap.coreMaxAddrBits.W)))
  io.tlOutA.valid := false.B
  io.writeBackReq.valid := false.B
  io.writeBackReq.bits := 0.U.asTypeOf(new VLdWriteBackRequest(ap))
  val lineBuffer: UInt = RegInit(0.U.asTypeOf(UInt(ap.cacheLineBits.W)))
  val sIdle :: sWaitArb :: sWaitData :: sWaitWB :: Nil = Enum(4)
  val state: UInt = RegInit(sIdle)
  val reg = RegInit(0.U.asTypeOf(Valid(new VLdRequest(ap))))
  when(state === sIdle){
    when(io.vLdReq.valid){
      reg <> io.vLdReq
      state := sWaitArb
    }
  }.elsewhen(state === sWaitArb){
    io.tlOutA.valid := true.B
    io.tlOutA.bits := DontCare
    io.tlOutA.bits := ap.tlEdge.Get(idx.U, lineAddress(reg.bits.address), log2Ceil(ap.cacheLineByteSize).U)._2
    when(io.tlOutA.fire()){
      state := sWaitData
    }
  }.elsewhen(state === sWaitData){
    when(io.tlInD.fire() && io.tlInD.bits.source === idx.U){
      lineBuffer := io.tlInD.bits.data
      state := sWaitWB
    }
  }.elsewhen(state === sWaitWB){
    io.writeBackReq.valid := true.B
    io.writeBackReq.bits.data := lineBuffer.asUInt()
    io.writeBackReq.bits.req := reg.bits
    when(io.writeBackReq.fire()){
     state := sIdle
    }
  }
  when(state =/= sIdle){
    io.status.valid := true.B
    io.status.bits := reg.bits.address
  }
  io.tlInD.ready := true.B
}