package boom.vlsu

import chisel3._
import chisel3.util._

class VTLB(ap: VLSUArchitecturalParams) extends VLSUModules(ap){
  val io = IO(new Bundle{
    val req = Flipped(Vec(2, Decoupled(new VTLBReq(ap))))
    val resp = Vec(2, Valid(new VTLBResp(ap)))
  })
  // Currently it is pass through.
  io.req zip io.resp foreach { case (req, resp) =>
    resp.valid := req.valid
    resp.bits.paddr := req.bits.vaddr
    resp.bits.hit := true.B
    resp.bits.exception := false.B
    resp.bits.queueIdx := req.bits.queueIdx
    resp.bits.isLoad := req.bits.isLoad
    resp.bits.reqBufferIdx := req.bits.reqBufferIdx
    resp.bits.snippet := req.bits.snippet
    resp.bits.segmentIdx := req.bits.segmentIdx
  }
  io.req.map(_.ready).foreach(_ := true.B)
}
