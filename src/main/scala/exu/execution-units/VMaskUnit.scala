package boom.exu

import chisel3._
import chisel3.util._

import boom.common._
import freechips.rocketchip.config.Parameters

object VMaskUnit
{
  val SZ_VMASKUNIT_FN = 4
  // def FN_VMX     = BitPat("b????")
  def FN_POPC  = 0.U(4.W)
  def FN_FIRST = 1.U(4.W)
  def FN_SBF   = 2.U(4.W)
  def FN_SIF   = 3.U(4.W)
  def FN_SOF   = 4.U(4.W)

  def setOnlyFirst(num: UInt): UInt = {
    val tmp = num & (num-1.U)
    val ret = tmp ^ num
    return ret
  }
}

import VMaskUnit._

class VMaskUnit(implicit p: Parameters) extends BoomModule{
  val io = IO(new Bundle {
    val fn = Input(Bits(SZ_VMASKUNIT_FN.W))
    val in = Input(UInt(xLen.W))
    val in_addend = Input(UInt(xLen.W))
    val out = Output(UInt(xLen.W))
  })

  // popc
  val popc = PopCount(io.in)

  val find_first_set = PriorityEncoder(io.in)

  val index_added = Mux(io.fn === FN_POPC, popc,
                    Mux(io.fn === FN_FIRST, find_first_set, 0.U(xLen.W)))

  val index_result = index_added + io.in_addend

  val set_only_first = setOnlyFirst(io.in)
  val set_before_first = set_only_first - 1.U
  val set_including_first = set_only_first | set_before_first

  val out = Mux((io.fn === FN_POPC) || (io.fn === FN_FIRST), index_added,
            Mux(io.fn === FN_SOF, set_only_first, 
            Mux(io.fn === FN_SBF, set_before_first,
            Mux(io.fn === FN_SIF, set_including_first, 0.U(xLen.W)))))

  io.out := out
}
