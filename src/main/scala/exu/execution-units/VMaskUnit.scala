package boom.exu

import chisel3._
import chisel3.util._

import boom.common._
import freechips.rocketchip.config.Parameters

object VMaskUnit
{
  // keep consistent with SZ_ALU_FN as they are in the same decode table
  val FN_VMASK_SZ = 4
  // def FN_VMX     = BitPat("b????")
  def FN_POPC  = 0.U(FN_VMASK_SZ.W)
  def FN_FIRST = 1.U(FN_VMASK_SZ.W)
  def FN_SBF   = 2.U(FN_VMASK_SZ.W)
  def FN_SIF   = 3.U(FN_VMASK_SZ.W)
  def FN_SOF   = 4.U(FN_VMASK_SZ.W)

  def setOnlyFirst(num: UInt): UInt = {
    val tmp = num & (num-1.U)
    val ret = tmp ^ num
    return ret
  }
}

import VMaskUnit._

class VMaskUnit(val offset: Int = 0)(implicit p: Parameters) extends BoomModule{
  val io = IO(new Bundle {
    val fn    = Input(Bits(FN_VMASK_SZ.W))
    val in1   = Input(UInt(eLen.W))
    val in2   = Input(UInt(eLen.W))
    val mask  = Input(UInt(eLen.W))
    val out   = Output(UInt(eLen.W))
    val index = Output(Valid(UInt(vLenSz.W)))
  })

  val srcData = io.in1 & io.mask
  val oriData = io.in2 & ~io.mask

  // popc
  val popc = PopCount(srcData)
  // the lowest numbered active element of the source mask vector
  val vfirstIdx = PriorityEncoder(srcData)
  // set-only-first mask bit
  val vmsof = setOnlyFirst(srcData)
  // set-before-first mask bit
  val vmsbf = vmsof - 1.U
  // set-including first mask bit
  val vmsif = vmsof | vmsbf
    
  val index = Mux(io.fn === FN_POPC,  popc,
              Mux(io.fn === FN_FIRST, vfirstIdx + offset.asUInt, 0.U))
  val out   = Mux(io.fn === FN_SBF,   vmsbf & io.mask | oriData,
              Mux(io.fn === FN_SIF,   vmsif & io.mask | oriData,
              Mux(io.fn === FN_SOF,   vmsof & io.mask | oriData, 0.U)))

  io.out := out
  // indicates whether vfirst is valid
  io.index.bits  := index
  io.index.valid := srcData.orR
}

object VIotaUnit
{
  val FN_VIOTA_SZ = 4
  def FN_IOTA  = 0.U(FN_VIOTA_SZ.W)
  def FN_INDEX = 1.U(FN_VIOTA_SZ.W)
}

import VIotaUnit._

class VIotaUnit(
  val maskWidth: Int,
  val dataWidth: Int,
  val offset: Int = 0
)(implicit p: Parameters) extends BoomModule{
  val io = IO(new Bundle {
    val fn   = Input(UInt(FN_VIOTA_SZ.W))
    val in   = Input(UInt(maskWidth.W))
    val base = Input(UInt(dataWidth.W))
    val mask = Input(UInt(maskWidth.W))
    val out  = Output(UInt(dataWidth.W))
  })

  require(maskWidth > 0)

  val popc = WireInit(0.U(vLenSz.W))
  if(maskWidth == 1) {
    popc := 0.U
  } else {
    val srcData = io.in(maskWidth-2, 0) & io.mask(maskWidth-2, 0)
    popc := PopCount(srcData)
  }
  io.out := Mux(io.fn === FN_IOTA, io.base + popc, io.base + offset.asUInt)
}