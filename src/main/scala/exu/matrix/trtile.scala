package boom.exu

import boom.common.{BoomBundle, BoomModule}
import boom.util._
import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.util._
import freechips.rocketchip.util._

class TrTileRegReadPortIO(implicit p: Parameters) extends BoomBundle {
  val addr: UInt = Input(UInt(log2Ceil(numMatTrPhysRegs).W)) // value of td num
  val data: UInt = Output(UInt(rLen.W))
  val index: UInt = Input(UInt(32.W)) //index slice number, value is rs2
  val tt: UInt = Input(UInt(2.W)) //2: tr_r, 3: tr_c
  val msew: UInt = Input(UInt(2.W))
}

class TrTileRegWritePortIO(implicit p: Parameters) extends BoomBundle {
  val addr: UInt = UInt(log2Ceil(numMatTrPhysRegs).W) // value of td num
  val data: UInt = UInt(rLen.W)
  val index: UInt = UInt(32.W) //index slice number, value is rs2
  val tt: UInt = UInt(2.W) //2: tr_r, 3: tr_c
  val msew: UInt = UInt(2.W)
  val byteMask: UInt = UInt(rLenb.W)
}

class TrTileReg(val numReadPorts: Int, val numWritePorts: Int)(implicit p: Parameters)  extends BoomModule {
  val io = IO(new Bundle {
    val readPorts: Vec[TrTileRegReadPortIO] = Vec(numReadPorts, new TrTileRegReadPortIO())
    val writePorts: Vec[ValidIO[TrTileRegWritePortIO]] = Flipped(Vec(numWritePorts, Valid(new TrTileRegWritePortIO())))
  })

  require(isPow2(mLen))
  require(isPow2(rLen))
  require(mLen >= rLen)
  val rowlen = mLen / rLen

  val readdircol = Wire(Vec(numReadPorts, Bool()))
  val readdirrow = Wire(Vec(numReadPorts, Bool()))
  val readmsew = Wire(Vec(numReadPorts, UInt(2.W)))
  val readindex = Wire(Vec(numReadPorts, UInt(32.W)))
  val readaddr = Wire(Vec(numReadPorts, UInt(log2Ceil(numMatTrPhysRegs).W)))
  val writedircol = Wire(Vec(numWritePorts, Bool()))
  val writedirrow = Wire(Vec(numWritePorts, Bool()))
  val writemsew = Wire(Vec(numWritePorts, UInt(2.W)))
  val writeindex = Wire(Vec(numWritePorts, UInt(32.W)))
  val writebtyemask = Wire(Vec(numWritePorts, UInt(rLenb.W)))

  for (i <- 0 until numReadPorts) {
    readdircol(i) := RegNext(io.readPorts(i).tt === 3.U)
    readdirrow(i) := RegNext(io.readPorts(i).tt === 2.U)
    readmsew(i)   := RegNext(io.readPorts(i).msew)
    readindex(i)  := RegNext(io.readPorts(i).index)
    readaddr(i)   := RegNext(io.readPorts(i).addr)
  }
  for (i <- 0 until numWritePorts) {
    writedircol(i)   := io.writePorts(i).bits.tt === 3.U
    writedirrow(i)   := io.writePorts(i).bits.tt === 2.U
    writemsew(i)     := io.writePorts(i).bits.msew
    writeindex(i)    := io.writePorts(i).bits.index
    writebtyemask(i) := io.writePorts(i).bits.byteMask
  }

  val trtile = Mem(numMatTrPhysRegs, Vec(rowlen, UInt(rLen.W))) //trNums = 8

  //Read Tr_Tile
  val readRegData: Vec[Vec[UInt]] = Wire(Vec(numReadPorts, Vec(rowlen, UInt(rLen.W))))

  for (w <- 0 until numReadPorts) {
    io.readPorts(w).data := 0.U
    readRegData(w) := trtile(readaddr(w))
    when(readdirrow(w)) {
      io.readPorts(w).data := readRegData(w)(readindex(w))
    }
    when(readdircol(w)) {
      val readColData = (0 until rowlen).map(r => VDataSel(readRegData(w)(r), readmsew(w), readindex(w), rLen, rLen))
      when(readmsew(w) === 0.U) {
        io.readPorts(w).data := Cat((0 until rowlen).map(r => readColData(r)(7, 0)).reverse)
      }.elsewhen(readmsew(w) === 1.U) {
        io.readPorts(w).data := Cat((0 until rowlen).map(r => readColData(r)(15, 0)).reverse)
      }.elsewhen(readmsew(w) === 2.U) {
        io.readPorts(w).data := Cat((0 until rowlen).map(r => readColData(r)(31, 0)).reverse)
      }.elsewhen(readmsew(w) === 1.U) {
        io.readPorts(w).data := Cat((0 until rowlen).map(r => readColData(r)(63, 0)).reverse)
      }
    }
  }

  //Write Tr_tile
  for (tr <- 0 until numMatTrPhysRegs) {
    for (w <- 0 until numWritePorts) {
      val writemask = FillInterleaved(8, writebtyemask(w))
      val writedata = writemask & io.writePorts(w).bits.data
      when(writedirrow(w) && io.writePorts(w).valid && tr.U === io.writePorts(w).bits.addr) {
        for (index <- 0 until rowlen) {
          when(writeindex(w) === index.U) {
            val oldData = trtile(io.writePorts(w).bits.addr)(index) //rlen
            trtile(io.writePorts(w).bits.addr)(index) := writedata | (oldData & ~writemask)
          }
        }
      }
      when(writedircol(w) && io.writePorts(w).valid && tr.U === io.writePorts(w).bits.addr) {
        for (row <- 0 until rowlen) {
          val oldData = trtile(io.writePorts(w).bits.addr)(row) //rlen
          val writeColData = VDataSel(writedata, writemsew(w), row.asUInt(), rLen, rLen)
          val writeColDataSel = Wire(UInt(rLen.W))
          val writeColShift = Wire(UInt(rLenSz.W))
          val writeColRowMask = Wire(UInt((rLen + 1).W))
          val writeColRowData = Wire(UInt(rLen.W))
          writeColShift := (writeindex(w) << (writemsew(w) +& 3.U))
          writeColRowMask := Mux1H(UIntToOH(writemsew(w)), Seq(Cat(Fill(rLen - 8, 0.U), Fill(8, 1.U)) << writeColShift,
            Cat(Fill(rLen - 16, 0.U), Fill(16, 1.U)) << writeColShift,
            Cat(Fill(rLen - 32, 0.U), Fill(32, 1.U)) << writeColShift,
            Cat(Fill(rLen - 63, 0.U), Fill(64, 1.U)) << writeColShift
          ))
          writeColRowData := writeColData << writeColShift
          writeColDataSel := Cat((0 until rLen).map(i => Mux(writeColRowMask(i), writeColRowData(i), oldData(i))).reverse)
          trtile(io.writePorts(w).bits.addr)(row) := writeColDataSel
        }
      }
    }
  }
  //ensure there is only 1 writer per trtile
  if (numWritePorts > 1) {
    for (i <- 0 until (numWritePorts - 1)) {
      for (j <- (i + 1) until numWritePorts) {
        assert(!io.writePorts(i).valid ||
          !io.writePorts(j).valid ||
          (io.writePorts(i).bits.addr =/= io.writePorts(j).bits.addr),
          "[regfile] too many writers a register")
      }
    }
  }
}
