package exu.matrix

import boom.common.{BoomBundle, BoomModule}
import boom.util._
import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.util._

class AccRegReadPortIO(implicit p: Parameters) extends BoomBundle {
  val addr = Input(UInt(log2Ceil(numMatAccPhysRegs).W)) // value of td num
  val data = Output(UInt(accRLen.W))
  val index = Input(UInt(32.W))       //index slice number, value is rs2
  val tt = Input(UInt(2.W))           //0: acc_r, 1: acc_c
  val msew = Input(UInt(2.W))
}

class AccRegWritePortIO(implicit p: Parameters) extends BoomBundle {
  val addr = UInt(log2Ceil(numMatAccPhysRegs).W) // value of td num
  val data = UInt(accRLen.W)
  val index = UInt(32.W) //index slice number, value is rs2
  val tt = UInt(2.W) //0: acc_r, 1: acc_c
  val msew = UInt(2.W)
  val byteMask = UInt(accRLenb.W)
}

class AccReg(val numReadPorts: Int, val numWritePorts: Int)(implicit p: Parameters)  extends BoomModule {
  val io = IO(new Bundle {
    val readPorts = Vec(numReadPorts, new AccRegReadPortIO())
    val writePorts = Flipped(Vec(numWritePorts, Valid(new AccRegWritePortIO())))
    //val clearPorts = Flipped(Vec(memWidth, Valid(UInt(log2Ceil(numMatAccPhysRegs).W))))
  })

  require(isPow2(accMLen))
  require(isPow2(accRLen))
  require(accMLen >= accRLen)

  val rowlen = accMLen / accRLen

  val readdircol = Wire(Vec(numReadPorts, Bool()))
  val readdirrow = Wire(Vec(numReadPorts, Bool()))
  val readmsew = Wire(Vec(numReadPorts, UInt(2.W)))
  val readindex = Wire(Vec(numReadPorts, UInt(32.W)))
  val readaddr = Wire(Vec(numReadPorts, UInt(log2Ceil(numMatAccPhysRegs).W)))

  val writedircol = Wire(Vec(numWritePorts, Bool()))
  val writedirrow = Wire(Vec(numWritePorts, Bool()))
  val writemsew = Wire(Vec(numWritePorts, UInt(2.W)))
  val writeindex = Wire(Vec(numWritePorts, UInt(32.W)))
  val writebtyemask = Wire(Vec(numWritePorts, UInt(accRLenb.W)))

  for (i <- 0 until numReadPorts) {
    readdircol(i) := io.readPorts(i).tt === 1.U
    readdirrow(i) := io.readPorts(i).tt === 0.U
    readmsew(i)   := io.readPorts(i).msew
    readindex(i)  := io.readPorts(i).index
    readaddr(i)   := io.readPorts(i).addr
  }
  for (i <- 0 until numWritePorts) {
    writedircol(i)   := io.writePorts(i).bits.tt === 1.U
    writedirrow(i)   := io.writePorts(i).bits.tt === 0.U
    writemsew(i)     := io.writePorts(i).bits.msew
    writeindex(i)    := io.writePorts(i).bits.index
    writebtyemask(i) := io.writePorts(i).bits.byteMask
  }

  val accreg = Mem(numMatAccPhysRegs, Vec(rowlen, UInt(accRLen.W)))

  //Read Tr_Tile
  val readRegData = Wire(Vec(numReadPorts, Vec(rowlen, UInt(accRLen.W))))

  for (w <- 0 until numReadPorts) {
    io.readPorts(w).data := 0.U
    readRegData(w) := accreg(readaddr(w))
    when(readdirrow(w)) {
      io.readPorts(w).data := readRegData(w)(readindex(w))
    }
    when(readdircol(w)) {
      val readColData = (0 until rowlen).map(r => VDataSel(readRegData(w)(r), readmsew(w), readindex(w), accRLen, accRLen))
      when(readmsew(w) === 0.U) {
        io.readPorts(w).data := Cat((0 until rowlen).map(r => readColData(r)(7, 0)).reverse)
      }.elsewhen(readmsew(w) === 1.U) {
        io.readPorts(w).data := Cat((0 until rowlen).map(r => readColData(r)(15, 0)).reverse)
      }.elsewhen(readmsew(w) === 2.U) {
        io.readPorts(w).data := Cat((0 until rowlen).map(r => readColData(r)(31, 0)).reverse)
      }
    }
  }

  //Write Tr_tile
  for (acc <- 0 until numMatAccPhysRegs) {
    for (w <- 0 until numWritePorts) {
      val writemask = FillInterleaved(8, writebtyemask(w))
      val writedata = writemask & io.writePorts(w).bits.data
      when(writedirrow(w) && io.writePorts(w).valid && acc.U === io.writePorts(w).bits.addr) {
        for (index <- 0 until rowlen) {
          when(writeindex(w) === index.U) {
            val oldData = accreg(io.writePorts(w).bits.addr)(index) //rlen
            accreg(io.writePorts(w).bits.addr)(index) := writedata | (oldData & (~writemask).asUInt())
          }
        }
      }
      when(writedircol(w) && io.writePorts(w).valid && acc.U === io.writePorts(w).bits.addr) {
        for (row <- 0 until rowlen) {
          val oldData = accreg(io.writePorts(w).bits.addr)(row) //rlen
          val writeColData = VDataSel(writedata, writemsew(w), row.asUInt(), accRLen, accRLen)
          val writeColDataSel = Wire(UInt(accRLen.W))
          val writeColShift = Wire(UInt(accRLenSz.W))
          val writeColRowMask = Wire(UInt((accRLen + 1).W))
		      val writeRowNumMask = WireInit(false.B)
          val writeColRowData = Wire(UInt(accRLen.W))
          writeColShift := (writeindex(w) << (writemsew(w) +& 3.U))
          writeColRowMask := Mux1H(UIntToOH(writemsew(w)), Seq(Cat(Fill(accRLen - 8, 0.U), Fill(8, 1.U)) << writeColShift,
            Cat(Fill(accRLen - 16, 0.U), Fill(16, 1.U)) << writeColShift,
            Cat(Fill(accRLen - 32, 0.U), Fill(32, 1.U)) << writeColShift
          ))
		      writeRowNumMask := (writebtyemask(w) >> (row.asUInt << writemsew(w)).asUInt()).asUInt() & 1.U
          writeColRowData := writeColData << writeColShift
		      writeColDataSel := Mux(writeRowNumMask, Cat((0 until accRLen).map(i => Mux(writeColRowMask(i), writeColRowData(i), oldData(i))).reverse), oldData)

          accreg(io.writePorts(w).bits.addr)(row) := writeColDataSel
        }
      }
    }
  }

  /*for (w <- 0 until memWidth) {
    when (io.clearPorts(w).valid) {
      for (r <- 0 until rowlen) {
        accreg(io.clearPorts(w).bits)(r) := 0.U(accRLen.W)
      }
    }
  }*/

  //ensure there is only 1 writer per trtile slice
  if (numWritePorts > 1) {
    for (i <- 0 until (numWritePorts - 1)) {
      for (j <- (i + 1) until numWritePorts) {
        assert(!((io.writePorts(i).bits.addr === io.writePorts(j).bits.addr) && io.writePorts(i).valid && io.writePorts(j).valid &&
        (io.writePorts(i).bits.index === io.writePorts(j).bits.index  || io.writePorts(i).bits.tt =/= io.writePorts(j).bits.tt)))
      }
    }
  }
}
