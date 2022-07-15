package boom.exu

import boom.common.{BoomBundle, BoomModule}
import boom.util._
import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.util._
import freechips.rocketchip.util._

class TrTileCSR(implicit p: Parameters) extends BoomBundle {
  val msew = UInt(2.W)
}

class TrtileAddr(implicit p: Parameters) extends BoomBundle {
  val addr = UInt(log2Ceil(numMatTrPhysRegs).W)
  val index: UInt = UInt(32.W) //index slice number, value is rs2
  val tt: UInt = UInt(2.W) //2: tr_r, 3: tr_c
}

class TrTileMleWrReq(implicit p: Parameters) extends BoomBundle {
  val mlereq = UInt(1.W) //mle, wr trtile register
  val data = UInt(mLen.W)
  val reqaddr = new TrtileAddr
}

class TrTileMseReadReq(implicit p: Parameters) extends BoomBundle {
  val msereq = UInt(1.W) //000 mse, read trtile register
  val reqaddr = new TrtileAddr
}

class TrTileMmvWrReq(implicit p: Parameters) extends BoomBundle {
  val mmvWrReq = UInt(3.W) //001 mmv.v.{tt}.{dim}  -> vreg move to tile
  //01 mwmv.v.{tt}.{dim} -> vreg nove to tile, ELEN is 2*sew
  //10 mqmv.v.{tt}.{dim} -> vreg nove to tile, ELEN is 4*sew
  val data = UInt(mLen.W)
  val reqaddr = new TrtileAddr
}

class TrTileMmvReadReq(implicit p: Parameters) extends BoomBundle {
  val mmvReadreq = UInt(3.W) //00 mmv.{tt}.v.{dim}  -> tile move to vreg
  //01 mwmv.{tt}.v.{dim} -> tile move to vreg, ELEN is 2*sew
  //10 mqmv.{tt}.v.{dim} -> tile move to vreg, ELEN is 4*sew
  val reqaddr = new TrtileAddr
}

class TrTileRegReadPortIO(implicit p: Parameters) extends BoomBundle {
  val addr: UInt = Input(UInt(log2Ceil(numMatTrPhysRegs).W)) // value of td num
  val data: UInt = Output(UInt(vLen.W))
  val index: UInt = Input(UInt(32.W)) //index slice number, value is rs2
  val tt: UInt = Input(UInt(2.W)) //2: tr_r, 3: tr_c
  val msew: UInt = Input(UInt(2.W))
}

class TrTileRegWritePortIO(implicit p: Parameters) extends BoomBundle {
  val addr: UInt = UInt(log2Ceil(numMatTrPhysRegs).W) // value of td num
  val data: UInt = UInt(vLen.W)
  val index: UInt = UInt(32.W) //index slice number, value is rs2
  val tt: UInt = UInt(2.W) //2: tr_r, 3: tr_c
  val msew: UInt = UInt(2.W)
  val byteMask: UInt = UInt(log2Ceil(vLenb).W)
}

class TrTileReg(val numReadPorts: Int, val numWritePorts: Int)(implicit p: Parameters)  extends BoomModule {
  val io = IO(new Bundle {
    val readPorts: Vec[TrTileRegReadPortIO] = Vec(numReadPorts, new TrTileRegReadPortIO())
    val writePorts: Vec[ValidIO[TrTileRegWritePortIO]] = Flipped(Vec(numWritePorts, Valid(new TrTileRegWritePortIO())))
  })

  require(isPow2(mLen))
  require(isPow2(vLen))
  require(mLen >= vLen)
  val rowlen = mLen / vLen

  val readdircol = Wire(Vec(numReadPorts, Bool()))
  val readdirrow = Wire(Vec(numReadPorts, Bool()))
  val readmsew = Wire(Vec(numReadPorts, UInt(2.W)))
  val readindex = Wire(Vec(numReadPorts, UInt(32.W)))
  val readaddr = Wire(Vec(numReadPorts, UInt(log2Ceil(numMatTrPhysRegs).W)))
  val writedircol = Wire(Vec(numWritePorts, Bool()))
  val writedirrow = Wire(Vec(numWritePorts, Bool()))
  val writemsew = Wire(Vec(numWritePorts, UInt(2.W)))
  val writeindex = Wire(Vec(numWritePorts, UInt(32.W)))
  val writebtyemask = Wire(Vec(numWritePorts, UInt(log2Ceil(vLenb).W)))

  for (i <- 0 until numReadPorts) {
    readdircol(i) := RegNext(io.readPorts(i).tt === 3.U)
    readdirrow(i) := RegNext(io.readPorts(i).tt === 2.U)
    readmsew(i) := RegNext(io.readPorts(i).msew)
    readindex(i) := RegNext(io.readPorts(i).index)
    readaddr(i) := RegNext(io.readPorts(i).addr)
  }
  for (i <- 0 until numWritePorts) {
    writedircol(i) := io.writePorts(i).bits.tt === 3.U
    writedirrow(i) := io.writePorts(i).bits.tt === 2.U
    writemsew(i) := io.writePorts(i).bits.msew
    writeindex(i) := io.writePorts(i).bits.index
    writebtyemask(i) := io.writePorts(i).bits.byteMask
  }


  //val trtile = Mem(trNums, UInt(mlen.W))  //trNums = 8
  val trtile = Mem(numMatTrPhysRegs, Vec(rowlen, UInt(vLen.W))) //trNums = 8

  //Read Tr_Tile
  val readRegData: Vec[Vec[UInt]] = Wire(Vec(numReadPorts, Vec(rowlen, UInt(vLen.W))))

  for (w <- 0 until numReadPorts) {
    io.readPorts(w).data := 0.U
    readRegData(w) := trtile(readaddr(w))
    when(readdirrow(w)) {
      io.readPorts(w).data := readRegData(w)(readindex(w))
    }
    when(readdircol(w)) {
      val readColData = (0 until rowlen).map(r => VDataSel(readRegData(w)(r), readmsew(w), readindex(w), vLen, vLen))
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
            trtile(io.writePorts(w).bits.addr)(index) := writedata
          }
        }
      }
      when(writedircol(w) && io.writePorts(w).valid && tr.U === io.writePorts(w).bits.addr) {
        //val writeColMask = UIntToOH(writewidth(w), rowlen) - 1.U
        for (row <- 0 until rowlen) {
          val oldData = trtile(io.writePorts(w).bits.addr)(row) //vlen
          val writeColData = VDataSel(writedata, writemsew(w), row.asUInt(), vLen, vLen)
          val writeColDataSel = Wire(UInt(vLen.W))
          val writeColShift = Wire(UInt(vLenSz.W))
          val writeColRowMask = Wire(UInt((vLen + 1).W))
          val writeColRowData = Wire(UInt(vLen.W))
          writeColShift := (writeindex(w) << (writemsew(w) +& 3.U))
          writeColRowMask := Mux1H(UIntToOH(writemsew(w)), Seq(Cat(Fill(vLen - 8, 0.U), Fill(8, 1.U)) << writeColShift,
            Cat(Fill(vLen - 16, 0.U), Fill(16, 1.U)) << writeColShift,
            Cat(Fill(vLen - 32, 0.U), Fill(32, 1.U)) << writeColShift,
            Cat(Fill(vLen - 63, 0.U), Fill(64, 1.U)) << writeColShift
          ))
          writeColRowData := writeColData << writeColShift
          writeColDataSel := Cat((0 until vLen).map(i => Mux(writeColRowMask(i), writeColRowData(i), oldData(i))).reverse)
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

class TrTile(val numReadPorts: Int, val numWritePorts: Int)(implicit p: Parameters)  extends BoomModule {
  val io = IO(new Bundle {
    val trtilecsr = Input(new TrTileCSR)
    //vreg <-> tr_tile
    val mleWrReq: ValidIO[TrTileMleWrReq] = Flipped(ValidIO(new TrTileMleWrReq))
    val mseReadReq: ValidIO[TrTileMseReadReq] = Flipped(ValidIO(new TrTileMseReadReq))
    val mseReadResp: ValidIO[UInt] = ValidIO(UInt(mLen.W))
    //vreg <-> vlsu
    val mmvWrReq: ValidIO[TrTileMmvWrReq] = Flipped(ValidIO(new TrTileMmvWrReq))
    val mmvReadReq: ValidIO[TrTileMmvReadReq] = Flipped(ValidIO(new TrTileMmvReadReq))
    val mmvReadResp = ValidIO(UInt(mLen.W))
  })
  val rowlen = mLen / vLen

  assert(!(io.mleWrReq.valid && io.mmvWrReq.valid && (io.mleWrReq.bits.reqaddr.addr === io.mmvWrReq.bits.reqaddr.addr)))

  val trtileReg = Module(new TrTileReg(numReadPorts, numWritePorts))

  //mseReadReq
  trtileReg.io.readPorts(0).msew := io.trtilecsr.msew
  trtileReg.io.readPorts(0).tt := io.mseReadReq.bits.reqaddr.tt
  trtileReg.io.readPorts(0).addr := io.mseReadReq.bits.reqaddr.addr
  trtileReg.io.readPorts(0).index := io.mseReadReq.bits.reqaddr.index

  //mmvReadReq
  trtileReg.io.readPorts(1).msew := io.trtilecsr.msew << io.mmvReadReq.bits.mmvReadreq
  trtileReg.io.readPorts(1).tt := io.mmvReadReq.bits.reqaddr.tt
  trtileReg.io.readPorts(1).addr := io.mmvReadReq.bits.reqaddr.addr
  trtileReg.io.readPorts(1).index := io.mmvReadReq.bits.reqaddr.index
  //mleWrReq
  trtileReg.io.writePorts(0).valid := io.mleWrReq.valid
  trtileReg.io.writePorts(0).bits.msew := io.trtilecsr.msew
  trtileReg.io.writePorts(0).bits.tt := io.mleWrReq.bits.reqaddr.tt
  trtileReg.io.writePorts(0).bits.addr := io.mleWrReq.bits.reqaddr.addr
  trtileReg.io.writePorts(0).bits.index := io.mleWrReq.bits.reqaddr.index
  trtileReg.io.writePorts(0).bits.data := io.mleWrReq.bits.data
  //mmvWrReq
  trtileReg.io.writePorts(1).valid := io.mmvWrReq.valid
  trtileReg.io.writePorts(1).bits.msew := io.trtilecsr.msew << io.mmvWrReq.bits.mmvWrReq
  trtileReg.io.writePorts(1).bits.tt := io.mmvWrReq.bits.reqaddr.tt
  trtileReg.io.writePorts(1).bits.addr := io.mmvWrReq.bits.reqaddr.addr
  trtileReg.io.writePorts(1).bits.index := io.mmvWrReq.bits.reqaddr.index
  trtileReg.io.writePorts(1).bits.data := io.mmvWrReq.bits.data

  io.mseReadResp.valid := RegNext(io.mseReadReq.valid)
  io.mseReadResp.bits := trtileReg.io.readPorts(0).data
  io.mmvReadResp.valid := RegNext(io.mmvReadReq.valid)
  io.mmvReadResp.bits := trtileReg.io.readPorts(1).data

  assert(!(io.mseReadReq.valid &&
    ((trtileReg.io.readPorts(0).tt === 2.U) && (trtileReg.io.readPorts(0).index >= rowlen.asUInt())
      || (trtileReg.io.readPorts(0).tt === 3.U) && (trtileReg.io.readPorts(0).index >= (vLenb.asUInt() >> trtileReg.io.readPorts(0).msew).asUInt()))),
    "mseRead index large than element width")
  assert(!(io.mmvReadReq.valid &&
    ((trtileReg.io.readPorts(1).tt === 2.U) && (trtileReg.io.readPorts(1).index >= rowlen.asUInt())
      || (trtileReg.io.readPorts(1).tt === 3.U) && (trtileReg.io.readPorts(1).index >= (vLenb.asUInt() >> trtileReg.io.readPorts(1).msew).asUInt()))),
    "mmvRead index large than element width")
  for (i <- 0 until numWritePorts) {
    assert(!(trtileReg.io.writePorts(i).valid &&
      ((trtileReg.io.writePorts(i).bits.tt === 2.U) && (trtileReg.io.writePorts(i).bits.index >= rowlen.asUInt())
        || (trtileReg.io.writePorts(i).bits.tt === 3.U) && (trtileReg.io.writePorts(i).bits.index >= (vLenb.asUInt() >> trtileReg.io.writePorts(i).bits.msew).asUInt()))),
      "Write index large than element width")
  }
}


import freechips.rocketchip.config._
import freechips.rocketchip.unittest._

class TrTileUT(timeout: Int = 4000)(implicit p: Parameters)
  extends UnitTest(timeout) {
  val cycle = Reg(UInt(32.W))
  when(io.start) {
    cycle := 0.U
  }.otherwise {
    cycle := cycle + 1.U
  }

  val active = RegInit(false.B)
  when(io.start) {
    active := true.B
  }

  val dataWidth: Int = 64
  val trNums: Int = 8
  val mLen: Int = 256
  val vLen: Int = 64
  val numReadPorts: Int = 2
  val numWritePorts: Int = 2

  val case1_start = 40
  val case2_start = 80
  val case3_start = 120
  val case4_start = 160
  val case5_start = 200

  val data1 = 6.U(8.W) //6
  val data2 = 13.U(8.W) //d
  val data3 = 39.U(8.W) //27
  val data4 = 51.U(8.W) //33

  val csr = Wire(new TrTileCSR())
  csr := DontCare
  //val msew = RegInit(0.U(2.W))
  // msew := LCG(2, active)
  csr.msew := LCG(2, active)
  //mle
  val mleWrReq = Wire(DecoupledIO(new TrTileMleWrReq()))
  mleWrReq := DontCare
  mleWrReq.valid := false.B
  mleWrReq.bits.reqaddr.addr := LCG(3, active)
  mleWrReq.bits.reqaddr.index := LCG(32, active)
  mleWrReq.bits.reqaddr.tt := LCG(2, active)
  mleWrReq.bits.data := LCG(dataWidth, active)
  mleWrReq.bits.mlereq := 0.U

  //mseReadReq
  val mseReadReq = Wire(DecoupledIO(new TrTileMseReadReq))
  mseReadReq := DontCare
  mseReadReq.valid := false.B
  mseReadReq.bits.reqaddr.addr := LCG(3, active)
  mseReadReq.bits.reqaddr.index := LCG(32, active)
  mseReadReq.bits.reqaddr.tt := LCG(2, active)
  mseReadReq.bits.msereq := 0.U

  //mmvWrReq
  val mmvWrReq = Wire(DecoupledIO(new TrTileMmvWrReq))
  mmvWrReq := DontCare
  mmvWrReq.valid := false.B
  mmvWrReq.bits.reqaddr.addr := LCG(3, active)
  mmvWrReq.bits.reqaddr.index := LCG(32, active)
  mmvWrReq.bits.reqaddr.tt := LCG(2, active)
  mmvWrReq.bits.data := LCG(dataWidth, active)
  mmvWrReq.bits.mmvWrReq := 0.U
  //mmvRead
  val mmvReadReq = Wire(DecoupledIO(new TrTileMmvReadReq))
  mmvReadReq := DontCare
  mmvReadReq.valid := false.B
  mmvReadReq.bits.reqaddr.addr := LCG(3, active)
  mmvReadReq.bits.reqaddr.index := LCG(32, active)
  mmvReadReq.bits.reqaddr.tt := LCG(2, active)
  mmvReadReq.bits.mmvReadreq := 0.U
/*
  //case1: Write then Read REG0 (msew = 0, tt = tr_row)
  //mleWrReq
  when(cycle >= case1_start.U && cycle < (case1_start + 6).U) {
    csr.msew := 0.U
    mleWrReq.bits.reqaddr.addr := 0.U
    mleWrReq.bits.reqaddr.index := 0.U
    mleWrReq.bits.reqaddr.tt := 2.U
    mleWrReq.bits.mlereq := 1.U
    mleWrReq.bits.data := Cat(data4, data3, data2, data1, data4, data3, data2, data1)
    when(cycle === (case1_start + 1).U) {
      mleWrReq.valid := true.B
      mleWrReq.bits.reqaddr.index := 0.U
      //mleWrReq.bits.data := Cat(data4, data3, data2, data1, data4, data3, data2, data1)
    }.elsewhen(cycle === (case1_start + 2).U) {
      mleWrReq.valid := true.B
      mleWrReq.bits.reqaddr.index := 1.U
      //mleWrReq.bits.data := Cat(data4, data3, data2, data1, data4, data3, data2, data1)
    }.elsewhen(cycle === (case1_start + 3).U) {
      mleWrReq.valid := true.B
      mleWrReq.bits.reqaddr.index := 2.U
      //mleWrReq.bits.data := Cat(data4, data3, data2, data1, data4, data3, data2, data1)
    }.elsewhen(cycle === (case1_start + 4).U) {
      mleWrReq.valid := true.B
      mleWrReq.bits.reqaddr.index := 3.U
      //mleWrReq.bits.data := Cat(data4, data3, data2, data1, data4, data3, data2, data1)
    }.elsewhen(cycle > (case1_start + 5).U) {
      mleWrReq.valid := false.B
    }
  }
  //mseReadReq
  when(cycle >= (case1_start + 6).U && cycle < (case1_start + 12).U) {
    csr.msew := 0.U
    mseReadReq.bits.reqaddr.addr := 0.U
    mseReadReq.bits.reqaddr.index := 0.U
    mseReadReq.bits.reqaddr.tt := 2.U
    mseReadReq.bits.reqaddr.dim := 2.U
    mseReadReq.bits.msereq := 1.U
    when(cycle === (case1_start + 7).U) {
      mseReadReq.valid := true.B
      mseReadReq.bits.reqaddr.index := 0.U
    }.elsewhen(cycle === (case1_start + 8).U) {
      mseReadReq.valid := true.B
      mseReadReq.bits.reqaddr.index := 1.U
    }.elsewhen(cycle === (case1_start + 9).U) {
      mseReadReq.valid := true.BU
      csr.msew := 1.U
      mseReadReq.bits.reqaddr.index := 2.U
    }.elsewhen(cycle === (case1_start + 10).U) {
      mseReadReq.valid := true.B
      csr.msew := 2.U
      mseReadReq.bits.reqaddr.index := 1.U
      printf("case1 Write then Read REG0 (msew = 0, tt = tr_row)")
    }.elsewhen(cycle > (case1_start + 11).U) {
      mseReadReq.valid := false.B
    }
  }

  //case2: Write then Read REG2 (msew = 2, tt = tr_row)
  //mmvWrReq
  when(cycle >= case2_start.U && cycle < (case2_start + 6).U) {
    csr.msew := 2.U
    mmvWrReq.bits.reqaddr.addr := 2.U
    mmvWrReq.bits.reqaddr.index := 0.U
    mmvWrReq.bits.reqaddr.tt := 2.U
    mmvWrReq.bits.mmvWrReq := 1.U
    mmvWrReq.bits.data := Cat(data4, data3, data2, data1, data4, data3, data2, data1)
    when(cycle === (case2_start + 0).U) {
      mmvWrReq.valid := true.B
      mmvWrReq.bits.reqaddr.index := 1.U
      //mmvWrReq.bits.data := Cat((0 until dataWidth).map(i => 5.U(32.W)).reverse)
    }.elsewhen(cycle === (case2_start + 1).U) {
      mmvWrReq.valid := true.B
      mmvWrReq.bits.reqaddr.index := 0.U
      //mmvWrReq.bits.data := Cat((0 until dataWidth).map(i => 6.U(32.W)).reverse)
    }.elsewhen(cycle > (case2_start + 2).U) {
      mmvWrReq.valid := false.B
    }
  }
  //mmvReadReq
  when(cycle >= (case2_start + 6).U && cycle < (case2_start + 12).U) {
    csr.msew := 2.U
    mmvReadReq.bits.reqaddr.addr := 2.U
    mmvReadReq.bits.reqaddr.index := 0.U
    mmvReadReq.bits.reqaddr.tt := 2.U
    mmvReadReq.bits.mmvReadreq := 1.U
    when(cycle === (case2_start + 7).U) {
      mmvReadReq.valid := true.B
      mmvReadReq.bits.reqaddr.index := 1.U
    }.elsewhen(cycle === (case2_start + 8).U) {
      mmvReadReq.valid := true.B
      mmvReadReq.bits.reqaddr.index := 0.U
    }.elsewhen(cycle > (case2_start + 9).U) {
      mmvReadReq.valid := false.B
      printf("case1 Write then Read REG2 (msew = 2, tt = tr_row)")
    }
  }

  //case3: Read REG0 (tt = tr_col)
  //mseReadReq
  when(cycle >= (case2_start + 14).U && cycle < (case2_start + 20).U) {
    csr.msew := 0.U
    mseReadReq.bits.reqaddr.addr := 0.U
    mseReadReq.bits.reqaddr.index := 0.U
    mseReadReq.bits.reqaddr.tt := 3.U
    mseReadReq.bits.msereq := 1.U
    when(cycle === (case2_start + 14).U) {
      mseReadReq.valid := true.B
      mseReadReq.bits.reqaddr.index := 0.U
    }.elsewhen(cycle === (case2_start + 15).U) {
      mseReadReq.valid := true.B
      mseReadReq.bits.reqaddr.index := 1.U
    }.elsewhen(cycle === (case2_start + 16).U) {
      mseReadReq.valid := true.B
      mseReadReq.bits.reqaddr.index := 0.U
      csr.msew := 3.U
    }.elsewhen(cycle === (case2_start + 17).U) {
      mseReadReq.valid := true.B
      mseReadReq.bits.reqaddr.index := 1.U
      csr.msew := 2.U
      printf("case3 Read REG0 (tt = tr_col)")
    }.elsewhen(cycle > (case2_start + 18).U) {
      mseReadReq.valid := false.B
    }
  }
*/
  //case4: Write then Read REG6/7 (tt = tr_col)
  //mleWrReq
  when(cycle >= case3_start.U && cycle < (case3_start + 6).U) {
    csr.msew := 3.U
    mleWrReq.bits.reqaddr.addr := 7.U
    mleWrReq.bits.reqaddr.index := 0.U
    mleWrReq.bits.reqaddr.tt := 3.U
    mleWrReq.bits.mlereq := 1.U
    mleWrReq.bits.data := Cat(data4, data3, data2, data1, data4, data3, data2, data1)
    when(cycle === (case3_start + 1).U) {
      mleWrReq.valid := true.B
      mleWrReq.bits.reqaddr.index := 0.U
    }.elsewhen(cycle === (case3_start + 2).U) {
      mleWrReq.valid := true.B
      csr.msew := 1.U
      mleWrReq.bits.reqaddr.addr := 6.U
      mleWrReq.bits.reqaddr.index := 0.U
    }.elsewhen(cycle === (case3_start + 6).U) {
      mleWrReq.valid := true.B
      csr.msew := 1.U
      mleWrReq.bits.reqaddr.addr := 6.U
      mleWrReq.bits.reqaddr.index := 1.U
    }.elsewhen(cycle === (case3_start + 3).U) {
      mleWrReq.valid := true.B
      csr.msew := 1.U
      mleWrReq.bits.reqaddr.addr := 6.U
      mleWrReq.bits.reqaddr.index := 2.U
    }.elsewhen(cycle === (case3_start + 4).U) {
      mleWrReq.valid := true.B
      csr.msew := 1.U
      mleWrReq.bits.reqaddr.addr := 6.U
      mleWrReq.bits.reqaddr.index := 3.U
  }.elsewhen(cycle > (case3_start + 5).U) {
      mleWrReq.valid := false.B
    }
  }
  //mseReadReq
  when(cycle >= (case3_start + 6).U && cycle < (case3_start + 12).U) {
    csr.msew := 3.U
    mseReadReq.bits.reqaddr.addr := 7.U
    mseReadReq.bits.reqaddr.index := 0.U
    mseReadReq.bits.reqaddr.tt := 3.U
    mseReadReq.bits.msereq := 1.U
    when(cycle === (case3_start + 7).U) {
      mseReadReq.valid := true.B
      mseReadReq.bits.reqaddr.index := 0.U
    }.elsewhen(cycle === (case3_start + 8).U) {
      mseReadReq.valid := true.B
      csr.msew := 1.U
      mseReadReq.bits.reqaddr.addr := 6.U
      mseReadReq.bits.reqaddr.index := 0.U
    }.elsewhen(cycle === (case3_start + 9).U) {
      mseReadReq.valid := true.B
      csr.msew := 1.U
      mseReadReq.bits.reqaddr.addr := 6.U
      mseReadReq.bits.reqaddr.index := 1.U
    }.elsewhen(cycle === (case3_start + 10).U) {
      mseReadReq.valid := true.B
      csr.msew := 1.U
      mseReadReq.bits.reqaddr.addr := 6.U
      mseReadReq.bits.reqaddr.index := 2.U
    }.elsewhen(cycle === (case3_start + 11).U) {
      mseReadReq.valid := true.B
      csr.msew := 1.U
      mseReadReq.bits.reqaddr.addr := 6.U
      mseReadReq.bits.reqaddr.index := 3.U
      printf("case4 REG6 REG7 (tt = tr_col)")
    }.elsewhen(cycle > (case3_start + 10).U) {
      mseReadReq.valid := false.B
    }
  }


  /*
  //case5: Write REG5 (msew = 1, tt = tr_row), then read(msew = 1, tt = tr_col)
  //mleWrReq
  when(cycle >= case5_start.U && cycle < (case5_start + 6).U) {
    csr.msew := 1.U
    mleWrReq.bits.reqaddr.addr := 5.U
    mleWrReq.bits.reqaddr.index := 0.U
    mleWrReq.bits.reqaddr.tt := 2.U
    mleWrReq.bits.mlereq := 1.U
    mleWrReq.bits.data := 0.U
    when(cycle === (case5_start + 1).U) {
      mleWrReq.valid := true.B
      mleWrReq.bits.reqaddr.index := 0.U
      mleWrReq.bits.data := Cat(data4, data3, data2, data1, data4, data3, data2, data1)
    }.elsewhen(cycle === (case5_start + 2).U) {
      mleWrReq.valid := true.B
      mleWrReq.bits.reqaddr.index := 1.U
      mleWrReq.bits.data := Cat(Fill(4, data1), Fill(4, data4))
    }.elsewhen(cycle === (case5_start + 3).U) {
      mleWrReq.valid := true.B
      mleWrReq.bits.reqaddr.index := 2.U
      mleWrReq.bits.data := Cat(data1, data2, data3, data4, data1, data2, data3, data4)
    }.elsewhen(cycle === (case5_start + 4).U) {
      mleWrReq.valid := true.B
      mleWrReq.bits.reqaddr.index := 3.U
      mleWrReq.bits.data := Cat(Fill(4, data3), Fill(4, data2))
    }.elsewhen(cycle > (case5_start + 5).U) {
      mleWrReq.valid := false.B
    }
  }
  //mseReadReq
  when(cycle >= (case5_start + 6).U && cycle < (case5_start + 12).U) {
    csr.msew := 1.U
    mseReadReq.bits.reqaddr.addr := 5.U
    mseReadReq.bits.reqaddr.index := 0.U
    mseReadReq.bits.reqaddr.tt := 3.U
    mseReadReq.bits.msereq := 1.U
    when(cycle === (case5_start + 7).U) {
      mseReadReq.valid := true.B
      mseReadReq.bits.reqaddr.index := 0.U
    }.elsewhen(cycle === (case5_start + 8).U) {
      mseReadReq.valid := true.B
      mseReadReq.bits.reqaddr.index := 1.U
    }.elsewhen(cycle === (case5_start + 9).U) {
      mseReadReq.valid := true.B
      mseReadReq.bits.reqaddr.index := 2.U
    }.elsewhen(cycle === (case5_start + 10).U) {
      mseReadReq.valid := true.B
      mseReadReq.bits.reqaddr.index := 3.U
      printf("case5Write then Read REG5 (msew = 1, tt = tr_col)")
    }.elsewhen(cycle > (case5_start + 11).U) {
      mseReadReq.valid := false.B
    }
  }
*/
  //printf source
  when(mmvReadReq.valid) {
    printf("mmvReadreq: uopc %x, tt %x, addr %x, index %x, msew %x\n",
      mmvReadReq.bits.mmvReadreq,
      mmvReadReq.bits.reqaddr.tt,
      mmvReadReq.bits.reqaddr.addr,
      mmvReadReq.bits.reqaddr.index,
      csr.msew)
  }
  when(mmvWrReq.valid) {
    printf("mmvWrreq: uopc %x, tt %x, addr %x, data %x, index %x, msew %x\n",
      mmvWrReq.bits.mmvWrReq,
      mmvWrReq.bits.reqaddr.tt,
      mmvWrReq.bits.reqaddr.addr,
      mmvWrReq.bits.data,
      mmvWrReq.bits.reqaddr.index,
      csr.msew)
  }
  when(mseReadReq.valid) {
    printf("mseReadreq: uopc %x, tt %x, addr %x, index %x, msew %x\n",
      mseReadReq.bits.msereq,
      mseReadReq.bits.reqaddr.tt,
      mseReadReq.bits.reqaddr.addr,
      mseReadReq.bits.reqaddr.index,
      csr.msew)
  }
  when(mleWrReq.valid) {
    printf("mleWrreq: uopc %x, tt %x, addr %x, data %x, index %x, msew %x\n",
      mleWrReq.bits.mlereq,
      mleWrReq.bits.reqaddr.tt,
      mleWrReq.bits.reqaddr.addr,
      mleWrReq.bits.data,
      mleWrReq.bits.reqaddr.index,
      csr.msew)
  }

  val dut = Module(new TrTile(8,2))
  dut.io.trtilecsr := csr
  dut.io.mleWrReq := mleWrReq
  dut.io.mmvWrReq := mmvWrReq
  dut.io.mmvReadReq := mmvReadReq
  dut.io.mseReadReq := mseReadReq

  //printf result
  when(dut.io.mmvReadResp.valid) {
    printf("mmvReadResp %x\n", dut.io.mmvReadResp.bits)
  }
  when(dut.io.mseReadResp.valid) {
    printf("mseReadResp %x\n", dut.io.mseReadResp.bits)
  }

}

class WithTrTileUT extends Config((site, here, up) => {
  case UnitTests => (q: Parameters) => {
    implicit val p = BoomTestUtils.getBoomParameters("StcBoomConfig", "chipyard")
    Seq(
      Module(new TrTileUT(5000))
    )
  }
})

class TrTileUTConfig extends Config(new WithTrTileUT)


