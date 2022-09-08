// See README.md for license details.
package boom.exu

import chisel3._
import chisel3.util._
import freechips.rocketchip.tile.FType
import freechips.rocketchip.config.Parameters
import hardfloat._

import boom.common._

// acc buffer related command
object AccTileConstants 
{
  // access cmd
  val SLICE_READ  = 0.U(1.W)
  val SLICE_WRITE = 1.U(1.W)
  // register type
  val INT8TYPE    = 0.U(3.W)        // cat(fp_val, sew)
  val INT16TYPE   = 1.U(3.W)
  val INT32TYPE   = 2.U(3.W)
  val INT64TYPE   = 3.U(3.W)
  val FP16TYPE    = 5.U(3.W)
  val FP32TYPE    = 6.U(3.W)
  val FP64TYPE    = 7.U(3.W)
}

import AccTileConstants._

class MacCtrls(val numAccTiles: Int = 2) extends Bundle 
{
  val srcRidx = UInt(log2Ceil(numAccTiles).W)
  val dstRidx = UInt(log2Ceil(numAccTiles).W)
  val srcType = UInt(3.W)
  val outType = UInt(3.W)
  val rm      = UInt(3.W)                      // rounding mode
}

class ClrCtrls(val numAccTiles: Int = 2) extends Bundle {
  val ridx  = UInt(log2Ceil(numAccTiles).W)    // register index
  // val rtype = UInt(3.W)                     // SEW = 8bits, 16bits, 32bits
}

class SliceCtrls(val sliceNums: Int, val numAccTiles: Int = 2) extends Bundle {
  val ridx  = UInt(log2Ceil(numAccTiles).W)    // register index
  val sidx  = UInt(log2Ceil(sliceNums).W)      // slice index
  val rtype = UInt(3.W)                        // SEW = 8bits, 16bits, 32bits
}

// acc 32-bits
class IntMacUnit extends Module 
{
  val io = IO(new Bundle {
    val src1    = Input(UInt(8.W))
    val src2    = Input(UInt(8.W))
    val src3    = Input(UInt(32.W))
    val srcType = Input(UInt(3.W))
    val outType = Input(UInt(3.W))
    val out     = Output(UInt(32.W))
  })

  val sMax = WireInit(0.U(32.W))
  val sMin = WireInit(0.U(32.W))
  sMax := Mux(io.outType === INT8TYPE,  0x7F.U,
          Mux(io.outType === INT16TYPE, 0x7FFF.U, 0x7FFFFFFF.U))
  sMin := Mux(io.outType === INT8TYPE,  Cat(Fill(24, 1.U(1.W)), 0x80.U(8.W)),
          Mux(io.outType === INT16TYPE, Cat(Fill(16, 1.U(1.W)), 0x8000.U(16.W)), Cat(1.U(1.W), Fill(31, 0.U(1.W)))))

  val lhs  = io.src1.asSInt
  val rhs  = io.src2.asSInt
  val acc  = WireInit(0.U(32.W))
  acc := Mux(io.outType === INT8TYPE,  Cat(Fill(24, io.src3(7)),  io.src3( 7, 0)),
         Mux(io.outType === INT16TYPE, Cat(Fill(16, io.src3(15)), io.src3(15, 0)), io.src3))
  val macc = lhs * rhs +& acc.asSInt

  io.out := Mux(macc > sMax.asSInt, sMax,
            Mux(macc < sMin.asSInt, sMin, macc(31, 0)))
}

// may use blackbox hardfloat modules
class FpMacUnit extends Module 
{
  val io = IO(new Bundle {
    val src1           = Input(UInt(16.W))
    val src2           = Input(UInt(16.W))
    val src3           = Input(UInt(32.W))
    val srcType        = Input(UInt(3.W))
    val outType        = Input(UInt(3.W))
    val roundingMode   = Input(UInt(3.W))
    val detectTininess = Input(UInt(1.W))
    val out            = Output(UInt(32.W))
  })
  val H = new FType(5, 11)
  val S = new FType(8, 24)
  // convert fp16 or fp32 to recoded format
  val recA     = recFNFromFN(H.exp, H.sig, io.src1)
  val recB     = recFNFromFN(H.exp, H.sig, io.src2)
  val recCFP16 = recFNFromFN(H.exp, H.sig, io.src3(15, 0))
  val recCFP32 = recFNFromFN(S.exp, S.sig, io.src3)

  val rec16ToRec32 = Module(new RecFNToRecFN(H.exp, H.sig, S.exp, S.sig))
  rec16ToRec32.io.in := recCFP16
  rec16ToRec32.io.roundingMode   := io.roundingMode
  rec16ToRec32.io.detectTininess := io.detectTininess

  val recC = Mux(io.srcType === FP32TYPE, recCFP32, rec16ToRec32.io.out)

  // recoded fp16 * fp16
  val recMul = Module(new MulRecFNToFullRaw(H.exp, H.sig))
  recMul.io.a := recA
  recMul.io.b := recB
  val recMulOut = Wire(new RawFloat(H.exp, H.sig*2-1))
  recMulOut.isNaN  := recMul.io.out_isNaN
  recMulOut.isInf  := recMul.io.out_isInf
  recMulOut.isZero := recMul.io.out_isZero
  recMulOut.sign   := recMul.io.out_sign
  recMulOut.sExp   := recMul.io.out_sExp
  recMulOut.sig    := recMul.io.out_sig

  // for debug
  val recMulFN = Module(new MulRecFN(H.exp, H.sig))
  recMulFN.io.a := recA
  recMulFN.io.b := recB
  recMulFN.io.roundingMode := io.roundingMode
  val recMulFNOut = fNFromRecFN(H.exp, H.sig, recMulFN.io.out)

  // round raw results to recFN(32)
  val anyRawToRec = Module(new RoundAnyRawFNToRecFN(H.exp, H.sig*2-1, S.exp, S.sig, 0))
  anyRawToRec.io.invalidExc     := recMul.io.invalidExc
  anyRawToRec.io.infiniteExc    := false.B
  anyRawToRec.io.in             := recMulOut
  anyRawToRec.io.roundingMode   := io.roundingMode
  anyRawToRec.io.detectTininess := io.detectTininess

  val anyRawToRecFP16 = Module(new RoundAnyRawFNToRecFN(H.exp, H.sig*2-1, H.exp, H.sig, 0))
  anyRawToRecFP16.io.invalidExc     := recMul.io.invalidExc
  anyRawToRecFP16.io.infiniteExc    := false.B
  anyRawToRecFP16.io.in             := recMulOut
  anyRawToRecFP16.io.roundingMode   := io.roundingMode
  anyRawToRecFP16.io.detectTininess := io.detectTininess

  // recoded fp32 + fp32
  val recAdd = Module(new AddRecFNToRaw(S.exp, S.sig))
  recAdd.io.subOp   := false.B
  recAdd.io.a       := recC
  recAdd.io.b       := anyRawToRec.io.out
  recAdd.io.roundingMode := io.roundingMode
  val recAddOut = Wire(new RawFloat(S.exp, S.sig+2))
  recAddOut.isNaN  := recAdd.io.out_isNaN
  recAddOut.isInf  := recAdd.io.out_isInf
  recAddOut.isZero := recAdd.io.out_isZero
  recAddOut.sign   := recAdd.io.out_sign
  recAddOut.sExp   := recAdd.io.out_sExp
  recAddOut.sig    := recAdd.io.out_sig

  // raw to FP32
  val rawToRec32 = Module(new RoundRawFNToRecFN(S.exp, S.sig, 0))
  rawToRec32.io.invalidExc     := recAdd.io.invalidExc
  rawToRec32.io.infiniteExc    := false.B
  rawToRec32.io.in             := recAddOut
  rawToRec32.io.roundingMode   := io.roundingMode
  rawToRec32.io.detectTininess := io.detectTininess
  val fp32Out = fNFromRecFN(S.exp, S.sig, rawToRec32.io.out)

  // raw to FP16
  val rawToRec16 = Module(new RoundAnyRawFNToRecFN(S.exp, S.sig+2, H.exp, H.sig, 0))
  rawToRec16.io.invalidExc     := recAdd.io.invalidExc
  rawToRec16.io.infiniteExc    := false.B
  rawToRec16.io.in             := recAddOut
  rawToRec16.io.roundingMode   := io.roundingMode
  rawToRec16.io.detectTininess := io.detectTininess
  val fp16Out = fNFromRecFN(H.exp, H.sig, rawToRec16.io.out)

  io.out := Mux(io.outType === FP16TYPE, Cat(0.U(16.W), fp16Out), fp32Out)
  
}

// TODO update documentation
/**
  * A PE implementing a fp16 MAC operation or two int8 MAC operations.
  * c1 = a1 * b1 + c1 (for fp16 and int8)
  * c2 = a2 * b2 + c2 (for int8 only)
  * A PE is located in a two-dimensional MESH, rowIndex and colIndex indicate its location in horizontal and vertical directions.
  */
class PE(
  val numMeshRows: Int, 
  val numMeshCols: Int, 
  val rowIndex:    Int, 
  val colIndex:    Int, 
  val numAccTiles: Int = 2
) extends Module 
{
  val io = IO(new Bundle{
    // matrix multiplication related: mutiply-accumulate
    val macReqIn     = Input(Valid(new MacCtrls(numAccTiles)))            // control signals
    val macReqSrcA   = Input(UInt(16.W))
    val macReqSrcB   = Input(UInt(16.W))
    val macReqOut    = Output(Valid(new MacCtrls(numAccTiles)))
    val macOutSrcA   = Output(UInt(16.W))                      // for systolic horizontally
    val macOutSrcB   = Output(UInt(16.W))                      // for systolic vertically
    // clear tile slices, control signals propagated vertically
    val clrReqIn     = Input(Valid(new ClrCtrls(numAccTiles)))
    val clrReqOut    = Output(Valid(new ClrCtrls(numAccTiles)))
    // read row slices, control signals propagated vertically
    val rowReadReq   = Input(Valid(new SliceCtrls(numMeshRows, numAccTiles)))
    val rowReadDin   = Input(UInt(64.W))
    val rowReadResp  = Output(Valid(new SliceCtrls(numMeshRows, numAccTiles)))
    val rowReadDout  = Output(UInt(64.W))
    // write row slices, control signals propagated vertically
    val rowWriteReq  = Input(Valid(new SliceCtrls(numMeshRows, numAccTiles)))
    val rowWriteDin  = Input(UInt(64.W))
    val rowWriteResp = Output(Valid(new SliceCtrls(numMeshRows, numAccTiles)))
    val rowWriteDout = Output(UInt(64.W))
    // read col slices, control signals propagated horizontally
    val colReadReq   = Input(Valid(new SliceCtrls(numMeshCols, numAccTiles)))
    val colReadDin   = Input(UInt(32.W))
    val colReadResp  = Output(Valid(new SliceCtrls(numMeshCols, numAccTiles)))
    val colReadDout  = Output(UInt(32.W))
    // write col slices, control signals propagated horizontally
    val colWriteReq  = Input(Valid(new SliceCtrls(numMeshCols, numAccTiles)))
    val colWriteDin  = Input(UInt(32.W))
    val colWriteResp = Output(Valid(new SliceCtrls(numMeshCols, numAccTiles)))
    val colWriteDout = Output(UInt(32.W))
  })

  // acc registers
  val c0 = RegInit(VecInit(Seq.fill(numAccTiles)(0.U(32.W))))
  val c1 = RegInit(VecInit(Seq.fill(numAccTiles)(0.U(32.W))))

  // -----------------------------------------------------------------------------------
  // matrix multiply-accumulate
  // -----------------------------------------------------------------------------------
  val macReqValid = io.macReqIn.valid
  val macReqCtrls = io.macReqIn.bits
  // fp16*fp16+fp32
  val fpMac = Module(new FpMacUnit())
  fpMac.io.src1           := io.macReqSrcA
  fpMac.io.src2           := io.macReqSrcB
  fpMac.io.src3           := c0(macReqCtrls.srcRidx)
  fpMac.io.srcType        := macReqCtrls.srcType
  fpMac.io.outType        := macReqCtrls.outType
  fpMac.io.roundingMode   := macReqCtrls.rm
  fpMac.io.detectTininess := hardfloat.consts.tininess_afterRounding

  // int8 MAC function units
  val intMac0 = Module(new IntMacUnit())
  val intMac1 = Module(new IntMacUnit())
  intMac0.io.src1    := io.macReqSrcA( 7, 0)
  intMac0.io.src2    := io.macReqSrcB( 7, 0)
  intMac0.io.src3    := c0(macReqCtrls.srcRidx)
  intMac0.io.srcType := macReqCtrls.srcType
  intMac0.io.outType := macReqCtrls.outType
  intMac1.io.src1    := io.macReqSrcA( 7, 0)
  intMac1.io.src2    := io.macReqSrcB(15, 8)
  intMac1.io.src3    := c1(macReqCtrls.srcRidx)
  intMac1.io.srcType := macReqCtrls.srcType
  intMac1.io.outType := macReqCtrls.outType

  // latency = 1 for int8 mac; 3 for fp16 mac
  when(macReqValid && macReqCtrls.srcType(2)) {
    c0(macReqCtrls.dstRidx) := fpMac.io.out
    // c0(macReqCtrls.ridx) := intMac0.io.out
  } .elsewhen(macReqValid) {
    c0(macReqCtrls.dstRidx) := intMac0.io.out
    c1(macReqCtrls.dstRidx) := intMac1.io.out
  }

  io.macReqOut   := io.macReqIn
  io.macOutSrcA  := io.macReqSrcA
  io.macOutSrcB  := io.macReqSrcB

  // -----------------------------------------------------------------------------------
  // clear acc tiles
  // -----------------------------------------------------------------------------------
  val clrReqValid = io.clrReqIn.valid
  val clrReqCtrls = io.clrReqIn.bits
  when(clrReqValid) {
    c0(clrReqCtrls.ridx) := 0.U
    c1(clrReqCtrls.ridx) := 0.U
  }

  io.clrReqOut := io.clrReqIn

  // -----------------------------------------------------------------------------------
  // read row slices
  // -----------------------------------------------------------------------------------
  val rowReadValid = io.rowReadReq.valid
  val rowReadCtrls = io.rowReadReq.bits
  val rowRdata     = WireInit(0.U(64.W))
  val rowReadHit   = rowReadValid && rowReadCtrls.sidx === rowIndex.U
  rowRdata := Mux(!rowReadHit, io.rowReadDin,
              Mux(rowReadCtrls.rtype(2), c0(rowReadCtrls.ridx), Cat(c1(rowReadCtrls.ridx), c0(rowReadCtrls.ridx))))

  io.rowReadResp := io.rowReadReq
  io.rowReadDout := rowRdata

  // -----------------------------------------------------------------------------------
  // write row slices
  // -----------------------------------------------------------------------------------
  val rowWriteValid = io.rowWriteReq.valid
  val rowWrtieCtrls = io.rowWriteReq.bits
  val rowWriteHit   = rowWriteValid && rowWrtieCtrls.sidx === rowIndex.U

  when(rowWriteHit) {
    c0(rowWrtieCtrls.ridx) := io.rowWriteDin(31, 0)
    when(!rowWrtieCtrls.rtype(2)) { c1(rowWrtieCtrls.ridx) := io.rowWriteDin(63, 32) }
  }

  io.rowWriteResp := io.rowWriteReq
  io.rowWriteDout := io.rowWriteDin

  // -----------------------------------------------------------------------------------
  // read col slices
  // -----------------------------------------------------------------------------------
  val colReadValid = io.colReadReq.valid
  val colReadCtrls = io.colReadReq.bits
  val colRdata     = WireInit(0.U(32.W))
  val colReadHitC0 = colReadValid && Mux(colReadCtrls.rtype(2), colReadCtrls.sidx === (colIndex >> 1).U, colReadCtrls.sidx === colIndex.U)
  val colReadHitC1 = colReadValid && !colReadCtrls.rtype(2) && colReadCtrls.sidx === (colIndex + 1).U
  colRdata := Mux(colReadHitC0, c0(colReadCtrls.ridx), 
              Mux(colReadHitC1, c1(colReadCtrls.ridx), io.colReadDin))

  io.colReadResp := io.colReadReq
  io.colReadDout := colRdata

  // -----------------------------------------------------------------------------------
  // write col slices
  // -----------------------------------------------------------------------------------
  val colWriteValid = io.colWriteReq.valid
  val colWriteCtrls = io.colWriteReq.bits
  val colWriteHitC0 = colWriteValid && Mux(colWriteCtrls.rtype(2), colWriteCtrls.sidx === (colIndex >> 1).U, colWriteCtrls.sidx === colIndex.U)
  val colWriteHitC1 = colWriteValid && !colWriteCtrls.rtype(2) && colWriteCtrls.sidx === (colIndex + 1).U

  when(colWriteHitC0) {
    c0(colWriteCtrls.ridx) := io.colWriteDin
  } .elsewhen(colWriteHitC1) {
    c1(colWriteCtrls.ridx) := io.colWriteDin
  }

  io.colWriteResp := io.colWriteReq
  io.colWriteDout := io.colWriteDin
}

/**
  * A Tile is a purely combinational 2D array of passThrough PEs.
  */
class Tile(
  val numMeshRows: Int, 
  val numMeshCols: Int, 
  val indexh:      Int, 
  val indexv:      Int, 
  val tileRows:    Int, 
  val tileCols:    Int,
  val numAccTiles: Int = 2
) extends Module 
{
  val io = IO(new Bundle{
    // matrix multiplication related: mutiply-accumulate
    val macReqIn     = Input(Valid(new MacCtrls(numAccTiles)))
    val macReqSrcA   = Input(UInt((tileRows*16).W))
    val macReqSrcB   = Input(UInt((tileCols*16).W))
    val macReqOut    = Output(Valid(new MacCtrls(numAccTiles)))
    val macOutSrcA   = Output(UInt((tileRows*16).W))
    val macOutSrcB   = Output(UInt((tileCols*16).W))
    // clear tile slices, control signals propagated vertically
    val clrReqIn     = Input(Valid(new ClrCtrls(numAccTiles)))
    val clrReqOut    = Output(Valid(new ClrCtrls(numAccTiles)))
    // read row slices, control signals propagated vertically
    val rowReadReq   = Input(Valid(new SliceCtrls(numMeshRows, numAccTiles)))
    val rowReadDin   = Input(UInt((tileCols*64).W))
    val rowReadResp  = Output(Valid(new SliceCtrls(numMeshRows, numAccTiles)))
    val rowReadDout  = Output(UInt((tileCols*64).W))
    // write row slices, control signals propagated vertically
    val rowWriteReq  = Input(Valid(new SliceCtrls(numMeshRows, numAccTiles)))
    val rowWriteDin  = Input(UInt((tileCols*64).W))
    val rowWriteResp = Output(Valid(new SliceCtrls(numMeshRows, numAccTiles)))
    val rowWriteDout = Output(UInt((tileCols*64).W))
    // read col slice, control signals propagated horizontally
    val colReadReq   = Input(Valid(new SliceCtrls(numMeshCols, numAccTiles)))
    val colReadDin   = Input(UInt((tileRows*32).W))
    val colReadResp  = Output(Valid(new SliceCtrls(numMeshCols, numAccTiles)))
    val colReadDout  = Output(UInt((tileRows*32).W))
    // write col slice, control signals propagated horizontally
    val colWriteReq  = Input(Valid(new SliceCtrls(numMeshCols, numAccTiles)))
    val colWriteDin  = Input(UInt((tileRows*32).W))
    val colWriteResp = Output(Valid(new SliceCtrls(numMeshCols, numAccTiles)))
    val colWriteDout = Output(UInt((tileRows*32).W))
  })

  val tile = Seq.tabulate(tileRows, tileCols)((i, j) => Module(new PE(numMeshRows, numMeshCols, indexh+i, indexv+j*2, numAccTiles)))
  val tileT = tile.transpose

  // broadcast horizontally across the tile
  for(r <- 0 until tileRows) {
    val peSrcA = io.macReqSrcA(16*r+15, 16*r)
    tile(r).foldLeft(peSrcA) {
      case (macReqSrcA, pe) => {
        pe.io.macReqSrcA := macReqSrcA
        pe.io.macOutSrcA
      }
    }
    tile(r).foldLeft(io.colReadReq) {
      case (colReadReq, pe) => {
        pe.io.colReadReq := colReadReq
        pe.io.colReadResp
      }
    }
    val peColRdata = io.colReadDin(32*r+31, 32*r)
    tile(r).foldLeft(peColRdata) {
      case (colReadData, pe) => {
        pe.io.colReadDin := colReadData
        pe.io.colReadDout
      }
    }
    tile(r).foldLeft(io.colWriteReq) {
      case (colWriteReq, pe) => {
        pe.io.colWriteReq := colWriteReq
        pe.io.colWriteResp
      }
    }
    val peColWdata = io.colWriteDin(32*r+31, 32*r)
    tile(r).foldLeft(peColWdata) {
      case (colWriteData, pe) => {
        pe.io.colWriteDin := colWriteData
        pe.io.colWriteDout
      }
    }
  }

  // broadcast vertically
  for(c <- 0 until tileCols) {
    tileT(c).foldLeft(io.macReqIn) {
      case (macReq, pe) => {
        pe.io.macReqIn := macReq
        pe.io.macReqOut
      }
    }
    val peSrcB = io.macReqSrcB(16*c+15, 16*c)
    tileT(c).foldLeft(peSrcB) {
      case(macReqSrcB, pe) => {
        pe.io.macReqSrcB := macReqSrcB
        pe.io.macOutSrcB
      }
    }
    tileT(c).foldLeft(io.clrReqIn) {
      case (clrReq, pe) => {
        pe.io.clrReqIn := clrReq
        pe.io.clrReqOut
      }
    }
    tileT(c).foldLeft(io.rowReadReq) {
      case(rowReadReq, pe) => {
        pe.io.rowReadReq := rowReadReq
        pe.io.rowReadResp
      }
    }
    val peRowRdata = io.rowReadDin(64*c+63, 64*c)
    tileT(c).foldLeft(peRowRdata) {
      case(rowReadData, pe) => {
        pe.io.rowReadDin := rowReadData
        pe.io.rowReadDout
      }
    }
    tileT(c).foldLeft(io.rowWriteReq) {
      case(rowWriteReq, pe) => {
        pe.io.rowWriteReq := rowWriteReq
        pe.io.rowWriteResp
      }
    }
    val peRowWdata = io.rowWriteDin(64*c+63, 64*c)
    tileT(c).foldLeft(peRowWdata) {
      case(rowWriteData, pe) => {
        pe.io.rowWriteDin := rowWriteData
        pe.io.rowWriteDout
      }
    }
  }

  // tile's bottom IO
  io.macReqOut     := io.macReqIn
  io.clrReqOut     := io.clrReqIn
  io.rowReadResp   := io.rowReadReq
  io.rowWriteResp  := io.rowWriteReq
  io.colReadResp   := io.colReadReq
  io.colWriteResp  := io.colWriteReq

  val macOutSrcAMux   = WireInit(VecInit(Seq.fill(tileRows)(0.U(16.W))))
  val colReadDataMux  = WireInit(VecInit(Seq.fill(tileRows)(0.U(32.W))))
  val colWriteDataMux = WireInit(VecInit(Seq.fill(tileRows)(0.U(32.W))))
  for(r <- 0 until tileRows) {
    macOutSrcAMux(r)   := tile(r)(tileCols-1).io.macOutSrcA
    colReadDataMux(r)  := tile(r)(tileCols-1).io.colReadDout
    colWriteDataMux(r) := tile(r)(tileCols-1).io.colWriteDout
  }

  val macOutSrcBMux   = WireInit(VecInit(Seq.fill(tileCols)(0.U(16.W))))
  val rowReadDataMux  = WireInit(VecInit(Seq.fill(tileCols)(0.U(64.W))))
  val rowWriteDataMux = WireInit(VecInit(Seq.fill(tileCols)(0.U(64.W))))
  for(c <- 0 until tileCols) {
    macOutSrcBMux(c)   := tile(tileRows-1)(c).io.macOutSrcB
    rowReadDataMux(c)  := tile(tileRows-1)(c).io.rowReadDout
    rowWriteDataMux(c) := tile(tileRows-1)(c).io.rowWriteDout
  }
  io.macOutSrcA   := macOutSrcAMux.asUInt
  io.macOutSrcB   := macOutSrcBMux.asUInt
  io.colReadDout  := colReadDataMux.asUInt
  io.colWriteDout := colWriteDataMux.asUInt
  io.rowReadDout  := rowReadDataMux.asUInt
  io.rowWriteDout := rowWriteDataMux.asUInt
}

class Mesh(
  val meshRows:    Int, 
  val meshCols:    Int, 
  val tileRows:    Int, 
  val tileCols:    Int,
  val numAccTiles: Int = 2
) extends Module 
{
  val io = IO(new Bundle {
    // matrix multiplication related: mutiply-accumulate
    val macReq       = Input(Vec(meshCols, Valid(new MacCtrls(numAccTiles))))
    val macReqSrcA   = Input(Vec(meshRows, UInt((tileRows*16).W)))
    val macReqSrcB   = Input(Vec(meshCols, UInt((tileCols*16).W)))
    val macResp      = Output(Valid(new MacCtrls(numAccTiles)))
    // clear tile slices, control signals propagated vertically
    val clrReq       = Input(Valid(new ClrCtrls(numAccTiles)))
    val clrResp      = Output(Valid(new ClrCtrls(numAccTiles)))
    // read row slices, control signals propagated vertically
    val rowReadReq   = Input(Valid(new SliceCtrls(meshRows*tileRows, numAccTiles)))
    val rowReadResp  = Output(Valid(new SliceCtrls(meshRows*tileRows, numAccTiles)))
    val rowReadData  = Output(UInt((meshCols*tileCols*64).W))
    // write row slices, control signals propagated vertically
    val rowWriteReq  = Input(Valid(new SliceCtrls(meshRows*tileRows, numAccTiles)))
    val rowWriteData  = Input(UInt((meshCols*tileCols*64).W))
    val rowWriteResp = Output(Valid(new SliceCtrls(meshRows*tileRows, numAccTiles)))
    // read col slice, control signals propagated horizontally
    val colReadReq   = Input(Valid(new SliceCtrls(meshCols*tileCols*2, numAccTiles)))
    val colReadResp  = Output(Valid(new SliceCtrls(meshCols*tileCols*2, numAccTiles)))
    val colReadData  = Output(UInt((meshRows*tileRows*32).W))
    // write col slice, control signals propagated horizontally
    val colWriteReq  = Input(Valid(new SliceCtrls(meshCols*tileCols*2, numAccTiles)))
    val colWriteData  = Input(UInt((meshRows*tileRows*32).W))
    val colWriteResp = Output(Valid(new SliceCtrls(meshCols*tileCols*2, numAccTiles)))
  })

  val mesh  = Seq.tabulate(meshRows, meshCols)((i, j) => Module(new Tile(meshRows*tileRows, meshCols*tileCols*2, i*tileRows, j*tileCols*2, tileRows, tileCols, numAccTiles)))
  val meshT = mesh.transpose

  // propagate horizontally across the mesh
  for(r <- 0 until meshRows) {
    mesh(r).foldLeft(io.macReqSrcA(r)) {
      case(macReqSrcA, tile) => {
        tile.io.macReqSrcA := RegNext(macReqSrcA)
        tile.io.macOutSrcA
      }
    }
    mesh(r).foldLeft(io.colReadReq) {
      case(colReadReq, tile) => {
        tile.io.colReadReq := RegNext(colReadReq)
        tile.io.colReadResp
      }
    }
    mesh(r).foldLeft(0.U(32.W)) {
      case(colReadData, tile) => {
        tile.io.colReadDin := RegNext(colReadData)
        tile.io.colReadDout
      }
    }
    mesh(r).foldLeft(io.colWriteReq) {
      case(colWriteReq, tile) => {
        tile.io.colWriteReq := RegNext(colWriteReq)
        tile.io.colWriteResp
      }
    }
    mesh(r).foldLeft(io.colWriteData(32*tileRows*(r+1)-1, 32*tileRows*r)) {
      case(colWriteData, tile) => {
        tile.io.colWriteDin := RegNext(colWriteData)
        tile.io.colWriteDout
      }
    }
  }

  // propagate vertically across the mesh
  for(c <- 0 until meshCols) {
    meshT(c).foldLeft(io.macReq(c)) {
      case(macReq, tile) => {
        tile.io.macReqIn := RegNext(macReq)
        tile.io.macReqOut
      }
    }
    meshT(c).foldLeft(io.macReqSrcB(c)) {
      case(macReqSrcB, tile) => {
        tile.io.macReqSrcB := RegNext(macReqSrcB)
        tile.io.macOutSrcB
      }
    }
    meshT(c).foldLeft(io.clrReq) {
      case(clrReq, tile) => {
        tile.io.clrReqIn := RegNext(clrReq)
        tile.io.clrReqOut
      }
    }
    meshT(c).foldLeft(io.rowReadReq) {
      case(rowReadReq, tile) => {
        tile.io.rowReadReq := RegNext(rowReadReq)
        tile.io.rowReadResp
      }
    }
    meshT(c).foldLeft(0.U(64.W)) {
      case(rowReadData, tile) => {
        tile.io.rowReadDin := RegNext(rowReadData)
        tile.io.rowReadDout
      }
    }
    meshT(c).foldLeft(io.rowWriteReq) {
      case(rowWriteReq, tile) => {
        tile.io.rowWriteReq := RegNext(rowWriteReq)
        tile.io.rowWriteResp
      }
    }
    meshT(c).foldLeft(io.rowWriteData(64*tileCols*(c+1)-1, 64*tileCols*c)) {
      case(rowWriteData, tile) => {
        tile.io.rowWriteDin := RegNext(rowWriteData)
        tile.io.rowWriteDout
      }
    }
  }

  // bottom IOs
  io.macResp      := mesh(meshRows-1)(meshCols-1).io.macReqOut
  io.clrResp      := mesh(meshRows-1)(meshCols-1).io.clrReqOut
  io.rowReadResp  := RegNext(mesh(meshRows-1)(meshCols-1).io.rowReadResp)
  io.rowWriteResp := RegNext(mesh(meshRows-1)(meshCols-1).io.rowWriteResp)
  io.colReadResp  := RegNext(mesh(meshRows-1)(meshCols-1).io.colReadResp)
  io.colWriteResp := RegNext(mesh(meshRows-1)(meshCols-1).io.colWriteResp)
  val colRdataMux = WireInit(VecInit(Seq.fill(meshRows)(0.U((tileRows*32).W))))
  for(r <- 0 until meshRows) {
    // use RegEnable to lock valid data
    colRdataMux(r) := RegEnable(mesh(r)(meshCols-1).io.colReadDout, mesh(r)(meshCols-1).io.colReadResp.valid)
  }

  val rowRdataMux = WireInit(VecInit(Seq.fill(meshCols)(0.U((tileCols*64).W))))
  for(c <- 0 until meshCols) {
    rowRdataMux(c) := RegEnable(mesh(meshRows-1)(c).io.rowReadDout, mesh(meshRows-1)(c).io.rowReadResp.valid)
  }
  io.colReadData := colRdataMux.asUInt
  io.rowReadData := rowRdataMux.asUInt
}

// instantiate MESH with Delays
class MXU(
  val meshRows:    Int, 
  val meshCols:    Int, 
  val tileRows:    Int, 
  val tileCols:    Int, 
  val dataWidth:   Int,
  val numAccTiles: Int = 2
)(implicit p: Parameters) extends BoomModule 
{
  val io = IO(new Bundle{
    // matrix multiplication relate
    val macReq          = Flipped(Decoupled(new MacCtrls(numAccTiles)))
    val macReqUop       = Input(new MicroOp())
    val macReqSrcA      = Input(UInt(dataWidth.W))
    val macReqSrcB      = Input(UInt(dataWidth.W))
    val macResp         = Output(Valid(new MacCtrls(numAccTiles)))
    val macRespUop      = Output(new MicroOp())
    // clear tile slices, control signals propagated vertically
    val clrReq          = Flipped(Decoupled(new ClrCtrls(numAccTiles)))
    val clrReqUop       = Input(new MicroOp())
    val clrResp         = Output(Valid(new ClrCtrls(numAccTiles)))
    val clrRespUop      = Output(new MicroOp())
    // read row slices
    val rowReadReq      = Flipped(Decoupled(new SliceCtrls(meshRows*tileRows, numAccTiles)))
    val rowReadReqUop   = Input(new MicroOp())
    val rowReadResp     = Output(Valid(new SliceCtrls(meshRows*tileRows, numAccTiles)))
    val rowReadRespUop  = Output(new MicroOp())
    val rowReadData     = Decoupled(UInt(dataWidth.W))
    // write row slices
    val rowWriteReq     = Flipped(Decoupled(new SliceCtrls(meshRows*tileRows, numAccTiles)))
    val rowWriteReqUop  = Input(new MicroOp())
    val rowWriteData    = Input(UInt(dataWidth.W))
    val rowWriteResp    = Output(Valid(new SliceCtrls(meshRows*tileRows, numAccTiles)))
    val rowWriteRespUop = Output(new MicroOp())
    // read col slices
    val colReadReq      = Flipped(Decoupled(new SliceCtrls(meshCols*tileCols*2, numAccTiles)))
    val colReadReqUop   = Input(new MicroOp())
    val colReadResp     = Output(Valid(new SliceCtrls(meshCols*tileCols*2, numAccTiles)))
    val colReadRespUop  = Output(new MicroOp())
    val colReadData     = Decoupled(UInt(dataWidth.W))
    // write col slices
    val colWriteReq     = Flipped(Decoupled(new SliceCtrls(meshCols*tileCols*2, numAccTiles)))
    val colWriteReqUop  = Input(new MicroOp())
    val colWriteData    = Input(UInt(dataWidth.W))
    val colWriteResp    = Output(Valid(new SliceCtrls(meshCols*tileCols*2, numAccTiles)))
    val colWriteRespUop = Output(new MicroOp())
  })

  require(dataWidth >= meshRows*tileRows*16)
  require(dataWidth >= meshCols*tileCols*16)

  val macReqFire = io.macReq.valid & io.macReq.ready
  val clrReqFire = io.clrReq.valid & io.clrReq.ready

  // row slice writes and col slice writes may occured simultaneously when writing different acc tiles,
  // should be guaranteed by rename/issue logic
  val sliceReady :: sliceWait :: Nil = Enum(2)
  // val sliceReady :: sliceWait :: sliceCross :: Nil = Enum(3)
  val rowReadState     = RegInit(sliceReady)
  val colReadState     = RegInit(sliceReady)
  val rowVsCount       = RegInit(0.U(3.W))
  val colVsCount       = RegInit(0.U(3.W))
  val rowRespCount     = RegInit(0.U(2.W))
  val colRespCount     = RegInit(0.U(2.W))
  val rowReadReqValid  = RegInit(false.B)
  val rowWriteReqValid = RegInit(false.B)
  val colReadReqValid  = RegInit(false.B)
  val colWriteReqValid = RegInit(false.B)
  val rowWriteDataAggr = RegInit(VecInit(Seq.fill(meshCols*tileCols*2)(0.U(32.W))))
  val colWriteDataAggr = RegInit(VecInit(Seq.fill(meshRows*tileRows)(0.U(32.W))))

  // -----------------------------------------------------------------------------------
  // read row slices
  // -----------------------------------------------------------------------------------
  val rowReadCtrls = io.rowReadReq.bits
  rowReadReqValid := false.B
  switch(rowReadState) {
    is(sliceReady) {
      when(io.rowReadReq.valid) {
        when(rowReadCtrls.rtype =/= INT8TYPE && rowReadCtrls.rtype =/= FP16TYPE) {
          rowReadState := sliceWait
          rowVsCount   := Mux(rowReadCtrls.rtype === INT32TYPE, 4.U, 2.U)
          assert(rowReadCtrls.rtype =/= INT64TYPE && rowReadCtrls.rtype =/= FP64TYPE, "INT64 and FP64 not supported\n")
        }
        rowReadReqValid := true.B
      }
    }
    is(sliceWait) {
      when(rowVsCount === 1.U) {
        rowReadState := sliceReady
        rowVsCount   := 0.U
      } .otherwise {
        rowVsCount := rowVsCount - 1.U
      }
    }
  }

  // -----------------------------------------------------------------------------------
  // write row slices
  // -----------------------------------------------------------------------------------
  val rowWriteCtrls = io.rowWriteReq.bits
  rowWriteReqValid := false.B

  when(io.rowWriteReq.valid) {
    when(rowWriteCtrls.rtype === INT8TYPE) {
      (0 until meshCols*tileCols*2).foreach(i => rowWriteDataAggr(i) := Cat(0.U(24.W), io.rowWriteData(8*i+7, 8*i)))
      rowWriteReqValid := true.B
    } .elsewhen(rowWriteCtrls.rtype === INT16TYPE) {
      when(rowVsCount === 0.U) {
        (0 until meshCols*tileCols).foreach(i => rowWriteDataAggr(i) := Cat(0.U(16.W), io.rowWriteData(16*i+15, 16*i)))
        rowVsCount := rowVsCount + 1.U
      } .otherwise {
        (0 until meshCols*tileCols).foreach(i => rowWriteDataAggr(meshCols*tileCols+i) := Cat(0.U(16.W), io.rowWriteData(16*i+15, 16*i)))
        rowVsCount := 0.U
        rowWriteReqValid := true.B
      }
    } .elsewhen(rowWriteCtrls.rtype === INT32TYPE) {
      when(rowVsCount === 0.U) {
        (0 until meshCols*tileCols/2).foreach(i => rowWriteDataAggr(i) := io.rowWriteData(32*i+31, 32*i))
        rowVsCount := rowVsCount + 1.U
      } .elsewhen(rowVsCount === 1.U) {
        (0 until meshCols*tileCols/2).foreach(i => rowWriteDataAggr(meshCols*tileCols/2+i) := io.rowWriteData(32*i+31, 32*i))
        rowVsCount := rowVsCount + 1.U
      } .elsewhen(rowVsCount === 2.U) {
        (0 until meshCols*tileCols/2).foreach(i => rowWriteDataAggr(meshCols*tileCols+i) := io.rowWriteData(32*i+31, 32*i))
        rowVsCount := rowVsCount + 1.U
      } .otherwise {
        (0 until meshCols*tileCols/2).foreach(i => rowWriteDataAggr(3*meshCols*tileCols/2+i) := io.rowWriteData(32*i+31, 32*i))
        rowVsCount := 0.U
        rowWriteReqValid := true.B
      }
    } .elsewhen(rowWriteCtrls.rtype === FP16TYPE) {
      (0 until meshCols*tileCols).foreach(i => rowWriteDataAggr(2*i)   := Cat(0.U(16.W), io.rowWriteData(16*i+15, 16*i)))
      (0 until meshCols*tileCols).foreach(i => rowWriteDataAggr(2*i+1) := 0.U(32.W))
      rowWriteReqValid := true.B
    } .otherwise {
      when(rowVsCount === 0.U) {
        (0 until meshCols*tileCols/2).foreach(i => rowWriteDataAggr(2*i)   := io.rowWriteData(32*i+31, 32*i))
        (0 until meshCols*tileCols/2).foreach(i => rowWriteDataAggr(2*i+1) := 0.U(32.W))
        rowVsCount := rowVsCount + 1.U
      } .otherwise {
        (0 until meshCols*tileCols/2).foreach(i => rowWriteDataAggr(meshCols*tileCols+2*i)   := Cat(0.U(32.W), io.rowWriteData(32*i+31, 32*i)))
        (0 until meshCols*tileCols/2).foreach(i => rowWriteDataAggr(meshCols*tileCols+2*i+1) := 0.U(32.W))
        rowVsCount := 0.U
        rowWriteReqValid := true.B
      }
      assert(rowWriteCtrls.rtype === FP32TYPE, "INT64 and FP64 not supported\n")
    }
  }

  // -----------------------------------------------------------------------------------
  // read col slices
  // -----------------------------------------------------------------------------------
  val colReadCtrls = io.colReadReq.bits
  colReadReqValid := false.B
  switch(colReadState) {
    is(sliceReady) {
      when(io.colReadReq.valid) {
        when(colReadCtrls.rtype === INT32TYPE || colReadCtrls.rtype === FP32TYPE) {
          colReadState := sliceWait
          colVsCount   := 2.U
        }
        colReadReqValid := true.B
      }
    }
    is(sliceWait) {
      when(colVsCount === 1.U) {
        colReadState := sliceReady
        colVsCount   := 0.U
      } .otherwise {
        colVsCount := colVsCount - 1.U
      }
    }
  }

  // -----------------------------------------------------------------------------------
  // write col slices
  // -----------------------------------------------------------------------------------
  val colWriteCtrls = io.colWriteReq.bits
  colWriteReqValid := false.B
  
  when(io.colWriteReq.valid) {
    when(colWriteCtrls.rtype === INT8TYPE) {
      (0 until meshRows*tileRows).foreach(i => colWriteDataAggr(i) := Cat(0.U(24.W), io.colWriteData(8*i+7, 8*i)))
      colWriteReqValid := true.B
    } .elsewhen(colWriteCtrls.rtype === INT16TYPE || colWriteCtrls.rtype === FP16TYPE) {
      (0 until meshRows*tileRows).foreach(i => colWriteDataAggr(i) := Cat(0.U(16.W), io.colWriteData(16*i+15, 16*i)))
      colWriteReqValid := true.B
    } .otherwise {
      when(colVsCount === 0.U) {
        (0 until meshRows*tileRows/2).foreach(i => colWriteDataAggr(i) := io.colWriteData(32*i+31, 32*i))
        colVsCount := colVsCount + 1.U
      } .otherwise {
        (0 until meshRows*tileRows/2).foreach(i => colWriteDataAggr(meshRows*tileRows/2+i) := io.colWriteData(32*i+31, 32*i))
        colVsCount := 0.U
        colWriteReqValid := true.B
      }
      assert(colWriteCtrls.rtype === INT32TYPE || colWriteCtrls.rtype === FP32TYPE, "INT64 and FP64 not supported\n")
    }
  }

  // -----------------------------------------------------------------------------------
  // mopa 
  // -----------------------------------------------------------------------------------
  val mesh = Module(new Mesh(meshRows, meshCols, tileRows, tileCols, numAccTiles))
  mesh.io.clrReq.valid      := clrReqFire
  mesh.io.clrReq.bits       := io.clrReq.bits
  mesh.io.rowReadReq.valid  := rowReadReqValid
  mesh.io.rowReadReq.bits   := RegNext(io.rowReadReq.bits)
  mesh.io.rowWriteReq.valid := rowWriteReqValid
  mesh.io.rowWriteReq.bits  := RegNext(io.rowWriteReq.bits)
  mesh.io.rowWriteData      := rowWriteDataAggr.asUInt
  mesh.io.colReadReq.valid  := colReadReqValid
  mesh.io.colReadReq.bits   := RegNext(io.colReadReq.bits)
  mesh.io.colWriteReq.valid := colWriteReqValid
  mesh.io.colWriteReq.bits  := RegNext(io.colWriteReq.bits)
  mesh.io.colWriteData      := colWriteDataAggr.asUInt
  for(c <- 0 until meshCols) {
    mesh.io.macReq(c).valid := ShiftRegister(macReqFire, c, true.B)
    mesh.io.macReq(c).bits  := ShiftRegister(io.macReq.bits, c, true.B)
    mesh.io.macReqSrcB(c)   := ShiftRegister(io.macReqSrcB(16*tileCols*(c+1)-1, 16*tileCols*c), c, true.B)
  }

  val widenSrcA = WireInit(VecInit(Seq.fill(meshRows*tileRows)(0.U(16.W))))
  (0 until meshRows*tileRows).foreach(i => widenSrcA(i) := io.macReqSrcA(8*i+7, 8*i))
  val muxSrcA = Mux(io.macReq.bits.srcType(2), io.macReqSrcA, widenSrcA.asUInt)
  for(r <- 0 until meshRows) {
    mesh.io.macReqSrcA(r) := ShiftRegister(muxSrcA(16*tileRows*(r+1)-1, 16*tileRows*r), r, true.B)
  }

  // ready control
  io.macReq.ready      := true.B
  io.clrReq.ready      := true.B
  io.rowWriteReq.ready := true.B
  io.colWriteReq.ready := true.B
  io.rowReadReq.ready  := (rowReadState === sliceReady)
  io.colReadReq.ready  := (colReadState === sliceReady)
  // output control
  io.macResp           := mesh.io.macResp
  io.macRespUop        := Pipe(macReqFire, io.macReqUop, meshRows+meshCols-1).bits
  io.clrResp           := mesh.io.clrResp
  io.clrRespUop        := Pipe(clrReqFire, io.clrReqUop, meshCols).bits
  io.rowReadResp       := mesh.io.rowReadResp
  io.rowReadRespUop    := Pipe(io.rowReadReq.valid, io.rowReadReqUop, meshCols).bits
  io.rowWriteResp      := mesh.io.rowWriteResp
  io.rowWriteRespUop   := Pipe(io.rowWriteReq.valid, io.rowWriteReqUop, meshCols).bits
  io.colReadResp       := mesh.io.colReadResp
  io.colReadRespUop    := Pipe(io.colReadReq.valid, io.colReadReqUop, meshRows).bits
  io.colWriteResp      := mesh.io.colWriteResp
  io.colWriteRespUop   := Pipe(io.colWriteReq.valid, io.colWriteReqUop, meshRows).bits

  val rowRespCtrls   = mesh.io.rowReadResp.bits
  val rowReadDataMux = WireInit(VecInit(Seq.fill(meshCols*tileCols*2)(0.U(8.W))))
  when(mesh.io.rowReadResp.valid && io.rowReadData.ready) {
    when(rowRespCtrls.rtype === INT8TYPE) {
      (0 until meshCols*tileCols*2).foreach(i => rowReadDataMux(i)     := mesh.io.rowReadData(32*i+7, 32*i))
    } .elsewhen(rowRespCtrls.rtype === INT16TYPE) {
      (0 until meshCols*tileCols).foreach(i => rowReadDataMux(2*i)     := mesh.io.rowReadData(32*i+7,  32*i))
      (0 until meshCols*tileCols).foreach(i => rowReadDataMux(2*i+1)   := mesh.io.rowReadData(32*i+15, 32*i+8))
      rowRespCount := 1.U
    } .elsewhen(rowRespCtrls.rtype === INT32TYPE) {
      (0 until meshCols*tileCols/2).foreach(i => rowReadDataMux(4*i)   := mesh.io.rowReadData(32*i+7,  32*i))
      (0 until meshCols*tileCols/2).foreach(i => rowReadDataMux(4*i+1) := mesh.io.rowReadData(32*i+15, 32*i+8))
      (0 until meshCols*tileCols/2).foreach(i => rowReadDataMux(4*i+2) := mesh.io.rowReadData(32*i+23, 32*i+16))
      (0 until meshCols*tileCols/2).foreach(i => rowReadDataMux(4*i+3) := mesh.io.rowReadData(32*i+31, 32*i+24))
      rowRespCount := 3.U
    } .elsewhen(rowRespCtrls.rtype === FP16TYPE) {
      (0 until meshCols*tileCols).foreach(i => rowReadDataMux(2*i)     := mesh.io.rowReadData(64*i+7,  64*i))
      (0 until meshCols*tileCols).foreach(i => rowReadDataMux(2*i+1)   := mesh.io.rowReadData(64*i+15, 64*i+8))
    } .otherwise {
      (0 until meshCols*tileCols/2).foreach(i => rowReadDataMux(4*i)   := mesh.io.rowReadData(64*i+7,  64*i))
      (0 until meshCols*tileCols/2).foreach(i => rowReadDataMux(4*i+1) := mesh.io.rowReadData(64*i+15, 64*i+8))
      (0 until meshCols*tileCols/2).foreach(i => rowReadDataMux(4*i+2) := mesh.io.rowReadData(64*i+23, 64*i+16))
      (0 until meshCols*tileCols/2).foreach(i => rowReadDataMux(4*i+3) := mesh.io.rowReadData(64*i+31, 64*i+24))
      rowRespCount := 1.U
    }
  } .elsewhen(rowRespCount > 0.U && io.rowReadData.ready) {
    when(rowRespCount === 3.U) {
      (0 until meshCols*tileCols/2).foreach(i => rowReadDataMux(4*i)   := mesh.io.rowReadData(16*meshCols*tileCols+32*i+7,  16*meshCols*tileCols+32*i))
      (0 until meshCols*tileCols/2).foreach(i => rowReadDataMux(4*i+1) := mesh.io.rowReadData(16*meshCols*tileCols+32*i+15, 16*meshCols*tileCols+32*i+8))
      (0 until meshCols*tileCols/2).foreach(i => rowReadDataMux(4*i+2) := mesh.io.rowReadData(16*meshCols*tileCols+32*i+23, 16*meshCols*tileCols+32*i+16))
      (0 until meshCols*tileCols/2).foreach(i => rowReadDataMux(4*i+3) := mesh.io.rowReadData(16*meshCols*tileCols+32*i+31, 16*meshCols*tileCols+32*i+24))
    } .elsewhen(rowRespCount === 2.U) {
      (0 until meshCols*tileCols/2).foreach(i => rowReadDataMux(4*i)   := mesh.io.rowReadData(32*meshCols*tileCols+32*i+7,  32*meshCols*tileCols+32*i))
      (0 until meshCols*tileCols/2).foreach(i => rowReadDataMux(4*i+1) := mesh.io.rowReadData(32*meshCols*tileCols+32*i+15, 32*meshCols*tileCols+32*i+8))
      (0 until meshCols*tileCols/2).foreach(i => rowReadDataMux(4*i+2) := mesh.io.rowReadData(32*meshCols*tileCols+32*i+23, 32*meshCols*tileCols+32*i+16))
      (0 until meshCols*tileCols/2).foreach(i => rowReadDataMux(4*i+3) := mesh.io.rowReadData(32*meshCols*tileCols+32*i+31, 32*meshCols*tileCols+32*i+24))
    } .elsewhen(rowRespCount === 1.U && rowRespCtrls.rtype === INT32TYPE) {
      (0 until meshCols*tileCols/2).foreach(i => rowReadDataMux(4*i)   := mesh.io.rowReadData(48*meshCols*tileCols+32*i+7,  48*meshCols*tileCols+32*i))
      (0 until meshCols*tileCols/2).foreach(i => rowReadDataMux(4*i+1) := mesh.io.rowReadData(48*meshCols*tileCols+32*i+15, 48*meshCols*tileCols+32*i+8))
      (0 until meshCols*tileCols/2).foreach(i => rowReadDataMux(4*i+2) := mesh.io.rowReadData(48*meshCols*tileCols+32*i+23, 48*meshCols*tileCols+32*i+16))
      (0 until meshCols*tileCols/2).foreach(i => rowReadDataMux(4*i+3) := mesh.io.rowReadData(48*meshCols*tileCols+32*i+31, 48*meshCols*tileCols+32*i+24))
    } .elsewhen(rowRespCount === 1.U && rowRespCtrls.rtype === INT16TYPE) {
      (0 until meshCols*tileCols).foreach(i => rowReadDataMux(2*i)     := mesh.io.rowReadData(32*meshCols*tileCols+32*i+7,  32*meshCols*tileCols+32*i))
      (0 until meshCols*tileCols).foreach(i => rowReadDataMux(2*i+1)   := mesh.io.rowReadData(32*meshCols*tileCols+32*i+15, 32*meshCols*tileCols+32*i+8))
    } .otherwise {
      (0 until meshCols*tileCols/2).foreach(i => rowReadDataMux(4*i)   := mesh.io.rowReadData(32*meshCols*tileCols+64*i+7,  32*meshCols*tileCols+64*i))
      (0 until meshCols*tileCols/2).foreach(i => rowReadDataMux(4*i+1) := mesh.io.rowReadData(32*meshCols*tileCols+64*i+15, 32*meshCols*tileCols+64*i+8))
      (0 until meshCols*tileCols/2).foreach(i => rowReadDataMux(4*i+2) := mesh.io.rowReadData(32*meshCols*tileCols+64*i+23, 32*meshCols*tileCols+64*i+16))
      (0 until meshCols*tileCols/2).foreach(i => rowReadDataMux(4*i+3) := mesh.io.rowReadData(32*meshCols*tileCols+64*i+31, 32*meshCols*tileCols+64*i+24))
    }
    rowRespCount := rowRespCount - 1.U
  }

  val colRespCtrls   = mesh.io.colReadResp.bits
  val colReadDataMux = WireInit(VecInit(Seq.fill(meshRows*tileRows*2)(0.U(8.W))))
  when(mesh.io.colReadResp.valid && io.colReadData.ready) {
    when(colRespCtrls.rtype === INT8TYPE) {
      (0 until meshRows*tileRows).foreach(i => colReadDataMux(i)       := mesh.io.colReadData(32*i+7, 32*i))
    } .elsewhen(colRespCtrls.rtype === INT16TYPE || colRespCtrls.rtype === FP16TYPE) {
      (0 until meshRows*tileRows).foreach(i => colReadDataMux(2*i)     := mesh.io.colReadData(32*i+7,  32*i))
      (0 until meshRows*tileRows).foreach(i => colReadDataMux(2*i+1)   := mesh.io.colReadData(32*i+15, 32*i+8))
    } .otherwise{
      (0 until meshRows*tileRows/2).foreach(i => colReadDataMux(4*i)   := mesh.io.colReadData(32*i+7,  32*i))
      (0 until meshRows*tileRows/2).foreach(i => colReadDataMux(4*i+1) := mesh.io.colReadData(32*i+15, 32*i+8))
      (0 until meshRows*tileRows/2).foreach(i => colReadDataMux(4*i+2) := mesh.io.colReadData(32*i+23, 32*i+16))
      (0 until meshRows*tileRows/2).foreach(i => colReadDataMux(4*i+3) := mesh.io.colReadData(32*i+31, 32*i+24))
      colRespCount := 1.U
    }
  } .elsewhen(colRespCount > 0.U && io.colReadData.ready) {
    (0 until meshRows*tileRows/2).foreach(i => colReadDataMux(4*i)     := mesh.io.colReadData(16*meshRows*tileRows+32*i+7,  16*meshRows*tileRows+32*i))
    (0 until meshRows*tileRows/2).foreach(i => colReadDataMux(4*i+1)   := mesh.io.colReadData(16*meshRows*tileRows+32*i+15, 16*meshRows*tileRows+32*i+8))
    (0 until meshRows*tileRows/2).foreach(i => colReadDataMux(4*i+2)   := mesh.io.colReadData(16*meshRows*tileRows+32*i+23, 16*meshRows*tileRows+32*i+16))
    (0 until meshRows*tileRows/2).foreach(i => colReadDataMux(4*i+3)   := mesh.io.colReadData(16*meshRows*tileRows+32*i+31, 16*meshRows*tileRows+32*i+24))
    colRespCount := 0.U
  }
  
  io.rowReadData.valid := mesh.io.rowReadResp.valid || rowRespCount > 0.U
  io.rowReadData.bits  := rowReadDataMux.asUInt
  
  io.colReadData.valid := mesh.io.colReadResp.valid || colRespCount > 0.U
  io.colReadData.bits  := colReadDataMux.asUInt
}