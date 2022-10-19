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
  // slice groups
  val W0          = 0.U(2.W)
  val W1          = 1.U(2.W)
  val Q0          = 0.U(2.W)
  val Q1          = 1.U(2.W)
  val Q2          = 2.U(2.W)
  val Q3          = 3.U(2.W)
  // alu types
  val MACC        = 0.U(3.W)
  val MULT        = 1.U(3.W)
  val ADD         = 2.U(3.W)
  val SUB         = 3.U(3.W)
  val CVT         = 4.U(3.W)
}

import AccTileConstants._

class MacCtrls(implicit p: Parameters) extends BoomBundle 
{
  val src1Ridx = UInt(log2Ceil(numAccTiles).W)
  val src2Ridx = UInt(log2Ceil(numAccTiles).W)
  val dstRidx  = UInt(log2Ceil(numAccTiles).W)
  val srcType  = UInt(3.W)
  val outType  = UInt(3.W)
  val aluType  = UInt(3.W)
  val rm       = UInt(3.W)                      // rounding mode
}

class ClrCtrls(implicit p: Parameters) extends BoomBundle {
  val ridx = UInt(log2Ceil(numAccTiles).W)    // register index
}

class TileReadReq(implicit p: Parameters) extends BoomBundle {
  val ridx     = UInt(tpregSz.W)              // register index
  val sidx     = UInt(rLenbSz.W)              // slice index
  val sew      = UInt(2.W)                    // SEW = 8bits, 16bits, 32bits
  val tt       = UInt(2.W)                    // 0, acc_row; 1, acc_col; 2, tile_row; 3, tile_col
  val quad     = UInt(2.W)                    // acc may support quad-width
  val vstq_idx = UInt(vstqAddrSz.W)           // index in vstq
}

class SliceCtrls(implicit p: Parameters) extends BoomBundle {
  val ridx = UInt(log2Ceil(numAccTiles).W)    // register index
  val sidx = UInt(rLenbSz.W)                  // slice index
  val sew  = UInt(2.W)                        // SEW = 8bits, 16bits, 32bits
}

class AccReadReq(implicit p: Parameters) extends BoomBundle {
  val sCtrls   = new SliceCtrls()             // slice control signals
  val tt       = UInt(1.W)                    // 0, row slice; 1, col slice
  val quad     = UInt(2.W)                    // acc may support quad-width
  val vstq_idx = UInt(vstqAddrSz.W)           // index in vstq
}

class AccReadResp(implicit p: Parameters) extends BoomBundle {
  val vstq_idx = UInt(vstqAddrSz.W)               // index in vstq
  val data     = UInt(rLen.W)
}

// acc 32-bits
class IntMacUnit extends Module 
{
  val io = IO(new Bundle {
    val src1    = Input(UInt(32.W))
    val src2    = Input(UInt(8.W))
    val src3    = Input(UInt(32.W))
    val srcType = Input(UInt(3.W))
    val outType = Input(UInt(3.W))
    val aluType = Input(UInt(3.W))
    val out     = Output(UInt(32.W))
  })

  val sMax = WireInit(0.U(32.W))
  val sMin = WireInit(0.U(32.W))
  sMax := Mux(io.outType === INT8TYPE,  0x7F.U,
          Mux(io.outType === INT16TYPE, 0x7FFF.U, 0x7FFFFFFF.U))
  sMin := Mux(io.outType === INT8TYPE,  Fill(24, 1.U(1.W)) ## 0x80.U(8.W),
          Mux(io.outType === INT16TYPE, Fill(16, 1.U(1.W)) ## 0x8000.U(16.W), 
                                        1.U(1.W) ## Fill(31, 0.U(1.W))))

  // macc, mult
  val lhs  = io.src1(7, 0).asSInt
  val rhs  = io.src2(7, 0).asSInt
  val acc  = WireInit(0.U(32.W))
  acc := Mux(io.outType === INT8TYPE,  Fill(24, io.src3(7))  ## io.src3( 7, 0),
         Mux(io.outType === INT16TYPE, Fill(16, io.src3(15)) ## io.src3(15, 0), io.src3))
  val macc = lhs * rhs +& acc.asSInt

  // add, sub
  val in1 = Mux(io.srcType === INT8TYPE,  Fill(24, io.src1(7))  ## io.src1( 7, 0),
            Mux(io.srcType === INT16TYPE, Fill(16, io.src1(15)) ## io.src1(15, 0), io.src1))
  val in2 = Mux(io.srcType === INT8TYPE,  Fill(24, io.src3(7))  ## io.src3( 7, 0),
            Mux(io.srcType === INT16TYPE, Fill(16, io.src3(15)) ## io.src3(15, 0), io.src3))
  val in2_inv = Mux(io.aluType === SUB, ~in2, in2)
  val adder   = in1.asSInt +& in2_inv.asSInt + Mux(io.aluType === SUB, 1.S, 0.S)

  val result  = Mux(io.aluType(1), adder, macc)

  io.out := Mux(result > sMax.asSInt, sMax,
            Mux(result < sMin.asSInt, sMin, result(31, 0)))
}

// may use blackbox hardfloat modules
class FpMacUnit extends Module 
{
  val io = IO(new Bundle {
    val src1           = Input(UInt(32.W))
    val src2           = Input(UInt(16.W))
    val src3           = Input(UInt(32.W))
    val srcType        = Input(UInt(3.W))
    val outType        = Input(UInt(3.W))
    val aluType        = Input(UInt(3.W))
    val roundingMode   = Input(UInt(3.W))
    val detectTininess = Input(UInt(1.W))
    val out            = Output(UInt(32.W))
  })
  val H = new FType(5, 11)
  val S = new FType(8, 24)
  // convert fp16 A, B, and C to recoded format
  val recAFP16 = recFNFromFN(H.exp, H.sig, io.src1(15, 0))
  val recBFP16 = recFNFromFN(H.exp, H.sig, io.src2(15, 0))
  val recCFP16 = recFNFromFN(H.exp, H.sig, io.src3(15, 0))
  // convert fp32 A, B and C to recoded format
  val recAFP32 = recFNFromFN(S.exp, S.sig, io.src1)
  val recCFP32 = recFNFromFN(S.exp, S.sig, io.src3)

  // convert fp16 to fp32 for float add/sub/rsub
  val recA16ToRec32 = Module(new RecFNToRecFN(H.exp, H.sig, S.exp, S.sig))
  recA16ToRec32.io.in := recAFP16
  recA16ToRec32.io.roundingMode   := io.roundingMode
  recA16ToRec32.io.detectTininess := io.detectTininess

  val recC16ToRec32 = Module(new RecFNToRecFN(H.exp, H.sig, S.exp, S.sig))
  recC16ToRec32.io.in := recCFP16
  recC16ToRec32.io.roundingMode   := io.roundingMode
  recC16ToRec32.io.detectTininess := io.detectTininess

  val recA = Mux(io.srcType === FP32TYPE, recAFP32, recA16ToRec32.io.out)
  val recC = Mux(io.srcType === FP32TYPE, recCFP32, recC16ToRec32.io.out)

  // FNCVT: convert fp32 to fp16 recoded format
  val recA32ToRec16 = Module(new RecFNToRecFN(S.exp, S.sig, H.exp, H.sig))
  recA32ToRec16.io.in := recAFP32
  recA32ToRec16.io.roundingMode   := io.roundingMode
  recA32ToRec16.io.detectTininess := io.detectTininess

  // recoded fp16 * fp16
  val recMul = Module(new MulFullRawFN(H.exp, H.sig))
  recMul.io.a := rawFloatFromRecFN(H.exp, H.sig, recAFP16)
  recMul.io.b := rawFloatFromRecFN(H.exp, H.sig, recBFP16)

  // round raw results to recFN(32)
  val anyRawToRec = Module(new RoundAnyRawFNToRecFN(H.exp, H.sig*2-1, S.exp, S.sig, 0))
  anyRawToRec.io.invalidExc     := recMul.io.invalidExc
  anyRawToRec.io.infiniteExc    := false.B
  anyRawToRec.io.in             := recMul.io.rawOut
  anyRawToRec.io.roundingMode   := io.roundingMode
  anyRawToRec.io.detectTininess := io.detectTininess

  // recoded fp32 + fp32
  val recAdd = Module(new AddRecFN(S.exp, S.sig))
  recAdd.io.subOp   := io.aluType === SUB
  recAdd.io.a       := Mux(io.aluType(1), recA, anyRawToRec.io.out)
  recAdd.io.b       := recC
  recAdd.io.roundingMode   := io.roundingMode
  recAdd.io.detectTininess := io.detectTininess
  val fp32Out = fNFromRecFN(S.exp, S.sig, recAdd.io.out)

  // FP32 to FP16
  val recAdd32ToRec16 = Module(new RecFNToRecFN(S.exp, S.sig, H.exp, H.sig))
  recAdd32ToRec16.io.in := recAdd.io.out
  recAdd32ToRec16.io.roundingMode   := io.roundingMode
  recAdd32ToRec16.io.detectTininess := io.detectTininess
  val fp16Out = Mux(io.aluType === CVT, fNFromRecFN(H.exp, H.sig, recA32ToRec16.io.out),
                                        fNFromRecFN(H.exp, H.sig, recAdd32ToRec16.io.out))

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
  val rowIndex:     Int, 
  val colIndex:     Int,
  val numReadPorts: Int = 1
)(implicit p: Parameters) extends BoomModule 
{
  val io = IO(new Bundle{
    // matrix multiplication related: mutiply-accumulate
    val macReqIn     = Input(Valid(new MacCtrls()))            // control signals
    val macReqSrcA   = Input(UInt(16.W))
    val macReqSrcB   = Input(UInt(16.W))
    val macReqOut    = Output(Valid(new MacCtrls()))
    val macOutSrcA   = Output(UInt(16.W))                                 // for systolic horizontally
    val macOutSrcB   = Output(UInt(16.W))                                 // for systolic vertically
    // clear tile slices, control signals propagated vertically
    val clrReqIn     = Input(Valid(new ClrCtrls()))
    val clrReqOut    = Output(Valid(new ClrCtrls()))
    // read row slices, control signals propagated vertically
    val rowReadReq   = Input(Vec(numReadPorts,  Valid(new SliceCtrls())))
    val rowReadDin   = Input(Vec(numReadPorts,  UInt(64.W)))
    val rowReadResp  = Output(Vec(numReadPorts, Valid(new SliceCtrls())))
    val rowReadDout  = Output(Vec(numReadPorts, UInt(64.W)))
    // write row slices, control signals propagated vertically
    val rowWriteReq  = Input(Valid(new SliceCtrls()))
    val rowWriteDin  = Input(UInt(64.W))
    val rowWriteMask = Input(UInt(2.W))
    val rowWriteResp = Output(Valid(new SliceCtrls()))
    val rowWriteDout = Output(UInt(64.W))
    val rowWriteMout = Output(UInt(2.W))
    // read col slices, control signals propagated horizontally
    val colReadReq   = Input(Vec(numReadPorts,  Valid(new SliceCtrls())))
    val colReadDin   = Input(Vec(numReadPorts,  UInt(32.W)))
    val colReadResp  = Output(Vec(numReadPorts, Valid(new SliceCtrls())))
    val colReadDout  = Output(Vec(numReadPorts, UInt(32.W)))
    // write col slices, control signals propagated horizontally
    val colWriteReq  = Input(Valid(new SliceCtrls()))
    val colWriteDin  = Input(UInt(32.W))
    val colWriteMask = Input(UInt(1.W))
    val colWriteResp = Output(Valid(new SliceCtrls()))
    val colWriteDout = Output(UInt(32.W))
    val colWriteMout = Output(UInt(1.W))
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
  fpMac.io.src1           := Mux(macReqCtrls.aluType === MACC, io.macReqSrcA, c0(macReqCtrls.src1Ridx))
  fpMac.io.src2           := io.macReqSrcB
  fpMac.io.src3           := c0(macReqCtrls.src2Ridx)
  fpMac.io.srcType        := macReqCtrls.outType      // Attention here, fpMac support fp16 multiply only
  fpMac.io.outType        := macReqCtrls.outType
  fpMac.io.aluType        := macReqCtrls.aluType
  fpMac.io.roundingMode   := macReqCtrls.rm
  fpMac.io.detectTininess := hardfloat.consts.tininess_afterRounding

  // int8 MAC function units
  val intMac0 = Module(new IntMacUnit())
  val intMac1 = Module(new IntMacUnit())
  intMac0.io.src1    := Mux(macReqCtrls.aluType === MACC, io.macReqSrcA( 7, 0), c0(macReqCtrls.src1Ridx))
  intMac0.io.src2    := io.macReqSrcB( 7, 0)
  intMac0.io.src3    := c0(macReqCtrls.src2Ridx)
  intMac0.io.srcType := macReqCtrls.outType
  intMac0.io.outType := macReqCtrls.outType
  intMac0.io.aluType := macReqCtrls.aluType
  intMac1.io.src1    := Mux(macReqCtrls.aluType === MACC, io.macReqSrcA( 7, 0), c1(macReqCtrls.src1Ridx))
  intMac1.io.src2    := io.macReqSrcB(15, 8)
  intMac1.io.src3    := c1(macReqCtrls.src2Ridx)
  intMac1.io.srcType := macReqCtrls.outType
  intMac1.io.outType := macReqCtrls.outType
  intMac1.io.aluType := macReqCtrls.aluType

  // TODO: Optimization, latency = 1 for int8 mac; 3 for fp16 mac
  when(macReqValid && macReqCtrls.srcType(2)) {
    c0(macReqCtrls.dstRidx) := fpMac.io.out
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
  for (i <- 0 until numReadPorts) {
    val rowReadValid = io.rowReadReq(i).valid
    val rowReadCtrls = io.rowReadReq(i).bits
    val rowRdata     = WireInit(0.U(64.W))
    val rowReadHit   = rowReadValid && rowReadCtrls.sidx === rowIndex.U
    rowRdata := Mux(!rowReadHit, io.rowReadDin(i), Cat(c1(rowReadCtrls.ridx), c0(rowReadCtrls.ridx)))

    io.rowReadResp(i) := io.rowReadReq(i)
    io.rowReadDout(i) := rowRdata
  }

  // -----------------------------------------------------------------------------------
  // write row slices
  // -----------------------------------------------------------------------------------
  val rowWriteValid = io.rowWriteReq.valid
  val rowWrtieCtrls = io.rowWriteReq.bits
  val rowWriteHit   = rowWriteValid && rowWrtieCtrls.sidx === rowIndex.U
  val rowWriteHitC0 = rowWriteHit && io.rowWriteMask(0).asBool
  val rowWriteHitC1 = rowWriteHit && io.rowWriteMask(1).asBool

  when(rowWriteHitC0) {
    c0(rowWrtieCtrls.ridx) := io.rowWriteDin(31, 0)
  }
  when(rowWriteHitC1) {
    c1(rowWrtieCtrls.ridx) := io.rowWriteDin(63, 32)
  }

  io.rowWriteResp := io.rowWriteReq
  io.rowWriteDout := io.rowWriteDin
  io.rowWriteMout := io.rowWriteMask

  // -----------------------------------------------------------------------------------
  // read col slices
  // -----------------------------------------------------------------------------------
  // c0 index = colIndex; c1 index = colIndex+numMeshCols/2
  for (i <- 0 until numReadPorts) {
    val colReadValid = io.colReadReq(i).valid
    val colReadCtrls = io.colReadReq(i).bits
    val colRdata     = WireInit(0.U(32.W))
    val colReadHitC0 = colReadValid && colReadCtrls.sidx === colIndex.U
    val colReadHitC1 = colReadValid && colReadCtrls.sidx === (colIndex + mxuPECols/2).U
    colRdata := Mux(colReadHitC0, c0(colReadCtrls.ridx), 
                Mux(colReadHitC1, c1(colReadCtrls.ridx), io.colReadDin(i)))

    io.colReadResp(i) := io.colReadReq(i)
    io.colReadDout(i) := colRdata
  }

  // -----------------------------------------------------------------------------------
  // write col slices
  // -----------------------------------------------------------------------------------
  val colWriteValid = io.colWriteReq.valid
  val colWriteCtrls = io.colWriteReq.bits
  val colWriteHitC0 = colWriteValid && colWriteCtrls.sidx === colIndex.U
  val colWriteHitC1 = colWriteValid && colWriteCtrls.sidx === (colIndex + mxuPECols/2).U

  when(colWriteHitC0 && io.colWriteMask.asBool) {
    c0(colWriteCtrls.ridx) := io.colWriteDin
  } .elsewhen(colWriteHitC1 && io.colWriteMask.asBool) {
    c1(colWriteCtrls.ridx) := io.colWriteDin
  }

  io.colWriteResp := io.colWriteReq
  io.colWriteDout := io.colWriteDin
  io.colWriteMout := io.colWriteMask
}

/**
  * A Tile is a purely combinational 2D array of passThrough PEs.
  */
class Tile(
  val indexh:       Int, 
  val indexv:       Int, 
  val numReadPorts: Int = 1
)(implicit p: Parameters) extends BoomModule 
{
  val io = IO(new Bundle{
    // matrix multiplication related: mutiply-accumulate
    val macReqIn     = Input(Valid(new MacCtrls()))
    val macReqSrcA   = Input(UInt((mxuTileRows*16).W))
    val macReqSrcB   = Input(UInt((mxuTileCols*16).W))
    val macReqOut    = Output(Valid(new MacCtrls()))
    val macOutSrcA   = Output(UInt((mxuTileRows*16).W))
    val macOutSrcB   = Output(UInt((mxuTileCols*16).W))
    // clear tile slices, control signals propagated vertically
    val clrReqIn     = Input(Valid(new ClrCtrls()))
    val clrReqOut    = Output(Valid(new ClrCtrls()))
    // read row slices, control signals propagated vertically
    val rowReadReq   = Input(Vec(numReadPorts,  Valid(new SliceCtrls())))
    val rowReadDin   = Input(Vec(numReadPorts,  UInt((mxuTileCols*64).W)))
    val rowReadResp  = Output(Vec(numReadPorts, Valid(new SliceCtrls())))
    val rowReadDout  = Output(Vec(numReadPorts, UInt((mxuTileCols*64).W)))
    // write row slices, control signals propagated vertically
    val rowWriteReq  = Input(Valid(new SliceCtrls()))
    val rowWriteDin  = Input(UInt((mxuTileCols*64).W))
    val rowWriteMask = Input(UInt((mxuTileCols*2).W))
    val rowWriteResp = Output(Valid(new SliceCtrls()))
    val rowWriteDout = Output(UInt((mxuTileCols*64).W))
    val rowWriteMout = Output(UInt((mxuTileCols*2).W))
    // read col slice, control signals propagated horizontally
    val colReadReq   = Input(Vec(numReadPorts,  Valid(new SliceCtrls())))
    val colReadDin   = Input(Vec(numReadPorts,  UInt((mxuTileRows*32).W)))
    val colReadResp  = Output(Vec(numReadPorts, Valid(new SliceCtrls())))
    val colReadDout  = Output(Vec(numReadPorts, UInt((mxuTileRows*32).W)))
    // write col slice, control signals propagated horizontally
    val colWriteReq  = Input(Valid(new SliceCtrls()))
    val colWriteDin  = Input(UInt((mxuTileRows*32).W))
    val colWriteMask = Input(UInt(mxuTileRows.W))
    val colWriteResp = Output(Valid(new SliceCtrls()))
    val colWriteDout = Output(UInt((mxuTileRows*32).W))
    val colWriteMout = Output(UInt(mxuTileRows.W))
  })

  val tile = Seq.tabulate(mxuTileRows, mxuTileCols)((i, j) => Module(new PE(indexh+i, indexv+j, numReadPorts)))
  val tileT = tile.transpose

  // broadcast horizontally across the tile
  for(r <- 0 until mxuTileRows) {
    val peSrcA = io.macReqSrcA(16*r+15, 16*r)
    tile(r).foldLeft(peSrcA) {
      case (macReqSrcA, pe) => {
        pe.io.macReqSrcA := macReqSrcA
        pe.io.macOutSrcA
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
    val peColWmask = io.colWriteMask(r).asUInt
    tile(r).foldLeft(peColWmask) {
      case (colWriteMask, pe) => {
        pe.io.colWriteMask := colWriteMask
        pe.io.colWriteMout
      }
    }
    for(i <- 0 until numReadPorts) {
      tile(r).foldLeft(io.colReadReq(i)) {
        case (colReadReq, pe) => {
          pe.io.colReadReq(i) := colReadReq
          pe.io.colReadResp(i)
        }
      }

      val peColRdata = io.colReadDin(i)(32*r+31, 32*r)
      tile(r).foldLeft(peColRdata) {
        case (colReadData, pe) => {
          pe.io.colReadDin(i) := colReadData
          pe.io.colReadDout(i)
        }
      }
    }
  }

  // broadcast vertically
  for(c <- 0 until mxuTileCols) {
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
    val peRowWmask = io.rowWriteMask(2*c+1, 2*c)
    tileT(c).foldLeft(peRowWmask) {
      case(rowWriteMask, pe) => {
        pe.io.rowWriteMask := rowWriteMask
        pe.io.rowWriteMout
      }
    }
    for(i <- 0 until numReadPorts) {
      tileT(c).foldLeft(io.rowReadReq(i)) {
        case(rowReadReq, pe) => {
          pe.io.rowReadReq(i) := rowReadReq
          pe.io.rowReadResp(i)
        }
      }
      val peRowRdata = io.rowReadDin(i)(64*c+63, 64*c)
      tileT(c).foldLeft(peRowRdata) {
        case(rowReadData, pe) => {
          pe.io.rowReadDin(i) := rowReadData
          pe.io.rowReadDout(i)
        }
      }
    }
  }

  // tile's bottom IO
  io.macReqOut     := io.macReqIn
  io.clrReqOut     := io.clrReqIn
  io.rowWriteResp  := io.rowWriteReq
  io.colWriteResp  := io.colWriteReq

  val macOutSrcAMux   = WireInit(VecInit(Seq.fill(mxuTileRows)(0.U(16.W))))
  val colWriteDataMux = WireInit(VecInit(Seq.fill(mxuTileRows)(0.U(32.W))))
  val colWriteMaskMux = WireInit(VecInit(Seq.fill(mxuTileRows)(0.U(1.W))))
  for(r <- 0 until mxuTileRows) {
    macOutSrcAMux(r)   := tile(r)(mxuTileCols-1).io.macOutSrcA
    colWriteDataMux(r) := tile(r)(mxuTileCols-1).io.colWriteDout
    colWriteMaskMux(r) := tile(r)(mxuTileCols-1).io.colWriteMout
  }

  val macOutSrcBMux   = WireInit(VecInit(Seq.fill(mxuTileCols)(0.U(16.W))))
  val rowWriteDataMux = WireInit(VecInit(Seq.fill(mxuTileCols)(0.U(64.W))))
  val rowWriteMaskMux = WireInit(VecInit(Seq.fill(mxuTileCols)(0.U(2.W))))
  for(c <- 0 until mxuTileCols) {
    macOutSrcBMux(c)   := tile(mxuTileRows-1)(c).io.macOutSrcB
    rowWriteDataMux(c) := tile(mxuTileRows-1)(c).io.rowWriteDout
    rowWriteMaskMux(c) := tile(mxuTileRows-1)(c).io.rowWriteMout
  }
  io.macOutSrcA   := macOutSrcAMux.asUInt
  io.macOutSrcB   := macOutSrcBMux.asUInt
  io.colWriteDout := colWriteDataMux.asUInt
  io.colWriteMout := colWriteMaskMux.asUInt
  io.rowWriteDout := rowWriteDataMux.asUInt
  io.rowWriteMout := rowWriteMaskMux.asUInt

  for(i <- 0 until numReadPorts) {
    val colReadDataMux  = WireInit(VecInit(Seq.fill(mxuTileRows)(0.U(32.W))))
    for(r <- 0 until mxuTileRows) {
      colReadDataMux(r) := tile(r)(mxuTileCols-1).io.colReadDout(i)
    }

    val rowReadDataMux  = WireInit(VecInit(Seq.fill(mxuTileCols)(0.U(64.W))))
    for(c <- 0 until mxuTileCols) {
      rowReadDataMux(c)  := tile(mxuTileRows-1)(c).io.rowReadDout(i)
    }

    io.rowReadResp(i) := io.rowReadReq(i)
    io.colReadResp(i) := io.colReadReq(i)
    io.colReadDout(i) := colReadDataMux.asUInt
    io.rowReadDout(i) := rowReadDataMux.asUInt
  }
}

class Mesh(
  val numReadPorts: Int = 1
)(implicit p: Parameters) extends BoomModule 
{
  val io = IO(new Bundle {
    // matrix multiplication related: mutiply-accumulate
    val macReq       = Input(Vec(mxuMeshCols, Valid(new MacCtrls())))
    val macReqSrcA   = Input(Vec(mxuMeshRows, UInt((mxuTileRows*16).W)))
    val macReqSrcB   = Input(Vec(mxuMeshCols, UInt((mxuTileCols*16).W)))
    val macResp      = Output(Valid(new MacCtrls()))
    // clear tile slices, control signals propagated vertically
    val clrReq       = Input(Valid(new ClrCtrls()))
    val clrResp      = Output(Valid(new ClrCtrls()))
    // read row slices, control signals propagated vertically
    val rowReadReq   = Input(Vec(numReadPorts,  Valid(new SliceCtrls())))
    val rowReadResp  = Output(Vec(numReadPorts, Valid(new SliceCtrls())))
    val rowReadData  = Output(Vec(numReadPorts, UInt((mxuPECols*32).W)))
    // write row slices, control signals propagated vertically
    val rowWriteReq  = Input(Valid(new SliceCtrls()))
    val rowWriteData = Input(UInt((mxuPECols*32).W))
    val rowWriteMask = Input(UInt((mxuPECols).W)) 
    val rowWriteResp = Output(Valid(new SliceCtrls()))
    // read col slice, control signals propagated horizontally
    val colReadReq   = Input(Vec(numReadPorts,  Valid(new SliceCtrls())))
    val colReadResp  = Output(Vec(numReadPorts, Valid(new SliceCtrls())))
    val colReadData  = Output(Vec(numReadPorts, UInt((mxuPERows*32).W)))
    // write col slice, control signals propagated horizontally
    val colWriteReq  = Input(Valid(new SliceCtrls()))
    val colWriteData = Input(UInt((mxuPERows*32).W))
    val colWriteMask = Input(UInt((mxuPERows).W))
    val colWriteResp = Output(Valid(new SliceCtrls()))
  })

  val mesh  = Seq.tabulate(mxuMeshRows, mxuMeshCols)((i, j) => Module(new Tile(i*mxuTileRows, j*mxuTileCols, numReadPorts)))
  val meshT = mesh.transpose

  // propagate horizontally across the mesh
  for(r <- 0 until mxuMeshRows) {
    mesh(r).foldLeft(io.macReqSrcA(r)) {
      case(macReqSrcA, tile) => {
        tile.io.macReqSrcA := RegNext(macReqSrcA)
        tile.io.macOutSrcA
      }
    }
    mesh(r).foldLeft(io.colWriteReq) {
      case(colWriteReq, tile) => {
        tile.io.colWriteReq := RegNext(colWriteReq)
        tile.io.colWriteResp
      }
    }
    mesh(r).foldLeft(io.colWriteData(32*mxuTileRows*(r+1)-1, 32*mxuTileRows*r)) {
      case(colWriteData, tile) => {
        tile.io.colWriteDin := RegNext(colWriteData)
        tile.io.colWriteDout
      }
    }
    mesh(r).foldLeft(io.colWriteMask(mxuTileRows*(r+1)-1, mxuTileRows*r)) {
      case(colWriteMask, tile) => {
        tile.io.colWriteMask := RegNext(colWriteMask)
        tile.io.colWriteMout
      }
    }
    for(i <- 0 until numReadPorts) {
      mesh(r).foldLeft(io.colReadReq(i)) {
        case(colReadReq, tile) => {
          tile.io.colReadReq(i) := RegNext(colReadReq)
          tile.io.colReadResp(i)
        }
      }
      mesh(r).foldLeft(0.U(32.W)) {
        case(colReadData, tile) => {
          tile.io.colReadDin(i) := RegNext(colReadData)
          tile.io.colReadDout(i)
        }
      }
    }
  }

  // propagate vertically across the mesh
  for(c <- 0 until mxuMeshCols) {
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
    meshT(c).foldLeft(io.rowWriteReq) {
      case(rowWriteReq, tile) => {
        tile.io.rowWriteReq := RegNext(rowWriteReq)
        tile.io.rowWriteResp
      }
    }
    meshT(c).foldLeft(io.rowWriteData(64*mxuTileCols*(c+1)-1, 64*mxuTileCols*c)) {
      case(rowWriteData, tile) => {
        tile.io.rowWriteDin := RegNext(rowWriteData)
        tile.io.rowWriteDout
      }
    }
    meshT(c).foldLeft(io.rowWriteMask(2*mxuTileCols*(c+1)-1, 2*mxuTileCols*c)) {
      case(rowWriteMask, tile) => {
        tile.io.rowWriteMask := RegNext(rowWriteMask)
        tile.io.rowWriteMout
      }
    }
    for(i <- 0 until numReadPorts) {
      meshT(c).foldLeft(io.rowReadReq(i)) {
        case(rowReadReq, tile) => {
          tile.io.rowReadReq(i) := RegNext(rowReadReq)
          tile.io.rowReadResp(i)
        }
      }
      meshT(c).foldLeft(0.U(64.W)) {
        case(rowReadData, tile) => {
          tile.io.rowReadDin(i) := RegNext(rowReadData)
          tile.io.rowReadDout(i)
        }
      }
    }
  }

  // bottom IOs
  io.macResp      := mesh(mxuMeshRows-1)(mxuMeshCols-1).io.macReqOut
  io.clrResp      := mesh(mxuMeshRows-1)(mxuMeshCols-1).io.clrReqOut
  io.rowWriteResp := RegNext(mesh(mxuMeshRows-1)(mxuMeshCols-1).io.rowWriteResp)
  io.colWriteResp := RegNext(mesh(mxuMeshRows-1)(mxuMeshCols-1).io.colWriteResp)

  for(i <- 0 until numReadPorts) {
    io.rowReadResp(i).valid := RegNext(mesh(mxuMeshRows-1)(mxuMeshCols-1).io.rowReadResp(i).valid)
    io.rowReadResp(i).bits  := RegEnable(mesh(mxuMeshRows-1)(mxuMeshCols-1).io.rowReadResp(i).bits, mesh(mxuMeshRows-1)(mxuMeshCols-1).io.rowReadResp(i).valid)
    io.colReadResp(i).valid := RegNext(mesh(mxuMeshRows-1)(mxuMeshCols-1).io.colReadResp(i).valid)
    io.colReadResp(i).bits  := RegEnable(mesh(mxuMeshRows-1)(mxuMeshCols-1).io.colReadResp(i).bits, mesh(mxuMeshRows-1)(mxuMeshCols-1).io.colReadResp(i).valid)
    val colRdataMux = WireInit(VecInit(Seq.fill(mxuMeshRows)(0.U((mxuTileRows*32).W))))
    for(r <- 0 until mxuMeshRows) {
      // use RegEnable to lock valid data
      colRdataMux(r) := RegEnable(mesh(r)(mxuMeshCols-1).io.colReadDout(i), mesh(r)(mxuMeshCols-1).io.colReadResp(i).valid)
    }

    val rowRdataMux = WireInit(VecInit(Seq.fill(mxuMeshCols)(0.U((mxuTileCols*64).W))))
    for(c <- 0 until mxuMeshCols) {
      rowRdataMux(c) := RegEnable(mesh(mxuMeshRows-1)(c).io.rowReadDout(i), mesh(mxuMeshRows-1)(c).io.rowReadResp(i).valid)
    }
    io.colReadData(i) := colRdataMux.asUInt
    io.rowReadData(i) := rowRdataMux.asUInt
  }
}

// instantiate MESH with Delays
class MXU(implicit p: Parameters) extends BoomModule 
{
  val io = IO(new Bundle{
    // matrix multiplication relate
    val macReq          = Flipped(Decoupled(new MacCtrls()))
    val macReqUop       = Input(new MicroOp())
    val macReqSrcA      = Input(UInt(rLen.W))
    val macReqSrcB      = Input(UInt(rLen.W))
    val macResp         = Output(Valid(new MacCtrls()))
    val macRespUop      = Output(new MicroOp())
    // clear tile slices, control signals propagated vertically
    val clrReq          = Flipped(Decoupled(new ClrCtrls()))
    val clrReqUop       = Input(new MicroOp())
    val clrResp         = Output(Valid(new ClrCtrls()))
    val clrRespUop      = Output(new MicroOp())
    // read row slices, m-pipeline
    val rowReadReq      = Flipped(Decoupled(new SliceCtrls()))
    val rowReadReqUop   = Input(new MicroOp())
    val rowReadResp     = Output(Valid(new SliceCtrls()))
    val rowReadRespUop  = Output(new MicroOp())
    val rowReadData     = Decoupled(UInt(rLen.W))
    val rowReadMask     = Output(UInt(rLenb.W))
    // write row slices
    val rowWriteReq     = Flipped(Decoupled(new SliceCtrls()))
    val rowWriteReqUop  = Input(new MicroOp())
    val rowWriteData    = Input(UInt(rLen.W))
    val rowWriteMask    = Input(UInt(rLenb.W))
    val rowWriteResp    = Output(Valid(new SliceCtrls()))
    val rowWriteRespUop = Output(new MicroOp())
    // read col slices, m-pipeline
    val colReadReq      = Flipped(Decoupled(new SliceCtrls()))
    val colReadReqUop   = Input(new MicroOp())
    val colReadResp     = Output(Valid(new SliceCtrls()))
    val colReadRespUop  = Output(new MicroOp())
    val colReadData     = Decoupled(UInt(rLen.W))
    val colReadMask     = Output(UInt(rLenb.W))
    // write col slices
    val colWriteReq     = Flipped(Decoupled(new SliceCtrls()))
    val colWriteReqUop  = Input(new MicroOp())
    val colWriteData    = Input(UInt(rLen.W))
    val colWriteMask    = Input(UInt(rLenb.W))
    val colWriteResp    = Output(Valid(new SliceCtrls()))
    val colWriteRespUop = Output(new MicroOp())
    // read row slices from acc, lsu
    val accReadReq      = Input(Valid(new AccReadReq()))
    val accReadResp     = Output(Valid(new AccReadResp()))
  })

  require(rLen >= mxuPERows*16)

  val macReqFire = io.macReq.valid & io.macReq.ready
  val clrReqFire = io.clrReq.valid & io.clrReq.ready

  val mesh = Module(new Mesh(numReadPorts = 2))

  // row slice writes and col slice writes may occured simultaneously when writing different acc tiles,
  // should be guaranteed by rename/issue logic
  val sliceReady :: sliceWait :: sliceResp :: trSliceResp :: Nil = Enum(4)
  val rowReadState     = RegInit(sliceReady)
  val colReadState     = RegInit(sliceReady)
  val rowVsCount       = RegInit(0.U(2.W))
  val colVsCount       = RegInit(0.U(2.W))
  val rowReadReqValid  = RegInit(false.B)
  val rowWriteReqValid = RegInit(false.B)
  val colReadReqValid  = RegInit(false.B)
  val colWriteReqValid = RegInit(false.B)
  val rowWriteDataAggr = RegInit(VecInit(Seq.fill(mxuPECols)(0.U(32.W))))
  val rowWriteMaskAggr = RegInit(VecInit(Seq.fill(mxuPECols)(0.U(1.W))))
  val colWriteDataAggr = RegInit(VecInit(Seq.fill(mxuPERows)(0.U(32.W))))
  val colWriteMaskAggr = RegInit(VecInit(Seq.fill(mxuPERows)(0.U(1.W))))

  // -----------------------------------------------------------------------------------
  // read row slices, write data to long-latency vector registers
  // add back-pressure mechanism (not pipelined)
  // -----------------------------------------------------------------------------------
  val trRowValid   = ShiftRegister(io.rowReadReq.valid && io.rowReadReqUop.rt(RS1, isTrTile), mxuMeshRows+2)
  val trRowData    = Pipe(io.rowReadReq.valid, io.macReqSrcA, mxuMeshRows+2).bits
  val rowReadCtrls = io.rowReadReq.bits
  val rowVregIdx   = RegInit(0.U(3.W))
  rowReadReqValid := false.B
  switch(rowReadState) {
    is(sliceReady) {
      when(io.rowReadReq.valid && io.rowReadReqUop.rt(RS1, isAccTile)) {
        rowVsCount      := Mux(io.rowReadReqUop.vd_emul === 2.U, 3.U, io.rowReadReqUop.vd_emul)
        rowReadState    := sliceWait
        rowReadReqValid := true.B
      } .elsewhen(io.rowReadReq.valid) {
        rowReadState    := sliceWait
      }
    }
    is(sliceWait) {
      when(io.rowReadData.fire && rowVsCount === 0.U) {
        rowReadState := sliceReady 
      } .elsewhen(io.rowReadData.fire) {
        rowReadState := sliceResp
        rowVregIdx   := rowVregIdx + 1.U
      } .elsewhen(mesh.io.rowReadResp(0).valid) {
        rowReadState := sliceResp
      } .elsewhen(trRowValid) {
        rowReadState := trSliceResp
      }
    }
    is(sliceResp) {
      when(io.rowReadData.fire && rowVregIdx === rowVsCount) {
        rowReadState := sliceReady
        rowVregIdx   := 0.U
      } .elsewhen(io.rowReadData.fire) {
        rowVregIdx   := rowVregIdx + 1.U
      }
    }
    is(trSliceResp) {
      when(io.rowReadData.fire) {
        rowReadState := sliceReady
      }
    }
  }

  // -----------------------------------------------------------------------------------
  // write row slices
  // -----------------------------------------------------------------------------------
  val rowWriteCtrls = io.rowWriteReq.bits
  rowWriteReqValid := false.B

  val rowSliceIdx = io.rowWriteReqUop.m_slice_quad
  when(io.rowWriteReq.valid) {
    (0 until mxuMeshCols*mxuTileCols*2).foreach { i => 
      rowWriteDataAggr(i) := 0.U
      rowWriteMaskAggr(i) := 0.U
    }
    when(rowWriteCtrls.sew === 0.U) {
      (0 until mxuMeshCols*mxuTileCols).foreach { i => 
        rowWriteDataAggr(2*i)   := io.rowWriteData(8*i+7, 8*i)
        rowWriteMaskAggr(2*i)   := io.rowWriteMask(i)
        rowWriteDataAggr(2*i+1) := io.rowWriteData(8*mxuMeshCols*mxuTileCols+8*i+7, 8*mxuMeshCols*mxuTileCols+8*i)
        rowWriteMaskAggr(2*i+1) := io.rowWriteMask(mxuMeshCols*mxuTileCols+i)
      }
    } .elsewhen(rowWriteCtrls.sew === 1.U) {
      when(rowSliceIdx === W0) {
        (0 until mxuMeshCols*mxuTileCols).foreach { i => 
          rowWriteDataAggr(2*i) := io.rowWriteData(16*i+15, 16*i)
          rowWriteMaskAggr(2*i) := io.rowWriteMask(2*i)
        }
      } .otherwise {
        (0 until mxuMeshCols*mxuTileCols).foreach { i => 
          rowWriteDataAggr(2*i+1) := io.rowWriteData(16*i+15, 16*i)
          rowWriteMaskAggr(2*i+1) := io.rowWriteMask(2*i)
        }
      }
    } .otherwise {
      assert(rowWriteCtrls.sew === 2.U, "sew === 3.U (64 bits) not supported!\n")
      when(rowSliceIdx === Q0) {
        (0 until mxuMeshCols*mxuTileCols/2).foreach { i =>
          rowWriteDataAggr(2*i) := io.rowWriteData(32*i+31, 32*i)
          rowWriteMaskAggr(2*i) := io.rowWriteMask(4*i)
        }
      } .elsewhen(rowSliceIdx === Q1) {
         (0 until mxuMeshCols*mxuTileCols/2).foreach { i => 
           rowWriteDataAggr(mxuMeshCols*mxuTileCols+2*i) := io.rowWriteData(32*i+31, 32*i)
           rowWriteMaskAggr(mxuMeshCols*mxuTileCols+2*i) := io.rowWriteMask(4*i)
         }
      } .elsewhen(rowSliceIdx === Q2) {
        (0 until mxuMeshCols*mxuTileCols/2).foreach { i => 
          rowWriteDataAggr(2*i+1) := io.rowWriteData(32*i+31, 32*i)
          rowWriteMaskAggr(2*i+1) := io.rowWriteMask(4*i)
        }
      } .otherwise {
        (0 until mxuMeshCols*mxuTileCols/2).foreach { i => 
          rowWriteDataAggr(mxuMeshCols*mxuTileCols+2*i+1) := io.rowWriteData(32*i+31, 32*i)
          rowWriteMaskAggr(mxuMeshCols*mxuTileCols+2*i+1) := io.rowWriteMask(4*i)
        }
      }
    }
  }

  // -----------------------------------------------------------------------------------
  // read col slices, write data to long-latency vector registers
  // add back-pressure mechanism (not pipelined)
  // -----------------------------------------------------------------------------------
  val trColValid   = ShiftRegister(io.colReadReq.valid && io.colReadReqUop.rt(RS1, isTrTile), mxuMeshCols+2)
  val trColData    = Pipe(io.colReadReq.valid, io.macReqSrcA, mxuMeshCols).bits
  val colReadCtrls = io.colReadReq.bits
  val colVregIdx   = RegInit(0.U(3.W))
  colReadReqValid := false.B
  switch(colReadState) {
    is(sliceReady) {
      when(io.colReadReq.valid && io.colReadReqUop.rt(RS1, isAccTile)) {
        colVsCount   := io.colReadReqUop.vd_emul
        colReadState := sliceWait
        colReadReqValid := true.B
      } .elsewhen(io.colReadReq.valid) {
        colReadState := sliceWait
      }
    }
    is(sliceWait) {
      when(io.colReadData.fire && colVsCount === 0.U) {
        colReadState := sliceReady 
      } .elsewhen(io.colReadData.fire) {
        colReadState := sliceResp
        colVregIdx   := colVregIdx + 1.U
      } .elsewhen(mesh.io.colReadResp(0).valid) {
        colReadState := sliceResp
      } .elsewhen(trColValid) {
        colReadState := trSliceResp
      }
    }
    is(sliceResp) {
      when(io.colReadData.fire && colVregIdx === colVsCount) {
        colReadState := sliceReady
        colVregIdx   := 0.U
      } .elsewhen(io.colReadData.fire) {
        colVregIdx   := colVregIdx + 1.U
      }
    }
    is(trSliceResp) {
      when(io.colReadData.fire) {
        colReadState := sliceReady
      }
    }
  }

  // -----------------------------------------------------------------------------------
  // write col slices
  // -----------------------------------------------------------------------------------
  val colWriteCtrls = io.colWriteReq.bits
  colWriteReqValid := false.B

  val colSliceIdx = io.colWriteReqUop.m_slice_quad
  when(io.colWriteReq.valid) {
    (0 until mxuMeshRows*mxuTileRows).foreach { i => 
      colWriteDataAggr(i) := 0.U
      colWriteMaskAggr(i) := 0.U
    }
    when(colWriteCtrls.sew === 0.U) {
      (0 until mxuMeshRows*mxuTileRows).foreach { i => 
        colWriteDataAggr(i) := io.colWriteData(8*i+7, 8*i)
        colWriteMaskAggr(i) := io.colWriteMask(i)
      }
    } .elsewhen(colWriteCtrls.sew === 1.U) {
      (0 until mxuMeshRows*mxuTileRows).foreach { i => 
        colWriteDataAggr(i) := io.colWriteData(16*i+15, 16*i)
        colWriteMaskAggr(i) := io.colWriteMask(2*i)
      }
    } .otherwise {
      when(colSliceIdx === Q0) {
        (0 until mxuMeshRows*mxuTileRows/2).foreach { i => 
          colWriteDataAggr(i) := io.colWriteData(32*i+31, 32*i)
          colWriteMaskAggr(i) := io.colWriteMask(4*i)
        }
      } .otherwise {
        (0 until mxuMeshRows*mxuTileRows/2).foreach { i => 
          colWriteDataAggr(mxuMeshRows*mxuTileRows/2+i) := io.colWriteData(32*i+31, 32*i)
          colWriteDataAggr(mxuMeshRows*mxuTileRows/2+i) := io.colWriteMask(4*i)
        }
      }
      assert(colWriteCtrls.sew === 2.U, "INT64 and FP64 not supported\n")
    }
  }

  // -----------------------------------------------------------------------------------
  // mopa 
  // -----------------------------------------------------------------------------------
  mesh.io.clrReq.valid        := clrReqFire
  mesh.io.clrReq.bits         := io.clrReq.bits
  mesh.io.rowWriteReq.valid   := RegNext(io.rowWriteReq.valid)
  mesh.io.rowWriteReq.bits    := RegNext(io.rowWriteReq.bits)
  mesh.io.rowWriteData        := rowWriteDataAggr.asUInt
  mesh.io.rowWriteMask        := rowWriteMaskAggr.asUInt
  mesh.io.colWriteReq.valid   := RegNext(io.colWriteReq.valid)
  mesh.io.colWriteReq.bits    := RegNext(io.colWriteReq.bits)
  mesh.io.colWriteData        := colWriteDataAggr.asUInt
  mesh.io.colWriteMask        := colWriteMaskAggr.asUInt
  mesh.io.rowReadReq(0).valid := rowReadReqValid
  mesh.io.rowReadReq(0).bits  := RegNext(io.rowReadReq.bits)
  mesh.io.colReadReq(0).valid := colReadReqValid
  mesh.io.colReadReq(0).bits  := RegNext(io.colReadReq.bits)
  mesh.io.rowReadReq(1).valid := io.accReadReq.valid && !io.accReadReq.bits.tt.asBool
  mesh.io.rowReadReq(1).bits  := io.accReadReq.bits.sCtrls
  mesh.io.colReadReq(1).valid := io.accReadReq.valid && io.accReadReq.bits.tt.asBool
  mesh.io.colReadReq(1).bits  := io.accReadReq.bits.sCtrls
  for(c <- 0 until mxuMeshCols) {
    mesh.io.macReq(c).valid := ShiftRegister(macReqFire, c, true.B)
    mesh.io.macReq(c).bits  := ShiftRegister(io.macReq.bits, c, true.B) 
  }

  val widenSrcA = WireInit(VecInit(Seq.fill(mxuMeshRows*mxuTileRows)(0.U(16.W))))
  (0 until mxuMeshRows*mxuTileRows).foreach(i => widenSrcA(i) := io.macReqSrcA(8*i+7, 8*i))
  val muxSrcA = Mux(io.macReq.bits.srcType(2), io.macReqSrcA, widenSrcA.asUInt)
  for(r <- 0 until mxuMeshRows) {
    mesh.io.macReqSrcA(r) := ShiftRegister(muxSrcA(16*mxuTileRows*(r+1)-1, 16*mxuTileRows*r), r, true.B)
  }

  val shuffSrcB = WireInit(VecInit(Seq.fill(mxuMeshCols*mxuTileCols*2)(0.U(8.W))))
  when(io.macReq.bits.srcType(2)) {
    (0 until mxuMeshCols*mxuTileCols).foreach { i =>
      shuffSrcB(2*i)   := io.macReqSrcB(16*i+7,  16*i)
      shuffSrcB(2*i+1) := io.macReqSrcB(16*i+15, 16*i+8)
    }
  } .otherwise {
    (0 until mxuMeshCols*mxuTileCols).foreach { i =>
      shuffSrcB(2*i)   := io.macReqSrcB(8*i+7, 8*i)
      shuffSrcB(2*i+1) := io.macReqSrcB(8*mxuMeshCols*mxuTileCols+8*i+7, 8*mxuMeshCols*mxuTileCols+8*i)
    }
  }
  val muxSrcB = shuffSrcB.asUInt
  for(c <- 0 until mxuMeshCols) {
    mesh.io.macReqSrcB(c) := ShiftRegister(muxSrcB(16*mxuTileCols*(c+1)-1, 16*mxuTileCols*c), c, true.B)
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
  io.macRespUop        := Pipe(macReqFire, io.macReqUop, mxuMeshRows+mxuMeshCols-1).bits
  io.clrResp           := mesh.io.clrResp
  io.clrRespUop        := Pipe(clrReqFire, io.clrReqUop, mxuMeshRows).bits
  io.rowWriteResp      := mesh.io.rowWriteResp
  io.rowWriteRespUop   := Pipe(io.rowWriteReq.valid, io.rowWriteReqUop, mxuMeshRows+2).bits
  io.colWriteResp      := mesh.io.colWriteResp
  io.colWriteRespUop   := Pipe(io.colWriteReq.valid, io.colWriteReqUop, mxuMeshCols+2).bits
  io.rowReadResp       := mesh.io.rowReadResp(0)
  io.colReadResp       := mesh.io.colReadResp(0)

  //------------------------------------------------------------
  // read response, m-pipeline
  val rowRespCtrls   = mesh.io.rowReadResp(0).bits
  val rowReadDataMux = WireInit(VecInit(Seq.fill(mxuMeshCols*mxuTileCols*2)(0.U(8.W))))
  when(rowRespCtrls.sew === 0.U) {
    (0 until mxuMeshCols*mxuTileCols).foreach { i => 
      rowReadDataMux(i)                   := mesh.io.rowReadData(0)(64*i+7,  64*i)
      rowReadDataMux(mxuMeshCols*mxuTileCols+i) := mesh.io.rowReadData(0)(64*i+39, 64*i+32)
    }
  } .elsewhen(rowRespCtrls.sew === 1.U) {
    when(rowVregIdx === 0.U) {
      (0 until mxuMeshCols*mxuTileCols).foreach { i => 
        rowReadDataMux(2*i)   := mesh.io.rowReadData(0)(64*i+7,  64*i)
        rowReadDataMux(2*i+1) := mesh.io.rowReadData(0)(64*i+15, 64*i+8)
      }
    } .otherwise {
      (0 until mxuMeshCols*mxuTileCols).foreach { i => 
        rowReadDataMux(2*i)   := mesh.io.rowReadData(0)(64*i+39, 64*i+32)
        rowReadDataMux(2*i+1) := mesh.io.rowReadData(0)(64*i+47, 64*i+40)
      }
    }
  } .otherwise {
    when(rowVregIdx === 0.U) {
      (0 until mxuMeshCols*mxuTileCols/2).foreach { i => 
        rowReadDataMux(4*i)   := mesh.io.rowReadData(0)(64*i+7,  64*i)
        rowReadDataMux(4*i+1) := mesh.io.rowReadData(0)(64*i+15, 64*i+8)
        rowReadDataMux(4*i+2) := mesh.io.rowReadData(0)(64*i+23, 64*i+16)
        rowReadDataMux(4*i+3) := mesh.io.rowReadData(0)(64*i+31, 64*i+24)
      }
    } .elsewhen(rowVregIdx === 1.U) {
      (0 until mxuMeshCols*mxuTileCols/2).foreach { i => 
        rowReadDataMux(4*i)   := mesh.io.rowReadData(0)(32*mxuMeshCols*mxuTileCols+64*i+7,  32*mxuMeshCols*mxuTileCols+64*i)
        rowReadDataMux(4*i+1) := mesh.io.rowReadData(0)(32*mxuMeshCols*mxuTileCols+64*i+15, 32*mxuMeshCols*mxuTileCols+64*i+8)
        rowReadDataMux(4*i+2) := mesh.io.rowReadData(0)(32*mxuMeshCols*mxuTileCols+64*i+23, 32*mxuMeshCols*mxuTileCols+64*i+16)
        rowReadDataMux(4*i+3) := mesh.io.rowReadData(0)(32*mxuMeshCols*mxuTileCols+64*i+31, 32*mxuMeshCols*mxuTileCols+64*i+24)
      }
    } .elsewhen(rowVregIdx === 2.U) {
      (0 until mxuMeshCols*mxuTileCols/2).foreach { i => 
        rowReadDataMux(4*i)   := mesh.io.rowReadData(0)(64*i+39, 64*i+32)
        rowReadDataMux(4*i+1) := mesh.io.rowReadData(0)(64*i+47, 64*i+40)
        rowReadDataMux(4*i+2) := mesh.io.rowReadData(0)(64*i+55, 64*i+48)
        rowReadDataMux(4*i+3) := mesh.io.rowReadData(0)(64*i+63, 64*i+56)
      }
    } .otherwise {
      (0 until mxuMeshCols*mxuTileCols/2).foreach { i => 
        rowReadDataMux(4*i)   := mesh.io.rowReadData(0)(32*mxuMeshCols*mxuTileCols+64*i+39, 32*mxuMeshCols*mxuTileCols+64*i+32)
        rowReadDataMux(4*i+1) := mesh.io.rowReadData(0)(32*mxuMeshCols*mxuTileCols+64*i+47, 32*mxuMeshCols*mxuTileCols+64*i+40)
        rowReadDataMux(4*i+2) := mesh.io.rowReadData(0)(32*mxuMeshCols*mxuTileCols+64*i+55, 32*mxuMeshCols*mxuTileCols+64*i+48)
        rowReadDataMux(4*i+3) := mesh.io.rowReadData(0)(32*mxuMeshCols*mxuTileCols+64*i+63, 32*mxuMeshCols*mxuTileCols+64*i+56)
      }
    }
  }

  val colRespCtrls   = mesh.io.colReadResp(0).bits
  val colReadDataMux = WireInit(VecInit(Seq.fill(mxuMeshRows*mxuTileRows*2)(0.U(8.W))))
  when(colRespCtrls.sew === 0.U) {
    (0 until mxuMeshRows*mxuTileRows).foreach { i => 
      colReadDataMux(i) := mesh.io.colReadData(0)(32*i+7, 32*i)
    }
  } .elsewhen(colRespCtrls.sew === 1.U) {
    (0 until mxuMeshRows*mxuTileRows).foreach { i => 
      colReadDataMux(2*i)   := mesh.io.colReadData(0)(32*i+7,  32*i)
      colReadDataMux(2*i+1) := mesh.io.colReadData(0)(32*i+15, 32*i+8)
    }
  } .otherwise {
    when(colVregIdx === 0.U) {
      (0 until mxuMeshRows*mxuTileRows/2).foreach { i => 
        colReadDataMux(4*i)   := mesh.io.colReadData(0)(32*i+7,  32*i)
        colReadDataMux(4*i+1) := mesh.io.colReadData(0)(32*i+15, 32*i+8)
        colReadDataMux(4*i+2) := mesh.io.colReadData(0)(32*i+23, 32*i+16)
        colReadDataMux(4*i+3) := mesh.io.colReadData(0)(32*i+31, 32*i+24)
      }
    } .otherwise {
      (0 until mxuMeshRows*mxuTileRows/2).foreach { i => 
        colReadDataMux(4*i)   := mesh.io.colReadData(0)(16*mxuMeshRows*mxuTileRows+32*i+7,  16*mxuMeshRows*mxuTileRows+32*i)
        colReadDataMux(4*i+1) := mesh.io.colReadData(0)(16*mxuMeshRows*mxuTileRows+32*i+15, 16*mxuMeshRows*mxuTileRows+32*i+8)
        colReadDataMux(4*i+2) := mesh.io.colReadData(0)(16*mxuMeshRows*mxuTileRows+32*i+23, 16*mxuMeshRows*mxuTileRows+32*i+16)
        colReadDataMux(4*i+3) := mesh.io.colReadData(0)(16*mxuMeshRows*mxuTileRows+32*i+31, 16*mxuMeshRows*mxuTileRows+32*i+24)
      }
    }
  }

  // mask generation:
  val rowVregOff  = rowVregIdx << (rLenbSz.U - io.rowReadRespUop.ts1_eew)
  val rowMask     = Cat((0 until rLenb).map(i => i.U + rowVregOff < io.rowReadRespUop.m_slice_len).reverse)
  val rowByteMask = Mux1H(UIntToOH(io.rowReadRespUop.ts1_eew),
                           Seq(rowMask,
                               FillInterleaved(2, rowMask(rLen/16-1, 0)),
                               FillInterleaved(4, rowMask(rLen/32-1, 0)),
                               FillInterleaved(8, rowMask(rLen/64-1, 0))))
  val colVregOff  = colVregIdx << (rLenbSz.U - io.colReadRespUop.ts1_eew)
  val colMask     = Cat((0 until rLenb).map(i => i.U + colVregOff < io.colReadRespUop.m_slice_len).reverse)
  val colByteMask = Mux1H(UIntToOH(io.colReadRespUop.ts1_eew),
                           Seq(colMask,
                               FillInterleaved(2, colMask(rLen/16-1, 0)),
                               FillInterleaved(4, colMask(rLen/32-1, 0)),
                               FillInterleaved(8, colMask(rLen/64-1, 0))))
  
  io.rowReadData.valid := mesh.io.rowReadResp(0).valid || (rowReadState === sliceResp) || trRowValid || (rowReadState === trSliceResp)
  io.rowReadData.bits  := Mux(trRowValid || (rowReadState === trSliceResp), trRowData, rowReadDataMux.asUInt)
  io.rowReadMask       := rowByteMask
  
  io.colReadData.valid := mesh.io.colReadResp(0).valid || (colReadState === sliceResp) || trColValid || (colReadState === trSliceResp)
  io.colReadData.bits  := Mux(trColValid || (colReadState === trSliceResp), trColData, colReadDataMux.asUInt)
  io.colReadMask       := colByteMask

  val pipeRowReadUop = Pipe(io.rowReadReq.valid, io.rowReadReqUop, mxuMeshRows+2).bits
  io.rowReadRespUop := pipeRowReadUop
  io.rowReadRespUop.m_split_last  := rowVregIdx === rowVsCount
  io.rowReadRespUop.v_split_last  := rowVregIdx === rowVsCount
  io.rowReadRespUop.pdst          := pipeRowReadUop.pvd(rowVregIdx).bits 
  io.rowReadRespUop.stale_pdst    := pipeRowReadUop.stale_pvd(rowVregIdx).bits

  val pipeColReadUop = Pipe(io.colReadReq.valid, io.colReadReqUop, mxuMeshCols+2).bits
  io.colReadRespUop := pipeColReadUop
  io.colReadRespUop.m_split_last  := colVregIdx === colVsCount
  io.colReadRespUop.v_split_last  := colVregIdx === colVsCount
  io.colReadRespUop.pdst          := pipeColReadUop.pvd(colVregIdx).bits
  io.colReadRespUop.stale_pdst    := pipeColReadUop.stale_pvd(colVregIdx).bits

  //------------------------------------------------------------
  // read response, lsu
  val accRowReadCtrls = Pipe(io.accReadReq, mxuMeshRows+1)
  val accRowReadData  = WireInit(VecInit(Seq.fill(mxuMeshCols*mxuTileCols*2)(0.U(8.W))))

  val accRowReadSew   = accRowReadCtrls.bits.sCtrls.sew
  val accRowReadQuad  = accRowReadCtrls.bits.quad
  when(accRowReadSew === 0.U) {
    (0 until mxuMeshCols*mxuTileCols).foreach { i => 
      accRowReadData(i)                   := mesh.io.rowReadData(1)(64*i+7,  64*i)
      accRowReadData(mxuMeshCols*mxuTileCols+i) := mesh.io.rowReadData(1)(64*i+39, 64*i+32)
    }
  } .elsewhen(accRowReadSew === 1.U) {
    when(accRowReadQuad === 0.U) {
      (0 until mxuMeshCols*mxuTileCols).foreach { i => 
        accRowReadData(2*i)   := mesh.io.rowReadData(1)(64*i+7,  64*i)
        accRowReadData(2*i+1) := mesh.io.rowReadData(1)(64*i+15, 64*i+8)
      }
    } .otherwise {
      (0 until mxuMeshCols*mxuTileCols).foreach { i => 
        accRowReadData(2*i)   := mesh.io.rowReadData(1)(64*i+39, 64*i+32)
        accRowReadData(2*i+1) := mesh.io.rowReadData(1)(64*i+47, 64*i+40)
      }
    }
  } .otherwise {
    when(accRowReadQuad === 0.U) {
      (0 until mxuMeshCols*mxuTileCols/2).foreach { i => 
        accRowReadData(4*i)   := mesh.io.rowReadData(1)(64*i+7,  64*i)
        accRowReadData(4*i+1) := mesh.io.rowReadData(1)(64*i+15, 64*i+8)
        accRowReadData(4*i+2) := mesh.io.rowReadData(1)(64*i+23, 64*i+16)
        accRowReadData(4*i+3) := mesh.io.rowReadData(1)(64*i+31, 64*i+24)
      }
    } .elsewhen(accRowReadQuad === 1.U) {
      (0 until mxuMeshCols*mxuTileCols/2).foreach { i => 
        accRowReadData(4*i)   := mesh.io.rowReadData(1)(32*mxuMeshCols*mxuTileCols+64*i+7,  32*mxuMeshCols*mxuTileCols+64*i)
        accRowReadData(4*i+1) := mesh.io.rowReadData(1)(32*mxuMeshCols*mxuTileCols+64*i+15, 32*mxuMeshCols*mxuTileCols+64*i+8)
        accRowReadData(4*i+2) := mesh.io.rowReadData(1)(32*mxuMeshCols*mxuTileCols+64*i+23, 32*mxuMeshCols*mxuTileCols+64*i+16)
        accRowReadData(4*i+3) := mesh.io.rowReadData(1)(32*mxuMeshCols*mxuTileCols+64*i+31, 32*mxuMeshCols*mxuTileCols+64*i+24)
      }
    } .elsewhen(accRowReadQuad === 2.U) {
      (0 until mxuMeshCols*mxuTileCols/2).foreach { i => 
        accRowReadData(4*i)   := mesh.io.rowReadData(1)(64*i+39, 64*i+32)
        accRowReadData(4*i+1) := mesh.io.rowReadData(1)(64*i+47, 64*i+40)
        accRowReadData(4*i+2) := mesh.io.rowReadData(1)(64*i+55, 64*i+48)
        accRowReadData(4*i+3) := mesh.io.rowReadData(1)(64*i+63, 64*i+56)
      }
    } .otherwise {
      (0 until mxuMeshCols*mxuTileCols/2).foreach { i => 
        accRowReadData(4*i)   := mesh.io.rowReadData(1)(32*mxuMeshCols*mxuTileCols+64*i+39, 32*mxuMeshCols*mxuTileCols+64*i+32)
        accRowReadData(4*i+1) := mesh.io.rowReadData(1)(32*mxuMeshCols*mxuTileCols+64*i+47, 32*mxuMeshCols*mxuTileCols+64*i+40)
        accRowReadData(4*i+2) := mesh.io.rowReadData(1)(32*mxuMeshCols*mxuTileCols+64*i+55, 32*mxuMeshCols*mxuTileCols+64*i+48)
        accRowReadData(4*i+3) := mesh.io.rowReadData(1)(32*mxuMeshCols*mxuTileCols+64*i+63, 32*mxuMeshCols*mxuTileCols+64*i+56)
      }
    }
  }

  val accColReadCtrls = Pipe(io.accReadReq, mxuMeshCols+1)
  val accColReadData  = WireInit(VecInit(Seq.fill(mxuMeshRows*mxuTileRows*2)(0.U(8.W))))

  val accColReadSew   = accColReadCtrls.bits.sCtrls.sew
  val accColReadQuad  = accColReadCtrls.bits.quad
  when(accColReadSew === 0.U) {
    (0 until mxuMeshRows*mxuTileRows).foreach { i => 
      accColReadData(i) := mesh.io.colReadData(1)(32*i+7, 32*i)
    }
  } .elsewhen(accColReadSew === 1.U) {
    (0 until mxuMeshRows*mxuTileRows).foreach { i => 
      accColReadData(2*i)   := mesh.io.colReadData(1)(32*i+7,  32*i)
      accColReadData(2*i+1) := mesh.io.colReadData(1)(32*i+15, 32*i+8)
    }
  } .otherwise {
    when(accColReadQuad === 0.U) {
      (0 until mxuMeshRows*mxuTileRows/2).foreach { i => 
        accColReadData(4*i)   := mesh.io.colReadData(1)(32*i+7,  32*i)
        accColReadData(4*i+1) := mesh.io.colReadData(1)(32*i+15, 32*i+8)
        accColReadData(4*i+2) := mesh.io.colReadData(1)(32*i+23, 32*i+16)
        accColReadData(4*i+3) := mesh.io.colReadData(1)(32*i+31, 32*i+24)
      }
    } .otherwise {
      (0 until mxuMeshRows*mxuTileRows/2).foreach { i => 
        accColReadData(4*i)   := mesh.io.colReadData(1)(16*mxuMeshRows*mxuTileRows+32*i+7,  16*mxuMeshRows*mxuTileRows+32*i)
        accColReadData(4*i+1) := mesh.io.colReadData(1)(16*mxuMeshRows*mxuTileRows+32*i+15, 16*mxuMeshRows*mxuTileRows+32*i+8)
        accColReadData(4*i+2) := mesh.io.colReadData(1)(16*mxuMeshRows*mxuTileRows+32*i+23, 16*mxuMeshRows*mxuTileRows+32*i+16)
        accColReadData(4*i+3) := mesh.io.colReadData(1)(16*mxuMeshRows*mxuTileRows+32*i+31, 16*mxuMeshRows*mxuTileRows+32*i+24)
      }
    }
  }

  // align acc read latency
  val pipeAccReadCtrls   = WireInit(accRowReadCtrls)
  val pipeAccRowReadData = WireInit(accRowReadData)
  val pipeAccColReadData = WireInit(accColReadData)
  if(mxuMeshCols > mxuMeshRows) {
    pipeAccReadCtrls   := accColReadCtrls
    pipeAccRowReadData := ShiftRegister(accRowReadData, mxuMeshCols-mxuMeshRows, true.B)
  } else if(mxuMeshCols < mxuMeshRows) {
    pipeAccReadCtrls   := accRowReadCtrls
    pipeAccColReadData := ShiftRegister(accColReadData, mxuMeshRows-mxuMeshCols, true.B)
  }

  io.accReadResp.valid         := pipeAccReadCtrls.valid
  io.accReadResp.bits.data     := Mux(pipeAccReadCtrls.valid && pipeAccReadCtrls.bits.tt.asBool, pipeAccColReadData.asUInt, pipeAccRowReadData.asUInt)
  io.accReadResp.bits.vstq_idx := pipeAccReadCtrls.bits.vstq_idx
}