// See README.md for license details.
package boom.exu

import chisel3._
import chisel3.util._
import freechips.rocketchip.tile.FType
import hardfloat._

// acc buffer related command
object AccTileConstants 
{
  // access cmd
  val SLICE_READ  = 0.U(2.W)
  val SLICE_WRITE = 1.U(2.W)
  val SLICE_CLEAR = 2.U(2.W)
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
  val ridx    = UInt(log2Ceil(numAccTiles).W)
  val srcType = UInt(3.W)
  val outType = UInt(3.W)
  val rm      = UInt(3.W)                      // rounding mode
}

class SliceCtrls(val sliceNums: Int, val numAccTiles: Int = 2) extends Bundle {
  val cmd   = UInt(2.W)
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
    val macReqIn       = Input(Valid(new MacCtrls()))            // control signals
    val macReqSrcA     = Input(UInt(16.W))
    val macReqSrcB     = Input(UInt(16.W))
    val macReqOut      = Output(Valid(new MacCtrls()))
    val macOutSrcA     = Output(UInt(16.W))                      // for systolic horizontally
    val macOutSrcB     = Output(UInt(16.W))                      // for systolic vertically
    // read or write tile slices, control signals propagated vertically
    val rowSliceReqIn  = Input(Valid(new SliceCtrls(numMeshRows)))
    val rowSliceWdata  = Input(UInt(64.W))
    val rowSliceReqOut = Output(Valid(new SliceCtrls(numMeshRows)))
    val rowSliceRdata  = Output(UInt(64.W))
    // read or write col slice, control signals propagated horizontally
    val colSliceReqIn  = Input(Valid(new SliceCtrls(numMeshCols)))
    val colSliceWdata  = Input(UInt(32.W))
    val colSliceReqOut = Output(Valid(new SliceCtrls(numMeshCols)))
    val colSliceRdata  = Output(UInt(32.W))
  })

  // acc registers
  val c0 = Reg(Vec(numAccTiles, UInt(32.W)))
  val c1 = Reg(Vec(numAccTiles, UInt(32.W)))

  // -----------------------------------------------------------------------------------
  // matrix multiply-accumulate
  // -----------------------------------------------------------------------------------
  val macReqValid = io.macReqIn.valid
  val macReqCtrls = io.macReqIn.bits
  // fp16*fp16+fp32
  val fpMac = Module(new FpMacUnit())
  fpMac.io.src1           := io.macReqSrcA
  fpMac.io.src2           := io.macReqSrcB
  fpMac.io.src3           := c0(macReqCtrls.ridx)
  fpMac.io.srcType        := macReqCtrls.srcType
  fpMac.io.outType        := macReqCtrls.outType
  fpMac.io.roundingMode   := macReqCtrls.rm
  fpMac.io.detectTininess := hardfloat.consts.tininess_afterRounding

  // int8 MAC function units
  val intMac0 = Module(new IntMacUnit())
  val intMac1 = Module(new IntMacUnit())
  intMac0.io.src1    := io.macReqSrcA( 7, 0)
  intMac0.io.src2    := io.macReqSrcB( 7, 0)
  intMac0.io.src3    := c0(macReqCtrls.ridx)
  intMac0.io.srcType := macReqCtrls.srcType
  intMac0.io.outType := macReqCtrls.outType
  intMac1.io.src1    := io.macReqSrcA( 7, 0)
  intMac1.io.src2    := io.macReqSrcB(15, 8)
  intMac1.io.src3    := c1(macReqCtrls.ridx)
  intMac1.io.srcType := macReqCtrls.srcType
  intMac1.io.outType := macReqCtrls.outType

  // latency = 1 for int8 mac; 3 for fp16 mac
  when(macReqValid && macReqCtrls.srcType(2)) {
    c0(macReqCtrls.ridx) := fpMac.io.out
    // c0(macReqCtrls.ridx) := intMac0.io.out
  } .elsewhen(macReqValid) {
    c0(macReqCtrls.ridx) := intMac0.io.out
    c1(macReqCtrls.ridx) := intMac1.io.out
  }

  io.macReqOut   := io.macReqIn
  io.macOutSrcA  := io.macReqSrcA
  io.macOutSrcB  := io.macReqSrcB

  // -----------------------------------------------------------------------------------
  // read or write row slices
  // -----------------------------------------------------------------------------------
  val rowReqValid = io.rowSliceReqIn.valid
  val rowReqCtrls = io.rowSliceReqIn.bits
  val rowRdata    = WireInit(0.U(64.W))
  val rowReqHit   = rowReqValid && rowReqCtrls.sidx === rowIndex.U
  rowRdata := Mux(!rowReqHit || rowReqCtrls.cmd =/= SLICE_READ, io.rowSliceWdata,
              Mux(rowReqCtrls.rtype(2), c0(rowReqCtrls.ridx), Cat(c1(rowReqCtrls.ridx), c0(rowReqCtrls.ridx))))

  when(rowReqValid && rowReqCtrls.cmd === SLICE_CLEAR) {
    c0(rowReqCtrls.ridx) := 0.U
    c1(rowReqCtrls.ridx) := 0.U
  } .elsewhen(rowReqHit && rowReqCtrls.cmd === SLICE_WRITE) {
    c0(rowReqCtrls.ridx) := io.rowSliceWdata(31, 0)
    when(!rowReqCtrls.rtype(2)) { c1(rowReqCtrls.ridx) := io.rowSliceWdata(63, 32) }
  }

  io.rowSliceReqOut := io.rowSliceReqIn
  io.rowSliceRdata  := rowRdata

  // -----------------------------------------------------------------------------------
  // read or write col slices
  // -----------------------------------------------------------------------------------
  val colReqValid = io.colSliceReqIn.valid
  val colReqCtrls = io.colSliceReqIn.bits
  val colRdata    = WireInit(0.U(32.W))
  val colReqHitC0 = colReqValid && Mux(colReqCtrls.rtype(2), colReqCtrls.sidx === (colIndex >> 1).U, colReqCtrls.sidx === colIndex.U)
  val colReqHitC1 = colReqValid && !colReqCtrls.rtype(2) && colReqCtrls.sidx === (colIndex + 1).U
  colRdata := Mux(colReqHitC0 && colReqCtrls.cmd === SLICE_READ, c0(colReqCtrls.ridx), 
              Mux(colReqHitC1 && colReqCtrls.cmd === SLICE_READ, c1(colReqCtrls.ridx), io.colSliceWdata))

  when(colReqHitC0 && colReqCtrls.cmd === SLICE_WRITE) {
    c0(colReqCtrls.ridx) := io.colSliceWdata
  } .elsewhen(colReqHitC1 && colReqCtrls.cmd === SLICE_WRITE) {
    c1(colReqCtrls.ridx) := io.colSliceWdata
  }

  io.colSliceReqOut := io.colSliceReqIn
  io.colSliceRdata  := colRdata
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
    val macReqIn       = Input(Valid(new MacCtrls()))
    val macReqSrcA     = Input(UInt((tileRows*16).W))
    val macReqSrcB     = Input(UInt((tileCols*16).W))
    val macReqOut      = Output(Valid(new MacCtrls()))
    val macOutSrcA     = Output(UInt((tileRows*16).W))
    val macOutSrcB     = Output(UInt((tileCols*16).W))
    // read or write tile slices, control signals propagated vertically
    val rowSliceReqIn  = Input(Valid(new SliceCtrls(numMeshRows)))
    val rowSliceWdata  = Input(UInt((tileCols*64).W))
    val rowSliceReqOut = Output(Valid(new SliceCtrls(numMeshRows)))
    val rowSliceRdata  = Output(UInt((tileCols*64).W))
    // read or write col slice, control signals propagated horizontally
    val colSliceReqIn  = Input(Valid(new SliceCtrls(numMeshCols)))
    val colSliceWdata  = Input(UInt((tileRows*32).W))
    val colSliceReqOut = Output(Valid(new SliceCtrls(numMeshCols)))
    val colSliceRdata  = Output(UInt((tileRows*32).W))
  })

  val tile = Seq.tabulate(tileRows, tileCols)((i, j) => Module(new PE(numMeshRows, numMeshCols, indexh+i, indexv+j*2)))
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
    tile(r).foldLeft(io.colSliceReqIn) {
      case (colSliceReq, pe) => {
        pe.io.colSliceReqIn := colSliceReq
        pe.io.colSliceReqOut
      }
    }
    val peColWdata = io.colSliceWdata(32*r+31, 32*r)
    tile(r).foldLeft(peColWdata) {
      case (colSliceWdata, pe) => {
        pe.io.colSliceWdata := colSliceWdata
        pe.io.colSliceRdata
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
    tileT(c).foldLeft(io.rowSliceReqIn) {
      case(rowSliceReq, pe) => {
        pe.io.rowSliceReqIn := rowSliceReq
        pe.io.rowSliceReqOut
      }
    }
    val peRowWdata = io.rowSliceWdata(64*c+63, 64*c)
    tileT(c).foldLeft(peRowWdata) {
      case(rowSliceWdata, pe) => {
        pe.io.rowSliceWdata := rowSliceWdata
        pe.io.rowSliceRdata
      }
    }
  }

  // tile's bottom IO
  io.macReqOut      := io.macReqIn
  io.rowSliceReqOut := io.rowSliceReqIn
  io.colSliceReqOut := io.colSliceReqIn

  val macOutSrcAMux    = WireInit(VecInit(Seq.fill(tileRows)(0.U(16.W))))
  val colSliceRdataMux = WireInit(VecInit(Seq.fill(tileRows)(0.U(32.W))))
  for(r <- 0 until tileRows) {
    macOutSrcAMux(r)    := tile(r)(tileCols-1).io.macOutSrcA
    colSliceRdataMux(r) := tile(r)(tileCols-1).io.colSliceRdata
  }

  val macOutSrcBMux    = WireInit(VecInit(Seq.fill(tileCols)(0.U(16.W))))
  val rowSliceRdataMux = WireInit(VecInit(Seq.fill(tileCols)(0.U(64.W))))
  for(c <- 0 until tileCols) {
    macOutSrcBMux(c)    := tile(tileRows-1)(c).io.macOutSrcB
    rowSliceRdataMux(c) := tile(tileRows-1)(c).io.rowSliceRdata
  }
  io.macOutSrcA := macOutSrcAMux.asUInt
  io.macOutSrcB := macOutSrcBMux.asUInt
  io.colSliceRdata := colSliceRdataMux.asUInt
  io.rowSliceRdata := rowSliceRdataMux.asUInt
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
    val macReqIn       = Input(Vec(meshCols, Valid(new MacCtrls())))
    val macReqSrcA     = Input(Vec(meshRows, UInt((tileRows*16).W)))
    val macReqSrcB     = Input(Vec(meshCols, UInt((tileCols*16).W)))
    // read or write row slices, control signals propagated vertically
    val rowSliceReqIn  = Input(Valid(new SliceCtrls(meshRows*tileRows)))
    val rowSliceWdata  = Input(UInt((meshCols*tileCols*64).W))
    val rowSliceReqOut = Output(Valid(new SliceCtrls(meshRows*tileRows)))
    val rowSliceRdata  = Output(UInt((meshCols*tileCols*64).W))
    // read or write col slice, control signals propagated horizontally
    val colSliceReqIn  = Input(Valid(new SliceCtrls(meshCols*tileCols*2)))
    val colSliceWdata  = Input(UInt((meshRows*tileRows*32).W))
    val colSliceReqOut = Output(Valid(new SliceCtrls(meshCols*tileCols*2)))
    val colSliceRdata  = Output(UInt((meshRows*tileRows*32).W))
  })

  val mesh  = Seq.tabulate(meshRows, meshCols)((i, j) => Module(new Tile(meshRows*tileRows, meshCols*tileCols*2, i*tileRows, j*tileCols*2, tileRows, tileCols)))
  val meshT = mesh.transpose

  // propagate horizontally across the mesh
  for(r <- 0 until meshRows) {
    mesh(r).foldLeft(io.macReqSrcA(r)) {
      case(macReqSrcA, tile) => {
        tile.io.macReqSrcA := RegNext(macReqSrcA)
        tile.io.macOutSrcA
      }
    }
    mesh(r).foldLeft(io.colSliceReqIn) {
      case(colSliceReq, tile) => {
        tile.io.colSliceReqIn := RegNext(colSliceReq)
        tile.io.colSliceReqOut
      }
    }
    mesh(r).foldLeft(io.colSliceWdata(32*tileRows*(r+1)-1, 32*tileRows*r)) {
      case(colSliceWdata, tile) => {
        tile.io.colSliceWdata := RegNext(colSliceWdata)
        tile.io.colSliceRdata
      }
    }
  }

  // propagate vertically across the mesh
  for(c <- 0 until meshCols) {
    meshT(c).foldLeft(io.macReqIn(c)) {
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
    meshT(c).foldLeft(io.rowSliceReqIn) {
      case(rowSliceReq, tile) => {
        tile.io.rowSliceReqIn := RegNext(rowSliceReq)
        tile.io.rowSliceReqOut
      }
    }
    meshT(c).foldLeft(io.rowSliceWdata(64*tileCols*(c+1)-1, 64*tileCols*c)) {
      case(rowSliceWdata, tile) => {
        tile.io.rowSliceWdata := RegNext(rowSliceWdata)
        tile.io.rowSliceRdata
      }
    }
  }

  // bottom IOs
  io.rowSliceReqOut := RegNext(mesh(meshRows-1)(meshCols-1).io.rowSliceReqOut)
  io.colSliceReqOut := RegNext(mesh(meshRows-1)(meshCols-1).io.colSliceReqOut)
  val colSliceRdataMux = WireInit(VecInit(Seq.fill(meshRows)(0.U((tileRows*32).W))))
  for(r <- 0 until meshRows) {
    // use RegEnable to lock valid data
    colSliceRdataMux(r) := RegEnable(mesh(r)(meshCols-1).io.colSliceRdata, mesh(r)(meshCols-1).io.colSliceReqOut.valid)
  }

  val rowSliceRdataMux = WireInit(VecInit(Seq.fill(meshCols)(0.U((tileCols*64).W))))
  for(c <- 0 until meshCols) {
    rowSliceRdataMux(c) := RegEnable(mesh(meshRows-1)(c).io.rowSliceRdata, mesh(meshRows-1)(c).io.rowSliceReqOut.valid)
  }
  io.colSliceRdata := colSliceRdataMux.asUInt
  io.rowSliceRdata := rowSliceRdataMux.asUInt
}

// instantiate MESH with Delays
class MXU(
  val meshRows:    Int, 
  val meshCols:    Int, 
  val tileRows:    Int, 
  val tileCols:    Int, 
  val dataWidth:   Int,
  val numAccTiles: Int = 2
) extends Module 
{
  val io = IO(new Bundle{
    // matrix-multiply related
    val macReq        = Flipped(Decoupled(new MacCtrls()))
    val macReqSrcA    = Input(UInt(dataWidth.W))
    val macReqSrcB    = Input(UInt(dataWidth.W))
    // read or write row slices
    val rowSliceReq   = Flipped(Decoupled(new SliceCtrls(meshRows*tileRows)))
    val rowSliceWdata = Input(UInt(dataWidth.W))
    val rowSliceRdata = Output(Valid(UInt(dataWidth.W)))
    // read or write col slices
    val colSliceReq   = Flipped(Decoupled(new SliceCtrls(meshCols*tileCols*2)))
    val colSliceWdata = Input(UInt(dataWidth.W))
    val colSliceRdata = Output(Valid(UInt(dataWidth.W)))
  })

  require(dataWidth >= meshRows*tileRows*16)
  require(dataWidth >= meshCols*tileCols*16)

  val macReqFire = io.macReq.valid & io.macReq.ready

  val sliceReady :: sliceWait :: Nil = Enum(2)
  val rowSliceState = RegInit(sliceReady)
  val colSliceState = RegInit(sliceReady)
  val rowVsCount    = RegInit(0.U(3.W))
  val colVsCount    = RegInit(0.U(3.W))
  val rowRespCount  = RegInit(0.U(2.W))
  val colRespCount  = RegInit(0.U(2.W))
  val rowSliceReqValid  = RegInit(false.B)
  val colSliceReqValid  = RegInit(false.B)
  val rowSliceWdataAggr = RegInit(VecInit(Seq.fill(meshCols*tileCols*2)(0.U(32.W))))
  val colSliceWdataAggr = RegInit(VecInit(Seq.fill(meshRows*tileRows)(0.U(32.W))))

  val rowSliceCtrls = io.rowSliceReq.bits
  rowSliceReqValid := false.B
  switch(rowSliceState) {
    is(sliceReady) {
      when(io.rowSliceReq.valid && rowSliceCtrls.cmd === SLICE_WRITE) {
        when(rowSliceCtrls.rtype === INT8TYPE) {
          (0 until meshCols*tileCols*2).foreach(i => rowSliceWdataAggr(i) := Cat(0.U(24.W), io.rowSliceWdata(8*i+7, 8*i)))
          rowSliceReqValid := true.B
        } .elsewhen(rowSliceCtrls.rtype === INT16TYPE) {
          when(rowVsCount === 0.U) {
            (0 until meshCols*tileCols).foreach(i => rowSliceWdataAggr(i) := Cat(0.U(16.W), io.rowSliceWdata(16*i+15, 16*i)))
            rowVsCount := rowVsCount + 1.U
          } .otherwise {
            (0 until meshCols*tileCols).foreach(i => rowSliceWdataAggr(meshCols*tileCols+i) := Cat(0.U(16.W), io.rowSliceWdata(16*i+15, 16*i)))
            rowVsCount := 0.U
            rowSliceReqValid := true.B
          }
        } .elsewhen(rowSliceCtrls.rtype === INT32TYPE) {
          when(rowVsCount === 0.U) {
            (0 until meshCols*tileCols/2).foreach(i => rowSliceWdataAggr(i) := io.rowSliceWdata(32*i+31, 32*i))
            rowVsCount := rowVsCount + 1.U
          } .elsewhen(rowVsCount === 1.U) {
            (0 until meshCols*tileCols/2).foreach(i => rowSliceWdataAggr(meshCols*tileCols/2+i) := io.rowSliceWdata(32*i+31, 32*i))
            rowVsCount := rowVsCount + 1.U
          } .elsewhen(rowVsCount === 2.U) {
            (0 until meshCols*tileCols/2).foreach(i => rowSliceWdataAggr(meshCols*tileCols+i) := io.rowSliceWdata(32*i+31, 32*i))
            rowVsCount := rowVsCount + 1.U
          } .otherwise {
            (0 until meshCols*tileCols/2).foreach(i => rowSliceWdataAggr(3*meshCols*tileCols/2+i) := io.rowSliceWdata(32*i+31, 32*i))
            rowVsCount := 0.U
            rowSliceReqValid := true.B
          }
        } .elsewhen(rowSliceCtrls.rtype === FP16TYPE) {
          (0 until meshCols*tileCols).foreach(i => rowSliceWdataAggr(2*i)   := Cat(0.U(16.W), io.rowSliceWdata(16*i+15, 16*i)))
          (0 until meshCols*tileCols).foreach(i => rowSliceWdataAggr(2*i+1) := 0.U(32.W))
          rowSliceReqValid := true.B
        } .otherwise {
          when(rowVsCount === 0.U) {
            (0 until meshCols*tileCols/2).foreach(i => rowSliceWdataAggr(2*i)   := io.rowSliceWdata(32*i+31, 32*i))
            (0 until meshCols*tileCols/2).foreach(i => rowSliceWdataAggr(2*i+1) := 0.U(32.W))
            rowVsCount := rowVsCount + 1.U
          } .otherwise {
            (0 until meshCols*tileCols/2).foreach(i => rowSliceWdataAggr(meshCols*tileCols+2*i)   := Cat(0.U(32.W), io.rowSliceWdata(32*i+31, 32*i)))
            (0 until meshCols*tileCols/2).foreach(i => rowSliceWdataAggr(meshCols*tileCols+2*i+1) := 0.U(32.W))
            rowVsCount := 0.U
            rowSliceReqValid := true.B
          }
          assert(rowSliceCtrls.rtype === FP32TYPE, "INT64 and FP64 not supported\n")
        }
      } .elsewhen(io.rowSliceReq.valid && rowSliceCtrls.cmd === SLICE_READ) {
        when(rowSliceCtrls.rtype =/= INT8TYPE && rowSliceCtrls.rtype =/= FP16TYPE) {
          rowSliceState := sliceWait
          rowVsCount    := Mux(rowSliceCtrls.rtype === INT32TYPE, 4.U, 2.U)
          assert(rowSliceCtrls.rtype =/= INT64TYPE && rowSliceCtrls.rtype =/= FP64TYPE, "INT64 and FP64 not supported\n")
        }
        rowSliceReqValid := true.B
      } .elsewhen(io.rowSliceReq.valid) {
        rowSliceReqValid := true.B
        assert(rowSliceCtrls.cmd === SLICE_CLEAR, "row slice request supports: write, read and clear\n")
      }
    }
    is(sliceWait) {
      when(rowVsCount === 1.U) {
        rowSliceState := sliceReady
        rowVsCount    := 0.U
      } .otherwise {
        rowVsCount := rowVsCount - 1.U
      }
    }
  }

  val colSliceCtrls = io.colSliceReq.bits
  colSliceReqValid := false.B
  switch(colSliceState) {
    is(sliceReady) {
      when(io.colSliceReq.valid && colSliceCtrls.cmd === SLICE_WRITE) {
        when(colSliceCtrls.rtype === INT8TYPE) {
          (0 until meshRows*tileRows).foreach(i => colSliceWdataAggr(i) := Cat(0.U(24.W), io.colSliceWdata(8*i+7, 8*i)))
          colSliceReqValid := true.B
        } .elsewhen(colSliceCtrls.rtype === INT16TYPE || colSliceCtrls.rtype === FP16TYPE) {
          (0 until meshRows*tileRows).foreach(i => colSliceWdataAggr(i) := Cat(0.U(16.W), io.colSliceWdata(16*i+15, 16*i)))
          colSliceReqValid := true.B
        } .otherwise {
          when(colVsCount === 0.U) {
            (0 until meshRows*tileRows/2).foreach(i => colSliceWdataAggr(i) := io.colSliceWdata(32*i+31, 32*i))
            colVsCount := colVsCount + 1.U
          } .otherwise {
            (0 until meshRows*tileRows/2).foreach(i => colSliceWdataAggr(meshRows*tileRows/2+i) := io.colSliceWdata(32*i+31, 32*i))
            colVsCount := 0.U
            colSliceReqValid := true.B
          }
          assert(colSliceCtrls.rtype === INT32TYPE || colSliceCtrls.rtype === FP32TYPE, "INT64 and FP64 not supported\n")
        }
      } .elsewhen(io.colSliceReq.valid) {
        when(colSliceCtrls.rtype === INT32TYPE || colSliceCtrls.rtype === FP32TYPE) {
          colSliceState := sliceWait
          colVsCount    := 2.U
        }
        assert(colSliceCtrls.cmd === SLICE_READ, "col slice request cmd supports only read and write\n")
        colSliceReqValid := true.B
      }
    }
    is(sliceWait) {
      when(colVsCount === 1.U) {
        colSliceState := sliceReady
        colVsCount    := 0.U
      } .otherwise {
        colVsCount := colVsCount - 1.U
      }
    }
  }

  val mesh = Module(new Mesh(meshRows, meshCols, tileRows, tileCols))
  mesh.io.rowSliceReqIn.valid := rowSliceReqValid
  mesh.io.rowSliceReqIn.bits  := RegNext(io.rowSliceReq.bits)
  mesh.io.rowSliceWdata       := rowSliceWdataAggr.asUInt
  mesh.io.colSliceReqIn.valid := colSliceReqValid
  mesh.io.colSliceReqIn.bits  := RegNext(io.colSliceReq.bits)
  mesh.io.colSliceWdata       := colSliceWdataAggr.asUInt
  for(c <- 0 until meshCols) {
    mesh.io.macReqIn(c).valid := ShiftRegister(macReqFire, c, true.B)
    mesh.io.macReqIn(c).bits  := ShiftRegister(io.macReq.bits, c, true.B)
    mesh.io.macReqSrcB(c)     := ShiftRegister(io.macReqSrcB(16*tileCols*(c+1)-1, 16*tileCols*c), c, true.B)
  }

  val widenSrcA = WireInit(VecInit(Seq.fill(meshRows*tileRows)(0.U(16.W))))
  (0 until meshRows*tileRows).foreach(i => widenSrcA(i) := io.macReqSrcA(8*i+7, 8*i))
  val muxSrcA = Mux(io.macReq.bits.srcType(2), io.macReqSrcA, widenSrcA.asUInt)
  for(r <- 0 until meshRows) {
    mesh.io.macReqSrcA(r) := ShiftRegister(muxSrcA(16*tileRows*(r+1)-1, 16*tileRows*r), r, true.B)
  }

  // ready control
  io.macReq.ready        := true.B
  io.rowSliceReq.ready   := (rowSliceState === sliceReady)
  io.colSliceReq.ready   := (colSliceState === sliceReady)
  // output control
  val rowRespCtrls     = mesh.io.rowSliceReqOut.bits
  val rowSliceRdataMux = WireInit(VecInit(Seq.fill(meshCols*tileCols*2)(0.U(8.W))))
  when(mesh.io.rowSliceReqOut.valid && rowRespCtrls.cmd === SLICE_READ) {
    when(rowRespCtrls.rtype === INT8TYPE) {
      (0 until meshCols*tileCols*2).foreach(i => rowSliceRdataMux(i)     := mesh.io.rowSliceRdata(32*i+7, 32*i))
    } .elsewhen(rowRespCtrls.rtype === INT16TYPE) {
      (0 until meshCols*tileCols).foreach(i => rowSliceRdataMux(2*i)     := mesh.io.rowSliceRdata(32*i+7,  32*i))
      (0 until meshCols*tileCols).foreach(i => rowSliceRdataMux(2*i+1)   := mesh.io.rowSliceRdata(32*i+15, 32*i+8))
      rowRespCount := 1.U
    } .elsewhen(rowRespCtrls.rtype === INT32TYPE) {
      (0 until meshCols*tileCols/2).foreach(i => rowSliceRdataMux(4*i)   := mesh.io.rowSliceRdata(32*i+7,  32*i))
      (0 until meshCols*tileCols/2).foreach(i => rowSliceRdataMux(4*i+1) := mesh.io.rowSliceRdata(32*i+15, 32*i+8))
      (0 until meshCols*tileCols/2).foreach(i => rowSliceRdataMux(4*i+2) := mesh.io.rowSliceRdata(32*i+23, 32*i+16))
      (0 until meshCols*tileCols/2).foreach(i => rowSliceRdataMux(4*i+3) := mesh.io.rowSliceRdata(32*i+31, 32*i+24))
      rowRespCount := 3.U
    } .elsewhen(rowRespCtrls.rtype === FP16TYPE) {
      (0 until meshCols*tileCols).foreach(i => rowSliceRdataMux(2*i)     := mesh.io.rowSliceRdata(64*i+7,  64*i))
      (0 until meshCols*tileCols).foreach(i => rowSliceRdataMux(2*i+1)   := mesh.io.rowSliceRdata(64*i+15, 64*i+8))
    } .otherwise {
      (0 until meshCols*tileCols/2).foreach(i => rowSliceRdataMux(4*i)   := mesh.io.rowSliceRdata(64*i+7,  64*i))
      (0 until meshCols*tileCols/2).foreach(i => rowSliceRdataMux(4*i+1) := mesh.io.rowSliceRdata(64*i+15, 64*i+8))
      (0 until meshCols*tileCols/2).foreach(i => rowSliceRdataMux(4*i+2) := mesh.io.rowSliceRdata(64*i+23, 64*i+16))
      (0 until meshCols*tileCols/2).foreach(i => rowSliceRdataMux(4*i+3) := mesh.io.rowSliceRdata(64*i+31, 64*i+24))
      rowRespCount := 1.U
    }
  } .elsewhen(rowRespCount > 0.U) {
    when(rowRespCount === 3.U) {
      (0 until meshCols*tileCols/2).foreach(i => rowSliceRdataMux(4*i)   := mesh.io.rowSliceRdata(16*meshCols*tileCols+32*i+7,  16*meshCols*tileCols+32*i))
      (0 until meshCols*tileCols/2).foreach(i => rowSliceRdataMux(4*i+1) := mesh.io.rowSliceRdata(16*meshCols*tileCols+32*i+15, 16*meshCols*tileCols+32*i+8))
      (0 until meshCols*tileCols/2).foreach(i => rowSliceRdataMux(4*i+2) := mesh.io.rowSliceRdata(16*meshCols*tileCols+32*i+23, 16*meshCols*tileCols+32*i+16))
      (0 until meshCols*tileCols/2).foreach(i => rowSliceRdataMux(4*i+3) := mesh.io.rowSliceRdata(16*meshCols*tileCols+32*i+31, 16*meshCols*tileCols+32*i+24))
    } .elsewhen(rowRespCount === 2.U) {
      (0 until meshCols*tileCols/2).foreach(i => rowSliceRdataMux(4*i)   := mesh.io.rowSliceRdata(32*meshCols*tileCols+32*i+7,  32*meshCols*tileCols+32*i))
      (0 until meshCols*tileCols/2).foreach(i => rowSliceRdataMux(4*i+1) := mesh.io.rowSliceRdata(32*meshCols*tileCols+32*i+15, 32*meshCols*tileCols+32*i+8))
      (0 until meshCols*tileCols/2).foreach(i => rowSliceRdataMux(4*i+2) := mesh.io.rowSliceRdata(32*meshCols*tileCols+32*i+23, 32*meshCols*tileCols+32*i+16))
      (0 until meshCols*tileCols/2).foreach(i => rowSliceRdataMux(4*i+3) := mesh.io.rowSliceRdata(32*meshCols*tileCols+32*i+31, 32*meshCols*tileCols+32*i+24))
    } .elsewhen(rowRespCount === 1.U && rowRespCtrls.rtype === INT32TYPE) {
      (0 until meshCols*tileCols/2).foreach(i => rowSliceRdataMux(4*i)   := mesh.io.rowSliceRdata(48*meshCols*tileCols+32*i+7,  48*meshCols*tileCols+32*i))
      (0 until meshCols*tileCols/2).foreach(i => rowSliceRdataMux(4*i+1) := mesh.io.rowSliceRdata(48*meshCols*tileCols+32*i+15, 48*meshCols*tileCols+32*i+8))
      (0 until meshCols*tileCols/2).foreach(i => rowSliceRdataMux(4*i+2) := mesh.io.rowSliceRdata(48*meshCols*tileCols+32*i+23, 48*meshCols*tileCols+32*i+16))
      (0 until meshCols*tileCols/2).foreach(i => rowSliceRdataMux(4*i+3) := mesh.io.rowSliceRdata(48*meshCols*tileCols+32*i+31, 48*meshCols*tileCols+32*i+24))
    } .elsewhen(rowRespCount === 1.U && rowRespCtrls.rtype === INT16TYPE) {
      (0 until meshCols*tileCols).foreach(i => rowSliceRdataMux(2*i)     := mesh.io.rowSliceRdata(32*meshCols*tileCols+32*i+7,  32*meshCols*tileCols+32*i))
      (0 until meshCols*tileCols).foreach(i => rowSliceRdataMux(2*i+1)   := mesh.io.rowSliceRdata(32*meshCols*tileCols+32*i+15, 32*meshCols*tileCols+32*i+8))
    } .otherwise {
      (0 until meshCols*tileCols/2).foreach(i => rowSliceRdataMux(4*i)   := mesh.io.rowSliceRdata(32*meshCols*tileCols+64*i+7,  32*meshCols*tileCols+64*i))
      (0 until meshCols*tileCols/2).foreach(i => rowSliceRdataMux(4*i+1) := mesh.io.rowSliceRdata(32*meshCols*tileCols+64*i+15, 32*meshCols*tileCols+64*i+8))
      (0 until meshCols*tileCols/2).foreach(i => rowSliceRdataMux(4*i+2) := mesh.io.rowSliceRdata(32*meshCols*tileCols+64*i+23, 32*meshCols*tileCols+64*i+16))
      (0 until meshCols*tileCols/2).foreach(i => rowSliceRdataMux(4*i+3) := mesh.io.rowSliceRdata(32*meshCols*tileCols+64*i+31, 32*meshCols*tileCols+64*i+24))
    }
    rowRespCount := rowRespCount - 1.U
  }

  val colRespCtrls     = mesh.io.colSliceReqOut.bits
  val colSliceRdataMux = WireInit(VecInit(Seq.fill(meshRows*tileRows*2)(0.U(8.W))))
  when(mesh.io.colSliceReqOut.valid && colRespCtrls.cmd === SLICE_READ) {
    when(colRespCtrls.rtype === INT8TYPE) {
      (0 until meshRows*tileRows).foreach(i => colSliceRdataMux(i)       := mesh.io.colSliceRdata(32*i+7, 32*i))
    } .elsewhen(colRespCtrls.rtype === INT16TYPE || colRespCtrls.rtype === FP16TYPE) {
      (0 until meshRows*tileRows).foreach(i => colSliceRdataMux(2*i)     := mesh.io.colSliceRdata(32*i+7,  32*i))
      (0 until meshRows*tileRows).foreach(i => colSliceRdataMux(2*i+1)   := mesh.io.colSliceRdata(32*i+15, 32*i+8))
    } .otherwise{
      (0 until meshRows*tileRows/2).foreach(i => colSliceRdataMux(4*i)   := mesh.io.colSliceRdata(32*i+7,  32*i))
      (0 until meshRows*tileRows/2).foreach(i => colSliceRdataMux(4*i+1) := mesh.io.colSliceRdata(32*i+15, 32*i+8))
      (0 until meshRows*tileRows/2).foreach(i => colSliceRdataMux(4*i+2) := mesh.io.colSliceRdata(32*i+23, 32*i+16))
      (0 until meshRows*tileRows/2).foreach(i => colSliceRdataMux(4*i+3) := mesh.io.colSliceRdata(32*i+31, 32*i+24))
      colRespCount := 1.U
    }
  } .elsewhen(colRespCount > 0.U) {
    (0 until meshRows*tileRows/2).foreach(i => colSliceRdataMux(4*i)     := mesh.io.colSliceRdata(16*meshRows*tileRows+32*i+7,  16*meshRows*tileRows+32*i))
    (0 until meshRows*tileRows/2).foreach(i => colSliceRdataMux(4*i+1)   := mesh.io.colSliceRdata(16*meshRows*tileRows+32*i+15, 16*meshRows*tileRows+32*i+8))
    (0 until meshRows*tileRows/2).foreach(i => colSliceRdataMux(4*i+2)   := mesh.io.colSliceRdata(16*meshRows*tileRows+32*i+23, 16*meshRows*tileRows+32*i+16))
    (0 until meshRows*tileRows/2).foreach(i => colSliceRdataMux(4*i+3)   := mesh.io.colSliceRdata(16*meshRows*tileRows+32*i+31, 16*meshRows*tileRows+32*i+24))
    colRespCount := 0.U
  }
  
  io.rowSliceRdata.valid := (mesh.io.rowSliceReqOut.valid && mesh.io.rowSliceReqOut.bits.cmd === SLICE_READ) || rowRespCount > 0.U
  io.rowSliceRdata.bits  := rowSliceRdataMux.asUInt
  
  io.colSliceRdata.valid := (mesh.io.colSliceReqOut.valid && mesh.io.colSliceReqOut.bits.cmd === SLICE_READ) || colRespCount > 0.U
  io.colSliceRdata.bits  := colSliceRdataMux.asUInt
}

class MXUWrapper(
  val meshRows:    Int, 
  val meshCols:    Int, 
  val tileRows:    Int, 
  val tileCols:    Int, 
  val dataWidth:   Int
) extends Module
{
  val io = IO(new Bundle {
    // matrix-multiply related
    val macReq        = Flipped(Decoupled(new MacCtrls()))
    val macReqSrcA    = Input(Vec(meshRows*tileRows, UInt(16.W)))
    val macReqSrcB    = Input(Vec(meshCols*tileCols, UInt(16.W)))
    // read or write row slices
    val rowSliceReq   = Flipped(Decoupled(new SliceCtrls(meshRows*tileRows)))
    val rowSliceWdata = Input(Vec(meshCols*tileCols, UInt(16.W)))
    val rowSliceRdata = Output(Valid(Vec(meshCols*tileCols, UInt(16.W))))
    // read or write col slices
    val colSliceReq   = Flipped(Decoupled(new SliceCtrls(meshCols*tileCols*2)))
    val colSliceWdata = Input(Vec(meshRows*tileRows, UInt(16.W)))
    val colSliceRdata = Output(Valid(Vec(meshRows*tileRows, UInt(16.W))))
  })

  val mxu = Module(new MXU(meshRows, meshCols, tileRows, tileCols, dataWidth))
  mxu.io.macReq.valid      := io.macReq.valid
  mxu.io.macReq.bits       := io.macReq.bits
  mxu.io.macReqSrcA        := io.macReqSrcA.asUInt
  mxu.io.macReqSrcB        := io.macReqSrcB.asUInt
  mxu.io.rowSliceReq.valid := io.rowSliceReq.valid
  mxu.io.rowSliceReq.bits  := io.rowSliceReq.bits
  mxu.io.rowSliceWdata     := io.rowSliceWdata.asUInt
  mxu.io.colSliceReq.valid := io.colSliceReq.valid
  mxu.io.colSliceReq.bits  := io.colSliceReq.bits
  mxu.io.colSliceWdata     := io.colSliceWdata.asUInt

  io.macReq.ready      := mxu.io.macReq.ready
  io.rowSliceReq.ready := mxu.io.rowSliceReq.ready
  io.colSliceReq.ready := mxu.io.colSliceReq.ready

  io.rowSliceRdata.valid := mxu.io.rowSliceRdata.valid
  for(i <- 0 until meshCols*tileCols) {
    io.rowSliceRdata.bits(i) := mxu.io.rowSliceRdata.bits(16*i+15, 16*i)
  }

  io.colSliceRdata.valid := mxu.io.colSliceRdata.valid
  for(i <- 0 until meshRows*tileRows) {
    io.colSliceRdata.bits(i) := mxu.io.colSliceRdata.bits(16*i+15, 16*i)
  }
}