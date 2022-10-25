//******************************************************************************
// Copyright (c) 2012 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// RISCV Processor Register Read
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.exu

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.Parameters
import freechips.rocketchip.util._
import freechips.rocketchip.rocket.{VConfig}

import FUConstants._
import boom.common._
import boom.common.MicroOpcodes._
import boom.util._

/**
 * Handle the register read and bypass network for the OoO backend
 * interfaces with the issue window on the enqueue side, and the execution
 * pipelines on the dequeue side.
 *
 * @param issueWidth total issue width from all issue queues
 * @param supportedUnitsArray seq of SupportedFuncUnits classes indicating what the functional units do
 * @param numTotalReadPorts number of read ports
 * @param numReadPortsArray execution units read port sequence
 * @param registerWidth size of register in bits
 */
class TileRegisterRead(
  issueWidth: Int,
  supportedUnitsArray: Seq[SupportedFuncUnits],
  numTotalReadPorts: Int,
  numReadPortsArray: Seq[Int],
                        // each exe_unit must tell us how many max
                        // operands it can accept (the sum should equal
                        // numTotalReadPorts)
  registerWidth: Int
)(implicit p: Parameters) extends BoomModule with freechips.rocketchip.tile.HasFPUParameters
{
  val io = IO(new Bundle {
    // issued micro-ops
    val iss_valids = Input(Vec(issueWidth, Bool()))
    val iss_uops   = Input(Vec(issueWidth, new MicroOp()))

    // interface with register file's read ports
    val tileReadPorts = Flipped(Vec(numTotalReadPorts, new TrTileRegReadPortIO()))

    // send micro-ops to the execution pipelines
    val exe_reqs = Vec(issueWidth, (new DecoupledIO(new FuncUnitReq(registerWidth))))

    val kill     = Input(Bool())
    val brupdate = Input(new BrUpdateInfo())
  })

  val rrd_valids       = Wire(Vec(issueWidth, Bool()))
  val rrd_uops         = Wire(Vec(issueWidth, new MicroOp()))

  val exe_reg_valids   = RegInit(VecInit(Seq.fill(issueWidth) { false.B }))
  val exe_reg_uops     = Reg(Vec(issueWidth, new MicroOp()))
  val exe_reg_ts1_data = Reg(Vec(issueWidth, Bits(registerWidth.W)))
  val exe_reg_ts2_data = Reg(Vec(issueWidth, Bits(registerWidth.W)))

  //-------------------------------------------------------------
  // read ports

  require (numTotalReadPorts == numReadPortsArray.reduce(_+_))

  val rrd_ts1_data = Wire(Vec(issueWidth, Bits(registerWidth.W)))
  val rrd_ts2_data = Wire(Vec(issueWidth, Bits(registerWidth.W)))

  var idx = 0 // index into flattened read_ports array
  for (w <- 0 until issueWidth) {
    //-------------------------------------------------------------
    // hook up inputs
    val rrd_decode_unit = Module(new RegisterReadDecode(supportedUnitsArray(w)))
    rrd_decode_unit.io.iss_valid := io.iss_valids(w)
    rrd_decode_unit.io.iss_uop   := io.iss_uops(w)

    rrd_valids(w) := RegNext(rrd_decode_unit.io.rrd_valid &&
                             !IsKilledByBranch(io.brupdate, rrd_decode_unit.io.rrd_uop))
    rrd_uops(w)   := RegNext(GetNewUopAndBrMask(rrd_decode_unit.io.rrd_uop, io.brupdate))

    val numReadPorts = numReadPortsArray(w)

    // port 0
    io.tileReadPorts(idx+0).msew      := io.iss_uops(w).ts1_eew
    io.tileReadPorts(idx+0).tt        := Mux(io.iss_uops(w).uopc.isOneOf(uopMMV_V) && io.iss_uops(w).isHSlice, 2.U, 3.U)
    io.tileReadPorts(idx+0).addr      := io.iss_uops(w).prs1
    io.tileReadPorts(idx+0).index     := io.iss_uops(w).m_sidx
    // port 1
    io.tileReadPorts(idx+1).msew      := io.iss_uops(w).ts2_eew
    io.tileReadPorts(idx+1).tt        := 2.U                          // tr_r, used in mopa instructions only
    io.tileReadPorts(idx+1).addr      := io.iss_uops(w).prs2
    io.tileReadPorts(idx+1).index     := io.iss_uops(w).m_sidx

    rrd_ts1_data(w) := io.tileReadPorts(idx+0).data
    rrd_ts2_data(w) := io.tileReadPorts(idx+1).data

    val rrd_kill = io.kill || IsKilledByBranch(io.brupdate, rrd_uops(w))

    // Execute Stage
    exe_reg_valids(w) := Mux(rrd_kill, false.B, rrd_valids(w))
    exe_reg_uops(w)   := Mux(rrd_kill, NullMicroOp(), rrd_uops(w))
    exe_reg_uops(w).br_mask := GetNewBrMask(io.brupdate, rrd_uops(w))
    exe_reg_ts1_data(w)  := rrd_ts1_data(w)
    exe_reg_ts2_data(w)  := rrd_ts2_data(w)

    idx += numReadPorts
  }

  //-------------------------------------------------------------
  // set outputs to execute pipelines
  for (w <- 0 until issueWidth) {
    val exe_uop = exe_reg_uops(w)
    io.exe_reqs(w).valid          := exe_reg_valids(w)
    io.exe_reqs(w).bits.uop       := exe_reg_uops(w)
    io.exe_reqs(w).bits.rs1_data  := exe_reg_ts1_data(w)
    io.exe_reqs(w).bits.rs2_data  := Mux(exe_uop.rt(RS2, isInt), Mux1H(UIntToOH(exe_uop.ts2_eew), 
                                                                       Seq(Fill(rLenb,   exe_uop.m_scalar_data( 7, 0)),
                                                                           Fill(rLenb/2, exe_uop.m_scalar_data(15, 0)),
                                                                           Fill(rLenb/4, exe_uop.m_scalar_data(31, 0)), 0.U)),
                                                                 exe_reg_ts2_data(w))
    io.exe_reqs(w).bits.rvm_data  := DontCare
    io.exe_reqs(w).bits.pred_data := DontCare
  }
}
