//******************************************************************************
// Copyright (c) 2013 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Execution Units
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// The issue window schedules micro-ops onto a specific execution pipeline
// A given execution pipeline may contain multiple functional units; one or more
// read ports, and one or more writeports.

package boom.exu

import scala.collection.mutable.{ArrayBuffer}

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.{Parameters}
import freechips.rocketchip.rocket.{BP, VConfig}
import freechips.rocketchip.tile.{XLen, RoCCCoreIO}
import freechips.rocketchip.tile

import FUConstants._
import boom.common._
import boom.common.MicroOpcodes._
import boom.ifu.{GetPCFromFtqIO}
import boom.util.{ImmGen, IsKilledByBranch, BranchKillableQueue, BoomCoreStringPrefix, VDataSwap}

/**
 * Response from Execution Unit. Bundles a MicroOp with data
 *
 * @param dataWidth width of the data coming from the execution unit
 */
class ExeUnitResp(val dataWidth: Int)(implicit p: Parameters) extends BoomBundle
  with HasBoomUOP
{
  val data = Bits(dataWidth.W)
  val predicated = Bool() // Was this predicated off?
  val fflags = new ValidIO(new FFlagsResp) // write fflags to ROB // TODO: Do this better
}

/**
 * Floating Point flag response
 */
class FFlagsResp(implicit p: Parameters) extends BoomBundle
{
  val uop = new MicroOp()
  val flags = Bits(tile.FPConstants.FLAGS_SZ.W)
}


/**
 * Abstract Top level Execution Unit that wraps lower level functional units to make a
 * multi function execution unit.
 *
 * @param readsIrf does this exe unit need a integer regfile port
 * @param writesIrf does this exe unit need a integer regfile port
 * @param readsFrf does this exe unit need a integer regfile port
 * @param writesFrf does this exe unit need a integer regfile port
 * @param writesLlIrf does this exe unit need a integer regfile port
 * @param writesLlFrf does this exe unit need a integer regfile port
 * @param numBypassStages number of bypass ports for the exe unit
 * @param dataWidth width of the data coming out of the exe unit
 * @param bypassable is the exe unit able to be bypassed
 * @param hasMem does the exe unit have a MemAddrCalcUnit
 * @param hasCSR does the exe unit write to the CSRFile
 * @param hasBrUnit does the exe unit have a branch unit
 * @param hasAlu does the exe unit have a alu
 * @param hasFpu does the exe unit have a fpu
 * @param hasMul does the exe unit have a multiplier
 * @param hasMacc does the exe unit have a multiply-accumulator
 * @param hasVMaskUnit does the exe unit have a VMaskUnit
 * @param hasDiv does the exe unit have a divider
 * @param hasFdiv does the exe unit have a FP divider
 * @param hasIfpu does the exe unit have a int to FP unit
 * @param hasFpiu does the exe unit have a FP to int unit
 */
abstract class ExecutionUnit(
  val readsIrf         : Boolean       = false,
  val writesIrf        : Boolean       = false,
  val readsFrf         : Boolean       = false,
  val writesFrf        : Boolean       = false,
  val readsVrf         : Boolean       = false,
  val writesVrf        : Boolean       = false,
  val writesLlIrf      : Boolean       = false,
  val writesLlFrf      : Boolean       = false,
  val writesLlVrf      : Boolean       = false,
  val numBypassStages  : Int,
  val dataWidth        : Int,
  val bypassable       : Boolean       = false, // TODO make override def for code clarity
  val alwaysBypassable : Boolean       = false,
  val hasMem           : Boolean       = false,
  val hasCSR           : Boolean       = false,
  val hasJmpUnit       : Boolean       = false,
  val hasAlu           : Boolean       = false,
  val hasFpu           : Boolean       = false,
  val hasMul           : Boolean       = false,
  val hasMacc          : Boolean       = false,
  val hasVMaskUnit     : Boolean       = false,
  val hasDiv           : Boolean       = false,
  val hasFdiv          : Boolean       = false,
  val hasIfpu          : Boolean       = false,
  val hasFpiu          : Boolean       = false,
  val hasVector        : Boolean       = false,
  val hasVMX           : Boolean       = false,
  val hasRocc          : Boolean       = false
  )(implicit p: Parameters) extends BoomModule
{

  val io = IO(new Bundle {
    val fu_types = Output(Bits(FUC_SZ.W))

    val req      = Flipped(new DecoupledIO(new FuncUnitReq(dataWidth)))

    val iresp    = if (writesIrf)   new DecoupledIO(new ExeUnitResp(dataWidth)) else null
    val fresp    = if (writesFrf)   new DecoupledIO(new ExeUnitResp(dataWidth)) else null
    val vresp    = if (writesVrf)   new DecoupledIO(new ExeUnitResp(dataWidth)) else null
    val ll_iresp = if (writesLlIrf) new DecoupledIO(new ExeUnitResp(dataWidth)) else null
    val ll_fresp = if (writesLlFrf) new DecoupledIO(new ExeUnitResp(dataWidth)) else null
    val ll_vresp = if (writesLlVrf) new DecoupledIO(new ExeUnitResp(dataWidth)) else null


    val bypass   = Output(Vec(numBypassStages, Valid(new ExeUnitResp(dataWidth))))
    val brupdate = Input(new BrUpdateInfo())


    // only used by the rocc unit
    val rocc = if (hasRocc) new RoCCShimCoreIO else null

    // only used by the branch unit
    val brinfo     = if (hasAlu && !hasVector) Output(new BrResolutionInfo()) else null
    val get_ftq_pc = if (hasJmpUnit) Flipped(new GetPCFromFtqIO()) else null
    val status     = Input(new freechips.rocketchip.rocket.MStatus())

    // only used by the fpu unit
    val fcsr_rm = if (hasFcsr) Input(Bits(tile.FPConstants.RM_SZ.W)) else null
    val vconfig = if (hasVConfig) Input(new VConfig) else null

    // only used by the mem unit
    val lsu_io = if (hasMem) Flipped(new boom.lsu.LSUExeIO) else null
    val bp = if (hasMem) Input(Vec(nBreakpoints, new BP)) else null
    val mcontext = if (hasMem) Input(UInt(coreParams.mcontextWidth.W)) else null
    val scontext = if (hasMem) Input(UInt(coreParams.scontextWidth.W)) else null

    // TODO move this out of ExecutionUnit
    val com_exception = if (hasMem || hasRocc) Input(Bool()) else null
  })

  if (writesIrf)   {
    io.iresp.bits.fflags.valid := false.B
    io.iresp.bits.predicated := false.B
    //assert(io.iresp.ready)
  }
  if (writesLlIrf) {
    io.ll_iresp.bits.fflags.valid := false.B
    io.ll_iresp.bits.predicated := false.B
  }
  if (writesFrf)   {
    io.fresp.bits.fflags.valid := false.B
    io.fresp.bits.predicated := false.B
    //assert(io.fresp.ready)
  }
  if (writesLlFrf) {
    io.ll_fresp.bits.fflags.valid := false.B
    io.ll_fresp.bits.predicated := false.B
  }
  if (writesVrf)   {
    io.vresp.bits.fflags.valid := false.B
    io.vresp.bits.predicated := false.B
//  assert(io.vresp.ready)
  }
  if (writesLlVrf) {
    io.ll_vresp.bits.fflags.valid := false.B
    io.ll_vresp.bits.predicated := false.B
  }

  // TODO add "number of fflag ports", so we can properly account for FPU+Mem combinations
  def hasFFlags     : Boolean = hasFpu || hasFdiv

  //require ((hasFpu || hasFdiv) ^ (hasAlu || hasMul || hasMem || hasIfpu),
    //"[execute] we no longer support mixing FP and Integer functional units in the same exe unit.")
  def hasFcsr = hasIfpu || hasFpu || hasFdiv
  def hasVConfig = usingVector && hasCSR

  require (bypassable || !alwaysBypassable,
    "[execute] an execution unit must be bypassable if it is always bypassable")

  def supportedFuncUnits = {
    new SupportedFuncUnits(
      alu = hasAlu,
      jmp = hasJmpUnit,
      mem = hasMem,
      muld = hasMul || hasDiv,
      fpu = hasFpu,
      csr = hasCSR,
      fdiv = hasFdiv,
      ifpu = hasIfpu)
  }
}

/**
 * ALU execution unit that can have a branch, alu, mul, div, int to FP,
 * and memory unit.
 *
 * @param hasBrUnit does the exe unit have a branch unit
 * @param hasCSR does the exe unit write to the CSRFile
 * @param hasAlu does the exe unit have a alu
 * @param hasMul does the exe unit have a multiplier
 * @param hasDiv does the exe unit have a divider
 * @param hasIfpu does the exe unit have a int to FP unit
 * @param hasMem does the exe unit have a MemAddrCalcUnit
 */
class ALUExeUnit(
  hasJmpUnit     : Boolean = false,
  hasCSR         : Boolean = false,
  hasAlu         : Boolean = true,
  hasMul         : Boolean = false,
  hasDiv         : Boolean = false,
  hasIfpu        : Boolean = false,
  hasMem         : Boolean = false,
  hasRocc        : Boolean = false)
  (implicit p: Parameters)
  extends ExecutionUnit(
    readsIrf         = true,
    writesIrf        = hasAlu || hasMul || hasDiv,
    writesLlIrf      = hasMem || hasRocc,
    writesLlFrf      = (hasIfpu || hasMem) && p(tile.TileKey).core.fpu != None,
    writesLlVrf      = (p(tile.TileKey).core.useVector && hasMem),
    numBypassStages  =
      if (hasAlu && hasMul) 3 //TODO XXX p(tile.TileKey).core.imulLatency
      else if (hasAlu) 1 else 0,
    dataWidth        = p(tile.XLen) + 1,
    bypassable       = hasAlu,
    alwaysBypassable = hasAlu && !(hasMem || hasJmpUnit || hasMul || hasDiv || hasCSR || hasIfpu || hasRocc),
    hasCSR           = hasCSR,
    hasJmpUnit       = hasJmpUnit,
    hasAlu           = hasAlu,
    hasMul           = hasMul,
    hasDiv           = hasDiv,
    hasIfpu          = hasIfpu,
    hasMem           = hasMem,
    hasRocc          = hasRocc)
  with freechips.rocketchip.rocket.constants.MemoryOpConstants
{
  require(!(hasRocc && !hasCSR),
    "RoCC needs to be shared with CSR unit")
  require(!(hasMem && hasRocc),
    "We do not support execution unit with both Mem and Rocc writebacks")
  require(!(hasMem && hasIfpu),
    "TODO. Currently do not support AluMemExeUnit with FP")

  val out_str =
    BoomCoreStringPrefix("==ExeUnit==") +
    (if (hasAlu)  BoomCoreStringPrefix(" - ALU") else "") +
    (if (hasMul)  BoomCoreStringPrefix(" - Mul") else "") +
    (if (hasDiv)  BoomCoreStringPrefix(" - Div") else "") +
    (if (hasIfpu) BoomCoreStringPrefix(" - IFPU") else "") +
    (if (hasMem)  BoomCoreStringPrefix(" - Mem") else "") +
    (if (hasRocc) BoomCoreStringPrefix(" - RoCC") else "")

  override def toString: String = out_str.toString

  val div_busy  = WireInit(false.B)
  val ifpu_busy = WireInit(false.B)

  // The Functional Units --------------------
  // Specifically the functional units with fast writeback to IRF
  val iresp_fu_units = ArrayBuffer[FunctionalUnit]()

  io.fu_types := Mux(hasAlu.B, FU_ALU, 0.U) |
                 Mux(hasMul.B, FU_MUL, 0.U) |
                 Mux(!div_busy && hasDiv.B, FU_DIV, 0.U) |
                 Mux(hasCSR.B, FU_CSR, 0.U) |
                 Mux(hasJmpUnit.B, FU_JMP, 0.U) |
                 Mux(!ifpu_busy && hasIfpu.B, FU_I2F, 0.U) |
                 Mux(hasMem.B, FU_MEM, 0.U)

  // ALU Unit -------------------------------
  var alu: ALUUnit = null
  if (hasAlu) {
    alu = Module(new ALUUnit(isJmpUnit = hasJmpUnit,
                             isCsrUnit = hasCSR,
                             numStages = numBypassStages,
                             dataWidth = xLen))
    alu.io.req.valid := (
      io.req.valid &&
      (io.req.bits.uop.fu_code === FU_ALU ||
       io.req.bits.uop.fu_code === FU_JMP ||
      (io.req.bits.uop.fu_code === FU_CSR && io.req.bits.uop.uopc =/= uopROCC)))
    //ROCC Rocc Commands are taken by the RoCC unit

    alu.io.req.bits.uop      := io.req.bits.uop
    alu.io.req.bits.kill     := io.req.bits.kill
    alu.io.req.bits.rs1_data := io.req.bits.rs1_data
    alu.io.req.bits.rs2_data := io.req.bits.rs2_data
    alu.io.req.bits.rs3_data := DontCare
    alu.io.req.bits.pred_data := io.req.bits.pred_data
    alu.io.resp.ready := DontCare
    alu.io.brupdate := io.brupdate

    iresp_fu_units += alu

    // Bypassing only applies to ALU
    io.bypass := alu.io.bypass

    // branch unit is embedded inside the ALU
    io.brinfo := alu.io.brinfo
    if (hasJmpUnit) {
      alu.io.get_ftq_pc <> io.get_ftq_pc
    }

    if (usingVector && hasCSR) alu.io.vconfig := io.vconfig
  }

  var rocc: RoCCShim = null
  if (hasRocc) {
    rocc = Module(new RoCCShim)
    rocc.io.req.valid         := io.req.valid && io.req.bits.uop.uopc === uopROCC
    rocc.io.req.bits          := DontCare
    rocc.io.req.bits.uop      := io.req.bits.uop
    rocc.io.req.bits.kill     := io.req.bits.kill
    rocc.io.req.bits.rs1_data := io.req.bits.rs1_data
    rocc.io.req.bits.rs2_data := io.req.bits.rs2_data
    rocc.io.brupdate          := io.brupdate // We should assert on this somewhere
    rocc.io.status            := io.status
    rocc.io.exception         := io.com_exception
    io.rocc                   <> rocc.io.core

    rocc.io.resp.ready        := io.ll_iresp.ready
    io.ll_iresp.valid         := rocc.io.resp.valid
    io.ll_iresp.bits.uop      := rocc.io.resp.bits.uop
    io.ll_iresp.bits.data     := rocc.io.resp.bits.data
  }


  // Pipelined, IMul Unit ------------------
  var imul: PipelinedMulUnit = null
  if (hasMul) {
    imul = Module(new PipelinedMulUnit(imulLatency, xLen))
    imul.io <> DontCare
    imul.io.req.valid         := io.req.valid && io.req.bits.uop.fu_code_is(FU_MUL)
    imul.io.req.bits.uop      := io.req.bits.uop
    imul.io.req.bits.rs1_data := io.req.bits.rs1_data
    imul.io.req.bits.rs2_data := io.req.bits.rs2_data
    imul.io.req.bits.kill     := io.req.bits.kill
    imul.io.brupdate := io.brupdate
    iresp_fu_units += imul
  }

  var ifpu: IntToFPUnit = null
  if (hasIfpu) {
    ifpu = Module(new IntToFPUnit(latency=intToFpLatency))
    ifpu.io.req        <> io.req
    ifpu.io.req.valid  := io.req.valid && io.req.bits.uop.fu_code_is(FU_I2F)
    ifpu.io.fcsr_rm    := io.fcsr_rm
    ifpu.io.brupdate   <> io.brupdate
    ifpu.io.resp.ready := DontCare

    // buffer up results since we share write-port on integer regfile.
    val queue = Module(new BranchKillableQueue(new ExeUnitResp(dataWidth),
      entries = intToFpLatency + 3)) // TODO being overly conservative
    queue.io.enq.valid       := ifpu.io.resp.valid
    queue.io.enq.bits.uop    := ifpu.io.resp.bits.uop
    queue.io.enq.bits.data   := ifpu.io.resp.bits.data
    queue.io.enq.bits.predicated := ifpu.io.resp.bits.predicated
    queue.io.enq.bits.fflags := ifpu.io.resp.bits.fflags
    queue.io.brupdate := io.brupdate
    queue.io.flush := io.req.bits.kill

    io.ll_fresp <> queue.io.deq
    ifpu_busy := !(queue.io.empty)
    assert (queue.io.enq.ready)
  }

  // Div/Rem Unit -----------------------
  var div: DivUnit = null
  val div_resp_val = WireInit(false.B)
  if (hasDiv) {
    div = Module(new DivUnit(xLen))
    div.io <> DontCare
    div.io.req.valid           := io.req.valid && io.req.bits.uop.fu_code_is(FU_DIV) && hasDiv.B
    div.io.req.bits.uop        := io.req.bits.uop
    div.io.req.bits.rs1_data   := io.req.bits.rs1_data
    div.io.req.bits.rs2_data   := io.req.bits.rs2_data
    div.io.brupdate            := io.brupdate
    div.io.req.bits.kill       := io.req.bits.kill

    // share write port with the pipelined units
    div.io.resp.ready := !(iresp_fu_units.map(_.io.resp.valid).reduce(_|_))

    div_resp_val := div.io.resp.valid
    div_busy     := !div.io.req.ready ||
                    (io.req.valid && io.req.bits.uop.fu_code_is(FU_DIV))

    iresp_fu_units += div
  }

  // Mem Unit --------------------------
  if (hasMem) {
    require(!hasAlu)
    val maddrcalc = Module(new MemAddrCalcUnit)
    maddrcalc.io.req        <> io.req
    maddrcalc.io.req.valid  := io.req.valid && io.req.bits.uop.fu_code_is(FU_MEM)
    maddrcalc.io.brupdate     <> io.brupdate
    maddrcalc.io.status     := io.status
    maddrcalc.io.bp         := io.bp
    maddrcalc.io.mcontext   := io.mcontext
    maddrcalc.io.scontext   := io.scontext
    maddrcalc.io.resp.ready := DontCare
    require(numBypassStages == 0)

    io.lsu_io.req := maddrcalc.io.resp

    io.ll_iresp <> io.lsu_io.iresp
    if (usingFPU) {
      io.ll_fresp <> io.lsu_io.fresp
    }
    if (usingVector) {
      io.ll_vresp <> io.lsu_io.vresp
    }
  }

  // Outputs (Write Port #0)  ---------------
  if (writesIrf) {
    io.iresp.valid     := iresp_fu_units.map(_.io.resp.valid).reduce(_|_)
    io.iresp.bits.uop  := PriorityMux(iresp_fu_units.map(f =>
      (f.io.resp.valid, f.io.resp.bits.uop)))
    io.iresp.bits.data := PriorityMux(iresp_fu_units.map(f =>
      (f.io.resp.valid, f.io.resp.bits.data)))
    io.iresp.bits.predicated := PriorityMux(iresp_fu_units.map(f =>
      (f.io.resp.valid, f.io.resp.bits.predicated)))

    // pulled out for critical path reasons
    // TODO: Does this make sense as part of the iresp bundle?
    if (hasAlu) {
      io.iresp.bits.uop.csr_addr := ImmGen(alu.io.resp.bits.uop.imm_packed, IS_I).asUInt
      io.iresp.bits.uop.ctrl.csr_cmd := alu.io.resp.bits.uop.ctrl.csr_cmd
    }
  }

  assert ((PopCount(iresp_fu_units.map(_.io.resp.valid)) <= 1.U && !div_resp_val) ||
          (PopCount(iresp_fu_units.map(_.io.resp.valid)) <= 2.U && (div_resp_val)),
          "Multiple functional units are fighting over the write port.")
}

/**
 * FPU-only unit, with optional second write-port for ToInt micro-ops.
 *
 * @param hasFpu does the exe unit have a fpu
 * @param hasFdiv does the exe unit have a FP divider
 * @param hasFpiu does the exe unit have a FP to int unit
 */
class FPUExeUnit(
  hasFpu  : Boolean = true,
  hasFdiv : Boolean = false,
  hasFpiu : Boolean = false
  )
  (implicit p: Parameters)
  extends ExecutionUnit(
    readsFrf  = true,
    writesFrf = true,
    writesLlIrf = hasFpiu,
    writesIrf = false,
    numBypassStages = 0,
    dataWidth = p(tile.TileKey).core.fpu.get.fLen + 1,
    bypassable = false,
    hasFpu  = hasFpu,
    hasFdiv = hasFdiv,
    hasFpiu = hasFpiu) with tile.HasFPUParameters
{
  val out_str =
    BoomCoreStringPrefix("==ExeUnit==")
    (if (hasFpu)  BoomCoreStringPrefix("- FPU (Latency: " + dfmaLatency + ")") else "") +
    (if (hasFdiv) BoomCoreStringPrefix("- FDiv/FSqrt") else "") +
    (if (hasFpiu) BoomCoreStringPrefix("- FPIU (writes to Integer RF)") else "")

  val fdiv_busy = WireInit(false.B)
  val fpiu_busy = WireInit(false.B)

  // The Functional Units --------------------
  val fu_units = ArrayBuffer[FunctionalUnit]()

  io.fu_types := Mux(hasFpu.B, FU_FPU, 0.U) |
                 Mux(!fdiv_busy && hasFdiv.B, FU_FDV, 0.U) |
                 Mux(!fpiu_busy && hasFpiu.B, FU_F2I, 0.U)

  // FPU Unit -----------------------
  var fpu: FPUUnit = null
  val fpu_resp_val = WireInit(false.B)
  val fpu_resp_fflags = Wire(new ValidIO(new FFlagsResp()))
  fpu_resp_fflags.valid := false.B
  if (hasFpu) {
    fpu = Module(new FPUUnit())
    fpu.io.req.valid         := io.req.valid &&
                                (io.req.bits.uop.fu_code_is(FU_FPU) ||
                                io.req.bits.uop.fu_code_is(FU_F2I)) // TODO move to using a separate unit
    fpu.io.req.bits.uop      := io.req.bits.uop
    fpu.io.req.bits.rs1_data := io.req.bits.rs1_data
    fpu.io.req.bits.rs2_data := io.req.bits.rs2_data
    fpu.io.req.bits.rs3_data := io.req.bits.rs3_data
    fpu.io.req.bits.pred_data := false.B
    fpu.io.req.bits.kill     := io.req.bits.kill
    fpu.io.fcsr_rm           := io.fcsr_rm
    fpu.io.brupdate          := io.brupdate
    fpu.io.resp.ready        := DontCare
    fpu_resp_val             := fpu.io.resp.valid
    fpu_resp_fflags          := fpu.io.resp.bits.fflags

    fu_units += fpu
  }

  // FDiv/FSqrt Unit -----------------------
  var fdivsqrt: FDivSqrtUnit = null
  val fdiv_resp_fflags = Wire(new ValidIO(new FFlagsResp()))
  fdiv_resp_fflags := DontCare
  fdiv_resp_fflags.valid := false.B
  if (hasFdiv) {
    fdivsqrt = Module(new FDivSqrtUnit())
    fdivsqrt.io.req.valid         := io.req.valid && io.req.bits.uop.fu_code_is(FU_FDV)
    fdivsqrt.io.req.bits.uop      := io.req.bits.uop
    fdivsqrt.io.req.bits.rs1_data := io.req.bits.rs1_data
    fdivsqrt.io.req.bits.rs2_data := io.req.bits.rs2_data
    fdivsqrt.io.req.bits.rs3_data := io.req.bits.rs3_data
    fdivsqrt.io.req.bits.pred_data := false.B
    fdivsqrt.io.req.bits.kill     := io.req.bits.kill
    fdivsqrt.io.fcsr_rm           := io.fcsr_rm
    fdivsqrt.io.brupdate          := io.brupdate

    // share write port with the pipelined units
    fdivsqrt.io.resp.ready := !(fu_units.map(_.io.resp.valid).reduce(_|_)) // TODO PERF will get blocked by fpiu.

    fdiv_busy := !fdivsqrt.io.req.ready || (io.req.valid && io.req.bits.uop.fu_code_is(FU_FDV))

    fdiv_resp_fflags := fdivsqrt.io.resp.bits.fflags

    fu_units += fdivsqrt
  }

  // Outputs (Write Port #0)  ---------------

  io.fresp.valid       := fu_units.map(_.io.resp.valid).reduce(_|_) &&
                          !(fpu.io.resp.valid && fpu.io.resp.bits.uop.fu_code_is(FU_F2I))
  io.fresp.bits.uop    := PriorityMux(fu_units.map(f => (f.io.resp.valid,
                                                         f.io.resp.bits.uop)))
  io.fresp.bits.data:= PriorityMux(fu_units.map(f => (f.io.resp.valid, f.io.resp.bits.data)))
  io.fresp.bits.fflags := Mux(fpu_resp_val, fpu_resp_fflags, fdiv_resp_fflags)

  // Outputs (Write Port #1) -- FpToInt Queuing Unit -----------------------

  if (hasFpiu) {
    // TODO instantiate our own fpiu; and remove it from fpu.scala.
    // buffer up results since we share write-port on integer regfile.
    val queue = Module(new BranchKillableQueue(new ExeUnitResp(dataWidth),
      entries = dfmaLatency + 3)) // TODO being overly conservative
    queue.io.enq.valid       := (fpu.io.resp.valid &&
                                 fpu.io.resp.bits.uop.fu_code_is(FU_F2I) &&
                                 fpu.io.resp.bits.uop.uopc =/= uopSTA) // STA means store data gen for floating point
    queue.io.enq.bits.uop    := fpu.io.resp.bits.uop
    queue.io.enq.bits.data   := fpu.io.resp.bits.data
    queue.io.enq.bits.predicated := fpu.io.resp.bits.predicated
    queue.io.enq.bits.fflags := fpu.io.resp.bits.fflags
    queue.io.brupdate          := io.brupdate
    queue.io.flush           := io.req.bits.kill

    assert (queue.io.enq.ready) // If this backs up, we've miscalculated the size of the queue.

    val fp_sdq = Module(new BranchKillableQueue(new ExeUnitResp(dataWidth),
      entries = 3)) // Lets us backpressure floating point store data
    fp_sdq.io.enq.valid      := io.req.valid && io.req.bits.uop.uopc === uopSTA && !IsKilledByBranch(io.brupdate, io.req.bits.uop)
    fp_sdq.io.enq.bits.uop   := io.req.bits.uop
    fp_sdq.io.enq.bits.data  := ieee(io.req.bits.rs2_data)
    fp_sdq.io.enq.bits.predicated := false.B
    fp_sdq.io.enq.bits.fflags := DontCare
    fp_sdq.io.brupdate         := io.brupdate
    fp_sdq.io.flush          := io.req.bits.kill

    assert(!(fp_sdq.io.enq.valid && !fp_sdq.io.enq.ready))

    val resp_arb = Module(new Arbiter(new ExeUnitResp(dataWidth), 2))
    resp_arb.io.in(0) <> queue.io.deq
    resp_arb.io.in(1) <> fp_sdq.io.deq
    io.ll_iresp       <> resp_arb.io.out

    fpiu_busy := !(queue.io.empty && fp_sdq.io.empty)
  }

  override def toString: String = out_str.toString
}

/**
 * VEC execution unit that can have a branch, alu, mul, div, int to FP,
 * and memory unit.
 *
 * @param hasAlu does the exe unit have a alu
 * @param hasMul does the exe unit have a multiplier
 * @param hasDiv does the exe unit have a divider
 */
class VecExeUnit(
  hasVMX         : Boolean = false,
  hasAlu         : Boolean = true,
  hasMacc        : Boolean = true,
  hasVMaskUnit   : Boolean = true,
  hasDiv         : Boolean = true,
  hasIfpu        : Boolean = false,
  hasFpu         : Boolean = false,
  hasFdiv        : Boolean = false
) (implicit p: Parameters)
  extends ExecutionUnit(
    readsVrf         = true,
    writesVrf        = true,
    writesIrf        = true,
    writesFrf        = true,
    numBypassStages  = if (hasMacc) 3 else 1,
    dataWidth        = p(tile.TileKey).core.eLen,
    bypassable       = false,
    alwaysBypassable = false,
    hasAlu           = hasAlu,
    hasMacc          = hasMacc,
    hasVMaskUnit     = hasVMaskUnit,
    hasDiv           = hasDiv,
    hasIfpu          = hasIfpu,
    hasFpu           = hasFpu,
    hasFpiu          = hasFpu,
    hasFdiv          = hasFdiv,
    hasVector        = true,
    hasVMX           = hasVMX
) with freechips.rocketchip.rocket.constants.MemoryOpConstants
{
  val out_str =
    BoomCoreStringPrefix("==VecExeUnit==") +
    (if (hasAlu)  BoomCoreStringPrefix(" - ALU") else "") +
    (if (hasMacc) BoomCoreStringPrefix(" - MACC") else "") +
    (if (hasDiv)  BoomCoreStringPrefix(" - DIV") else "") +
    (if (hasIfpu) BoomCoreStringPrefix(" - IFPU") else "") +
    (if (hasFpu)  BoomCoreStringPrefix(" - FPU/FPIU (Latency: " + dfmaLatency + ")") else "")

  val numStages = dfmaLatency

  override def toString: String = out_str.toString

  val div_busy  = WireInit(false.B)
  val fdiv_busy = WireInit(false.B)

  // The Functional Units --------------------
  val vec_fu_units   = ArrayBuffer[FunctionalUnit]()
  //val vresp_fu_units = ArrayBuffer[FunctionalUnit]()

  io.fu_types := Mux(hasAlu.B,                FU_ALU, 0.U) |
                 Mux(hasMacc.B,               FU_MAC, 0.U) |
                 Mux(hasVMaskUnit.B,          FU_VMASKU, 0.U) |
                 Mux(!div_busy && hasDiv.B,   FU_DIV, 0.U) |
                 Mux(hasIfpu.B,               FU_I2F, 0.U) |
                 Mux(hasFpu.B,       FU_FPU | FU_F2I, 0.U) |
                 Mux(hasVMX.B,                FU_VMX, 0.U) |
                 Mux(!fdiv_busy && hasFdiv.B, FU_FDV, 0.U)

  // ALU Unit -------------------------------
  var alu: ALUUnit = null
  if (hasAlu) {
    alu = Module(new ALUUnit(numStages = numStages, dataWidth = eLen, hasVMX = hasVMX, vector = hasVector))
    alu.io.req.valid := io.req.valid && (io.req.bits.uop.fu_code === FU_ALU || io.req.bits.uop.fu_code_is(FU_VMX)) &&
      (io.req.bits.uop.uopc =/= uopVFMV_F_S) &&
      (io.req.bits.uop.uopc =/= uopVMV_X_S)
    vec_fu_units += alu
    //vresp_fu_units += alu
  }

  // Pipelined, IMul Unit ------------------
  var imacc: IntMulAcc = null
  if (hasMacc) {
    imacc = Module(new IntMulAcc(numStages, eLen))
    imacc.io <> DontCare
    imacc.io.req.valid         := io.req.valid && io.req.bits.uop.fu_code_is(FU_MAC)
    vec_fu_units += imacc
    //vresp_fu_units += imacc
  }

  // Pipelined, VMaskUnit ------------------
  var vmaskunit: PipelinedVMaskUnit = null
  if (hasVMaskUnit) {
    vmaskunit = Module(new PipelinedVMaskUnit(numStages, eLen))
    vmaskunit.io <> DontCare
    vmaskunit.io.req.valid  := io.req.valid && io.req.bits.uop.fu_code_is(FU_VMASKU)

    vec_fu_units += vmaskunit
  }

  // Div/Rem Unit -----------------------
  var div: DivUnit = null
  val div_resp_val = WireInit(false.B)
  if (hasDiv) {
    div = Module(new DivUnit(xLen))
    div.io <> DontCare
    div.io.req.valid           := io.req.valid && io.req.bits.uop.fu_code_is(FU_DIV) && hasDiv.B

    // share write port with the pipelined units
    div.io.resp.ready := !(vec_fu_units.map(_.io.resp.valid).reduce(_|_)) && io.vresp.ready

    div_resp_val := div.io.resp.valid
    div_busy     := !div.io.req.ready ||
                    (io.req.valid && io.req.bits.uop.fu_code_is(FU_DIV))

    vec_fu_units += div
    //vresp_fu_units += div
  }

  var ifpu: IntToFPUnit = null
  if (hasIfpu) {
    ifpu = Module(new IntToFPUnit(latency=numStages, vector = true))
    ifpu.io.req.valid  := io.req.valid && io.req.bits.uop.fu_code_is(FU_I2F)
    ifpu.io.fcsr_rm    := io.fcsr_rm
    ifpu.io.resp.ready := DontCare
    vec_fu_units += ifpu
    //vresp_fu_units += ifpu
  }

  // FPU Unit -----------------------
  var fpu: FPUUnit = null
  val fpu_resp_val = WireInit(false.B)
  val fpu_resp_fflags = Wire(new ValidIO(new FFlagsResp()))
  fpu_resp_fflags.valid := false.B
  if (hasFpu) {
    fpu = Module(new FPUUnit(vector = true))
    fpu.io.req.valid         := io.req.valid &&
                                (io.req.bits.uop.fu_code_is(FU_FPU) ||
                                io.req.bits.uop.fu_code_is(FU_F2I)) // TODO move to using a separate unit
    assert(fpu.io.req.bits.pred_data === false.B, "Expecting operations without predication for FPU")
    fpu.io.fcsr_rm           := io.fcsr_rm
    fpu.io.resp.ready        := DontCare
    fpu_resp_val             := fpu.io.resp.valid
    fpu_resp_fflags          := fpu.io.resp.bits.fflags

    vec_fu_units += fpu
    //vresp_fu_units += fpu
  }

  // FDiv/FSqrt Unit -----------------------
  var fdivsqrt: FDivSqrtUnit = null
  val fdiv_resp_fflags = Wire(new ValidIO(new FFlagsResp()))
  fdiv_resp_fflags := DontCare
  fdiv_resp_fflags.valid := false.B
  if (hasFdiv) {
    fdivsqrt = Module(new FDivSqrtUnit(vector = true))
    fdivsqrt.io.req.valid         := io.req.valid && io.req.bits.uop.fu_code_is(FU_FDV)
    fdivsqrt.io.req.bits.uop      := io.req.bits.uop
    fdivsqrt.io.req.bits.rs1_data := io.req.bits.rs1_data
    fdivsqrt.io.req.bits.rs2_data := io.req.bits.rs2_data
    fdivsqrt.io.req.bits.rs3_data := DontCare
    fdivsqrt.io.req.bits.pred_data := false.B
    fdivsqrt.io.req.bits.kill     := io.req.bits.kill
    fdivsqrt.io.fcsr_rm           := io.fcsr_rm
    fdivsqrt.io.brupdate          := io.brupdate

    // share write port with the pipelined units
    //fdivsqrt.io.resp.ready := !(vresp_fu_units.map(_.io.resp.valid).reduce(_|_)) && io.vresp.ready  // TODO PERF will get blocked by fpiu.
    fdivsqrt.io.resp.ready := !(vec_fu_units.map(_.io.resp.valid).reduce(_|_)) && io.vresp.ready  // TODO PERF will get blocked by fpiu.


    fdiv_busy := !fdivsqrt.io.req.ready || (io.req.valid && io.req.bits.uop.fu_code_is(FU_FDV))

    fdiv_resp_fflags := fdivsqrt.io.resp.bits.fflags

    vec_fu_units += fdivsqrt
    //vresp_fu_units += fdivsqrt
  }

  vec_fu_units.map(f => {
    f.io.req.bits.uop := io.req.bits.uop
    f.io.req.bits.rs1_data := io.req.bits.rs1_data
    f.io.req.bits.rs2_data := io.req.bits.rs2_data
    f.io.req.bits.rs3_data := io.req.bits.rs3_data
    f.io.req.bits.pred_data := io.req.bits.pred_data
    f.io.req.bits.kill := io.req.bits.kill
    f.io.brupdate := io.brupdate
    if (f != div && f != fdivsqrt) f.io.resp.ready := io.vresp.ready
  })

  // Outputs (Write Port #0)  ---------------
  if (writesVrf) {
    io.vresp.valid     := vec_fu_units.map(_.io.resp.valid).reduce(_|_) && vec_fu_units.map(f => f.io.resp.bits.uop.uopc =/= uopVPOPC).foldLeft(true.B)(_ && _)
    io.vresp.bits.uop  := PriorityMux(vec_fu_units.map(f =>
      (f.io.resp.valid, f.io.resp.bits.uop)))
    io.vresp.bits.data := PriorityMux(vec_fu_units.map(f =>
      (f.io.resp.valid, f.io.resp.bits.data)))
    io.vresp.bits.predicated := PriorityMux(vec_fu_units.map(f =>
      (f.io.resp.valid, f.io.resp.bits.predicated)))
  }

  if (writesFrf){
    val vecToFPQueue = Module(new BranchKillableQueue(new ExeUnitResp(dataWidth),
        numStages))
    vecToFPQueue.io.enq.valid := io.req.valid && (io.req.bits.uop.uopc === uopVFMV_F_S) && !io.req.bits.uop.vstart.orR()
    vecToFPQueue.io.enq.bits.uop := io.req.bits.uop
    /* @fixme: fix elen 64bits currently, flexible sew need to be supported. */
    vecToFPQueue.io.enq.bits.uop.mem_size := 2.U
    vecToFPQueue.io.enq.bits.data := io.req.bits.rs2_data
    vecToFPQueue.io.enq.bits.predicated := false.B
    vecToFPQueue.io.enq.bits.fflags := DontCare
    vecToFPQueue.io.brupdate := io.brupdate
    vecToFPQueue.io.flush := io.req.bits.kill
    io.fresp <> vecToFPQueue.io.deq
    assert(!(vecToFPQueue.io.enq.valid && !vecToFPQueue.io.enq.ready))
  }

  if (writesIrf){
    val vecToIntQueue = Module(new BranchKillableQueue(new ExeUnitResp(dataWidth), numStages))

    val vmv_valid = io.req.valid && (io.req.bits.uop.uopc === uopVMV_X_S) && !io.req.bits.uop.vstart.orR()
    val vmv_is_last = (io.req.bits.uop.uopc === uopVMV_X_S) && !io.req.bits.uop.vstart.orR()
    val vpopc_is_last = io.req.bits.uop.v_is_last
    val vpopc_valid = vmaskunit.io.resp.valid
    val vpopc_result = vmaskunit.io.resp.bits.data

    vecToIntQueue.io.enq.valid := vmv_valid | vpopc_valid
    vecToIntQueue.io.enq.bits.uop := io.req.bits.uop
    vecToIntQueue.io.enq.bits.data := Mux(vmv_valid, io.req.bits.rs2_data, Mux(vpopc_valid, vpopc_result, 0.U))
    vecToIntQueue.io.enq.bits.uop.v_is_last := vmv_is_last
    vecToIntQueue.io.enq.bits.predicated := false.B
    vecToIntQueue.io.enq.bits.fflags := DontCare
    vecToIntQueue.io.brupdate := io.brupdate
    vecToIntQueue.io.flush := io.req.bits.kill
    io.iresp <> vecToIntQueue.io.deq
    assert(!(vecToIntQueue.io.enq.valid && !vecToIntQueue.io.enq.ready))
  }

  /*
  no more guaranteed as div and fdiv are both unpipelined
  assert ((PopCount(vec_fu_units.map(_.io.resp.valid)) <= 1.U && !div_resp_val) ||
          (PopCount(vec_fu_units.map(_.io.resp.valid)) <= 2.U && (div_resp_val)),
          "Multiple functional units are fighting over the write port.")
  */

  override def supportedFuncUnits = {
    new SupportedFuncUnits(
      alu = hasAlu,
      jmp = hasJmpUnit,
      mem = hasMem,
      muld = hasMul || hasDiv,
      fpu = hasFpu,
      csr = hasCSR,
      fdiv = hasFdiv,
      ifpu = hasIfpu,
      vector = true
    )
  }
}

