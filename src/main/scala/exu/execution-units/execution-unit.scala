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
import freechips.rocketchip.util.{UIntIsOneOf, LCG}

import FUConstants._
import boom.common._
import boom.common.MicroOpcodes._
import boom.ifu.{GetPCFromFtqIO}
import boom.util._

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

    val iresp    = if (writesIrf)   new DecoupledIO(new ExeUnitResp(xLen)) else null
    val fresp    = if (hasVector && writesFrf) new DecoupledIO(new ExeUnitResp(xLen)) 
                   else if(writesFrf)          new DecoupledIO(new ExeUnitResp(xLen+1)) else null
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
    val vxrm    = if (hasVxrm) Input(UInt(2.W)) else null
    val vconfig = if (hasVConfig) Input(new VConfig) else null

    // only used by the mem unit
    val lsu_io = if (hasMem) Flipped(new boom.lsu.LSUExeIO) else null
    val bp = if (hasMem) Input(Vec(nBreakpoints, new BP)) else null
    val mcontext = if (hasMem) Input(UInt(coreParams.mcontextWidth.W)) else null
    val scontext = if (hasMem) Input(UInt(coreParams.scontextWidth.W)) else null

    // TODO move this out of ExecutionUnit
    val com_exception = if (hasMem || hasRocc) Input(Bool()) else null

    val perf = Output(new Bundle {
      val div_busy  = if (hasDiv) Output(Bool())  else null
      val fdiv_busy = if (hasFdiv) Output(Bool()) else null
    })
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
  def hasVxrm = hasMacc
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
    alu.io.req.bits.rvm_data := DontCare
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

    io.perf.div_busy := div_busy

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
    fpu.io.req.bits.rvm_data := DontCare
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
    fdivsqrt.io.req.bits.rvm_data := DontCare
    fdivsqrt.io.req.bits.pred_data := false.B
    fdivsqrt.io.req.bits.kill     := io.req.bits.kill
    fdivsqrt.io.fcsr_rm           := io.fcsr_rm
    fdivsqrt.io.brupdate          := io.brupdate

    // share write port with the pipelined units
    fdivsqrt.io.resp.ready := !(fu_units.map(_.io.resp.valid).reduce(_|_)) // TODO PERF will get blocked by fpiu.

    fdiv_busy := !fdivsqrt.io.req.ready || (io.req.valid && io.req.bits.uop.fu_code_is(FU_FDV))

    fdiv_resp_fflags := fdivsqrt.io.resp.bits.fflags

    fu_units += fdivsqrt

    io.perf.fdiv_busy := fdiv_busy
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
    writesIrf        = hasAlu,
    writesFrf        = hasAlu,
    numBypassStages  = if (hasMacc) 3 else 1,
    dataWidth        = p(tile.TileKey).core.vLen,
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
    (if (hasFpu)  BoomCoreStringPrefix(" - FPU/FPIU (Latency: " + dfmaLatency + ")") else "") +
    (if (hasDiv)  BoomCoreStringPrefix(" - FDIV/FR7") else "")

  val numStages = if (hasAlu) dfmaLatency else 1

  override def toString: String = out_str.toString

  val div_busy  = WireInit(false.B)
  val vmx_busy  = WireInit(false.B)
  val fdiv_busy = WireInit(false.B)
  val vrp_busy  = WireInit(false.B)

  // The Functional Units --------------------
  val vec_fu_units   = ArrayBuffer[FunctionalUnit]()
  //val vresp_fu_units = ArrayBuffer[FunctionalUnit]()

  io.fu_types := Mux(!vrp_busy && hasAlu.B,   FU_ALU, 0.U) |
                 Mux(hasMacc.B,               FU_MAC, 0.U) |
                 Mux(hasVMaskUnit.B,          FU_VMASKU, 0.U) |
                 Mux(!div_busy && hasDiv.B,   FU_DIV, 0.U) |
                 Mux(hasIfpu.B,               FU_I2F, 0.U) |
                 Mux(!vrp_busy && hasFpu.B,   FU_FPU | FU_F2I, 0.U) |
                 Mux(!vmx_busy && hasVMX.B,   FU_VMX, 0.U) |
                 Mux(!fdiv_busy && hasFdiv.B, FU_FDV, 0.U) |
                 Mux(hasFdiv.B,               FU_FR7, 0.U) |
                 Mux(!vrp_busy && hasAlu.B,   FU_VRP, 0.U)

  // ALU Unit -------------------------------
  var valu: VecALUUnit = null
  if (hasAlu) {
    valu = Module(new VecALUUnit(numStages = numStages, dataWidth = vLen))
    //valu.io.req.valid := io.req.valid && (io.req.bits.uop.fu_code & FU_ALU).orR //&& 
                        //(io.req.bits.uop.uopc =/= uopVFMV_F_S) && (io.req.bits.uop.uopc =/= uopVMV_X_S)
    vec_fu_units += valu
  }

  // Pipelined, FixMulAcc Unit ------------------
  var vfix: VecFixUnit = null
  if (hasMacc) {
    vfix = Module(new VecFixUnit(numStages, vLen))
    vfix.suggestName("vfix")
    vfix.io           <> DontCare
    vfix.io.req.valid := io.req.valid && io.req.bits.uop.fu_code_is(FU_MAC)
    vfix.io.vxrm      := io.vxrm
    vec_fu_units += vfix
  }

  // Pipelined, VMaskUnit ------------------
  /*var vmaskunit: PipelinedVMaskUnit = null
  if (hasVMaskUnit) {
    vmaskunit = Module(new PipelinedVMaskUnit(numStages, eLen)).suggestName("vmaskunit")
    vmaskunit.io <> DontCare
    vmaskunit.io.req.valid  := io.req.valid && io.req.bits.uop.fu_code_is(FU_VMASKU)

    vec_fu_units += vmaskunit
  }*/
  var vmaskUnit: VecMaskUnit = null
  if (hasVMaskUnit) {
    vmaskUnit = Module(new VecMaskUnit(dataWidth = vLen)).suggestName("vmaskunit")
    vmaskUnit.io <> DontCare
    vmaskUnit.io.req.valid  := io.req.valid && io.req.bits.uop.fu_code_is(FU_VMASKU)

    vec_fu_units += vmaskUnit
  }

  /*
  var ifpu: IntToFPUnit = null
  if (hasIfpu) {
    ifpu = Module(new IntToFPUnit(latency=numStages, vector = true))
    ifpu.io.req.valid  := io.req.valid && io.req.bits.uop.fu_code_is(FU_I2F)
    ifpu.io.fcsr_rm    := io.fcsr_rm
    ifpu.io.resp.ready := DontCare
    vec_fu_units += ifpu
  } */
  var ifpu: VecIntToFPUnit = null
  if (hasIfpu) {
    ifpu = Module(new VecIntToFPUnit(dataWidth = vLen, latency=numStages))
    ifpu.io.req.valid  := io.req.valid && io.req.bits.uop.fu_code_is(FU_I2F)
    ifpu.io.fcsr_rm    := io.fcsr_rm
    ifpu.io.resp.ready := DontCare
    vec_fu_units += ifpu
  }


  // FPU Unit -----------------------
  var vfpu: VecFPUUnit = null
  val fpu_resp_val = WireInit(false.B)
  val fpu_resp_fflags = Wire(new ValidIO(new FFlagsResp()))
  fpu_resp_fflags.valid := false.B
  fpu_resp_fflags.bits  := DontCare
  if (hasFpu) {
    vfpu = Module(new VecFPUUnit(vLen))
    vfpu.suggestName("vfpu")
    //vfpu.io.req.valid         := io.req.valid &&
                                //(io.req.bits.uop.fu_code_is(FU_FPU) ||
                                //io.req.bits.uop.fu_code_is(FU_F2I)) // TODO move to using a separate unit
    assert(vfpu.io.req.bits.pred_data === false.B, "Expecting operations without predication for VFPU")
    vfpu.io.fcsr_rm           := io.fcsr_rm
    vfpu.io.resp.ready        := DontCare
    fpu_resp_val             := vfpu.io.resp.valid
    fpu_resp_fflags          := vfpu.io.resp.bits.fflags

    vec_fu_units += vfpu
  }

  // FR7 Unit
  var fr7: VecFR7Unit = null
  if(hasFdiv) {
    fr7 = Module(new VecFR7Unit(latency=numStages, dataWidth=vLen))
    fr7.io.req.valid  := io.req.valid && io.req.bits.uop.fu_code_is(FU_FR7)
    fr7.io.fcsr_rm    := io.fcsr_rm
    fr7.io.resp.ready := DontCare

    vec_fu_units += fr7
  }

  // VMX Unit -------------------------------
  // It is the only FU in VMX pipe
  var vmx: VMXUnit = null
  if (hasVMX) {
    vmx = Module(new VMXUnit(vLen))
    vmx.suggestName("vmx_unit")
    vmx.io.req.valid := io.req.valid && io.req.bits.uop.fu_code_is(FU_VMX)

    // separate write port
    vmx.io.resp.ready := io.vresp.ready
    vmx_busy := !vmx.io.req.ready

    vec_fu_units += vmx
  }

  // Div/Rem Unit with SRT4 Divider --------
  /*
  var div: SRT4DivUnit = null
  val div_resp_val = WireInit(false.B)
  if (hasDiv) {
    div = Module(new SRT4DivUnit(xLen)) //Module(new DivUnit(xLen))
    div.io.req.valid  := io.req.valid && io.req.bits.uop.fu_code_is(FU_DIV)

    // share write port with the pipelined units
    div.io.resp.ready := !(vec_fu_units.map(_.io.resp.valid).reduce(_|_)) && io.vresp.ready

    div_resp_val := div.io.resp.valid
    div_busy     := !div.io.req.ready || (io.req.valid && io.req.bits.uop.fu_code_is(FU_DIV))

    io.perf.div_busy := div_busy

    vec_fu_units += div
  }*/

  var vdiv: VecSRT4DivUnit = null
  val div_resp_val = WireInit(false.B)
  if (hasDiv) {
    vdiv = Module(new VecSRT4DivUnit(vLen))
    vdiv.io.req.valid := io.req.valid && io.req.bits.uop.fu_code_is(FU_DIV)

    // share write port with the pipelined units
    vdiv.io.resp.ready := !(vec_fu_units.map(_.io.resp.valid).reduce(_|_)) && io.vresp.ready

    div_resp_val := vdiv.io.resp.valid
    div_busy     := !vdiv.io.req.ready || (io.req.valid && io.req.bits.uop.fu_code_is(FU_DIV))

    vec_fu_units += vdiv

    io.perf.div_busy := div_busy
  }



  // FDiv/FSqrt Unit -----------------------
  /*
  var fdivsqrt: FDivSqrtUnit = null
  val fdiv_resp_fflags = Wire(new ValidIO(new FFlagsResp()))
  fdiv_resp_fflags := DontCare
  fdiv_resp_fflags.valid := false.B
  if (hasFdiv) {
    fdivsqrt = Module(new FDivSqrtUnit(vector = true))
    fdivsqrt.io.req.valid := io.req.valid && io.req.bits.uop.fu_code_is(FU_FDV)
    fdivsqrt.io.fcsr_rm   := io.fcsr_rm

    // share write port with the pipelined units
    fdivsqrt.io.resp.ready := !(vec_fu_units.map(_.io.resp.valid).reduce(_|_)) && io.vresp.ready  // TODO PERF will get blocked by fpiu.

    fdiv_busy := !fdivsqrt.io.req.ready || (io.req.valid && io.req.bits.uop.fu_code_is(FU_FDV))

    fdiv_resp_fflags := fdivsqrt.io.resp.bits.fflags

    vec_fu_units += fdivsqrt
    //vresp_fu_units += fdivsqrt
  }*/

  var fdivsqrt: VecFDivSqrtUnit = null
  val fdiv_resp_fflags = Wire(new ValidIO(new FFlagsResp()))
  fdiv_resp_fflags := DontCare
  fdiv_resp_fflags.valid := false.B
  if (hasFdiv) {
    fdivsqrt = Module(new VecFDivSqrtUnit(dataWidth = vLen))
    fdivsqrt.io.req.valid := io.req.valid && io.req.bits.uop.fu_code_is(FU_FDV)
    fdivsqrt.io.fcsr_rm   := io.fcsr_rm

    // share write port with the pipelined units
    fdivsqrt.io.resp.ready := !(vec_fu_units.map(_.io.resp.valid).reduce(_|_)) && io.vresp.ready  // TODO PERF will get blocked by fpiu.

    fdiv_busy := !fdivsqrt.io.req.ready || (io.req.valid && io.req.bits.uop.fu_code_is(FU_FDV))

    fdiv_resp_fflags := fdivsqrt.io.resp.bits.fflags

    vec_fu_units += fdivsqrt
    //vresp_fu_units += fdivsqrt

    io.perf.fdiv_busy := fdiv_busy
  }

  vec_fu_units.map(f => {
    f.io.req.bits.uop       := io.req.bits.uop
    f.io.req.bits.rs1_data  := io.req.bits.rs1_data
    f.io.req.bits.rs2_data  := io.req.bits.rs2_data
    f.io.req.bits.rs3_data  := io.req.bits.rs3_data
    f.io.req.bits.rvm_data  := io.req.bits.rvm_data
    f.io.req.bits.pred_data := io.req.bits.pred_data
    f.io.req.bits.kill      := io.req.bits.kill
    f.io.brupdate           := io.brupdate
    if (f != vdiv && f != fdivsqrt && f!= vmx) 
      f.io.resp.ready := io.vresp.ready
  })

  if (hasAlu) {
    val vrp = Module(new VecRPAssist())

    // Red-Perm Assist and related arbiter
    //vrp.io.exreq.valid    := io.req.valid && (io.req.bits.uop.fu_code & FU_IVRP).orR
    vrp.io.exreq.valid    := io.req.valid && (io.req.bits.uop.fu_code & (FU_ALU | FU_FPU)).orR && (io.req.bits.uop.fu_code & FU_VRP).orR
    vrp.io.exreq.bits     := io.req.bits
    vrp.io.fbreq.ready    := true.B
    val valu_resp_vrp = valu.io.resp.valid && (valu.io.resp.bits.uop.fu_code & FU_VRP).orR
    val vfpu_resp_vrp = vfpu.io.resp.valid && (vfpu.io.resp.bits.uop.fu_code & FU_VRP).orR
    vrp.io.fbrsp.valid    := valu_resp_vrp || vfpu_resp_vrp

    vrp.io.fbrsp.bits     := Mux(vfpu_resp_vrp, vfpu.io.resp.bits, valu.io.resp.bits)
    vrp.io.brupdate       := io.brupdate
    vrp_busy              := vrp.io.busy

    val valu_exreq_valid = io.req.valid &&
                           (io.req.bits.uop.fu_code & FU_ALU).orR &&
                           !(io.req.bits.uop.fu_code & FU_VRP).orR
    valu.io.req.valid         := Mux(vrp_busy, vrp.io.fbreq.valid && (vrp.io.fbreq.bits.uop.fu_code & FU_ALU).orR,
                                               valu_exreq_valid)
    valu.io.req.bits.uop      := Mux(vrp_busy, vrp.io.fbreq.bits.uop, io.req.bits.uop)
    valu.io.req.bits.rs1_data := Mux(vrp_busy, vrp.io.fbreq.bits.rs1_data, io.req.bits.rs1_data)
    valu.io.req.bits.rs2_data := Mux(vrp_busy, vrp.io.fbreq.bits.rs2_data, io.req.bits.rs2_data)
    valu.io.req.bits.rs3_data := Mux(vrp_busy, vrp.io.fbreq.bits.rs3_data, io.req.bits.rs3_data)
    valu.io.req.bits.rvm_data := Mux(vrp_busy, vrp.io.fbreq.bits.rvm_data, io.req.bits.rvm_data)

    val vfpu_exreq_valid = io.req.valid &&
                            (io.req.bits.uop.fu_code_is(FU_FPU) && !io.req.bits.uop.fu_code_is(FU_VRP) ||
                             io.req.bits.uop.fu_code_is(FU_F2I))
    vfpu.io.req.valid         := Mux(vrp_busy, vrp.io.fbreq.valid && (vrp.io.fbreq.bits.uop.fu_code & FU_FPU).orR,
                                               vfpu_exreq_valid)
    vfpu.io.req.bits.uop      := Mux(vrp_busy, vrp.io.fbreq.bits.uop, io.req.bits.uop)
    vfpu.io.req.bits.rs1_data := Mux(vrp_busy, vrp.io.fbreq.bits.rs1_data, io.req.bits.rs1_data)
    vfpu.io.req.bits.rs2_data := Mux(vrp_busy, vrp.io.fbreq.bits.rs2_data, io.req.bits.rs2_data)
    vfpu.io.req.bits.rs3_data := Mux(vrp_busy, vrp.io.fbreq.bits.rs3_data, io.req.bits.rs3_data)
    vfpu.io.req.bits.rvm_data := Mux(vrp_busy, vrp.io.fbreq.bits.rvm_data, io.req.bits.rvm_data)
  }

  // Outputs (Write Port #0)  ---------------
  if (writesVrf) {
    io.vresp.valid     := vec_fu_units.map(f => 
      f.io.resp.valid && !(f.io.resp.bits.uop.fu_code & FU_VRP).orR && !f.io.resp.bits.uop.uopc.isOneOf(uopVPOPC, uopVFIRST, uopVFMV_F_S, uopVMV_X_S)).reduce(_||_)
    io.vresp.bits.uop  := PriorityMux(vec_fu_units.map(f =>
      (f.io.resp.valid, f.io.resp.bits.uop)))
    io.vresp.bits.data := PriorityMux(vec_fu_units.map(f =>
      (f.io.resp.valid, f.io.resp.bits.data)))
    io.vresp.bits.predicated := PriorityMux(vec_fu_units.map(f =>
      (f.io.resp.valid, f.io.resp.bits.predicated)))
  }

  if (writesFrf){
    val vecToFPQueue = Module(new BranchKillableQueue(new ExeUnitResp(eLen), numStages))
    val vsew = io.req.bits.uop.vconfig.vtype.vsew(1,0)
    val rs2_data_elem0 = Mux1H(UIntToOH(vsew-1.U), Seq(io.req.bits.rs2_data(15, 0),
                                                       io.req.bits.rs2_data(31, 0),
                                                       io.req.bits.rs2_data(63, 0),
                                                       0.U))
    vecToFPQueue.io.enq.valid := io.req.valid && (io.req.bits.uop.uopc === uopVFMV_F_S) && !io.req.bits.uop.v_eidx.orR()
    vecToFPQueue.io.enq.bits.uop := io.req.bits.uop
    /* @fixme: FIXME: fix elen 64bits currently, flexible sew need to be supported. */
    assert(!vecToFPQueue.io.enq.valid || vsew =/= 0.U, "Unexpected FP vsew");
    vecToFPQueue.io.enq.bits.uop.mem_size := vsew - 1.U //2.U
    vecToFPQueue.io.enq.bits.data := rs2_data_elem0
    vecToFPQueue.io.enq.bits.predicated := false.B
    vecToFPQueue.io.enq.bits.fflags := DontCare
    vecToFPQueue.io.brupdate := io.brupdate
    vecToFPQueue.io.flush := io.req.bits.kill
    io.fresp <> vecToFPQueue.io.deq
    assert(!(vecToFPQueue.io.enq.valid && !vecToFPQueue.io.enq.ready))
  }

  if (writesIrf && hasAlu){
    val vecToIntQueue = Module(new BranchKillableQueue(new ExeUnitResp(eLen), numStages))

    val vmv_valid = io.req.valid && (io.req.bits.uop.uopc === uopVMV_X_S) && !io.req.bits.uop.v_eidx.orR()
    val vmv_is_last = (io.req.bits.uop.uopc === uopVMV_X_S) && !io.req.bits.uop.v_eidx.orR()
    val vl         = io.req.bits.uop.vconfig.vl
    val vmaskUop   = vmaskUnit.io.resp.bits.uop
    val vmaskValid = vmaskUnit.io.resp.valid && vmaskUop.uopc.isOneOf(uopVPOPC, uopVFIRST)
    val vsew = io.req.bits.uop.vconfig.vtype.vsew(1,0)
    val rs2_data_elem0 = Mux1H(UIntToOH(vsew), Seq(io.req.bits.rs2_data(63, 0),
                                                   io.req.bits.rs2_data(31, 0),
                                                   io.req.bits.rs2_data(15, 0),
                                                   io.req.bits.rs2_data(7, 0)))

    vecToIntQueue.io.enq.valid := vmv_valid | vmaskValid
    vecToIntQueue.io.enq.bits.uop := Mux(vmv_valid, io.req.bits.uop, vmaskUop)
    vecToIntQueue.io.enq.bits.data := Mux(vmv_valid, rs2_data_elem0, Mux(vmaskValid, vmaskUnit.io.resp.bits.data, 0.U))
    vecToIntQueue.io.enq.bits.uop.v_split_last := Mux(vmv_valid, vmv_is_last, vmaskUop.v_split_last)
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
      alu    = hasAlu,
      jmp    = hasJmpUnit,
      mem    = hasMem,
      muld   = hasMul || hasDiv,
      fpu    = hasFpu,
      csr    = hasCSR,
      fdiv   = hasFdiv,
      ifpu   = hasIfpu,
      fr7    = hasFdiv,
      vmx    = hasVMX,
      vector = true
    )
  }
}

import freechips.rocketchip.rocket._
import freechips.rocketchip.config._
import freechips.rocketchip.unittest._
import freechips.rocketchip.tile._
import freechips.rocketchip.subsystem._
import freechips.rocketchip.tilelink.LFSR64
import boom.ifu._
import boom.lsu._

class VecExeUT(timeout: Int = 10000)(implicit p: Parameters)
  extends UnitTest(timeout)
{
  val cycle = Reg(UInt(32.W))
  when (io.start) {
    cycle := 0.U
  } .otherwise {
    cycle := cycle + 1.U
  }

  val active = RegInit(false.B)
  when (io.start) {
    active := true.B
  }

  val tileParams = p(TilesLocated(InSubsystem))(0).tileParams
  val coreParams = tileParams.core.asInstanceOf[BoomCoreParams]
  val vLen = coreParams.vLen

  val dut_req   = Wire(DecoupledIO(new FuncUnitReq(vLen)))
  val vsew      = WireInit(0.U(2.W))
  val vlmul     = WireInit(0.U(3.W))
  val vlen_ecnt = ((vLen/8).U >> vsew)
  val vtype     = Wire(new VType)
  dut_req                             := DontCare
  dut_req.bits.uop                    := NullMicroOp(true)
  dut_req.bits.uop.v_eidx             := 0.U
  dut_req.bits.uop.v_split_ecnt       := vlen_ecnt
  dut_req.bits.rs1_data               := LCG(vLen, active)
  dut_req.bits.rs2_data               := LCG(vLen, active)
  dut_req.bits.rs3_data               := LCG(vLen, active)
  dut_req.bits.rvm_data               := LCG(vLen, active)
  dut_req.bits.uop.vd_emul            := 0.U
  dut_req.bits.uop.vs1_emul           := 0.U
  dut_req.bits.uop.vs2_emul           := 0.U
  dut_req.bits.uop.vd_eew             := 0.U
  dut_req.bits.uop.vs1_eew            := 0.U
  dut_req.bits.uop.vs2_eew            := 0.U
  dut_req.valid := false.B
  vtype.vill := false.B
  vtype.vta := false.B
  vtype.vma := false.B
  vtype.vsew := 0.U
  vtype.vlmul_sign := false.B
  vtype.vlmul_mag := 0.U
  vtype.reserved := DontCare
  val case1_start = 100
  val case2_start = 200
  val case3_start = 300
  val case4_start = 400
  val case5_start = 500
  when (cycle >= case1_start.U && cycle < (case1_start+8).U) {
    vsew                          := 0.U
    vlmul                         := 5.U
    vtype.vsew                    := vsew
    vtype.vlmul_sign              := vlmul(2)
    vtype.vlmul_mag               := vlmul(1,0)
    dut_req.bits.uop.uopc         := uopVADD
    dut_req.bits.uop.fu_code      := FU_ALU | FU_VRP
    dut_req.bits.uop.rob_idx      := 0x55.U
    dut_req.bits.uop.ldst_val     := true.B
    dut_req.bits.uop.dst_rtype    := RT_VEC
    dut_req.bits.uop.lrs1_rtype   := RT_VRED
    dut_req.bits.uop.lrs2_rtype   := RT_VEC
    dut_req.bits.uop.is_rvv       := true.B
    dut_req.bits.uop.vd_emul      := 0.U
    dut_req.bits.uop.vs1_emul     := 0.U
    dut_req.bits.uop.vs2_emul     := vlmul
    dut_req.bits.uop.vd_eew       := vsew
    dut_req.bits.uop.vs1_eew      := vsew
    dut_req.bits.uop.vs2_eew      := vsew
    dut_req.bits.uop.v_unmasked   := true.B
    dut_req.bits.uop.vconfig.vtype:= vtype
    dut_req.bits.uop.vconfig.vl   := vtype.vlMax
    dut_req.bits.uop.v_split_ecnt := vtype.vlMax
    dut_req.valid := cycle === case1_start.U //true.B
    dut_req.bits.rs1_data         := 1.U
    dut_req.bits.rs2_data         := Cat((0 until 128).map(i => 1.U(8.W)).reverse)
    dut_req.bits.rs3_data         := 0.U
    dut_req.bits.rvm_data         := Cat((0 until 16).map(i => 0x55.U(8.W)).reverse)
    when (cycle === (case1_start+0).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*0.U
    } .elsewhen (cycle === (case1_start+1).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*1.U
    } .elsewhen (cycle === (case1_start+2).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*2.U
    } .elsewhen (cycle === (case1_start+3).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*3.U
    } .elsewhen (cycle === (case1_start+4).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*4.U
    } .elsewhen (cycle === (case1_start+5).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*5.U
    } .elsewhen (cycle === (case1_start+6).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*6.U
    } .elsewhen (cycle === (case1_start+7).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*7.U
    }
  } .elsewhen (cycle >= (case2_start+0).U && cycle < (case2_start+8).U) {
    vsew                          := 0.U
    vlmul                         := 3.U
    vtype.vsew                    := vsew
    vtype.vlmul_sign              := false.B
    vtype.vlmul_mag               := 3.U
    dut_req.bits.uop.uopc         := uopVADD
    dut_req.bits.uop.fu_code      := FU_ALU | FU_VRP
    dut_req.bits.uop.rob_idx      := 0x55.U
    dut_req.bits.uop.ldst_val     := true.B
    dut_req.bits.uop.dst_rtype    := RT_VW
    dut_req.bits.uop.lrs1_rtype   := RT_VRW
    dut_req.bits.uop.lrs2_rtype   := RT_VEC
    dut_req.bits.uop.is_rvv       := true.B
    dut_req.bits.uop.vd_emul      := 0.U
    dut_req.bits.uop.vs1_emul     := 0.U
    dut_req.bits.uop.vs2_emul     := 3.U
    dut_req.bits.uop.vd_eew       := 1.U
    dut_req.bits.uop.vs1_eew      := 1.U
    dut_req.bits.uop.vs2_eew      := 0.U
    dut_req.bits.uop.v_unmasked   := false.B
    dut_req.bits.uop.vconfig.vtype:= vtype
    dut_req.bits.uop.vconfig.vl   := vtype.vlMax
    dut_req.bits.uop.v_split_ecnt := vlen_ecnt
    dut_req.valid := true.B
    dut_req.bits.rs1_data         := 1.U
    dut_req.bits.rs2_data         := Cat((0 until 128).map(i => 1.U(8.W)).reverse)
    dut_req.bits.rs3_data         := 0.U
    dut_req.bits.rvm_data         := Cat((0 until 16).map(i => 0x55.U(8.W)).reverse)
    when (cycle === (case2_start+0).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*0.U
    } .elsewhen (cycle === (case2_start+1).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*1.U
    } .elsewhen (cycle === (case2_start+2).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*2.U
    } .elsewhen (cycle === (case2_start+3).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*3.U
    } .elsewhen (cycle === (case2_start+4).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*4.U
    } .elsewhen (cycle === (case2_start+5).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*5.U
    } .elsewhen (cycle === (case2_start+6).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*6.U
    } .elsewhen (cycle === (case2_start+7).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*7.U
    }
  } .elsewhen (cycle >= (case3_start+0).U && cycle < (case3_start+8).U) {
    vsew                          := 0.U
    vlmul                         := 3.U
    vtype.vsew                    := vsew
    vtype.vlmul_sign              := false.B
    vtype.vlmul_mag               := 3.U
    dut_req.bits.uop.uopc         := uopVADD
    dut_req.bits.uop.fu_code      := FU_ALU | FU_VRP
    dut_req.bits.uop.rob_idx      := 0x55.U
    dut_req.bits.uop.ldst_val     := true.B
    dut_req.bits.uop.dst_rtype    := RT_VW
    dut_req.bits.uop.lrs1_rtype   := RT_VRW
    dut_req.bits.uop.lrs2_rtype   := RT_VEC
    dut_req.bits.uop.is_rvv       := true.B
    dut_req.bits.uop.vd_emul      := 0.U
    dut_req.bits.uop.vs1_emul     := 0.U
    dut_req.bits.uop.vs2_emul     := 3.U
    dut_req.bits.uop.vd_eew       := 1.U
    dut_req.bits.uop.vs1_eew      := 1.U
    dut_req.bits.uop.vs2_eew      := 0.U
    dut_req.bits.uop.v_unmasked   := true.B
    dut_req.bits.uop.vconfig.vtype:= vtype
    dut_req.bits.uop.vconfig.vl   := vtype.vlMax - 1.U
    dut_req.bits.uop.v_split_ecnt := vlen_ecnt
    dut_req.valid := true.B
    dut_req.bits.rs1_data         := 1.U
    dut_req.bits.rs2_data         := Cat((0 until 128).map(i => 1.U(8.W)).reverse)
    dut_req.bits.rs3_data         := 0.U
    dut_req.bits.rvm_data         := Cat((0 until 16).map(i => 0x55.U(8.W)).reverse)
    when (cycle === (case3_start+0).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*0.U
    } .elsewhen (cycle === (case3_start+1).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*1.U
    } .elsewhen (cycle === (case3_start+2).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*2.U
    } .elsewhen (cycle === (case3_start+3).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*3.U
    } .elsewhen (cycle === (case3_start+4).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*4.U
    } .elsewhen (cycle === (case3_start+5).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*5.U
    } .elsewhen (cycle === (case3_start+6).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*6.U
    } .elsewhen (cycle === (case3_start+7).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*7.U
    }
  } .elsewhen (cycle >= (case4_start).U && cycle < (case4_start+8).U) {
    vsew                          := 1.U
    vlmul                         := 3.U
    vtype.vsew                    := vsew
    vtype.vlmul_sign              := false.B
    vtype.vlmul_mag               := 3.U
    dut_req.bits.uop.uopc         := uopVFADD
    dut_req.bits.uop.fu_code      := FU_FPU | FU_VRP
    dut_req.bits.uop.rob_idx      := 0x55.U
    dut_req.bits.uop.ldst_val     := true.B
    dut_req.bits.uop.dst_rtype    := RT_VRED
    dut_req.bits.uop.lrs1_rtype   := RT_VRED
    dut_req.bits.uop.lrs2_rtype   := RT_VEC
    dut_req.bits.uop.is_rvv       := true.B
    dut_req.bits.uop.fp_val       := true.B
    dut_req.bits.uop.vd_emul      := 0.U
    dut_req.bits.uop.vs1_emul     := 0.U
    dut_req.bits.uop.vs2_emul     := 3.U
    dut_req.bits.uop.vd_eew       := 1.U
    dut_req.bits.uop.vs1_eew      := 1.U
    dut_req.bits.uop.vs2_eew      := 1.U
    dut_req.bits.uop.v_unmasked   := true.B
    dut_req.bits.uop.vconfig.vtype:= vtype
    dut_req.bits.uop.vconfig.vl   := vtype.vlMax - 1.U
    dut_req.bits.uop.v_split_ecnt := vlen_ecnt
    dut_req.valid := true.B
    dut_req.bits.rs1_data         := 0x3c00.U
    dut_req.bits.rs2_data         := Cat((0 until 64).map(i => 0x3c00.U(16.W)).reverse)
    dut_req.bits.rs3_data         := 0.U
    dut_req.bits.rvm_data         := Cat((0 until 16).map(i => 0x55.U(8.W)).reverse)
    when (cycle === (case4_start+0).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*0.U
    } .elsewhen (cycle === (case4_start+1).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*1.U
    } .elsewhen (cycle === (case4_start+2).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*2.U
    } .elsewhen (cycle === (case4_start+3).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*3.U
    } .elsewhen (cycle === (case4_start+4).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*4.U
    } .elsewhen (cycle === (case4_start+5).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*5.U
    } .elsewhen (cycle === (case4_start+6).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*6.U
    } .elsewhen (cycle === (case4_start+7).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*7.U
    }
  } .elsewhen (cycle >= (case5_start).U && cycle < (case5_start+8).U) {
    vsew                          := 1.U
    vlmul                         := 3.U
    vtype.vsew                    := vsew
    vtype.vlmul_sign              := false.B
    vtype.vlmul_mag               := 2.U
    dut_req.bits.uop.uopc         := uopVFADD
    dut_req.bits.uop.fu_code      := FU_FPU | FU_VRP
    dut_req.bits.uop.rob_idx      := 0x55.U
    dut_req.bits.uop.ldst_val     := true.B
    dut_req.bits.uop.dst_rtype    := RT_VW
    dut_req.bits.uop.lrs1_rtype   := RT_VRW
    dut_req.bits.uop.lrs2_rtype   := RT_VEC
    dut_req.bits.uop.is_rvv       := true.B
    dut_req.bits.uop.fp_val       := true.B
    dut_req.bits.uop.vd_emul      := 0.U
    dut_req.bits.uop.vs1_emul     := 0.U
    dut_req.bits.uop.vs2_emul     := 3.U
    dut_req.bits.uop.vd_eew       := 2.U
    dut_req.bits.uop.vs1_eew      := 2.U
    dut_req.bits.uop.vs2_eew      := 1.U
    dut_req.bits.uop.v_unmasked   := true.B
    dut_req.bits.uop.vconfig.vtype:= vtype
    dut_req.bits.uop.vconfig.vl   := vtype.vlMax - 1.U
    dut_req.bits.uop.v_split_ecnt := vlen_ecnt
    dut_req.valid := false.B //true.B
    dut_req.bits.rs1_data         := 0x3f800000.U
    dut_req.bits.rs2_data         := Cat((0 until 64).map(i => 0x3c00.U(16.W)).reverse)
    dut_req.bits.rs3_data         := 0.U
    dut_req.bits.rvm_data         := Cat((0 until 16).map(i => 0x55.U(8.W)).reverse)
    when (cycle === (case5_start+0).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*0.U
    } .elsewhen (cycle === (case5_start+1).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*1.U
    } .elsewhen (cycle === (case5_start+2).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*2.U
    } .elsewhen (cycle === (case5_start+3).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*3.U
    } .elsewhen (cycle === (case5_start+4).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*4.U
    } .elsewhen (cycle === (case5_start+5).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*5.U
    } .elsewhen (cycle === (case5_start+6).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*6.U
    } .elsewhen (cycle === (case5_start+7).U) {
      dut_req.bits.uop.v_eidx := vlen_ecnt*7.U
    }
  }

  val dut = Module(new VecExeUnit(hasVMX=false, hasIfpu=true, hasFpu=true, hasFdiv=true))
  dut.io := DontCare
  dut.io.req <> dut_req
  dut.io.iresp.ready := true.B
  dut.io.fresp.ready := true.B
  dut.io.vresp.ready := true.B
  dut.io.brupdate := BoomTestUtils.NullBrUpdateInfo
  dut.io.fcsr_rm := 0.U
  dut.io.vxrm := 0.U
  when(dut.io.req.valid) {
    printf("req: uopc %x, rob %x, vsew %x, vlmul %x, eidx %x, vl %x\n",
        dut.io.req.bits.uop.uopc,
        dut.io.req.bits.uop.rob_idx,
        dut.io.req.bits.uop.vconfig.vtype.vsew,
        dut.io.req.bits.uop.vconfig.vtype.vlmul_signed.asUInt,
        dut.io.req.bits.uop.v_eidx,
        dut.io.req.bits.uop.vconfig.vl)
    printf("     rs1 %x\n", dut.io.req.bits.rs1_data)
    printf("     rs2 %x\n", dut.io.req.bits.rs2_data)
    printf("     rs3 %x\n", dut.io.req.bits.rs3_data)
    printf("     rvm %x\n", dut.io.req.bits.rvm_data)
  }
  when(dut.io.vresp.valid) {
    printf("rsp(%x): res %x\n", dut.io.vresp.bits.uop.rob_idx, dut.io.vresp.bits.data)
  }
}

class WithVecExeUT extends Config((site, here, up) => {
  case UnitTests => (q: Parameters) => {
    implicit val p = BoomTestUtils.getBoomParameters("StcBoomConfig", "chipyard")
    Seq(
      Module(new VecExeUT(4000))
    )
  }
})

class VecExeUTConfig extends Config(new WithVecExeUT)
