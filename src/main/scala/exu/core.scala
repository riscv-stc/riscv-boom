//******************************************************************************
// Copyright (c) 2015 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// RISC-V Processor Core
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// BOOM has the following (conceptual) stages:
//   if0 - Instruction Fetch 0 (next-pc select)
//   if1 - Instruction Fetch 1 (I$ access)
//   if2 - Instruction Fetch 2 (instruction return)
//   if3 - Instruction Fetch 3 (enqueue to fetch buffer)
//   if4 - Instruction Fetch 4 (redirect from bpd)
//   dec - Decode
//   ren - Rename1
//   dis - Rename2/Dispatch
//   iss - Issue
//   rrd - Register Read
//   exe - Execute
//   mem - Memory
//   sxt - Sign-extend
//   wb  - Writeback
//   com - Commit

package boom.exu

import java.nio.file.Paths
import chisel3._
import chisel3.util._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.rocket.Instructions._
import freechips.rocketchip.rocket.{CSR, Causes, EventSet, EventSets, PRV, SuperscalarEventSets, VConfig, VType}
import freechips.rocketchip.rocket.{EventSet, EventSets, SuperscalarEventSets}
import freechips.rocketchip.util.{CoreMonitorBundle, SeqBoolBitwiseOps, Str, UIntIsOneOf}
import freechips.rocketchip.util._
import freechips.rocketchip.devices.tilelink.{CLINTConsts, PLICConsts}
import testchipip.ExtendedTracedInstruction

import boom.common._
import boom.common.MicroOpcodes._
import boom.ifu.{GlobalHistory, HasBoomFrontendParameters}
import boom.exu.FUConstants._
import boom.util._
import boom.vlsu.{VLSMicroOP, VLSUArchitecturalParams, VLSUTopBundle}

import scala.collection.mutable

import boom.ifu._

/**
 * Top level core object that connects the Frontend to the rest of the pipeline.
 */
class BoomCore(usingTrace: Boolean, vlsuparam: Option[VLSUArchitecturalParams])(implicit p: Parameters) extends BoomModule
  with HasBoomFrontendParameters // TODO: Don't add this trait
{
  val io = new freechips.rocketchip.tile.CoreBundle
  {
    val hartid = Input(UInt(hartIdLen.W))
    val interrupts = Input(new freechips.rocketchip.tile.CoreInterrupts())
    val ifu = new boom.ifu.BoomFrontendIO
    val ptw = Flipped(new freechips.rocketchip.rocket.DatapathPTWIO())
    val rocc = Flipped(new freechips.rocketchip.tile.RoCCCoreIO())
    val lsu = Flipped(new boom.lsu.LSUCoreIO)
    val ptw_tlb = new freechips.rocketchip.rocket.TLBPTWIO()
    val trace = Output(Vec(coreParams.retireWidth, new ExtendedTracedInstruction))
    val fcsr_rm = UInt(freechips.rocketchip.tile.FPConstants.RM_SZ.W)
    val vlsu: Option[VLSUTopBundle] = if(usingVector) Some(Flipped(new VLSUTopBundle(vlsuparam.get))) else None
  }
  val vlsuIO: VLSUTopBundle = io.vlsu.get
  //**********************************
  // construct all of the modules

  // Only holds integer-registerfile execution units.
  val exe_units = new boom.exu.ExecutionUnits(fpu=false)
  val jmp_unit_idx = exe_units.jmp_unit_idx
  val jmp_unit = exe_units(jmp_unit_idx)
  val csr_unit_idx = exe_units.csr_unit_idx

  // Meanwhile, the FP pipeline holds the FP issue window, FP regfile, and FP arithmetic units.
  var fp_pipeline: FpPipeline = null
  if (usingFPU) fp_pipeline = Module(new FpPipeline)

  // ********************************************************
  // Clear fp_pipeline before use
  if (usingFPU) {
    fp_pipeline.io.ll_wports := DontCare
  }

  var v_pipeline: VecPipeline = null
  //val vmupdate = Wire(Vec(vecWidth, Valid(new MicroOp)))
  //vmupdate.map(_.valid := false.B)
  //vmupdate.map(_.bits := DontCare)
  if (usingVector) {
    v_pipeline = Module(new VecPipeline)
    //v_pipeline.io.ll_wports := DontCare
    //vmupdate := v_pipeline.io.vmupdate
    v_pipeline.io.intupdate := DontCare
    v_pipeline.io.fpupdate := DontCare
  }

  val numIrfWritePorts        = exe_units.numIrfWritePorts + memWidth
  val numLlIrfWritePorts      = exe_units.numLlIrfWritePorts
  val numIrfReadPorts         = exe_units.numIrfReadPorts

  val numFastWakeupPorts      = exe_units.count(_.bypassable)
  val numAlwaysBypassable     = exe_units.count(_.alwaysBypassable)

  val numIntIssueWakeupPorts  = numIrfWritePorts + numFastWakeupPorts - numAlwaysBypassable // + memWidth for ll_wb
  val numIntRenameWakeupPorts = numIntIssueWakeupPorts
  val numFpWakeupPorts        = if (usingFPU) fp_pipeline.io.wakeups.length else 0
  val numVecWakeupPorts       = if (usingVector) v_pipeline.io.wakeups.length else 0

  val decode_units     = for (w <- 0 until decodeWidth) yield { val d = Module(new DecodeUnit); d }
  val dec_brmask_logic = Module(new BranchMaskGenerationLogic(coreWidth))
  val rename_stage     = Module(new RenameStage(coreWidth, numIntPhysRegs, numIntRenameWakeupPorts, false))
  val fp_rename_stage  = if (usingFPU) Module(new RenameStage(coreWidth, numFpPhysRegs, numFpWakeupPorts, true)) else null
  val pred_rename_stage= Module(new PredRenameStage(coreWidth, ftqSz, 1))
  val v_rename_stage   = if (usingVector) Module(new VecRenameStage(coreWidth, numVecPhysRegs, numVecWakeupPorts)) else null
  //vconfig decode and mask
  val dec_vconfigmask_logic = Module(new VconfigMaskGenerationLogic(coreWidth))

  // usingVector implies usingFPU
  val rename_stages    = if (usingVector)
      Seq(rename_stage, fp_rename_stage, v_rename_stage, pred_rename_stage)
    else if (usingFPU)
      Seq(rename_stage, fp_rename_stage, pred_rename_stage)
    else
      Seq(rename_stage, pred_rename_stage)

  rename_stage.suggestName("i_rename_stage")
  if (usingFPU) fp_rename_stage.suggestName("fp_rename_stage")
  if (usingVector) v_rename_stage.suggestName("v_rename_stage")

  val mem_iss_unit     = Module(new IssueUnitCollapsing(memIssueParam, numIntIssueWakeupPorts))
  mem_iss_unit.suggestName("mem_issue_unit")
  val int_iss_unit     = Module(new IssueUnitCollapsing(intIssueParam, numIntIssueWakeupPorts))
  int_iss_unit.suggestName("int_issue_unit")

  val issue_units      = Seq(mem_iss_unit, int_iss_unit)
  val dispatcher       = Module(new BasicDispatcher)

  val iregfile         = Module(new RegisterFileSynthesizable(
                             numIntPhysRegs,
                             numIrfReadPorts,
                             numIrfWritePorts,
                             xLen,
                             Seq.fill(memWidth) {true} ++ exe_units.bypassable_write_port_mask)) // bypassable ll_wb
  val pregfile         = Module(new RegisterFileSynthesizable(
                            ftqSz,
                            exe_units.numIrfReaders,
                            1,
                            1,
                            Seq(true))) // The jmp unit is always bypassable
  pregfile.io := DontCare // Only use the IO if enableSFBOpt

  // wb arbiter for the 0th ll writeback
  // TODO: should this be a multi-arb?
  val ll_wbarb         = Module(new Arbiter(new ExeUnitResp(xLen), 1 +
                                                                   (if (usingFPU) 1 else 0) +
                                                                   (if (usingRoCC) 1 else 0) +
                                                                   (if (usingVector) vecWidth else 0))) // for vec pipe write to int reg
  val iregister_read   = Module(new RegisterRead(
                           issue_units.map(_.issueWidth).sum,
                           exe_units.withFilter(_.readsIrf).map(_.supportedFuncUnits),
                           numIrfReadPorts,
                           exe_units.withFilter(_.readsIrf).map(x => 2),
                           exe_units.numTotalBypassPorts,
                           jmp_unit.numBypassStages,
                           xLen))
  val rob              = Module(new Rob(
                           numIrfWritePorts+numFpWakeupPorts+numVecWakeupPorts, // +memWidth for ll writebacks
                           numFpWakeupPorts))
  // Used to wakeup registers in rename and issue. ROB needs to listen to something else.
  val int_iss_wakeups  = Wire(Vec(numIntIssueWakeupPorts, Valid(new ExeUnitResp(xLen))))
  val int_ren_wakeups  = Wire(Vec(numIntRenameWakeupPorts, Valid(new ExeUnitResp(xLen))))
  val pred_wakeup  = Wire(Valid(new ExeUnitResp(1)))

  val vl_wakeup_dec = RegInit(0.U.asTypeOf(Valid(new VlWakeupResp())))
  val vl_wakeup = WireInit(0.U.asTypeOf(Valid(new VlWakeupResp())))
  val update_vtype = WireInit(0.U.asTypeOf(new VType()))
  //vl-ready wake up logic, bypass from execution unit
  vl_wakeup.valid := exe_units(csr_unit_idx).io.iresp.valid && (exe_units(csr_unit_idx).io.iresp.bits.uop.is_vsetvli || exe_units(csr_unit_idx).io.iresp.bits.uop.is_vsetivli)
  vl_wakeup.bits.vl := exe_units(csr_unit_idx).io.iresp.bits.data(maxVLMax.log2, 0)
  vl_wakeup.bits.vcq_idx := exe_units(csr_unit_idx).io.iresp.bits.uop.vcq_idx
  vl_wakeup.bits.vconfig_mask := exe_units(csr_unit_idx).io.iresp.bits.uop.vconfig_mask
  vl_wakeup.bits.vconfig_tag := exe_units(csr_unit_idx).io.iresp.bits.uop.vconfig_tag

  update_vtype := exe_units(csr_unit_idx).io.iresp.bits.uop.vconfig.vtype

  when(vl_wakeup.valid) {
    vl_wakeup_dec := vl_wakeup
  }

  when(rob.io.flush.valid) {
    vl_wakeup_dec.valid := false.B
    vl_wakeup_dec.bits := DontCare
  }

  require (exe_units.length == issue_units.map(_.issueWidth).sum)

  //***********************************
  // Pipeline State Registers and Wires

  // Decode/Rename1 Stage
  val dec_valids = Wire(Vec(coreWidth, Bool()))  // are the decoded instruction valid? It may be held up though.
  val dec_uops   = Wire(Vec(coreWidth, new MicroOp()))
  val dec_hazards= Wire(Vec(coreWidth, Bool()))
  val dec_stalls = Wire(Vec(coreWidth, Bool()))
  val dec_fe_fire= Wire(Vec(coreWidth, Bool()))  // can the instruction pop from instruction buffer
  val dec_fire   = Wire(Vec(coreWidth, Bool()))  // can the instruction fire beyond decode?
                                                    // (can still be stopped in ren or dis)
  val dec_ready  = Wire(Bool())
  val dec_xcpts  = Wire(Vec(coreWidth, Bool()))
  val ren_stalls = Wire(Vec(coreWidth, Bool()))

  //// are the decoded instruction is vconfig instruction valid? It may be held up though.
  val dec_vconfig   = Wire(Vec(coreWidth, new VconfigDecodeSignals()))
  val dec_vconfig_valid = Wire(Vec(coreWidth, Bool()))
  // stall fetch/dcode because we ran out of vconfig tags
  val vconfig_mask_full = Wire(Vec(coreWidth, Bool()))

  // Rename2/Dispatch stage
  val disValids = Wire(Vec(coreWidth, Bool()))
  val disUops   = Wire(Vec(coreWidth, new MicroOp))
  val disFire   = Wire(Vec(coreWidth, Bool()))
  val dis_fire_fb= Wire(Vec(coreWidth, Bool())) // upstream dis_fire: ren/ifu
  val dis_ready  = Wire(Bool())
  //val dis_split       = Wire(Vec(coreWidth, Bool()))
  //val dis_fired       = RegInit(VecInit(0.U(coreWidth.W).asBools))
  val disSplitFirst  = Wire(Vec(coreWidth, Bool()))
  val dis_split_last  = Wire(Vec(coreWidth, Bool()))
  val dis_split_cand  = Wire(Vec(coreWidth, Bool()))
  val dis_split_actv  = Wire(Vec(coreWidth, Bool()))
  val dis_prev_split_actv  = Wire(Vec(coreWidth, Bool()))
  val wait_for_empty_pipeline = Wire(Vec(coreWidth, Bool()))

  // Issue Stage/Register Read
  val iss_valids = Wire(Vec(exe_units.numIrfReaders, Bool()))
  val iss_uops   = Wire(Vec(exe_units.numIrfReaders, new MicroOp()))
  val bypasses   = Wire(Vec(exe_units.numTotalBypassPorts, Valid(new ExeUnitResp(xLen))))
  val pred_bypasses = Wire(Vec(jmp_unit.numBypassStages, Valid(new ExeUnitResp(1))))
  require(jmp_unit.bypassable)

  // --------------------------------------
  // Dealing with branch resolutions

  // The individual branch resolutions from each ALU
  val brinfos = Reg(Vec(coreWidth, new BrResolutionInfo()))

  // "Merged" branch update info from all ALUs
  // brmask contains masks for rapidly clearing mispredicted instructions
  // brindices contains indices to reset pointers for allocated structures
  //           brindices is delayed a cycle
  val brupdate  = Wire(new BrUpdateInfo)
  val b1    = Wire(new BrUpdateMasks)
  val b2    = Reg(new BrResolutionInfo)

  brupdate.b1 := b1
  brupdate.b2 := b2

  for ((b, a) <- brinfos zip exe_units.alu_units) {
    b := a.io.brinfo
    b.valid := a.io.brinfo.valid && !rob.io.flush.valid
  }
  b1.resolve_mask := brinfos.map(x => x.valid << x.uop.br_tag).reduce(_|_)
  b1.mispredict_mask := brinfos.map(x => (x.valid && x.mispredict) << x.uop.br_tag).reduce(_|_)

  // Find the oldest mispredict and use it to update indices
  var mispredict_val = false.B
  var oldest_mispredict = brinfos(0)
  for (b <- brinfos) {
    val use_this_mispredict = !mispredict_val ||
    b.valid && b.mispredict && IsOlder(b.uop.rob_idx, oldest_mispredict.uop.rob_idx, rob.io.rob_head_idx)

    mispredict_val = mispredict_val || (b.valid && b.mispredict)
    oldest_mispredict = Mux(use_this_mispredict, b, oldest_mispredict)
  }
  val mispredict_cnt = RegInit(0.U(64.W))
  when (mispredict_val.asBool) {
    mispredict_cnt := mispredict_cnt + 1.U
    printf("mispredict_cnt: %d\n", mispredict_cnt.asUInt())
  }
  b2.mispredict  := mispredict_val
  b2.cfi_type    := oldest_mispredict.cfi_type
  b2.taken       := oldest_mispredict.taken
  b2.pc_sel      := oldest_mispredict.pc_sel
  b2.uop         := UpdateBrMask(brupdate, oldest_mispredict.uop)
  b2.jalr_target := RegNext(jmp_unit.io.brinfo.jalr_target)
  b2.target_offset := oldest_mispredict.target_offset

  val oldest_mispredict_ftq_idx = oldest_mispredict.uop.ftq_idx


  assert (!((brupdate.b1.mispredict_mask =/= 0.U || brupdate.b2.mispredict)
    && rob.io.commit.rollback), "Can't have a mispredict during rollback.")

  io.ifu.brupdate := brupdate

  for (eu <- exe_units) {
    eu.io.brupdate := brupdate
  }

  if (usingFPU) {
    fp_pipeline.io.brupdate := brupdate
  }

  if (usingVector) {
    v_pipeline.io.brupdate := brupdate
    v_pipeline.io.vbusy_status := v_rename_stage.io.vbusy_status

    v_pipeline.io.vl_wakeup :=  vl_wakeup
  }

  // Load/Store Unit & ExeUnits
  val mem_units = exe_units.memory_units
  val mem_resps = mem_units.map(_.io.ll_iresp)
  for (i <- 0 until memWidth) {
    mem_units(i).io.lsu_io <> io.lsu.exe(i)
  }
  if(usingVector){
    val vlsuReqVld = mem_units(0).io.vlsuReqRr.valid && mem_units(0).io.vlsuReqRr.bits.uop.is_rvv &&
      (mem_units(0).io.vlsuReqRr.bits.uop.uses_ldq || mem_units(0).io.vlsuReqRr.bits.uop.uses_stq)
    /* For unmasked/unindexed vector load store that doesn't need to read vrf. */
    vlsuIO.fromRr.vuop(0).valid := vlsuReqVld
    vlsuIO.fromRr.vuop(0).bits := uopConvertToVuop(mem_units(0).io.vlsuReqRr.bits.uop)
    vlsuIO.fromRr.vuop(0).bits.rs1 := mem_units(0).io.vlsuReqRr.bits.rs1_data
    vlsuIO.fromRr.vuop(0).bits.rs2 := mem_units(0).io.vlsuReqRr.bits.rs2_data // for unmasked constant stride
    vlsuIO.fromRr.vuop(0).bits.vm := Fill(vLen, 1.U(1.W))

    /* For masked/indexed vector load store. */
    vlsuIO.fromRr.vuop(1).valid := v_pipeline.io.toVlsuRr.valid
    vlsuIO.fromRr.vuop(1).bits := uopConvertToVuop(v_pipeline.io.toVlsuRr.bits.uop)
    vlsuIO.fromRr.vuop(1).bits.rs1 := v_pipeline.io.toVlsuRr.bits.uop.v_scalar_data
    vlsuIO.fromRr.vuop(1).bits.rs2 := v_pipeline.io.toVlsuRr.bits.uop.vStrideLength
    vlsuIO.fromRr.vuop(1).bits.vs2 := v_pipeline.io.toVlsuRr.bits.rs3_data  //for indexed load/stores
    vlsuIO.fromRr.vuop(1).bits.vm := v_pipeline.io.toVlsuRr.bits.rvmFull
    vlsuIO.vrfBusyStatus := v_rename_stage.io.vbusy_status
  }

  //-------------------------------------------------------------
  // Uarch Hardware Performance Events (HPEs)
  // scalar int instructions
  val retired_int       = Wire(Vec(coreWidth, Bool()))
  val retired_load      = Wire(Vec(coreWidth, Bool()))
  val retired_store     = Wire(Vec(coreWidth, Bool()))
  val retired_amo       = Wire(Vec(coreWidth, Bool()))
  val retired_system    = Wire(Vec(coreWidth, Bool()))
  val retired_fence     = Wire(Vec(coreWidth, Bool()))
  val retired_fencei    = Wire(Vec(coreWidth, Bool()))
  val retired_branch    = Wire(Vec(coreWidth, Bool()))
  val retired_jal       = Wire(Vec(coreWidth, Bool()))
  val retired_jalr      = Wire(Vec(coreWidth, Bool()))
  val retired_alu       = Wire(Vec(coreWidth, Bool()))
  val retired_mul       = Wire(Vec(coreWidth, Bool()))
  val retired_div       = Wire(Vec(coreWidth, Bool()))
  // scalar float instuctions
  val retired_fp        = Wire(Vec(coreWidth, Bool()))
  val retired_fp_load   = Wire(Vec(coreWidth, Bool()))
  val retired_fp_store  = Wire(Vec(coreWidth, Bool()))
  val retired_fp_fpu    = Wire(Vec(coreWidth, Bool()))
  val retired_fp_div    = Wire(Vec(coreWidth, Bool()))
  // vector instructions
  val retired_rvv       = Wire(Vec(coreWidth, Bool()))
  val retired_rvv_vset  = Wire(Vec(coreWidth, Bool()))
  val retired_rvv_load  = Wire(Vec(coreWidth, Bool()))
  val retired_rvv_store = Wire(Vec(coreWidth, Bool()))
  val retired_rvv_int   = Wire(Vec(coreWidth, Bool()))
  val retired_rvv_float = Wire(Vec(coreWidth, Bool()))
  val retired_rvv_div   = Wire(Vec(coreWidth, Bool()))
  val retired_rvv_fdiv  = Wire(Vec(coreWidth, Bool()))
  val retired_rvv_red   = Wire(Vec(coreWidth, Bool()))

  val branch_cnt     = RegInit(0.U(64.W))
  val store_cnt      = RegInit(0.U(64.W))
  val load_cnt       = RegInit(0.U(64.W))
  val retire_cnt     = RegInit(0.U(64.W))

  for (w <- 0 until coreWidth) {
    val isInsRvv = rob.io.commit.valids(w) &&  rob.io.commit.uops(w).is_rvv
    val isInsFp  = rob.io.commit.valids(w) && !rob.io.commit.uops(w).is_rvv &&  rob.io.commit.uops(w).fp_val
    val isInsInt = rob.io.commit.valids(w) && !rob.io.commit.uops(w).is_rvv && !rob.io.commit.uops(w).fp_val
    // retired scalar int instructions
    retired_int(w)      := isInsInt
    retired_load(w)     := isInsInt && rob.io.commit.uops(w).uses_ldq
    retired_amo(w)      := isInsInt && rob.io.commit.uops(w).is_amo
    retired_system(w)   := isInsInt && (rob.io.commit.uops(w).ctrl.csr_cmd =/= freechips.rocketchip.rocket.CSR.N)
    retired_fence(w)    := isInsInt && rob.io.commit.uops(w).is_fence
    retired_fencei(w)   := isInsInt && rob.io.commit.uops(w).is_fencei
    retired_store(w)    := isInsInt && rob.io.commit.uops(w).uses_stq && !rob.io.commit.uops(w).is_amo && !rob.io.commit.uops(w).is_fence
    retired_branch(w)   := isInsInt && rob.io.commit.uops(w).is_br
    retired_jal(w)      := isInsInt && rob.io.commit.uops(w).is_jal
    retired_jalr(w)     := isInsInt && rob.io.commit.uops(w).is_jalr
    retired_alu(w)      := isInsInt && rob.io.commit.uops(w).fu_code.isOneOf(FU_ALU) && !rob.io.commit.uops(w).is_br
    retired_mul(w)      := isInsInt && rob.io.commit.uops(w).fu_code.isOneOf(FU_MUL)
    retired_div(w)      := isInsInt && rob.io.commit.uops(w).fu_code.isOneOf(FU_DIV)
    // retired scalar float instructions
    retired_fp(w)       := isInsFp
    retired_fp_load(w)  := isInsFp && rob.io.commit.uops(w).uses_ldq
    retired_fp_store(w) := isInsFp && rob.io.commit.uops(w).uses_stq
    retired_fp_fpu(w)   := isInsFp && rob.io.commit.uops(w).fu_code.isOneOf(FU_FPU, FU_F2I, FU_I2F)
    retired_fp_div(w)   := isInsFp && rob.io.commit.uops(w).fu_code.isOneOf(FU_FDV)
    // retired vector instructions
    retired_rvv(w)      := isInsRvv && rob.io.commit.uops(w).v_split_last
    retired_rvv_vset(w) := retired_rvv(w) && rob.io.commit.uops(w).uopc.isOneOf(uopVSETVL, uopVSETVLI, uopVSETIVLI)
    retired_rvv_load(w) := retired_rvv(w) && rob.io.commit.uops(w).uses_ldq
    retired_rvv_store(w):= retired_rvv(w) && rob.io.commit.uops(w).uses_stq
    retired_rvv_int(w)  := retired_rvv(w) && !rob.io.commit.uops(w).fp_val && !rob.io.commit.uops(w).uses_ldq && !rob.io.commit.uops(w).uses_stq && !rob.io.commit.uops(w).uopc.isOneOf(uopVSETVL, uopVSETVLI, uopVSETIVLI)
    retired_rvv_float(w):= retired_rvv(w) && rob.io.commit.uops(w).fp_val
    retired_rvv_div(w)  := retired_rvv_int(w)   && rob.io.commit.uops(w).fu_code.isOneOf(FU_DIV)
    retired_rvv_fdiv(w) := retired_rvv_float(w) && rob.io.commit.uops(w).fu_code.isOneOf(FU_FDV)
    retired_rvv_red(w)  := retired_rvv(w)       && rob.io.commit.uops(w).fu_code.isOneOf(FU_IVRP, FU_FVRP)

    when(retired_branch(w).asBool) {
      branch_cnt := branch_cnt + 1.U
      printf("branch_cnt: %d\n", branch_cnt.asUInt())
    }

    when(retired_load(w).asBool) {
      load_cnt := load_cnt + 1.U
      printf("load_cnt: %d\n", load_cnt.asUInt())
    }

    when(retired_store(w).asBool) {
      store_cnt := store_cnt + 1.U
      printf("store_cnt: %d\n", store_cnt.asUInt())
    }

    when(retired_fp_load(w).asBool) {
      load_cnt := load_cnt + 1.U
      printf("load_cnt: %d\n", load_cnt.asUInt())
    }

    when(retired_fp_store(w).asBool) {
      store_cnt := store_cnt + 1.U
      printf("store_cnt: %d\n", store_cnt.asUInt())
    }
  }

  val insnCommitBaseEvents = (0 until coreWidth).map(w => new EventSet((mask, hits) => (mask & hits).orR, Seq(
    ("int total",   () => retired_int(w)),
    ("int load",    () => retired_load(w)),
    ("int store",   () => retired_store(w)),
    ("int amo",     () => retired_amo(w)),
    ("int system",  () => retired_system(w)),
    ("int fence",   () => retired_fence(w)),
    ("int fencei",  () => retired_fencei(w)),
    ("int branch",  () => retired_branch(w)),
    ("int jal",     () => retired_jal(w)),
    ("int jalr",    () => retired_jalr(w)),
    ("int alu",     () => retired_alu(w)),
    ("int mul",     () => retired_mul(w)),
    ("int div",     () => retired_div(w)))
    ++ (if (!usingFPU) Seq() else Seq(
      ("fp total",    () => retired_fp(w)),
      ("fp load",     () => retired_fp_load(w)),
      ("fp store",    () => retired_fp_store(w)),
      ("fp fpu",      () => retired_fp_fpu(w)),
      ("fp div",      () => retired_fp_div(w))))
    ++ (if (!usingVector) Seq() else Seq(
      ("vector total",      () => retired_rvv(w)),
      ("vector vset",       () => retired_rvv_vset(w)),
      ("vector load",       () => retired_rvv_load(w)),
      ("vector store",      () => retired_rvv_store(w)),
      ("vector int",        () => retired_rvv_int(w)),
      ("vector float",      () => retired_rvv_float(w))))
  ))

  val micrArchEvents = new EventSet((mask, hits) => (mask & hits).orR, Seq(
    ("exception",                         () => rob.io.com_xcpt.valid),
    ("front-end f1 is resteered",         () => io.ifu.perf.f1_clear),
    ("front-end f2 is resteered",         () => io.ifu.perf.f2_clear),
    ("front-end f3 is resteered",         () => io.ifu.perf.f3_clear),
    ("front-end f4 is resteered",         () => io.ifu.perf.f4_clear)
  ))

  val intIssueSlotsEmpty = int_iss_unit.io.perf.empty
  val memIssueSlotsEmpty = mem_iss_unit.io.perf.empty
  val fpIssueSlotsEmpty  = fp_pipeline.io.perf.iss_slots_empty
  val vecIssueSlotsEmpty = v_pipeline.io.perf.vec_iss_slots_empty
  val vmxIssueSlotsEmpty = v_pipeline.io.perf.vmx_iss_slots_empty
  val allIssueSlotsEmpty = intIssueSlotsEmpty && memIssueSlotsEmpty && fpIssueSlotsEmpty && vecIssueSlotsEmpty && vmxIssueSlotsEmpty

  val resourceEvents = new EventSet((mask, hits) => (mask & hits).orR, Seq(
    ("frontend fb full",                  () => io.ifu.perf.fb_full),
    ("frontend fb empty",                 () => io.ifu.perf.fb_empty),
    ("frontend ftq full",                 () => io.ifu.perf.ftq_full),
    ("branch mask full",                  () => dec_brmask_logic.io.is_full.reduce(_||_)),
    ("int physical register full",        () => rename_stage.io.ren_stalls.reduce(_||_)),
    ("pred physical register full",       () => pred_rename_stage.io.ren_stalls.reduce(_||_)),
    ("fp  physical register full",        () => fp_rename_stage.io.ren_stalls.reduce(_||_)),
    ("issue slots empty",                 () => allIssueSlotsEmpty),
    ("int issue slots full",              () => int_iss_unit.io.perf.full),
    ("int issue slots empty",             () => int_iss_unit.io.perf.empty),
    ("mem issue slots full",              () => mem_iss_unit.io.perf.full),
    ("mem issue slots empty",             () => mem_iss_unit.io.perf.empty),
    ("fp issue slots full",               () => fp_pipeline.io.perf.iss_slots_full),
    ("fp issue slots empty",              () => fp_pipeline.io.perf.iss_slots_empty),
    ("load queue almost full",            () => io.lsu.ldq_full.reduce(_||_)),
    ("store queue almost full",           () => io.lsu.stq_full.reduce(_||_)),
    ("rob entry empty",                   () => rob.io.perf.empty),
    ("rob entry full",                    () => rob.io.perf.full)
  ))

  val memorySystemEvents = new EventSet((mask, hits) => (mask & hits).orR, Seq(
    ("I$ loads",             () => false.B),
    ("I$ load miss",         () => io.ifu.perf.acquire),
    ("I$ prefetches",        () => false.B),
    ("I$ prefetches miss",   () => false.B),
    ("D$ loads",             () => false.B),
    ("D$ load miss",         () => io.lsu.perf.acquire),
    ("D$ stores",            () => false.B),
    ("D$ store miss",        () => false.B),
    ("D$ prefetches",        () => false.B),
    ("D$ prefetch misses",   () => false.B),
    ("D$ release",           () => io.lsu.perf.release),
    ("ITLB loads",           () => false.B),
    ("ITLB load miss",       () => io.ifu.perf.tlbMiss),
    ("ITLB prefetches",      () => false.B),
    ("ITLB prefetch misses", () => false.B),
    ("DTLB loads",           () => false.B),
    ("DTLB load miss",       () => io.lsu.perf.tlbMiss),
    ("DTLB stores",          () => false.B),
    ("DTLB store miss",      () => false.B),
    ("DTLB prefetches",      () => false.B),
    ("DTLB prefetch misses", () => false.B),
    ("L2 hits",              () => false.B),
    ("L2 misses",            () => false.B),
    ("L2 loads",             () => false.B),
    ("L2 load miss",         () => false.B),
    ("L2 stores",            () => false.B),
    ("L2 store miss",        () => false.B),
    ("L2 prefetches",        () => false.B),
    ("L2 prefetches miss",   () => false.B),
    ("L2 TLB load",          () => false.B),
    ("L2 TLB miss",          () => io.ptw.perf.l2miss),
    ("L2 TLB stores",        () => false.B),
    ("L2 TLB store miss",    () => false.B),
    ("MSHR reuse",           () => io.lsu.perf.mshrs_reuse),
    ("MSHR load establish",  () => io.lsu.perf.mshrs_load_establish),
    ("MSHR load reuse",      () => io.lsu.perf.mshrs_load_reuse),
    ("MSHR store establish", () => io.lsu.perf.mshrs_store_establish),
    ("MSHR store reuse",     () => io.lsu.perf.mshrs_store_reuse)
  ))

  // split at ifu-fetuchBuffer < - > decode
  val backend_stall   = dec_hazards.reduce(_||_)
  val backend_nostall = !backend_stall

  val uopsDelivered_sum_leN = Wire(Vec(coreWidth, Bool()))
  val uopsDelivered_sum = PopCount(dec_fire)
  (0 until coreWidth).map(n => uopsDelivered_sum_leN(n) := (uopsDelivered_sum <= n.U) && backend_nostall)
  val uopsDelivered_le_events: Seq[(String, () => Bool)] = uopsDelivered_sum_leN.zipWithIndex.map{case(v,i) => ("less than or equal to $i uops delivered", () => v)}
  val uopsDelivered_stall = uopsDelivered_sum_leN(0)

  val uopsDispatched_valids = rob.io.enq_valids
  val uopsDispatched_stall  = !uopsDispatched_valids.reduce(_||_)

  val mem_iss_valids = mem_iss_unit.io.iss_valids
  val int_iss_valids = int_iss_unit.io.iss_valids
  val fp_iss_valids  = fp_pipeline.io.perf.iss_valids
  val vec_iss_valids = v_pipeline.io.perf.vec_iss_valids
  val vmx_iss_valids = v_pipeline.io.perf.vmx_iss_valids

  val uopsIssued_valids  = mem_iss_valids ++ int_iss_valids ++ fp_iss_valids ++ vec_iss_valids ++ vmx_iss_valids
  val issueWidthSum      = issueParams.map(_.issueWidth).sum
  val uopsIssued_sum_leN = Wire(Vec(issueWidthSum, Bool()))
  val uopsIssued_sum     = PopCount(uopsIssued_valids)
  (0 until issueWidthSum).map(n => uopsIssued_sum_leN(n) := (uopsIssued_sum <= n.U) && ~allIssueSlotsEmpty)
  val uopsIssued_le_events: Seq[(String, () => Bool)] = uopsIssued_sum_leN.zipWithIndex.map{case(v,i) => ("less than or equal to $i uops issued", () => v)}

  val uopsIssued_stall = uopsIssued_sum_leN(1)
  val uopsIssued_stall_on_loads  = uopsIssued_stall &&  io.lsu.perf.in_flight_load
  val uopsIssued_stall_on_stores = uopsIssued_stall && !io.lsu.perf.in_flight_load && io.lsu.perf.stq_full
  //val uopsIssued_stall_on_loads   = uopsIssued_stall && io.lsu.perf.ldq_nonempty && rob.io.perf.com_load_is_at_rob_head
  //val uopsIssued_stall_on_stores  = uopsIssued_stall && io.lsu.perf.stq_full && (!io.lsu.perf.ldq_nonempty || !rob.io.perf.com_load_is_at_rob_head)

  val uopsExeActive_valids = if(usingVector)
      exe_units.map(u => u.io.req.valid) ++ fp_pipeline.io.perf.exe_units_req_valids ++ v_pipeline.io.perf.vec_req_valids ++ v_pipeline.io.perf.vmx_req_valids
    else if (usingFPU)
      exe_units.map(u => u.io.req.valid) ++ fp_pipeline.io.perf.exe_units_req_valids
    else
      exe_units.map(u => u.io.req.valid)
  val uopsExeActive_events: Seq[(String, () => Bool)] = uopsExeActive_valids.zipWithIndex.map{case(v,i) => ("Excution unit $i active cycle", () => v)}

  val uopsExecuted_valids = rob.io.wb_resps.map(r => r.valid)
  val uopsExecuted_sum_geN = Wire(Vec(rob.numWakeupPorts, Bool()))
  val uopsExecuted_sum_leN = Wire(Vec(rob.numWakeupPorts, Bool()))
  val uopsExecuted_sum = PopCount(uopsExecuted_valids)
  (0 until rob.numWakeupPorts).map(n => uopsExecuted_sum_geN(n) := (uopsExecuted_sum >= (n.U+1.U)) && ~allIssueSlotsEmpty)
  val uopsExecuted_ge_events: Seq[(String, () => Bool)] = uopsExecuted_sum_geN.zipWithIndex.map{case(v,i) => ("more than ${i+1} uops executed", () => v)}
  (0 until rob.numWakeupPorts).map(n => uopsExecuted_sum_leN(n) := (uopsExecuted_sum <= n.U) && ~allIssueSlotsEmpty)
  val uopsExecuted_le_events: Seq[(String, () => Bool)] = uopsExecuted_sum_leN.zipWithIndex.map{case(v,i) => ("less than or equal to $i uops executed", () => v)}
  val uopsExecuted_stall = uopsExecuted_sum_leN(0)
  val uopsExecuted_stall_on_loads   = uopsExecuted_stall && io.lsu.perf.ldq_nonempty && rob.io.perf.com_load_is_at_rob_head
  val uopsExecuted_stall_on_stores  = uopsExecuted_stall && io.lsu.perf.stq_full && (~uopsExecuted_stall_on_loads)

  val uopsRetired_valids = rob.io.commit.valids
  val uopsRetired_sum_leN = Wire(Vec(rob.retireWidth, Bool()))
  val uopsRetired_sum = PopCount(uopsRetired_valids)
  (0 until rob.retireWidth).map(n => uopsRetired_sum_leN(n) := (uopsRetired_sum <= n.U) && ~rob.io.perf.empty)
  val uopsRetired_le_events: Seq[(String, () => Bool)] = uopsRetired_sum_leN.zipWithIndex.map{case(v,i) => ("less than or equal to $i uops Retired", () => v)}
  val uopsRetired_stall  = uopsRetired_sum_leN(0)

  val bad_resteers_stat = RegInit(false.B)
  when(io.ifu.redirect_flush || io.ifu.sfence.valid){
    bad_resteers_stat := true.B
  } .elsewhen(dec_fire.reduce(_||_)){
    bad_resteers_stat:= false.B
  }

  val resource_allocator_recovery_stat = RegInit(false.B)
  when(brupdate.b2.mispredict){
    resource_allocator_recovery_stat := true.B
  } .elsewhen(uopsIssued_valids.reduce(_||_)){
    resource_allocator_recovery_stat := false.B
  }


  val br_insn_retired_cond_ntaken = Wire(Vec(coreWidth, Bool()))
  val br_insn_retired_cond_taken = Wire(Vec(coreWidth, Bool()))
  br_insn_retired_cond_taken  := (0 until coreWidth).map(w => retired_branch(w) && rob.io.commit.uops(w).taken)
  br_insn_retired_cond_ntaken := (0 until coreWidth).map(w => retired_branch(w) && ~rob.io.commit.uops(w).taken)

  val divider_actives = if(usingVector)
      exe_units.map(e => if(e.hasDiv) e.io.perf.div_busy else false.B) ++ fp_pipeline.io.perf.fdiv_busy ++ v_pipeline.io.perf.div_busy ++ v_pipeline.io.perf.fdiv_busy
    else if(usingFPU)
      exe_units.map(e => if(e.hasDiv) e.io.perf.div_busy else false.B) ++ fp_pipeline.io.perf.fdiv_busy
    else
      exe_units.map(e => if(e.hasDiv) e.io.perf.div_busy else false.B)
  val divider_active_events: Seq[(String, () => Bool)] = Seq(("cycles when divider unit is busy", () => divider_actives.orR))


  val topDownslotsVec = (0 until coreWidth).map(w => new EventSet((mask, hits) => (mask & hits).orR, Seq(
    ("slots issued",                      () => dec_fire(w)),
    ("fetch bubbles",                     () => !dec_fire(w) && !dec_stalls(w)),
    ("branch instruction retired",        () => retired_branch(w)),
    ("taken conditional branch instructions retired",     () => br_insn_retired_cond_taken(w)),
    ("not taken conditional branch instructions retired", () => br_insn_retired_cond_ntaken(w)),
    ("Counts the number of dispatched",   () => uopsDispatched_valids(w)),
    ("Counts the number of retirement",   () => uopsRetired_valids(w)),
    ("retirement bubbles",                () => ~uopsRetired_valids(w))
  )))

  val topDownIssVec = (0 until issueParams.map(_.issueWidth).sum).map(w => new EventSet((mask, hits) => (mask & hits).orR, Seq(
    ("issued uops sum",            () => uopsIssued_valids(w)),
    ("exe active sum",             () => uopsExeActive_valids(w))
    )))

  val topDownWBVec = (0 until rob.numWakeupPorts).map(w => new EventSet((mask, hits) => (mask & hits).orR, Seq(
    ("executed sum",               () => uopsExecuted_valids(w))
    )))

  val mem_stall_l1d_miss = uopsIssued_stall && io.lsu.perf.in_flight_load && io.lsu.perf.mshrs_has_busy
  val mem_stall_l2_miss  = false.B
  val mem_stall_dram     = false.B

  val resource_any_stalls = backend_stall
  val resource_rob_stalls = !rob.io.perf.ready
  val resource_issueslots_stalls = !dispatcher.io.ren_uops.map(u => u.ready).reduce(_||_)

  val br_misp_retired = brupdate.b2.mispredict
  val br_misp_target = br_misp_retired && oldest_mispredict.cfi_type === CFI_JALR
  val br_misp_dir    = br_misp_retired && oldest_mispredict.cfi_type === CFI_BR
  val br_misp_retired_cond_taken  = br_misp_dir && brupdate.b2.taken
  val br_misp_retired_cond_ntaken = br_misp_dir && (~brupdate.b2.taken)

  val topDownCyclesEvents0 = new EventSet((mask, hits) => (mask & hits).orR, Seq(
    ("bad speculation resteers cycle",     () => io.ifu.perf.badResteers),
    ("recovery cycle",                     () => resource_allocator_recovery_stat),
    ("frontend unkowns branched resteers", () => io.ifu.perf.unknownsBranchCycles),
    ("branch mispred retired",             () => brupdate.b2.mispredict),
    ("machine clears",                     () => rob.io.flush.valid),
    ("icache stalls cycles",               () => io.ifu.perf.iCache_stalls),
    ("iTLB stalls cycles",                 () => io.ifu.perf.iTLB_stalls),
    ("any load mem stall",                 () => uopsIssued_stall_on_loads),
    ("stores mem stall",                   () => uopsIssued_stall_on_stores),
    ("l1d miss mem stall",                 () => mem_stall_l1d_miss),
    ("l2 miss mem stall",                  () => mem_stall_l2_miss),
    ("dram mem stall",                     () => mem_stall_dram),
    ("mem latency",                        () => false.B),
    ("resource stall",                     () => resource_any_stalls),
    ("issueslots stall",                   () => resource_issueslots_stalls),
    ("rob unit cause excution stall",      () => resource_rob_stalls),
    ("control-flow target misprediction",  () => br_misp_target),
    ("mispredicted conditional branch instructions retired", () => br_misp_dir),
    ("taken conditional mispredicted branch instructions retired", () => br_misp_retired_cond_taken),
    ("not taken conditional mispredicted branch instructions retired", () => br_misp_retired_cond_ntaken),
    ("not actually retired uops",          () => uopsRetired_stall)
  ))

  val topDownCyclesEvents1 = new EventSet((mask, hits) => (mask & hits).orR,
       uopsDelivered_le_events            // coreWidth
    ++ uopsIssued_le_events               // exu num
    ++ uopsExeActive_events               // exu num
    ++ divider_active_events              // divider busy
    ++ uopsExecuted_le_events             // rob.numWakeupPorts
    ++ uopsExecuted_ge_events             // rob.numWakeupPorts
    ++ uopsRetired_le_events              // rob.retireWidth
  )

  val perfEvents = new SuperscalarEventSets(Seq(
    (topDownslotsVec,           (m, n) => m +& n),
    (Seq(topDownCyclesEvents0), (m, n) => m +& n),
    (Seq(topDownCyclesEvents1), (m, n) => m +& n),
    (topDownIssVec,             (m, n) => m +& n),
    (topDownWBVec,              (m, n) => m +& n),
    (insnCommitBaseEvents,      (m, n) => m +& n),
    (Seq(micrArchEvents),       (m, n) => m +& n),
    (Seq(resourceEvents),       (m, n) => m +& n),
    (Seq(memorySystemEvents),   (m, n) => m +& n)
  ))

  val csr = Module(new freechips.rocketchip.rocket.CSRFile(perfEvents.toScalarEventSets, boomParams.customCSRs.decls))
  csr.io.inst foreach { c => c := DontCare }
  csr.io.rocc_interrupt := io.rocc.interrupt

  val custom_csrs = Wire(new BoomCustomCSRs)
  (custom_csrs.csrs zip csr.io.customCSRs).map { case (lhs, rhs) => lhs := rhs }

  //val icache_blocked = !(io.ifu.fetchpacket.valid || RegNext(io.ifu.fetchpacket.valid))
  val icache_blocked = false.B
  csr.io.counters foreach { c => c.inc := RegNext(perfEvents.evaluate(c.eventSel)) }

  //****************************************
  // Time Stamp Counter & Retired Instruction Counter
  // (only used for printf and vcd dumps - the actual counters are in the CSRFile)
  val debug_tsc_reg = RegInit(0.U(xLen.W))
  val debug_irt_reg = RegInit(0.U(xLen.W))
  val debug_brs     = Reg(Vec(4, UInt(xLen.W)))
  val debug_jals    = Reg(Vec(4, UInt(xLen.W)))
  val debug_jalrs   = Reg(Vec(4, UInt(xLen.W)))

  for (j <- 0 until 4) {
    debug_brs(j) := debug_brs(j) + PopCount(VecInit((0 until coreWidth) map {i =>
      rob.io.commit.arch_valids(i) &&
      (rob.io.commit.uops(i).debug_fsrc === j.U) &&
      rob.io.commit.uops(i).is_br
    }))
    debug_jals(j) := debug_jals(j) + PopCount(VecInit((0 until coreWidth) map {i =>
      rob.io.commit.arch_valids(i) &&
      (rob.io.commit.uops(i).debug_fsrc === j.U) &&
      rob.io.commit.uops(i).is_jal
    }))
    debug_jalrs(j) := debug_jalrs(j) + PopCount(VecInit((0 until coreWidth) map {i =>
      rob.io.commit.arch_valids(i) &&
      (rob.io.commit.uops(i).debug_fsrc === j.U) &&
      rob.io.commit.uops(i).is_jalr
    }))
  }

  dontTouch(debug_brs)
  dontTouch(debug_jals)
  dontTouch(debug_jalrs)

  debug_tsc_reg := debug_tsc_reg + Mux(O3PIPEVIEW_PRINTF.B, O3_CYCLE_TIME.U, 1.U)
  debug_irt_reg := debug_irt_reg + PopCount(rob.io.commit.arch_valids.asUInt)
  dontTouch(debug_tsc_reg)
  dontTouch(debug_irt_reg)

  //****************************************
  // Print-out information about the machine

  val issStr =
    if (enableAgePriorityIssue) " (Age-based Priority)"
    else " (Unordered Priority)"

  // val btbStr =
  //   if (enableBTB) ("" + boomParams.btb.nSets * boomParams.btb.nWays + " entries (" + boomParams.btb.nSets + " x " + boomParams.btb.nWays + " ways)")
  //   else 0
  val btbStr = ""

  val fpPipelineStr =
    if (usingFPU) fp_pipeline.toString
    else ""

  override def toString: String =
    (BoomCoreStringPrefix("====Overall Core Params====") + "\n"
    + exe_units.toString + "\n"
    + fpPipelineStr + "\n"
    + rob.toString + "\n"
    + BoomCoreStringPrefix(
        "===Other Core Params===",
        "Fetch Width           : " + fetchWidth,
        "Decode Width          : " + coreWidth,
        "Issue Width           : " + issueParams.map(_.issueWidth).sum,
        "ROB Size              : " + numRobEntries,
        "Issue Window Size     : " + issueParams.map(_.numEntries) + issStr,
        "Load/Store Unit Size  : " + numLdqEntries + "/" + numStqEntries,
        "Num Int Phys Registers: " + numIntPhysRegs,
        "Num FP  Phys Registers: " + numFpPhysRegs,
        "Num Vec Phys Registers: " + numVecPhysRegs,
        "Max Branch Count      : " + maxBrCount)
    + iregfile.toString + "\n"
    + BoomCoreStringPrefix(
        "Num Slow Wakeup Ports : " + numIrfWritePorts,
        "Num Fast Wakeup Ports : " + exe_units.count(_.bypassable),
        "Num Bypass Ports      : " + exe_units.numTotalBypassPorts) + "\n"
    + BoomCoreStringPrefix(
        "DCache Ways           : " + dcacheParams.nWays,
        "DCache Sets           : " + dcacheParams.nSets,
        "DCache nMSHRs         : " + dcacheParams.nMSHRs,
        "ICache Ways           : " + icacheParams.nWays,
        "ICache Sets           : " + icacheParams.nSets,
        "D-TLB Ways            : " + dcacheParams.nTLBWays,
        "I-TLB Ways            : " + icacheParams.nTLBWays,
        "Paddr Bits            : " + paddrBits,
        "Vaddr Bits            : " + vaddrBits) + "\n"
    + BoomCoreStringPrefix(
        "Using FPU Unit?       : " + usingFPU.toString,
        "Using Vector?         : " + usingVector.toString,
        "VLEN Bits             : " + vLen,
        "ELEN Bits             : " + eLen,
        "Using FDivSqrt?       : " + usingFDivSqrt.toString,
        "Using VM?             : " + usingVM.toString) + "\n")

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Fetch Stage/Frontend ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------
  io.ifu.redirect_val         := false.B
  io.ifu.redirect_flush       := false.B

  // Breakpoint info
  io.ifu.status  := csr.io.status
  io.ifu.bp      := csr.io.bp
  io.ifu.mcontext := csr.io.mcontext
  io.ifu.scontext := csr.io.scontext
  io.ifu.tsc_reg := debug_tsc_reg

  io.ifu.flush_icache := (0 until coreWidth).map { i =>
    (rob.io.commit.arch_valids(i) && rob.io.commit.uops(i).is_fencei) ||
    (RegNext(dec_valids(i) && dec_uops(i).is_jalr && csr.io.status.debug))
  }.reduce(_||_)

  // TODO FIX THIS HACK
  // The below code works because of two quirks with the flush mechanism
  //  1 ) All flush_on_commit instructions are also is_unique,
  //      In the future, this constraint will be relaxed.
  //  2 ) We send out flush signals one cycle after the commit signal. We need to
  //      mux between one/two cycle delay for the following cases:
  //       ERETs are reported to the CSR two cycles before we send the flush
  //       Exceptions are reported to the CSR on the cycle we send the flush
  // This discrepency should be resolved elsewhere.
  when (RegNext(rob.io.flush.valid)) {
    io.ifu.redirect_val   := true.B
    io.ifu.redirect_flush := true.B
    val flush_typ = RegNext(rob.io.flush.bits.flush_typ)
    // Clear the global history when we flush the ROB (exceptions, AMOs, unique instructions, etc.)
    val new_ghist = WireInit((0.U).asTypeOf(new GlobalHistory))
    new_ghist.current_saw_branch_not_taken := true.B
    new_ghist.ras_idx := io.ifu.get_pc(0).entry.ras_idx
    io.ifu.redirect_ghist := new_ghist
    when (FlushTypes.useCsrEvec(flush_typ)) {
      io.ifu.redirect_pc  := Mux(flush_typ === FlushTypes.eret,
                                 RegNext(RegNext(csr.io.evec)),
                                 csr.io.evec)
    } .otherwise {
      val flush_pc = (AlignPCToBoundary(io.ifu.get_pc(0).pc, icBlockBytes)
                      + RegNext(rob.io.flush.bits.pc_lob)
                      - Mux(RegNext(rob.io.flush.bits.edge_inst), 2.U, 0.U))
      val flush_pc_next = flush_pc + Mux(RegNext(rob.io.flush.bits.is_rvc), 2.U, 4.U)
      io.ifu.redirect_pc := Mux(FlushTypes.useSamePC(flush_typ),
                                flush_pc, flush_pc_next)

    }
    io.ifu.redirect_ftq_idx := RegNext(rob.io.flush.bits.ftq_idx)
  } .elsewhen (brupdate.b2.mispredict && !RegNext(rob.io.flush.valid)) {
    val block_pc = AlignPCToBoundary(io.ifu.get_pc(1).pc, icBlockBytes)
    val uop_maybe_pc = block_pc | brupdate.b2.uop.pc_lob
    val npc = uop_maybe_pc + Mux(brupdate.b2.uop.is_rvc || brupdate.b2.uop.edge_inst, 2.U, 4.U)
    val jal_br_target = Wire(UInt(vaddrBitsExtended.W))
    jal_br_target := (uop_maybe_pc.asSInt + brupdate.b2.target_offset +
      (Fill(vaddrBitsExtended-1, brupdate.b2.uop.edge_inst) << 1).asSInt).asUInt
    val bj_addr = Mux(brupdate.b2.cfi_type === CFI_JALR, brupdate.b2.jalr_target, jal_br_target)
    val mispredict_target = Mux(brupdate.b2.pc_sel === PC_PLUS4, npc, bj_addr)
    io.ifu.redirect_val     := true.B
    io.ifu.redirect_pc      := mispredict_target
    io.ifu.redirect_flush   := true.B
    io.ifu.redirect_ftq_idx := brupdate.b2.uop.ftq_idx
    val use_same_ghist = (brupdate.b2.cfi_type === CFI_BR &&
                          !brupdate.b2.taken &&
                          bankAlign(block_pc) === bankAlign(npc))
    val ftq_entry = io.ifu.get_pc(1).entry
    val cfi_idx = (brupdate.b2.uop.pc_lob ^
      Mux(ftq_entry.start_bank === 1.U, 1.U << log2Ceil(bankBytes), 0.U))(log2Ceil(fetchWidth), 1)
    val ftq_ghist = io.ifu.get_pc(1).ghist
    val next_ghist = ftq_ghist.update(
      ftq_entry.br_mask.asUInt,
      brupdate.b2.taken,
      brupdate.b2.cfi_type === CFI_BR,
      cfi_idx,
      true.B,
      io.ifu.get_pc(1).pc,
      ftq_entry.cfi_is_call && ftq_entry.cfi_idx.bits === cfi_idx,
      ftq_entry.cfi_is_ret  && ftq_entry.cfi_idx.bits === cfi_idx)


    io.ifu.redirect_ghist   := Mux(
      use_same_ghist,
      ftq_ghist,
      next_ghist)
    io.ifu.redirect_ghist.current_saw_branch_not_taken := use_same_ghist
  } .elsewhen (rob.io.flush_frontend || brupdate.b1.mispredict_mask =/= 0.U) {
    io.ifu.redirect_flush   := true.B
  }

  // Tell the FTQ it can deallocate entries by passing youngest ftq_idx.
  val youngest_com_idx = (coreWidth-1).U - PriorityEncoder(rob.io.commit.valids.reverse)
  io.ifu.commit.valid := rob.io.commit.valids.reduce(_|_) || rob.io.com_xcpt.valid
  io.ifu.commit.bits  := Mux(rob.io.com_xcpt.valid,
                             rob.io.com_xcpt.bits.ftq_idx,
                             rob.io.commit.uops(youngest_com_idx).ftq_idx)

  assert(!(rob.io.commit.valids.reduce(_|_) && rob.io.com_xcpt.valid),
    "ROB can't commit and except in same cycle!")
  val sfence_take_pc = Wire(Bool())
  sfence_take_pc := false.B
  for (i <- 0 until memWidth) {
    when (RegNext(io.lsu.exe(i).req.bits.sfence.valid)) {
      io.ifu.sfence := RegNext(io.lsu.exe(i).req.bits.sfence)
      sfence_take_pc := true.B
    }
  }

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Branch Prediction ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Decode Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // track mask of finished instructions in the bundle
  // use this to mask out insts coming from FetchBuffer that have been finished
  // for example, back pressure may cause us to only issue some instructions from FetchBuffer
  // but on the next cycle, we only want to retry a subset
  val dec_finished_mask = RegInit(0.U(coreWidth.W))

  //-------------------------------------------------------------
  // Pull out instructions and send to the Decoders

  io.ifu.fetchpacket.ready := dec_ready
  val dec_fbundle = io.ifu.fetchpacket.bits

  //-------------------------------------------------------------
  // Decoders
  val vcq_data = Wire(new VconfigDecodeSignals())
  val vcq_empty = WireInit(false.B)
  //val vcq_idx = Wire(UInt(log2Ceil(vcqSz).W))

  for (w <- 0 until coreWidth) {
    dec_valids(w)                      := io.ifu.fetchpacket.valid && dec_fbundle.uops(w).valid &&
                                          !dec_finished_mask(w)
    decode_units(w).io.enq.uop         := dec_fbundle.uops(w).bits
    decode_units(w).io.deq_fire        := dec_fire(w)
    decode_units(w).io.kill            := io.ifu.redirect_flush
    decode_units(w).io.status          := csr.io.status
    decode_units(w).io.csr_decode      <> csr.io.decode(w)
    decode_units(w).io.interrupt       := csr.io.interrupt
    decode_units(w).io.interrupt_cause := csr.io.interrupt_cause
    if (usingVector) {
      decode_units(w).io.csr_vstart       := csr.io.vector.get.vstart
      decode_units(w).io.csr_vconfig      := vcq_data.vconfig
      decode_units(w).io.enq.uop.vl_ready := vcq_data.vl_ready

      decode_units(w).io.csr_vconfig.vtype.reserved := DontCare

      dec_vconfig_valid(w)              := dec_valids(w) && (dec_fbundle.uops(w).bits.inst(6,0) === 87.U) && (dec_fbundle.uops(w).bits.inst(14,12) === 7.U) && ((dec_fbundle.uops(w).bits.inst(31,30) === 3.U) || !dec_fbundle.uops(w).bits.inst(31))
      dec_vconfig(w).vconfig.vl         := Mux(dec_fbundle.uops(w).bits.inst(31), dec_fbundle.uops(w).bits.inst(19,15), VType.fromUInt(dec_fbundle.uops(w).bits.inst(27,20)).vlMax)
      dec_vconfig(w).vconfig.vtype      := VType.fromUInt(dec_fbundle.uops(w).bits.inst(27,20))
      dec_vconfig(w).vl_ready           := (dec_fbundle.uops(w).bits.inst(19,15) === 0.U && dec_fbundle.uops(w).bits.inst(11,7)  =/= 0.U) || dec_fbundle.uops(w).bits.inst(31)

    }

    dec_uops(w) := decode_units(w).io.deq.uop
  }

  for (w <- 0 until coreWidth) {
    when(vl_wakeup_dec.valid && (vl_wakeup_dec.bits.vconfig_tag + 1.U) === dec_uops(w).vconfig_tag) {
      dec_uops(w).vl_ready := true.B
      dec_uops(w).vconfig.vl := vl_wakeup_dec.bits.vl
    }
  }

  // Vconfig Mask Logic
  dec_vconfigmask_logic.io.vconfig_mask_update := RegNext(Mux(rob.io.commit.valids.head & rob.io.commit.uops.head.is_vsetvli, rob.io.commit.uops.head.vconfig_mask,
                                                          Mux(rob.io.commit.valids.last & rob.io.commit.uops.last.is_vsetvli, rob.io.commit.uops.last.vconfig_mask, 0.U)))
  dec_vconfigmask_logic.io.flush_pipeline := RegNext(rob.io.flush.valid) || io.ifu.redirect_flush
  vconfig_mask_full := dec_vconfigmask_logic.io.is_full

  for (w <- 0 until coreWidth) {
    dec_vconfigmask_logic.io.is_vconfig(w) := !dec_finished_mask(w) && (dec_uops(w).is_vsetvli && !(dec_uops(w).ldst === 0.U && dec_uops(w).lrs1 === 0.U) || dec_uops(w).is_vsetivli)
    dec_vconfigmask_logic.io.will_fire(w)  := dec_fire(w) && (dec_uops(w).is_vsetvli && !(dec_uops(w).ldst === 0.U && dec_uops(w).lrs1 === 0.U) || dec_uops(w).is_vsetivli)
    //dec_uops(w).vconfig_tag := dec_vconfigmask_logic.io.vconfig_tag(w)
    dec_uops(w).vconfig_mask := dec_vconfigmask_logic.io.vconfig_mask(w)
  }

  dec_uops.head.vconfig_tag := dec_vconfigmask_logic.io.vconfig_tag.head
  dec_uops.last.vconfig_tag := dec_vconfigmask_logic.io.vconfig_tag.head + (dec_vconfigmask_logic.io.is_vconfig.head && dec_vconfigmask_logic.io.will_fire.head).asUInt()

  //vconfig instruction decode info enq to VCQ
  val vcq = Module(new VconfigQueue())
    vcq.io.enq.bits  := Mux(dec_vconfig_valid.last, dec_vconfig.last, dec_vconfig.head)
    vcq.io.enq.valid := dec_vconfig_valid.reduce(_||_) && !io.ifu.redirect_flush
    vcq.io.deq       := RegNext((rob.io.commit.valids zip rob.io.commit.uops).map{case(v,u) => Mux(v, u.is_vsetivli||u.is_vsetvli, false.B)}.reduce(_ | _))
    vcq.io.flush     := RegNext(rob.io.flush.valid) || io.ifu.redirect_flush
    vcq_empty        := vcq.io.empty

    vcq_data.vconfig := Mux(dec_vconfig_valid.head, dec_vconfig.head.vconfig, Mux(vcq_empty, csr.io.vector.get.vconfig, vcq.io.get_vconfig.vconfig))

  //vcq_data.head.vl_ready := Mux(vcq_empty, true.B, vcq.io.get_vconfig.vl_ready)
    vcq_data.vl_ready := Mux(dec_vconfig_valid.head, dec_vconfig.head.vl_ready, Mux(vcq_empty, true.B, vcq.io.get_vconfig.vl_ready))

    vcq.io.update_vl.valid := vl_wakeup.valid
    vcq.io.update_vl.bits.vl_ready := vl_wakeup.valid
    vcq.io.update_vl.bits.vconfig.vl := vl_wakeup.bits.vl
    vcq.io.update_vl.bits.vconfig.vtype := update_vtype
    vcq.io.update_vl_idx := vl_wakeup.bits.vcq_idx

    dec_uops.map(d => d.vcq_idx := vcq.io.enq_idx)
  //-------------------------------------------------------------
  // FTQ GetPC Port Arbitration

  val jmp_pc_req  = Wire(Decoupled(UInt(log2Ceil(ftqSz).W)))
  val xcpt_pc_req = Wire(Decoupled(UInt(log2Ceil(ftqSz).W)))
  val flush_pc_req = Wire(Decoupled(UInt(log2Ceil(ftqSz).W)))

  val ftq_arb = Module(new Arbiter(UInt(log2Ceil(ftqSz).W), 3))

  // Order by the oldest. Flushes come from the oldest instructions in pipe
  // Decoding exceptions come from youngest
  ftq_arb.io.in(0) <> flush_pc_req
  ftq_arb.io.in(1) <> jmp_pc_req
  ftq_arb.io.in(2) <> xcpt_pc_req

  // Hookup FTQ
  io.ifu.get_pc(0).ftq_idx := ftq_arb.io.out.bits
  ftq_arb.io.out.ready  := true.B

  // Branch Unit Requests (for JALs) (Should delay issue of JALs if this not ready)
  jmp_pc_req.valid := RegNext(iss_valids(jmp_unit_idx) && iss_uops(jmp_unit_idx).fu_code === FU_JMP)
  jmp_pc_req.bits  := RegNext(iss_uops(jmp_unit_idx).ftq_idx)

  jmp_unit.io.get_ftq_pc := DontCare
  jmp_unit.io.get_ftq_pc.pc               := io.ifu.get_pc(0).pc
  jmp_unit.io.get_ftq_pc.entry            := io.ifu.get_pc(0).entry
  jmp_unit.io.get_ftq_pc.next_val         := io.ifu.get_pc(0).next_val
  jmp_unit.io.get_ftq_pc.next_pc          := io.ifu.get_pc(0).next_pc


  // Frontend Exception Requests
  val xcpt_idx = PriorityEncoder(dec_xcpts)
  xcpt_pc_req.valid    := dec_xcpts.reduce(_||_)
  xcpt_pc_req.bits     := dec_uops(xcpt_idx).ftq_idx
  //rob.io.xcpt_fetch_pc := RegEnable(io.ifu.get_pc.fetch_pc, dis_ready)
  rob.io.xcpt_fetch_pc := io.ifu.get_pc(0).pc
  if (usingVector) {
    rob.io.csrVConfig := csr.io.vector.get.vconfig
  }

  flush_pc_req.valid   := rob.io.flush.valid
  flush_pc_req.bits    := rob.io.flush.bits.ftq_idx

  // Mispredict requests (to get the correct target)
  io.ifu.get_pc(1).ftq_idx := oldest_mispredict_ftq_idx


  //-------------------------------------------------------------
  // Decode/Rename1 pipeline logic

  dec_xcpts := dec_uops zip dec_valids map {case (u,v) => u.exception && v}
  val dec_xcpt_stall = dec_xcpts.reduce(_||_) && !xcpt_pc_req.ready
  // stall fetch/dcode because we ran out of branch tags
  val branch_mask_full = Wire(Vec(coreWidth, Bool()))

  val dec_enq_stalls = decode_units.zipWithIndex.map{
    case (d,i) => dec_valids(i) && d.io.enq_stall
  }.scanLeft(false.B) ((s,h) => s || h).takeRight(coreWidth)
  val dec_prev_enq_stalls = Wire(Vec(coreWidth, Bool()))
  dec_prev_enq_stalls(0) := false.B
  (1 until coreWidth).map(w => dec_prev_enq_stalls(w) := dec_enq_stalls(w-1))

  dec_hazards     := (0 until coreWidth).map(w =>
                      dec_valids(w) &&
                      (  !dis_ready
                      || dec_prev_enq_stalls(w)
                      || rob.io.commit.rollback
                      || dec_xcpt_stall
                      || branch_mask_full(w)
                      || vconfig_mask_full(w)
                      || brupdate.b1.mispredict_mask =/= 0.U
                      || brupdate.b2.mispredict
                      || io.ifu.redirect_flush))

  dec_stalls  := dec_hazards.scanLeft(false.B) ((s,h) => s || h).takeRight(coreWidth)
  dec_fe_fire := (0 until coreWidth).map(w => dec_valids(w) && !dec_stalls(w) && !dec_enq_stalls(w))
  dec_fire := (0 until coreWidth).map(w => dec_valids(w) && !dec_stalls(w))

  // all decoders are empty and ready for new instructions
  dec_ready := dec_fe_fire.last

  when (dec_ready || io.ifu.redirect_flush) {
    dec_finished_mask := 0.U
  } .otherwise {
    dec_finished_mask := dec_fe_fire.asUInt | dec_finished_mask
  }

  //-------------------------------------------------------------
  // Branch Mask Logic

  dec_brmask_logic.io.brupdate := brupdate
  dec_brmask_logic.io.flush_pipeline := RegNext(rob.io.flush.valid)

  for (w <- 0 until coreWidth) {
    dec_brmask_logic.io.is_branch(w) := !dec_finished_mask(w) && dec_uops(w).allocate_brtag
    dec_brmask_logic.io.will_fire(w) :=  dec_fire(w) &&
                                         dec_uops(w).allocate_brtag // ren, dis can back pressure us
    dec_uops(w).br_tag  := dec_brmask_logic.io.br_tag(w)
    dec_uops(w).br_mask := dec_brmask_logic.io.br_mask(w)
  }

  branch_mask_full := dec_brmask_logic.io.is_full

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Register Rename Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // Inputs
  for (rename <- rename_stages) {
    rename.io.kill := io.ifu.redirect_flush
    rename.io.brupdate := brupdate

    rename.io.debug_rob_empty := rob.io.empty

    rename.io.dec_fire := dec_fire
    rename.io.dec_uops := dec_uops

    rename.io.dis_fire := dis_fire_fb
    rename.io.dis_ready := dis_ready

    rename.io.com_valids := rob.io.commit.valids
    rename.io.com_uops := rob.io.commit.uops
    rename.io.rbk_valids := rob.io.commit.rbk_valids
    rename.io.rollback := rob.io.commit.rollback
    rename.io.vl_wakeup_port := vl_wakeup_dec
  }


  // Outputs
  disUops := rename_stage.io.ren2_uops
  disValids := rename_stage.io.ren2_mask
  ren_stalls := rename_stage.io.ren_stalls

  /**
   * TODO This is a bit nasty, but it's currently necessary to
   * split the INT/FP rename pipelines into separate instantiations.
   * Won't have to do this anymore with a properly decoupled FP pipeline.
   */
  for (w <- 0 until coreWidth) {
    val i_uop   = rename_stage.io.ren2_uops(w)
    val f_uop   = if (usingFPU) fp_rename_stage.io.ren2_uops(w) else NullMicroOp()
    val v_uop   = if (usingVector) v_rename_stage.io.ren2_uops(w) else NullMicroOp()
    val p_uop   = if (enableSFBOpt) pred_rename_stage.io.ren2_uops(w) else NullMicroOp()
    val f_stall = if (usingFPU) fp_rename_stage.io.ren_stalls(w) else false.B
    val v_stall = if (usingVector) v_rename_stage.io.ren_stalls(w) else false.B
    val p_stall = if (enableSFBOpt) pred_rename_stage.io.ren_stalls(w) else false.B

    // lrs1 can "pass through" to prs1. Used solely to index the csr file.
    disUops(w).prs1 := Mux(disUops(w).rt(RS1, isFloat ), f_uop.prs1,
                        Mux(disUops(w).rt(RS1, isInt   ), i_uop.prs1, disUops(w).lrs1))
    disUops(w).prs2 := Mux(disUops(w).rt(RS2, isFloat ), f_uop.prs2,
                        Mux(disUops(w).rt(RS2, isInt   ), i_uop.prs2, disUops(w).lrs2))
    disUops(w).prs3 := f_uop.prs3
    disUops(w).ppred := p_uop.ppred
    disUops(w).pdst := Mux(disUops(w).rt(RD, isFloat ), f_uop.pdst,
                        Mux(disUops(w).rt(RD, isInt   ), i_uop.pdst, p_uop.pdst))
    disUops(w).stale_pdst := Mux(disUops(w).rt(RD, isFloat ), f_uop.stale_pdst, i_uop.stale_pdst)

    disUops(w).prs3_busy   := f_uop.prs3_busy & disUops(w).frs3_en
    disUops(w).ppred_busy  := p_uop.ppred_busy && disUops(w).is_sfb_shadow
    if (usingVector) {
      disUops(w).prs1_busy := Mux1H(Seq((disUops(w).rt(RS1, isInt   ), i_uop.prs1_busy),
                                         (disUops(w).rt(RS1, isFloat ), f_uop.prs1_busy)))
      disUops(w).prs2_busy := Mux1H(Seq((disUops(w).rt(RS2, isInt   ), i_uop.prs2_busy),
                                         (disUops(w).rt(RS2, isFloat ), f_uop.prs2_busy)))
      disUops(w).pvd       := v_uop.pvd
      disUops(w).stale_pvd := v_uop.stale_pvd
      disUops(w).pvs1      := v_uop.pvs1
      disUops(w).pvs2      := v_uop.pvs2
      disUops(w).pvm       := v_uop.pvm
      disUops(w).vstartSrc := v_uop.vstartSrc
      disUops(w).vstart    := Mux(v_uop.vstartSrc === VSTART_ZERO, 0.U, csr.io.vector.get.vstart)
      disUops(w).v_scalar_busy := disUops(w).is_rvv && disUops(w).uses_scalar
      disUops(w).uopc      := Mux(i_uop.uopc.isOneOf(uopVLM), uopVL, i_uop.uopc)
    } else {
      disUops(w).prs1_busy := i_uop.prs1_busy & (disUops(w).rt(RS1, isInt)) |
                               f_uop.prs1_busy & (disUops(w).rt(RS1, isFloat))
      disUops(w).prs2_busy := i_uop.prs2_busy & (disUops(w).rt(RS2, isInt)) |
                               f_uop.prs2_busy & (disUops(w).rt(RS2, isFloat))
    }

    ren_stalls(w) := rename_stage.io.ren_stalls(w) || f_stall || p_stall || v_stall
  }

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Dispatch Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  //-------------------------------------------------------------
  // Rename2/Dispatch pipeline logic

  val dis_prior_slot_valid = disValids.scanLeft(false.B) ((s, v) => s || v)
  val dis_prior_slot_unique = (disUops zip disValids).scanLeft(false.B) {case (s,(u,v)) => s || v && u.is_unique}
  wait_for_empty_pipeline := (0 until coreWidth).map(w => (disUops(w).is_unique || custom_csrs.disableOOO) &&
                                  (!rob.io.empty || !io.lsu.fencei_rdy || dis_prior_slot_valid(w)))
  val rocc_shim_busy = if (usingRoCC) !exe_units.rocc_unit.io.rocc.rxq_empty else false.B
  val wait_for_rocc = (0 until coreWidth).map(w =>
                        (disUops(w).is_fence || disUops(w).is_fencei) && (io.rocc.busy || rocc_shim_busy))
  val rxq_full = if (usingRoCC) exe_units.rocc_unit.io.rocc.rxq_full else false.B
  val block_rocc = (disUops zip disValids).map{case (u,v) => v && u.uopc === uopROCC}.scanLeft(rxq_full)(_||_)
  val dis_rocc_alloc_stall = (disUops.map(_.uopc === uopROCC) zip block_rocc) map {case (p,r) =>
                               if (usingRoCC) p && r else false.B}

  dis_prev_split_actv(0) := false.B
  (1 until coreWidth).map(w => dis_prev_split_actv(w) := dis_split_actv.slice(0, w).reduce(_ || _))
  val dis_hazards = (0 until coreWidth).map(w =>
                      disValids(w) &&
                      (  !rob.io.ready
                      || ren_stalls(w)
                      || dis_prev_split_actv(w)
                      || io.lsu.ldq_full(w) && disUops(w).uses_ldq
                      || io.lsu.stq_full(w) && disUops(w).uses_stq
                      || !dispatcher.io.ren_uops(w).ready
                      || wait_for_empty_pipeline(w)
                      || wait_for_rocc(w)
                      || dis_prior_slot_unique(w)
                      || dis_rocc_alloc_stall(w)
                      || brupdate.b1.mispredict_mask =/= 0.U
                      || brupdate.b2.mispredict
                      || io.ifu.redirect_flush
                      || vlsuIO.toDis.vLdQFull(w) && disUops(w).uses_ldq && disUops(w).is_rvv
                      || vlsuIO.toDis.vStQFull(w) && disUops(w).uses_stq && disUops(w).is_rvv))


  io.lsu.fence_dmem := (disValids zip wait_for_empty_pipeline).map {case (v,w) => v && w} .reduce(_||_)

  val dis_stalls = dis_hazards.scanLeft(false.B) ((s,h) => s || h).takeRight(coreWidth)
  val disLast = (0 until coreWidth).map(i => !disValids(i) || !dis_split_cand(i) || dis_split_last(i)).andR
  if (usingVector) {
    //dis_ready := !dis_stalls.last &&
                 //(!dis_valids.last || !dis_split_cand.last || dis_split_last.last)
    dis_ready := !dis_stalls.last && disLast
    //dis_ready := dis_fire_fb.last
  } else {
    dis_ready := !dis_stalls.last
  }

  //-------------------------------------------------------------
  // RVV L/S split rules:
  // 1. No split for unitstride even lmul is larger than 1;
  // 2. Split constantstride and indexed when lmul is larger than 1 according to this lmul.
  // 3. Split and convert segment unitstride/constantstride into constantstride according to nf.
  // 4. Split and convert segment indexed into indexed according to nf.
  if (usingVector) {
    for (w <- 0 until coreWidth) {
      val dis_split_segf = RegInit(0.U(3.W))
      val lmulLargerThanOne = !disUops(w).vd_emul(2) && disUops(w).vd_emul(1,0) =/= 0.U
      val vseg_ls = disUops(w).is_rvv && disUops(w).v_seg_nf > 1.U
      /** Indeicates if this uop needs to split according to lmul */
      val splitLmul = lmulLargerThanOne && disUops(w).v_idx_ls // @todo not considering seg and constant stride that need to split
      val splitLmulLast: Bool = (dis_split_segf + 1.U(4.W)) === (1.U << disUops(w).vd_emul(1,0))
      val isSplitLast = Mux(splitLmul, splitLmulLast, Mux(vseg_ls, dis_split_segf + 1.U(4.W) === disUops(w).v_seg_nf, true.B))
      val needSplitDis = splitLmul || vseg_ls
      when (io.ifu.redirect_flush) {
        dis_split_segf := 0.U
      } .elsewhen (dis_ready) {
        dis_split_segf := 0.U
      } .elsewhen (disFire(w)) {
        when (needSplitDis) {
          dis_split_segf := dis_split_segf + Mux(isSplitLast, 0.U, 1.U)
        }
      }
      dis_split_last(w) := !needSplitDis || isSplitLast
      dis_split_cand(w) := needSplitDis && (disUops(w).uses_ldq || disUops(w).uses_stq)
      dis_split_actv(w) := dis_split_cand(w) && disValids(w) && !dis_split_last(w)
      disFire(w)    := disValids(w) && !dis_stalls(w)
      dis_fire_fb(w) := disFire(w) && (!dis_split_cand(w) || dis_split_last(w))
      when (dis_split_cand(w) && disValids(w)) {
        when (needSplitDis) { disUops(w).v_seg_f := dis_split_segf }
      }
      disUops(w).v_split_last  := dis_split_last(w)

      //For masked VLS, dispatch to vector pipe to get mask/index.
      val vlsReadsMVMX = disUops(w).is_rvv && (disUops(w).uses_stq || disUops(w).uses_ldq) &&
        (!disUops(w).v_unmasked || disUops(w).v_idx_ls)
      when (vlsReadsMVMX) {
        // if uop need both scalar addr and vector mask(index), addr arrive at vmx-issue as scalar-data.
        disUops(w).iq_type := IQT_MVMX
        disUops(w).v_scalar_busy := true.B
      }
      disSplitFirst(w) := Mux(needSplitDis, dis_split_segf === 0.U, true.B)
      disUops(w).v_split_first := disSplitFirst(w)
      v_rename_stage.io.dis_fire_first(w) := disFire(w) && disSplitFirst(w)
    }
  } else {
    disFire := disValids zip dis_stalls map {case (v,s) => v && !s}

    for (w <- 0 until coreWidth) {
      dis_fire_fb(w)    := disFire(w)
      dis_split_actv(w) := false.B
      dis_split_last(w) := true.B
    }
  }

  //-------------------------------------------------------------
  // LDQ/STQ Allocation Logic

  for (w <- 0 until coreWidth) {
    // Dispatching instructions request load/store queue entries when they can proceed.
    disUops(w).ldq_idx := Mux(disUops(w).is_rvv, vlsuIO.toDis.disVLdQIdx(w).bits.qIdx, io.lsu.dis_ldq_idx(w))
    disUops(w).stq_idx := Mux(disUops(w).is_rvv, vlsuIO.toDis.disVStQIdx(w).bits.qIdx, io.lsu.dis_stq_idx(w))
  }

  //-------------------------------------------------------------
  // Rob Allocation Logic

  rob.io.enq_valids := disFire
  rob.io.enq_uops   := disUops
  if (usingVector) {
    rob.io.enq_partial_stall := (dis_split_actv zip dis_stalls).foldRight(dis_stalls.last){
      case((actv, stall), prev) => Mux(actv, stall, prev)}
  } else {
    rob.io.enq_partial_stall := dis_stalls.last // TODO come up with better ROB compacting scheme.
  }
  rob.io.debug_tsc := debug_tsc_reg
  rob.io.csr_stall := csr.io.csr_stall

  // Minor hack: ecall and breaks need to increment the FTQ deq ptr earlier than commit, since
  // they write their PC into the CSR the cycle before they commit.
  // Since these are also unique, increment the FTQ ptr when they are dispatched
  when (RegNext(disFire.reduce(_||_) && disUops(PriorityEncoder(disFire)).is_sys_pc2epc)) {
    io.ifu.commit.valid := true.B
    io.ifu.commit.bits  := RegNext(disUops(PriorityEncoder(disValids)).ftq_idx)
  }

  for (w <- 0 until coreWidth) {
    // note: this assumes uops haven't been shifted - there's a 1:1 match between PC's LSBs and "w" here
    // (thus the LSB of the rob_idx gives part of the PC)
    if (coreWidth == 1) {
      disUops(w).rob_idx := rob.io.rob_tail_idx
    } else {
      disUops(w).rob_idx := Cat(rob.io.rob_tail_idx >> log2Ceil(coreWidth).U,
                               w.U(log2Ceil(coreWidth).W))
    }
  }

  //-------------------------------------------------------------
  // RoCC allocation logic
  if (usingRoCC) {
    for (w <- 0 until coreWidth) {
      // We guarantee only decoding 1 RoCC instruction per cycle
      disUops(w).rxq_idx := exe_units.rocc_unit.io.rocc.rxq_idx(w)
    }
  }

  //-------------------------------------------------------------
  // Dispatch to issue queues

  // Get uops from rename2
  for (w <- 0 until coreWidth) {
    dispatcher.io.ren_uops(w).valid := disFire(w)
    dispatcher.io.ren_uops(w).bits  := disUops(w)
  }

  var iu_idx = 0
  // Send dispatched uops to correct issue queues
  // Backpressure through dispatcher if necessary
  for (i <- 0 until issueParams.size) {
    if (issueParams(i).iqType == IQT_VEC.litValue) {
       v_pipeline.io.vec_dis_uops <> dispatcher.io.dis_uops(i)
     } else if (issueParams(i).iqType == IQT_VMX.litValue) {
       v_pipeline.io.vmx_dis_uops <> dispatcher.io.dis_uops(i)
    } else if (issueParams(i).iqType == IQT_FP.litValue) {
       fp_pipeline.io.dis_uops <> dispatcher.io.dis_uops(i)
    } else {
       issue_units(iu_idx).io.dis_uops <> dispatcher.io.dis_uops(i)
       iu_idx += 1
    }
  }

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Issue Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  require (issue_units.map(_.issueWidth).sum == exe_units.length)

  var iss_wu_idx = 1
  var ren_wu_idx = 1
  // The 0th wakeup port goes to the ll_wbarb
  int_iss_wakeups(0).valid := ll_wbarb.io.out.fire() && ll_wbarb.io.out.bits.uop.rt(RD, isInt)
  int_iss_wakeups(0).bits  := ll_wbarb.io.out.bits

  int_ren_wakeups(0).valid := ll_wbarb.io.out.fire() && ll_wbarb.io.out.bits.uop.rt(RD, isInt)
  int_ren_wakeups(0).bits  := ll_wbarb.io.out.bits

  for (i <- 1 until memWidth) {
    int_iss_wakeups(i).valid := mem_resps(i).valid && mem_resps(i).bits.uop.rt(RD, isInt)
    int_iss_wakeups(i).bits  := mem_resps(i).bits

    int_ren_wakeups(i).valid := mem_resps(i).valid && mem_resps(i).bits.uop.rt(RD, isInt)
    int_ren_wakeups(i).bits  := mem_resps(i).bits
    iss_wu_idx += 1
    ren_wu_idx += 1
  }

  // loop through each issue-port (exe_units are statically connected to an issue-port)
  for (i <- 0 until exe_units.length) {
    if (exe_units(i).writesIrf) {
      val fast_wakeup = Wire(Valid(new ExeUnitResp(xLen)))
      val slow_wakeup = Wire(Valid(new ExeUnitResp(xLen)))
      fast_wakeup := DontCare
      slow_wakeup := DontCare

      val resp = exe_units(i).io.iresp
      assert(!(resp.valid && resp.bits.uop.rf_wen && resp.bits.uop.rt(RD, isNotInt)))

      // Fast Wakeup (uses just-issued uops that have known latencies)
      fast_wakeup.bits.uop := iss_uops(i)
      fast_wakeup.valid    := iss_valids(i) &&
                              iss_uops(i).bypassable &&
                              iss_uops(i).rt(RD, isInt) &&
                              iss_uops(i).ldst_val &&
                              !(io.lsu.ld_miss && (iss_uops(i).iw_p1_poisoned || iss_uops(i).iw_p2_poisoned))

      // Slow Wakeup (uses write-port to register file)
      slow_wakeup.bits.uop := resp.bits.uop
      slow_wakeup.valid    := resp.valid &&
                                resp.bits.uop.rf_wen &&
                                !resp.bits.uop.bypassable &&
                                resp.bits.uop.rt(RD, isInt)

      if (exe_units(i).bypassable) {
        int_iss_wakeups(iss_wu_idx) := fast_wakeup
        iss_wu_idx += 1
      }
      if (!exe_units(i).alwaysBypassable) {
        int_iss_wakeups(iss_wu_idx) := slow_wakeup
        iss_wu_idx += 1
      }

      if (exe_units(i).bypassable) {
        int_ren_wakeups(ren_wu_idx) := fast_wakeup
        ren_wu_idx += 1
      }
      if (!exe_units(i).alwaysBypassable) {
        int_ren_wakeups(ren_wu_idx) := slow_wakeup
        ren_wu_idx += 1
      }
    }
  }
  require (iss_wu_idx == numIntIssueWakeupPorts)
  require (ren_wu_idx == numIntRenameWakeupPorts)
  require (iss_wu_idx == ren_wu_idx)

  // jmp unit performs fast wakeup of the predicate bits
  require (jmp_unit.bypassable)
  pred_wakeup.valid := (iss_valids(jmp_unit_idx) &&
                        iss_uops(jmp_unit_idx).is_sfb_br &&
                        !(io.lsu.ld_miss && (iss_uops(jmp_unit_idx).iw_p1_poisoned || iss_uops(jmp_unit_idx).iw_p2_poisoned))
  )
  pred_wakeup.bits.uop := iss_uops(jmp_unit_idx)
  pred_wakeup.bits.fflags := DontCare
  pred_wakeup.bits.data := DontCare
  pred_wakeup.bits.predicated := DontCare

  // Perform load-hit speculative wakeup through a special port (performs a poison wake-up).
  issue_units map { iu =>
    iu.io.spec_ld_wakeup := io.lsu.spec_ld_wakeup

    iu.io.vl_wakeup_port := vl_wakeup
  }

  // Connect the predicate wakeup port
  issue_units map { iu =>
    iu.io.pred_wakeup_port.valid := false.B
    iu.io.pred_wakeup_port.bits := DontCare
  }
  if (enableSFBOpt) {
    int_iss_unit.io.pred_wakeup_port.valid := pred_wakeup.valid
    int_iss_unit.io.pred_wakeup_port.bits := pred_wakeup.bits.uop.pdst
  }

  // ----------------------------------------------------------------
  // Connect the wakeup ports to the busy tables in the rename stages

  for ((renport, intport) <- rename_stage.io.wakeups zip int_ren_wakeups) {
    renport <> intport
  }
  if (usingFPU) {
    for ((renport, fpport) <- fp_rename_stage.io.wakeups zip fp_pipeline.io.wakeups) {
       renport <> fpport
    }
  }
  if (usingVector) {
    for ((renport, vport) <- v_rename_stage.io.wakeups zip v_pipeline.io.wakeups) {
       renport <> vport
    }
  }
  if (enableSFBOpt) {
    pred_rename_stage.io.wakeups(0) := pred_wakeup
  } else {
    pred_rename_stage.io.wakeups := DontCare
  }

  // If we issue loads back-to-back endlessly (probably because we are executing some tight loop)
  // the store buffer will never drain, breaking the memory-model forward-progress guarantee
  // If we see a large number of loads saturate the LSU, pause for a cycle to let a store drain
  val loads_saturating = (mem_iss_unit.io.iss_valids(0) && mem_iss_unit.io.iss_uops(0).uses_ldq)
  val saturating_loads_counter = RegInit(0.U(5.W))
  when (loads_saturating) { saturating_loads_counter := saturating_loads_counter + 1.U }
  .otherwise { saturating_loads_counter := 0.U }
  val pause_mem = RegNext(loads_saturating) && saturating_loads_counter === ~(0.U(5.W))

  var iss_idx = 0
  var int_iss_cnt = 0
  var mem_iss_cnt = 0
  for (w <- 0 until exe_units.length) {
    var fu_types = exe_units(w).io.fu_types
    val exe_unit = exe_units(w)
    if (exe_unit.readsIrf) {
      if (exe_unit.supportedFuncUnits.muld) {
        // Supress just-issued divides from issuing back-to-back, since it's an iterative divider.
        // But it takes a cycle to get to the Exe stage, so it can't tell us it is busy yet.
        val idiv_issued = iss_valids(iss_idx) && iss_uops(iss_idx).fu_code_is(FU_DIV)
        fu_types = fu_types & RegNext(~Mux(idiv_issued, FU_DIV, 0.U))
      }

      if (exe_unit.hasMem) {
        iss_valids(iss_idx) := mem_iss_unit.io.iss_valids(mem_iss_cnt)
        iss_uops(iss_idx)   := mem_iss_unit.io.iss_uops(mem_iss_cnt)
        mem_iss_unit.io.fu_types(mem_iss_cnt) := Mux(pause_mem, 0.U, fu_types)
        mem_iss_cnt += 1
      } else {
        iss_valids(iss_idx) := int_iss_unit.io.iss_valids(int_iss_cnt)
        iss_uops(iss_idx)   := int_iss_unit.io.iss_uops(int_iss_cnt)
        int_iss_unit.io.fu_types(int_iss_cnt) := fu_types
        int_iss_cnt += 1
      }
      iss_idx += 1
    }
  }
  require(iss_idx == exe_units.numIrfReaders)

  issue_units.map(_.io.tsc_reg := debug_tsc_reg)
  issue_units.map(_.io.brupdate := brupdate)
  issue_units.map(_.io.flush_pipeline := RegNext(rob.io.flush.valid))
  if (usingVector) {
    //mem_iss_unit.io.vmupdate := vmupdate
    //int_iss_unit.io.vmupdate.map(_.valid := false.B)
    //int_iss_unit.io.vmupdate.map(_.bits := DontCare)
    v_pipeline.io.intupdate := iregister_read.io.intupdate
    v_pipeline.io.fpupdate := fp_pipeline.io.fpupdate
    v_pipeline.io.vlsuReadReq.valid := vlsuIO.toVrf.readReq.valid
    v_pipeline.io.vlsuReadReq.bits := vlsuIO.toVrf.readReq.bits.addr
    vlsuIO.fromVrf.readResp.valid := v_pipeline.io.vlsuReadResp.valid
    vlsuIO.fromVrf.readResp.bits.data := v_pipeline.io.vlsuReadResp.bits
  }

  // Load-hit Misspeculations
  require (mem_iss_unit.issueWidth <= 2)
  issue_units.map(_.io.ld_miss := io.lsu.ld_miss)

  mem_units.map(u => u.io.com_exception := RegNext(rob.io.flush.valid))

  // Wakeup (Issue & Writeback)
  for {
    iu <- issue_units
    (issport, wakeup) <- iu.io.wakeup_ports zip int_iss_wakeups
  }{
    issport.valid := wakeup.valid
    issport.bits.pdst := wakeup.bits.uop.pdst
    issport.bits.poisoned := wakeup.bits.uop.iw_p1_poisoned || wakeup.bits.uop.iw_p2_poisoned

    require (iu.io.wakeup_ports.length == int_iss_wakeups.length)
  }

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Register Read Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // Register Read <- Issue (rrd <- iss)
  iregister_read.io.rf_read_ports <> iregfile.io.read_ports
  iregister_read.io.prf_read_ports := DontCare
  if (enableSFBOpt) {
    iregister_read.io.prf_read_ports <> pregfile.io.read_ports
  }

  for (w <- 0 until exe_units.numIrfReaders) {
    iregister_read.io.iss_valids(w) :=
      iss_valids(w) && !(io.lsu.ld_miss && (iss_uops(w).iw_p1_poisoned || iss_uops(w).iw_p2_poisoned))
  }
  iregister_read.io.iss_uops := iss_uops
  iregister_read.io.iss_uops map { u => u.iw_p1_poisoned := false.B; u.iw_p2_poisoned := false.B }

  iregister_read.io.brupdate := brupdate
  iregister_read.io.kill   := RegNext(rob.io.flush.valid)

  iregister_read.io.bypass := bypasses
  iregister_read.io.pred_bypass := pred_bypasses

  //-------------------------------------------------------------
  // Privileged Co-processor 0 Register File
  // Note: Normally this would be bad in that I'm writing state before
  // committing, so to get this to work I stall the entire pipeline for
  // CSR instructions so I never speculate these instructions.

  val csr_exe_unit = exe_units.csr_unit

  // for critical path reasons, we aren't zero'ing this out if resp is not valid
  val csr_rw_cmd = csr_exe_unit.io.iresp.bits.uop.ctrl.csr_cmd
  val wb_wdata = csr_exe_unit.io.iresp.bits.data

  csr.io.rw.addr        := csr_exe_unit.io.iresp.bits.uop.csr_addr
  csr.io.rw.cmd         := freechips.rocketchip.rocket.CSR.maskCmd(csr_exe_unit.io.iresp.valid, csr_rw_cmd)
  csr.io.rw.wdata       := wb_wdata

  // Extra I/O
  // Delay retire/exception 1 cycle
  if(usingVector) {
    val cmt_valids = (0 until coreParams.retireWidth).map(i =>
      rob.io.commit.valids(i) && (!rob.io.commit.uops(i).is_rvv || rob.io.commit.uops(i).v_split_last))
      //rob.io.commit.arch_valids(i) && (!rob.io.commit.uops(i).is_rvv || rob.io.commit.uops(i).v_split_last))
    csr.io.retire  := RegNext(PopCount(cmt_valids))
    when(cmt_valids.orR) {
      retire_cnt := retire_cnt + PopCount(cmt_valids)
      printf("retire_cnt: %d\n", retire_cnt.asUInt())
    }
  } else {
    csr.io.retire  := RegNext(PopCount(rob.io.commit.arch_valids.asUInt))
  }
  csr.io.exception := RegNext(rob.io.com_xcpt.valid)
  // csr.io.pc used for setting EPC during exception or CSR.io.trace.

  csr.io.pc        := (boom.util.AlignPCToBoundary(io.ifu.get_pc(0).com_pc, icBlockBytes)
                     + RegNext(rob.io.com_xcpt.bits.pc_lob)
                     - Mux(RegNext(rob.io.com_xcpt.bits.edge_inst), 2.U, 0.U))
  // Cause not valid for for CALL or BREAKPOINTs (CSRFile will override it).
  csr.io.cause     := RegNext(rob.io.com_xcpt.bits.cause)
  csr.io.ungated_clock := clock

  val tval_valid = csr.io.exception &&
    csr.io.cause.isOneOf(
      //Causes.illegal_instruction.U, we currently only write 0x0 for illegal instructions
      Causes.breakpoint.U,
      Causes.misaligned_load.U,
      Causes.misaligned_store.U,
      Causes.load_access.U,
      Causes.store_access.U,
      Causes.fetch_access.U,
      Causes.load_page_fault.U,
      Causes.store_page_fault.U,
      Causes.fetch_page_fault.U)

  csr.io.tval := Mux(tval_valid,
    RegNext(encodeVirtualAddress(rob.io.com_xcpt.bits.badvaddr, rob.io.com_xcpt.bits.badvaddr)), 0.U)

  // TODO move this function to some central location (since this is used elsewhere).
  def encodeVirtualAddress(a0: UInt, ea: UInt) =
    if (vaddrBitsExtended == vaddrBits) {
      ea
    } else {
      // Efficient means to compress 64-bit VA into vaddrBits+1 bits.
      // (VA is bad if VA(vaddrBits) != VA(vaddrBits-1)).
      val a = a0.asSInt >> vaddrBits
      val msb = Mux(a === 0.S || a === -1.S, ea(vaddrBits), !ea(vaddrBits-1))
      Cat(msb, ea(vaddrBits-1,0))
    }

  // reading requires serializing the entire pipeline
  csr.io.fcsr_flags.valid := rob.io.commit.fflags.valid
  csr.io.fcsr_flags.bits  := rob.io.commit.fflags.bits
  csr.io.set_fs_dirty.get := rob.io.commit.fflags.valid

  exe_units.withFilter(_.hasFcsr).map(_.io.fcsr_rm := csr.io.fcsr_rm)
  io.fcsr_rm := csr.io.fcsr_rm

  if (usingFPU) {
    fp_pipeline.io.fcsr_rm := csr.io.fcsr_rm
  }

  csr.io.hartid := io.hartid
  csr.io.interrupts := io.interrupts

  if (usingVector) {
    val csr_vld = csr_exe_unit.io.iresp.valid
    val csr_uop = csr_exe_unit.io.iresp.bits.uop
    val vsetvl = csr_uop.uopc === uopVSETVL
    val cmt_rvv = (0 until coreParams.retireWidth).map{i =>
        rob.io.commit.arch_valids(i) && rob.io.commit.uops(i).is_rvv
    }.reduce(_ || _)
    val cmt_archlast_rvv = (0 until coreParams.retireWidth).map{i =>
        rob.io.commit.arch_valids(i) && rob.io.commit.uops(i).is_rvv &&
        rob.io.commit.uops(i).v_split_last // architectural last split
        //&& !rob.io.commit.uops(i).is_red_vadd && // excluding inserted VADD
        //!rob.io.commit.uops(i).is_perm_vadd
    }.reduce(_ || _)
    val cmt_sat = (0 until coreParams.retireWidth).map{i =>
      rob.io.commit.arch_valids(i) && rob.io.commit.uops(i).is_rvv && rob.io.commit.uops(i).vxsat
    }.reduce(_ || _)
    val vleffSetVL = rob.io.setVL
    val vcq_setVL = (rob.io.commit.valids zip rob.io.commit.uops).map { case (v, u) => Mux(v, u.is_vsetivli || u.is_vsetvli, false.B) }.reduce(_ | _)
    assert(!(vsetvl && csr_vld && vleffSetVL.valid), "vsetvl and vleff setvl should not happen at same time!")
    csr.io.vector.get.set_vs_dirty := cmt_rvv
    csr.io.vector.get.set_vconfig.valid := csr_vld && vsetvl || vleffSetVL.valid || vcq_setVL
    csr.io.vector.get.set_vconfig.bits := Mux(csr_vld && vsetvl, csr_uop.vconfig, Mux(vcq_setVL, vcq.io.vcq_Wcsr.vconfig, csr.io.vector.get.vconfig))
    csr.io.vector.get.set_vconfig.bits.vl := Mux(csr_vld && vsetvl, csr_uop.vconfig.vl,
                                              Mux(vleffSetVL.valid, vleffSetVL.bits,
                                              Mux(vcq_setVL, vcq.io.vcq_Wcsr.vconfig.vl,
                                                              csr.io.vector.get.vconfig.vl)))
    csr.io.vector.get.set_vconfig.bits.vtype.reserved := DontCare
    csr.io.vector.get.set_vstart.valid := cmt_archlast_rvv || rob.io.com_xcpt.bits.vls_xcpt.valid
    csr.io.vector.get.set_vstart.bits := Mux(cmt_archlast_rvv, 0.U, rob.io.com_xcpt.bits.vls_xcpt.bits)
    csr.io.vector.get.set_vxsat := cmt_sat
    v_pipeline.io.fcsr_rm := csr.io.fcsr_rm
    v_pipeline.io.vxrm := csr.io.vector.get.vxrm

    csr_exe_unit.io.vconfig := csr.io.vector.get.vconfig
  }

// TODO can we add this back in, but handle reset properly and save us
//      the mux above on csr.io.rw.cmd?
//   assert (!(csr_rw_cmd =/= rocket.CSR.N && !exe_units(0).io.resp(0).valid),
//   "CSRFile is being written to spuriously.")

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Execute Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  iss_idx = 0
  var bypass_idx = 0
  for (w <- 0 until exe_units.length) {
    val exe_unit = exe_units(w)
    if (exe_unit.readsIrf) {
      exe_unit.io.req <> iregister_read.io.exe_reqs(iss_idx)

      if (exe_unit.bypassable) {
        for (i <- 0 until exe_unit.numBypassStages) {
          bypasses(bypass_idx) := exe_unit.io.bypass(i)
          bypass_idx += 1
        }
      }
      iss_idx += 1
    }
  }
  require (bypass_idx == exe_units.numTotalBypassPorts)
  for (i <- 0 until jmp_unit.numBypassStages) {
    pred_bypasses(i) := jmp_unit.io.bypass(i)
  }

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Load/Store Unit ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // enqueue basic load/store info in Decode
  for (w <- 0 until coreWidth) {
    io.lsu.dis_uops(w).valid := disFire(w) && !disUops(w).is_rvv
    io.lsu.dis_uops(w).bits  := disUops(w)
    if(usingVector){
      vlsuIO.fromDis.vuopDis(w).valid := disFire(w) && disUops(w).is_rvv && (disUops(w).uses_ldq || disUops(w).uses_stq)
      vlsuIO.fromDis.vuopDis(w).bits := uopConvertToVuop(disUops(w))
    }
  }

  // tell LSU about committing loads and stores to clear entries
  io.lsu.commit                  := rob.io.commit
  if(usingVector){
    vlsuIO.fromRob.retireEntries.zipWithIndex.foreach {case (entry, i) =>
      entry.valid := (rob.io.commit.valids(i) || rob.io.commit.arch_valids(i)) && rob.io.commit.uops(i).is_rvv
      entry.bits.isStore := rob.io.commit.uops(i).uses_stq
      entry.bits.isLoad := rob.io.commit.uops(i).uses_ldq
      entry.bits.qEntryIdx := Mux(rob.io.commit.uops(i).uses_stq, rob.io.commit.uops(i).stq_idx, rob.io.commit.uops(i).ldq_idx)
    }
  }

  // tell LSU that it should fire a load that waits for the rob to clear
  io.lsu.commit_load_at_rob_head := rob.io.com_load_is_at_rob_head

  //com_xcpt.valid comes too early, will fight against a branch that resolves same cycle as an exception
  io.lsu.exception := RegNext(rob.io.flush.valid)

  // Handle Branch Mispeculations
  io.lsu.brupdate := brupdate
  //if (usingVector) {
  //  io.lsu.vmupdate := vmupdate
  //}
  io.lsu.rob_head_idx := rob.io.rob_head_idx
  io.lsu.rob_pnr_idx  := rob.io.rob_pnr_idx

  io.lsu.tsc_reg := debug_tsc_reg


  if (usingFPU) {
    io.lsu.fp_stdata <> fp_pipeline.io.to_sdq
//} else {
//  io.lsu.fp_stdata.valid := false.B
//  io.lsu.fp_stdata.bits := DontCare
//  fp_pipeline.io.to_sdq.ready := false.B
  }

  //if (usingVector) {
  //  io.lsu.v_stdata <> v_pipeline.io.to_sdq
//} else {
//  io.lsu.v_stdata.valid := false.B
//  io.lsu.v_stdata.bits := DontCare
//  v_pipeline.io.to_sdq.ready := false.B
  //}

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Writeback Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  var w_cnt = 1
  iregfile.io.write_ports(0) := WritePort(ll_wbarb.io.out, ipregSz, xLen, isInt)
  ll_wbarb.io.in(0) <> mem_resps(0)
  assert (ll_wbarb.io.in(0).ready) // never backpressure the memory unit.
  for (i <- 1 until memWidth) {
    iregfile.io.write_ports(w_cnt) := WritePort(mem_resps(i), ipregSz, xLen, isInt)
    w_cnt += 1
  }

  for (i <- 0 until exe_units.length) {
    if (exe_units(i).writesIrf) {
      val wbresp = exe_units(i).io.iresp
      val wbpdst = wbresp.bits.uop.pdst
      val wbdata = wbresp.bits.data

      def wbIsValid(rtype: UInt => Bool) =
        wbresp.valid && wbresp.bits.uop.rf_wen && wbresp.bits.uop.rt(RD, rtype)
      val wbReadsCSR = wbresp.bits.uop.ctrl.csr_cmd =/= freechips.rocketchip.rocket.CSR.N
      val wbWritesCSR =freechips.rocketchip.rocket.CSR.maskCmd(wbresp.valid, wbresp.bits.uop.ctrl.csr_cmd)(2)

      iregfile.io.write_ports(w_cnt).valid     := wbIsValid(isInt)
      iregfile.io.write_ports(w_cnt).bits.addr := wbpdst
      wbresp.ready := true.B
      if (exe_units(i).hasCSR) {
        iregfile.io.write_ports(w_cnt).bits.data := Mux(wbReadsCSR, csr.io.rw.rdata, wbdata)
      } else {
        iregfile.io.write_ports(w_cnt).bits.data := wbdata
      }

      assert (!wbIsValid(isFloat), "[fppipeline] An FP writeback is being attempted to the Int Regfile.")

      assert (!(wbresp.valid &&
        !(wbresp.bits.uop.rf_wen || wbWritesCSR || wbresp.bits.uop.pdst === 0.U) &&
        wbresp.bits.uop.rt(RD, isInt)),
        "[fppipeline] An Int writeback is being attempted with rf_wen disabled.")

      assert (!(wbresp.valid &&
        wbresp.bits.uop.rf_wen &&
        wbresp.bits.uop.rt(RD, isNotInt)),
        "[fppipeline] writeback being attempted to Int RF with dst != Int type exe_units("+i+").iresp")
      w_cnt += 1
    }
  }
  require(w_cnt == iregfile.io.write_ports.length)

  if (enableSFBOpt) {
    pregfile.io.write_ports(0).valid     := jmp_unit.io.iresp.valid && jmp_unit.io.iresp.bits.uop.is_sfb_br
    pregfile.io.write_ports(0).bits.addr := jmp_unit.io.iresp.bits.uop.pdst
    pregfile.io.write_ports(0).bits.data := jmp_unit.io.iresp.bits.data
  }

  if (usingFPU) {
    // Connect IFPU
    fp_pipeline.io.from_int  <> exe_units.ifpu_unit.io.ll_fresp
    // Connect FPIU
    ll_wbarb.io.in(1)        <> fp_pipeline.io.to_int
    // Connect FLDs
    fp_pipeline.io.ll_wports <> exe_units.memory_units.map(_.io.ll_fresp)
  }

  if (usingVector) {
    fp_pipeline.io.fromVec <> v_pipeline.io.to_fp
    Seq.tabulate(vecWidth)(i => i).foreach { i =>
      val attachBase: Int = 2 + (if(usingRoCC) 1 else 0)
      ll_wbarb.io.in(attachBase + i) <> v_pipeline.io.to_int(i)
    }
    //v_pipeline.io.to_fp.ready := true.B
    //v_pipeline.io.ll_wports  <> exe_units.memory_units.map(_.io.ll_vresp)
    v_pipeline.io.vlsuWritePort.valid := vlsuIO.toVrf.write.valid
    v_pipeline.io.vlsuWritePort.bits.data := vlsuIO.toVrf.write.bits.data
    v_pipeline.io.vlsuWritePort.bits.addr := vlsuIO.toVrf.write.bits.addr
    v_pipeline.io.vlsuWritePort.bits.byteMask := vlsuIO.toVrf.write.bits.byteMask
    v_pipeline.io.vlsuLoadWakeUp.valid := vlsuIO.wakeUpVReg.valid
    v_pipeline.io.vlsuLoadWakeUp.bits := vlsuIO.wakeUpVReg.bits
  }

  if (usingRoCC) {
    require(usingFPU)
    ll_wbarb.io.in(2)       <> exe_units.rocc_unit.io.ll_iresp
  }

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Commit Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // Writeback
  // ---------
  // First connect the ll_wport
  val ll_uop = ll_wbarb.io.out.bits.uop
  rob.io.wb_resps(0).valid  := ll_wbarb.io.out.valid && !(ll_uop.uses_stq && !ll_uop.is_amo)
  rob.io.wb_resps(0).bits   <> ll_wbarb.io.out.bits
  rob.io.debug_wb_valids(0) := ll_wbarb.io.out.valid && ll_uop.rt(RD, isSomeReg)
  rob.io.debug_wb_wdata(0)  := ll_wbarb.io.out.bits.data
  var cnt = 1
  for (i <- 1 until memWidth) {
    val mem_uop = mem_resps(i).bits.uop
    rob.io.wb_resps(cnt).valid := mem_resps(i).valid && !(mem_uop.uses_stq && !mem_uop.is_amo)
    rob.io.wb_resps(cnt).bits  := mem_resps(i).bits
    rob.io.debug_wb_valids(cnt) := mem_resps(i).valid && mem_uop.rt(RD, isSomeReg)
    rob.io.debug_wb_wdata(cnt)  := mem_resps(i).bits.data
    cnt += 1
  }
  var f_cnt = 0 // rob fflags port index
  for (eu <- exe_units) {
    if (eu.writesIrf)
    {
      val resp   = eu.io.iresp
      val wb_uop = resp.bits.uop
      val data   = resp.bits.data

      rob.io.wb_resps(cnt).valid := resp.valid && !(wb_uop.uses_stq && !wb_uop.is_amo)
      rob.io.wb_resps(cnt).bits  <> resp.bits
      rob.io.debug_wb_valids(cnt) := resp.valid && wb_uop.rf_wen && wb_uop.rt(RD, isInt)
      if (eu.hasFFlags) {
        rob.io.fflags(f_cnt) <> resp.bits.fflags
        f_cnt += 1
      }
      if (eu.hasCSR) {
        rob.io.debug_wb_wdata(cnt) := Mux(wb_uop.ctrl.csr_cmd =/= freechips.rocketchip.rocket.CSR.N,
          csr.io.rw.rdata,
          data)
      } else {
        rob.io.debug_wb_wdata(cnt) := data
      }
      cnt += 1
    }
  }
  require(cnt == numIrfWritePorts)

  if (usingFPU) {
    for ((wdata, wakeup) <- fp_pipeline.io.debug_wb_wdata zip fp_pipeline.io.wakeups) {
      rob.io.wb_resps(cnt) <> wakeup
      rob.io.fflags(f_cnt) <> wakeup.bits.fflags
      rob.io.debug_wb_valids(cnt) := wakeup.valid
      rob.io.debug_wb_wdata(cnt) := wdata
      cnt += 1
      f_cnt += 1

      assert (!(wakeup.valid && !wakeup.bits.uop.rt(RD, isFloat)),
        "[core] FP wakeup does not write back to a FP register.")

      assert (!(wakeup.valid && !wakeup.bits.uop.fp_val),
        "[core] FP wakeup does not involve an FP instruction.")
    }
  }
  require (f_cnt == rob.numFpuPorts)

  if (usingVector) {
    for ((wdata, wakeup) <- v_pipeline.io.debug_wb_wdata zip v_pipeline.io.wakeups) {
      rob.io.wb_resps(cnt) <> wakeup
      rob.io.debug_wb_valids(cnt) := wakeup.valid
      rob.io.debug_wb_wdata(cnt) := wdata
      cnt += 1

      assert (!(wakeup.valid && !wakeup.bits.uop.rt(RD, isVector)),
        "[core] VEC wakeup does not write back to a VEC register.")

      assert (!(wakeup.valid && !wakeup.bits.uop.is_rvv),
        "[core] VEC wakeup does not involve an VEC instruction.")
    }
  }

  require (cnt == rob.numWakeupPorts, s"rob wb port mismatch: ${cnt}, ${rob.numWakeupPorts}")

  // branch resolution
  rob.io.brupdate <> brupdate
  // vector mask update
  //if (usingVector) {
  //  rob.io.vmupdate <> vmupdate
  //}

  exe_units.map(u => u.io.status := csr.io.status)
  if (usingFPU)
    fp_pipeline.io.status := csr.io.status

  if (usingVector) {
    v_pipeline.io.status := csr.io.status
  }

  // Connect breakpoint info to memaddrcalcunit
  for (i <- 0 until memWidth) {
    mem_units(i).io.status   := csr.io.status
    mem_units(i).io.bp       := csr.io.bp
    mem_units(i).io.mcontext := csr.io.mcontext
    mem_units(i).io.scontext := csr.io.scontext
  }

  // LSU <> ROB
  //rob.io.lsu_clr_bsy    := io.lsu.clr_bsy
  rob.io.lsu_clr_bsy.foreach( _ := DontCare)
  (rob.io.lsu_clr_bsy.slice(0, memWidth) zip io.lsu.clr_bsy).foreach { case (rob, lsu) => rob := lsu }
  rob.io.lsu_clr_unsafe := io.lsu.clr_unsafe
  rob.io.lxcpt          <> io.lsu.lxcpt

  // VLSU store <> ROB
  (rob.io.lsu_clr_bsy.slice(memWidth, memWidth + coreWidth) zip vlsuIO.stToRob.robIdx).foreach { case (rob, vlsu) =>
    rob.valid := vlsu.valid
    rob.bits := vlsu.bits
  }
  // VLSU load <> ROB
  (rob.io.lsu_clr_bsy.slice(memWidth + coreWidth, memWidth + 2 * coreWidth) zip vlsuIO.ldToRob.robIdx).foreach{ case (rob, vlsu) =>
    rob.valid := vlsu.valid
    rob.bits := vlsu.bits
  }
  assert (!(csr.io.singleStep), "[core] single-step is unsupported.")


  //-------------------------------------------------------------
  // **** Flush Pipeline ****
  //-------------------------------------------------------------
  // flush on exceptions, miniexeptions, and after some special instructions

  if (usingFPU) {
    fp_pipeline.io.flush_pipeline := RegNext(rob.io.flush.valid)
  }

  if (usingVector) {
    v_pipeline.io.flush_pipeline := RegNext(rob.io.flush.valid)
  }

  for (w <- 0 until exe_units.length) {
    exe_units(w).io.req.bits.kill := RegNext(rob.io.flush.valid)
  }

  assert (!(rob.io.com_xcpt.valid && !rob.io.flush.valid),
    "[core] exception occurred, but pipeline flush signal not set!")

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Outputs to the External World ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // detect pipeline freezes and throw error
  val idle_cycles = freechips.rocketchip.util.WideCounter(32)
  when (rob.io.commit.valids.asUInt.orR ||
        csr.io.csr_stall ||
        io.rocc.busy ||
        reset.asBool) {
    idle_cycles := 0.U
  }
  assert (!(idle_cycles.value(14)), "Pipeline has hung.")

  if (usingFPU) {
    fp_pipeline.io.debug_tsc_reg := debug_tsc_reg
  }

  if (usingVector) {
    v_pipeline.io.debug_tsc_reg := debug_tsc_reg
  }

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Handle Cycle-by-Cycle Printouts ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------


  if (COMMIT_LOG_PRINTF) {
    var new_commit_cnt = 0.U

    for (w <- 0 until coreWidth) {
      val priv = RegNext(csr.io.status.prv) // erets change the privilege. Get the old one

      // To allow for diffs against spike :/
      def printf_inst(uop: MicroOp) = {
        when (uop.is_rvc) {
          printf("(0x%x)", uop.debug_inst(15,0))
        } .otherwise {
          printf("(0x%x)", uop.debug_inst)
        }
      }

      when (rob.io.commit.arch_valids(w)) {
        printf("%d 0x%x ",
          priv,
          Sext(rob.io.commit.uops(w).debug_pc(vaddrBits-1,0), xLen))
        printf_inst(rob.io.commit.uops(w))
        when (rob.io.commit.uops(w).rt(RD, isInt) && rob.io.commit.uops(w).ldst =/= 0.U) {
          printf(" x%d 0x%x\n",
            rob.io.commit.uops(w).ldst,
            rob.io.commit.debug_wdata(w))
        } .elsewhen (rob.io.commit.uops(w).rt(RD, isFloat)) {
          printf(" f%d 0x%x\n",
            rob.io.commit.uops(w).ldst,
            rob.io.commit.debug_wdata(w))
        } .elsewhen (rob.io.commit.uops(w).rt(RD, isVector)) {
          if (usingVector) {
            printf(" v%d[%d] 0x%x\n",
              rob.io.commit.uops(w).ldst,
              rob.io.commit.uops(w).v_eidx,
              rob.io.commit.debug_wdata(w))
          }
        } .otherwise {
          printf("\n")
        }
      }
    }
  } else if (BRANCH_PRINTF) {
    val debug_ghist = RegInit(0.U(globalHistoryLength.W))
    when (rob.io.flush.valid && FlushTypes.useCsrEvec(rob.io.flush.bits.flush_typ)) {
      debug_ghist := 0.U
    }

    var new_ghist = debug_ghist

    for (w <- 0 until coreWidth) {
      when (rob.io.commit.arch_valids(w) &&
        (rob.io.commit.uops(w).is_br || rob.io.commit.uops(w).is_jal || rob.io.commit.uops(w).is_jalr)) {
        // for (i <- 0 until globalHistoryLength) {
        //   printf("%x", new_ghist(globalHistoryLength-i-1))
        // }
        // printf("\n")
        printf("%x %x %x %x %x %x\n",
          rob.io.commit.uops(w).debug_fsrc, rob.io.commit.uops(w).taken,
          rob.io.commit.uops(w).is_br, rob.io.commit.uops(w).is_jal,
          rob.io.commit.uops(w).is_jalr, Sext(rob.io.commit.uops(w).debug_pc(vaddrBits-1,0), xLen))

      }
      new_ghist = Mux(rob.io.commit.arch_valids(w) && rob.io.commit.uops(w).is_br,
        Mux(rob.io.commit.uops(w).taken, new_ghist << 1 | 1.U(1.W), new_ghist << 1),
        new_ghist)
    }
    debug_ghist := new_ghist
  }

  // TODO: Does anyone want this debugging functionality?
  val coreMonitorBundle = Wire(new CoreMonitorBundle(xLen, fLen))
  coreMonitorBundle := DontCare
  coreMonitorBundle.clock  := clock
  coreMonitorBundle.reset  := reset


   //-------------------------------------------------------------
  //-------------------------------------------------------------
  // Pipeview Visualization
  val flush_ifu = brupdate.b2.mispredict || // In practice, means flushing everything prior to dispatch.
                  rob.io.flush.valid || // i.e. 'flush in-order part of the pipeline'
                  sfence_take_pc

  if (O3PIPEVIEW_PRINTF) {
    println("   O3Pipeview Visualization Enabled\n")

    // did we already print out the instruction sitting at the front of the fetchbuffer/decode stage?
    val dec_printed_mask = RegInit(0.U(coreWidth.W))

    for (w <- 0 until coreWidth) {
      when (dec_valids(w) && !dec_printed_mask(w)) {
        printf("%d; O3PipeView:decode:%d\n", dec_uops(w).debug_events.fetch_seq, debug_tsc_reg)
      }
      // Rename begins when uop leaves fetch buffer (Dec+Ren1 are in same stage).
      when (dec_fire(w)) {
        printf("%d; O3PipeView:rename:%d\n", dec_uops(w).debug_events.fetch_seq, debug_tsc_reg)
      }
      when (dispatcher.io.ren_uops(w).valid) {
        printf("%d; O3PipeView:dispatch:%d\n", dispatcher.io.ren_uops(w).bits.debug_events.fetch_seq, debug_tsc_reg)
      }

      when (dec_ready || flush_ifu) {
        dec_printed_mask := 0.U
      } .otherwise {
        dec_printed_mask := dec_valids.asUInt | dec_printed_mask
      }
    }

    for (i <- 0 until coreWidth) {
      when (rob.io.commit.valids(i)) {
        printf("%d; O3PipeView:retire:%d:store:0\n",
          rob.io.commit.uops(i).debug_events.fetch_seq,
          debug_tsc_reg)
      }
    }
  }

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // Page Table Walker

  io.ptw.ptbr       := csr.io.ptbr
  io.ptw.status     := csr.io.status
  io.ptw.pmp        := csr.io.pmp
  io.ptw.sfence     := io.ifu.sfence

  //-------------------------------------------------------------
  //-------------------------------------------------------------

  io.rocc := DontCare
  io.rocc.exception := csr.io.exception && csr.io.status.xs.orR
  if (usingRoCC) {
    exe_units.rocc_unit.io.rocc.rocc         <> io.rocc
    exe_units.rocc_unit.io.rocc.dis_uops     := disUops
    exe_units.rocc_unit.io.rocc.rob_head_idx := rob.io.rob_head_idx
    exe_units.rocc_unit.io.rocc.rob_pnr_idx  := rob.io.rob_pnr_idx
    exe_units.rocc_unit.io.com_exception     := rob.io.flush.valid
    exe_units.rocc_unit.io.status            := csr.io.status

    for (w <- 0 until coreWidth) {
      exe_units.rocc_unit.io.rocc.dis_rocc_vals(w) := (
        disFire(w) &&
        disUops(w).uopc === uopROCC &&
        !disUops(w).exception
      )
    }
  }

  if (usingTrace) {
    for (w <- 0 until coreWidth) {
      // Delay the trace so we have a cycle to pull PCs out of the FTQ
      io.trace(w).valid      := RegNext(rob.io.commit.arch_valids(w))

      // Recalculate the PC
      io.ifu.debug_ftq_idx(w) := rob.io.commit.uops(w).ftq_idx
      val iaddr = (AlignPCToBoundary(io.ifu.debug_fetch_pc(w), icBlockBytes)
                   + RegNext(rob.io.commit.uops(w).pc_lob)
                   - Mux(RegNext(rob.io.commit.uops(w).edge_inst), 2.U, 0.U))(vaddrBits-1,0)
      io.trace(w).iaddr      := Sext(iaddr, xLen)

      def getInst(uop: MicroOp, inst: UInt): UInt = {
        Mux(uop.is_rvc, Cat(0.U(16.W), inst(15,0)), inst)
      }

      def getWdata(uop: MicroOp, wdata: UInt): UInt = {
        Mux((uop.rt(RD, isInt) && uop.ldst =/= 0.U) || uop.rt(RD, isFloat), wdata, 0.U(xLen.W))
      }

      // use debug_insts instead of uop.debug_inst to use the rob's debug_inst_mem
      // note: rob.debug_insts comes 1 cycle later
      io.trace(w).insn       := getInst(RegNext(rob.io.commit.uops(w)), rob.io.commit.debug_insts(w))
      io.trace(w).wdata.map { _ := RegNext(getWdata(rob.io.commit.uops(w), rob.io.commit.debug_wdata(w))) }

      // Comment out this assert because it blows up FPGA synth-asserts
      // This tests correctedness of the debug_inst mem
      // when (RegNext(rob.io.commit.valids(w))) {
      //   assert(rob.io.commit.debug_insts(w) === RegNext(rob.io.commit.uops(w).debug_inst))
      // }
      // This tests correctedness of recovering pcs through ftq debug ports
      // when (RegNext(rob.io.commit.valids(w))) {
      //   assert(Sext(io.trace(w).iaddr, xLen) ===
      //     RegNext(Sext(rob.io.commit.uops(w).debug_pc(vaddrBits-1,0), xLen)))
      // }

      // These csr signals do not exactly match up with the ROB commit signals.
      io.trace(w).priv       := RegNext(csr.io.status.prv)
      // Can determine if it is an interrupt or not based on the MSB of the cause
      io.trace(w).exception  := RegNext(rob.io.com_xcpt.valid && !rob.io.com_xcpt.bits.cause(xLen - 1))
      io.trace(w).interrupt  := RegNext(rob.io.com_xcpt.valid && rob.io.com_xcpt.bits.cause(xLen - 1))
      io.trace(w).cause      := RegNext(rob.io.com_xcpt.bits.cause)
      io.trace(w).tval       := RegNext(csr.io.tval)
    }
    dontTouch(io.trace)
  } else {
    io.trace := DontCare
    io.trace map (t => t.valid := false.B)
    io.ifu.debug_ftq_idx := DontCare
  }

  def uopConvertToVuop(uop: MicroOp): VLSMicroOP = {
    val vuop = WireInit(0.U.asTypeOf(new VLSMicroOP(vlsuparam.get)))
    vuop.vLdQIdx := uop.ldq_idx
    vuop.vStQIdx := uop.stq_idx
    vuop.robIdx := uop.rob_idx
    vuop.uCtrlSig.accessType.isLoad := uop.uses_ldq
    vuop.uCtrlSig.accessType.isStore := uop.uses_stq
    vuop.uCtrlSig.accessStyle.isUnitStride := uop.uopc.isOneOf(uopVL, uopVLFF, uopVSA)
    vuop.uCtrlSig.accessStyle.isConstantStride := uop.uopc.isOneOf(uopVLS, uopVSSA)
    vuop.uCtrlSig.accessStyle.isSegment := false.B //fixme: explicit segment required in uop.
    vuop.uCtrlSig.accessStyle.isWholeAccess := uop.uopc.isOneOf(uopVLR, uopVSR)
    vuop.uCtrlSig.accessStyle.isIndexed := uop.uopc.isOneOf(uopVLUX, uopVLOX, uopVSOXA, uopVSUXA)
    vuop.uCtrlSig.accessStyle.dataEew := uop.vd_eew
    vuop.uCtrlSig.accessStyle.indexEew := uop.vs2_eew
    vuop.uCtrlSig.accessStyle.vStart := uop.vstart
    vuop.uCtrlSig.accessStyle.vl := uop.vconfig.vl
    vuop.uCtrlSig.accessStyle.vlmul := uop.vd_emul
    vuop.uCtrlSig.accessStyle.nf := uop.v_seg_nf
    vuop.uCtrlSig.accessStyle.fieldIdx := uop.v_seg_f
    vuop.uCtrlSig.accessStyle.vma := uop.vconfig.vtype.vma
    vuop.uCtrlSig.accessStyle.vta := uop.vconfig.vtype.vta
    vuop.vs1 := 0.U
    vuop.vs2 := 0.U
    vuop.rs1 := 0.U
    vuop.rs2 := 0.U
    vuop.vm := 0.U
    vuop.vpdst := Mux(uop.uses_ldq, VecInit(uop.pvd.map(_.bits)), VecInit(uop.stale_pvd.map(_.bits)))
    vuop.staleRegIdxes := VecInit(uop.stale_pvd.map(_.bits))
    vuop
  }
}
