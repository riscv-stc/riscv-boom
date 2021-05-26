//******************************************************************************
// Copyright (c) 2015 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// RISCV Processor Issue Logic
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.exu

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.Parameters
import freechips.rocketchip.util._

import boom.common._
import boom.common.MicroOpcodes._
import boom.exu.FUConstants._
import boom.util._

/**
 * Class used for configurations
 *
 * @param issueWidth amount of things that can be issued
 * @param numEntries size of issue queue
 * @param iqType type of issue queue
 */
case class IssueParams(
  dispatchWidth: Int = 1,
  issueWidth: Int = 1,
  numEntries: Int = 8,
  iqType: BigInt
)

/**
 * Constants for knowing about the status of a MicroOp
 */
trait IssueUnitConstants
{
  // invalid  : slot holds no valid uop.
  // s_valid_1: slot holds a valid uop.
  // s_valid_2: slot holds a store-like uop that may be broken into two micro-ops.
  val s_invalid :: s_valid_1 :: s_valid_2 :: Nil = Enum(3)
}

/**
 * What physical register is broadcasting its wakeup?
 * Is the physical register poisoned (aka, was it woken up by a speculative issue)?
 *
 * @param pregSz size of physical destination register
 */
class IqWakeup(
  val pregSz: Int,
  val vector: Boolean = false,
)(implicit p: Parameters) extends BoomBundle {
  val pdst = UInt(width=pregSz.W)
  val poisoned = Bool()
  val uop = if (vector) new MicroOp() else null
}

/**
 * IO bundle to interact with the issue unit
 *
 * @param issueWidth amount of operations that can be issued at once
 * @param numWakeupPorts number of wakeup ports for issue unit
 */
class IssueUnitIO(
  val issueWidth: Int,
  val numWakeupPorts: Int,
  val dispatchWidth: Int,
  val vector: Boolean = false)
(implicit p: Parameters) extends BoomBundle {
  val dis_uops         = Vec(dispatchWidth, Flipped(Decoupled(new MicroOp)))

  val iss_valids       = Output(Vec(issueWidth, Bool()))
  val iss_uops         = Output(Vec(issueWidth, new MicroOp()))
  val wakeup_ports     = Flipped(Vec(numWakeupPorts, Valid(new IqWakeup(maxPregSz, vector))))
  val pred_wakeup_port = Flipped(Valid(UInt(log2Ceil(ftqSz).W)))

  val spec_ld_wakeup   = Flipped(Vec(memWidth, Valid(UInt(width=maxPregSz.W))))

  // tell the issue unit what each execution pipeline has in terms of functional units
  val fu_types         = Input(Vec(issueWidth, Bits(width=FUC_SZ.W)))

  val brupdate         = Input(new BrUpdateInfo())
  val vmupdate         = if (usingVector && !vector) Input(Vec(vecWidth, Valid(new MicroOp))) else null
  val intupdate        = if (usingVector && vector) Input(Vec(intWidth, Valid(new ExeUnitResp(eLen)))) else null
  val fpupdate         = if (usingVector && vector) Input(Vec(fpWidth, Valid(new ExeUnitResp(eLen)))) else null
  val flush_pipeline   = Input(Bool())
  val ld_miss          = Input(Bool())

  val event_empty      = Output(Bool()) // used by HPM events; is the issue unit empty?

  val tsc_reg          = Input(UInt(width=xLen.W))
}

/**
 * Abstract top level issue unit
 *
 * @param numIssueSlots depth of issue queue
 * @param issueWidth amount of operations that can be issued at once
 * @param numWakeupPorts number of wakeup ports for issue unit
 * @param iqType type of issue queue (mem, int, fp)
 */
abstract class IssueUnit(
  val numIssueSlots: Int,
  val issueWidth: Int,
  val numWakeupPorts: Int,
  val iqType: BigInt,
  val dispatchWidth: Int,
  val vector: Boolean = false)
(implicit p: Parameters) extends BoomModule with IssueUnitConstants {
  val io = IO(new IssueUnitIO(issueWidth, numWakeupPorts, dispatchWidth, vector))

  //-------------------------------------------------------------
  // Set up the dispatch uops
  // special case "storing" 2 uops within one issue slot.

  val dis_uops = Array.fill(dispatchWidth) {Wire(new MicroOp())}
  for (w <- 0 until dispatchWidth) {
    dis_uops(w) := io.dis_uops(w).bits
    dis_uops(w).iw_p1_poisoned := false.B
    dis_uops(w).iw_p2_poisoned := false.B
    dis_uops(w).iw_state := s_valid_1

    if (iqType == IQT_MEM.litValue || iqType == IQT_INT.litValue) {
      // For StoreAddrGen for Int, or AMOAddrGen, we go to addr gen state
      when ((io.dis_uops(w).bits.uopc === uopSTA && io.dis_uops(w).bits.rt(RS2, isInt)) ||
             io.dis_uops(w).bits.uopc === uopAMO_AG) {
        dis_uops(w).iw_state := s_valid_2
      } .elsewhen (io.dis_uops(w).bits.uopc.isOneOf(uopSTA, uopVSA) && io.dis_uops(w).bits.rt(RS2, isNotInt)) {
        // For store addr gen for FP, rs2 is the FP/VEC register, and we don't wait for that here
        when (io.dis_uops(w).bits.fp_val) {
          //dis_uops(w).lrs2_rtype := RT_X
          dis_uops(w).prs2_busy  := 0.U
        }
      }
      dis_uops(w).prs3_busy := 0.U
      if (usingVector && iqType == IQT_INT.litValue) {
        when (io.dis_uops(w).valid && io.dis_uops(w).bits.is_rvv && !io.dis_uops(w).bits.uopc.isOneOf(uopVSETVL, uopVSETVLI, uopVSETIVLI)) {
          assert(io.dis_uops(w).bits.uses_scalar, "unexpected rvv in INT pipe")
          dis_uops(w).fu_code := FU_ALU
        }
      } else {
        dis_uops(w).prvm_busy := 0.U
      }
      if (usingVector && iqType == IQT_INT.litValue) {
        when (dis_uops(w).rt(RS2, isVector)) { dis_uops(w).prs2_busy := 0.U }
        when (dis_uops(w).frs3_en )          { dis_uops(w).prs3_busy := 0.U }
      }
    } else if (iqType == IQT_FP.litValue) {
      // FP "StoreAddrGen" is really storeDataGen, and rs1 is the integer address register
      when (io.dis_uops(w).bits.uopc.isOneOf(uopSTA)) {
        //dis_uops(w).lrs1_rtype := RT_FIX
        dis_uops(w).prs1_busy  := 0.U
      }
      if (usingVector) {
        dis_uops(w).prvm_busy := 0.U
        when (dis_uops(w).rt(RS2, isVector)) { dis_uops(w).prs2_busy := 0.U }
        when (dis_uops(w).is_rvv )           { dis_uops(w).prs3_busy := 0.U }
      }
    } else if (iqType == IQT_VEC.litValue) {
      // VEC "StoreAddrGen" is really storeDataGen, and rs1 is the integer address register
      // VEC Load that arrives here are tail splits, prs3 holds stale register name, read in RRD
      when (io.dis_uops(w).bits.uopc.isOneOf(uopVSA, uopVL)) {
        //dis_uops(w).lrs1_rtype := RT_FIX
        dis_uops(w).prs1_busy  := 0.U
        dis_uops(w).prs2_busy  := 0.U
      }
    }

    if (iqType != IQT_INT.litValue) {
      assert(!(io.dis_uops(w).bits.ppred_busy && io.dis_uops(w).valid))
      dis_uops(w).ppred_busy := false.B
    }
  }

  //-------------------------------------------------------------
  // Issue Table

  val slots = for (i <- 0 until numIssueSlots) yield {
    val slot = Module(new IssueSlot(numWakeupPorts, iqType, vector));
    slot
  }
  val issue_slots = VecInit(slots.map(_.io))

  for (i <- 0 until numIssueSlots) {
    issue_slots(i).wakeup_ports     := io.wakeup_ports
    issue_slots(i).pred_wakeup_port := io.pred_wakeup_port
    issue_slots(i).spec_ld_wakeup   := io.spec_ld_wakeup
    issue_slots(i).ldspec_miss      := io.ld_miss
    issue_slots(i).brupdate         := io.brupdate
    issue_slots(i).kill             := io.flush_pipeline
    if (usingVector) {
      if (vector) {
        issue_slots(i).intupdate      := io.intupdate
        issue_slots(i).fpupdate       := io.fpupdate
      } else {
        issue_slots(i).vmupdate       := io.vmupdate
      }
    }
  }

  io.event_empty := !(issue_slots.map(s => s.valid).reduce(_|_))

  val count = PopCount(slots.map(_.io.valid))
  dontTouch(count)

  //-------------------------------------------------------------

  assert (PopCount(issue_slots.map(s => s.grant)) <= issueWidth.U, "[issue] window giving out too many grants.")


  //-------------------------------------------------------------

  if (O3PIPEVIEW_PRINTF) {
    for (i <- 0 until issueWidth) {
      // only print stores once!
      when (io.iss_valids(i) && io.iss_uops(i).uopc =/= uopSTD) {
         printf("%d; O3PipeView:issue:%d\n",
           io.iss_uops(i).debug_events.fetch_seq,
           io.tsc_reg)
      }
    }
  }

  def getType: String =
    if (iqType == IQT_INT.litValue) "int"
    else if (iqType == IQT_MEM.litValue) "mem"
    else if (iqType == IQT_FP.litValue) " fp"
    else if (iqType == IQT_VEC.litValue) "vec"
    else "unknown"
}
