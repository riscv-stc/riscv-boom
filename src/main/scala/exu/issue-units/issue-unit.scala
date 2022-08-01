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
  val matrix: Boolean = false
)(implicit p: Parameters) extends BoomBundle {
  val pdst = UInt(width=pregSz.W)
  val poisoned = Bool()
  val uop = if (vector || matrix) new MicroOp() else null
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
  val vector: Boolean = false,
  val matrix: Boolean = false)
(implicit p: Parameters) extends BoomBundle {
  val dis_uops         = Vec(dispatchWidth, Flipped(Decoupled(new MicroOp)))

  val iss_valids       = Output(Vec(issueWidth, Bool()))
  val iss_uops         = Output(Vec(issueWidth, new MicroOp()))
  val wakeup_ports     = Flipped(Vec(numWakeupPorts, Valid(new IqWakeup(maxPregSz, vector, matrix))))
  val pred_wakeup_port = Flipped(Valid(UInt(log2Ceil(ftqSz).W)))
  val vl_wakeup_port   = Flipped(Valid(new VlWakeupResp()))

  val spec_ld_wakeup   = Flipped(Vec(memWidth, Valid(UInt(width=maxPregSz.W))))

  // tell the issue unit what each execution pipeline has in terms of functional units
  val fu_types         = Input(Vec(issueWidth, Bits(width=FUC_SZ.W)))

  val brupdate         = Input(new BrUpdateInfo())
  //val vmupdate         = if (usingVector && !vector) Input(Vec(vecWidth, Valid(new MicroOp))) else null
  val intupdate        = if (vector || matrix) Input(Vec(intWidth, Valid(new ExeUnitResp(eLen)))) else null
  val fpupdate         = if (vector) Input(Vec(fpWidth, Valid(new ExeUnitResp(eLen)))) else null
  //val vecUpdate        = if (vector) Input(Vec(vecWidth, Valid(new ExeUnitResp(eLen)))) else null
  val vbusy_status     = if (vector) Input(UInt(numVecPhysRegs.W)) else null
  val flush_pipeline   = Input(Bool())
  val ld_miss          = Input(Bool())

  val event_empty      = Output(Bool()) // used by HPM events; is the issue unit empty?

  val tsc_reg          = Input(UInt(width=xLen.W))

  val perf = Output(new Bundle {
    val empty = Bool()
    val full  = Bool()
  })
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
  val vector: Boolean = false,
  val matrix: Boolean = false)
(implicit p: Parameters) extends BoomModule with IssueUnitConstants {
  val io = IO(new IssueUnitIO(issueWidth, numWakeupPorts, dispatchWidth, vector, matrix))

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
      when ((dis_uops(w).uopc === uopSTA && dis_uops(w).rt(RS2, isInt)) ||
             dis_uops(w).uopc === uopAMO_AG) {
        dis_uops(w).iw_state := s_valid_2
      } .elsewhen (dis_uops(w).uopc.isOneOf(uopSTA, uopVSA) && dis_uops(w).rt(RS2, isNotInt)) {
        // For store addr gen for FP, rs2 is the FP/VEC register, and we don't wait for that here
        when (dis_uops(w).fp_val) {
          //dis_uops(w).lrs2_rtype := RT_X
          dis_uops(w).prs2_busy := 0.U
        }
      }
      if (usingMatrix) {
        when (dis_uops(w).is_rvm) {
          dis_uops(w).prs3_busy := 0.U
        }
        if (iqType == IQT_INT.litValue) {
          when (io.dis_uops(w).valid && dis_uops(w).is_rvm && !dis_uops(w).rt(RD, isInt)) {
            assert(dis_uops(w).uses_scalar, "unexpected rvv in INT pipe")
            dis_uops(w).fu_code := FU_ALU
          }
        }
      }
      if (usingVector) {
        when (dis_uops(w).is_rvv && dis_uops(w).v_idx_ls) {
          dis_uops(w).prs2_busy := 0.U
          //dis_uops(w).prvm_busy := 1.U // Force waiting on vmupdate for indexed load/store
        }
        when (dis_uops(w).is_rvv && !dis_uops(w).v_unmasked) {
          //dis_uops(w).prvm_busy := 1.U // Force waiting on vmupdate for indexed load/store
        }
        if (iqType == IQT_INT.litValue) {
          when (io.dis_uops(w).valid && dis_uops(w).is_rvv && !dis_uops(w).uopc.isOneOf(uopVSETVL, uopVSETVLI, uopVSETIVLI)) {
            assert(dis_uops(w).uses_scalar, "unexpected rvv in INT pipe")
            dis_uops(w).fu_code := FU_ALU
          }
          when (dis_uops(w).rt(RS2, isVector)) { dis_uops(w).prs2_busy := 0.U }
          when (dis_uops(w).frs3_en )          { dis_uops(w).prs3_busy := 0.U }
        } else { // iqType == IQT_MEM.litvalue
          when (io.dis_uops(w).valid && dis_uops(w).is_rvv && (dis_uops(w).uses_ldq || dis_uops(w).uses_stq)) {
            val vd_idx = VRegSel(dis_uops(w).v_eidx, dis_uops(w).vd_eew, eLenSelSz)
            dis_uops(w).pdst := dis_uops(w).pvd(vd_idx).bits
          }
        }
      }
      dis_uops(w).prs3_busy := 0.U
    } else if (iqType == IQT_FP.litValue) {
      // FP "StoreAddrGen" is really storeDataGen, and rs1 is the integer address register
      when (dis_uops(w).uopc.isOneOf(uopSTA)) {
        //dis_uops(w).lrs1_rtype := RT_FIX
        dis_uops(w).prs1_busy  := 0.U
      }
      if (usingVector) {
        //dis_uops(w).prvm_busy := 0.U
        when (dis_uops(w).rt(RS2, isVector)) { dis_uops(w).prs2_busy := 0.U }
        when (dis_uops(w).is_rvv )           { dis_uops(w).prs3_busy := 0.U }
        // hack fu_code if the instruction is merge.
        when (dis_uops(w).is_rvv && dis_uops(w).uopc.isOneOf(uopMERGE)){
          dis_uops(w).fu_code := FU_FPU
        }
      }
    //} else if (iqType == IQT_VEC.litValue) {
    //} else if (iqType == IQT_VMX.litValue) {
      // VEC "StoreAddrGen" is really storeDataGen, and rs1 is the integer address register
      // VEC Load that arrives here are tail splits, prs3 holds stale register name, read in RRD
      //when (dis_uops(w).uopc.isOneOf(uopVL, uopVLFF, uopVLS)) {
      //  //dis_uops(w).lrs1_rtype := RT_FIX
      //  dis_uops(w).prs1_busy   := 0.U
      //  dis_uops(w).prs2_busy   := 0.U
      //  // NOTE: for unit-stride load, do undisturb load
      //  dis_uops(w).v_split_ecnt:= (vLenb.U >> dis_uops(w).vd_eew) * nrVecGroup(dis_uops(w).vd_emul, dis_uops(w).v_seg_nf)
      //  dis_uops(w).v_eidx      := dis_uops(w).vconfig.vl
      //  dis_uops(w).fu_code     := FU_VMX
      //  dis_uops(w).uses_ldq    := false.B
      //} .elsewhen (dis_uops(w).uopc.isOneOf(uopVSA, uopVSSA)) {
      //  dis_uops(w).prs1_busy   := 0.U
      //  dis_uops(w).prs2_busy   := 0.U
      //} .elsewhen (dis_uops(w).uopc.isOneOf(uopVLUX, uopVSUXA, uopVLOX, uopVSOXA)) {
      //  dis_uops(w).prs1_busy   := 0.U
      //}
    } else if (iqType == IQT_MAT.litValue) {
      when(dis_uops(w).uopc.isOneOf(uopMOPA, uopMWOPA, uopMQOPA, uopMFOPA, uopMFWOPA)) {
        dis_uops(w).prs1_busy := false.B
        dis_uops(w).prs2_busy := false.B
        dis_uops(w).prs3_busy := false.B
      } .elsewhen(dis_uops(w).uopc.isOneOf(uopMMV_V, uopMWMV_V, uopMQMV_V)) {
        dis_uops(w).prs1_busy := false.B
        dis_uops(w).pts2_busy := 0.U
        dis_uops(w).pts3_busy := 0.U
      } .elsewhen(dis_uops(w).uopc.isOneOf(uopMMUL, uopMWMUL, uopMQMUL)) {
        dis_uops(w).pts2_busy := 0.U
      }
    }

    if (iqType != IQT_INT.litValue) {
      assert(!(dis_uops(w).ppred_busy && io.dis_uops(w).valid))
      dis_uops(w).ppred_busy := false.B
    }
  }

  //-------------------------------------------------------------
  // Issue Table

  val slots = for (i <- 0 until numIssueSlots) yield {
    val slot = Module(new IssueSlot(numWakeupPorts, iqType, vector, matrix))
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
    issue_slots(i).vl_wakeup_port   := io.vl_wakeup_port
    if (usingMatrix && matrix) {
      issue_slots(i).intupdate      := io.intupdate
    } else if (usingVector) {
      if (vector) {
        issue_slots(i).vbusy_status   := io.vbusy_status
        issue_slots(i).intupdate      := io.intupdate
        issue_slots(i).fpupdate       := io.fpupdate
        //issue_slots(i).vecUpdate      := io.vecUpdate
      //} else {
        //issue_slots(i).vmupdate       := io.vmupdate
      }
    }
  }

  io.event_empty := !(issue_slots.map(s => s.valid).reduce(_|_))

  val count = PopCount(slots.map(_.io.valid))
  dontTouch(count)

  io.perf.empty := io.event_empty
  io.perf.full  := count === numIssueSlots.U
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
    else if (iqType == IQT_MAT.litValue) "mat"
    else "unknown"
}
