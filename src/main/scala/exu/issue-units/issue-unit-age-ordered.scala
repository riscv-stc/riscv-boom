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
import chisel3.util.{PopCount, PriorityMux, RegEnable, log2Ceil}
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.util.{Str, UIntIsOneOf}
import FUConstants._
import boom.common.MicroOpcodes._
import boom.common._
import chisel3.util.BitPat.bitPatToUInt

object FusionCode extends Enumeration {
  val CLR_ID = Value
  val CVT_ID = Value
  val WMA_ID = Value
  val INV_ID = Value
  val END_ID = Value

  val FUS_SZ = END_ID.id
  val FC_CLR = (1 << CLR_ID.id).U(FUS_SZ.W)
  val FC_CVT = (1 << CVT_ID.id).U(FUS_SZ.W)
  val FC_WMA = (1 << WMA_ID.id).U(FUS_SZ.W)
  val FC_INV = (1 << INV_ID.id).U(FUS_SZ.W)
}

import boom.exu.FusionCode._

/**
 * Specific type of issue unit
 *
 * @param params issue queue params
 * @param numWakeupPorts number of wakeup ports for the issue queue
 */
class IssueUnitCollapsing(
  params: IssueParams,
  numWakeupPorts: Int,
  vector: Boolean = false,
  matrix: Boolean = false,
  fusedUop: Boolean = true)
(implicit p: Parameters) extends IssueUnit(
  params.numEntries,
  params.issueWidth,
  numWakeupPorts,
  params.iqType,
  params.dispatchWidth,
  vector,
  matrix
) {
  
  //-------------------------------------------------------------
  // which entries' uops will still be next cycle? (not being issued and vacated)
  val will_be_valid = (0 until numIssueSlots).map(i => issue_slots(i).will_be_valid) ++
                      (0 until dispatchWidth).map(i => io.dis_uops(i).valid &&
                                                            !dis_uops(i).exception &&
                                                            !dis_uops(i).is_fence &&
                                                            !dis_uops(i).is_fencei)
  
  val uops = issue_slots.map(s => s.out_uop) ++ dis_uops.map(s => s)
  
  //-------------------------------------------------------------
  // Figure out how much to shift entries by

  val maxShift = dispatchWidth
  val vacants = issue_slots.map(s => !(s.valid)) ++ io.dis_uops.map(_.valid).map(!_.asBool)
  val shamts_oh = Array.fill(numIssueSlots+dispatchWidth) {Wire(UInt(width=maxShift.W))}
  // track how many to shift up this entry by by counting previous vacant spots
  def SaturatingCounterOH(count_oh:UInt, inc: Bool, max: Int): UInt = {
     val next = Wire(UInt(width=max.W))
     next := count_oh
     when (count_oh === 0.U && inc) {
       next := 1.U
     } .elsewhen (!count_oh(max-1) && inc) {
       next := (count_oh << 1.U)
     }
     next
  }
  shamts_oh(0) := 0.U
  for (i <- 1 until numIssueSlots + dispatchWidth) {
    shamts_oh(i) := SaturatingCounterOH(shamts_oh(i-1), vacants(i-1), maxShift)
  }

  //-------------------------------------------------------------


  for (i <- 0 until numIssueSlots) {
    issue_slots(i).in_uop.valid := false.B
    issue_slots(i).in_uop.bits  := uops(i+1)
    for (j <- 1 to maxShift by 1) {
      when (shamts_oh(i+j) === (1 << (j-1)).U) {
        issue_slots(i).in_uop.valid := will_be_valid(i+j)
        issue_slots(i).in_uop.bits  := uops(i+j)
      }
    }
    issue_slots(i).clear        := shamts_oh(i) =/= 0.U
  }

  //-------------------------------------------------------------
  // Dispatch/Entry Logic
  // did we find a spot to slide the new dispatched uops into?

  val will_be_available = (0 until numIssueSlots).map(i =>
                            (!issue_slots(i).will_be_valid || issue_slots(i).clear) && !(issue_slots(i).in_uop.valid))
  val num_available = PopCount(will_be_available)
  for (w <- 0 until dispatchWidth) {
    io.dis_uops(w).ready := RegNext(num_available > w.U)
  }

  //-------------------------------------------------------------
  // Issue Select Logic

  // set default
  for (w <- 0 until issueWidth) {
    io.iss_valids(w) := false.B
    io.wake_tile_r(w) := false.B
    io.iss_uops(w)   := NullMicroOp()
    // unsure if this is overkill
    io.iss_uops(w).prs1 := 0.U
    io.iss_uops(w).prs2 := 0.U
    io.iss_uops(w).prs3 := 0.U
    io.iss_uops(w).lrs1_rtype := RT_X
    io.iss_uops(w).lrs2_rtype := RT_X
  }

  val requests = issue_slots.map(s => s.request)
  val port_issued = Array.fill(issueWidth){Bool()}
  for (w <- 0 until issueWidth) {
    port_issued(w) = false.B
  }

  val iss_valids_prev = RegNext(io.iss_valids)
  val iss_fucode_prev = RegNext(io.iss_uops)
  
  val valid_bits_prev = (0 until issueWidth).map(i => RegEnable(true.B, false.B, io.iss_valids(i)))
  val valid_uops_prev = (0 until issueWidth).map(i => RegEnable(io.iss_uops(i), io.iss_valids(i)))

  for (i <- 0 until numIssueSlots) {
    issue_slots(i).grant := false.B
    var uop_issued = false.B

    for (w <- 0 until issueWidth) {
      val can_allocate = (issue_slots(i).uop.fu_code & io.fu_types(w)) =/= 0.U
      // Div (vdiv) cannot be issued continuously
      val isDivPrev = iss_valids_prev(w) && iss_fucode_prev(w).fu_code === FU_DIV
      val isDivCurr = issue_slots(i).uop.fu_code === FU_DIV
      val canDivIss = !(isDivCurr && isDivPrev)
      // mmv cannot be issued continuously
      val canMMVIss = if (matrix) WireInit(true.B) else true.B
      val canMACIss = if (matrix) WireInit(true.B) else true.B
      if (matrix) {
        val isMMVPrev = iss_valids_prev(w) && (iss_fucode_prev(w).fu_code === FU_HSLICE || iss_fucode_prev(w).fu_code === FU_VSLICE)
        val isMMVCurr = issue_slots(i).uop.fu_code === FU_HSLICE || issue_slots(i).uop.fu_code === FU_VSLICE
        canMMVIss    := !(isMMVCurr && isMMVPrev)
        
        val isMACPrev = valid_bits_prev(w) && valid_uops_prev(w).fu_code === FU_GEMM && !valid_uops_prev(w).m_split_last
        val isMACCurr = issue_slots(i).uop.fu_code === FU_GEMM && issue_slots(i).uop.rob_idx =/= valid_uops_prev(w).rob_idx
        canMACIss    := !(isMACCurr && isMACPrev)
//        dontTouch(isMACPrev)
//        dontTouch(isMACCurr)
//        dontTouch(canMACIss)
      }

      when (requests(i) && !uop_issued && can_allocate && !port_issued(w) && canDivIss && canMMVIss && canMACIss) {
        issue_slots(i).grant := true.B
        io.iss_valids(w) := true.B
        io.iss_uops(w) := issue_slots(i).uop
        io.wake_tile_r(w) := issue_slots(i).uop.is_rvm && issue_slots(i).uop.m_split_last
      }
      val was_port_issued_yet = port_issued(w)
      //port_issued(w) = (requests(i) && !uop_issued && can_allocate) | port_issued(w)
      port_issued(w) = (requests(i) && !uop_issued && can_allocate && canDivIss && canMMVIss && canMACIss) | port_issued(w)
      //uop_issued = (requests(i) && can_allocate && !was_port_issued_yet) | uop_issued
      uop_issued = (requests(i) && can_allocate && !was_port_issued_yet && canDivIss && canMMVIss && canMACIss) | uop_issued
    }
  }
}
