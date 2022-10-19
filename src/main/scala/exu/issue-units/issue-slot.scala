//******************************************************************************
// Copyright (c) 2015 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// RISCV Processor Issue Slot Logic
//--------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// Note: stores (and AMOs) are "broken down" into 2 uops, but stored within a single issue-slot.
// TODO XXX make a separate issueSlot for MemoryIssueSlots, and only they break apart stores.
// TODO Disable ldspec for FP queue.

package boom.exu

import Chisel.UInt
import chisel3._
import chisel3.util._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.util.{UIntIsOneOf, UIntToAugmentedUInt}
import boom.common._
import boom.common.MicroOpcodes._
import boom.util._
import FUConstants._
import freechips.rocketchip.util._

/**
 * IO bundle to interact with Issue slot
 *
 * @param numWakeupPorts number of wakeup ports for the slot
 */
class IssueSlotIO(val numWakeupPorts: Int, val vector: Boolean = false, val matrix: Boolean = false)
(implicit p: Parameters) extends BoomBundle {
  val valid         = Output(Bool())
  val will_be_valid = Output(Bool()) // TODO code review, do we need this signal so explicitely?
  val request       = Output(Bool())
  val request_hp    = Output(Bool())
  val grant         = Input(Bool())

  val brupdate      = Input(new BrUpdateInfo())
  val kill          = Input(Bool()) // pipeline flush
  val clear         = Input(Bool()) // entry being moved elsewhere (not mutually exclusive with grant)
  val ldspec_miss   = Input(Bool()) // Previous cycle's speculative load wakeup was mispredicted.

  val wakeup_ports  = Flipped(Vec(numWakeupPorts, Valid(new IqWakeup(maxPregSz, vector, matrix))))
  val pred_wakeup_port = Flipped(Valid(UInt(log2Ceil(ftqSz).W)))
  val vl_wakeup = Flipped(Valid(new VlWakeupResp()))
  val spec_ld_wakeup= Flipped(Vec(memWidth, Valid(UInt(width=maxPregSz.W))))
  val in_uop        = Flipped(Valid(new MicroOp())) // if valid, this WILL overwrite an entry!
  val out_uop       = Output(new MicroOp()) // the updated slot uop; will be shifted upwards in a collasping queue.
  val uop           = Output(new MicroOp()) // the current Slot's uop. Sent down the pipeline when issued.
  val intupdate     = if (vector || matrix) Input(Vec(intWidth, Valid(new ExeUnitResp(eLen)))) else null
  val fpupdate      = if (vector) Input(Vec(fpWidth, Valid(new ExeUnitResp(eLen)))) else null
  val vbusy_status  = if (vector) Input(UInt(numVecPhysRegs.W)) else null

  val debug = {
    val result = new Bundle {
      val p1 = if (matrix) UInt(vLenb.W) else Bool()
      val p2 = if (matrix) UInt(vLenb.W) else Bool()
      val p3 = if (matrix) UInt(vLenb.W) else Bool()
      val ppred = Bool()
      val state = UInt(width=2.W)
    }
    Output(result)
  }
}

/**
 * Single issue slot. Holds a uop within the issue queue
 *
 * @param numWakeupPorts number of wakeup ports
 */
class IssueSlot(
  val numWakeupPorts: Int,
  val iqType: BigInt = IQT_INT.litValue,
  val vector: Boolean = false,
  val matrix: Boolean = false
)(implicit p: Parameters)
  extends BoomModule
  with IssueUnitConstants
{
  val io = IO(new IssueSlotIO(numWakeupPorts, vector, matrix))

  // slot invalid?
  // slot is valid, holding 1 uop
  // slot is valid, holds 2 uops (like a store)
  def is_invalid = state === s_invalid
  def is_valid = state =/= s_invalid
  val io_fire = io.request && io.grant

  val next_state      = Wire(UInt()) // the next state of this slot (which might then get moved to a new slot)
  val next_uopc       = Wire(UInt()) // the next uopc of this slot (which might then get moved to a new slot)
  val next_lrs1_rtype = Wire(UInt()) // the next reg type of this slot (which might then get moved to a new slot)
  val next_lrs2_rtype = Wire(UInt()) // the next reg type of this slot (which might then get moved to a new slot)

  val state = RegInit(s_invalid)
  val p1    = if(matrix) RegInit(0.U(vLenb.W)) else RegInit(false.B)
  val p2    = if(matrix) RegInit(0.U(vLenb.W)) else RegInit(false.B)
  val p3    = if(matrix) RegInit(0.U(vLenb.W)) else RegInit(false.B)
  val ps    = if(vector || matrix) RegInit(false.B) else null
  val sdata = if(vector) RegInit(0.U(eLen.W)) else null
  val sidx  = if(matrix) RegInit(0.U(vLenSz.W)) else null
  val ppred = RegInit(false.B)
  val slot_uop = RegInit(NullMicroOp(usingVector))
  val vl_ready = WireInit(slot_uop.vl_ready)
  // Poison if woken up by speculative load.
  // Poison lasts 1 cycle (as ldMiss will come on the next cycle).
  // SO if poisoned is true, set it to false!
  val p1_poisoned = RegInit(false.B)
  val p2_poisoned = RegInit(false.B)
  p1_poisoned := false.B
  p2_poisoned := false.B
  val next_p1_poisoned = Mux(io.in_uop.valid, io.in_uop.bits.iw_p1_poisoned, p1_poisoned)
  val next_p2_poisoned = Mux(io.in_uop.valid, io.in_uop.bits.iw_p2_poisoned, p2_poisoned)

  val next_uop = WireInit(Mux(io.in_uop.valid, io.in_uop.bits, slot_uop))
  val vcompress = slot_uop.uopc === uopVCOMPRESS
  val vrg_has_vupd = slot_uop.rt(RS1, isVector) && slot_uop.uopc === uopVRGATHER || slot_uop.uopc === uopVRGATHEREI16
  val next_uop_has_vupd = next_uop.rt(RS1, isVector) && next_uop.uopc === uopVRGATHER || next_uop.uopc.isOneOf(uopVRGATHEREI16, uopVCOMPRESS)
  val next_uop_vcompress = next_uop.uopc === uopVCOMPRESS
  val next_uop_mma        = if (matrix) next_uop.uopc.isOneOf(uopMMA, uopMWMA, uopMQMA) else null
  val next_uop_ts1_hslice = if (matrix) Mux(next_uop_mma, false.B, next_uop.isHSlice) else null
  val next_uop_ts2_hslice = if (matrix) Mux(next_uop_mma, true.B,  next_uop.isHSlice) else null

  val last_check: Bool = {
    val ret = Wire(Bool())
    if (vector) {
      ret := io.uop.v_split_last || io.uop.is_reduce || vcompress
    } else if(matrix) {
      ret := io.uop.m_split_last
    } else {
      ret := true.B
    }
    ret
  }

  val rs1check: Bool = {
    val ret = Wire(Bool())
    val uop = slot_uop
    if (vector) {
      val vsew   = uop.vs1_eew
      val vLen_ecnt   = vLenb.U >> vsew
      val v_eidx = Mux(uop.is_reduce, 0.U, uop.v_eidx)
      when(vcompress) {  
        ret := (0 until 8).toList.map(i => {
          val rsel   = VRegSel(v_eidx+& vLen_ecnt*i.U, uop.vs1_eew, eLenSelSz)
          val pvs1   = uop.pvs1(rsel).bits
          !io.vbusy_status(pvs1)
        }).reduceLeft(_&_)
      }.otherwise {
        val rsel   = VRegSel(v_eidx, uop.vs1_eew, eLenSelSz)
        val pvs1   = uop.pvs1(rsel).bits
        ret       := Mux(uop.rt(RS1, isVector), !io.vbusy_status(pvs1),
                     Mux(uop.uses_scalar, ps, true.B))
      }
    } else if(matrix) {
      ret := Mux(uop.rt(RS1, isMatrix) && uop.m_scalar_busy, false.B,
             Mux(uop.rt(RS1, isTrTile), p1(uop.m_sidx),
             Mux(uop.fu_code === FU_GEMM, p1.andR,
             Mux(uop.rt(RS1, isAccTile), p1(uop.m_sidx), true.B))))
    } else {
      ret := p1(0)
    }
    ret
  }

  val rs2check: Bool = {
    val ret = Wire(Bool())
    val uop = slot_uop
    if (vector) {
      val vsew   = uop.vs2_eew
      val vLen_ecnt   = vLenb.U >> vsew
      when(vcompress) {
        ret := (0 until 8).toList.map(i => {
          val rsel   = VRegSel(uop.v_eidx +& vLen_ecnt*i.U, uop.vs1_eew, eLenSelSz)
          val pvs2   = uop.pvs2(rsel).bits
          !io.vbusy_status(pvs2)
        }).reduceLeft(_&_)
      }.otherwise {
        val v_eidx = Mux(uop.uopc === uopVIOTA, 0.U, uop.v_eidx)
        val eew    = Mux(uop.uses_v_ls_ew, uop.v_ls_ew, uop.vs2_eew)
        val rsel   = VRegSel(v_eidx, eew, eLenSelSz)
        val pvs2   = uop.pvs2(rsel).bits
        val reduce_busy = uop.pvs2.map(pvs2 => pvs2.valid && io.vbusy_status(pvs2.bits)).reduce(_ || _)
        ret       := !uop.rt(RS2, isVector) || Mux(uop.is_reduce, !reduce_busy, !io.vbusy_status(pvs2))
      }
    } else if(matrix) {
      ret := Mux(uop.rt(RS2, isMatrix), p2(uop.m_sidx),
             Mux(uop.rt(RS2, isInt),    ps, true.B))
    } else {
      ret := p2(0)
    }
    ret
  }

  val rs3check: Bool = {
    val ret = Wire(Bool())
    val uop = slot_uop
    if (vector) {
      val vsew        = uop.vs2_eew
      val vLen_ecnt   = vLenb.U >> vsew
      when(vcompress) {
        ret := (0 until 8).toList.map(i => {
          val rsel      = VRegSel(uop.v_eidx +& vLen_ecnt*i.U, uop.vs1_eew, eLenSelSz)
          val stale_pvd = uop.stale_pvd(rsel).bits
          !io.vbusy_status(stale_pvd)
        }).reduceLeft(_&_)
      }.otherwise {
          val v_eidx    = Mux(uop.rt(RS1, isMaskVD), 0.U, uop.v_eidx)
          val rsel      = VRegSel(v_eidx, uop.vd_eew, eLenSelSz)
          val stale_pvd = uop.stale_pvd(rsel).bits
          ret          := !uop.rt(RD, isVector) || !io.vbusy_status(stale_pvd)
      }
    } else if(matrix) {
      ret := Mux(!uop.rt(RD, isAccTile), true.B,
             Mux(uop.fu_code === FU_GEMM, p3.andR, p3(uop.m_sidx)))
    } else {
      ret := p3(0)
    }
    ret
  }

  val vmcheck: Bool = {
    val ret = Wire(Bool())
    val uop = slot_uop
    if (vector) {
      val vstart = uop.v_eidx
      val tail = vstart > uop.vconfig.vl
      // FIXME consider excluding prestart
      // val prestart = vstart < io.csr.v_eidx
      val pvm = uop.pvm
      ret := uop.v_unmasked || tail || !io.vbusy_status(pvm) // || !perm_ready
    } else {
      ret := true.B
    }
    ret
  }

  //-----------------------------------------------------------------------------
  // next slot state computation
  // compute the next state for THIS entry slot (in a collasping queue, the
  // current uop may get moved elsewhere, and a new uop can enter

  when (io.kill) {
    state := s_invalid
  } .elsewhen (io.in_uop.valid) {
    state := io.in_uop.bits.iw_state
  } .elsewhen (io.clear) {
    state := s_invalid
  } .otherwise {
    state := next_state
  }

  //-----------------------------------------------------------------------------
  // "update" state
  // compute the next state for the micro-op in this slot. This micro-op may
  // be moved elsewhere, so the "next_state" travels with it.

  // defaults
  next_state := state
  next_uopc := slot_uop.uopc
  next_lrs1_rtype := slot_uop.lrs1_rtype
  next_lrs2_rtype := slot_uop.lrs2_rtype

  when (io.kill) {
    next_state := s_invalid
  } .elsewhen ((io.grant && (state === s_valid_1)) ||
    (io.grant && (state === s_valid_2) && rs1check && rs2check && ppred && vl_ready)) {
    if (vector || matrix) {
      when (state === s_valid_1) {
        when(last_check) {
          next_state := s_invalid
        }
      }
    } else {
      // try to issue this uop.
      when (!(io.ldspec_miss && (p1_poisoned || p2_poisoned))) {
        next_state := s_invalid
      }
    }
  } .elsewhen (io.grant && (state === s_valid_2)) {
    when (!(io.ldspec_miss && (p1_poisoned || p2_poisoned))) {
      next_state := s_valid_1
      when (rs1check) {
        slot_uop.uopc := uopSTD
        next_uopc := uopSTD
        slot_uop.lrs1_rtype := RT_X
        next_lrs1_rtype := RT_X
      } .otherwise {
        slot_uop.lrs2_rtype := RT_X
        next_lrs2_rtype := RT_X
      }
    }
  }

  // Wakeup Compare Logic

  // these signals are the "next_p*" for the current slot's micro-op.
  // they are important for shifting the current slot_uop up to an other entry.
  val next_p1 = WireInit(p1)
  val next_p2 = WireInit(p2)
  val next_p3 = WireInit(p3)
  val in_p1 = if (vector || matrix) WireInit(p1) else null
  val in_p2 = if (vector || matrix) WireInit(p2) else null
  val in_p3 = if (vector || matrix) WireInit(p3) else null
  val next_ppred = WireInit(ppred)
  val wake_p1 = if (matrix) Wire(Vec(numWakeupPorts, UInt(vLenb.W))) else null
  val wake_p2 = if (matrix) Wire(Vec(numWakeupPorts, UInt(vLenb.W))) else null
  val wake_p3 = if (matrix) Wire(Vec(numWakeupPorts, UInt(vLenb.W))) else null

  when (io.in_uop.valid) {
    if (usingMatrix && matrix) {
      in_p1 := ~io.in_uop.bits.pts1_busy
      in_p2 := ~io.in_uop.bits.pts2_busy
      in_p3 := ~io.in_uop.bits.pts3_busy
      ps    := ~io.in_uop.bits.m_scalar_busy
      sidx  :=  io.in_uop.bits.m_sidx
    } else if (usingVector) {
      if(vector) {
        in_p1 := ~io.in_uop.bits.prs1_busy
        in_p2 := ~io.in_uop.bits.prs2_busy
        in_p3 := ~io.in_uop.bits.prs3_busy
        ps    := ~io.in_uop.bits.v_scalar_busy
        sdata := io.in_uop.bits.v_scalar_data
      } else {
        next_p1 := !io.in_uop.bits.prs1_busy(0)
        next_p2 := !io.in_uop.bits.prs2_busy(0)
        next_p3 := !io.in_uop.bits.prs3_busy(0)
      }
    } else {
      next_p1 := !io.in_uop.bits.prs1_busy
      next_p2 := !io.in_uop.bits.prs2_busy
      next_p3 := !io.in_uop.bits.prs3_busy
    }
    next_ppred := !(io.in_uop.bits.ppred_busy)
  }

  when (io.ldspec_miss && next_p1_poisoned) {
    assert(next_uop.prs1 =/= 0.U, "Poison bit can't be set for prs1=x0!")
    if (matrix) {
      next_p1 := 0.U
    } else {
      next_p1 := false.B
    }
  }
  when (io.ldspec_miss && next_p2_poisoned) {
    assert(next_uop.prs2 =/= 0.U, "Poison bit can't be set for prs2=x0!")
    if (matrix) {
      next_p2 := 0.U
    } else {
      next_p2 := false.B
    }
  }

  for (i <- 0 until numWakeupPorts) {
    val wk_valid = io.wakeup_ports(i).valid
    val wk_pdst  = io.wakeup_ports(i).bits.pdst
    if (matrix) {
      val wk_uop = io.wakeup_ports(i).bits.uop
      val wk_ts1 = wk_valid && (wk_pdst === next_uop.prs1) && (wk_uop.dst_rtype === next_uop.lrs1_rtype)
      val wk_ts2 = wk_valid && (wk_pdst === next_uop.prs2) && (wk_uop.dst_rtype === next_uop.lrs2_rtype)
      val wk_ts3 = wk_valid && (wk_pdst === next_uop.prs3) &&  wk_uop.rt(RD, isAccTile)
      wake_p1(i) := Mux(next_uop.rt(RS1, isAccTile) && wk_uop.m_is_split, wk_ts1 << wk_uop.m_sidx,
                    Mux(next_uop.rt(RS1, isTrTile)  && wk_uop.isHSlice === next_uop_ts1_hslice, wk_ts1 << wk_uop.m_sidx,
                    Mux(wk_uop.m_split_last, Fill(vLenb, wk_ts1), 0.U)))
      wake_p2(i) := Mux(next_uop.rt(RS2, isTrTile)  && wk_uop.isHSlice === next_uop_ts2_hslice, wk_ts2 << wk_uop.m_sidx,
                    Mux(wk_uop.m_split_last, Fill(vLenb, wk_ts2), 0.U))
      wake_p3(i) := Mux(next_uop.rt(RS1, isAccTile) && wk_uop.m_is_split, wk_ts3 << wk_uop.m_sidx,
                    Mux(wk_uop.m_split_last, Fill(vLenb, wk_ts3), 0.U))
    } else {
      when (wk_valid && (wk_pdst === next_uop.prs1)) {
        next_p1 := true.B
      }
      when (wk_valid && (wk_pdst === next_uop.prs2)) {
        next_p2 := true.B
      }
      when (wk_valid && (wk_pdst === next_uop.prs3)) {
        next_p3 := true.B
      }
    }
  }
  when (io.pred_wakeup_port.valid && io.pred_wakeup_port.bits === next_uop.ppred) {
    next_ppred := true.B
  }

  for (w <- 0 until memWidth) {
    assert (!(io.spec_ld_wakeup(w).valid && io.spec_ld_wakeup(w).bits === 0.U),
      "Loads to x0 should never speculatively wakeup other instructions")
  }

  // TODO disable if FP IQ.
  for (w <- 0 until memWidth) {
    when (io.spec_ld_wakeup(w).valid &&
      io.spec_ld_wakeup(w).bits === next_uop.prs1 &&
      next_uop.rt(RS1, isInt)) {
      if (matrix) {
        next_p1 := 0.U
      } else if(vector) {
        next_p1 := false.B
      } else {
        next_p1 := true.B
      }
      p1_poisoned := true.B
      assert (!is_valid || !next_p1_poisoned)
    }
    when (io.spec_ld_wakeup(w).valid &&
      io.spec_ld_wakeup(w).bits === next_uop.prs2 &&
      next_uop.rt(RS2, isInt)) {
      if (matrix) {
        next_p2 := 0.U
      } else if(vector) {
        next_p2 := false.B
      } else {
        next_p2 := true.B
      }
      p2_poisoned := true.B
      assert (!is_valid || !next_p2_poisoned)
    }
  }


  // Handle branch misspeculations
  val next_br_mask = GetNewBrMask(io.brupdate, slot_uop)

  // was this micro-op killed by a branch? if yes, we can't let it be valid if
  // we compact it into an other entry
  when (IsKilledByBranch(io.brupdate, slot_uop)) {
    next_state := s_invalid
  }

  when (!io.in_uop.valid) {
    slot_uop.br_mask := next_br_mask
  }

  if (matrix) {
    // when intupdate, we need to tell if data is really we need.
    when (io.intupdate.map(_.valid).reduce(_||_)) {
      val int_sel  = io.intupdate.map(u => u.valid && u.bits.uop.prs2 === next_uop.prs2 && next_uop.rt(RS2, isInt))
      val int_data = io.intupdate.map(u => u.bits.data)
      when(int_sel.asUInt.orR) {
        ps   := true.B
        sidx := Mux1H(int_sel, int_data)
      }
      assert(PopCount(int_sel) <= 1.U, "Multiple drivers to matrix intupdate")
    }

  }
  if (vector) {
    // when intupdate or fpupdate, we need to tell if data is really we need.
    when (io.intupdate.map(_.valid).reduce(_||_) || io.fpupdate.map(_.valid).reduce(_||_)) {
      val int_sel  = io.intupdate.map(u => u.valid && u.bits.uop.prs1 === next_uop.prs1 && next_uop.rt(RS1, isInt))
      val int_data = io.intupdate.map(u => Mux(u.bits.uop.rt(RS1, isIntU), Mux1H(UIntToOH(u.bits.uop.vconfig.vtype.vsew(1,0)),
                                                                                 Seq(u.bits.data(7,0),
                                                                                     u.bits.data(15,0),
                                                                                     u.bits.data(31,0),
                                                                                     u.bits.data)),
                                                                           Mux1H(UIntToOH(u.bits.uop.vconfig.vtype.vsew(1,0)),
                                                                                 Seq(u.bits.data(7,0).sextTo(eLen),
                                                                                     u.bits.data(15,0).sextTo(eLen),
                                                                                     u.bits.data(31,0).sextTo(eLen),
                                                                                     u.bits.data))))
      val fp_sel   = io.fpupdate.map(u => u.valid && u.bits.uop.prs1 === next_uop.prs1 && next_uop.rt(RS1, isFloat))
      val fp_data  = io.fpupdate.map(_.bits.data)
      when(int_sel.reduce(_||_) || fp_sel.reduce(_||_)) {
        ps    := true.B
        sdata := Mux1H(int_sel++fp_sel, int_data++fp_data)
      }
      assert(PopCount(int_sel++fp_sel) <= 1.U, "Multiple drivers")
    }

  }

  //-------------------------------------------------------------
  // Request Logic
  //io.request := is_valid && rs1check && rs2check && rs3check && vmcheck && ppred && !io.kill
  when (state === s_valid_1) {
    io.request := ppred && rs1check && rs2check && rs3check && vmcheck && vl_ready && !io.kill
  } .elsewhen (state === s_valid_2) {
    io.request := (rs1check || rs2check) && ppred && vl_ready && !io.kill
  } .otherwise {
    io.request := false.B
  }

  val high_priority = slot_uop.is_br || slot_uop.is_jal || slot_uop.is_jalr
  io.request_hp := io.request && high_priority

  //assign outputs
  io.valid := is_valid
  io.uop := slot_uop
  io.uop.iw_p1_poisoned := p1_poisoned
  io.uop.iw_p2_poisoned := p2_poisoned

  // micro-op will vacate due to grant.
  val may_vacate = io.grant && ((state === s_valid_1) || (state === s_valid_2)) &&
                   ppred && rs1check && rs2check && rs3check && vmcheck && vl_ready && last_check
  val squash_grant = io.ldspec_miss && (p1_poisoned || p2_poisoned)
  io.will_be_valid := is_valid && !(may_vacate && !squash_grant)

  io.out_uop            := slot_uop
  io.out_uop.iw_state   := next_state
  io.out_uop.uopc       := next_uopc
  io.out_uop.lrs1_rtype := next_lrs1_rtype
  io.out_uop.lrs2_rtype := next_lrs2_rtype
  io.out_uop.br_mask    := next_br_mask
  if(usingMatrix && matrix) {
    io.out_uop.pts1_busy := ~p1
    io.out_uop.pts2_busy := ~p2
    io.out_uop.pts3_busy := ~p3
    io.out_uop.m_sidx    := sidx
    io.out_uop.m_scalar_busy := ~ps
    io.uop.m_sidx        := sidx
    when(io.request && io.grant && next_uop_mma) {
      io.uop.m_split_first := slot_uop.m_sidx === 0.U
      io.uop.m_split_last  := slot_uop.m_sidx === slot_uop.m_tilek - 1.U
      io.uop.m_sidx        := slot_uop.m_sidx
      io.out_uop.m_sidx    := slot_uop.m_sidx + 1.U
      io.out_uop.prs3      := slot_uop.pdst
      slot_uop.m_sidx      := slot_uop.m_sidx + 1.U
      slot_uop.prs3        := slot_uop.pdst
    }
    next_p1 := Mux(io.in_uop.valid, in_p1, p1) | wake_p1.reduce(_|_)
    next_p2 := Mux(io.in_uop.valid, in_p2, p2) | wake_p2.reduce(_|_)
    next_p3 := Mux(io.in_uop.valid, in_p3, p3) | wake_p3.reduce(_|_)
  } else if (usingVector) {
    io.uop.v_eidx := slot_uop.v_eidx
    if (vector) {
      // value to next slot should be current latched version
      // ignore element busy masking, we keep busy status for entire v-register (i.e. p1,p2,p3)
      io.out_uop.prs1_busy  := ~p1 //& (slot_mask << Cat(slot_rsel, 0.U(3.W)))
      io.out_uop.prs2_busy  := ~p2 //& (slot_mask << Cat(slot_rsel, 0.U(3.W)))
      io.out_uop.prs3_busy  := ~p3 //& (slot_mask << Cat(slot_rsel, 0.U(3.W)))
      io.out_uop.v_scalar_busy := ~ps
      io.out_uop.v_scalar_data := sdata
      io.out_uop.v_perm_busy := false.B //!perm_ready || perm_ready_vrg_bsy
      io.out_uop.v_perm_wait := false.B //perm_wait || perm_wait_vrg_set
      io.out_uop.v_perm_idx  := 0.U //perm_idx
      // handle VOP_VI, prs1 records the value of lrpwdls
      // s1, and is used as simm5
      io.uop.v_scalar_data  := Mux(io.uop.rt(RS1, isRvvSImm5), Cat(Fill(eLen-5, io.uop.prs1(4).asUInt), io.uop.prs1(4,0)),
                               Mux(io.uop.rt(RS1, isRvvUImm5), Cat(Fill(eLen-5, 0.U(1.W)), slot_uop.prs1(4,0)), sdata))
      io.uop.v_perm_busy    := false.B //~perm_ready
      io.uop.v_perm_idx     := 0.U
      when (vcompress) {
        io.uop.pvm := slot_uop.prs1
      }
      when (io.request && io.grant) {
        val vd_idx = Mux(slot_uop.rt(RD, isMaskVD), 0.U, VRegSel(slot_uop.v_eidx, slot_uop.vd_eew, eLenSelSz))
        io.uop.pdst := Mux(slot_uop.rt(RD, isVector), slot_uop.pvd(vd_idx).bits, slot_uop.pdst)
        assert(is_invalid || !slot_uop.rt(RD, isVector) || slot_uop.pvd(vd_idx).valid)
        when (!io.uop.is_reduce) {
          val vsew = Mux(slot_uop.rt(RS2, isWidenV) || slot_uop.rt(RD, isMaskVD), slot_uop.vs2_eew, slot_uop.vd_eew)
          val vLen_ecnt   = vLenb.U >> vsew
          val next_offset = slot_uop.v_eidx +& vLen_ecnt
          io.uop.v_split_ecnt  := vLen_ecnt
          io.uop.v_split_first := slot_uop.v_eidx === 0.U
          io.uop.v_split_last  := next_offset >= slot_uop.v_split_ecnt
          //io.uop.pdst := Mux(slot_uop.rt(RD, isVector), slot_uop.pvd(vd_idx).bits, slot_uop.pdst)
          io.out_uop.v_eidx := next_offset
          slot_uop.v_eidx   := next_offset
        }
      }
    } else {
      io.out_uop.prs1_busy  := ~p1
      io.out_uop.prs2_busy  := ~p2
      io.out_uop.prs3_busy  := ~p3
    }
  } else {
    io.out_uop.prs1_busy  := !p1
    io.out_uop.prs2_busy  := !p2
    io.out_uop.prs3_busy  := !p3
  }
  io.out_uop.ppred_busy := !ppred
  io.out_uop.iw_p1_poisoned := p1_poisoned
  io.out_uop.iw_p2_poisoned := p2_poisoned

  when (io.in_uop.valid) {
    slot_uop := io.in_uop.bits
    assert (is_invalid || io.clear || io.kill, "trying to overwrite a valid issue slot.")
  }

  when(io.vl_wakeup.valid && !next_uop.vl_ready && (io.vl_wakeup.bits.vconfig_tag + 1.U) === next_uop.vconfig_tag) {
    slot_uop.vl_ready := true.B
    slot_uop.vconfig.vl := Mux(next_uop.uopc.isOneOf(uopVLM, uopVSMA), (io.vl_wakeup.bits.vl + 7.U) >> 3.U, io.vl_wakeup.bits.vl)
  }

  when (state === s_valid_2) {
    when (rs1check && rs2check && ppred && vl_ready)  {
      ; // send out the entire instruction as one uop
    } .elsewhen (rs1check && ppred && vl_ready) {
      io.uop.uopc := slot_uop.uopc
      io.uop.lrs2_rtype := RT_X
    } .elsewhen (rs2check && ppred && vl_ready) {
      io.uop.uopc := uopSTD
      io.uop.lrs1_rtype := RT_X
    }
  }

  p1 := next_p1
  p2 := next_p2
  p3 := next_p3
  ppred := next_ppred

  // debug outputs
  io.debug.p1 := p1
  io.debug.p2 := p2
  io.debug.p3 := p3
  io.debug.ppred := ppred
  io.debug.state := state
}
