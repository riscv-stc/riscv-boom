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

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.Parameters
import freechips.rocketchip.util.{UIntIsOneOf}

import boom.common._
import boom.common.MicroOpcodes._
import boom.util._
import FUConstants._

/**
 * IO bundle to interact with Issue slot
 *
 * @param numWakeupPorts number of wakeup ports for the slot
 */
class IssueSlotIO(val numWakeupPorts: Int, val vector: Boolean = false)
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

  val wakeup_ports  = Flipped(Vec(numWakeupPorts, Valid(new IqWakeup(maxPregSz, vector))))
  val pred_wakeup_port = Flipped(Valid(UInt(log2Ceil(ftqSz).W)))
  val spec_ld_wakeup= Flipped(Vec(memWidth, Valid(UInt(width=maxPregSz.W))))
  val in_uop        = Flipped(Valid(new MicroOp())) // if valid, this WILL overwrite an entry!
  val out_uop       = Output(new MicroOp()) // the updated slot uop; will be shifted upwards in a collasping queue.
  val uop           = Output(new MicroOp()) // the current Slot's uop. Sent down the pipeline when issued.
  val vmupdate      = if (usingVector && !vector) Input(Vec(vecWidth, Valid(new MicroOp))) else null
  val intupdate     = if (vector) Input(Vec(intWidth, Valid(new ExeUnitResp(eLen)))) else null
  val fpupdate      = if (vector) Input(Vec(fpWidth, Valid(new ExeUnitResp(eLen)))) else null
  val vecUpdate     = if (vector) Input(Vec(vecWidth, Valid(new ExeUnitResp(eLen)))) else null
  val cur_vs2_busy  = if (vector) Output(UInt(numVecPhysRegs.W)) else null
  val agg_vs2_busy  = if (vector) Input(UInt(numVecPhysRegs.W)) else null
  val cur_vs3_busy  = if (vector) Output(UInt(numVecPhysRegs.W)) else null
  val agg_vs3_busy  = if (vector) Input(UInt(numVecPhysRegs.W)) else null

  val debug = {
    val result = new Bundle {
      val p1 = if (vector) UInt(vLenb.W) else Bool()
      val p2 = if (vector) UInt(vLenb.W) else Bool()
      val p3 = if (vector) UInt(vLenb.W) else Bool()
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
  val vector: Boolean = false
)(implicit p: Parameters)
  extends BoomModule
  with IssueUnitConstants
{
  val io = IO(new IssueSlotIO(numWakeupPorts, vector))

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
  val p1    = if(vector) RegInit(0.U(vLenb.W)) else RegInit(false.B)
  val p2    = if(vector) RegInit(0.U(vLenb.W)) else RegInit(false.B)
  val p3    = if(vector) RegInit(0.U(vLenb.W)) else RegInit(false.B)
  val pm    = if(vector) RegInit(0.U(vLenb.W)) else if (usingVector && iqType == IQT_MEM.litValue) RegInit(false.B) else null
  val ps    = if(vector) RegInit(false.B) else null
  val sdata = if(vector) RegInit(0.U(eLen.W)) else null
  val vxofs = if(usingVector && iqType == IQT_MEM.litValue) RegInit(0.U(eLen.W)) else null
  val perm_ready = if (vector) RegInit(false.B) else null
  val perm_wait  = if (vector) RegInit(false.B) else null
  val perm_idx   = if (vector) Reg(UInt(eLen.W)) else null
  val ppred = RegInit(false.B)

  // Poison if woken up by speculative load.
  // Poison lasts 1 cycle (as ldMiss will come on the next cycle).
  // SO if poisoned is true, set it to false!
  val p1_poisoned = RegInit(false.B)
  val p2_poisoned = RegInit(false.B)
  p1_poisoned := false.B
  p2_poisoned := false.B
  val next_p1_poisoned = Mux(io.in_uop.valid, io.in_uop.bits.iw_p1_poisoned, p1_poisoned)
  val next_p2_poisoned = Mux(io.in_uop.valid, io.in_uop.bits.iw_p2_poisoned, p2_poisoned)

  val slot_uop = RegInit(NullMicroOp(usingVector))
  val next_uop = WireInit(Mux(io.in_uop.valid, io.in_uop.bits, slot_uop))
  val has_vecupdate = slot_uop.rt(RS1, isVector) && slot_uop.uopc === uopVRGATHER || slot_uop.uopc.isOneOf(uopVRGATHEREI16, uopVCOMPRESS)

  def last_check: Bool = {
    val ret = Wire(Bool())
    if (vector) {
      val need_vecupdate = io.uop.rt(RS1, isVector) && io.uop.uopc === uopVRGATHER || io.uop.uopc === uopVRGATHEREI16
      ret := io.uop.v_is_last && (!need_vecupdate || perm_ready) ||
             io.uop.uopc.isOneOf(uopVL, uopVSA, uopVLS, uopVSSA, uopVLUX, uopVSUXA, uopVLOX, uopVSOXA)
    } else {
      ret := true.B
    }
    ret
  }

  def rs1check(ecnt: UInt = 1.U): Bool = {
    val ret = Wire(Bool())
    val uop = slot_uop
    if (vector) {
      val ret_common = Wire(Bool())
      val vstart  = Mux(uop.rt(RS1, isReduceV), 0.U, uop.vstart + uop.voffset)
      val tail    = vstart > uop.vconfig.vl
      val eew     = Mux(uop.uses_v_ls_ew, uop.v_ls_ew,
                    Mux(uop.uopc === uopVRGATHEREI16, 1.U, uop.vconfig.vtype.vsew))
      val (rsel, rmsk) = VRegSel(vstart, eew, ecnt, vLenb, eLenSelSz) // TODO, consider eLen_ecnt
      val rsh     = Cat(rsel, 0.U(3.W))
      val mask    = ((p1 >> rsh) & rmsk)
      ret_common  := Mux(uop.rt(RS1, isVector), tail || mask === rmsk,
                     Mux(uop.uses_scalar, ps, true.B))
      when (uop.uopc.isOneOf(uopVRGATHER, uopVRGATHEREI16)) {
        ret := Mux(perm_ready, true.B, !perm_wait && ret_common) // requires feedback from vecupdate
      } .elsewhen (uop.uopc === uopVCOMPRESS) {
        ret := Mux(perm_ready, true.B, !perm_wait && p1(perm_idx(vLenSz-1,0) >> 3.U))
      } .otherwise {
        ret := ret_common
      }
    } else {
      ret := p1(0)
    }
    ret
  }

  def rs2check(ecnt: UInt = 1.U): Bool = {
    val ret = Wire(Bool())
    val uop = slot_uop
    val vstart = uop.vstart + uop.voffset
    if (vector) {
      when (uop.uopc.isOneOf(uopVSLIDEDOWN, uopVSLIDE1DOWN, uopVRGATHER, uopVRGATHEREI16)) {
        // check vreg-wise, permutation ops have identical sew on vs2
        assert(!uop.rt(RS2, isNarrowV) && !uop.rt(RS2, isWidenV), "unexpected VS2 modifier")
        val vs2_sew  = uop.vconfig.vtype.vsew
        val vs2_eidx = Mux(uop.uopc === uopVSLIDEDOWN,  vstart + Mux(uop.rt(RS1, isInt), sdata, uop.prs1(4,0)),
                       Mux(uop.uopc === uopVSLIDE1DOWN, vstart + 1.U,
                       Mux(uop.uopc === uopVRGATHER && uop.rt(RS1, isInt), sdata,
                       Mux(uop.uopc === uopVRGATHER && uop.rt(RS1, isRvvUImm5), Cat(Fill(eLen-5, 0.U(1.W)), uop.prs1(4,0)),
                       perm_idx))))
        val vd_emul  = uop.vd_emul // vs2 and vd have identical emul/eew
        val prs2     = uop.prs2
        val vs2_rinc = vs2_eidx >> (vLenSz.U - 3.U - vs2_sew)
        val vs2_ridx = Mux(vd_emul === 1.U, Cat(prs2(prs2.getWidth-1, 1), vs2_rinc(0)),
                       Mux(vd_emul === 2.U, Cat(prs2(prs2.getWidth-1, 2), vs2_rinc(1,0)),
                       Mux(vd_emul === 3.U, Cat(prs2(prs2.getWidth-1, 3), vs2_rinc(2,0)), prs2)))
        val vs2_ready= !io.agg_vs2_busy(vs2_ridx) // check any busy on corresponding vreg
        // fire and get vs1[i] ASAP
        val need_vecupdate = uop.rt(RS1, isVector) && uop.uopc === uopVRGATHER || uop.uopc === uopVRGATHEREI16
        ret := need_vecupdate && !perm_ready || vs2_ready
      } .otherwise {
        val tail    = vstart > uop.vconfig.vl
        val vsew    = uop.vconfig.vtype.vsew
        val vs2_sew = Mux(uop.uopc === uopVEXT8, vsew - 3.U,
                      Mux(uop.uopc === uopVEXT4, vsew - 2.U,
                      Mux(uop.rt(RS2, isNarrowV), vsew - 1.U,
                      Mux(uop.rt(RS2, isWidenV),  vsew + 1.U, vsew))))
        val eew     = Mux(uop.uses_v_ls_ew, uop.v_ls_ew, vs2_sew)
        val (rsel, rmsk) = VRegSel(vstart, eew, ecnt, vLenb, eLenSelSz) // TODO, consider eLen_ecnt
        val rsh     = Cat(rsel, 0.U(3.W))
        val mask    = ((p2 >> rsh) & rmsk)
        ret         := Mux(uop.uopc === uopVCOMPRESS && !perm_ready, true.B, tail || mask === rmsk)
      }
    } else {
      ret := p2(0)
    }
    ret
  }

  def rs3check(ecnt: UInt = 1.U): Bool = {
    val ret = Wire(Bool())
    val uop = slot_uop
    val vstart = uop.vstart + uop.voffset
    if (vector) {
      when (uop.uopc.isOneOf(uopVSLIDEUP, uopVSLIDE1UP, uopVCOMPRESS)) {
        assert(!uop.rt(RD, isNarrowV) && !uop.rt(RD, isWidenV), "unexpected VD modifier")
        val vs3_sew  = uop.vconfig.vtype.vsew
        val vs3_eidx = Mux(uop.uopc === uopVSLIDEUP,  vstart + Mux(uop.rt(RS1, isInt), sdata, uop.prs1(4,0)),
                       Mux(uop.uopc === uopVSLIDE1UP, vstart + 1.U,
                                                      perm_idx))
        val vd_emul  = uop.vd_emul
        val prs3     = uop.prs3
        val vs3_rinc = vs3_eidx >> (vLenSz.U - 3.U - vs3_sew)
        val vs3_ridx = Mux(vd_emul === 1.U, Cat(prs3(prs3.getWidth-1, 1), vs3_rinc(0)),
                       Mux(vd_emul === 2.U, Cat(prs3(prs3.getWidth-1, 2), vs3_rinc(1,0)),
                       Mux(vd_emul === 3.U, Cat(prs3(prs3.getWidth-1, 3), vs3_rinc(2,0)), prs3)))
        val vs3_ready= !io.agg_vs3_busy(vs3_ridx) // check any busy on corresponding vreg
        val need_vecupdate = uop.uopc === uopVCOMPRESS
        ret := need_vecupdate && !perm_ready || vs3_ready
      } .otherwise {
        val (rsel, rmsk) = VRegSel(vstart, uop.vd_eew, ecnt, vLenb, eLenSelSz)
        val rsh     = Cat(rsel, 0.U(3.W))
        val mask    = ((p3 >> rsh) & rmsk)
        ret         := mask === rmsk
      }
    } else {
      ret := p3(0)
    }
    ret
  }

  def vmcheck(ecnt: UInt = 1.U): Bool = {
    val ret = Wire(Bool())
    val uop = slot_uop
    if (vector) {
      val vstart = uop.vstart + uop.voffset
      val tail = vstart > uop.vconfig.vl
      // FIXME consider excluding prestart
      // val prestart = vstart < io.csr_vconfig.vstart
      ret := !perm_ready || uop.v_unmasked || tail || pm(vstart >> 3.U)
    } else {
      if (usingVector && iqType == IQT_MEM.litValue) {
        ret := !uop.is_rvv || (uop.v_unmasked && !uop.v_idx_ls) || pm(0)
      } else {
        ret := true.B
      }
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
    (io.grant && (state === s_valid_2) && rs1check() && rs2check() && ppred)) {
    if (vector) {
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
      when (rs1check()) {
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
  val next_pm = if (vector || usingVector && iqType == IQT_MEM.litValue) WireInit(pm) else null
  val next_perm_ready = if (vector) WireInit(perm_ready) else null
  val next_perm_wait  = if (vector) WireInit(perm_wait)  else null
  val next_perm_idx   = if (vector) WireInit(perm_idx)   else null
  val in_p1 = if (vector) WireInit(p1) else null
  val in_p2 = if (vector) WireInit(p2) else null
  val in_p3 = if (vector) WireInit(p3) else null
  val in_pm = if (vector || usingVector && iqType == IQT_MEM.litValue) WireInit(pm) else null
  val in_perm_ready = if (vector) WireInit(false.B)  else null
  val in_perm_wait  = if (vector) WireInit(false.B)  else null
  val in_perm_idx   = if (vector) WireInit(0.U(eLen.W)) else null
  val wake_p1 = if (vector) Wire(Vec(numWakeupPorts, UInt(vLenb.W))) else null
  val wake_p2 = if (vector) Wire(Vec(numWakeupPorts, UInt(vLenb.W))) else null
  val wake_p3 = if (vector) Wire(Vec(numWakeupPorts, UInt(vLenb.W))) else null
  val wake_pm = if (vector) Wire(Vec(numWakeupPorts, UInt(vLenb.W))) else null
  val next_ppred = WireInit(ppred)
  val vupd_perm_hit = if (vector) WireInit(false.B) else null
  val vupd_perm_idx = if (vector) WireInit(0.U(eLen.W)) else null

  when (io.in_uop.valid) {
    if (usingVector) {
      if (vector) {
        val in_reduce = io.in_uop.bits.rt(RD, isReduceV)
        val in_first  = io.in_uop.bits.v_is_first
        in_p1 := ~io.in_uop.bits.prs1_busy & Fill(vLenb, Mux(in_reduce, in_first, true.B).asUInt)
        in_p2 := ~io.in_uop.bits.prs2_busy
        in_p3 := ~io.in_uop.bits.prs3_busy
        in_pm := ~io.in_uop.bits.prvm_busy
        ps    := ~io.in_uop.bits.v_scalar_busy
        sdata := io.in_uop.bits.v_scalar_data
        in_perm_ready := ~io.in_uop.bits.v_perm_busy
        in_perm_wait  := io.in_uop.bits.v_perm_wait
        in_perm_idx := io.in_uop.bits.v_perm_idx
      } else {
        next_p1 := !io.in_uop.bits.prs1_busy(0)
        next_p2 := !io.in_uop.bits.prs2_busy(0)
        next_p3 := !io.in_uop.bits.prs3_busy(0)
        if (iqType == IQT_MEM.litValue) {
          in_pm := !io.in_uop.bits.prvm_busy(0)
          vxofs := io.in_uop.bits.v_xls_offset
        }
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
    if (vector) {
      next_p1 := 0.U
    } else {
      next_p1 := false.B
    }
  }
  when (io.ldspec_miss && next_p2_poisoned) {
    assert(next_uop.prs2 =/= 0.U, "Poison bit can't be set for prs2=x0!")
    if (vector) {
      next_p2 := 0.U
    } else {
      next_p2 := false.B
    }
  }

  for (i <- 0 until numWakeupPorts) {
    val wk_valid = io.wakeup_ports(i).valid
    val wk_pdst = io.wakeup_ports(i).bits.pdst
    if (vector) {
      val wk_uop = io.wakeup_ports(i).bits.uop
      val wk_vstart = wk_uop.vstart
      val wk_ecnt   = wk_uop.v_split_ecnt
      val (wk_sel, wk_mask) = VRegSel(wk_vstart, wk_uop.vd_eew, wk_ecnt, eLenb, eLenSelSz)
      val wk_rs1 = wk_valid && (wk_pdst === next_uop.prs1)
      val wk_rs2 = wk_valid && (wk_pdst === next_uop.prs2)
      val wk_rs3 = wk_valid && (wk_pdst === next_uop.prs3) && next_uop.frs3_en
      val wk_rvm = wk_valid && (wk_pdst === next_uop.prvm) && !next_uop.v_unmasked
      wake_p1(i) := (wk_mask << Cat(wk_sel, 0.U(3.W))) & Fill(vLenb, wk_rs1.asUInt)
      wake_p2(i) := (wk_mask << Cat(wk_sel, 0.U(3.W))) & Fill(vLenb, wk_rs2.asUInt)
      wake_p3(i) := (wk_mask << Cat(wk_sel, 0.U(3.W))) & Fill(vLenb, wk_rs3.asUInt)
      wake_pm(i) := (wk_mask << Cat(wk_sel, 0.U(3.W))) & Fill(vLenb, wk_rvm.asUInt)
      when (wk_uop.rt(RS1, isReduceV)) {
        // make sure splits across issue slots will be woken in order
        // consider uops that depends on the reduction result, they must not be incorrectly woken
        val wk_act  = wk_uop.rt(RD, isReduceV)
        val i_am_red= next_uop.rt(RD, isReduceV)
        val wk_rd   = wk_valid && (wk_pdst === next_uop.pdst)
        val wk_next = (wk_vstart + 1.U) === (next_uop.vstart + next_uop.voffset)
        val wk_last = (wk_vstart + 1.U) === wk_uop.vconfig.vtype.vlMax
        wake_p1(i) := Fill(vLenb, Mux(i_am_red, wk_rd && wk_act && wk_next, wk_rs1 && wk_last).asUInt)
        wake_p2(i) := Fill(vLenb, (wk_rs2 && wk_act && wk_last).asUInt)
        wake_p3(i) := Fill(vLenb, (wk_rs3 && wk_act && wk_last).asUInt)
        wake_pm(i) := Fill(vLenb, (wk_rvm && wk_act && wk_last).asUInt)
      } .elsewhen(wk_uop.rt(RD, isMaskVD)) {
        val rsel_mvd     = wk_vstart(eLenSelSz+5, 6)
        val is_last_mvd  = (wk_vstart + 1.U) === wk_uop.vconfig.vl
        val is_eLast_mvd = wk_vstart(5, 0) === "h_3f".U
        val emask_mvd    = Fill(eLenb, is_eLast_mvd)
        val mask_mvd     = Fill(vLenb, is_last_mvd) | (emask_mvd << Cat(rsel_mvd, 0.U(3.W)))
        wake_p1(i) := mask_mvd & Fill(vLenb, wk_rs1.asUInt)
        wake_p2(i) := mask_mvd & Fill(vLenb, wk_rs2.asUInt)
        wake_p3(i) := mask_mvd & Fill(vLenb, wk_rs3.asUInt)
        wake_pm(i) := mask_mvd & Fill(vLenb, wk_rvm.asUInt)
      }
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
      if (vector) {
        next_p1 := 0.U
      } else {
        next_p1 := true.B
      }
      p1_poisoned := true.B
      assert (!is_valid || !next_p1_poisoned)
    }
    when (io.spec_ld_wakeup(w).valid &&
      io.spec_ld_wakeup(w).bits === next_uop.prs2 &&
      next_uop.rt(RS2, isInt)) {
      if (vector) {
        next_p2 := 0.U
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

  if (usingVector && !vector && iqType == IQT_MEM.litValue) {
    next_pm := io.vmupdate.map(x => x.valid && x.bits.rob_idx === next_uop.rob_idx).reduce(_||_)
    when (next_uop.v_idx_ls && io.vmupdate.map(x => x.valid && x.bits.rob_idx === next_uop.rob_idx && (io.in_uop.valid || is_valid)).reduce(_||_)) {
      vxofs := Mux1H(io.vmupdate.map(x => (x.valid && x.bits.rob_idx === next_uop.rob_idx && (io.in_uop.valid || is_valid), x.bits.v_xls_offset)))
    }
    when (IsKilledByVM(io.vmupdate, slot_uop)) {
      next_state := s_invalid
    }
  }

  when (!io.in_uop.valid) {
    slot_uop.br_mask := next_br_mask
  }

  if (vector) {
    when (io.intupdate.map(_.valid).reduce(_||_) || io.fpupdate.map(_.valid).reduce(_||_)) {
      val int_sel  = io.intupdate.map(u => u.valid && u.bits.uop.prs1 === next_uop.prs1 && next_uop.rt(RS1, isInt))
      val int_data = io.intupdate.map(_.bits.data)
      val fp_sel   = io.fpupdate.map(u => u.valid && u.bits.uop.prs1 === next_uop.prs1 && next_uop.rt(RS1, isFloat))
      val fp_data  = io.fpupdate.map(_.bits.data)
      ps := int_sel.reduce(_||_) || fp_sel.reduce(_||_)
      sdata := Mux1H(int_sel++fp_sel, int_data++fp_data)
      assert(PopCount(int_sel++fp_sel) <= 1.U, "Multiple drivers")
    }

    val vm_sel = io.vecUpdate.map(v => v.valid && v.bits.uop.match_group(next_uop.pdst))
    when (is_valid && io.vecUpdate.map(_.valid).reduce(_||_)) {
      val vm_rs1 = io.vecUpdate.map(v => v.bits.data)
      val vm_actv= io.vecUpdate.map(v => v.valid && v.bits.uop.v_active).reduce(_||_)
      vupd_perm_hit := vm_sel.reduce(_||_)
      vupd_perm_idx := Mux(next_uop.uopc === uopVCOMPRESS, perm_idx+vm_actv, Mux1H(vm_sel, vm_rs1))
      assert(io.vecUpdate.map(v => v.valid && v.bits.uop.uopc.isOneOf(uopVRGATHER, uopVRGATHEREI16, uopVCOMPRESS)).reduce(_||_))
      when (vupd_perm_hit) {
        assert(!perm_ready && has_vecupdate)
      }
    }
  }

  //-------------------------------------------------------------
  // Request Logic
  //io.request := is_valid && rs1check() && rs2check() && rs3check() && vmcheck() && ppred && !io.kill
  when (state === s_valid_1) {
    io.request := ppred && rs1check() && rs2check() && rs3check() && vmcheck() && !io.kill
  } .elsewhen (state === s_valid_2) {
    io.request := (rs1check() || rs2check()) && ppred && !io.kill
  } .otherwise {
    io.request := false.B
  }

  val high_priority = slot_uop.is_br || slot_uop.is_jal || slot_uop.is_jalr
  io.request_hp := io.request && high_priority

  //assign outputs
  io.valid := is_valid
  io.uop := slot_uop
  io.uop.vstart := slot_uop.vstart + slot_uop.voffset
  io.uop.iw_p1_poisoned := p1_poisoned
  io.uop.iw_p2_poisoned := p2_poisoned

  // micro-op will vacate due to grant.
  val may_vacate = io.grant && ((state === s_valid_1) || (state === s_valid_2)) &&
                   ppred && rs1check() && rs2check() && rs3check() && vmcheck() && last_check
  val squash_grant = io.ldspec_miss && (p1_poisoned || p2_poisoned)
  io.will_be_valid := is_valid && !(may_vacate && !squash_grant)

  io.out_uop            := slot_uop
  io.out_uop.iw_state   := next_state
  io.out_uop.uopc       := next_uopc
  io.out_uop.lrs1_rtype := next_lrs1_rtype
  io.out_uop.lrs2_rtype := next_lrs2_rtype
  io.out_uop.br_mask    := next_br_mask
  if (usingVector) {
    if (vector) {
      // value to next slot should be current latched version
      // ignore element busy masking, we keep busy status for entire v-register (i.e. p1,p2,p3,pm)
      io.out_uop.prs1_busy  := ~p1 //& (slot_mask << Cat(slot_rsel, 0.U(3.W)))
      io.out_uop.prs2_busy  := ~p2 //& (slot_mask << Cat(slot_rsel, 0.U(3.W)))
      io.out_uop.prs3_busy  := ~p3 //& (slot_mask << Cat(slot_rsel, 0.U(3.W)))
      io.out_uop.prvm_busy  := ~pm
      io.out_uop.v_scalar_busy := ~ps
      io.out_uop.v_scalar_data := sdata
      io.out_uop.v_perm_busy := ~next_perm_ready
      io.out_uop.v_perm_idx  := next_perm_idx
      io.out_uop.v_perm_wait := next_perm_wait
      io.cur_vs2_busy       := UIntToOH(slot_uop.prs2)(vpregSz-1,0) & Fill(vpregSz, !(p2.andR) && is_valid)
      io.cur_vs3_busy       := UIntToOH(slot_uop.prs3)(vpregSz-1,0) & Fill(vpregSz, !(p3.andR) && is_valid)
      // handle VOP_VI, prs1 records the value of lrs1, and is used as simm5
      io.uop.v_scalar_data  := Mux(io.uop.rt(RS1, isRvvSImm5), Cat(Fill(eLen-5, io.uop.prs1(4).asUInt), io.uop.prs1(4,0)),
                               Mux(io.uop.rt(RS1, isRvvUImm5), Cat(Fill(eLen-5, 0.U(1.W)), slot_uop.prs1(4,0)), sdata))
      io.uop.v_perm_busy    := ~perm_ready
      io.uop.v_perm_idx     := Mux(slot_uop.uopc.isOneOf(uopVSLIDEUP, uopVSLIDEDOWN),   io.uop.vstart + Mux(slot_uop.rt(RS1, isInt), sdata, slot_uop.prs1(4,0)),
                               Mux(slot_uop.uopc.isOneOf(uopVSLIDE1UP, uopVSLIDE1DOWN), io.uop.vstart + 1.U,
                               Mux(slot_uop.uopc === uopVRGATHER && slot_uop.rt(RS1, isInt), sdata,
                               Mux(slot_uop.uopc === uopVRGATHER && slot_uop.rt(RS1, isRvvUImm5), Cat(Fill(eLen-5, 0.U(1.W)), slot_uop.prs1(4,0)),
                               perm_idx))))
      when (slot_uop.uopc === uopVCOMPRESS) {
        io.uop.prvm := slot_uop.prs1
      }
      val red_iss_p1 = WireInit(p1)
      when (io.request && io.grant && !io.uop.uopc.isOneOf(uopVL, uopVSA, uopVLS, uopVSSA, uopVLUX, uopVSUXA, uopVLOX, uopVSOXA)) {
        val vsew = slot_uop.vconfig.vtype.vsew(1,0)
        //val eLen_ecnt = eLen.U >> (vsew+3.U)
        val ren_mask = ~(Fill(vLenSz,1.U) << (7.U-vsew))
        io.uop.v_split_ecnt := 1.U //eLen_ecnt, TODO consider masking
        io.uop.v_is_first := (slot_uop.voffset & ren_mask(vLenSz,0)) === 0.U
        io.uop.v_is_last  := slot_uop.voffset + io.uop.v_split_ecnt === slot_uop.v_split_ecnt
        val next_voffset   = slot_uop.voffset + io.uop.v_split_ecnt
        io.out_uop.voffset:= next_voffset
        slot_uop.voffset  := next_voffset
        when (slot_uop.rt(RD, isReduceV)) {
          // prs1 uses true rs1 only for the first split of entire reduction op
          io.uop.prs1 := Mux(io.uop.vstart === 0.U, slot_uop.prs1, slot_uop.pdst)
          // clear ready on p1
          red_iss_p1 := 0.U
          io.out_uop.prs1_busy := ~(0.U(vLenb.W))
          slot_uop.prs1_busy := ~(0.U(vLenb.W))
        }
        when (has_vecupdate) {
          when (!perm_ready) {
            io.out_uop.voffset:= slot_uop.voffset
            slot_uop.voffset  := slot_uop.voffset
          }
        }
      }
      // merge input busy status and wake-up status
      next_p1 := Mux(io.in_uop.valid, in_p1, red_iss_p1) | wake_p1.reduce(_|_)
      next_p2 := Mux(io.in_uop.valid, in_p2, p2) | wake_p2.reduce(_|_)
      next_p3 := Mux(io.in_uop.valid, in_p3, p3) | wake_p3.reduce(_|_)
      next_pm := Mux(io.in_uop.valid, in_pm, pm) | wake_pm.reduce(_|_)
    } else {
      io.out_uop.prs1_busy  := Cat(0.U, !p1)
      io.out_uop.prs2_busy  := Cat(0.U, !p2)
      io.out_uop.prs3_busy  := Cat(0.U, !p3)
      if (iqType == IQT_MEM.litValue) {
        io.out_uop.prvm_busy := Cat(0.U, !pm)
        io.out_uop.v_xls_offset := vxofs
        io.uop.v_xls_offset := vxofs
      } else {
        io.out_uop.prvm_busy := 0.U
      }
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

  when (state === s_valid_2) {
    when (rs1check() && rs2check() && ppred) {
      ; // send out the entire instruction as one uop
    } .elsewhen (rs1check() && ppred) {
      io.uop.uopc := slot_uop.uopc
      io.uop.lrs2_rtype := RT_X
    } .elsewhen (rs2check() && ppred) {
      io.uop.uopc := uopSTD
      io.uop.lrs1_rtype := RT_X
    }
  }

  p1 := next_p1
  p2 := next_p2
  p3 := next_p3
  ppred := next_ppred
  if (vector) {
    pm := next_pm
    next_perm_ready := Mux(io.in_uop.valid, in_perm_ready, //||vupd_perm_hit,
                                            Mux(perm_ready, Mux(has_vecupdate,!io_fire,true.B), vupd_perm_hit))
    next_perm_wait  := Mux(io.in_uop.valid, in_perm_wait, //&& !vupd_perm_hit,
                                            Mux(perm_wait, !vupd_perm_hit, io_fire&&has_vecupdate&& !perm_ready))
    next_perm_idx   := Mux(io.in_uop.valid, in_perm_idx,
                                            Mux(vupd_perm_hit, vupd_perm_idx, perm_idx))
    perm_ready := next_perm_ready
    perm_wait  := next_perm_wait
    perm_idx   := next_perm_idx
  } else if (usingVector && iqType == IQT_MEM.litValue) {
    pm := Mux(io.in_uop.valid, in_pm, pm) | next_pm
  }

  // debug outputs
  io.debug.p1 := p1
  io.debug.p2 := p2
  io.debug.p3 := p3
  io.debug.ppred := ppred
  io.debug.state := state
}
