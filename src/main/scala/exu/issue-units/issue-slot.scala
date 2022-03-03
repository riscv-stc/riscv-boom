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
  val vl_wakeup_port = Flipped(Valid(new VlWakeupResp()))
  val spec_ld_wakeup= Flipped(Vec(memWidth, Valid(UInt(width=maxPregSz.W))))
  val in_uop        = Flipped(Valid(new MicroOp())) // if valid, this WILL overwrite an entry!
  val out_uop       = Output(new MicroOp()) // the updated slot uop; will be shifted upwards in a collasping queue.
  val uop           = Output(new MicroOp()) // the current Slot's uop. Sent down the pipeline when issued.
  //val vmupdate      = if (usingVector && !vector) Input(Vec(1, Valid(new MicroOp))) else null
  val intupdate     = if (vector) Input(Vec(intWidth + memWidth, Valid(new ExeUnitResp(eLen)))) else null
  val fpupdate      = if (vector) Input(Vec(fpWidth, Valid(new ExeUnitResp(eLen)))) else null
  //val vecUpdate     = if (vector) Input(Vec(vecWidth, Valid(new ExeUnitResp(eLen)))) else null
  val vbusy_status  = if (vector) Input(UInt(numVecPhysRegs.W)) else null

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
  //val pm    = if(vector) RegInit(0.U(vLenb.W)) else if (usingVector && iqType == IQT_MEM.litValue) RegInit(false.B) else null
  val ps    = if(vector) RegInit(false.B) else null
  val sdata = if(vector) RegInit(0.U(eLen.W)) else null
  val strideLength = if(iqType == IQT_VMX.litValue) RegInit(0.U(eLen.W)) else null
  val vxofs = if(usingVector && iqType == IQT_MEM.litValue) RegInit(0.U(eLen.W)) else null
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

  val last_check: Bool = {
    val ret = Wire(Bool())
    if (vector) {
      ret := io.uop.v_split_last || io.uop.is_reduce ||
             io.uop.uopc.isOneOf(uopVL, uopVLFF, uopVLS, uopVLUX, uopVLOX, uopVSA, uopVSSA, uopVSUXA, uopVSOXA)
    } else {
      ret := true.B
    }
    ret
  }

  val rs1check: Bool = {
    val ret = Wire(Bool())
    val uop = slot_uop
    if (vector) {
      val v_eidx = Mux(uop.is_reduce, 0.U, uop.v_eidx)
      val rsel   = VRegSel(v_eidx, uop.vs1_eew, eLenSelSz)
      val pvs1   = uop.pvs1(rsel).bits
      ret       := Mux(uop.rt(RS1, isVector), !io.vbusy_status(pvs1),
                   Mux(uop.uses_scalar, ps, true.B))
    } else {
      ret := p1(0)
    }
    ret
  }

  val rs2check: Bool = {
    val ret = Wire(Bool())
    val uop = slot_uop
    if (vector) {
      val v_eidx = Mux(uop.uopc === uopVIOTA, 0.U, uop.v_eidx)
      val eew    = Mux(uop.uses_v_ls_ew, uop.v_ls_ew, uop.vs2_eew)
      val isIndexed = uop.v_idx_ls
      /** Indexed is split according to vs2_emul. */
      val fieldIdx = uop.v_seg_f
      val indexPick: UInt = fieldIdx
      val rsel   = Mux(isIndexed, indexPick, VRegSel(v_eidx, eew, eLenSelSz))
      val pvs2   = uop.pvs2(rsel).bits
      val reduce_busy = uop.pvs2.map(pvs2 => pvs2.valid && io.vbusy_status(pvs2.bits)).reduce(_ || _)
      ret       := !uop.rt(RS2, isVector) || Mux(uop.is_reduce, !reduce_busy, !io.vbusy_status(pvs2))
    } else {
      ret := p2(0)
    }
    ret
  }

  val rs3check: Bool = {
    val ret = Wire(Bool())
    val uop = slot_uop
    if (vector) {
      val v_eidx    = Mux(uop.rt(RS1, isMaskVD), 0.U, uop.v_eidx)
      val rsel      = VRegSel(v_eidx, uop.vd_eew, eLenSelSz)
      val stale_pvd = uop.stale_pvd(rsel).bits
      ret          := !uop.rt(RD, isVector) || !io.vbusy_status(stale_pvd)
    } else {
      ret := p3(0)
    }
    ret
  }

  val vmcheck: Bool = {
    val ret = Wire(Bool())
    val uop = slot_uop
    if (vector) {
      //val vstart = uop.v_eidx
      //val tail = vstart > uop.vconfig.vl
      // FIXME consider excluding prestart
      // val prestart = vstart < io.csr.v_eidx
      val pvm = uop.pvm
      //ret := uop.v_unmasked || tail || !io.vbusy_status(pvm) // || !perm_ready || pm(vstart >> 3.U)
      ret := uop.v_unmasked || !io.vbusy_status(pvm) // || !perm_ready || pm(vstart >> 3.U)
    } else {
      //if (usingVector && iqType == IQT_MEM.litValue) {
      // ret := !uop.is_rvv || (uop.v_unmasked && !uop.v_idx_ls) // || pm(0)
      //} else {
      ret := true.B
      //}
    }
    ret
  }
  /** true means that this uop can fire to rrd. only for vls that reads both scalar and vector. */
  val scalarCheck: Bool = {
    val ret = Wire(Bool())
    if (usingVector && iqType == IQT_VMX.litValue){
      val needWaitScalar = slot_uop.is_rvv && slot_uop.v_scalar_busy
      ret := Mux(needWaitScalar, ps, true.B)
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
  //val next_pm = if (vector || usingVector && iqType == IQT_MEM.litValue) WireInit(pm) else null
  val in_p1 = if (vector) WireInit(p1) else null
  val in_p2 = if (vector) WireInit(p2) else null
  val in_p3 = if (vector) WireInit(p3) else null
  val next_ppred = WireInit(ppred)

  when (io.in_uop.valid) {
    if (usingVector) {
      if (vector) {
        in_p1 := ~io.in_uop.bits.prs1_busy
        in_p2 := ~io.in_uop.bits.prs2_busy
        in_p3 := ~io.in_uop.bits.prs3_busy
        //in_pm := ~io.in_uop.bits.prvm_busy
        ps    := ~io.in_uop.bits.v_scalar_busy
        sdata := io.in_uop.bits.v_scalar_data
        if(iqType == IQT_VMX.litValue) {
          strideLength := io.in_uop.bits.vStrideLength
        }
      } else {
        next_p1 := !io.in_uop.bits.prs1_busy(0)
        next_p2 := !io.in_uop.bits.prs2_busy(0)
        next_p3 := !io.in_uop.bits.prs3_busy(0)
        if (iqType == IQT_MEM.litValue) {
          //in_pm := !io.in_uop.bits.prvm_busy(0)
          //vxofs := io.in_uop.bits.v_xls_offset
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
      // no wakeup logics anymore
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

  //if (usingVector && !vector && iqType == IQT_MEM.litValue) {
    //next_pm := io.vmupdate.map(x => x.valid && x.bits.rob_idx === next_uop.rob_idx).reduce(_||_)
    //when(io.vmupdate.map(x => x.valid && x.bits.rob_idx === next_uop.rob_idx).reduce(_||_)) {
    //  slot_uop.v_unmasked := true.B
    //}
    //when (next_uop.v_idx_ls && io.vmupdate.map(x => x.valid && x.bits.rob_idx === next_uop.rob_idx && (io.in_uop.valid || is_valid)).reduce(_||_)) {
    //  vxofs := Mux1H(io.vmupdate.map(x => (x.valid && x.bits.rob_idx === next_uop.rob_idx && (io.in_uop.valid || is_valid), x.bits.v_xls_offset)))
    //}
    //when (IsKilledByVM(io.vmupdate, slot_uop)) {
    //  next_state := s_invalid
    //}
  //}

  when (!io.in_uop.valid) {
    slot_uop.br_mask := next_br_mask
  }

  if (vector) {
    // when intupdate or fpupdate, we need to tell if data is really we need.
    when (io.intupdate.map(_.valid).reduce(_||_) || io.fpupdate.map(_.valid).reduce(_||_)) {
      val int_sel  = io.intupdate.map(u => u.valid && u.bits.uop.prs1 === next_uop.prs1 && next_uop.rt(RS1, isInt))
      val int_data = io.intupdate.map(u => Mux(u.bits.uop.uses_stq || u.bits.uop.uses_ldq, u.bits.data,
                                           Mux(u.bits.uop.rt(RS1, isIntU), Mux1H(UIntToOH(u.bits.uop.vconfig.vtype.vsew(1,0)),
                                                                                 Seq(u.bits.data(7,0),
                                                                                     u.bits.data(15,0),
                                                                                     u.bits.data(31,0),
                                                                                     u.bits.data)),
                                                                           Mux1H(UIntToOH(u.bits.uop.vconfig.vtype.vsew(1,0)),
                                                                                 Seq(u.bits.data(7,0).sextTo(eLen),
                                                                                     u.bits.data(15,0).sextTo(eLen),
                                                                                     u.bits.data(31,0).sextTo(eLen),
                                                                                     u.bits.data)))))
      val fp_sel   = io.fpupdate.map(u => u.valid && u.bits.uop.prs1 === next_uop.prs1 && next_uop.rt(RS1, isFloat))
      val fp_data  = io.fpupdate.map(_.bits.data)
      val needUpdatePS = VecInit(int_sel ++ fp_sel).asUInt().orR()
      val updatedSData = Mux1H(int_sel ++ fp_sel, int_data ++ fp_data)
      when(needUpdatePS){
        ps := true.B
        sdata := updatedSData
      }
      if(iqType == IQT_VMX.litValue){
        val strideLengthVec = io.intupdate.map(_.bits.uop.vStrideLength)
        strideLength := Mux1H(int_sel, strideLengthVec)
      }
      assert(PopCount(int_sel++fp_sel) <= 1.U, "Multiple drivers")
    }

  }

  //-------------------------------------------------------------
  // Request Logic
  //io.request := is_valid && rs1check() && rs2check() && rs3check() && vmcheck() && ppred && !io.kill
  when (state === s_valid_1) {
    io.request := ppred && rs1check && rs2check && rs3check && vmcheck && scalarCheck && vl_ready && !io.kill
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
  if (usingVector) {
    io.uop.v_eidx := slot_uop.v_eidx
    if (vector) {
      // value to next slot should be current latched version
      // ignore element busy masking, we keep busy status for entire v-register (i.e. p1,p2,p3,pm)
      io.out_uop.prs1_busy  := ~p1 //& (slot_mask << Cat(slot_rsel, 0.U(3.W)))
      io.out_uop.prs2_busy  := ~p2 //& (slot_mask << Cat(slot_rsel, 0.U(3.W)))
      io.out_uop.prs3_busy  := ~p3 //& (slot_mask << Cat(slot_rsel, 0.U(3.W)))
      //io.out_uop.prvm_busy  := ~pm
      io.out_uop.v_scalar_busy := ~ps
      io.out_uop.v_scalar_data := sdata
      if(iqType == IQT_VMX.litValue){
        io.out_uop.vStrideLength := strideLength
      }
      io.out_uop.v_perm_busy := false.B //!perm_ready || perm_ready_vrg_bsy
      io.out_uop.v_perm_wait := false.B //perm_wait || perm_wait_vrg_set
      io.out_uop.v_perm_idx  := 0.U //perm_idx
      //io.cur_vs2_busy       := UIntToOH(slot_uop.prs2)(vpregSz-1,0) & Fill(vpregSz, !(p2.andR) && is_valid)
      // handle VOP_VI, prs1 records the value of lrpwdls
      // s1, and is used as simm5
      io.uop.v_scalar_data  := Mux(io.uop.rt(RS1, isRvvSImm5), Cat(Fill(eLen-5, io.uop.prs1(4).asUInt), io.uop.prs1(4,0)),
                               Mux(io.uop.rt(RS1, isRvvUImm5), Cat(Fill(eLen-5, 0.U(1.W)), slot_uop.prs1(4,0)), sdata))
      io.uop.v_perm_busy    := false.B //~perm_ready
      io.uop.v_perm_idx     := 0.U
      when (vcompress) {
        io.uop.pvm := slot_uop.prs1
      }
           when (io.request && io.grant && !io.uop.uopc.isOneOf(/*uopVL, uopVLFF, uopVLS, uopVLUX, uopVLOX, */uopVSA, uopVSSA, uopVSUXA, uopVSOXA)) {
        val vd_idx = Mux(slot_uop.rt(RD, isMaskVD), 0.U, VRegSel(slot_uop.v_eidx, slot_uop.vd_eew, eLenSelSz))
        io.uop.pdst := Mux(slot_uop.rt(RD, isVector), slot_uop.pvd(vd_idx).bits, slot_uop.pdst)
        assert(is_invalid || !slot_uop.rt(RD, isVector) || slot_uop.pvd(vd_idx).valid)
        when (!io.uop.is_reduce) {
          val vsew = Mux(slot_uop.rt(RS2, isWidenV) || slot_uop.rt(RD, isMaskVD), slot_uop.vs2_eew, slot_uop.vd_eew)
          val vLen_ecnt: UInt = ((vLen.U >> 3.U) >> vsew).asUInt()
          val isVLoad: Bool = slot_uop.uopc.isOneOf(uopVL, uopVLFF, uopVLS, uopVLUX, uopVLOX)
          val vLenEcntSz = vLenSz.asUInt - 3.U - vsew
          val next_offset: UInt = Mux(isVLoad, (slot_uop.v_eidx >> vLenEcntSz << vLenEcntSz).asUInt() + vLen_ecnt,
                                         slot_uop.v_eidx + vLen_ecnt)
          io.uop.v_split_ecnt := vLen_ecnt
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
      if (iqType == IQT_MEM.litValue) {
        //io.out_uop.prvm_busy := Cat(0.U, !pm)
        //io.out_uop.v_xls_offset := vxofs
        //io.uop.v_xls_offset := vxofs
      } else {
        //io.out_uop.prvm_busy := 0.U
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
    //if(usingVector && !vector && iqType == IQT_MEM.litValue) {
    //  when(io.vmupdate.map(x => x.valid && x.bits.rob_idx === io.in_uop.bits.rob_idx).reduce(_||_)) {
    //    slot_uop.v_unmasked := true.B
    //  }
    //}
    assert (is_invalid || io.clear || io.kill, "trying to overwrite a valid issue slot.")
  }

  when(io.vl_wakeup_port.valid && (io.vl_wakeup_port.bits.vconfig_tag +1.U) === next_uop.vconfig_tag) {
    slot_uop.vconfig.vl := io.vl_wakeup_port.bits.vl
    slot_uop.vl_ready := true.B
  }

  io.out_uop.vl_ready := slot_uop.vl_ready
  io.out_uop.vconfig.vl := slot_uop.vconfig.vl

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
  if (vector) {
    //pm := next_pm
  } else if (usingVector && iqType == IQT_MEM.litValue) {
    //pm := Mux(io.in_uop.valid, in_pm, pm) | next_pm
  }

  // debug outputs
  io.debug.p1 := p1
  io.debug.p2 := p2
  io.debug.p3 := p3
  io.debug.ppred := ppred
  io.debug.state := state
}
