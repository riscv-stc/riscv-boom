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
 * @param numTotalBypassPorts number of bypass ports out of the execution units
 * @param registerWidth size of register in bits
 */
class RegisterRead(
  issueWidth: Int,
  supportedUnitsArray: Seq[SupportedFuncUnits],
  numTotalReadPorts: Int,
  numReadPortsArray: Seq[Int],
                        // each exe_unit must tell us how many max
                        // operands it can accept (the sum should equal
                        // numTotalReadPorts)
  numTotalBypassPorts: Int,
  numTotalPredBypassPorts: Int,
  registerWidth: Int,
  float: Boolean = false,
  vector: Boolean = false
)(implicit p: Parameters) extends BoomModule with freechips.rocketchip.tile.HasFPUParameters
{
  val io = IO(new Bundle {
    // issued micro-ops
    val iss_valids = Input(Vec(issueWidth, Bool()))
    val iss_uops   = Input(Vec(issueWidth, new MicroOp()))

    // interface with register file's read ports
    val rf_read_ports = Flipped(Vec(numTotalReadPorts, new RegisterFileReadPortIO(maxPregSz, registerWidth)))
    val prf_read_ports = Flipped(Vec(issueWidth, new RegisterFileReadPortIO(log2Ceil(ftqSz), 1)))

    val bypass = Input(Vec(numTotalBypassPorts, Valid(new ExeUnitResp(registerWidth))))
    val pred_bypass = Input(Vec(numTotalPredBypassPorts, Valid(new ExeUnitResp(1))))

    // send micro-ops to the execution pipelines
    val exe_reqs = Vec(issueWidth, (new DecoupledIO(new FuncUnitReq(registerWidth))))
    val vmupdate = if (vector) Output(Vec(issueWidth, Valid(new MicroOp))) else null
    val vecUpdate = if (vector) Output(Vec(issueWidth, Valid(new ExeUnitResp(eLen)))) else null
    val intupdate= if (usingVector && !vector && !float) Output(Vec(intWidth, Valid(new ExeUnitResp(eLen)))) else null
    val fpupdate = if (usingVector && float) Output(Vec(issueWidth, Valid(new ExeUnitResp(eLen)))) else null
    require(!(float && vector))

    val kill   = Input(Bool())
    val brupdate = Input(new BrUpdateInfo())
  })

  val rrd_valids       = Wire(Vec(issueWidth, Bool()))
  val rrd_uops         = Wire(Vec(issueWidth, new MicroOp()))

  val exe_reg_valids   = RegInit(VecInit(Seq.fill(issueWidth) { false.B }))
  val exe_reg_uops     = Reg(Vec(issueWidth, new MicroOp()))
  val exe_reg_rs1_data = Reg(Vec(issueWidth, Bits(registerWidth.W)))
  val exe_reg_rs2_data = Reg(Vec(issueWidth, Bits(registerWidth.W)))
  val exe_reg_rs3_data = Reg(Vec(issueWidth, Bits(registerWidth.W)))
  val exe_reg_rvm_data = if (vector) Reg(Vec(issueWidth, Bool())) else null
  val exe_reg_vmaskInsn_rvm_data = if (vector) Reg(Vec(issueWidth, Bits(registerWidth.W))) else null
  val exe_reg_pred_data = Reg(Vec(issueWidth, Bool()))

  //-------------------------------------------------------------
  // hook up inputs

  for (w <- 0 until issueWidth) {
    val rrd_decode_unit = Module(new RegisterReadDecode(supportedUnitsArray(w)))
    rrd_decode_unit.io.iss_valid := io.iss_valids(w)
    rrd_decode_unit.io.iss_uop   := io.iss_uops(w)

    rrd_valids(w) := RegNext(rrd_decode_unit.io.rrd_valid &&
                !IsKilledByBranch(io.brupdate, rrd_decode_unit.io.rrd_uop))
    rrd_uops(w)   := RegNext(GetNewUopAndBrMask(rrd_decode_unit.io.rrd_uop, io.brupdate))
  }

  //-------------------------------------------------------------
  // read ports

  require (numTotalReadPorts == numReadPortsArray.reduce(_+_))

  val rrd_rs1_data   = Wire(Vec(issueWidth, Bits(registerWidth.W)))
  val rrd_rs2_data   = Wire(Vec(issueWidth, Bits(registerWidth.W)))
  val rrd_rs3_data   = Wire(Vec(issueWidth, Bits(registerWidth.W)))
  val rrd_rvm_data   = Wire(Vec(issueWidth, Bool()))
  val rrd_pred_data  = Wire(Vec(issueWidth, Bool()))
  val rrd_vmaskInsn_rvm_data   = Wire(Vec(issueWidth, Bits(registerWidth.W)))
  rrd_rs1_data := DontCare
  rrd_rs2_data := DontCare
  rrd_rs3_data := DontCare
  rrd_rvm_data := DontCare
  rrd_pred_data := DontCare
  rrd_vmaskInsn_rvm_data := DontCare

  io.prf_read_ports := DontCare

  var idx = 0 // index into flattened read_ports array
  for (w <- 0 until issueWidth) {
    val numReadPorts = numReadPortsArray(w)

    // NOTE:
    // rrdLatency==1, we need to send read address at end of ISS stage,
    //    in order to get read data back at end of RRD stage.

    val rs1_addr = io.iss_uops(w).prs1
    val rs2_addr = WireInit(io.iss_uops(w).prs2)
    val rs3_addr = WireInit(io.iss_uops(w).prs3)
    val pred_addr = io.iss_uops(w).ppred

    if (vector) {
      val rvm_addr = io.iss_uops(w).prvm
      val ecnt = io.iss_uops(w).v_split_ecnt
      // segment load/store have different element index
      val vstart = io.iss_uops(w).vstart
      // mask bits uses ordinary vstart as element index
      val vsew = io.iss_uops(w).vconfig.vtype.vsew
      if (numReadPorts > 0) {
        val is_reduce = io.iss_uops(w).rt(RD, isReduceV)
        val vs1_sew = vsew + io.iss_uops(w).rt(RS1, isWidenV).asUInt
        val vs1_vstart = Mux(is_reduce, 0.U, vstart)
        val r1_sh = Mux1H(UIntToOH(vs1_sew(1,0)), Seq(Cat(vs1_vstart(2,0),0.U(3.W)), Cat(vs1_vstart(1,0),0.U(4.W)), Cat(vs1_vstart(0),0.U(5.W)), 0.U(6.W)))
        val (r1sel,r1msk) = VRegSel(vs1_vstart, vs1_sew, ecnt, eLenb, eLenSelSz)
        val r1_bit_mask = Cat((0 until eLenb).map(i => Fill(8, r1msk(i).asUInt)).reverse)
        val rf_data1 = (io.rf_read_ports(idx+0).data & RegNext(r1_bit_mask)) >> RegNext(r1_sh)
        val signext1 = Mux1H(UIntToOH(RegNext(vs1_sew(1,0))), Seq(rf_data1(7,0).sextTo(eLen), rf_data1(15,0).sextTo(eLen), rf_data1(31,0).sextTo(eLen), rf_data1(63,0)))
        io.rf_read_ports(idx+0).addr := Cat(rs1_addr, r1sel)
        rrd_rs1_data(w) := Mux(RegNext(io.iss_uops(w).uses_scalar || io.iss_uops(w).uses_v_simm5 || io.iss_uops(w).uses_v_uimm5), RegNext(io.iss_uops(w).v_scalar_data),
                           Mux(RegNext(rs1_addr === 0.U), 0.U, Mux(RegNext(io.iss_uops(w).rt(RS1, isUnsignedV)), rf_data1, signext1)))
      }
      if (numReadPorts > 1) {
        // Note: v_ls_ew records index sew for index load/store
        val perm_on_vs2 = io.iss_uops(w).uopc.isOneOf(uopVSLIDEDOWN, uopVSLIDE1DOWN, uopVRGATHER, uopVRGATHEREI16)
        val vs2_sew = Mux(io.iss_uops(w).rt(RS2, isWidenV),  vsew+1.U,
                      Mux(io.iss_uops(w).uopc === uopVEXT8,  vsew-3.U,
                      Mux(io.iss_uops(w).uopc === uopVEXT4,  vsew-2.U,
                      Mux(io.iss_uops(w).rt(RS2, isNarrowV), vsew-1.U,
                      Mux(io.iss_uops(w).v_idx_ls, io.iss_uops(w).v_ls_ew, vsew)))))
        val vd_emul   = io.iss_uops(w).vd_emul
        val perm_idx  = io.iss_uops(w).v_perm_idx
        val perm_rinc = perm_idx >> (vLenSz.U - 3.U - vsew)
        val prs2      = io.iss_uops(w).prs2
        val perm_ridx = Mux(vd_emul === 1.U, Cat(prs2(prs2.getWidth-1, 1), perm_rinc(0)),
                        Mux(vd_emul === 2.U, Cat(prs2(prs2.getWidth-1, 2), perm_rinc(1,0)),
                        Mux(vd_emul === 3.U, Cat(prs2(prs2.getWidth-1, 3), perm_rinc(2,0)), prs2)))
        when (perm_on_vs2) {
          rs2_addr := perm_ridx
        }
        val eidx = Mux(perm_on_vs2, perm_idx(vLenSz-1,0), vstart)
        val r2_sh = Mux1H(UIntToOH(vs2_sew(1,0)), Seq(Cat(eidx(2,0),0.U(3.W)), Cat(eidx(1,0),0.U(4.W)), Cat(eidx(0),0.U(5.W)), 0.U(6.W)))
        val (r2sel,r2msk) = VRegSel(eidx, vs2_sew, ecnt, eLenb, eLenSelSz)
        val r2_bit_mask = Cat((0 until eLenb).map(i => Fill(8, r2msk(i).asUInt)).reverse)
        val rf_data2 = (io.rf_read_ports(idx+1).data & RegNext(r2_bit_mask)) >> RegNext(r2_sh)
        val signext2 = Mux1H(UIntToOH(RegNext(vs2_sew(1,0))), Seq(rf_data2(7,0).sextTo(eLen), rf_data2(15,0).sextTo(eLen), rf_data2(31,0).sextTo(eLen), rf_data2(63,0)))

        val is_vmask_iota_m = io.iss_uops(w).uopc.isOneOf(uopVIOTA)
        val vmask_iota_addr = Cat(rs2_addr, vstart >> log2Ceil(eLen))

        io.rf_read_ports(idx+1).addr := Mux(is_vmask_iota_m, vmask_iota_addr, Cat(rs2_addr, r2sel))
        rrd_rs2_data(w) := Mux(RegNext(rs2_addr === 0.U), 0.U, Mux(is_vmask_iota_m, io.rf_read_ports(idx+1).data, Mux(RegNext(io.iss_uops(w).rt(RS2, isUnsignedV)), rf_data2, signext2)))
      }
      if (numReadPorts > 2) {
        val perm_on_vd = io.iss_uops(w).uopc.isOneOf(uopVSLIDEUP, uopVSLIDE1UP, uopVCOMPRESS)
        val vd_sew  = Mux(io.iss_uops(w).uses_v_ls_ew, io.iss_uops(w).v_ls_ew,
                      Mux(io.iss_uops(w).rt(RD, isWidenV ), vsew + 1.U,
                      Mux(io.iss_uops(w).rt(RD, isNarrowV), vsew - 1.U, vsew)))
        val vd_emul   = io.iss_uops(w).vd_emul
        val perm_idx  = io.iss_uops(w).v_perm_idx
        val perm_rinc = perm_idx >> (vLenSz.U - 3.U - vsew)
        val prs3      = io.iss_uops(w).prs3
        val perm_ridx = Mux(vd_emul === 1.U, Cat(prs3(prs3.getWidth-1, 1), perm_rinc(0)),
                        Mux(vd_emul === 2.U, Cat(prs3(prs3.getWidth-1, 2), perm_rinc(1,0)),
                        Mux(vd_emul === 3.U, Cat(prs3(prs3.getWidth-1, 3), perm_rinc(2,0)), prs3)))
        when (perm_on_vd) {
          rs3_addr := perm_ridx
        }
        val eidx = Mux(perm_on_vd, perm_idx(vLenSz-1,0), vstart)
        val rd_sh = Mux1H(UIntToOH(vd_sew(1,0)),  Seq(Cat(eidx(2,0),0.U(3.W)), Cat(eidx(1,0),0.U(4.W)), Cat(eidx(0),0.U(5.W)), 0.U(6.W)))
        val (rdsel,rdmsk) = VRegSel(eidx, vd_sew,  ecnt, eLenb, eLenSelSz)
        val rd_bit_mask = Cat((0 until eLenb).map(i => Fill(8, rdmsk(i).asUInt)).reverse)
        val rf_data3 = (io.rf_read_ports(idx+2).data & RegNext(rd_bit_mask)) >> RegNext(rd_sh)
        val signext3 = Mux1H(UIntToOH(RegNext(vd_sew(1,0))), Seq(rf_data3(7,0).sextTo(eLen), rf_data3(15,0).sextTo(eLen), rf_data3(31,0).sextTo(eLen), rf_data3(63,0)))
        val maskvd = io.iss_uops(w).rt(RD, isMaskVD)
        val maskvd_rsel = (vstart >> eLenSz)(eLenSelSz-1, 0)
        val maskvd_data = Cat(0.U((eLen-1).W), io.rf_read_ports(idx+2).data(RegNext(vstart(eLenSz-1, 0))))
        io.rf_read_ports(idx+2).addr := Cat(rs3_addr, Mux(maskvd, maskvd_rsel, rdsel))
        rrd_rs3_data(w) := Mux(RegNext(rs3_addr === 0.U), 0.U, Mux(RegNext(maskvd), maskvd_data,
                                                               Mux(RegNext(io.iss_uops(w).rt(RD, isUnsignedV)), rf_data3, signext3)))
      }
      if (numReadPorts > 3) { // handle vector mask
        val is_vmask_cnt_m = io.iss_uops(w).uopc.isOneOf(uopVPOPC, uopVFIRST)
        val is_vmask_set_m = io.iss_uops(w).uopc.isOneOf(uopVMSOF, uopVMSBF, uopVMSIF)
        val perm_on_vd     = io.iss_uops(w).uopc.isOneOf(uopVSLIDEUP, uopVSLIDE1UP)
        val perm_idx       = io.iss_uops(w).v_perm_idx
        val eidx           = Mux(perm_on_vd, perm_idx(vLenSz-1,0), vstart)
        // need make mask and element sync
        val vmask_cnt_set_insn_mask_addr = Cat(rvm_addr, vstart(3,0))
        io.rf_read_ports(idx+3).addr := Mux(is_vmask_cnt_m | is_vmask_set_m, vmask_cnt_set_insn_mask_addr,
                                        Cat(rvm_addr, eidx >> log2Ceil(eLen)))
        rrd_rvm_data(w) := io.rf_read_ports(idx+3).data(RegNext(eidx(log2Ceil(eLen)-1,0)))
        rrd_vmaskInsn_rvm_data(w) := io.rf_read_ports(idx+3).data
        assert(!(io.iss_valids(w) && io.iss_uops(w).uopc.isOneOf(uopVL, uopVLS) && io.iss_uops(w).v_unmasked &&
               io.iss_uops(w).vstart < io.iss_uops(w).vconfig.vl), "unmasked vecotr load (except indexed load) should not come here.")
      }
    } else {
      if (numReadPorts > 0) io.rf_read_ports(idx+0).addr := rs1_addr
      if (numReadPorts > 1) io.rf_read_ports(idx+1).addr := rs2_addr
      if (numReadPorts > 2) io.rf_read_ports(idx+2).addr := rs3_addr
      if (numReadPorts > 0) rrd_rs1_data(w) := Mux(RegNext(rs1_addr === 0.U), 0.U, io.rf_read_ports(idx+0).data)
      if (numReadPorts > 1) rrd_rs2_data(w) := Mux(RegNext(rs2_addr === 0.U), 0.U, io.rf_read_ports(idx+1).data)
      if (numReadPorts > 2) rrd_rs3_data(w) := Mux(RegNext(rs3_addr === 0.U), 0.U, io.rf_read_ports(idx+2).data)
    }

    if (enableSFBOpt) {
      io.prf_read_ports(w).addr := pred_addr
      rrd_pred_data(w) := Mux(RegNext(io.iss_uops(w).is_sfb_shadow), io.prf_read_ports(w).data, false.B)
    }

    val rrd_kill = io.kill || IsKilledByBranch(io.brupdate, rrd_uops(w))

    exe_reg_valids(w) := Mux(rrd_kill, false.B, rrd_valids(w))
    // TODO use only the valids signal, don't require us to set nullUop
    exe_reg_uops(w)   := Mux(rrd_kill, NullMicroOp(), rrd_uops(w))

    exe_reg_uops(w).br_mask := GetNewBrMask(io.brupdate, rrd_uops(w))

    idx += numReadPorts
  }

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // BYPASS MUXES -----------------------------------------------
  // performed at the end of the register read stage

  // NOTES: this code is fairly hard-coded. Sorry.
  // ASSUMPTIONS:
  //    - rs3 is used for FPU ops which are NOT bypassed (so don't check
  //       them!).
  //    - only bypass integer registers.

  val bypassed_rs1_data = Wire(Vec(issueWidth, Bits(registerWidth.W)))
  val bypassed_rs2_data = Wire(Vec(issueWidth, Bits(registerWidth.W)))
  val bypassed_pred_data = Wire(Vec(issueWidth, Bool()))
  bypassed_pred_data := DontCare

  for (w <- 0 until issueWidth) {
    val numReadPorts = numReadPortsArray(w)
    var rs1_cases = Array((false.B, 0.U(registerWidth.W)))
    var rs2_cases = Array((false.B, 0.U(registerWidth.W)))
    var pred_cases = Array((false.B, 0.U(1.W)))

    val prs1       = rrd_uops(w).prs1
    val prs2       = rrd_uops(w).prs2
    val ppred      = rrd_uops(w).ppred

    for (b <- 0 until numTotalBypassPorts)
    {
      val bypass = io.bypass(b)
      // can't use "io.bypass.valid(b) since it would create a combinational loop on branch kills"
      rs1_cases ++= Array((bypass.valid && (prs1 === bypass.bits.uop.pdst) && bypass.bits.uop.rf_wen
        && bypass.bits.uop.rt(RD, isInt) && rrd_uops(w).rt(RS1, isInt) && (prs1 =/= 0.U), bypass.bits.data))
      rs2_cases ++= Array((bypass.valid && (prs2 === bypass.bits.uop.pdst) && bypass.bits.uop.rf_wen
        && bypass.bits.uop.rt(RD, isInt) && rrd_uops(w).rt(RS2, isInt) && (prs2 =/= 0.U), bypass.bits.data))
    }

    for (b <- 0 until numTotalPredBypassPorts)
    {
      val bypass = io.pred_bypass(b)
      pred_cases ++= Array((bypass.valid && (ppred === bypass.bits.uop.pdst) && bypass.bits.uop.is_sfb_br, bypass.bits.data))
    }

    if (numReadPorts > 0) bypassed_rs1_data(w)  := MuxCase(rrd_rs1_data(w), rs1_cases)
    if (numReadPorts > 1) bypassed_rs2_data(w)  := MuxCase(rrd_rs2_data(w), rs2_cases)
    if (enableSFBOpt)     bypassed_pred_data(w) := MuxCase(rrd_pred_data(w), pred_cases)
  }

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Execute Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  for (w <- 0 until issueWidth) {
    val numReadPorts = numReadPortsArray(w)
    if (numReadPorts > 0) exe_reg_rs1_data(w) := bypassed_rs1_data(w)
    if (numReadPorts > 1) exe_reg_rs2_data(w) := bypassed_rs2_data(w)
    if (numReadPorts > 2) exe_reg_rs3_data(w) := rrd_rs3_data(w)
    if (numReadPorts > 3 && vector) exe_reg_rvm_data(w) := rrd_rvm_data(w)
    if (numReadPorts > 3 && vector) exe_reg_vmaskInsn_rvm_data(w) := rrd_vmaskInsn_rvm_data(w)
    if (enableSFBOpt)     exe_reg_pred_data(w) := bypassed_pred_data(w)
    // ASSUMPTION: rs3 is FPU which is NOT bypassed
  }
  // TODO add assert to detect bypass conflicts on non-bypassable things
  // TODO add assert that checks bypassing to verify there isn't something it hits rs3

  //-------------------------------------------------------------
  // set outputs to execute pipelines
  for (w <- 0 until issueWidth) {
    val numReadPorts = numReadPortsArray(w)

    io.exe_reqs(w).valid    := exe_reg_valids(w)
    io.exe_reqs(w).bits.uop := exe_reg_uops(w)
    if (numReadPorts > 0) io.exe_reqs(w).bits.rs1_data := exe_reg_rs1_data(w)
    if (numReadPorts > 1) io.exe_reqs(w).bits.rs2_data := exe_reg_rs2_data(w)
    if (numReadPorts > 2) io.exe_reqs(w).bits.rs3_data := exe_reg_rs3_data(w)
    if (enableSFBOpt)     io.exe_reqs(w).bits.pred_data := exe_reg_pred_data(w)

    if (usingVector) {
      if (!vector && !float) {
        // avoid mem pipes (lower indexed)
        if (w >= memWidth && w < memWidth+intWidth) {
          val is_setvl = exe_reg_uops(w).uopc.isOneOf(uopVSETVLI, uopVSETIVLI, uopVSETVL)
          io.exe_reqs(w).valid := exe_reg_valids(w) && (is_setvl || !exe_reg_uops(w).is_rvv)
          io.intupdate(w - memWidth).valid := exe_reg_valids(w) && exe_reg_uops(w).is_rvv && !is_setvl
          io.intupdate(w - memWidth).bits.uop := exe_reg_uops(w)
          io.intupdate(w - memWidth).bits.data := exe_reg_rs1_data(w)
        }
      } else if (float) {
        io.exe_reqs(w).valid := exe_reg_valids(w) && !exe_reg_uops(w).is_rvv
        io.fpupdate(w).valid := exe_reg_valids(w) && exe_reg_uops(w).is_rvv
        io.fpupdate(w).bits.uop := exe_reg_uops(w)
        io.fpupdate(w).bits.data := ieee(exe_reg_rs1_data(w))
      } else if (vector) {
        val uses_ldq   = exe_reg_uops(w).is_rvv && exe_reg_uops(w).uses_ldq
        val uses_stq   = exe_reg_uops(w).is_rvv && exe_reg_uops(w).uses_stq
        val is_masked  = !exe_reg_uops(w).v_unmasked
        val is_idx_ls  = exe_reg_uops(w).is_rvv && exe_reg_uops(w).v_idx_ls
        val vstart     = exe_reg_uops(w).vstart
        val vl         = exe_reg_uops(w).vconfig.vl
        val vmlogic    = exe_reg_uops(w).ctrl.is_vmlogic
        val is_vmask_cnt_m     = exe_reg_uops(w).uopc.isOneOf(uopVPOPC, uopVFIRST)
        val is_vmask_set_m     = exe_reg_uops(w).uopc.isOneOf(uopVMSOF, uopVMSBF, uopVMSIF)
        val is_vmask_iota_m    = exe_reg_uops(w).uopc.isOneOf(uopVIOTA)
        val is_vmaskInsn       = vmlogic || is_vmask_cnt_m || is_vmask_set_m
        val byteWidth          = 3.U
        val vsew64bit          = 3.U
        val vmaskInsn_vl       = vl(5,0).orR +& (vl>>(byteWidth +& vsew64bit))
        val vmaskInsn_active   = vstart < vmaskInsn_vl
        val is_active          = Mux(is_vmaskInsn, vmaskInsn_active, Mux(is_masked, exe_reg_rvm_data(w), true.B)) && vstart < vl
        val vmaskInsn_rs2_data = Mux(is_masked, exe_reg_rs2_data(w) & exe_reg_vmaskInsn_rvm_data(w), exe_reg_rs2_data(w))
        val is_perm_fdbk       = exe_reg_uops(w).uopc.isOneOf(uopVRGATHER, uopVRGATHEREI16, uopVCOMPRESS) && exe_reg_uops(w).v_perm_busy

        io.exe_reqs(w).bits.rs2_data    := Mux(is_vmask_cnt_m | is_vmask_set_m, vmaskInsn_rs2_data, exe_reg_rs2_data(w))
        io.exe_reqs(w).bits.rs1_data    := Mux(is_vmask_set_m | is_vmask_iota_m, exe_reg_vmaskInsn_rvm_data(w), exe_reg_rs1_data(w))

        io.exe_reqs(w).valid    := exe_reg_valids(w) && !(uses_ldq && is_active) && !is_perm_fdbk
        io.vmupdate(w).valid    := exe_reg_valids(w) && ((uses_stq || uses_ldq) && (is_masked || is_idx_ls))
        val vmove: Bool = VecInit(Seq(exe_reg_uops(w).uopc === uopVFMV_S_F,
          exe_reg_uops(w).uopc === uopVFMV_F_S,
          exe_reg_uops(w).uopc === uopVMV_X_S,
          exe_reg_uops(w).uopc === uopVMV_S_X
        )).asUInt().orR()

        io.vmupdate(w).bits               := exe_reg_uops(w)
        io.vmupdate(w).bits.v_active      := is_active
        io.vmupdate(w).bits.v_xls_offset  := exe_reg_rs2_data(w)
        io.vecUpdate(w).valid             := exe_reg_valids(w) && is_perm_fdbk
        io.vecUpdate(w).bits.uop          := exe_reg_uops(w)
        io.vecUpdate(w).bits.uop.v_active := is_active
        io.vecUpdate(w).bits.data         := exe_reg_rs1_data(w)
        io.exe_reqs(w).bits.uop.v_active := Mux(vmove, !vstart.orR(), is_active)
        val vfdiv_sqrt = (io.exe_reqs(w).bits.uop.uopc === uopVFDIV)  ||
                         (io.exe_reqs(w).bits.uop.uopc === uopVFRDIV) ||
                         (io.exe_reqs(w).bits.uop.uopc === uopVFSQRT)
        // forward inactive ops to ALU
        val withCarry  = io.exe_reqs(w).bits.uop.uopc.isOneOf(uopVADC, uopVSBC, uopVMADC, uopVMSBC)
        val vmscmp     = io.exe_reqs(w).bits.uop.ctrl.is_vmscmp
        when ((io.exe_reqs(w).bits.uop.is_rvv && !is_active && !vfdiv_sqrt && !withCarry && !vmscmp && !is_vmask_iota_m) || (vmove && vstart.orR())) {
          io.exe_reqs(w).bits.uop.fu_code := boom.exu.FUConstants.FU_ALU
          io.exe_reqs(w).bits.uop.ctrl.op_fcn := freechips.rocketchip.rocket.ALU.FN_ADD
        }
      }
    }
  }
}
