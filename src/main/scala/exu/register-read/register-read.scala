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
import freechips.rocketchip.rocket.{VConfig}

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
    val rrd_stall  = Output(Vec(issueWidth, Bool()))

    // interface with register file's read ports
    val rf_read_ports = Flipped(Vec(numTotalReadPorts, new RegisterFileReadPortIO(maxPregSz, registerWidth)))
    val prf_read_ports = Flipped(Vec(issueWidth, new RegisterFileReadPortIO(log2Ceil(ftqSz), 1)))

    val bypass = Input(Vec(numTotalBypassPorts, Valid(new ExeUnitResp(registerWidth))))
    val pred_bypass = Input(Vec(numTotalPredBypassPorts, Valid(new ExeUnitResp(1))))

    // send micro-ops to the execution pipelines
    val exe_reqs = Vec(issueWidth, (new DecoupledIO(new FuncUnitReq(registerWidth))))
    //val vmupdate = if (vector) Output(Vec(1, Valid(new MicroOp))) else null
    val intupdate = if (usingVector && !vector && !float) Output(Vec(issueWidth, Valid(new ExeUnitResp(eLen)))) else null
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
  val exe_reg_rvm_data = if (vector) Reg(Vec(issueWidth, Bits((registerWidth).W))) else null
  val exe_reg_rvm_align = if (vector) Reg(Vec(issueWidth, Bits((registerWidth/8).W))) else null
  //val exe_reg_vmaskInsn_rvm_data = if (vector) Reg(Vec(issueWidth, Bits(registerWidth.W))) else null
  val exe_reg_pred_data = Reg(Vec(issueWidth, Bool()))

  //-------------------------------------------------------------
  // read ports

  require (numTotalReadPorts == numReadPortsArray.reduce(_+_))

  val rrd_rs1_data   = Wire(Vec(issueWidth, Bits(registerWidth.W)))
  val rrd_rs2_data   = Wire(Vec(issueWidth, Bits(registerWidth.W)))
  val rrd_rs3_data   = Wire(Vec(issueWidth, Bits(registerWidth.W)))
  val rrd_rvm_data   = Wire(Vec(issueWidth, Bits(registerWidth.W)))
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
    //-------------------------------------------------------------
    // hook up inputs
    val rrd_decode_unit = Module(new RegisterReadDecode(supportedUnitsArray(w)))
    rrd_decode_unit.io.iss_valid := io.iss_valids(w)
    rrd_decode_unit.io.iss_uop   := io.iss_uops(w)

    rrd_valids(w) := RegNext(rrd_decode_unit.io.rrd_valid &&
                !IsKilledByBranch(io.brupdate, rrd_decode_unit.io.rrd_uop))
    rrd_uops(w)   := RegNext(GetNewUopAndBrMask(rrd_decode_unit.io.rrd_uop, io.brupdate))

    val numReadPorts = numReadPortsArray(w)

    // NOTE:
    // rrdLatency==1, we need to send read address at end of ISS stage,
    //    in order to get read data back at end of RRD stage.

    val rs1_addr = WireInit(io.iss_uops(w).prs1)
    val rs2_addr = WireInit(io.iss_uops(w).prs2)
    val rs3_addr = WireInit(io.iss_uops(w).prs3)
    val rvm_addr = if (vector) WireInit(io.iss_uops(w).pvm) else 0.U
    val pred_addr = io.iss_uops(w).ppred
    io.rrd_stall(w) := false.B

    if (vector) {
      val iss_uop = Wire(new MicroOp())
      iss_uop := io.iss_uops(w)
      val vs1_nr  = nrVecGroup(iss_uop.vs1_emul)
      val vs2_nr  = nrVecGroup(iss_uop.vs2_emul)
      val vd_nr   = nrVecGroup(iss_uop.vd_emul)
      if (numReadPorts > 3) { // only for vec pipe, skip for vmx pipe
        val vrp_iss = io.iss_valids(w) && (io.iss_uops(w).fu_code & FU_VRP).orR && vs2_nr > 1.U
        val vrp_val = RegInit(false.B)
        val vrp_last = Wire(Bool())
        val vrp_uop = Reg(new MicroOp())
        val vlen_ecnt = Wire(UInt((vLen/8).W))
        vrp_last := false.B
        when (vrp_val) {
          val rs2_sel = VRegSel(vrp_uop.v_eidx, vrp_uop.vs2_eew, eLenSelSz)
          vlen_ecnt := ((vLen/8).U >> vrp_uop.vs2_eew)
          vrp_last := (rs2_sel +& 1.U === nrVecGroup(vrp_uop.vs2_emul))
          when (vrp_last) {
            vrp_val := false.B
          }
          vrp_uop.v_eidx := vrp_uop.v_eidx + vlen_ecnt
          // loop on vrp_uop
          iss_uop := vrp_uop
          iss_uop.v_split_last := vrp_last
        } .otherwise {
          vlen_ecnt := ((vLen/8).U >> io.iss_uops(w).vs2_eew)
          vrp_val := vrp_iss
          when(vrp_iss) {
            vrp_uop := io.iss_uops(w)
            vrp_uop.v_eidx := io.iss_uops(w).v_eidx + vlen_ecnt
          }
        }
        when (vrp_iss) {
          assert(!vrp_val)
        }
        io.rrd_stall(w) := vrp_val
        rrd_decode_unit.io.iss_valid := io.iss_valids(w) | vrp_val
        rrd_decode_unit.io.iss_uop   := Mux(vrp_val, vrp_uop, io.iss_uops(w))
        when (vrp_val || vrp_iss) {
          rrd_decode_unit.io.iss_uop.v_split_ecnt := vlen_ecnt
        }
      }
      val rs1_sel = Mux(vs1_nr === 0.U, 0.U, VRegSel(iss_uop.v_eidx, iss_uop.vs1_eew, eLenSelSz))
      val rs2_sel = Mux(vs2_nr === 0.U, 0.U, VRegSel(iss_uop.v_eidx, iss_uop.vs2_eew, eLenSelSz))
      val rs3_sel = Mux(vd_nr  === 0.U, 0.U, VRegSel(iss_uop.v_eidx, iss_uop.vd_eew, eLenSelSz))
      val isIndexedLS = iss_uop.v_idx_ls
//    val rs3_sel = Mux(iss_uop.rt(RD, isMaskVD), VRegSel(iss_uop.v_eidx >> 3, 0.U, eLenSelSz),
//                                                VRegSel(iss_uop.v_eidx, iss_uop.vd_eew, eLenSelSz))
      rs1_addr := iss_uop.pvs1(rs1_sel).bits
      rs2_addr := Mux(iss_uop.is_vmv_s2v, 0.U, iss_uop.pvs2(rs2_sel).bits)
      rs3_addr := Mux(isIndexedLS, iss_uop.pvs2(0).bits, iss_uop.stale_pvd(rs3_sel).bits)
      rvm_addr := iss_uop.pvm
    }

    if (vector) {
      if (numReadPorts > 0) io.rf_read_ports(idx+0).addr := rvm_addr
      if (numReadPorts > 1) io.rf_read_ports(idx+1).addr := rs3_addr
      if (numReadPorts > 2) io.rf_read_ports(idx+2).addr := rs2_addr
      if (numReadPorts > 3) io.rf_read_ports(idx+3).addr := rs1_addr
      if (numReadPorts > 0) rrd_rvm_data(w) := Mux(RegNext(rvm_addr === 0.U), 0.U, io.rf_read_ports(idx+0).data)
      if (numReadPorts > 1) rrd_rs3_data(w) := Mux(RegNext(rs3_addr === 0.U), 0.U, io.rf_read_ports(idx+1).data)
      if (numReadPorts > 2) rrd_rs2_data(w) := Mux(RegNext(rs2_addr === 0.U), 0.U, io.rf_read_ports(idx+2).data)
      if (numReadPorts > 3) rrd_rs1_data(w) := Mux(RegNext(rs1_addr === 0.U), 0.U, io.rf_read_ports(idx+3).data)
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
    if (vector) {
      if (numReadPorts > 0) {
        exe_reg_rvm_data(w)  := rrd_rvm_data(w)
        exe_reg_rvm_align(w) := rrd_rvm_data(w) >> rrd_uops(w).v_eidx
      }
      if (numReadPorts > 1) {
        exe_reg_rs3_data(w) := Mux(rrd_uops(w).is_rvv & (rrd_uops(w).uses_stq | rrd_uops(w).uses_ldq), rrd_rs3_data(w),
                               Mux(rrd_uops(w).rt(RD, isMaskVD), rrd_rs3_data(w) >> rrd_uops(w).v_eidx,
                                   rrd_rs3_data(w) >> (rrd_uops(w).v_eidx << (rrd_uops(w).vd_eew +& 3.U))(vLenSz-1, 0)))
      }
      if (numReadPorts > 2) {
        val isViotaId = rrd_uops(w).uopc.isOneOf(uopVIOTA, uopVID)
        exe_reg_rs2_data(w) := Mux(isViotaId, rrd_rs2_data(w) >> rrd_uops(w).v_eidx,
                                              rrd_rs2_data(w) >> (rrd_uops(w).v_eidx << (rrd_uops(w).vs2_eew +& 3.U))(vLenSz-1, 0))
      }
      if (numReadPorts > 3) {
        val useScalar = rrd_uops(w).uses_scalar || rrd_uops(w).uses_v_simm5 || rrd_uops(w).uses_v_uimm5
        val isVmask   = rrd_uops(w).uopc.isOneOf(uopVPOPC, uopVFIRST, uopVMSBF, uopVMSIF, uopVMSOF)
        exe_reg_rs1_data(w) := Mux(useScalar, Mux1H(UIntToOH(rrd_uops(w).vs1_eew),
                                                    Seq(Fill(8*numELENinVLEN, rrd_uops(w).v_scalar_data(7,  0)),
                                                        Fill(4*numELENinVLEN, rrd_uops(w).v_scalar_data(15, 0)),
                                                        Fill(2*numELENinVLEN, rrd_uops(w).v_scalar_data(31, 0)),
                                                        Fill(numELENinVLEN,   rrd_uops(w).v_scalar_data(63, 0)))),
                               Mux(isVmask,   rrd_rvm_data(w),
                                              rrd_rs1_data(w) >> (rrd_uops(w).v_eidx << (rrd_uops(w).vs1_eew +& 3.U))(vLenSz-1, 0)))
      }
    } else {
      if (numReadPorts > 0) exe_reg_rs1_data(w) := bypassed_rs1_data(w)
      if (numReadPorts > 1) exe_reg_rs2_data(w) := bypassed_rs2_data(w)
      if (numReadPorts > 2) exe_reg_rs3_data(w) := rrd_rs3_data(w)
    }
    //if (numReadPorts > 3 && vector) exe_reg_vmaskInsn_rvm_data(w) := rrd_vmaskInsn_rvm_data(w)
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
    io.exe_reqs(w).bits.rvmFull := DontCare
    if (vector) {
      if (numReadPorts > 0) io.exe_reqs(w).bits.rvm_data := exe_reg_rvm_align(w)
      if (numReadPorts > 0) io.exe_reqs(w).bits.rvmFull := exe_reg_rvm_data(w)
      if (numReadPorts > 1) io.exe_reqs(w).bits.rs3_data := exe_reg_rs3_data(w)
      if (numReadPorts > 2) io.exe_reqs(w).bits.rs2_data := exe_reg_rs2_data(w)
      if (numReadPorts > 3) io.exe_reqs(w).bits.rs1_data := exe_reg_rs1_data(w)
    } else {
      if (numReadPorts > 0) io.exe_reqs(w).bits.rs1_data := exe_reg_rs1_data(w)
      if (numReadPorts > 1) io.exe_reqs(w).bits.rs2_data := exe_reg_rs2_data(w)
      if (numReadPorts > 2) io.exe_reqs(w).bits.rs3_data := exe_reg_rs3_data(w)
    }
    if (enableSFBOpt)     io.exe_reqs(w).bits.pred_data := exe_reg_pred_data(w)

    if (usingVector) {
      if (!vector && !float) {
        val is_setvl = exe_reg_uops(w).uopc.isOneOf(uopVSETVLI, uopVSETIVLI, uopVSETVL)
        io.intupdate(w).valid := exe_reg_valids(w) && exe_reg_uops(w).is_rvv && !is_setvl
        io.intupdate(w).bits.uop := exe_reg_uops(w)
        io.intupdate(w).bits.data := exe_reg_rs1_data(w)
        io.intupdate(w).bits.uop.vStrideLength := exe_reg_rs2_data(w)
        // avoid mem pipes (lower indexed)
        if (w >= memWidth && w < memWidth+intWidth) {
          io.exe_reqs(w).valid := exe_reg_valids(w) && (is_setvl || !exe_reg_uops(w).is_rvv)
          //io.intupdate(w - memWidth).valid := exe_reg_valids(w) && exe_reg_uops(w).is_rvv && !is_setvl
          //io.intupdate(w - memWidth).bits.uop := exe_reg_uops(w)
          //io.intupdate(w - memWidth).bits.data := exe_reg_rs1_data(w)
          //io.intupdate(w - memWidth).bits.uop.vStrideLength := exe_reg_rs2_data(w)
        }
      } else if (float) {
        io.exe_reqs(w).valid := exe_reg_valids(w) && !exe_reg_uops(w).is_rvv
        io.fpupdate(w).valid := exe_reg_valids(w) && exe_reg_uops(w).is_rvv
        io.fpupdate(w).bits.uop := exe_reg_uops(w)
        io.fpupdate(w).bits.data := ieee(exe_reg_rs1_data(w))
      } else if (vector) {
        //val uses_ldq   = exe_reg_uops(w).is_rvv && exe_reg_uops(w).uses_ldq
        //val uses_stq   = exe_reg_uops(w).is_rvv && exe_reg_uops(w).uses_stq
        //val is_load    = exe_reg_uops(w).ctrl.is_load
        //val is_sta     = exe_reg_uops(w).ctrl.is_sta
        //val is_masked  = !exe_reg_uops(w).v_unmasked
        //val is_idx_ls  = exe_reg_uops(w).is_rvv && exe_reg_uops(w).v_idx_ls
        //val v_eidx     = exe_reg_uops(w).v_eidx
        //val vl         = exe_reg_uops(w).vconfig.vl
        //val vmlogic    = exe_reg_uops(w).ctrl.is_vmlogic
        //val is_vmask_cnt_m     = exe_reg_uops(w).uopc.isOneOf(uopVPOPC, uopVFIRST)
        //val is_vmask_set_m     = exe_reg_uops(w).uopc.isOneOf(uopVMSOF, uopVMSBF, uopVMSIF)
        //val is_vmask_iota_m    = exe_reg_uops(w).uopc.isOneOf(uopVIOTA)
        //val is_vmaskInsn       = vmlogic || is_vmask_cnt_m || is_vmask_set_m
        //val byteWidth          = 3.U
        //val vsew64bit          = 3.U
        //val vmaskInsn_vl       = vl(5,0).orR +& (vl>>(byteWidth +& vsew64bit))
        //val vmaskInsn_active   = v_eidx < vmaskInsn_vl
        //val vslideup           = exe_reg_uops(w).uopc === uopVSLIDEUP
        //val vcompress          = exe_reg_uops(w).uopc === uopVCOMPRESS
        //val perm_idx           = exe_reg_uops(w).v_perm_idx
        //val is_active          = Mux(is_masked, exe_reg_rvm_data(w)(v_eidx), true.B) && v_eidx < vl && v_eidx >= exe_reg_uops(w).vstart
                                 //Mux(is_vmaskInsn, vmaskInsn_active,
                                 //Mux(vslideup,  exe_reg_uops(w).v_eidx >= exe_reg_uops(w).v_scalar_data && (exe_reg_rvm_data(w) || !is_masked),
                                 //Mux(is_masked || vcompress, exe_reg_rvm_data(w), true.B))) && Mux(vcompress, perm_idx, v_eidx) < vl && v_eidx >= io.csr_vstart
        //val vmaskInsn_rs2_data = Mux(is_masked, exe_reg_rs2_data(w) & exe_reg_vmaskInsn_rvm_data(w), exe_reg_rs2_data(w))

        //val vshift  = exe_reg_uops(w).uopc.isOneOf(uopVSLL, uopVSRL, uopVSRA)
        //val vs2_sew = exe_reg_uops(w).vs2_eew
        //val vshift_rs1_data = Mux(vs2_sew(1,0) === 3.U, exe_reg_rs1_data(w)(5,0),
        //                      Mux(vs2_sew(1,0) === 2.U, exe_reg_rs1_data(w)(4,0),
        //                      Mux(vs2_sew(1,0) === 1.U, exe_reg_rs1_data(w)(3,0), exe_reg_rs1_data(w)(2,0))))

        //io.exe_reqs(w).bits.rs1_data    := Mux(is_vmask_set_m | is_vmask_iota_m, exe_reg_vmaskInsn_rvm_data(w),
        //                                   Mux(vshift, vshift_rs1_data, exe_reg_rs1_data(w)))
        //io.exe_reqs(w).bits.rs2_data    := Mux(is_vmask_cnt_m | is_vmask_set_m, vmaskInsn_rs2_data, exe_reg_rs2_data(w))

        //val is_perm_fdbk       = exe_reg_uops(w).uopc.isOneOf(uopVRGATHER, uopVRGATHEREI16, uopVCOMPRESS) && exe_reg_uops(w).v_perm_busy
        //io.exe_reqs(w).valid    := exe_reg_valids(w) && !(uses_ldq && is_active) && (!is_perm_fdbk || vcompress && (!exe_reg_uops(w).v_perm_busy || exe_reg_rvm_data(w)))
        //io.exe_reqs(w).valid    := exe_reg_valids(w) && (!is_sta || is_active)
        io.exe_reqs(w).valid := exe_reg_valids(w)
        //val vmove: Bool = VecInit(Seq(exe_reg_uops(w).uopc === uopVFMV_S_F,
          //exe_reg_uops(w).uopc === uopVFMV_F_S,
          //exe_reg_uops(w).uopc === uopVMV_X_S,
          //exe_reg_uops(w).uopc === uopVMV_S_X
        //)).asUInt().orR()

        //if (w >= vecWidth) {
        //  io.vmupdate(w-vecWidth).valid  := exe_reg_valids(w) && ((is_sta || is_load) && (is_masked || is_idx_ls))
        //  io.vmupdate(w-vecWidth).bits   := exe_reg_uops(w)
        //  io.vmupdate(w-vecWidth).bits.v_active        := is_active
        //  io.vmupdate(w-vecWidth).bits.v_xls_offset    := exe_reg_rs2_data(w)
        //}
        //io.vecUpdate(w).valid               := false.B // FIXME: exe_reg_valids(w) && is_perm_fdbk
        //io.vecUpdate(w).bits.uop            := exe_reg_uops(w)
        //io.vecUpdate(w).bits.uop.v_active   := false.B //exe_reg_rvm_data(w) && (v_eidx < vl)
        //io.vecUpdate(w).bits.uop.v_perm_idx := 0.U //perm_idx + (v_eidx < vl)
        //io.vecUpdate(w).bits.data           := 0.U //exe_reg_rs1_data(w)
        //io.exe_reqs(w).bits.uop.v_active := is_active //Mux(vmove, !v_eidx.orR(), is_active)
        when(io.exe_reqs(w).bits.uop.is_rvv && io.exe_reqs(w).bits.uop.uopc.isOneOf(uopVFMV_V_F, uopVFMV_S_F)) {
          io.exe_reqs(w).bits.uop.fu_code := boom.exu.FUConstants.FU_ALU
        }
        //val vdiv_sqrt = io.exe_reqs(w).bits.uop.uopc.isOneOf(uopVFDIV, uopVFRDIV, uopVFSQRT, uopVDIV, uopVDIVU, uopVREM, uopVREMU)
        //val is_vmx = io.exe_reqs(w).bits.uop.uopc.isOneOf(uopVSA, uopVSMA, uopVSSA, uopVLUX, uopVSUXA, uopVLOX, uopVSOXA, uopVSR)
        // forward inactive ops to ALU
        //val withCarry  = io.exe_reqs(w).bits.uop.uopc.isOneOf(uopVADC, uopVSBC, uopVMADC, uopVMSBC)
        //val vmscmp     = io.exe_reqs(w).bits.uop.ctrl.is_vmscmp
        //when ((io.exe_reqs(w).bits.uop.is_rvv && !is_active && !vdiv_sqrt && !is_vmx && !withCarry && !vmscmp && !is_vmask_iota_m) || (vmove && v_eidx.orR())) {
        //  io.exe_reqs(w).bits.uop.fu_code := boom.exu.FUConstants.FU_ALU
        //  io.exe_reqs(w).bits.uop.ctrl.op_fcn := freechips.rocketchip.rocket.ALU.FN_ADD
        //}
      }
    }
  }
}
