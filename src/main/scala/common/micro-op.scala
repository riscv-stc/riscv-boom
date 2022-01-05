//******************************************************************************
// Copyright (c) 2015 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// MicroOp
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.common

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.Parameters
import freechips.rocketchip.rocket.VConfig
import freechips.rocketchip.util.{ElaborationArtefacts,UIntIsOneOf}

import boom.exu.FUConstants

/**
 * Extension to BoomBundle to add a MicroOp
 */
abstract trait HasBoomUOP extends BoomBundle
{
  val uop = new MicroOp()
}

/**
 * MicroOp passing through the pipeline
 */
class MicroOp(implicit p: Parameters) extends BoomBundle
  with freechips.rocketchip.rocket.constants.MemoryOpConstants
  with freechips.rocketchip.rocket.constants.ScalarOpConstants
{
  val uopc             = UInt(MicroOpcodes.UOPC_SZ.W)       // micro-op code
  val inst             = UInt(32.W)
  val debug_inst       = UInt(32.W)
  val is_rvc           = Bool()
  val debug_pc         = UInt(coreMaxAddrBits.W)
  val iq_type          = UInt(IQT_SZ.W)        // which issue unit do we use?
  val fu_code          = UInt(FUConstants.FUC_SZ.W) // which functional unit do we use?
  val ctrl             = new CtrlSignals

  // What is the next state of this uop in the issue window? useful
  // for the compacting queue.
  val iw_state         = UInt(2.W)
  // Has operand 1 or 2 been waken speculatively by a load?
  // Only integer operands are speculaively woken up,
  // so we can ignore p3.
  val iw_p1_poisoned   = Bool()
  val iw_p2_poisoned   = Bool()

  val is_br            = Bool()                      // is this micro-op a (branch) vs a regular PC+4 inst?
  val is_jalr          = Bool()                      // is this a jump? (jal or jalr)
  val is_jal           = Bool()                      // is this a JAL (doesn't include JR)? used for branch unit
  val is_sfb           = Bool()                      // is this a sfb or in the shadow of a sfb

  val br_mask          = UInt(maxBrCount.W)  // which branches are we being speculated under?
  val br_tag           = UInt(brTagSz.W)

  //vconfig speculation info
  val is_vsetvli       = Bool()                      // is this a vsetvli?
  val is_vsetivli      = Bool()                      // is this a vsetivli?
  val vl_ready         = Bool()

  val vconfig_mask          = UInt(maxVconfigCount.W)  // which vconfig are we being speculated under?
  val vconfig_tag           = UInt(vconfigTagSz.W)
  val vcq_idx               = UInt(log2Ceil(vcqSz).W)

  // Index into FTQ to figure out our fetch PC.
  val ftq_idx          = UInt(log2Ceil(ftqSz).W)
  // This inst straddles two fetch packets
  val edge_inst        = Bool()
  // Low-order bits of our own PC. Combine with ftq[ftq_idx] to get PC.
  // Aligned to a cache-line size, as that is the greater fetch granularity.
  // TODO: Shouldn't this be aligned to fetch-width size?
  val pc_lob           = UInt(log2Ceil(icBlockBytes).W)

  // Was this a branch that was predicted taken?
  val taken            = Bool()

  val imm_packed       = UInt(LONGEST_IMM_SZ.W) // densely pack the imm in decode...
                                                // then translate and sign-extend in execute
  val csr_addr         = UInt(CSR_ADDR_SZ.W)    // only used for critical path reasons in Exe
  val rob_idx          = UInt(robAddrSz.W)
  val ldq_idx          = UInt(ldqAddrSz.W)
  val stq_idx          = UInt(stqAddrSz.W)
  val rxq_idx          = UInt(log2Ceil(numRxqEntries).W)
  val pdst             = UInt(maxPregSz.W)
  val prs1             = UInt(maxPregSz.W)
  val prs2             = UInt(maxPregSz.W)
  val prs3             = UInt(maxPregSz.W)
  val stale_pdst       = UInt(maxPregSz.W)
  val ppred            = UInt(log2Ceil(ftqSz).W)

  val prs1_busy        = Bool()
  val prs2_busy        = Bool()
  val prs3_busy        = Bool()
  val ppred_busy       = Bool()
  val exception        = Bool()
  val exc_cause        = UInt(xLen.W)          // TODO compress this down, xlen is insanity
  val bypassable       = Bool()                      // can we bypass ALU results? (doesn't include loads, csr, etc...)
  val mem_cmd          = UInt(M_SZ.W)          // sync primitives/cache flushes
  val mem_size         = UInt(2.W)
  val mem_signed       = Bool()
  val is_fence         = Bool()
  val is_fencei        = Bool()
  val is_amo           = Bool()
  val uses_ldq         = Bool()
  val uses_stq         = Bool()
  val is_sys_pc2epc    = Bool()                      // Is a ECall or Breakpoint -- both set EPC to PC.
  val is_unique        = Bool()                      // only allow this instruction in the pipeline, wait for STQ to
                                                     // drain, clear fetcha fter it (tell ROB to un-ready until empty)
  val flush_on_commit  = Bool()                      // some instructions need to flush the pipeline behind them

  // Preditation
  def is_sfb_br        = is_br && is_sfb && enableSFBOpt.B // Does this write a predicate
  def is_sfb_shadow    = !is_br && is_sfb && enableSFBOpt.B // Is this predicated
  val ldst_is_rs1      = Bool() // If this is set and we are predicated off, copy rs1 to dst,
                                // else copy rs2 to dst

  // logical specifiers (only used in Decode->Rename), except rollback (ldst)
  val ldst             = UInt(lregSz.W)
  val lrs1             = UInt(lregSz.W)
  val lrs2             = UInt(lregSz.W)
  val lrs3             = UInt(lregSz.W)

  val ldst_val         = Bool()              // is there a destination? invalid for stores, rd==x0, etc.
  val dst_rtype        = UInt(RT_X.getWidth.W)
  val lrs1_rtype       = UInt(RT_X.getWidth.W)
  val lrs2_rtype       = UInt(RT_X.getWidth.W)
  val frs3_en          = Bool()

  // floating point information
  val fp_val           = Bool()             // is a floating-point instruction (F- or D-extension)?
                                            // If it's non-ld/st it will write back exception bits to the fcsr.
  val fp_single        = Bool()             // single-precision floating point instruction (F-extension)

  // frontend exception information
  val xcpt_pf_if       = Bool()             // I-TLB page fault.
  val xcpt_ae_if       = Bool()             // I$ access exception.
  val xcpt_ma_if       = Bool()             // Misaligned fetch (jal/brjumping to misaligned addr).
  val bp_debug_if      = Bool()             // Breakpoint
  val bp_xcpt_if       = Bool()             // Breakpoint

  // vector extension
  val is_rvv           = if (usingVector) Bool() else false.B         // is vector instruction
  //val can_be_split     = if (usingVector) Bool() else false.B
  val pvd              = if (usingVector) Vec(8, Valid(UInt(vpregSz.W))) else null
  val stale_pvd        = if (usingVector) Vec(8, Valid(UInt(vpregSz.W))) else null
  val pvs1             = if (usingVector) Vec(8, Valid(UInt(vpregSz.W))) else null
  val pvs2             = if (usingVector) Vec(8, Valid(UInt(vpregSz.W))) else null
  val pvm              = if (usingVector) UInt(vpregSz.W) else UInt(0.W)
  val v_scalar_busy    = if (usingVector) Bool() else false.B
  val v_scalar_data    = if (usingVector) UInt(eLen.W) else UInt(0.W) // scalar value for vector pipe
  val v_active         = if (usingVector) Bool() else false.B
  val v_ls_ew          = if (usingVector) UInt(2.W) else UInt(0.W)    // EEW encoded in load/store instruction
  val v_unmasked       = if (usingVector) Bool() else false.B
  val v_eidx           = if (usingVector) UInt(vLenSz.W) else UInt(0.W) // element-index
  //val v_eofs           = if (usingVector) UInt(vLenSz.W) else UInt(0.W) // element-wise offset from first split
  val v_is_split       = if (usingVector) Bool() else false.B         // is a split of a vector instruction
  val v_split_ecnt     = if (usingVector) UInt((vLenSz+1).W) else UInt(0.W)
  val v_split_first    = if (usingVector) Bool() else false.B
  val v_split_last     = if (usingVector) Bool() else false.B
  val vs1_eew          = if (usingVector) UInt(2.W) else UInt(0.W)
  val vs2_eew          = if (usingVector) UInt(2.W) else UInt(0.W)
  val vd_eew           = if (usingVector) UInt(2.W) else UInt(0.W)
  val vs1_emul         = if (usingVector) UInt(3.W) else UInt(0.W)
  val vs2_emul         = if (usingVector) UInt(3.W) else UInt(0.W)
  val vd_emul          = if (usingVector) UInt(3.W) else UInt(0.W)
  val vxsat            = if (usingVector) Bool() else false.B         // saturating flag
  val vconfig          = if (usingVector) new VConfig else null       // TODO: tag since DECODE
  //val v_seg_ls         = if (usingVector) Bool() else false.B         // segment load/store indicator
  val v_seg_f          = if (usingVector) UInt(3.W) else UInt(0.W)    // field index, for segment load/store
  val v_seg_nf         = if (usingVector) UInt(4.W) else UInt(0.W)    // number of fields
  val v_idx_ls         = if (usingVector) Bool() else false.B         // indexed load/store indicator
  val v_xls_offset     = if (usingVector) UInt(eLen.W) else UInt(0.W) // address offset for indexed load/store
  val v_perm_busy      = if (usingVector) Bool() else false.B         // for vrgather/vslide/vcompress
  val v_perm_wait      = if (usingVector) Bool() else false.B         // wait vecUpdate
  val v_perm_idx       = if (usingVector) UInt(eLen.W) else UInt(0.W) // maximum VLMAX is 65536
  val vstart           = if (usingVector) UInt(vLenSz.W) else UInt(0.W) // VSTART CSR
  val vstartSrc        = if (usingVector) UInt(1.W) else false.B      // vstart source: CSR or speculative zero
  // purely debug information
  val debug_wdata      = UInt(xLen.W)
  val debug_events     = new DebugStageEvents

  // What prediction structure provides the prediction FROM this op
  val debug_fsrc       = UInt(BSRC_SZ.W)
  // What prediction structure provides the prediction TO this op
  val debug_tsrc       = UInt(BSRC_SZ.W)

  // Do we allocate a branch tag for this?
  // SFB branches don't get a mask, they get a predicate bit
  def allocate_brtag   = (is_br && !is_sfb) || is_jalr

  // Does this register write-back
  def rf_wen           = ldst_val && (dst_rtype =/= RT_X)

  // Is it possible for this uop to misspeculate, preventing the commit of subsequent uops?
  def unsafe           = uses_ldq || (uses_stq && !is_fence) || is_br || is_jalr

  def fu_code_is(_fu: UInt) = (fu_code & _fu) =/= 0.U

  def uses_scalar      = is_rvv && (iq_type & (IQT_INT | IQT_FP)) =/= 0.U
  def uses_v_simm5     = rt(RS1, isRvvSImm5)
  def uses_v_uimm5     = rt(RS1, isRvvUImm5)
  def is_perm_vadd     = rt(RS1, isPermVADD)
  def is_ureduce       = rt(RS1, isReduceV) && !rt(RD, isReduceV)
  def is_oreduce       = rt(RS1, isReduceV) && rt(RD, isReduceV)
  def is_reduce        = rt(RS1, isReduceV)
  def is_v_ls          = is_rvv && (uses_ldq || uses_stq)
  def uses_v_ls_ew     = uopc.isOneOf(MicroOpcodes.uopVL,
                                      MicroOpcodes.uopVLFF,
                                      MicroOpcodes.uopVSA,
                                      MicroOpcodes.uopVLS,
                                      MicroOpcodes.uopVSSA)
  def rt(rs: UInt, f: UInt => Bool): Bool = f(Mux1H(UIntToOH(rs), Seq(dst_rtype, lrs1_rtype, lrs2_rtype)))
  def is_vmv_s2v       = uopc.isOneOf(MicroOpcodes.uopVMV_V, MicroOpcodes.uopVFMV_V_F) && !rt(RS1, isVector)

  def match_group(prd: UInt): Bool = {
    val ret = Wire(Bool())
    ret := false.B
    if (usingVector) {
      ret := pvd.map(e => e.valid && e.bits === prd).reduce(_ || _)
    }
    ret
  }
}

/**
 * Control signals within a MicroOp
 *
 * TODO REFACTOR this, as this should no longer be true, as bypass occurs in stage before branch resolution
 */
class CtrlSignals(implicit p: Parameters) extends BoomBundle()
{
  val br_type     = UInt(BR_N.getWidth.W)
  val op1_sel     = UInt(OP1_X.getWidth.W)
  val op2_sel     = UInt(OP2_X.getWidth.W)
  val imm_sel     = UInt(IS_X.getWidth.W)
  val op_fcn      = UInt(freechips.rocketchip.rocket.ALU.SZ_ALU_FN.W)
  val fcn_dw      = Bool()
  val csr_cmd     = UInt(freechips.rocketchip.rocket.CSR.SZ.W)
  val is_load     = Bool()   // will invoke TLB address lookup
  val is_sta      = Bool()   // will invoke TLB address lookup
  val is_std      = Bool()
  val is_vmlogic  = if (usingVector) Bool() else null
  val is_vmscmp   = if (usingVector) Bool() else null
  //val is_vmfscmp   = if (usingVector) Bool() else null
}

/**
 * Debug stage events for Fetch stage
 */
class DebugStageEvents extends Bundle()
{
  // Track the sequence number of each instruction fetched.
  val fetch_seq        = UInt(32.W)
}

// Micro-op opcodes
object MicroOpcodes extends Enumeration {
  type MicroOpcodes = Value
  val UOPC_SZ       = 9


  val uopNOP_enum   = Value
  val uopNOP        = uopNOP_enum.id.U(UOPC_SZ.W)
  val uopLD_enum    = Value
  val uopLD         = uopLD_enum.id.U(UOPC_SZ.W)
  val uopSTA_enum   = Value
  val uopSTA        = uopSTA_enum.id.U(UOPC_SZ.W)  // store address generation
  val uopSTD_enum   = Value
  val uopSTD        = uopSTD_enum.id.U(UOPC_SZ.W)  // store data generation
  val uopLUI_enum   = Value
  val uopLUI        = uopLUI_enum.id.U(UOPC_SZ.W)

  val uopADDI_enum  = Value
  val uopADDI       = uopADDI_enum.id.U(UOPC_SZ.W)
  val uopANDI_enum  = Value
  val uopANDI       = uopANDI_enum.id.U(UOPC_SZ.W)
  val uopORI_enum   = Value
  val uopORI        = uopORI_enum.id.U(UOPC_SZ.W)
  val uopXORI_enum  = Value
  val uopXORI       = uopXORI_enum.id.U(UOPC_SZ.W)
  val uopSLTI_enum  = Value
  val uopSLTI       = uopSLTI_enum.id.U(UOPC_SZ.W)
  val uopSLTIU_enum = Value
  val uopSLTIU      = uopSLTIU_enum.id.U(UOPC_SZ.W)
  val uopSLLI_enum  = Value
  val uopSLLI       = uopSLLI_enum.id.U(UOPC_SZ.W)
  val uopSRAI_enum  = Value
  val uopSRAI       = uopSRAI_enum.id.U(UOPC_SZ.W)
  val uopSRLI_enum  = Value
  val uopSRLI       = uopSRLI_enum.id.U(UOPC_SZ.W)

  val uopSLL_enum   = Value
  val uopSLL        = uopSLL_enum.id.U(UOPC_SZ.W)
  val uopADD_enum   = Value
  val uopADD        = uopADD_enum.id.U(UOPC_SZ.W)
  val uopSUB_enum   = Value
  val uopSUB        = uopSUB_enum.id.U(UOPC_SZ.W)
  val uopSLT_enum   = Value
  val uopSLT        = uopSLT_enum.id.U(UOPC_SZ.W)
  val uopSLTU_enum  = Value
  val uopSLTU       = uopSLTU_enum.id.U(UOPC_SZ.W)
  val uopAND_enum   = Value
  val uopAND        = uopAND_enum.id.U(UOPC_SZ.W)
  val uopOR_enum    = Value
  val uopOR         = uopOR_enum.id.U(UOPC_SZ.W)
  val uopXOR_enum   = Value
  val uopXOR        = uopXOR_enum.id.U(UOPC_SZ.W)
  val uopSRA_enum   = Value
  val uopSRA        = uopSRA_enum.id.U(UOPC_SZ.W)
  val uopSRL_enum   = Value
  val uopSRL        = uopSRL_enum.id.U(UOPC_SZ.W)

  val uopBEQ_enum   = Value
  val uopBEQ        = uopBEQ_enum.id.U(UOPC_SZ.W)
  val uopBNE_enum   = Value
  val uopBNE        = uopBNE_enum.id.U(UOPC_SZ.W)
  val uopBGE_enum   = Value
  val uopBGE        = uopBGE_enum.id.U(UOPC_SZ.W)
  val uopBGEU_enum  = Value
  val uopBGEU       = uopBGEU_enum.id.U(UOPC_SZ.W)
  val uopBLT_enum   = Value
  val uopBLT        = uopBLT_enum.id.U(UOPC_SZ.W)
  val uopBLTU_enum  = Value
  val uopBLTU       = uopBLTU_enum.id.U(UOPC_SZ.W)
  val uopCSRRW_enum = Value
  val uopCSRRW      = uopCSRRW_enum.id.U(UOPC_SZ.W)
  val uopCSRRS_enum = Value
  val uopCSRRS      = uopCSRRS_enum.id.U(UOPC_SZ.W)
  val uopCSRRC_enum = Value
  val uopCSRRC      = uopCSRRC_enum.id.U(UOPC_SZ.W)
  val uopCSRRWI_enum= Value
  val uopCSRRWI     = uopCSRRWI_enum.id.U(UOPC_SZ.W)
  val uopCSRRSI_enum= Value
  val uopCSRRSI     = uopCSRRSI_enum.id.U(UOPC_SZ.W)
  val uopCSRRCI_enum= Value
  val uopCSRRCI     = uopCSRRCI_enum.id.U(UOPC_SZ.W)

  val uopJ_enum     = Value
  val uopJ          = uopJ_enum.id.U(UOPC_SZ.W)
  val uopJAL_enum   = Value
  val uopJAL        = uopJAL_enum.id.U(UOPC_SZ.W)
  val uopJALR_enum  = Value
  val uopJALR       = uopJALR_enum.id.U(UOPC_SZ.W)
  val uopAUIPC_enum = Value
  val uopAUIPC      = uopAUIPC_enum.id.U(UOPC_SZ.W)

//val uopSRET_enum  = Value
//val uopSRET       = uopSRET_enum.id.U(UOPC_SZ.W)
  val uopCFLSH_enum = Value
  val uopCFLSH      = uopCFLSH_enum.id.U(UOPC_SZ.W)
  val uopFENCE_enum = Value
  val uopFENCE      = uopFENCE_enum.id.U(UOPC_SZ.W)

  val uopADDIW_enum = Value
  val uopADDIW      = uopADDIW_enum.id.U(UOPC_SZ.W)
  val uopADDW_enum  = Value
  val uopADDW       = uopADDW_enum.id.U(UOPC_SZ.W)
  val uopSUBW_enum  = Value
  val uopSUBW       = uopSUBW_enum.id.U(UOPC_SZ.W)
  val uopSLLIW_enum = Value
  val uopSLLIW      = uopSLLIW_enum.id.U(UOPC_SZ.W)
  val uopSLLW_enum  = Value
  val uopSLLW       = uopSLLW_enum.id.U(UOPC_SZ.W)
  val uopSRAIW_enum = Value
  val uopSRAIW      = uopSRAIW_enum.id.U(UOPC_SZ.W)
  val uopSRAW_enum  = Value
  val uopSRAW       = uopSRAW_enum.id.U(UOPC_SZ.W)
  val uopSRLIW_enum = Value
  val uopSRLIW      = uopSRLIW_enum.id.U(UOPC_SZ.W)
  val uopSRLW_enum  = Value
  val uopSRLW       = uopSRLW_enum.id.U(UOPC_SZ.W)
  val uopMUL_enum   = Value
  val uopMUL        = uopMUL_enum.id.U(UOPC_SZ.W)
  val uopMULH_enum  = Value
  val uopMULH       = uopMULH_enum.id.U(UOPC_SZ.W)
  val uopMULHU_enum = Value
  val uopMULHU      = uopMULHU_enum.id.U(UOPC_SZ.W)
  val uopMULHSU_enum= Value
  val uopMULHSU     = uopMULHSU_enum.id.U(UOPC_SZ.W)
  val uopMULW_enum  = Value
  val uopMULW       = uopMULW_enum.id.U(UOPC_SZ.W)
  val uopDIV_enum   = Value
  val uopDIV        = uopDIV_enum.id.U(UOPC_SZ.W)
  val uopDIVU_enum  = Value
  val uopDIVU       = uopDIVU_enum.id.U(UOPC_SZ.W)
  val uopREM_enum   = Value
  val uopREM        = uopREM_enum.id.U(UOPC_SZ.W)
  val uopREMU_enum  = Value
  val uopREMU       = uopREMU_enum.id.U(UOPC_SZ.W)
  val uopDIVW_enum  = Value
  val uopDIVW       = uopDIVW_enum.id.U(UOPC_SZ.W)
  val uopDIVUW_enum = Value
  val uopDIVUW      = uopDIVUW_enum.id.U(UOPC_SZ.W)
  val uopREMW_enum  = Value
  val uopREMW       = uopREMW_enum.id.U(UOPC_SZ.W)
  val uopREMUW_enum = Value
  val uopREMUW      = uopREMUW_enum.id.U(UOPC_SZ.W)

  val uopFENCEI_enum= Value
  val uopFENCEI     = uopFENCEI_enum.id.U(UOPC_SZ.W)
  //                = Value
  val uopAMO_AG_enum= Value
  val uopAMO_AG     = uopAMO_AG_enum.id.U(UOPC_SZ.W) // AMO-address gen (use normal STD for datagen)

  val uopFMV_S_X_enum   = Value
  val uopFMV_S_X        = uopFMV_S_X_enum.id.U(UOPC_SZ.W)
  val uopFMV_D_X_enum   = Value
  val uopFMV_D_X        = uopFMV_D_X_enum.id.U(UOPC_SZ.W)
  val uopFMV_X_S_enum   = Value
  val uopFMV_X_S        = uopFMV_X_S_enum.id.U(UOPC_SZ.W)
  val uopFMV_X_D_enum   = Value
  val uopFMV_X_D        = uopFMV_X_D_enum.id.U(UOPC_SZ.W)

  val uopFSGNJ_S_enum   = Value
  val uopFSGNJ_S        = uopFSGNJ_S_enum.id.U(UOPC_SZ.W)
  val uopFSGNJ_D_enum   = Value
  val uopFSGNJ_D        = uopFSGNJ_D_enum.id.U(UOPC_SZ.W)

  val uopFCVT_S_D_enum  = Value
  val uopFCVT_S_D       = uopFCVT_S_D_enum.id.U(UOPC_SZ.W)
  val uopFCVT_D_S_enum  = Value
  val uopFCVT_D_S       = uopFCVT_D_S_enum.id.U(UOPC_SZ.W)
  val uopFCVT_S_X_enum  = Value
  val uopFCVT_S_X       = uopFCVT_S_X_enum.id.U(UOPC_SZ.W)
  val uopFCVT_D_X_enum  = Value
  val uopFCVT_D_X       = uopFCVT_D_X_enum.id.U(UOPC_SZ.W)
  val uopFCVT_X_S_enum  = Value
  val uopFCVT_X_S       = uopFCVT_X_S_enum.id.U(UOPC_SZ.W)
  val uopFCVT_X_D_enum  = Value
  val uopFCVT_X_D       = uopFCVT_X_D_enum.id.U(UOPC_SZ.W)

  val uopCMPR_S_enum    = Value
  val uopCMPR_S         = uopCMPR_S_enum.id.U(UOPC_SZ.W)
  val uopCMPR_D_enum    = Value
  val uopCMPR_D         = uopCMPR_D_enum.id.U(UOPC_SZ.W)

  val uopFCLASS_S_enum  = Value
  val uopFCLASS_S       = uopFCLASS_S_enum.id.U(UOPC_SZ.W)
  val uopFCLASS_D_enum  = Value
  val uopFCLASS_D       = uopFCLASS_D_enum.id.U(UOPC_SZ.W)

  val uopFMINMAX_S_enum = Value
  val uopFMINMAX_S      = uopFMINMAX_S_enum.id.U(UOPC_SZ.W)
  val uopFMINMAX_D_enum = Value
  val uopFMINMAX_D      = uopFMINMAX_D_enum.id.U(UOPC_SZ.W)

  val uopFADD_S_enum    = Value
  val uopFADD_S         = uopFADD_S_enum.id.U(UOPC_SZ.W)
  val uopFSUB_S_enum    = Value
  val uopFSUB_S         = uopFSUB_S_enum.id.U(UOPC_SZ.W)
  val uopFMUL_S_enum    = Value
  val uopFMUL_S         = uopFMUL_S_enum.id.U(UOPC_SZ.W)
  val uopFADD_D_enum    = Value
  val uopFADD_D         = uopFADD_D_enum.id.U(UOPC_SZ.W)
  val uopFSUB_D_enum    = Value
  val uopFSUB_D         = uopFSUB_D_enum.id.U(UOPC_SZ.W)
  val uopFMUL_D_enum    = Value
  val uopFMUL_D         = uopFMUL_D_enum.id.U(UOPC_SZ.W)

  val uopFMADD_S_enum   = Value
  val uopFMADD_S        = uopFMADD_S_enum.id.U(UOPC_SZ.W)
  val uopFMSUB_S_enum   = Value
  val uopFMSUB_S        = uopFMSUB_S_enum.id.U(UOPC_SZ.W)
  val uopFNMADD_S_enum  = Value
  val uopFNMADD_S       = uopFNMADD_S_enum.id.U(UOPC_SZ.W)
  val uopFNMSUB_S_enum  = Value
  val uopFNMSUB_S       = uopFNMSUB_S_enum.id.U(UOPC_SZ.W)
  val uopFMADD_D_enum   = Value
  val uopFMADD_D        = uopFMADD_D_enum.id.U(UOPC_SZ.W)
  val uopFMSUB_D_enum   = Value
  val uopFMSUB_D        = uopFMSUB_D_enum.id.U(UOPC_SZ.W)
  val uopFNMADD_D_enum  = Value
  val uopFNMADD_D       = uopFNMADD_D_enum.id.U(UOPC_SZ.W)
  val uopFNMSUB_D_enum  = Value
  val uopFNMSUB_D       = uopFNMSUB_D_enum.id.U(UOPC_SZ.W)

  val uopFDIV_S_enum    = Value
  val uopFDIV_S         = uopFDIV_S_enum.id.U(UOPC_SZ.W)
  val uopFDIV_D_enum    = Value
  val uopFDIV_D         = uopFDIV_D_enum.id.U(UOPC_SZ.W)
  val uopFSQRT_S_enum   = Value
  val uopFSQRT_S        = uopFSQRT_S_enum.id.U(UOPC_SZ.W)
  val uopFSQRT_D_enum   = Value
  val uopFSQRT_D        = uopFSQRT_D_enum.id.U(UOPC_SZ.W)

  //zfh-extension
  val uopFMV_H_X_enum   = Value
  val uopFMV_H_X        = uopFMV_H_X_enum.id.U(UOPC_SZ.W)
  val uopFMV_X_H_enum   = Value
  val uopFMV_X_H        = uopFMV_X_H_enum.id.U(UOPC_SZ.W)

  val uopFSGNJ_H_enum   = Value
  val uopFSGNJ_H        = uopFSGNJ_H_enum.id.U(UOPC_SZ.W)

  val uopFCVT_H_D_enum  = Value
  val uopFCVT_H_D       = uopFCVT_H_D_enum.id.U(UOPC_SZ.W)
  val uopFCVT_H_S_enum  = Value
  val uopFCVT_H_S       = uopFCVT_H_S_enum.id.U(UOPC_SZ.W)
  val uopFCVT_D_H_enum  = Value
  val uopFCVT_D_H       = uopFCVT_D_H_enum.id.U(UOPC_SZ.W)
  val uopFCVT_S_H_enum  = Value
  val uopFCVT_S_H       = uopFCVT_S_H_enum.id.U(UOPC_SZ.W)
  val uopFCVT_H_W_enum  = Value
  val uopFCVT_H_W       = uopFCVT_H_W_enum.id.U(UOPC_SZ.W)
  val uopFCVT_W_H_enum  = Value
  val uopFCVT_W_H      = uopFCVT_W_H_enum.id.U(UOPC_SZ.W)
  val uopFCVT_H_L_enum  = Value
  val uopFCVT_H_L       = uopFCVT_H_L_enum.id.U(UOPC_SZ.W)
  val uopFCVT_L_H_enum  = Value
  val uopFCVT_L_H      = uopFCVT_L_H_enum.id.U(UOPC_SZ.W)


  val uopCMPR_H_enum    = Value
  val uopCMPR_H         = uopCMPR_H_enum.id.U(UOPC_SZ.W)

  val uopFCLASS_H_enum  = Value
  val uopFCLASS_H       = uopFCLASS_H_enum.id.U(UOPC_SZ.W)

  val uopFMINMAX_H_enum = Value
  val uopFMINMAX_H      = uopFMINMAX_H_enum.id.U(UOPC_SZ.W)

  val uopFADD_H_enum    = Value
  val uopFADD_H         = uopFADD_H_enum.id.U(UOPC_SZ.W)
  val uopFSUB_H_enum    = Value
  val uopFSUB_H         = uopFSUB_H_enum.id.U(UOPC_SZ.W)
  val uopFMUL_H_enum    = Value
  val uopFMUL_H         = uopFMUL_H_enum.id.U(UOPC_SZ.W)

  val uopFMADD_H_enum   = Value
  val uopFMADD_H        = uopFMADD_H_enum.id.U(UOPC_SZ.W)
  val uopFMSUB_H_enum   = Value
  val uopFMSUB_H        = uopFMSUB_H_enum.id.U(UOPC_SZ.W)
  val uopFNMADD_H_enum  = Value
  val uopFNMADD_H       = uopFNMADD_H_enum.id.U(UOPC_SZ.W)
  val uopFNMSUB_H_enum  = Value
  val uopFNMSUB_H       = uopFNMSUB_H_enum.id.U(UOPC_SZ.W)

  val uopFDIV_H_enum    = Value
  val uopFDIV_H         = uopFDIV_H_enum.id.U(UOPC_SZ.W)
  val uopFSQRT_H_enum   = Value
  val uopFSQRT_H        = uopFSQRT_H_enum.id.U(UOPC_SZ.W)

  val uopWFI_enum       = Value
  val uopWFI            = uopWFI_enum.id.U(UOPC_SZ.W) // pass uop down the CSR pipeline
  val uopERET_enum      = Value
  val uopERET           = uopERET_enum.id.U(UOPC_SZ.W) // pass uop down the CSR pipeline, also is ERET
  val uopSFENCE_enum    = Value
  val uopSFENCE         = uopSFENCE_enum.id.U(UOPC_SZ.W)

  val uopROCC_enum      = Value
  val uopROCC           = uopROCC_enum.id.U(UOPC_SZ.W)

  val uopMOV_enum       = Value
  val uopMOV            = uopMOV_enum.id.U(UOPC_SZ.W) // conditional mov decoded from "add rd, x0, rs2"

  // RVV config
  val uopVSETVL_enum    = Value
  val uopVSETVL         = uopVSETVL_enum.id.U(UOPC_SZ.W)
  val uopVSETVLI_enum   = Value
  val uopVSETVLI        = uopVSETVLI_enum.id.U(UOPC_SZ.W)
  val uopVSETIVLI_enum  = Value
  val uopVSETIVLI       = uopVSETIVLI_enum.id.U(UOPC_SZ.W)
  // RVV load / store
  val uopVL_enum        = Value
  val uopVL             = uopVL_enum.id.U(UOPC_SZ.W)
  val uopVSA_enum       = Value
  val uopVSA            = uopVSA_enum.id.U(UOPC_SZ.W)
  val uopVLS_enum       = Value
  val uopVLS            = uopVLS_enum.id.U(UOPC_SZ.W)
  val uopVSSA_enum      = Value
  val uopVSSA           = uopVSSA_enum.id.U(UOPC_SZ.W)
  val uopVLUX_enum      = Value
  val uopVLUX           = uopVLUX_enum.id.U(UOPC_SZ.W)
  val uopVSUXA_enum     = Value
  val uopVSUXA          = uopVSUXA_enum.id.U(UOPC_SZ.W)
  val uopVLOX_enum      = Value
  val uopVLOX           = uopVLOX_enum.id.U(UOPC_SZ.W)
  val uopVSOXA_enum     = Value
  val uopVSOXA          = uopVSOXA_enum.id.U(UOPC_SZ.W)
  val uopVLM_enum       = Value
  val uopVLM            = uopVLM_enum.id.U(UOPC_SZ.W)
  val uopVSMA_enum      = Value
  val uopVSMA           = uopVSMA_enum.id.U(UOPC_SZ.W)
  val uopVLR_enum       = Value
  val uopVLR            = uopVLR_enum.id.U(UOPC_SZ.W) // sub to uopVL
  val uopVSR_enum       = Value
  val uopVSR            = uopVSR_enum.id.U(UOPC_SZ.W) // sub to uopVL
  val uopVLFF_enum      = Value
  val uopVLFF           = uopVLFF_enum.id.U(UOPC_SZ.W) // sub to uopVL
  val uopVAMO_enum      = Value
  val uopVAMO           = uopVAMO_enum.id.U(UOPC_SZ.W)
  // 12.1. single-width integer add/sub
  val uopVADD_enum      = Value
  val uopVADD           = uopVADD_enum.id.U(UOPC_SZ.W)
  // VADD_V*
  val uopVSUB_enum      = Value
  val uopVSUB           = uopVSUB_enum.id.U(UOPC_SZ.W)
  // VSUB_V*
  val uopVRSUB_enum     = Value
  val uopVRSUB          = uopVRSUB_enum.id.U(UOPC_SZ.W)
  // VRSUB_V*
  //  12.2. widening integer add/sub
//val uopVWADDU_enum    = Value
//val uopVWADDU         = uopVWADDU_enum.id.U(UOPC_SZ.W)
  // VWADDU_V*
//val uopVWSUBU_enum    = Value
//val uopVWSUBU         = uopVWSUBU_enum.id.U(UOPC_SZ.W)
  // VWSUBU_V*
//val uopVWADD_enum     = Value
//val uopVWADD          = uopVWADD_enum.id.U(UOPC_SZ.W)
  // VWADD_V*
//val uopVWSUB_enum     = Value
//val uopVWSUB          = uopVWSUB_enum.id.U(UOPC_SZ.W)
  // VWSUB_V*
  // 12.3. integer extension
  val uopVEXT2_enum    = Value
  val uopVEXT2         = uopVEXT2_enum.id.U(UOPC_SZ.W)
  val uopVEXT4_enum    = Value
  val uopVEXT4         = uopVEXT4_enum.id.U(UOPC_SZ.W)
  val uopVEXT8_enum    = Value
  val uopVEXT8         = uopVEXT8_enum.id.U(UOPC_SZ.W)
  // VZEXT_VF*
  // VSEXT_VF*
  // 12.4. integer add-with-carry/sub-with-borrow
  val uopVADC_enum      = Value
  val uopVADC           = uopVADC_enum.id.U(UOPC_SZ.W)
  // VADC_V*M
  val uopVMADC_enum     = Value
  val uopVMADC          = uopVMADC_enum.id.U(UOPC_SZ.W)
  // VMADC_V*
  val uopVSBC_enum      = Value
  val uopVSBC           = uopVSBC_enum.id.U(UOPC_SZ.W)
  // VSBC_V*M
  val uopVMSBC_enum     = Value
  val uopVMSBC          = uopVMSBC_enum.id.U(UOPC_SZ.W)
  // VMSBC_V*
  // 12.5. bitwise logical instrucitons
  val uopVAND_enum      = Value
  val uopVAND           = uopVAND_enum.id.U(UOPC_SZ.W)
  // VAND_V*
  val uopVOR_enum       = Value
  val uopVOR            = uopVOR_enum.id.U(UOPC_SZ.W)
  // VOR_V*
  val uopVXOR_enum      = Value
  val uopVXOR           = uopVXOR_enum.id.U(UOPC_SZ.W)
  // VXOR_V*
  // 12.6. single width bit shift
  val uopVSLL_enum      = Value
  val uopVSLL           = uopVSLL_enum.id.U(UOPC_SZ.W)
  // VSLL_V*
  val uopVSRL_enum      = Value
  val uopVSRL           = uopVSRL_enum.id.U(UOPC_SZ.W)
  // VSRL_V*
  val uopVSRA_enum      = Value
  val uopVSRA           = uopVSRA_enum.id.U(UOPC_SZ.W)
  // VSRA_V*
  // 12.7. narrowing integer right shift
  // VNSRL_W*
  // VNSRA_W*
  // 12.8. integer comparison
  val uopVMSEQ_enum     = Value
  val uopVMSEQ          = uopVMSEQ_enum.id.U(UOPC_SZ.W)
  // VMSEQ_V*
  val uopVMSNE_enum     = Value
  val uopVMSNE          = uopVMSNE_enum.id.U(UOPC_SZ.W)
  // VMSNE_V*
  val uopVMSLTU_enum    = Value
  val uopVMSLTU         = uopVMSLTU_enum.id.U(UOPC_SZ.W)
  // VMSLTU_V*
  val uopVMSLT_enum     = Value
  val uopVMSLT          = uopVMSLT_enum.id.U(UOPC_SZ.W)
  // VMSLT_V*
  val uopVMSLEU_enum    = Value
  val uopVMSLEU         = uopVMSLEU_enum.id.U(UOPC_SZ.W)
  // VMSLEU_V*
  val uopVMSLE_enum     = Value
  val uopVMSLE          = uopVMSLE_enum.id.U(UOPC_SZ.W)
  // VMSLE_V*
  val uopVMSGTU_enum    = Value
  val uopVMSGTU         = uopVMSGTU_enum.id.U(UOPC_SZ.W)
  // VMSGTU_V*
  val uopVMSGT_enum     = Value
  val uopVMSGT          = uopVMSGT_enum.id.U(UOPC_SZ.W)
  // VMSGT_V*
  // 12.9. integer min/max
  val uopVMINU_enum     = Value
  val uopVMINU          = uopVMINU_enum.id.U(UOPC_SZ.W)
  // VMINU_V*
  val uopVMIN_enum      = Value
  val uopVMIN           = uopVMIN_enum.id.U(UOPC_SZ.W)
  // VMIN_V*
  val uopVMAXU_enum     = Value
  val uopVMAXU          = uopVMAXU_enum.id.U(UOPC_SZ.W)
  // VMAXU_V*
  val uopVMAX_enum      = Value
  val uopVMAX           = uopVMAX_enum.id.U(UOPC_SZ.W)
  // VMAX_V*
  // 12.10. single-width integer multiply
  val uopVMUL_enum      = Value
  val uopVMUL           = uopVMUL_enum.id.U(UOPC_SZ.W)
  // VMUL_V*
  val uopVMULH_enum     = Value
  val uopVMULH          = uopVMULH_enum.id.U(UOPC_SZ.W)
  // VMULH_V*
  val uopVMULHU_enum    = Value
  val uopVMULHU         = uopVMULHU_enum.id.U(UOPC_SZ.W)
  // VMULHU_V*
  val uopVMULHSU_enum   = Value
  val uopVMULHSU        = uopVMULHSU_enum.id.U(UOPC_SZ.W)
  // VMULHSU_V*
  // 12.11. integer divide
  val uopVDIVU_enum     = Value
  val uopVDIVU          = uopVDIVU_enum.id.U(UOPC_SZ.W)
  // VDIVU_V*
  val uopVDIV_enum      = Value
  val uopVDIV           = uopVDIV_enum.id.U(UOPC_SZ.W)
  // VDIV_V*
  val uopVREMU_enum     = Value
  val uopVREMU          = uopVREMU_enum.id.U(UOPC_SZ.W)
  // VREMU_V*
  val uopVREM_enum      = Value
  val uopVREM           = uopVREM_enum.id.U(UOPC_SZ.W)
  // VREM_V*
  // 12.12. widening integer multiply
//val uopVWMUL_enum     = Value // reuse uopVMUL
//val uopVWMUL          = uopVWMUL_enum.id.U(UOPC_SZ.W)
  // VWMUL_V*
  val uopVWMULU_enum    = Value
  val uopVWMULU         = uopVWMULU_enum.id.U(UOPC_SZ.W)
  // VWMULU_V*
  val uopVWMULSU_enum   = Value
  val uopVWMULSU        = uopVWMULSU_enum.id.U(UOPC_SZ.W)
  // VWMULSU_V*
  // 12.13. single-width integer multiply-add
  val uopVMACC_enum     = Value
  val uopVMACC          = uopVMACC_enum.id.U(UOPC_SZ.W)
  // VMACC_V*
  val uopVNMSAC_enum    = Value
  val uopVNMSAC         = uopVNMSAC_enum.id.U(UOPC_SZ.W)
  // VNMSAC_V*
  val uopVMADD_enum     = Value
  val uopVMADD          = uopVMADD_enum.id.U(UOPC_SZ.W)
  // VMADD_V*
  val uopVNMSUB_enum    = Value
  val uopVNMSUB         = uopVNMSUB_enum.id.U(UOPC_SZ.W)
  // VNMSUB_V*
  // 12.14. widening integer multiply-add
  val uopVWMACCU_enum   = Value
  val uopVWMACCU        = uopVWMACCU_enum.id.U(UOPC_SZ.W)
  // VWMACCU_V*
//val uopVWMACC_enum    = Value // reuse uopVMACC
//val uopVWMACC         = uopVWMACC_enum.id.U(UOPC_SZ.W)
  // VWMACC_V*
  val uopVWMACCSU_enum  = Value
  val uopVWMACCSU       = uopVWMACCSU_enum.id.U(UOPC_SZ.W)
  // VWMACCSU_V*
  val uopVWMACCUS_enum  = Value
  val uopVWMACCUS       = uopVWMACCUS_enum.id.U(UOPC_SZ.W)
  // VWMACCUS_VX
  // 12.15. integer merge
  val uopMERGE_enum     = Value
  val uopMERGE          = uopMERGE_enum.id.U(UOPC_SZ.W)
  // VMERGE_V*M
  // 12.16. integer move
  val uopVMV_V_enum     = Value
  val uopVMV_V          = uopVMV_V_enum.id.U(UOPC_SZ.W)
  // VMV_V_*
  // 13.1. single-width saturating add/sub
  val uopVSADDU_enum    = Value
  val uopVSADDU         = uopVSADDU_enum.id.U(UOPC_SZ.W)
  // VSADDU_V*
  val uopVSADD_enum     = Value
  val uopVSADD          = uopVSADD_enum.id.U(UOPC_SZ.W)
  // VSADD_V*
  val uopVSSUBU_enum    = Value
  val uopVSSUBU         = uopVSSUBU_enum.id.U(UOPC_SZ.W)
  // VSSUBU_V*
  val uopVSSUB_enum     = Value
  val uopVSSUB          = uopVSSUB_enum.id.U(UOPC_SZ.W)
  // VSSUB_V*
  // 13.2. single-width averaging add/sub
  val uopVAADDU_enum    = Value
  val uopVAADDU         = uopVAADDU_enum.id.U(UOPC_SZ.W)
  // VAADDU_V*
  val uopVAADD_enum     = Value
  val uopVAADD          = uopVAADD_enum.id.U(UOPC_SZ.W)
  // VAADD_V*
  val uopVASUBU_enum    = Value
  val uopVASUBU         = uopVASUBU_enum.id.U(UOPC_SZ.W)
  // VASUBU_V*
  val uopVASUB_enum     = Value
  val uopVASUB          = uopVASUB_enum.id.U(UOPC_SZ.W)
  // VASUB_V*
  // 13.3. single-width fractional multiply with rounding and saturation
  val uopVSMUL_enum     = Value
  val uopVSMUL          = uopVSMUL_enum.id.U(UOPC_SZ.W)
  // VSMUL_V*
  // 13.4. single-width scaling shift instruction
  val uopVSSRL_enum     = Value
  val uopVSSRL          = uopVSSRL_enum.id.U(UOPC_SZ.W)
  // VSSRL_V*
  val uopVSSRA_enum     = Value
  val uopVSSRA          = uopVSSRA_enum.id.U(UOPC_SZ.W)
  // VSSRA_V*
  // 13.5. narrowing fixed-point clip
  val uopVNCLIPU_enum   = Value
  val uopVNCLIPU        = uopVNCLIPU_enum.id.U(UOPC_SZ.W)
  // VNCLIPU_W*
  val uopVNCLIP_enum    = Value
  val uopVNCLIP         = uopVNCLIP_enum.id.U(UOPC_SZ.W)
  // VNCLIP_W*
  // 15.1. single-width INT reduction, reuse corresponding uopc
//val uopVRED_enum      = Value
//val uopVRED           = uopVRED_enum.id.U(UOPC_SZ.W)
  // VREDSUM_VS
  // VREDMAXU_VS
  // VREDMAX_VS
  // VREDMINU_VS
  // VREDMIN_VS
  // VREDAND_VS
  // VREDOR_VS
  // VREDXOR_VS
  // 15.2. widening INT reduction
  // VWREDSUMU_VS
  // VWREDSUM_VS
  // 16.1. mask logical
  // val uopVMLOGIC_enum   = Value
  // val uopVMLOGIC        = uopVMLOGIC_enum.id.U(UOPC_SZ.W)
  // 16.1. vector mask-register logical instructions
  val uopVMAND_enum     = Value
  val uopVMAND          = uopVMAND_enum.id.U(UOPC_SZ.W)
  val uopVMNAND_enum    = Value
  val uopVMNAND         = uopVMNAND_enum.id.U(UOPC_SZ.W)
  val uopVMANDNOT_enum  = Value
  val uopVMANDNOT       = uopVMANDNOT_enum.id.U(UOPC_SZ.W)
  val uopVMXOR_enum     = Value
  val uopVMXOR          = uopVMXOR_enum.id.U(UOPC_SZ.W)
  val uopVMOR_enum      = Value
  val uopVMOR           = uopVMOR_enum.id.U(UOPC_SZ.W)
  val uopVMNOR_enum     = Value
  val uopVMNOR          = uopVMNOR_enum.id.U(UOPC_SZ.W)
  val uopVMORNOT_enum   = Value
  val uopVMORNOT        = uopVMORNOT_enum.id.U(UOPC_SZ.W)
  val uopVMXNOR_enum     = Value
  val uopVMXNOR          = uopVMXNOR_enum.id.U(UOPC_SZ.W)
  // 16.2. mask popc
  val uopVPOPC_enum     = Value
  val uopVPOPC          = uopVPOPC_enum.id.U(UOPC_SZ.W)
  // VPOPC_M
  // 16.3. find first set
  val uopVFIRST_enum    = Value
  val uopVFIRST         = uopVFIRST_enum.id.U(UOPC_SZ.W)
  // VFIRST_M
  // 16.4. set before first
  val uopVMSBF_enum     = Value
  val uopVMSBF          = uopVMSBF_enum.id.U(UOPC_SZ.W)
  // VMSBF_M
  // 16.5. set including first
  val uopVMSIF_enum     = Value
  val uopVMSIF          = uopVMSIF_enum.id.U(UOPC_SZ.W)
  // VMSIF_M
  // 16.6. set only first
  val uopVMSOF_enum     = Value
  val uopVMSOF          = uopVMSOF_enum.id.U(UOPC_SZ.W)
  // VMSOF_M
  // 16.8. iota
  val uopVIOTA_enum     = Value
  val uopVIOTA          = uopVIOTA_enum.id.U(UOPC_SZ.W)
  // VIOTA_M
  // 16.9. element index
  val uopVID_enum       = Value
  val uopVID            = uopVID_enum.id.U(UOPC_SZ.W)
  // VID_V
  // 17.1. INT scalar move
  val uopVMV_X_S_enum   = Value
  val uopVMV_X_S        = uopVMV_X_S_enum.id.U(UOPC_SZ.W)
  val uopVMV_S_X_enum   = Value
  val uopVMV_S_X        = uopVMV_S_X_enum.id.U(UOPC_SZ.W)
  // 17.3. slide
  val uopVSLIDEUP_enum  = Value
  val uopVSLIDEUP       = uopVSLIDEUP_enum.id.U(UOPC_SZ.W)
  // VSLIDEUP_V*
  val uopVSLIDEDOWN_enum= Value
  val uopVSLIDEDOWN     = uopVSLIDEDOWN_enum.id.U(UOPC_SZ.W)
  // VSLIDEDOWN_V*
  val uopVSLIDE1UP_enum = Value
  val uopVSLIDE1UP      = uopVSLIDE1UP_enum.id.U(UOPC_SZ.W)
  // VSLIDE1UP_V*
  val uopVSLIDE1DOWN_enum= Value
  val uopVSLIDE1DOWN    = uopVSLIDE1DOWN_enum.id.U(UOPC_SZ.W)
  // VSLIDE1DOWN_V*
  // 17.4. register gather
  val uopVRGATHER_enum  = Value
  val uopVRGATHER       = uopVRGATHER_enum.id.U(UOPC_SZ.W)
  // VRGATHER_V*
  val uopVRGATHEREI16_enum = Value
  val uopVRGATHEREI16   = uopVRGATHEREI16_enum.id.U(UOPC_SZ.W)
  // VRGATHEREI16_VV
  // 17.5. compress
  val uopVCOMPRESS_enum = Value
  val uopVCOMPRESS      = uopVCOMPRESS_enum.id.U(UOPC_SZ.W)
  // VCOMPRESS_VM
  // 17.6. whole reg move
  val uopVMVR_enum      = Value
  val uopVMVR           = uopVMVR_enum.id.U(UOPC_SZ.W)
  // VMV*R_V

  // 14.2. single-width floating-point add/sub
  val uopVFADD_enum     = Value
  val uopVFADD          = uopVFADD_enum.id.U(UOPC_SZ.W)
  // VFADD_V*
  val uopVFSUB_enum     = Value
  val uopVFSUB          = uopVFSUB_enum.id.U(UOPC_SZ.W)
  // VFSUB_V*
  val uopVFRSUB_enum    = Value
  val uopVFRSUB         = uopVFRSUB_enum.id.U(UOPC_SZ.W)
  // 14.3. widening floating-point add/sub
  // VFWADD_V*
  // VFWSUB_V*
  // VFWADD_W*
  // VFWSUB_V*
  // 14.4. single-width FP mul/div
  val uopVFMUL_enum     = Value
  val uopVFMUL          = uopVFMUL_enum.id.U(UOPC_SZ.W)
  // VFMUL_V*
  val uopVFDIV_enum     = Value
  val uopVFDIV          = uopVFDIV_enum.id.U(UOPC_SZ.W)
  // VFDIV_V*
  val uopVFRDIV_enum    = Value
  val uopVFRDIV         = uopVFRDIV_enum.id.U(UOPC_SZ.W)
  // VFRDIV_VF
  // 14.5. widening FP mul
  //val uopVFWMUL_V*
  // 14.6 single-width FP fused mul-add
  // VFMACC_V*
  val uopVFMACC_enum    = Value
  val uopVFMACC         = uopVFMACC_enum.id.U(UOPC_SZ.W)
  // VFNMACC_V*
  val uopVFNMACC_enum   = Value
  val uopVFNMACC        = uopVFNMACC_enum.id.U(UOPC_SZ.W)
  // VFMSAC_V*
  val uopVFMSAC_enum    = Value
  val uopVFMSAC         = uopVFMSAC_enum.id.U(UOPC_SZ.W)
  // VFNMSAC_V*
  val uopVFNMSAC_enum   = Value
  val uopVFNMSAC        = uopVFNMSAC_enum.id.U(UOPC_SZ.W)
  // VFMADD_V*
  val uopVFMADD_enum    = Value
  val uopVFMADD         = uopVFMADD_enum.id.U(UOPC_SZ.W)
  // VFNMADD_V*
  val uopVFNMADD_enum   = Value
  val uopVFNMADD        = uopVFNMADD_enum.id.U(UOPC_SZ.W)
  // VFMSUB_V*
  val uopVFMSUB_enum    = Value
  val uopVFMSUB         = uopVFMSUB_enum.id.U(UOPC_SZ.W)
  // VFNMSUB_V*
  val uopVFNMSUB_enum   = Value
  val uopVFNMSUB        = uopVFNMSUB_enum.id.U(UOPC_SZ.W)
  // 14.7. widening FP fused mul-add
  // VFWMACC_V*
  // VFWNMACC_V*
  // VFWMSAC_V*
  // VFWNMSAC_V*
  // 14.8. FP sqrt
  val uopVFSQRT_enum    = Value
  val uopVFSQRT         = uopVFSQRT_enum.id.U(UOPC_SZ.W)
  // VFSQRT_V
  // 14.9. FP reciprocal sqrt estimate to 7 bits
  val uopVFRSQRT7_enum   = Value
  val uopVFRSQRT7        = uopVFRSQRT7_enum.id.U(UOPC_SZ.W)
  // VFRSQRT7_V
  // 14.10. FP reciprocal estimate
  val uopVFREC7_enum    = Value
  val uopVFREC7         = uopVFREC7_enum.id.U(UOPC_SZ.W)
  // VFREC7_V
  // 14.11. FP min/max
  val uopVFMIN_enum     = Value
  val uopVFMIN          = uopVFMIN_enum.id.U(UOPC_SZ.W)
  // VFMIN_V*
  val uopVFMAX_enum     = Value
  val uopVFMAX          = uopVFMAX_enum.id.U(UOPC_SZ.W)
  // VFMAX_V*
  // 14.12. FP sign-injection
  val uopVFSGNJ_enum    = Value
  val uopVFSGNJ         = uopVFSGNJ_enum.id.U(UOPC_SZ.W)
  // VFSGNJ_V*
  val uopVFSGNJN_enum   = Value
  val uopVFSGNJN        = uopVFSGNJN_enum.id.U(UOPC_SZ.W)
  // VFSGNJN_V*
  val uopVFSGNJX_enum   = Value
  val uopVFSGNJX        = uopVFSGNJX_enum.id.U(UOPC_SZ.W)
  // VFSGNJX_V*
  // 14.13. FP compare
  val uopVMFEQ_enum     = Value
  val uopVMFEQ          = uopVMFEQ_enum.id.U(UOPC_SZ.W)
  // VMFEQ_V*
  val uopVMFNE_enum     = Value
  val uopVMFNE          = uopVMFNE_enum.id.U(UOPC_SZ.W)
  // VMFNE_V*
  val uopVMFLT_enum     = Value
  val uopVMFLT          = uopVMFLT_enum.id.U(UOPC_SZ.W)
  // VMFLT_V*
  val uopVMFLE_enum     = Value
  val uopVMFLE          = uopVMFLE_enum.id.U(UOPC_SZ.W)
  // VMFLE_V*
  val uopVMFGT_enum     = Value
  val uopVMFGT          = uopVMFGT_enum.id.U(UOPC_SZ.W)
  // VMFGT_VF
  val uopVMFGE_enum     = Value
  val uopVMFGE          = uopVMFGE_enum.id.U(UOPC_SZ.W)
  // VMFGE_VF
  // 14.14. FP classify
  val uopVFCLASS_enum   = Value
  val uopVFCLASS        = uopVFCLASS_enum.id.U(UOPC_SZ.W)
  // 14.15. FP merge, share with uopVMERGE
  //val uopVFMERGE_enum   = Value
  //val uopVFMERGE        = uopVFMERGE_enum.id.U(UOPC_SZ.W)
  // VFMERGE_VFM
  // 14.16. FP move
  val uopVFMV_V_F_enum  = Value
  val uopVFMV_V_F       = uopVFMV_V_F_enum.id.U(UOPC_SZ.W)
  // VFMV_V_F
  // 14.17. Single-Width FP integer type-convert
  val uopVFCVT_F2I_enum = Value
  val uopVFCVT_F2I      = uopVFCVT_F2I_enum.id.U(UOPC_SZ.W)
  val uopVFCVT_I2F_enum = Value
  val uopVFCVT_I2F      = uopVFCVT_I2F_enum.id.U(UOPC_SZ.W)
  val uopVFCVT_F2F_enum = Value
  val uopVFCVT_F2F      = uopVFCVT_F2F_enum.id.U(UOPC_SZ.W)
  // VFCVT_XU_F_V
  // VFCVT_X_F_V
  // VFCVT_RTZ_XU_F_V
  // VFCVT_RTZ_X_F_V
  // VFCVT_F_XU_V
  // VFCVT_F_X_D
  // 14.18. Widening FP/Int type-convert
  // VFWCVT_XU_F_V
  // VFWCVT_X_F_V
  // VFWCVT_RTZ_XU_F_V
  // VFWCVT_RTZ_X_F_V
  // VFWCVT_F_XU_V
  // VFWCVT_F_X_V
  // VFWCVT_F_F_V
  // 14.19. narrowing FP/Int type-convert
  // VFNCVT_XU_F_W
  // VFNCVT_X_F_W
  // VFNCVT_RTZ_XU_F_W
  // VFNCVT_RTZ_X_F_W
  // VFNCVT_F_XU_W
  // VFNCVT_F_X_W
  // VFNCVT_F_F_W
  // VFNCVT_ROD_F_F_W
  // 15.3. single-width FP reduction, reuse corresponding uopc
//val uopVFRED_enum     = Value
//val uopVFRED          = uopVFRED_enum.id.U(UOPC_SZ.W)
  // VFREDOSUM_VS
  // VFREDSUM_VS
  // VFREDMAX_VS
  // VFREDMIN_VS
  // 15.4. widening FP reduction
  // VFWREDOSUM_VS
  // VFWREDSUM_VS
  // 17.2. FP salar move
  val uopVFMV_F_S_enum   = Value
  val uopVFMV_F_S        = uopVFMV_F_S_enum.id.U(UOPC_SZ.W)
  val uopVFMV_S_F_enum   = Value
  val uopVFMV_S_F        = uopVFMV_S_F_enum.id.U(UOPC_SZ.W)
  // 17.3. slide
  val uopVFSLIDE1UP_enum= Value
  val uopVFSLIDE1UP     = uopVFSLIDE1UP_enum.id.U(UOPC_SZ.W)
  val uopVFSLIDE1DOWN_enum = Value
  val uopVFSLIDE1DOWN   = uopVFSLIDE1DOWN_enum.id.U(UOPC_SZ.W)
  require(log2Ceil(maxId) <= UOPC_SZ)

  def alias(v: Value): String = {
    val enum = raw"uop(\w+)_enum".r
    val op = enum.findFirstMatchIn(v.toString).get.group(1)
    val ret = s"${op}\t\t${v.id}\t\tNULL"
    ret
  }

  def gen_aliases(): String = {
    val all = values.map(alias(_)).mkString("\n")
    val ret = s"alias uopc\n$all\nendalias"
    ret
  }

  // generate aliases of uop encodings for verdi
  ElaborationArtefacts.add("uop.alias", gen_aliases)
}

