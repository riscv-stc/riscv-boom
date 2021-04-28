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
import freechips.rocketchip.util.{ElaborationArtefacts}

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
  val ppred            = UInt(log2Ceil(ftqSz).W)

  val prs1_busy        = if (usingVector) UInt(vLenb.W) else Bool()
  val prs2_busy        = if (usingVector) UInt(vLenb.W) else Bool()
  val prs3_busy        = if (usingVector) UInt(vLenb.W) else Bool()
  val ppred_busy       = Bool()
  val stale_pdst       = UInt(maxPregSz.W)
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
  val dst_rtype        = UInt(2.W)
  val lrs1_rtype       = UInt(2.W)
  val lrs2_rtype       = UInt(2.W)
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
  val is_rvv           = Bool()             // is vector instruction
  val v_ls_ew          = UInt(2.W)          // EEW encoded in load/store instruction
  val v_unmasked       = Bool()
  val vstart           = UInt(vLenSz.W)
  val v_is_split       = Bool()             // is a split of a vector instruction
  val v_split_ecnt     = UInt((log2Ceil(vLenb)+1).W)
  val v_is_first       = Bool()
  val v_is_last        = Bool()
  val v_re_alloc       = Bool()             // do rename allocation on first split for every vreg
  val vxsat            = Bool()             // saturating flag
  val vconfig          = new VConfig        // TODO: tag since DECODE
//val pvm              = UInt(maxPregSz.W)  // FIXME:vm0 phiscal name for masked
//val pvm_busy         = Bool()
//val pvd_grpnum       = UInt(3.W)          // store mappings in ROB for rollback
//val pvs1_grpnum      = UInt(3.W)
//val pvs2_grpnum      = UInt(3.W)
//val pvs3_grpnum      = UInt(3.W)
//val pvd_grpmap       = Vec(7,UInt(maxPregSz.W))
//val pvs1_grpmap      = Vec(7,UInt(maxPregSz.W))
//val pvs2_grpmap      = Vec(7,UInt(maxPregSz.W))
//val pvs3_grpmap      = Vec(7,UInt(maxPregSz.W))

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
  def rf_wen           = dst_rtype =/= RT_X

  // Is it possible for this uop to misspeculate, preventing the commit of subsequent uops?
  def unsafe           = uses_ldq || (uses_stq && !is_fence) || is_br || is_jalr

  def fu_code_is(_fu: UInt) = (fu_code & _fu) =/= 0.U
}

/**
 * Control signals within a MicroOp
 *
 * TODO REFACTOR this, as this should no longer be true, as bypass occurs in stage before branch resolution
 */
class CtrlSignals extends Bundle()
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
  val UOPC_SZ       = 8

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
//val uopVLR_enum       = Value
//val uopVLR            = uopVLR_enum.id.U(UOPC_SZ.W) // sub to uopVL
//val uopVSR_enum       = Value
//val uopVSR            = uopVSR_enum.id.U(UOPC_SZ.W) // sub to uopVL
//val uopVLFF_enum      = Value
//val uopVLFF           = uopVLFF_enum.id.U(UOPC_SZ.W) // sub to uopVL
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
  // VWADDU_V*
  // VWSUBU_V*
  // VWADD_V*
  // VWSUB_V*
  // 12.3. integer extension
  val uopVSEXT_enum     = Value
  val uopVSEXT          = uopVSEXT_enum.id.U(UOPC_SZ.W)
  // VZEXT_V*
  // VSEXT_V*
  // 12.4. integer add-with-carry/sub-with-borrow
  val uopVADC_enum      = Value
  val uopVADC           = uopVADC_enum.id.U(UOPC_SZ.W)
  // VADC_V*
  val uopVMADC_enum     = Value
  val uopVMADC          = uopVMADC_enum.id.U(UOPC_SZ.W)
  // VMADC_V*
  val uopVSBC_enum      = Value
  val uopVSBC           = uopVSBC_enum.id.U(UOPC_SZ.W)
  // VSBC_V*
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
  val uopVSHL_enum      = Value
  val uopVSHL           = uopVSHL_enum.id.U(UOPC_SZ.W)
  // VSLL_V*
  val uopVSHR_enum      = Value
  val uopVSHR           = uopVSHR_enum.id.U(UOPC_SZ.W)
  // VSRL_V*
  // VSRA_V*
  // 12.7. narrowing integer right shift
  // VNSRL_W*
  // VNSRA_W*
  // 12.8. integer comparison
  val uopVMSCMP_enum    = Value
  val uopVMSCMP         = uopVMSCMP_enum.id.U(UOPC_SZ.W)
  // VMSEQ_V*
  // VMSNE_V*
  // VMSLTU_V*
  // VMSLT_V*
  // VMSLEU_V*
  // VMSLE_V*
  // VMSGTU_V*
  // VMSGT_V*
  // 12.9. integer min/max
  val uopVMIN_enum      = Value
  val uopVMIN           = uopVMIN_enum.id.U(UOPC_SZ.W)
  // VMINU_V*
  // VMIN_V*
  val uopVMAX_enum      = Value
  val uopVMAX           = uopVMAX_enum.id.U(UOPC_SZ.W)
  // VMAXU_V*
  // VMAX_V*
  // 12.10. single-width integer multiply
  val uopVMUL_enum      = Value
  val uopVMUL           = uopVMUL_enum.id.U(UOPC_SZ.W)
  // VMUL_V*
  // VMULH_V*
  // VMULHU_V*
  // VMULHSU_V*
  // 12.11. integer divide
  val uopVDIV_enum      = Value
  val uopVDIV           = uopVDIV_enum.id.U(UOPC_SZ.W)
  // VDIVU_V*
  // VDIV_V*
//val uopVREM_enum      = Value
//val uopVREM           = uopVREM_enum.id.U(UOPC_SZ.W)
  // VREMU_V*
  // VREM_V*
  // 12.12. widening integer multiply
  // VWMUL_V*
  // VWMULU_V*
  // VWMULSU_V*
  // 12.13. single-width integer multiply-add
  val uopVMACC_enum     = Value
  val uopVMACC          = uopVMACC_enum.id.U(UOPC_SZ.W)
  // VMACC_V*
  // VNMSAC_V*
  // VMADD_V*
  // VNMSUB_V*
  // 12.14. widening integer multiply-add
  // VWMACCU_V*
  // VWMACC_V*
  // VWMACCSU_V*
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
  val uopVSADD_enum     = Value
  val uopVSADD          = uopVSADD_enum.id.U(UOPC_SZ.W)
  // VSADDU_V*
  // VSADD_V*
  val uopVSSUB_enum     = Value
  val uopVSSUB          = uopVSSUB_enum.id.U(UOPC_SZ.W)
  // VSSUBU_V*
  // VSSUB_V*
  // 13.2. single-width averaging add/sub
  val uopVAADD_enum     = Value
  val uopVAADD          = uopVAADD_enum.id.U(UOPC_SZ.W)
  // VAADDU_V*
  // VAADD_V*
  val uopVASUB_enum     = Value
  val uopVASUB          = uopVASUB_enum.id.U(UOPC_SZ.W)
  // VASUBU_V*
  // VASUB_V*
  // 13.3. single-width fractional multiply with rounding and saturation
  val uopVSMUL_enum     = Value
  val uopVSMUL          = uopVSMUL_enum.id.U(UOPC_SZ.W)
  // VSMUL_V*
  // 13.4. single-width scaling shift instruction
  val uopVSSR_enum      = Value
  val uopVSSR           = uopVSSR_enum.id.U(UOPC_SZ.W)
  // VSSRL_V*
  // VSSRA_V*
  // 13.5. narrowing fixed-point clip
  val uopVNCLIP_enum    = Value
  val uopVNCLIP         = uopVNCLIP_enum.id.U(UOPC_SZ.W)
  // VNCLIPU_W*
  // VNCLIP_W*
  // 15.1. single-width INT reduction
  val uopVRED_enum      = Value
  val uopVRED           = uopVRED_enum.id.U(UOPC_SZ.W)
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
  val uopVMLOGIC_enum   = Value
  val uopVMLOGIC        = uopVMLOGIC_enum.id.U(UOPC_SZ.W)
  // VMAND_MM
  // VMNAND_MM
  // VMANDNOT_MM
  // VMXOR_MM
  // VMOR_MM
  // VMNOR_MM
  // VMORNOT_MM
  // VMXNOR_MM
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
  // VFRSUB_VF
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
  val uopVFMACC_enum    = Value
  val uopVFMACC         = uopVFMACC_enum.id.U(UOPC_SZ.W)
  // VFMACC_V*
  // VFNMACC_V*
  // VFMSAC_V*
  // VFNMSAC_V*
  // VFMADD_V*
  // VFNMADD_V*
  // VFMSUB_V*
  // VFNMSUB_V*
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
  val uopVFSQRT7_enum   = Value
  val uopVFSQRT7        = uopVFSQRT7_enum.id.U(UOPC_SZ.W)
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
  // VFSGNJN_V*
  // VFSGNJX_V*
  // 14.13. FP compare
  val uopVMFCMP_I2F_enum= Value
  val uopVMFCMP_I2F     = uopVMFCMP_I2F_enum.id.U(UOPC_SZ.W)
  val uopVMFCMP_F2I_enum= Value
  val uopVMFCMP_F2I     = uopVMFCMP_F2I_enum.id.U(UOPC_SZ.W)
  val uopVMFCMP_F2F_enum= Value
  val uopVMFCMP_F2F     = uopVMFCMP_F2F_enum.id.U(UOPC_SZ.W)
  // VMFEQ_V*
  // VMFNE_V*
  // VMFLT_V*
  // VMFLE_V*
  // VMFGT_VF
  // VMFGE_VF
  // 14.14. FP classify
  val uopVFCLASS_enum   = Value
  val uopVFCLASS        = uopVFCLASS_enum.id.U(UOPC_SZ.W)
  // 14.15. FP merge
  val uopVFMERGE_enum   = Value
  val uopVFMERGE        = uopVFMERGE_enum.id.U(UOPC_SZ.W)
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
  // 15.3. single-width FP reduction
  val uopVFRED_enum     = Value
  val uopVFRED          = uopVFRED_enum.id.U(UOPC_SZ.W)
  // VFREDOSUM_VS
  // VFREDSUM_VS
  // VFREDMAX_VS
  // VFREDMIN_VS
  // 15.4. widening FP reduction
  // VFWREDOSUM_VS
  // VFWREDSUM_VS
  // 17.2. FP salar move
  val uopVMV_F_S_enum   = Value
  val uopVMV_F_S        = uopVMV_F_S_enum.id.U(UOPC_SZ.W)
  val uopVMV_S_F_enum   = Value
  val uopVMV_S_F        = uopVMV_S_F_enum.id.U(UOPC_SZ.W)
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

