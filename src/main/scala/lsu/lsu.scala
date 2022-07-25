//******************************************************************************
// Copyright (c) 2012 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// RISCV Out-of-Order Load/Store Unit
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// Load/Store Unit is made up of the Load-Address Queue, the Store-Address
// Queue, and the Store-Data queue (LAQ, SAQ, and SDQ).
//
// Stores are sent to memory at (well, after) commit, loads are executed
// optimstically ASAP.  If a misspeculation was discovered, the pipeline is
// cleared. Loads put to sleep are retried.  If a LoadAddr and StoreAddr match,
// the Load can receive its data by forwarding data out of the Store-Data
// Queue.
//
// Currently, loads are sent to memory immediately, and in parallel do an
// associative search of the SAQ, on entering the LSU. If a hit on the SAQ
// search, the memory request is killed on the next cycle, and if the SDQ entry
// is valid, the store data is forwarded to the load (delayed to match the
// load-use delay to delay with the write-port structural hazard). If the store
// data is not present, or it's only a partial match (SB->LH), the load is put
// to sleep in the LAQ.
//
// Memory ordering violations are detected by stores at their addr-gen time by
// associatively searching the LAQ for newer loads that have been issued to
// memory.
//
// The store queue contains both speculated and committed stores.
//
// Only one port to memory... loads and stores have to fight for it, West Side
// Story style.
//
// TODO:
//    - Add predicting structure for ordering failures
//    - currently won't STD forward if DMEM is busy
//    - ability to turn off things if VM is disabled
//    - reconsider port count of the wakeup, retry stuff

package boom.lsu

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.Parameters
import freechips.rocketchip.rocket
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util.{Str, UIntIsOneOf}

import freechips.rocketchip.diplomacy._

import boom.common._
import boom.common.MicroOpcodes._
import boom.exu.{BrUpdateInfo, Exception, FuncUnitReq, FuncUnitResp, CommitSignals, ExeUnitResp, VlWakeupResp, RegisterFileReadPortIO, TrTileRegReadPortIO}
import boom.util._

class LSUExeIO(implicit p: Parameters) extends BoomBundle()(p)
{
  // The "resp" of the maddrcalc is really a "req" to the LSU
  val req       = Flipped(new ValidIO(new FuncUnitResp(xLen)))
  // Send load data to regfiles
  val iresp    = new DecoupledIO(new ExeUnitResp(xLen))
  val fresp    = new DecoupledIO(new ExeUnitResp(xLen+1)) // TODO: Should this be fLen?
  //val vresp    = if (usingVector) new DecoupledIO(new ExeUnitResp(vLen)) else null
}

class BoomDCacheReq(implicit p: Parameters) extends BoomBundle()(p)
  with HasBoomUOP
{
  val addr  = UInt(coreMaxAddrBits.W)
  val data  = Bits(coreDataBits.W)
  val is_hella = Bool() // Is this the hellacache req? If so this is not tracked in LDQ or STQ
}

class BoomDCacheResp(implicit p: Parameters) extends BoomBundle()(p)
  with HasBoomUOP
{
  val data = Bits(coreDataBits.W)
  val is_hella = Bool()
}

class LSUDMemIO(implicit p: Parameters, edge: TLEdgeOut) extends BoomBundle()(p)
{
  // In LSU's dmem stage, send the request
  val req         = new DecoupledIO(Vec(memWidth, Valid(new BoomDCacheReq)))
  // In LSU's LCAM search stage, kill if order fail (or forwarding possible)
  val s1_kill     = Output(Vec(memWidth, Bool()))
  // Get a request any cycle
  val resp        = Flipped(Vec(memWidth, new ValidIO(new BoomDCacheResp)))
  // In our response stage, if we get a nack, we need to reexecute
  val nack        = Flipped(Vec(memWidth, new ValidIO(new BoomDCacheReq)))

  val brupdate     = Output(new BrUpdateInfo)
  val exception    = Output(Bool())
  val rob_pnr_idx  = Output(UInt(robAddrSz.W))
  val rob_head_idx = Output(UInt(robAddrSz.W))

  val release = Flipped(new DecoupledIO(new TLBundleC(edge.bundle)))

  // Clears prefetching MSHRs
  val force_order  = Output(Bool())
  val ordered     = Input(Bool())

  val perf = Input(new Bundle {
    val acquire = Bool()
    val release = Bool()
    val mshrs_has_busy = Bool()
    val mshrs_all_busy = Bool()
    val mshrs_reuse = Bool()
    val mshrs_load_establish = Bool()
    val mshrs_load_reuse = Bool()
    val mshrs_store_establish = Bool()
    val mshrs_store_reuse = Bool()
    val iomshrs_has_busy = Bool()
    val iomshrs_all_busy = Bool()
  })

  override def cloneType = new LSUDMemIO().asInstanceOf[this.type]
}

class BoomVMemReq(implicit p: Parameters) extends BoomBundle()(p)
  with HasBoomUOP
{
  val addr  = UInt(coreMaxAddrBits.W)
  val mask  = UInt(vLenb.W)
  val shdir = Bool()
  val shamt = UInt(log2Ceil(vLenb.max(p(freechips.rocketchip.subsystem.CacheBlockBytes))).W)
  val vldq_idx = UInt(vldqAddrSz.W)
  val vstq_idx = UInt(vstqAddrSz.W)
}

class BoomVMemResp(implicit p: Parameters) extends BoomBundle()(p)
  with HasBoomUOP
{
  val is_vst   = Bool()
  val vldq_idx = UInt(vldqAddrSz.W)
  val vstq_idx = UInt(vstqAddrSz.W)
  val cdata = UInt((8*p(freechips.rocketchip.subsystem.CacheBlockBytes)).W)
}

class BKQUInt(val wid: Int = 0)(implicit p: Parameters) extends BoomBundle()(p)
  with HasBoomUOP
{
  val data = UInt(wid.W)
}

class MLESplitCnt(implicit p: Parameters) extends BoomBundle()(p)
{
  val rob_idx = UInt(robAddrSz.W)
  val mle_cnt = UInt(vLenSz.W)
}

class VecMemIO(implicit p: Parameters, edge: TLEdgeOut) extends BoomBundle()(p)
{
  val req           = new DecoupledIO(new BoomVMemReq)
  val s1_vdata      = Output(UInt(vLen.W))
  val s1_kill       = Output(Bool())
  val resp          = Flipped(new ValidIO(new BoomVMemResp))
  val vsdq_ready    = Input(Bool())

  val brupdate      = Output(new BrUpdateInfo)
  val exception     = Output(Bool())
  val rob_pnr_idx   = Output(UInt(robAddrSz.W))
  val rob_head_idx  = Output(UInt(robAddrSz.W))

  val release = Flipped(new DecoupledIO(new TLBundleC(edge.bundle)))

  override def cloneType = new VecMemIO().asInstanceOf[this.type]
}

class LSUCoreIO(implicit p: Parameters) extends BoomBundle()(p)
{
  val exe = Vec(memWidth, new LSUExeIO)

  val dis_uops    = Flipped(Vec(coreWidth, Valid(new MicroOp)))
  val dis_ldq_idx = Output(Vec(coreWidth, UInt(ldqAddrSz.W)))
  val dis_stq_idx = Output(Vec(coreWidth, UInt(stqAddrSz.W)))

  val ldq_full    = Output(Vec(coreWidth, Bool()))
  val stq_full    = Output(Vec(coreWidth, Bool()))

  val fp_stdata    = if (usingFPU) Flipped(Decoupled(new ExeUnitResp(fLen))) else null
  val vrf_rport    = if (usingVector) Flipped(new RegisterFileReadPortIO(vpregSz, vLen)) else null
  val vrf_wbk      = if (usingVector) Decoupled(new ExeUnitResp(vLen)) else null
  val tile_rport   = if (usingMatrix) Flipped(new TrTileRegReadPortIO()) else null
  val tile_wbk     = if (usingMatrix) Decoupled(new ExeUnitResp(vLen)) else null
  val vbusy_status = if (usingVector) Input(UInt(numVecPhysRegs.W)) else null

  val commit      = Input(new CommitSignals)
  val commit_load_at_rob_head = Input(Bool())

  // Stores clear busy bit when stdata is received
  // memWidth for int, 1 for fp (to avoid back-pressure fpstdat)
  val clr_bsy         = Output(Vec(memWidth + 1, Valid(new MicroOp)))

  // Speculatively safe load (barring memory ordering failure)
  val clr_unsafe      = Output(Vec(memWidth, Valid(new MicroOp)))

  val update_mle      = if (usingMatrix) Output(Valid(new MLESplitCnt())) else null

  // Tell the DCache to clear prefetches/speculating misses
  val fence_dmem   = Input(Bool())

  // Speculatively tell the IQs that we'll get load data back next cycle
  val spec_ld_wakeup = Output(Vec(memWidth, Valid(UInt(maxPregSz.W))))
  // Tell the IQs that the load we speculated last cycle was misspeculated
  val ld_miss      = Output(Bool())

  // speculative vconfig wakeup
  val vl_wakeup    = Input(Valid(new VlWakeupResp()))

  val brupdate       = Input(new BrUpdateInfo)
  //val vmupdate       = if (usingVector) Input(Vec(vecWidth, Valid(new MicroOp))) else null
  val rob_pnr_idx  = Input(UInt(robAddrSz.W))
  val rob_head_idx = Input(UInt(robAddrSz.W))
  val exception    = Input(Bool())

  val fencei_rdy  = Output(Bool())

  val lxcpt       = Output(Valid(new Exception))

  val tsc_reg     = Input(UInt())

  val perf        = Output(new Bundle {
    val acquire = Bool()
    val release = Bool()
    val tlbMiss = Bool()
    val ldq_nonempty = Bool()
    val stq_nonempty = Bool()
    val stq_full = Bool()
    val mshrs_has_busy = Bool()
    val mshrs_all_busy = Bool()
    val mshrs_reuse = Bool()
    val mshrs_load_establish = Bool()
    val mshrs_load_reuse = Bool()
    val mshrs_store_establish = Bool()
    val mshrs_store_reuse = Bool()
    val iomshrs_has_busy = Bool()
    val iomshrs_all_busy = Bool()
    val in_flight_load   = Bool()
  })
}

class LSUIO(implicit p: Parameters, edge: TLEdgeOut) extends BoomBundle()(p)
{
  val ptw   = new rocket.TLBPTWIO
  val core  = new LSUCoreIO
  val dmem  = new LSUDMemIO
  val vmem  = new VecMemIO

  val hellacache = Flipped(new freechips.rocketchip.rocket.HellaCacheIO)
}

class LDQEntry(implicit p: Parameters) extends BoomBundle()(p)
    with HasBoomUOP
{
  val addr                = Valid(UInt(coreMaxAddrBits.W))
  val addr_is_virtual     = Bool() // Virtual address, we got a TLB miss
  val addr_is_uncacheable = Bool() // Uncacheable, wait until head of ROB to execute

  val executed            = Bool() // load sent to memory, reset by NACKs
  val succeeded           = Bool()
  val order_fail          = Bool()
  val observed            = Bool()

  val st_dep_mask         = UInt(numStqEntries.W) // list of stores older than us
  val youngest_stq_idx    = UInt(stqAddrSz.W) // index of the oldest store younger than us

  val forward_std_val     = Bool()
  val forward_stq_idx     = UInt(stqAddrSz.W)   // Which store did we get the store-load forward from?

  val const_stride        = Valid(UInt(xLen.W)) // const stride in constant strided load; row strides in MLE
  val debug_wb_data       = UInt(xLen.W)
}

class STQEntry(implicit p: Parameters) extends BoomBundle()(p)
   with HasBoomUOP
{
  val addr                = Valid(UInt(coreMaxAddrBits.W)) // valid is cleared when VS is processed with VSAGU
  val addr_is_virtual     = Bool() // Virtual address, we got a TLB miss
  val data                = Valid(UInt(xLen.W))

  val committed           = Bool() // committed by ROB
  val succeeded           = Bool() // D$ has ack'd this, we don't need to maintain this anymore

  val const_stride        = Valid(UInt(xLen.W)) // const stride in constant strided load; row strides in MLE
  val debug_wb_data       = UInt(xLen.W)
}

class VLDQEntry(implicit p: Parameters) extends LDQEntry()(p)
{
  //val is_aumus            = Bool()
  val vmask               = UInt(vLenb.W) // for fast unit-stride vl
  val shdir               = Bool()
  val shamt               = UInt(log2Ceil(vLenb.max(p(freechips.rocketchip.subsystem.CacheBlockBytes))).W)
}

class VSTQEntry(implicit p: Parameters) extends STQEntry()(p)
{
  //val is_aumus            = Bool()
  val vmask               = UInt(vLenb.W) // for fast unit-stride vl
  val shdir               = Bool()
  val shamt               = UInt(log2Ceil(vLenb.max(p(freechips.rocketchip.subsystem.CacheBlockBytes))).W)
}

class LSU(implicit p: Parameters, edge: TLEdgeOut) extends BoomModule()(p)
  with rocket.HasL1HellaCacheParameters
{
  val io = IO(new LSUIO)


  val ldq = Reg(Vec(numLdqEntries, Valid(new LDQEntry)))
  val stq = Reg(Vec(numStqEntries, Valid(new STQEntry)))

  val ldq_head         = Reg(UInt(ldqAddrSz.W))
  val ldq_tail         = Reg(UInt(ldqAddrSz.W))
  val stq_head         = Reg(UInt(stqAddrSz.W)) // point to next store to clear from STQ (i.e., send to memory)
  val stq_tail         = Reg(UInt(stqAddrSz.W))
  val stq_commit_head  = Reg(UInt(stqAddrSz.W)) // point to next store to commit
  val stq_execute_head = Reg(UInt(stqAddrSz.W)) // point to next store to execute

  val vldq = Reg(Vec(numVLdqEntries, Valid(new VLDQEntry)))
  val vstq = Reg(Vec(numVStqEntries, Valid(new VSTQEntry)))
  val vldq_head = Reg(UInt(vldqAddrSz.W))
  val vldq_tail = Reg(UInt(vldqAddrSz.W))
  val vstq_head = Reg(UInt(vstqAddrSz.W))
  val vstq_tail = Reg(UInt(vstqAddrSz.W))
  //val vstq_commit_head  = Reg(UInt(vstqAddrSz.W))
  val vstq_execute_head = Reg(UInt(vstqAddrSz.W))

  // If we got a mispredict, the tail will be misaligned for 1 extra cycle
  assert (io.core.brupdate.b2.mispredict ||
          stq(stq_execute_head).valid ||
          stq_head === stq_execute_head ||
          stq_tail === stq_execute_head,
            "stq_execute_head got off track.")

  val h_ready :: h_s1 :: h_s2 :: h_s2_nack :: h_wait :: h_replay :: h_dead :: Nil = Enum(7)
  // s1 : do TLB, if success and not killed, fire request go to h_s2
  //      store s1_data to register
  //      if tlb miss, go to s2_nack
  //      if don't get TLB, go to s2_nack
  //      store tlb xcpt
  // s2 : If kill, go to dead
  //      If tlb xcpt, send tlb xcpt, go to dead
  // s2_nack : send nack, go to dead
  // wait : wait for response, if nack, go to replay
  // replay : refire request, use already translated address
  // dead : wait for response, ignore it
  val hella_state           = RegInit(h_ready)
  val hella_req             = Reg(new rocket.HellaCacheReq)
  val hella_data            = Reg(new rocket.HellaCacheWriteData)
  val hella_paddr           = Reg(UInt(paddrBits.W))
  val hella_xcpt            = Reg(new rocket.HellaCacheExceptions)


  val dtlb = Module(new NBDTLB(
    instruction = false,
    lgMaxSize = log2Ceil(coreDataBytes),
    rocket.TLBConfig(dcacheParams.nTLBSets, dcacheParams.nTLBWays))
  )

  var vlagu:VecLSAddrGenUnit = null
  var vsagu:VecLSAddrGenUnit = null
  // var vlud_vrf_q:BranchKillableQueue[BKQUInt] = null
  var vlud_dat_q:BranchKillableQueue[BKQUInt] = null
  //var vstd_vrf_q:Queue[UInt] = null
  //var vstd_dat_q:Queue[UInt] = null
  var vrf_rarb:Arbiter[UInt] = null
  if (usingVector) {
    vlagu = Module(new VecLSAddrGenUnit())
    vsagu = Module(new VecLSAddrGenUnit())
    // vlud_vrf_q = Module(new BranchKillableQueue(new BKQUInt(0), 4, flow = false))
    vlud_dat_q = Module(new BranchKillableQueue(new BKQUInt(vLen), 4, flow = false))
    //vstd_vrf_q = Module(new Queue(new UInt(0.W), 4))
    //vstd_dat_q = Module(new Queue(new UInt(vLen.W), 4))
    vrf_rarb = Module(new Arbiter(UInt(vpregSz.W), 3))
  }

  io.ptw <> dtlb.io.ptw
  io.core.perf.tlbMiss := io.ptw.req.fire()
  io.core.perf.acquire := io.dmem.perf.acquire
  io.core.perf.release := io.dmem.perf.release
  io.core.perf.mshrs_has_busy := io.dmem.perf.mshrs_has_busy
  io.core.perf.mshrs_all_busy := io.dmem.perf.mshrs_all_busy
  io.core.perf.mshrs_reuse := io.dmem.perf.mshrs_reuse
  io.core.perf.mshrs_load_establish := io.dmem.perf.mshrs_load_establish
  io.core.perf.mshrs_load_reuse := io.dmem.perf.mshrs_load_reuse
  io.core.perf.mshrs_store_establish := io.dmem.perf.mshrs_store_establish
  io.core.perf.mshrs_store_reuse := io.dmem.perf.mshrs_store_reuse
  io.core.perf.iomshrs_has_busy := io.dmem.perf.iomshrs_has_busy
  io.core.perf.iomshrs_all_busy := io.dmem.perf.iomshrs_all_busy
  io.core.perf.in_flight_load := (0 until numLdqEntries).map(i => ldq(i).valid && ldq(i).bits.executed && !ldq(i).bits.succeeded).reduce(_ || _)

  val clear_store     = WireInit(false.B)
  val live_store_mask = RegInit(0.U(numStqEntries.W))
  var next_live_store_mask = Mux(clear_store, live_store_mask & ~(1.U << stq_head),
                                              live_store_mask)


  def widthMap[T <: Data](f: Int => T) = VecInit((0 until memWidth).map(f))


  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // Enqueue new entries
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // This is a newer store than existing loads, so clear the bit in all the store dependency masks
  for (i <- 0 until numLdqEntries)
  {
    when (clear_store)
    {
      ldq(i).bits.st_dep_mask := ldq(i).bits.st_dep_mask & ~(1.U << stq_head)
    }
  }

  // Decode stage
  var ld_enq_idx = ldq_tail
  var st_enq_idx = stq_tail

  val ldq_nonempty = (0 until numLdqEntries).map{ i => ldq(i).valid }.reduce(_||_) =/= 0.U
  val stq_nonempty = (0 until numStqEntries).map{ i => stq(i).valid }.reduce(_||_) =/= 0.U
  // when usingVector, no need to check vldq/vstq, since they will be empty ALA ldq/stq are empty
  io.core.perf.ldq_nonempty := ldq_nonempty
  io.core.perf.stq_nonempty := stq_nonempty

  var ldq_full = Bool()
  var stq_full = Bool()

  for (w <- 0 until coreWidth)
  {
    ldq_full = WrapInc(ld_enq_idx, numLdqEntries) === ldq_head
    io.core.ldq_full(w)    := ldq_full
    io.core.dis_ldq_idx(w) := ld_enq_idx

    stq_full = WrapInc(st_enq_idx, numStqEntries) === stq_head
    io.core.stq_full(w)    := stq_full
    io.core.dis_stq_idx(w) := st_enq_idx

    val dis_ld_val = io.core.dis_uops(w).valid && io.core.dis_uops(w).bits.uses_ldq && !io.core.dis_uops(w).bits.exception
    val dis_st_val = io.core.dis_uops(w).valid && io.core.dis_uops(w).bits.uses_stq && !io.core.dis_uops(w).bits.exception
    when (dis_ld_val)
    {
      ldq(ld_enq_idx).valid                := true.B
      ldq(ld_enq_idx).bits.uop             := io.core.dis_uops(w).bits
      ldq(ld_enq_idx).bits.youngest_stq_idx  := st_enq_idx
      ldq(ld_enq_idx).bits.st_dep_mask     := next_live_store_mask

      ldq(ld_enq_idx).bits.addr.valid      := false.B
      ldq(ld_enq_idx).bits.executed        := false.B
      ldq(ld_enq_idx).bits.succeeded       := false.B
      ldq(ld_enq_idx).bits.order_fail      := false.B
      ldq(ld_enq_idx).bits.observed        := false.B
      ldq(ld_enq_idx).bits.forward_std_val := false.B

      assert (ld_enq_idx === io.core.dis_uops(w).bits.ldq_idx, "[lsu] mismatch enq load tag.")
      assert (!ldq(ld_enq_idx).valid, "[lsu] Enqueuing uop is overwriting ldq entries")
    }
      .elsewhen (dis_st_val)
    {
      stq(st_enq_idx).valid           := true.B
      stq(st_enq_idx).bits.uop        := io.core.dis_uops(w).bits
      stq(st_enq_idx).bits.addr.valid := false.B
      stq(st_enq_idx).bits.data.valid := false.B
      stq(st_enq_idx).bits.committed  := false.B
      stq(st_enq_idx).bits.succeeded  := false.B
      //stq(st_enq_idx).bits.vmkilled   := false.B

      assert (st_enq_idx === io.core.dis_uops(w).bits.stq_idx, "[lsu] mismatch enq store tag.")
      assert (!stq(st_enq_idx).valid, "[lsu] Enqueuing uop is overwriting stq entries")
    }

    ld_enq_idx = Mux(dis_ld_val, WrapInc(ld_enq_idx, numLdqEntries),
                                 ld_enq_idx)

    next_live_store_mask = Mux(dis_st_val, next_live_store_mask | (1.U << st_enq_idx),
                                           next_live_store_mask)
    st_enq_idx = Mux(dis_st_val, WrapInc(st_enq_idx, numStqEntries),
                                 st_enq_idx)

    assert(!(dis_ld_val && dis_st_val), "A UOP is trying to go into both the LDQ and the STQ")
  }

  ldq_tail := ld_enq_idx
  stq_tail := st_enq_idx

  io.dmem.force_order   := io.core.fence_dmem
  io.core.fencei_rdy    := !stq_nonempty && io.dmem.ordered


  io.core.perf.stq_full := io.core.stq_full.reduce(_||_)

  // vl_wakeup, speculative vconfig wakeup
  for (i <- 0 until numLdqEntries)
  {
    when (io.core.vl_wakeup.valid && (io.core.vl_wakeup.bits.vconfig_tag+1.U) === ldq(i).bits.uop.vconfig_tag && !ldq(i).bits.uop.vl_ready)
    { 
      ldq(i).bits.uop.vl_ready   := true.B
      ldq(i).bits.uop.vconfig.vl := io.core.vl_wakeup.bits.vl
    }
  }

  for (i <- 0 until numStqEntries) 
  {
    when (io.core.vl_wakeup.valid && (io.core.vl_wakeup.bits.vconfig_tag+1.U) === stq(i).bits.uop.vconfig_tag && !stq(i).bits.uop.vl_ready) {
      stq(i).bits.uop.vl_ready   := true.B
      stq(i).bits.uop.vconfig.vl := io.core.vl_wakeup.bits.vl
    }
  }
  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // Execute stage (access TLB, send requests to Memory)
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // We can only report 1 exception per cycle.
  // Just be sure to report the youngest one
  val mem_xcpt_valid  = Wire(Bool())
  val mem_xcpt_cause  = Wire(UInt())
  val mem_xcpt_uop    = Wire(new MicroOp)
  val mem_xcpt_vaddr  = Wire(UInt())


  //---------------------------------------
  // Can-fire logic and wakeup/retry select
  //
  // First we determine what operations are waiting to execute.
  // These are the "can_fire"/"will_fire" signals

  val will_fire_load_incoming  = Wire(Vec(memWidth, Bool()))
  val will_fire_stad_incoming  = Wire(Vec(memWidth, Bool()))
  val will_fire_sta_incoming   = Wire(Vec(memWidth, Bool()))
  val will_fire_std_incoming   = Wire(Vec(memWidth, Bool()))
  val will_fire_sfence         = Wire(Vec(memWidth, Bool()))
  val will_fire_hella_incoming = Wire(Vec(memWidth, Bool()))
  val will_fire_hella_wakeup   = Wire(Vec(memWidth, Bool()))
  val will_fire_release        = Wire(Vec(memWidth, Bool()))
  val will_fire_load_retry     = Wire(Vec(memWidth, Bool()))
  val will_fire_sta_retry      = Wire(Vec(memWidth, Bool()))
  val will_fire_store_commit   = Wire(Vec(memWidth, Bool()))
  val will_fire_load_wakeup    = Wire(Vec(memWidth, Bool()))
  val will_fire_vload_incoming = Wire(Vec(memWidth, Bool()))
  val will_fire_vload_addrgen  = Wire(Vec(memWidth, Bool()))
  val will_fire_vldq_lookup    = Wire(Vec(memWidth, Bool()))
  val will_fire_vstore_incoming= Wire(Vec(memWidth, Bool()))
  val will_fire_vstore_addrgen = Wire(Vec(memWidth, Bool()))
  val will_fire_vstq_lookup    = Wire(Vec(memWidth, Bool()))
  val will_fire_vstq_commit    = Wire(Vec(memWidth, Bool()))

  val exe_req = WireInit(VecInit(io.core.exe.map(_.req)))
  // Sfence goes through all pipes
  for (i <- 0 until memWidth) {
    when (io.core.exe(i).req.bits.sfence.valid) {
      exe_req := VecInit(Seq.fill(memWidth) { io.core.exe(i).req })
    }
  }

  // -------------------------------
  // Assorted signals for scheduling

  // Don't wakeup a load if we just sent it last cycle or two cycles ago
  // The block_load_mask may be wrong, but the executing_load mask must be accurate
  val block_load_mask    = WireInit(VecInit((0 until numLdqEntries).map(x=>false.B)))
  val p1_block_load_mask = RegNext(block_load_mask)
  val p2_block_load_mask = RegNext(p1_block_load_mask)

 // Prioritize emptying the store queue when it is almost full
  val stq_almost_full = RegNext(WrapInc(WrapInc(st_enq_idx, numStqEntries), numStqEntries) === stq_head ||
                                WrapInc(st_enq_idx, numStqEntries) === stq_head)

  // The store at the commit head needs the DCache to appear ordered
  // Delay firing load wakeups and retries now
  val store_needs_order = WireInit(false.B)

  val ldq_incoming_idx = widthMap(i => exe_req(i).bits.uop.ldq_idx)
  val ldq_incoming_e   = widthMap(i => ldq(ldq_incoming_idx(i)))

  val stq_incoming_idx = widthMap(i => exe_req(i).bits.uop.stq_idx)
  val stq_incoming_e   = widthMap(i => stq(stq_incoming_idx(i)))

  val ldq_retry_idx = RegNext(AgePriorityEncoder((0 until numLdqEntries).map(i => {
    val e = ldq(i).bits
    val block = block_load_mask(i) || p1_block_load_mask(i)
    e.addr.valid && e.addr_is_virtual && !block && !e.uop.is_rvv
  }), ldq_head))
  val ldq_retry_e            = ldq(ldq_retry_idx)

  val stq_retry_idx = RegNext(AgePriorityEncoder((0 until numStqEntries).map(i => {
    val e = stq(i).bits
    e.addr.valid && e.addr_is_virtual && !e.uop.is_rvv
  }), stq_commit_head))
  val stq_retry_e   = stq(stq_retry_idx)

  val stq_commit_e  = stq(stq_execute_head)

  val ldq_wakeup_idx = RegNext(AgePriorityEncoder((0 until numLdqEntries).map(i=> {
    val e = ldq(i).bits
    val block = block_load_mask(i) || p1_block_load_mask(i)
    e.addr.valid && !e.executed && !e.succeeded && !e.addr_is_virtual && !block
  }), ldq_head))
  val ldq_wakeup_e   = ldq(ldq_wakeup_idx)

  // -----------------------
  // Determine what can fire

  // Can we fire a incoming load
  val can_fire_load_incoming = widthMap(w => exe_req(w).valid && exe_req(w).bits.uop.ctrl.is_load
                                                              && !exe_req(w).bits.uop.is_vm_ext)

  // Can we fire an incoming store addrgen + store datagen
  val can_fire_stad_incoming = widthMap(w => exe_req(w).valid && exe_req(w).bits.uop.ctrl.is_sta
                                                              && exe_req(w).bits.uop.ctrl.is_std
                                                              && !exe_req(w).bits.uop.is_vm_ext)

  // Can we fire an incoming store addrgen
  val can_fire_sta_incoming  = widthMap(w => exe_req(w).valid && exe_req(w).bits.uop.ctrl.is_sta
                                                              && !exe_req(w).bits.uop.ctrl.is_std
                                                              && !exe_req(w).bits.uop.is_vm_ext)

  // Can we fire an incoming store datagen
  val can_fire_std_incoming  = widthMap(w => exe_req(w).valid && exe_req(w).bits.uop.ctrl.is_std
                                                              && !exe_req(w).bits.uop.ctrl.is_sta
                                                              && !exe_req(w).bits.uop.is_vm_ext)

  // Can we fire an incoming sfence
  val can_fire_sfence        = widthMap(w => exe_req(w).valid && exe_req(w).bits.sfence.valid)

  // Can we fire a request from dcache to release a line
  // This needs to go through LDQ search to mark loads as dangerous
  val can_fire_release       = widthMap(w => (w == memWidth-1).B && io.dmem.release.valid)
  io.dmem.release.ready     := will_fire_release.reduce(_||_)

  // Can we retry a load that missed in the TLB
  val can_fire_load_retry    = widthMap(w =>
                               ( ldq_retry_e.valid                            &&
                                 ldq_retry_e.bits.addr.valid                  &&
                                 ldq_retry_e.bits.addr_is_virtual             &&
                                !ldq_retry_e.bits.uop.is_vm_ext               &&
                                !p1_block_load_mask(ldq_retry_idx)            &&
                                !p2_block_load_mask(ldq_retry_idx)            &&
                                RegNext(dtlb.io.miss_rdy)                     &&
                                !store_needs_order                            &&
                                (w == memWidth-1).B                           && // TODO: Is this best scheduling?
                                !ldq_retry_e.bits.order_fail))

  // Can we retry a store addrgen that missed in the TLB
  // - Weird edge case when sta_retry and std_incoming for same entry in same cycle. Delay this
  val can_fire_sta_retry     = widthMap(w =>
                               ( stq_retry_e.valid                            &&
                                 stq_retry_e.bits.addr.valid                  &&
                                 stq_retry_e.bits.addr_is_virtual             &&
                                !stq_retry_e.bits.uop.is_vm_ext               &&
                                 (w == memWidth-1).B                          &&
                                 RegNext(dtlb.io.miss_rdy)                    &&
                                 !(widthMap(i => (i != w).B               &&
                                                 can_fire_std_incoming(i) &&
                                                 stq_incoming_idx(i) === stq_retry_idx).reduce(_||_))
                               ))
  // Can we commit a store
  val can_fire_store_commit  = widthMap(w =>
                               ( stq_commit_e.valid                           &&
                                !stq_commit_e.bits.uop.is_fence               &&
                                !stq_commit_e.bits.uop.is_vm_ext              &&
                                !mem_xcpt_valid                               &&
                                !stq_commit_e.bits.uop.exception              &&
                                (w == 0).B                                    &&
                                (stq_commit_e.bits.committed || ( stq_commit_e.bits.uop.is_amo      &&
                                                                  stq_commit_e.bits.addr.valid      &&
                                                                 !stq_commit_e.bits.addr_is_virtual &&
                                                                  stq_commit_e.bits.data.valid))))

  // Can we wakeup a load that was nack'd
  val block_load_wakeup = WireInit(false.B)
  val can_fire_load_wakeup = widthMap(w =>
                             ( ldq_wakeup_e.valid                                      &&
                               ldq_wakeup_e.bits.addr.valid                            &&
                              !ldq_wakeup_e.bits.succeeded                             &&
                              !ldq_wakeup_e.bits.addr_is_virtual                       &&
                              !ldq_wakeup_e.bits.executed                              &&
                              !ldq_wakeup_e.bits.order_fail                            &&
                              !p1_block_load_mask(ldq_wakeup_idx)                      &&
                              !p2_block_load_mask(ldq_wakeup_idx)                      &&
                              !store_needs_order                                       &&
                              !block_load_wakeup                                       &&
                              (w == memWidth-1).B                                      &&
                              (!ldq_wakeup_e.bits.addr_is_uncacheable || (io.core.commit_load_at_rob_head &&
                                                                          ldq_head === ldq_wakeup_idx &&
                                                                          ldq_wakeup_e.bits.st_dep_mask.asUInt === 0.U))))

  // Can we fire an incoming hellacache request
  val can_fire_hella_incoming  = WireInit(widthMap(w => false.B)) // This is assigned to in the hellashim ocntroller

  // Can we fire a hellacache request that the dcache nack'd
  val can_fire_hella_wakeup    = WireInit(widthMap(w => false.B)) // This is assigned to in the hellashim controller

  //val block_vldq_mask    = WireInit(VecInit((0 until numVLdqEntries).map(x=>false.B)))
  //val p1_block_vldq_mask = RegNext(block_vldq_mask)
  //val p2_block_vldq_mask = RegNext(p1_block_vldq_mask)

  // Can we fire a incoming rvv load
  val can_fire_vload_incoming = widthMap(w => exe_req(w).valid && exe_req(w).bits.uop.ctrl.is_load
                                                               && exe_req(w).bits.uop.is_vm_ext)

  // Can we fire a incoming rvv store
  val can_fire_vstore_incoming = widthMap(w => exe_req(w).valid && (exe_req(w).bits.uop.ctrl.is_sta || exe_req(w).bits.uop.ctrl.is_std)
                                                                && exe_req(w).bits.uop.is_vm_ext)

  // Can we start vload addrgen
  val ldq_vag_idx = AgePriorityEncoder((0 until numLdqEntries).map(i=> {
    val e = ldq(i)
    e.valid && e.bits.addr.valid && !e.bits.executed && !e.bits.succeeded && !e.bits.order_fail && e.bits.uop.is_vm_ext
  }), ldq_head)
  val ldq_vag_e = ldq(ldq_vag_idx)
  val ldq_vag_uop = ldq_vag_e.bits.uop
  val can_fire_vload_addrgen   = widthMap(w =>      
                                 ((w == 0).B                                                  &&
                                  !vlagu.io.busy                                              &&
                                  ldq_vag_e.valid                                             &&
                                  ldq_vag_e.bits.addr.valid                                   &&
                                  !ldq_vag_e.bits.executed                                    &&
                                  !ldq_vag_e.bits.succeeded                                   &&
                                  ((ldq_vag_e.bits.uop.is_rvm)  || 
                                   (ldq_vag_e.bits.uop.is_rvv &&
                                    ~io.core.vbusy_status(ldq_vag_uop.stale_pvd(0).bits) &&
                                    (ldq_vag_uop.v_unmasked || ~io.core.vbusy_status(ldq_vag_uop.pvm)) &&
                                    (~ldq_vag_uop.v_idx_ls  || ~io.core.vbusy_status(ldq_vag_uop.pvs2(0).bits)) &&
                                    ldq_vag_uop.vl_ready))))

  // Can we start vstore addrgen
  val stq_vag_idx = AgePriorityEncoder((0 until numStqEntries).map(i => {
    val e = stq(i)
    e.valid && e.bits.addr.valid && !e.bits.committed && !e.bits.succeeded && e.bits.uop.is_rvv
  }), stq_head)
  val stq_vag_e = stq(stq_vag_idx)
  val stq_vag_uop = stq_vag_e.bits.uop
  val can_fire_vstore_addrgen  = widthMap(w =>
                                 ((w == memWidth-1).B                                   &&
                                  !vsagu.io.busy                                        &&
                                  stq_vag_e.valid                                       &&
                                  stq_vag_e.bits.addr.valid                             &&
                                  !stq_vag_e.bits.committed                             &&
                                  !stq_vag_e.bits.succeeded                             &&
                                  ((stq_vag_e.bits.uop.is_rvm)   ||
                                   (stq_vag_e.bits.uop.is_rvv &&
                                   (stq_vag_uop.v_unmasked || ~io.core.vbusy_status(stq_vag_uop.pvm)) &&
                                   (~stq_vag_uop.v_idx_ls  || ~io.core.vbusy_status(stq_vag_uop.pvs2(0).bits)) &&
                                   stq_vag_uop.vl_ready))))

  // Can we fire a vldq lookup
  val vldq_lkup_sel = AgePriorityEncoderOH((0 until numVLdqEntries).map(i => {
    val e = vldq(i)
    e.valid && e.bits.addr.valid && e.bits.addr_is_virtual && !e.bits.executed && !e.bits.order_fail
  }), vldq_head)
  val vldq_lkup_e = Mux1H(vldq_lkup_sel, vldq)
  val vldq_lkup_idx = Mux1H(vldq_lkup_sel, (0 until numVLdqEntries).map(i => i.U(vldqAddrSz.W)))
  val can_fire_vldq_lookup     = widthMap(w =>
                                 ((w == 0).B                                            &&
                                  vldq_lkup_e.valid                                     &&
                                  vldq_lkup_e.bits.addr.valid                           &&
                                  vldq_lkup_e.bits.addr_is_virtual                      &&
                                  !vldq_lkup_e.bits.executed                            &&
                                  !vldq_lkup_e.bits.order_fail                          &&
                                  io.vmem.req.ready))

  // Can we fire a vstq lookup
  val vstq_lkup_sel = AgePriorityEncoderOH((0 until numVStqEntries).map(i => {
    val e = vstq(i)
    e.valid && e.bits.addr.valid && e.bits.addr_is_virtual && !e.bits.committed
  }), vstq_head)
  val vstq_lkup_e = Mux1H(vstq_lkup_sel, vstq)
  val can_fire_vstq_lookup     = widthMap(w =>
                                 ((w == memWidth-1).B                                   &&
                                  vstq_lkup_e.valid                                     &&
                                  vstq_lkup_e.bits.addr.valid                           &&
                                  vstq_lkup_e.bits.addr_is_virtual                      &&
                                  !vstq_lkup_e.bits.committed))

  // Can we fire a vstq (partial) commit
  val vstq_commit_e = vstq(vstq_execute_head)
  val can_fire_vstq_commit     = widthMap(w =>
                                 ((w == memWidth-1).B                                        &&
                                  vstq_commit_e.valid                                        &&
                                  (vstq_commit_e.bits.data.valid || 
                                  ~io.core.vbusy_status(vstq_commit_e.bits.uop.stale_pdst))  &&       // FIXME: what about mse ?
                                  vstq_commit_e.bits.committed                               &&
                                  !mem_xcpt_valid                                            &&
                                  !vstq_commit_e.bits.uop.exception                          &&
                                  io.vmem.req.ready                                          &&
                                  io.vmem.vsdq_ready
                                 ))

  //---------------------------------------------------------
  // vldq/vstq enqueue logic
  val vldq_full = (WrapInc(vldq_tail, numVLdqEntries) === vldq_head)
  vlagu.io.resp.ready := !vldq_full
  when (vlagu.io.resp.fire) {
    vldq(vldq_tail).valid                 := true.B
    vldq(vldq_tail).bits.uop              := vlagu.io.resp.bits.uop
    vldq(vldq_tail).bits.youngest_stq_idx := ldq(vlagu.io.resp.bits.uop.ldq_idx).bits.youngest_stq_idx
    vldq(vldq_tail).bits.st_dep_mask      := ldq(vlagu.io.resp.bits.uop.ldq_idx).bits.st_dep_mask
    vldq(vldq_tail).bits.addr_is_virtual  := true.B
    vldq(vldq_tail).bits.addr.valid       := true.B
    vldq(vldq_tail).bits.addr.bits        := vlagu.io.resp.bits.addr
    vldq(vldq_tail).bits.executed         := false.B
    vldq(vldq_tail).bits.succeeded        := false.B
    vldq(vldq_tail).bits.order_fail       := false.B
    vldq(vldq_tail).bits.observed         := false.B
    vldq(vldq_tail).bits.forward_std_val  := false.B
    vldq(vldq_tail).bits.vmask            := vlagu.io.resp_vm
    vldq(vldq_tail).bits.shamt            := vlagu.io.resp_shamt
    vldq(vldq_tail).bits.shdir            := vlagu.io.resp_shdir
    vldq_tail := WrapInc(vldq_tail, numVLdqEntries)
  }

  // update mle cnt 
  if (usingMatrix) {
    when (vlagu.io.resp.fire && vlagu.io.resp.bits.uop.uopc.isOneOf(uopMLE)) {
      io.core.update_mle.valid        := true.B
      io.core.update_mle.bits.rob_idx := vlagu.io.resp.bits.uop.rob_idx
      io.core.update_mle.bits.mle_cnt := vlagu.io.resp.bits.data
    } .otherwise {
      io.core.update_mle.valid        := false.B
      io.core.update_mle.bits.rob_idx := 0.U
      io.core.update_mle.bits.mle_cnt := 0.U
    }
  }

  val vstq_full = WrapInc(vstq_tail, numVStqEntries) === vstq_head
  vsagu.io.resp.ready := !vstq_full
  when (vsagu.io.resp.fire) {
    vstq(vstq_tail).valid                 := true.B
    vstq(vstq_tail).bits.uop              := vsagu.io.resp.bits.uop
    vstq(vstq_tail).bits.addr_is_virtual  := true.B
    vstq(vstq_tail).bits.addr.valid       := true.B
    vstq(vstq_tail).bits.addr.bits        := vsagu.io.resp.bits.addr
    vstq(vstq_tail).bits.data.valid       := ~io.core.vbusy_status(vsagu.io.resp.bits.uop.stale_pdst)
    vstq(vstq_tail).bits.committed        := false.B
    vstq(vstq_tail).bits.succeeded        := false.B
    vstq(vstq_tail).bits.vmask            := vsagu.io.resp_vm
    vstq(vstq_tail).bits.shamt            := vsagu.io.resp_shamt
    vstq(vstq_tail).bits.shdir            := vsagu.io.resp_shdir
    vstq_tail := WrapInc(vstq_tail, numVStqEntries)
  }

  //---------------------------------------------------------
  // Controller logic. Arbitrate which request actually fires

  val exe_tlb_valid = Wire(Vec(memWidth, Bool()))
  for (w <- 0 until memWidth) {
    var tlb_avail  = true.B
    var dc_avail   = true.B
    var lcam_avail = true.B
    var rob_avail  = true.B
    var vmem_avail = true.B
    var vla_avail  = true.B
    var vsa_avail  = true.B

    def lsu_sched(can_fire: Bool, uses_tlb:  Boolean,
                                  uses_dc:   Boolean,
                                  uses_lcam: Boolean,
                                  uses_rob:  Boolean,
                                  uses_vla:  Boolean = false,
                                  uses_vsa:  Boolean = false,
                                  uses_vrf:  Boolean = false
                 ): Bool = {
      val will_fire = can_fire && !(uses_tlb.B && !tlb_avail) &&
                                  !(uses_lcam.B && !lcam_avail) &&
                                  !(uses_dc.B && !dc_avail) &&
                                  !(uses_rob.B && !rob_avail) &&
                                  !(uses_vla.B && !vla_avail) &&
                                  !(uses_vsa.B && !vsa_avail) &&
                                  !(uses_vrf.B && !vmem_avail)
      tlb_avail  = tlb_avail  && !(will_fire && uses_tlb.B)
      lcam_avail = lcam_avail && !(will_fire && uses_lcam.B)
      dc_avail   = dc_avail   && !(will_fire && uses_dc.B)
      rob_avail  = rob_avail  && !(will_fire && uses_rob.B)
      vla_avail  = vla_avail  && !(will_fire && uses_vla.B)
      vsa_avail  = vsa_avail  && !(will_fire && uses_vsa.B)
      vmem_avail = vmem_avail  && !(will_fire && uses_vrf.B)
      dontTouch(will_fire) // dontTouch these so we can inspect the will_fire signals
      will_fire
    }

    val O = true
    val X = false
    // The order of these statements is the priority
    // Some restrictions
    //  - Incoming ops must get precedence, can't backpresure memaddrgen
    //  - Incoming hellacache ops must get precedence over retrying ops (PTW must get precedence over retrying translation)
    // Notes on performance
    //  - Prioritize releases, this speeds up cache line writebacks and refills
    //  - Store commits are lowest priority, since they don't "block" younger instructions unless stq fills up
    will_fire_load_incoming (w) := lsu_sched(can_fire_load_incoming (w), O, O, O, X)          // TLB, DC , LCAM,
    will_fire_stad_incoming (w) := lsu_sched(can_fire_stad_incoming (w), O, X, O, O)          // TLB,    , LCAM, ROB
    will_fire_sta_incoming  (w) := lsu_sched(can_fire_sta_incoming  (w), O, X, O, O)          // TLB,    , LCAM, ROB
    will_fire_std_incoming  (w) := lsu_sched(can_fire_std_incoming  (w), X, X, X, O)          //               , ROB
    will_fire_sfence        (w) := lsu_sched(can_fire_sfence        (w), O, X, X, O)          // TLB,    ,     , ROB
    will_fire_release       (w) := lsu_sched(can_fire_release       (w), X, X, O, X)          //           LCAM
    will_fire_hella_incoming(w) := lsu_sched(can_fire_hella_incoming(w), O, O, X, X)          // TLB, DC
    will_fire_hella_wakeup  (w) := lsu_sched(can_fire_hella_wakeup  (w), X, O, X, X)          //    , DC
    will_fire_load_retry    (w) := lsu_sched(can_fire_load_retry    (w), O, O, O, X)          // TLB, DC , LCAM
    will_fire_sta_retry     (w) := lsu_sched(can_fire_sta_retry     (w), O, X, O, O)          // TLB,    , LCAM, ROB // TODO: This should be higher priority
    will_fire_load_wakeup   (w) := lsu_sched(can_fire_load_wakeup   (w), X, O, O, X)          //    , DC , LCAM
    will_fire_store_commit  (w) := lsu_sched(can_fire_store_commit  (w), X, O, X, X)          //    , DC
    will_fire_vload_incoming(w) := lsu_sched(can_fire_vload_incoming(w), X, X, X, X)          //
    will_fire_vstore_incoming(w):= lsu_sched(can_fire_vstore_incoming(w),X, X, X, X)          //
    will_fire_vstq_commit   (w) := lsu_sched(can_fire_vstq_commit   (w), X, X, X, X, O, X, X) //    ,    ,     ,    , VMEM
    will_fire_vldq_lookup   (w) := lsu_sched(can_fire_vldq_lookup   (w), O, X, O, X, O, X, X) // TLB,    , LCAM,    , VMEM
    will_fire_vstq_lookup   (w) := lsu_sched(can_fire_vstq_lookup   (w), O, X, O, O, X, X, X) // TLB,    , LCAM, ROB
    will_fire_vload_addrgen (w) := lsu_sched(can_fire_vload_addrgen (w), X, X, X, X, X, O, X) //    ,    ,     ,    ,     , VLA
    will_fire_vstore_addrgen(w) := lsu_sched(can_fire_vstore_addrgen(w), X, X, X, X, X, X, O) //    ,    ,     ,    ,     ,    , VSA


    assert(!(exe_req(w).valid && !(will_fire_load_incoming(w) ||
                                   will_fire_stad_incoming(w) ||
                                   will_fire_sta_incoming(w)  ||
                                   will_fire_std_incoming(w)  ||
                                   will_fire_sfence(w)        ||
                                   exe_req(w).bits.uop.is_vm_ext)))

    when (will_fire_load_wakeup(w)) {
      block_load_mask(ldq_wakeup_idx)           := true.B
    } .elsewhen (will_fire_load_incoming(w)) {
      block_load_mask(exe_req(w).bits.uop.ldq_idx) := true.B
    } .elsewhen (will_fire_load_retry(w)) {
      block_load_mask(ldq_retry_idx)            := true.B
    }
    exe_tlb_valid(w) := !tlb_avail
  }
  assert((memWidth == 1).B ||
    (!(will_fire_sfence.reduce(_||_) && !will_fire_sfence.reduce(_&&_)) &&
     !will_fire_hella_incoming.reduce(_&&_) &&
     !will_fire_hella_wakeup.reduce(_&&_)   &&
     !will_fire_load_retry.reduce(_&&_)     &&
     !will_fire_sta_retry.reduce(_&&_)      &&
     !will_fire_store_commit.reduce(_&&_)   &&
     !will_fire_load_wakeup.reduce(_&&_)),
    "Some operations is proceeding down multiple pipes")

  require(memWidth <= 2)

  //--------------------------------------------
  // TLB Access

  assert(!(hella_state =/= h_ready && hella_req.cmd === rocket.M_SFENCE),
    "SFENCE through hella interface not supported")

  val exe_tlb_uop = widthMap(w => MuxCase(NullMicroOp(), Seq(
                     (will_fire_load_incoming (w) ||
                      will_fire_stad_incoming (w) ||
                      will_fire_sta_incoming  (w) ||
                      will_fire_sfence        (w)) -> exe_req(w).bits.uop,
                      will_fire_load_retry    (w)  -> ldq_retry_e.bits.uop,
                      will_fire_sta_retry     (w)  -> stq_retry_e.bits.uop,
                      //will_fire_hella_incoming(w)  -> NullMicroOp(),
                      will_fire_vldq_lookup   (w)  -> vldq_lkup_e.bits.uop,
                      will_fire_vstq_lookup   (w)  -> vstq_lkup_e.bits.uop
                    )))

  val exe_tlb_vaddr = widthMap(w => MuxCase(0.U, Seq(
                       (will_fire_load_incoming (w) ||
                        will_fire_stad_incoming (w) ||
                        will_fire_sta_incoming  (w)) -> exe_req(w).bits.addr,
                        will_fire_sfence        (w)  -> exe_req(w).bits.sfence.bits.addr,
                        will_fire_load_retry    (w)  -> ldq_retry_e.bits.addr.bits,
                        will_fire_sta_retry     (w)  -> stq_retry_e.bits.addr.bits,
                        will_fire_hella_incoming(w)  -> hella_req.addr,
                        will_fire_vldq_lookup   (w)  -> vldq_lkup_e.bits.addr.bits,
                        will_fire_vstq_lookup   (w)  -> vstq_lkup_e.bits.addr.bits
                      )))

  val exe_sfence = WireInit((0.U).asTypeOf(Valid(new rocket.SFenceReq)))
  for (w <- 0 until memWidth) {
    when (will_fire_sfence(w)) {
      exe_sfence := exe_req(w).bits.sfence
    }
  }

  val exe_size   = widthMap(w =>
                   Mux(will_fire_load_incoming (w) ||
                       will_fire_stad_incoming (w) ||
                       will_fire_sta_incoming  (w) ||
                       will_fire_sfence        (w) ||
                       will_fire_load_retry    (w) ||
                       will_fire_sta_retry     (w)  , exe_tlb_uop(w).mem_size,
                   Mux(will_fire_hella_incoming(w)  , hella_req.size,
                                                      0.U)))
  val exe_cmd    = widthMap(w =>
                   Mux(will_fire_load_incoming (w) ||
                       will_fire_stad_incoming (w) ||
                       will_fire_sta_incoming  (w) ||
                       will_fire_sfence        (w) ||
                       will_fire_load_retry    (w) ||
                       will_fire_sta_retry     (w)  , exe_tlb_uop(w).mem_cmd,
                   Mux(will_fire_hella_incoming(w)  , hella_req.cmd,
                                                      0.U)))

  val exe_passthr= widthMap(w =>
                   Mux(will_fire_hella_incoming(w)  , hella_req.phys,
                                                      false.B))
  val exe_kill   = widthMap(w =>
                   Mux(will_fire_hella_incoming(w)  , io.hellacache.s1_kill,
                                                      false.B))
  for (w <- 0 until memWidth) {
    dtlb.io.req(w).valid            := exe_tlb_valid(w)
    dtlb.io.req(w).bits.vaddr       := exe_tlb_vaddr(w)
    dtlb.io.req(w).bits.size        := exe_size(w)
    dtlb.io.req(w).bits.cmd         := exe_cmd(w)
    dtlb.io.req(w).bits.passthrough := exe_passthr(w)
  }
  dtlb.io.kill                      := exe_kill.reduce(_||_)
  dtlb.io.sfence                    := exe_sfence

  // exceptions
  val ma_ld = widthMap(w => will_fire_load_incoming(w) && exe_req(w).bits.mxcpt.valid) // We get ma_ld in memaddrcalc
  val ma_st = widthMap(w => (will_fire_sta_incoming(w) || will_fire_stad_incoming(w)) && exe_req(w).bits.mxcpt.valid) // We get ma_ld in memaddrcalc
  val pf_ld = widthMap(w => dtlb.io.req(w).valid && dtlb.io.resp(w).pf.ld && exe_tlb_uop(w).uses_ldq)
  val pf_st = widthMap(w => dtlb.io.req(w).valid && dtlb.io.resp(w).pf.st && exe_tlb_uop(w).uses_stq)
  val ae_ld = widthMap(w => dtlb.io.req(w).valid && dtlb.io.resp(w).ae.ld && exe_tlb_uop(w).uses_ldq)
  val ae_st = widthMap(w => dtlb.io.req(w).valid && dtlb.io.resp(w).ae.st && exe_tlb_uop(w).uses_stq)

  // TODO check for xcpt_if and verify that never happens on non-speculative instructions.
  val mem_xcpt_valids = RegNext(widthMap(w =>
                     (pf_ld(w) || pf_st(w) || ae_ld(w) || ae_st(w) || ma_ld(w) || ma_st(w)) &&
                     !io.core.exception &&
                     !IsKilledByBranch(io.core.brupdate, exe_tlb_uop(w))))
  val mem_xcpt_uops   = RegNext(widthMap(w => UpdateBrMask(io.core.brupdate, exe_tlb_uop(w))))
  val mem_xcpt_causes = RegNext(widthMap(w =>
    Mux(ma_ld(w), rocket.Causes.misaligned_load.U,
    Mux(ma_st(w), rocket.Causes.misaligned_store.U,
    Mux(pf_ld(w), rocket.Causes.load_page_fault.U,
    Mux(pf_st(w), rocket.Causes.store_page_fault.U,
    Mux(ae_ld(w), rocket.Causes.load_access.U,
                  rocket.Causes.store_access.U)))))))
  val mem_xcpt_vaddrs = RegNext(exe_tlb_vaddr)

  for (w <- 0 until memWidth) {
    assert (!(dtlb.io.req(w).valid && exe_tlb_uop(w).is_fence), "Fence is pretending to talk to the TLB")
    assert (!((will_fire_load_incoming(w) || will_fire_sta_incoming(w) || will_fire_stad_incoming(w)) &&
      exe_req(w).bits.mxcpt.valid && dtlb.io.req(w).valid &&
    !(exe_tlb_uop(w).ctrl.is_load || exe_tlb_uop(w).ctrl.is_sta)),
      "A uop that's not a load or store-address is throwing a memory exception.")
  }

  mem_xcpt_valid := mem_xcpt_valids.reduce(_||_)
  mem_xcpt_cause := mem_xcpt_causes(0)
  mem_xcpt_uop   := mem_xcpt_uops(0)
  mem_xcpt_vaddr := mem_xcpt_vaddrs(0)
  var xcpt_found = mem_xcpt_valids(0)
  var oldest_xcpt_rob_idx = mem_xcpt_uops(0).rob_idx
  for (w <- 1 until memWidth) {
    val is_older = WireInit(false.B)
    when (mem_xcpt_valids(w) &&
      (IsOlder(mem_xcpt_uops(w).rob_idx, oldest_xcpt_rob_idx, io.core.rob_head_idx) || !xcpt_found)) {
      is_older := true.B
      mem_xcpt_cause := mem_xcpt_causes(w)
      mem_xcpt_uop   := mem_xcpt_uops(w)
      mem_xcpt_vaddr := mem_xcpt_vaddrs(w)
    }
    xcpt_found = xcpt_found || mem_xcpt_valids(w)
    oldest_xcpt_rob_idx = Mux(is_older, mem_xcpt_uops(w).rob_idx, oldest_xcpt_rob_idx)
  }

  val exe_tlb_miss  = widthMap(w => dtlb.io.req(w).valid && (dtlb.io.resp(w).miss || !dtlb.io.req(w).ready))
  val exe_tlb_paddr = widthMap(w => Cat(dtlb.io.resp(w).paddr(paddrBits-1,corePgIdxBits),
                                        exe_tlb_vaddr(w)(corePgIdxBits-1,0)))
  val exe_tlb_uncacheable = widthMap(w => !(dtlb.io.resp(w).cacheable))

  for (w <- 0 until memWidth) {
    assert (exe_tlb_paddr(w) === dtlb.io.resp(w).paddr || exe_req(w).bits.sfence.valid, "[lsu] paddrs should match.")

    when (mem_xcpt_valids(w))
    {
      assert(RegNext(will_fire_load_incoming(w) || will_fire_stad_incoming(w) || will_fire_sta_incoming(w) ||
        will_fire_load_retry(w) || will_fire_sta_retry(w)))
      // Technically only faulting AMOs need this
      assert(mem_xcpt_uops(w).uses_ldq ^ mem_xcpt_uops(w).uses_stq)
      when (mem_xcpt_uops(w).uses_ldq)
      {
        ldq(mem_xcpt_uops(w).ldq_idx).bits.uop.exception := true.B
      }
        .otherwise
      {
        stq(mem_xcpt_uops(w).stq_idx).bits.uop.exception := true.B
      }
    }
  }



  //------------------------------
  // Issue Someting to Memory
  //
  // A memory op can come from many different places
  // The address either was freshly translated, or we are
  // reading a physical address from the LDQ,STQ, or the HellaCache adapter


  // defaults
  io.dmem.brupdate       := io.core.brupdate
  io.dmem.exception      := io.core.exception
  io.dmem.rob_head_idx   := io.core.rob_head_idx
  io.dmem.rob_pnr_idx    := io.core.rob_pnr_idx

  val dmem_req = Wire(Vec(memWidth, Valid(new BoomDCacheReq)))
  io.dmem.req.valid := dmem_req.map(_.valid).reduce(_||_)
  io.dmem.req.bits  := dmem_req
  val dmem_req_fire = widthMap(w => dmem_req(w).valid && io.dmem.req.fire())

  val s0_executing_loads = WireInit(VecInit((0 until numLdqEntries).map(x=>false.B)))

  for (w <- 0 until memWidth) {
    dmem_req(w).valid := false.B
    dmem_req(w).bits.uop   := NullMicroOp()
    dmem_req(w).bits.addr  := 0.U
    dmem_req(w).bits.data  := 0.U
    dmem_req(w).bits.is_hella := false.B

    io.dmem.s1_kill(w) := false.B

    when (will_fire_load_incoming(w)) {
      dmem_req(w).valid      := !exe_tlb_miss(w) && !exe_tlb_uncacheable(w)
      dmem_req(w).bits.addr  := exe_tlb_paddr(w)
      dmem_req(w).bits.uop   := exe_tlb_uop(w)

      s0_executing_loads(ldq_incoming_idx(w)) := dmem_req_fire(w)
      assert(!ldq_incoming_e(w).bits.executed)
    } .elsewhen (will_fire_load_retry(w)) {
      dmem_req(w).valid      := !exe_tlb_miss(w) && !exe_tlb_uncacheable(w)
      dmem_req(w).bits.addr  := exe_tlb_paddr(w)
      dmem_req(w).bits.uop   := exe_tlb_uop(w)

      s0_executing_loads(ldq_retry_idx) := dmem_req_fire(w)
      assert(!ldq_retry_e.bits.executed)
    } .elsewhen (will_fire_store_commit(w)) {
      dmem_req(w).valid         := true.B
      dmem_req(w).bits.addr     := stq_commit_e.bits.addr.bits
      dmem_req(w).bits.data     := (new freechips.rocketchip.rocket.StoreGen(
                                    stq_commit_e.bits.uop.mem_size, 0.U,
                                    stq_commit_e.bits.data.bits,
                                    coreDataBytes)).data
      dmem_req(w).bits.uop      := stq_commit_e.bits.uop

      stq_execute_head                     := Mux(dmem_req_fire(w),
                                                WrapInc(stq_execute_head, numStqEntries),
                                                stq_execute_head)

      stq(stq_execute_head).bits.succeeded := false.B
    } .elsewhen (will_fire_load_wakeup(w)) {
      dmem_req(w).valid      := true.B
      dmem_req(w).bits.addr  := ldq_wakeup_e.bits.addr.bits
      dmem_req(w).bits.uop   := ldq_wakeup_e.bits.uop

      s0_executing_loads(ldq_wakeup_idx) := dmem_req_fire(w)

      assert(!ldq_wakeup_e.bits.executed && !ldq_wakeup_e.bits.addr_is_virtual)
    } .elsewhen (will_fire_hella_incoming(w)) {
      assert(hella_state === h_s1)

      dmem_req(w).valid               := !io.hellacache.s1_kill && (!exe_tlb_miss(w) || hella_req.phys)
      dmem_req(w).bits.addr           := exe_tlb_paddr(w)
      dmem_req(w).bits.data           := (new freechips.rocketchip.rocket.StoreGen(
        hella_req.size, 0.U,
        io.hellacache.s1_data.data,
        coreDataBytes)).data
      dmem_req(w).bits.uop.mem_cmd    := hella_req.cmd
      dmem_req(w).bits.uop.mem_size   := hella_req.size
      dmem_req(w).bits.uop.mem_signed := hella_req.signed
      dmem_req(w).bits.is_hella       := true.B

      hella_paddr := exe_tlb_paddr(w)
    } .elsewhen (will_fire_hella_wakeup(w)) {
      assert(hella_state === h_replay)
      dmem_req(w).valid               := true.B
      dmem_req(w).bits.addr           := hella_paddr
      dmem_req(w).bits.data           := (new freechips.rocketchip.rocket.StoreGen(
        hella_req.size, 0.U,
        hella_data.data,
        coreDataBytes)).data
      dmem_req(w).bits.uop.mem_cmd    := hella_req.cmd
      dmem_req(w).bits.uop.mem_size   := hella_req.size
      dmem_req(w).bits.uop.mem_signed := hella_req.signed
      dmem_req(w).bits.is_hella       := true.B
    }

    //-------------------------------------------------------------
    // Write Addr into the LAQ/SAQ
    when (will_fire_load_incoming(w) || will_fire_load_retry(w))
    {
      val ldq_idx = Mux(will_fire_load_incoming(w), ldq_incoming_idx(w), ldq_retry_idx)
      ldq(ldq_idx).bits.addr.valid          := true.B
      ldq(ldq_idx).bits.addr.bits           := Mux(exe_tlb_miss(w), exe_tlb_vaddr(w), exe_tlb_paddr(w))
      ldq(ldq_idx).bits.uop.pdst            := exe_tlb_uop(w).pdst
      ldq(ldq_idx).bits.addr_is_virtual     := exe_tlb_miss(w)
      ldq(ldq_idx).bits.addr_is_uncacheable := exe_tlb_uncacheable(w) && !exe_tlb_miss(w)

      assert(!(will_fire_load_incoming(w) && ldq_incoming_e(w).bits.addr.valid),
        "[lsu] Incoming load is overwriting a valid address")
    }

    when (will_fire_sta_incoming(w) || will_fire_stad_incoming(w) || will_fire_sta_retry(w))
    {
      val stq_idx = Mux(will_fire_sta_incoming(w) || will_fire_stad_incoming(w),
        stq_incoming_idx(w), stq_retry_idx)

      stq(stq_idx).bits.addr.valid := !pf_st(w) // Prevent AMOs from executing!
      stq(stq_idx).bits.addr.bits  := Mux(exe_tlb_miss(w), exe_tlb_vaddr(w), exe_tlb_paddr(w))
      stq(stq_idx).bits.uop.pdst   := exe_tlb_uop(w).pdst // Needed for AMOs
      stq(stq_idx).bits.addr_is_virtual := exe_tlb_miss(w)

      assert(!(will_fire_sta_incoming(w) && stq_incoming_e(w).bits.addr.valid),
        "[lsu] Incoming store is overwriting a valid address")

    }

    //-------------------------------------------------------------
    // Write data into the STQ
    if (w == 0) {
      io.core.fp_stdata.ready := !will_fire_std_incoming(w) && !will_fire_stad_incoming(w)
    }
    val fp_stdata_fire = io.core.fp_stdata.fire() && (w == 0).B
    when (will_fire_std_incoming(w) || will_fire_stad_incoming(w) || fp_stdata_fire)
    {
      val sidx = Mux(will_fire_std_incoming(w) || will_fire_stad_incoming(w), stq_incoming_idx(w),
                  io.core.fp_stdata.bits.uop.stq_idx)
      stq(sidx).bits.data.valid := true.B
      stq(sidx).bits.data.bits  := Mux(will_fire_std_incoming(w) || will_fire_stad_incoming(w), exe_req(w).bits.data,
                                   io.core.fp_stdata.bits.data)
      assert(!(stq(sidx).bits.data.valid),
        "[lsu] Incoming store is overwriting a valid data entry")
    }

    when (will_fire_vload_incoming(w)) {
      val ldq_e = ldq(ldq_incoming_idx(w))
      ldq_e.bits.addr.valid           := true.B
      ldq_e.bits.addr.bits            := exe_req(w).bits.addr
      ldq_e.bits.const_stride.valid   := exe_req(w).bits.uop.uopc.isOneOf(uopVLS, uopMLE)
      ldq_e.bits.const_stride.bits    := exe_req(w).bits.data
      ldq_e.bits.addr_is_virtual      := true.B
      ldq_e.bits.addr_is_uncacheable  := false.B
    }

    when (will_fire_vstore_incoming(w)) {
      val stq_e = stq(stq_incoming_idx(w))
      stq_e.bits.addr.valid           := true.B
      stq_e.bits.addr.bits            := exe_req(w).bits.addr
      stq_e.bits.const_stride.valid   := exe_req(w).bits.uop.uopc.isOneOf(uopVSSA, uopMSE)
      stq_e.bits.const_stride.bits    := exe_req(w).bits.data
      stq_e.bits.addr_is_virtual      := true.B
    }
  }

  val vmem_req = Wire(new BoomVMemReq)

  vmem_req.uop   := NullMicroOp()
  vmem_req.addr  := 0.U
  vmem_req.mask  := 0.U
  vmem_req.shdir := false.B
  vmem_req.shamt := 0.U
  vmem_req.vldq_idx := 0.U
  vmem_req.vstq_idx := 0.U

  when (will_fire_vldq_lookup(0)) {
    vmem_req.uop   := vldq_lkup_e.bits.uop
    vmem_req.addr  := vldq_lkup_e.bits.addr.bits
    vmem_req.mask  := vldq_lkup_e.bits.vmask
    vmem_req.shdir := vldq_lkup_e.bits.shdir
    vmem_req.shamt := vldq_lkup_e.bits.shamt
    vmem_req.vldq_idx := vldq_lkup_idx
  } .elsewhen (will_fire_vstq_commit(memWidth-1)) {
    vmem_req.uop   := vstq_commit_e.bits.uop
    vmem_req.addr  := vstq_commit_e.bits.addr.bits
    vmem_req.mask  := vstq_commit_e.bits.vmask
    vmem_req.shdir := vstq_commit_e.bits.shdir
    vmem_req.shamt := vstq_commit_e.bits.shamt
    vmem_req.vstq_idx := vstq_execute_head
    vstq_execute_head := WrapInc(vstq_execute_head, numVStqEntries)
    when ((vstq_commit_e.bits.uop.is_rvv && vstq_commit_e.bits.uop.v_split_last) ||
          (vstq_commit_e.bits.uop.is_rvm && vstq_commit_e.bits.uop.m_split_last)) {
      stq_execute_head := WrapInc(stq_execute_head, numStqEntries)
    }
  }
  io.vmem.req.valid     := will_fire_vldq_lookup(0) && !exe_tlb_miss(0) && !exe_tlb_uncacheable(0) ||
                           will_fire_vstq_commit(memWidth-1)
  io.vmem.req.bits      := vmem_req
  io.vmem.s1_vdata      := Mux(RegNext(vmem_req.uop.is_rvv), io.core.vrf_rport.data, io.core.tile_rport.data) //FIXME: confirm latency
  io.vmem.s1_kill       := false.B // fixme
  io.vmem.brupdate      := io.core.brupdate
  io.vmem.exception     := io.core.exception
  io.vmem.rob_pnr_idx   := io.core.rob_pnr_idx
  io.vmem.rob_head_idx  := io.core.rob_head_idx

  for (i <- 0 until numLdqEntries) {
    when (ldq_vag_idx === i.U && will_fire_vload_addrgen(0)) {
      ldq(i).bits.executed := true.B
    }
  }

  for (i <- 0 until numStqEntries) {
    when (stq_vag_idx === i.U && will_fire_vstore_addrgen(0)) {
      stq(i).bits.addr.valid := false.B
    }
  }

  for (i <- 0 until numVLdqEntries) {
    when (vldq_lkup_sel(i) && will_fire_vldq_lookup(0)) {
      when (!exe_tlb_miss(0)) {
        vldq(i).bits.addr.bits := exe_tlb_paddr(0)
      }
      vldq(i).bits.addr_is_virtual     := exe_tlb_miss(0)
      vldq(i).bits.addr_is_uncacheable := exe_tlb_uncacheable(0) && !exe_tlb_miss(0)
      vldq(i).bits.executed            := !exe_tlb_miss(0)
    }
  }

  for (i <- 0 until numVStqEntries) {
    // FIXME use io.vmem.resp
    when (i.U === io.vmem.resp.bits.vstq_idx && io.vmem.resp.valid && io.vmem.resp.bits.is_vst) {
      vstq(i).bits.succeeded := true.B
    }

    when (vstq_lkup_sel(i) && will_fire_vstq_lookup(0)) {
      when (!exe_tlb_miss(0)) {
        vstq(i).bits.addr.bits := exe_tlb_paddr(0)
      }
      vstq(i).bits.addr_is_virtual     := exe_tlb_miss(0)
    }
  }

  val will_fire_stdf_incoming = io.core.fp_stdata.fire()
  require (xLen >= fLen) // for correct SDQ size

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // Cache Access Cycle (Mem)
  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // Note the DCache may not have accepted our request

  val exe_req_killed = widthMap(w => IsKilledByBranch(io.core.brupdate, exe_req(w).bits.uop))
  val stdf_killed = IsKilledByBranch(io.core.brupdate, io.core.fp_stdata.bits.uop)

  val fired_load_incoming  = widthMap(w => RegNext(will_fire_load_incoming(w) && !exe_req_killed(w)))
  val fired_stad_incoming  = widthMap(w => RegNext(will_fire_stad_incoming(w) && !exe_req_killed(w)))
  val fired_sta_incoming   = widthMap(w => RegNext(will_fire_sta_incoming (w) && !exe_req_killed(w)))
  val fired_std_incoming   = widthMap(w => RegNext(will_fire_std_incoming (w) && !exe_req_killed(w)))
  val fired_stdf_incoming  = RegNext(will_fire_stdf_incoming && !stdf_killed)
  val fired_sfence         = RegNext(will_fire_sfence)
  val fired_release        = RegNext(will_fire_release)
  val fired_load_retry     = widthMap(w => RegNext(will_fire_load_retry   (w) && !IsKilledByBranch(io.core.brupdate, ldq_retry_e.bits.uop)))
  val fired_sta_retry      = widthMap(w => RegNext(will_fire_sta_retry    (w) && !IsKilledByBranch(io.core.brupdate, stq_retry_e.bits.uop)))
  //val fired_store_commit   = RegNext(will_fire_store_commit)
  val fired_load_wakeup    = widthMap(w => RegNext(will_fire_load_wakeup  (w) && !IsKilledByBranch(io.core.brupdate, ldq_wakeup_e.bits.uop)))
  val fired_hella_incoming = RegNext(will_fire_hella_incoming)
  val fired_hella_wakeup   = RegNext(will_fire_hella_wakeup)
  val fired_vldq_lookup    = RegNext(will_fire_vldq_lookup)
  val fired_vstq_lookup    = RegNext(will_fire_vstq_lookup)
  val fired_vstq_commit    = RegNext(will_fire_vstq_commit)

  val mem_incoming_uop     = RegNext(widthMap(w => UpdateBrMask(io.core.brupdate, exe_req(w).bits.uop)))
  val mem_ldq_incoming_e   = RegNext(widthMap(w => UpdateBrMask(io.core.brupdate, ldq_incoming_e(w))))
  val mem_stq_incoming_e   = RegNext(widthMap(w => UpdateBrMask(io.core.brupdate, stq_incoming_e(w))))
  val mem_ldq_wakeup_e     = RegNext(UpdateBrMask(io.core.brupdate, ldq_wakeup_e))
  val mem_ldq_retry_e      = RegNext(UpdateBrMask(io.core.brupdate, ldq_retry_e))
  val mem_stq_retry_e      = RegNext(UpdateBrMask(io.core.brupdate, stq_retry_e))
  val mem_ldq_e            = widthMap(w =>
                             Mux(fired_load_incoming(w), mem_ldq_incoming_e(w),
                             Mux(fired_load_retry   (w), mem_ldq_retry_e,
                             Mux(fired_load_wakeup  (w), mem_ldq_wakeup_e, (0.U).asTypeOf(Valid(new LDQEntry))))))
  val mem_stq_e            = widthMap(w =>
                             Mux(fired_stad_incoming(w) ||
                                 fired_sta_incoming (w), mem_stq_incoming_e(w),
                             Mux(fired_sta_retry    (w), mem_stq_retry_e, (0.U).asTypeOf(Valid(new STQEntry)))))
  val mem_stdf_uop         = RegNext(UpdateBrMask(io.core.brupdate, io.core.fp_stdata.bits.uop))
  val mem_vldq_lkup_e      = RegNext(UpdateBrMask(io.core.brupdate, vldq_lkup_e))
  val mem_vstq_lkup_e      = RegNext(UpdateBrMask(io.core.brupdate, vstq_lkup_e))


  val mem_tlb_miss             = RegNext(exe_tlb_miss)
  val mem_tlb_uncacheable      = RegNext(exe_tlb_uncacheable)
  val mem_paddr                = RegNext(widthMap(w => dmem_req(w).bits.addr))

  // Task 1: Clr ROB busy bit
  val clr_bsy_valid   = RegInit(widthMap(w => false.B))
  val clr_bsy_uop     = Reg(Vec(memWidth, new MicroOp))
  val clr_bsy_brmask  = Reg(Vec(memWidth, UInt(maxBrCount.W)))

  for (w <- 0 until memWidth) {
    clr_bsy_valid   (w) := false.B
    clr_bsy_uop     (w) := NullMicroOp()
    clr_bsy_brmask  (w) := 0.U


    when (fired_stad_incoming(w)) {
      clr_bsy_valid   (w) := mem_stq_incoming_e(w).valid           &&
                            !mem_tlb_miss(w)                       &&
                            !mem_stq_incoming_e(w).bits.uop.is_amo &&
                            !IsKilledByBranch(io.core.brupdate, mem_stq_incoming_e(w).bits.uop)
      clr_bsy_uop     (w) := mem_stq_incoming_e(w).bits.uop
      clr_bsy_brmask  (w) := GetNewBrMask(io.core.brupdate, mem_stq_incoming_e(w).bits.uop)
    } .elsewhen (fired_sta_incoming(w)) {
      clr_bsy_valid   (w) := mem_stq_incoming_e(w).valid            &&
                             mem_stq_incoming_e(w).bits.data.valid  &&
                            !mem_tlb_miss(w)                        &&
                            !mem_stq_incoming_e(w).bits.uop.is_amo  &&
                            !IsKilledByBranch(io.core.brupdate, mem_stq_incoming_e(w).bits.uop)
      clr_bsy_uop     (w) := mem_stq_incoming_e(w).bits.uop
      clr_bsy_brmask  (w) := GetNewBrMask(io.core.brupdate, mem_stq_incoming_e(w).bits.uop)
    } .elsewhen (fired_std_incoming(w)) {
      clr_bsy_valid   (w) := mem_stq_incoming_e(w).valid                 &&
                             mem_stq_incoming_e(w).bits.addr.valid       &&
                            !mem_stq_incoming_e(w).bits.addr_is_virtual  &&
                            !mem_stq_incoming_e(w).bits.uop.is_amo       &&
                            !IsKilledByBranch(io.core.brupdate, mem_stq_incoming_e(w).bits.uop)
      clr_bsy_uop     (w) := mem_stq_incoming_e(w).bits.uop
      clr_bsy_brmask  (w) := GetNewBrMask(io.core.brupdate, mem_stq_incoming_e(w).bits.uop)
    } .elsewhen (fired_sfence(w)) {
      clr_bsy_valid   (w) := (w == 0).B // SFence proceeds down all paths, only allow one to clr the rob
      clr_bsy_uop     (w) := mem_incoming_uop(w)
      clr_bsy_brmask  (w) := GetNewBrMask(io.core.brupdate, mem_incoming_uop(w))
    } .elsewhen (fired_sta_retry(w)) {
      clr_bsy_valid   (w) := mem_stq_retry_e.valid            &&
                             mem_stq_retry_e.bits.data.valid  &&
                            !mem_tlb_miss(w)                  &&
                            !mem_stq_retry_e.bits.uop.is_amo  &&
                            !IsKilledByBranch(io.core.brupdate, mem_stq_retry_e.bits.uop)
      clr_bsy_uop     (w) := mem_stq_retry_e.bits.uop
      clr_bsy_brmask  (w) := GetNewBrMask(io.core.brupdate, mem_stq_retry_e.bits.uop)
    } .elsewhen (fired_vstq_lookup(w)) {
      clr_bsy_valid   (w) := mem_vstq_lkup_e.valid            &&
                             mem_vstq_lkup_e.bits.data.valid  &&
                            !mem_tlb_miss(w)                  &&
                            !IsKilledByBranch(io.core.brupdate, mem_vstq_lkup_e.bits.uop)
      clr_bsy_uop     (w) := mem_vstq_lkup_e.bits.uop
      clr_bsy_brmask  (w) := GetNewBrMask(io.core.brupdate, mem_vstq_lkup_e.bits.uop)
    }

    io.core.clr_bsy(w).valid := clr_bsy_valid(w) &&
                               !IsKilledByBranch(io.core.brupdate, clr_bsy_brmask(w)) &&
                               !io.core.exception && !RegNext(io.core.exception) && !RegNext(RegNext(io.core.exception))
    io.core.clr_bsy(w).bits  := clr_bsy_uop(w)
  }

  val stdf_clr_bsy_valid   = RegInit(false.B)
  val stdf_clr_bsy_uop     = RegInit(NullMicroOp())
  val stdf_clr_bsy_brmask  = Reg(UInt(maxBrCount.W))
  stdf_clr_bsy_valid   := false.B
  stdf_clr_bsy_uop     := NullMicroOp()
  stdf_clr_bsy_brmask  := 0.U
  when (fired_stdf_incoming) {
    val s_idx = mem_stdf_uop.stq_idx
    stdf_clr_bsy_valid   := stq(s_idx).valid                 &&
                            stq(s_idx).bits.addr.valid       &&
                            !stq(s_idx).bits.addr_is_virtual &&
                            !stq(s_idx).bits.uop.is_amo      &&
                            !IsKilledByBranch(io.core.brupdate, mem_stdf_uop)
    stdf_clr_bsy_uop     := mem_stdf_uop
    stdf_clr_bsy_brmask  := GetNewBrMask(io.core.brupdate, mem_stdf_uop)
  }

  io.core.clr_bsy(memWidth).valid := stdf_clr_bsy_valid && !IsKilledByBranch(io.core.brupdate, stdf_clr_bsy_brmask) &&
                                     !io.core.exception && !RegNext(io.core.exception) && !RegNext(RegNext(io.core.exception))
  io.core.clr_bsy(memWidth).bits  := stdf_clr_bsy_uop

  // access VRF
  vrf_rarb.io.in(0).valid := will_fire_vstq_commit(memWidth-1) && vstq_commit_e.bits.uop.is_rvv
  vrf_rarb.io.in(0).bits  := vstq_commit_e.bits.uop.stale_pdst
  when (will_fire_vstq_commit(memWidth-1)) {
    assert (vrf_rarb.io.in(0).ready)
  }

  vrf_rarb.io.in(1).valid := vlagu.io.vrf_raddr.valid
  vrf_rarb.io.in(1).bits  := vlagu.io.vrf_raddr.bits

  vrf_rarb.io.in(2).valid := vsagu.io.vrf_raddr.valid
  vrf_rarb.io.in(2).bits  := vsagu.io.vrf_raddr.bits

  vrf_rarb.io.out.ready   := true.B
  io.core.vrf_rport.addr  := Mux(vrf_rarb.io.out.valid, vrf_rarb.io.out.bits, vstq_commit_e.bits.uop.stale_pdst)

  // access tile register
  if (usingMatrix) {
    io.core.tile_rport.addr  := vstq_commit_e.bits.uop.stale_pdst
    io.core.tile_rport.index := vstq_commit_e.bits.uop.m_sidx
    io.core.tile_rport.tt    := vstq_commit_e.bits.uop.rt(RD, isTrTile).asUInt ## !vstq_commit_e.bits.uop.isHSlice.asUInt
    io.core.tile_rport.msew  := vstq_commit_e.bits.uop.m_ls_ew
  }

  val vldq_resp_valid    = io.vmem.resp.valid && !io.vmem.resp.bits.is_vst
  val vldq_resp_e        = vldq(io.vmem.resp.bits.vldq_idx)
  val vle_wbk_valid      = vldq_resp_valid && vldq_resp_e.bits.uop.is_rvv
  val mle_wbk_valid      = vldq_resp_valid && vldq_resp_e.bits.uop.is_rvm
  val vldq_resp_data_shl = Wire(UInt(vLen.W))
  val vldq_resp_data_shr = Wire(UInt(vLen.W))
  vldq_resp_data_shr    := io.vmem.resp.bits.cdata >> (vldq_resp_e.bits.shamt << 3.U)
  vldq_resp_data_shl    := io.vmem.resp.bits.cdata << (vldq_resp_e.bits.shamt << 3.U)

  // vlud_vrf_q.io.brupdate        := io.core.brupdate
  // vlud_vrf_q.io.flush           := false.B // FIXME
  // vlud_vrf_q.io.enq.valid       := vlagu.io.vrf_raddr.valid && vlagu.io.vrf_rtag === 1.U
  // vlud_vrf_q.io.enq.bits.uop    := vlagu.io.resp.bits.uop
  // vlud_vrf_q.io.enq.bits.uop.pdst := vlagu.io.resp.bits.uop.pvd(vlagu.io.vrf_emul).bits
  // vlud_vrf_q.io.enq.bits.data   := 0.U
  // vlud_vrf_q.io.deq.ready       := vlud_vrf_q.io.deq.valid && vlud_dat_q.io.enq.ready && !vldq_resp_valid

  vlud_dat_q.io.brupdate          := io.core.brupdate
  vlud_dat_q.io.flush             := false.B // FIXME
  // vlud_dat_q.io.enq.valid       := vlud_vrf_q.io.deq.valid && vlud_dat_q.io.enq.ready && !vldq_resp_valid
  // vlud_dat_q.io.enq.bits.uop    := vlud_vrf_q.io.deq.bits.uop
  vlud_dat_q.io.enq.valid         := RegNext(vlagu.io.vrf_raddr.fire && vlagu.io.vrf_rtag === 1.U)
  vlud_dat_q.io.enq.bits.uop      := RegNext(vlagu.io.resp.bits.uop)
  vlud_dat_q.io.enq.bits.uop.pdst := RegNext(vlagu.io.resp.bits.uop.pvd(vlagu.io.vrf_emul).bits)
  vlud_dat_q.io.enq.bits.data     := io.core.vrf_rport.data
  vlud_dat_q.io.deq.ready         := vlud_dat_q.io.deq.valid && !vldq_resp_valid

  // VRF write back
  io.core.vrf_wbk               := DontCare
  io.core.vrf_wbk.valid         := vle_wbk_valid || vlud_dat_q.io.deq.valid
  io.core.vrf_wbk.bits.uop      := Mux(vle_wbk_valid, vldq_resp_e.bits.uop, vlud_dat_q.io.deq.bits.uop)
  io.core.vrf_wbk.bits.data     := Mux(vle_wbk_valid, Mux(vldq_resp_e.bits.shdir, vldq_resp_data_shr, vldq_resp_data_shl),
                                                      vlud_dat_q.io.deq.bits.data)
  io.core.vrf_wbk.bits.vmask    := Mux(vle_wbk_valid, vldq_resp_e.bits.vmask, Fill(vLenb, 1.U(1.W)))
  io.core.vrf_wbk.bits.predicated   := false.B
  io.core.vrf_wbk.bits.fflags.valid := false.B
  when (vlud_dat_q.io.deq.valid && !vle_wbk_valid) {
    io.core.vrf_wbk.bits.uop.uses_ldq := false.B
  }

  when (io.core.vrf_wbk.valid) {
    assert (io.core.vrf_wbk.ready)
  }
  // tile register write back
  io.core.tile_wbk              := DontCare
  io.core.tile_wbk.valid        := mle_wbk_valid
  io.core.tile_wbk.bits.uop     := vldq_resp_e.bits.uop
  io.core.tile_wbk.bits.data    := Mux(vldq_resp_e.bits.shdir, vldq_resp_data_shr, vldq_resp_data_shl)
  io.core.tile_wbk.bits.vmask   := vldq_resp_e.bits.vmask
  io.core.tile_wbk.bits.predicated   := false.B
  io.core.tile_wbk.bits.fflags.valid := false.B
  when (io.core.tile_wbk.valid) {
    assert (io.core.tile_wbk.ready)
  }

  val vldq_done = Cat(vldq.map(x => x.valid && x.bits.executed && x.bits.succeeded).reverse)
  when (vldq_done(vldq_head)) {
    vldq_head := WrapInc(vldq_head, numVLdqEntries)
  }

  for (i <- 0 until numVLdqEntries) {
    when (vldq_resp_valid && (i.U === io.vmem.resp.bits.vldq_idx)) {
      vldq(i).bits.succeeded := true.B
    }

    when (vldq_done(i) && i.U === vldq_head) {
      vldq(i).valid := false.B
      vldq(i).bits.succeeded := false.B
      vldq(i).bits.executed  := false.B
    }
  }

  when (vldq_resp_valid && ((vldq_resp_e.bits.uop.is_rvv && vldq_resp_e.bits.uop.v_split_last) ||
                            (vldq_resp_e.bits.uop.is_rvm && vldq_resp_e.bits.uop.m_split_last))) {
    ldq(vldq_resp_e.bits.uop.ldq_idx).bits.succeeded := true.B
  }

  val vstq_done = Cat(vstq.map(x => x.valid && x.bits.committed && x.bits.succeeded).reverse)
  when (vstq_done(vstq_head)) {
    vstq_head := WrapInc(vstq_head, numVStqEntries)
    when ((vstq(vstq_head).bits.uop.is_rvv && vstq(vstq_head).bits.uop.v_split_last) ||
          (vstq(vstq_head).bits.uop.is_rvm && vstq(vstq_head).bits.uop.m_split_last)) {
      val stq_idx = vstq(vstq_head).bits.uop.stq_idx
      stq(stq_idx).bits.succeeded := true.B
    }
  }

  // rvv agu
  vlagu.io.req                := DontCare
  vlagu.io.req.valid          := will_fire_vload_addrgen(0)
  vlagu.io.req.bits.uop       := ldq_vag_e.bits.uop
  vlagu.io.req.bits.rs1_data  := ldq_vag_e.bits.addr.bits
  vlagu.io.req.bits.rs2_data  := ldq_vag_e.bits.const_stride.bits
  vlagu.io.brupdate           := io.core.brupdate
  // vlagu.io.vrf_raddr.ready    := Mux(vlagu.io.vrf_rtag === 1.U, vlud_vrf_q.io.enq.ready, true.B) &&
  //                                vrf_rarb.io.in(1).ready
  vlagu.io.vrf_raddr.ready    := vrf_rarb.io.in(1).ready
  vlagu.io.vrf_rdata          := io.core.vrf_rport.data
  vlagu.io.vbusy_status       := io.core.vbusy_status

  vsagu.io.req                := DontCare
  vsagu.io.req.valid          := will_fire_vstore_addrgen(memWidth-1)
  vsagu.io.req.bits.uop       := stq_vag_e.bits.uop
  vsagu.io.req.bits.rs1_data  := stq_vag_e.bits.addr.bits
  vsagu.io.req.bits.rs2_data  := stq_vag_e.bits.const_stride.bits
  vsagu.io.brupdate           := io.core.brupdate
  vsagu.io.vrf_raddr.ready    := vrf_rarb.io.in(2).ready
  vsagu.io.vrf_rdata          := io.core.vrf_rport.data
  vsagu.io.vbusy_status       := io.core.vbusy_status

  // Task 2: Do LD-LD. ST-LD searches for ordering failures
  //         Do LD-ST search for forwarding opportunities
  // We have the opportunity to kill a request we sent last cycle. Use it wisely!

  // We translated a store last cycle
  val do_st_search = widthMap(w => (fired_stad_incoming(w) || fired_sta_incoming(w) || fired_sta_retry(w)) && !mem_tlb_miss(w))
  // We translated a load last cycle
  val do_ld_search = widthMap(w => ((fired_load_incoming(w) || fired_load_retry(w)) && !mem_tlb_miss(w)) ||
                     fired_load_wakeup(w))
  // We are making a local line visible to other harts
  val do_release_search = widthMap(w => fired_release(w))

  // Store addrs don't go to memory yet, get it from the TLB response
  // Load wakeups don't go through TLB, get it through memory
  // Load incoming and load retries go through both

  val lcam_addr  = widthMap(w => Mux(fired_stad_incoming(w) || fired_sta_incoming(w) || fired_sta_retry(w),
                                     RegNext(exe_tlb_paddr(w)),
                                     Mux(fired_release(w), RegNext(io.dmem.release.bits.address),
                                         mem_paddr(w))))
  val lcam_uop   = widthMap(w => Mux(do_st_search(w), mem_stq_e(w).bits.uop,
                                 Mux(do_ld_search(w), mem_ldq_e(w).bits.uop, NullMicroOp())))

  val lcam_mask  = widthMap(w => GenByteMask(lcam_addr(w), lcam_uop(w).mem_size))
  val lcam_st_dep_mask = widthMap(w => mem_ldq_e(w).bits.st_dep_mask)
  val lcam_is_release = widthMap(w => fired_release(w))
  val lcam_ldq_idx  = widthMap(w =>
                      Mux(fired_load_incoming(w), mem_incoming_uop(w).ldq_idx,
                      Mux(fired_load_wakeup  (w), RegNext(ldq_wakeup_idx),
                      Mux(fired_load_retry   (w), RegNext(ldq_retry_idx), 0.U))))
  val lcam_stq_idx  = widthMap(w =>
                      Mux(fired_stad_incoming(w) ||
                          fired_sta_incoming (w), mem_incoming_uop(w).stq_idx,
                      Mux(fired_sta_retry    (w), RegNext(stq_retry_idx), 0.U)))

  val can_forward = WireInit(widthMap(w =>
    Mux(fired_load_incoming(w) || fired_load_retry(w), !mem_tlb_uncacheable(w),
      !ldq(lcam_ldq_idx(w)).bits.addr_is_uncacheable)))

  // Mask of stores which we conflict on address with
  val ldst_addr_matches    = WireInit(widthMap(w => VecInit((0 until numStqEntries).map(x=>false.B))))
  // Mask of stores which we can forward from
  val ldst_forward_matches = WireInit(widthMap(w => VecInit((0 until numStqEntries).map(x=>false.B))))

  val failed_loads     = WireInit(VecInit((0 until numLdqEntries).map(x=>false.B))) // Loads which we will report as failures (throws a mini-exception)
  val nacking_loads    = WireInit(VecInit((0 until numLdqEntries).map(x=>false.B))) // Loads which are being nacked by dcache in the next stage

  val s1_executing_loads = RegNext(s0_executing_loads)
  val s1_set_execute     = WireInit(s1_executing_loads)

  val mem_forward_valid   = Wire(Vec(memWidth, Bool()))
  val mem_forward_ldq_idx = lcam_ldq_idx
  val mem_forward_ld_addr = lcam_addr
  val mem_forward_stq_idx = Wire(Vec(memWidth, UInt(log2Ceil(numStqEntries).W)))

  val wb_forward_valid    = RegNext(mem_forward_valid)
  val wb_forward_ldq_idx  = RegNext(mem_forward_ldq_idx)
  val wb_forward_ld_addr  = RegNext(mem_forward_ld_addr)
  val wb_forward_stq_idx  = RegNext(mem_forward_stq_idx)

  for (i <- 0 until numLdqEntries) {
    val l_valid = ldq(i).valid
    val l_bits  = ldq(i).bits
    val l_addr  = ldq(i).bits.addr.bits
    val l_mask  = GenByteMask(l_addr, l_bits.uop.mem_size)

    val l_forwarders      = widthMap(w => wb_forward_valid(w) && wb_forward_ldq_idx(w) === i.U)
    val l_is_forwarding   = l_forwarders.reduce(_||_)
    val l_forward_stq_idx = Mux(l_is_forwarding, Mux1H(l_forwarders, wb_forward_stq_idx), l_bits.forward_stq_idx)


    val block_addr_matches = widthMap(w => lcam_addr(w) >> blockOffBits === l_addr >> blockOffBits)
    val dword_addr_matches = widthMap(w => block_addr_matches(w) && lcam_addr(w)(blockOffBits-1,3) === l_addr(blockOffBits-1,3))
    val mask_match   = widthMap(w => (l_mask & lcam_mask(w)) === l_mask)
    val mask_overlap = widthMap(w => (l_mask & lcam_mask(w)).orR)

    // Searcher is a store
    for (w <- 0 until memWidth) {

      when (do_release_search(w) &&
            l_valid              &&
            l_bits.addr.valid    &&
            block_addr_matches(w)) {
        // This load has been observed, so if a younger load to the same address has not
        // executed yet, this load must be squashed
        ldq(i).bits.observed := true.B
      } .elsewhen (do_st_search(w)                                                                                                &&
                   l_valid                                                                                                        &&
                   l_bits.addr.valid                                                                                              &&
                   (l_bits.executed || l_bits.succeeded || l_is_forwarding)                                                       &&
                   !l_bits.addr_is_virtual                                                                                        &&
                   l_bits.st_dep_mask(lcam_stq_idx(w))                                                                            &&
                   dword_addr_matches(w)                                                                                          &&
                   mask_overlap(w)) {

        val forwarded_is_older = IsOlder(l_forward_stq_idx, lcam_stq_idx(w), l_bits.youngest_stq_idx)
        // We are older than this load, which overlapped us.
        when (!l_bits.forward_std_val || // If the load wasn't forwarded, it definitely failed
          ((l_forward_stq_idx =/= lcam_stq_idx(w)) && forwarded_is_older)) { // If the load forwarded from us, we might be ok
          ldq(i).bits.order_fail := true.B
          failed_loads(i)        := true.B
        }
      } .elsewhen (do_ld_search(w)            &&
                   l_valid                    &&
                   l_bits.addr.valid          &&
                   !l_bits.addr_is_virtual    &&
                   dword_addr_matches(w)      &&
                   mask_overlap(w)) {
        val searcher_is_older = IsOlder(lcam_ldq_idx(w), i.U, ldq_head)
        when (searcher_is_older) {
          when ((l_bits.executed || l_bits.succeeded || l_is_forwarding) &&
                !s1_executing_loads(i) && // If the load is proceeding in parallel we don't need to kill it
                l_bits.observed) {        // Its only a ordering failure if the cache line was observed between the younger load and us
            ldq(i).bits.order_fail := true.B
            failed_loads(i)        := true.B
          }
        } .elsewhen (lcam_ldq_idx(w) =/= i.U) {
          // The load is older, and either it hasn't executed, it was nacked, or it is ignoring its response
          // we need to kill ourselves, and prevent forwarding
          val older_nacked = nacking_loads(i) || RegNext(nacking_loads(i))
          when (!(l_bits.executed || l_bits.succeeded) || older_nacked) {
            s1_set_execute(lcam_ldq_idx(w))    := false.B
            io.dmem.s1_kill(w)                 := RegNext(dmem_req_fire(w))
            can_forward(w)                     := false.B
          }
        }
      }
    }
  }

  for (i <- 0 until numStqEntries) {
    val s_addr = stq(i).bits.addr.bits
    val s_uop  = stq(i).bits.uop
    val dword_addr_matches = widthMap(w =>
                             ( stq(i).bits.addr.valid      &&
                              !stq(i).bits.addr_is_virtual &&
                              (s_addr(corePAddrBits-1,3) === lcam_addr(w)(corePAddrBits-1,3))))
    val write_mask = GenByteMask(s_addr, s_uop.mem_size)
    for (w <- 0 until memWidth) {
      when (do_ld_search(w) && stq(i).valid && lcam_st_dep_mask(w)(i)) {
        when (((lcam_mask(w) & write_mask) === lcam_mask(w)) && !s_uop.is_fence && dword_addr_matches(w) && can_forward(w))
        {
          ldst_addr_matches(w)(i)            := true.B
          ldst_forward_matches(w)(i)         := true.B
          io.dmem.s1_kill(w)                 := RegNext(dmem_req_fire(w))
          s1_set_execute(lcam_ldq_idx(w))    := false.B
        }
          .elsewhen (((lcam_mask(w) & write_mask) =/= 0.U) && dword_addr_matches(w))
        {
          ldst_addr_matches(w)(i)            := true.B
          io.dmem.s1_kill(w)                 := RegNext(dmem_req_fire(w))
          s1_set_execute(lcam_ldq_idx(w))    := false.B
        }
          .elsewhen (s_uop.is_fence || s_uop.is_amo)
        {
          ldst_addr_matches(w)(i)            := true.B
          io.dmem.s1_kill(w)                 := RegNext(dmem_req_fire(w))
          s1_set_execute(lcam_ldq_idx(w))    := false.B
        }
      }
    }
  }

  // Set execute bit in LDQ
  for (i <- 0 until numLdqEntries) {
    when (s1_set_execute(i)) { ldq(i).bits.executed := true.B }
  }

  // Find the youngest store which the load is dependent on
  val forwarding_age_logic = Seq.fill(memWidth) { Module(new ForwardingAgeLogic(numStqEntries)) }
  for (w <- 0 until memWidth) {
    forwarding_age_logic(w).io.addr_matches    := ldst_addr_matches(w).asUInt
    forwarding_age_logic(w).io.youngest_st_idx := lcam_uop(w).stq_idx
  }
  val forwarding_idx = widthMap(w => forwarding_age_logic(w).io.forwarding_idx)

  // Forward if st-ld forwarding is possible from the writemask and loadmask
  mem_forward_valid       := widthMap(w =>
                                  (ldst_forward_matches(w)(forwarding_idx(w))        &&
                                 !IsKilledByBranch(io.core.brupdate, lcam_uop(w))    &&
                                 !io.core.exception && !RegNext(io.core.exception)))
  mem_forward_stq_idx     := forwarding_idx

  // Avoid deadlock with a 1-w LSU prioritizing load wakeups > store commits
  // On a 2W machine, load wakeups and store commits occupy separate pipelines,
  // so only add this logic for 1-w LSU
  if (memWidth == 1) {
    // Wakeups may repeatedly find a st->ld addr conflict and fail to forward,
    // repeated wakeups may block the store from ever committing
    // Disallow load wakeups 1 cycle after this happens to allow the stores to drain
    when (RegNext(ldst_addr_matches(0).reduce(_||_) && !mem_forward_valid(0))) {
      block_load_wakeup := true.B
    }

    // If stores remain blocked for 15 cycles, block load wakeups to get a store through
    val store_blocked_counter = Reg(UInt(4.W))
    when (will_fire_store_commit(0) || !can_fire_store_commit(0)) {
      store_blocked_counter := 0.U
    } .elsewhen (can_fire_store_commit(0) && !will_fire_store_commit(0)) {
      store_blocked_counter := Mux(store_blocked_counter === 15.U, store_blocked_counter + 1.U, 15.U)
    }
    when (store_blocked_counter === 15.U) {
      block_load_wakeup := true.B
    }
  }


  // Task 3: Clr unsafe bit in ROB for succesful translations
  //         Delay this a cycle to avoid going ahead of the exception broadcast
  //         The unsafe bit is cleared on the first translation, so no need to fire for load wakeups
  for (w <- 0 until memWidth) {
    io.core.clr_unsafe(w).valid := RegNext((do_st_search(w) || do_ld_search(w)) && !fired_load_wakeup(w)) && false.B
    io.core.clr_unsafe(w).bits  := RegNext(lcam_uop(w))
  }

  // detect which loads get marked as failures, but broadcast to the ROB the oldest failing load
  // TODO encapsulate this in an age-based  priority-encoder
  //   val l_idx = AgePriorityEncoder((Vec(Vec.tabulate(numLdqEntries)(i => failed_loads(i) && i.U >= laq_head)
  //   ++ failed_loads)).asUInt)
  val temp_bits = (VecInit(VecInit.tabulate(numLdqEntries)(i =>
    failed_loads(i) && i.U >= ldq_head) ++ failed_loads)).asUInt
  val l_idx = PriorityEncoder(temp_bits)

  // one exception port, but multiple causes!
  // - 1) the incoming store-address finds a faulting load (it is by definition younger)
  // - 2) the incoming load or store address is excepting. It must be older and thus takes precedent.
  val r_xcpt_valid = RegInit(false.B)
  val r_xcpt       = Reg(new Exception)

  val ld_xcpt_valid = failed_loads.reduce(_|_)
  val ld_xcpt_uop   = ldq(Mux(l_idx >= numLdqEntries.U, l_idx - numLdqEntries.U, l_idx)).bits.uop

  val use_mem_xcpt = (mem_xcpt_valid && IsOlder(mem_xcpt_uop.rob_idx, ld_xcpt_uop.rob_idx, io.core.rob_head_idx)) || !ld_xcpt_valid

  val xcpt_uop = Mux(use_mem_xcpt, mem_xcpt_uop, ld_xcpt_uop)

  r_xcpt_valid := (ld_xcpt_valid || mem_xcpt_valid) &&
                   !io.core.exception &&
                   !IsKilledByBranch(io.core.brupdate, xcpt_uop)
  r_xcpt.uop         := xcpt_uop
  r_xcpt.uop.br_mask := GetNewBrMask(io.core.brupdate, xcpt_uop)
  r_xcpt.cause       := Mux(use_mem_xcpt, mem_xcpt_cause, MINI_EXCEPTION_MEM_ORDERING)
  r_xcpt.badvaddr    := mem_xcpt_vaddr // TODO is there another register we can use instead?

  io.core.lxcpt.valid := r_xcpt_valid && !io.core.exception && !IsKilledByBranch(io.core.brupdate, r_xcpt.uop)
  io.core.lxcpt.bits  := r_xcpt

  // Task 4: Speculatively wakeup loads 1 cycle before they come back
  for (w <- 0 until memWidth) {
    if (usingVector) {
      io.core.spec_ld_wakeup(w).valid := enableFastLoadUse.B             &&
                                         fired_load_incoming(w)          &&
                                         !mem_incoming_uop(w).fp_val     &&
                                         !mem_incoming_uop(w).is_vm_ext  &&
                                         mem_incoming_uop(w).pdst =/= 0.U
    } else {
      io.core.spec_ld_wakeup(w).valid := enableFastLoadUse.B          &&
                                         fired_load_incoming(w)       &&
                                         !mem_incoming_uop(w).fp_val  &&
                                         mem_incoming_uop(w).pdst =/= 0.U
    }
    io.core.spec_ld_wakeup(w).bits  := mem_incoming_uop(w).pdst
  }


  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // Writeback Cycle (St->Ld Forwarding Path)
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // Handle Memory Responses and nacks
  //----------------------------------
  for (w <- 0 until memWidth) {
    io.core.exe(w).iresp.valid := false.B
    io.core.exe(w).fresp.valid := false.B
    //if (usingVector) io.core.exe(w).vresp.valid := false.B
  }

  val dmem_resp_fired = WireInit(widthMap(w => false.B))

  for (w <- 0 until memWidth) {
    // Handle nacks
    when (io.dmem.nack(w).valid)
    {
      // We have to re-execute this!
      when (io.dmem.nack(w).bits.is_hella)
      {
        assert(hella_state === h_wait || hella_state === h_dead)
      }
        .elsewhen (io.dmem.nack(w).bits.uop.uses_ldq)
      {
        assert(ldq(io.dmem.nack(w).bits.uop.ldq_idx).bits.executed)
        ldq(io.dmem.nack(w).bits.uop.ldq_idx).bits.executed  := false.B
        nacking_loads(io.dmem.nack(w).bits.uop.ldq_idx) := true.B
      }
        .otherwise
      {
        assert(io.dmem.nack(w).bits.uop.uses_stq)
        when (IsOlder(io.dmem.nack(w).bits.uop.stq_idx, stq_execute_head, stq_head)) {
          stq_execute_head := io.dmem.nack(w).bits.uop.stq_idx
        }
      }
    }
    // Handle the response
    when (io.dmem.resp(w).valid)
    {
      when (io.dmem.resp(w).bits.uop.uses_ldq)
      {
        assert(!io.dmem.resp(w).bits.is_hella)
        val ldq_idx = io.dmem.resp(w).bits.uop.ldq_idx
        val send_iresp = ldq(ldq_idx).bits.uop.rt(RD, isInt)
        val send_fresp = ldq(ldq_idx).bits.uop.rt(RD, isFloat)
        //val send_vresp = ldq(ldq_idx).bits.uop.rt(RD, isVector)

        io.core.exe(w).iresp.bits.uop  := ldq(ldq_idx).bits.uop
        io.core.exe(w).fresp.bits.uop  := ldq(ldq_idx).bits.uop
        io.core.exe(w).iresp.valid     := send_iresp
        io.core.exe(w).iresp.bits.data := io.dmem.resp(w).bits.data
        io.core.exe(w).fresp.valid     := send_fresp
        io.core.exe(w).fresp.bits.data := io.dmem.resp(w).bits.data
        //if (usingVector) {
          //io.core.exe(w).vresp.valid     := send_vresp
          //io.core.exe(w).vresp.bits.uop  := ldq(ldq_idx).bits.uop
          //io.core.exe(w).vresp.bits.data := io.dmem.resp(w).bits.data
        //}

        assert(send_iresp ^ send_fresp)
        dmem_resp_fired(w) := true.B

        ldq(ldq_idx).bits.succeeded      := true.B
        ldq(ldq_idx).bits.debug_wb_data  := io.dmem.resp(w).bits.data
      }
        .elsewhen (io.dmem.resp(w).bits.uop.uses_stq)
      {
        assert(!io.dmem.resp(w).bits.is_hella)
        stq(io.dmem.resp(w).bits.uop.stq_idx).bits.succeeded := true.B
        when (io.dmem.resp(w).bits.uop.is_amo) {
          dmem_resp_fired(w) := true.B
          io.core.exe(w).iresp.valid     := true.B
          io.core.exe(w).iresp.bits.uop  := stq(io.dmem.resp(w).bits.uop.stq_idx).bits.uop
          io.core.exe(w).iresp.bits.data := io.dmem.resp(w).bits.data

          stq(io.dmem.resp(w).bits.uop.stq_idx).bits.debug_wb_data := io.dmem.resp(w).bits.data
        }
      }
    }


    when (dmem_resp_fired(w) && wb_forward_valid(w))
    {
      // Twiddle thumbs. Can't forward because dcache response takes precedence
    }
      .elsewhen (!dmem_resp_fired(w) && wb_forward_valid(w))
    {
      val f_idx       = wb_forward_ldq_idx(w)
      val forward_uop = ldq(f_idx).bits.uop
      val stq_e       = stq(wb_forward_stq_idx(w))
      val data_ready  = stq_e.bits.data.valid
      val live        = !IsKilledByBranch(io.core.brupdate, forward_uop)
      val storegen = new freechips.rocketchip.rocket.StoreGen(
                                stq_e.bits.uop.mem_size, stq_e.bits.addr.bits,
                                stq_e.bits.data.bits, coreDataBytes)
      val loadgen  = new freechips.rocketchip.rocket.LoadGen(
                                forward_uop.mem_size, forward_uop.mem_signed,
                                wb_forward_ld_addr(w),
                                storegen.data, false.B, coreDataBytes)

      io.core.exe(w).iresp.valid := (forward_uop.rt(RD, isInt)) && data_ready && live
      io.core.exe(w).fresp.valid := (forward_uop.rt(RD, isFloat)) && data_ready && live
      io.core.exe(w).iresp.bits.uop  := forward_uop
      io.core.exe(w).fresp.bits.uop  := forward_uop
      io.core.exe(w).iresp.bits.data := loadgen.data
      io.core.exe(w).fresp.bits.data := loadgen.data
      //if (usingVector) {
        //io.core.exe(w).vresp.valid := (forward_uop.rt(RD, isVector)) && data_ready && live
        //io.core.exe(w).vresp.bits.uop  := forward_uop
        //io.core.exe(w).vresp.bits.data := loadgen.data
      //}

      when (data_ready && live) {
        ldq(f_idx).bits.succeeded := data_ready
        ldq(f_idx).bits.forward_std_val := true.B
        ldq(f_idx).bits.forward_stq_idx := wb_forward_stq_idx(w)

        ldq(f_idx).bits.debug_wb_data   := loadgen.data
      }
    }
  }

  // Initially assume the speculative load wakeup failed
  io.core.ld_miss         := RegNext(io.core.spec_ld_wakeup.map(_.valid).reduce(_||_))
  val spec_ld_succeed = widthMap(w =>
    !RegNext(io.core.spec_ld_wakeup(w).valid) ||
    (io.core.exe(w).iresp.valid &&
      io.core.exe(w).iresp.bits.uop.ldq_idx === RegNext(mem_incoming_uop(w).ldq_idx)
    )
  ).reduce(_&&_)
  when (spec_ld_succeed) {
    io.core.ld_miss := false.B
  }


  //-------------------------------------------------------------
  // Kill speculated entries on branch mispredict
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // Kill stores
  val st_brkilled_mask = Wire(Vec(numStqEntries, Bool()))
  for (i <- 0 until numStqEntries)
  {
    st_brkilled_mask(i) := false.B

    when (stq(i).valid)
    {
      stq(i).bits.uop.br_mask := GetNewBrMask(io.core.brupdate, stq(i).bits.uop.br_mask)

      when (IsKilledByBranch(io.core.brupdate, stq(i).bits.uop))
      {
        stq(i).valid           := false.B
        stq(i).bits.addr.valid := false.B
        stq(i).bits.data.valid := false.B
        st_brkilled_mask(i)    := true.B
      }
    }

    assert (!(IsKilledByBranch(io.core.brupdate, stq(i).bits.uop) && stq(i).valid && stq(i).bits.committed),
      "Branch is trying to clear a committed store.")
  }

  // Kill loads
  for (i <- 0 until numLdqEntries)
  {
    when (ldq(i).valid)
    {
      ldq(i).bits.uop.br_mask := GetNewBrMask(io.core.brupdate, ldq(i).bits.uop.br_mask)
      when (IsKilledByBranch(io.core.brupdate, ldq(i).bits.uop))
      {
        ldq(i).valid           := false.B
        ldq(i).bits.addr.valid := false.B
      }
    }
  }

  //-------------------------------------------------------------
  when (io.core.brupdate.b2.mispredict && !io.core.exception)
  {
    stq_tail := io.core.brupdate.b2.uop.stq_idx
    ldq_tail := io.core.brupdate.b2.uop.ldq_idx
  }

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // dequeue old entries on commit
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  var temp_stq_commit_head = stq_commit_head
  var temp_ldq_head        = ldq_head
  for (w <- 0 until coreWidth)
  {
    val commit_store = io.core.commit.valids(w) && io.core.commit.uops(w).uses_stq
    val commit_load  = io.core.commit.valids(w) && io.core.commit.uops(w).uses_ldq
    val idx = Mux(commit_store, temp_stq_commit_head, temp_ldq_head)
    when (commit_store)
    {
      stq(idx).bits.committed := true.B

      for (i <- 0 until numVStqEntries)
      {
        when (vstq(i).valid && vstq(i).bits.uop.stq_idx === idx) {
          vstq(i).bits.committed  := true.B
        }
      }
    } .elsewhen (commit_load) {
      assert (ldq(idx).valid, "[lsu] trying to commit an un-allocated load entry.")
      assert ((ldq(idx).bits.executed || ldq(idx).bits.forward_std_val) && ldq(idx).bits.succeeded,
        "[lsu] trying to commit an un-executed load entry.")

      ldq(idx).valid                 := false.B
      ldq(idx).bits.addr.valid       := false.B
      ldq(idx).bits.executed         := false.B
      ldq(idx).bits.succeeded        := false.B
      ldq(idx).bits.order_fail       := false.B
      ldq(idx).bits.forward_std_val  := false.B

    }

    if (MEMTRACE_PRINTF) {
      when (commit_store || commit_load) {
        val uop    = Mux(commit_store, stq(idx).bits.uop, ldq(idx).bits.uop)
        val addr   = Mux(commit_store, stq(idx).bits.addr.bits, ldq(idx).bits.addr.bits)
        val stdata = Mux(commit_store, stq(idx).bits.data.bits, 0.U)
        val wbdata = Mux(commit_store, stq(idx).bits.debug_wb_data, ldq(idx).bits.debug_wb_data)
        printf("MT %x %x %x %x %x %x %x\n",
          io.core.tsc_reg, uop.uopc, uop.mem_cmd, uop.mem_size, addr, stdata, wbdata)
      }
    }

    temp_stq_commit_head = Mux(commit_store,
                               WrapInc(temp_stq_commit_head, numStqEntries),
                               temp_stq_commit_head)

    temp_ldq_head        = Mux(commit_load,
                               WrapInc(temp_ldq_head, numLdqEntries),
                               temp_ldq_head)
  }
  stq_commit_head := temp_stq_commit_head
  ldq_head        := temp_ldq_head

  // store has been committed AND successfully sent data to memory
  when (stq(stq_head).valid && stq(stq_head).bits.committed)
  {
    when (stq(stq_head).bits.uop.is_fence && !io.dmem.ordered) {
      io.dmem.force_order := true.B
      store_needs_order   := true.B
    }
    clear_store := Mux(stq(stq_head).bits.uop.is_fence, io.dmem.ordered,
                                                        stq(stq_head).bits.succeeded)
  }

  when (clear_store)
  {
    stq(stq_head).valid           := false.B
    stq(stq_head).bits.addr.valid := false.B
    stq(stq_head).bits.data.valid := false.B
    stq(stq_head).bits.succeeded  := false.B
    stq(stq_head).bits.committed  := false.B

    stq_head := WrapInc(stq_head, numStqEntries)
    when (stq(stq_head).bits.uop.is_fence)
    {
      stq_execute_head := WrapInc(stq_execute_head, numStqEntries)
    }
  }


  // -----------------------
  // Hellacache interface
  // We need to time things like a HellaCache would
  io.hellacache.req.ready := false.B
  io.hellacache.s2_nack   := false.B
  io.hellacache.s2_xcpt   := (0.U).asTypeOf(new rocket.HellaCacheExceptions)
  io.hellacache.resp.valid := false.B
  when (hella_state === h_ready) {
    io.hellacache.req.ready := true.B
    when (io.hellacache.req.fire()) {
      hella_req   := io.hellacache.req.bits
      hella_state := h_s1
    }
  } .elsewhen (hella_state === h_s1) {
    can_fire_hella_incoming(memWidth-1) := true.B

    hella_data := io.hellacache.s1_data
    hella_xcpt := dtlb.io.resp(memWidth-1)

    when (io.hellacache.s1_kill) {
      when (will_fire_hella_incoming(memWidth-1) && dmem_req_fire(memWidth-1)) {
        hella_state := h_dead
      } .otherwise {
        hella_state := h_ready
      }
    } .elsewhen (will_fire_hella_incoming(memWidth-1) && dmem_req_fire(memWidth-1)) {
      hella_state := h_s2
    } .otherwise {
      hella_state := h_s2_nack
    }
  } .elsewhen (hella_state === h_s2_nack) {
    io.hellacache.s2_nack := true.B
    hella_state := h_ready
  } .elsewhen (hella_state === h_s2) {
    io.hellacache.s2_xcpt := hella_xcpt
    when (io.hellacache.s2_kill || hella_xcpt.asUInt =/= 0.U) {
      hella_state := h_dead
    } .otherwise {
      hella_state := h_wait
    }
  } .elsewhen (hella_state === h_wait) {
    for (w <- 0 until memWidth) {
      when (io.dmem.resp(w).valid && io.dmem.resp(w).bits.is_hella) {
        hella_state := h_ready

        io.hellacache.resp.valid       := true.B
        io.hellacache.resp.bits.addr   := hella_req.addr
        io.hellacache.resp.bits.tag    := hella_req.tag
        io.hellacache.resp.bits.cmd    := hella_req.cmd
        io.hellacache.resp.bits.signed := hella_req.signed
        io.hellacache.resp.bits.size   := hella_req.size
        io.hellacache.resp.bits.data   := io.dmem.resp(w).bits.data
      } .elsewhen (io.dmem.nack(w).valid && io.dmem.nack(w).bits.is_hella) {
        hella_state := h_replay
      }
    }
  } .elsewhen (hella_state === h_replay) {
    can_fire_hella_wakeup(memWidth-1) := true.B

    when (will_fire_hella_wakeup(memWidth-1) && dmem_req_fire(memWidth-1)) {
      hella_state := h_wait
    }
  } .elsewhen (hella_state === h_dead) {
    for (w <- 0 until memWidth) {
      when (io.dmem.resp(w).valid && io.dmem.resp(w).bits.is_hella) {
        hella_state := h_ready
      }
    }
  }

  //-------------------------------------------------------------
  // Exception / Reset

  // for the live_store_mask, need to kill stores that haven't been committed
  val st_exc_killed_mask = WireInit(VecInit((0 until numStqEntries).map(x=>false.B)))

  when (reset.asBool || io.core.exception)
  {
    ldq_head := 0.U
    ldq_tail := 0.U

    for (i <- 0 until numLdqEntries)
    {
      ldq(i).valid           := false.B
      ldq(i).bits.addr.valid := false.B
      ldq(i).bits.executed   := false.B
    }

    vldq_head := 0.U
    vldq_tail := 0.U

    for (i <- 0 until numVLdqEntries)
    {
      vldq(i).valid           := false.B
      vldq(i).bits.addr.valid := false.B
      vldq(i).bits.executed   := false.B
      vldq(i).bits.succeeded  := false.B
    }

    when (reset.asBool)
    {
      stq_head := 0.U
      stq_tail := 0.U
      stq_commit_head  := 0.U
      stq_execute_head := 0.U

      for (i <- 0 until numStqEntries)
      {
        stq(i).valid           := false.B
        stq(i).bits.addr.valid := false.B
        stq(i).bits.data.valid := false.B
        stq(i).bits.uop        := NullMicroOp()
      }

      vstq_head := 0.U
      vstq_tail := 0.U
      //vstq_commit_head  := 0.U
      vstq_execute_head := 0.U

      for (i <- 0 until numVStqEntries)
      {
        vstq(i).valid           := false.B
        vstq(i).bits.addr.valid := false.B
        vstq(i).bits.committed  := false.B
        vstq(i).bits.succeeded  := false.B
      }
    }
      .otherwise // exception
    {
      stq_tail := stq_commit_head

      for (i <- 0 until numStqEntries)
      {
        when (!stq(i).bits.committed && !stq(i).bits.succeeded)
        {
          stq(i).valid           := false.B
          stq(i).bits.addr.valid := false.B
          stq(i).bits.data.valid := false.B
          st_exc_killed_mask(i)  := true.B
        }
      }
    }

  }

  //-------------------------------------------------------------
  // Live Store Mask
  // track a bit-array of stores that are alive
  // (could maybe be re-produced from the stq_head/stq_tail, but need to know include spec_killed entries)

  // TODO is this the most efficient way to compute the live store mask?
  live_store_mask := next_live_store_mask &
                    ~(st_brkilled_mask.asUInt) &
                    ~(st_exc_killed_mask.asUInt)


}

/**
 * Object to take an address and generate an 8-bit mask of which bytes within a
 * double-word.
 */
object GenByteMask
{
   def apply(addr: UInt, size: UInt): UInt =
   {
      val mask = Wire(UInt(8.W))
      mask := MuxCase(255.U(8.W), Array(
                   (size === 0.U) -> (1.U(8.W) << addr(2,0)),
                   (size === 1.U) -> (3.U(8.W) << (addr(2,1) << 1.U)),
                   (size === 2.U) -> Mux(addr(2), 240.U(8.W), 15.U(8.W)),
                   (size === 3.U) -> 255.U(8.W)))
      mask
   }
}

/**
 * ...
 */
class ForwardingAgeLogic(num_entries: Int)(implicit p: Parameters) extends BoomModule()(p)
{
   val io = IO(new Bundle
   {
      val addr_matches    = Input(UInt(num_entries.W)) // bit vector of addresses that match
                                                       // between the load and the SAQ
      val youngest_st_idx = Input(UInt(stqAddrSz.W)) // needed to get "age"

      val forwarding_val  = Output(Bool())
      val forwarding_idx  = Output(UInt(stqAddrSz.W))
   })

   // generating mask that zeroes out anything younger than tail
   val age_mask = Wire(Vec(num_entries, Bool()))
   for (i <- 0 until num_entries)
   {
      age_mask(i) := true.B
      when (i.U >= io.youngest_st_idx) // currently the tail points PAST last store, so use >=
      {
         age_mask(i) := false.B
      }
   }

   // Priority encoder with moving tail: double length
   val matches = Wire(UInt((2*num_entries).W))
   matches := Cat(io.addr_matches & age_mask.asUInt,
                  io.addr_matches)

   val found_match = Wire(Bool())
   found_match       := false.B
   io.forwarding_idx := 0.U

   // look for youngest, approach from the oldest side, let the last one found stick
   for (i <- 0 until (2*num_entries))
   {
      when (matches(i))
      {
         found_match := true.B
         io.forwarding_idx := (i % num_entries).U
      }
   }

   io.forwarding_val := found_match
}

class VecLSAddrGenUnit(implicit p: Parameters) extends BoomModule()(p)
{
  val io = IO(new Bundle {
    val req       = Flipped(new DecoupledIO(new FuncUnitReq(xLen)))
    val resp      = (new DecoupledIO(new FuncUnitResp(xLen)))
    val resp_vm   = Output(UInt(vLenb.W))
    val resp_shdir= Output(Bool()) // 0: vl:left/vs:right, 1: vl:right/vs:left
    val resp_shamt= Output(UInt(log2Ceil(vLenb.max(p(freechips.rocketchip.subsystem.CacheBlockBytes))).W))
    val brupdate  = Input(new BrUpdateInfo())
    val busy      = Output(Bool())
    val vrf_rtag  = Output(UInt(2.W)) // 0:idle, 1:ud, 2:vm, 3:idx
    val vrf_raddr = new DecoupledIO(UInt(vpregSz.W))
    val vrf_rdata = Input(UInt(vLen.W))
    val vrf_emul  = Output(UInt(3.W))
    val vbusy_status = if (usingVector) Input(UInt(numVecPhysRegs.W)) else null
    //val vrf_mask  = Output(UInt(log2Ceil(vLenb).W)) // FIXME: sends vm for masked ud-copy
  })

  val clSize = p(freechips.rocketchip.subsystem.CacheBlockBytes)
  val clSizeLog2 = log2Up(clSize)
  val vcRatio    = if(vLenb > clSize) vLenb/clSize      else 1
  val vcRatioSz  = if(vLenb > clSize) log2Ceil(vcRatio) else 1
  require(isPow2(vcRatio))

  val s_idle :: s_udcpy :: s_vmask :: s_index :: s_split :: s_slice :: Nil = Enum(6)
  val state = RegInit(s_idle)

  val emulCtr  = RegInit(0.U(4.W))
  val vmask    = Reg(UInt(vLen.W))
  val vmaskSel = WireInit(0.U(vLenb.W))
  val eindex   = Reg(Vec(8, UInt(vLen.W)))
  // val rdata    = if(vLenb <= clSize) Reg(Vec(8, UInt(vLen.W))) else null
  val ioUop    = io.req.bits.uop
  val req      = RegEnable(io.req.bits, io.req.fire)
  val uop      = req.uop
  val eew      = uop.vd_eew
  val vLenECnt = vLenb.U >> eew
  val emul     = uop.vd_emul
  val op1      = req.rs1_data                      // base address
  val op2      = Mux(uop.is_rvm, req.rs2_data,     // row strides in mle and mse
                 Mux(uop.v_idx_ls, VDataSel(Cat(eindex.reverse), uop.vs2_eew, uop.v_eidx, vLen*8, eLen), 0.U))
  val clOffset = if(vLenb > clSize) RegEnable(io.req.bits.rs1_data(clSizeLog2-1, 0), io.req.valid) 
                 else               op1(clSizeLog2-1, 0)
  val isUnitStride = uop.uopc.isOneOf(uopVL, uopVLFF, uopVSA)
  val usSplitCtr = RegInit(0.U((vcRatioSz+1).W))
  val addrInc    = WireInit(0.U(xLen.W))
  val eidxInc    = WireInit(0.U(vLenSz.W))
  val usSplitLeftCnt = RegInit(0.U(vLenSz.W))
  if (vLenb > clSize) {
    addrInc := Mux(uop.v_idx_ls, 0.U,
               Mux(uop.uopc.isOneOf(uopVLS, uopVSSA), req.rs2_data,
               Mux(usSplitCtr === 0.U, clSize.U - clOffset,
               Mux(usSplitCtr === vcRatio.U, clOffset, clSize.U))))
    eidxInc := Mux(!isUnitStride, 1.U,
               Mux(usSplitCtr === 0.U, (clSize.U - clOffset) >> eew,
               Mux(usSplitCtr === vcRatio.U, clOffset >> eew, clSize.U >> eew)))
  } else {
    addrInc := Mux(uop.v_idx_ls, 0.U,
               Mux(uop.uopc.isOneOf(uopVLS, uopVSSA), req.rs2_data,
               Mux(usSplitCtr === 1.U, usSplitLeftCnt,
                                       vLenb.U.min(clSize.U - clOffset))))
    eidxInc := Mux(!isUnitStride, 1.U,
               Mux(usSplitCtr === 1.U, usSplitLeftCnt >> eew,
                                       vLenb.U.min(clSize.U - clOffset) >> eew))
  }
  // appended for mle and mse control
  val sliceCntCtr    = RegInit(0.U(vLenbSz.W))
  val sliceLenCtr    = RegInit(0.U((vcRatioSz+1).W))
  val splitCnt       = RegInit(0.U(vLenSz.W))
  val sliceBaseAddr  = RegInit(0.U(xLen.W))
  val sliceBlockAddr = RegInit(0.U(xLen.W))
  val sliceBlockOff  = sliceBaseAddr(clSizeLog2-1, 0)
  val sliceAddrInc   = WireInit(0.U((clSizeLog2+1).W))
  val sliceLenLast   = WireInit(false.B)
  if (vLenb > clSize) {
    sliceAddrInc := Mux(sliceLenCtr === 0.U,       clSize.U - sliceBlockOff,
                    Mux(sliceLenCtr === vcRatio.U, sliceBlockOff, clSize.U))
    sliceLenLast := sliceLenCtr + 1.U === vcRatio.U + (sliceBlockOff =/= 0.U).asUInt
  } else {
    sliceAddrInc := Mux(sliceLenCtr === 0.U, (clSize.U - sliceBlockOff).min(vLenb.U),
                                             RegNext(sliceBlockOff) +& vLenb.U - clSize.U)
    sliceLenLast := sliceBlockOff <= (clSize-vLenb).asUInt
  }

  when (io.req.valid) {
    assert(ioUop.is_vm_ext && (ioUop.uses_ldq || ioUop.uses_stq))
    assert(state === s_idle)
  }

  // FIXME: handle segment ls
  if (vLenb > clSize) 
  {
    switch(state)
    {
      is (s_idle) {
        when (io.req.valid) {
          val ioAligned = io.req.bits.rs1_data(clSizeLog2-1, 0) === 0.U && ioUop.vstart === 0.U &&
                          ((ioUop.vconfig.vl & (0x3F.U >> ioUop.vd_eew)) === 0.U)
          emulCtr        := 0.U
          sliceCntCtr    := 0.U
          sliceBaseAddr  := Mux(ioUop.is_rvm, io.req.bits.rs1_data, 0.U)
          sliceBlockAddr := 0.U
          splitCnt       := 0.U
          state := Mux(ioUop.is_rvm, s_slice,
                   Mux(!ioAligned || !ioUop.v_unmasked, s_udcpy, // does aligned idx ls perform ud copy?
                   Mux(!ioUop.v_unmasked, s_vmask,
                   Mux(ioUop.v_idx_ls, s_index, s_split))))
        }
      }
      is (s_udcpy) {
        when (io.vrf_raddr.fire) {
          emulCtr := emulCtr + 1.U
          when (emulCtr + 1.U === nrVecGroup(emul, uop.v_seg_nf)) {
            emulCtr := 0.U
            state := Mux(!uop.v_unmasked, s_vmask,
                     Mux(uop.v_idx_ls, s_index, s_split))
          }
        }
      }
      is (s_vmask) {
        when (io.vrf_raddr.fire) {
          state := Mux(uop.v_idx_ls, s_index, s_split)
        }
      }
      is (s_index) {
        when (io.vrf_raddr.fire) {
          emulCtr := emulCtr + 1.U
          when (emulCtr + 1.U === (1.U << emul)) {
            emulCtr := 0.U
            state := s_split
          }
        }
      }
      is (s_split) {
        when (io.resp.fire) {
          op1 := op1 + addrInc
          uop.v_eidx := uop.v_eidx + eidxInc
          when (isUnitStride) {
            usSplitCtr := usSplitCtr + 1.U
            when (usSplitCtr + 1.U === vcRatio.U + (clOffset =/= 0.U).asUInt) {
              usSplitCtr := 0.U
              emulCtr := emulCtr + 1.U
            }
          }.otherwise {
            when (uop.v_eidx + 1.U === vLenECnt) {
              emulCtr := emulCtr + 1.U
            }
          }
          when (uop.v_eidx +& eidxInc >= uop.vconfig.vl) {
            emulCtr := 0.U
            state := s_idle
          }
        }
      }
      is (s_slice) {
        when (io.resp.fire) {
          sliceLenCtr      := sliceLenCtr + 1.U
          sliceBlockAddr   := sliceBlockAddr + sliceAddrInc
          splitCnt         := splitCnt + 1.U
          when (sliceLenLast) {
            sliceLenCtr    := 0.U
            sliceCntCtr    := sliceCntCtr + 1.U
            sliceBaseAddr  := sliceBaseAddr + op2
            sliceBlockAddr := 0.U
            when (sliceCntCtr +& 1.U === uop.m_slice_cnt) {
              sliceCntCtr  := 0.U
              state        := s_idle
            }
          }
        }
      }
    }
  } else {
    switch(state)
    {
      is (s_idle) {
        when (io.req.valid) {
          val ioAligned = io.req.bits.rs1_data(clSizeLog2-1, 0) === 0.U && ioUop.vstart === 0.U &&
                          ((ioUop.vconfig.vl & (0x3F.U >> ioUop.vd_eew)) === 0.U)
          emulCtr        := 0.U
          sliceCntCtr    := 0.U
          sliceBaseAddr  := Mux(ioUop.is_rvm, io.req.bits.rs1_data, 0.U)
          sliceBlockAddr := 0.U
          splitCnt       := 0.U
          state := Mux(ioUop.is_rvm, s_slice,
                   Mux(!ioAligned || !ioUop.v_unmasked, s_udcpy, // does aligned idx ls perform ud copy?
                   Mux(!ioUop.v_unmasked, s_vmask,
                   Mux(ioUop.v_idx_ls, s_index, s_split))))
        }
      }
      is (s_udcpy) {
        when (io.vrf_raddr.fire) {
          emulCtr := emulCtr + 1.U
          when (emulCtr + 1.U === nrVecGroup(emul, uop.v_seg_nf)) {
            emulCtr := 0.U
            state := Mux(!uop.v_unmasked, s_vmask,
                     Mux(uop.v_idx_ls, s_index, s_split))
          }
        }
      }
      is (s_vmask) {
        when (io.vrf_raddr.fire) {
          state := Mux(uop.v_idx_ls, s_index, s_split)
        }
      }
      is (s_index) {
        when (io.vrf_raddr.fire) {
          emulCtr := emulCtr + 1.U
          when (emulCtr + 1.U === (1.U << emul)) {
            emulCtr := 0.U
            state := s_split
          }
        }
      }
      is (s_split) {
        when (io.resp.fire) {
          op1 := op1 + addrInc
          uop.v_eidx := uop.v_eidx + eidxInc
          when (isUnitStride && clOffset <= (clSize-vLenb).asUInt) {
            usSplitCtr := 0.U
            emulCtr    := emulCtr + 1.U
          } .elsewhen (isUnitStride) {
            usSplitCtr     := usSplitCtr + 1.U
            usSplitLeftCnt := clOffset +& vLenb.U - clSize.U
          } .otherwise {
            when (uop.v_eidx +& 1.U === vLenECnt) {
              emulCtr := emulCtr + 1.U
            }
          }
          when (uop.v_eidx +& eidxInc >= uop.vconfig.vl) {
            emulCtr := 0.U
            state := s_idle
          }
        }
      }
      is (s_slice) {
        when (io.resp.fire) {
          sliceLenCtr      := sliceLenCtr + 1.U
          sliceBlockAddr   := sliceBlockAddr + sliceAddrInc
          splitCnt         := splitCnt + 1.U
          when (sliceLenLast) {
            sliceLenCtr    := 0.U
            sliceCntCtr    := sliceCntCtr + 1.U
            sliceBaseAddr  := sliceBaseAddr + op2
            sliceBlockAddr := 0.U
            when (sliceCntCtr +& 1.U === uop.m_slice_cnt) {
              sliceCntCtr  := 0.U
              state        := s_idle
            }
          }
        }
      }
    }
  }

  val branchKill = Mux(io.req.fire, IsKilledByBranch(io.brupdate, ioUop), IsKilledByBranch(io.brupdate, uop))
  when (branchKill) {
    state := s_idle
  }

  when (RegNext(state === s_vmask && io.vrf_raddr.fire)) {
    vmask := io.vrf_rdata
  }
  val uopVeidx = uop.v_eidx >> (vLenbSz.U - uop.vd_eew) << (vLenbSz.U - uop.vd_eew)
  vmaskSel := Mux(RegNext(state === s_vmask && io.vrf_raddr.fire), io.vrf_rdata, vmask) >> uopVeidx
  val vmByteMask   = Mux1H(UIntToOH(uop.vd_eew),
                           Seq(vmaskSel,
                               Cat((0 until vLenb/2).map(i => Fill(2, vmaskSel(i))).reverse),
                               Cat((0 until vLenb/4).map(i => Fill(4, vmaskSel(i))).reverse),
                               Cat((0 until vLenb/8).map(i => Fill(8, vmaskSel(i))).reverse)))
  val bodyMask     = Cat((0 until vLenb).map(i => uopVeidx +& i.U >= uop.vstart && uopVeidx +& i.U < uop.vconfig.vl).reverse)
  val bodyByteMask = Mux1H(UIntToOH(uop.vd_eew),
                           Seq(bodyMask,
                               Cat((0 until vLenb/2).map(i => Fill(2, bodyMask(i))).reverse),
                               Cat((0 until vLenb/4).map(i => Fill(4, bodyMask(i))).reverse),
                               Cat((0 until vLenb/8).map(i => Fill(8, bodyMask(i))).reverse)))

  when (RegNext(state === s_index && io.vrf_raddr.fire)) {
    eindex(RegNext(emulCtr)) := io.vrf_rdata
  }

  io.busy := state =/= s_idle
  io.vrf_rtag := MuxCase(0.U, Seq((state === s_udcpy) -> 1.U,
                                  (state === s_vmask) -> 2.U,
                                  (state === s_index) -> 3.U))
  io.vrf_raddr.valid := Mux(state === s_udcpy, ~io.vbusy_status(uop.stale_pvd(emulCtr).bits),
                        Mux(state === s_vmask, ~io.vbusy_status(uop.pvm),
                        Mux(state === s_index, ~io.vbusy_status(uop.pvs2(emulCtr).bits), false.B)))
  io.vrf_raddr.bits  := Mux(state === s_vmask, uop.pvm,
                        Mux(state === s_udcpy, uop.stale_pvd(emulCtr).bits, uop.pvs2(emulCtr).bits))
  io.vrf_emul        := emulCtr

  io.req.ready := true.B

  // io.resp.valid                 := ((state === s_split && (uop.v_unmasked || vmask(uop.v_eidx))) ||
  //                                   (state === s_slice)) &&
  //                                  !IsKilledByBranch(io.brupdate, uop)
  io.resp.valid                 := (state === s_split || state === s_slice) && !IsKilledByBranch(io.brupdate, uop)
  io.resp.bits.uop              := UpdateBrMask(io.brupdate, uop)
  io.resp.bits.uop.pdst         := Mux(uop.is_rvv, uop.pvd(emulCtr).bits, uop.pdst)
  io.resp.bits.uop.stale_pdst   := Mux(uop.is_rvv, uop.stale_pvd(emulCtr).bits, uop.stale_pdst)
  io.resp.bits.uop.v_split_ecnt := Mux(uop.is_rvv, eidxInc, 0.U)
  when (state === s_udcpy) {
    io.resp.bits.uop.v_eidx       := vLenECnt * emulCtr(2,0)
    io.resp.bits.uop.v_split_ecnt := vLenECnt
  }
  io.resp.bits.uop.v_split_first:= uop.v_eidx === 0.U
  io.resp.bits.uop.v_split_last := Mux(state === s_udcpy, emulCtr + 1.U === nrVecGroup(emul, uop.v_seg_nf), uop.v_eidx +& eidxInc >= uop.vconfig.vl)
  io.resp.bits.uop.m_sidx       := sliceCntCtr
  io.resp.bits.uop.m_split_first:= (sliceCntCtr === 0.U) && (sliceLenCtr === 0.U)
  io.resp.bits.uop.m_split_last := (sliceCntCtr +& 1.U === uop.m_slice_cnt) && sliceLenLast
  io.resp_vm                    := Mux(uop.is_rvv, VRegMask(uop.v_eidx, eew, eidxInc, vLenb) & Mux(uop.v_unmasked, Fill(vLenb, 1.U(1.W)), vmByteMask) & bodyByteMask,
                                                   VRegMask(sliceBlockAddr, 0.U, sliceAddrInc, vLenb))
  io.resp_shdir                 := Mux(uop.is_rvm && sliceLenCtr === 0.U, true.B,
                                   Mux(uop.is_rvv && !isUnitStride, false.B,
                                   Mux(uop.is_rvv && usSplitCtr === 0.U, true.B, false.B)))
  val shamt = if(vLenb > clSize) (usSplitCtr << clSizeLog2.U) - clOffset
              else               vLenb.U - usSplitLeftCnt
  io.resp_shamt                 := Mux(uop.is_rvm && sliceLenCtr === 0.U, sliceBlockOff,
                                   Mux(uop.is_rvm, (sliceLenCtr << clSizeLog2.U) - sliceBlockOff,
                                   Mux(!isUnitStride, 0.U, // FIXME
                                   Mux(usSplitCtr === 0.U, clOffset, shamt))))

  io.resp.bits.addr := Mux(uop.is_rvv, Cat((op1 + op2) >> clSizeLog2.U, 0.U(clSizeLog2.W)),
                                       ((sliceBaseAddr+sliceBlockAddr) >> clSizeLog2.U) ## 0.U(clSizeLog2.W))
  io.resp.bits.data := Mux(uop.is_rvm && (sliceCntCtr +& 1.U === uop.m_slice_cnt) && sliceLenLast, splitCnt + 1.U,
                       Mux(uop.is_rvm, splitCnt + 2.U, 0.U))

  // FIXME exceptions: misaligned, breakpoints
  io.resp.bits.mxcpt.valid := false.B
  io.resp.bits.mxcpt.bits  := 0.U
}

class VecMem(implicit p: Parameters) extends LazyModule
{
  val node = TLClientNode(Seq(TLMasterPortParameters.v1(Seq(TLMasterParameters.v1(
      name = s"l1-vec-memq",
      sourceId = IdRange(0, 32),
      supportsProbe = TransferSizes.none
  )))))
  lazy val module = new VecMemImp(this)
}

class VecMemImp(outer: VecMem) extends LazyModuleImp(outer)
  with rocket.HasL1HellaCacheParameters
  with HasBoomCoreParameters
{
  implicit val edge = outer.node.edges.out(0)
  val (tl_out, _) = outer.node.out(0)

  val io = IO(new Bundle{
    val lsu = Flipped(new VecMemIO)
  })

  val vmemq = Module(new BranchKillableQueue(new BoomVMemReq, 3, flow = false))
  val vsdq  = Module(new Queue(UInt(vLen.W), 3))
  //val histq = Module(new BranchKillableQueue(new BoomVMemReq, 16, flow = false)) // use L2 nMSHRs

  vmemq.io.brupdate  := io.lsu.brupdate
  vmemq.io.flush     := false.B //FIXME

  vmemq.io.enq.valid := io.lsu.req.valid
  vmemq.io.enq.bits  := io.lsu.req.bits
  vmemq.io.deq.ready := tl_out.a.fire

  vsdq.io.enq.valid  := RegNext(io.lsu.req.valid && io.lsu.req.bits.uop.uses_stq)
  vsdq.io.enq.bits   := io.lsu.s1_vdata
  vsdq.io.deq.ready  := tl_out.a.fire && vmemq.io.deq.bits.uop.uses_stq

  //histq.io.brupdate  := io.lsu.brupdate
  //histq.io.flush     := false.B
  //histq.io.enq.valid := tl_out.a.fire
  //histq.io.enq.bits  := vmemq.io.deq.bits
  //histq.io.deq.ready := tl_out.d.fire

  tl_out.a.valid  := vmemq.io.deq.valid && (vmemq.io.deq.bits.uop.uses_ldq || vsdq.io.deq.valid)
  when (vmemq.io.deq.bits.uop.uses_ldq) {
    tl_out.a.bits   := edge.Get(
      fromSource = Cat(0.U, vmemq.io.deq.bits.vldq_idx),
      toAddress  = vmemq.io.deq.bits.addr,
      lgSize     = lgCacheBlockBytes.U
    )._2
  } .otherwise {
    tl_out.a.bits   := edge.Put(
      fromSource = Cat(1.U(1.W), vmemq.io.deq.bits.vstq_idx),
      toAddress  = vmemq.io.deq.bits.addr,
      lgSize     = lgCacheBlockBytes.U,
      data       = Mux(vmemq.io.deq.bits.shdir, vsdq.io.deq.bits << (vmemq.io.deq.bits.shamt << 3.U),
                                                vsdq.io.deq.bits >> (vmemq.io.deq.bits.shamt << 3.U)),
      mask       = Mux(vmemq.io.deq.bits.shdir, vmemq.io.deq.bits.mask << vmemq.io.deq.bits.shamt,
                                                vmemq.io.deq.bits.mask >> vmemq.io.deq.bits.shamt)
    )._2
  }

  io.lsu.resp.valid := tl_out.d.valid
  //io.lsu.resp.bits.uop := histq.io.deq.bits.uop
  io.lsu.resp.bits.is_vst   := Reverse(tl_out.d.bits.source)(0)
  io.lsu.resp.bits.vldq_idx := tl_out.d.bits.source //histq.io.deq.bits.vldq_idx
  io.lsu.resp.bits.vstq_idx := tl_out.d.bits.source //histq.io.deq.bits.vstq_idx
  io.lsu.resp.bits.cdata := tl_out.d.bits.data //Mux(histq.io.deq.bits.shdir, tl_data_shr, tl_data_shl)
  tl_out.d.ready := true.B

  io.lsu.req.ready  := vmemq.io.enq.ready
  io.lsu.vsdq_ready := vsdq.io.enq.ready

  io.lsu.release       := DontCare
  io.lsu.release.valid := false.B // FIXME
}
