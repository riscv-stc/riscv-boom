//******************************************************************************
// Copyright (c) 2018 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Fetch Buffer
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// Takes a FetchBundle and converts into a vector of MicroOps.

package boom.ifu

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.{Parameters}
import freechips.rocketchip.rocket.{MStatus, BP, BreakpointUnit}

import boom.common._
import boom.util.{BoolToChar, MaskUpper}
import freechips.rocketchip.util._

/**
 * Bundle that is made up of converted MicroOps from the Fetch Bundle
 * input to the Fetch Buffer. This is handed to the Decode stage.
 */
class FetchBufferResp(implicit p: Parameters) extends BoomBundle
{
  val uops = Vec(coreWidth, Valid(new MicroOp()))
}

/**
 * Buffer to hold fetched packets and convert them into a vector of MicroOps
 * to give the Decode stage
 *
 * @param num_entries effectively the number of full-sized fetch packets we can hold.
 */
class FetchBuffer(implicit p: Parameters) extends BoomModule
  with HasBoomCoreParameters
  with HasBoomFrontendParameters
{
  val numEntries = numFetchBufferEntries
  val io = IO(new BoomBundle {
    val enq = Flipped(Decoupled(new FetchBundle()))
    val deq = new DecoupledIO(new FetchBufferResp())

    // Was the pipeline redirected? Clear/reset the fetchbuffer.
    val clear = Input(Bool())

    val perf = Output(new Bundle {
      val maybe_full  = Bool()
      val amost_empty = Bool()
    })
  })

  require (numEntries > fetchWidth)
  require (numEntries % coreWidth == 0)
  val numRows = numEntries / coreWidth

  val ram = Reg(Vec(numEntries, new MicroOp))
  ram.suggestName("fb_uop_ram")
  val deq_vec = Wire(Vec(numRows, Vec(coreWidth, new MicroOp)))
  //-------------------------------------------------------------
  // **** vset control info ****
  //-------------------------------------------------------------
  val is_vset = Wire(Vec(coreWidth, Bool()))
  val vset_idx = RegInit(VecInit(Seq.fill(coreWidth + 1)(true.B)))
  //-------------------------------------------------------------
  // **** mset control info ****
  //-------------------------------------------------------------
  val is_mtypeset = Wire(Vec(coreWidth, Bool()))
  val is_mtileset = Wire(Vec(coreWidth, Bool()))
  val is_ntileset = Wire(Vec(coreWidth, Bool()))
  val is_ktileset = Wire(Vec(coreWidth, Bool()))
  val is_outshset = Wire(Vec(coreWidth, Bool()))
  val is_inshset  = Wire(Vec(coreWidth, Bool()))
  val is_skset    = Wire(Vec(coreWidth, Bool()))
  val mtypeset_idx = RegInit(VecInit(Seq.fill(coreWidth + 1)(true.B)))
  val mtileset_idx = RegInit(VecInit(Seq.fill(coreWidth + 1)(true.B)))
  val ntileset_idx = RegInit(VecInit(Seq.fill(coreWidth + 1)(true.B)))
  val ktileset_idx = RegInit(VecInit(Seq.fill(coreWidth + 1)(true.B)))
  val outshset_idx = RegInit(VecInit(Seq.fill(coreWidth + 1)(true.B)))
  val inshset_idx  = RegInit(VecInit(Seq.fill(coreWidth + 1)(true.B)))
  val skset_idx    = RegInit(VecInit(Seq.fill(coreWidth + 1)(true.B)))

  val head = RegInit(1.U(numRows.W))
  val tail = RegInit(1.U(numEntries.W))

  val maybe_full = RegInit(false.B)

  //-------------------------------------------------------------
  // **** Enqueue Uops ****
  //-------------------------------------------------------------
  // Step 1: Convert FetchPacket into a vector of MicroOps.
  // Step 2: Generate one-hot write indices.
  // Step 3: Write MicroOps into the RAM.

  def rotateLeft(in: UInt, k: Int) = {
    val n = in.getWidth
    Cat(in(n-k-1,0), in(n-1, n-k))
  }

  val might_hit_head = (1 until fetchWidth).map(k => VecInit(rotateLeft(tail, k).asBools.zipWithIndex.filter
    {case (e,i) => i % coreWidth == 0}.map {case (e,i) => e}).asUInt).map(tail => head & tail).reduce(_|_).orR
  val at_head = (VecInit(tail.asBools.zipWithIndex.filter {case (e,i) => i % coreWidth == 0}
    .map {case (e,i) => e}).asUInt & head).orR
  val do_enq = !(at_head && maybe_full || might_hit_head)

  io.enq.ready := do_enq

  // Input microops.
  val in_mask = Wire(Vec(fetchWidth, Bool()))
  val in_uops = Wire(Vec(fetchWidth, new MicroOp()))

  // Step 1: Convert FetchPacket into a vector of MicroOps.
  for (b <- 0 until nBanks) {
    for (w <- 0 until bankWidth) {
      val i = (b * bankWidth) + w

      val pc = (bankAlign(io.enq.bits.pc) + (i << 1).U)

      in_uops(i)                := DontCare
      in_mask(i)                := io.enq.valid && io.enq.bits.mask(i)
      in_uops(i).edge_inst      := false.B
      in_uops(i).debug_pc       := pc
      in_uops(i).pc_lob         := pc

      in_uops(i).is_sfb         := io.enq.bits.sfbs(i) || io.enq.bits.shadowed_mask(i)

      if (w == 0) {
        when (io.enq.bits.edge_inst(b)) {
          in_uops(i).debug_pc  := bankAlign(io.enq.bits.pc) + (b * bankBytes).U - 2.U
          in_uops(i).pc_lob    := bankAlign(io.enq.bits.pc) + (b * bankBytes).U
          in_uops(i).edge_inst := true.B
        }
      }
      in_uops(i).ftq_idx        := io.enq.bits.ftq_idx
      in_uops(i).ftq_off        := i.U
      in_uops(i).inst           := io.enq.bits.exp_insts(i)
      in_uops(i).debug_inst     := io.enq.bits.insts(i)
      in_uops(i).is_rvc         := io.enq.bits.insts(i)(1,0) =/= 3.U
      in_uops(i).taken          := io.enq.bits.cfi_idx.bits === i.U && io.enq.bits.cfi_idx.valid

      in_uops(i).xcpt_pf_if     := io.enq.bits.xcpt_pf_if
      in_uops(i).xcpt_ae_if     := io.enq.bits.xcpt_ae_if
      in_uops(i).bp_debug_if    := io.enq.bits.bp_debug_if_oh(i)
      in_uops(i).bp_xcpt_if     := io.enq.bits.bp_xcpt_if_oh(i)

      in_uops(i).debug_fsrc     := io.enq.bits.fsrc
      in_uops(i).debug_events   := io.enq.bits.debug_events(i)
    }
  }

  // Step 2. Generate one-hot write indices.
  val enq_idxs = Wire(Vec(fetchWidth, UInt(numEntries.W)))

  def inc(ptr: UInt) = {
    val n = ptr.getWidth
    Cat(ptr(n-2,0), ptr(n-1))
  }

  var enq_idx = tail
  for (i <- 0 until fetchWidth) {
    enq_idxs(i) := enq_idx
    enq_idx = Mux(in_mask(i), inc(enq_idx), enq_idx)
  }

  // Step 3: Write MicroOps into the RAM.
  for (i <- 0 until fetchWidth) {
    for (j <- 0 until numEntries) {
      when (do_enq && in_mask(i) && enq_idxs(i)(j)) {
        ram(j) := in_uops(i)
      }
    }
  }

  //-------------------------------------------------------------
  // **** VSET INST CONTROL ****
  //-------------------------------------------------------------

  for (i <- 0 until coreWidth) {
    var inst         =  ram(Cat(Fill(numEntries,0.U(1.W)),PriorityEncoder(head)) * coreWidth.U +& i.U(numEntries.W)).inst
    is_vset(i)      :=  (inst(6, 0) === 87.U) &&
                        (inst(14, 12) === 7.U) &&
                        ((inst(31, 30) === 3.U) || !inst(31))
    is_mtypeset(i)  :=  (inst(6, 0) === 119.U) &&
                        (inst(14, 12) === 7.U) &&
                        ((inst(31, 28) === 0.U) || (inst(31, 28) === 1.U))
    is_mtileset(i)  :=  (inst(6, 0) === 119.U) &&
                        (inst(14, 12) === 7.U) &&
                        ((inst(31, 28) === 2.U) || (inst(31, 28) === 3.U))
    is_ntileset(i)  :=  (inst(6, 0) === 119.U) &&
                        (inst(14, 12) === 7.U) &&
                        ((inst(31, 28) === 6.U) || (inst(31, 28) === 7.U))
    is_ktileset(i)  :=  (inst(6, 0) === 119.U) &&
                        (inst(14, 12) === 7.U) &&
                        ((inst(31, 28) === 4.U) || (inst(31, 28) === 5.U))
    is_outshset(i)  :=  (inst(6, 0) === 119.U) &&
                        (inst(14, 12) === 7.U) &&
                        (inst(31, 28) === 8.U)
    is_inshset(i)   :=  (inst(6, 0) === 119.U) &&
                        (inst(14, 12) === 7.U) &&
                        (inst(31, 28) === 9.U)
    is_skset(i)     :=  (inst(6, 0) === 119.U) &&
                        (inst(14, 12) === 7.U) &&
                        (inst(31, 28) === 10.U)
  }
  // val is_more_vset = dec_vconfig_nums > 1.U
  val setv = (is_vset zip vset_idx) map { case(i,v) => (i && v ) }
  val dec_vconfig_nums = setv.scanLeft(0.U)(_.asUInt + _.asUInt)
  val vconfig_stall = WireInit(false.B).asTypeOf(Vec(coreWidth, Bool()))
  val youngest_vconfig_idx = (coreWidth - 1).U - PriorityEncoder(setv.reverse)
  val oldest_vconfig_idx = PriorityEncoder(setv)
  vconfig_stall(oldest_vconfig_idx + 1.U) := dec_vconfig_nums(youngest_vconfig_idx + 1.U) - dec_vconfig_nums(oldest_vconfig_idx + 1.U) > 0.U
  //-------------------------------------------------------------
  // **** MSET INST CONTROL ****
  //-------------------------------------------------------------
  val mtype_set = (is_mtypeset zip mtypeset_idx) map { case(i,v) => (i && v ) }
  val dec_mtypeconfig_nums = mtype_set.scanLeft(0.U)(_.asUInt + _.asUInt)
  val mtypeconfig_stall = WireInit(false.B).asTypeOf(Vec(coreWidth, Bool()))
  val youngest_mtypeconfig_idx = (coreWidth - 1).U - PriorityEncoder(mtype_set.reverse)
  val oldest_mtypeconfig_idx = PriorityEncoder(mtype_set)
  mtypeconfig_stall(oldest_mtypeconfig_idx + 1.U) := dec_mtypeconfig_nums(youngest_mtypeconfig_idx + 1.U) - dec_mtypeconfig_nums(oldest_mtypeconfig_idx + 1.U) > 0.U

  val mtile_set = (is_mtileset zip mtileset_idx) map { case(i,v) => (i && v ) }
  val dec_mtileconfig_nums = mtile_set.scanLeft(0.U)(_.asUInt + _.asUInt)
  val mtileconfig_stall = WireInit(false.B).asTypeOf(Vec(coreWidth, Bool()))
  val youngest_mtileconfig_idx = (coreWidth - 1).U - PriorityEncoder(mtile_set.reverse)
  val oldest_mtileconfig_idx = PriorityEncoder(mtile_set)
  mtileconfig_stall(oldest_mtileconfig_idx + 1.U) := dec_mtileconfig_nums(youngest_mtileconfig_idx + 1.U) - dec_mtileconfig_nums(oldest_mtileconfig_idx + 1.U) > 0.U

  val ntile_set = (is_ntileset zip ntileset_idx) map { case(i,v) => (i && v ) }
  val dec_ntileconfig_nums = ntile_set.scanLeft(0.U)(_.asUInt + _.asUInt)
  val ntileconfig_stall = WireInit(false.B).asTypeOf(Vec(coreWidth, Bool()))
  val youngest_ntileconfig_idx = (coreWidth - 1).U - PriorityEncoder(ntile_set.reverse)
  val oldest_ntileconfig_idx = PriorityEncoder(ntile_set)
  ntileconfig_stall(oldest_ntileconfig_idx + 1.U) := dec_ntileconfig_nums(youngest_ntileconfig_idx + 1.U) - dec_ntileconfig_nums(oldest_ntileconfig_idx + 1.U) > 0.U

  val ktile_set = (is_ktileset zip ktileset_idx) map { case(i,v) => (i && v ) }
  val dec_ktileconfig_nums = ktile_set.scanLeft(0.U)(_.asUInt + _.asUInt)
  val ktileconfig_stall = WireInit(false.B).asTypeOf(Vec(coreWidth, Bool()))
  val youngest_ktileconfig_idx = (coreWidth - 1).U - PriorityEncoder(ktile_set.reverse)
  val oldest_ktileconfig_idx = PriorityEncoder(ktile_set)
  ktileconfig_stall(oldest_ktileconfig_idx + 1.U) := dec_ktileconfig_nums(youngest_ktileconfig_idx + 1.U) - dec_ktileconfig_nums(oldest_ktileconfig_idx + 1.U) > 0.U

  val outsh_set = (is_outshset zip outshset_idx) map { case(i,v) => (i && v ) }
  val dec_outshconfig_nums = outsh_set.scanLeft(0.U)(_.asUInt + _.asUInt)
  val outshconfig_stall = WireInit(false.B).asTypeOf(Vec(coreWidth, Bool()))
  val youngest_outshconfig_idx = (coreWidth - 1).U - PriorityEncoder(outsh_set.reverse)
  val oldest_outshconfig_idx = PriorityEncoder(outsh_set)
  outshconfig_stall(oldest_outshconfig_idx + 1.U) := dec_outshconfig_nums(youngest_outshconfig_idx + 1.U) - dec_outshconfig_nums(oldest_outshconfig_idx + 1.U) > 0.U

  val insh_set = (is_inshset zip inshset_idx) map { case(i,v) => (i && v ) }
  val dec_inshconfig_nums = insh_set.scanLeft(0.U)(_.asUInt + _.asUInt)
  val inshconfig_stall = WireInit(false.B).asTypeOf(Vec(coreWidth, Bool()))
  val youngest_inshconfig_idx = (coreWidth - 1).U - PriorityEncoder(insh_set.reverse)
  val oldest_inshconfig_idx = PriorityEncoder(insh_set)
  inshconfig_stall(oldest_inshconfig_idx + 1.U) := dec_inshconfig_nums(youngest_inshconfig_idx + 1.U) - dec_inshconfig_nums(oldest_inshconfig_idx + 1.U) > 0.U

  val sk_set = (is_skset zip skset_idx) map { case(i,v) => (i && v ) }
  val dec_skconfig_nums = sk_set.scanLeft(0.U)(_.asUInt + _.asUInt)
  val skconfig_stall = WireInit(false.B).asTypeOf(Vec(coreWidth, Bool()))
  val youngest_skconfig_idx = (coreWidth - 1).U - PriorityEncoder(sk_set.reverse)
  val oldest_skconfig_idx = PriorityEncoder(sk_set)
  skconfig_stall(oldest_skconfig_idx + 1.U) := dec_skconfig_nums(youngest_skconfig_idx + 1.U) - dec_skconfig_nums(oldest_skconfig_idx + 1.U) > 0.U

  //-------------------------------------------------------------
  // vset_idx(oldest_vconfig_idx) := false.B

  //-------------------------------------------------------------
  // **** Dequeue Uops ****
  //-------------------------------------------------------------

  val tail_collisions = VecInit((0 until numEntries).map(i =>
                          head(i/coreWidth) && (!maybe_full || (i % coreWidth != 0).B))).asUInt & tail
  val slot_will_hit_tail = (0 until numRows).map(i => tail_collisions((i+1)*coreWidth-1, i*coreWidth)).reduce(_|_) |
                            vconfig_stall.asUInt | mtypeconfig_stall.asUInt |
                            mtileconfig_stall.asUInt | ntileconfig_stall.asUInt | ktileconfig_stall.asUInt |
                            outshconfig_stall.asUInt | inshconfig_stall.asUInt | skconfig_stall.asUInt

  val will_hit_tail = slot_will_hit_tail.orR

  val do_deq = io.deq.ready && !will_hit_tail

  val deq_valids = (~MaskUpper(slot_will_hit_tail)).asBools

  // Generate vec for dequeue read port.
  for (i <- 0 until numEntries) {
    deq_vec(i/coreWidth)(i%coreWidth) := ram(i)
  }

  when(deq_valids(oldest_vconfig_idx) === true.B) {
    vset_idx(oldest_vconfig_idx) := false.B
  }

  when(deq_valids(oldest_mtypeconfig_idx) === true.B) {
    mtypeset_idx(oldest_mtypeconfig_idx) := false.B
  }

  when(deq_valids(oldest_mtileconfig_idx) === true.B) {
    mtileset_idx(oldest_mtileconfig_idx) := false.B
  }

  when(deq_valids(oldest_ntileconfig_idx) === true.B) {
    ntileset_idx(oldest_ntileconfig_idx) := false.B
  }

  when(deq_valids(oldest_ktileconfig_idx) === true.B) {
    ktileset_idx(oldest_ktileconfig_idx) := false.B
  }

  when(deq_valids(oldest_outshconfig_idx) === true.B) {
    outshset_idx(oldest_outshconfig_idx) := false.B
  }

  when(deq_valids(oldest_inshconfig_idx) === true.B) {
    inshset_idx(oldest_inshconfig_idx) := false.B
  }

  when(deq_valids(oldest_skconfig_idx) === true.B) {
    skset_idx(oldest_skconfig_idx) := false.B
  }

  io.deq.bits.uops zip deq_valids           map {case (d,v) => d.valid := v}
  io.deq.bits.uops zip Mux1H(head, deq_vec) map {case (d,q) => d.bits  := q}
  io.deq.valid := deq_valids.reduce(_||_)

  //-------------------------------------------------------------
  // **** Update State ****
  //-------------------------------------------------------------

  when (do_enq) {
    tail := enq_idx
    when (in_mask.reduce(_||_)) {
      maybe_full := true.B
    }
  }

  when (do_deq) {
    head := inc(head)
    maybe_full := false.B
    (0 until coreWidth).map(i => vset_idx(i):= true.B)
    (0 until coreWidth).map(i => mtypeset_idx(i):= true.B)
    (0 until coreWidth).map(i => mtileset_idx(i):= true.B)
    (0 until coreWidth).map(i => ntileset_idx(i):= true.B)
    (0 until coreWidth).map(i => ktileset_idx(i):= true.B)
    (0 until coreWidth).map(i => outshset_idx(i):= true.B)
    (0 until coreWidth).map(i => inshset_idx(i):= true.B)
    (0 until coreWidth).map(i => skset_idx(i):= true.B)
  }

  when (io.clear) {
    head := 1.U
    tail := 1.U
    maybe_full := false.B
    (0 until coreWidth).map(i => vset_idx(i):= true.B)
    (0 until coreWidth).map(i => mtypeset_idx(i):= true.B)
    (0 until coreWidth).map(i => mtileset_idx(i):= true.B)
    (0 until coreWidth).map(i => ntileset_idx(i):= true.B)
    (0 until coreWidth).map(i => ktileset_idx(i):= true.B)
    (0 until coreWidth).map(i => outshset_idx(i):= true.B)
    (0 until coreWidth).map(i => inshset_idx(i):= true.B)
    (0 until coreWidth).map(i => skset_idx(i):= true.B)
  }

  // TODO Is this necessary?
  when (reset.toBool) {
    io.deq.bits.uops map { u => u.valid := false.B }
  }

  // perf event
  io.perf.maybe_full := maybe_full
  io.perf.amost_empty := will_hit_tail

}
