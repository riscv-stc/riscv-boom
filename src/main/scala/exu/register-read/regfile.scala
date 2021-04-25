//******************************************************************************
// Copyright (c) 2013 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Register File (Abstract class and Synthesizable RegFile)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.exu

import scala.collection.mutable.ArrayBuffer

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.Parameters

import boom.common._
import boom.util._

/**
 * IO bundle for a register read port
 *
 * @param addrWidth size of register address in bits
 * @param dataWidth size of register in bits
 */
class RegisterFileReadPortIO(val addrWidth: Int, val dataWidth: Int)(implicit p: Parameters) extends BoomBundle
{
  val addr = Input(UInt(addrWidth.W))
  val data = Output(UInt(dataWidth.W))
}

/**
 * IO bundle for the register write port
 *
 * @param addrWidth size of register address in bits
 * @param dataWidth size of register in bits
 */
class RegisterFileWritePort(val addrWidth: Int, val dataWidth: Int, val vector: Boolean = false)(implicit p: Parameters) extends BoomBundle
{
  val addr = UInt(addrWidth.W)
  val data = UInt(dataWidth.W)
  val mask = if (vector) UInt(eLenb.W) else null
}

/**
 * Utility function to turn ExeUnitResps to match the regfile's WritePort I/Os.
 */
object WritePort
{
  def apply(
    enq: DecoupledIO[ExeUnitResp],
    addrWidth: Int,
    dataWidth: Int,
    rtype: UInt,
    vector: Boolean = false, eLenb: Int = 8, eLenSelSz: Int = 4)
    (implicit p: Parameters): Valid[RegisterFileWritePort] = {
     val wport = Wire(Valid(new RegisterFileWritePort(addrWidth, dataWidth, vector)))
     val enq_uop = enq.bits.uop

     wport.valid := enq.valid && enq_uop.dst_rtype === rtype
     if (vector) {
       val vstart = enq_uop.vstart
       val vsew = enq_uop.vconfig.vtype.vsew
       val ecnt = enq_uop.v_split_ecnt
       val (rsel, rmsk) = VRegSel(vstart, vsew, ecnt, eLenb, eLenSelSz)
       wport.bits.addr := Cat(enq_uop.pdst, rsel)
       wport.bits.mask := rmsk
       wport.bits.data := VDataFill(enq.bits.data, vsew, eLenb*8)
     } else {
       wport.bits.addr := enq_uop.pdst
       wport.bits.data := enq.bits.data
     }
     enq.ready := true.B
     wport
  }
}

/**
 * Register file abstract class
 *
 * @param numRegisters number of registers
 * @param numReadPorts number of read ports
 * @param numWritePorts number of write ports
 * @param registerWidth size of registers in bits
 * @param bypassableArray list of write ports from func units to the read port of the regfile
 */
abstract class RegisterFile(
  numRegisters: Int,
  numReadPorts: Int,
  numWritePorts: Int,
  registerWidth: Int,
  bypassableArray: Seq[Boolean], // which write ports can be bypassed to the read ports?
  vector: Boolean = false)
  (implicit p: Parameters) extends BoomModule
{
  val io = IO(new BoomBundle {
    val read_ports = Vec(numReadPorts, new RegisterFileReadPortIO(maxPregSz, registerWidth))
    val write_ports = Flipped(Vec(numWritePorts, Valid(new RegisterFileWritePort(maxPregSz, registerWidth, vector))))
  })

  private val rf_cost = (numReadPorts + numWritePorts) * (numReadPorts + 2*numWritePorts)
  private val type_str = if (registerWidth == fLen+1) "Floating Point" else "Integer"
  override def toString: String = BoomCoreStringPrefix(
    "==" + type_str + " Regfile==",
    "Num RF Read Ports     : " + numReadPorts,
    "Num RF Write Ports    : " + numWritePorts,
    "RF Cost (R+W)*(R+2W)  : " + rf_cost,
    "Bypassable Units      : " + bypassableArray)
}

/**
 * A synthesizable model of a Register File. You will likely want to blackbox this for more than modest port counts.
 *
 * @param numRegisters number of registers
 * @param numReadPorts number of read ports
 * @param numWritePorts number of write ports
 * @param registerWidth size of registers in bits
 * @param bypassableArray list of write ports from func units to the read port of the regfile
 */
class RegisterFileSynthesizable(
   numRegisters: Int,
   numReadPorts: Int,
   numWritePorts: Int,
   registerWidth: Int,
   bypassableArray: Seq[Boolean],
   vector: Boolean = false)
   (implicit p: Parameters)
   extends RegisterFile(numRegisters, numReadPorts, numWritePorts, registerWidth, bypassableArray, vector)
{
  // --------------------------------------------------------------

  val regfile = Mem(numRegisters, UInt(registerWidth.W))

  // --------------------------------------------------------------
  // Read ports.

  val read_data = Wire(Vec(numReadPorts, UInt(registerWidth.W)))

  // Register the read port addresses to give a full cycle to the RegisterRead Stage (if desired).
  val read_addrs = io.read_ports.map(p => RegNext(p.addr))

  for (i <- 0 until numReadPorts) {
    read_data(i) := regfile(read_addrs(i))
  }

  // --------------------------------------------------------------
  // Bypass out of the ALU's write ports.
  // We are assuming we cannot bypass a writer to a reader within the regfile memory
  // for a write that occurs at the end of cycle S1 and a read that returns data on cycle S1.
  // But since these bypasses are expensive, and not all write ports need to bypass their data,
  // only perform the w->r bypass on a select number of write ports.

  require (bypassableArray.length == io.write_ports.length)

  if (bypassableArray.reduce(_||_)) {
    val bypassable_wports = ArrayBuffer[Valid[RegisterFileWritePort]]()
    io.write_ports zip bypassableArray map { case (wport, b) => if (b) { bypassable_wports += wport} }

    for (i <- 0 until numReadPorts) {
      val bypass_ens = bypassable_wports.map(x => x.valid &&
        x.bits.addr === read_addrs(i))

      val bypass_data = Mux1H(VecInit(bypass_ens), VecInit(bypassable_wports.map(_.bits.data)))

      io.read_ports(i).data := Mux(bypass_ens.reduce(_|_), bypass_data, read_data(i))
    }
  } else {
    for (i <- 0 until numReadPorts) {
      io.read_ports(i).data := read_data(i)
    }
  }

  // --------------------------------------------------------------
  // Write ports.

  if (vector) {
    require(registerWidth == eLen)

//  for (wport <- io.write_ports) {
//    when (wport.valid) {
//      val old_data = regfile(wport.bits.addr)
//      val new_data = Wire(Vec(eLenb, UInt(8.W)))
//      require(registerWidth == eLen)
//      for (i <- 0 until eLenb) {
//        new_data(i) := Mux(wport.bits.mask(i), wport.bits.data(i*8+7,i*8), old_data(i*8+7,i*8))
//      }
//      regfile(wport.bits.addr) := Cat(new_data.reverse)
//    }
//  }

    // for vector you need merge write ports since they may access the same ELEN on different elements
    for (r <- 0 until numRegisters) {
      when (io.write_ports.map(w => w.valid && r.U === w.bits.addr).reduce(_|_)) {
        val old_data = regfile(r)
        val new_data = Wire(Vec(eLenb, UInt(8.W)))
        (0 until eLenb).map(x => new_data(x) := old_data(x*8+7, x*8))
        for (wport <- io.write_ports) {
          for (i <- 0 until eLenb) {
            when (wport.valid && r.U === wport.bits.addr && wport.bits.mask(i)) {
              new_data(i) := wport.bits.data(i*8+7,i*8)
            }
          }
        }
        regfile(r) := Cat(new_data.reverse)
      }
    }

    // ensure there is only 1 writer per element (unless to preg0)
    if (numWritePorts > 1) {
      for (i <- 0 until (numWritePorts - 1)) {
        for (j <- (i + 1) until numWritePorts) {
          assert(!io.write_ports(i).valid ||
                 !io.write_ports(j).valid ||
                 (io.write_ports(i).bits.addr =/= io.write_ports(j).bits.addr) ||
                 (io.write_ports(i).bits.addr === io.write_ports(j).bits.addr) && ((io.write_ports(i).bits.mask & io.write_ports(j).bits.mask) === 0.U) ||
                 (io.write_ports(i).bits.addr === 0.U),
            "[regfile] too many writers a register")
        }
      }
    }
  } else {
    for (wport <- io.write_ports) {
      when (wport.valid) {
        regfile(wport.bits.addr) := wport.bits.data
      }
    }

    // ensure there is only 1 writer per register (unless to preg0)
    if (numWritePorts > 1) {
      for (i <- 0 until (numWritePorts - 1)) {
        for (j <- (i + 1) until numWritePorts) {
          assert(!io.write_ports(i).valid ||
                 !io.write_ports(j).valid ||
                 (io.write_ports(i).bits.addr =/= io.write_ports(j).bits.addr) ||
                 (io.write_ports(i).bits.addr === 0.U), // note: you only have to check one here
            "[regfile] too many writers a register")
        }
      }
    }
  }
}
