//******************************************************************************
// Copyright (c) 2015 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// RISC-V Constructing the Execution Units
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.exu

import scala.collection.mutable.{ArrayBuffer}

import chisel3._

import freechips.rocketchip.config.{Parameters}

import boom.common._
import boom.util.{BoomCoreStringPrefix}

/**
 * Top level class to wrap all execution units together into a "collection"
 *
 * @param fpu using a FPU?
 */
class ExecutionUnits(val fpu: Boolean = false, val vector: Boolean = false, val matrix: Boolean = false)(implicit val p: Parameters) extends HasBoomCoreParameters
{
  val totalIssueWidth = issueParams.map(_.issueWidth).sum

  //*******************************
  // Instantiate the ExecutionUnits

  private val exe_units = ArrayBuffer[ExecutionUnit]()

  //*******************************
  // Act like a collection

  def length = exe_units.length

  def apply(n: Int) = exe_units(n)

  def map[T](f: ExecutionUnit => T) = {
    exe_units.map(f)
  }

  def withFilter(f: ExecutionUnit => Boolean) = {
    exe_units.withFilter(f)
  }

  def foreach[U](f: ExecutionUnit => U) = {
    exe_units.foreach(f)
  }

  def zipWithIndex = {
    exe_units.zipWithIndex
  }

  def indexWhere(f: ExecutionUnit => Boolean) = {
    exe_units.indexWhere(f)
  }

  def count(f: ExecutionUnit => Boolean) = {
    exe_units.count(f)
  }

  lazy val memory_units = {
    exe_units.filter(_.hasMem)
  }

  lazy val alu_units = {
    exe_units.filter(_.hasAlu)
  }

  lazy val csr_unit = {
    require (exe_units.count(_.hasCSR) == 1)
    exe_units.find(_.hasCSR).get
  }

  lazy val ifpu_unit = {
    require (usingFPU)
    require (exe_units.count(_.hasIfpu) == 1)
    exe_units.find(_.hasIfpu).get
  }

  lazy val fpiu_unit = {
    require (usingFPU)
    require (exe_units.count(_.hasFpiu) == 1)
    exe_units.find(_.hasFpiu).get
  }


  lazy val jmp_unit_idx = {
    exe_units.indexWhere(_.hasJmpUnit)
  }

  lazy val csr_unit_idx = {
    exe_units.indexWhere(_.hasCSR)
  }


  lazy val rocc_unit = {
    require (usingRoCC)
    require (exe_units.count(_.hasRocc) == 1)
    exe_units.find(_.hasRocc).get
  }

  //lazy val vmx_unit = {
    //require (usingVector)
    //require (exe_units.count(_.hasVMX) == 1)
    //exe_units.find(_.hasVMX).get
  //}

  if (!fpu && !vector && !matrix) {
    // scalar integer
    val int_width = issueParams.find(_.iqType == IQT_INT.litValue).get.issueWidth

    for (w <- 0 until memWidth) {
      val memExeUnit = Module(new ALUExeUnit(
        hasAlu = false,
        hasMem = true))

      memExeUnit.io.ll_iresp.ready := DontCare

      exe_units += memExeUnit
    }

    for (w <- 0 until int_width) {
      def is_nth(n: Int): Boolean = w == ((n) % int_width)
      val alu_exe_unit = Module(new ALUExeUnit(
        hasJmpUnit     = is_nth(0),
        hasCSR         = is_nth(1),
        hasRocc        = is_nth(1) && usingRoCC,
        hasMul         = is_nth(2),
        hasDiv         = is_nth(3),
        hasIfpu        = is_nth(4) && usingFPU))
      exe_units += alu_exe_unit
    }
  } else if (fpu && !vector && !matrix) {
    // scalar float
    val fp_width = issueParams.find(_.iqType == IQT_FP.litValue).get.issueWidth
    for (w <- 0 until fp_width) {
      val fpu_exe_unit = Module(new FPUExeUnit(hasFpu = true,
                                             hasFdiv = usingFDivSqrt && (w==0),
                                             hasFpiu = (w==0)))
      exe_units += fpu_exe_unit
    }
  } else if (vector && !matrix) { // vector
    val vec_width = issueParams.find(_.iqType == IQT_VEC.litValue).get.issueWidth
    for (w <- 0 until vec_width) {
      val vec_exe_unit = Module(new VecExeUnit(//hasVMX = false,
                                               hasIfpu = true,
                                               hasFpu = true,
                                               hasFdiv = usingFDivSqrt))
      vec_exe_unit.suggestName("vec_exe_unit")
      exe_units += vec_exe_unit
    }

    //val vmx_exe_unit = Module(new VecExeUnit(hasVMX = true,
    //                                         hasAlu = false,
    //                                         hasMacc = false,
    //                                         hasVMaskUnit = false,
    //                                         hasDiv = false))
    //vmx_exe_unit.suggestName("vmx_exe_unit")
    //exe_units += vmx_exe_unit
  }
  else { // matrix
    for(w <- 0 until matWidth) {
      val mat_exe_unit = Module(new MatExeUnit())
      mat_exe_unit.suggestName("mat_exe_unit")
      exe_units += mat_exe_unit
      //FIX ME
      mat_exe_unit.io.mclrResp.ready := DontCare
      mat_exe_unit.io.mopaResp.ready := DontCare
    }
  }

  val exeUnitsStr = new StringBuilder
  for (exe_unit <- exe_units) {
    exeUnitsStr.append(exe_unit.toString)
  }

  override def toString: String =
    (BoomCoreStringPrefix("===ExecutionUnits===") + "\n"
    + (if (!fpu) {
      BoomCoreStringPrefix(
        "==" + coreWidth + "-wide Machine==",
        "==" + totalIssueWidth + " Issue==")
    } else {
      ""
    }) + "\n"
    + exeUnitsStr.toString)

  require (exe_units.length != 0)
  if (!fpu && !vector & !matrix) {
    // if this is for FPU units, we don't need a memory unit (or other integer units).
    require (exe_units.map(_.hasMem).reduce(_|_), "Datapath is missing a memory unit.")
    require (exe_units.map(_.hasMul).reduce(_|_), "Datapath is missing a multiplier.")
    require (exe_units.map(_.hasDiv).reduce(_|_), "Datapath is missing a divider.")
  } else if (fpu && !vector & !matrix) {
    require (exe_units.map(_.hasFpu).reduce(_|_),
      "Datapath is missing a fpu (or has an fpu and shouldnt).")
  }

  val numIrfReaders       = exe_units.count(_.readsIrf)
  val numIrfReadPorts     = exe_units.count(_.readsIrf) * 2
  val numIrfWritePorts    = exe_units.count(_.writesIrf)
  val numLlIrfWritePorts  = exe_units.count(_.writesLlIrf)
  val numTotalBypassPorts = exe_units.withFilter(_.bypassable).map(_.numBypassStages).foldLeft(0)(_+_)

  val numFrfReaders       = exe_units.count(_.readsFrf)
  val numFrfReadPorts     = exe_units.count(_.readsFrf) * 3
  val numFrfWritePorts    = exe_units.count(_.writesFrf)
  val numLlFrfWritePorts  = exe_units.count(_.writesLlFrf)

  val numVrfReaders       = exe_units.count(_.readsVrf)
  val numVrfReadPorts     = exe_units.count(_.readsVrf) * 4 // - exe_units.count(_.hasVMX) * 2
  val numVrfWritePorts    = exe_units.count(_.writesVrf)
  val numLlVrfWritePorts  = exe_units.count(_.writesLlVrf)

  val numTrTileReaders    = exe_units.count(_.readsTrTile)
  val numTrTileReadPorts  = exe_units.count(_.readsTrTile) * 2

  // The mem-unit will also bypass writes to readers in the RRD stage.
  // NOTE: This does NOT include the ll_wport
  val bypassable_write_port_mask = exe_units.withFilter(x => x.writesIrf).map(u => u.bypassable)
}
