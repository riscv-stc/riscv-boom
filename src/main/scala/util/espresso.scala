// Thanks to sequencer@github.com
// Refer to: https://github.com/chipsalliance/chisel3/pull/1914/files
//
// SPDX-License-Identifier: Apache-2.0

package boom.util

import chisel3._
import chisel3.util._
import logger.LazyLogging
import scala.collection.mutable.{ArrayBuffer, Map}

object bpStr {
  def apply(bitPat: BitPat): String = {
    val width = bitPat.getWidth
    val ret = "" + (0 until width).map(i =>
                if (((bitPat.mask >> i) & 1) == 1) if (((bitPat.value >> i) & 1) == 1)  "1" else "0" else "?"
              ).reverse.reduce(_ + _)
    ret
  }
}

object pla {

  /** Construct a [[https://en.wikipedia.org/wiki/Programmable_logic_array]] from specified table.
    *
    * Each position in the input matrix corresponds to an input variable where
    * `0` implies the corresponding input literal appears complemented in the product term.
    * `1` implies the input literal appears uncomplemented in the product term
    * `?` implies the input literal does not appear in the product term.
    *
    * For each output
    * a `1` means this product term makes the function value to `1`
    * and a `0` or `?` means this product term make the function value to `0`
    *
    * @param table A [[Seq]] of inputs -> outputs mapping
    * @param invert A [[BitPat]] specify which bit of the output should be inverted. `1` means the correspond position
    *               of the output should be inverted in the PLA, a `0` or a `?` means direct output from the OR matrix.
    * @return the (input, output) [[Wire]] of [[UInt]] of the constructed pla.
    * {{{
    *   // A 1-of-8 decoder (like the 74xx138) can be constructed as follow
    *   val (inputs, outputs) = pla(Seq(
    *     (BitPat("b000"), BitPat("b00000001")),
    *     (BitPat("b001"), BitPat("b00000010")),
    *     (BitPat("b010"), BitPat("b00000100")),
    *     (BitPat("b011"), BitPat("b00001000")),
    *     (BitPat("b100"), BitPat("b00010000")),
    *     (BitPat("b101"), BitPat("b00100000")),
    *     (BitPat("b110"), BitPat("b01000000")),
    *     (BitPat("b111"), BitPat("b10000000")),
    *   ))
    * }}}
    */
  def apply(table: Seq[(BitPat, BitPat)], invert: BitPat = BitPat("b0")): (UInt, UInt) = {
    require(table.nonEmpty, "pla table must not be empty")

    val (inputTerms, outputTerms) = table.unzip
    require(
      inputTerms.map(_.getWidth).distinct.size == 1,
      "all `BitPat`s in the input part of specified PLA table must have the same width"
    )
    require(
      outputTerms.map(_.getWidth).distinct.size == 1,
      "all `BitPat`s in the output part of specified PLA table must have the same width"
    )

    // now all inputs / outputs have the same width
    val numberOfInputs = inputTerms.head.getWidth
    val numberOfOutputs = outputTerms.head.getWidth

    val inverterMask = invert.value & invert.mask
    if (inverterMask.bitCount != 0)
      require(invert.getWidth == numberOfOutputs,
        "non-zero inverter mask must have the same width as the output part of specified PLA table"
      )

    // input wires of the generated PLA
    val inputs = Wire(UInt(numberOfInputs.W))
    val invInputs = ~inputs

    // output wires of the generated PLA
    val outputs = Wire(UInt(numberOfOutputs.W))

    // the AND matrix
    // use `term -> AND line` map to reuse AND matrix output lines
    val andMatrixOutputs = Map[String, Bool]()
    inputTerms.map { t =>
      andMatrixOutputs += t.toString -> Cat(
        Seq
          .tabulate(numberOfInputs) { i =>
            if (t.mask.testBit(i)) {
              Some(
                if (t.value.testBit(i)) inputs(i)
                else invInputs(i)
              )
            } else {
              None
            }
          }
          .flatten
      ).andR()
    }

    // the OR matrix
    val orMatrixOutputs: UInt = Cat(
        Seq
          .tabulate(numberOfOutputs) { i =>
            val andMatrixLines = table
              // OR matrix composed by input terms which makes this output bit a `1`
              .filter {
                case (_, or) => or.mask.testBit(i) && or.value.testBit(i)
              }.map {
                case (inputTerm, _) =>
                  andMatrixOutputs(inputTerm.toString)
              }
            if (andMatrixLines.isEmpty) false.B
            else Cat(andMatrixLines).orR()
          }
          .reverse
      )

    // the INV matrix, useful for decoders
    val invMatrixOutputs: UInt = Cat(
      Seq
        .tabulate(numberOfOutputs) { i =>
          if (inverterMask.testBit(i)) ~orMatrixOutputs(i)
          else                          orMatrixOutputs(i)
        }
        .reverse
    )

    outputs := invMatrixOutputs

    (inputs, outputs)
  }
}

final class TruthTable(val table: Map[BitPat, BitPat], val default: BitPat) {
  def inputWidth = table.head._1.getWidth
  def outputWidth = table.head._2.getWidth

  override def toString: String = {
    def writeRow(map: (BitPat, BitPat)): String =
      s"${bpStr(map._1)}->${bpStr(map._2)}"
    (table.map(writeRow) ++ Seq(s"${" "*(inputWidth + 2)}${bpStr(default)}")).toSeq.sorted.mkString("\n")
  }

  def copy(table: Map[BitPat, BitPat] = this.table, default: BitPat = this.default) = new TruthTable(table, default)

  override def equals(y: Any): Boolean = {
    y match {
      case y: TruthTable => toString == y.toString
      case _ => false
    }
  }
}

object TruthTable {
  /** Parse TruthTable from its string representation. */
  def apply(tableString: String): TruthTable = {
    TruthTable(
      tableString
        .split("\n")
        .filter(_.contains("->"))
        .map(_.split("->").map(str => BitPat(s"b$str")))
        .map(bps => bps(0) -> bps(1))
        .toSeq,
      BitPat(s"b${tableString.split("\n").filterNot(_.contains("->")).head.replace(" ", "")}")
    )
  }

  /** Convert a table and default output into a [[TruthTable]]. */
  def apply(table: Iterable[(BitPat, BitPat)], default: BitPat): TruthTable = {
    require(table.map(_._1.getWidth).toSet.size == 1, "input width not equal.")
    val t2wset = table.map(_._2.getWidth).toSet
    require(t2wset.size == 1, s"output width not equal.")
    val outputWidth = table.map(_._2.getWidth).head
    val ttmap = Map[BitPat, BitPat]()
    table.toSeq.groupBy(_._1.toString).map { case (key, values) =>
      // merge same input inputs.
      ttmap += values.head._1 -> BitPat(s"b${
        Seq.tabulate(outputWidth) { i =>
          val outputSet = values.map(_._2)
            .map(bpStr.apply)
            .map(_ (i))
            .toSet
            .filterNot(_ == '?')
          require(outputSet.size != 2, s"TruthTable conflict in :\n${values.map { case (i, o) => s"${bpStr(i)}->${bpStr(o)}" }.mkString("\n")}")
          outputSet.headOption.getOrElse('?')
        }.mkString
      }")
    }
    new TruthTable(ttmap, default)
  }

  /** consume 1 table, split it into up to 3 tables with the same default bits.
    *
    * @return table and its indexes from original bits.
    * @note
    * Since most of minimizer(like espresso) cannot handle a multiple default table.
    * It is useful to split a table into 3 tables based on the default type.
    */
  def split( table: TruthTable): Seq[(TruthTable, Seq[Int])] = {
    def bpFilter(bitPat: BitPat, indexes: Seq[Int]): BitPat =
      BitPat(s"b${bpStr(bitPat).zipWithIndex.filter(b => indexes.contains(b._2)).map(_._1).mkString}")

    def tableFilter(indexes: Seq[Int]): Option[(TruthTable, Seq[Int])] = {
      if(indexes.nonEmpty) Some((TruthTable(
        table.table.map { case (in, out) => in -> bpFilter(out, indexes) },
        bpFilter(table.default, indexes)
      ), indexes)) else None
    }

    def index(bitPat: BitPat, bpType: Char): Seq[Int] =
      bpStr(bitPat).zipWithIndex.filter(_._1 == bpType).map(_._2)

    Seq('1', '0', '?').flatMap(t => tableFilter(index(table.default, t)))
  }

  /** consume tables, merge it into single table with different default bits.
    *
    * @note
    * Since most of minimizer(like espresso) cannot handle a multiple default table.
    * It is useful to split a table into 3 tables based on the default type.
    */
  def merge( tables: Seq[(TruthTable, Seq[Int])]): TruthTable = {
    def reIndex(bitPat: BitPat, table: TruthTable, indexes: Seq[Int]): Seq[(Char, Int)] =
      bpStr(table.table.getOrElse(bitPat, BitPat.dontCare(indexes.size))).zip(indexes)
    def bitPat(indexedChar: Seq[(Char, Int)]) = BitPat(s"b${indexedChar
      .sortBy(_._2)
      .map(_._1)
      .mkString}")
    TruthTable(
      tables
        .flatMap(_._1.table.keys)
        .map { key =>
          key -> bitPat(tables.flatMap { case (table, indexes) => reIndex(key, table, indexes) })
        }
        .toMap,
      bitPat(tables.flatMap { case (table, indexes) => bpStr(table.default).zip(indexes) })
    )
  }
}

abstract class Minimizer {
  /** Minimize a multi-input multi-output logic function given by the truth table `table`, with function output values
    * on unspecified inputs treated as `default`, and return a minimized PLA-like representation of the function.
    *
    * Each bit of `table[]._1` encodes one 1-bit input variable of the logic function, and each bit of `default` and
    * `table[]._2` represents one 1-bit output value of the function.
    *
    * @param table    Truth table, can have don't cares in both inputs and outputs, specified as [(inputs, outputs), ...]
    * @return         Minimized truth table, [(inputs, outputs), ...]
    *
    * @example {{{
    *          minimize(BitPat("b?"), Seq(
    *              (BitPat("b000"), BitPat("b0")),
    *              // (BitPat("b001"), BitPat("b?")),  // same as default, can be omitted
    *              // (BitPat("b010"), BitPat("b?")),  // same as default, can be omitted
    *              (BitPat("b011"), BitPat("b0")),
    *              (BitPat("b100"), BitPat("b1")),
    *              (BitPat("b101"), BitPat("b1")),
    *              (BitPat("b110"), BitPat("b0")),
    *              (BitPat("b111"), BitPat("b1")),
    *          ))
    * }}}
    */
  def minimize(table: TruthTable): TruthTable
}

object EspressoMinimizer extends Minimizer with LazyLogging {
  def minimize(table: TruthTable): TruthTable =
    TruthTable.merge(TruthTable.split(table).map{case (table, indexes) => (espresso(table), indexes)})

  def espresso(table: TruthTable): TruthTable = {
    def writeTable(table: TruthTable): String = {
      def invert(string: String) = string
        .replace('0', 't')
        .replace('1', '0')
        .replace('t', '1')
      val defaultType: Char = {
        val t = bpStr(table.default).toCharArray.distinct
        require(t.length == 1, "Internal Error: espresso only accept unified default type.")
        t.head
      }
      val tableType: String = defaultType match {
        case '?' => "fr"
        case _ => "fd"
      }
      val rawTable = table
        .toString
        .split("\n")
        .filter(_.contains("->"))
        .mkString("\n")
        .replace("->", " ")
        .replace('?', '-')
      // invert all output, since espresso cannot handle default is on.
      val invertRawTable = rawTable
        .split("\n")
        .map(_.split(" "))
        .map(row => s"${row(0)} ${invert(row(1))}")
        .mkString("\n")
      s""".i ${table.inputWidth}
         |.o ${table.outputWidth}
         |.type ${tableType}
         |""".stripMargin ++ (if (defaultType == '1') invertRawTable else rawTable)
    }

    def readTable(espressoTable: String): Map[BitPat, BitPat] = {
      val ret = Map[BitPat, BitPat]()
      def bitPat(espresso: String): BitPat = BitPat("b" + espresso.replace('-', '?'))

      espressoTable
        .split("\n")
        .filterNot(_.startsWith("."))
        .map(_.split(' '))
        .map(row => ret += bitPat(row(0)) -> bitPat(row(1)))

      ret
    }

    // Since Espresso don't implements pipe, we use a temp file to do so.
    val input = writeTable(table)
    logger.trace(s"""espresso input table:
                    |$input
                    |""".stripMargin)
    val f = os.temp(input)
    println("Calling Espresso heuristic minimizer with " + f.toString + s" (${table.outputWidth}.W)")
    val o = os.proc("espresso", f).call().out.chunks.mkString
    logger.trace(s"""espresso output table:
                    |$o
                    |""".stripMargin)
    TruthTable(readTable(o), table.default)
  }
}

object BoomDecoder extends LazyLogging {
  def apply(minimizer: Minimizer, input: UInt, truthTable: TruthTable): UInt = {
    val minimizedTable = minimizer.minimize(truthTable)
    val (plaInput, plaOutput) =
      pla(minimizedTable.table.toSeq, BitPat(minimizedTable.default.value.U(minimizedTable.default.getWidth.W)))
    plaInput := input
    plaOutput
  }
  def apply(addr: UInt, default: BitPat, mapping: Iterable[(BitPat, BitPat)]): UInt = {
    apply(EspressoMinimizer, addr, TruthTable(mapping, default))
  }
  def apply(addr: UInt, default: Seq[BitPat], mappingIn: Iterable[(BitPat, Seq[BitPat])]): Seq[UInt] = {
//  val mapping = ArrayBuffer.fill(default.size)(ArrayBuffer[(BitPat, BitPat)]())
//  for ((key, values) <- mappingIn)
//    for ((value, i) <- values zipWithIndex)
//      mapping(i) += key -> value
//  for ((thisDefault, thisMapping) <- default zip mapping)
//    yield apply(addr, thisDefault, thisMapping)

    val sizes = default.map(_.getWidth)
    val catdef = BitPat("b" + default.map(bpStr.apply).reduce(_+_))
    val catmap = mappingIn.map {case (k, v) => k -> BitPat("b" + v.map(bpStr.apply).reduce(_+_))}
    val minied = apply(addr, catdef, catmap)
    val ret = ArrayBuffer[UInt]()
    var acc = 0
    for (s <- sizes.reverse) {
      ret += minied(acc + s - 1, acc)
      acc += s
    }
    ret.reverse
  }
  def apply(addr: UInt, default: Seq[BitPat], mappingIn: List[(UInt, Seq[BitPat])]): Seq[UInt] =
    apply(addr, default, mappingIn.map(m => (BitPat(m._1), m._2)).asInstanceOf[Iterable[(BitPat, Seq[BitPat])]])
}
