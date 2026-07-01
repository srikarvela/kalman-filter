package kalman

import chisel3._
import chisel3.util._

// Q16.16 signed fixed point: 32-bit SInt, 16 integer bits (incl. sign), 16 fractional bits.
object FixedPoint {
  val FRAC_BITS  = 16
  val WIDTH      = 32
  val ONE        = BigInt(1) << FRAC_BITS
  val MAX        = (BigInt(1) << (WIDTH - 1)) - 1
  val MIN        = -(BigInt(1) << (WIDTH - 1))

  def toFixed(d: Double): BigInt = BigInt(math.round(d * (1L << FRAC_BITS)))
  def fromFixed(v: BigInt): Double = v.toDouble / (1L << FRAC_BITS)
}

// Saturating Q16.16 add: widens by 1 bit, clamps to the 32-bit signed range.
object satAdd {
  def apply(a: SInt, b: SInt): SInt = {
    val wide = a +& b // (max(a.width,b.width)+1)-bit exact sum
    val hi = FixedPoint.MAX.S(wide.getWidth.W)
    val lo = FixedPoint.MIN.S(wide.getWidth.W)
    val clamped = Mux(wide > hi, hi, Mux(wide < lo, lo, wide))
    clamped(FixedPoint.WIDTH - 1, 0).asSInt
  }
}

// Pipelined Q16.16 x Q16.16 -> Q16.16 multiplier. 2-cycle latency, II=1.
// Stage 1: register the full 64-bit product.
// Stage 2: round-half-up, arithmetic shift by FRAC_BITS, saturate to 32 bits.
class FixedPointMul extends Module {
  val io = IO(new Bundle {
    val a     = Input(SInt(FixedPoint.WIDTH.W))
    val b     = Input(SInt(FixedPoint.WIDTH.W))
    val valid = Input(Bool())
    val y     = Output(SInt(FixedPoint.WIDTH.W))
    val yValid = Output(Bool())
  })

  // Stage 1: multiply. Width is declared explicitly (32+32=64 bits) rather than inferred,
  // since a Reg's width is otherwise left to FIRRTL's width-inference pass and isn't
  // queryable via .getWidth immediately after elaboration.
  val PROD_WIDTH = 2 * FixedPoint.WIDTH
  val prodReg  = Reg(SInt(PROD_WIDTH.W))
  prodReg     := io.a * io.b
  val valid1   = RegNext(io.valid, false.B)

  // Stage 2: round + shift + saturate
  val roundBit = (BigInt(1) << (FixedPoint.FRAC_BITS - 1)).S(prodReg.getWidth.W)
  val rounded  = prodReg +& roundBit
  val shifted  = (rounded >> FixedPoint.FRAC_BITS).asSInt

  val hi = FixedPoint.MAX.S(shifted.getWidth.W)
  val lo = FixedPoint.MIN.S(shifted.getWidth.W)
  val clamped = Mux(shifted > hi, hi, Mux(shifted < lo, lo, shifted))

  val yReg      = RegNext(clamped(FixedPoint.WIDTH - 1, 0).asSInt)
  val yValidReg = RegNext(valid1, false.B)

  io.y      := yReg
  io.yValid := yValidReg
}
