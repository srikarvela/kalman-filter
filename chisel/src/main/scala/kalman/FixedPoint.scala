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

  // Clamp an exact (widened) SInt result to the 32-bit signed range. A value fits in
  // 32 signed bits iff every bit from the sign bit down to bit 31 is equal, so the
  // decision is an AND/OR reduce of the top bits (one or two LUT levels), not a
  // magnitude comparator (which Vivado builds as another carry chain).
  def clamp(wide: SInt): SInt = {
    val n = wide.getWidth
    require(n > WIDTH, s"clamp expects a widened value, got $n bits")
    val top     = wide(n - 1, WIDTH - 1)
    val inRange = top.andR || !top.orR
    val neg     = wide(n - 1)
    Mux(inRange, wide(WIDTH - 1, 0).asSInt, Mux(neg, MIN.S(WIDTH.W), MAX.S(WIDTH.W)))
  }

  // Register an exact sum with an explicit width (RegNext would leave the width to
  // inference, which clamp cannot query at elaboration time).
  def regWide(next: SInt): SInt = { val r = Reg(SInt(next.getWidth.W)); r := next; r }
}

// Saturating Q16.16 add: the exact 33-bit sum is registered, then clamped. Two register
// stages of logic (one carry chain, then the clamp mux into whatever register consumes
// the result), so a saturating add costs one cycle of latency. Bit-identical to a
// combinational clamp(a +& b).
object satAdd {
  def apply(a: SInt, b: SInt): SInt = FixedPoint.clamp(FixedPoint.regWide(a +& b))
  val LATENCY = 1
}

// Saturating Q16.16 subtract, a - b, same structure as satAdd. Bit-identical to
// satAdd(a, -b) (both are exact before the clamp) but a single subtract chain instead
// of a negate chain feeding an add chain.
object satSub {
  def apply(a: SInt, b: SInt): SInt = FixedPoint.clamp(FixedPoint.regWide(a -& b))
  val LATENCY = 1
}

object FixedPointMul {
  // Cycles from io.a/io.b/io.valid to io.y/io.yValid.
  val LATENCY = 8
}

// Pipelined Q16.16 x Q16.16 -> Q16.16 multiplier. 8-cycle latency, II=1, bit-exact
// with the single-cycle-product version it replaces (same 64-bit product, same
// round-half-up at bit 15, same saturation).
//
// The product is decomposed so that each partial product fits one DSP48E1 (25x18)
// and Vivado can absorb the surrounding registers as the DSP's A/B, M and P
// pipeline registers, instead of building one 32x32 product as a cascade of four
// DSPs with an un-registered internal adder chain (the 16 ns path in the first
// timing report):
//
//   a = aHi * 2^16 + aLo   (aHi: signed upper 16 bits, aLo: unsigned lower 16 bits)
//   a*b = aHi*bHi * 2^32 + (aHi*bLo + aLo*bHi) * 2^16 + aLo*bLo
//
//   S0  register a, b                           (DSP A/B registers)
//   S1  four partial products                   (DSP M registers)
//   S2  partial products again                  (DSP P registers)
//   S3  mid = hl + lh;  ll + round bit
//   S4  low  = (mid << 16) + ll_rounded
//   S5  full = (hh << 32) + low                 (each stage: one carry chain <= 36 bits)
//   S6  arithmetic shift by 16; register the 32-bit slice and the overflow flags
//   S7  saturate (a 32-bit mux on two registered flags) -> io.y
class FixedPointMul extends Module {
  val io = IO(new Bundle {
    val a     = Input(SInt(FixedPoint.WIDTH.W))
    val b     = Input(SInt(FixedPoint.WIDTH.W))
    val valid = Input(Bool())
    val y     = Output(SInt(FixedPoint.WIDTH.W))
    val yValid = Output(Bool())
  })
  val W = FixedPoint.WIDTH
  val H = W / 2

  // S0: input registers
  val aReg = RegNext(io.a)
  val bReg = RegNext(io.b)
  val v0   = RegNext(io.valid, false.B)

  val aHi = aReg(W - 1, H).asSInt                 // 16-bit signed
  val aLo = Cat(0.U(1.W), aReg(H - 1, 0)).asSInt  // 17-bit, non-negative
  val bHi = bReg(W - 1, H).asSInt
  val bLo = Cat(0.U(1.W), bReg(H - 1, 0)).asSInt

  // S1: partial products (each <= 17x17 signed: one DSP48E1)
  val hh1 = RegNext(aHi * bHi)   // 32 bits
  val hl1 = RegNext(aHi * bLo)   // 33 bits
  val lh1 = RegNext(aLo * bHi)   // 33 bits
  val ll1 = RegNext(aLo * bLo)   // 34 bits, non-negative
  val v1  = RegNext(v0, false.B)

  // S2: second product register (DSP P register)
  val hh2 = RegNext(hh1)
  val hl2 = RegNext(hl1)
  val lh2 = RegNext(lh1)
  val ll2 = RegNext(ll1)
  val v2  = RegNext(v1, false.B)

  // S3: combine the two middle terms; fold the round-half-up constant into the low term
  val roundBit = (BigInt(1) << (FixedPoint.FRAC_BITS - 1)).S((FixedPoint.FRAC_BITS + 1).W)
  val hh3  = RegNext(hh2)
  val mid3 = RegNext(hl2 +& lh2)          // 34 bits
  val llr3 = RegNext(ll2 +& roundBit)     // 35 bits
  val v3   = RegNext(v2, false.B)

  // S4: low part = (mid << 16) + (ll + 2^15)      (51-bit result, carries only above bit 15)
  val low4 = Reg(SInt(52.W))
  low4 := (mid3 << H) +& llr3
  val hh4 = RegNext(hh3)
  val v4  = RegNext(v3, false.B)

  // S5: full rounded product = a*b + 2^15 = (hh << 32) + low  (exact, 65 bits)
  val full5 = Reg(SInt((2 * W + 2).W))
  full5 := (hh4 << (2 * H)) +& low4
  val v5 = RegNext(v4, false.B)

  // S6: shift out the fraction bits; register the 32-bit slice and the overflow decision.
  //     (Deciding and applying the saturation in one stage put a 19-bit AND/OR reduce on
  //     the output registers' set/reset pins, the last failing path at 4.000 ns.)
  val shifted = (full5 >> FixedPoint.FRAC_BITS).asSInt
  val n       = shifted.getWidth
  val top     = shifted(n - 1, W - 1)
  val inRange6 = RegNext(top.andR || !top.orR)
  val neg6     = RegNext(shifted(n - 1))
  val val6     = RegNext(shifted(W - 1, 0).asSInt)
  val v6       = RegNext(v5, false.B)

  // S7: saturate
  val yReg      = RegNext(Mux(inRange6, val6, Mux(neg6, FixedPoint.MIN.S(W.W), FixedPoint.MAX.S(W.W))))
  val yValidReg = RegNext(v6, false.B)

  io.y      := yReg
  io.yValid := yValidReg
}
