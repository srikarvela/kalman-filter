package kalman

import chisel3._
import chisel3.util._

object Reciprocal {
  // normalize (3: leading-zero count, both shift directions, select)
  // + seed multiply (L) + seed subtract (satSub 1 + register 1)
  // + iterations x (multiply L + subtract satSub.LATENCY + multiply L)
  // + denormalize (3: both shift directions, select, clamp)
  def latency(iterations: Int): Int =
    3 + FixedPointMul.LATENCY + satSub.LATENCY + 1 +
      iterations * (2 * FixedPointMul.LATENCY + satSub.LATENCY) + 3
}

// Pipelined Q16.16 reciprocal (1/x) for strictly-positive x, via Newton-Raphson.
// Avoids any hardware divider: the only division the Kalman filter needs is the
// scalar gain K = P_pred_col0 / S, since H = [1,0] makes the measurement-side matrix
// math trivial (selection, not a real matrix inverse).
//
// Algorithm: normalize x into m in [0.5, 1.0) by locating its highest set bit and
// shifting it to bit 15 (the Q16.16 bit for 0.5), recording the shift amount `e` such
// that x = m * 2^e. Seed y0 = 1/m via the minimax linear approximation
// y0 = 48/17 - 32/17*m (accurate to within ~1/17 over [0.5,1.0), the standard NR seed
// for this domain). Refine with `iterations` rounds of y_{n+1} = y_n*(2 - m*y_n), each
// of which asymptotically doubles the number of correct bits. Finally rescale by 2^-e
// (shifting by the exact same amount/direction as the normalization step, since the
// exponent flip of the reciprocal exactly cancels the sign flip of "-e") to undo the
// normalization and produce 1/x.
//
// Pipelining: the leading-zero count, the two barrel-shift directions and the
// left/right select are separate register stages (the shift amount otherwise fans out
// to both 32-bit shifters at once, which was a 4.2 ns path), as are the denormalizing
// shift, its select and the final clamp, so no register-to-register path holds more
// than one shifter or one saturating add/sub.
class Reciprocal(val iterations: Int = 3) extends Module {
  val W = FixedPoint.WIDTH
  val L = FixedPointMul.LATENCY
  val S = satSub.LATENCY
  def reg32(next: SInt): SInt = { val r = Reg(SInt(W.W)); r := next; r }

  val io = IO(new Bundle {
    val x      = Input(SInt(W.W))  // Q16.16, must be > 0
    val valid  = Input(Bool())
    val y      = Output(SInt(W.W)) // Q16.16, approx 1/x
    val yValid = Output(Bool())
  })

  // shiftedTo15(v, amt) = amt < 0 ? v << |amt| : v >> |amt|, computed as two register stages:
  // both directions in the first, the select in the second. The shift amount is held in
  // REP replicated registers, one per 8-bit slice of the shifter outputs, so that no
  // amount bit fans out to more than 16 mux LUTs (with a single copy the 64-way fan-out
  // was 3.3 ns of routing at 4.000 ns). tcl/kalman_synth.tcl runs synth_design with
  // -keep_equivalent_registers so Vivado does not merge the copies back together.
  val REP   = 4
  val SLICE = W / REP
  def shiftMag(amt: SInt): UInt = Mux(amt < 0.S, (-amt).asUInt, amt.asUInt)
  def amtCopies(pre: SInt): Seq[SInt] = Seq.fill(REP)(RegNext(pre))
  class ShiftPair extends Bundle {
    val left  = UInt(W.W)
    val right = UInt(W.W)
    val neg   = Bool()
  }
  def shiftBoth(v: UInt, copies: Seq[SInt]): ShiftPair = {
    val p = Wire(new ShiftPair)
    val lefts  = copies.map(a => (v << shiftMag(a))(W - 1, 0))
    val rights = copies.map(a => (v >> shiftMag(a))(W - 1, 0))
    p.left  := Cat((REP - 1 to 0 by -1).map(j => lefts(j)(SLICE * j + SLICE - 1, SLICE * j)))
    p.right := Cat((REP - 1 to 0 by -1).map(j => rights(j)(SLICE * j + SLICE - 1, SLICE * j)))
    p.neg   := copies.head < 0.S
    p
  }
  def shiftSelect(p: ShiftPair): UInt = Mux(p.neg, p.left, p.right)

  // ---- Stage N0: locate the highest set bit ----
  val rawMag       = io.x.asUInt
  val leadingZeros = PriorityEncoder(Reverse(rawMag))
  val highestBit   = (W - 1).U - leadingZeros                      // 0..31
  val shiftAmt     = highestBit.zext - (FixedPoint.FRAC_BITS - 1).S // signed; target bit 15

  val magRegN0        = RegNext(rawMag)
  val shiftAmtCopiesN0 = amtCopies(shiftAmt)   // REP copies; copy 0 also feeds the exponent delay line
  val shiftAmtRegN0   = shiftAmtCopiesN0.head
  val validRegN0      = RegNext(io.valid, false.B)

  // ---- Stage N1: both shift directions ----
  val normPairN1    = RegNext(shiftBoth(magRegN0, shiftAmtCopiesN0))
  val shiftAmtRegN1 = RegNext(shiftAmtRegN0)
  val validRegN1    = RegNext(validRegN0, false.B)

  // ---- Stage N2: select -> normalized mantissa in [0x8000, 0xFFFF] (Q16.16 in [0.5, 1.0)) ----
  val normReg0     = RegNext(shiftSelect(normPairN1).asSInt)
  val shiftAmtReg0 = RegNext(shiftAmtRegN1)
  val validReg0    = RegNext(validRegN1, false.B)

  // ---- Seed y0 = 48/17 - 32/17 * normalized (1 FixedPointMul + 1 registered subtract) ----
  val C1 = FixedPoint.toFixed(48.0 / 17.0)
  val C2 = FixedPoint.toFixed(32.0 / 17.0)

  val seedMul = Module(new FixedPointMul)
  seedMul.io.a     := C2.S(W.W)
  seedMul.io.b     := normReg0
  seedMul.io.valid := validReg0

  val normAtSeed     = ShiftRegister(normReg0, L)
  val shiftAmtAtSeed = ShiftRegister(shiftAmtReg0, L)

  val y0Reg             = reg32(satSub(C1.S(W.W), seedMul.io.y))            // S + 1 cycles after seedMul.io.y
  val normAfterSeed     = ShiftRegister(normAtSeed, S + 1)
  val shiftAmtPreSeed   = ShiftRegister(shiftAmtAtSeed, S)                   // one cycle before shiftAmtAfterSeed
  val shiftAmtAfterSeed = RegNext(shiftAmtPreSeed)
  val validAfterSeed    = ShiftRegister(seedMul.io.yValid, S + 1, false.B, true.B)

  // ---- Newton-Raphson iterations: y_{n+1} = y_n * (2 - m*y_n) (2 muls: 2L cycles each) ----
  var yCur        = y0Reg
  var normCur     = normAfterSeed
  var shiftCur    = shiftAmtAfterSeed
  var shiftCurPre = shiftAmtPreSeed   // shiftCur one cycle earlier, so the denormalizer can register its own copies
  var validCur    = validAfterSeed

  val TWO = FixedPoint.toFixed(2.0)

  for (_ <- 0 until iterations) {
    val mulNY = Module(new FixedPointMul) // m * y_n
    mulNY.io.a     := normCur
    mulNY.io.b     := yCur
    mulNY.io.valid := validCur

    val twoMinusMY  = satSub(TWO.S(W.W), mulNY.io.y) // 2 - m*y_n: registered subtract, clamp into the next multiplier's operand register
    val yCurAligned = ShiftRegister(yCur, L + S)     // realign y_n with mulNY's latency + the subtract

    val mulYNext = Module(new FixedPointMul) // y_n * (2 - m*y_n)
    mulYNext.io.a     := yCurAligned
    mulYNext.io.b     := twoMinusMY
    mulYNext.io.valid := ShiftRegister(mulNY.io.yValid, S, false.B, true.B)

    yCur        = mulYNext.io.y
    normCur     = ShiftRegister(normCur, 2 * L + S)
    shiftCurPre = ShiftRegister(shiftCur, 2 * L + S - 1)
    shiftCur    = RegNext(shiftCurPre)
    validCur    = mulYNext.io.yValid
  }

  // ---- Denormalize: rescale by 2^-shiftAmt (same shift direction as normalization), then clamp ----
  val denormPair   = RegNext(shiftBoth(yCur.asUInt, amtCopies(shiftCurPre)))   // copies land in the same cycle as shiftCur
  val validDenormA = RegNext(validCur, false.B)
  val denormReg    = RegNext(shiftSelect(denormPair).asSInt)
  val validDenorm  = RegNext(validDenormA, false.B)

  val hi = FixedPoint.MAX.S(W.W)
  val lo = FixedPoint.MIN.S(W.W)
  val clamped = Mux(denormReg > hi, hi, Mux(denormReg < lo, lo, denormReg))

  val yOut      = RegNext(clamped)
  val yValidOut = RegNext(validDenorm, false.B)

  io.y      := yOut
  io.yValid := yValidOut
}
