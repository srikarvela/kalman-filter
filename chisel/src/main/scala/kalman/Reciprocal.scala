package kalman

import chisel3._
import chisel3.util._

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
class Reciprocal(val iterations: Int = 3) extends Module {
  val W = FixedPoint.WIDTH

  val io = IO(new Bundle {
    val x      = Input(SInt(W.W))  // Q16.16, must be > 0
    val valid  = Input(Bool())
    val y      = Output(SInt(W.W)) // Q16.16, approx 1/x
    val yValid = Output(Bool())
  })

  // ---- Stage 0 (combinational): locate highest set bit, compute normalized mantissa ----
  val rawMag       = io.x.asUInt
  val leadingZeros = PriorityEncoder(Reverse(rawMag))
  val highestBit   = (W - 1).U - leadingZeros                      // 0..31
  val shiftAmt     = highestBit.zext - (FixedPoint.FRAC_BITS - 1).S // signed; target bit 15

  def shiftedTo15(v: UInt, amt: SInt): UInt = {
    val mag = Mux(amt < 0.S, (-amt).asUInt, amt.asUInt)
    Mux(amt < 0.S, (v << mag)(W - 1, 0), (v >> mag)(W - 1, 0))
  }

  val normalizedRaw = shiftedTo15(rawMag, shiftAmt).asSInt // Q16.16, in [0x8000, 0xFFFF]

  val normReg0     = RegNext(normalizedRaw)
  val shiftAmtReg0 = RegNext(shiftAmt)
  val validReg0    = RegNext(io.valid, false.B)

  // ---- Stage 1: seed y0 = 48/17 - 32/17 * normalized (1 FixedPointMul: 2 cycles + 1 reg) ----
  val C1 = FixedPoint.toFixed(48.0 / 17.0)
  val C2 = FixedPoint.toFixed(32.0 / 17.0)

  val seedMul = Module(new FixedPointMul)
  seedMul.io.a     := C2.S(W.W)
  seedMul.io.b     := normReg0
  seedMul.io.valid := validReg0

  val normAtSeed     = ShiftRegister(normReg0, 2)
  val shiftAmtAtSeed = ShiftRegister(shiftAmtReg0, 2)

  val y0Reg          = RegNext(satAdd(C1.S(W.W), -seedMul.io.y))
  val normAfterSeed  = RegNext(normAtSeed)
  val shiftAmtAfterSeed = RegNext(shiftAmtAtSeed)
  val validAfterSeed = RegNext(seedMul.io.yValid, false.B)

  // ---- Newton-Raphson iterations: y_{n+1} = y_n * (2 - m*y_n) (2 muls: 4 cycles each) ----
  var yCur     = y0Reg
  var normCur  = normAfterSeed
  var shiftCur = shiftAmtAfterSeed
  var validCur = validAfterSeed

  val TWO = FixedPoint.toFixed(2.0)

  for (_ <- 0 until iterations) {
    val mulNY = Module(new FixedPointMul) // m * y_n
    mulNY.io.a     := normCur
    mulNY.io.b     := yCur
    mulNY.io.valid := validCur

    val twoMinusMY = satAdd(TWO.S(W.W), -mulNY.io.y) // 2 - m*y_n, combinational off mulNY
    val yCurAligned = ShiftRegister(yCur, 2)          // realign y_n with mulNY's 2-cycle latency

    val mulYNext = Module(new FixedPointMul) // y_n * (2 - m*y_n)
    mulYNext.io.a     := yCurAligned
    mulYNext.io.b     := twoMinusMY
    mulYNext.io.valid := mulNY.io.yValid

    yCur     = mulYNext.io.y
    normCur  = ShiftRegister(ShiftRegister(normCur, 2), 2)
    shiftCur = ShiftRegister(ShiftRegister(shiftCur, 2), 2)
    validCur = mulYNext.io.yValid
  }

  // ---- Final: denormalize by rescaling 2^-shiftAmt (same shift direction as normalization) ----
  val denormMag = shiftedTo15(yCur.asUInt, shiftCur)
  val denormSInt = denormMag.asSInt
  val hi = FixedPoint.MAX.S(W.W)
  val lo = FixedPoint.MIN.S(W.W)
  val clamped = Mux(denormSInt > hi, hi, Mux(denormSInt < lo, lo, denormSInt))

  val yOut      = RegNext(clamped)
  val yValidOut = RegNext(validCur, false.B)

  io.y      := yOut
  io.yValid := yValidOut
}
