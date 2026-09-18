package kalman

import chisel3._
import chisel3.util._

// Row-major 2x2 fixed-point (Q16.16) matrix.
class Matrix2x2 extends Bundle {
  val m00 = SInt(FixedPoint.WIDTH.W)
  val m01 = SInt(FixedPoint.WIDTH.W)
  val m10 = SInt(FixedPoint.WIDTH.W)
  val m11 = SInt(FixedPoint.WIDTH.W)
}

object Matrix2x2FixedMul {
  // 8 parallel FixedPointMul terms + a saturating add (registered sum, then clamp into the output register).
  val LATENCY = FixedPointMul.LATENCY + satAdd.LATENCY + 1
}

// Generic pipelined 2x2 x 2x2 fixed-point matrix multiply: y = a * b.
// 8 parallel FixedPointMul dot-product terms (FixedPointMul.LATENCY cycles) + a
// saturating add per output element (registered sum + clamp into the output register:
// 2 cycles), fully pipelined (new
// inputs accepted every cycle). Used for the covariance-predict step F*P*F^T in
// KalmanFilter; tested here against dense, arbitrary matrices to prove it is
// genuinely general-purpose.
class Matrix2x2FixedMul extends Module {
  val io = IO(new Bundle {
    val a      = Input(new Matrix2x2)
    val b      = Input(new Matrix2x2)
    val valid  = Input(Bool())
    val y      = Output(new Matrix2x2)
    val yValid = Output(Bool())
  })

  def term(x: SInt, y: SInt): (SInt, Bool) = {
    val m = Module(new FixedPointMul)
    m.io.a := x
    m.io.b := y
    m.io.valid := io.valid
    (m.io.y, m.io.yValid)
  }

  val (p00a, v00a) = term(io.a.m00, io.b.m00)
  val (p00b, _)     = term(io.a.m01, io.b.m10)
  val (p01a, _)     = term(io.a.m00, io.b.m01)
  val (p01b, _)     = term(io.a.m01, io.b.m11)
  val (p10a, _)     = term(io.a.m10, io.b.m00)
  val (p10b, _)     = term(io.a.m11, io.b.m10)
  val (p11a, _)     = term(io.a.m10, io.b.m01)
  val (p11b, _)     = term(io.a.m11, io.b.m11)

  def reg32(next: SInt): SInt = { val r = Reg(SInt(FixedPoint.WIDTH.W)); r := next; r }
  val sumValidReg = ShiftRegister(v00a, satAdd.LATENCY + 1, false.B, true.B)
  val y00Reg = reg32(satAdd(p00a, p00b))
  val y01Reg = reg32(satAdd(p01a, p01b))
  val y10Reg = reg32(satAdd(p10a, p10b))
  val y11Reg = reg32(satAdd(p11a, p11b))

  io.y.m00  := y00Reg
  io.y.m01  := y01Reg
  io.y.m10  := y10Reg
  io.y.m11  := y11Reg
  io.yValid := sumValidReg
}
