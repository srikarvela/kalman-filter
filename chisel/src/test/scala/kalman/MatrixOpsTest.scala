package kalman

import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec
import org.scalatest.matchers.should.Matchers

// Independent Scala reference for 2x2 fixed-point matrix multiply, built from the
// already-cross-checked FixedMulRef primitive plus a saturating-add reference.
object Matrix2x2Ref {
  def satAddRef(a: BigInt, b: BigInt): BigInt = {
    val s = a + b
    if (s > FixedPoint.MAX) FixedPoint.MAX
    else if (s < FixedPoint.MIN) FixedPoint.MIN
    else s
  }

  // a, b: (m00, m01, m10, m11). Returns (y00, y01, y10, y11).
  def apply(a: (BigInt, BigInt, BigInt, BigInt), b: (BigInt, BigInt, BigInt, BigInt)):
      (BigInt, BigInt, BigInt, BigInt) = {
    val (a00, a01, a10, a11) = a
    val (b00, b01, b10, b11) = b
    val y00 = satAddRef(FixedMulRef(a00, b00), FixedMulRef(a01, b10))
    val y01 = satAddRef(FixedMulRef(a00, b01), FixedMulRef(a01, b11))
    val y10 = satAddRef(FixedMulRef(a10, b00), FixedMulRef(a11, b10))
    val y11 = satAddRef(FixedMulRef(a10, b01), FixedMulRef(a11, b11))
    (y00, y01, y10, y11)
  }
}

class MatrixOpsTest extends AnyFlatSpec with ChiselScalatestTester with Matchers {
  def pokeMatrix(m: Matrix2x2, v: (BigInt, BigInt, BigInt, BigInt)): Unit = {
    m.m00.poke(v._1.S(32.W))
    m.m01.poke(v._2.S(32.W))
    m.m10.poke(v._3.S(32.W))
    m.m11.poke(v._4.S(32.W))
  }

  def mulOnce(dut: Matrix2x2FixedMul, a: (BigInt, BigInt, BigInt, BigInt), b: (BigInt, BigInt, BigInt, BigInt)):
      (BigInt, BigInt, BigInt, BigInt) = {
    pokeMatrix(dut.io.a, a)
    pokeMatrix(dut.io.b, b)
    dut.io.valid.poke(true.B)
    dut.clock.step(1)
    dut.io.valid.poke(false.B)
    dut.clock.step(2) // 3-cycle total latency
    dut.io.yValid.expect(true.B)
    (dut.io.y.m00.peek().litValue, dut.io.y.m01.peek().litValue,
     dut.io.y.m10.peek().litValue, dut.io.y.m11.peek().litValue)
  }

  def fx(d: Double): BigInt = FixedPoint.toFixed(d)

  behavior of "Matrix2x2FixedMul"

  it should "multiply by the identity matrix" in {
    test(new Matrix2x2FixedMul) { dut =>
      val a = (fx(1.5), fx(-2.25), fx(3.0), fx(0.5))
      val id = (fx(1.0), fx(0.0), fx(0.0), fx(1.0))
      val got = mulOnce(dut, a, id)
      got shouldBe Matrix2x2Ref(a, id)
    }
  }

  it should "multiply two dense arbitrary matrices with negative entries" in {
    test(new Matrix2x2FixedMul) { dut =>
      val a = (fx(1.25), fx(-3.5), fx(2.0), fx(-0.75))
      val b = (fx(-1.0), fx(4.25), fx(0.5), fx(-2.0))
      val got = mulOnce(dut, a, b)
      got shouldBe Matrix2x2Ref(a, b)
    }
  }

  it should "match the reference for the F * P covariance-predict use case (sparse F, dense symmetric P)" in {
    test(new Matrix2x2FixedMul) { dut =>
      // F = [[1, dt], [0, 1]] with dt = 0.004 (matches a 250 MHz-ish sample interval),
      // P = symmetric covariance [[2.0, 0.1], [0.1, 0.5]]
      val f = (fx(1.0), fx(0.004), fx(0.0), fx(1.0))
      val p = (fx(2.0), fx(0.1), fx(0.1), fx(0.5))
      val got = mulOnce(dut, f, p)
      got shouldBe Matrix2x2Ref(f, p)
    }
  }

  it should "saturate on overflowing products" in {
    test(new Matrix2x2FixedMul) { dut =>
      val a = (fx(1000.0), fx(1000.0), fx(1000.0), fx(1000.0))
      val b = (fx(1000.0), fx(1000.0), fx(1000.0), fx(1000.0))
      val got = mulOnce(dut, a, b)
      got shouldBe Matrix2x2Ref(a, b)
      got._1 shouldBe FixedPoint.MAX
    }
  }

  it should "be fully pipelined (accept a new pair of matrices every cycle)" in {
    test(new Matrix2x2FixedMul) { dut =>
      val a0 = (fx(1.0), fx(2.0), fx(3.0), fx(4.0))
      val b0 = (fx(0.5), fx(-1.0), fx(2.0), fx(1.5))
      val a1 = (fx(-1.0), fx(0.25), fx(1.5), fx(-2.0))
      val b1 = (fx(2.0), fx(3.0), fx(-0.5), fx(1.0))

      pokeMatrix(dut.io.a, a0); pokeMatrix(dut.io.b, b0); dut.io.valid.poke(true.B)
      dut.clock.step(1)
      pokeMatrix(dut.io.a, a1); pokeMatrix(dut.io.b, b1); dut.io.valid.poke(true.B)
      dut.clock.step(1)
      dut.io.valid.poke(false.B)
      dut.clock.step(1) // 3 cycles since a0/b0 issued

      dut.io.yValid.expect(true.B)
      (dut.io.y.m00.peek().litValue, dut.io.y.m01.peek().litValue,
       dut.io.y.m10.peek().litValue, dut.io.y.m11.peek().litValue) shouldBe Matrix2x2Ref(a0, b0)

      dut.clock.step(1) // 3 cycles since a1/b1 issued
      dut.io.yValid.expect(true.B)
      (dut.io.y.m00.peek().litValue, dut.io.y.m01.peek().litValue,
       dut.io.y.m10.peek().litValue, dut.io.y.m11.peek().litValue) shouldBe Matrix2x2Ref(a1, b1)
    }
  }
}
