package kalman

import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec
import org.scalatest.matchers.should.Matchers

// Independent Scala reference for the FixedPointMul algorithm (round-half-up, saturate),
// used to cross-check the RTL rather than hand-computing expected numbers per case.
object FixedMulRef {
  def apply(rawA: BigInt, rawB: BigInt): BigInt = {
    val prod    = rawA * rawB
    val rounded = prod + (BigInt(1) << (FixedPoint.FRAC_BITS - 1))
    var shifted = rounded >> FixedPoint.FRAC_BITS
    if (shifted > FixedPoint.MAX) shifted = FixedPoint.MAX
    if (shifted < FixedPoint.MIN) shifted = FixedPoint.MIN
    shifted
  }
}

class FixedPointTest extends AnyFlatSpec with ChiselScalatestTester with Matchers {
  def mulRaw(dut: FixedPointMul, rawA: BigInt, rawB: BigInt): BigInt = {
    dut.io.a.poke(rawA.S(32.W))
    dut.io.b.poke(rawB.S(32.W))
    dut.io.valid.poke(true.B)
    dut.clock.step(1)
    dut.io.valid.poke(false.B)
    dut.clock.step(1)
    dut.io.yValid.expect(true.B)
    dut.io.y.peek().litValue
  }

  def mulD(dut: FixedPointMul, a: Double, b: Double): BigInt =
    mulRaw(dut, FixedPoint.toFixed(a), FixedPoint.toFixed(b))

  behavior of "FixedPointMul"

  it should "multiply positive values correctly (1.5 * 2.5 = 3.75)" in {
    test(new FixedPointMul) { dut =>
      val rawA = FixedPoint.toFixed(1.5)
      val rawB = FixedPoint.toFixed(2.5)
      val got  = mulRaw(dut, rawA, rawB)
      got shouldBe FixedMulRef(rawA, rawB)
      FixedPoint.fromFixed(got) shouldBe 3.75 +- 1e-4
    }
  }

  it should "handle negative operands (-2.0 * 3.0 = -6.0)" in {
    test(new FixedPointMul) { dut =>
      val rawA = FixedPoint.toFixed(-2.0)
      val rawB = FixedPoint.toFixed(3.0)
      val got  = mulRaw(dut, rawA, rawB)
      got shouldBe FixedMulRef(rawA, rawB)
      FixedPoint.fromFixed(got) shouldBe -6.0 +- 1e-4
    }
  }

  it should "handle negative * negative" in {
    test(new FixedPointMul) { dut =>
      val rawA = FixedPoint.toFixed(-1.25)
      val rawB = FixedPoint.toFixed(-4.0)
      val got  = mulRaw(dut, rawA, rawB)
      got shouldBe FixedMulRef(rawA, rawB)
      FixedPoint.fromFixed(got) shouldBe 5.0 +- 1e-4
    }
  }

  it should "round half up at the exact tie boundary (positive)" in {
    test(new FixedPointMul) { dut =>
      // raw product = 1 * 98304 (1.5 in Q16.16) -> low 16 bits exactly 0x8000
      val got = mulRaw(dut, 1, 98304)
      got shouldBe FixedMulRef(1, 98304)
      got shouldBe BigInt(2) // round(1.5) = 2
    }
  }

  it should "round half up at the exact tie boundary (negative)" in {
    test(new FixedPointMul) { dut =>
      val got = mulRaw(dut, -1, 98304)
      got shouldBe FixedMulRef(-1, 98304)
      got shouldBe BigInt(-1) // round-half-up (toward +inf): round(-1.5) = -1
    }
  }

  it should "saturate on positive overflow" in {
    test(new FixedPointMul) { dut =>
      val rawA = FixedPoint.toFixed(1000.0)
      val rawB = FixedPoint.toFixed(1000.0)
      val got  = mulRaw(dut, rawA, rawB)
      got shouldBe FixedPoint.MAX
    }
  }

  it should "saturate on negative overflow" in {
    test(new FixedPointMul) { dut =>
      val rawA = FixedPoint.toFixed(1000.0)
      val rawB = FixedPoint.toFixed(-1000.0)
      val got  = mulRaw(dut, rawA, rawB)
      got shouldBe FixedPoint.MIN
    }
  }

  it should "be fully pipelined (accept a new input every cycle)" in {
    test(new FixedPointMul) { dut =>
      val rawA0 = FixedPoint.toFixed(2.0); val rawB0 = FixedPoint.toFixed(3.0)
      val rawA1 = FixedPoint.toFixed(-1.0); val rawB1 = FixedPoint.toFixed(5.0)

      dut.io.a.poke(rawA0.S); dut.io.b.poke(rawB0.S); dut.io.valid.poke(true.B)
      dut.clock.step(1)
      dut.io.a.poke(rawA1.S); dut.io.b.poke(rawB1.S); dut.io.valid.poke(true.B)
      dut.clock.step(1)
      dut.io.valid.poke(false.B)
      dut.io.yValid.expect(true.B)
      dut.io.y.peek().litValue shouldBe FixedMulRef(rawA0, rawB0)
      dut.clock.step(1)
      dut.io.yValid.expect(true.B)
      dut.io.y.peek().litValue shouldBe FixedMulRef(rawA1, rawB1)
    }
  }

  behavior of "satAdd"

  class SatAddHarness extends Module {
    val io = IO(new Bundle {
      val a = Input(SInt(32.W))
      val b = Input(SInt(32.W))
      val y = Output(SInt(32.W))
    })
    io.y := satAdd(io.a, io.b)
  }

  it should "add normally within range" in {
    test(new SatAddHarness) { dut =>
      dut.io.a.poke(FixedPoint.toFixed(1.5).S)
      dut.io.b.poke(FixedPoint.toFixed(2.25).S)
      dut.io.y.peek().litValue shouldBe FixedPoint.toFixed(3.75)
    }
  }

  it should "saturate on overflow" in {
    test(new SatAddHarness) { dut =>
      dut.io.a.poke(FixedPoint.MAX.S(32.W))
      dut.io.b.poke(FixedPoint.toFixed(10.0).S)
      dut.io.y.peek().litValue shouldBe FixedPoint.MAX
    }
  }

  it should "saturate on underflow" in {
    test(new SatAddHarness) { dut =>
      dut.io.a.poke(FixedPoint.MIN.S(32.W))
      dut.io.b.poke(FixedPoint.toFixed(-10.0).S)
      dut.io.y.peek().litValue shouldBe FixedPoint.MIN
    }
  }
}
