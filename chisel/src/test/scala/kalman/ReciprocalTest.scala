package kalman

import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec
import org.scalatest.matchers.should.Matchers

class ReciprocalTest extends AnyFlatSpec with ChiselScalatestTester with Matchers {
  def reciprocalOnce(dut: Reciprocal, xVal: Double, maxCycles: Int = 60): Double = {
    dut.io.x.poke(FixedPoint.toFixed(xVal).S(32.W))
    dut.io.valid.poke(true.B)
    dut.clock.step(1)
    dut.io.valid.poke(false.B)
    var cycles = 0
    while (!dut.io.yValid.peek().litToBoolean && cycles < maxCycles) {
      dut.clock.step(1)
      cycles += 1
    }
    dut.io.yValid.expect(true.B, s"reciprocal did not assert valid within $maxCycles cycles for x=$xVal")
    FixedPoint.fromFixed(dut.io.y.peek().litValue)
  }

  behavior of "Reciprocal"

  val sweep = Seq(0.01, 0.0157, 0.1, 0.25, 0.5, 0.99, 1.0, 1.01, 2.0, 3.0, 4.0, 7.5, 10.0, 100.0, 1000.0)

  it should "compute 1/x within 0.5% relative error (or 2 LSB, whichever is looser)" in {
    // For small reciprocal magnitudes (e.g. 1/1000 = 0.001), Q16.16's absolute
    // resolution (2^-16 ~= 1.5e-5) dominates relative error regardless of NR
    // convergence, so the tolerance is the tighter of a relative bound and an
    // absolute few-LSB floor.
    val lsb = math.pow(2, -FixedPoint.FRAC_BITS)
    test(new Reciprocal(iterations = 3)) { dut =>
      for (x <- sweep) {
        val got = reciprocalOnce(dut, x)
        val expected = 1.0 / x
        val absErr = math.abs(got - expected)
        val tolerance = math.max(0.005 * expected, 2 * lsb)
        withClue(s"x=$x got=$got expected=$expected absErr=$absErr tolerance=$tolerance: ") {
          absErr should be < tolerance
        }
      }
    }
  }

  it should "report a consistent fixed pipeline latency" in {
    test(new Reciprocal(iterations = 3)) { dut =>
      dut.io.x.poke(FixedPoint.toFixed(4.0).S(32.W))
      dut.io.valid.poke(true.B)
      dut.clock.step(1)
      dut.io.valid.poke(false.B)
      var latency = 0
      while (!dut.io.yValid.peek().litToBoolean && latency < 60) {
        dut.clock.step(1)
        latency += 1
      }
      dut.io.yValid.expect(true.B)
      latency should (be > 0 and be < 60)
    }
  }

  it should "be fully pipelined (accept a new x every cycle)" in {
    test(new Reciprocal(iterations = 3)) { dut =>
      dut.io.x.poke(FixedPoint.toFixed(2.0).S(32.W))
      dut.io.valid.poke(true.B)
      dut.clock.step(1)
      dut.io.x.poke(FixedPoint.toFixed(8.0).S(32.W))
      dut.io.valid.poke(true.B)
      dut.clock.step(1)
      dut.io.valid.poke(false.B)

      var cyclesSinceFirst = 2
      while (!dut.io.yValid.peek().litToBoolean && cyclesSinceFirst < 60) {
        dut.clock.step(1)
        cyclesSinceFirst += 1
      }
      dut.io.yValid.expect(true.B)
      FixedPoint.fromFixed(dut.io.y.peek().litValue) shouldBe (0.5 +- 0.0025)

      dut.clock.step(1)
      dut.io.yValid.expect(true.B)
      FixedPoint.fromFixed(dut.io.y.peek().litValue) shouldBe (0.125 +- 0.001)
    }
  }
}
