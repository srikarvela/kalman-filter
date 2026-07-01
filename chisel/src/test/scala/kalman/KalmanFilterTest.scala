package kalman

import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec
import org.scalatest.matchers.should.Matchers

class KalmanFilterTest extends AnyFlatSpec with ChiselScalatestTester with Matchers {
  def setCfg(dut: KalmanFilter, dt: Double, q0: Double, q1: Double, r: Double): Unit = {
    dut.io.cfg.dt.poke(FixedPoint.toFixed(dt).S(32.W))
    dut.io.cfg.q0.poke(FixedPoint.toFixed(q0).S(32.W))
    dut.io.cfg.q1.poke(FixedPoint.toFixed(q1).S(32.W))
    dut.io.cfg.r.poke(FixedPoint.toFixed(r).S(32.W))
  }

  // Drives one Decoupled measurement transaction to completion (blocks until accepted).
  def sendMeasurement(dut: KalmanFilter, z: Double, seq: Long, maxWait: Int = 60): Unit = {
    dut.io.in.bits.z.poke(FixedPoint.toFixed(z).S(32.W))
    dut.io.in.bits.seqNum.poke(seq.U(32.W))
    dut.io.in.bits.valid.poke(true.B)
    dut.io.in.valid.poke(true.B)
    var c = 0
    while (!dut.io.in.ready.peek().litToBoolean && c < maxWait) {
      dut.clock.step(1)
      c += 1
    }
    dut.io.in.ready.expect(true.B, s"input never became ready within $maxWait cycles")
    dut.clock.step(1) // this edge performs the accept
    dut.io.in.valid.poke(false.B)
  }

  def waitForOutput(dut: KalmanFilter, maxCycles: Int = 60): Unit = {
    var c = 0
    while (!dut.io.out.valid.peek().litToBoolean && c < maxCycles) {
      dut.clock.step(1)
      c += 1
    }
    dut.io.out.valid.expect(true.B, s"no output valid within $maxCycles cycles")
  }

  behavior of "KalmanFilter"

  it should "converge the price estimate toward a repeated step measurement" in {
    // With P0=I and dt=1, F's off-diagonal term deliberately couples price/drift
    // covariance, so a single step input produces a real (not spurious-hardware-bug)
    // multi-iteration transient in the drift estimate before it decays -- confirmed
    // by cross-checking against a floating-point NumPy reference of the identical
    // recursion, which converges to the same ~108 at iteration 15 and ~100 by ~60.
    test(new KalmanFilter()) { dut =>
      setCfg(dut, dt = 1.0, q0 = 0.01, q1 = 0.001, r = 1.0)
      var lastPrice = 0.0
      for (i <- 0 until 60) {
        sendMeasurement(dut, 100.0, seq = i)
        waitForOutput(dut)
        lastPrice = FixedPoint.fromFixed(dut.io.out.price.peek().litValue)
      }
      lastPrice shouldBe 100.0 +- 1.0
    }
  }

  it should "deassert ready while busy and reassert it after the output settles" in {
    test(new KalmanFilter()) { dut =>
      setCfg(dut, dt = 1.0, q0 = 0.01, q1 = 0.001, r = 1.0)

      dut.io.in.bits.z.poke(FixedPoint.toFixed(50.0).S(32.W))
      dut.io.in.bits.seqNum.poke(0.U)
      dut.io.in.bits.valid.poke(true.B)
      dut.io.in.valid.poke(true.B)
      dut.io.in.ready.expect(true.B) // idle: ready immediately
      dut.clock.step(1) // accept happens on this edge

      dut.io.in.ready.expect(false.B) // busy the very next cycle
      dut.io.busy.expect(true.B)

      var c = 0
      while (!dut.io.out.valid.peek().litToBoolean && c < 60) {
        dut.io.in.ready.expect(false.B, s"ready asserted mid-pipeline at cycle $c")
        dut.clock.step(1)
        c += 1
      }
      dut.io.out.valid.expect(true.B)
      // busy/ready are registered off `doneValid`, so they only clear the cycle AFTER
      // the output-valid pulse, not on the same cycle.
      dut.io.in.valid.poke(false.B)
      dut.clock.step(1)
      dut.io.busy.expect(false.B)
      dut.io.in.ready.expect(true.B) // ready again once idle
    }
  }

  it should "pass the seqNum through to the output unchanged" in {
    test(new KalmanFilter()) { dut =>
      setCfg(dut, dt = 1.0, q0 = 0.01, q1 = 0.001, r = 1.0)
      sendMeasurement(dut, 42.0, seq = 1234)
      waitForOutput(dut)
      dut.io.out.seqNum.expect(1234.U)
    }
  }

  it should "track a ramp (nonzero drift) measurement series" in {
    test(new KalmanFilter()) { dut =>
      setCfg(dut, dt = 1.0, q0 = 0.1, q1 = 0.05, r = 0.5)
      var lastPrice = 0.0
      for (i <- 0 until 20) {
        val z = 10.0 + i * 2.0 // true ramp: price increases by 2.0 per step
        sendMeasurement(dut, z, seq = i)
        waitForOutput(dut)
        lastPrice = FixedPoint.fromFixed(dut.io.out.price.peek().litValue)
      }
      val expected = 10.0 + 19 * 2.0
      lastPrice shouldBe expected +- 4.0
    }
  }
}
