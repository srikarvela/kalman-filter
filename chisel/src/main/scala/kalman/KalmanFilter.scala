package kalman

import chisel3._
import chisel3.util._

// Top-level 2-state (price, drift) Kalman filter: predict -> innovate -> reciprocal ->
// gain -> update, structurally pipelined (no FSM) the same way Reciprocal itself is
// pipelined internally.
//
// Key insight that keeps this simple: the persistent filter state (x0, x1, P00, P01,
// P11) and the per-request latched inputs (z, seqNum, dt, q0, q1, r) are held CONSTANT
// for the entire duration of processing one measurement (nothing else can write them
// until the pipeline finishes, since io.in.ready is gated on !busy). Every submodule in
// this design is a plain combinational-plus-register pipeline with no internal state
// beyond its own latency, so when fed constant inputs it settles into a steady-state
// output that remains correct indefinitely after its first `yValid` pulse. That means
// no ShiftRegister realignment is needed anywhere here -- each downstream stage simply
// reads the upstream stage's *data* wire directly, gated on the upstream stage's single
// `valid` pulse (whichever arrives last among its dependencies).
//
// Because the filter is loop-carried (predict_{k+1} depends on the fully-updated
// x_k/P_k), it cannot accept a new measurement mid-pipeline -- unlike the sibling repo's
// order book (stateless per-message, always-ready). This uses a Decoupled input with a
// `busy` flag instead: `ready` deasserts for one full predict->update pass (~27 cycles
// @ this config), which is a non-issue since HFT quote arrival intervals are orders of
// magnitude slower than a ~27-cycle/~108ns pass at 250 MHz.
class KalmanFilter(
  val p0_00: Double = 1.0,
  val p0_01: Double = 0.0,
  val p0_11: Double = 1.0,
  val x0Init: Double = 0.0,
  val x1Init: Double = 0.0,
  val nrIterations: Int = 3,
) extends Module {
  val W = FixedPoint.WIDTH
  val ONE = FixedPoint.toFixed(1.0)

  val io = IO(new Bundle {
    val cfg  = Input(new KalmanConfig)
    val in   = Flipped(Decoupled(new KalmanMeasurement))
    val out  = Output(new KalmanEstimate)
    val busy = Output(Bool())
  })

  // ---- Persistent filter state (stable for an entire request; only committed at doneValid) ----
  val x0  = RegInit(FixedPoint.toFixed(x0Init).S(W.W))  // price estimate
  val x1  = RegInit(FixedPoint.toFixed(x1Init).S(W.W))  // drift estimate
  val P00 = RegInit(FixedPoint.toFixed(p0_00).S(W.W))
  val P01 = RegInit(FixedPoint.toFixed(p0_01).S(W.W))
  val P11 = RegInit(FixedPoint.toFixed(p0_11).S(W.W))

  // ---- Input latch / backpressure ----
  val busyReg = RegInit(false.B)
  val accept  = io.in.valid && !busyReg
  io.in.ready := !busyReg
  io.busy     := busyReg

  val zReg, dtReg, q0Reg, q1Reg, rReg = Reg(SInt(W.W))
  val seqReg = Reg(UInt(32.W))
  val stage1Valid = RegNext(accept, false.B)

  when(accept) {
    zReg  := io.in.bits.z
    seqReg := io.in.bits.seqNum
    dtReg := io.cfg.dt
    q0Reg := io.cfg.q0
    q1Reg := io.cfg.q1
    rReg  := io.cfg.r
    busyReg := true.B
  }

  // ---- Predict x: x_pred0 = x0 + dt*x1 (F*x hand-optimized: sparse, not a real matmul); x_pred1 = x1 ----
  val mulDtX1 = Module(new FixedPointMul)
  mulDtX1.io.a := dtReg
  mulDtX1.io.b := x1
  mulDtX1.io.valid := stage1Valid
  val xPred0 = satAdd(x0, mulDtX1.io.y) // correct/stable from mulDtX1.io.yValid onward

  // ---- Predict P: P_pred = F*P*F^T + Q (genuine pipelined 2x2 matrix multiply, chained) ----
  def const(d: Double): SInt = FixedPoint.toFixed(d).S(W.W)

  val fMat = Wire(new Matrix2x2)
  fMat.m00 := const(1.0); fMat.m01 := dtReg
  fMat.m10 := const(0.0); fMat.m11 := const(1.0)

  val fTMat = Wire(new Matrix2x2)
  fTMat.m00 := const(1.0); fTMat.m01 := const(0.0)
  fTMat.m10 := dtReg;      fTMat.m11 := const(1.0)

  val pMat = Wire(new Matrix2x2)
  pMat.m00 := P00; pMat.m01 := P01
  pMat.m10 := P01; pMat.m11 := P11

  val mm1 = Module(new Matrix2x2FixedMul) // F * P
  mm1.io.a := fMat
  mm1.io.b := pMat
  mm1.io.valid := stage1Valid

  val mm2 = Module(new Matrix2x2FixedMul) // (F * P) * F^T
  mm2.io.a := mm1.io.y
  mm2.io.b := fTMat
  mm2.io.valid := mm1.io.yValid

  val pPred00 = satAdd(mm2.io.y.m00, q0Reg)
  val pPred01 = mm2.io.y.m01
  val pPred11 = satAdd(mm2.io.y.m11, q1Reg)

  // ---- Innovation: y = z - H*x_pred (H=[1,0] selects x_pred0); S = H*P_pred*H^T + R (selects P_pred00) ----
  val yInnov = satAdd(zReg, -xPred0)
  val sVar   = satAdd(pPred00, rReg)

  // ---- Reciprocal: invS = 1/S, triggered once predict-P (the long pole) settles ----
  val reciprocal = Module(new Reciprocal(nrIterations))
  reciprocal.io.x     := sVar
  reciprocal.io.valid := mm2.io.yValid

  // ---- Gain: K = P_pred_col0 * invS (H trivial selector => no real inverse needed) ----
  val k0Mul = Module(new FixedPointMul)
  k0Mul.io.a := pPred00
  k0Mul.io.b := reciprocal.io.y
  k0Mul.io.valid := reciprocal.io.yValid

  val k1Mul = Module(new FixedPointMul)
  k1Mul.io.a := pPred01
  k1Mul.io.b := reciprocal.io.y
  k1Mul.io.valid := reciprocal.io.yValid

  val gainValid = k0Mul.io.yValid // k1Mul pulses on the same cycle (identical trigger + latency)
  val k0 = k0Mul.io.y
  val k1 = k1Mul.io.y

  // ---- Update x: x_new = x_pred + K*y ----
  val k0YMul = Module(new FixedPointMul)
  k0YMul.io.a := k0
  k0YMul.io.b := yInnov
  k0YMul.io.valid := gainValid

  val k1YMul = Module(new FixedPointMul)
  k1YMul.io.a := k1
  k1YMul.io.b := yInnov
  k1YMul.io.valid := gainValid

  val xNew0 = satAdd(xPred0, k0YMul.io.y)
  val xNew1 = satAdd(x1, k1YMul.io.y)

  // ---- Update P: P_new = (I - K*H)*P_pred, H=[1,0] => rank-1 outer-product subtract ----
  // P_new00 = P00 - K0*P00, P_new01 = P01 - K0*P01, P_new11 = P11 - K1*P01
  // (P_new10 = P01 - K1*P00 is algebraically identical to P_new01 here since
  // K0*P01 == K1*P00 by construction: K0=P00*invS, K1=P01*invS, so symmetry is exact.)
  val k0P00Mul = Module(new FixedPointMul)
  k0P00Mul.io.a := k0; k0P00Mul.io.b := pPred00; k0P00Mul.io.valid := gainValid

  val k0P01Mul = Module(new FixedPointMul)
  k0P01Mul.io.a := k0; k0P01Mul.io.b := pPred01; k0P01Mul.io.valid := gainValid

  val k1P01Mul = Module(new FixedPointMul)
  k1P01Mul.io.a := k1; k1P01Mul.io.b := pPred01; k1P01Mul.io.valid := gainValid

  val pNew00 = satAdd(pPred00, -k0P00Mul.io.y)
  val pNew01 = satAdd(pPred01, -k0P01Mul.io.y)
  val pNew11 = satAdd(pPred11, -k1P01Mul.io.y)

  // ---- Commit + output (all of update-x/update-P's multipliers share the same trigger and latency) ----
  val doneValid = k0YMul.io.yValid

  when(doneValid) {
    x0  := xNew0
    x1  := xNew1
    P00 := pNew00
    P01 := pNew01
    P11 := pNew11
    busyReg := false.B
  }

  io.out.price  := xNew0
  io.out.drift  := xNew1
  io.out.seqNum := seqReg
  io.out.valid  := doneValid
}

object KalmanFilterVerilog extends App {
  import chisel3.stage.ChiselStage
  (new ChiselStage).emitVerilog(
    new KalmanFilter(),
    Array("--target-dir", "generated"),
  )
}
