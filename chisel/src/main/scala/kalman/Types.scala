package kalman

import chisel3._

// Streaming measurement input: midprice (or any scalar signal) in Q16.16.
// seqNum passes through unchanged, styled after the sibling order-book repo's
// BookSnapshot.seqNum convention so a future integration can carry a sequence
// number end-to-end from the order book through this filter.
class KalmanMeasurement extends Bundle {
  val z      = SInt(FixedPoint.WIDTH.W) // measurement, Q16.16
  val seqNum = UInt(32.W)
  val valid  = Bool()
}

// Filter output: smoothed price estimate + drift (velocity) estimate.
class KalmanEstimate extends Bundle {
  val price  = SInt(FixedPoint.WIDTH.W) // x_hat[0], Q16.16
  val drift  = SInt(FixedPoint.WIDTH.W) // x_hat[1], Q16.16 (price change per sample interval)
  val seqNum = UInt(32.W)
  val valid  = Bool()
}

// Runtime-loadable tuning registers. `dt` is deliberately NOT a Scala literal baked
// into F at elaboration time -- it's a config input specifically so the F*P*F^T
// matrix multiply in KalmanFilter is a genuine (non-constant-folded) multiply in the
// synthesized netlist, not something Vivado can algebraically simplify away.
class KalmanConfig extends Bundle {
  val dt = SInt(FixedPoint.WIDTH.W) // Q16.16, state-transition time step
  val q0 = SInt(FixedPoint.WIDTH.W) // Q16.16, process noise variance (price)
  val q1 = SInt(FixedPoint.WIDTH.W) // Q16.16, process noise variance (drift)
  val r  = SInt(FixedPoint.WIDTH.W) // Q16.16, measurement noise variance
}
