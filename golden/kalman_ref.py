"""Golden model for the Chisel Kalman filter: a floating-point reference (the "true"
math) plus a bit-exact Q16.16 fixed-point emulation of the exact RTL algorithm
(FixedPointMul's round-half-up + saturate, Matrix2x2FixedMul's dot-product-then-satAdd,
and Reciprocal's normalize/seed/Newton-Raphson/denormalize sequence), so hardware
simulation output can be diffed bit-exactly against this model rather than only within
a floating-point tolerance.
"""
import csv
import os

import numpy as np

FRAC_BITS = 16
WIDTH = 32
MAX = (1 << (WIDTH - 1)) - 1
MIN = -(1 << (WIDTH - 1))

# Simulation-wide config constants. The Chisel replay test (KalmanFilterReplayTest.scala)
# and diff_kalman.py must use these same values / the same P0 defaults as KalmanFilter's
# constructor defaults (p0_00=1.0, p0_01=0.0, p0_11=1.0, x0=0.0, x1=0.0).
DT = 1.0
Q0 = 0.05
Q1 = 0.01
R = 2.0
P0_00, P0_01, P0_11 = 1.0, 0.0, 1.0
X0_INIT, X1_INIT = 0.0, 0.0
NR_ITERATIONS = 3


def to_fixed(d: float) -> int:
    return int(round(d * (1 << FRAC_BITS)))


def from_fixed(v: int) -> float:
    return v / (1 << FRAC_BITS)


def to_signed32(v: int) -> int:
    v &= 0xFFFFFFFF
    return v - 0x100000000 if v >= 0x80000000 else v


def sat_add(a: int, b: int) -> int:
    s = a + b
    if s > MAX:
        return MAX
    if s < MIN:
        return MIN
    return s


def fixed_mul(a: int, b: int) -> int:
    """Mirrors FixedPointMul: full-precision product, round-half-up, saturate."""
    prod = a * b
    rounded = prod + (1 << (FRAC_BITS - 1))
    shifted = rounded >> FRAC_BITS  # Python's >> is arithmetic (floor) for negative ints too
    if shifted > MAX:
        return MAX
    if shifted < MIN:
        return MIN
    return shifted


def matrix2x2_fixed_mul(a, b):
    """Mirrors Matrix2x2FixedMul: 4 dot products, each via fixed_mul + sat_add."""
    a00, a01, a10, a11 = a
    b00, b01, b10, b11 = b
    y00 = sat_add(fixed_mul(a00, b00), fixed_mul(a01, b10))
    y01 = sat_add(fixed_mul(a00, b01), fixed_mul(a01, b11))
    y10 = sat_add(fixed_mul(a10, b00), fixed_mul(a11, b10))
    y11 = sat_add(fixed_mul(a10, b01), fixed_mul(a11, b11))
    return (y00, y01, y10, y11)


def _shifted_to_15(v_unsigned: int, amt: int) -> int:
    mask = (1 << WIDTH) - 1
    if amt < 0:
        return (v_unsigned << (-amt)) & mask
    return (v_unsigned >> amt) & mask


def reciprocal_fixed(x_raw: int, iterations: int = NR_ITERATIONS) -> int:
    """Mirrors Reciprocal.scala exactly: normalize into [0.5,1.0), minimax linear seed
    (48/17 - 32/17*m), `iterations` rounds of Newton-Raphson, denormalize. x_raw must be
    a strictly-positive Q16.16 raw value.
    """
    assert x_raw > 0, "reciprocal_fixed requires a strictly positive input"
    raw_mag = x_raw  # x > 0 so its bit pattern equals its unsigned magnitude
    highest_bit = raw_mag.bit_length() - 1  # index of the top set bit, 0..31
    shift_amt = highest_bit - (FRAC_BITS - 1)  # target: top bit at bit 15

    normalized = to_signed32(_shifted_to_15(raw_mag, shift_amt))

    c1 = to_fixed(48.0 / 17.0)
    c2 = to_fixed(32.0 / 17.0)
    y = sat_add(c1, -fixed_mul(c2, normalized))

    two = to_fixed(2.0)
    for _ in range(iterations):
        my = fixed_mul(normalized, y)
        two_minus_my = sat_add(two, -my)
        y = fixed_mul(y, two_minus_my)

    y_unsigned = y & 0xFFFFFFFF
    denorm = to_signed32(_shifted_to_15(y_unsigned, shift_amt))
    if denorm > MAX:
        return MAX
    if denorm < MIN:
        return MIN
    return denorm


class KalmanFilterFixedRef:
    """Bit-exact Q16.16 emulation of KalmanFilter.scala's predict/innovate/gain/update
    pipeline. Deliberately does NOT special-case F's sparsity -- it runs the full
    Matrix2x2FixedMul-equivalent dot products for F*P and (F*P)*F^T, exactly like the
    RTL does, so rounding behaves identically.
    """

    def __init__(self, p0_00=P0_00, p0_01=P0_01, p0_11=P0_11, x0=X0_INIT, x1=X1_INIT,
                 iterations=NR_ITERATIONS):
        self.x0 = to_fixed(x0)
        self.x1 = to_fixed(x1)
        self.P00 = to_fixed(p0_00)
        self.P01 = to_fixed(p0_01)
        self.P11 = to_fixed(p0_11)
        self.iterations = iterations

    def step(self, z: float, dt: float, q0: float, q1: float, r: float):
        return self.step_raw(to_fixed(z), to_fixed(dt), to_fixed(q0), to_fixed(q1), to_fixed(r))

    def step_raw(self, z_raw: int, dt_raw: int, q0_raw: int, q1_raw: int, r_raw: int):
        one = to_fixed(1.0)
        zero = 0

        x_pred0 = sat_add(self.x0, fixed_mul(dt_raw, self.x1))
        x_pred1 = self.x1

        f = (one, dt_raw, zero, one)
        ft = (one, zero, dt_raw, one)
        p = (self.P00, self.P01, self.P01, self.P11)

        fp = matrix2x2_fixed_mul(f, p)
        fpft = matrix2x2_fixed_mul(fp, ft)

        p_pred00 = sat_add(fpft[0], q0_raw)
        p_pred01 = fpft[1]
        p_pred11 = sat_add(fpft[3], q1_raw)

        y_innov = sat_add(z_raw, -x_pred0)
        s_var = sat_add(p_pred00, r_raw)

        inv_s = reciprocal_fixed(s_var, self.iterations)

        k0 = fixed_mul(p_pred00, inv_s)
        k1 = fixed_mul(p_pred01, inv_s)

        x_new0 = sat_add(x_pred0, fixed_mul(k0, y_innov))
        x_new1 = sat_add(x_pred1, fixed_mul(k1, y_innov))

        p_new00 = sat_add(p_pred00, -fixed_mul(k0, p_pred00))
        p_new01 = sat_add(p_pred01, -fixed_mul(k0, p_pred01))
        p_new11 = sat_add(p_pred11, -fixed_mul(k1, p_pred01))

        self.x0, self.x1 = x_new0, x_new1
        self.P00, self.P01, self.P11 = p_new00, p_new01, p_new11

        return x_new0, x_new1


class KalmanFilterRef:
    """Floating-point reference: the "true" mathematical Kalman filter, same equations
    as KalmanFilterFixedRef but without fixed-point quantization. Used as the ground
    truth for the signal-overlay plot, not for bit-exact hardware comparison.
    """

    def __init__(self, p0_00=P0_00, p0_01=P0_01, p0_11=P0_11, x0=X0_INIT, x1=X1_INIT):
        self.x = np.array([x0, x1], dtype=float)
        self.P = np.array([[p0_00, p0_01], [p0_01, p0_11]], dtype=float)

    def step(self, z: float, dt: float, q0: float, q1: float, r: float):
        F = np.array([[1.0, dt], [0.0, 1.0]])
        H = np.array([[1.0, 0.0]])
        Q = np.diag([q0, q1])

        x_pred = F @ self.x
        P_pred = F @ self.P @ F.T + Q

        y = z - (H @ x_pred)[0]
        S = (H @ P_pred @ H.T)[0, 0] + r
        K = (P_pred @ H.T).flatten() / S

        self.x = x_pred + K * y
        self.P = (np.eye(2) - np.outer(K, H)) @ P_pred

        return self.x[0], self.x[1]


def generate_series(n=200, seed=42, true_price0=100.0, true_drift=0.05,
                     process_std=0.1, meas_std=1.5):
    """Synthetic noisy price random walk with underlying drift, for a visually
    interesting demonstration of the filter's smoothing behavior.
    """
    rng = np.random.default_rng(seed)
    true_price = true_price0
    measurements = []
    for _ in range(n):
        true_price += true_drift + rng.normal(0, process_std)
        z = true_price + rng.normal(0, meas_std)
        measurements.append(z)
    return measurements


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    vectors_dir = os.path.join(here, "..", "vectors")
    os.makedirs(vectors_dir, exist_ok=True)

    measurements = generate_series()

    float_ref = KalmanFilterRef()
    fixed_ref = KalmanFilterFixedRef()

    with open(os.path.join(vectors_dir, "kalman_input.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["seq", "z"])
        for i, z in enumerate(measurements):
            w.writerow([i, z])

    with open(os.path.join(vectors_dir, "kalman_golden_float.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["seq", "price", "drift"])
        for i, z in enumerate(measurements):
            price, drift = float_ref.step(z, DT, Q0, Q1, R)
            w.writerow([i, price, drift])

    with open(os.path.join(vectors_dir, "kalman_golden_fixed.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["seq", "price_raw", "drift_raw", "price", "drift"])
        for i, z in enumerate(measurements):
            price_raw, drift_raw = fixed_ref.step(z, DT, Q0, Q1, R)
            w.writerow([i, price_raw, drift_raw, from_fixed(price_raw), from_fixed(drift_raw)])

    print(f"Wrote {len(measurements)} rows to {vectors_dir}")


if __name__ == "__main__":
    main()
