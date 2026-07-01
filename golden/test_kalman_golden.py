"""Validates the Python fixed-point emulation itself before trusting it as the
hardware checker: correctness of fixed_mul rounding, reciprocal_fixed convergence, and
that KalmanFilterFixedRef tracks the floating-point KalmanFilterRef within the expected
fixed-point quantization error. Mirrors the sibling repo's test_golden.py pattern of
validating the golden model before using it to diff against RTL simulation output.
"""
import math

import pytest

from kalman_ref import (
    DT, MAX, MIN, P0_00, P0_01, P0_11, Q0, Q1, R, X0_INIT, X1_INIT,
    KalmanFilterFixedRef, KalmanFilterRef, fixed_mul, from_fixed, generate_series,
    reciprocal_fixed, sat_add, to_fixed,
)


class TestFixedMul:
    def test_basic_multiply(self):
        got = from_fixed(fixed_mul(to_fixed(1.5), to_fixed(2.5)))
        assert got == pytest.approx(3.75, abs=1e-4)

    def test_negative_operands(self):
        got = from_fixed(fixed_mul(to_fixed(-2.0), to_fixed(3.0)))
        assert got == pytest.approx(-6.0, abs=1e-4)

    def test_round_half_up_tie_positive(self):
        # raw product 1 * 98304 (1.5 in Q16.16) -> low 16 bits exactly 0x8000
        assert fixed_mul(1, 98304) == 2

    def test_round_half_up_tie_negative(self):
        assert fixed_mul(-1, 98304) == -1

    def test_saturates_on_overflow(self):
        assert fixed_mul(to_fixed(1000.0), to_fixed(1000.0)) == MAX
        assert fixed_mul(to_fixed(1000.0), to_fixed(-1000.0)) == MIN


class TestSatAdd:
    def test_normal_add(self):
        assert sat_add(to_fixed(1.5), to_fixed(2.25)) == to_fixed(3.75)

    def test_saturates_high(self):
        assert sat_add(MAX, to_fixed(10.0)) == MAX

    def test_saturates_low(self):
        assert sat_add(MIN, to_fixed(-10.0)) == MIN


class TestReciprocalFixed:
    @pytest.mark.parametrize("x", [0.01, 0.0157, 0.1, 0.25, 0.5, 0.99, 1.0, 1.01,
                                    2.0, 3.0, 4.0, 7.5, 10.0, 100.0, 1000.0])
    def test_matches_floating_reciprocal(self, x):
        lsb = 2 ** -16
        got = from_fixed(reciprocal_fixed(to_fixed(x)))
        expected = 1.0 / x
        tolerance = max(0.005 * expected, 2 * lsb)
        assert abs(got - expected) < tolerance

    def test_rejects_nonpositive_input(self):
        with pytest.raises(AssertionError):
            reciprocal_fixed(0)
        with pytest.raises(AssertionError):
            reciprocal_fixed(-1)


class TestKalmanFilterFixedRef:
    def test_tracks_floating_reference_on_step_input(self):
        float_ref = KalmanFilterRef()
        fixed_ref = KalmanFilterFixedRef()
        for _ in range(30):
            fp, fd = float_ref.step(100.0, DT, Q0, Q1, R)
            xp_raw, xd_raw = fixed_ref.step(100.0, DT, Q0, Q1, R)
            xp, xd = from_fixed(xp_raw), from_fixed(xd_raw)
            assert xp == pytest.approx(fp, abs=0.05)
            assert xd == pytest.approx(fd, abs=0.05)

    def test_tracks_floating_reference_on_ramp_input(self):
        float_ref = KalmanFilterRef()
        fixed_ref = KalmanFilterFixedRef()
        for i in range(30):
            z = 10.0 + i * 2.0
            fp, fd = float_ref.step(z, DT, Q0, Q1, R)
            xp_raw, xd_raw = fixed_ref.step(z, DT, Q0, Q1, R)
            xp, xd = from_fixed(xp_raw), from_fixed(xd_raw)
            assert xp == pytest.approx(fp, abs=0.05)
            assert xd == pytest.approx(fd, abs=0.05)

    def test_tracks_floating_reference_on_synthetic_series(self):
        measurements = generate_series(n=100)
        float_ref = KalmanFilterRef()
        fixed_ref = KalmanFilterFixedRef()
        max_price_err = 0.0
        for z in measurements:
            fp, _ = float_ref.step(z, DT, Q0, Q1, R)
            xp_raw, _ = fixed_ref.step(z, DT, Q0, Q1, R)
            max_price_err = max(max_price_err, abs(from_fixed(xp_raw) - fp))
        assert max_price_err < 0.1
