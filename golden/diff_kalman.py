"""Bit-exact diff of the Chisel simulation's output (vectors/kalman_hw_output.csv,
written by KalmanFilterReplayTest.scala) against the Python fixed-point golden model
(vectors/kalman_golden_fixed.csv, written by kalman_ref.py). Since both implement the
identical Q16.16 algorithm (fixed_mul rounding, matrix2x2_fixed_mul, reciprocal_fixed),
a correct RTL implementation should match row-for-row exactly, not just within a
floating-point tolerance.
"""
import csv
import os
import sys


def load_csv(path, keys):
    rows = {}
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            seq = int(row["seq"])
            rows[seq] = {k: int(row[k]) for k in keys}
    return rows


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    vectors_dir = os.path.join(here, "..", "vectors")
    hw_path = os.path.join(vectors_dir, "kalman_hw_output.csv")
    golden_path = os.path.join(vectors_dir, "kalman_golden_fixed.csv")

    for path in (hw_path, golden_path):
        if not os.path.exists(path):
            print(f"FAIL: {path} not found")
            print("Run `make kf-golden` then the Chisel replay test (`make kf-test-replay`) first.")
            sys.exit(1)

    hw = load_csv(hw_path, ["price_raw", "drift_raw"])
    golden = load_csv(golden_path, ["price_raw", "drift_raw"])

    if set(hw.keys()) != set(golden.keys()):
        print(f"FAIL: row count/seq mismatch (hw={len(hw)} rows, golden={len(golden)} rows)")
        sys.exit(1)

    mismatches = []
    for seq in sorted(hw.keys()):
        h, g = hw[seq], golden[seq]
        if h["price_raw"] != g["price_raw"] or h["drift_raw"] != g["drift_raw"]:
            mismatches.append((seq, h, g))

    if mismatches:
        print(f"FAIL: {len(mismatches)}/{len(hw)} rows mismatched")
        for seq, h, g in mismatches[:10]:
            print(f"  seq={seq} hw={h} golden={g}")
        if len(mismatches) > 10:
            print(f"  ... and {len(mismatches) - 10} more")
        sys.exit(1)

    print(f"PASS: {len(hw)}/{len(hw)} rows bit-exact match")


if __name__ == "__main__":
    main()
