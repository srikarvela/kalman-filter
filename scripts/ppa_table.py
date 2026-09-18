"""Build reports/ppa.csv from the committed Vivado reports.

Adapted from cordic-engine/scripts/ppa_table.py. Every number is parsed from
reports/N_<period>ns/utilization.rpt and timing_summary.rpt (written by
tcl/kalman_synth.tcl). Nothing is counted from the netlist by other means, so
the CSV and the .rpt files cannot disagree.

Column meanings (Vivado report_utilization / report_timing_summary names):
  LUT       "Slice LUTs"     -- every LUT used, logic plus LUT-as-memory (SRL)
  LUT_logic "LUT as Logic"   -- the subset used as logic
  LUT_mem   "LUT as Memory"  -- the subset used as SRL/distributed RAM
  FF        "Slice Registers"
  CARRY4, DSP ("DSPs"), BRAM ("Block RAM Tile")
  WNS/WHS   worst setup / hold slack, with the failing-endpoint counts
  Fmax      1000 / (period - WNS): the setup-limited clock, from post-route slack
"""
import csv
import glob
import os
import re

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")

UTIL_FIELDS = {
    "LUT": "Slice LUTs",
    "LUT_logic": "LUT as Logic",
    "LUT_mem": "LUT as Memory",
    "FF": "Slice Registers",
    "CARRY4": "CARRY4",
    "DSP": "DSPs",
    "BRAM": "Block RAM Tile",
}


def util_field(text, name):
    m = re.search(r"^\|\s*" + re.escape(name) + r"\s*\|\s*(\d+)\s*\|", text, re.M)
    if not m:
        raise ValueError(f"'{name}' not found in utilization report")
    return int(m.group(1))


def parse_run(rdir):
    tag = os.path.basename(rdir)
    m = re.fullmatch(r"N_([\d.]+)ns", tag)
    if not m:
        return None
    with open(os.path.join(rdir, "utilization.rpt")) as f:
        util = f.read()
    with open(os.path.join(rdir, "timing_summary.rpt")) as f:
        tim = f.read()
    row = {"run": tag}
    row.update({k: util_field(util, v) for k, v in UTIL_FIELDS.items()})

    clk = re.search(r"^clock\s+\{\d+\.\d+\s+\d+\.\d+\}\s+([\d.]+)\s", tim, re.M)
    setup = re.search(r"^Setup\s*:\s*(\d+)\s+Failing Endpoints,\s+Worst Slack\s+(-?[\d.]+)ns", tim, re.M)
    hold = re.search(r"^Hold\s*:\s*(\d+)\s+Failing Endpoints,\s+Worst Slack\s+(-?[\d.]+)ns", tim, re.M)
    if not (clk and setup and hold):
        raise ValueError(f"timing summary fields not found in {rdir}")
    row["period_ns"] = float(clk.group(1))
    row["WNS_ns"] = float(setup.group(2))
    row["setup_fail"] = int(setup.group(1))
    row["WHS_ns"] = float(hold.group(2))
    row["hold_fail"] = int(hold.group(1))
    row["constraints_met"] = "no" if "Timing constraints are not met" in tim else "yes"
    row["Fmax_MHz"] = round(1000.0 / (row["period_ns"] - row["WNS_ns"]), 1)
    return row


def main():
    runs = [parse_run(d) for d in sorted(glob.glob(os.path.join(ROOT, "reports", "N_*ns")))]
    runs = sorted((r for r in runs if r), key=lambda r: r["period_ns"])
    if not runs:
        raise SystemExit("no reports/N_*ns/ directories found (run `make vm-sweep` first)")

    cols = ["run", "LUT", "LUT_logic", "LUT_mem", "FF", "CARRY4", "DSP", "BRAM", "period_ns",
            "WNS_ns", "setup_fail", "WHS_ns", "hold_fail", "constraints_met", "Fmax_MHz"]
    csv_path = os.path.join(ROOT, "reports", "ppa.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(runs)

    lines = [
        "| run | Slice LUTs (logic + mem) | FF | CARRY4 | DSP | BRAM | period (ns) "
        "| WNS (ns) / failing | WHS (ns) / failing | constraints met | Fmax (MHz) |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---|---:|",
    ]
    for r in runs:
        lines.append(
            f"| {r['run']} | {r['LUT']} ({r['LUT_logic']} + {r['LUT_mem']}) | {r['FF']} | {r['CARRY4']} "
            f"| {r['DSP']} | {r['BRAM']} | {r['period_ns']:.3f} | {r['WNS_ns']:+.3f} / {r['setup_fail']} "
            f"| {r['WHS_ns']:+.3f} / {r['hold_fail']} | {r['constraints_met']} | {r['Fmax_MHz']:.1f} |"
        )
    print("\n".join(lines))
    print(f"wrote {csv_path}")


if __name__ == "__main__":
    main()
