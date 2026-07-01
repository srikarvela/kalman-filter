"""
Generate README visual assets, matching the fpga-crypto-feed-handler sibling repo's
dark-theme matplotlib style. Produces:
  1. fixed_point_format.png — Q16.16 bit-layout diagram
  2. signal_overlay.png     — real verified vectors: noisy input, floating golden KF,
                              hardware-simulated output, error subplot
  3. utilization.png        — FPGA resource utilization, parsed from a real Vivado
                              utilization.rpt if Tier 2 synthesis has been run,
                              otherwise an explicit "pending synthesis" placeholder
                              (no fabricated numbers).

Unlike the sibling repo's gen_visuals.py (synthetic data throughout), (2) and (3) here
read real, verified artifacts (make sim / make synth outputs) for higher credibility.
"""
import csv
import os
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.ticker import FuncFormatter

HERE = os.path.dirname(os.path.abspath(__file__))
VECTORS = os.path.join(HERE, "..", "vectors")
VIVADO = os.path.join(HERE, "..", "vivado")
OUT = os.path.join(HERE, "previews")
os.makedirs(OUT, exist_ok=True)

DARK   = "#0d1117"
PANEL  = "#161b22"
BORDER = "#30363d"
GREEN  = "#3fb950"
RED    = "#f85149"
BLUE   = "#58a6ff"
PURPLE = "#d2a8ff"
YELLOW = "#e3b341"
CYAN   = "#79c0ff"
WHITE  = "#e6edf3"
GREY   = "#8b949e"

plt.rcParams.update({
    "figure.facecolor":  DARK,
    "axes.facecolor":    PANEL,
    "axes.edgecolor":    BORDER,
    "axes.labelcolor":   WHITE,
    "xtick.color":       GREY,
    "ytick.color":       GREY,
    "text.color":        WHITE,
    "grid.color":        BORDER,
    "grid.linewidth":    0.5,
    "font.family":       "monospace",
    "font.size":         10,
})

FRAC_BITS = 16
WIDTH = 32


def to_fixed(d):
    return int(round(d * (1 << FRAC_BITS)))


# ─────────────────────────────────────────────────────────────────────────────
# 1. Q16.16 FIXED-POINT FORMAT DIAGRAM
# ─────────────────────────────────────────────────────────────────────────────

def make_fixed_point_format():
    fig, ax = plt.subplots(figsize=(13, 4.5), facecolor=DARK)
    ax.set_facecolor(DARK)
    ax.set_xlim(0, 32)
    ax.set_ylim(-3.2, 2.6)
    ax.axis("off")
    fig.suptitle("Q16.16 Fixed-Point Format  (32-bit signed)", color=WHITE,
                 fontsize=13, fontweight="bold", y=0.98)

    # bit cells, MSB (bit 31) on the left down to LSB (bit 0) on the right
    cell_w = 1.0
    for bit in range(32):
        x = 31 - bit
        if bit == 31:
            color, label = RED, "S"
        elif bit >= FRAC_BITS:
            color, label = BLUE, ""
        else:
            color, label = GREEN, ""
        rect = mpatches.Rectangle((x, 0), cell_w, 1, facecolor=color, alpha=0.35,
                                   edgecolor=BORDER, linewidth=0.8)
        ax.add_patch(rect)
        if label:
            ax.text(x + 0.5, 0.5, label, ha="center", va="center",
                     fontsize=9, color=WHITE, fontweight="bold")
        if bit in (31, 30, 16, 15, 1, 0):
            ax.text(x + 0.5, -0.35, str(bit), ha="center", va="top",
                     fontsize=7, color=GREY)

    # bracket labels
    ax.annotate("", xy=(1, 1.5), xytext=(31, 1.5),
                arrowprops=dict(arrowstyle="-", color=RED, lw=1.5))
    ax.text(16, 1.65, "sign + 15 integer bits", ha="center", color=RED, fontsize=9)

    ax.annotate("", xy=(16, -1.15), xytext=(32, -1.15),
                arrowprops=dict(arrowstyle="-", color=BLUE, lw=1.5))
    ax.text(24, -1.35, "integer [31:16]", ha="center", color=BLUE, fontsize=9)

    ax.annotate("", xy=(0, -1.15), xytext=(16, -1.15),
                arrowprops=dict(arrowstyle="-", color=GREEN, lw=1.5))
    ax.text(8, -1.35, "fraction [15:0]", ha="center", color=GREEN, fontsize=9)

    resolution = 2 ** -FRAC_BITS
    max_val = (2 ** (WIDTH - 1) - 1) / (1 << FRAC_BITS)
    min_val = -(2 ** (WIDTH - 1)) / (1 << FRAC_BITS)
    ax.text(16, -2.1,
            f"resolution = 2^-16 ~= {resolution:.3e}      "
            f"range = [{min_val:,.1f}, {max_val:,.1f}]",
            ha="center", color=GREY, fontsize=9.5)

    example = 100.25
    raw = to_fixed(example)
    ax.text(16, -2.7,
            f"example: {example} -> raw = {raw} = 0x{raw & 0xFFFFFFFF:08X}",
            ha="center", color=YELLOW, fontsize=9.5)

    fig.savefig(f"{OUT}/fixed_point_format.png", dpi=150, bbox_inches="tight", facecolor=DARK)
    plt.close(fig)
    print("fixed_point_format.png done")


# ─────────────────────────────────────────────────────────────────────────────
# 2. SIGNAL OVERLAY: real vectors from make sim
# ─────────────────────────────────────────────────────────────────────────────

def read_csv(path, cols):
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        rows = [{c: float(row[c]) for c in cols} for row in reader]
    return rows


def make_signal_overlay():
    input_path = os.path.join(VECTORS, "kalman_input.csv")
    float_path = os.path.join(VECTORS, "kalman_golden_float.csv")
    fixed_path = os.path.join(VECTORS, "kalman_golden_fixed.csv")
    hw_path = os.path.join(VECTORS, "kalman_hw_output.csv")

    missing = [p for p in (input_path, float_path, fixed_path, hw_path) if not os.path.exists(p)]
    if missing:
        print(f"signal_overlay.png skipped -- missing {missing}; run `make sim` first")
        return

    z = read_csv(input_path, ["seq", "z"])
    golden_float = read_csv(float_path, ["seq", "price"])
    golden_fixed = read_csv(fixed_path, ["seq", "price"])
    hw = read_csv(hw_path, ["seq", "price_raw"])

    seq = [r["seq"] for r in z]
    z_vals = [r["z"] for r in z]
    gf_vals = [r["price"] for r in golden_float]
    hw_vals = [r["price_raw"] / (1 << FRAC_BITS) for r in hw]
    gfix_vals = [r["price"] for r in golden_fixed]

    err_vs_fixed = [h - g for h, g in zip(hw_vals, gfix_vals)]
    err_vs_float = [h - g for h, g in zip(hw_vals, gf_vals)]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(13, 7), facecolor=DARK,
                                    gridspec_kw={"height_ratios": [2.3, 1]}, sharex=True)
    fig.suptitle("Kalman Filter Output  —  Chisel RTL Simulation vs Python Golden Model",
                 color=WHITE, fontsize=13, fontweight="bold")

    ax1.set_facecolor(PANEL)
    ax1.scatter(seq, z_vals, s=6, color=GREY, alpha=0.5, label="Noisy measurement (z)")
    ax1.plot(seq, gf_vals, color=YELLOW, lw=3.0, linestyle="--", alpha=0.9, dashes=(4, 3),
              label="Floating-point golden KF", zorder=2)
    ax1.plot(seq, hw_vals, color=GREEN, lw=1.3, alpha=1.0,
              label="Chisel RTL simulation output", zorder=3)
    ax1.set_ylabel("Price", color=GREY)
    ax1.legend(fontsize=9, facecolor=PANEL, edgecolor=BORDER, labelcolor=WHITE)
    ax1.grid(alpha=0.25)
    ax1.spines[["top", "right"]].set_visible(False)
    ax1.set_title("Noise reduction: raw measurement vs filtered estimate", color=GREY,
                  fontsize=9, loc="left")

    ax2.set_facecolor(PANEL)
    ax2.axhline(0, color=BORDER, lw=1)
    ax2.plot(seq, err_vs_fixed, color=CYAN, lw=1.2,
              label=f"hw - fixed golden (max |err| = {max(abs(e) for e in err_vs_fixed):.2e})")
    ax2.plot(seq, err_vs_float, color=PURPLE, lw=1.0, alpha=0.8,
              label="hw - floating golden (quantization + filtering delta)")
    ax2.set_xlabel("Message sequence", color=GREY)
    ax2.set_ylabel("Price error", color=GREY)
    ax2.legend(fontsize=8.5, facecolor=PANEL, edgecolor=BORDER, labelcolor=WHITE)
    ax2.grid(alpha=0.25)
    ax2.spines[["top", "right"]].set_visible(False)
    ax2.set_title("Error vs both golden models (bit-exact vs the fixed-point emulation)",
                  color=GREY, fontsize=9, loc="left")

    fig.savefig(f"{OUT}/signal_overlay.png", dpi=150, bbox_inches="tight", facecolor=DARK)
    plt.close(fig)
    print("signal_overlay.png done")


# ─────────────────────────────────────────────────────────────────────────────
# 3. FPGA UTILIZATION: parsed from a real Vivado report if available
# ─────────────────────────────────────────────────────────────────────────────

def parse_utilization_rpt(path):
    """Extract Slice LUTs / Registers / DSPs / BRAM (used, available) from a Vivado
    report_utilization text report. Returns None if the file doesn't look parseable.
    """
    with open(path) as f:
        text = f.read()

    patterns = {
        "LUTs\n(logic)":  r"Slice LUTs\s*\|\s*(\d+)\s*\|.*?\|\s*([\d.]+)\s*\|\s*(\d+)",
        "Registers":      r"Slice Registers\s*\|\s*(\d+)\s*\|.*?\|\s*([\d.]+)\s*\|\s*(\d+)",
        "DSPs":           r"DSPs\s*\|\s*(\d+)\s*\|.*?\|\s*([\d.]+)\s*\|\s*(\d+)",
        "Block RAM\n(tiles)": r"Block RAM Tile\s*\|\s*([\d.]+)\s*\|.*?\|\s*([\d.]+)\s*\|\s*(\d+)",
    }
    resources = {}
    for label, pat in patterns.items():
        m = re.search(pat, text)
        if not m:
            return None
        used = float(m.group(1))
        avail = int(m.group(3))
        resources[label] = (used, avail)
    return resources


def make_utilization():
    rpt_path = os.path.join(VIVADO, "utilization.rpt")
    resources = parse_utilization_rpt(rpt_path) if os.path.exists(rpt_path) else None

    fig, ax = plt.subplots(figsize=(11, 5), facecolor=DARK)
    ax.set_facecolor(PANEL)

    if resources is None:
        ax.axis("off")
        fig.suptitle("FPGA Resource Utilization  —  XC7Z020  (Zynq-7020, PYNQ-Z2)",
                     color=WHITE, fontsize=12, fontweight="bold")
        ax.text(0.5, 0.55, "Tier 2 synthesis has not been run in this environment",
                ha="center", va="center", fontsize=12, color=YELLOW, transform=ax.transAxes)
        ax.text(0.5, 0.42,
                "Run `make synth` with Vivado installed to populate this chart from a\n"
                "real vivado/utilization.rpt -- no placeholder/estimated numbers are shown here.",
                ha="center", va="center", fontsize=9.5, color=GREY, transform=ax.transAxes)
        fig.savefig(f"{OUT}/utilization.png", dpi=150, bbox_inches="tight", facecolor=DARK)
        plt.close(fig)
        print("utilization.png done (pending-synthesis placeholder)")
        return

    labels = list(resources.keys())
    used = [v[0] for v in resources.values()]
    avail = [v[1] for v in resources.values()]
    pct = [u / a * 100 if a else 0 for u, a in zip(used, avail)]

    fig.suptitle("FPGA Resource Utilization  —  XC7Z020  (Zynq-7020, PYNQ-Z2, real synth)",
                 color=WHITE, fontsize=12, fontweight="bold")
    y = range(len(labels))
    colors = [GREEN if p < 40 else YELLOW if p < 70 else RED for p in pct]
    ax.barh(y, pct, color=colors, alpha=0.85, height=0.55)
    ax.axvline(25, color=BORDER, lw=1, linestyle="--")
    ax.axvline(50, color=BORDER, lw=1, linestyle="--")
    ax.axvline(75, color=RED, lw=1, linestyle="--", alpha=0.5)
    for i, (p, u, a) in enumerate(zip(pct, used, avail)):
        ax.text(p + 0.5, i, f"{u:,.0f} / {a:,}", va="center", fontsize=8, color=GREY)
    ax.set_yticks(list(y))
    ax.set_yticklabels(labels, fontsize=9)
    ax.set_xlabel("Utilization (%)", color=GREY)
    ax.set_xlim(0, 100)
    ax.grid(axis="x", alpha=0.2)
    ax.spines[["top", "right"]].set_visible(False)

    fig.savefig(f"{OUT}/utilization.png", dpi=150, bbox_inches="tight", facecolor=DARK)
    plt.close(fig)
    print("utilization.png done (from real vivado/utilization.rpt)")


if __name__ == "__main__":
    make_fixed_point_format()
    make_signal_overlay()
    make_utilization()
    print("\nAll assets written to previews/")
