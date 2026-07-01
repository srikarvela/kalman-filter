"""
Generate README visual assets, matching the fpga-crypto-feed-handler sibling repo's
dark-theme matplotlib style. Produces:
  1. fixed_point_format.png     — Q16.16 bit-layout diagram
  2. architecture_block_diagram.png — detailed pipeline block diagram (stages, submodules,
                                  cycle latencies, state feedback loop)
  3. pipeline_latency.png       — cycle-by-cycle latency breakdown bar chart
  4. signal_overlay.png         — real verified vectors: noisy input, floating golden KF,
                                  hardware-simulated output, error subplot
  5. convergence_demo.png       — sample demo: step-input convergence (the overshoot-then-
                                  settle transient discussed in the README's verification
                                  section), computed from the same bit-exact golden model
                                  already proven equivalent to the RTL
  6. utilization.png            — FPGA resource utilization, parsed from a real Vivado
                                  utilization.rpt if Tier 2 synthesis has been run,
                                  otherwise an explicit "pending synthesis" placeholder
                                  (no fabricated numbers).

Unlike the sibling repo's gen_visuals.py (synthetic data throughout), (4)-(6) here read
real, verified artifacts (make sim / make synth outputs, or the same golden model already
bit-exact-verified against the RTL) for higher credibility.
"""
import csv
import os
import re
import sys

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

sys.path.insert(0, os.path.join(HERE, "..", "golden"))

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
# 2. ARCHITECTURE BLOCK DIAGRAM
# ─────────────────────────────────────────────────────────────────────────────

def _box(ax, x, y, w, h, title, body, color, fontsize_title=9.5, fontsize_body=7.8):
    rect = mpatches.FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.4",
                                    linewidth=1.4, edgecolor=color,
                                    facecolor=color + "22")
    ax.add_patch(rect)
    ax.text(x + w / 2, y + h - 1.6, title, ha="center", va="top",
            fontsize=fontsize_title, color=color, fontweight="bold")
    ax.text(x + w / 2, y + h - 3.2, body, ha="center", va="top",
            fontsize=fontsize_body, color=WHITE, linespacing=1.6)
    return (x, y, w, h)


def _arrow(ax, src, dst, color=GREY, style="-|>", lw=1.3, connectionstyle="arc3,rad=0.0"):
    x0, y0, w0, h0 = src
    x1, y1, w1, h1 = dst
    p0 = (x0 + w0, y0 + h0 / 2)
    p1 = (x1, y1 + h1 / 2)
    ax.annotate("", xy=p1, xytext=p0,
                arrowprops=dict(arrowstyle=style, color=color, lw=lw,
                                 connectionstyle=connectionstyle))


def make_architecture_block_diagram():
    fig, ax = plt.subplots(figsize=(16, 10.5), facecolor=DARK)
    ax.set_facecolor(DARK)
    ax.set_xlim(0, 118)
    ax.set_ylim(-6, 70)
    ax.axis("off")
    fig.suptitle("KalmanFilter Pipeline — Structural (FSM-free) Architecture", color=WHITE,
                 fontsize=15, fontweight="bold", y=0.975)
    ax.text(59, 67,
            "Every block is a plain combinational-plus-register pipeline; no FSM anywhere. Downstream stages read upstream data directly,\n"
            "gated on whichever dependency's valid pulse arrives last -- correct because inputs are held constant for the whole request,\n"
            "so each block settles into a stable steady-state output.",
            ha="center", va="top", fontsize=8.8, color=GREY, linespacing=1.7)

    # Inputs
    inp = _box(ax, 1, 24, 13, 14, "Inputs", "KalmanMeasurement\n(z, seqNum, valid)\n\n"
               "KalmanConfig\n(dt, q0, q1, r)", BLUE)

    latch = _box(ax, 18, 24, 13, 14, "Input latch", "busy-gated accept\nzReg,seqReg,\ndtReg,q0Reg,\nq1Reg,rReg\n\n1 cycle", CYAN)
    _arrow(ax, inp, latch)

    # Persistent state (feedback loop, drawn above)
    state = _box(ax, 18, 50, 13, 12, "Filter state", "x0, x1\nP00, P01, P11\n\n(persistent regs,\nstable all request)", YELLOW,
                 fontsize_body=7.3)

    # Predict x (upper branch)
    predx = _box(ax, 35, 34, 15, 12, "Predict x", "x_pred0 = x0+dt*x1\nx_pred1 = x1\n\n1x FixedPointMul\n2 cycles", GREEN,
                 fontsize_body=7.3)
    _arrow(ax, latch, predx)
    _arrow(ax, state, predx, color=YELLOW, style="-|>", connectionstyle="arc3,rad=-0.15")

    # Predict P (lower branch) — the real matrix multiply hardware
    predp = _box(ax, 35, 8, 15, 20, "Predict P", "F.P.F^T + Q\n\n2x Matrix2x2FixedMul\n(chained)\n\n6 cycles\n(long pole vs predict x)", GREEN,
                 fontsize_body=7.3)
    _arrow(ax, latch, predp)
    _arrow(ax, state, predp, color=YELLOW, style="-|>", connectionstyle="arc3,rad=0.25")

    # Innovate
    innov = _box(ax, 55, 24, 14, 14, "Innovate", "y = z - x_pred0\nS = P_pred00 + R\n\ncombinational\n(gated on predict-P,\nthe slower branch)", PURPLE,
                 fontsize_body=7.3)
    _arrow(ax, predx, innov, connectionstyle="arc3,rad=-0.2")
    _arrow(ax, predp, innov, connectionstyle="arc3,rad=0.2")

    # Reciprocal
    recip = _box(ax, 74, 24, 14, 14, "Reciprocal", "invS = 1/S\n\nNewton-Raphson\n(no HW divider)\n\n~17 cycles", RED,
                 fontsize_body=7.3)
    _arrow(ax, innov, recip)

    # Gain
    gain = _box(ax, 93, 24, 12, 14, "Gain", "K0 = P_pred00*invS\nK1 = P_pred01*invS\n\n2x FixedPointMul\n2 cycles", CYAN,
                fontsize_body=7.3)
    _arrow(ax, recip, gain)

    # Update x/P
    updx = _box(ax, 93, 42, 12, 12, "Update x", "x_new = x_pred\n     + K*y\n\n2x FixedPointMul\n2 cycles", GREEN,
                fontsize_body=7.0)
    updp = _box(ax, 93, 6, 12, 14, "Update P", "P_new = P_pred\n  - K.row0(P_pred)\n\n3x FixedPointMul\n2 cycles", GREEN,
                fontsize_body=7.0)
    _arrow(ax, gain, updx, connectionstyle="arc3,rad=-0.2")
    _arrow(ax, gain, updp, connectionstyle="arc3,rad=0.2")

    # Commit / output
    out = _box(ax, 108, 24, 9, 14, "Commit", "x0,x1,P00,P01,\nP11 <= new vals\nbusy <= false\n\nio.out.valid", YELLOW,
               fontsize_body=6.8)
    _arrow(ax, updx, out, connectionstyle="arc3,rad=-0.15")
    _arrow(ax, updp, out, connectionstyle="arc3,rad=0.15")

    # feedback loop back to state -- routed high above every box so it reads as one
    # clean arc rather than crossing through the diagram
    ax.annotate("", xy=(31, 62.5), xytext=(112.5, 52.5),
                arrowprops=dict(arrowstyle="-|>", color=YELLOW, lw=1.7, alpha=0.9,
                                 connectionstyle="arc3,rad=-0.35"))
    ax.text(72, 64.3, "state feedback: next predict uses the just-committed x/P (recursive, loop-carried)",
            ha="center", color=YELLOW, fontsize=8.3, style="italic")

    ax.text(115, 24 + 7, "KalmanEstimate\n(price, drift,\nseqNum, valid)",
            ha="left", va="center", fontsize=8, color=WHITE)

    fig.savefig(f"{OUT}/architecture_block_diagram.png", dpi=150, bbox_inches="tight", facecolor=DARK)
    plt.close(fig)
    print("architecture_block_diagram.png done")


# ─────────────────────────────────────────────────────────────────────────────
# 3. PIPELINE LATENCY BREAKDOWN
# ─────────────────────────────────────────────────────────────────────────────

def make_pipeline_latency():
    # Cumulative cycle counts derived from the design (see KalmanFilter.scala comments):
    # accept -> stage1Valid (1c) -> predict-P settles (mm1 3c + mm2 3c = 6c from stage1Valid,
    # predict-x's 2c branch is off the critical path) -> reciprocal (~17c) -> gain (2c) ->
    # update x/P (2c). Total ~27 cycles @ 250 MHz = 4 ns/cycle.
    stages = [
        ("Input\nlatch", 1, CYAN),
        ("Predict P\n(2x matmul, F.P.F^T+Q)", 5, GREEN),
        ("Reciprocal\n(Newton-Raphson 1/S)", 17, RED),
        ("Gain\n(K0,K1)", 2, CYAN),
        ("Update x/P\n(commit)", 2, GREEN),
    ]

    fig, ax = plt.subplots(figsize=(13, 4.6), facecolor=DARK)
    ax.set_facecolor(PANEL)
    fig.suptitle("Pipeline Stage Latency — 250 MHz clock (1 cycle = 4 ns)",
                 color=WHITE, fontsize=12, fontweight="bold")
    ax.text(0, 1.05, "Predict x (2c) runs in parallel with Predict P and is off the critical path.",
            transform=ax.transAxes, fontsize=8.5, color=GREY)

    x = 0
    for label, cycles, color in stages:
        ax.barh(0, cycles, left=x, height=0.55, color=color, alpha=0.8,
                edgecolor=DARK, linewidth=1.5)
        cx = x + cycles / 2
        ax.text(cx, 0, f"{cycles}c\n{cycles * 4} ns", ha="center", va="center", fontsize=9,
                color=DARK if color in (GREEN, CYAN) else WHITE, fontweight="bold")
        ax.text(cx, 0.35, label, ha="center", va="bottom", fontsize=7.8, color=color)
        x += cycles

    total = sum(c for _, c, _ in stages)
    ax.annotate("", xy=(total, -0.32), xytext=(0, -0.32),
                arrowprops=dict(arrowstyle="<->", color=WHITE, lw=1.5))
    ax.text(total / 2, -0.42,
            f"Total: {total} cycles = {total * 4} ns @ 250 MHz  "
            f"(reciprocal is {17 / total * 100:.0f}% of the critical path)",
            ha="center", va="top", fontsize=10, color=WHITE, fontweight="bold")

    ax.set_xlim(-1, total + 1)
    ax.set_ylim(-0.65, 0.75)
    ax.set_yticks([])
    ax.set_xlabel("Clock cycles", color=GREY)
    ax.grid(axis="x", alpha=0.2)
    ax.spines[["top", "right", "left"]].set_visible(False)

    fig.savefig(f"{OUT}/pipeline_latency.png", dpi=150, bbox_inches="tight", facecolor=DARK)
    plt.close(fig)
    print("pipeline_latency.png done")


# ─────────────────────────────────────────────────────────────────────────────
# 4. SIGNAL OVERLAY: real vectors from make sim
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
# 5. CONVERGENCE DEMO: step-input transient (sample "demo" picture)
# ─────────────────────────────────────────────────────────────────────────────

def make_convergence_demo():
    """Reproduces the exact scenario from KalmanFilterTest.scala's step-input test:
    a repeated constant measurement from a cold start (x=[0,0], P=I). Uses the
    bit-exact fixed-point golden model (already proven identical to the RTL via the
    200/200-row replay diff) as a stand-in for "what the hardware actually outputs."

    This is the scenario that initially looked like a hardware bug (108 instead of the
    naive expectation of ~100 after 15 iterations) until cross-checking against the
    floating-point reference showed it was real filter behavior: P0's off-diagonal
    coupling between price and drift produces a real overshoot-then-decay transient.
    """
    from kalman_ref import KalmanFilterFixedRef, KalmanFilterRef, from_fixed

    dt, q0, q1, r = 1.0, 0.01, 0.001, 1.0
    target = 100.0
    n = 60

    float_ref = KalmanFilterRef()
    fixed_ref = KalmanFilterFixedRef()

    f_price, f_drift, x_price, x_drift = [], [], [], []
    for _ in range(n):
        fp, fd = float_ref.step(target, dt, q0, q1, r)
        xp_raw, xd_raw = fixed_ref.step(target, dt, q0, q1, r)
        f_price.append(fp); f_drift.append(fd)
        x_price.append(from_fixed(xp_raw)); x_drift.append(from_fixed(xd_raw))

    it = list(range(n))
    peak_i = max(range(n), key=lambda i: x_price[i])

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(13, 7.5), facecolor=DARK, sharex=True)
    fig.suptitle("Demo: Cold-Start Step Response (repeated measurement, z=100)",
                 color=WHITE, fontsize=13, fontweight="bold")

    ax1.set_facecolor(PANEL)
    ax1.axhline(target, color=BORDER, lw=1.2, linestyle=":", label="True price (z=100)")
    ax1.plot(it, f_price, color=YELLOW, lw=3.0, linestyle="--", dashes=(4, 3), alpha=0.9,
              label="Floating-point golden KF", zorder=2)
    ax1.plot(it, x_price, color=GREEN, lw=1.4, alpha=1.0,
              label="Fixed-point (bit-exact RTL equivalent)", zorder=3)
    ax1.annotate(f"overshoot to {x_price[peak_i]:.1f}\n(P0's off-diagonal coupling\ncreates a real transient)",
                 xy=(peak_i, x_price[peak_i]), xytext=(peak_i + 14, x_price[peak_i] - 8),
                 color=RED, fontsize=8.5, ha="left", va="top",
                 arrowprops=dict(arrowstyle="->", color=RED, lw=1.2))
    ax1.set_ylabel("Price estimate", color=GREY)
    ax1.legend(fontsize=9, facecolor=PANEL, edgecolor=BORDER, labelcolor=WHITE, loc="lower right")
    ax1.grid(alpha=0.25)
    ax1.spines[["top", "right"]].set_visible(False)
    ax1.set_title("Price state converges to the true value after the initial transient decays",
                  color=GREY, fontsize=9, loc="left")

    ax2.set_facecolor(PANEL)
    ax2.axhline(0, color=BORDER, lw=1.2, linestyle=":", label="True drift (0, since z is constant)")
    ax2.plot(it, f_drift, color=YELLOW, lw=3.0, linestyle="--", dashes=(4, 3), alpha=0.9,
              label="Floating-point golden KF")
    ax2.plot(it, x_drift, color=CYAN, lw=1.4, alpha=1.0,
              label="Fixed-point (bit-exact RTL equivalent)")
    ax2.set_xlabel("Measurement index (iteration)", color=GREY)
    ax2.set_ylabel("Drift estimate", color=GREY)
    ax2.legend(fontsize=9, facecolor=PANEL, edgecolor=BORDER, labelcolor=WHITE)
    ax2.grid(alpha=0.25)
    ax2.spines[["top", "right"]].set_visible(False)
    ax2.set_title("Drift state, spuriously large at first, decays back toward zero",
                  color=GREY, fontsize=9, loc="left")

    fig.savefig(f"{OUT}/convergence_demo.png", dpi=150, bbox_inches="tight", facecolor=DARK)
    plt.close(fig)
    print("convergence_demo.png done")


# ─────────────────────────────────────────────────────────────────────────────
# 6. FPGA UTILIZATION: parsed from a real Vivado report if available
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
    make_architecture_block_diagram()
    make_pipeline_latency()
    make_signal_overlay()
    make_convergence_demo()
    make_utilization()
    print("\nAll assets written to previews/")
