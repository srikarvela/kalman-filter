# kalman-filter master Makefile
# Targets map to two build tiers (Tier 3 on-hardware PYNQ integration is roadmap, see README).

SBT      ?= sbt
VIVADO   ?= vivado
PYTHON   ?= .venv/bin/python

# ── Tier 1: simulation ────────────────────────────────────────────────────────

.PHONY: kf-test
kf-test:
	cd chisel && $(SBT) test

.PHONY: kf-golden
kf-golden:
	cd golden && ../$(PYTHON) kalman_ref.py

.PHONY: kf-test-replay
kf-test-replay: kf-golden
	cd chisel && $(SBT) "testOnly kalman.KalmanFilterReplayTest"

.PHONY: kf-golden-test
kf-golden-test:
	cd golden && ../$(PYTHON) -m pytest test_kalman_golden.py -v
	cd golden && ../$(PYTHON) diff_kalman.py

.PHONY: sim
sim: kf-test kf-test-replay kf-golden-test
	@echo "=== Tier 1 simulation complete ==="

# ── Tier 2: synthesis ─────────────────────────────────────────────────────────

.PHONY: kf-verilog
kf-verilog:
	cd chisel && $(SBT) --batch "runMain kalman.KalmanFilterVerilog"

.PHONY: kf-synth
kf-synth: kf-verilog
	$(VIVADO) -mode batch -source tcl/kalman_synth.tcl
	@echo "=== Timing report: vivado/timing_summary.rpt ==="
	@echo "=== Utilization:   vivado/utilization.rpt    ==="

.PHONY: synth
synth: kf-synth

# ── Visuals ────────────────────────────────────────────────────────────────────

.PHONY: kf-visuals
kf-visuals:
	cd docs && ../$(PYTHON) gen_visuals.py

# ── Clean ─────────────────────────────────────────────────────────────────────

.PHONY: clean
clean:
	rm -rf chisel/generated chisel/target chisel/project/target chisel/test_run_dir
	rm -rf vivado
	rm -f vectors/*.csv

.PHONY: all
all: sim synth
