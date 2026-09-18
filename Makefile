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

# One OOC synth + P&R run per clock period -> reports/N_<period>ns/{timing_summary,utilization,hold_paths}.rpt
PERIODS ?= 4.000 3.000

.PHONY: kf-synth
kf-synth: kf-verilog
	$(VIVADO) -mode batch -nolog -nojournal -source tcl/kalman_synth.tcl
	$(PYTHON) scripts/ppa_table.py

.PHONY: kf-sweep
kf-sweep: kf-verilog
	for p in $(PERIODS); do $(VIVADO) -mode batch -nolog -nojournal -source tcl/kalman_synth.tcl -tclargs period:$$p; done
	$(PYTHON) scripts/ppa_table.py

# Same two, driven from macOS into a Parallels Windows VM running Vivado (Apple Silicon)
.PHONY: vm-synth vm-sweep
vm-synth: kf-verilog
	./scripts/vivado_in_parallels.sh 4.000
	$(PYTHON) scripts/ppa_table.py

vm-sweep: kf-verilog
	./scripts/vivado_in_parallels.sh $(PERIODS)
	$(PYTHON) scripts/ppa_table.py

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
	rm -rf vivado build
	rm -f vectors/*.csv

.PHONY: all
all: sim synth
