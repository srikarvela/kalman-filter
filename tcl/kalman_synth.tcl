# Out-of-context (OOC) synthesis + implementation for the KalmanFilter IP block.
# Unlike the sibling repo's full block-design flow (which wires HLS IPs + the order
# book into a PS7/AXI system targeting real PYNQ-Z2 pins), this is a standalone
# non-project-mode flow: it synthesizes and places/routes KalmanFilter on its own,
# with a clock constraint but no board/pin bring-up, since Tier 3 on-hardware
# integration is out of scope for this pass (see README roadmap). This still produces
# a real post-route timing/utilization report, not just a synth-only estimate.
#
# Usage: vivado -mode batch -source tcl/kalman_synth.tcl

set part "xc7z020clg400-1" ;# Zynq-7020 / PYNQ-Z2, matches the sibling repo's target

set outdir "./vivado"
file mkdir $outdir

read_verilog -sv ./chisel/generated/KalmanFilter.v
read_xdc ./constraints/kalman_clock.xdc

synth_design -top KalmanFilter -part $part -mode out_of_context

opt_design
place_design
route_design

report_timing_summary -file $outdir/timing_summary.rpt
report_utilization    -file $outdir/utilization.rpt

puts "=== KalmanFilter OOC synthesis complete -- check $outdir/timing_summary.rpt and $outdir/utilization.rpt ==="
