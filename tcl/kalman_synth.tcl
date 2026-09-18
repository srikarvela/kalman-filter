# =============================================================================
# kalman_synth.tcl -- out-of-context synthesis + place & route of KalmanFilter
#
#   vivado -mode batch -source tcl/kalman_synth.tcl                       (4.000 ns, from the XDC)
#   vivado -mode batch -source tcl/kalman_synth.tcl -tclargs period:3.000 (override the clock period)
#
# Non-project flow, no board or pin bring-up (the same flow as cordic-engine's
# tcl/cordic_synth.tcl): the Chisel-generated KalmanFilter.v is synthesized,
# placed and routed on its own against constraints/kalman_clock.xdc, so the
# numbers are real post-route timing/utilization, not synth-only estimates.
# Unlike the sibling feed-handler repo's block-design flow this does not wire
# a PS7/AXI system; Tier 3 on-hardware integration is out of scope (README).
#
# Outputs (committed)
#   reports/N_<period>ns/timing_summary.rpt      report_timing_summary
#   reports/N_<period>ns/utilization.rpt         report_utilization
#   reports/N_<period>ns/hold_paths.rpt          report_timing -hold -max_paths 30
#   reports/ppa.csv is then built by scripts/ppa_table.py from those reports.
# Outputs (local only, gitignored)
#   build/kalman_<period>ns_routed.dcp           routed checkpoint for later queries
# =============================================================================

set part "xc7z020clg400-1" ;# Zynq-7020 / PYNQ-Z2, matches the sibling repos

set period_override ""
if {[string match "period:*" [lindex $argv 0]]} {
    set period_override [string range [lindex $argv 0] 7 end]
}

set here [file normalize [file dirname [info script]]]
set root [file normalize [file join $here ..]]
set rdir [file join $root reports]
set bdir [file join $root build]
file mkdir $rdir
file mkdir $bdir

create_project -in_memory -part $part
read_verilog [file join $root chisel generated KalmanFilter.v]
read_xdc -mode out_of_context [file join $root constraints kalman_clock.xdc]

# -keep_equivalent_registers: the reciprocal replicates its shift-amount registers on purpose (fan-out)
synth_design -top KalmanFilter -part $part -mode out_of_context -keep_equivalent_registers
if {$period_override ne ""} {
    create_clock -name clock -period $period_override [get_ports clock]
}
set period [get_property PERIOD [get_clocks clock]]
set tag [format "N_%.3fns" $period]
set out [file join $rdir $tag]
file mkdir $out
puts "=== KalmanFilter OOC: period=$period ns part=$part -> $out ==="

opt_design
place_design
phys_opt_design
route_design

report_timing_summary -file [file join $out timing_summary.rpt]
report_utilization    -file [file join $out utilization.rpt]
report_timing -hold -max_paths 30 -nworst 1 -file [file join $out hold_paths.rpt]
write_checkpoint -force [file join $bdir "kalman_${period}ns_routed.dcp"]

set wns  [get_property SLACK [get_timing_paths -max_paths 1 -setup]]
set whs  [get_property SLACK [get_timing_paths -max_paths 1 -hold]]
set fmax [format %.1f [expr {1000.0 / ($period - $wns)}]]
puts "=== KalmanFilter period=$period: WNS=$wns WHS=$whs Fmax=${fmax}MHz (post-route; counts are in utilization.rpt) ==="
close_project
puts "=== reports written under $out; run scripts/ppa_table.py to rebuild reports/ppa.csv ==="
