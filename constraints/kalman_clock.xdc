# OOC IP-block clock constraint for KalmanFilter. Chisel emits implicit `clock`/`reset`
# ports (unlike the sibling repo's PS7-sourced clock naming in its board-level design),
# targeting the same 250 MHz / 4.000 ns period used throughout the fpga-crypto-feed-handler
# pipeline for a consistent cross-project comparison.
create_clock -name clock -period 4.000 [get_ports clock]

# Standard OOC boundary constraints: treat block I/O as arriving/departing near the
# clock edge, since this isn't a full board-level timing closure (no real board I/O).
set_input_delay  -clock clock 0.500 [all_inputs]
set_output_delay -clock clock 0.500 [all_outputs]
