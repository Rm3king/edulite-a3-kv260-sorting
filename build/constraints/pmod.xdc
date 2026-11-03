# User confirmed wiring on 2026-10-16: J2 schematic pins 1/3/9/11.
# Official Kria-PYNQ reference commit is recorded in third_party/reference_constraints.
set_property PACKAGE_PIN H12 [get_ports can_tx]
set_property PACKAGE_PIN E10 [get_ports can_rx]
set_property IOSTANDARD LVCMOS33 [get_ports {can_tx can_rx}]
set_property DRIVE 4 [get_ports can_tx]
set_property SLEW SLOW [get_ports can_tx]
set_property PULLUP true [get_ports can_rx]
# Asynchronous CAN RX is sampled by the core's two-register synchronizer.
set_false_path -from [get_ports can_rx] -to [get_cells -hier -filter {NAME =~ *rx_sync_tmp_reg}]
