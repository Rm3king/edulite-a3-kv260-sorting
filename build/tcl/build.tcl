set root [file normalize [file join [file dirname [info script]] ../../..]]
set exp "$root/contest/kv260_can"
set stage [lindex $argv 0]
proc main {} {
 global root exp stage
 set jobs 2
 set threads 4
 if {[info exists ::env(FPGA_BUILD_JOBS)]} {set jobs $::env(FPGA_BUILD_JOBS)}
 if {[info exists ::env(FPGA_BUILD_THREADS)]} {set threads $::env(FPGA_BUILD_THREADS)}
 if {![string is integer -strict $jobs] || $jobs < 1 || $jobs > 16} {error "FPGA_BUILD_JOBS must be 1..16"}
 if {![string is integer -strict $threads] || $threads < 1 || $threads > 8} {error "FPGA_BUILD_THREADS must be 1..8"}
 set_param general.maxThreads $threads
 set_param board.repoPaths [list "$root/third_party/board_files"]
 file mkdir "$exp/build" "$exp/result"
 file delete -force "$exp/result/build_validation.txt"
 create_project -force kv260_can "$exp/build/project" -part xck26-sfvc784-2LV-c
 set_property board_part xilinx.com:kv260_som:part0:1.4 [current_project]
 set_property target_language Verilog [current_project]
 add_files [glob "$root/third_party/opencores_can/*.v"]
 add_files [glob "$exp/rtl/*.sv"]
 add_files [glob "$exp/rtl/*.v"]
 set_property include_dirs [list "$root/third_party/opencores_can"] [current_fileset]
 set_property verilog_define CAN_WISHBONE_IF [current_fileset]
 if {$stage eq "ooc"} {
  set_property top kv260_can_top [current_fileset]
  synth_design -top kv260_can_top -part xck26-sfvc784-2LV-c -mode out_of_context
  create_clock -period 10 [get_ports clk]
  report_utilization -file "$exp/result/ooc_utilization.rpt"
  report_timing_summary -file "$exp/result/ooc_timing.rpt"
  write_checkpoint -force "$exp/build/kv260_can_ooc.dcp"
  return
 }
 set vision_repo "$root/contest/vision_pre/build/vision_pre/hls/impl/ip"
 if {![file exists "$vision_repo/component.xml"]} {error "Build and validate contest/vision_pre HLS IP first"}
 set_property ip_repo_paths [list $vision_repo] [current_project]
 update_ip_catalog
 create_bd_design system
 create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e:* ps
 apply_bd_automation -rule xilinx.com:bd_rule:zynq_ultra_ps_e -config {apply_board_preset "1"} [get_bd_cells ps]
 set_property -dict [list CONFIG.PSU__USE__M_AXI_GP0 1 CONFIG.PSU__USE__M_AXI_GP1 0 CONFIG.PSU__USE__M_AXI_GP2 0 CONFIG.PSU__USE__S_AXI_GP2 1 CONFIG.PSU__SAXIGP2__DATA_WIDTH 128 CONFIG.PSU__FPGA_PL0_ENABLE 1 CONFIG.PSU__FPGA_PL1_ENABLE 0 CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ 100] [get_bd_cells ps]
 create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:* reset
 set_property CONFIG.C_AUX_RESET_HIGH 1 [get_bd_cells reset]
 foreach {name value} {zero 0 one 1} {
  create_bd_cell -type inline_hdl -vlnv xilinx.com:inline_hdl:ilconstant $name
  set_property -dict [list CONFIG.CONST_WIDTH 1 CONFIG.CONST_VAL $value] [get_bd_cells $name]
 }
 create_bd_cell -type module -reference kv260_can_top can
 # Wrist-camera pre-processing: AXI-Lite control at 0xA0020000, frame read / mask write to DDR through HP0 (not coherent;
 # the ARM cleans/invalidates the shared buffer). Isolated from the CAN engine: no shared logic or registers.
 create_bd_cell -type ip -vlnv xilinx.com:hls:vision_pre:1.0 vision
 create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:* interconnect
 set_property -dict [list CONFIG.NUM_SI 1 CONFIG.NUM_MI 2] [get_bd_cells interconnect]
 create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:* vision_memory
 set_property -dict [list CONFIG.NUM_SI 2 CONFIG.NUM_MI 1] [get_bd_cells vision_memory]
 puts "CAN interfaces: [get_bd_intf_pins can/*]"
 connect_bd_intf_net [get_bd_intf_pins ps/M_AXI_HPM0_FPD] [get_bd_intf_pins interconnect/S00_AXI]
 connect_bd_intf_net [get_bd_intf_pins interconnect/M00_AXI] [get_bd_intf_pins can/s_axi]
 connect_bd_intf_net [get_bd_intf_pins interconnect/M01_AXI] [get_bd_intf_pins vision/s_axi_control]
 connect_bd_intf_net [get_bd_intf_pins vision/m_axi_gmem0] [get_bd_intf_pins vision_memory/S00_AXI]
 connect_bd_intf_net [get_bd_intf_pins vision/m_axi_gmem1] [get_bd_intf_pins vision_memory/S01_AXI]
 connect_bd_intf_net [get_bd_intf_pins vision_memory/M00_AXI] [get_bd_intf_pins ps/S_AXI_HP0_FPD]
 connect_bd_net [get_bd_pins ps/pl_clk0] [get_bd_pins ps/maxihpm0_fpd_aclk] [get_bd_pins reset/slowest_sync_clk] [get_bd_pins interconnect/aclk] [get_bd_pins can/clk] [get_bd_pins vision/ap_clk] [get_bd_pins vision_memory/aclk] [get_bd_pins ps/saxihp0_fpd_aclk]
 connect_bd_net [get_bd_pins ps/pl_resetn0] [get_bd_pins reset/ext_reset_in]
 connect_bd_net [get_bd_pins zero/dout] [get_bd_pins reset/aux_reset_in] [get_bd_pins reset/mb_debug_sys_rst]
 connect_bd_net [get_bd_pins one/dout] [get_bd_pins reset/dcm_locked]
 connect_bd_net [get_bd_pins reset/peripheral_aresetn] [get_bd_pins interconnect/aresetn] [get_bd_pins can/resetn] [get_bd_pins vision/ap_rst_n] [get_bd_pins vision_memory/aresetn]
 create_bd_port -dir I can_rx
 create_bd_port -dir O can_tx
 connect_bd_net [get_bd_ports can_rx] [get_bd_pins can/can_rx]
 connect_bd_net [get_bd_ports can_tx] [get_bd_pins can/can_tx]
 assign_bd_address -offset 0xA0000000 -range 64K -target_address_space [get_bd_addr_spaces ps/Data] [get_bd_addr_segs can/s_axi/reg0] -force
 # FK stays out of this bitstream (0xA0010000 unused); the vision core keeps its address so software does not change.
 assign_bd_address -offset 0xA0020000 -range 64K -target_address_space [get_bd_addr_spaces ps/Data] [get_bd_addr_segs vision/s_axi_control/Reg] -force
 # CMA (cma=1000M) sits at 0x37800000 on this board: the low DDR window is enough. HP0 stays 128-bit: the boot firmware
 # programs every AFI FM port to 128 bits (RDCTRL/WRCTRL = 0x3b0, read 2026-10-28) and loading a bitstream does not change it.
 foreach space {Data_m_axi_gmem0 Data_m_axi_gmem1} {
  assign_bd_address -offset 0x00000000 -range 2G -target_address_space [get_bd_addr_spaces vision/$space] [get_bd_addr_segs ps/SAXIGP2/HP0_DDR_LOW] -force
 }
 validate_bd_design
 if {[get_property CONFIG.C_AUX_RESET_HIGH [get_bd_cells reset]] != 1} {error "Aux reset polarity wrong"}
 save_bd_design
 write_bd_tcl -force "$exp/result/system_bd.tcl"
 generate_target all [get_files system.bd]
 add_files [make_wrapper -files [get_files system.bd] -top]
 set_property top system_wrapper [current_fileset]
 add_files -fileset constrs_1 "$exp/constraints/pmod.xdc"
 update_compile_order -fileset sources_1
 if {$stage eq "create"} {return}
 launch_runs synth_1 -jobs $jobs
 wait_on_run synth_1
 if {![string match "*Complete*" [get_property STATUS [get_runs synth_1]]]} {error "Synthesis failed"}
 open_run synth_1
 report_utilization -file "$exp/result/synth_utilization.rpt"
 report_timing_summary -file "$exp/result/synth_timing.rpt"
 if {$stage eq "synth"} {return}
 set_property strategy Performance_Explore [get_runs impl_1]
 launch_runs impl_1 -to_step route_design -jobs $jobs
 wait_on_run impl_1
 if {![string match "*Complete*" [get_property STATUS [get_runs impl_1]]]} {error "Implementation failed"}
 open_run impl_1
 report_utilization -file "$exp/result/impl_utilization.rpt"
 report_timing_summary -report_unconstrained -file "$exp/result/impl_timing.rpt"
 report_drc -file "$exp/result/drc.rpt"
 report_clocks -file "$exp/result/clocks.rpt"
 check_timing -verbose -file "$exp/result/check_timing.rpt"
 report_bus_skew -file "$exp/result/bus_skew.rpt"
 foreach dtype {max min} {
  set p [get_timing_paths -delay_type $dtype -max_paths 1]
  if {[llength $p]==0 || [get_property SLACK $p]<0} {error "Timing failed: $dtype"}
 }
 if {$stage eq "impl"} {return}
 close_design
 launch_runs impl_1 -to_step write_bitstream -jobs $jobs
 wait_on_run impl_1
 if {![string match "*Complete*" [get_property STATUS [get_runs impl_1]]]} {error "Bitstream failed"}
 open_run impl_1
 file copy -force "$exp/build/project/kv260_can.runs/impl_1/system_wrapper.bit" "$exp/build/kv260_can.bit"
 write_hw_platform -fixed -include_bit -force -file "$exp/build/kv260_can.xsa"
 set f [open "$exp/result/build_validation.txt" w];puts $f "PASS: complete CAN system implementation and bitstream";close $f
}
if {$stage ni {create ooc synth impl bitstream}} {error "Unknown stage $stage"}
if {[catch main message opts]} {puts stderr [dict get $opts -errorinfo];exit 1}
puts "TEST_PASS Vivado CAN stage $stage"
exit 0
