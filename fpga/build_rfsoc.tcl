# ═══════════════════════════════════════════════════════════════════════════════
# NX-MIMOSA Vivado Build Script
# Target: RFSoC ZU48DR (RFSoC 4x2 / ZCU208)
# 
# Usage:
#   vivado -mode batch -source build_rfsoc.tcl
#   vivado -mode batch -source build_rfsoc.tcl -tclargs rfsoc4x2
#   vivado -mode batch -source build_rfsoc.tcl -tclargs zcu208
#
# Author: Dr. Mladen Mešter / Nexellum d.o.o.
# ═══════════════════════════════════════════════════════════════════════════════

# ═══════════════════════════════════════════════════════════════════════════════
# Configuration
# ═══════════════════════════════════════════════════════════════════════════════

# Default target board
set board "rfsoc4x2"
if {$argc > 0} {
    set board [lindex $argv 0]
}

# Board-specific settings
switch $board {
    "rfsoc4x2" {
        set part "xczu48dr-ffvg1517-2-e"
        set board_part "realdigital.org:rfsoc4x2:part0:1.0"
        set project_name "nx_mimosa_rfsoc4x2"
    }
    "zcu208" {
        set part "xczu48dr-fsvg1517-2-e"
        set board_part "xilinx.com:zcu208:part0:2.0"
        set project_name "nx_mimosa_zcu208"
    }
    "zcu216" {
        set part "xczu49dr-ffvf1760-2-e"
        set board_part "xilinx.com:zcu216:part0:2.0"
        set project_name "nx_mimosa_zcu216"
    }
    default {
        puts "ERROR: Unknown board '$board'"
        puts "Supported: rfsoc4x2, zcu208, zcu216"
        exit 1
    }
}

puts "═══════════════════════════════════════════════════════════════════════════════"
puts "NX-MIMOSA FPGA Build"
puts "Target: $board ($part)"
puts "═══════════════════════════════════════════════════════════════════════════════"

# Paths
set script_dir [file dirname [info script]]
set rtl_dir [file join $script_dir "../rtl"]
set proj_dir [file join $script_dir "build_$board"]
set output_dir [file join $script_dir "output"]

# ═══════════════════════════════════════════════════════════════════════════════
# Create Project
# ═══════════════════════════════════════════════════════════════════════════════

puts "\n[1/8] Creating project..."

# Remove existing project
file delete -force $proj_dir

create_project $project_name $proj_dir -part $part
set_property board_part $board_part [current_project]
set_property target_language Verilog [current_project]

# ═══════════════════════════════════════════════════════════════════════════════
# Add RTL Sources
# ═══════════════════════════════════════════════════════════════════════════════

puts "\n[2/8] Adding RTL sources..."

# Add all SystemVerilog files
set rtl_files [glob -nocomplain [file join $rtl_dir "*.sv"]]
foreach f $rtl_files {
    puts "  Adding: [file tail $f]"
    add_files -norecurse $f
}

# Set top module
set_property top nx_mimosa_axi_wrapper [current_fileset]

# ═══════════════════════════════════════════════════════════════════════════════
# Create Block Design
# ═══════════════════════════════════════════════════════════════════════════════

puts "\n[3/8] Creating block design..."

create_bd_design "system"

# Add Zynq UltraScale+ MPSoC
create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e:3.5 zynq_ultra_ps_e_0
apply_bd_automation -rule xilinx.com:bd_rule:zynq_ultra_ps_e -config {apply_board_preset "1"} [get_bd_cells zynq_ultra_ps_e_0]

# Configure PS for AXI
set_property -dict [list \
    CONFIG.PSU__USE__M_AXI_GP0 {1} \
    CONFIG.PSU__USE__M_AXI_GP1 {0} \
    CONFIG.PSU__USE__S_AXI_GP0 {0} \
    CONFIG.PSU__FPGA_PL0_ENABLE {1} \
    CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ {250} \
] [get_bd_cells zynq_ultra_ps_e_0]

# Add NX-MIMOSA module
create_bd_cell -type module -reference nx_mimosa_axi_wrapper nx_mimosa_0

# Add AXI Interconnect
create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 axi_interconnect_0
set_property CONFIG.NUM_MI {1} [get_bd_cells axi_interconnect_0]

# Connect clocks and resets
connect_bd_net [get_bd_pins zynq_ultra_ps_e_0/pl_clk0] [get_bd_pins nx_mimosa_0/aclk]
connect_bd_net [get_bd_pins zynq_ultra_ps_e_0/pl_clk0] [get_bd_pins axi_interconnect_0/ACLK]
connect_bd_net [get_bd_pins zynq_ultra_ps_e_0/pl_clk0] [get_bd_pins axi_interconnect_0/S00_ACLK]
connect_bd_net [get_bd_pins zynq_ultra_ps_e_0/pl_clk0] [get_bd_pins axi_interconnect_0/M00_ACLK]

connect_bd_net [get_bd_pins zynq_ultra_ps_e_0/pl_resetn0] [get_bd_pins nx_mimosa_0/aresetn]
connect_bd_net [get_bd_pins zynq_ultra_ps_e_0/pl_resetn0] [get_bd_pins axi_interconnect_0/ARESETN]
connect_bd_net [get_bd_pins zynq_ultra_ps_e_0/pl_resetn0] [get_bd_pins axi_interconnect_0/S00_ARESETN]
connect_bd_net [get_bd_pins zynq_ultra_ps_e_0/pl_resetn0] [get_bd_pins axi_interconnect_0/M00_ARESETN]

# Connect AXI interfaces
connect_bd_intf_net [get_bd_intf_pins zynq_ultra_ps_e_0/M_AXI_HPM0_FPD] [get_bd_intf_pins axi_interconnect_0/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins axi_interconnect_0/M00_AXI] [get_bd_intf_pins nx_mimosa_0/s_axi]

# Connect interrupt
connect_bd_net [get_bd_pins nx_mimosa_0/irq] [get_bd_pins zynq_ultra_ps_e_0/pl_ps_irq0]

# Assign address
assign_bd_address -target_address_space /zynq_ultra_ps_e_0/Data [get_bd_addr_segs nx_mimosa_0/s_axi/reg0] -range 64K -offset 0xA0000000

# Validate design
validate_bd_design

# Generate output products
generate_target all [get_files system.bd]

# Create wrapper
make_wrapper -files [get_files system.bd] -top
add_files -norecurse [file join $proj_dir "$project_name.gen/sources_1/bd/system/hdl/system_wrapper.v"]
set_property top system_wrapper [current_fileset]

# ═══════════════════════════════════════════════════════════════════════════════
# Add Constraints
# ═══════════════════════════════════════════════════════════════════════════════

puts "\n[4/8] Adding constraints..."

# Create timing constraints
set constraints_file [file join $proj_dir "timing.xdc"]
set fp [open $constraints_file w]
puts $fp "# NX-MIMOSA Timing Constraints"
puts $fp ""
puts $fp "# Clock constraint (250 MHz from PS)"
puts $fp "# Clock is automatically constrained by PS configuration"
puts $fp ""
puts $fp "# False paths for async resets"
puts $fp "set_false_path -from \[get_pins zynq_ultra_ps_e_0/inst/PS8_i/PLPSIRQ0\]"
puts $fp ""
puts $fp "# Input/Output delays"
puts $fp "# Adjust based on actual board routing"
puts $fp "# set_input_delay -clock \[get_clocks clk_pl_0\] 2.0 \[get_ports s_axis_*\]"
puts $fp "# set_output_delay -clock \[get_clocks clk_pl_0\] 2.0 \[get_ports m_axis_*\]"
close $fp

add_files -fileset constrs_1 $constraints_file

# ═══════════════════════════════════════════════════════════════════════════════
# Synthesis
# ═══════════════════════════════════════════════════════════════════════════════

puts "\n[5/8] Running synthesis..."

set_property strategy Flow_PerfOptimized_high [get_runs synth_1]
launch_runs synth_1 -jobs 8
wait_on_run synth_1

if {[get_property PROGRESS [get_runs synth_1]] != "100%"} {
    puts "ERROR: Synthesis failed!"
    exit 1
}

# Generate synthesis reports
file mkdir [file join $output_dir "reports"]
open_run synth_1
report_utilization -file [file join $output_dir "reports/utilization_synth.rpt"]
report_timing_summary -file [file join $output_dir "reports/timing_synth.rpt"]

# ═══════════════════════════════════════════════════════════════════════════════
# Implementation
# ═══════════════════════════════════════════════════════════════════════════════

puts "\n[6/8] Running implementation..."

set_property strategy Performance_ExplorePostRoutePhysOpt [get_runs impl_1]
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1

if {[get_property PROGRESS [get_runs impl_1]] != "100%"} {
    puts "ERROR: Implementation failed!"
    exit 1
}

# ═══════════════════════════════════════════════════════════════════════════════
# Generate Reports
# ═══════════════════════════════════════════════════════════════════════════════

puts "\n[7/8] Generating reports..."

open_run impl_1

report_utilization -file [file join $output_dir "reports/utilization_impl.rpt"]
report_timing_summary -file [file join $output_dir "reports/timing_impl.rpt"]
report_power -file [file join $output_dir "reports/power.rpt"]
report_drc -file [file join $output_dir "reports/drc.rpt"]
report_methodology -file [file join $output_dir "reports/methodology.rpt"]

# ═══════════════════════════════════════════════════════════════════════════════
# Copy Outputs
# ═══════════════════════════════════════════════════════════════════════════════

puts "\n[8/8] Copying output files..."

file mkdir $output_dir

# Copy bitstream
set bit_file [glob -nocomplain [file join $proj_dir "$project_name.runs/impl_1/*.bit"]]
if {[llength $bit_file] > 0} {
    file copy -force [lindex $bit_file 0] [file join $output_dir "${project_name}.bit"]
    puts "  Bitstream: ${project_name}.bit"
}

# Copy hardware handoff
set xsa_file [file join $output_dir "${project_name}.xsa"]
write_hw_platform -fixed -include_bit -force -file $xsa_file
puts "  Hardware platform: ${project_name}.xsa"

# ═══════════════════════════════════════════════════════════════════════════════
# Summary
# ═══════════════════════════════════════════════════════════════════════════════

puts "\n═══════════════════════════════════════════════════════════════════════════════"
puts "BUILD COMPLETE"
puts "═══════════════════════════════════════════════════════════════════════════════"
puts ""
puts "Output files:"
puts "  Bitstream:  $output_dir/${project_name}.bit"
puts "  XSA:        $output_dir/${project_name}.xsa"
puts "  Reports:    $output_dir/reports/"
puts ""

# Print utilization summary
puts "Resource Utilization:"
set rpt [report_utilization -return_string]
foreach line [split $rpt "\n"] {
    if {[regexp {^\| (CLB|LUT|FF|BRAM|DSP)} $line]} {
        puts "  $line"
    }
}

puts ""
puts "Timing:"
set timing_rpt [report_timing_summary -return_string]
foreach line [split $timing_rpt "\n"] {
    if {[regexp {WNS|TNS|WHS|THS} $line]} {
        puts "  $line"
    }
}

close_project
puts "\nDone."
