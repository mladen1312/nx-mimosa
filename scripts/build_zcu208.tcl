#==============================================================================
# NX-MIMOSA — Vivado Build Script for ZCU208
# Multi-model IMM Optimal Smoothing Algorithm
# Target: AMD ZCU208 Evaluation Kit (XCZU48DR-2FSVG1517E)
#==============================================================================
# Author: Dr. Mladen Mešter / Nexellum d.o.o.
#==============================================================================

set project_name "nx_mimosa_zcu208"
set part_number "xczu48dr-fsvg1517-2-e"
set top_module "nx_mimosa_top"
set target_freq_mhz 250

puts "=============================================="
puts "NX-MIMOSA Build for ZCU208"
puts "Multi-model IMM Optimal Smoothing Algorithm"
puts "Part: $part_number"
puts "Target: $target_freq_mhz MHz"
puts "=============================================="

create_project ${project_name} ./build_zcu208 -part ${part_number} -force
set_property board_part xilinx.com:zcu208:part0:2.0 [current_project]

add_files -fileset sources_1 [glob ../rtl/*.sv]
set_property top ${top_module} [current_fileset]
set_property verilog_define {TARGET_ZCU208} [current_fileset]

# Constraints
set constraints_file ./build_zcu208/${project_name}.srcs/constrs_1/new/timing.xdc
file mkdir [file dirname $constraints_file]
set fp [open $constraints_file w]
puts $fp "# NX-MIMOSA Timing Constraints - ZCU208 @ 250 MHz"
puts $fp "create_clock -period 4.000 -name aclk \[get_ports aclk\]"
puts $fp "set_input_delay -clock aclk -max 1.5 \[get_ports s_axis_*\]"
puts $fp "set_input_delay -clock aclk -min 0.5 \[get_ports s_axis_*\]"
puts $fp "set_output_delay -clock aclk -max 1.5 \[get_ports m_axis_*\]"
puts $fp "set_output_delay -clock aclk -min 0.5 \[get_ports m_axis_*\]"
puts $fp "set_input_delay -clock aclk -max 2.0 \[get_ports s_axi_*\]"
puts $fp "set_output_delay -clock aclk -max 2.0 \[get_ports s_axi_*\]"
puts $fp "set_false_path -from \[get_cells -hier -filter {NAME =~ *cfg_*_reg*}\]"
close $fp
add_files -fileset constrs_1 $constraints_file

# Synthesis
set_property strategy Flow_PerfOptimized_high [get_runs synth_1]
set_property STEPS.SYNTH_DESIGN.ARGS.RETIMING true [get_runs synth_1]

# Implementation
set_property strategy Performance_ExploreWithRemap [get_runs impl_1]
set_property STEPS.PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]

puts "Starting synthesis..."
launch_runs synth_1 -jobs 8
wait_on_run synth_1

file mkdir ./reports_zcu208
open_run synth_1
report_utilization -file ./reports_zcu208/synth_util.rpt

puts "Starting implementation..."
launch_runs impl_1 -jobs 8
wait_on_run impl_1

open_run impl_1
report_utilization -file ./reports_zcu208/impl_util.rpt
report_timing_summary -file ./reports_zcu208/timing.rpt

set wns [get_property STATS.WNS [get_runs impl_1]]
puts "Timing: WNS = $wns ns"

puts "Generating bitstream..."
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1

write_hw_platform -fixed -include_bit -force ./nx_mimosa_zcu208.xsa

puts "=============================================="
puts "Build complete!"
puts "Bitstream: nx_mimosa_zcu208.bit"
puts "XSA: nx_mimosa_zcu208.xsa"
puts "=============================================="
