#==============================================================================
# QEDMMA v3.1 Pro — Vivado Build Script for RFSoC 4x2
# Target: RFSoC 4x2 Board (XCZU48DR-1FFVG1517E)
#==============================================================================
# Author: Dr. Mladen Mešter / Nexellum d.o.o.
#==============================================================================

# Configuration
set project_name "qedmma_v31_rfsoc4x2"
set part_number "xczu48dr-ffvg1517-1-e"
set top_module "qedmma_v31_top"
set target_freq_mhz 250

puts "=============================================="
puts "QEDMMA v3.1 Pro Build for RFSoC 4x2"
puts "Part: $part_number"
puts "Target: $target_freq_mhz MHz"
puts "=============================================="

# Create project
create_project ${project_name} ./build_rfsoc4x2 -part ${part_number} -force

# Set board part (RFSoC 4x2)
set_property board_part realdigital.org:rfsoc4x2:part0:1.0 [current_project]

# Add RTL sources with RFSoC 4x2 define (default)
add_files -fileset sources_1 [glob ../rtl/*.sv]
set_property top ${top_module} [current_fileset]

# Set RFSoC 4x2 target define (explicit, though it's default)
set_property verilog_define {TARGET_RFSOC_4X2} [current_fileset]

# Create constraints
set constraints_file ./build_rfsoc4x2/${project_name}.srcs/constrs_1/new/rfsoc4x2_timing.xdc
file mkdir [file dirname $constraints_file]
set fp [open $constraints_file w]

puts $fp "#=============================================="
puts $fp "# QEDMMA v3.1 Pro Timing Constraints"
puts $fp "# Target: RFSoC 4x2 @ 250 MHz"
puts $fp "#=============================================="
puts $fp ""
puts $fp "# Primary clock (from PS)"
puts $fp "create_clock -period 4.000 -name aclk \[get_ports aclk\]"
puts $fp ""
puts $fp "# I/O delays for AXI-Stream"
puts $fp "set_input_delay -clock aclk -max 1.5 \[get_ports s_axis_*\]"
puts $fp "set_input_delay -clock aclk -min 0.5 \[get_ports s_axis_*\]"
puts $fp "set_output_delay -clock aclk -max 1.5 \[get_ports m_axis_*\]"
puts $fp "set_output_delay -clock aclk -min 0.5 \[get_ports m_axis_*\]"
puts $fp ""
puts $fp "# AXI-Lite (slower, from PS)"
puts $fp "set_input_delay -clock aclk -max 2.0 \[get_ports s_axi_*\]"
puts $fp "set_output_delay -clock aclk -max 2.0 \[get_ports s_axi_*\]"
puts $fp ""
puts $fp "# False paths for configuration registers"
puts $fp "set_false_path -from \[get_cells -hier -filter {NAME =~ *cfg_*_reg*}\]"
puts $fp ""
puts $fp "# RFSoC 4x2-specific: Integrated baluns on SMA (10MHz-10GHz rated)"
puts $fp "# No external balun delays needed"

close $fp
add_files -fileset constrs_1 $constraints_file

# Synthesis settings
set_property strategy Flow_PerfOptimized_high [get_runs synth_1]
set_property STEPS.SYNTH_DESIGN.ARGS.RETIMING true [get_runs synth_1]
set_property STEPS.SYNTH_DESIGN.ARGS.FSM_EXTRACTION one_hot [get_runs synth_1]

# Implementation settings
set_property strategy Performance_ExploreWithRemap [get_runs impl_1]
set_property STEPS.PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]

# Run synthesis
puts "Starting synthesis..."
launch_runs synth_1 -jobs 8
wait_on_run synth_1

if {[get_property STATUS [get_runs synth_1]] != "synth_design Complete!"} {
    puts "ERROR: Synthesis failed!"
    exit 1
}

# Generate reports
file mkdir ./reports_rfsoc4x2
open_run synth_1
report_utilization -file ./reports_rfsoc4x2/synth_util.rpt
report_timing_summary -file ./reports_rfsoc4x2/synth_timing.rpt

# Run implementation
puts "Starting implementation..."
launch_runs impl_1 -jobs 8
wait_on_run impl_1

# Generate reports
open_run impl_1
report_utilization -file ./reports_rfsoc4x2/impl_util.rpt
report_utilization -hierarchical -file ./reports_rfsoc4x2/impl_util_hier.rpt
report_timing_summary -file ./reports_rfsoc4x2/timing.rpt
report_power -file ./reports_rfsoc4x2/power.rpt

# Check timing
set wns [get_property STATS.WNS [get_runs impl_1]]
if {$wns < 0} {
    puts "WARNING: Timing not met! WNS = $wns ns"
} else {
    puts "Timing met: WNS = $wns ns"
}

# Generate bitstream
puts "Generating bitstream..."
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1

# Export for PYNQ
write_hw_platform -fixed -include_bit -force ./qedmma_v31_rfsoc4x2.xsa

puts "=============================================="
puts "Build complete!"
puts "Bitstream: ./build_rfsoc4x2/${project_name}.runs/impl_1/${top_module}.bit"
puts "XSA: ./qedmma_v31_rfsoc4x2.xsa (PYNQ compatible)"
puts "=============================================="
