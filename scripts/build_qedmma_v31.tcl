#==============================================================================
# QEDMMA v3.1 Pro — Vivado Build Script
# Target: Xilinx RFSoC ZU48DR (RFSoC 4x2 Board) @ 250MHz
#==============================================================================

# Project settings
set project_name "qedmma_v31"
set part_number "xczu48dr-ffvg1517-2-e"
set top_module "qedmma_v31_top"

# Create project
create_project ${project_name} ./vivado_project -part ${part_number} -force

# Set target board (RFSoC 4x2)
set_property board_part realdigital.org:rfsoc4x2:part0:1.0 [current_project]

# Add RTL sources
add_files -fileset sources_1 [glob ../rtl/*.sv]

# Set top module
set_property top ${top_module} [current_fileset]

# Add constraints
set_property PROCESSING_ORDER EARLY [get_files -of_objects [get_filesets constrs_1]]

# Create constraints file
set fp [open ./constraints.xdc w]
puts $fp "#=============================================================================="
puts $fp "# QEDMMA v3.1 Pro — Timing Constraints"
puts $fp "# Target: RFSoC 4x2 (ZU48DR) @ 250 MHz"
puts $fp "#=============================================================================="
puts $fp ""
puts $fp "# Primary clock (250 MHz from PS)"
puts $fp "create_clock -period 4.000 -name aclk \[get_ports aclk\]"
puts $fp ""
puts $fp "# Clock uncertainty for setup"
puts $fp "set_clock_uncertainty 0.100 \[get_clocks aclk\]"
puts $fp ""
puts $fp "# AXI-Stream input constraints"
puts $fp "set_input_delay -clock aclk -max 1.0 \[get_ports s_axis_*\]"
puts $fp "set_input_delay -clock aclk -min 0.5 \[get_ports s_axis_*\]"
puts $fp ""
puts $fp "# AXI-Stream output constraints"
puts $fp "set_output_delay -clock aclk -max 1.0 \[get_ports m_axis_*\]"
puts $fp "set_output_delay -clock aclk -min 0.5 \[get_ports m_axis_*\]"
puts $fp ""
puts $fp "# AXI-Lite constraints (slower, from PS)"
puts $fp "set_input_delay -clock aclk -max 2.0 \[get_ports s_axi_*\]"
puts $fp "set_output_delay -clock aclk -max 2.0 \[get_ports s_axi_*\]"
puts $fp ""
puts $fp "# Configuration registers are static — false path"
puts $fp "set_false_path -from \[get_cells cfg_*_reg*\]"
puts $fp ""
puts $fp "# Status outputs are multicycle"
puts $fp "set_multicycle_path 2 -setup -to \[get_ports dominant_mode*\]"
puts $fp "set_multicycle_path 1 -hold -to \[get_ports dominant_mode*\]"
puts $fp "set_multicycle_path 2 -setup -to \[get_ports track_count*\]"
puts $fp "set_multicycle_path 1 -hold -to \[get_ports track_count*\]"
close $fp

add_files -fileset constrs_1 ./constraints.xdc

# Synthesis settings — optimize for ZU48DR
set_property strategy Flow_PerfOptimized_high [get_runs synth_1]
set_property STEPS.SYNTH_DESIGN.ARGS.RETIMING true [get_runs synth_1]
set_property STEPS.SYNTH_DESIGN.ARGS.FLATTEN_HIERARCHY rebuilt [get_runs synth_1]
set_property STEPS.SYNTH_DESIGN.ARGS.KEEP_EQUIVALENT_REGISTERS true [get_runs synth_1]
set_property STEPS.SYNTH_DESIGN.ARGS.FSM_EXTRACTION one_hot [get_runs synth_1]

# Implementation settings — aggressive timing closure
set_property strategy Performance_ExploreWithRemap [get_runs impl_1]
set_property STEPS.OPT_DESIGN.ARGS.DIRECTIVE ExploreWithRemap [get_runs impl_1]
set_property STEPS.PLACE_DESIGN.ARGS.DIRECTIVE ExtraNetDelay_high [get_runs impl_1]
set_property STEPS.PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]
set_property STEPS.PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]
set_property STEPS.ROUTE_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]

# Run synthesis
puts "Starting synthesis..."
launch_runs synth_1 -jobs 8
wait_on_run synth_1

# Check synthesis status
if {[get_property STATUS [get_runs synth_1]] != "synth_design Complete!"} {
    puts "ERROR: Synthesis failed!"
    exit 1
}

# Report utilization after synthesis
open_run synth_1
report_utilization -file ./reports/synth_utilization.rpt
report_timing_summary -file ./reports/synth_timing.rpt

# Run implementation
puts "Starting implementation..."
launch_runs impl_1 -jobs 8
wait_on_run impl_1

# Check implementation status
if {[get_property STATUS [get_runs impl_1]] != "route_design Complete!"} {
    puts "ERROR: Implementation failed!"
    exit 1
}

# Generate reports
file mkdir ./reports
open_run impl_1
report_utilization -file ./reports/impl_utilization.rpt
report_utilization -hierarchical -file ./reports/impl_utilization_hier.rpt
report_timing_summary -file ./reports/timing_summary.rpt
report_timing -sort_by group -max_paths 50 -path_type summary -file ./reports/timing_paths.rpt
report_power -file ./reports/power.rpt
report_drc -file ./reports/drc.rpt
report_methodology -file ./reports/methodology.rpt

# Check timing
set timing_slack [get_property SLACK [get_timing_paths -max_paths 1 -setup]]
if {$timing_slack < 0} {
    puts "WARNING: Design has negative slack: ${timing_slack} ns"
} else {
    puts "Timing met with slack: ${timing_slack} ns"
}

# Generate bitstream
puts "Generating bitstream..."
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1

# Export hardware for Vitis/PYNQ
write_hw_platform -fixed -include_bit -force ./qedmma_v31.xsa

puts "=============================================="
puts "Build complete!"
puts "Bitstream: ./vivado_project/${project_name}.runs/impl_1/${top_module}.bit"
puts "XSA: ./qedmma_v31.xsa"
puts "Reports: ./reports/"
puts "=============================================="
