#==============================================================================
# QEDMMA v3.1 Pro — Vivado Build Script
# Target: Xilinx RFSoC ZU28DR @ 250MHz
#==============================================================================

# Project settings
set project_name "qedmma_v31"
set part_number "xczu28dr-ffvg1517-2-e"
set top_module "qedmma_v31_top"

# Create project
create_project ${project_name} ./vivado_project -part ${part_number} -force

# Add RTL sources
add_files -fileset sources_1 [glob ../rtl/*.sv]

# Set top module
set_property top ${top_module} [current_fileset]

# Add constraints
set_property PROCESSING_ORDER EARLY [get_files -of_objects [get_filesets constrs_1]]

# Create constraints file
set fp [open ./constraints.xdc w]
puts $fp "# Clock constraint (250 MHz)"
puts $fp "create_clock -period 4.000 -name aclk \[get_ports aclk\]"
puts $fp ""
puts $fp "# IO constraints"
puts $fp "set_input_delay -clock aclk -max 1.0 \[get_ports s_axis_*\]"
puts $fp "set_input_delay -clock aclk -min 0.5 \[get_ports s_axis_*\]"
puts $fp "set_output_delay -clock aclk -max 1.0 \[get_ports m_axis_*\]"
puts $fp "set_output_delay -clock aclk -min 0.5 \[get_ports m_axis_*\]"
puts $fp ""
puts $fp "# Timing exceptions"
puts $fp "set_false_path -from \[get_cells cfg_*_reg*\]"
close $fp

add_files -fileset constrs_1 ./constraints.xdc

# Synthesis settings
set_property strategy Flow_PerfOptimized_high [get_runs synth_1]
set_property STEPS.SYNTH_DESIGN.ARGS.RETIMING true [get_runs synth_1]
set_property STEPS.SYNTH_DESIGN.ARGS.FLATTEN_HIERARCHY rebuilt [get_runs synth_1]

# Implementation settings
set_property strategy Performance_ExploreWithRemap [get_runs impl_1]
set_property STEPS.OPT_DESIGN.ARGS.DIRECTIVE ExploreWithRemap [get_runs impl_1]
set_property STEPS.PLACE_DESIGN.ARGS.DIRECTIVE ExtraNetDelay_high [get_runs impl_1]
set_property STEPS.ROUTE_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]

# Run synthesis
launch_runs synth_1 -jobs 8
wait_on_run synth_1

# Report utilization after synthesis
open_run synth_1
report_utilization -file ./synth_utilization.rpt

# Run implementation
launch_runs impl_1 -jobs 8
wait_on_run impl_1

# Generate reports
open_run impl_1
report_utilization -file ./impl_utilization.rpt
report_timing_summary -file ./timing_summary.rpt
report_power -file ./power.rpt

# Generate bitstream
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1

puts "Build complete!"
puts "Bitstream: ./vivado_project/${project_name}.runs/impl_1/${top_module}.bit"
