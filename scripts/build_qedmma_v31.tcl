#==============================================================================
# QEDMMA v3.1 Vivado Build Script
# Target: Xilinx RFSoC ZU28DR
#==============================================================================

# Project setup
set project_name "qedmma_v31"
set part_name "xczu28dr-ffvg1517-2-e"
set top_module "qedmma_v31_top"

# Create project
create_project ${project_name} ./${project_name} -part ${part_name} -force

# Add RTL sources
add_files -norecurse [glob ../rtl/*.sv]
set_property file_type SystemVerilog [get_files *.sv]

# Add constraints
# add_files -fileset constrs_1 ../constraints/timing.xdc

# Synthesis settings
set_property strategy Flow_PerfOptimized_high [get_runs synth_1]
set_property STEPS.SYNTH_DESIGN.ARGS.FLATTEN_HIERARCHY rebuilt [get_runs synth_1]
set_property STEPS.SYNTH_DESIGN.ARGS.RETIMING on [get_runs synth_1]

# Implementation settings
set_property strategy Performance_ExtraTimingOpt [get_runs impl_1]

# Run synthesis
launch_runs synth_1 -jobs 8
wait_on_run synth_1

# Check utilization
open_run synth_1
report_utilization -file qedmma_v31_utilization.rpt
report_timing_summary -file qedmma_v31_timing.rpt

# Run implementation
launch_runs impl_1 -jobs 8
wait_on_run impl_1

# Generate bitstream
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1

puts "Build complete!"
