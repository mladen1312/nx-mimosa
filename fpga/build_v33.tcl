# ==============================================================================
# NX-MIMOSA v3.3 — Vivado Build Script (Forge Automated)
# [REQ-BUILD-01] Dual-board: RFSoC 4x2 + ZCU208
# [REQ-BUILD-02] v3.3 dual-mode top with three output streams
# [REQ-BUILD-03] Timing closure @ 250MHz
# ==============================================================================
# Usage:
#   vivado -mode batch -source build_v33.tcl -tclargs rfsoc4x2
#   vivado -mode batch -source build_v33.tcl -tclargs zcu208
# ==============================================================================
# Author: Dr. Mladen Mešter / Nexellum d.o.o.
# ==============================================================================

# ---- Board Selection ----
set board [lindex $argv 0]
if {$board eq ""} { set board "rfsoc4x2" }

puts "========================================="
puts " NX-MIMOSA v3.3 Dual-Mode Build"
puts " Board: $board"
puts "========================================="

# ---- Part & Define ----
switch $board {
    "rfsoc4x2" {
        set part     "xczu48dr-ffvg1517-2-e"
        set defines  ""
        set xdc_file "../constraints/rfsoc4x2_v33.xdc"
        set proj_name "nx_mimosa_v33_rfsoc4x2"
    }
    "zcu208" {
        set part     "xczu48dr-fsvg1517-2-e"
        set defines  "TARGET_ZCU208"
        set xdc_file "../constraints/zcu208_v33.xdc"
        set proj_name "nx_mimosa_v33_zcu208"
    }
    default {
        puts "ERROR: Unknown board '$board'. Use rfsoc4x2 or zcu208."
        exit 1
    }
}

# ---- Project ----
create_project $proj_name ./build_${board} -part $part -force

# ---- Source Files ----
set rtl_dir "../rtl"
set rtl_files [list \
    "${rtl_dir}/nx_mimosa_pkg_v33.sv" \
    "${rtl_dir}/nx_mimosa_v33_top.sv" \
    "${rtl_dir}/imm_core.sv" \
    "${rtl_dir}/kalman_filter_core.sv" \
    "${rtl_dir}/maneuver_detector.sv" \
    "${rtl_dir}/window_rts_smoother.sv" \
    "${rtl_dir}/fixed_lag_smoother.sv" \
    "${rtl_dir}/matrix_multiply_4x4.sv" \
    "${rtl_dir}/matrix_inverse_4x4.sv" \
    "${rtl_dir}/matrix_vector_mult.sv" \
    "${rtl_dir}/sincos_lut.sv" \
    "${rtl_dir}/nx_mimosa_axi_wrapper.sv" \
]

add_files -fileset sources_1 $rtl_files
set_property file_type SystemVerilog [get_files *.sv]
set_property top nx_mimosa_v33_top [current_fileset]

# ---- Defines ----
if {$defines ne ""} {
    set_property verilog_define $defines [current_fileset]
    puts "INFO: Set define: $defines"
}

# ---- Constraints ----
if {[file exists $xdc_file]} {
    add_files -fileset constrs_1 $xdc_file
} else {
    puts "WARNING: Constraint file $xdc_file not found. Using default timing."
    # Create minimal timing constraint
    set xdc_tmp "./build_${board}/timing_v33.xdc"
    set fp [open $xdc_tmp w]
    puts $fp "# NX-MIMOSA v3.3 Timing Constraints"
    puts $fp "create_clock -period 4.000 -name aclk \[get_ports aclk\]"
    puts $fp ""
    puts $fp "# AXI-Stream I/O"
    puts $fp "set_input_delay -clock aclk -max 1.5 \[get_ports s_axis_meas_*\]"
    puts $fp "set_input_delay -clock aclk -min 0.5 \[get_ports s_axis_meas_*\]"
    puts $fp "set_output_delay -clock aclk -max 1.5 \[get_ports m_axis_*\]"
    puts $fp "set_output_delay -clock aclk -min 0.5 \[get_ports m_axis_*\]"
    puts $fp ""
    puts $fp "# AXI-Lite"
    puts $fp "set_input_delay -clock aclk -max 2.0 \[get_ports s_axi_*\]"
    puts $fp "set_input_delay -clock aclk -min 0.5 \[get_ports s_axi_*\]"
    puts $fp "set_output_delay -clock aclk -max 2.0 \[get_ports s_axi_*\]"
    puts $fp "set_output_delay -clock aclk -min 0.5 \[get_ports s_axi_*\]"
    close $fp
    add_files -fileset constrs_1 $xdc_tmp
}

# ---- Synthesis ----
puts "INFO: Running Synthesis..."
set synth_opts [list \
    -flatten_hierarchy rebuilt \
    -retiming on \
    -directive AreaOptimized_high \
    -fsm_extraction one_hot \
]
launch_runs synth_1 -jobs 8
wait_on_run synth_1

# ---- Check Synthesis Results ----
open_run synth_1
set util_rpt [report_utilization -return_string]
puts $util_rpt
report_utilization -file ./build_${board}/synth_utilization_v33.rpt

set timing_rpt [report_timing_summary -return_string -max_paths 10]
puts $timing_rpt
report_timing_summary -file ./build_${board}/synth_timing_v33.rpt

# ---- Implementation ----
puts "INFO: Running Implementation..."
set_property strategy Performance_ExplorePostRoutePhysOpt [get_runs impl_1]
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1

# ---- Reports ----
open_run impl_1
report_utilization -file ./build_${board}/impl_utilization_v33.rpt
report_timing_summary -file ./build_${board}/impl_timing_v33.rpt
report_power -file ./build_${board}/impl_power_v33.rpt

# ---- Check Timing ----
set wns [get_property STATS.WNS [get_runs impl_1]]
set tns [get_property STATS.TNS [get_runs impl_1]]
puts "========================================="
puts " TIMING: WNS = ${wns}ns, TNS = ${tns}ns"
if {$wns >= 0} {
    puts " STATUS: TIMING MET ✅"
} else {
    puts " STATUS: TIMING VIOLATION ❌"
}
puts "========================================="

# ---- Bitstream ----
set bit_file "./build_${board}/${proj_name}.runs/impl_1/nx_mimosa_v33_top.bit"
if {[file exists $bit_file]} {
    file copy -force $bit_file ./build_${board}/nx_mimosa_v33_${board}.bit
    puts "Bitstream: ./build_${board}/nx_mimosa_v33_${board}.bit"
}

puts ""
puts "BUILD COMPLETE: NX-MIMOSA v3.3 for $board"
puts "========================================="

exit 0
