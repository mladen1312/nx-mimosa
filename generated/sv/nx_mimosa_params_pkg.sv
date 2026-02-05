// ═══════════════════════════════════════════════════════════════════════════════
// nx_mimosa_params_pkg.sv - Auto-generated from system_config.yaml
// ═══════════════════════════════════════════════════════════════════════════════
// System: NX-MIMOSA v1.1.0
// Generated: 2026-02-05 01:55:15
// DO NOT EDIT MANUALLY - Regenerate from YAML
// ═══════════════════════════════════════════════════════════════════════════════

`timescale 1ns/1ps

package nx_mimosa_params_pkg;

    // ═══════════════════════════════════════════════════════════════════════════
    // RADAR PARAMETERS
    // ═══════════════════════════════════════════════════════════════════════════
    localparam RADAR_FS = 100000000;
    localparam RADAR_FC = 10000000000;
    localparam RADAR_BANDWIDTH = 10000000;
    localparam RADAR_PRF = 1000;
    localparam RADAR_MAX_RANGE = 150000;

    // ═══════════════════════════════════════════════════════════════════════════
    // FPGA PARAMETERS
    // ═══════════════════════════════════════════════════════════════════════════
    localparam CLOCK_FREQ = 250000000;
    localparam AXI_DATA_WIDTH = 32;
    localparam AXI_ADDR_WIDTH = 32;
    localparam FIFO_DEPTH = 1024;

    // ═══════════════════════════════════════════════════════════════════════════
    // TRACKING PARAMETERS
    // ═══════════════════════════════════════════════════════════════════════════
    localparam UKF_DIM_X = 6;
    localparam UKF_DIM_Z = 3;
    localparam IMM_N_MODES = 3;

    // ═══════════════════════════════════════════════════════════════════════════
    // CFAR PARAMETERS
    // ═══════════════════════════════════════════════════════════════════════════
    localparam CFAR_WINDOW_SIZE = 20;
    localparam CFAR_GUARD_CELLS = 4;

    // ═══════════════════════════════════════════════════════════════════════════
    // COGNITIVE CFAR PARAMETERS
    // ═══════════════════════════════════════════════════════════════════════════
    localparam COGNITIVE_ENABLED = 1;
    localparam SVM_NUM_SV = 64;
    localparam SVM_PROB_THRESH = 16'h7999;
    localparam FEATURE_WIDTH = 16;
    localparam FRAC_BITS = 8;

    // ═══════════════════════════════════════════════════════════════════════════
    // ECCM PARAMETERS
    // ═══════════════════════════════════════════════════════════════════════════
    localparam ECCM_ENABLED = 1;
    localparam FA_N_CHANNELS = 64;
    localparam FA_HOP_INTERVAL = 100;
    localparam JAMMER_POWER_RATIO = 16'h0500;

endpackage
