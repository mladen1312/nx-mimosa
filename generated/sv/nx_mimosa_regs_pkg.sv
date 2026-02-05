// ═══════════════════════════════════════════════════════════════════════════════
// nx_mimosa_regs_pkg.sv - Auto-generated from system_config.yaml
// ═══════════════════════════════════════════════════════════════════════════════
// System: NX-MIMOSA v1.1.0
// Generated: 2026-02-05 01:55:15
// DO NOT EDIT MANUALLY - Regenerate from YAML
// ═══════════════════════════════════════════════════════════════════════════════

`timescale 1ns/1ps

package nx_mimosa_regs_pkg;

    // Base address
    localparam logic [31:0] BASE_ADDR = 32'h80000000;

    // Register offsets
    localparam logic [31:0] REG_CONTROL_OFFSET = 32'h00000000;
    localparam logic [31:0] REG_STATUS_OFFSET = 32'h00000004;
    localparam logic [31:0] REG_CFAR_CONFIG_OFFSET = 32'h00000008;
    localparam logic [31:0] REG_TRACK_COUNT_OFFSET = 32'h0000000C;
    localparam logic [31:0] REG_DETECTION_COUNT_OFFSET = 32'h00000010;
    localparam logic [31:0] REG_VERSION_OFFSET = 32'h000000FC;

    // Field definitions

    // REG_CONTROL fields
    localparam REG_CONTROL_ENABLE_HI = 0;
    localparam REG_CONTROL_ENABLE_LO = 0;
    localparam REG_CONTROL_RESET_HI = 1;
    localparam REG_CONTROL_RESET_LO = 1;
    localparam REG_CONTROL_MODE_HI = 3;
    localparam REG_CONTROL_MODE_LO = 2;

    // REG_STATUS fields
    localparam REG_STATUS_READY_HI = 0;
    localparam REG_STATUS_READY_LO = 0;
    localparam REG_STATUS_OVERFLOW_HI = 1;
    localparam REG_STATUS_OVERFLOW_LO = 1;
    localparam REG_STATUS_TRACK_COUNT_HI = 15;
    localparam REG_STATUS_TRACK_COUNT_LO = 8;

    // REG_CFAR_CONFIG fields
    localparam REG_CFAR_CONFIG_THRESHOLD_HI = 15;
    localparam REG_CFAR_CONFIG_THRESHOLD_LO = 0;
    localparam REG_CFAR_CONFIG_WINDOW_SIZE_HI = 23;
    localparam REG_CFAR_CONFIG_WINDOW_SIZE_LO = 16;
    localparam REG_CFAR_CONFIG_GUARD_CELLS_HI = 27;
    localparam REG_CFAR_CONFIG_GUARD_CELLS_LO = 24;

    // REG_TRACK_COUNT fields
    localparam REG_TRACK_COUNT_COUNT_HI = 15;
    localparam REG_TRACK_COUNT_COUNT_LO = 0;

    // REG_DETECTION_COUNT fields
    localparam REG_DETECTION_COUNT_COUNT_HI = 31;
    localparam REG_DETECTION_COUNT_COUNT_LO = 0;

    // REG_VERSION fields
    localparam REG_VERSION_MINOR_HI = 7;
    localparam REG_VERSION_MINOR_LO = 0;
    localparam REG_VERSION_MAJOR_HI = 15;
    localparam REG_VERSION_MAJOR_LO = 8;
    localparam REG_VERSION_REVISION_HI = 23;
    localparam REG_VERSION_REVISION_LO = 16;

endpackage
