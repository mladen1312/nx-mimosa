//==============================================================================
// NX-MIMOSA v3.3 — Multi-model IMM Optimal Smoothing Algorithm
// Package Definitions (Dual-Mode Architecture)
//==============================================================================
// [REQ-PKG-33-01] Dual-mode parameters (RT + Window + Offline)
// [REQ-PKG-33-02] Maneuver detection thresholds
// [REQ-PKG-33-03] Window smoother configuration types
// [REQ-PKG-33-04] Extended AXI register map for v3.3
//==============================================================================
// Target: Xilinx RFSoC ZU48DR (Gen 3) @ 250MHz
// Author: Dr. Mladen Mešter / Nexellum d.o.o.
// License: Commercial — Nexellum d.o.o.
//
// SUPPORTED BOARDS:
//   • RFSoC 4x2 (Real Digital)    — $2,499 academic
//   • ZCU208 (AMD/Xilinx)         — $13,194
//==============================================================================

`timescale 1ns/1ps

package nx_mimosa_pkg_v33;

    //==========================================================================
    // TARGET DEVICE: ZU48DR (Gen 3 RFSoC)
    //==========================================================================
    // Resources: 425K LUT, 850K FF, 4272 DSP48E2, 1080 BRAM36, 80 URAM
    // RF-ADC: 8× 14-bit @ 5 GSPS | RF-DAC: 8× 14-bit @ 9.85 GSPS

`ifndef TARGET_ZCU208
    `define TARGET_RFSOC_4X2
`endif

`ifdef TARGET_RFSOC_4X2
    parameter int NUM_ADC_CHANNELS = 4;
    parameter int NUM_DAC_CHANNELS = 2;
    parameter bit HAS_100G_ETH = 1'b1;
    parameter bit HAS_10G_ETH = 1'b1;
    parameter string BOARD_NAME = "RFSoC 4x2";
`else
    parameter int NUM_ADC_CHANNELS = 8;
    parameter int NUM_DAC_CHANNELS = 8;
    parameter bit HAS_100G_ETH = 1'b0;
    parameter bit HAS_10G_ETH = 1'b1;
    parameter string BOARD_NAME = "ZCU208";
`endif

    //--------------------------------------------------------------------------
    // Fixed-Point Format: Q15.16 (32-bit signed)
    //--------------------------------------------------------------------------
    parameter int DATA_WIDTH = 32;
    parameter int FRAC_BITS  = 16;
    parameter int INT_BITS   = DATA_WIDTH - FRAC_BITS - 1;
    parameter int EXT_WIDTH  = 48;
    
    parameter logic signed [DATA_WIDTH-1:0] FP_ONE   = 32'h0001_0000;
    parameter logic signed [DATA_WIDTH-1:0] FP_ZERO  = 32'h0000_0000;
    parameter logic signed [DATA_WIDTH-1:0] FP_HALF  = 32'h0000_8000;
    parameter logic signed [DATA_WIDTH-1:0] FP_EPS   = 32'h0000_0001;
    parameter logic signed [DATA_WIDTH-1:0] FP_TWO   = 32'h0002_0000;
    parameter logic signed [DATA_WIDTH-1:0] FP_NEG1  = 32'hFFFF_0000;
    parameter logic signed [DATA_WIDTH-1:0] FP_PI    = 32'h0003_243F;

    //--------------------------------------------------------------------------
    // Core System Parameters
    //--------------------------------------------------------------------------
    parameter int STATE_DIM   = 4;    // [x, y, vx, vy]
    parameter int MEAS_DIM    = 2;    // [x, y]
    parameter int N_MODELS    = 3;    // CV, CT+, CT-
    parameter int MAX_TARGETS = 8;
    parameter int TARGET_ID_WIDTH = $clog2(MAX_TARGETS);

    //--------------------------------------------------------------------------
    // v3.3 DUAL-MODE PARAMETERS
    // [REQ-PKG-33-01]
    //--------------------------------------------------------------------------
    // Window smoother depth (configurable via AXI-Lite, default 30)
    parameter int WINDOW_DEPTH_DEFAULT = 30;
    parameter int WINDOW_DEPTH_MAX     = 64;   // Max supported window
    parameter int WINDOW_PTR_BITS      = $clog2(WINDOW_DEPTH_MAX);

    // Full-track buffer (URAM-backed for large tracks)
    parameter int TRACK_BUFFER_DEPTH   = 2048;  // Max track length
    parameter int TRACK_PTR_BITS       = $clog2(TRACK_BUFFER_DEPTH);

    // Fixed-lag (legacy, kept for backward compat)
    parameter int LAG_DEPTH = 50;

    // Output stream IDs
    typedef enum logic [1:0] {
        STREAM_REALTIME  = 2'b00,  // Forward IMM, 0 latency
        STREAM_WINDOW    = 2'b01,  // Window-N RTS, ~Ndt latency
        STREAM_OFFLINE   = 2'b10   // Full-track RTS, post-mission
    } stream_id_t;

    //--------------------------------------------------------------------------
    // MANEUVER DETECTION PARAMETERS
    // [REQ-PKG-33-02]
    //--------------------------------------------------------------------------
    // Innovation-based chi-squared threshold (Q15.16)
    parameter logic signed [DATA_WIDTH-1:0] INNOV_THRESHOLD_DEFAULT  = 32'h0004_0000; // 4.0
    // Covariance trace spike threshold
    parameter logic signed [DATA_WIDTH-1:0] COV_TRACE_THRESHOLD      = 32'h0064_0000; // 100.0
    // Mode transition detector: CT→CV probability delta
    parameter logic signed [DATA_WIDTH-1:0] MODE_TRANS_THRESHOLD     = 32'h0000_4CCD; // 0.3
    // Minimum consecutive CV steps before declaring maneuver end
    parameter int MANEUVER_END_COUNT = 5;

    typedef enum logic [2:0] {
        MNV_IDLE       = 3'b000,  // No maneuver detected
        MNV_ONSET      = 3'b001,  // Maneuver just detected
        MNV_SUSTAINED  = 3'b010,  // In sustained maneuver
        MNV_ENDING     = 3'b011,  // Transitioning back to CV
        MNV_ENDED      = 3'b100   // Maneuver ended — trigger smooth
    } maneuver_state_t;

    //--------------------------------------------------------------------------
    // WINDOW SMOOTHER TRIGGER MODES
    // [REQ-PKG-33-03]
    //--------------------------------------------------------------------------
    typedef enum logic [1:0] {
        TRIG_SLIDING    = 2'b00,  // Continuous: smooth last N on every step
        TRIG_MANEUVER   = 2'b01,  // Trigger on maneuver end
        TRIG_COV_SPIKE  = 2'b10,  // Trigger on covariance spike
        TRIG_MANUAL     = 2'b11   // Software trigger via AXI-Lite
    } smooth_trigger_t;

    //--------------------------------------------------------------------------
    // RF-ADC Parameters (Gen 3)
    //--------------------------------------------------------------------------
    parameter int ADC_BITS = 14;
    parameter int ADC_RATE_GSPS = 5;
    parameter int ADC_BW_GHZ = 6;
    parameter int DAC_RATE_GSPS = 10;

    //--------------------------------------------------------------------------
    // Timing
    //--------------------------------------------------------------------------
    parameter int CLK_FREQ_MHZ = 250;
    parameter logic signed [DATA_WIDTH-1:0] DT_100MS = 32'h0000_199A;
    parameter logic signed [DATA_WIDTH-1:0] DT_50MS  = 32'h0000_0CCD;
    parameter logic signed [DATA_WIDTH-1:0] DT_10MS  = 32'h0000_028F;

    //--------------------------------------------------------------------------
    // Types
    //--------------------------------------------------------------------------
    typedef logic signed [DATA_WIDTH-1:0] fp_t;
    typedef logic signed [EXT_WIDTH-1:0] fp_ext_t;
    
    typedef fp_t state_vec_t [STATE_DIM];
    typedef fp_t meas_vec_t [MEAS_DIM];
    typedef fp_t state_mat_t [STATE_DIM][STATE_DIM];
    typedef fp_t meas_mat_t [MEAS_DIM][STATE_DIM];
    typedef fp_t innov_cov_t [MEAS_DIM][MEAS_DIM];
    typedef fp_t kalman_gain_t [STATE_DIM][MEAS_DIM];
    typedef fp_t mode_prob_t [N_MODELS];
    
    typedef fp_t target_state_t [MAX_TARGETS][STATE_DIM];
    typedef logic [TARGET_ID_WIDTH-1:0] target_id_t;
    typedef logic [MAX_TARGETS-1:0] target_mask_t;

    //--------------------------------------------------------------------------
    // v3.3 Extended Register Map
    // [REQ-PKG-33-04]
    //--------------------------------------------------------------------------
    // 0x00: Control (enable, smoother_enable, reset, window_enable, offline_enable)
    // 0x04: Omega
    // 0x08: dt
    // 0x0C: q_cv
    // 0x10: q_ct
    // 0x14: r (measurement noise)
    // 0x18: p_stay
    // 0x1C: Status (read-only)
    // 0x20-0x2C: x_init[0..3]
    // 0x30-0x3C: P_init (diagonal)
    // --- v3.3 NEW registers ---
    // 0x40: Window size (1..WINDOW_DEPTH_MAX)
    // 0x44: Trigger mode (smooth_trigger_t)
    // 0x48: Innovation threshold (Q15.16)
    // 0x4C: Cov trace threshold (Q15.16)
    // 0x50: Manual trigger (write 1 to trigger)
    // 0x54: Window output count (read-only)
    // 0x58: Maneuver state (read-only)
    // 0x5C: Performance counters — forward cycles (read-only)
    // 0x60: Performance counters — smooth cycles (read-only)
    // 0x64: Track buffer fill level (read-only)
    // 0x68: v3.3 version register (read-only, returns 0x00030003)

    parameter int AXI_ADDR_CONTROL        = 8'h00;
    parameter int AXI_ADDR_OMEGA          = 8'h04;
    parameter int AXI_ADDR_DT             = 8'h08;
    parameter int AXI_ADDR_Q_CV           = 8'h0C;
    parameter int AXI_ADDR_Q_CT           = 8'h10;
    parameter int AXI_ADDR_R              = 8'h14;
    parameter int AXI_ADDR_P_STAY         = 8'h18;
    parameter int AXI_ADDR_STATUS         = 8'h1C;
    parameter int AXI_ADDR_X_INIT_BASE    = 8'h20;
    parameter int AXI_ADDR_P_INIT_BASE    = 8'h30;
    // v3.3
    parameter int AXI_ADDR_WINDOW_SIZE    = 8'h40;
    parameter int AXI_ADDR_TRIGGER_MODE   = 8'h44;
    parameter int AXI_ADDR_INNOV_THRESH   = 8'h48;
    parameter int AXI_ADDR_COV_THRESH     = 8'h4C;
    parameter int AXI_ADDR_MANUAL_TRIG    = 8'h50;
    parameter int AXI_ADDR_WINDOW_COUNT   = 8'h54;
    parameter int AXI_ADDR_MNV_STATE      = 8'h58;
    parameter int AXI_ADDR_FWD_CYCLES     = 8'h5C;
    parameter int AXI_ADDR_SMOOTH_CYCLES  = 8'h60;
    parameter int AXI_ADDR_TRACK_FILL     = 8'h64;
    parameter int AXI_ADDR_VERSION        = 8'h68;

    parameter logic [31:0] VERSION_V33    = 32'h0003_0003;

    //--------------------------------------------------------------------------
    // AXI-Stream Widths
    //--------------------------------------------------------------------------
    parameter int AXIS_MEAS_WIDTH = 64;
    parameter int AXIS_STATE_WIDTH = 128;

    //--------------------------------------------------------------------------
    // Utility Functions
    //--------------------------------------------------------------------------
    
    function automatic fp_t fp_mul(input fp_t a, input fp_t b);
        logic signed [2*DATA_WIDTH-1:0] prod;
        prod = a * b;
        return prod[DATA_WIDTH+FRAC_BITS-1:FRAC_BITS];
    endfunction
    
    function automatic fp_t fp_div(input fp_t a, input fp_t b);
        logic signed [2*DATA_WIDTH-1:0] num;
        num = {{DATA_WIDTH{a[DATA_WIDTH-1]}}, a} << FRAC_BITS;
        return (b != 0) ? num / b : FP_ONE;
    endfunction
    
    function automatic fp_t fp_abs(input fp_t val);
        return (val < 0) ? -val : val;
    endfunction
    
    function automatic fp_t fp_saturate(input fp_ext_t val);
        if (val > $signed({{(EXT_WIDTH-DATA_WIDTH){1'b0}}, {1'b0, {(DATA_WIDTH-1){1'b1}}}}))
            return {1'b0, {(DATA_WIDTH-1){1'b1}}};
        else if (val < $signed({{(EXT_WIDTH-DATA_WIDTH){1'b1}}, {1'b1, {(DATA_WIDTH-1){1'b0}}}}))
            return {1'b1, {(DATA_WIDTH-1){1'b0}}};
        else
            return val[DATA_WIDTH-1:0];
    endfunction

    // v3.3: Trace of 4x4 matrix
    function automatic fp_t fp_trace4(input fp_t M [STATE_DIM][STATE_DIM]);
        return M[0][0] + M[1][1] + M[2][2] + M[3][3];
    endfunction

    // v3.3: Normalized innovation squared (NIS) for 2D measurement
    // NIS = nu' * S^-1 * nu (simplified for diagonal S)
    function automatic fp_t fp_nis_diag(
        input fp_t nu [MEAS_DIM],
        input fp_t S_diag [MEAS_DIM]
    );
        fp_t sum;
        sum = FP_ZERO;
        for (int i = 0; i < MEAS_DIM; i++) begin
            sum = sum + fp_div(fp_mul(nu[i], nu[i]), S_diag[i] + FP_EPS);
        end
        return sum;
    endfunction

endpackage
