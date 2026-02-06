//==============================================================================
// NX-MIMOSA v4.0.2 SENTINEL — Package Definitions
//==============================================================================
// Multi-Pipeline Architecture: Forward IMM + Fixed-Lag RTS + Hybrid Selector
// Target: Xilinx RFSoC ZU48DR (Gen 3) @ 250MHz
// Author: Dr. Mladen Mešter / Nexellum d.o.o.
// [REQ-PKG-001] All data types for 6-model IMM with dual state dimensions
//==============================================================================

`timescale 1ns/1ps

package nx_mimosa_v40_pkg;

    //==========================================================================
    // Fixed-Point Format: Q15.16 (32-bit signed)
    //==========================================================================
    parameter int DW       = 32;       // Data width
    parameter int FW       = 16;       // Fractional bits
    parameter int IW       = DW-FW-1;  // Integer bits (15)
    parameter int EW       = 48;       // Extended width for accumulation
    
    // Core constants
    parameter logic signed [DW-1:0] FP_ONE   = 32'h0001_0000;
    parameter logic signed [DW-1:0] FP_ZERO  = 32'h0000_0000;
    parameter logic signed [DW-1:0] FP_HALF  = 32'h0000_8000;
    parameter logic signed [DW-1:0] FP_EPS   = 32'h0000_0001;
    parameter logic signed [DW-1:0] FP_TWO   = 32'h0002_0000;

    //==========================================================================
    // [REQ-SYS-001] System Dimensions — v4.0.2 SENTINEL
    //==========================================================================
    // CV/CT models:  state_dim=4 [x, y, vx, vy]
    // CA/Jerk/Ball:  state_dim=6 [x, y, vx, vy, ax, ay]
    // Measurement:   meas_dim=2  [x, y]
    //
    // To avoid dual-dimension complexity in RTL, we use unified state_dim=6
    // with CV/CT models having ax=ay=0 (and zero-padded F/Q rows/cols).
    //==========================================================================
    parameter int STATE_DIM    = 6;    // Unified: [x, y, vx, vy, ax, ay]
    parameter int MEAS_DIM     = 2;    // [x, y]
    parameter int N_MODELS     = 6;    // CV, CA, CT+, CT-, Jerk, Ballistic
    parameter int N_MODELS_LOG = 3;    // ceil(log2(N_MODELS))
    
    // Model indices
    parameter int MDL_CV    = 0;
    parameter int MDL_CA    = 1;
    parameter int MDL_CTP   = 2;   // CT+ (positive turn)
    parameter int MDL_CTN   = 3;   // CT- (negative turn)
    parameter int MDL_JERK  = 4;
    parameter int MDL_BALL  = 5;
    
    //==========================================================================
    // [REQ-SYS-002] Multi-Target & Timing
    //==========================================================================
    parameter int MAX_TARGETS     = 8;
    parameter int TGT_ID_W        = $clog2(MAX_TARGETS);
    parameter int CLK_FREQ_MHZ    = 250;
    
    //==========================================================================
    // [REQ-SYS-003] Smoother Parameters
    //==========================================================================
    parameter int RTS_LAG_DEPTH   = 40;    // Fixed-lag window (steps)
    parameter int HYBRID_LAG      = 15;    // Hybrid selector lookback
    parameter int NIS_WINDOW      = 20;    // NIS averaging window
    
    //==========================================================================
    // [REQ-SYS-004] Multi-Pipeline Stream IDs
    //==========================================================================
    parameter int STREAM_FWD     = 0;   // Forward IMM (real-time display)
    parameter int STREAM_HYB     = 1;   // Hybrid best (fire control, 1.5s lag)
    parameter int STREAM_CVR     = 2;   // CV-RTS (post-mission)
    parameter int STREAM_CAR     = 3;   // CA-RTS (post-mission)
    parameter int N_STREAMS      = 4;
    parameter int STREAM_ID_W    = 2;
    
    //==========================================================================
    // Types
    //==========================================================================
    typedef logic signed [DW-1:0]  fp_t;
    typedef logic signed [EW-1:0]  fp_ext_t;
    
    // Vector/Matrix types for unified 6-state
    typedef fp_t  state_vec_t  [STATE_DIM];
    typedef fp_t  meas_vec_t   [MEAS_DIM];
    typedef fp_t  state_mat_t  [STATE_DIM][STATE_DIM];
    typedef fp_t  meas_mat_t   [MEAS_DIM][STATE_DIM];   // H matrix
    typedef fp_t  innov_cov_t  [MEAS_DIM][MEAS_DIM];    // S = HPH'+R
    typedef fp_t  kalman_gain_t[STATE_DIM][MEAS_DIM];    // K
    typedef fp_t  mode_prob_t  [N_MODELS];
    
    // Per-model state storage
    typedef struct packed {
        logic signed [DW-1:0] x  [STATE_DIM];     // State vector
        logic signed [DW-1:0] P  [STATE_DIM][STATE_DIM]; // Covariance
        logic signed [DW-1:0] mu;                  // Mode probability
        logic signed [DW-1:0] nis;                 // Normalized Innovation Squared
        logic                 valid;
    } model_state_t;
    
    // IMM output bundle
    typedef struct packed {
        logic signed [DW-1:0] x_mixed [STATE_DIM]; // Mixed estimate
        logic signed [DW-1:0] mu      [N_MODELS];  // Mode probabilities
        logic signed [DW-1:0] nis_cv;               // CV model NIS
        logic signed [DW-1:0] nis_ca;               // CA model NIS
        logic signed [DW-1:0] omega;                 // Turn rate estimate
        logic [TGT_ID_W-1:0]  tgt_id;
        logic                  valid;
    } imm_output_t;
    
    // RTS smoother output
    typedef struct packed {
        logic signed [DW-1:0] x_smooth [STATE_DIM];
        logic signed [DW-1:0] P_smooth [STATE_DIM][STATE_DIM];
        logic [TGT_ID_W-1:0]  tgt_id;
        logic [STREAM_ID_W-1:0] stream_id;
        logic                  valid;
    } rts_output_t;
    
    // Multi-stream output (to AXI-Stream)
    typedef struct packed {
        logic signed [DW-1:0] x_out [STATE_DIM];
        logic [TGT_ID_W-1:0]  tgt_id;
        logic [STREAM_ID_W-1:0] stream_id;
        logic signed [DW-1:0] quality;   // NIS or smoothness metric
        logic                  valid;
    } stream_output_t;
    
    // Platform classifier features (from IMM dynamics)
    typedef struct packed {
        logic signed [DW-1:0] spd_avg;
        logic signed [DW-1:0] omega_peak;
        logic signed [DW-1:0] omega_avg;
        logic signed [DW-1:0] nis_cv_avg;
        logic signed [DW-1:0] nis_cv_peak;
        logic signed [DW-1:0] mu_ct_peak;
        logic signed [DW-1:0] mu_ca_peak;
        logic                  is_maneuvering;
        logic                  is_high_dynamics;
    } classifier_features_t;
    
    //==========================================================================
    // [REQ-SYS-005] Transition Probability Matrix (6×6) — Q15.16
    //==========================================================================
    // Default TPM for moderate maneuvering scenario
    // Rows sum to 1.0 (FP_ONE = 0x00010000)
    //
    //         CV      CA      CT+     CT-     Jerk    Ball
    // CV    [0.75   0.05    0.07    0.07    0.03    0.03]
    // CA    [0.10   0.60    0.10    0.10    0.05    0.05]
    // CT+   [0.10   0.05    0.70    0.05    0.05    0.05]
    // CT-   [0.10   0.05    0.05    0.70    0.05    0.05]
    // Jerk  [0.10   0.10    0.10    0.10    0.50    0.10]
    // Ball  [0.15   0.05    0.05    0.05    0.05    0.65]
    //==========================================================================
    parameter fp_t DEFAULT_TPM [N_MODELS][N_MODELS] = '{
        '{32'h0000_C000, 32'h0000_0CCD, 32'h0000_11EB, 32'h0000_11EB, 32'h0000_07AE, 32'h0000_07AE},
        '{32'h0000_199A, 32'h0000_999A, 32'h0000_199A, 32'h0000_199A, 32'h0000_0CCD, 32'h0000_0CCD},
        '{32'h0000_199A, 32'h0000_0CCD, 32'h0000_B333, 32'h0000_0CCD, 32'h0000_0CCD, 32'h0000_0CCD},
        '{32'h0000_199A, 32'h0000_0CCD, 32'h0000_0CCD, 32'h0000_B333, 32'h0000_0CCD, 32'h0000_0CCD},
        '{32'h0000_199A, 32'h0000_199A, 32'h0000_199A, 32'h0000_199A, 32'h0000_8000, 32'h0000_199A},
        '{32'h0000_2666, 32'h0000_0CCD, 32'h0000_0CCD, 32'h0000_0CCD, 32'h0000_0CCD, 32'h0000_A666}
    };
    
    //==========================================================================
    // AXI-Lite Register Map Offsets (v4.0.2)
    //==========================================================================
    parameter int REG_CTRL          = 12'h000;  // Control: [0]=enable, [1]=reset
    parameter int REG_STATUS        = 12'h004;  // Status: [0]=ready, [7:4]=active_tgts
    parameter int REG_DT            = 12'h008;  // Sample period (Q15.16)
    parameter int REG_R_STD         = 12'h00C;  // Measurement noise std
    parameter int REG_STREAM_SEL    = 12'h010;  // Active output stream
    parameter int REG_TGT_MASK      = 12'h014;  // Active target bitmask
    parameter int REG_TPM_BASE      = 12'h100;  // TPM[i][j] = 0x100 + (i*24 + j*4)
    parameter int REG_PLATFORM_BASE = 12'h200;  // Platform DB registers
    parameter int REG_STATS_BASE    = 12'h300;  // Runtime statistics
    
    //==========================================================================
    // Utility Functions
    //==========================================================================
    
    // Q15.16 multiply
    function automatic fp_t fp_mul(input fp_t a, input fp_t b);
        logic signed [2*DW-1:0] prod;
        prod = a * b;
        return prod[DW+FW-1:FW];
    endfunction
    
    // Q15.16 divide
    function automatic fp_t fp_div(input fp_t a, input fp_t b);
        logic signed [2*DW-1:0] num;
        num = {{DW{a[DW-1]}}, a} << FW;
        return (b != 0) ? fp_t'(num / b) : FP_ONE;
    endfunction
    
    // Absolute value
    function automatic fp_t fp_abs(input fp_t val);
        return (val < 0) ? -val : val;
    endfunction
    
    // Saturate extended to DW
    function automatic fp_t fp_sat(input fp_ext_t val);
        localparam fp_ext_t POS_MAX = {{(EW-DW){1'b0}}, {1'b0, {(DW-1){1'b1}}}};
        localparam fp_ext_t NEG_MIN = {{(EW-DW){1'b1}}, {1'b1, {(DW-1){1'b0}}}};
        if (val > POS_MAX) return {1'b0, {(DW-1){1'b1}}};
        else if (val < NEG_MIN) return {1'b1, {(DW-1){1'b0}}};
        else return val[DW-1:0];
    endfunction
    
    // Clamp mode probability [0, FP_ONE]
    function automatic fp_t mu_clamp(input fp_t val);
        if (val < FP_ZERO) return FP_EPS;
        else if (val > FP_ONE) return FP_ONE;
        else return val;
    endfunction

endpackage
