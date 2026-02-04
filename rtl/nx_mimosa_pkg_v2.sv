//==============================================================================
// NX-MIMOSA v2.0 — Enhanced Package with Adaptive Features
// [REQ-V2-PKG-01] Extended state dimension support (4D/6D)
// [REQ-V2-PKG-02] UKF/CKF sigma point parameters
// [REQ-V2-PKG-03] Adaptive Q configuration
//==============================================================================
// Target: Xilinx RFSoC ZU48DR (Gen 3) @ 250MHz
// Author: Dr. Mladen Mešter / Nexellum d.o.o.
// License: Commercial
//==============================================================================

`timescale 1ns/1ps

package nx_mimosa_pkg_v2;

    //==========================================================================
    // Fixed-Point Format: Q15.16 (32-bit signed)
    //==========================================================================
    parameter int DATA_WIDTH = 32;
    parameter int FRAC_BITS  = 16;
    parameter int INT_BITS   = DATA_WIDTH - FRAC_BITS - 1;
    parameter int EXT_WIDTH  = 48;
    
    // Fixed-point constants
    parameter logic signed [DATA_WIDTH-1:0] FP_ONE   = 32'h0001_0000;
    parameter logic signed [DATA_WIDTH-1:0] FP_ZERO  = 32'h0000_0000;
    parameter logic signed [DATA_WIDTH-1:0] FP_HALF  = 32'h0000_8000;
    parameter logic signed [DATA_WIDTH-1:0] FP_TWO   = 32'h0002_0000;
    parameter logic signed [DATA_WIDTH-1:0] FP_PI    = 32'h0003_243F;
    
    // Adaptive Q constants
    parameter logic signed [DATA_WIDTH-1:0] FP_0_95  = 32'h0000_F333;  // 0.95
    parameter logic signed [DATA_WIDTH-1:0] FP_0_99  = 32'h0000_FD70;  // 0.99
    parameter logic signed [DATA_WIDTH-1:0] FP_1_5   = 32'h0001_8000;  // 1.5
    parameter logic signed [DATA_WIDTH-1:0] FP_2_0   = 32'h0002_0000;  // 2.0
    
    //==========================================================================
    // Configurable State Dimension
    //==========================================================================
`ifdef STATE_6D
    parameter int STATE_DIM   = 6;    // [x, y, vx, vy, ax, ay]
`else
    parameter int STATE_DIM   = 4;    // [x, y, vx, vy]
`endif

    parameter int MEAS_DIM    = 2;    // [x, y] or [r, θ]
    
    //==========================================================================
    // Model Configuration
    //==========================================================================
`ifdef MODELS_4
    parameter int N_MODELS    = 4;    // CV, CA, CT+, CT-
`else
    parameter int N_MODELS    = 3;    // CV, CT+, CT-
`endif

    parameter int LAG_DEPTH   = 50;
    parameter int MAX_TARGETS = 8;
    
    //==========================================================================
    // UKF/CKF Parameters
    //==========================================================================
    // UKF: alpha, beta, kappa
    parameter logic signed [DATA_WIDTH-1:0] UKF_ALPHA = 32'h0000_0148;  // 0.001
    parameter logic signed [DATA_WIDTH-1:0] UKF_BETA  = 32'h0002_0000;  // 2.0
    parameter logic signed [DATA_WIDTH-1:0] UKF_KAPPA = 32'h0000_0000;  // 0
    
    // Number of sigma points
    parameter int N_SIGMA_UKF = 2 * STATE_DIM + 1;  // 2n+1 = 9 or 13
    parameter int N_SIGMA_CKF = 2 * STATE_DIM;      // 2n = 8 or 12
    
    //==========================================================================
    // Adaptive Q Thresholds
    //==========================================================================
    // Chi-squared threshold for MEAS_DIM=2, α=0.05 → ~5.99
    parameter logic signed [DATA_WIDTH-1:0] CHI2_THRESHOLD = 32'h0005_FD70;  // ~6.0
    
    // Q scaling factors
    parameter logic signed [DATA_WIDTH-1:0] Q_INCREASE_FACTOR = 32'h0001_8000;  // 1.5x
    parameter logic signed [DATA_WIDTH-1:0] Q_DECREASE_FACTOR = 32'h0000_F333;  // 0.95x
    parameter logic signed [DATA_WIDTH-1:0] Q_MAX_SCALE = 32'h0005_0000;        // 5.0x
    parameter logic signed [DATA_WIDTH-1:0] Q_MIN_SCALE = 32'h0000_199A;        // 0.1x
    
    //==========================================================================
    // Dynamic TPM Thresholds
    //==========================================================================
    parameter logic signed [DATA_WIDTH-1:0] MU_HIGH_CONF = 32'h0000_E666;  // 0.9
    parameter logic signed [DATA_WIDTH-1:0] P_STAY_HIGH  = 32'h0000_FD70;  // 0.99
    parameter logic signed [DATA_WIDTH-1:0] P_STAY_MED   = 32'h0000_E666;  // 0.9
    parameter logic signed [DATA_WIDTH-1:0] P_STAY_LOW   = 32'h0000_CCCD;  // 0.8
    
    //==========================================================================
    // Types
    //==========================================================================
    typedef logic signed [DATA_WIDTH-1:0] fp_t;
    typedef logic signed [EXT_WIDTH-1:0] fp_ext_t;
    
    typedef fp_t state_vec_t [STATE_DIM];
    typedef fp_t meas_vec_t [MEAS_DIM];
    typedef fp_t state_mat_t [STATE_DIM][STATE_DIM];
    typedef fp_t meas_mat_t [MEAS_DIM][STATE_DIM];
    typedef fp_t innov_cov_t [MEAS_DIM][MEAS_DIM];
    typedef fp_t mode_prob_t [N_MODELS];
    
    // UKF sigma points
    typedef fp_t sigma_points_t [N_SIGMA_UKF][STATE_DIM];
    typedef fp_t sigma_weights_t [N_SIGMA_UKF];
    
    // Filter type enumeration
    typedef enum logic [1:0] {
        FILTER_EKF = 2'b00,
        FILTER_UKF = 2'b01,
        FILTER_CKF = 2'b10
    } filter_type_t;
    
    //==========================================================================
    // Utility Functions
    //==========================================================================
    
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
    
    function automatic fp_t fp_sqrt_approx(input fp_t val);
        // Newton-Raphson approximation for sqrt
        // Good enough for sigma point generation
        fp_t x, x_new;
        x = val >>> 1;  // Initial guess: val/2
        for (int i = 0; i < 4; i++) begin
            x_new = (x + fp_div(val, x)) >>> 1;
            x = x_new;
        end
        return x;
    endfunction
    
    function automatic fp_t fp_clamp(input fp_t val, input fp_t min_val, input fp_t max_val);
        if (val < min_val) return min_val;
        else if (val > max_val) return max_val;
        else return val;
    endfunction

endpackage
