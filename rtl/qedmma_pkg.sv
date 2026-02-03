//==============================================================================
// QEDMMA v3.1 Pro — Package Definitions
// [REQ-RTL-PKG-01] Fixed-point parameters and types
// [REQ-RTL-PKG-02] Target: Xilinx RFSoC ZU48DR (RFSoC 4x2) @ 250MHz
//==============================================================================
// Author: Dr. Mladen Mešter / Nexellum d.o.o.
// License: Commercial (qedmma-pro)
// Target: RFSoC 4x2 (XCZU48DR-2FFVG1517E) — Gen 3 RFSoC
//==============================================================================

`timescale 1ns/1ps

package qedmma_pkg;

    //--------------------------------------------------------------------------
    // Target Device: ZU48DR (RFSoC 4x2)
    //--------------------------------------------------------------------------
    // PL Resources:
    //   - Logic Cells: ~930,000
    //   - DSP48E2:     4,272
    //   - BRAM36:      1,080
    //   - URAM:        80
    // RF Resources:
    //   - RF-ADC: 8× 14-bit @ 5 GSPS (4 exposed on board)
    //   - RF-DAC: 8× 14-bit @ 9.85 GSPS (2 exposed on board)
    //   - Bandwidth: 6 GHz input
    //--------------------------------------------------------------------------
    
    //--------------------------------------------------------------------------
    // Fixed-Point Format: Q15.16 (32-bit signed)
    //--------------------------------------------------------------------------
    parameter int DATA_WIDTH = 32;
    parameter int FRAC_BITS  = 16;
    parameter int INT_BITS   = DATA_WIDTH - FRAC_BITS - 1;  // 15 bits integer
    
    // Fixed-point constants
    parameter logic signed [DATA_WIDTH-1:0] FP_ONE   = 32'h0001_0000;  // 1.0
    parameter logic signed [DATA_WIDTH-1:0] FP_ZERO  = 32'h0000_0000;  // 0.0
    parameter logic signed [DATA_WIDTH-1:0] FP_HALF  = 32'h0000_8000;  // 0.5
    parameter logic signed [DATA_WIDTH-1:0] FP_EPS   = 32'h0000_0001;  // ~1.5e-5
    parameter logic signed [DATA_WIDTH-1:0] FP_TWO   = 32'h0002_0000;  // 2.0
    parameter logic signed [DATA_WIDTH-1:0] FP_PI    = 32'h0003_243F;  // π ≈ 3.14159
    
    //--------------------------------------------------------------------------
    // System Parameters
    //--------------------------------------------------------------------------
    parameter int STATE_DIM   = 4;    // [x, y, vx, vy]
    parameter int MEAS_DIM    = 2;    // [x, y]
    parameter int N_MODELS    = 3;    // CV, CT+, CT-
    parameter int LAG_DEPTH   = 50;   // Smoothing lag
    
    // Multi-target support (ZU48DR has 4,272 DSP48 — can support 8+ targets)
    parameter int MAX_TARGETS = 8;    // Parallel target tracking
    
    //--------------------------------------------------------------------------
    // Default Timing (dt = 0.1s, 10 Hz update rate)
    //--------------------------------------------------------------------------
    parameter logic signed [DATA_WIDTH-1:0] DT = 32'h0000_199A;  // 0.1 in Q15.16
    
    //--------------------------------------------------------------------------
    // RF-ADC Interface Parameters (ZU48DR Gen 3)
    //--------------------------------------------------------------------------
    parameter int ADC_BITS       = 14;           // 14-bit ADC
    parameter int ADC_RATE_GSPS  = 5;            // 5 GSPS max
    parameter int ADC_BW_GHZ     = 6;            // 6 GHz input bandwidth
    parameter int NUM_ADC_TILES  = 4;            // 4 RF-ADC tiles available
    
    //--------------------------------------------------------------------------
    // Types
    //--------------------------------------------------------------------------
    typedef logic signed [DATA_WIDTH-1:0] fp_t;
    typedef fp_t state_vec_t [STATE_DIM];
    typedef fp_t meas_vec_t [MEAS_DIM];
    typedef fp_t state_mat_t [STATE_DIM][STATE_DIM];
    typedef fp_t meas_mat_t [MEAS_DIM][STATE_DIM];
    typedef fp_t innov_cov_t [MEAS_DIM][MEAS_DIM];
    typedef fp_t kalman_gain_t [STATE_DIM][MEAS_DIM];
    
    // Multi-target types
    typedef fp_t target_state_t [MAX_TARGETS][STATE_DIM];
    typedef logic [MAX_TARGETS-1:0] target_valid_t;
    
    //--------------------------------------------------------------------------
    // Utility Functions
    //--------------------------------------------------------------------------
    
    // Fixed-point multiply: (a * b) >> FRAC_BITS
    function automatic fp_t fp_mul(input fp_t a, input fp_t b);
        logic signed [2*DATA_WIDTH-1:0] prod;
        prod = a * b;
        return prod[DATA_WIDTH+FRAC_BITS-1:FRAC_BITS];
    endfunction
    
    // Fixed-point divide (combinational, use sparingly)
    function automatic fp_t fp_div(input fp_t a, input fp_t b);
        logic signed [2*DATA_WIDTH-1:0] num;
        num = {{DATA_WIDTH{a[DATA_WIDTH-1]}}, a} << FRAC_BITS;
        return (b != 0) ? num / b : FP_ONE;
    endfunction
    
    // Fixed-point square root approximation (Newton-Raphson, 3 iterations)
    function automatic fp_t fp_sqrt(input fp_t x);
        fp_t guess, next_guess;
        if (x <= 0) return FP_ZERO;
        guess = x >> 1;  // Initial guess x/2
        for (int i = 0; i < 3; i++) begin
            next_guess = (guess + fp_div(x, guess)) >> 1;
            guess = next_guess;
        end
        return guess;
    endfunction

endpackage
