//==============================================================================
// QEDMMA v3.1 Pro — Package Definitions
// [REQ-RTL-PKG-01] Fixed-point parameters and types
//==============================================================================
// Author: Dr. Mladen Mešter / Nexellum d.o.o.
// License: Commercial (qedmma-pro)
//==============================================================================

`timescale 1ns/1ps

package qedmma_pkg;

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
    
    //--------------------------------------------------------------------------
    // System Parameters
    //--------------------------------------------------------------------------
    parameter int STATE_DIM  = 4;   // [x, y, vx, vy]
    parameter int MEAS_DIM   = 2;   // [x, y]
    parameter int N_MODELS   = 3;   // CV, CT+, CT-
    parameter int LAG_DEPTH  = 50;  // Smoothing lag
    
    //--------------------------------------------------------------------------
    // Default Timing (dt = 0.1s)
    //--------------------------------------------------------------------------
    parameter logic signed [DATA_WIDTH-1:0] DT = 32'h0000_199A;  // 0.1 in Q15.16
    
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

endpackage
