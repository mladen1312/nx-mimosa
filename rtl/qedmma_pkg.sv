//==============================================================================
// QEDMMA v3.1 Pro — Package Definitions
// Target: Xilinx RFSoC ZU48DR (Gen 3) @ 250MHz
//==============================================================================
// Author: Dr. Mladen Mešter / Nexellum d.o.o.
// License: Commercial (qedmma-pro)
// Board: RFSoC 4x2 (XCZU48DR-1FFVG1517E)
//==============================================================================

`timescale 1ns/1ps

package qedmma_pkg;

    //==========================================================================
    // TARGET: ZU48DR (Gen 3 RFSoC)
    //==========================================================================
    // Resources:
    //   LUTs:      425,280
    //   FFs:       850,560  
    //   DSP48E2:   4,272
    //   BRAM36:    1,080
    //   URAM:      80
    //   RF-ADC:    8× 14-bit @ 5 GSPS (4 on board)
    //   RF-DAC:    8× 14-bit @ 9.85 GSPS (2 on board)
    //==========================================================================

    // Fixed-Point: Q15.16
    parameter int DATA_WIDTH = 32;
    parameter int FRAC_BITS  = 16;
    parameter int EXT_WIDTH  = 48;
    
    // Constants
    parameter logic signed [DATA_WIDTH-1:0] FP_ONE  = 32'h0001_0000;
    parameter logic signed [DATA_WIDTH-1:0] FP_ZERO = 32'h0000_0000;
    parameter logic signed [DATA_WIDTH-1:0] FP_HALF = 32'h0000_8000;
    parameter logic signed [DATA_WIDTH-1:0] FP_TWO  = 32'h0002_0000;
    parameter logic signed [DATA_WIDTH-1:0] FP_PI   = 32'h0003_243F;
    
    // System Parameters
    parameter int STATE_DIM  = 4;   // [x, y, vx, vy]
    parameter int MEAS_DIM   = 2;   // [x, y]
    parameter int N_MODELS   = 3;   // CV, CT+, CT-
    parameter int LAG_DEPTH  = 50;  // Smoothing lag
    parameter int MAX_TARGETS = 8;  // Multi-target (ZU48DR has headroom)
    
    // ADC (Gen 3)
    parameter int ADC_BITS = 14;
    parameter int ADC_RATE_GSPS = 5;
    parameter int ADC_CHANNELS = 4;
    
    // Types
    typedef logic signed [DATA_WIDTH-1:0] fp_t;
    typedef logic signed [EXT_WIDTH-1:0] fp_ext_t;
    typedef fp_t state_vec_t [STATE_DIM];
    typedef fp_t meas_vec_t [MEAS_DIM];
    typedef fp_t state_mat_t [STATE_DIM][STATE_DIM];
    
    // Fixed-point multiply
    function automatic fp_t fp_mul(input fp_t a, input fp_t b);
        logic signed [2*DATA_WIDTH-1:0] prod;
        prod = a * b;
        return prod[DATA_WIDTH+FRAC_BITS-1:FRAC_BITS];
    endfunction
    
    // Fixed-point divide
    function automatic fp_t fp_div(input fp_t a, input fp_t b);
        logic signed [2*DATA_WIDTH-1:0] num;
        num = {{DATA_WIDTH{a[DATA_WIDTH-1]}}, a} << FRAC_BITS;
        return (b != 0) ? num / b : FP_ONE;
    endfunction

endpackage
