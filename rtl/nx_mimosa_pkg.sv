//==============================================================================
// NX-MIMOSA — Multi-model IMM Optimal Smoothing Algorithm
// Package Definitions
//==============================================================================
// Target: Xilinx RFSoC ZU48DR (Gen 3) @ 250MHz
// Author: Dr. Mladen Mešter / Nexellum d.o.o.
// License: Commercial
//
// SUPPORTED BOARDS:
//   • RFSoC 4x2 (Real Digital)    — $2,499 academic
//   • ZCU208 (AMD/Xilinx)         — $13,194
//
// Both use XCZU48DR — same RTL, different constraints/build scripts
//==============================================================================

`timescale 1ns/1ps

package nx_mimosa_pkg;

    //==========================================================================
    // TARGET DEVICE: ZU48DR (Gen 3 RFSoC)
    //==========================================================================
    // Common to RFSoC 4x2 and ZCU208:
    //   - Part: XCZU48DR-xFFVG1517E (speed grade -1 or -2)
    //   - LUTs:      425,280
    //   - FFs:       850,560
    //   - DSP48E2:   4,272
    //   - BRAM36:    1,080
    //   - URAM:      80
    //   - RF-ADC:    8× 14-bit @ 5 GSPS
    //   - RF-DAC:    8× 14-bit @ 9.85 GSPS (10 GSPS with different config)
    //   - SD-FEC:    8 cores
    //
    // Board-specific differences:
    // ┌─────────────────┬────────────────────┬────────────────────┐
    // │ Feature         │ RFSoC 4x2          │ ZCU208             │
    // ├─────────────────┼────────────────────┼────────────────────┤
    // │ ADC Exposed     │ 4× SMA             │ 8× via XM650       │
    // │ DAC Exposed     │ 2× SMA             │ 8× via XM650       │
    // │ Ethernet        │ 100G QSFP28        │ 10G SFP+           │
    // │ Price           │ $2,499 (academic)  │ $13,194            │
    // └─────────────────┴────────────────────┴────────────────────┘
    //==========================================================================
    
    //--------------------------------------------------------------------------
    // Board Selection (compile-time)
    //--------------------------------------------------------------------------
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
    
    // Fixed-point constants
    parameter logic signed [DATA_WIDTH-1:0] FP_ONE   = 32'h0001_0000;
    parameter logic signed [DATA_WIDTH-1:0] FP_ZERO  = 32'h0000_0000;
    parameter logic signed [DATA_WIDTH-1:0] FP_HALF  = 32'h0000_8000;
    parameter logic signed [DATA_WIDTH-1:0] FP_EPS   = 32'h0000_0001;
    parameter logic signed [DATA_WIDTH-1:0] FP_TWO   = 32'h0002_0000;
    parameter logic signed [DATA_WIDTH-1:0] FP_NEG1  = 32'hFFFF_0000;
    parameter logic signed [DATA_WIDTH-1:0] FP_PI    = 32'h0003_243F;
    
    //--------------------------------------------------------------------------
    // System Parameters
    //--------------------------------------------------------------------------
    parameter int STATE_DIM   = 4;    // [x, y, vx, vy]
    parameter int MEAS_DIM    = 2;    // [x, y]
    parameter int N_MODELS    = 3;    // CV, CT+, CT-
    parameter int LAG_DEPTH   = 50;   // Smoothing lag samples
    
    //--------------------------------------------------------------------------
    // Multi-Target Support
    //--------------------------------------------------------------------------
    parameter int MAX_TARGETS = 8;
    parameter int TARGET_ID_WIDTH = $clog2(MAX_TARGETS);
    
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

endpackage
