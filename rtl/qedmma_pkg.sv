//==============================================================================
// QEDMMA v3.1 Pro — Package Definitions
// Target: Xilinx RFSoC ZU48DR (Gen 3) @ 250MHz
//==============================================================================
// Author: Dr. Mladen Mešter / Nexellum d.o.o.
// License: Commercial (qedmma-pro)
//
// SUPPORTED BOARDS:
//   • RFSoC 4x2 (Real Digital)    — $2,499 academic
//   • ZCU208 (AMD/Xilinx)         — $13,194
//
// Both use XCZU48DR — same RTL, different constraints/build scripts
//==============================================================================

`timescale 1ns/1ps

package qedmma_pkg;

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
    // │ Availability    │ Academic only      │ General            │
    // │ PYNQ Support    │ Native             │ Supported          │
    // │ Open Source PCB │ Yes                │ No                 │
    // └─────────────────┴────────────────────┴────────────────────┘
    //==========================================================================
    
    //--------------------------------------------------------------------------
    // Board Selection (compile-time)
    //--------------------------------------------------------------------------
    // Define one of these in your build:
    //   `define TARGET_RFSOC_4X2
    //   `define TARGET_ZCU208
    // Default: RFSoC 4x2
    //--------------------------------------------------------------------------
    
`ifndef TARGET_ZCU208
    `define TARGET_RFSOC_4X2
`endif

`ifdef TARGET_RFSOC_4X2
    // RFSoC 4x2 Board Configuration
    parameter int NUM_ADC_CHANNELS = 4;   // 4 exposed via SMA
    parameter int NUM_DAC_CHANNELS = 2;   // 2 exposed via SMA
    parameter bit HAS_100G_ETH = 1'b1;    // QSFP28 present
    parameter bit HAS_10G_ETH = 1'b1;     // Also has 1G/10G
    parameter string BOARD_NAME = "RFSoC 4x2";
`else
    // ZCU208 Board Configuration
    parameter int NUM_ADC_CHANNELS = 8;   // 8 via XM650 daughtercard
    parameter int NUM_DAC_CHANNELS = 8;   // 8 via XM650 daughtercard
    parameter bit HAS_100G_ETH = 1'b0;    // No QSFP28
    parameter bit HAS_10G_ETH = 1'b1;     // SFP+ present
    parameter string BOARD_NAME = "ZCU208";
`endif

    //--------------------------------------------------------------------------
    // Fixed-Point Format: Q15.16 (32-bit signed)
    //--------------------------------------------------------------------------
    parameter int DATA_WIDTH = 32;
    parameter int FRAC_BITS  = 16;
    parameter int INT_BITS   = DATA_WIDTH - FRAC_BITS - 1;  // 15 bits
    parameter int EXT_WIDTH  = 48;  // Extended precision
    
    // Fixed-point constants
    parameter logic signed [DATA_WIDTH-1:0] FP_ONE   = 32'h0001_0000;  // 1.0
    parameter logic signed [DATA_WIDTH-1:0] FP_ZERO  = 32'h0000_0000;  // 0.0
    parameter logic signed [DATA_WIDTH-1:0] FP_HALF  = 32'h0000_8000;  // 0.5
    parameter logic signed [DATA_WIDTH-1:0] FP_EPS   = 32'h0000_0001;  // ~1.5e-5
    parameter logic signed [DATA_WIDTH-1:0] FP_TWO   = 32'h0002_0000;  // 2.0
    parameter logic signed [DATA_WIDTH-1:0] FP_NEG1  = 32'hFFFF_0000;  // -1.0
    parameter logic signed [DATA_WIDTH-1:0] FP_PI    = 32'h0003_243F;  // π
    
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
    // ZU48DR has 4,272 DSP48E2 — supports 8+ parallel trackers
    parameter int MAX_TARGETS = 8;
    parameter int TARGET_ID_WIDTH = $clog2(MAX_TARGETS);
    
    //--------------------------------------------------------------------------
    // RF-ADC Parameters (Gen 3)
    //--------------------------------------------------------------------------
    parameter int ADC_BITS = 14;          // 14-bit resolution
    parameter int ADC_RATE_GSPS = 5;      // 5 GSPS max sample rate
    parameter int ADC_BW_GHZ = 6;         // 6 GHz input bandwidth
    parameter int DAC_RATE_GSPS = 10;     // 9.85/10 GSPS DAC
    
    //--------------------------------------------------------------------------
    // Timing
    //--------------------------------------------------------------------------
    parameter int CLK_FREQ_MHZ = 250;     // System clock
    parameter logic signed [DATA_WIDTH-1:0] DT_100MS = 32'h0000_199A;  // 0.1s
    parameter logic signed [DATA_WIDTH-1:0] DT_50MS  = 32'h0000_0CCD;  // 0.05s
    parameter logic signed [DATA_WIDTH-1:0] DT_10MS  = 32'h0000_028F;  // 0.01s
    
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
    
    // Multi-target types
    typedef fp_t target_state_t [MAX_TARGETS][STATE_DIM];
    typedef logic [TARGET_ID_WIDTH-1:0] target_id_t;
    typedef logic [MAX_TARGETS-1:0] target_mask_t;
    
    //--------------------------------------------------------------------------
    // AXI-Stream Widths
    //--------------------------------------------------------------------------
    parameter int AXIS_MEAS_WIDTH = 64;    // {z_y, z_x}
    parameter int AXIS_STATE_WIDTH = 128;  // {vy, vx, y, x}
    
    //--------------------------------------------------------------------------
    // Utility Functions
    //--------------------------------------------------------------------------
    
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
    
    // Absolute value
    function automatic fp_t fp_abs(input fp_t val);
        return (val < 0) ? -val : val;
    endfunction
    
    // Saturate extended to standard width
    function automatic fp_t fp_saturate(input fp_ext_t val);
        if (val > $signed({{(EXT_WIDTH-DATA_WIDTH){1'b0}}, {1'b0, {(DATA_WIDTH-1){1'b1}}}}))
            return {1'b0, {(DATA_WIDTH-1){1'b1}}};
        else if (val < $signed({{(EXT_WIDTH-DATA_WIDTH){1'b1}}, {1'b1, {(DATA_WIDTH-1){1'b0}}}}))
            return {1'b1, {(DATA_WIDTH-1){1'b0}}};
        else
            return val[DATA_WIDTH-1:0];
    endfunction

endpackage
