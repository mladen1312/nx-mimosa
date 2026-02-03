//-----------------------------------------------------------------------------
// QEDMMA v3.0 - Parallel Correlator Engine
// Author: Dr. Mladen Mešter
// Copyright (c) 2026 - All Rights Reserved
//
// [REQ-CORR-001] 200 Mchip/s correlation throughput
// [REQ-CORR-003] 33-48 dB processing gain
// [REQ-CORR-007] Q1.15 fixed-point processing
//
// Description:
//   8-lane parallel correlator for spread-spectrum radar.
//   Processes 8 chips per clock at 25 MHz (200 Mchip/s effective).
//   Uses DSP48E2 cascading for efficient accumulation.
//-----------------------------------------------------------------------------

`timescale 1ns / 1ps

module parallel_correlator_engine #(
    parameter int PARALLEL_WIDTH  = 8,           // Chips per clock
    parameter int SAMPLE_WIDTH    = 16,          // ADC sample width (Q1.15)
    parameter int ACC_WIDTH       = 48,          // Accumulator width (DSP48)
    parameter int OUTPUT_WIDTH    = 32,          // Output magnitude width
    parameter int MAX_CODE_LEN    = 65536,       // Maximum code length
    parameter int CODE_ADDR_WIDTH = 16           // log2(MAX_CODE_LEN)
)(
    input  logic                        clk,
    input  logic                        rst_n,
    
    //-------------------------------------------------------------------------
    // Input Samples (from DDC/ADC)
    //-------------------------------------------------------------------------
    input  logic signed [SAMPLE_WIDTH-1:0] sample_i [PARALLEL_WIDTH],  // I channel
    input  logic signed [SAMPLE_WIDTH-1:0] sample_q [PARALLEL_WIDTH],  // Q channel
    input  logic                           sample_valid,
    output logic                           sample_ready,
    
    //-------------------------------------------------------------------------
    // Reference Code Input (from PRBS generator)
    //-------------------------------------------------------------------------
    input  logic [PARALLEL_WIDTH-1:0]      code_chips,    // 8 parallel code chips
    input  logic                           code_valid,
    
    //-------------------------------------------------------------------------
    // Configuration
    //-------------------------------------------------------------------------
    input  logic [CODE_ADDR_WIDTH-1:0]     cfg_code_length,    // Chips per integration
    input  logic                           cfg_enable,
    input  logic                           cfg_accumulate,     // Continue accumulating
    input  logic                           cfg_clear,          // Clear accumulators
    
    //-------------------------------------------------------------------------
    // Correlation Output
    //-------------------------------------------------------------------------
    output logic [OUTPUT_WIDTH-1:0]        corr_magnitude_sq,  // |I|² + |Q|²
    output logic signed [ACC_WIDTH-1:0]    corr_i,             // Raw I accumulator
    output logic signed [ACC_WIDTH-1:0]    corr_q,             // Raw Q accumulator
    output logic                           corr_valid,
    output logic [CODE_ADDR_WIDTH-1:0]     corr_chip_count,
    
    //-------------------------------------------------------------------------
    // Status
    //-------------------------------------------------------------------------
    output logic                           overflow_detected,
    output logic                           integration_done
);

    //=========================================================================
    // Convert Code Chips to Signed (+1/-1)
    //=========================================================================
    // BPSK: 0 → -1 (0x8001 in Q1.15), 1 → +1 (0x7FFF in Q1.15)
    localparam logic signed [SAMPLE_WIDTH-1:0] CODE_PLUS_ONE  = 16'sh7FFF;  // +0.99997
    localparam logic signed [SAMPLE_WIDTH-1:0] CODE_MINUS_ONE = 16'sh8001;  // -0.99997
    
    logic signed [SAMPLE_WIDTH-1:0] code_signed [PARALLEL_WIDTH];
    
    always_comb begin
        for (int i = 0; i < PARALLEL_WIDTH; i++) begin
            code_signed[i] = code_chips[i] ? CODE_PLUS_ONE : CODE_MINUS_ONE;
        end
    end
    
    //=========================================================================
    // Parallel Multiply Stage (8 lanes × I/Q)
    //=========================================================================
    // Each multiply: 16-bit × 16-bit → 32-bit
    logic signed [2*SAMPLE_WIDTH-1:0] mult_i [PARALLEL_WIDTH];
    logic signed [2*SAMPLE_WIDTH-1:0] mult_q [PARALLEL_WIDTH];
    logic mult_valid_d1;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < PARALLEL_WIDTH; i++) begin
                mult_i[i] <= '0;
                mult_q[i] <= '0;
            end
            mult_valid_d1 <= 1'b0;
        end else if (cfg_enable && sample_valid && code_valid) begin
            for (int i = 0; i < PARALLEL_WIDTH; i++) begin
                // Correlation: sample × code (BPSK → multiply by ±1)
                mult_i[i] <= sample_i[i] * code_signed[i];
                mult_q[i] <= sample_q[i] * code_signed[i];
            end
            mult_valid_d1 <= 1'b1;
        end else begin
            mult_valid_d1 <= 1'b0;
        end
    end
    
    //=========================================================================
    // Parallel Sum Stage (reduce 8 lanes to 1)
    //=========================================================================
    // Pipelined adder tree for timing closure
    
    // Stage 1: 8 → 4
    logic signed [2*SAMPLE_WIDTH:0] sum1_i [4];
    logic signed [2*SAMPLE_WIDTH:0] sum1_q [4];
    logic sum1_valid;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 4; i++) begin
                sum1_i[i] <= '0;
                sum1_q[i] <= '0;
            end
            sum1_valid <= 1'b0;
        end else if (mult_valid_d1) begin
            for (int i = 0; i < 4; i++) begin
                sum1_i[i] <= mult_i[2*i] + mult_i[2*i+1];
                sum1_q[i] <= mult_q[2*i] + mult_q[2*i+1];
            end
            sum1_valid <= 1'b1;
        end else begin
            sum1_valid <= 1'b0;
        end
    end
    
    // Stage 2: 4 → 2
    logic signed [2*SAMPLE_WIDTH+1:0] sum2_i [2];
    logic signed [2*SAMPLE_WIDTH+1:0] sum2_q [2];
    logic sum2_valid;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 2; i++) begin
                sum2_i[i] <= '0;
                sum2_q[i] <= '0;
            end
            sum2_valid <= 1'b0;
        end else if (sum1_valid) begin
            for (int i = 0; i < 2; i++) begin
                sum2_i[i] <= sum1_i[2*i] + sum1_i[2*i+1];
                sum2_q[i] <= sum1_q[2*i] + sum1_q[2*i+1];
            end
            sum2_valid <= 1'b1;
        end else begin
            sum2_valid <= 1'b0;
        end
    end
    
    // Stage 3: 2 → 1
    logic signed [2*SAMPLE_WIDTH+2:0] sum3_i;
    logic signed [2*SAMPLE_WIDTH+2:0] sum3_q;
    logic sum3_valid;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sum3_i <= '0;
            sum3_q <= '0;
            sum3_valid <= 1'b0;
        end else if (sum2_valid) begin
            sum3_i <= sum2_i[0] + sum2_i[1];
            sum3_q <= sum2_q[0] + sum2_q[1];
            sum3_valid <= 1'b1;
        end else begin
            sum3_valid <= 1'b0;
        end
    end
    
    //=========================================================================
    // Accumulator (48-bit for DSP48 compatibility)
    //=========================================================================
    logic signed [ACC_WIDTH-1:0] acc_i;
    logic signed [ACC_WIDTH-1:0] acc_q;
    logic [CODE_ADDR_WIDTH-1:0] chip_counter;
    logic acc_overflow_i, acc_overflow_q;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc_i <= '0;
            acc_q <= '0;
            chip_counter <= '0;
            integration_done <= 1'b0;
            acc_overflow_i <= 1'b0;
            acc_overflow_q <= 1'b0;
        end else if (cfg_clear || (!cfg_accumulate && integration_done)) begin
            acc_i <= '0;
            acc_q <= '0;
            chip_counter <= '0;
            integration_done <= 1'b0;
            acc_overflow_i <= 1'b0;
            acc_overflow_q <= 1'b0;
        end else if (sum3_valid && cfg_enable) begin
            // Accumulate with overflow detection
            {acc_overflow_i, acc_i} <= {acc_i[ACC_WIDTH-1], acc_i} + 
                                        {{(ACC_WIDTH-2*SAMPLE_WIDTH-3){sum3_i[2*SAMPLE_WIDTH+2]}}, sum3_i};
            {acc_overflow_q, acc_q} <= {acc_q[ACC_WIDTH-1], acc_q} + 
                                        {{(ACC_WIDTH-2*SAMPLE_WIDTH-3){sum3_q[2*SAMPLE_WIDTH+2]}}, sum3_q};
            
            // Chip counter
            if (chip_counter + PARALLEL_WIDTH >= cfg_code_length) begin
                chip_counter <= '0;
                integration_done <= 1'b1;
            end else begin
                chip_counter <= chip_counter + PARALLEL_WIDTH;
                integration_done <= 1'b0;
            end
        end else begin
            integration_done <= 1'b0;
        end
    end
    
    //=========================================================================
    // Magnitude Squared Calculation
    //=========================================================================
    // |corr|² = I² + Q²
    // Pipeline for timing
    
    logic signed [ACC_WIDTH-1:0] acc_i_latched, acc_q_latched;
    logic [2*ACC_WIDTH-1:0] mag_i_sq, mag_q_sq;
    logic [2*ACC_WIDTH-1:0] mag_total;
    logic mag_valid_d1, mag_valid_d2;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc_i_latched <= '0;
            acc_q_latched <= '0;
            mag_i_sq <= '0;
            mag_q_sq <= '0;
            mag_total <= '0;
            mag_valid_d1 <= 1'b0;
            mag_valid_d2 <= 1'b0;
            corr_valid <= 1'b0;
        end else begin
            // Stage 1: Latch accumulators
            if (integration_done) begin
                acc_i_latched <= acc_i;
                acc_q_latched <= acc_q;
                mag_valid_d1 <= 1'b1;
            end else begin
                mag_valid_d1 <= 1'b0;
            end
            
            // Stage 2: Square
            if (mag_valid_d1) begin
                mag_i_sq <= acc_i_latched * acc_i_latched;
                mag_q_sq <= acc_q_latched * acc_q_latched;
                mag_valid_d2 <= 1'b1;
            end else begin
                mag_valid_d2 <= 1'b0;
            end
            
            // Stage 3: Sum
            if (mag_valid_d2) begin
                mag_total <= mag_i_sq + mag_q_sq;
                corr_valid <= 1'b1;
            end else begin
                corr_valid <= 1'b0;
            end
        end
    end
    
    //=========================================================================
    // Output Assignment
    //=========================================================================
    // Take upper bits of magnitude for output (scaling)
    assign corr_magnitude_sq = mag_total[2*ACC_WIDTH-1 -: OUTPUT_WIDTH];
    assign corr_i = acc_i;
    assign corr_q = acc_q;
    assign corr_chip_count = chip_counter;
    assign overflow_detected = acc_overflow_i | acc_overflow_q;
    assign sample_ready = cfg_enable && !integration_done;

endmodule
