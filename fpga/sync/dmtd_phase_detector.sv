//-----------------------------------------------------------------------------
// QEDMMA v3.0 - Dual Mixer Time Difference (DMTD) Phase Detector
// Author: Dr. Mladen Mešter
// Copyright (c) 2026 - All Rights Reserved
//
// [REQ-SYNC-004] Sub-picosecond phase measurement resolution
// [REQ-SYNC-005] DMTD technique for White Rabbit
//
// Description:
//   Implements DMTD phase detector for measuring phase difference
//   between recovered clock and reference clock with sub-ps resolution.
//   Core technique from White Rabbit / CERN timing systems.
//
// Principle:
//   Two clocks (REF and RX) are sampled by a slightly offset clock (DMTD).
//   The beat frequency reveals phase relationship with high resolution.
//-----------------------------------------------------------------------------

`timescale 1ps / 1ps

module dmtd_phase_detector #(
    parameter int COUNTER_BITS = 28,      // Phase counter width
    parameter int NAVG_BITS    = 8,       // Averaging factor (2^N)
    parameter int DEGLITCH     = 4        // Deglitch filter cycles
)(
    input  logic                      clk_dmtd,     // DMTD sampling clock
    input  logic                      clk_ref,      // Reference clock
    input  logic                      clk_rx,       // Recovered RX clock
    input  logic                      rst_n,
    
    // Control
    input  logic                      enable,
    input  logic                      clear,
    input  logic [NAVG_BITS-1:0]      navg,         // Averaging 2^navg samples
    
    // Phase output
    output logic [COUNTER_BITS-1:0]   phase_raw,    // Raw phase count
    output logic [COUNTER_BITS-1:0]   phase_avg,    // Averaged phase
    output logic                      phase_valid,
    output logic                      phase_error,  // Phase measurement error
    
    // Debug
    output logic                      dbg_ref_edge,
    output logic                      dbg_rx_edge
);

    //=========================================================================
    // DMTD Samplers
    //=========================================================================
    // Sample ref and rx clocks with dmtd clock
    logic ref_sample_d1, ref_sample_d2, ref_sample_d3;
    logic rx_sample_d1, rx_sample_d2, rx_sample_d3;
    
    // Synchronizers for metastability
    always_ff @(posedge clk_dmtd or negedge rst_n) begin
        if (!rst_n) begin
            ref_sample_d1 <= 1'b0;
            ref_sample_d2 <= 1'b0;
            ref_sample_d3 <= 1'b0;
            rx_sample_d1 <= 1'b0;
            rx_sample_d2 <= 1'b0;
            rx_sample_d3 <= 1'b0;
        end else begin
            ref_sample_d1 <= clk_ref;
            ref_sample_d2 <= ref_sample_d1;
            ref_sample_d3 <= ref_sample_d2;
            rx_sample_d1 <= clk_rx;
            rx_sample_d2 <= rx_sample_d1;
            rx_sample_d3 <= rx_sample_d2;
        end
    end
    
    //=========================================================================
    // Edge Detection with Deglitch
    //=========================================================================
    logic [DEGLITCH-1:0] ref_edge_shift, rx_edge_shift;
    logic ref_edge_raw, rx_edge_raw;
    logic ref_edge_deglitch, rx_edge_deglitch;
    
    // Detect rising edges
    assign ref_edge_raw = ref_sample_d2 & ~ref_sample_d3;
    assign rx_edge_raw = rx_sample_d2 & ~rx_sample_d3;
    
    // Deglitch filter
    always_ff @(posedge clk_dmtd or negedge rst_n) begin
        if (!rst_n) begin
            ref_edge_shift <= '0;
            rx_edge_shift <= '0;
        end else begin
            ref_edge_shift <= {ref_edge_shift[DEGLITCH-2:0], ref_edge_raw};
            rx_edge_shift <= {rx_edge_shift[DEGLITCH-2:0], rx_edge_raw};
        end
    end
    
    // Deglitched edges (require consistent detection)
    assign ref_edge_deglitch = &ref_edge_shift[DEGLITCH-1:DEGLITCH-2];
    assign rx_edge_deglitch = &rx_edge_shift[DEGLITCH-1:DEGLITCH-2];
    
    //=========================================================================
    // Phase Counter
    //=========================================================================
    // Count DMTD cycles between ref edge and rx edge
    logic [COUNTER_BITS-1:0] phase_counter;
    logic counting;
    logic [COUNTER_BITS-1:0] phase_latch;
    logic phase_captured;
    
    typedef enum logic [1:0] {
        IDLE,
        WAIT_REF,
        COUNTING,
        WAIT_RX
    } phase_state_t;
    
    phase_state_t state;
    
    always_ff @(posedge clk_dmtd or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            phase_counter <= '0;
            phase_latch <= '0;
            phase_captured <= 1'b0;
            phase_error <= 1'b0;
        end else if (!enable || clear) begin
            state <= IDLE;
            phase_counter <= '0;
            phase_captured <= 1'b0;
        end else begin
            phase_captured <= 1'b0;
            
            case (state)
                IDLE: begin
                    phase_counter <= '0;
                    state <= WAIT_REF;
                end
                
                WAIT_REF: begin
                    if (ref_edge_deglitch) begin
                        phase_counter <= '0;
                        state <= COUNTING;
                    end
                end
                
                COUNTING: begin
                    phase_counter <= phase_counter + 1;
                    
                    if (rx_edge_deglitch) begin
                        phase_latch <= phase_counter;
                        phase_captured <= 1'b1;
                        state <= WAIT_REF;
                    end
                    
                    // Timeout detection (shouldn't happen with valid clocks)
                    if (phase_counter[COUNTER_BITS-1]) begin
                        phase_error <= 1'b1;
                        state <= IDLE;
                    end
                end
                
                default: state <= IDLE;
            endcase
        end
    end
    
    assign phase_raw = phase_latch;
    assign dbg_ref_edge = ref_edge_deglitch;
    assign dbg_rx_edge = rx_edge_deglitch;
    
    //=========================================================================
    // Phase Averaging
    //=========================================================================
    logic [COUNTER_BITS+NAVG_BITS-1:0] phase_sum;
    logic [NAVG_BITS-1:0] avg_count;
    logic avg_complete;
    
    always_ff @(posedge clk_dmtd or negedge rst_n) begin
        if (!rst_n) begin
            phase_sum <= '0;
            avg_count <= '0;
            phase_avg <= '0;
            phase_valid <= 1'b0;
            avg_complete <= 1'b0;
        end else if (clear) begin
            phase_sum <= '0;
            avg_count <= '0;
            phase_valid <= 1'b0;
        end else if (enable) begin
            phase_valid <= 1'b0;
            
            if (phase_captured) begin
                phase_sum <= phase_sum + phase_latch;
                avg_count <= avg_count + 1;
                
                // Check if averaging complete
                if (avg_count >= (1 << navg) - 1) begin
                    phase_avg <= phase_sum >> navg;
                    phase_valid <= 1'b1;
                    phase_sum <= '0;
                    avg_count <= '0;
                end
            end
        end
    end

endmodule
