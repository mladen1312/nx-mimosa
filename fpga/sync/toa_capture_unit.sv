//-----------------------------------------------------------------------------
// QEDMMA v3.0 - Time-of-Arrival Capture Unit
// Author: Dr. Mladen Mešter
// Copyright (c) 2026 - All Rights Reserved
//
// [REQ-SYNC-006] Sub-100 ps ToA precision for multilateration
// [REQ-SYNC-007] Multiple simultaneous capture channels
//
// Description:
//   High-precision timestamp capture for radar pulse ToA measurement.
//   Uses White Rabbit synchronized time base for coherent multistatic
//   operation. Supports multiple channels for array elements.
//-----------------------------------------------------------------------------

`timescale 1ps / 1ps

module toa_capture_unit #(
    parameter int NUM_CHANNELS    = 8,           // Number of capture channels
    parameter int TIMESTAMP_BITS  = 80,          // 48-bit TAI + 32-bit sub-ns
    parameter int FINE_BITS       = 12,          // Fine interpolation bits
    parameter int FIFO_DEPTH      = 32           // Per-channel FIFO depth
)(
    input  logic                              clk,
    input  logic                              rst_n,
    
    //=========================================================================
    // Time Base Input (from White Rabbit PTP)
    //=========================================================================
    input  logic [47:0]                       tai_seconds,
    input  logic [31:0]                       tai_nanoseconds,
    input  logic                              time_valid,
    input  logic                              wr_locked,       // WR sync locked
    
    //=========================================================================
    // Capture Trigger Inputs
    //=========================================================================
    input  logic [NUM_CHANNELS-1:0]           trig_pulse,      // Rising edge triggers
    input  logic [NUM_CHANNELS-1:0]           trig_enable,     // Channel enables
    
    //=========================================================================
    // Fine Interpolation (TDC)
    //=========================================================================
    // Tapped delay line outputs for sub-clock resolution
    input  logic [FINE_BITS-1:0]              fine_phase [NUM_CHANNELS],
    input  logic [NUM_CHANNELS-1:0]           fine_valid,
    
    //=========================================================================
    // Readout Interface
    //=========================================================================
    input  logic [$clog2(NUM_CHANNELS)-1:0]   rd_channel,
    input  logic                              rd_req,
    output logic [TIMESTAMP_BITS-1:0]         rd_timestamp,
    output logic [FINE_BITS-1:0]              rd_fine,
    output logic [$clog2(NUM_CHANNELS)-1:0]   rd_channel_id,
    output logic                              rd_valid,
    output logic [NUM_CHANNELS-1:0]           fifo_empty,
    output logic [NUM_CHANNELS-1:0]           fifo_overflow,
    
    //=========================================================================
    // Statistics
    //=========================================================================
    output logic [31:0]                       capture_count [NUM_CHANNELS],
    output logic [31:0]                       total_captures
);

    //=========================================================================
    // Per-Channel Capture Logic
    //=========================================================================
    
    // Capture FIFOs
    typedef struct packed {
        logic [TIMESTAMP_BITS-1:0] timestamp;
        logic [FINE_BITS-1:0]      fine;
    } toa_entry_t;
    
    toa_entry_t fifo_data [NUM_CHANNELS][FIFO_DEPTH];
    logic [$clog2(FIFO_DEPTH)-1:0] wr_ptr [NUM_CHANNELS];
    logic [$clog2(FIFO_DEPTH)-1:0] rd_ptr [NUM_CHANNELS];
    logic [$clog2(FIFO_DEPTH):0]   fifo_count [NUM_CHANNELS];
    
    // Edge detection for triggers
    logic [NUM_CHANNELS-1:0] trig_d1, trig_d2;
    logic [NUM_CHANNELS-1:0] trig_rising;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            trig_d1 <= '0;
            trig_d2 <= '0;
        end else begin
            trig_d1 <= trig_pulse;
            trig_d2 <= trig_d1;
        end
    end
    
    assign trig_rising = trig_d1 & ~trig_d2;
    
    // Capture and FIFO write
    generate
        for (genvar ch = 0; ch < NUM_CHANNELS; ch++) begin : gen_channel
            
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    wr_ptr[ch] <= '0;
                    fifo_count[ch] <= '0;
                    capture_count[ch] <= '0;
                    fifo_overflow[ch] <= 1'b0;
                end else begin
                    fifo_overflow[ch] <= 1'b0;
                    
                    // Capture on rising edge if enabled and WR locked
                    if (trig_rising[ch] && trig_enable[ch] && wr_locked && time_valid) begin
                        if (fifo_count[ch] < FIFO_DEPTH) begin
                            // Store timestamp
                            fifo_data[ch][wr_ptr[ch]].timestamp <= {tai_seconds, tai_nanoseconds};
                            fifo_data[ch][wr_ptr[ch]].fine <= fine_valid[ch] ? fine_phase[ch] : '0;
                            
                            wr_ptr[ch] <= wr_ptr[ch] + 1;
                            fifo_count[ch] <= fifo_count[ch] + 1;
                            capture_count[ch] <= capture_count[ch] + 1;
                        end else begin
                            fifo_overflow[ch] <= 1'b1;
                        end
                    end
                end
            end
            
            assign fifo_empty[ch] = (fifo_count[ch] == 0);
            
        end
    endgenerate
    
    //=========================================================================
    // Readout Logic
    //=========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_timestamp <= '0;
            rd_fine <= '0;
            rd_channel_id <= '0;
            rd_valid <= 1'b0;
            for (int i = 0; i < NUM_CHANNELS; i++) begin
                rd_ptr[i] <= '0;
            end
        end else begin
            rd_valid <= 1'b0;
            
            if (rd_req && fifo_count[rd_channel] > 0) begin
                rd_timestamp <= fifo_data[rd_channel][rd_ptr[rd_channel]].timestamp;
                rd_fine <= fifo_data[rd_channel][rd_ptr[rd_channel]].fine;
                rd_channel_id <= rd_channel;
                rd_valid <= 1'b1;
                
                rd_ptr[rd_channel] <= rd_ptr[rd_channel] + 1;
                fifo_count[rd_channel] <= fifo_count[rd_channel] - 1;
            end
        end
    end
    
    //=========================================================================
    // Total Capture Counter
    //=========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            total_captures <= '0;
        end else begin
            if (|trig_rising & |trig_enable & wr_locked) begin
                total_captures <= total_captures + 1;
            end
        end
    end

endmodule
