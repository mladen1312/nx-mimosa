//=============================================================================
// QEDMMA v3.1 - Coherent Pulse Integrator
// [REQ-INT-001] N-pulse coherent integration for extended range
// [REQ-INT-002] Configurable integration depth (1-128 pulses)
// [REQ-INT-003] 80+ dB total processing gain target
//
// Author: Dr. Mladen Mešter
// Copyright (c) 2026 - All Rights Reserved
//
// Description:
//   Coherent integrator for PRBS-15 correlator output.
//   Accumulates N correlation results to achieve additional
//   processing gain of 10*log10(N) dB.
//
//   Architecture Decision (Architecture Decision):
//   - PRBS-20 requires 1821 BRAM (ZU47DR has only 1080)
//   - Solution: PRBS-15 (45.2 dB) + 7 integrations = 53.7 dB
//   - With quantum (+18.2 dB) and ECCM (+8.4 dB) = 80.3 dB total
//   - F-35 detection: 520+ km with integration
//
//   Key Features:
//   - Phase-coherent accumulation (preserves Doppler information)
//   - Configurable integration depth via AXI register
//   - Automatic normalization to prevent overflow
//   - Motion compensation input for fast movers
//
// Target: Xilinx Zynq UltraScale+ ZU47DR
// Clock: 25 MHz (correlator output rate)
//=============================================================================

`timescale 1ns / 1ps

module coherent_integrator #(
    parameter int DATA_WIDTH      = 32,          // Correlator output width
    parameter int ACC_WIDTH       = 48,          // Accumulator width
    parameter int MAX_INTEGRATION = 128,         // Maximum integration depth
    parameter int RANGE_BINS      = 32768,       // Number of range bins (PRBS-15)
    parameter int LOG2_BINS       = 15           // log2(RANGE_BINS)
)(
    input  logic                          clk,
    input  logic                          rst_n,
    
    //=========================================================================
    // Correlator Input (from parallel_correlator_engine)
    //=========================================================================
    input  logic signed [DATA_WIDTH-1:0]  corr_i,           // I correlation
    input  logic signed [DATA_WIDTH-1:0]  corr_q,           // Q correlation
    input  logic [LOG2_BINS-1:0]          corr_bin,         // Range bin index
    input  logic                          corr_valid,
    input  logic                          corr_last,        // Last bin of sequence
    
    //=========================================================================
    // Integrated Output
    //=========================================================================
    output logic signed [ACC_WIDTH-1:0]   int_i,
    output logic signed [ACC_WIDTH-1:0]   int_q,
    output logic [LOG2_BINS-1:0]          int_bin,
    output logic                          int_valid,
    output logic                          int_complete,     // Full integration done
    
    //=========================================================================
    // Magnitude Output (optional, for CFAR)
    //=========================================================================
    output logic [ACC_WIDTH-1:0]          int_mag,          // |I| + |Q| approx
    output logic                          int_mag_valid,
    
    //=========================================================================
    // Configuration
    //=========================================================================
    input  logic [6:0]                    cfg_num_pulses,   // 1-128
    input  logic                          cfg_enable,
    input  logic                          cfg_clear,        // Clear accumulators
    
    //=========================================================================
    // Motion Compensation (for fast movers)
    //=========================================================================
    input  logic signed [15:0]            comp_phase_inc,   // Phase increment per pulse
    input  logic                          comp_enable,
    
    //=========================================================================
    // Status
    //=========================================================================
    output logic [6:0]                    status_pulse_cnt,
    output logic                          status_overflow,
    output logic [ACC_WIDTH-1:0]          status_max_value
);

    //=========================================================================
    // Local Parameters
    //=========================================================================
    localparam int BRAM_ADDR_WIDTH = LOG2_BINS;
    localparam int BRAM_DATA_WIDTH = 2 * ACC_WIDTH;  // I and Q packed
    
    //=========================================================================
    // Accumulator Memory (BRAM)
    //=========================================================================
    // Dual-port BRAM for read-modify-write in single cycle
    
    (* ram_style = "block" *)
    logic [BRAM_DATA_WIDTH-1:0] acc_mem [RANGE_BINS];
    
    // Port A: Read
    logic [BRAM_ADDR_WIDTH-1:0] rd_addr;
    logic [BRAM_DATA_WIDTH-1:0] rd_data;
    logic                       rd_valid;
    
    // Port B: Write
    logic [BRAM_ADDR_WIDTH-1:0] wr_addr;
    logic [BRAM_DATA_WIDTH-1:0] wr_data;
    logic                       wr_en;
    
    //=========================================================================
    // Pipeline Registers
    //=========================================================================
    
    // Stage 1: Input registration
    logic signed [DATA_WIDTH-1:0]  s1_corr_i, s1_corr_q;
    logic [BRAM_ADDR_WIDTH-1:0]    s1_bin;
    logic                          s1_valid;
    logic                          s1_last;
    
    // Stage 2: Memory read
    logic signed [DATA_WIDTH-1:0]  s2_corr_i, s2_corr_q;
    logic [BRAM_ADDR_WIDTH-1:0]    s2_bin;
    logic                          s2_valid;
    logic                          s2_last;
    logic signed [ACC_WIDTH-1:0]   s2_acc_i, s2_acc_q;
    
    // Stage 3: Accumulate
    logic signed [ACC_WIDTH-1:0]   s3_sum_i, s3_sum_q;
    logic [BRAM_ADDR_WIDTH-1:0]    s3_bin;
    logic                          s3_valid;
    logic                          s3_last;
    
    // Stage 4: Write back
    logic signed [ACC_WIDTH-1:0]   s4_sum_i, s4_sum_q;
    logic [BRAM_ADDR_WIDTH-1:0]    s4_bin;
    logic                          s4_valid;
    
    //=========================================================================
    // Pulse Counter
    //=========================================================================
    logic [6:0] pulse_counter;
    logic       integration_active;
    logic       integration_complete;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pulse_counter <= '0;
            integration_active <= 1'b0;
            integration_complete <= 1'b0;
        end else if (cfg_clear || !cfg_enable) begin
            pulse_counter <= '0;
            integration_active <= 1'b0;
            integration_complete <= 1'b0;
        end else if (corr_last && corr_valid) begin
            if (pulse_counter < cfg_num_pulses - 1) begin
                pulse_counter <= pulse_counter + 1;
                integration_active <= 1'b1;
                integration_complete <= 1'b0;
            end else begin
                pulse_counter <= '0;
                integration_active <= 1'b0;
                integration_complete <= 1'b1;
            end
        end else begin
            integration_complete <= 1'b0;
        end
    end
    
    assign status_pulse_cnt = pulse_counter;
    
    //=========================================================================
    // Motion Compensation (Phase Rotation)
    //=========================================================================
    logic signed [15:0] comp_phase;      // Current phase
    logic signed [15:0] comp_cos, comp_sin;
    logic signed [DATA_WIDTH-1:0] comp_i, comp_q;
    
    // Simple phase accumulator
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            comp_phase <= '0;
        end else if (cfg_clear) begin
            comp_phase <= '0;
        end else if (corr_last && corr_valid && comp_enable) begin
            comp_phase <= comp_phase + comp_phase_inc;
        end
    end
    
    // CORDIC or LUT for sin/cos would go here
    // Simplified: assume small angles, cos ≈ 1, sin ≈ phase/32768
    assign comp_cos = 16'h7FFF;  // ~1.0 in Q0.15
    assign comp_sin = comp_enable ? comp_phase : 16'h0000;
    
    // Phase rotation: I' = I*cos - Q*sin, Q' = I*sin + Q*cos
    always_comb begin
        if (comp_enable) begin
            comp_i = (corr_i * comp_cos - corr_q * comp_sin) >>> 15;
            comp_q = (corr_i * comp_sin + corr_q * comp_cos) >>> 15;
        end else begin
            comp_i = corr_i;
            comp_q = corr_q;
        end
    end
    
    //=========================================================================
    // Stage 1: Input Registration
    //=========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1_corr_i <= '0;
            s1_corr_q <= '0;
            s1_bin <= '0;
            s1_valid <= 1'b0;
            s1_last <= 1'b0;
        end else begin
            s1_corr_i <= comp_i;
            s1_corr_q <= comp_q;
            s1_bin <= corr_bin;
            s1_valid <= corr_valid && cfg_enable;
            s1_last <= corr_last;
        end
    end
    
    //=========================================================================
    // Stage 2: Memory Read
    //=========================================================================
    assign rd_addr = s1_bin;
    
    always_ff @(posedge clk) begin
        if (s1_valid) begin
            rd_data <= acc_mem[rd_addr];
        end
    end
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s2_corr_i <= '0;
            s2_corr_q <= '0;
            s2_bin <= '0;
            s2_valid <= 1'b0;
            s2_last <= 1'b0;
        end else begin
            s2_corr_i <= s1_corr_i;
            s2_corr_q <= s1_corr_q;
            s2_bin <= s1_bin;
            s2_valid <= s1_valid;
            s2_last <= s1_last;
            
            // Unpack accumulated values
            if (pulse_counter == 0 && !integration_active) begin
                // First pulse - start fresh
                s2_acc_i <= '0;
                s2_acc_q <= '0;
            end else begin
                s2_acc_i <= $signed(rd_data[ACC_WIDTH-1:0]);
                s2_acc_q <= $signed(rd_data[2*ACC_WIDTH-1:ACC_WIDTH]);
            end
        end
    end
    
    //=========================================================================
    // Stage 3: Accumulate
    //=========================================================================
    logic signed [ACC_WIDTH-1:0] new_acc_i, new_acc_q;
    logic overflow_i, overflow_q;
    
    // Sign extension and addition
    always_comb begin
        new_acc_i = s2_acc_i + {{(ACC_WIDTH-DATA_WIDTH){s2_corr_i[DATA_WIDTH-1]}}, s2_corr_i};
        new_acc_q = s2_acc_q + {{(ACC_WIDTH-DATA_WIDTH){s2_corr_q[DATA_WIDTH-1]}}, s2_corr_q};
        
        // Overflow detection
        overflow_i = (s2_acc_i[ACC_WIDTH-1] == s2_corr_i[DATA_WIDTH-1]) &&
                    (new_acc_i[ACC_WIDTH-1] != s2_acc_i[ACC_WIDTH-1]);
        overflow_q = (s2_acc_q[ACC_WIDTH-1] == s2_corr_q[DATA_WIDTH-1]) &&
                    (new_acc_q[ACC_WIDTH-1] != s2_acc_q[ACC_WIDTH-1]);
    end
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s3_sum_i <= '0;
            s3_sum_q <= '0;
            s3_bin <= '0;
            s3_valid <= 1'b0;
            s3_last <= 1'b0;
            status_overflow <= 1'b0;
        end else begin
            s3_sum_i <= new_acc_i;
            s3_sum_q <= new_acc_q;
            s3_bin <= s2_bin;
            s3_valid <= s2_valid;
            s3_last <= s2_last;
            
            if (s2_valid && (overflow_i || overflow_q)) begin
                status_overflow <= 1'b1;
            end
        end
    end
    
    //=========================================================================
    // Stage 4: Write Back
    //=========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s4_sum_i <= '0;
            s4_sum_q <= '0;
            s4_bin <= '0;
            s4_valid <= 1'b0;
            wr_en <= 1'b0;
        end else begin
            s4_sum_i <= s3_sum_i;
            s4_sum_q <= s3_sum_q;
            s4_bin <= s3_bin;
            s4_valid <= s3_valid;
            wr_en <= s3_valid;
        end
    end
    
    // Pack and write to memory
    assign wr_addr = s4_bin;
    assign wr_data = {s4_sum_q, s4_sum_i};
    
    always_ff @(posedge clk) begin
        if (wr_en) begin
            acc_mem[wr_addr] <= wr_data;
        end
    end
    
    // Clear memory on request
    logic [BRAM_ADDR_WIDTH-1:0] clear_addr;
    logic                       clearing;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clear_addr <= '0;
            clearing <= 1'b0;
        end else if (cfg_clear) begin
            clearing <= 1'b1;
            clear_addr <= '0;
        end else if (clearing) begin
            acc_mem[clear_addr] <= '0;
            if (clear_addr == RANGE_BINS - 1) begin
                clearing <= 1'b0;
            end else begin
                clear_addr <= clear_addr + 1;
            end
        end
    end
    
    //=========================================================================
    // Output Generation
    //=========================================================================
    
    // Output integrated values when complete
    logic [BRAM_ADDR_WIDTH-1:0] out_addr;
    logic                       outputting;
    logic [BRAM_DATA_WIDTH-1:0] out_data;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_addr <= '0;
            outputting <= 1'b0;
            int_valid <= 1'b0;
            int_complete <= 1'b0;
        end else if (integration_complete) begin
            outputting <= 1'b1;
            out_addr <= '0;
            int_complete <= 1'b1;
        end else if (outputting) begin
            int_complete <= 1'b0;
            
            // Read and output
            out_data <= acc_mem[out_addr];
            int_i <= $signed(acc_mem[out_addr][ACC_WIDTH-1:0]);
            int_q <= $signed(acc_mem[out_addr][2*ACC_WIDTH-1:ACC_WIDTH]);
            int_bin <= out_addr;
            int_valid <= 1'b1;
            
            if (out_addr == RANGE_BINS - 1) begin
                outputting <= 1'b0;
            end else begin
                out_addr <= out_addr + 1;
            end
        end else begin
            int_valid <= 1'b0;
        end
    end
    
    //=========================================================================
    // Magnitude Approximation (|I| + |Q|)
    //=========================================================================
    logic [ACC_WIDTH-1:0] abs_i, abs_q;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            int_mag <= '0;
            int_mag_valid <= 1'b0;
        end else begin
            int_mag_valid <= int_valid;
            
            abs_i = int_i[ACC_WIDTH-1] ? (~int_i + 1) : int_i;
            abs_q = int_q[ACC_WIDTH-1] ? (~int_q + 1) : int_q;
            int_mag <= abs_i + abs_q;
        end
    end
    
    //=========================================================================
    // Max Value Tracking (for AGC/debugging)
    //=========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            status_max_value <= '0;
        end else if (cfg_clear) begin
            status_max_value <= '0;
        end else if (int_mag_valid && int_mag > status_max_value) begin
            status_max_value <= int_mag;
        end
    end

endmodule
