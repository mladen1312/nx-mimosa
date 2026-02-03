//=============================================================================
// QEDMMA v3.0 - Digital Automatic Gain Control (AGC)
// [REQ-AGC-001] Prevent quantum receiver saturation
// [REQ-AGC-002] Fast attack / slow decay dynamics
// [REQ-AGC-003] Maintain optimal ADC dynamic range
//
// Author: Dr. Mladen Mešter
// Copyright (c) 2026 - All Rights Reserved
//
// Description:
//   Digital AGC for Rydberg quantum receiver frontend.
//   Protects sensitive quantum states from saturation while
//   maintaining optimal signal levels for PRBS correlation.
//
//   Features:
//   - Dual-slope dynamics: fast attack (1 µs), slow decay (100 ms)
//   - Power estimation via IIR filter
//   - Configurable target level and hysteresis
//   - Jammer blanking integration
//   - Gain word output for RF attenuator control
//
// Target: Xilinx Zynq UltraScale+ ZU47DR
// Clock: 200 MHz
//=============================================================================

`timescale 1ns / 1ps

module digital_agc #(
    parameter int DATA_WIDTH     = 16,        // I/Q sample width
    parameter int GAIN_WIDTH     = 12,        // Gain control word width
    parameter int POWER_WIDTH    = 32,        // Power accumulator width
    parameter int ATTACK_SHIFT   = 4,         // Attack time constant (2^N samples)
    parameter int DECAY_SHIFT    = 14         // Decay time constant (2^N samples)
)(
    input  logic                          clk,
    input  logic                          rst_n,
    
    //=========================================================================
    // ADC Input (I/Q samples from quantum receiver)
    //=========================================================================
    input  logic signed [DATA_WIDTH-1:0]  adc_i,
    input  logic signed [DATA_WIDTH-1:0]  adc_q,
    input  logic                          adc_valid,
    
    //=========================================================================
    // Gain-Adjusted Output
    //=========================================================================
    output logic signed [DATA_WIDTH-1:0]  out_i,
    output logic signed [DATA_WIDTH-1:0]  out_q,
    output logic                          out_valid,
    
    //=========================================================================
    // Gain Control Word (to RF attenuator DAC)
    //=========================================================================
    output logic [GAIN_WIDTH-1:0]         gain_word,
    output logic                          gain_valid,
    
    //=========================================================================
    // Status
    //=========================================================================
    output logic                          agc_locked,
    output logic                          saturation_flag,
    output logic                          weak_signal_flag,
    output logic [POWER_WIDTH-1:0]        power_estimate,
    
    //=========================================================================
    // ECCM Integration
    //=========================================================================
    input  logic                          jammer_detected,
    input  logic                          blank_enable,
    output logic                          agc_freeze,
    
    //=========================================================================
    // Configuration
    //=========================================================================
    input  logic [POWER_WIDTH-1:0]        cfg_target_power,
    input  logic [POWER_WIDTH-1:0]        cfg_max_power,
    input  logic [POWER_WIDTH-1:0]        cfg_min_power,
    input  logic [GAIN_WIDTH-1:0]         cfg_max_gain,
    input  logic [GAIN_WIDTH-1:0]         cfg_min_gain,
    input  logic [GAIN_WIDTH-1:0]         cfg_initial_gain,
    input  logic                          cfg_enable,
    input  logic                          cfg_freeze
);

    //=========================================================================
    // Local Parameters
    //=========================================================================
    localparam logic signed [DATA_WIDTH-1:0] SAT_THRESH_POS = (1 << (DATA_WIDTH-1)) - (1 << (DATA_WIDTH-4));
    localparam logic signed [DATA_WIDTH-1:0] SAT_THRESH_NEG = -SAT_THRESH_POS;
    localparam logic [GAIN_WIDTH-1:0] ATTACK_STEP = 12'h040;
    localparam logic [GAIN_WIDTH-1:0] DECAY_STEP  = 12'h002;
    
    //=========================================================================
    // Internal Signals
    //=========================================================================
    logic [2*DATA_WIDTH-1:0] power_inst;
    logic [POWER_WIDTH-1:0]  power_avg;
    logic [GAIN_WIDTH-1:0]   gain_reg;
    logic                    sat_detect;
    logic                    weak_detect;
    logic [7:0]              lock_counter;
    logic                    blank_active;
    logic [15:0]             blank_counter;
    logic signed [DATA_WIDTH-1:0] adc_i_d1, adc_q_d1;
    logic valid_d1, valid_d2;
    
    //=========================================================================
    // Saturation Detection
    //=========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sat_detect <= 1'b0;
        end else if (adc_valid) begin
            sat_detect <= (adc_i >= SAT_THRESH_POS) || (adc_i <= SAT_THRESH_NEG) ||
                         (adc_q >= SAT_THRESH_POS) || (adc_q <= SAT_THRESH_NEG);
        end
    end
    
    assign saturation_flag = sat_detect;
    
    //=========================================================================
    // Instantaneous Power (I² + Q²)
    //=========================================================================
    logic signed [2*DATA_WIDTH-1:0] i_squared, q_squared;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            i_squared <= '0;
            q_squared <= '0;
            power_inst <= '0;
        end else if (adc_valid) begin
            i_squared <= adc_i * adc_i;
            q_squared <= adc_q * adc_q;
            power_inst <= i_squared + q_squared;
        end
    end
    
    //=========================================================================
    // IIR Power Filter (Fast Attack / Slow Decay)
    //=========================================================================
    logic [POWER_WIDTH-1:0] power_diff;
    logic power_increasing;
    logic [3:0] filter_shift;
    
    always_comb begin
        power_increasing = ({16'b0, power_inst} > power_avg);
        filter_shift = power_increasing ? ATTACK_SHIFT[3:0] : DECAY_SHIFT[3:0];
        
        if (power_increasing)
            power_diff = {16'b0, power_inst} - power_avg;
        else
            power_diff = power_avg - {16'b0, power_inst};
    end
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            power_avg <= '0;
        end else if (valid_d1 && cfg_enable && !agc_freeze) begin
            if (power_increasing)
                power_avg <= power_avg + (power_diff >> filter_shift);
            else
                power_avg <= power_avg - (power_diff >> filter_shift);
        end
    end
    
    assign power_estimate = power_avg;
    
    //=========================================================================
    // Gain Control FSM
    //=========================================================================
    typedef enum logic [2:0] {
        AGC_INIT,
        AGC_TRACK,
        AGC_ATTACK,
        AGC_DECAY,
        AGC_FREEZE_ST
    } agc_state_t;
    
    agc_state_t state;
    
    logic [POWER_WIDTH-1:0] thresh_upper, thresh_lower;
    logic gain_increase, gain_decrease, power_in_range;
    
    always_comb begin
        thresh_upper = cfg_target_power + (cfg_target_power >> 3);
        thresh_lower = cfg_target_power - (cfg_target_power >> 3);
        gain_increase = (power_avg < thresh_lower) && !sat_detect;
        gain_decrease = (power_avg > thresh_upper) || sat_detect;
        power_in_range = (power_avg >= thresh_lower) && (power_avg <= thresh_upper);
    end
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= AGC_INIT;
            gain_reg <= '0;
            lock_counter <= '0;
        end else begin
            case (state)
                AGC_INIT: begin
                    gain_reg <= cfg_initial_gain;
                    lock_counter <= '0;
                    if (cfg_enable) state <= AGC_TRACK;
                end
                
                AGC_TRACK: begin
                    if (!cfg_enable) begin
                        state <= AGC_INIT;
                    end else if (cfg_freeze || blank_active) begin
                        state <= AGC_FREEZE_ST;
                    end else if (sat_detect || gain_decrease) begin
                        state <= AGC_ATTACK;
                    end else if (gain_increase) begin
                        state <= AGC_DECAY;
                    end else if (power_in_range) begin
                        if (lock_counter < 8'hFF)
                            lock_counter <= lock_counter + 1;
                    end
                end
                
                AGC_ATTACK: begin
                    // Fast gain reduction
                    if (gain_reg > cfg_min_gain + ATTACK_STEP)
                        gain_reg <= gain_reg - ATTACK_STEP;
                    else
                        gain_reg <= cfg_min_gain;
                    lock_counter <= '0;
                    
                    if (!cfg_enable) state <= AGC_INIT;
                    else if (cfg_freeze || blank_active) state <= AGC_FREEZE_ST;
                    else if (power_in_range) state <= AGC_TRACK;
                end
                
                AGC_DECAY: begin
                    // Slow gain increase
                    if (gain_reg < cfg_max_gain - DECAY_STEP)
                        gain_reg <= gain_reg + DECAY_STEP;
                    else
                        gain_reg <= cfg_max_gain;
                    lock_counter <= '0;
                    
                    if (!cfg_enable) state <= AGC_INIT;
                    else if (cfg_freeze || blank_active) state <= AGC_FREEZE_ST;
                    else if (sat_detect || gain_decrease) state <= AGC_ATTACK;
                    else if (power_in_range) state <= AGC_TRACK;
                end
                
                AGC_FREEZE_ST: begin
                    if (!cfg_enable) state <= AGC_INIT;
                    else if (!cfg_freeze && !blank_active) state <= AGC_TRACK;
                end
            endcase
        end
    end
    
    //=========================================================================
    // Blanking Control
    //=========================================================================
    localparam int BLANK_DURATION = 1000;  // 5 µs @ 200 MHz
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            blank_active <= 1'b0;
            blank_counter <= '0;
        end else begin
            if (blank_enable || jammer_detected) begin
                blank_active <= 1'b1;
                blank_counter <= BLANK_DURATION;
            end else if (blank_counter > 0) begin
                blank_counter <= blank_counter - 1;
            end else begin
                blank_active <= 1'b0;
            end
        end
    end
    
    assign agc_freeze = blank_active || cfg_freeze || (state == AGC_FREEZE_ST);
    
    //=========================================================================
    // Weak Signal Detection
    //=========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            weak_detect <= 1'b0;
        end else begin
            weak_detect <= (power_avg < cfg_min_power) && (gain_reg >= cfg_max_gain - DECAY_STEP);
        end
    end
    
    assign weak_signal_flag = weak_detect;
    assign agc_locked = (lock_counter >= 8'h80) && power_in_range;
    
    //=========================================================================
    // Gain Application (Digital Scaling)
    //=========================================================================
    logic signed [DATA_WIDTH+GAIN_WIDTH-1:0] scaled_i, scaled_q;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scaled_i <= '0;
            scaled_q <= '0;
            adc_i_d1 <= '0;
            adc_q_d1 <= '0;
            valid_d1 <= 1'b0;
            valid_d2 <= 1'b0;
        end else begin
            adc_i_d1 <= adc_i;
            adc_q_d1 <= adc_q;
            valid_d1 <= adc_valid;
            valid_d2 <= valid_d1;
            
            if (cfg_enable) begin
                scaled_i <= (adc_i_d1 * $signed({1'b0, gain_reg})) >>> 8;
                scaled_q <= (adc_q_d1 * $signed({1'b0, gain_reg})) >>> 8;
            end else begin
                scaled_i <= {{(GAIN_WIDTH){adc_i_d1[DATA_WIDTH-1]}}, adc_i_d1};
                scaled_q <= {{(GAIN_WIDTH){adc_q_d1[DATA_WIDTH-1]}}, adc_q_d1};
            end
        end
    end
    
    // Output with saturation
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_i <= '0;
            out_q <= '0;
            out_valid <= 1'b0;
        end else begin
            out_valid <= valid_d2;
            
            // Saturate I
            if (scaled_i > $signed({{(GAIN_WIDTH){1'b0}}, {(DATA_WIDTH-1){1'b1}}}))
                out_i <= {1'b0, {(DATA_WIDTH-1){1'b1}}};
            else if (scaled_i < $signed({{(GAIN_WIDTH){1'b1}}, 1'b0, {(DATA_WIDTH-1){1'b0}}}))
                out_i <= {1'b1, {(DATA_WIDTH-1){1'b0}}};
            else
                out_i <= scaled_i[DATA_WIDTH-1:0];
            
            // Saturate Q
            if (scaled_q > $signed({{(GAIN_WIDTH){1'b0}}, {(DATA_WIDTH-1){1'b1}}}))
                out_q <= {1'b0, {(DATA_WIDTH-1){1'b1}}};
            else if (scaled_q < $signed({{(GAIN_WIDTH){1'b1}}, 1'b0, {(DATA_WIDTH-1){1'b0}}}))
                out_q <= {1'b1, {(DATA_WIDTH-1){1'b0}}};
            else
                out_q <= scaled_q[DATA_WIDTH-1:0];
        end
    end
    
    //=========================================================================
    // Gain Output to RF Attenuator
    //=========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gain_word <= '0;
            gain_valid <= 1'b0;
        end else begin
            gain_word <= cfg_max_gain - gain_reg;
            gain_valid <= (state != AGC_INIT);
        end
    end

endmodule
