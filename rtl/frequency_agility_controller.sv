//═══════════════════════════════════════════════════════════════════════════════
// NX-MIMOSA Frequency Agility Controller
// Auto-generated RTL for DRFM RGPO Countermeasure
// Target: Xilinx RFSoC ZU48DR
//═══════════════════════════════════════════════════════════════════════════════

`timescale 1ns / 1ps

module frequency_agility_controller #(
    parameter int N_CHANNELS = 64,
    parameter int CHANNEL_BITS = $clog2(N_CHANNELS),
    parameter int SEED = 32'h12345678,
    parameter int MIN_HOP_DISTANCE = 8,
    parameter int PATTERN_LENGTH = 1024
)(
    input  logic        clk,                    // System clock (250 MHz)
    input  logic        rst_n,                  // Active-low reset
    input  logic        enable,                 // Enable frequency hopping
    input  logic        hop_trigger,            // Trigger next hop
    input  logic [31:0] new_seed,               // New seed for pattern
    input  logic        load_seed,              // Load new seed
    
    // Frequency output
    output logic [CHANNEL_BITS-1:0] channel_out,    // Current channel
    output logic [47:0] frequency_out,              // Frequency in Hz (fixed-point)
    output logic        channel_valid,              // Channel output valid
    output logic        pll_update,                 // PLL update strobe
    
    // Status
    output logic [31:0] hop_count,
    output logic        pattern_wrap
);

    //═══════════════════════════════════════════════════════════════════════════
    // PARAMETERS
    //═══════════════════════════════════════════════════════════════════════════
    
    localparam logic [47:0] CENTER_FREQ = 48'd3000000000;     // 3.000 GHz
    localparam logic [47:0] BANDWIDTH   = 48'd500000000;            // 500 MHz
    localparam logic [47:0] CHAN_SPACE  = BANDWIDTH / N_CHANNELS;
    localparam logic [47:0] FREQ_START  = CENTER_FREQ - (BANDWIDTH >> 1);
    
    // LFSR polynomial: x^32 + x^22 + x^2 + x + 1
    localparam logic [31:0] LFSR_POLY = 32'h80200003;
    
    //═══════════════════════════════════════════════════════════════════════════
    // STATE
    //═══════════════════════════════════════════════════════════════════════════
    
    typedef enum logic [2:0] {
        IDLE,
        GENERATE,
        VALIDATE,
        OUTPUT,
        WAIT_PLL
    } state_t;
    
    state_t state, next_state;
    
    logic [31:0] lfsr_reg;
    logic [CHANNEL_BITS-1:0] current_channel;
    logic [CHANNEL_BITS-1:0] last_channel;
    logic [31:0] hop_counter;
    logic [15:0] pattern_index;
    
    // PLL settling counter
    logic [7:0] pll_settle_cnt;
    localparam int PLL_SETTLE_CYCLES = 500;  // @ 250 MHz
    
    //═══════════════════════════════════════════════════════════════════════════
    // LFSR NEXT STATE
    //═══════════════════════════════════════════════════════════════════════════
    
    function automatic logic [31:0] lfsr_next(input logic [31:0] current);
        logic [31:0] next_val;
        logic feedback;
        
        feedback = current[0];
        next_val = current >> 1;
        
        if (feedback)
            next_val = next_val ^ LFSR_POLY;
        
        return next_val;
    endfunction
    
    //═══════════════════════════════════════════════════════════════════════════
    // CHANNEL COMPUTATION
    //═══════════════════════════════════════════════════════════════════════════
    
    logic [CHANNEL_BITS-1:0] raw_channel;
    logic [CHANNEL_BITS-1:0] adjusted_channel;
    logic channel_valid_int;
    
    // Extract channel from LFSR (modulo N_CHANNELS)
    assign raw_channel = lfsr_reg[CHANNEL_BITS-1:0];
    
    // Check hop distance constraint
    logic [CHANNEL_BITS:0] hop_distance;
    assign hop_distance = (raw_channel > last_channel) ? 
                          (raw_channel - last_channel) :
                          (last_channel - raw_channel);
    
    logic distance_ok;
    assign distance_ok = (hop_distance >= MIN_HOP_DISTANCE) || 
                         (hop_distance <= (N_CHANNELS - MIN_HOP_DISTANCE));
    
    // Adjust channel if needed
    always_comb begin
        if (distance_ok)
            adjusted_channel = raw_channel;
        else
            adjusted_channel = (last_channel + MIN_HOP_DISTANCE) % N_CHANNELS;
    end
    
    //═══════════════════════════════════════════════════════════════════════════
    // FREQUENCY COMPUTATION
    //═══════════════════════════════════════════════════════════════════════════
    
    logic [47:0] freq_computed;
    assign freq_computed = FREQ_START + ({(48-CHANNEL_BITS){1'b0}}, current_channel} * CHAN_SPACE);
    
    //═══════════════════════════════════════════════════════════════════════════
    // STATE MACHINE
    //═══════════════════════════════════════════════════════════════════════════
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            lfsr_reg <= SEED;
            current_channel <= '0;
            last_channel <= '0;
            hop_counter <= '0;
            pattern_index <= '0;
            channel_valid <= 1'b0;
            pll_update <= 1'b0;
            pll_settle_cnt <= '0;
            frequency_out <= CENTER_FREQ;
        end else begin
            state <= next_state;
            
            case (state)
                IDLE: begin
                    channel_valid <= 1'b0;
                    pll_update <= 1'b0;
                    
                    if (load_seed) begin
                        lfsr_reg <= new_seed;
                        pattern_index <= '0;
                    end
                end
                
                GENERATE: begin
                    // Advance LFSR
                    lfsr_reg <= lfsr_next(lfsr_reg);
                end
                
                VALIDATE: begin
                    // Apply constraints and compute final channel
                    current_channel <= adjusted_channel;
                end
                
                OUTPUT: begin
                    // Output new frequency
                    last_channel <= current_channel;
                    frequency_out <= freq_computed;
                    channel_valid <= 1'b1;
                    pll_update <= 1'b1;
                    hop_counter <= hop_counter + 1;
                    pattern_index <= (pattern_index == PATTERN_LENGTH-1) ? '0 : pattern_index + 1;
                    pll_settle_cnt <= '0;
                end
                
                WAIT_PLL: begin
                    pll_update <= 1'b0;
                    pll_settle_cnt <= pll_settle_cnt + 1;
                end
            endcase
        end
    end
    
    // Next state logic
    always_comb begin
        next_state = state;
        
        case (state)
            IDLE: begin
                if (enable && hop_trigger)
                    next_state = GENERATE;
            end
            
            GENERATE: begin
                next_state = VALIDATE;
            end
            
            VALIDATE: begin
                next_state = OUTPUT;
            end
            
            OUTPUT: begin
                next_state = WAIT_PLL;
            end
            
            WAIT_PLL: begin
                if (pll_settle_cnt >= PLL_SETTLE_CYCLES)
                    next_state = IDLE;
            end
        endcase
    end
    
    //═══════════════════════════════════════════════════════════════════════════
    // OUTPUTS
    //═══════════════════════════════════════════════════════════════════════════
    
    assign channel_out = current_channel;
    assign hop_count = hop_counter;
    assign pattern_wrap = (pattern_index == '0) && (hop_counter > 0);

endmodule

//═══════════════════════════════════════════════════════════════════════════════
// RGPO Detection Module
//═══════════════════════════════════════════════════════════════════════════════

module rgpo_detector #(
    parameter int HISTORY_DEPTH = 8,
    parameter int RANGE_BITS = 24,
    parameter int THRESHOLD_M = 50          // Detection threshold in meters
)(
    input  logic                    clk,
    input  logic                    rst_n,
    input  logic                    sample_valid,
    input  logic [RANGE_BITS-1:0]   range_in,           // Range in cm
    input  logic [15:0]             doppler_in,         // Doppler in Hz (signed)
    input  logic                    freq_match,         // Frequency matched expected
    
    output logic                    rgpo_detected,
    output logic                    jammer_present,
    output logic [7:0]              confidence          // 0-255
);

    //═══════════════════════════════════════════════════════════════════════════
    // RANGE HISTORY
    //═══════════════════════════════════════════════════════════════════════════
    
    logic [RANGE_BITS-1:0] range_history [HISTORY_DEPTH-1:0];
    logic [$clog2(HISTORY_DEPTH)-1:0] history_ptr;
    logic [3:0] valid_samples;
    
    // Range rate computation (delta range per sample)
    logic signed [RANGE_BITS:0] range_rate [HISTORY_DEPTH-2:0];
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < HISTORY_DEPTH; i++)
                range_history[i] <= '0;
            history_ptr <= '0;
            valid_samples <= '0;
        end else if (sample_valid) begin
            range_history[history_ptr] <= range_in;
            history_ptr <= (history_ptr == HISTORY_DEPTH-1) ? '0 : history_ptr + 1;
            if (valid_samples < HISTORY_DEPTH)
                valid_samples <= valid_samples + 1;
        end
    end
    
    // Compute range rates
    always_comb begin
        for (int i = 0; i < HISTORY_DEPTH-1; i++) begin
            int next_idx = (i + 1) % HISTORY_DEPTH;
            range_rate[i] = $signed(range_history[next_idx]) - $signed(range_history[i]);
        end
    end
    
    //═══════════════════════════════════════════════════════════════════════════
    // RGPO DETECTION LOGIC
    //═══════════════════════════════════════════════════════════════════════════
    
    // RGPO signature: consistent positive range rate
    logic rates_positive;
    logic rates_consistent;
    logic rate_above_threshold;
    
    always_comb begin
        rates_positive = 1'b1;
        rates_consistent = 1'b1;
        rate_above_threshold = 1'b0;
        
        // Check if all rates are positive (moving away)
        for (int i = 0; i < HISTORY_DEPTH-1; i++) begin
            if (range_rate[i] <= 0)
                rates_positive = 1'b0;
            
            // Check rate magnitude (> threshold)
            if (range_rate[i] > THRESHOLD_M * 100)  // cm threshold
                rate_above_threshold = 1'b1;
        end
        
        // Check consistency (low variance)
        // Simplified: check if all rates are similar
        for (int i = 0; i < HISTORY_DEPTH-2; i++) begin
            logic signed [RANGE_BITS:0] diff;
            diff = range_rate[i+1] - range_rate[i];
            if (diff > 1000 || diff < -1000)  // 10m variance threshold
                rates_consistent = 1'b0;
        end
    end
    
    // Detection outputs
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rgpo_detected <= 1'b0;
            jammer_present <= 1'b0;
            confidence <= 8'd0;
        end else begin
            // RGPO detected if: positive rates, consistent, above threshold
            rgpo_detected <= (valid_samples >= 4) && 
                            rates_positive && 
                            rates_consistent && 
                            rate_above_threshold;
            
            // Jammer present if frequency mismatch or RGPO
            jammer_present <= !freq_match || rgpo_detected;
            
            // Confidence computation
            if (!freq_match)
                confidence <= 8'd32;   // Low confidence
            else if (rgpo_detected)
                confidence <= 8'd64;   // Medium confidence
            else
                confidence <= 8'd255;  // High confidence
        end
    end

endmodule

