// ═══════════════════════════════════════════════════════════════════════════════════════════════════
// NX-MIMOSA 7D AIE Policy Wrapper for Versal AI Core
// ═══════════════════════════════════════════════════════════════════════════════════════════════════
//
// SystemVerilog wrapper for 7D Cognitive ECCM policy on Versal AIE-ML
// 
// Integration:
//   - PL-side: AXI-Stream interface for state/action
//   - AIE-side: INT8 quantized neural network inference
//   - Latency: <120 ns end-to-end @ 600 MHz
//
// Traceability:
//   [REQ-RL-7D-001] 7D waveform agility
//   [REQ-DEPLOY-7D-001] Versal AIE deployment
//   [REQ-EW-KRASUKHA-001] Krasukha countermeasures
//   [REQ-EW-GROWLER-001] Growler/NGJ countermeasures
//
// Author: Dr. Mladen Mešter / Nexellum d.o.o.
// Version: 2.0.0
// License: AGPL v3 / Commercial
// ═══════════════════════════════════════════════════════════════════════════════════════════════════

`timescale 1ns/1ps
`default_nettype none

module aie_7d_policy_wrapper #(
    // Configuration
    parameter int STATE_DIM      = 10,      // State vector dimension
    parameter int ACTION_DIM     = 7,       // 7D action output
    parameter int DATA_WIDTH     = 16,      // Q8.8 fixed-point
    parameter int AIE_DATA_WIDTH = 128,     // AIE interface width (4x32b)
    parameter int FIFO_DEPTH     = 4        // Input/output FIFO depth
)(
    // Clock and reset
    input  wire                         clk,
    input  wire                         rst_n,
    
    // ═══════════════════════════════════════════════════════════════════════════
    // State Input Interface (AXI-Stream from PL)
    // ═══════════════════════════════════════════════════════════════════════════
    input  wire [DATA_WIDTH-1:0]        s_state_tdata [STATE_DIM-1:0],
    input  wire                         s_state_tvalid,
    output logic                        s_state_tready,
    
    // ═══════════════════════════════════════════════════════════════════════════
    // Action Output Interface (AXI-Stream to Waveform Generator)
    // ═══════════════════════════════════════════════════════════════════════════
    output logic [DATA_WIDTH-1:0]       m_action_tdata [ACTION_DIM-1:0],
    output logic                        m_action_tvalid,
    input  wire                         m_action_tready,
    
    // ═══════════════════════════════════════════════════════════════════════════
    // AIE Interface (PL-AIE Bridge)
    // ═══════════════════════════════════════════════════════════════════════════
    output logic [AIE_DATA_WIDTH-1:0]   aie_to_pl_tdata,
    output logic                        aie_to_pl_tvalid,
    input  wire                         aie_to_pl_tready,
    
    input  wire [AIE_DATA_WIDTH-1:0]    pl_to_aie_tdata,
    input  wire                         pl_to_aie_tvalid,
    output logic                        pl_to_aie_tready,
    
    // ═══════════════════════════════════════════════════════════════════════════
    // Status & Debug
    // ═══════════════════════════════════════════════════════════════════════════
    output logic [31:0]                 stat_inference_count,
    output logic [15:0]                 stat_latency_cycles,
    output logic                        stat_busy,
    output logic [2:0]                  dbg_fsm_state
);

    // ═══════════════════════════════════════════════════════════════════════════
    // Local Parameters
    // ═══════════════════════════════════════════════════════════════════════════
    localparam int STATE_WORDS  = (STATE_DIM * DATA_WIDTH + AIE_DATA_WIDTH - 1) / AIE_DATA_WIDTH;
    localparam int ACTION_WORDS = (ACTION_DIM * DATA_WIDTH + AIE_DATA_WIDTH - 1) / AIE_DATA_WIDTH;
    
    // FSM States
    typedef enum logic [2:0] {
        ST_IDLE         = 3'b000,
        ST_PACK_STATE   = 3'b001,
        ST_SEND_TO_AIE  = 3'b010,
        ST_WAIT_AIE     = 3'b011,
        ST_RECV_ACTION  = 3'b100,
        ST_UNPACK       = 3'b101,
        ST_OUTPUT       = 3'b110
    } fsm_state_t;
    
    fsm_state_t state, next_state;
    
    // ═══════════════════════════════════════════════════════════════════════════
    // Internal Signals
    // ═══════════════════════════════════════════════════════════════════════════
    
    // State packing buffer
    logic [DATA_WIDTH-1:0] state_buffer [STATE_DIM-1:0];
    logic [AIE_DATA_WIDTH-1:0] packed_state [STATE_WORDS-1:0];
    logic [$clog2(STATE_WORDS):0] pack_cnt;
    
    // Action unpacking buffer
    logic [AIE_DATA_WIDTH-1:0] packed_action [ACTION_WORDS-1:0];
    logic [DATA_WIDTH-1:0] action_buffer [ACTION_DIM-1:0];
    logic [$clog2(ACTION_WORDS):0] unpack_cnt;
    
    // Latency measurement
    logic [15:0] latency_counter;
    
    // ═══════════════════════════════════════════════════════════════════════════
    // FSM: Sequential Logic
    // ═══════════════════════════════════════════════════════════════════════════
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
        end else begin
            state <= next_state;
        end
    end
    
    // ═══════════════════════════════════════════════════════════════════════════
    // FSM: Next State Logic
    // ═══════════════════════════════════════════════════════════════════════════
    always_comb begin
        next_state = state;
        
        case (state)
            ST_IDLE: begin
                if (s_state_tvalid) begin
                    next_state = ST_PACK_STATE;
                end
            end
            
            ST_PACK_STATE: begin
                next_state = ST_SEND_TO_AIE;
            end
            
            ST_SEND_TO_AIE: begin
                if (aie_to_pl_tready && pack_cnt >= STATE_WORDS) begin
                    next_state = ST_WAIT_AIE;
                end
            end
            
            ST_WAIT_AIE: begin
                if (pl_to_aie_tvalid) begin
                    next_state = ST_RECV_ACTION;
                end
            end
            
            ST_RECV_ACTION: begin
                if (unpack_cnt >= ACTION_WORDS) begin
                    next_state = ST_UNPACK;
                end
            end
            
            ST_UNPACK: begin
                next_state = ST_OUTPUT;
            end
            
            ST_OUTPUT: begin
                if (m_action_tready) begin
                    next_state = ST_IDLE;
                end
            end
            
            default: next_state = ST_IDLE;
        endcase
    end
    
    // ═══════════════════════════════════════════════════════════════════════════
    // State Capture & Packing
    // ═══════════════════════════════════════════════════════════════════════════
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < STATE_DIM; i++) begin
                state_buffer[i] <= '0;
            end
            pack_cnt <= '0;
        end else begin
            case (state)
                ST_IDLE: begin
                    if (s_state_tvalid) begin
                        // Capture input state
                        for (int i = 0; i < STATE_DIM; i++) begin
                            state_buffer[i] <= s_state_tdata[i];
                        end
                    end
                    pack_cnt <= '0;
                end
                
                ST_PACK_STATE: begin
                    // Pack state into 128-bit words for AIE
                    // State: 10 x 16-bit = 160 bits → 2 x 128-bit words
                    packed_state[0] <= {state_buffer[7], state_buffer[6], state_buffer[5], 
                                        state_buffer[4], state_buffer[3], state_buffer[2],
                                        state_buffer[1], state_buffer[0]};
                    packed_state[1] <= {48'b0, state_buffer[9], state_buffer[8]};
                end
                
                ST_SEND_TO_AIE: begin
                    if (aie_to_pl_tready) begin
                        pack_cnt <= pack_cnt + 1;
                    end
                end
                
                default: ;
            endcase
        end
    end
    
    // ═══════════════════════════════════════════════════════════════════════════
    // Action Reception & Unpacking
    // ═══════════════════════════════════════════════════════════════════════════
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < ACTION_DIM; i++) begin
                action_buffer[i] <= '0;
            end
            unpack_cnt <= '0;
        end else begin
            case (state)
                ST_WAIT_AIE: begin
                    unpack_cnt <= '0;
                end
                
                ST_RECV_ACTION: begin
                    if (pl_to_aie_tvalid) begin
                        packed_action[unpack_cnt] <= pl_to_aie_tdata;
                        unpack_cnt <= unpack_cnt + 1;
                    end
                end
                
                ST_UNPACK: begin
                    // Unpack 128-bit words to 7 x 16-bit actions
                    // Action: 7 x 16-bit = 112 bits → 1 x 128-bit word
                    action_buffer[0] <= packed_action[0][15:0];
                    action_buffer[1] <= packed_action[0][31:16];
                    action_buffer[2] <= packed_action[0][47:32];
                    action_buffer[3] <= packed_action[0][63:48];
                    action_buffer[4] <= packed_action[0][79:64];
                    action_buffer[5] <= packed_action[0][95:80];
                    action_buffer[6] <= packed_action[0][111:96];
                end
                
                default: ;
            endcase
        end
    end
    
    // ═══════════════════════════════════════════════════════════════════════════
    // Latency Counter
    // ═══════════════════════════════════════════════════════════════════════════
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            latency_counter <= '0;
            stat_latency_cycles <= '0;
        end else begin
            case (state)
                ST_IDLE: begin
                    latency_counter <= '0;
                end
                
                ST_OUTPUT: begin
                    if (m_action_tready) begin
                        stat_latency_cycles <= latency_counter;
                    end
                end
                
                default: begin
                    latency_counter <= latency_counter + 1;
                end
            endcase
        end
    end
    
    // ═══════════════════════════════════════════════════════════════════════════
    // Inference Counter
    // ═══════════════════════════════════════════════════════════════════════════
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stat_inference_count <= '0;
        end else begin
            if (state == ST_OUTPUT && m_action_tready) begin
                stat_inference_count <= stat_inference_count + 1;
            end
        end
    end
    
    // ═══════════════════════════════════════════════════════════════════════════
    // Output Assignments
    // ═══════════════════════════════════════════════════════════════════════════
    
    // State input ready
    assign s_state_tready = (state == ST_IDLE);
    
    // AIE interface
    assign aie_to_pl_tdata = (pack_cnt < STATE_WORDS) ? packed_state[pack_cnt] : '0;
    assign aie_to_pl_tvalid = (state == ST_SEND_TO_AIE);
    assign pl_to_aie_tready = (state == ST_RECV_ACTION);
    
    // Action output
    always_comb begin
        for (int i = 0; i < ACTION_DIM; i++) begin
            m_action_tdata[i] = action_buffer[i];
        end
    end
    assign m_action_tvalid = (state == ST_OUTPUT);
    
    // Status
    assign stat_busy = (state != ST_IDLE);
    assign dbg_fsm_state = state;

endmodule


// ═══════════════════════════════════════════════════════════════════════════════════════════════════
// 7D Waveform Generator Controller
// ═══════════════════════════════════════════════════════════════════════════════════════════════════
//
// Converts 7D action vector to waveform generator control signals
//
// [REQ-RL-7D-001] Maps RL actions to physical waveform parameters

module waveform_7d_controller #(
    parameter int DATA_WIDTH     = 16,      // Q8.8 fixed-point
    parameter int ACTION_DIM     = 7,
    
    // Physical parameter ranges
    parameter int FREQ_CENTER_MHZ = 10000,  // 10 GHz center
    parameter int FREQ_RANGE_MHZ  = 500,    // ±500 MHz
    parameter int PRF_CENTER_HZ   = 3000,   // 3 kHz center
    parameter int BW_CENTER_MHZ   = 50      // 50 MHz bandwidth
)(
    input  wire                         clk,
    input  wire                         rst_n,
    
    // Action input (from AIE policy)
    input  wire [DATA_WIDTH-1:0]        action_in [ACTION_DIM-1:0],
    input  wire                         action_valid,
    output logic                        action_ready,
    
    // ═══════════════════════════════════════════════════════════════════════════
    // Waveform Generator Control Outputs
    // ═══════════════════════════════════════════════════════════════════════════
    
    // Frequency control (to NCO/DDS)
    output logic [31:0]                 freq_word,           // Frequency tuning word
    output logic                        freq_valid,
    
    // PRF control
    output logic [15:0]                 prf_period,          // PRF period in clock cycles
    output logic                        prf_valid,
    
    // Bandwidth/chirp control
    output logic [23:0]                 chirp_rate,          // LFM chirp rate
    output logic [15:0]                 pulse_width,         // Pulse width
    output logic                        chirp_valid,
    
    // Power control (to PA)
    output logic [7:0]                  power_level,         // DAC code for PA
    output logic                        power_valid,
    
    // Polarization control (to feed network)
    output logic [1:0]                  pol_state,           // 00=H, 01=V, 10=RHCP, 11=LHCP
    output logic [7:0]                  pol_phase_h,         // H-port phase
    output logic [7:0]                  pol_phase_v,         // V-port phase
    output logic                        pol_valid,
    
    // Phase control (for DRFM mitigation)
    output logic [15:0]                 phase_offset,        // Phase offset (0-65535 = 0-360°)
    output logic                        phase_valid,
    
    // Code selection (7D - waveform coding)
    output logic [2:0]                  code_select,         // 0-7 code types
    output logic [7:0]                  code_param,          // Code-specific parameter
    output logic                        code_valid,
    
    // Combined update strobe
    output logic                        waveform_update
);

    // ═══════════════════════════════════════════════════════════════════════════
    // Local Parameters
    // ═══════════════════════════════════════════════════════════════════════════
    
    // Fixed-point Q8.8 conversion
    localparam int Q_FRAC = 8;
    
    // Frequency tuning word calculation (for 600 MHz sample clock)
    // FTW = (f_out / f_clk) * 2^32
    localparam real FREQ_SCALE = 4294967296.0 / 600e6;  // 2^32 / 600 MHz
    
    // ═══════════════════════════════════════════════════════════════════════════
    // Pipeline Stage
    // ═══════════════════════════════════════════════════════════════════════════
    logic signed [DATA_WIDTH-1:0] action_reg [ACTION_DIM-1:0];
    logic action_pending;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            action_pending <= 1'b0;
            for (int i = 0; i < ACTION_DIM; i++) begin
                action_reg[i] <= '0;
            end
        end else begin
            if (action_valid && action_ready) begin
                for (int i = 0; i < ACTION_DIM; i++) begin
                    action_reg[i] <= action_in[i];
                end
                action_pending <= 1'b1;
            end else if (waveform_update) begin
                action_pending <= 1'b0;
            end
        end
    end
    
    assign action_ready = !action_pending;
    
    // ═══════════════════════════════════════════════════════════════════════════
    // Action to Waveform Parameter Conversion
    // ═══════════════════════════════════════════════════════════════════════════
    
    // Helper: Q8.8 to scaled integer
    function automatic logic signed [31:0] q8_to_scaled(
        input logic signed [15:0] q8_val,
        input logic signed [31:0] center,
        input logic signed [31:0] range
    );
        logic signed [31:0] scaled;
        scaled = (q8_val * range) >>> Q_FRAC;
        return center + scaled;
    endfunction
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            freq_word     <= 32'd0;
            prf_period    <= 16'd0;
            chirp_rate    <= 24'd0;
            pulse_width   <= 16'd0;
            power_level   <= 8'd128;
            pol_state     <= 2'b00;
            pol_phase_h   <= 8'd0;
            pol_phase_v   <= 8'd0;
            phase_offset  <= 16'd0;
            code_select   <= 3'd0;
            code_param    <= 8'd0;
            
            freq_valid    <= 1'b0;
            prf_valid     <= 1'b0;
            chirp_valid   <= 1'b0;
            power_valid   <= 1'b0;
            pol_valid     <= 1'b0;
            phase_valid   <= 1'b0;
            code_valid    <= 1'b0;
            waveform_update <= 1'b0;
            
        end else if (action_pending) begin
            
            // ═══════════════════════════════════════════════════════════════════
            // ACTION 0: Frequency offset
            // ═══════════════════════════════════════════════════════════════════
            // Input: [-128, 127] Q8.8 → [-0.5, 0.5]
            // Output: freq_center ± range MHz
            begin
                logic signed [31:0] freq_mhz;
                freq_mhz = q8_to_scaled(action_reg[0], FREQ_CENTER_MHZ, FREQ_RANGE_MHZ);
                freq_word <= freq_mhz * 32'd7158;  // Approximate FTW scaling
            end
            freq_valid <= 1'b1;
            
            // ═══════════════════════════════════════════════════════════════════
            // ACTION 1: PRF scaling
            // ═══════════════════════════════════════════════════════════════════
            // Input: [-0.5, 0.5] → PRF_CENTER * (1 ± 0.5)
            begin
                logic signed [31:0] prf_hz;
                prf_hz = q8_to_scaled(action_reg[1], PRF_CENTER_HZ, PRF_CENTER_HZ / 2);
                // Convert to period (cycles @ 600 MHz)
                prf_period <= 16'd600000 / prf_hz[15:0];  // Simplified
            end
            prf_valid <= 1'b1;
            
            // ═══════════════════════════════════════════════════════════════════
            // ACTION 2: Bandwidth/chirp rate
            // ═══════════════════════════════════════════════════════════════════
            begin
                logic signed [31:0] bw_mhz;
                bw_mhz = q8_to_scaled(action_reg[2], BW_CENTER_MHZ, BW_CENTER_MHZ / 2);
                chirp_rate <= bw_mhz[23:0] * 24'd100;  // Chirp rate scaling
                pulse_width <= 16'd1000;  // Fixed for now
            end
            chirp_valid <= 1'b1;
            
            // ═══════════════════════════════════════════════════════════════════
            // ACTION 3: Power level
            // ═══════════════════════════════════════════════════════════════════
            begin
                logic signed [15:0] power_delta;
                power_delta = action_reg[3] >>> 1;  // Scale down
                power_level <= 8'd128 + power_delta[7:0];  // Center at 128
            end
            power_valid <= 1'b1;
            
            // ═══════════════════════════════════════════════════════════════════
            // ACTION 4: Polarization
            // ═══════════════════════════════════════════════════════════════════
            // Map continuous to 4 states: H, V, RHCP, LHCP
            begin
                logic [1:0] pol_idx;
                pol_idx = (action_reg[4] + 16'sd128) >> 6;  // Quantize to 4 levels
                pol_state <= pol_idx;
                
                // Phase for circular polarization
                case (pol_idx)
                    2'b00: begin pol_phase_h <= 8'd0;   pol_phase_v <= 8'd0;   end  // H
                    2'b01: begin pol_phase_h <= 8'd0;   pol_phase_v <= 8'd0;   end  // V
                    2'b10: begin pol_phase_h <= 8'd0;   pol_phase_v <= 8'd64;  end  // RHCP (+90°)
                    2'b11: begin pol_phase_h <= 8'd0;   pol_phase_v <= 8'd192; end  // LHCP (-90°)
                endcase
            end
            pol_valid <= 1'b1;
            
            // ═══════════════════════════════════════════════════════════════════
            // ACTION 5: Phase offset
            // ═══════════════════════════════════════════════════════════════════
            // Full 360° range
            begin
                phase_offset <= (action_reg[5] + 16'sd128) << 8;  // Scale to 0-65535
            end
            phase_valid <= 1'b1;
            
            // ═══════════════════════════════════════════════════════════════════
            // ACTION 6: Code selection (7D)
            // ═══════════════════════════════════════════════════════════════════
            // 8 waveform codes: LFM variants, Barker, polyphase
            begin
                logic [2:0] code_idx;
                code_idx = (action_reg[6] + 16'sd128) >> 5;  // Quantize to 8 codes
                code_select <= code_idx;
                
                // Code-specific parameter (e.g., Barker length, polyphase order)
                case (code_idx)
                    3'd0: code_param <= 8'd13;  // Barker-13
                    3'd1: code_param <= 8'd7;   // Barker-7
                    3'd2: code_param <= 8'd16;  // Frank-16
                    3'd3: code_param <= 8'd16;  // P1-16
                    3'd4: code_param <= 8'd16;  // P2-16
                    3'd5: code_param <= 8'd5;   // Zadoff-Chu root 5
                    3'd6: code_param <= 8'd7;   // Zadoff-Chu root 7
                    3'd7: code_param <= 8'd0;   // Random (dynamic)
                endcase
            end
            code_valid <= 1'b1;
            
            // Combined update strobe
            waveform_update <= 1'b1;
            
        end else begin
            freq_valid <= 1'b0;
            prf_valid <= 1'b0;
            chirp_valid <= 1'b0;
            power_valid <= 1'b0;
            pol_valid <= 1'b0;
            phase_valid <= 1'b0;
            code_valid <= 1'b0;
            waveform_update <= 1'b0;
        end
    end

endmodule


// ═══════════════════════════════════════════════════════════════════════════════════════════════════
// Top-Level 7D Cognitive ECCM Pipeline
// ═══════════════════════════════════════════════════════════════════════════════════════════════════

module top_cognitive_eccm_7d #(
    parameter int DATA_WIDTH = 16,
    parameter int STATE_DIM  = 10,
    parameter int ACTION_DIM = 7
)(
    input  wire                         clk_600mhz,
    input  wire                         rst_n,
    
    // Jammer features from classifier (PL)
    input  wire [31:0]                  feat_jnr,
    input  wire [31:0]                  feat_temporal_var,
    input  wire [31:0]                  feat_spectral_var,
    input  wire [31:0]                  feat_prf_stability,
    input  wire [1:0]                   jammer_class,
    input  wire                         features_valid,
    
    // Radar state feedback
    input  wire [DATA_WIDTH-1:0]        radar_freq_current,
    input  wire [DATA_WIDTH-1:0]        radar_prf_current,
    input  wire [DATA_WIDTH-1:0]        radar_bw_current,
    
    // Waveform generator outputs
    output wire [31:0]                  freq_word,
    output wire                         freq_valid,
    output wire [15:0]                  prf_period,
    output wire                         prf_valid,
    output wire [23:0]                  chirp_rate,
    output wire                         chirp_valid,
    output wire [7:0]                   power_level,
    output wire                         power_valid,
    output wire [1:0]                   pol_state,
    output wire                         pol_valid,
    output wire [15:0]                  phase_offset,
    output wire                         phase_valid,
    output wire [2:0]                   code_select,
    output wire                         code_valid,
    output wire                         waveform_update,
    
    // AIE interface
    output wire [127:0]                 aie_to_pl_tdata,
    output wire                         aie_to_pl_tvalid,
    input  wire                         aie_to_pl_tready,
    input  wire [127:0]                 pl_to_aie_tdata,
    input  wire                         pl_to_aie_tvalid,
    output wire                         pl_to_aie_tready,
    
    // Status
    output wire [31:0]                  inference_count,
    output wire [15:0]                  latency_cycles,
    output wire                         busy
);

    // ═══════════════════════════════════════════════════════════════════════════
    // State Encoder
    // ═══════════════════════════════════════════════════════════════════════════
    logic [DATA_WIDTH-1:0] state_vector [STATE_DIM-1:0];
    logic state_valid;
    
    always_ff @(posedge clk_600mhz or negedge rst_n) begin
        if (!rst_n) begin
            state_valid <= 1'b0;
            for (int i = 0; i < STATE_DIM; i++) begin
                state_vector[i] <= '0;
            end
        end else if (features_valid) begin
            // Encode state vector from features
            state_vector[0] <= radar_freq_current;                        // Current freq
            state_vector[1] <= radar_prf_current;                         // Current PRF
            state_vector[2] <= radar_bw_current;                          // Current BW
            state_vector[3] <= feat_jnr[15:0];                            // JNR estimate
            state_vector[4] <= feat_temporal_var[15:0];                   // Temporal variance
            state_vector[5] <= feat_spectral_var[15:0];                   // Spectral variance
            state_vector[6] <= {14'b0, jammer_class};                     // Jammer class
            state_vector[7] <= 16'h0000;                                  // Reserved
            state_vector[8] <= 16'h0000;                                  // Time (from counter)
            state_vector[9] <= feat_prf_stability[15:0];                  // PRF stability
            state_valid <= 1'b1;
        end else begin
            state_valid <= 1'b0;
        end
    end
    
    // ═══════════════════════════════════════════════════════════════════════════
    // AIE Policy Wrapper
    // ═══════════════════════════════════════════════════════════════════════════
    wire [DATA_WIDTH-1:0] action_out [ACTION_DIM-1:0];
    wire action_valid_w;
    wire action_ready_w;
    wire [2:0] dbg_fsm;
    
    aie_7d_policy_wrapper #(
        .STATE_DIM(STATE_DIM),
        .ACTION_DIM(ACTION_DIM),
        .DATA_WIDTH(DATA_WIDTH)
    ) u_aie_wrapper (
        .clk(clk_600mhz),
        .rst_n(rst_n),
        
        .s_state_tdata(state_vector),
        .s_state_tvalid(state_valid),
        .s_state_tready(),
        
        .m_action_tdata(action_out),
        .m_action_tvalid(action_valid_w),
        .m_action_tready(action_ready_w),
        
        .aie_to_pl_tdata(aie_to_pl_tdata),
        .aie_to_pl_tvalid(aie_to_pl_tvalid),
        .aie_to_pl_tready(aie_to_pl_tready),
        
        .pl_to_aie_tdata(pl_to_aie_tdata),
        .pl_to_aie_tvalid(pl_to_aie_tvalid),
        .pl_to_aie_tready(pl_to_aie_tready),
        
        .stat_inference_count(inference_count),
        .stat_latency_cycles(latency_cycles),
        .stat_busy(busy),
        .dbg_fsm_state(dbg_fsm)
    );
    
    // ═══════════════════════════════════════════════════════════════════════════
    // Waveform Controller
    // ═══════════════════════════════════════════════════════════════════════════
    waveform_7d_controller #(
        .DATA_WIDTH(DATA_WIDTH),
        .ACTION_DIM(ACTION_DIM)
    ) u_waveform_ctrl (
        .clk(clk_600mhz),
        .rst_n(rst_n),
        
        .action_in(action_out),
        .action_valid(action_valid_w),
        .action_ready(action_ready_w),
        
        .freq_word(freq_word),
        .freq_valid(freq_valid),
        .prf_period(prf_period),
        .prf_valid(prf_valid),
        .chirp_rate(chirp_rate),
        .pulse_width(),
        .chirp_valid(chirp_valid),
        .power_level(power_level),
        .power_valid(power_valid),
        .pol_state(pol_state),
        .pol_phase_h(),
        .pol_phase_v(),
        .pol_valid(pol_valid),
        .phase_offset(phase_offset),
        .phase_valid(phase_valid),
        .code_select(code_select),
        .code_param(),
        .code_valid(code_valid),
        .waveform_update(waveform_update)
    );

endmodule

`default_nettype wire
