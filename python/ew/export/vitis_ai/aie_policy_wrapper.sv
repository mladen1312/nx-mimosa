// ═══════════════════════════════════════════════════════════════════════════════
// NX-MIMOSA AIE Policy Wrapper
// ═══════════════════════════════════════════════════════════════════════════════
// [REQ-RL-DEPLOY-001] Wraps Vitis AI inference for radar integration
//
// Architecture:
//   PL State Collector → AXI4-Stream → AIE Policy → AXI4-Stream → PL Action Controller
//
// Latency: <50 µs total (AIE inference + wrapper overhead)
// ═══════════════════════════════════════════════════════════════════════════════

`timescale 1ns/1ps

module aie_policy_wrapper #(
    parameter int STATE_DIM = 6,
    parameter int ACTION_DIM = 2,
    parameter int DATA_WIDTH = 32          // Float32 for AIE interface
)(
    // Clock & Reset
    input  logic                        clk,
    input  logic                        rst_n,
    
    // State Input (from radar sensors)
    input  logic                        state_valid,
    input  logic [DATA_WIDTH-1:0]       state_data [STATE_DIM],
    output logic                        state_ready,
    
    // Action Output (to waveform generator)
    output logic                        action_valid,
    output logic [DATA_WIDTH-1:0]       action_data [ACTION_DIM],
    input  logic                        action_ready,
    
    // AIE Interface (AXI4-Stream to/from AIE)
    output logic                        m_axis_state_tvalid,
    output logic [DATA_WIDTH*STATE_DIM-1:0] m_axis_state_tdata,
    output logic                        m_axis_state_tlast,
    input  logic                        m_axis_state_tready,
    
    input  logic                        s_axis_action_tvalid,
    input  logic [DATA_WIDTH*ACTION_DIM-1:0] s_axis_action_tdata,
    input  logic                        s_axis_action_tlast,
    output logic                        s_axis_action_tready,
    
    // Status
    output logic [15:0]                 stat_inferences,
    output logic [15:0]                 stat_latency_cycles
);

    // ═══════════════════════════════════════════════════════════════════════════
    // State Machine
    // ═══════════════════════════════════════════════════════════════════════════
    
    typedef enum logic [2:0] {
        IDLE,
        PACK_STATE,
        SEND_TO_AIE,
        WAIT_AIE,
        UNPACK_ACTION,
        OUTPUT_ACTION
    } state_t;
    
    state_t fsm_state;
    
    // Latency measurement
    logic [15:0] latency_counter;
    logic latency_counting;
    
    // State packing
    logic [DATA_WIDTH*STATE_DIM-1:0] packed_state;
    logic [DATA_WIDTH*ACTION_DIM-1:0] packed_action;
    
    // ═══════════════════════════════════════════════════════════════════════════
    // State Packing
    // ═══════════════════════════════════════════════════════════════════════════
    
    always_comb begin
        packed_state = '0;
        for (int i = 0; i < STATE_DIM; i++) begin
            packed_state[i*DATA_WIDTH +: DATA_WIDTH] = state_data[i];
        end
    end
    
    // ═══════════════════════════════════════════════════════════════════════════
    // Main FSM
    // ═══════════════════════════════════════════════════════════════════════════
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fsm_state <= IDLE;
            m_axis_state_tvalid <= 1'b0;
            m_axis_state_tdata <= '0;
            m_axis_state_tlast <= 1'b0;
            s_axis_action_tready <= 1'b0;
            action_valid <= 1'b0;
            state_ready <= 1'b1;
            latency_counter <= '0;
            latency_counting <= 1'b0;
            stat_inferences <= '0;
            stat_latency_cycles <= '0;
        end else begin
            case (fsm_state)
                IDLE: begin
                    state_ready <= 1'b1;
                    if (state_valid && state_ready) begin
                        fsm_state <= PACK_STATE;
                        state_ready <= 1'b0;
                    end
                end
                
                PACK_STATE: begin
                    m_axis_state_tdata <= packed_state;
                    m_axis_state_tvalid <= 1'b1;
                    m_axis_state_tlast <= 1'b1;
                    latency_counting <= 1'b1;
                    latency_counter <= '0;
                    fsm_state <= SEND_TO_AIE;
                end
                
                SEND_TO_AIE: begin
                    if (m_axis_state_tready) begin
                        m_axis_state_tvalid <= 1'b0;
                        s_axis_action_tready <= 1'b1;
                        fsm_state <= WAIT_AIE;
                    end
                end
                
                WAIT_AIE: begin
                    latency_counter <= latency_counter + 1;
                    
                    if (s_axis_action_tvalid) begin
                        packed_action <= s_axis_action_tdata;
                        s_axis_action_tready <= 1'b0;
                        latency_counting <= 1'b0;
                        stat_latency_cycles <= latency_counter;
                        fsm_state <= UNPACK_ACTION;
                    end
                end
                
                UNPACK_ACTION: begin
                    for (int i = 0; i < ACTION_DIM; i++) begin
                        action_data[i] <= packed_action[i*DATA_WIDTH +: DATA_WIDTH];
                    end
                    action_valid <= 1'b1;
                    fsm_state <= OUTPUT_ACTION;
                end
                
                OUTPUT_ACTION: begin
                    if (action_ready) begin
                        action_valid <= 1'b0;
                        stat_inferences <= stat_inferences + 1;
                        fsm_state <= IDLE;
                    end
                end
                
                default: fsm_state <= IDLE;
            endcase
        end
    end

endmodule
