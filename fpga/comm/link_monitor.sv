//-----------------------------------------------------------------------------
// QEDMMA v2.0 Link Monitor
// Author: Dr. Mladen Mešter
// Copyright (c) 2026 Dr. Mladen Mešter - All Rights Reserved
//
// Description:
//   Per-link health monitoring module. Sends periodic ping frames and
//   measures packet loss, latency, and jitter for each communication link.
//
// Features:
//   - Ping/pong health checks at configurable interval
//   - Packet loss calculation over sliding window
//   - Round-trip latency measurement
//   - Jitter calculation
//   - Physical layer status monitoring
//
// [REQ-COMM-005] Link health monitoring at 50 ms intervals
// [REQ-COMM-006] Packet loss measurement with 1% resolution
// [REQ-COMM-007] Latency measurement with µs resolution
//-----------------------------------------------------------------------------

`timescale 1ns / 1ps

module link_monitor #(
    parameter int CLK_FREQ_HZ     = 250_000_000,  // 250 MHz
    parameter int PING_INTERVAL_MS = 50,          // Ping every 50 ms
    parameter int TIMEOUT_MS       = 200,         // Response timeout
    parameter int WINDOW_SIZE      = 100,         // Stats window (pings)
    parameter int DATA_WIDTH       = 64           // Data bus width
)(
    input  logic                    clk,
    input  logic                    rst_n,
    
    //-------------------------------------------------------------------------
    // Data Interface (TX/RX for ping/pong frames)
    //-------------------------------------------------------------------------
    output logic [DATA_WIDTH-1:0]   tx_data,
    output logic                    tx_valid,
    input  logic                    tx_ready,
    output logic                    tx_last,
    
    input  logic [DATA_WIDTH-1:0]   rx_data,
    input  logic                    rx_valid,
    input  logic                    rx_last,
    output logic                    rx_ready,
    
    //-------------------------------------------------------------------------
    // Physical Layer Status
    //-------------------------------------------------------------------------
    input  logic                    phy_link_up,
    input  logic                    phy_rx_error,
    input  logic [31:0]             phy_rx_bytes,
    input  logic [31:0]             phy_tx_bytes,
    
    //-------------------------------------------------------------------------
    // Control
    //-------------------------------------------------------------------------
    input  logic                    enable,
    input  logic [31:0]             cfg_ping_interval,  // Clocks (0 = use default)
    input  logic [31:0]             cfg_timeout,        // Clocks (0 = use default)
    
    //-------------------------------------------------------------------------
    // Status Outputs
    //-------------------------------------------------------------------------
    output logic                    link_up,
    output logic                    link_healthy,
    output logic [7:0]              packet_loss_pct,    // 0-100
    output logic [31:0]             latency_ns,         // Round-trip
    output logic [31:0]             jitter_ns,          // Latency variance
    output logic [31:0]             rx_packets,
    output logic [31:0]             tx_packets,
    output logic [31:0]             rx_errors,
    output logic [31:0]             timeouts
);

    //-------------------------------------------------------------------------
    // Local Parameters
    //-------------------------------------------------------------------------
    localparam int PING_CYCLES    = (CLK_FREQ_HZ / 1000) * PING_INTERVAL_MS;
    localparam int TIMEOUT_CYCLES = (CLK_FREQ_HZ / 1000) * TIMEOUT_MS;
    localparam int NS_PER_CYCLE   = 1_000_000_000 / CLK_FREQ_HZ;  // 4 ns @ 250 MHz
    
    // Frame magic numbers
    localparam logic [63:0] PING_MAGIC = 64'hQEDM_PING_REQ1;
    localparam logic [63:0] PONG_MAGIC = 64'hQEDM_PONG_RSP1;
    
    //-------------------------------------------------------------------------
    // State Machine
    //-------------------------------------------------------------------------
    typedef enum logic [2:0] {
        ST_IDLE,
        ST_SEND_PING,
        ST_WAIT_PONG,
        ST_PROCESS_PONG,
        ST_TIMEOUT,
        ST_CALC_STATS
    } state_t;
    
    state_t state, next_state;
    
    //-------------------------------------------------------------------------
    // Internal Signals
    //-------------------------------------------------------------------------
    logic [31:0] interval_cnt;
    logic        interval_tick;
    
    logic [31:0] timeout_cnt;
    logic        timeout_expired;
    
    logic [31:0] ping_timestamp;
    logic [31:0] pong_timestamp;
    logic [31:0] current_latency;
    
    // Sequence number for matching
    logic [15:0] seq_num;
    logic [15:0] rx_seq_num;
    
    // Statistics sliding window
    logic        success_history [0:WINDOW_SIZE-1];
    logic [31:0] latency_history [0:WINDOW_SIZE-1];
    logic [6:0]  history_idx;
    
    // Calculated statistics
    logic [31:0] success_count;
    logic [31:0] total_latency;
    logic [31:0] latency_squared_sum;
    logic [31:0] mean_latency;
    
    // Frame construction
    logic [63:0] ping_frame;
    logic [63:0] pong_frame;
    
    // Effective configuration
    logic [31:0] eff_ping_interval;
    logic [31:0] eff_timeout;
    
    assign eff_ping_interval = (cfg_ping_interval != 0) ? cfg_ping_interval : PING_CYCLES;
    assign eff_timeout       = (cfg_timeout != 0)       ? cfg_timeout       : TIMEOUT_CYCLES;
    
    //-------------------------------------------------------------------------
    // Interval Tick Generator
    //-------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            interval_cnt  <= '0;
            interval_tick <= 1'b0;
        end else if (enable && phy_link_up) begin
            if (interval_cnt >= eff_ping_interval - 1) begin
                interval_cnt  <= '0;
                interval_tick <= 1'b1;
            end else begin
                interval_cnt  <= interval_cnt + 1;
                interval_tick <= 1'b0;
            end
        end else begin
            interval_cnt  <= '0;
            interval_tick <= 1'b0;
        end
    end
    
    //-------------------------------------------------------------------------
    // Timeout Counter
    //-------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            timeout_cnt     <= '0;
            timeout_expired <= 1'b0;
        end else begin
            case (state)
                ST_WAIT_PONG: begin
                    if (timeout_cnt >= eff_timeout - 1) begin
                        timeout_expired <= 1'b1;
                    end else begin
                        timeout_cnt     <= timeout_cnt + 1;
                        timeout_expired <= 1'b0;
                    end
                end
                default: begin
                    timeout_cnt     <= '0;
                    timeout_expired <= 1'b0;
                end
            endcase
        end
    end
    
    //-------------------------------------------------------------------------
    // State Register
    //-------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= ST_IDLE;
        else if (!enable || !phy_link_up)
            state <= ST_IDLE;
        else
            state <= next_state;
    end
    
    //-------------------------------------------------------------------------
    // Timestamp Capture
    //-------------------------------------------------------------------------
    logic [31:0] cycle_counter;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            cycle_counter <= '0;
        else
            cycle_counter <= cycle_counter + 1;
    end
    
    //-------------------------------------------------------------------------
    // Sequence Number
    //-------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            seq_num <= '0;
        else if (state == ST_SEND_PING && tx_valid && tx_ready)
            seq_num <= seq_num + 1;
    end
    
    //-------------------------------------------------------------------------
    // Frame Construction
    //-------------------------------------------------------------------------
    // Ping frame: [MAGIC(64)] [SEQ(16)] [TIMESTAMP(32)] [PADDING(16)]
    assign ping_frame = PING_MAGIC;
    
    //-------------------------------------------------------------------------
    // Next State Logic
    //-------------------------------------------------------------------------
    always_comb begin
        next_state = state;
        
        case (state)
            ST_IDLE: begin
                if (interval_tick && enable && phy_link_up)
                    next_state = ST_SEND_PING;
            end
            
            ST_SEND_PING: begin
                if (tx_valid && tx_ready)
                    next_state = ST_WAIT_PONG;
            end
            
            ST_WAIT_PONG: begin
                if (rx_valid && rx_data[63:0] == PONG_MAGIC)
                    next_state = ST_PROCESS_PONG;
                else if (timeout_expired)
                    next_state = ST_TIMEOUT;
            end
            
            ST_PROCESS_PONG: begin
                next_state = ST_CALC_STATS;
            end
            
            ST_TIMEOUT: begin
                next_state = ST_CALC_STATS;
            end
            
            ST_CALC_STATS: begin
                next_state = ST_IDLE;
            end
            
            default: next_state = ST_IDLE;
        endcase
    end
    
    //-------------------------------------------------------------------------
    // TX Data Path
    //-------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_data      <= '0;
            tx_valid     <= 1'b0;
            tx_last      <= 1'b0;
            tx_packets   <= '0;
            ping_timestamp <= '0;
        end else begin
            case (state)
                ST_SEND_PING: begin
                    tx_data  <= ping_frame;
                    tx_valid <= 1'b1;
                    tx_last  <= 1'b1;
                    ping_timestamp <= cycle_counter;
                    
                    if (tx_ready) begin
                        tx_packets <= tx_packets + 1;
                    end
                end
                
                default: begin
                    tx_valid <= 1'b0;
                    tx_last  <= 1'b0;
                end
            endcase
        end
    end
    
    //-------------------------------------------------------------------------
    // RX Data Path
    //-------------------------------------------------------------------------
    assign rx_ready = 1'b1;  // Always ready to receive
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_packets     <= '0;
            rx_errors      <= '0;
            pong_timestamp <= '0;
            current_latency <= '0;
        end else begin
            if (rx_valid) begin
                rx_packets <= rx_packets + 1;
                
                if (rx_data == PONG_MAGIC) begin
                    pong_timestamp  <= cycle_counter;
                    current_latency <= (cycle_counter - ping_timestamp) * NS_PER_CYCLE;
                end
            end
            
            if (phy_rx_error)
                rx_errors <= rx_errors + 1;
        end
    end
    
    //-------------------------------------------------------------------------
    // Statistics Window Management
    //-------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            history_idx <= '0;
            timeouts    <= '0;
            
            for (int i = 0; i < WINDOW_SIZE; i++) begin
                success_history[i] <= 1'b0;
                latency_history[i] <= '0;
            end
        end else begin
            case (state)
                ST_PROCESS_PONG: begin
                    success_history[history_idx] <= 1'b1;
                    latency_history[history_idx] <= current_latency;
                    history_idx <= (history_idx + 1) % WINDOW_SIZE;
                end
                
                ST_TIMEOUT: begin
                    success_history[history_idx] <= 1'b0;
                    latency_history[history_idx] <= '0;
                    history_idx <= (history_idx + 1) % WINDOW_SIZE;
                    timeouts    <= timeouts + 1;
                end
                
                default: ;
            endcase
        end
    end
    
    //-------------------------------------------------------------------------
    // Statistics Calculation
    //-------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            success_count    <= '0;
            total_latency    <= '0;
            mean_latency     <= '0;
            packet_loss_pct  <= 8'd100;
            latency_ns       <= '0;
            jitter_ns        <= '0;
        end else if (state == ST_CALC_STATS) begin
            // Calculate success count and total latency
            automatic logic [31:0] s_count = 0;
            automatic logic [31:0] t_latency = 0;
            automatic logic [31:0] l_squared = 0;
            
            for (int i = 0; i < WINDOW_SIZE; i++) begin
                if (success_history[i]) begin
                    s_count = s_count + 1;
                    t_latency = t_latency + latency_history[i];
                end
            end
            
            success_count   <= s_count;
            total_latency   <= t_latency;
            
            // Packet loss percentage
            packet_loss_pct <= 8'(100 - (s_count * 100 / WINDOW_SIZE));
            
            // Mean latency
            if (s_count > 0) begin
                mean_latency <= t_latency / s_count;
                latency_ns   <= t_latency / s_count;
            end else begin
                mean_latency <= '0;
                latency_ns   <= 32'hFFFFFFFF;  // Max value indicates no data
            end
            
            // Simplified jitter (would need more complex calc for real std dev)
            // Using max-min as approximation
            automatic logic [31:0] max_lat = 0;
            automatic logic [31:0] min_lat = 32'hFFFFFFFF;
            
            for (int i = 0; i < WINDOW_SIZE; i++) begin
                if (success_history[i] && latency_history[i] > 0) begin
                    if (latency_history[i] > max_lat) max_lat = latency_history[i];
                    if (latency_history[i] < min_lat) min_lat = latency_history[i];
                end
            end
            
            jitter_ns <= (min_lat != 32'hFFFFFFFF) ? (max_lat - min_lat) : '0;
        end
    end
    
    //-------------------------------------------------------------------------
    // Link Status
    //-------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            link_up      <= 1'b0;
            link_healthy <= 1'b0;
        end else begin
            link_up      <= phy_link_up;
            link_healthy <= phy_link_up && (packet_loss_pct < 50) && (success_count > 50);
        end
    end

endmodule
