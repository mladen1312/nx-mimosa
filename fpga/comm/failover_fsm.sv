//-----------------------------------------------------------------------------
// QEDMMA v2.0 Failover State Machine
// Author: Dr. Mladen Mešter
// Copyright (c) 2026 Dr. Mladen Mešter - All Rights Reserved
//
// Description:
//   Automatic link failover controller for tri-modal communication system.
//   Monitors FSO, E-band, and HF links and switches between them based on
//   packet loss thresholds and link health.
//
// Failover sequence: FSO → E-band → HF (degradation)
// Failback sequence: HF → E-band → FSO (recovery)
//
// [REQ-COMM-001] Failover time FSO→E-band: <100 ms
// [REQ-COMM-002] Failover time E-band→HF: <30 s
// [REQ-COMM-003] Automatic failback when primary recovers
// [REQ-COMM-004] Manual override capability
//-----------------------------------------------------------------------------

`timescale 1ns / 1ps

module failover_fsm #(
    parameter int FAILOVER_THRESHOLD  = 80,         // Packet loss % to trigger failover
    parameter int FAILBACK_THRESHOLD  = 20,         // Packet loss % to allow failback
    parameter int MONITOR_INTERVAL    = 25_000_000, // 100 ms @ 250 MHz
    parameter int FAILOVER_DELAY_FSO  = 25_000,     // 100 µs delay for FSO→E-band
    parameter int FAILOVER_DELAY_HF   = 7_500_000   // 30 ms delay for E-band→HF
)(
    input  logic        clk,
    input  logic        rst_n,
    
    // Link status inputs (from link_monitor modules)
    input  logic        fso_link_up,
    input  logic        eband_link_up,
    input  logic        hf_link_up,
    input  logic [7:0]  fso_packet_loss,      // 0-100%
    input  logic [7:0]  eband_packet_loss,
    input  logic [7:0]  hf_packet_loss,
    input  logic [31:0] fso_latency_ns,
    input  logic [31:0] eband_latency_ns,
    input  logic [31:0] hf_latency_ms,
    
    // Manual control inputs
    input  logic        force_failover,
    input  logic [2:0]  force_link_sel,       // One-hot: [2]=HF, [1]=E-band, [0]=FSO
    input  logic        enable,
    
    // Configuration
    input  logic [7:0]  cfg_failover_threshold,
    input  logic [7:0]  cfg_failback_threshold,
    input  logic [31:0] cfg_monitor_interval,
    
    // Output
    output logic [2:0]  active_link,          // One-hot: current active link
    output logic        failover_in_progress,
    output logic [3:0]  failover_reason,
    output logic [3:0]  current_state,
    output logic        all_links_down
);

    //-------------------------------------------------------------------------
    // Local Parameters
    //-------------------------------------------------------------------------
    
    // Failover reason codes
    localparam logic [3:0] REASON_NONE         = 4'd0;
    localparam logic [3:0] REASON_PACKET_LOSS  = 4'd1;
    localparam logic [3:0] REASON_LINK_DOWN    = 4'd2;
    localparam logic [3:0] REASON_TIMEOUT      = 4'd3;
    localparam logic [3:0] REASON_MANUAL       = 4'd4;
    localparam logic [3:0] REASON_FAILBACK     = 4'd5;
    
    // Link identifiers (one-hot)
    localparam logic [2:0] LINK_FSO   = 3'b001;
    localparam logic [2:0] LINK_EBAND = 3'b010;
    localparam logic [2:0] LINK_HF    = 3'b100;
    localparam logic [2:0] LINK_NONE  = 3'b000;
    
    //-------------------------------------------------------------------------
    // State Machine
    //-------------------------------------------------------------------------
    
    typedef enum logic [3:0] {
        ST_INIT             = 4'b0000,
        ST_FSO_ACTIVE       = 4'b0001,
        ST_FSO_DEGRADED     = 4'b0010,
        ST_EBAND_ACTIVE     = 4'b0011,
        ST_EBAND_DEGRADED   = 4'b0100,
        ST_HF_ACTIVE        = 4'b0101,
        ST_HF_DEGRADED      = 4'b0110,
        ST_FAILOVER_FSO_EB  = 4'b0111,
        ST_FAILOVER_EB_HF   = 4'b1000,
        ST_FAILBACK_HF_EB   = 4'b1001,
        ST_FAILBACK_EB_FSO  = 4'b1010,
        ST_ALL_DEGRADED     = 4'b1111
    } state_t;
    
    state_t state, next_state;
    
    //-------------------------------------------------------------------------
    // Internal Signals
    //-------------------------------------------------------------------------
    
    logic [31:0] monitor_cnt;
    logic        monitor_tick;
    logic [31:0] failover_delay_cnt;
    logic        failover_delay_done;
    
    // Link health assessment
    logic        fso_healthy;
    logic        eband_healthy;
    logic        hf_healthy;
    logic        fso_recoverable;
    logic        eband_recoverable;
    
    // Thresholds (use config or defaults)
    logic [7:0]  failover_thresh;
    logic [7:0]  failback_thresh;
    
    assign failover_thresh = (cfg_failover_threshold != 0) ? cfg_failover_threshold : FAILOVER_THRESHOLD;
    assign failback_thresh = (cfg_failback_threshold != 0) ? cfg_failback_threshold : FAILBACK_THRESHOLD;
    
    //-------------------------------------------------------------------------
    // Link Health Assessment
    //-------------------------------------------------------------------------
    
    always_comb begin
        fso_healthy   = fso_link_up   && (fso_packet_loss   < failover_thresh);
        eband_healthy = eband_link_up && (eband_packet_loss < failover_thresh);
        hf_healthy    = hf_link_up    && (hf_packet_loss    < failover_thresh);
        
        fso_recoverable   = fso_link_up   && (fso_packet_loss   < failback_thresh);
        eband_recoverable = eband_link_up && (eband_packet_loss < failback_thresh);
        
        all_links_down = !fso_healthy && !eband_healthy && !hf_healthy;
    end
    
    //-------------------------------------------------------------------------
    // Monitor Tick Generator (100 ms default)
    //-------------------------------------------------------------------------
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            monitor_cnt  <= '0;
            monitor_tick <= 1'b0;
        end else if (enable) begin
            if (monitor_cnt >= cfg_monitor_interval - 1) begin
                monitor_cnt  <= '0;
                monitor_tick <= 1'b1;
            end else begin
                monitor_cnt  <= monitor_cnt + 1;
                monitor_tick <= 1'b0;
            end
        end else begin
            monitor_cnt  <= '0;
            monitor_tick <= 1'b0;
        end
    end
    
    //-------------------------------------------------------------------------
    // Failover Delay Counter
    //-------------------------------------------------------------------------
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            failover_delay_cnt  <= '0;
            failover_delay_done <= 1'b0;
        end else begin
            case (state)
                ST_FAILOVER_FSO_EB: begin
                    if (failover_delay_cnt >= FAILOVER_DELAY_FSO) begin
                        failover_delay_done <= 1'b1;
                    end else begin
                        failover_delay_cnt  <= failover_delay_cnt + 1;
                        failover_delay_done <= 1'b0;
                    end
                end
                
                ST_FAILOVER_EB_HF: begin
                    if (failover_delay_cnt >= FAILOVER_DELAY_HF) begin
                        failover_delay_done <= 1'b1;
                    end else begin
                        failover_delay_cnt  <= failover_delay_cnt + 1;
                        failover_delay_done <= 1'b0;
                    end
                end
                
                default: begin
                    failover_delay_cnt  <= '0;
                    failover_delay_done <= 1'b0;
                end
            endcase
        end
    end
    
    //-------------------------------------------------------------------------
    // State Register
    //-------------------------------------------------------------------------
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= ST_INIT;
        else if (!enable)
            state <= ST_INIT;
        else
            state <= next_state;
    end
    
    //-------------------------------------------------------------------------
    // Next State Logic
    //-------------------------------------------------------------------------
    
    always_comb begin
        next_state          = state;
        active_link         = LINK_NONE;
        failover_in_progress = 1'b0;
        failover_reason     = REASON_NONE;
        current_state       = state;
        
        // Manual override has highest priority
        if (force_failover) begin
            active_link     = force_link_sel;
            failover_reason = REASON_MANUAL;
        end else begin
            case (state)
                //-------------------------------------------------------------
                // Initialization
                //-------------------------------------------------------------
                ST_INIT: begin
                    active_link = LINK_NONE;
                    if (fso_healthy)
                        next_state = ST_FSO_ACTIVE;
                    else if (eband_healthy)
                        next_state = ST_EBAND_ACTIVE;
                    else if (hf_healthy)
                        next_state = ST_HF_ACTIVE;
                    else
                        next_state = ST_ALL_DEGRADED;
                end
                
                //-------------------------------------------------------------
                // FSO Active States
                //-------------------------------------------------------------
                ST_FSO_ACTIVE: begin
                    active_link = LINK_FSO;
                    if (!fso_link_up) begin
                        next_state      = ST_FAILOVER_FSO_EB;
                        failover_reason = REASON_LINK_DOWN;
                    end else if (fso_packet_loss >= failover_thresh) begin
                        next_state      = ST_FSO_DEGRADED;
                        failover_reason = REASON_PACKET_LOSS;
                    end
                end
                
                ST_FSO_DEGRADED: begin
                    active_link = LINK_FSO;
                    if (!fso_link_up || fso_packet_loss >= failover_thresh + 10) begin
                        next_state      = ST_FAILOVER_FSO_EB;
                        failover_reason = REASON_PACKET_LOSS;
                    end else if (fso_healthy) begin
                        next_state = ST_FSO_ACTIVE;
                    end
                end
                
                //-------------------------------------------------------------
                // FSO → E-band Failover
                //-------------------------------------------------------------
                ST_FAILOVER_FSO_EB: begin
                    active_link          = LINK_EBAND;
                    failover_in_progress = 1'b1;
                    failover_reason      = REASON_PACKET_LOSS;
                    
                    if (failover_delay_done) begin
                        if (eband_healthy)
                            next_state = ST_EBAND_ACTIVE;
                        else
                            next_state = ST_FAILOVER_EB_HF;
                    end
                end
                
                //-------------------------------------------------------------
                // E-band Active States
                //-------------------------------------------------------------
                ST_EBAND_ACTIVE: begin
                    active_link = LINK_EBAND;
                    
                    // Check for failback to FSO
                    if (fso_recoverable) begin
                        next_state      = ST_FAILBACK_EB_FSO;
                        failover_reason = REASON_FAILBACK;
                    end
                    // Check for failover to HF
                    else if (!eband_link_up) begin
                        next_state      = ST_FAILOVER_EB_HF;
                        failover_reason = REASON_LINK_DOWN;
                    end else if (eband_packet_loss >= failover_thresh) begin
                        next_state      = ST_EBAND_DEGRADED;
                        failover_reason = REASON_PACKET_LOSS;
                    end
                end
                
                ST_EBAND_DEGRADED: begin
                    active_link = LINK_EBAND;
                    
                    if (fso_recoverable) begin
                        next_state      = ST_FAILBACK_EB_FSO;
                        failover_reason = REASON_FAILBACK;
                    end else if (!eband_link_up || eband_packet_loss >= failover_thresh + 10) begin
                        next_state      = ST_FAILOVER_EB_HF;
                        failover_reason = REASON_PACKET_LOSS;
                    end else if (eband_healthy) begin
                        next_state = ST_EBAND_ACTIVE;
                    end
                end
                
                //-------------------------------------------------------------
                // E-band → HF Failover
                //-------------------------------------------------------------
                ST_FAILOVER_EB_HF: begin
                    active_link          = LINK_HF;
                    failover_in_progress = 1'b1;
                    failover_reason      = REASON_PACKET_LOSS;
                    
                    if (failover_delay_done) begin
                        if (hf_healthy)
                            next_state = ST_HF_ACTIVE;
                        else
                            next_state = ST_ALL_DEGRADED;
                    end
                end
                
                //-------------------------------------------------------------
                // HF Active States
                //-------------------------------------------------------------
                ST_HF_ACTIVE: begin
                    active_link = LINK_HF;
                    
                    // Check for failback to E-band
                    if (eband_recoverable) begin
                        next_state      = ST_FAILBACK_HF_EB;
                        failover_reason = REASON_FAILBACK;
                    end else if (!hf_healthy) begin
                        next_state = ST_HF_DEGRADED;
                    end
                end
                
                ST_HF_DEGRADED: begin
                    active_link = LINK_HF;
                    
                    if (eband_recoverable) begin
                        next_state      = ST_FAILBACK_HF_EB;
                        failover_reason = REASON_FAILBACK;
                    end else if (fso_recoverable) begin
                        next_state      = ST_FAILBACK_EB_FSO;
                        failover_reason = REASON_FAILBACK;
                    end else if (!hf_link_up) begin
                        next_state = ST_ALL_DEGRADED;
                    end
                end
                
                //-------------------------------------------------------------
                // Failback States
                //-------------------------------------------------------------
                ST_FAILBACK_HF_EB: begin
                    active_link          = LINK_EBAND;
                    failover_in_progress = 1'b1;
                    failover_reason      = REASON_FAILBACK;
                    next_state           = ST_EBAND_ACTIVE;
                end
                
                ST_FAILBACK_EB_FSO: begin
                    active_link          = LINK_FSO;
                    failover_in_progress = 1'b1;
                    failover_reason      = REASON_FAILBACK;
                    next_state           = ST_FSO_ACTIVE;
                end
                
                //-------------------------------------------------------------
                // All Links Degraded
                //-------------------------------------------------------------
                ST_ALL_DEGRADED: begin
                    active_link = LINK_NONE;
                    
                    // Try to recover any link
                    if (fso_healthy)
                        next_state = ST_FSO_ACTIVE;
                    else if (eband_healthy)
                        next_state = ST_EBAND_ACTIVE;
                    else if (hf_healthy)
                        next_state = ST_HF_ACTIVE;
                end
                
                default: next_state = ST_INIT;
            endcase
        end
    end

endmodule
