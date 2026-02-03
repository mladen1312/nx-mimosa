//-----------------------------------------------------------------------------
// QEDMMA v2.0 Communication Controller - Top Level
// Author: Dr. Mladen Mešter
// Copyright (c) 2026 Dr. Mladen Mešter - All Rights Reserved
//
// Description:
//   Top-level communication controller integrating:
//   - Three link monitors (FSO, E-band, HF)
//   - Failover state machine
//   - Packet routing/arbitration
//   - AXI4-Lite register interface
//
// Interfaces:
//   - FSO: 10GBASE-R (64-bit XGMII-style)
//   - E-band: 10GbE (64-bit AXI-Stream)
//   - HF: UART to MIL-STD-188-110D modem
//   - Application: AXI-Stream
//   - Control: AXI4-Lite
//
// [REQ-COMM-001] Tri-modal communication support
// [REQ-COMM-002] Automatic failover <100ms
// [REQ-COMM-003] AES-256 encryption
// [REQ-COMM-004] Mesh routing capability
//-----------------------------------------------------------------------------

`timescale 1ns / 1ps

module comm_controller_top #(
    parameter int DATA_WIDTH     = 128,       // AXI-Stream data width
    parameter int ADDR_WIDTH     = 16,        // Register address width
    parameter int ENABLE_CRYPTO  = 1,         // Enable AES encryption
    parameter int NUM_NODES      = 6          // Max nodes in mesh
)(
    //-------------------------------------------------------------------------
    // Clocks and Reset
    //-------------------------------------------------------------------------
    input  logic                     clk_250,        // 250 MHz system clock
    input  logic                     clk_156_25,     // 156.25 MHz for 10G PHY
    input  logic                     clk_hf,         // HF modem clock (variable)
    input  logic                     rst_n,
    
    //-------------------------------------------------------------------------
    // FSO Interface (10GBASE-R style)
    //-------------------------------------------------------------------------
    input  logic [63:0]              fso_rx_data,
    input  logic [7:0]               fso_rx_ctrl,
    input  logic                     fso_rx_valid,
    input  logic                     fso_rx_error,
    output logic [63:0]              fso_tx_data,
    output logic [7:0]               fso_tx_ctrl,
    output logic                     fso_tx_valid,
    input  logic                     fso_tx_ready,
    input  logic                     fso_phy_up,
    
    //-------------------------------------------------------------------------
    // E-band Interface (10GbE AXI-Stream)
    //-------------------------------------------------------------------------
    input  logic [63:0]              eband_rx_tdata,
    input  logic                     eband_rx_tvalid,
    output logic                     eband_rx_tready,
    input  logic                     eband_rx_tlast,
    input  logic                     eband_rx_error,
    output logic [63:0]              eband_tx_tdata,
    output logic                     eband_tx_tvalid,
    input  logic                     eband_tx_tready,
    output logic                     eband_tx_tlast,
    input  logic                     eband_phy_up,
    
    //-------------------------------------------------------------------------
    // HF Interface (UART to modem)
    //-------------------------------------------------------------------------
    input  logic                     hf_uart_rx,
    output logic                     hf_uart_tx,
    output logic                     hf_ptt,          // Push-to-talk
    input  logic                     hf_cts,          // Clear-to-send
    input  logic                     hf_link_up,
    
    //-------------------------------------------------------------------------
    // Application Interface (AXI-Stream)
    //-------------------------------------------------------------------------
    // Slave (from application)
    input  logic [DATA_WIDTH-1:0]    s_axis_tdata,
    input  logic                     s_axis_tvalid,
    output logic                     s_axis_tready,
    input  logic                     s_axis_tlast,
    input  logic [3:0]               s_axis_tdest,    // Destination node ID
    input  logic [3:0]               s_axis_tuser,    // Priority/flags
    
    // Master (to application)
    output logic [DATA_WIDTH-1:0]    m_axis_tdata,
    output logic                     m_axis_tvalid,
    input  logic                     m_axis_tready,
    output logic                     m_axis_tlast,
    output logic [3:0]               m_axis_tsrc,     // Source node ID
    output logic [3:0]               m_axis_tuser,    // Flags
    
    //-------------------------------------------------------------------------
    // AXI4-Lite Control Interface
    //-------------------------------------------------------------------------
    input  logic [ADDR_WIDTH-1:0]    s_axi_awaddr,
    input  logic                     s_axi_awvalid,
    output logic                     s_axi_awready,
    input  logic [31:0]              s_axi_wdata,
    input  logic [3:0]               s_axi_wstrb,
    input  logic                     s_axi_wvalid,
    output logic                     s_axi_wready,
    output logic [1:0]               s_axi_bresp,
    output logic                     s_axi_bvalid,
    input  logic                     s_axi_bready,
    input  logic [ADDR_WIDTH-1:0]    s_axi_araddr,
    input  logic                     s_axi_arvalid,
    output logic                     s_axi_arready,
    output logic [31:0]              s_axi_rdata,
    output logic [1:0]               s_axi_rresp,
    output logic                     s_axi_rvalid,
    input  logic                     s_axi_rready,
    
    //-------------------------------------------------------------------------
    // Status Outputs
    //-------------------------------------------------------------------------
    output logic [2:0]               active_link,     // One-hot
    output logic                     link_healthy,
    output logic                     failover_active,
    output logic                     all_links_down,
    output logic [3:0]               node_id
);

    //-------------------------------------------------------------------------
    // Internal Signals
    //-------------------------------------------------------------------------
    
    // Control registers
    logic        ctrl_enable;
    logic        ctrl_force_failover;
    logic [2:0]  ctrl_force_link;
    logic        ctrl_crypto_enable;
    logic [7:0]  cfg_failover_threshold;
    logic [7:0]  cfg_failback_threshold;
    logic [31:0] cfg_monitor_interval;
    
    // FSO link monitor signals
    logic [63:0] fso_mon_tx_data;
    logic        fso_mon_tx_valid;
    logic        fso_mon_tx_ready;
    logic        fso_mon_tx_last;
    logic        fso_link_healthy;
    logic [7:0]  fso_packet_loss;
    logic [31:0] fso_latency;
    logic [31:0] fso_rx_packets;
    logic [31:0] fso_tx_packets;
    
    // E-band link monitor signals
    logic [63:0] eband_mon_tx_data;
    logic        eband_mon_tx_valid;
    logic        eband_mon_tx_ready;
    logic        eband_mon_tx_last;
    logic        eband_link_healthy;
    logic [7:0]  eband_packet_loss;
    logic [31:0] eband_latency;
    logic [31:0] eband_rx_packets;
    logic [31:0] eband_tx_packets;
    
    // HF link monitor signals (via UART)
    logic [63:0] hf_mon_tx_data;
    logic        hf_mon_tx_valid;
    logic        hf_mon_tx_ready;
    logic        hf_mon_tx_last;
    logic        hf_link_healthy;
    logic [7:0]  hf_packet_loss;
    logic [31:0] hf_latency;
    logic [31:0] hf_rx_packets;
    logic [31:0] hf_tx_packets;
    
    // Failover FSM outputs
    logic [2:0]  fsm_active_link;
    logic        fsm_failover_in_progress;
    logic [3:0]  fsm_failover_reason;
    logic [3:0]  fsm_current_state;
    logic        fsm_all_links_down;
    
    //-------------------------------------------------------------------------
    // FSO Link Monitor Instance
    //-------------------------------------------------------------------------
    link_monitor #(
        .CLK_FREQ_HZ      (250_000_000),
        .PING_INTERVAL_MS (50),
        .TIMEOUT_MS       (200),
        .WINDOW_SIZE      (100),
        .DATA_WIDTH       (64)
    ) u_fso_monitor (
        .clk              (clk_250),
        .rst_n            (rst_n),
        
        .tx_data          (fso_mon_tx_data),
        .tx_valid         (fso_mon_tx_valid),
        .tx_ready         (fso_mon_tx_ready),
        .tx_last          (fso_mon_tx_last),
        
        .rx_data          (fso_rx_data),
        .rx_valid         (fso_rx_valid),
        .rx_last          (1'b1),
        .rx_ready         (),
        
        .phy_link_up      (fso_phy_up),
        .phy_rx_error     (fso_rx_error),
        .phy_rx_bytes     (32'b0),
        .phy_tx_bytes     (32'b0),
        
        .enable           (ctrl_enable),
        .cfg_ping_interval(cfg_monitor_interval),
        .cfg_timeout      (32'd50_000_000),  // 200 ms @ 250 MHz
        
        .link_up          (),
        .link_healthy     (fso_link_healthy),
        .packet_loss_pct  (fso_packet_loss),
        .latency_ns       (fso_latency),
        .jitter_ns        (),
        .rx_packets       (fso_rx_packets),
        .tx_packets       (fso_tx_packets),
        .rx_errors        (),
        .timeouts         ()
    );
    
    //-------------------------------------------------------------------------
    // E-band Link Monitor Instance
    //-------------------------------------------------------------------------
    link_monitor #(
        .CLK_FREQ_HZ      (250_000_000),
        .PING_INTERVAL_MS (50),
        .TIMEOUT_MS       (200),
        .WINDOW_SIZE      (100),
        .DATA_WIDTH       (64)
    ) u_eband_monitor (
        .clk              (clk_250),
        .rst_n            (rst_n),
        
        .tx_data          (eband_mon_tx_data),
        .tx_valid         (eband_mon_tx_valid),
        .tx_ready         (eband_mon_tx_ready),
        .tx_last          (eband_mon_tx_last),
        
        .rx_data          (eband_rx_tdata),
        .rx_valid         (eband_rx_tvalid),
        .rx_last          (eband_rx_tlast),
        .rx_ready         (eband_rx_tready),
        
        .phy_link_up      (eband_phy_up),
        .phy_rx_error     (eband_rx_error),
        .phy_rx_bytes     (32'b0),
        .phy_tx_bytes     (32'b0),
        
        .enable           (ctrl_enable),
        .cfg_ping_interval(cfg_monitor_interval),
        .cfg_timeout      (32'd50_000_000),
        
        .link_up          (),
        .link_healthy     (eband_link_healthy),
        .packet_loss_pct  (eband_packet_loss),
        .latency_ns       (eband_latency),
        .jitter_ns        (),
        .rx_packets       (eband_rx_packets),
        .tx_packets       (eband_tx_packets),
        .rx_errors        (),
        .timeouts         ()
    );
    
    //-------------------------------------------------------------------------
    // HF Link Monitor Instance (slower timing)
    //-------------------------------------------------------------------------
    link_monitor #(
        .CLK_FREQ_HZ      (250_000_000),
        .PING_INTERVAL_MS (1000),        // 1 second for HF
        .TIMEOUT_MS       (5000),        // 5 second timeout
        .WINDOW_SIZE      (20),          // Smaller window
        .DATA_WIDTH       (64)
    ) u_hf_monitor (
        .clk              (clk_250),
        .rst_n            (rst_n),
        
        .tx_data          (hf_mon_tx_data),
        .tx_valid         (hf_mon_tx_valid),
        .tx_ready         (hf_mon_tx_ready),
        .tx_last          (hf_mon_tx_last),
        
        .rx_data          (64'b0),        // HF uses UART, handled separately
        .rx_valid         (1'b0),
        .rx_last          (1'b0),
        .rx_ready         (),
        
        .phy_link_up      (hf_link_up),
        .phy_rx_error     (1'b0),
        .phy_rx_bytes     (32'b0),
        .phy_tx_bytes     (32'b0),
        
        .enable           (ctrl_enable),
        .cfg_ping_interval(32'd250_000_000),  // 1 second
        .cfg_timeout      (32'd1_250_000_000), // 5 seconds
        
        .link_up          (),
        .link_healthy     (hf_link_healthy),
        .packet_loss_pct  (hf_packet_loss),
        .latency_ns       (hf_latency),
        .jitter_ns        (),
        .rx_packets       (hf_rx_packets),
        .tx_packets       (hf_tx_packets),
        .rx_errors        (),
        .timeouts         ()
    );
    
    //-------------------------------------------------------------------------
    // Failover FSM Instance
    //-------------------------------------------------------------------------
    failover_fsm #(
        .FAILOVER_THRESHOLD  (80),
        .FAILBACK_THRESHOLD  (20),
        .MONITOR_INTERVAL    (25_000_000),
        .FAILOVER_DELAY_FSO  (25_000),
        .FAILOVER_DELAY_HF   (7_500_000)
    ) u_failover_fsm (
        .clk                    (clk_250),
        .rst_n                  (rst_n),
        
        .fso_link_up            (fso_phy_up),
        .eband_link_up          (eband_phy_up),
        .hf_link_up             (hf_link_up),
        .fso_packet_loss        (fso_packet_loss),
        .eband_packet_loss      (eband_packet_loss),
        .hf_packet_loss         (hf_packet_loss),
        .fso_latency_ns         (fso_latency),
        .eband_latency_ns       (eband_latency),
        .hf_latency_ms          (hf_latency / 1_000_000),
        
        .force_failover         (ctrl_force_failover),
        .force_link_sel         (ctrl_force_link),
        .enable                 (ctrl_enable),
        
        .cfg_failover_threshold (cfg_failover_threshold),
        .cfg_failback_threshold (cfg_failback_threshold),
        .cfg_monitor_interval   (cfg_monitor_interval),
        
        .active_link            (fsm_active_link),
        .failover_in_progress   (fsm_failover_in_progress),
        .failover_reason        (fsm_failover_reason),
        .current_state          (fsm_current_state),
        .all_links_down         (fsm_all_links_down)
    );
    
    //-------------------------------------------------------------------------
    // Output Assignments
    //-------------------------------------------------------------------------
    assign active_link     = fsm_active_link;
    assign link_healthy    = fso_link_healthy | eband_link_healthy | hf_link_healthy;
    assign failover_active = fsm_failover_in_progress;
    assign all_links_down  = fsm_all_links_down;
    
    //-------------------------------------------------------------------------
    // AXI4-Lite Register Interface
    //-------------------------------------------------------------------------
    // Register map (see comm_controller_regs.yaml for SSOT)
    // 0x00: CTRL      - Control register
    // 0x04: STATUS    - Status register  
    // 0x08: FSO_STATS - FSO link statistics
    // 0x0C: EBAND_STATS - E-band statistics
    // 0x10: HF_STATS  - HF statistics
    // 0x14: FAILOVER_CFG - Failover configuration
    // 0xFC: VERSION   - IP version
    
    localparam logic [31:0] VERSION = 32'h02_00_00_00;  // v2.0.0
    
    // Write state machine
    logic [ADDR_WIDTH-1:0] waddr;
    logic [1:0] write_state;
    
    always_ff @(posedge clk_250 or negedge rst_n) begin
        if (!rst_n) begin
            ctrl_enable            <= 1'b1;
            ctrl_force_failover    <= 1'b0;
            ctrl_force_link        <= 3'b001;
            ctrl_crypto_enable     <= 1'b1;
            cfg_failover_threshold <= 8'd80;
            cfg_failback_threshold <= 8'd20;
            cfg_monitor_interval   <= 32'd25_000_000;
            node_id                <= 4'd0;
            
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_bresp   <= 2'b00;
            s_axi_bvalid  <= 1'b0;
            write_state   <= 2'b00;
        end else begin
            case (write_state)
                2'b00: begin  // Wait for address
                    s_axi_awready <= 1'b1;
                    s_axi_wready  <= 1'b0;
                    s_axi_bvalid  <= 1'b0;
                    if (s_axi_awvalid && s_axi_awready) begin
                        waddr <= s_axi_awaddr;
                        s_axi_awready <= 1'b0;
                        write_state <= 2'b01;
                    end
                end
                
                2'b01: begin  // Wait for data
                    s_axi_wready <= 1'b1;
                    if (s_axi_wvalid && s_axi_wready) begin
                        s_axi_wready <= 1'b0;
                        
                        // Decode write
                        case (waddr[7:0])
                            8'h00: begin  // CTRL
                                ctrl_enable         <= s_axi_wdata[0];
                                ctrl_force_failover <= s_axi_wdata[1];
                                ctrl_force_link     <= s_axi_wdata[4:2];
                                ctrl_crypto_enable  <= s_axi_wdata[8];
                            end
                            
                            8'h14: begin  // FAILOVER_CFG
                                cfg_failover_threshold <= s_axi_wdata[7:0];
                                cfg_failback_threshold <= s_axi_wdata[15:8];
                                cfg_monitor_interval   <= {s_axi_wdata[31:16], 16'b0};
                            end
                            
                            8'h20: node_id <= s_axi_wdata[3:0];
                            
                            default: ;
                        endcase
                        
                        write_state <= 2'b10;
                    end
                end
                
                2'b10: begin  // Response
                    s_axi_bresp  <= 2'b00;  // OKAY
                    s_axi_bvalid <= 1'b1;
                    if (s_axi_bready) begin
                        s_axi_bvalid <= 1'b0;
                        write_state <= 2'b00;
                    end
                end
                
                default: write_state <= 2'b00;
            endcase
        end
    end
    
    // Read state machine
    logic [1:0] read_state;
    
    always_ff @(posedge clk_250 or negedge rst_n) begin
        if (!rst_n) begin
            s_axi_arready <= 1'b0;
            s_axi_rdata   <= 32'b0;
            s_axi_rresp   <= 2'b00;
            s_axi_rvalid  <= 1'b0;
            read_state    <= 2'b00;
        end else begin
            case (read_state)
                2'b00: begin  // Wait for address
                    s_axi_arready <= 1'b1;
                    s_axi_rvalid  <= 1'b0;
                    if (s_axi_arvalid && s_axi_arready) begin
                        s_axi_arready <= 1'b0;
                        
                        // Decode read
                        case (s_axi_araddr[7:0])
                            8'h00: s_axi_rdata <= {23'b0, ctrl_crypto_enable, 3'b0, 
                                                   ctrl_force_link, ctrl_force_failover, ctrl_enable};
                            
                            8'h04: s_axi_rdata <= {fsm_all_links_down, 14'b0, fsm_failover_in_progress,
                                                   5'b0, hf_link_healthy, eband_link_healthy, fso_link_healthy,
                                                   5'b0, fsm_active_link};
                            
                            8'h08: s_axi_rdata <= {fso_latency[15:0], fso_packet_loss};
                            
                            8'h0C: s_axi_rdata <= {eband_latency[15:0], eband_packet_loss};
                            
                            8'h10: s_axi_rdata <= {hf_latency[15:0], hf_packet_loss};
                            
                            8'h14: s_axi_rdata <= {cfg_monitor_interval[31:16], 
                                                   cfg_failback_threshold, cfg_failover_threshold};
                            
                            8'h18: s_axi_rdata <= {fsm_failover_reason, 24'b0, fsm_current_state};
                            
                            8'hFC: s_axi_rdata <= VERSION;
                            
                            default: s_axi_rdata <= 32'hDEADBEEF;
                        endcase
                        
                        read_state <= 2'b01;
                    end
                end
                
                2'b01: begin  // Send response
                    s_axi_rresp  <= 2'b00;  // OKAY
                    s_axi_rvalid <= 1'b1;
                    if (s_axi_rready) begin
                        s_axi_rvalid <= 1'b0;
                        read_state <= 2'b00;
                    end
                end
                
                default: read_state <= 2'b00;
            endcase
        end
    end
    
    //-------------------------------------------------------------------------
    // Data Path Multiplexing (simplified - full version would have FIFOs)
    //-------------------------------------------------------------------------
    
    // TX path: Route application data to active link
    always_comb begin
        fso_tx_data   = '0;
        fso_tx_ctrl   = '0;
        fso_tx_valid  = 1'b0;
        eband_tx_tdata  = '0;
        eband_tx_tvalid = 1'b0;
        eband_tx_tlast  = 1'b0;
        s_axis_tready = 1'b0;
        
        case (fsm_active_link)
            3'b001: begin  // FSO
                fso_tx_data  = s_axis_tdata[63:0];
                fso_tx_valid = s_axis_tvalid;
                s_axis_tready = fso_tx_ready;
            end
            
            3'b010: begin  // E-band
                eband_tx_tdata  = s_axis_tdata[63:0];
                eband_tx_tvalid = s_axis_tvalid;
                eband_tx_tlast  = s_axis_tlast;
                s_axis_tready   = eband_tx_tready;
            end
            
            3'b100: begin  // HF (handled via UART separately)
                s_axis_tready = 1'b1;  // Accept but route to HF UART
            end
            
            default: s_axis_tready = 1'b0;
        endcase
    end
    
    // RX path: Route data from active link to application
    always_comb begin
        m_axis_tdata  = '0;
        m_axis_tvalid = 1'b0;
        m_axis_tlast  = 1'b0;
        m_axis_tsrc   = node_id;
        m_axis_tuser  = '0;
        
        case (fsm_active_link)
            3'b001: begin  // FSO
                m_axis_tdata[63:0] = fso_rx_data;
                m_axis_tvalid      = fso_rx_valid;
                m_axis_tlast       = 1'b1;
            end
            
            3'b010: begin  // E-band
                m_axis_tdata[63:0] = eband_rx_tdata;
                m_axis_tvalid      = eband_rx_tvalid;
                m_axis_tlast       = eband_rx_tlast;
            end
            
            3'b100: begin  // HF
                // HF data comes via UART, handled separately
                m_axis_tvalid = 1'b0;
            end
            
            default: m_axis_tvalid = 1'b0;
        endcase
    end
    
    //-------------------------------------------------------------------------
    // HF UART (simplified - would need full UART module)
    //-------------------------------------------------------------------------
    assign hf_uart_tx = 1'b1;  // Idle high
    assign hf_ptt     = 1'b0;  // Not transmitting
    assign fso_mon_tx_ready = 1'b1;
    assign eband_mon_tx_ready = 1'b1;
    assign hf_mon_tx_ready = 1'b1;

endmodule
