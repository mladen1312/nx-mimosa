//═══════════════════════════════════════════════════════════════════════════════
// NX-MIMOSA AXI4 SOC WRAPPER
// Integration wrapper for Xilinx RFSoC platforms
// Target: ZU48DR, ZU28DR (Zynq UltraScale+ RFSoC)
//═══════════════════════════════════════════════════════════════════════════════
//
// AXI4-Lite: Control/Status registers
// AXI4-Stream: High-bandwidth data path (measurements in, tracks out)
// AXI4-Full: DMA interface for bulk data transfer
//
// Register Map:
// 0x00: CONTROL      - [0]=enable, [1]=reset, [7:4]=mode
// 0x04: STATUS       - [0]=busy, [1]=error, [15:8]=n_tracks
// 0x08: CONFIG0      - dt (Q16.16 fixed-point)
// 0x0C: CONFIG1      - sigma_meas (Q16.16)
// 0x10: TRACK_COUNT  - Number of active tracks
// 0x14: VERSION      - Hardware version
// 0x18: CAPABILITIES - Feature flags
// 0x1C: IRQ_STATUS   - Interrupt status
// 0x20: IRQ_ENABLE   - Interrupt enable mask
//
// Author: Dr. Mladen Mešter / Nexellum d.o.o.
// License: AGPL v3 / Commercial
//═══════════════════════════════════════════════════════════════════════════════

`timescale 1ns / 1ps
`default_nettype none

module nx_mimosa_axi_wrapper #(
    // AXI Parameters
    parameter integer C_S_AXI_DATA_WIDTH = 32,
    parameter integer C_S_AXI_ADDR_WIDTH = 8,
    parameter integer C_AXIS_TDATA_WIDTH = 128,
    
    // NX-MIMOSA Parameters
    parameter integer MAX_TRACKS = 256,
    parameter integer STATE_DIM = 6,
    parameter integer MEAS_DIM = 3,
    parameter integer N_MODES = 3,
    
    // Fixed-point format
    parameter integer FRAC_BITS = 16
)(
    // ═══════════════════════════════════════════════════════════════════════
    // CLOCK AND RESET
    // ═══════════════════════════════════════════════════════════════════════
    input  wire                                 aclk,
    input  wire                                 aresetn,
    
    // ═══════════════════════════════════════════════════════════════════════
    // AXI4-LITE SLAVE (Control/Status)
    // ═══════════════════════════════════════════════════════════════════════
    // Write Address Channel
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]       s_axi_awaddr,
    input  wire [2:0]                          s_axi_awprot,
    input  wire                                s_axi_awvalid,
    output wire                                s_axi_awready,
    
    // Write Data Channel
    input  wire [C_S_AXI_DATA_WIDTH-1:0]       s_axi_wdata,
    input  wire [(C_S_AXI_DATA_WIDTH/8)-1:0]   s_axi_wstrb,
    input  wire                                s_axi_wvalid,
    output wire                                s_axi_wready,
    
    // Write Response Channel
    output wire [1:0]                          s_axi_bresp,
    output wire                                s_axi_bvalid,
    input  wire                                s_axi_bready,
    
    // Read Address Channel
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]       s_axi_araddr,
    input  wire [2:0]                          s_axi_arprot,
    input  wire                                s_axi_arvalid,
    output wire                                s_axi_arready,
    
    // Read Data Channel
    output wire [C_S_AXI_DATA_WIDTH-1:0]       s_axi_rdata,
    output wire [1:0]                          s_axi_rresp,
    output wire                                s_axi_rvalid,
    input  wire                                s_axi_rready,
    
    // ═══════════════════════════════════════════════════════════════════════
    // AXI4-STREAM SLAVE (Measurement Input)
    // ═══════════════════════════════════════════════════════════════════════
    input  wire [C_AXIS_TDATA_WIDTH-1:0]       s_axis_meas_tdata,
    input  wire [(C_AXIS_TDATA_WIDTH/8)-1:0]   s_axis_meas_tkeep,
    input  wire                                s_axis_meas_tlast,
    input  wire                                s_axis_meas_tvalid,
    output wire                                s_axis_meas_tready,
    
    // ═══════════════════════════════════════════════════════════════════════
    // AXI4-STREAM MASTER (Track Output)
    // ═══════════════════════════════════════════════════════════════════════
    output wire [C_AXIS_TDATA_WIDTH-1:0]       m_axis_track_tdata,
    output wire [(C_AXIS_TDATA_WIDTH/8)-1:0]   m_axis_track_tkeep,
    output wire                                m_axis_track_tlast,
    output wire                                m_axis_track_tvalid,
    input  wire                                m_axis_track_tready,
    
    // ═══════════════════════════════════════════════════════════════════════
    // INTERRUPT
    // ═══════════════════════════════════════════════════════════════════════
    output wire                                irq
);

    // ═══════════════════════════════════════════════════════════════════════
    // LOCAL PARAMETERS
    // ═══════════════════════════════════════════════════════════════════════
    
    localparam ADDR_LSB = (C_S_AXI_DATA_WIDTH/32) + 1;
    localparam OPT_MEM_ADDR_BITS = 5;
    
    // Register addresses
    localparam REG_CONTROL      = 6'h00;
    localparam REG_STATUS       = 6'h04;
    localparam REG_CONFIG0      = 6'h08;
    localparam REG_CONFIG1      = 6'h0C;
    localparam REG_TRACK_COUNT  = 6'h10;
    localparam REG_VERSION      = 6'h14;
    localparam REG_CAPABILITIES = 6'h18;
    localparam REG_IRQ_STATUS   = 6'h1C;
    localparam REG_IRQ_ENABLE   = 6'h20;
    
    // Version
    localparam VERSION_MAJOR = 8'd1;
    localparam VERSION_MINOR = 8'd0;
    localparam VERSION_PATCH = 8'd0;
    
    // ═══════════════════════════════════════════════════════════════════════
    // REGISTERS
    // ═══════════════════════════════════════════════════════════════════════
    
    // Control register
    reg [31:0] reg_control;
    wire       ctrl_enable     = reg_control[0];
    wire       ctrl_reset      = reg_control[1];
    wire [3:0] ctrl_mode       = reg_control[7:4];
    
    // Configuration registers
    reg [31:0] reg_config0;     // dt (Q16.16)
    reg [31:0] reg_config1;     // sigma_meas (Q16.16)
    
    // Status register (read-only)
    wire [31:0] reg_status;
    wire        status_busy;
    wire        status_error;
    wire [7:0]  status_n_tracks;
    
    // Interrupt registers
    reg [31:0] reg_irq_enable;
    wire [31:0] reg_irq_status;
    
    // Track count
    wire [31:0] reg_track_count;
    
    // ═══════════════════════════════════════════════════════════════════════
    // AXI4-LITE STATE MACHINE
    // ═══════════════════════════════════════════════════════════════════════
    
    reg axi_awready;
    reg axi_wready;
    reg [1:0] axi_bresp;
    reg axi_bvalid;
    reg axi_arready;
    reg [C_S_AXI_DATA_WIDTH-1:0] axi_rdata;
    reg [1:0] axi_rresp;
    reg axi_rvalid;
    
    reg [C_S_AXI_ADDR_WIDTH-1:0] axi_awaddr;
    reg [C_S_AXI_ADDR_WIDTH-1:0] axi_araddr;
    reg aw_en;
    
    // Assign outputs
    assign s_axi_awready = axi_awready;
    assign s_axi_wready  = axi_wready;
    assign s_axi_bresp   = axi_bresp;
    assign s_axi_bvalid  = axi_bvalid;
    assign s_axi_arready = axi_arready;
    assign s_axi_rdata   = axi_rdata;
    assign s_axi_rresp   = axi_rresp;
    assign s_axi_rvalid  = axi_rvalid;
    
    // Write Address Ready
    always @(posedge aclk) begin
        if (!aresetn) begin
            axi_awready <= 1'b0;
            aw_en <= 1'b1;
        end else begin
            if (~axi_awready && s_axi_awvalid && s_axi_wvalid && aw_en) begin
                axi_awready <= 1'b1;
                aw_en <= 1'b0;
            end else if (s_axi_bready && axi_bvalid) begin
                aw_en <= 1'b1;
                axi_awready <= 1'b0;
            end else begin
                axi_awready <= 1'b0;
            end
        end
    end
    
    // Write Address Latch
    always @(posedge aclk) begin
        if (!aresetn)
            axi_awaddr <= 0;
        else if (~axi_awready && s_axi_awvalid && s_axi_wvalid && aw_en)
            axi_awaddr <= s_axi_awaddr;
    end
    
    // Write Data Ready
    always @(posedge aclk) begin
        if (!aresetn)
            axi_wready <= 1'b0;
        else if (~axi_wready && s_axi_wvalid && s_axi_awvalid && aw_en)
            axi_wready <= 1'b1;
        else
            axi_wready <= 1'b0;
    end
    
    // Register Write
    wire slv_reg_wren = axi_wready && s_axi_wvalid && axi_awready && s_axi_awvalid;
    
    always @(posedge aclk) begin
        if (!aresetn) begin
            reg_control   <= 32'h0;
            reg_config0   <= 32'h00010000;  // dt = 1.0 (Q16.16)
            reg_config1   <= 32'h001E0000;  // sigma = 30.0 (Q16.16)
            reg_irq_enable <= 32'h0;
        end else if (slv_reg_wren) begin
            case (axi_awaddr[5:0])
                REG_CONTROL:    reg_control   <= s_axi_wdata;
                REG_CONFIG0:    reg_config0   <= s_axi_wdata;
                REG_CONFIG1:    reg_config1   <= s_axi_wdata;
                REG_IRQ_ENABLE: reg_irq_enable <= s_axi_wdata;
                default: ;
            endcase
        end
    end
    
    // Write Response
    always @(posedge aclk) begin
        if (!aresetn) begin
            axi_bvalid <= 0;
            axi_bresp  <= 2'b0;
        end else if (axi_awready && s_axi_awvalid && ~axi_bvalid && axi_wready && s_axi_wvalid) begin
            axi_bvalid <= 1'b1;
            axi_bresp  <= 2'b0;  // OKAY
        end else if (s_axi_bready && axi_bvalid) begin
            axi_bvalid <= 1'b0;
        end
    end
    
    // Read Address Ready
    always @(posedge aclk) begin
        if (!aresetn) begin
            axi_arready <= 1'b0;
            axi_araddr  <= 0;
        end else if (~axi_arready && s_axi_arvalid) begin
            axi_arready <= 1'b1;
            axi_araddr  <= s_axi_araddr;
        end else begin
            axi_arready <= 1'b0;
        end
    end
    
    // Read Data
    wire slv_reg_rden = axi_arready & s_axi_arvalid & ~axi_rvalid;
    
    always @(posedge aclk) begin
        if (!aresetn) begin
            axi_rvalid <= 0;
            axi_rresp  <= 0;
        end else if (axi_arready && s_axi_arvalid && ~axi_rvalid) begin
            axi_rvalid <= 1'b1;
            axi_rresp  <= 2'b0;  // OKAY
        end else if (axi_rvalid && s_axi_rready) begin
            axi_rvalid <= 1'b0;
        end
    end
    
    // Register Read Mux
    always @(*) begin
        case (axi_araddr[5:0])
            REG_CONTROL:      axi_rdata = reg_control;
            REG_STATUS:       axi_rdata = reg_status;
            REG_CONFIG0:      axi_rdata = reg_config0;
            REG_CONFIG1:      axi_rdata = reg_config1;
            REG_TRACK_COUNT:  axi_rdata = reg_track_count;
            REG_VERSION:      axi_rdata = {8'h0, VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH};
            REG_CAPABILITIES: axi_rdata = {16'h0, MAX_TRACKS[15:0]};
            REG_IRQ_STATUS:   axi_rdata = reg_irq_status;
            REG_IRQ_ENABLE:   axi_rdata = reg_irq_enable;
            default:          axi_rdata = 32'hDEADBEEF;
        endcase
    end
    
    // ═══════════════════════════════════════════════════════════════════════
    // NX-MIMOSA CORE INSTANCE
    // ═══════════════════════════════════════════════════════════════════════
    
    // Measurement FIFO interface
    wire                        meas_valid;
    wire                        meas_ready;
    wire [MEAS_DIM*32-1:0]      meas_data;
    wire [15:0]                 meas_track_id;
    
    // Track output interface
    wire                        track_valid;
    wire                        track_ready;
    wire [STATE_DIM*32-1:0]     track_state;
    wire [15:0]                 track_id;
    wire [N_MODES*16-1:0]       track_mode_probs;
    
    // AXI-Stream to internal interface conversion
    assign meas_valid = s_axis_meas_tvalid & ctrl_enable;
    assign s_axis_meas_tready = meas_ready & ctrl_enable;
    assign meas_data = s_axis_meas_tdata[MEAS_DIM*32-1:0];
    assign meas_track_id = s_axis_meas_tdata[127:112];
    
    // Core instantiation
    // Note: This instantiates the NX-MIMOSA tracking core
    // The actual core would be imported from nx_mimosa_top.sv
    
    /*
    nx_mimosa_top #(
        .MAX_TRACKS(MAX_TRACKS),
        .STATE_DIM(STATE_DIM),
        .MEAS_DIM(MEAS_DIM),
        .N_MODES(N_MODES),
        .FRAC_BITS(FRAC_BITS)
    ) u_core (
        .clk(aclk),
        .rst_n(aresetn & ~ctrl_reset),
        .enable(ctrl_enable),
        
        // Configuration
        .cfg_dt(reg_config0),
        .cfg_sigma(reg_config1),
        .cfg_mode(ctrl_mode),
        
        // Measurement input
        .meas_valid(meas_valid),
        .meas_ready(meas_ready),
        .meas_data(meas_data),
        .meas_track_id(meas_track_id),
        
        // Track output
        .track_valid(track_valid),
        .track_ready(track_ready),
        .track_state(track_state),
        .track_id(track_id),
        .track_mode_probs(track_mode_probs),
        
        // Status
        .status_busy(status_busy),
        .status_error(status_error),
        .status_n_tracks(status_n_tracks)
    );
    */
    
    // Placeholder for when core is not instantiated
    assign meas_ready = 1'b1;
    assign status_busy = 1'b0;
    assign status_error = 1'b0;
    assign status_n_tracks = 8'd0;
    assign track_valid = 1'b0;
    assign track_state = '0;
    assign track_id = 16'd0;
    assign track_mode_probs = '0;
    
    // Status register assembly
    assign reg_status = {16'h0, status_n_tracks, 6'h0, status_error, status_busy};
    assign reg_track_count = {24'h0, status_n_tracks};
    
    // ═══════════════════════════════════════════════════════════════════════
    // OUTPUT AXI-STREAM
    // ═══════════════════════════════════════════════════════════════════════
    
    assign m_axis_track_tdata = {track_mode_probs, track_id, track_state};
    assign m_axis_track_tkeep = {(C_AXIS_TDATA_WIDTH/8){1'b1}};
    assign m_axis_track_tlast = 1'b1;  // Single-beat transfers
    assign m_axis_track_tvalid = track_valid;
    assign track_ready = m_axis_track_tready;
    
    // ═══════════════════════════════════════════════════════════════════════
    // INTERRUPT LOGIC
    // ═══════════════════════════════════════════════════════════════════════
    
    // Interrupt sources
    wire irq_track_ready = track_valid;
    wire irq_error = status_error;
    
    assign reg_irq_status = {30'h0, irq_error, irq_track_ready};
    assign irq = |(reg_irq_status & reg_irq_enable);

endmodule

`default_nettype wire


//═══════════════════════════════════════════════════════════════════════════════
// AXI4-STREAM MEASUREMENT PACKETIZER
// Converts raw ADC samples to measurement packets
//═══════════════════════════════════════════════════════════════════════════════

module axis_meas_packetizer #(
    parameter integer TDATA_WIDTH = 128,
    parameter integer MEAS_DIM = 3
)(
    input  wire                     aclk,
    input  wire                     aresetn,
    
    // Raw measurement input (range, azimuth, elevation)
    input  wire [31:0]              range_m,       // Q16.16
    input  wire [31:0]              azimuth_rad,   // Q16.16
    input  wire [31:0]              elevation_rad, // Q16.16
    input  wire [15:0]              track_id,
    input  wire                     meas_valid_in,
    output wire                     meas_ready_out,
    
    // AXI-Stream output
    output wire [TDATA_WIDTH-1:0]   m_axis_tdata,
    output wire                     m_axis_tvalid,
    input  wire                     m_axis_tready
);

    // Convert spherical to Cartesian
    // x = r * cos(el) * cos(az)
    // y = r * cos(el) * sin(az)
    // z = r * sin(el)
    
    // For simplicity, pass through as-is (conversion done in SW or DSP)
    wire [31:0] x_pos = range_m;     // Placeholder
    wire [31:0] y_pos = azimuth_rad; // Placeholder
    wire [31:0] z_pos = elevation_rad; // Placeholder
    
    assign m_axis_tdata = {16'h0, track_id, 32'h0, z_pos, y_pos, x_pos};
    assign m_axis_tvalid = meas_valid_in;
    assign meas_ready_out = m_axis_tready;

endmodule


//═══════════════════════════════════════════════════════════════════════════════
// AXI4-STREAM TRACK DEPACKETIZER
// Converts track packets to individual signals
//═══════════════════════════════════════════════════════════════════════════════

module axis_track_depacketizer #(
    parameter integer TDATA_WIDTH = 128,
    parameter integer STATE_DIM = 6,
    parameter integer N_MODES = 3
)(
    input  wire                     aclk,
    input  wire                     aresetn,
    
    // AXI-Stream input
    input  wire [TDATA_WIDTH-1:0]   s_axis_tdata,
    input  wire                     s_axis_tvalid,
    output wire                     s_axis_tready,
    
    // Track output
    output wire [31:0]              pos_x,
    output wire [31:0]              pos_y,
    output wire [31:0]              pos_z,
    output wire [31:0]              vel_x,
    output wire [31:0]              vel_y,
    output wire [31:0]              vel_z,
    output wire [15:0]              track_id,
    output wire [15:0]              mode_prob_0,
    output wire [15:0]              mode_prob_1,
    output wire [15:0]              mode_prob_2,
    output wire                     track_valid
);

    assign s_axis_tready = 1'b1;  // Always ready
    
    // Unpack state vector
    assign pos_x = s_axis_tdata[31:0];
    assign pos_y = s_axis_tdata[63:32];
    assign pos_z = s_axis_tdata[95:64];
    assign vel_x = s_axis_tdata[127:96];
    // Note: Full 6D state would require wider bus
    assign vel_y = 32'h0;
    assign vel_z = 32'h0;
    
    assign track_id = 16'h0;  // From extended fields
    assign mode_prob_0 = 16'h0;
    assign mode_prob_1 = 16'h0;
    assign mode_prob_2 = 16'h0;
    
    assign track_valid = s_axis_tvalid;

endmodule
