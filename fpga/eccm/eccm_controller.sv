//-----------------------------------------------------------------------------
// QEDMMA v2.0 ECCM Controller Top-Level
// Author: Dr. Mladen Mešter
// Copyright (c) 2026 Dr. Mladen Mešter - All Rights Reserved
//
// Description:
//   Top-level Electronic Counter-Countermeasures (ECCM) controller.
//   Orchestrates all ECCM functions:
//   - ML-assisted CFAR detection
//   - Adaptive integration control
//   - Jammer localization (Home-on-Jam)
//   - Deception rejection
//   - Cognitive mode selection
//
// [REQ-ECCM-100] Coordinate all ECCM subsystems
// [REQ-ECCM-101] Provide unified threat assessment
// [REQ-ECCM-102] Support autonomous and manual modes
//-----------------------------------------------------------------------------

`timescale 1ns / 1ps

module eccm_controller #(
    parameter int MAX_RX_NODES = 6,
    parameter int MAX_JAMMERS  = 8,
    parameter int DATA_WIDTH   = 32,
    parameter int COORD_WIDTH  = 32,
    parameter int TIME_WIDTH   = 64
)(
    input  logic        clk,
    input  logic        rst_n,
    
    //-------------------------------------------------------------------------
    // AXI-Lite Register Interface
    //-------------------------------------------------------------------------
    input  logic [15:0] s_axi_awaddr,
    input  logic        s_axi_awvalid,
    output logic        s_axi_awready,
    input  logic [31:0] s_axi_wdata,
    input  logic [3:0]  s_axi_wstrb,
    input  logic        s_axi_wvalid,
    output logic        s_axi_wready,
    output logic [1:0]  s_axi_bresp,
    output logic        s_axi_bvalid,
    input  logic        s_axi_bready,
    input  logic [15:0] s_axi_araddr,
    input  logic        s_axi_arvalid,
    output logic        s_axi_arready,
    output logic [31:0] s_axi_rdata,
    output logic [1:0]  s_axi_rresp,
    output logic        s_axi_rvalid,
    input  logic        s_axi_rready,
    
    //-------------------------------------------------------------------------
    // Range-Doppler Map Input (from Signal Processor)
    //-------------------------------------------------------------------------
    input  logic [DATA_WIDTH-1:0]  rd_map_data,
    input  logic [11:0]            rd_map_range_idx,
    input  logic [8:0]             rd_map_doppler_idx,
    input  logic                   rd_map_valid,
    output logic                   rd_map_ready,
    
    //-------------------------------------------------------------------------
    // Detection Output (to Track Manager)
    //-------------------------------------------------------------------------
    output logic [11:0]            det_range_idx,
    output logic [8:0]             det_doppler_idx,
    output logic [DATA_WIDTH-1:0]  det_magnitude,
    output logic [7:0]             det_snr,
    output logic [1:0]             det_class,
    output logic                   det_valid,
    input  logic                   det_ready,
    
    //-------------------------------------------------------------------------
    // Jammer Measurements Input (from each Rx node)
    //-------------------------------------------------------------------------
    input  logic [MAX_RX_NODES-1:0]       jam_detected_nodes,
    input  logic [DATA_WIDTH-1:0]         jam_power_nodes [MAX_RX_NODES],
    input  logic [TIME_WIDTH-1:0]         jam_toa_nodes [MAX_RX_NODES],
    input  logic [15:0]                   jam_aoa_nodes [MAX_RX_NODES],
    input  logic                          jam_measurements_valid,
    
    //-------------------------------------------------------------------------
    // Rx Node Configuration
    //-------------------------------------------------------------------------
    input  logic signed [COORD_WIDTH-1:0] rx_pos_x [MAX_RX_NODES],
    input  logic signed [COORD_WIDTH-1:0] rx_pos_y [MAX_RX_NODES],
    input  logic signed [COORD_WIDTH-1:0] rx_pos_z [MAX_RX_NODES],
    input  logic [MAX_RX_NODES-1:0]       rx_node_active,
    
    //-------------------------------------------------------------------------
    // Waveform Generator Control Output
    //-------------------------------------------------------------------------
    output logic [7:0]             wfg_n_pulses,
    output logic [31:0]            wfg_t_chirp_us,
    output logic [31:0]            wfg_pri_us,
    output logic [31:0]            wfg_cpi_ms,
    output logic                   wfg_update_valid,
    
    //-------------------------------------------------------------------------
    // Home-on-Jam Cueing Output
    //-------------------------------------------------------------------------
    output logic                   hoj_cue_valid,
    output logic [15:0]            hoj_azimuth,
    output logic [15:0]            hoj_elevation,
    output logic [COORD_WIDTH-1:0] hoj_range_estimate,
    output logic [7:0]             hoj_confidence,
    
    //-------------------------------------------------------------------------
    // Jammer Track Output
    //-------------------------------------------------------------------------
    output logic [2:0]                    jammer_id,
    output logic signed [COORD_WIDTH-1:0] jammer_pos_x,
    output logic signed [COORD_WIDTH-1:0] jammer_pos_y,
    output logic signed [COORD_WIDTH-1:0] jammer_pos_z,
    output logic                          jammer_track_valid,
    
    //-------------------------------------------------------------------------
    // Status Output
    //-------------------------------------------------------------------------
    output logic                   eccm_active,
    output logic [1:0]             eccm_mode,           // 0=passive, 1=adaptive, 2=aggressive
    output logic [1:0]             integration_mode,
    output logic [7:0]             jam_duty_cycle,
    output logic                   jam_detected,
    output logic [1:0]             jam_type,
    output logic [7:0]             effective_eccm_gain_db
);

    //-------------------------------------------------------------------------
    // Register Addresses
    //-------------------------------------------------------------------------
    localparam logic [15:0] REG_CTRL           = 16'h0000;
    localparam logic [15:0] REG_STATUS         = 16'h0004;
    localparam logic [15:0] REG_JAM_THRESH_HI  = 16'h0008;
    localparam logic [15:0] REG_JAM_THRESH_LO  = 16'h000C;
    localparam logic [15:0] REG_INT_CFG        = 16'h0010;
    localparam logic [15:0] REG_CFAR_CFG       = 16'h0014;
    localparam logic [15:0] REG_JAM_POWER      = 16'h0020;
    localparam logic [15:0] REG_JAM_DUTY       = 16'h0024;
    localparam logic [15:0] REG_JAM_POS_X      = 16'h0028;
    localparam logic [15:0] REG_JAM_POS_Y      = 16'h002C;
    localparam logic [15:0] REG_STATS_DET      = 16'h0040;
    localparam logic [15:0] REG_STATS_JAM      = 16'h0044;
    localparam logic [15:0] REG_STATS_HOJ      = 16'h0048;
    localparam logic [15:0] REG_VERSION        = 16'h00FC;
    
    //-------------------------------------------------------------------------
    // Configuration Registers
    //-------------------------------------------------------------------------
    logic        cfg_eccm_enable;
    logic        cfg_ml_cfar_enable;
    logic        cfg_adaptive_int_enable;
    logic        cfg_hoj_enable;
    logic [1:0]  cfg_eccm_mode;
    logic [7:0]  cfg_js_threshold_high;
    logic [7:0]  cfg_js_threshold_low;
    logic [DATA_WIDTH-1:0] cfg_cfar_threshold;
    logic [7:0]  cfg_alpha_clutter;
    logic [7:0]  cfg_alpha_jam;
    
    //-------------------------------------------------------------------------
    // Internal Signals
    //-------------------------------------------------------------------------
    // ML-CFAR outputs
    logic [DATA_WIDTH-1:0] cfar_jam_power;
    logic [7:0]            cfar_jam_duty;
    logic                  cfar_jam_detected;
    logic [1:0]            cfar_jam_type;
    logic [127:0]          cfar_ml_features;
    logic                  cfar_ml_features_valid;
    logic [31:0]           cfar_total_dets;
    logic [31:0]           cfar_jam_dets;
    
    // Integration controller outputs
    logic [7:0]            int_n_pulses;
    logic [31:0]           int_t_chirp;
    logic [31:0]           int_pri;
    logic [31:0]           int_cpi;
    logic                  int_update_valid;
    logic [1:0]            int_mode;
    logic [7:0]            int_gain_db;
    
    // Jammer localizer outputs
    logic [2:0]            loc_jammer_id;
    logic signed [COORD_WIDTH-1:0] loc_jammer_x, loc_jammer_y, loc_jammer_z;
    logic                  loc_jammer_valid;
    logic                  loc_hoj_valid;
    logic [15:0]           loc_hoj_az, loc_hoj_el;
    logic [COORD_WIDTH-1:0] loc_hoj_range;
    logic [7:0]            loc_hoj_conf;
    logic [31:0]           loc_count;
    logic [31:0]           hoj_count;
    
    //-------------------------------------------------------------------------
    // ML-CFAR Engine Instance
    //-------------------------------------------------------------------------
    ml_cfar_engine #(
        .RANGE_BINS(4096),
        .DOPPLER_BINS(512),
        .DATA_WIDTH(DATA_WIDTH)
    ) u_ml_cfar (
        .clk(clk),
        .rst_n(rst_n),
        
        // Inputs
        .rd_map_data(rd_map_data),
        .rd_map_range_idx(rd_map_range_idx),
        .rd_map_doppler_idx(rd_map_doppler_idx),
        .rd_map_valid(rd_map_valid),
        .rd_map_ready(rd_map_ready),
        
        // Detection outputs
        .det_range_idx(det_range_idx),
        .det_doppler_idx(det_doppler_idx),
        .det_magnitude(det_magnitude),
        .det_snr(det_snr),
        .det_class(det_class),
        .det_valid(det_valid),
        .det_ready(det_ready),
        
        // ML features
        .ml_features(cfar_ml_features),
        .ml_features_valid(cfar_ml_features_valid),
        
        // Jamming metrics
        .jam_power_estimate(cfar_jam_power),
        .jam_duty_cycle(cfar_jam_duty),
        .jam_detected(cfar_jam_detected),
        .jam_type(cfar_jam_type),
        
        // Configuration
        .cfg_pfa_threshold(cfg_cfar_threshold),
        .cfg_alpha_clutter(cfg_alpha_clutter),
        .cfg_alpha_jam(cfg_alpha_jam),
        .cfg_ml_enable(cfg_ml_cfar_enable),
        .cfg_jam_threshold(cfg_js_threshold_high),
        
        // Stats
        .total_detections(cfar_total_dets),
        .jam_detections(cfar_jam_dets),
        .clutter_detections()
    );
    
    //-------------------------------------------------------------------------
    // Integration Controller Instance
    //-------------------------------------------------------------------------
    integration_controller #(
        .DATA_WIDTH(DATA_WIDTH)
    ) u_int_ctrl (
        .clk(clk),
        .rst_n(rst_n),
        
        // Jamming metrics
        .jam_power_estimate(cfar_jam_power),
        .jam_duty_cycle(cfar_jam_duty),
        .jam_detected(cfar_jam_detected),
        .jam_type(cfar_jam_type),
        
        // Signal metrics
        .signal_power_estimate(det_magnitude),
        .current_snr_db(det_snr),
        
        // Waveform outputs
        .cfg_n_pulses(int_n_pulses),
        .cfg_t_chirp_us(int_t_chirp),
        .cfg_pri_us(int_pri),
        .cfg_cpi_ms(int_cpi),
        .cfg_update_valid(int_update_valid),
        
        // Mode
        .integration_mode(int_mode),
        .effective_gain_db(int_gain_db),
        .mode_transition(),
        
        // Track manager
        .active_track_count(10'd0),
        .priority_track_present(1'b0),
        .track_update_inhibit(),
        
        // Configuration
        .cfg_js_threshold_high(cfg_js_threshold_high),
        .cfg_js_threshold_low(cfg_js_threshold_low),
        .cfg_min_snr_target(8'd56),  // 14 dB × 4
        .cfg_auto_mode_enable(cfg_adaptive_int_enable),
        .cfg_manual_mode(2'b00),
        
        // Stats
        .current_js_ratio_db(),
        .mode_transitions_count(),
        .time_in_jam_mode_ms()
    );
    
    //-------------------------------------------------------------------------
    // Jammer Localizer Instance
    //-------------------------------------------------------------------------
    jammer_localizer #(
        .MAX_RX_NODES(MAX_RX_NODES),
        .MAX_JAMMERS(MAX_JAMMERS),
        .COORD_WIDTH(COORD_WIDTH),
        .TIME_WIDTH(TIME_WIDTH)
    ) u_jam_loc (
        .clk(clk),
        .rst_n(rst_n),
        
        // Measurements
        .jam_detected(jam_detected_nodes),
        .jam_power(jam_power_nodes),
        .jam_toa(jam_toa_nodes),
        .jam_aoa(jam_aoa_nodes),
        .jam_measurements_valid(jam_measurements_valid && cfg_hoj_enable),
        
        // Rx positions
        .rx_pos_x(rx_pos_x),
        .rx_pos_y(rx_pos_y),
        .rx_pos_z(rx_pos_z),
        .rx_node_active(rx_node_active),
        
        // Jammer track
        .jammer_id(loc_jammer_id),
        .jammer_pos_x(loc_jammer_x),
        .jammer_pos_y(loc_jammer_y),
        .jammer_pos_z(loc_jammer_z),
        .jammer_vel_x(),
        .jammer_vel_y(),
        .jammer_vel_z(),
        .jammer_cep(),
        .jammer_erp(),
        .jammer_track_valid(loc_jammer_valid),
        .active_jammer_mask(),
        
        // HOJ cueing
        .hoj_cue_valid(loc_hoj_valid),
        .hoj_azimuth(loc_hoj_az),
        .hoj_elevation(loc_hoj_el),
        .hoj_range_estimate(loc_hoj_range),
        .hoj_confidence(loc_hoj_conf),
        
        // Config
        .cfg_tdoa_tolerance(64'd100),  // 100 ps
        .cfg_max_range(32'd600_000),   // 600 km
        .cfg_min_nodes(8'd3),
        
        // Stats
        .localizations_performed(loc_count),
        .hoj_cues_generated(hoj_count),
        .gdop()
    );
    
    //-------------------------------------------------------------------------
    // Output Assignments
    //-------------------------------------------------------------------------
    // Waveform generator
    assign wfg_n_pulses     = int_n_pulses;
    assign wfg_t_chirp_us   = int_t_chirp;
    assign wfg_pri_us       = int_pri;
    assign wfg_cpi_ms       = int_cpi;
    assign wfg_update_valid = int_update_valid && cfg_eccm_enable;
    
    // HOJ cueing
    assign hoj_cue_valid    = loc_hoj_valid && cfg_hoj_enable;
    assign hoj_azimuth      = loc_hoj_az;
    assign hoj_elevation    = loc_hoj_el;
    assign hoj_range_estimate = loc_hoj_range;
    assign hoj_confidence   = loc_hoj_conf;
    
    // Jammer track
    assign jammer_id        = loc_jammer_id;
    assign jammer_pos_x     = loc_jammer_x;
    assign jammer_pos_y     = loc_jammer_y;
    assign jammer_pos_z     = loc_jammer_z;
    assign jammer_track_valid = loc_jammer_valid && cfg_hoj_enable;
    
    // Status
    assign eccm_active      = cfg_eccm_enable;
    assign eccm_mode        = cfg_eccm_mode;
    assign integration_mode = int_mode;
    assign jam_duty_cycle   = cfar_jam_duty;
    assign jam_detected     = cfar_jam_detected;
    assign jam_type         = cfar_jam_type;
    assign effective_eccm_gain_db = int_gain_db;
    
    //-------------------------------------------------------------------------
    // AXI-Lite Register Interface
    //-------------------------------------------------------------------------
    logic [15:0] axi_awaddr_reg;
    logic [15:0] axi_araddr_reg;
    
    // Write channel
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axi_awready <= 1'b1;
            s_axi_wready  <= 1'b0;
            s_axi_bvalid  <= 1'b0;
            s_axi_bresp   <= 2'b00;
            axi_awaddr_reg <= '0;
            
            // Default config
            cfg_eccm_enable       <= 1'b1;
            cfg_ml_cfar_enable    <= 1'b1;
            cfg_adaptive_int_enable <= 1'b1;
            cfg_hoj_enable        <= 1'b1;
            cfg_eccm_mode         <= 2'b01;  // Adaptive
            cfg_js_threshold_high <= 8'd48;  // 12 dB × 4
            cfg_js_threshold_low  <= 8'd24;  // 6 dB × 4
            cfg_cfar_threshold    <= 32'd1000;
            cfg_alpha_clutter     <= 8'd32;  // 2.0 in Q4.4
            cfg_alpha_jam         <= 8'd64;  // 4.0 in Q4.4
        end else begin
            // Address phase
            if (s_axi_awvalid && s_axi_awready) begin
                axi_awaddr_reg <= s_axi_awaddr;
                s_axi_awready  <= 1'b0;
                s_axi_wready   <= 1'b1;
            end
            
            // Data phase
            if (s_axi_wvalid && s_axi_wready) begin
                s_axi_wready <= 1'b0;
                s_axi_bvalid <= 1'b1;
                
                case (axi_awaddr_reg)
                    REG_CTRL: begin
                        cfg_eccm_enable       <= s_axi_wdata[0];
                        cfg_ml_cfar_enable    <= s_axi_wdata[1];
                        cfg_adaptive_int_enable <= s_axi_wdata[2];
                        cfg_hoj_enable        <= s_axi_wdata[3];
                        cfg_eccm_mode         <= s_axi_wdata[5:4];
                    end
                    REG_JAM_THRESH_HI: cfg_js_threshold_high <= s_axi_wdata[7:0];
                    REG_JAM_THRESH_LO: cfg_js_threshold_low  <= s_axi_wdata[7:0];
                    REG_CFAR_CFG: begin
                        cfg_cfar_threshold <= s_axi_wdata;
                    end
                    default: ;
                endcase
            end
            
            // Response phase
            if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid  <= 1'b0;
                s_axi_awready <= 1'b1;
            end
        end
    end
    
    // Read channel
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axi_arready <= 1'b1;
            s_axi_rvalid  <= 1'b0;
            s_axi_rdata   <= '0;
            s_axi_rresp   <= 2'b00;
            axi_araddr_reg <= '0;
        end else begin
            if (s_axi_arvalid && s_axi_arready) begin
                axi_araddr_reg <= s_axi_araddr;
                s_axi_arready  <= 1'b0;
                s_axi_rvalid   <= 1'b1;
                
                case (s_axi_araddr)
                    REG_CTRL: s_axi_rdata <= {26'b0, cfg_eccm_mode, cfg_hoj_enable, 
                                              cfg_adaptive_int_enable, cfg_ml_cfar_enable, cfg_eccm_enable};
                    REG_STATUS: s_axi_rdata <= {22'b0, int_mode, cfar_jam_type, 
                                                cfar_jam_detected, cfg_eccm_enable};
                    REG_JAM_THRESH_HI: s_axi_rdata <= {24'b0, cfg_js_threshold_high};
                    REG_JAM_THRESH_LO: s_axi_rdata <= {24'b0, cfg_js_threshold_low};
                    REG_JAM_POWER: s_axi_rdata <= cfar_jam_power;
                    REG_JAM_DUTY: s_axi_rdata <= {24'b0, cfar_jam_duty};
                    REG_JAM_POS_X: s_axi_rdata <= loc_jammer_x;
                    REG_JAM_POS_Y: s_axi_rdata <= loc_jammer_y;
                    REG_STATS_DET: s_axi_rdata <= cfar_total_dets;
                    REG_STATS_JAM: s_axi_rdata <= cfar_jam_dets;
                    REG_STATS_HOJ: s_axi_rdata <= hoj_count;
                    REG_VERSION: s_axi_rdata <= 32'h02010000;  // v2.1.0
                    default: s_axi_rdata <= '0;
                endcase
            end
            
            if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid  <= 1'b0;
                s_axi_arready <= 1'b1;
            end
        end
    end

endmodule
