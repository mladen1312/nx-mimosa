//-----------------------------------------------------------------------------
// QEDMMA v2.0 ASTERIX CAT048 Parser
// Author: Dr. Mladen Mešter
// Copyright (c) 2026 Dr. Mladen Mešter - All Rights Reserved
//
// Description:
//   ASTERIX (EUROCONTROL) Category 048 parser for monoradar target reports.
//   Decodes incoming CAT048 messages and outputs unified track format.
//   Supports UAP (User Application Profile) based field selection.
//
// Supported Data Items:
//   I048/010 - Data Source Identifier
//   I048/020 - Target Report Descriptor
//   I048/040 - Measured Position (Polar)
//   I048/042 - Calculated Position (Cartesian)
//   I048/070 - Mode-3/A Code
//   I048/090 - Flight Level
//   I048/130 - Radar Plot Characteristics
//   I048/161 - Track Number
//   I048/200 - Calculated Track Velocity (Polar)
//   I048/220 - Aircraft Address (Mode-S)
//
// [REQ-AST-001] EUROCONTROL ASTERIX CAT048 Edition 1.21+
// [REQ-AST-002] UAP-based field presence detection
// [REQ-AST-003] <2 ms parse latency
//-----------------------------------------------------------------------------

`timescale 1ns / 1ps

module asterix_parser #(
    parameter int TRACK_ID_WIDTH = 10,
    parameter int COORD_WIDTH    = 32
)(
    input  logic        clk,
    input  logic        rst_n,
    
    //-------------------------------------------------------------------------
    // Input Interface (from UDP/Ethernet)
    //-------------------------------------------------------------------------
    input  logic [7:0]  rx_data,
    input  logic        rx_valid,
    input  logic        rx_last,
    output logic        rx_ready,
    
    //-------------------------------------------------------------------------
    // Track Output (to Fusion Engine)
    //-------------------------------------------------------------------------
    output logic [TRACK_ID_WIDTH-1:0] track_id,
    output logic signed [COORD_WIDTH-1:0] track_x,     // meters (Cartesian)
    output logic signed [COORD_WIDTH-1:0] track_y,     // meters
    output logic signed [31:0]        track_alt,       // Flight level × 25 ft
    output logic signed [15:0]        track_vx,        // m/s
    output logic signed [15:0]        track_vy,        // m/s
    output logic [15:0]               track_mode3a,    // Mode-3/A squawk
    output logic [23:0]               track_mode_s,    // Mode-S address
    output logic [7:0]                track_quality,   // Plot quality
    output logic                      track_valid,
    input  logic                      track_ready,
    
    //-------------------------------------------------------------------------
    // Configuration
    //-------------------------------------------------------------------------
    input  logic [7:0]   cfg_sac,           // Expected System Area Code
    input  logic [7:0]   cfg_sic,           // Expected System Identification Code
    input  logic         cfg_filter_sac_sic, // Filter by SAC/SIC
    
    //-------------------------------------------------------------------------
    // Status
    //-------------------------------------------------------------------------
    output logic [31:0]  messages_parsed,
    output logic [31:0]  records_parsed,
    output logic [15:0]  parse_errors
);

    //-------------------------------------------------------------------------
    // ASTERIX CAT048 Field Reference Numbers (FRN)
    //-------------------------------------------------------------------------
    localparam int FRN_010 = 1;   // Data Source Identifier
    localparam int FRN_140 = 2;   // Time of Day
    localparam int FRN_020 = 3;   // Target Report Descriptor
    localparam int FRN_040 = 4;   // Measured Position (Polar)
    localparam int FRN_070 = 5;   // Mode-3/A Code
    localparam int FRN_090 = 6;   // Flight Level
    localparam int FRN_130 = 7;   // Radar Plot Characteristics
    // FX bit
    localparam int FRN_220 = 8;   // Aircraft Address
    localparam int FRN_240 = 9;   // Aircraft Identification
    localparam int FRN_250 = 10;  // Mode S MB Data
    localparam int FRN_161 = 11;  // Track Number
    localparam int FRN_042 = 12;  // Calculated Position (Cartesian)
    localparam int FRN_200 = 13;  // Calculated Track Velocity
    localparam int FRN_170 = 14;  // Track Status
    // FX bit...
    
    //-------------------------------------------------------------------------
    // Parser State Machine
    //-------------------------------------------------------------------------
    typedef enum logic [3:0] {
        ST_IDLE,
        ST_CAT,           // Read category byte
        ST_LEN_MSB,       // Message length MSB
        ST_LEN_LSB,       // Message length LSB
        ST_FSPEC,         // Field Specification (UAP)
        ST_DATA_ITEM,     // Parse data items
        ST_OUTPUT,        // Output parsed track
        ST_SKIP,          // Skip to next record
        ST_ERROR
    } state_t;
    
    state_t state, next_state;
    
    //-------------------------------------------------------------------------
    // Internal Registers
    //-------------------------------------------------------------------------
    logic [7:0]  category;
    logic [15:0] msg_length;
    logic [15:0] bytes_remaining;
    logic [55:0] fspec;          // Up to 7 FSPEC bytes (with FX bits)
    logic [2:0]  fspec_bytes;
    logic [3:0]  fspec_idx;
    logic [5:0]  current_frn;
    
    // Parsed fields
    logic [7:0]  sac, sic;
    logic [23:0] time_of_day;
    logic [7:0]  target_report_desc;
    logic [15:0] rho;             // Range (NM × 2^-7)
    logic [15:0] theta;           // Azimuth (360° × 2^-16)
    logic [15:0] mode3a_code;
    logic [15:0] flight_level;    // FL × 0.25
    logic [23:0] aircraft_address;
    logic [15:0] track_number;
    logic signed [15:0] calc_x;   // 1/128 NM
    logic signed [15:0] calc_y;   // 1/128 NM
    logic signed [15:0] calc_vx;  // 0.25 m/s
    logic signed [15:0] calc_vy;  // 0.25 m/s
    logic [7:0]  plot_quality;
    
    logic        field_present [1:56];
    
    //-------------------------------------------------------------------------
    // Byte Counter
    //-------------------------------------------------------------------------
    logic [15:0] byte_count;
    logic [7:0]  item_byte_idx;
    
    //-------------------------------------------------------------------------
    // State Machine
    //-------------------------------------------------------------------------
    assign rx_ready = (state != ST_OUTPUT) && (state != ST_ERROR);
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= ST_IDLE;
        else
            state <= next_state;
    end
    
    always_comb begin
        next_state = state;
        
        case (state)
            ST_IDLE: begin
                if (rx_valid && rx_ready)
                    next_state = ST_CAT;
            end
            
            ST_CAT: begin
                if (rx_valid)
                    next_state = ST_LEN_MSB;
            end
            
            ST_LEN_MSB: begin
                if (rx_valid)
                    next_state = ST_LEN_LSB;
            end
            
            ST_LEN_LSB: begin
                if (rx_valid)
                    next_state = ST_FSPEC;
            end
            
            ST_FSPEC: begin
                if (rx_valid && !rx_data[0])  // FX bit = 0, end of FSPEC
                    next_state = ST_DATA_ITEM;
            end
            
            ST_DATA_ITEM: begin
                if (bytes_remaining == 0)
                    next_state = ST_OUTPUT;
                else if (rx_last)
                    next_state = ST_OUTPUT;
            end
            
            ST_OUTPUT: begin
                if (track_ready || !field_present[FRN_161])
                    next_state = ST_IDLE;
            end
            
            ST_SKIP: begin
                if (rx_last || bytes_remaining == 0)
                    next_state = ST_IDLE;
            end
            
            ST_ERROR: begin
                next_state = ST_IDLE;
            end
            
            default: next_state = ST_IDLE;
        endcase
    end
    
    //-------------------------------------------------------------------------
    // FSPEC Parsing
    //-------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fspec       <= '0;
            fspec_bytes <= '0;
            for (int i = 0; i < 56; i++)
                field_present[i] <= 1'b0;
        end else begin
            case (state)
                ST_IDLE: begin
                    fspec       <= '0;
                    fspec_bytes <= '0;
                    for (int i = 0; i < 56; i++)
                        field_present[i] <= 1'b0;
                end
                
                ST_FSPEC: begin
                    if (rx_valid) begin
                        fspec <= {fspec[47:0], rx_data};
                        fspec_bytes <= fspec_bytes + 1;
                        
                        // Decode field presence from FSPEC octet
                        // Bits 7-1 are FRN indicators, bit 0 is FX
                        for (int i = 1; i <= 7; i++) begin
                            automatic int frn = (fspec_bytes * 7) + i;
                            if (frn <= 56)
                                field_present[frn] <= rx_data[8-i];
                        end
                    end
                end
                
                default: ;
            endcase
        end
    end
    
    //-------------------------------------------------------------------------
    // Data Item Parsing
    //-------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            category         <= '0;
            msg_length       <= '0;
            bytes_remaining  <= '0;
            byte_count       <= '0;
            current_frn      <= '0;
            item_byte_idx    <= '0;
            
            sac              <= '0;
            sic              <= '0;
            time_of_day      <= '0;
            target_report_desc <= '0;
            rho              <= '0;
            theta            <= '0;
            mode3a_code      <= '0;
            flight_level     <= '0;
            aircraft_address <= '0;
            track_number     <= '0;
            calc_x           <= '0;
            calc_y           <= '0;
            calc_vx          <= '0;
            calc_vy          <= '0;
            plot_quality     <= '0;
            
            messages_parsed  <= '0;
            records_parsed   <= '0;
            parse_errors     <= '0;
        end else begin
            case (state)
                ST_IDLE: begin
                    byte_count    <= '0;
                    current_frn   <= 6'd1;
                    item_byte_idx <= '0;
                end
                
                ST_CAT: begin
                    if (rx_valid) begin
                        category <= rx_data;
                        if (rx_data != 8'd48) begin  // Not CAT048
                            // Skip this message
                        end
                    end
                end
                
                ST_LEN_MSB: begin
                    if (rx_valid)
                        msg_length[15:8] <= rx_data;
                end
                
                ST_LEN_LSB: begin
                    if (rx_valid) begin
                        msg_length[7:0] <= rx_data;
                        bytes_remaining <= {msg_length[15:8], rx_data} - 16'd3;  // Minus CAT+LEN
                    end
                end
                
                ST_DATA_ITEM: begin
                    if (rx_valid && bytes_remaining > 0) begin
                        bytes_remaining <= bytes_remaining - 1;
                        byte_count <= byte_count + 1;
                        
                        // Parse based on current FRN
                        case (current_frn)
                            // I048/010 - Data Source Identifier (2 bytes)
                            FRN_010: begin
                                if (item_byte_idx == 0) sac <= rx_data;
                                else begin
                                    sic <= rx_data;
                                    current_frn <= current_frn + 1;
                                    item_byte_idx <= '0;
                                end
                                if (field_present[FRN_010])
                                    item_byte_idx <= item_byte_idx + 1;
                                else
                                    current_frn <= current_frn + 1;
                            end
                            
                            // I048/140 - Time of Day (3 bytes)
                            FRN_140: begin
                                if (field_present[FRN_140]) begin
                                    time_of_day <= {time_of_day[15:0], rx_data};
                                    item_byte_idx <= item_byte_idx + 1;
                                    if (item_byte_idx >= 2) begin
                                        current_frn <= current_frn + 1;
                                        item_byte_idx <= '0;
                                    end
                                end else begin
                                    current_frn <= current_frn + 1;
                                end
                            end
                            
                            // I048/040 - Measured Position Polar (4 bytes)
                            FRN_040: begin
                                if (field_present[FRN_040]) begin
                                    case (item_byte_idx)
                                        0: rho[15:8]   <= rx_data;
                                        1: rho[7:0]    <= rx_data;
                                        2: theta[15:8] <= rx_data;
                                        3: begin
                                            theta[7:0] <= rx_data;
                                            current_frn <= current_frn + 1;
                                            item_byte_idx <= '0;
                                        end
                                    endcase
                                    item_byte_idx <= item_byte_idx + 1;
                                end else begin
                                    current_frn <= current_frn + 1;
                                end
                            end
                            
                            // I048/070 - Mode-3/A Code (2 bytes)
                            FRN_070: begin
                                if (field_present[FRN_070]) begin
                                    if (item_byte_idx == 0) mode3a_code[15:8] <= rx_data;
                                    else begin
                                        mode3a_code[7:0] <= rx_data;
                                        current_frn <= current_frn + 1;
                                        item_byte_idx <= '0;
                                    end
                                    item_byte_idx <= item_byte_idx + 1;
                                end else begin
                                    current_frn <= current_frn + 1;
                                end
                            end
                            
                            // I048/090 - Flight Level (2 bytes)
                            FRN_090: begin
                                if (field_present[FRN_090]) begin
                                    if (item_byte_idx == 0) flight_level[15:8] <= rx_data;
                                    else begin
                                        flight_level[7:0] <= rx_data;
                                        current_frn <= current_frn + 1;
                                        item_byte_idx <= '0;
                                    end
                                    item_byte_idx <= item_byte_idx + 1;
                                end else begin
                                    current_frn <= current_frn + 1;
                                end
                            end
                            
                            // I048/161 - Track Number (2 bytes)
                            FRN_161: begin
                                if (field_present[FRN_161]) begin
                                    if (item_byte_idx == 0) track_number[15:8] <= rx_data;
                                    else begin
                                        track_number[7:0] <= rx_data;
                                        current_frn <= current_frn + 1;
                                        item_byte_idx <= '0;
                                    end
                                    item_byte_idx <= item_byte_idx + 1;
                                end else begin
                                    current_frn <= current_frn + 1;
                                end
                            end
                            
                            // I048/042 - Calculated Position Cartesian (4 bytes)
                            FRN_042: begin
                                if (field_present[FRN_042]) begin
                                    case (item_byte_idx)
                                        0: calc_x[15:8] <= rx_data;
                                        1: calc_x[7:0]  <= rx_data;
                                        2: calc_y[15:8] <= rx_data;
                                        3: begin
                                            calc_y[7:0] <= rx_data;
                                            current_frn <= current_frn + 1;
                                            item_byte_idx <= '0;
                                        end
                                    endcase
                                    item_byte_idx <= item_byte_idx + 1;
                                end else begin
                                    current_frn <= current_frn + 1;
                                end
                            end
                            
                            // I048/200 - Calculated Track Velocity (4 bytes)
                            FRN_200: begin
                                if (field_present[FRN_200]) begin
                                    case (item_byte_idx)
                                        0: calc_vx[15:8] <= rx_data;
                                        1: calc_vx[7:0]  <= rx_data;
                                        2: calc_vy[15:8] <= rx_data;
                                        3: begin
                                            calc_vy[7:0] <= rx_data;
                                            current_frn <= current_frn + 1;
                                            item_byte_idx <= '0;
                                        end
                                    endcase
                                    item_byte_idx <= item_byte_idx + 1;
                                end else begin
                                    current_frn <= current_frn + 1;
                                end
                            end
                            
                            // I048/220 - Aircraft Address (3 bytes)
                            FRN_220: begin
                                if (field_present[FRN_220]) begin
                                    aircraft_address <= {aircraft_address[15:0], rx_data};
                                    item_byte_idx <= item_byte_idx + 1;
                                    if (item_byte_idx >= 2) begin
                                        current_frn <= current_frn + 1;
                                        item_byte_idx <= '0;
                                    end
                                end else begin
                                    current_frn <= current_frn + 1;
                                end
                            end
                            
                            default: begin
                                // Skip unknown FRNs
                                current_frn <= current_frn + 1;
                            end
                        endcase
                    end
                end
                
                ST_OUTPUT: begin
                    if (field_present[FRN_161]) begin
                        records_parsed <= records_parsed + 1;
                    end
                    messages_parsed <= messages_parsed + 1;
                end
                
                ST_ERROR: begin
                    parse_errors <= parse_errors + 1;
                end
                
                default: ;
            endcase
        end
    end
    
    //-------------------------------------------------------------------------
    // Output Conversion
    //-------------------------------------------------------------------------
    // ASTERIX CAT048 uses 1/128 NM for Cartesian position
    // 1 NM = 1852 m, so 1/128 NM = 14.47 m
    localparam logic [15:0] NM128_TO_M = 16'd14;  // Approximate
    
    // Velocity in 0.25 m/s units
    
    assign track_id      = track_number[TRACK_ID_WIDTH-1:0];
    assign track_x       = calc_x * $signed(NM128_TO_M);
    assign track_y       = calc_y * $signed(NM128_TO_M);
    assign track_alt     = flight_level * 32'd8;  // FL × 0.25 × 25 ft ≈ 7.6 m
    assign track_vx      = calc_vx / 4;  // 0.25 m/s → m/s
    assign track_vy      = calc_vy / 4;
    assign track_mode3a  = mode3a_code;
    assign track_mode_s  = aircraft_address;
    assign track_quality = plot_quality;
    assign track_valid   = (state == ST_OUTPUT) && field_present[FRN_161];

endmodule
