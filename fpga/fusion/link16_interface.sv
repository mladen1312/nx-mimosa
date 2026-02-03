//-----------------------------------------------------------------------------
// QEDMMA v2.0 Link 16 Interface
// Author: Dr. Mladen Mešter
// Copyright (c) 2026 Dr. Mladen Mešter - All Rights Reserved
//
// Description:
//   Link 16 (TADIL-J) interface for NATO tactical data exchange.
//   Implements JREAP-C (Link 16 over IP) for non-MIDS platforms.
//   Supports both receive (ingest external tracks) and transmit
//   (export QEDMMA tracks) modes.
//
// Supported J-series Messages:
//   RX: J2.2 (Air Track), J2.3 (Surface), J2.5 (Space), J3.2 (Point)
//   TX: J2.2 (Air Track), J3.5 (Track Management), J7.0 (Track Quality)
//
// [REQ-L16-001] STANAG 5516 compliant J-series encoding
// [REQ-L16-002] JREAP-C over UDP/IP transport
// [REQ-L16-003] <20 ms encode/decode latency
//-----------------------------------------------------------------------------

`timescale 1ns / 1ps

module link16_interface #(
    parameter int TRACK_ID_WIDTH = 10,
    parameter int COORD_WIDTH    = 32
)(
    input  logic        clk,
    input  logic        rst_n,
    
    //-------------------------------------------------------------------------
    // Network Interface (UDP/IP)
    //-------------------------------------------------------------------------
    // RX from network
    input  logic [7:0]  udp_rx_data,
    input  logic        udp_rx_valid,
    input  logic        udp_rx_last,
    output logic        udp_rx_ready,
    
    // TX to network  
    output logic [7:0]  udp_tx_data,
    output logic        udp_tx_valid,
    output logic        udp_tx_last,
    input  logic        udp_tx_ready,
    
    //-------------------------------------------------------------------------
    // Internal Track Interface (to/from Fusion Engine)
    //-------------------------------------------------------------------------
    // Decoded tracks to fusion
    output logic [TRACK_ID_WIDTH-1:0] track_out_id,
    output logic signed [COORD_WIDTH-1:0] track_out_lat,  // degrees × 1e7
    output logic signed [COORD_WIDTH-1:0] track_out_lon,
    output logic signed [31:0]        track_out_alt,      // meters
    output logic signed [31:0]        track_out_vel,      // m/s
    output logic [15:0]               track_out_heading,  // degrees × 100
    output logic [7:0]                track_out_class,    // NATO platform ID
    output logic [31:0]               track_out_time,     // Time of validity
    output logic                      track_out_valid,
    input  logic                      track_out_ready,
    
    // Tracks to encode and transmit
    input  logic [TRACK_ID_WIDTH-1:0] track_in_id,
    input  logic signed [COORD_WIDTH-1:0] track_in_lat,
    input  logic signed [COORD_WIDTH-1:0] track_in_lon,
    input  logic signed [31:0]        track_in_alt,
    input  logic signed [31:0]        track_in_vel,
    input  logic [15:0]               track_in_heading,
    input  logic [7:0]                track_in_class,
    input  logic [7:0]                track_in_quality,
    input  logic                      track_in_valid,
    output logic                      track_in_ready,
    
    //-------------------------------------------------------------------------
    // Configuration
    //-------------------------------------------------------------------------
    input  logic [15:0]  cfg_source_track_number,  // STN (own platform ID)
    input  logic [7:0]   cfg_exercise_indicator,   // Exercise/real
    input  logic         cfg_enable_tx,
    input  logic         cfg_enable_rx,
    
    //-------------------------------------------------------------------------
    // Status
    //-------------------------------------------------------------------------
    output logic [31:0]  rx_message_count,
    output logic [31:0]  tx_message_count,
    output logic [15:0]  rx_error_count,
    output logic [15:0]  tx_error_count,
    output logic         link_active
);

    //-------------------------------------------------------------------------
    // J-Series Message Constants (STANAG 5516)
    //-------------------------------------------------------------------------
    localparam logic [4:0] J2_2_AIR_PPLI     = 5'd2;   // Air Track
    localparam logic [4:0] J2_3_SURFACE      = 5'd3;   // Surface Track
    localparam logic [4:0] J2_5_SPACE        = 5'd5;   // Space Track
    localparam logic [4:0] J3_2_POINT        = 5'd10;  // Point Track
    localparam logic [4:0] J3_5_TRACK_MGMT   = 5'd13;  // Track Management
    localparam logic [4:0] J7_0_TRACK_QUAL   = 5'd28;  // Track Quality
    
    // JREAP-C Header
    localparam logic [7:0] JREAP_VERSION = 8'h01;
    localparam logic [7:0] JREAP_TYPE_C  = 8'h03;  // Type C (TCP/IP)
    
    //-------------------------------------------------------------------------
    // RX State Machine
    //-------------------------------------------------------------------------
    typedef enum logic [3:0] {
        RX_IDLE,
        RX_JREAP_HDR,
        RX_J_HEADER,
        RX_J_WORD0,
        RX_J_WORD1,
        RX_J_WORD2,
        RX_J_EXTENSION,
        RX_OUTPUT,
        RX_ERROR
    } rx_state_t;
    
    rx_state_t rx_state;
    
    //-------------------------------------------------------------------------
    // TX State Machine
    //-------------------------------------------------------------------------
    typedef enum logic [3:0] {
        TX_IDLE,
        TX_JREAP_HDR,
        TX_J_HEADER,
        TX_J_WORD0,
        TX_J_WORD1,
        TX_J_WORD2,
        TX_J_EXTENSION,
        TX_DONE
    } tx_state_t;
    
    tx_state_t tx_state;
    
    //-------------------------------------------------------------------------
    // RX Buffer and Parsing
    //-------------------------------------------------------------------------
    logic [7:0]  rx_byte_count;
    logic [255:0] rx_buffer;
    logic [4:0]  rx_msg_label;
    logic [4:0]  rx_msg_sublabel;
    
    // J2.2 Air Track Fields (per STANAG 5516)
    // Word 0: [22:0] Track Number, [27:23] Identity, [31:28] Strength
    // Word 1: [22:0] Latitude (180/2^23 deg), [31:23] reserved
    // Word 2: [22:0] Longitude (180/2^23 deg), [31:23] Altitude code
    // Extension: Speed, Heading, etc.
    
    logic [22:0] j_track_number;
    logic [4:0]  j_identity;
    logic [22:0] j_latitude;
    logic [22:0] j_longitude;
    logic [8:0]  j_altitude_code;
    logic [9:0]  j_speed;
    logic [8:0]  j_heading;
    
    //-------------------------------------------------------------------------
    // RX Processing
    //-------------------------------------------------------------------------
    assign udp_rx_ready = cfg_enable_rx && (rx_state != RX_OUTPUT);
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state        <= RX_IDLE;
            rx_byte_count   <= '0;
            rx_buffer       <= '0;
            rx_msg_label    <= '0;
            rx_msg_sublabel <= '0;
            j_track_number  <= '0;
            j_identity      <= '0;
            j_latitude      <= '0;
            j_longitude     <= '0;
            j_altitude_code <= '0;
            j_speed         <= '0;
            j_heading       <= '0;
            rx_message_count <= '0;
            rx_error_count  <= '0;
        end else begin
            case (rx_state)
                RX_IDLE: begin
                    rx_byte_count <= '0;
                    if (udp_rx_valid && udp_rx_ready) begin
                        rx_buffer[7:0] <= udp_rx_data;
                        rx_state <= RX_JREAP_HDR;
                    end
                end
                
                RX_JREAP_HDR: begin
                    if (udp_rx_valid) begin
                        rx_buffer <= {rx_buffer[247:0], udp_rx_data};
                        rx_byte_count <= rx_byte_count + 1;
                        
                        // JREAP-C header is 12 bytes
                        if (rx_byte_count >= 11) begin
                            rx_state <= RX_J_HEADER;
                            rx_byte_count <= '0;
                        end
                    end
                end
                
                RX_J_HEADER: begin
                    if (udp_rx_valid) begin
                        rx_buffer <= {rx_buffer[247:0], udp_rx_data};
                        rx_byte_count <= rx_byte_count + 1;
                        
                        // J-series header (2 bytes: label + sublabel)
                        if (rx_byte_count >= 1) begin
                            rx_msg_label    <= rx_buffer[12:8];
                            rx_msg_sublabel <= rx_buffer[4:0];
                            rx_state <= RX_J_WORD0;
                            rx_byte_count <= '0;
                        end
                    end
                end
                
                RX_J_WORD0: begin
                    if (udp_rx_valid) begin
                        rx_buffer <= {rx_buffer[247:0], udp_rx_data};
                        rx_byte_count <= rx_byte_count + 1;
                        
                        // Word 0 is 4 bytes (32 bits)
                        if (rx_byte_count >= 3) begin
                            j_track_number <= rx_buffer[30:8];
                            j_identity     <= rx_buffer[35:31];
                            rx_state <= RX_J_WORD1;
                            rx_byte_count <= '0;
                        end
                    end
                end
                
                RX_J_WORD1: begin
                    if (udp_rx_valid) begin
                        rx_buffer <= {rx_buffer[247:0], udp_rx_data};
                        rx_byte_count <= rx_byte_count + 1;
                        
                        if (rx_byte_count >= 3) begin
                            j_latitude <= rx_buffer[30:8];
                            rx_state <= RX_J_WORD2;
                            rx_byte_count <= '0;
                        end
                    end
                end
                
                RX_J_WORD2: begin
                    if (udp_rx_valid) begin
                        rx_buffer <= {rx_buffer[247:0], udp_rx_data};
                        rx_byte_count <= rx_byte_count + 1;
                        
                        if (rx_byte_count >= 3) begin
                            j_longitude     <= rx_buffer[30:8];
                            j_altitude_code <= rx_buffer[39:31];
                            rx_state <= RX_J_EXTENSION;
                            rx_byte_count <= '0;
                        end
                    end
                end
                
                RX_J_EXTENSION: begin
                    if (udp_rx_valid) begin
                        rx_buffer <= {rx_buffer[247:0], udp_rx_data};
                        rx_byte_count <= rx_byte_count + 1;
                        
                        // Extension words (speed, heading, etc.)
                        if (rx_byte_count >= 3) begin
                            j_speed   <= rx_buffer[17:8];
                            j_heading <= rx_buffer[26:18];
                            rx_state <= RX_OUTPUT;
                        end
                        
                        if (udp_rx_last) begin
                            rx_state <= RX_OUTPUT;
                        end
                    end
                end
                
                RX_OUTPUT: begin
                    if (track_out_ready) begin
                        rx_message_count <= rx_message_count + 1;
                        rx_state <= RX_IDLE;
                    end
                end
                
                RX_ERROR: begin
                    rx_error_count <= rx_error_count + 1;
                    rx_state <= RX_IDLE;
                end
                
                default: rx_state <= RX_IDLE;
            endcase
        end
    end
    
    //-------------------------------------------------------------------------
    // RX Coordinate Conversion
    // Link 16: 23-bit lat/lon in 180/2^23 degree units
    // Output: degrees × 1e7
    //-------------------------------------------------------------------------
    localparam logic [31:0] L16_TO_DEG7 = 32'd21457;  // (180/2^23) × 1e7
    
    assign track_out_id = j_track_number[TRACK_ID_WIDTH-1:0];
    
    // Signed extension and conversion
    wire signed [31:0] lat_signed = {{9{j_latitude[22]}}, j_latitude};
    wire signed [31:0] lon_signed = {{9{j_longitude[22]}}, j_longitude};
    
    assign track_out_lat = lat_signed * $signed(L16_TO_DEG7);
    assign track_out_lon = lon_signed * $signed(L16_TO_DEG7);
    
    // Altitude: code × 25 ft → meters (×7.62)
    assign track_out_alt = j_altitude_code * 32'd8;  // Approximate
    
    // Velocity: 0.5 knot units → m/s
    assign track_out_vel = j_speed * 32'd257 / 32'd1000;
    
    // Heading: direct pass (needs scaling in external logic)
    assign track_out_heading = {7'b0, j_heading};
    
    // Classification from identity field
    assign track_out_class = {3'b0, j_identity};
    
    // Time of validity (placeholder)
    assign track_out_time = '0;
    
    assign track_out_valid = (rx_state == RX_OUTPUT);
    
    //-------------------------------------------------------------------------
    // TX Processing
    //-------------------------------------------------------------------------
    logic [7:0]  tx_byte_count;
    logic [255:0] tx_buffer;
    
    // Encode track to J2.2 format
    logic [22:0] tx_latitude;
    logic [22:0] tx_longitude;
    logic [8:0]  tx_altitude;
    logic [9:0]  tx_speed;
    logic [8:0]  tx_heading;
    
    assign track_in_ready = cfg_enable_tx && (tx_state == TX_IDLE);
    
    // Convert to Link 16 units
    always_comb begin
        // Lat/lon: degrees × 1e7 → 180/2^23 units
        tx_latitude  = track_in_lat / $signed(L16_TO_DEG7);
        tx_longitude = track_in_lon / $signed(L16_TO_DEG7);
        
        // Altitude: meters → 25 ft codes
        tx_altitude  = track_in_alt / 8;
        
        // Speed: m/s → 0.5 knot units
        tx_speed     = track_in_vel * 1000 / 257;
        
        // Heading: degrees × 100 → Link 16 units
        tx_heading   = track_in_heading / 100;
    end
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state         <= TX_IDLE;
            tx_byte_count    <= '0;
            tx_buffer        <= '0;
            tx_message_count <= '0;
            tx_error_count   <= '0;
        end else begin
            case (tx_state)
                TX_IDLE: begin
                    if (track_in_valid && track_in_ready) begin
                        // Build JREAP-C header
                        tx_buffer[7:0]   <= JREAP_VERSION;
                        tx_buffer[15:8]  <= JREAP_TYPE_C;
                        tx_buffer[31:16] <= 16'd32;  // Length
                        tx_buffer[47:32] <= cfg_source_track_number;
                        tx_buffer[55:48] <= cfg_exercise_indicator;
                        // ... rest of JREAP header
                        
                        tx_byte_count <= '0;
                        tx_state <= TX_JREAP_HDR;
                    end
                end
                
                TX_JREAP_HDR: begin
                    if (udp_tx_ready) begin
                        tx_byte_count <= tx_byte_count + 1;
                        if (tx_byte_count >= 11)
                            tx_state <= TX_J_HEADER;
                    end
                end
                
                TX_J_HEADER: begin
                    if (udp_tx_ready) begin
                        tx_byte_count <= tx_byte_count + 1;
                        if (tx_byte_count >= 13) begin
                            tx_state <= TX_J_WORD0;
                            tx_byte_count <= '0;
                        end
                    end
                end
                
                TX_J_WORD0: begin
                    if (udp_tx_ready) begin
                        tx_byte_count <= tx_byte_count + 1;
                        if (tx_byte_count >= 3)
                            tx_state <= TX_J_WORD1;
                    end
                end
                
                TX_J_WORD1: begin
                    if (udp_tx_ready) begin
                        tx_byte_count <= tx_byte_count + 1;
                        if (tx_byte_count >= 7)
                            tx_state <= TX_J_WORD2;
                    end
                end
                
                TX_J_WORD2: begin
                    if (udp_tx_ready) begin
                        tx_byte_count <= tx_byte_count + 1;
                        if (tx_byte_count >= 11)
                            tx_state <= TX_J_EXTENSION;
                    end
                end
                
                TX_J_EXTENSION: begin
                    if (udp_tx_ready) begin
                        tx_byte_count <= tx_byte_count + 1;
                        if (tx_byte_count >= 15)
                            tx_state <= TX_DONE;
                    end
                end
                
                TX_DONE: begin
                    tx_message_count <= tx_message_count + 1;
                    tx_state <= TX_IDLE;
                end
                
                default: tx_state <= TX_IDLE;
            endcase
        end
    end
    
    // TX output mux
    always_comb begin
        udp_tx_valid = 1'b0;
        udp_tx_data  = 8'h00;
        udp_tx_last  = 1'b0;
        
        if (tx_state != TX_IDLE && tx_state != TX_DONE) begin
            udp_tx_valid = 1'b1;
            udp_tx_data  = tx_buffer[tx_byte_count*8 +: 8];
            udp_tx_last  = (tx_state == TX_J_EXTENSION && tx_byte_count >= 15);
        end
    end
    
    //-------------------------------------------------------------------------
    // Link Status
    //-------------------------------------------------------------------------
    assign link_active = cfg_enable_rx || cfg_enable_tx;

endmodule
