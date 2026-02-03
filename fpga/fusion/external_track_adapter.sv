//-----------------------------------------------------------------------------
// QEDMMA v2.0 External Track Adapter
// Author: Dr. Mladen Mešter
// Copyright (c) 2026 Dr. Mladen Mešter - All Rights Reserved
//
// Description:
//   Universal adapter for ingesting external sensor tracks into QEDMMA
//   fusion engine. Supports multiple input formats:
//   - Link 16 J-series (J2.2 Air Track)
//   - ASTERIX CAT048 (Monoradar target)
//   - IRST (Angle-only)
//   - ESM (AOA + frequency)
//   - ADS-B Mode-S
//
// [REQ-FUSION-001] Support 5+ external sensor types
// [REQ-FUSION-002] Latency <2 ms per track conversion
// [REQ-FUSION-003] Unified internal track format output
//-----------------------------------------------------------------------------

`timescale 1ns / 1ps

module external_track_adapter #(
    parameter int MAX_TRACKS      = 1024,
    parameter int TRACK_ID_WIDTH  = 10,
    parameter int COORD_WIDTH     = 32,  // Fixed-point Q16.16
    parameter int COV_WIDTH       = 32
)(
    input  logic        clk,
    input  logic        rst_n,
    
    //-------------------------------------------------------------------------
    // External Input Interface (AXI-Stream)
    //-------------------------------------------------------------------------
    input  logic [255:0] s_axis_tdata,    // Raw external data
    input  logic         s_axis_tvalid,
    output logic         s_axis_tready,
    input  logic         s_axis_tlast,
    input  logic [3:0]   s_axis_tid,      // Source ID (see below)
    
    //-------------------------------------------------------------------------
    // Unified Track Output (to Fusion Engine)
    //-------------------------------------------------------------------------
    output logic [TRACK_ID_WIDTH-1:0] track_id,
    output logic [3:0]                track_source,  // Bitmap of contributing sensors
    output logic [7:0]                track_class,   // NATO classification
    
    // Position (ENU, meters, Q16.16 fixed-point)
    output logic signed [COORD_WIDTH-1:0] pos_east,
    output logic signed [COORD_WIDTH-1:0] pos_north,
    output logic signed [COORD_WIDTH-1:0] pos_up,
    
    // Velocity (m/s, Q16.16)
    output logic signed [COORD_WIDTH-1:0] vel_east,
    output logic signed [COORD_WIDTH-1:0] vel_north,
    output logic signed [COORD_WIDTH-1:0] vel_up,
    
    // Covariance diagonal (simplified)
    output logic [COV_WIDTH-1:0] cov_pos,   // Position variance (m²)
    output logic [COV_WIDTH-1:0] cov_vel,   // Velocity variance (m²/s²)
    
    // Metadata
    output logic [63:0]  timestamp,        // GPS microseconds
    output logic [7:0]   quality,          // 0-255 confidence
    output logic         track_valid,      // Output valid strobe
    
    // Sensor-specific fields
    output logic signed [15:0] rcs_dbsm,   // RCS estimate (dBsm × 100)
    output logic [31:0]  emitter_freq,     // ESM frequency (Hz)
    output logic [15:0]  ir_intensity,     // IRST intensity
    
    //-------------------------------------------------------------------------
    // Configuration
    //-------------------------------------------------------------------------
    input  logic signed [COORD_WIDTH-1:0] origin_lat,  // WGS84 origin (deg × 1e7)
    input  logic signed [COORD_WIDTH-1:0] origin_lon,
    input  logic signed [COORD_WIDTH-1:0] origin_alt,  // meters
    
    //-------------------------------------------------------------------------
    // Status
    //-------------------------------------------------------------------------
    output logic [31:0]  tracks_received,
    output logic [31:0]  tracks_converted,
    output logic [7:0]   error_count
);

    //-------------------------------------------------------------------------
    // Source ID Encoding
    //-------------------------------------------------------------------------
    localparam logic [3:0] SRC_QEDMMA  = 4'd0;   // Internal QEDMMA tracks
    localparam logic [3:0] SRC_LINK16  = 4'd1;   // Link 16 J-series
    localparam logic [3:0] SRC_ASTERIX = 4'd2;   // ASTERIX CAT048
    localparam logic [3:0] SRC_IRST    = 4'd3;   // IRST angle-only
    localparam logic [3:0] SRC_ESM     = 4'd4;   // ESM/ELINT
    localparam logic [3:0] SRC_ADSB    = 4'd5;   // ADS-B Mode-S
    localparam logic [3:0] SRC_RADAR   = 4'd6;   // Generic radar
    localparam logic [3:0] SRC_CUSTOM  = 4'd7;   // Custom format
    
    //-------------------------------------------------------------------------
    // State Machine
    //-------------------------------------------------------------------------
    typedef enum logic [2:0] {
        ST_IDLE,
        ST_PARSE,
        ST_CONVERT,
        ST_OUTPUT,
        ST_ERROR
    } state_t;
    
    state_t state, next_state;
    
    //-------------------------------------------------------------------------
    // Internal Signals
    //-------------------------------------------------------------------------
    logic [255:0] input_buffer;
    logic [3:0]   current_source;
    
    // Parsed fields (source-agnostic)
    logic signed [63:0] raw_lat;      // Latitude (various formats)
    logic signed [63:0] raw_lon;      // Longitude
    logic signed [31:0] raw_alt;      // Altitude
    logic signed [31:0] raw_vx;       // Velocity X
    logic signed [31:0] raw_vy;       // Velocity Y
    logic signed [31:0] raw_vz;       // Velocity Z
    logic [31:0]        raw_time;     // Timestamp
    logic [15:0]        raw_track_id; // Source track ID
    
    // Conversion pipeline registers
    logic signed [63:0] enu_east_full;
    logic signed [63:0] enu_north_full;
    logic signed [63:0] enu_up_full;
    
    //-------------------------------------------------------------------------
    // Input Buffering
    //-------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            input_buffer   <= '0;
            current_source <= '0;
            tracks_received <= '0;
        end else if (s_axis_tvalid && s_axis_tready) begin
            input_buffer   <= s_axis_tdata;
            current_source <= s_axis_tid;
            tracks_received <= tracks_received + 1;
        end
    end
    
    assign s_axis_tready = (state == ST_IDLE);
    
    //-------------------------------------------------------------------------
    // State Machine
    //-------------------------------------------------------------------------
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
                if (s_axis_tvalid && s_axis_tready)
                    next_state = ST_PARSE;
            end
            
            ST_PARSE: begin
                next_state = ST_CONVERT;
            end
            
            ST_CONVERT: begin
                next_state = ST_OUTPUT;
            end
            
            ST_OUTPUT: begin
                next_state = ST_IDLE;
            end
            
            ST_ERROR: begin
                next_state = ST_IDLE;
            end
            
            default: next_state = ST_IDLE;
        endcase
    end
    
    //-------------------------------------------------------------------------
    // Format-Specific Parsing
    //-------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            raw_lat      <= '0;
            raw_lon      <= '0;
            raw_alt      <= '0;
            raw_vx       <= '0;
            raw_vy       <= '0;
            raw_vz       <= '0;
            raw_time     <= '0;
            raw_track_id <= '0;
        end else if (state == ST_PARSE) begin
            case (current_source)
                //-------------------------------------------------------------
                // Link 16 J2.2 Air Track Format (simplified)
                // Bits: [15:0]=Track ID, [47:16]=Lat, [79:48]=Lon, [111:80]=Alt
                //       [143:112]=Vel, [175:144]=Heading, [191:176]=Time
                //-------------------------------------------------------------
                SRC_LINK16: begin
                    raw_track_id <= input_buffer[15:0];
                    // Link 16 uses 23-bit lat/lon in 180/2^23 deg units
                    raw_lat      <= $signed(input_buffer[47:16]) * 64'd21457;  // Convert to 1e-7 deg
                    raw_lon      <= $signed(input_buffer[79:48]) * 64'd21457;
                    raw_alt      <= $signed(input_buffer[111:80]) * 32'd3;     // 25 ft units to meters
                    // Velocity in 0.5 knot units
                    raw_vx       <= $signed(input_buffer[143:128]) * 32'd257 / 32'd1000;  // to m/s
                    raw_vy       <= '0;  // Need heading decomposition
                    raw_vz       <= $signed(input_buffer[159:144]) * 32'd6 / 32'd1000;    // ft/min to m/s
                    raw_time     <= input_buffer[191:176];
                end
                
                //-------------------------------------------------------------
                // ASTERIX CAT048 Format (simplified)
                // Uses various field specifications per EUROCONTROL
                //-------------------------------------------------------------
                SRC_ASTERIX: begin
                    raw_track_id <= input_buffer[15:0];
                    // CAT048 I048/130 - Calculated Position (WGS84)
                    raw_lat      <= $signed(input_buffer[55:24]);   // Already in 180/2^25 deg
                    raw_lon      <= $signed(input_buffer[87:56]);
                    // Altitude in 25 ft units
                    raw_alt      <= $signed(input_buffer[103:88]) * 32'd8;  // 25 ft to meters approx
                    // Ground speed and heading
                    raw_vx       <= $signed(input_buffer[119:104]) / 32'd4;  // NM/s to m/s approx
                    raw_vy       <= '0;
                    raw_vz       <= $signed(input_buffer[135:120]) * 32'd6 / 32'd1000;
                    raw_time     <= input_buffer[167:136];
                end
                
                //-------------------------------------------------------------
                // IRST Angle-Only Format
                // [15:0]=ID, [31:16]=Azimuth(deg×100), [47:32]=Elevation(deg×100)
                // [63:48]=Intensity, [95:64]=Time
                //-------------------------------------------------------------
                SRC_IRST: begin
                    raw_track_id <= input_buffer[15:0];
                    // Angle-only: no position, use sensor location + bearing
                    raw_lat      <= '0;  // Computed from bearing
                    raw_lon      <= '0;
                    raw_alt      <= '0;
                    raw_vx       <= '0;
                    raw_vy       <= '0;
                    raw_vz       <= '0;
                    raw_time     <= input_buffer[95:64];
                end
                
                //-------------------------------------------------------------
                // ESM/ELINT Format
                // [15:0]=ID, [31:16]=AOA(deg×100), [63:32]=Frequency(Hz)
                // [79:64]=PRI, [111:80]=Time
                //-------------------------------------------------------------
                SRC_ESM: begin
                    raw_track_id <= input_buffer[15:0];
                    // Angle-only like IRST
                    raw_lat      <= '0;
                    raw_lon      <= '0;
                    raw_alt      <= '0;
                    raw_vx       <= '0;
                    raw_vy       <= '0;
                    raw_vz       <= '0;
                    raw_time     <= input_buffer[111:80];
                end
                
                //-------------------------------------------------------------
                // ADS-B Mode-S Extended Squitter
                // [23:0]=ICAO, [55:24]=Lat(CPR), [87:56]=Lon(CPR)
                // [103:88]=Alt(ft), [119:104]=Velocity, [135:120]=Heading
                //-------------------------------------------------------------
                SRC_ADSB: begin
                    raw_track_id <= input_buffer[15:0];  // Use lower bits of ICAO
                    // CPR decoding would be done externally
                    raw_lat      <= $signed(input_buffer[55:24]);
                    raw_lon      <= $signed(input_buffer[87:56]);
                    raw_alt      <= $signed(input_buffer[103:88]) * 32'd30 / 32'd100;  // ft to m
                    raw_vx       <= $signed(input_buffer[119:104]) * 32'd514 / 32'd1000;  // knots to m/s
                    raw_vy       <= '0;
                    raw_vz       <= $signed(input_buffer[135:120]) * 32'd6 / 32'd1000;
                    raw_time     <= input_buffer[167:136];
                end
                
                //-------------------------------------------------------------
                // Generic/Custom Format
                // Direct pass-through with assumed internal format
                //-------------------------------------------------------------
                default: begin
                    raw_track_id <= input_buffer[15:0];
                    raw_lat      <= $signed(input_buffer[79:16]);
                    raw_lon      <= $signed(input_buffer[143:80]);
                    raw_alt      <= $signed(input_buffer[175:144]);
                    raw_vx       <= $signed(input_buffer[207:176]);
                    raw_vy       <= $signed(input_buffer[239:208]);
                    raw_vz       <= '0;
                    raw_time     <= input_buffer[255:224];
                end
            endcase
        end
    end
    
    //-------------------------------------------------------------------------
    // Coordinate Conversion (WGS84 → ENU)
    // Simplified: assumes small area, uses linear approximation
    // Full implementation would use proper geodetic transforms
    //-------------------------------------------------------------------------
    localparam int METERS_PER_DEG_LAT = 111320;  // At equator, approximate
    localparam int METERS_PER_DEG_LON = 111320;  // Varies with latitude
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            enu_east_full  <= '0;
            enu_north_full <= '0;
            enu_up_full    <= '0;
        end else if (state == ST_CONVERT) begin
            // Convert lat/lon difference to ENU (simplified linear)
            // Note: raw_lat/lon in 1e-7 degrees, output in Q16.16 meters
            enu_north_full <= ((raw_lat - origin_lat) * METERS_PER_DEG_LAT) / 10_000_000;
            enu_east_full  <= ((raw_lon - origin_lon) * METERS_PER_DEG_LON) / 10_000_000;
            enu_up_full    <= (raw_alt - origin_alt);
        end
    end
    
    //-------------------------------------------------------------------------
    // Output Generation
    //-------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            track_id       <= '0;
            track_source   <= '0;
            track_class    <= '0;
            pos_east       <= '0;
            pos_north      <= '0;
            pos_up         <= '0;
            vel_east       <= '0;
            vel_north      <= '0;
            vel_up         <= '0;
            cov_pos        <= '0;
            cov_vel        <= '0;
            timestamp      <= '0;
            quality        <= '0;
            track_valid    <= 1'b0;
            rcs_dbsm       <= '0;
            emitter_freq   <= '0;
            ir_intensity   <= '0;
            tracks_converted <= '0;
        end else if (state == ST_OUTPUT) begin
            track_id       <= raw_track_id[TRACK_ID_WIDTH-1:0];
            track_source   <= (4'b0001 << current_source);  // Bitmap
            track_class    <= 8'h00;  // Unknown until classified
            
            // Position (truncate to Q16.16)
            pos_east       <= enu_east_full[COORD_WIDTH-1:0];
            pos_north      <= enu_north_full[COORD_WIDTH-1:0];
            pos_up         <= enu_up_full[COORD_WIDTH-1:0];
            
            // Velocity
            vel_east       <= raw_vx;
            vel_north      <= raw_vy;
            vel_up         <= raw_vz;
            
            // Covariance (source-dependent defaults)
            case (current_source)
                SRC_LINK16:  begin cov_pos <= 32'd10000;  cov_vel <= 32'd100; end   // 100m, 10m/s
                SRC_ASTERIX: begin cov_pos <= 32'd2500;   cov_vel <= 32'd25;  end   // 50m, 5m/s
                SRC_IRST:    begin cov_pos <= 32'd1000000; cov_vel <= 32'd10000; end // Angle-only
                SRC_ESM:     begin cov_pos <= 32'd1000000; cov_vel <= 32'd10000; end
                SRC_ADSB:    begin cov_pos <= 32'd400;    cov_vel <= 32'd4;   end   // 20m, 2m/s
                default:     begin cov_pos <= 32'd10000;  cov_vel <= 32'd100; end
            endcase
            
            timestamp      <= {32'b0, raw_time};
            quality        <= 8'd128;  // Default 50%
            track_valid    <= 1'b1;
            
            // Sensor-specific
            rcs_dbsm       <= '0;  // Not available from external
            emitter_freq   <= (current_source == SRC_ESM) ? input_buffer[63:32] : '0;
            ir_intensity   <= (current_source == SRC_IRST) ? input_buffer[63:48] : '0;
            
            tracks_converted <= tracks_converted + 1;
        end else begin
            track_valid <= 1'b0;
        end
    end
    
    //-------------------------------------------------------------------------
    // Error Handling
    //-------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            error_count <= '0;
        else if (state == ST_ERROR)
            error_count <= error_count + 1;
    end

endmodule
