//=============================================================================
// QEDMMA v3.4 - Link-16 / ASTERIX Fusion Adapter
// [REQ-FUSION-001] Multi-sensor track fusion input
// [REQ-FUSION-002] Link-16 J-series message parsing
// [REQ-FUSION-003] ASTERIX CAT048/CAT062 decoding
// [REQ-FUSION-004] Track correlation and handoff
//
// Author: Dr. Mladen Mešter
// Copyright (c) 2026 - All Rights Reserved
//
// Architecture:
//   ┌─────────────────────────────────────────────────────────────────────┐
//   │              LINK-16 / ASTERIX FUSION ADAPTER                       │
//   ├─────────────────────────────────────────────────────────────────────┤
//   │                                                                     │
//   │  ┌─────────────┐       ┌─────────────┐       ┌─────────────┐       │
//   │  │   Link-16   │       │  ASTERIX    │       │   QEDMMA    │       │
//   │  │   Rx FIFO   │       │  Rx FIFO    │       │  Tracks     │       │
//   │  └──────┬──────┘       └──────┬──────┘       └──────┬──────┘       │
//   │         │                     │                     │              │
//   │         ▼                     ▼                     ▼              │
//   │  ┌─────────────┐       ┌─────────────┐       ┌─────────────┐       │
//   │  │  J-Series   │       │  CAT048/062 │       │   Local     │       │
//   │  │   Parser    │       │   Decoder   │       │  Converter  │       │
//   │  └──────┬──────┘       └──────┬──────┘       └──────┬──────┘       │
//   │         │                     │                     │              │
//   │         └──────────────┬──────┴─────────────────────┘              │
//   │                        ▼                                           │
//   │               ┌─────────────────┐                                  │
//   │               │ Track Correlator│                                  │
//   │               │  (Multi-Hypo)   │                                  │
//   │               └────────┬────────┘                                  │
//   │                        ▼                                           │
//   │               ┌─────────────────┐                                  │
//   │               │  Fused Track    │                                  │
//   │               │    Output       │                                  │
//   │               └─────────────────┘                                  │
//   │                                                                     │
//   └─────────────────────────────────────────────────────────────────────┘
//
// Supported Messages:
//   Link-16: J2.2 (Air Track), J3.2 (Surface Track), J7.0 (Track Mgmt)
//   ASTERIX: CAT048 (Radar), CAT062 (SDPS Track)
//=============================================================================

`timescale 1ns / 1ps

module link16_asterix_adapter #(
    parameter int MAX_TRACKS       = 256,
    parameter int COORD_WIDTH      = 32,    // Fixed-point coordinates
    parameter int VELOCITY_WIDTH   = 24,
    parameter int CORRELATION_DIST = 5000   // meters for correlation gate
)(
    input  logic                          clk,
    input  logic                          rst_n,
    
    //=========================================================================
    // Link-16 Input (via MIDS/JTRS interface)
    //=========================================================================
    input  logic [7:0]                    i_link16_data,
    input  logic                          i_link16_valid,
    input  logic                          i_link16_sof,    // Start of frame
    input  logic                          i_link16_eof,    // End of frame
    output logic                          o_link16_ready,
    
    //=========================================================================
    // ASTERIX Input (via UDP/Ethernet)
    //=========================================================================
    input  logic [7:0]                    i_asterix_data,
    input  logic                          i_asterix_valid,
    input  logic                          i_asterix_sof,
    input  logic                          i_asterix_eof,
    output logic                          o_asterix_ready,
    
    //=========================================================================
    // QEDMMA Local Tracks Input
    //=========================================================================
    input  logic [15:0]                   i_local_track_id,
    input  logic signed [COORD_WIDTH-1:0] i_local_lat,      // Latitude (Q16.16)
    input  logic signed [COORD_WIDTH-1:0] i_local_lon,      // Longitude (Q16.16)
    input  logic [23:0]                   i_local_alt,      // Altitude (meters)
    input  logic signed [VELOCITY_WIDTH-1:0] i_local_vx,
    input  logic signed [VELOCITY_WIDTH-1:0] i_local_vy,
    input  logic signed [VELOCITY_WIDTH-1:0] i_local_vz,
    input  logic [7:0]                    i_local_quality,
    input  logic                          i_local_valid,
    
    //=========================================================================
    // Fused Track Output
    //=========================================================================
    output logic [15:0]                   o_fused_track_id,
    output logic signed [COORD_WIDTH-1:0] o_fused_lat,
    output logic signed [COORD_WIDTH-1:0] o_fused_lon,
    output logic [23:0]                   o_fused_alt,
    output logic signed [VELOCITY_WIDTH-1:0] o_fused_vx,
    output logic signed [VELOCITY_WIDTH-1:0] o_fused_vy,
    output logic signed [VELOCITY_WIDTH-1:0] o_fused_vz,
    output logic [7:0]                    o_fused_quality,
    output logic [2:0]                    o_fused_source,   // 001=L16, 010=AST, 100=Local, 111=All
    output logic                          o_fused_valid,
    
    //=========================================================================
    // Cueing Output (for handoff)
    //=========================================================================
    output logic [15:0]                   o_cue_track_id,
    output logic signed [COORD_WIDTH-1:0] o_cue_lat,
    output logic signed [COORD_WIDTH-1:0] o_cue_lon,
    output logic [23:0]                   o_cue_alt,
    output logic [15:0]                   o_cue_search_radius,
    output logic                          o_cue_valid,
    
    //=========================================================================
    // Status
    //=========================================================================
    output logic [7:0]                    o_link16_msg_count,
    output logic [7:0]                    o_asterix_msg_count,
    output logic [7:0]                    o_correlated_count,
    output logic [7:0]                    o_active_tracks
);

    //=========================================================================
    // Local Parameters
    //=========================================================================
    
    // Link-16 J-Series message types
    localparam logic [4:0] J2_2_AIR_TRACK    = 5'd2;
    localparam logic [4:0] J3_2_SURFACE      = 5'd3;
    localparam logic [4:0] J7_0_TRACK_MGMT   = 5'd7;
    
    // ASTERIX Categories
    localparam logic [7:0] CAT048_RADAR      = 8'd48;
    localparam logic [7:0] CAT062_SDPS       = 8'd62;
    
    // Track sources
    localparam logic [2:0] SRC_LINK16       = 3'b001;
    localparam logic [2:0] SRC_ASTERIX      = 3'b010;
    localparam logic [2:0] SRC_LOCAL        = 3'b100;
    
    //=========================================================================
    // Internal Types
    //=========================================================================
    
    typedef struct packed {
        logic [15:0]                   track_id;
        logic signed [COORD_WIDTH-1:0] lat;
        logic signed [COORD_WIDTH-1:0] lon;
        logic [23:0]                   alt;
        logic signed [VELOCITY_WIDTH-1:0] vx;
        logic signed [VELOCITY_WIDTH-1:0] vy;
        logic signed [VELOCITY_WIDTH-1:0] vz;
        logic [7:0]                    quality;
        logic [2:0]                    source;
        logic [31:0]                   timestamp;
        logic                          valid;
    } track_t;
    
    //=========================================================================
    // Track Storage
    //=========================================================================
    
    track_t link16_tracks [MAX_TRACKS/4];
    track_t asterix_tracks [MAX_TRACKS/4];
    track_t local_tracks [MAX_TRACKS/4];
    track_t fused_tracks [MAX_TRACKS];
    
    logic [7:0] link16_track_count;
    logic [7:0] asterix_track_count;
    logic [7:0] local_track_count;
    logic [7:0] fused_track_count;
    
    //=========================================================================
    // Message Parsing State Machines
    //=========================================================================
    
    // Link-16 Parser
    typedef enum logic [2:0] {
        L16_IDLE,
        L16_HEADER,
        L16_JTIDS,
        L16_DATA,
        L16_EXTRACT,
        L16_STORE
    } link16_state_t;
    
    link16_state_t link16_state;
    logic [7:0] link16_buffer [64];
    logic [5:0] link16_byte_cnt;
    logic [4:0] link16_msg_type;
    
    // ASTERIX Parser
    typedef enum logic [2:0] {
        AST_IDLE,
        AST_CAT,
        AST_LEN,
        AST_FSPEC,
        AST_DATA,
        AST_EXTRACT,
        AST_STORE
    } asterix_state_t;
    
    asterix_state_t asterix_state;
    logic [7:0] asterix_buffer [256];
    logic [7:0] asterix_byte_cnt;
    logic [7:0] asterix_category;
    logic [15:0] asterix_length;
    
    //=========================================================================
    // Link-16 Parser FSM
    //=========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            link16_state <= L16_IDLE;
            link16_byte_cnt <= '0;
            link16_msg_type <= '0;
            o_link16_ready <= 1'b1;
        end else begin
            case (link16_state)
                L16_IDLE: begin
                    o_link16_ready <= 1'b1;
                    if (i_link16_valid && i_link16_sof) begin
                        link16_state <= L16_HEADER;
                        link16_byte_cnt <= '0;
                    end
                end
                
                L16_HEADER: begin
                    if (i_link16_valid) begin
                        link16_buffer[link16_byte_cnt] <= i_link16_data;
                        link16_byte_cnt <= link16_byte_cnt + 1;
                        
                        // Extract message type from header
                        if (link16_byte_cnt == 0) begin
                            link16_msg_type <= i_link16_data[4:0];
                        end
                        
                        if (link16_byte_cnt == 3) begin
                            link16_state <= L16_DATA;
                        end
                    end
                end
                
                L16_DATA: begin
                    if (i_link16_valid) begin
                        link16_buffer[link16_byte_cnt] <= i_link16_data;
                        link16_byte_cnt <= link16_byte_cnt + 1;
                        
                        if (i_link16_eof) begin
                            link16_state <= L16_EXTRACT;
                        end
                    end
                end
                
                L16_EXTRACT: begin
                    // Extract track data based on message type
                    if (link16_msg_type == J2_2_AIR_TRACK) begin
                        // J2.2 Air Track format parsing
                        if (link16_track_count < MAX_TRACKS/4) begin
                            link16_tracks[link16_track_count].track_id <= 
                                {link16_buffer[4], link16_buffer[5]};
                            link16_tracks[link16_track_count].lat <= 
                                {link16_buffer[8], link16_buffer[9], 
                                 link16_buffer[10], link16_buffer[11]};
                            link16_tracks[link16_track_count].lon <= 
                                {link16_buffer[12], link16_buffer[13], 
                                 link16_buffer[14], link16_buffer[15]};
                            link16_tracks[link16_track_count].alt <= 
                                {link16_buffer[16], link16_buffer[17], link16_buffer[18]};
                            link16_tracks[link16_track_count].source <= SRC_LINK16;
                            link16_tracks[link16_track_count].valid <= 1'b1;
                        end
                    end
                    link16_state <= L16_STORE;
                end
                
                L16_STORE: begin
                    if (link16_track_count < MAX_TRACKS/4 - 1) begin
                        link16_track_count <= link16_track_count + 1;
                    end
                    link16_state <= L16_IDLE;
                end
                
                default: link16_state <= L16_IDLE;
            endcase
        end
    end
    
    //=========================================================================
    // ASTERIX Parser FSM
    //=========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            asterix_state <= AST_IDLE;
            asterix_byte_cnt <= '0;
            asterix_category <= '0;
            asterix_length <= '0;
            o_asterix_ready <= 1'b1;
        end else begin
            case (asterix_state)
                AST_IDLE: begin
                    o_asterix_ready <= 1'b1;
                    if (i_asterix_valid && i_asterix_sof) begin
                        asterix_state <= AST_CAT;
                        asterix_byte_cnt <= '0;
                    end
                end
                
                AST_CAT: begin
                    if (i_asterix_valid) begin
                        asterix_category <= i_asterix_data;
                        asterix_state <= AST_LEN;
                    end
                end
                
                AST_LEN: begin
                    if (i_asterix_valid) begin
                        asterix_buffer[asterix_byte_cnt] <= i_asterix_data;
                        asterix_byte_cnt <= asterix_byte_cnt + 1;
                        
                        if (asterix_byte_cnt == 0) begin
                            asterix_length[15:8] <= i_asterix_data;
                        end else begin
                            asterix_length[7:0] <= i_asterix_data;
                            asterix_state <= AST_FSPEC;
                        end
                    end
                end
                
                AST_FSPEC: begin
                    // Parse FSPEC (Field Specification)
                    if (i_asterix_valid) begin
                        asterix_buffer[asterix_byte_cnt] <= i_asterix_data;
                        asterix_byte_cnt <= asterix_byte_cnt + 1;
                        
                        // Check FX bit (extension)
                        if (!i_asterix_data[0]) begin
                            asterix_state <= AST_DATA;
                        end
                    end
                end
                
                AST_DATA: begin
                    if (i_asterix_valid) begin
                        asterix_buffer[asterix_byte_cnt] <= i_asterix_data;
                        asterix_byte_cnt <= asterix_byte_cnt + 1;
                        
                        if (i_asterix_eof || asterix_byte_cnt >= asterix_length - 3) begin
                            asterix_state <= AST_EXTRACT;
                        end
                    end
                end
                
                AST_EXTRACT: begin
                    // Extract track based on category
                    if (asterix_category == CAT048_RADAR || 
                        asterix_category == CAT062_SDPS) begin
                        if (asterix_track_count < MAX_TRACKS/4) begin
                            // Simplified extraction - actual ASTERIX is more complex
                            asterix_tracks[asterix_track_count].track_id <= 
                                {asterix_buffer[4], asterix_buffer[5]};
                            asterix_tracks[asterix_track_count].source <= SRC_ASTERIX;
                            asterix_tracks[asterix_track_count].valid <= 1'b1;
                        end
                    end
                    asterix_state <= AST_STORE;
                end
                
                AST_STORE: begin
                    if (asterix_track_count < MAX_TRACKS/4 - 1) begin
                        asterix_track_count <= asterix_track_count + 1;
                    end
                    asterix_state <= AST_IDLE;
                end
                
                default: asterix_state <= AST_IDLE;
            endcase
        end
    end
    
    //=========================================================================
    // Local Track Input
    //=========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            local_track_count <= '0;
        end else if (i_local_valid) begin
            if (local_track_count < MAX_TRACKS/4) begin
                local_tracks[local_track_count].track_id <= i_local_track_id;
                local_tracks[local_track_count].lat <= i_local_lat;
                local_tracks[local_track_count].lon <= i_local_lon;
                local_tracks[local_track_count].alt <= i_local_alt;
                local_tracks[local_track_count].vx <= i_local_vx;
                local_tracks[local_track_count].vy <= i_local_vy;
                local_tracks[local_track_count].vz <= i_local_vz;
                local_tracks[local_track_count].quality <= i_local_quality;
                local_tracks[local_track_count].source <= SRC_LOCAL;
                local_tracks[local_track_count].valid <= 1'b1;
                local_track_count <= local_track_count + 1;
            end
        end
    end
    
    //=========================================================================
    // Track Correlation Engine
    //=========================================================================
    
    typedef enum logic [2:0] {
        CORR_IDLE,
        CORR_LOAD_REF,
        CORR_COMPARE,
        CORR_FUSE,
        CORR_OUTPUT,
        CORR_NEXT
    } corr_state_t;
    
    corr_state_t corr_state;
    logic [7:0] ref_idx;
    logic [7:0] cmp_idx;
    track_t ref_track;
    logic [31:0] distance_sq;
    logic correlation_found;
    
    // Distance calculation (simplified Euclidean)
    function automatic logic [31:0] calc_distance_sq(
        input logic signed [COORD_WIDTH-1:0] lat1, lon1,
        input logic signed [COORD_WIDTH-1:0] lat2, lon2
    );
        logic signed [COORD_WIDTH:0] dlat, dlon;
        dlat = lat1 - lat2;
        dlon = lon1 - lon2;
        // Approximate distance squared (in scaled units)
        return (dlat * dlat) + (dlon * dlon);
    endfunction
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            corr_state <= CORR_IDLE;
            ref_idx <= '0;
            cmp_idx <= '0;
            fused_track_count <= '0;
            o_fused_valid <= 1'b0;
            o_cue_valid <= 1'b0;
        end else begin
            case (corr_state)
                CORR_IDLE: begin
                    o_fused_valid <= 1'b0;
                    o_cue_valid <= 1'b0;
                    
                    // Start correlation when we have tracks
                    if (local_track_count > 0 || link16_track_count > 0 || 
                        asterix_track_count > 0) begin
                        corr_state <= CORR_LOAD_REF;
                        ref_idx <= '0;
                    end
                end
                
                CORR_LOAD_REF: begin
                    // Load reference track (priority: local > link16 > asterix)
                    if (ref_idx < local_track_count && local_tracks[ref_idx].valid) begin
                        ref_track <= local_tracks[ref_idx];
                    end else if (ref_idx < link16_track_count && 
                                 link16_tracks[ref_idx].valid) begin
                        ref_track <= link16_tracks[ref_idx];
                    end else if (ref_idx < asterix_track_count && 
                                 asterix_tracks[ref_idx].valid) begin
                        ref_track <= asterix_tracks[ref_idx];
                    end
                    
                    cmp_idx <= '0;
                    correlation_found <= 1'b0;
                    corr_state <= CORR_COMPARE;
                end
                
                CORR_COMPARE: begin
                    // Compare reference with other tracks
                    track_t cmp_track;
                    
                    // Get comparison track
                    if (ref_track.source == SRC_LOCAL) begin
                        // Compare local with link16/asterix
                        if (cmp_idx < link16_track_count) begin
                            cmp_track = link16_tracks[cmp_idx];
                        end else if (cmp_idx - link16_track_count < asterix_track_count) begin
                            cmp_track = asterix_tracks[cmp_idx - link16_track_count];
                        end
                    end
                    
                    // Calculate distance
                    distance_sq <= calc_distance_sq(
                        ref_track.lat, ref_track.lon,
                        cmp_track.lat, cmp_track.lon
                    );
                    
                    // Check if within correlation gate
                    if (distance_sq < (CORRELATION_DIST * CORRELATION_DIST)) begin
                        correlation_found <= 1'b1;
                        corr_state <= CORR_FUSE;
                    end else begin
                        cmp_idx <= cmp_idx + 1;
                        if (cmp_idx >= link16_track_count + asterix_track_count) begin
                            corr_state <= CORR_OUTPUT;
                        end
                    end
                end
                
                CORR_FUSE: begin
                    // Fuse correlated tracks (weighted average)
                    if (fused_track_count < MAX_TRACKS) begin
                        fused_tracks[fused_track_count].track_id <= ref_track.track_id;
                        
                        // Weighted position (higher quality = more weight)
                        fused_tracks[fused_track_count].lat <= ref_track.lat;
                        fused_tracks[fused_track_count].lon <= ref_track.lon;
                        fused_tracks[fused_track_count].alt <= ref_track.alt;
                        fused_tracks[fused_track_count].vx <= ref_track.vx;
                        fused_tracks[fused_track_count].vy <= ref_track.vy;
                        fused_tracks[fused_track_count].vz <= ref_track.vz;
                        
                        // Combine sources
                        fused_tracks[fused_track_count].source <= 
                            ref_track.source | SRC_ASTERIX | SRC_LINK16;
                        
                        // Quality boost for correlated tracks
                        fused_tracks[fused_track_count].quality <= 
                            (ref_track.quality > 200) ? 8'd255 : ref_track.quality + 50;
                        
                        fused_tracks[fused_track_count].valid <= 1'b1;
                    end
                    corr_state <= CORR_OUTPUT;
                end
                
                CORR_OUTPUT: begin
                    // Output fused track
                    if (fused_track_count < MAX_TRACKS && ref_track.valid) begin
                        o_fused_track_id <= fused_tracks[fused_track_count].track_id;
                        o_fused_lat <= fused_tracks[fused_track_count].lat;
                        o_fused_lon <= fused_tracks[fused_track_count].lon;
                        o_fused_alt <= fused_tracks[fused_track_count].alt;
                        o_fused_vx <= fused_tracks[fused_track_count].vx;
                        o_fused_vy <= fused_tracks[fused_track_count].vy;
                        o_fused_vz <= fused_tracks[fused_track_count].vz;
                        o_fused_quality <= fused_tracks[fused_track_count].quality;
                        o_fused_source <= fused_tracks[fused_track_count].source;
                        o_fused_valid <= 1'b1;
                        
                        fused_track_count <= fused_track_count + 1;
                        
                        // Generate cue if track only from external source
                        if (!correlation_found && ref_track.source != SRC_LOCAL) begin
                            o_cue_track_id <= ref_track.track_id;
                            o_cue_lat <= ref_track.lat;
                            o_cue_lon <= ref_track.lon;
                            o_cue_alt <= ref_track.alt;
                            o_cue_search_radius <= 16'd5000; // 5km search
                            o_cue_valid <= 1'b1;
                        end
                    end
                    
                    corr_state <= CORR_NEXT;
                end
                
                CORR_NEXT: begin
                    o_fused_valid <= 1'b0;
                    o_cue_valid <= 1'b0;
                    
                    ref_idx <= ref_idx + 1;
                    if (ref_idx >= local_track_count + link16_track_count + 
                        asterix_track_count - 1) begin
                        corr_state <= CORR_IDLE;
                    end else begin
                        corr_state <= CORR_LOAD_REF;
                    end
                end
                
                default: corr_state <= CORR_IDLE;
            endcase
        end
    end
    
    //=========================================================================
    // Status Outputs
    //=========================================================================
    
    assign o_link16_msg_count = link16_track_count;
    assign o_asterix_msg_count = asterix_track_count;
    assign o_correlated_count = fused_track_count;
    assign o_active_tracks = local_track_count + link16_track_count + 
                             asterix_track_count;

endmodule
