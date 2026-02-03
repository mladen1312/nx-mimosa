//-----------------------------------------------------------------------------
// QEDMMA v2.0 Track Fusion Engine
// Author: Dr. Mladen Mešter
// Copyright (c) 2026 Dr. Mladen Mešter - All Rights Reserved
//
// Description:
//   Multi-sensor track fusion engine implementing:
//   - Track-to-track association (Global Nearest Neighbor)
//   - State fusion (Covariance Intersection)
//   - Track management (initiation, maintenance, deletion)
//   - Quality estimation
//
// [REQ-FUSION-004] Fuse up to 1024 simultaneous tracks
// [REQ-FUSION-005] Association latency <10 ms
// [REQ-FUSION-006] Support 8 sensor sources
//-----------------------------------------------------------------------------

`timescale 1ns / 1ps

module track_fusion_engine #(
    parameter int MAX_TRACKS      = 1024,
    parameter int TRACK_ID_WIDTH  = 10,
    parameter int COORD_WIDTH     = 32,
    parameter int COV_WIDTH       = 32,
    parameter int MAX_SOURCES     = 8,
    parameter int ASSOC_THRESHOLD = 32'd1000  // Mahalanobis distance squared threshold
)(
    input  logic        clk,
    input  logic        rst_n,
    
    //-------------------------------------------------------------------------
    // Track Input (from adapters)
    //-------------------------------------------------------------------------
    input  logic [TRACK_ID_WIDTH-1:0] in_track_id,
    input  logic [3:0]                in_track_source,
    input  logic [7:0]                in_track_class,
    input  logic signed [COORD_WIDTH-1:0] in_pos_east,
    input  logic signed [COORD_WIDTH-1:0] in_pos_north,
    input  logic signed [COORD_WIDTH-1:0] in_pos_up,
    input  logic signed [COORD_WIDTH-1:0] in_vel_east,
    input  logic signed [COORD_WIDTH-1:0] in_vel_north,
    input  logic signed [COORD_WIDTH-1:0] in_vel_up,
    input  logic [COV_WIDTH-1:0]      in_cov_pos,
    input  logic [COV_WIDTH-1:0]      in_cov_vel,
    input  logic [63:0]               in_timestamp,
    input  logic [7:0]                in_quality,
    input  logic                      in_valid,
    output logic                      in_ready,
    
    //-------------------------------------------------------------------------
    // Fused Track Output (to C2 interfaces)
    //-------------------------------------------------------------------------
    output logic [TRACK_ID_WIDTH-1:0] out_track_id,
    output logic [MAX_SOURCES-1:0]    out_source_bitmap,
    output logic [7:0]                out_track_class,
    output logic signed [COORD_WIDTH-1:0] out_pos_east,
    output logic signed [COORD_WIDTH-1:0] out_pos_north,
    output logic signed [COORD_WIDTH-1:0] out_pos_up,
    output logic signed [COORD_WIDTH-1:0] out_vel_east,
    output logic signed [COORD_WIDTH-1:0] out_vel_north,
    output logic signed [COORD_WIDTH-1:0] out_vel_up,
    output logic [COV_WIDTH-1:0]      out_cov_pos,
    output logic [COV_WIDTH-1:0]      out_cov_vel,
    output logic [63:0]               out_timestamp,
    output logic [7:0]                out_quality,
    output logic                      out_valid,
    input  logic                      out_ready,
    
    //-------------------------------------------------------------------------
    // Track Database Query Interface
    //-------------------------------------------------------------------------
    output logic [TRACK_ID_WIDTH-1:0] db_query_id,
    output logic                      db_query_valid,
    input  logic                      db_query_ready,
    input  logic                      db_query_found,
    input  logic [511:0]              db_query_data,
    
    //-------------------------------------------------------------------------
    // Track Database Update Interface
    //-------------------------------------------------------------------------
    output logic [TRACK_ID_WIDTH-1:0] db_update_id,
    output logic [511:0]              db_update_data,
    output logic                      db_update_valid,
    output logic                      db_update_create,  // 1=new track, 0=update
    input  logic                      db_update_ready,
    
    //-------------------------------------------------------------------------
    // Configuration
    //-------------------------------------------------------------------------
    input  logic [31:0]  cfg_assoc_threshold,
    input  logic [31:0]  cfg_track_timeout_ms,
    input  logic [7:0]   cfg_min_quality,
    
    //-------------------------------------------------------------------------
    // Status
    //-------------------------------------------------------------------------
    output logic [TRACK_ID_WIDTH-1:0] active_tracks,
    output logic [31:0]  fusions_performed,
    output logic [31:0]  new_tracks_created,
    output logic [31:0]  tracks_deleted
);

    //-------------------------------------------------------------------------
    // Track Database Entry Format (512 bits)
    //-------------------------------------------------------------------------
    // [9:0]    = Fused Track ID
    // [17:10]  = Source bitmap
    // [25:18]  = Classification
    // [57:26]  = Position East (Q16.16)
    // [89:58]  = Position North
    // [121:90] = Position Up
    // [153:122] = Velocity East
    // [185:154] = Velocity North
    // [217:186] = Velocity Up
    // [249:218] = Covariance Position
    // [281:250] = Covariance Velocity
    // [345:282] = Last Update Timestamp
    // [353:346] = Quality
    // [361:354] = Update Count
    // [393:362] = Age (ms since creation)
    // [511:394] = Reserved
    
    //-------------------------------------------------------------------------
    // State Machine
    //-------------------------------------------------------------------------
    typedef enum logic [3:0] {
        ST_IDLE,
        ST_RECEIVE,
        ST_SEARCH,
        ST_WAIT_SEARCH,
        ST_ASSOCIATE,
        ST_FUSE,
        ST_CREATE,
        ST_UPDATE_DB,
        ST_OUTPUT,
        ST_CLEANUP
    } state_t;
    
    state_t state, next_state;
    
    //-------------------------------------------------------------------------
    // Input Registers
    //-------------------------------------------------------------------------
    logic [TRACK_ID_WIDTH-1:0] reg_in_track_id;
    logic [3:0]                reg_in_source;
    logic [7:0]                reg_in_class;
    logic signed [COORD_WIDTH-1:0] reg_in_pos_e, reg_in_pos_n, reg_in_pos_u;
    logic signed [COORD_WIDTH-1:0] reg_in_vel_e, reg_in_vel_n, reg_in_vel_u;
    logic [COV_WIDTH-1:0]      reg_in_cov_pos, reg_in_cov_vel;
    logic [63:0]               reg_in_timestamp;
    logic [7:0]                reg_in_quality;
    
    //-------------------------------------------------------------------------
    // Association Variables
    //-------------------------------------------------------------------------
    logic [TRACK_ID_WIDTH-1:0] search_idx;
    logic [TRACK_ID_WIDTH-1:0] best_match_id;
    logic [31:0]               best_match_dist;
    logic                      match_found;
    
    // Mahalanobis distance calculation
    logic signed [63:0] delta_pos_e, delta_pos_n, delta_pos_u;
    logic signed [63:0] delta_vel_e, delta_vel_n, delta_vel_u;
    logic [63:0]        distance_squared;
    
    //-------------------------------------------------------------------------
    // Fused Track Registers
    //-------------------------------------------------------------------------
    logic [TRACK_ID_WIDTH-1:0] fused_id;
    logic [MAX_SOURCES-1:0]    fused_sources;
    logic [7:0]                fused_class;
    logic signed [COORD_WIDTH-1:0] fused_pos_e, fused_pos_n, fused_pos_u;
    logic signed [COORD_WIDTH-1:0] fused_vel_e, fused_vel_n, fused_vel_u;
    logic [COV_WIDTH-1:0]      fused_cov_pos, fused_cov_vel;
    logic [63:0]               fused_timestamp;
    logic [7:0]                fused_quality;
    logic [7:0]                fused_update_count;
    
    //-------------------------------------------------------------------------
    // Database Track Parsing
    //-------------------------------------------------------------------------
    logic [TRACK_ID_WIDTH-1:0] db_track_id;
    logic [MAX_SOURCES-1:0]    db_sources;
    logic signed [COORD_WIDTH-1:0] db_pos_e, db_pos_n, db_pos_u;
    logic signed [COORD_WIDTH-1:0] db_vel_e, db_vel_n, db_vel_u;
    logic [COV_WIDTH-1:0]      db_cov_pos, db_cov_vel;
    logic [63:0]               db_timestamp;
    logic [7:0]                db_quality;
    logic [7:0]                db_update_count;
    
    // Parse database response
    always_comb begin
        db_track_id    = db_query_data[9:0];
        db_sources     = db_query_data[17:10];
        db_pos_e       = $signed(db_query_data[57:26]);
        db_pos_n       = $signed(db_query_data[89:58]);
        db_pos_u       = $signed(db_query_data[121:90]);
        db_vel_e       = $signed(db_query_data[153:122]);
        db_vel_n       = $signed(db_query_data[185:154]);
        db_vel_u       = $signed(db_query_data[217:186]);
        db_cov_pos     = db_query_data[249:218];
        db_cov_vel     = db_query_data[281:250];
        db_timestamp   = db_query_data[345:282];
        db_quality     = db_query_data[353:346];
        db_update_count = db_query_data[361:354];
    end
    
    //-------------------------------------------------------------------------
    // Track ID Counter (for new tracks)
    //-------------------------------------------------------------------------
    logic [TRACK_ID_WIDTH-1:0] next_track_id;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            next_track_id <= '0;
        else if (state == ST_CREATE && db_update_ready)
            next_track_id <= next_track_id + 1;
    end
    
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
                if (in_valid && in_ready)
                    next_state = ST_RECEIVE;
            end
            
            ST_RECEIVE: begin
                next_state = ST_SEARCH;
            end
            
            ST_SEARCH: begin
                if (db_query_valid && db_query_ready)
                    next_state = ST_WAIT_SEARCH;
            end
            
            ST_WAIT_SEARCH: begin
                if (db_query_found)
                    next_state = ST_ASSOCIATE;
                else if (search_idx >= MAX_TRACKS - 1)
                    next_state = match_found ? ST_FUSE : ST_CREATE;
                else
                    next_state = ST_SEARCH;
            end
            
            ST_ASSOCIATE: begin
                // Calculate distance and check threshold
                next_state = ST_SEARCH;  // Continue searching for best match
            end
            
            ST_FUSE: begin
                next_state = ST_UPDATE_DB;
            end
            
            ST_CREATE: begin
                if (db_update_ready)
                    next_state = ST_UPDATE_DB;
            end
            
            ST_UPDATE_DB: begin
                if (db_update_ready)
                    next_state = ST_OUTPUT;
            end
            
            ST_OUTPUT: begin
                if (out_ready)
                    next_state = ST_IDLE;
            end
            
            ST_CLEANUP: begin
                next_state = ST_IDLE;
            end
            
            default: next_state = ST_IDLE;
        endcase
    end
    
    //-------------------------------------------------------------------------
    // Input Registration
    //-------------------------------------------------------------------------
    assign in_ready = (state == ST_IDLE);
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reg_in_track_id  <= '0;
            reg_in_source    <= '0;
            reg_in_class     <= '0;
            reg_in_pos_e     <= '0;
            reg_in_pos_n     <= '0;
            reg_in_pos_u     <= '0;
            reg_in_vel_e     <= '0;
            reg_in_vel_n     <= '0;
            reg_in_vel_u     <= '0;
            reg_in_cov_pos   <= '0;
            reg_in_cov_vel   <= '0;
            reg_in_timestamp <= '0;
            reg_in_quality   <= '0;
        end else if (state == ST_IDLE && in_valid) begin
            reg_in_track_id  <= in_track_id;
            reg_in_source    <= in_track_source;
            reg_in_class     <= in_track_class;
            reg_in_pos_e     <= in_pos_east;
            reg_in_pos_n     <= in_pos_north;
            reg_in_pos_u     <= in_pos_up;
            reg_in_vel_e     <= in_vel_east;
            reg_in_vel_n     <= in_vel_north;
            reg_in_vel_u     <= in_vel_up;
            reg_in_cov_pos   <= in_cov_pos;
            reg_in_cov_vel   <= in_cov_vel;
            reg_in_timestamp <= in_timestamp;
            reg_in_quality   <= in_quality;
        end
    end
    
    //-------------------------------------------------------------------------
    // Search Index Management
    //-------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            search_idx <= '0;
        end else begin
            case (state)
                ST_RECEIVE: search_idx <= '0;
                ST_WAIT_SEARCH: begin
                    if (!db_query_found || (db_query_found && state == ST_ASSOCIATE))
                        search_idx <= search_idx + 1;
                end
                default: ;
            endcase
        end
    end
    
    //-------------------------------------------------------------------------
    // Database Query
    //-------------------------------------------------------------------------
    assign db_query_id    = search_idx;
    assign db_query_valid = (state == ST_SEARCH);
    
    //-------------------------------------------------------------------------
    // Mahalanobis Distance Calculation
    //-------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (state == ST_ASSOCIATE && db_query_found) begin
            // Position deltas
            delta_pos_e <= reg_in_pos_e - db_pos_e;
            delta_pos_n <= reg_in_pos_n - db_pos_n;
            delta_pos_u <= reg_in_pos_u - db_pos_u;
            
            // Velocity deltas
            delta_vel_e <= reg_in_vel_e - db_vel_e;
            delta_vel_n <= reg_in_vel_n - db_vel_n;
            delta_vel_u <= reg_in_vel_u - db_vel_u;
            
            // Combined covariance (simplified: sum of diagonals)
            automatic logic [COV_WIDTH-1:0] cov_sum_pos = reg_in_cov_pos + db_cov_pos;
            automatic logic [COV_WIDTH-1:0] cov_sum_vel = reg_in_cov_vel + db_cov_vel;
            
            // Mahalanobis distance squared (simplified)
            // d² = Σ (Δx_i)² / σ_i²
            if (cov_sum_pos > 0 && cov_sum_vel > 0) begin
                distance_squared <= (delta_pos_e * delta_pos_e + 
                                    delta_pos_n * delta_pos_n + 
                                    delta_pos_u * delta_pos_u) / cov_sum_pos +
                                   (delta_vel_e * delta_vel_e + 
                                    delta_vel_n * delta_vel_n + 
                                    delta_vel_u * delta_vel_u) / cov_sum_vel;
            end else begin
                distance_squared <= 64'hFFFF_FFFF_FFFF_FFFF;  // Max distance
            end
        end
    end
    
    //-------------------------------------------------------------------------
    // Best Match Tracking
    //-------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            best_match_id   <= '0;
            best_match_dist <= 32'hFFFF_FFFF;
            match_found     <= 1'b0;
        end else begin
            case (state)
                ST_RECEIVE: begin
                    best_match_id   <= '0;
                    best_match_dist <= 32'hFFFF_FFFF;
                    match_found     <= 1'b0;
                end
                
                ST_ASSOCIATE: begin
                    if (db_query_found && distance_squared < best_match_dist && 
                        distance_squared < cfg_assoc_threshold) begin
                        best_match_id   <= db_track_id;
                        best_match_dist <= distance_squared[31:0];
                        match_found     <= 1'b1;
                    end
                end
                
                default: ;
            endcase
        end
    end
    
    //-------------------------------------------------------------------------
    // Covariance Intersection Fusion
    //-------------------------------------------------------------------------
    // P_fused^-1 = ω * P1^-1 + (1-ω) * P2^-1
    // x_fused = P_fused * [ω * P1^-1 * x1 + (1-ω) * P2^-1 * x2]
    // Simplified: using ω = 0.5
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fused_id        <= '0;
            fused_sources   <= '0;
            fused_class     <= '0;
            fused_pos_e     <= '0;
            fused_pos_n     <= '0;
            fused_pos_u     <= '0;
            fused_vel_e     <= '0;
            fused_vel_n     <= '0;
            fused_vel_u     <= '0;
            fused_cov_pos   <= '0;
            fused_cov_vel   <= '0;
            fused_timestamp <= '0;
            fused_quality   <= '0;
            fused_update_count <= '0;
        end else if (state == ST_FUSE) begin
            fused_id <= best_match_id;
            fused_sources <= db_sources | (8'b1 << reg_in_source);
            fused_class <= (reg_in_class != 0) ? reg_in_class : db_query_data[25:18];
            
            // Covariance Intersection with ω = 0.5
            // Simplified: weighted average inversely proportional to covariance
            automatic logic [63:0] w1 = (reg_in_cov_pos > 0) ? 
                                        (64'd1_000_000 / reg_in_cov_pos) : 64'd1;
            automatic logic [63:0] w2 = (db_cov_pos > 0) ? 
                                        (64'd1_000_000 / db_cov_pos) : 64'd1;
            automatic logic [63:0] w_sum = w1 + w2;
            
            if (w_sum > 0) begin
                fused_pos_e <= (w1 * reg_in_pos_e + w2 * db_pos_e) / w_sum;
                fused_pos_n <= (w1 * reg_in_pos_n + w2 * db_pos_n) / w_sum;
                fused_pos_u <= (w1 * reg_in_pos_u + w2 * db_pos_u) / w_sum;
                fused_vel_e <= (w1 * reg_in_vel_e + w2 * db_vel_e) / w_sum;
                fused_vel_n <= (w1 * reg_in_vel_n + w2 * db_vel_n) / w_sum;
                fused_vel_u <= (w1 * reg_in_vel_u + w2 * db_vel_u) / w_sum;
            end
            
            // Fused covariance (CI formula simplified)
            fused_cov_pos <= (reg_in_cov_pos * db_cov_pos) / (reg_in_cov_pos + db_cov_pos + 1);
            fused_cov_vel <= (reg_in_cov_vel * db_cov_vel) / (reg_in_cov_vel + db_cov_vel + 1);
            
            fused_timestamp <= reg_in_timestamp;
            fused_quality <= (reg_in_quality + db_quality) / 2 + 8'd10;  // Boost for fusion
            fused_update_count <= db_update_count + 1;
            
        end else if (state == ST_CREATE) begin
            // New track from single source
            fused_id <= next_track_id;
            fused_sources <= (8'b1 << reg_in_source);
            fused_class <= reg_in_class;
            fused_pos_e <= reg_in_pos_e;
            fused_pos_n <= reg_in_pos_n;
            fused_pos_u <= reg_in_pos_u;
            fused_vel_e <= reg_in_vel_e;
            fused_vel_n <= reg_in_vel_n;
            fused_vel_u <= reg_in_vel_u;
            fused_cov_pos <= reg_in_cov_pos;
            fused_cov_vel <= reg_in_cov_vel;
            fused_timestamp <= reg_in_timestamp;
            fused_quality <= reg_in_quality;
            fused_update_count <= 8'd1;
        end
    end
    
    //-------------------------------------------------------------------------
    // Database Update
    //-------------------------------------------------------------------------
    assign db_update_id = fused_id;
    assign db_update_create = (state == ST_CREATE);
    assign db_update_valid = (state == ST_UPDATE_DB);
    
    // Pack fused track into database format
    assign db_update_data = {
        118'b0,                        // Reserved [511:394]
        32'b0,                         // Age [393:362]
        fused_update_count,            // Update count [361:354]
        fused_quality,                 // Quality [353:346]
        fused_timestamp,               // Timestamp [345:282]
        fused_cov_vel,                 // Cov velocity [281:250]
        fused_cov_pos,                 // Cov position [249:218]
        fused_vel_u,                   // Vel up [217:186]
        fused_vel_n,                   // Vel north [185:154]
        fused_vel_e,                   // Vel east [153:122]
        fused_pos_u,                   // Pos up [121:90]
        fused_pos_n,                   // Pos north [89:58]
        fused_pos_e,                   // Pos east [57:26]
        fused_class,                   // Class [25:18]
        fused_sources,                 // Sources [17:10]
        fused_id                       // Track ID [9:0]
    };
    
    //-------------------------------------------------------------------------
    // Output
    //-------------------------------------------------------------------------
    assign out_track_id     = fused_id;
    assign out_source_bitmap = fused_sources;
    assign out_track_class  = fused_class;
    assign out_pos_east     = fused_pos_e;
    assign out_pos_north    = fused_pos_n;
    assign out_pos_up       = fused_pos_u;
    assign out_vel_east     = fused_vel_e;
    assign out_vel_north    = fused_vel_n;
    assign out_vel_up       = fused_vel_u;
    assign out_cov_pos      = fused_cov_pos;
    assign out_cov_vel      = fused_cov_vel;
    assign out_timestamp    = fused_timestamp;
    assign out_quality      = fused_quality;
    assign out_valid        = (state == ST_OUTPUT);
    
    //-------------------------------------------------------------------------
    // Statistics
    //-------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fusions_performed  <= '0;
            new_tracks_created <= '0;
            tracks_deleted     <= '0;
            active_tracks      <= '0;
        end else begin
            if (state == ST_FUSE)
                fusions_performed <= fusions_performed + 1;
            if (state == ST_CREATE && db_update_ready)
                new_tracks_created <= new_tracks_created + 1;
            // active_tracks updated by external track manager
        end
    end

endmodule
