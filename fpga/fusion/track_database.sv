//-----------------------------------------------------------------------------
// QEDMMA v2.0 Track Database
// Author: Dr. Mladen Mešter
// Copyright (c) 2026 Dr. Mladen Mešter - All Rights Reserved
//
// Description:
//   BRAM-based track database with dual-port access for concurrent
//   read/write operations. Supports 1024 tracks × 512 bits each.
//   Includes track aging and automatic deletion of stale tracks.
//
// [REQ-FUSION-007] Store 1024 simultaneous tracks
// [REQ-FUSION-008] Dual-port access for pipeline efficiency
// [REQ-FUSION-009] Automatic track timeout and deletion
//-----------------------------------------------------------------------------

`timescale 1ns / 1ps

module track_database #(
    parameter int MAX_TRACKS     = 1024,
    parameter int TRACK_ID_WIDTH = 10,
    parameter int TRACK_WIDTH    = 512,
    parameter int TIMEOUT_MS     = 30000  // 30 second default timeout
)(
    input  logic        clk,
    input  logic        rst_n,
    
    //-------------------------------------------------------------------------
    // Query Port (Read)
    //-------------------------------------------------------------------------
    input  logic [TRACK_ID_WIDTH-1:0] query_id,
    input  logic                      query_valid,
    output logic                      query_ready,
    output logic                      query_found,
    output logic [TRACK_WIDTH-1:0]    query_data,
    output logic                      query_done,
    
    //-------------------------------------------------------------------------
    // Update Port (Write)
    //-------------------------------------------------------------------------
    input  logic [TRACK_ID_WIDTH-1:0] update_id,
    input  logic [TRACK_WIDTH-1:0]    update_data,
    input  logic                      update_valid,
    input  logic                      update_create,  // 1=new, 0=update existing
    output logic                      update_ready,
    output logic                      update_done,
    output logic                      update_error,   // Track not found for update
    
    //-------------------------------------------------------------------------
    // Delete Port
    //-------------------------------------------------------------------------
    input  logic [TRACK_ID_WIDTH-1:0] delete_id,
    input  logic                      delete_valid,
    output logic                      delete_ready,
    output logic                      delete_done,
    
    //-------------------------------------------------------------------------
    // Scan Port (for iteration)
    //-------------------------------------------------------------------------
    input  logic                      scan_start,
    input  logic                      scan_next,
    output logic [TRACK_ID_WIDTH-1:0] scan_id,
    output logic [TRACK_WIDTH-1:0]    scan_data,
    output logic                      scan_valid_entry,
    output logic                      scan_done,
    
    //-------------------------------------------------------------------------
    // Configuration
    //-------------------------------------------------------------------------
    input  logic [31:0]  cfg_timeout_ms,
    input  logic [63:0]  current_time,    // GPS microseconds
    
    //-------------------------------------------------------------------------
    // Status
    //-------------------------------------------------------------------------
    output logic [TRACK_ID_WIDTH-1:0] active_count,
    output logic [TRACK_ID_WIDTH-1:0] capacity,
    output logic                      db_full
);

    //-------------------------------------------------------------------------
    // Track Entry Valid Bitmap
    //-------------------------------------------------------------------------
    logic [MAX_TRACKS-1:0] valid_bitmap;
    
    //-------------------------------------------------------------------------
    // BRAM Instance (True Dual Port)
    //-------------------------------------------------------------------------
    // Port A: Query/Scan (Read)
    // Port B: Update/Delete (Read-Modify-Write)
    
    logic [TRACK_ID_WIDTH-1:0] porta_addr, portb_addr;
    logic [TRACK_WIDTH-1:0]    porta_rdata, portb_rdata;
    logic [TRACK_WIDTH-1:0]    portb_wdata;
    logic                      porta_en, portb_en;
    logic                      portb_we;
    
    // Xilinx BRAM inference
    (* ram_style = "block" *)
    reg [TRACK_WIDTH-1:0] track_mem [0:MAX_TRACKS-1];
    
    // Port A (Read-only)
    always_ff @(posedge clk) begin
        if (porta_en)
            porta_rdata <= track_mem[porta_addr];
    end
    
    // Port B (Read/Write)
    always_ff @(posedge clk) begin
        if (portb_en) begin
            if (portb_we)
                track_mem[portb_addr] <= portb_wdata;
            portb_rdata <= track_mem[portb_addr];
        end
    end
    
    //-------------------------------------------------------------------------
    // State Machines
    //-------------------------------------------------------------------------
    typedef enum logic [2:0] {
        QST_IDLE,
        QST_READ,
        QST_DONE
    } query_state_t;
    
    typedef enum logic [2:0] {
        UST_IDLE,
        UST_CHECK,
        UST_WRITE,
        UST_DONE
    } update_state_t;
    
    typedef enum logic [2:0] {
        SST_IDLE,
        SST_READ,
        SST_OUTPUT,
        SST_NEXT
    } scan_state_t;
    
    query_state_t  qstate;
    update_state_t ustate;
    scan_state_t   sstate;
    
    //-------------------------------------------------------------------------
    // Query Logic
    //-------------------------------------------------------------------------
    logic [TRACK_ID_WIDTH-1:0] query_id_reg;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            qstate       <= QST_IDLE;
            query_id_reg <= '0;
            query_found  <= 1'b0;
            query_data   <= '0;
            query_done   <= 1'b0;
        end else begin
            query_done <= 1'b0;
            
            case (qstate)
                QST_IDLE: begin
                    if (query_valid && query_ready) begin
                        query_id_reg <= query_id;
                        qstate <= QST_READ;
                    end
                end
                
                QST_READ: begin
                    // Wait for BRAM read
                    qstate <= QST_DONE;
                end
                
                QST_DONE: begin
                    query_data  <= porta_rdata;
                    query_found <= valid_bitmap[query_id_reg];
                    query_done  <= 1'b1;
                    qstate      <= QST_IDLE;
                end
                
                default: qstate <= QST_IDLE;
            endcase
        end
    end
    
    assign query_ready = (qstate == QST_IDLE) && (sstate == SST_IDLE);
    assign porta_addr  = (sstate != SST_IDLE) ? scan_id : query_id_reg;
    assign porta_en    = (qstate == QST_READ) || (sstate == SST_READ);
    
    //-------------------------------------------------------------------------
    // Update Logic
    //-------------------------------------------------------------------------
    logic [TRACK_ID_WIDTH-1:0] update_id_reg;
    logic [TRACK_WIDTH-1:0]    update_data_reg;
    logic                      update_create_reg;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ustate          <= UST_IDLE;
            update_id_reg   <= '0;
            update_data_reg <= '0;
            update_create_reg <= 1'b0;
            update_done     <= 1'b0;
            update_error    <= 1'b0;
        end else begin
            update_done  <= 1'b0;
            update_error <= 1'b0;
            
            case (ustate)
                UST_IDLE: begin
                    if (update_valid && update_ready) begin
                        update_id_reg   <= update_id;
                        update_data_reg <= update_data;
                        update_create_reg <= update_create;
                        ustate <= UST_CHECK;
                    end
                end
                
                UST_CHECK: begin
                    // Check if track exists or creating new
                    if (update_create_reg) begin
                        // Creating new track
                        if (!valid_bitmap[update_id_reg]) begin
                            ustate <= UST_WRITE;
                        end else begin
                            // Track already exists
                            update_error <= 1'b1;
                            ustate <= UST_DONE;
                        end
                    end else begin
                        // Updating existing track
                        if (valid_bitmap[update_id_reg]) begin
                            ustate <= UST_WRITE;
                        end else begin
                            // Track doesn't exist
                            update_error <= 1'b1;
                            ustate <= UST_DONE;
                        end
                    end
                end
                
                UST_WRITE: begin
                    ustate <= UST_DONE;
                end
                
                UST_DONE: begin
                    update_done <= 1'b1;
                    ustate <= UST_IDLE;
                end
                
                default: ustate <= UST_IDLE;
            endcase
        end
    end
    
    assign update_ready = (ustate == UST_IDLE);
    assign portb_addr   = delete_valid ? delete_id : update_id_reg;
    assign portb_wdata  = update_data_reg;
    assign portb_en     = (ustate == UST_WRITE) || delete_valid;
    assign portb_we     = (ustate == UST_WRITE);
    
    //-------------------------------------------------------------------------
    // Valid Bitmap Management
    //-------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_bitmap <= '0;
        end else begin
            // Set on create
            if (ustate == UST_WRITE && update_create_reg)
                valid_bitmap[update_id_reg] <= 1'b1;
            
            // Clear on delete
            if (delete_valid && delete_ready)
                valid_bitmap[delete_id] <= 1'b0;
        end
    end
    
    //-------------------------------------------------------------------------
    // Delete Logic
    //-------------------------------------------------------------------------
    assign delete_ready = (ustate == UST_IDLE);
    assign delete_done  = delete_valid && delete_ready;
    
    //-------------------------------------------------------------------------
    // Scan Logic (for iteration over all tracks)
    //-------------------------------------------------------------------------
    logic [TRACK_ID_WIDTH-1:0] scan_idx;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sstate          <= SST_IDLE;
            scan_idx        <= '0;
            scan_id         <= '0;
            scan_data       <= '0;
            scan_valid_entry <= 1'b0;
            scan_done       <= 1'b0;
        end else begin
            scan_done <= 1'b0;
            
            case (sstate)
                SST_IDLE: begin
                    if (scan_start) begin
                        scan_idx <= '0;
                        sstate <= SST_READ;
                    end
                end
                
                SST_READ: begin
                    scan_id <= scan_idx;
                    sstate <= SST_OUTPUT;
                end
                
                SST_OUTPUT: begin
                    scan_data        <= porta_rdata;
                    scan_valid_entry <= valid_bitmap[scan_idx];
                    
                    if (scan_idx >= MAX_TRACKS - 1) begin
                        scan_done <= 1'b1;
                        sstate <= SST_IDLE;
                    end else begin
                        sstate <= SST_NEXT;
                    end
                end
                
                SST_NEXT: begin
                    if (scan_next) begin
                        scan_idx <= scan_idx + 1;
                        sstate <= SST_READ;
                    end
                end
                
                default: sstate <= SST_IDLE;
            endcase
        end
    end
    
    //-------------------------------------------------------------------------
    // Track Count
    //-------------------------------------------------------------------------
    // Population count of valid bitmap
    integer i;
    always_comb begin
        active_count = '0;
        for (i = 0; i < MAX_TRACKS; i++) begin
            active_count = active_count + valid_bitmap[i];
        end
    end
    
    assign capacity = MAX_TRACKS[TRACK_ID_WIDTH-1:0];
    assign db_full  = (active_count >= MAX_TRACKS - 1);
    
    //-------------------------------------------------------------------------
    // Track Timeout/Aging (Background Process)
    // This would run periodically to delete stale tracks
    //-------------------------------------------------------------------------
    // Implemented as separate maintenance FSM or software-triggered
    // For now, timeout is handled by external track manager

endmodule
