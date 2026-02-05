// ═══════════════════════════════════════════════════════════════════════════════
// NX-MIMOSA VGPO CONTINUITY FILTER
// ═══════════════════════════════════════════════════════════════════════════════
//
// Velocity Gate Pull-Off (VGPO) jammer detection via continuity analysis.
// Detects non-physical acceleration patterns indicating Doppler manipulation.
//
// Algorithm:
//   1. Track velocity over N pulses in CPI
//   2. Compute acceleration (1st derivative) and jerk (2nd derivative)
//   3. Flag if |acceleration| > MAX_ACCEL (physical limit)
//   4. Flag if velocity variance exceeds expected noise
//
// Traceability:
//   [REQ-ECCM-VGPO-001] VGPO jammer detection
//   [REQ-SIM-VGPO-001] Physics validation (non-physical accel detected)
//
// Author: Dr. Mladen Mešter / Nexellum d.o.o.
// Version: 1.1.0
// License: AGPL v3 / Commercial
// ═══════════════════════════════════════════════════════════════════════════════

`timescale 1ns/1ps

module vgpo_continuity_filter #(
    // ═══════════════════════════════════════════════════════════════════════════
    // Parameters
    // ═══════════════════════════════════════════════════════════════════════════
    parameter int DATA_WIDTH      = 16,           // Velocity data width (Q8.8)
    parameter int TRACK_ID_WIDTH  = 8,            // Track ID bits
    parameter int CPI_LENGTH      = 64,           // Pulses per CPI
    parameter int MAX_TRACKS      = 64,           // Maximum concurrent tracks
    
    // Physical limits (Q8.8 format)
    // 50 m/s² = 50 * 256 = 12800 in Q8.8
    parameter logic signed [DATA_WIDTH-1:0] MAX_ACCEL = 16'sd12800,
    
    // Variance threshold for noise-based detection
    // Expected velocity noise σ² ≈ 4 m²/s² → 4 * 256² = 262144
    parameter int MAX_VARIANCE = 300000
)(
    // ═══════════════════════════════════════════════════════════════════════════
    // Clock & Reset
    // ═══════════════════════════════════════════════════════════════════════════
    input  logic                          clk,
    input  logic                          rst_n,
    
    // ═══════════════════════════════════════════════════════════════════════════
    // Input: Track Velocity Stream
    // ═══════════════════════════════════════════════════════════════════════════
    input  logic                          velocity_valid,
    input  logic [TRACK_ID_WIDTH-1:0]     velocity_track_id,
    input  logic signed [DATA_WIDTH-1:0]  velocity_data,      // Current velocity (Q8.8 m/s)
    input  logic                          velocity_cpi_start, // First pulse of CPI
    input  logic                          velocity_cpi_end,   // Last pulse of CPI
    output logic                          velocity_ready,
    
    // ═══════════════════════════════════════════════════════════════════════════
    // Output: Jammer Detection
    // ═══════════════════════════════════════════════════════════════════════════
    output logic                          jammer_valid,
    output logic [TRACK_ID_WIDTH-1:0]     jammer_track_id,
    output logic                          jammer_detected,    // 1 = VGPO suspected
    output logic [1:0]                    jammer_reason,      // 0=none, 1=accel, 2=variance, 3=both
    output logic signed [31:0]            jammer_max_accel,   // Maximum observed acceleration
    
    // ═══════════════════════════════════════════════════════════════════════════
    // Configuration
    // ═══════════════════════════════════════════════════════════════════════════
    input  logic                          cfg_enable,
    input  logic signed [DATA_WIDTH-1:0]  cfg_max_accel,      // Override MAX_ACCEL
    input  logic [31:0]                   cfg_max_variance    // Override MAX_VARIANCE
);

    // ═══════════════════════════════════════════════════════════════════════════
    // Local Parameters
    // ═══════════════════════════════════════════════════════════════════════════
    localparam int HISTORY_DEPTH = 4;  // Velocity history for derivative calculation
    
    // ═══════════════════════════════════════════════════════════════════════════
    // Track State Memory
    // ═══════════════════════════════════════════════════════════════════════════
    
    // Velocity history buffer per track
    logic signed [DATA_WIDTH-1:0] vel_history [0:MAX_TRACKS-1][0:HISTORY_DEPTH-1];
    logic [$clog2(HISTORY_DEPTH)-1:0] history_ptr [0:MAX_TRACKS-1];
    logic [HISTORY_DEPTH-1:0] history_valid [0:MAX_TRACKS-1];
    
    // Running statistics per track
    logic signed [31:0] vel_sum [0:MAX_TRACKS-1];
    logic signed [63:0] vel_sum_sq [0:MAX_TRACKS-1];
    logic [$clog2(CPI_LENGTH)-1:0] sample_count [0:MAX_TRACKS-1];
    logic signed [31:0] max_accel_observed [0:MAX_TRACKS-1];
    
    // ═══════════════════════════════════════════════════════════════════════════
    // Pipeline Registers
    // ═══════════════════════════════════════════════════════════════════════════
    
    // Stage 1: Capture and history update
    logic                          s1_valid;
    logic [TRACK_ID_WIDTH-1:0]     s1_track_id;
    logic signed [DATA_WIDTH-1:0]  s1_velocity;
    logic signed [DATA_WIDTH-1:0]  s1_prev_velocity;
    logic signed [DATA_WIDTH-1:0]  s1_prev_prev_velocity;
    logic                          s1_cpi_end;
    logic                          s1_history_valid;
    
    // Stage 2: Derivative computation
    logic                          s2_valid;
    logic [TRACK_ID_WIDTH-1:0]     s2_track_id;
    logic signed [31:0]            s2_acceleration;
    logic signed [31:0]            s2_jerk;
    logic                          s2_cpi_end;
    
    // Stage 3: Threshold check and variance
    logic                          s3_valid;
    logic [TRACK_ID_WIDTH-1:0]     s3_track_id;
    logic                          s3_accel_exceeded;
    logic                          s3_variance_exceeded;
    logic signed [31:0]            s3_max_accel;
    
    // ═══════════════════════════════════════════════════════════════════════════
    // Effective thresholds (config or default)
    // ═══════════════════════════════════════════════════════════════════════════
    wire signed [DATA_WIDTH-1:0] eff_max_accel = (cfg_max_accel != '0) ? cfg_max_accel : MAX_ACCEL;
    wire [31:0] eff_max_variance = (cfg_max_variance != '0) ? cfg_max_variance : MAX_VARIANCE;
    
    // ═══════════════════════════════════════════════════════════════════════════
    // Flow Control
    // ═══════════════════════════════════════════════════════════════════════════
    assign velocity_ready = cfg_enable;
    
    // ═══════════════════════════════════════════════════════════════════════════
    // STAGE 1: Capture Velocity & Update History
    // ═══════════════════════════════════════════════════════════════════════════
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1_valid <= 1'b0;
            s1_history_valid <= 1'b0;
            
            // Initialize all track histories
            for (int t = 0; t < MAX_TRACKS; t++) begin
                for (int h = 0; h < HISTORY_DEPTH; h++) begin
                    vel_history[t][h] <= '0;
                end
                history_ptr[t] <= '0;
                history_valid[t] <= '0;
                vel_sum[t] <= '0;
                vel_sum_sq[t] <= '0;
                sample_count[t] <= '0;
                max_accel_observed[t] <= '0;
            end
        end else if (cfg_enable && velocity_valid) begin
            automatic int tid = velocity_track_id;
            
            // Reset statistics at CPI start
            if (velocity_cpi_start) begin
                vel_sum[tid] <= velocity_data;
                vel_sum_sq[tid] <= velocity_data * velocity_data;
                sample_count[tid] <= 1;
                max_accel_observed[tid] <= '0;
            end else begin
                vel_sum[tid] <= vel_sum[tid] + velocity_data;
                vel_sum_sq[tid] <= vel_sum_sq[tid] + velocity_data * velocity_data;
                sample_count[tid] <= sample_count[tid] + 1;
            end
            
            // Capture previous velocities from history
            s1_prev_velocity <= vel_history[tid][history_ptr[tid]];
            s1_prev_prev_velocity <= vel_history[tid][(history_ptr[tid] + HISTORY_DEPTH - 1) % HISTORY_DEPTH];
            s1_history_valid <= (history_valid[tid] >= 2'b11);
            
            // Update history buffer
            vel_history[tid][history_ptr[tid]] <= velocity_data;
            history_ptr[tid] <= (history_ptr[tid] + 1) % HISTORY_DEPTH;
            
            if (history_valid[tid] != {HISTORY_DEPTH{1'b1}}) begin
                history_valid[tid] <= {history_valid[tid][HISTORY_DEPTH-2:0], 1'b1};
            end
            
            // Pass to next stage
            s1_valid <= 1'b1;
            s1_track_id <= velocity_track_id;
            s1_velocity <= velocity_data;
            s1_cpi_end <= velocity_cpi_end;
        end else begin
            s1_valid <= 1'b0;
        end
    end
    
    // ═══════════════════════════════════════════════════════════════════════════
    // STAGE 2: Compute Derivatives
    // ═══════════════════════════════════════════════════════════════════════════
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s2_valid <= 1'b0;
            s2_acceleration <= '0;
            s2_jerk <= '0;
        end else if (s1_valid && s1_history_valid) begin
            // First derivative: acceleration = dv/dt
            // Assuming unit time step between pulses
            s2_acceleration <= s1_velocity - s1_prev_velocity;
            
            // Second derivative: jerk = d²v/dt²
            // Using central difference approximation
            s2_jerk <= s1_velocity - 2 * s1_prev_velocity + s1_prev_prev_velocity;
            
            s2_valid <= 1'b1;
            s2_track_id <= s1_track_id;
            s2_cpi_end <= s1_cpi_end;
        end else begin
            s2_valid <= 1'b0;
        end
    end
    
    // ═══════════════════════════════════════════════════════════════════════════
    // STAGE 3: Threshold Check & Variance Analysis
    // ═══════════════════════════════════════════════════════════════════════════
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s3_valid <= 1'b0;
            s3_accel_exceeded <= 1'b0;
            s3_variance_exceeded <= 1'b0;
        end else if (s2_valid) begin
            automatic int tid = s2_track_id;
            automatic logic signed [31:0] abs_accel;
            automatic logic signed [63:0] variance;
            automatic logic signed [31:0] mean;
            automatic int n;
            
            // Absolute acceleration
            abs_accel = (s2_acceleration < 0) ? -s2_acceleration : s2_acceleration;
            
            // Update max acceleration observed
            if (abs_accel > max_accel_observed[tid]) begin
                max_accel_observed[tid] <= abs_accel;
            end
            
            // Check acceleration threshold
            s3_accel_exceeded <= (abs_accel > eff_max_accel);
            
            // Compute variance at CPI end
            if (s2_cpi_end && sample_count[tid] > 1) begin
                n = sample_count[tid];
                mean = vel_sum[tid] / n;
                // Var = E[X²] - E[X]²
                variance = (vel_sum_sq[tid] / n) - (mean * mean);
                
                s3_variance_exceeded <= (variance > eff_max_variance);
            end else begin
                s3_variance_exceeded <= 1'b0;
            end
            
            s3_valid <= 1'b1;
            s3_track_id <= s2_track_id;
            s3_max_accel <= max_accel_observed[tid];
        end else begin
            s3_valid <= 1'b0;
        end
    end
    
    // ═══════════════════════════════════════════════════════════════════════════
    // Output Generation
    // ═══════════════════════════════════════════════════════════════════════════
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            jammer_valid <= 1'b0;
            jammer_detected <= 1'b0;
            jammer_reason <= 2'b00;
        end else if (s3_valid) begin
            jammer_valid <= 1'b1;
            jammer_track_id <= s3_track_id;
            jammer_detected <= s3_accel_exceeded || s3_variance_exceeded;
            jammer_reason <= {s3_variance_exceeded, s3_accel_exceeded};
            jammer_max_accel <= s3_max_accel;
        end else begin
            jammer_valid <= 1'b0;
        end
    end

endmodule


// ═══════════════════════════════════════════════════════════════════════════════
// VGPO MITIGATION MODULE
// ═══════════════════════════════════════════════════════════════════════════════
// Applies mitigation strategies when VGPO is detected:
//   1. Switch to leading-edge tracking
//   2. Freeze velocity estimate
//   3. Drop track and re-initialize

module vgpo_mitigation #(
    parameter int DATA_WIDTH = 16,
    parameter int TRACK_ID_WIDTH = 8
)(
    input  logic                          clk,
    input  logic                          rst_n,
    
    // Jammer detection input
    input  logic                          jammer_valid,
    input  logic [TRACK_ID_WIDTH-1:0]     jammer_track_id,
    input  logic                          jammer_detected,
    input  logic [1:0]                    jammer_reason,
    
    // Track input
    input  logic                          track_valid,
    input  logic [TRACK_ID_WIDTH-1:0]     track_id,
    input  logic signed [DATA_WIDTH-1:0]  track_range,
    input  logic signed [DATA_WIDTH-1:0]  track_velocity,
    
    // Mitigated track output
    output logic                          mitigated_valid,
    output logic [TRACK_ID_WIDTH-1:0]     mitigated_track_id,
    output logic signed [DATA_WIDTH-1:0]  mitigated_range,
    output logic signed [DATA_WIDTH-1:0]  mitigated_velocity,
    output logic                          mitigated_frozen,
    output logic                          mitigated_drop,
    
    // Configuration
    input  logic [1:0]                    cfg_mitigation_mode  // 0=freeze, 1=leading-edge, 2=drop
);

    // Track jammer status memory
    logic jammer_status [0:255];
    logic [1:0] jammer_count [0:255];
    logic signed [DATA_WIDTH-1:0] frozen_velocity [0:255];
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mitigated_valid <= 1'b0;
            for (int i = 0; i < 256; i++) begin
                jammer_status[i] <= 1'b0;
                jammer_count[i] <= 2'b00;
                frozen_velocity[i] <= '0;
            end
        end else begin
            // Update jammer status
            if (jammer_valid) begin
                if (jammer_detected) begin
                    jammer_status[jammer_track_id] <= 1'b1;
                    if (jammer_count[jammer_track_id] < 2'b11) begin
                        jammer_count[jammer_track_id] <= jammer_count[jammer_track_id] + 1;
                    end
                end else begin
                    // Clear after consecutive clean detections
                    if (jammer_count[jammer_track_id] > 0) begin
                        jammer_count[jammer_track_id] <= jammer_count[jammer_track_id] - 1;
                    end else begin
                        jammer_status[jammer_track_id] <= 1'b0;
                    end
                end
            end
            
            // Apply mitigation to tracks
            if (track_valid) begin
                mitigated_valid <= 1'b1;
                mitigated_track_id <= track_id;
                mitigated_range <= track_range;
                
                if (jammer_status[track_id]) begin
                    case (cfg_mitigation_mode)
                        2'b00: begin // Freeze velocity
                            if (!jammer_status[track_id]) begin
                                frozen_velocity[track_id] <= track_velocity;
                            end
                            mitigated_velocity <= frozen_velocity[track_id];
                            mitigated_frozen <= 1'b1;
                            mitigated_drop <= 1'b0;
                        end
                        2'b01: begin // Leading-edge (use range only)
                            mitigated_velocity <= '0;
                            mitigated_frozen <= 1'b1;
                            mitigated_drop <= 1'b0;
                        end
                        2'b10: begin // Drop track
                            mitigated_velocity <= track_velocity;
                            mitigated_frozen <= 1'b0;
                            mitigated_drop <= 1'b1;
                        end
                        default: begin
                            mitigated_velocity <= track_velocity;
                            mitigated_frozen <= 1'b0;
                            mitigated_drop <= 1'b0;
                        end
                    endcase
                end else begin
                    mitigated_velocity <= track_velocity;
                    mitigated_frozen <= 1'b0;
                    mitigated_drop <= 1'b0;
                end
            end else begin
                mitigated_valid <= 1'b0;
            end
        end
    end

endmodule
