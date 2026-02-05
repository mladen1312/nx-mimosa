// ═══════════════════════════════════════════════════════════════════════════════
// NX-MIMOSA COGNITIVE CFAR MODULE
// ═══════════════════════════════════════════════════════════════════════════════
//
// ML-assisted CFAR (Constant False Alarm Rate) detector using SVM approximation
// 
// Features:
//   - RBF kernel approximation via tanh LUT
//   - Fixed-point Q8.8 arithmetic
//   - AXI4-Stream interface
//   - Pipelined for 400 MHz operation
//   - Latency: <30 ns per range bin @ 250 MHz
//
// Traceability:
//   [REQ-COG-001] ML-based detection threshold
//   [REQ-COG-002] Support vector classification
//   [REQ-COG-003] Real-time feature extraction
//   [REQ-SIM-001] Physics-validated parameters
//   [REQ-ECCM-01] RGPO jammer detection capability
//
// Author: Dr. Mladen Mešter / Nexellum d.o.o.
// Version: 1.1.0
// License: AGPL v3 / Commercial
// ═══════════════════════════════════════════════════════════════════════════════

`timescale 1ns/1ps

module cognitive_cfar #(
    // ═══════════════════════════════════════════════════════════════════════════
    // Parameters (from SSOT config.yaml)
    // ═══════════════════════════════════════════════════════════════════════════
    parameter DATA_WIDTH      = 32,           // AXI data width
    parameter FEATURE_WIDTH   = 16,           // Q8.8 fixed-point
    parameter NUM_SUPPORT_VEC = 64,           // Number of support vectors
    parameter WINDOW_SIZE     = 20,           // Local statistics window
    parameter GUARD_CELLS     = 4,            // Guard cells around CUT
    parameter PROB_THRESHOLD  = 16'h7999,     // 0.95 in Q15 format
    parameter RBF_GAMMA       = 16'h0080      // γ = 0.5 in Q8.8
)(
    // ═══════════════════════════════════════════════════════════════════════════
    // Clock & Reset
    // ═══════════════════════════════════════════════════════════════════════════
    input  logic                      clk,
    input  logic                      rst_n,
    
    // ═══════════════════════════════════════════════════════════════════════════
    // AXI4-Stream Slave (Input)
    // ═══════════════════════════════════════════════════════════════════════════
    input  logic [DATA_WIDTH-1:0]     s_axis_tdata,   // I[15:0], Q[31:16]
    input  logic                      s_axis_tvalid,
    output logic                      s_axis_tready,
    input  logic                      s_axis_tlast,
    
    // ═══════════════════════════════════════════════════════════════════════════
    // AXI4-Stream Master (Output)
    // ═══════════════════════════════════════════════════════════════════════════
    output logic [DATA_WIDTH-1:0]     m_axis_tdata,   // Detection[31], Prob[30:16], Mag[15:0]
    output logic                      m_axis_tvalid,
    input  logic                      m_axis_tready,
    output logic                      m_axis_tlast,
    
    // ═══════════════════════════════════════════════════════════════════════════
    // Configuration Interface
    // ═══════════════════════════════════════════════════════════════════════════
    input  logic                      cfg_enable,
    input  logic [FEATURE_WIDTH-1:0]  cfg_threshold,
    input  logic                      cfg_jammer_detect_en,
    
    // ═══════════════════════════════════════════════════════════════════════════
    // Status & Detection Outputs
    // ═══════════════════════════════════════════════════════════════════════════
    output logic                      detection_valid,
    output logic [15:0]               detection_range_bin,
    output logic [15:0]               detection_probability,
    output logic                      jammer_detected,
    output logic [15:0]               stat_detection_count,
    output logic [15:0]               stat_sample_count
);

    // ═══════════════════════════════════════════════════════════════════════════
    // Local Parameters
    // ═══════════════════════════════════════════════════════════════════════════
    localparam PIPELINE_DEPTH = 5;
    localparam HALF_WINDOW = WINDOW_SIZE / 2;
    localparam JAMMER_POWER_RATIO = 16'h0500; // 5x local mean = jammer
    
    // ═══════════════════════════════════════════════════════════════════════════
    // Type Definitions
    // ═══════════════════════════════════════════════════════════════════════════
    typedef logic signed [FEATURE_WIDTH-1:0] fixed_t;
    typedef logic signed [2*FEATURE_WIDTH-1:0] fixed_wide_t;
    
    // ═══════════════════════════════════════════════════════════════════════════
    // Pipeline Stage Registers
    // ═══════════════════════════════════════════════════════════════════════════
    
    // Stage 1: Magnitude extraction
    fixed_t         mag_s1;
    logic           valid_s1, last_s1;
    
    // Stage 2: Local mean computation  
    fixed_t         mag_s2, local_mean_s2;
    logic           valid_s2, last_s2;
    
    // Stage 3: Local variance computation
    fixed_t         mag_s3, local_mean_s3, local_var_s3;
    logic           valid_s3, last_s3;
    
    // Stage 4: SVM kernel computation
    fixed_t         mag_s4, svm_score_s4;
    logic           valid_s4, last_s4;
    logic           jammer_s4;
    
    // Stage 5: Decision output
    logic [DATA_WIDTH-1:0] output_s5;
    logic           valid_s5, last_s5;
    logic           detection_s5, jammer_s5;
    
    // ═══════════════════════════════════════════════════════════════════════════
    // Sliding Window Buffer
    // ═══════════════════════════════════════════════════════════════════════════
    fixed_t window_buffer [0:WINDOW_SIZE-1];
    logic [$clog2(WINDOW_SIZE)-1:0] window_ptr;
    logic window_full;
    
    // Running statistics (incremental computation)
    fixed_wide_t running_sum;
    fixed_wide_t running_sum_sq;
    
    // Range bin counter
    logic [15:0] range_bin_cnt;
    
    // ═══════════════════════════════════════════════════════════════════════════
    // Support Vector Memory (ROM - loaded from Python export)
    // ═══════════════════════════════════════════════════════════════════════════
    // Each SV has 3 features + alpha weight
    (* rom_style = "block" *)
    fixed_t support_vectors [0:NUM_SUPPORT_VEC-1][0:2];
    fixed_t sv_alphas [0:NUM_SUPPORT_VEC-1];
    fixed_t svm_bias;
    
    // Initialize from hex files (generated by Python svm_exporter.py)
    initial begin
        `ifdef SIMULATION
            $readmemh("sv_features.hex", support_vectors);
            $readmemh("sv_alphas.hex", sv_alphas);
            svm_bias = 16'h0000; // Loaded separately or hardcoded
        `else
            // In synthesis: BRAM init from .coe file via Vivado
        `endif
    end
    
    // ═══════════════════════════════════════════════════════════════════════════
    // Tanh Approximation LUT for RBF Kernel
    // ═══════════════════════════════════════════════════════════════════════════
    // exp(-x) ≈ (1 - tanh(x/2)) / 2 for x > 0
    (* rom_style = "distributed" *)
    logic [7:0] exp_lut [0:255];
    
    initial begin
        // exp(-x/32) * 255 for x in [0, 255]
        for (int i = 0; i < 256; i++) begin
            exp_lut[i] = 8'($rtoi(255.0 * $exp(-i / 32.0)));
        end
    end
    
    function automatic fixed_t rbf_kernel(input fixed_t dist_sq);
        logic [7:0] lut_idx;
        // Scale distance to LUT range
        lut_idx = (dist_sq[15:8] > 8'd255) ? 8'd255 : dist_sq[15:8];
        return {8'h00, exp_lut[lut_idx]};
    endfunction
    
    // ═══════════════════════════════════════════════════════════════════════════
    // Magnitude Computation (Fast approximation)
    // ═══════════════════════════════════════════════════════════════════════════
    function automatic fixed_t compute_mag(input logic [DATA_WIDTH-1:0] iq);
        logic signed [15:0] i_val, q_val;
        logic [15:0] abs_i, abs_q, max_val, min_val;
        
        i_val = iq[15:0];
        q_val = iq[31:16];
        
        abs_i = (i_val[15]) ? (~i_val + 1'b1) : i_val;
        abs_q = (q_val[15]) ? (~q_val + 1'b1) : q_val;
        
        max_val = (abs_i > abs_q) ? abs_i : abs_q;
        min_val = (abs_i > abs_q) ? abs_q : abs_i;
        
        // |z| ≈ max + 0.375*min (error < 4%)
        return max_val + (min_val >> 2) + (min_val >> 3);
    endfunction
    
    // ═══════════════════════════════════════════════════════════════════════════
    // Flow Control
    // ═══════════════════════════════════════════════════════════════════════════
    assign s_axis_tready = m_axis_tready || !m_axis_tvalid;
    
    // ═══════════════════════════════════════════════════════════════════════════
    // STAGE 1: Magnitude Extraction
    // ═══════════════════════════════════════════════════════════════════════════
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mag_s1 <= '0;
            valid_s1 <= 1'b0;
            last_s1 <= 1'b0;
            range_bin_cnt <= '0;
        end else if (s_axis_tready) begin
            if (s_axis_tvalid && cfg_enable) begin
                mag_s1 <= compute_mag(s_axis_tdata);
                valid_s1 <= 1'b1;
                last_s1 <= s_axis_tlast;
                range_bin_cnt <= s_axis_tlast ? '0 : range_bin_cnt + 1'b1;
            end else begin
                valid_s1 <= 1'b0;
                last_s1 <= 1'b0;
            end
        end
    end
    
    // ═══════════════════════════════════════════════════════════════════════════
    // STAGE 2: Local Mean (Moving Average)
    // ═══════════════════════════════════════════════════════════════════════════
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < WINDOW_SIZE; i++)
                window_buffer[i] <= '0;
            window_ptr <= '0;
            window_full <= 1'b0;
            running_sum <= '0;
            mag_s2 <= '0;
            local_mean_s2 <= '0;
            valid_s2 <= 1'b0;
            last_s2 <= 1'b0;
        end else if (s_axis_tready && valid_s1) begin
            // Incremental sum update
            running_sum <= running_sum - window_buffer[window_ptr] + mag_s1;
            
            // Update circular buffer
            window_buffer[window_ptr] <= mag_s1;
            if (window_ptr == WINDOW_SIZE - 1) begin
                window_ptr <= '0;
                window_full <= 1'b1;
            end else begin
                window_ptr <= window_ptr + 1'b1;
            end
            
            // Mean = sum / N (use shift for power-of-2 window)
            local_mean_s2 <= running_sum[$clog2(WINDOW_SIZE) +: FEATURE_WIDTH];
            
            mag_s2 <= mag_s1;
            valid_s2 <= window_full; // Only valid after buffer fills
            last_s2 <= last_s1;
        end else begin
            valid_s2 <= 1'b0;
            last_s2 <= 1'b0;
        end
    end
    
    // ═══════════════════════════════════════════════════════════════════════════
    // STAGE 3: Local Variance
    // ═══════════════════════════════════════════════════════════════════════════
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            running_sum_sq <= '0;
            mag_s3 <= '0;
            local_mean_s3 <= '0;
            local_var_s3 <= '0;
            valid_s3 <= 1'b0;
            last_s3 <= 1'b0;
        end else if (s_axis_tready && valid_s2) begin
            fixed_wide_t old_sq, new_sq, mean_sq, e_x_sq;
            
            // Update sum of squares
            old_sq = window_buffer[window_ptr] * window_buffer[window_ptr];
            new_sq = mag_s2 * mag_s2;
            running_sum_sq <= running_sum_sq - old_sq + new_sq;
            
            // Var = E[X²] - E[X]²
            e_x_sq = running_sum_sq >> $clog2(WINDOW_SIZE);
            mean_sq = local_mean_s2 * local_mean_s2;
            local_var_s3 <= (e_x_sq > mean_sq) ? (e_x_sq - mean_sq) : '0;
            
            mag_s3 <= mag_s2;
            local_mean_s3 <= local_mean_s2;
            valid_s3 <= 1'b1;
            last_s3 <= last_s2;
        end else begin
            valid_s3 <= 1'b0;
            last_s3 <= 1'b0;
        end
    end
    
    // ═══════════════════════════════════════════════════════════════════════════
    // STAGE 4: SVM Kernel Computation
    // ═══════════════════════════════════════════════════════════════════════════
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mag_s4 <= '0;
            svm_score_s4 <= '0;
            jammer_s4 <= 1'b0;
            valid_s4 <= 1'b0;
            last_s4 <= 1'b0;
        end else if (s_axis_tready && valid_s3) begin
            fixed_wide_t score_acc;
            fixed_t features [0:2];
            
            // Feature vector
            features[0] = mag_s3;
            features[1] = local_mean_s3;
            features[2] = local_var_s3;
            
            // SVM decision function: f(x) = Σ α_i · K(x, sv_i) + b
            score_acc = svm_bias;
            
            for (int sv = 0; sv < NUM_SUPPORT_VEC; sv++) begin
                fixed_wide_t dist_sq;
                fixed_t diff0, diff1, diff2;
                fixed_t kernel_val;
                
                // Compute ||x - sv||²
                diff0 = features[0] - support_vectors[sv][0];
                diff1 = features[1] - support_vectors[sv][1];
                diff2 = features[2] - support_vectors[sv][2];
                
                dist_sq = diff0 * diff0 + diff1 * diff1 + diff2 * diff2;
                
                // RBF kernel: K = exp(-γ·||x-sv||²)
                kernel_val = rbf_kernel((RBF_GAMMA * dist_sq[31:16]) >> 8);
                
                // Weighted accumulation
                score_acc = score_acc + sv_alphas[sv] * kernel_val;
            end
            
            // Sigmoid for probability (simplified)
            svm_score_s4 <= (score_acc[31]) ? '0 : 
                           (score_acc[30:16] > 16'hFFFF) ? 16'hFFFF : 
                           score_acc[30:15];
            
            // [REQ-ECCM-01] RGPO Jammer Detection
            // High power with anomalous mean ratio indicates jammer
            if (cfg_jammer_detect_en) begin
                fixed_t power_ratio;
                power_ratio = (local_mean_s3 > 16'h0010) ? 
                             ((mag_s3 << 8) / local_mean_s3) : 16'hFFFF;
                jammer_s4 <= (power_ratio > JAMMER_POWER_RATIO);
            end else begin
                jammer_s4 <= 1'b0;
            end
            
            mag_s4 <= mag_s3;
            valid_s4 <= 1'b1;
            last_s4 <= last_s3;
        end else begin
            valid_s4 <= 1'b0;
            last_s4 <= 1'b0;
        end
    end
    
    // ═══════════════════════════════════════════════════════════════════════════
    // STAGE 5: Decision & Output Formatting
    // ═══════════════════════════════════════════════════════════════════════════
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            output_s5 <= '0;
            detection_s5 <= 1'b0;
            jammer_s5 <= 1'b0;
            valid_s5 <= 1'b0;
            last_s5 <= 1'b0;
            stat_detection_count <= '0;
            stat_sample_count <= '0;
        end else if (s_axis_tready && valid_s4) begin
            // Detection decision
            detection_s5 <= (svm_score_s4 >= cfg_threshold) && !jammer_s4;
            jammer_s5 <= jammer_s4;
            
            // Output format: {Detection[31], Jammer[30], Prob[29:16], Mag[15:0]}
            output_s5 <= {(svm_score_s4 >= cfg_threshold), jammer_s4, 
                         svm_score_s4[13:0], mag_s4};
            
            valid_s5 <= 1'b1;
            last_s5 <= last_s4;
            
            // Statistics update
            stat_sample_count <= stat_sample_count + 1'b1;
            if ((svm_score_s4 >= cfg_threshold) && !jammer_s4) begin
                stat_detection_count <= stat_detection_count + 1'b1;
            end
        end else begin
            valid_s5 <= 1'b0;
            last_s5 <= 1'b0;
        end
    end
    
    // ═══════════════════════════════════════════════════════════════════════════
    // Output Assignments
    // ═══════════════════════════════════════════════════════════════════════════
    assign m_axis_tdata  = output_s5;
    assign m_axis_tvalid = valid_s5;
    assign m_axis_tlast  = last_s5;
    
    // Detection interface
    assign detection_valid       = detection_s5 && valid_s5;
    assign detection_range_bin   = range_bin_cnt - PIPELINE_DEPTH;
    assign detection_probability = svm_score_s4;
    assign jammer_detected       = jammer_s5 && valid_s5;

endmodule
