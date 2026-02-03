//-----------------------------------------------------------------------------
// QEDMMA v2.0 ML-Assisted CFAR Engine
// Author: Dr. Mladen Mešter
// Copyright (c) 2026 Dr. Mladen Mešter - All Rights Reserved
//
// Description:
//   Machine Learning-assisted Constant False Alarm Rate detector with
//   jammer/clutter classification capability. Implements:
//   - Cell-Averaging CFAR (CA-CFAR) baseline
//   - Feature extraction for ML classification
//   - Lightweight inference engine (decision tree / simple NN)
//   - Adaptive threshold based on environment classification
//
// [REQ-ECCM-001] Distinguish jamming from clutter
// [REQ-ECCM-002] Maintain Pfa = 1e-6 in all conditions
// [REQ-ECCM-003] Latency < 1 ms per range-Doppler cell
//-----------------------------------------------------------------------------

`timescale 1ns / 1ps

module ml_cfar_engine #(
    parameter int RANGE_BINS     = 4096,
    parameter int DOPPLER_BINS   = 512,
    parameter int DATA_WIDTH     = 32,
    parameter int GUARD_CELLS    = 4,
    parameter int REF_CELLS      = 16,
    parameter int FEATURE_WIDTH  = 16,
    parameter int NUM_FEATURES   = 8
)(
    input  logic        clk,
    input  logic        rst_n,
    
    //-------------------------------------------------------------------------
    // Range-Doppler Map Input (magnitude squared)
    //-------------------------------------------------------------------------
    input  logic [DATA_WIDTH-1:0]  rd_map_data,
    input  logic [11:0]            rd_map_range_idx,
    input  logic [8:0]             rd_map_doppler_idx,
    input  logic                   rd_map_valid,
    output logic                   rd_map_ready,
    
    //-------------------------------------------------------------------------
    // Detection Output
    //-------------------------------------------------------------------------
    output logic [11:0]            det_range_idx,
    output logic [8:0]             det_doppler_idx,
    output logic [DATA_WIDTH-1:0]  det_magnitude,
    output logic [7:0]             det_snr,           // SNR estimate (dB × 4)
    output logic [1:0]             det_class,         // 0=target, 1=clutter, 2=jammer, 3=unknown
    output logic                   det_valid,
    input  logic                   det_ready,
    
    //-------------------------------------------------------------------------
    // ML Features Output (for external training/logging)
    //-------------------------------------------------------------------------
    output logic [FEATURE_WIDTH*NUM_FEATURES-1:0] ml_features,
    output logic                   ml_features_valid,
    
    //-------------------------------------------------------------------------
    // Jamming Metrics (to ECCM controller)
    //-------------------------------------------------------------------------
    output logic [DATA_WIDTH-1:0]  jam_power_estimate,
    output logic [7:0]             jam_duty_cycle,     // 0-100%
    output logic                   jam_detected,
    output logic [1:0]             jam_type,           // 0=none, 1=barrage, 2=spot, 3=sweep
    
    //-------------------------------------------------------------------------
    // Configuration
    //-------------------------------------------------------------------------
    input  logic [DATA_WIDTH-1:0]  cfg_pfa_threshold,  // Base threshold for Pfa
    input  logic [7:0]             cfg_alpha_clutter,  // Multiplier for clutter (Q4.4)
    input  logic [7:0]             cfg_alpha_jam,      // Multiplier for jamming (Q4.4)
    input  logic                   cfg_ml_enable,      // Enable ML classification
    input  logic [7:0]             cfg_jam_threshold,  // J/N threshold for jam detect
    
    //-------------------------------------------------------------------------
    // ML Model Weights (loadable)
    //-------------------------------------------------------------------------
    input  logic [15:0]            ml_weight_addr,
    input  logic [31:0]            ml_weight_data,
    input  logic                   ml_weight_we,
    
    //-------------------------------------------------------------------------
    // Status
    //-------------------------------------------------------------------------
    output logic [31:0]            total_detections,
    output logic [31:0]            jam_detections,
    output logic [31:0]            clutter_detections
);

    //-------------------------------------------------------------------------
    // Classification Codes
    //-------------------------------------------------------------------------
    localparam logic [1:0] CLASS_TARGET  = 2'b00;
    localparam logic [1:0] CLASS_CLUTTER = 2'b01;
    localparam logic [1:0] CLASS_JAMMER  = 2'b10;
    localparam logic [1:0] CLASS_UNKNOWN = 2'b11;
    
    //-------------------------------------------------------------------------
    // CFAR Window Buffer (reference cells + guard cells + CUT)
    //-------------------------------------------------------------------------
    localparam int WINDOW_SIZE = 2 * (GUARD_CELLS + REF_CELLS) + 1;
    
    logic [DATA_WIDTH-1:0] window_buffer [0:WINDOW_SIZE-1];
    logic [5:0]            window_idx;
    logic                  window_full;
    
    //-------------------------------------------------------------------------
    // State Machine
    //-------------------------------------------------------------------------
    typedef enum logic [3:0] {
        ST_IDLE,
        ST_FILL_WINDOW,
        ST_COMPUTE_STATS,
        ST_EXTRACT_FEATURES,
        ST_ML_INFERENCE,
        ST_THRESHOLD,
        ST_OUTPUT,
        ST_NEXT_CELL
    } state_t;
    
    state_t state, next_state;
    
    //-------------------------------------------------------------------------
    // Statistics Accumulators
    //-------------------------------------------------------------------------
    logic [DATA_WIDTH+7:0]  sum_leading;      // Sum of leading ref cells
    logic [DATA_WIDTH+7:0]  sum_trailing;     // Sum of trailing ref cells
    logic [DATA_WIDTH+7:0]  sum_total;        // Combined sum
    logic [DATA_WIDTH-1:0]  mean_noise;       // Estimated noise floor
    logic [DATA_WIDTH-1:0]  cell_under_test;  // CUT magnitude
    
    // Higher-order statistics for ML
    logic [63:0]            sum_squared;      // For variance
    logic [DATA_WIDTH-1:0]  variance;
    logic [DATA_WIDTH-1:0]  max_in_window;
    logic [DATA_WIDTH-1:0]  min_in_window;
    
    //-------------------------------------------------------------------------
    // ML Feature Vector
    //-------------------------------------------------------------------------
    // Feature 0: Mean noise level
    // Feature 1: Variance (normalized)
    // Feature 2: Peak-to-mean ratio
    // Feature 3: Spectral flatness (kurtosis proxy)
    // Feature 4: Temporal correlation (from previous CPI)
    // Feature 5: Range derivative (gradient)
    // Feature 6: Doppler derivative
    // Feature 7: CUT / Mean ratio (SNR proxy)
    
    logic [FEATURE_WIDTH-1:0] features [0:NUM_FEATURES-1];
    
    //-------------------------------------------------------------------------
    // ML Inference Engine (Simple Decision Tree)
    //-------------------------------------------------------------------------
    // Implemented as configurable thresholds for FPGA efficiency
    // Full NN inference optional via soft core
    
    logic [15:0] ml_weights [0:63];  // Decision tree thresholds
    logic [1:0]  ml_class_result;
    logic        ml_inference_done;
    
    //-------------------------------------------------------------------------
    // Current Cell Position
    //-------------------------------------------------------------------------
    logic [11:0] current_range;
    logic [8:0]  current_doppler;
    
    //-------------------------------------------------------------------------
    // Window Buffer Management
    //-------------------------------------------------------------------------
    assign rd_map_ready = (state == ST_IDLE) || (state == ST_FILL_WINDOW);
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            window_idx  <= '0;
            window_full <= 1'b0;
            for (int i = 0; i < WINDOW_SIZE; i++)
                window_buffer[i] <= '0;
        end else if (state == ST_IDLE) begin
            window_idx  <= '0;
            window_full <= 1'b0;
        end else if (state == ST_FILL_WINDOW && rd_map_valid) begin
            window_buffer[window_idx] <= rd_map_data;
            window_idx <= window_idx + 1;
            if (window_idx >= WINDOW_SIZE - 1)
                window_full <= 1'b1;
        end else if (state == ST_NEXT_CELL && rd_map_valid) begin
            // Shift window
            for (int i = 0; i < WINDOW_SIZE - 1; i++)
                window_buffer[i] <= window_buffer[i + 1];
            window_buffer[WINDOW_SIZE - 1] <= rd_map_data;
        end
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
                if (rd_map_valid)
                    next_state = ST_FILL_WINDOW;
            end
            
            ST_FILL_WINDOW: begin
                if (window_full)
                    next_state = ST_COMPUTE_STATS;
            end
            
            ST_COMPUTE_STATS: begin
                next_state = ST_EXTRACT_FEATURES;
            end
            
            ST_EXTRACT_FEATURES: begin
                if (cfg_ml_enable)
                    next_state = ST_ML_INFERENCE;
                else
                    next_state = ST_THRESHOLD;
            end
            
            ST_ML_INFERENCE: begin
                if (ml_inference_done)
                    next_state = ST_THRESHOLD;
            end
            
            ST_THRESHOLD: begin
                next_state = ST_OUTPUT;
            end
            
            ST_OUTPUT: begin
                if (det_ready || !det_valid)
                    next_state = ST_NEXT_CELL;
            end
            
            ST_NEXT_CELL: begin
                if (rd_map_valid)
                    next_state = ST_COMPUTE_STATS;
                else if (current_range >= RANGE_BINS - 1)
                    next_state = ST_IDLE;  // Done with this Doppler row
            end
            
            default: next_state = ST_IDLE;
        endcase
    end
    
    //-------------------------------------------------------------------------
    // Statistics Computation
    //-------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sum_leading   <= '0;
            sum_trailing  <= '0;
            sum_total     <= '0;
            mean_noise    <= '0;
            cell_under_test <= '0;
            sum_squared   <= '0;
            variance      <= '0;
            max_in_window <= '0;
            min_in_window <= '1;  // Max value
        end else if (state == ST_COMPUTE_STATS) begin
            // CUT is at center of window
            cell_under_test <= window_buffer[GUARD_CELLS + REF_CELLS];
            
            // Sum leading reference cells (before guard cells)
            sum_leading <= '0;
            for (int i = 0; i < REF_CELLS; i++) begin
                sum_leading <= sum_leading + window_buffer[i];
            end
            
            // Sum trailing reference cells (after guard cells)
            sum_trailing <= '0;
            for (int i = 0; i < REF_CELLS; i++) begin
                sum_trailing <= sum_trailing + window_buffer[GUARD_CELLS + REF_CELLS + 1 + GUARD_CELLS + i];
            end
            
            // Total and mean
            sum_total  <= sum_leading + sum_trailing;
            mean_noise <= (sum_leading + sum_trailing) / (2 * REF_CELLS);
            
            // Compute variance (sum of squared differences)
            sum_squared <= '0;
            for (int i = 0; i < REF_CELLS; i++) begin
                automatic logic [DATA_WIDTH-1:0] diff_lead = 
                    (window_buffer[i] > mean_noise) ? 
                    (window_buffer[i] - mean_noise) : 
                    (mean_noise - window_buffer[i]);
                automatic logic [DATA_WIDTH-1:0] diff_trail = 
                    (window_buffer[GUARD_CELLS + REF_CELLS + 1 + GUARD_CELLS + i] > mean_noise) ?
                    (window_buffer[GUARD_CELLS + REF_CELLS + 1 + GUARD_CELLS + i] - mean_noise) :
                    (mean_noise - window_buffer[GUARD_CELLS + REF_CELLS + 1 + GUARD_CELLS + i]);
                sum_squared <= sum_squared + (diff_lead * diff_lead) + (diff_trail * diff_trail);
            end
            variance <= sum_squared[DATA_WIDTH+15:16] / (2 * REF_CELLS);
            
            // Min/Max
            max_in_window <= '0;
            min_in_window <= '1;
            for (int i = 0; i < WINDOW_SIZE; i++) begin
                if (window_buffer[i] > max_in_window)
                    max_in_window <= window_buffer[i];
                if (window_buffer[i] < min_in_window)
                    min_in_window <= window_buffer[i];
            end
        end
    end
    
    //-------------------------------------------------------------------------
    // Feature Extraction
    //-------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < NUM_FEATURES; i++)
                features[i] <= '0;
        end else if (state == ST_EXTRACT_FEATURES) begin
            // Feature 0: Mean noise level (normalized to 16 bits)
            features[0] <= mean_noise[DATA_WIDTH-1:DATA_WIDTH-FEATURE_WIDTH];
            
            // Feature 1: Variance (normalized)
            features[1] <= variance[DATA_WIDTH-1:DATA_WIDTH-FEATURE_WIDTH];
            
            // Feature 2: Peak-to-mean ratio
            if (mean_noise > 0)
                features[2] <= (max_in_window / mean_noise);
            else
                features[2] <= '1;
            
            // Feature 3: Dynamic range (max/min)
            if (min_in_window > 0)
                features[3] <= (max_in_window / min_in_window);
            else
                features[3] <= '1;
            
            // Feature 4: Coefficient of variation (std/mean)
            // Approximate sqrt(variance) / mean
            if (mean_noise > 0)
                features[4] <= variance / mean_noise;
            else
                features[4] <= '0;
            
            // Feature 5: CUT prominence (CUT - mean) / variance
            if (variance > 0)
                features[5] <= (cell_under_test > mean_noise) ?
                               ((cell_under_test - mean_noise) / variance) : '0;
            else
                features[5] <= '0;
            
            // Feature 6: Range bin index (location context)
            features[6] <= current_range[11:11-FEATURE_WIDTH+1];
            
            // Feature 7: SNR proxy (CUT / mean)
            if (mean_noise > 0)
                features[7] <= (cell_under_test / mean_noise);
            else
                features[7] <= '1;
        end
    end
    
    // Pack features for output
    generate
        for (genvar i = 0; i < NUM_FEATURES; i++) begin : gen_features
            assign ml_features[(i+1)*FEATURE_WIDTH-1:i*FEATURE_WIDTH] = features[i];
        end
    endgenerate
    
    //-------------------------------------------------------------------------
    // ML Inference (Decision Tree Implementation)
    //-------------------------------------------------------------------------
    // Simple decision tree for real-time classification:
    // IF variance > jam_threshold AND peak_ratio > X THEN jammer
    // ELSE IF variance > clutter_threshold THEN clutter
    // ELSE target
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ml_class_result   <= CLASS_UNKNOWN;
            ml_inference_done <= 1'b0;
        end else if (state == ST_ML_INFERENCE) begin
            // Decision tree logic
            if (features[1] > ml_weights[0] && features[2] > ml_weights[1]) begin
                // High variance + high peak ratio = likely jammer
                ml_class_result <= CLASS_JAMMER;
            end else if (features[4] > ml_weights[2]) begin
                // High CV = likely clutter
                ml_class_result <= CLASS_CLUTTER;
            end else if (features[7] > ml_weights[3]) begin
                // High SNR = likely target
                ml_class_result <= CLASS_TARGET;
            end else begin
                ml_class_result <= CLASS_UNKNOWN;
            end
            ml_inference_done <= 1'b1;
        end else begin
            ml_inference_done <= 1'b0;
        end
    end
    
    //-------------------------------------------------------------------------
    // Adaptive Threshold
    //-------------------------------------------------------------------------
    logic [DATA_WIDTH-1:0] adaptive_threshold;
    logic                  detection_flag;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            adaptive_threshold <= '0;
            detection_flag     <= 1'b0;
        end else if (state == ST_THRESHOLD) begin
            // Select alpha based on classification
            case (cfg_ml_enable ? ml_class_result : CLASS_UNKNOWN)
                CLASS_JAMMER:  adaptive_threshold <= (mean_noise * cfg_alpha_jam) >> 4;
                CLASS_CLUTTER: adaptive_threshold <= (mean_noise * cfg_alpha_clutter) >> 4;
                default:       adaptive_threshold <= (mean_noise * cfg_pfa_threshold[7:0]) >> 4;
            endcase
            
            // Detection decision
            detection_flag <= (cell_under_test > adaptive_threshold);
        end
    end
    
    //-------------------------------------------------------------------------
    // Output Generation
    //-------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            det_range_idx   <= '0;
            det_doppler_idx <= '0;
            det_magnitude   <= '0;
            det_snr         <= '0;
            det_class       <= CLASS_UNKNOWN;
            det_valid       <= 1'b0;
        end else if (state == ST_OUTPUT && detection_flag) begin
            det_range_idx   <= current_range;
            det_doppler_idx <= current_doppler;
            det_magnitude   <= cell_under_test;
            
            // SNR in dB × 4 (approximate: 10*log10(CUT/mean) ≈ 3.3 * log2(CUT/mean))
            if (mean_noise > 0 && cell_under_test > mean_noise)
                det_snr <= ((cell_under_test / mean_noise) > 255) ? 8'd255 : 
                           (cell_under_test / mean_noise);
            else
                det_snr <= 8'd0;
            
            det_class <= cfg_ml_enable ? ml_class_result : CLASS_TARGET;
            det_valid <= 1'b1;
        end else if (det_ready) begin
            det_valid <= 1'b0;
        end
    end
    
    //-------------------------------------------------------------------------
    // Jamming Metrics
    //-------------------------------------------------------------------------
    logic [31:0] jam_power_acc;
    logic [15:0] jam_cell_count;
    logic [15:0] total_cell_count;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            jam_power_estimate <= '0;
            jam_duty_cycle     <= '0;
            jam_detected       <= 1'b0;
            jam_type           <= 2'b00;
            jam_power_acc      <= '0;
            jam_cell_count     <= '0;
            total_cell_count   <= '0;
        end else if (state == ST_OUTPUT) begin
            total_cell_count <= total_cell_count + 1;
            
            if (ml_class_result == CLASS_JAMMER) begin
                jam_cell_count <= jam_cell_count + 1;
                jam_power_acc  <= jam_power_acc + cell_under_test;
            end
            
            // Update metrics periodically (e.g., every 1000 cells)
            if (total_cell_count >= 1000) begin
                jam_detected <= (jam_cell_count > 100);  // >10% jam cells
                jam_duty_cycle <= (jam_cell_count * 100) / total_cell_count;
                
                if (jam_cell_count > 0)
                    jam_power_estimate <= jam_power_acc / jam_cell_count;
                
                // Classify jam type
                if (jam_duty_cycle > 80)
                    jam_type <= 2'b01;  // Barrage (continuous)
                else if (jam_duty_cycle > 20)
                    jam_type <= 2'b10;  // Spot (partial)
                else
                    jam_type <= 2'b00;  // None or minimal
                
                // Reset accumulators
                jam_power_acc    <= '0;
                jam_cell_count   <= '0;
                total_cell_count <= '0;
            end
        end
    end
    
    //-------------------------------------------------------------------------
    // ML Weights Memory
    //-------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (ml_weight_we)
            ml_weights[ml_weight_addr[5:0]] <= ml_weight_data[15:0];
    end
    
    //-------------------------------------------------------------------------
    // Statistics Counters
    //-------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            total_detections   <= '0;
            jam_detections     <= '0;
            clutter_detections <= '0;
        end else if (state == ST_OUTPUT && detection_flag) begin
            total_detections <= total_detections + 1;
            
            case (ml_class_result)
                CLASS_JAMMER:  jam_detections <= jam_detections + 1;
                CLASS_CLUTTER: clutter_detections <= clutter_detections + 1;
                default: ;
            endcase
        end
    end
    
    //-------------------------------------------------------------------------
    // Cell Position Tracking
    //-------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_range   <= '0;
            current_doppler <= '0;
        end else if (state == ST_IDLE && rd_map_valid) begin
            current_range   <= rd_map_range_idx;
            current_doppler <= rd_map_doppler_idx;
        end else if (state == ST_NEXT_CELL && rd_map_valid) begin
            current_range <= current_range + 1;
        end
    end
    
    // ML features valid strobe
    assign ml_features_valid = (state == ST_EXTRACT_FEATURES);

endmodule
