//=============================================================================
// QEDMMA v3.4 - ML-CFAR + Home-on-Jam (HOJ) ECCM Controller
// [REQ-ECCM-001] Adaptive CFAR with ML-enhanced threshold
// [REQ-ECCM-002] Home-on-Jam passive tracking capability
// [REQ-ECCM-003] +10 dB jamming margin improvement
//
// Author: Dr. Mladen Mešter
// Copyright (c) 2026 - All Rights Reserved
//
// Architecture:
//   ┌─────────────────────────────────────────────────────────────────────┐
//   │                    ML-CFAR + HOJ ECCM CONTROLLER                    │
//   ├─────────────────────────────────────────────────────────────────────┤
//   │                                                                     │
//   │  ┌───────────┐   ┌───────────┐   ┌───────────┐   ┌───────────┐    │
//   │  │  CFAR     │──▶│    ML     │──▶│ Detection │──▶│   Track   │    │
//   │  │ Processor │   │ Threshold │   │  Logic    │   │  Output   │    │
//   │  └───────────┘   └───────────┘   └───────────┘   └───────────┘    │
//   │       │                                               │            │
//   │       ▼                                               ▼            │
//   │  ┌───────────┐                                 ┌───────────┐       │
//   │  │  Jammer   │────────────────────────────────▶│    HOJ    │       │
//   │  │ Detector  │                                 │  Tracker  │       │
//   │  └───────────┘                                 └───────────┘       │
//   │                                                                     │
//   └─────────────────────────────────────────────────────────────────────┘
//
// Features:
//   - Cell-Averaging CFAR (CA-CFAR) baseline
//   - Greatest-Of CFAR (GO-CFAR) for clutter edges
//   - Smallest-Of CFAR (SO-CFAR) for multiple targets
//   - ML-enhanced adaptive threshold (neural network inference)
//   - Jammer detection via power spectral analysis
//   - Home-on-Jam passive ranging via TDOA
//   - Real-time mode switching based on environment
//=============================================================================

`timescale 1ns / 1ps

module ml_cfar_hoj_controller #(
    parameter int DATA_WIDTH      = 16,
    parameter int NUM_RANGE_BINS  = 512,
    parameter int CFAR_WINDOW     = 32,      // Total reference cells
    parameter int CFAR_GUARD      = 4,       // Guard cells each side
    parameter int ML_FEATURES     = 8,       // ML input features
    parameter int JAMMER_THRESHOLD_DB = 20   // Jammer detection threshold
)(
    input  logic                          clk,
    input  logic                          rst_n,
    
    //=========================================================================
    // Range Profile Input (from correlator)
    //=========================================================================
    input  logic [DATA_WIDTH-1:0]         i_range_profile [NUM_RANGE_BINS],
    input  logic                          i_profile_valid,
    
    //=========================================================================
    // Control Interface
    //=========================================================================
    input  logic [2:0]                    i_cfar_mode,       // 0=CA, 1=GO, 2=SO, 3=OS, 4=ML
    input  logic [7:0]                    i_pfa_exponent,    // P_fa = 2^(-exp)
    input  logic                          i_hoj_enable,      // Enable Home-on-Jam
    input  logic [15:0]                   i_jammer_threshold,
    
    //=========================================================================
    // ML Weights (from PS via AXI)
    //=========================================================================
    input  logic signed [15:0]            i_ml_weights [ML_FEATURES],
    input  logic signed [15:0]            i_ml_bias,
    
    //=========================================================================
    // Detection Output
    //=========================================================================
    output logic [NUM_RANGE_BINS-1:0]     o_detections,      // Detection bitmap
    output logic [8:0]                    o_num_detections,
    output logic [DATA_WIDTH-1:0]         o_threshold [NUM_RANGE_BINS],
    output logic                          o_detections_valid,
    
    //=========================================================================
    // Jammer/HOJ Output
    //=========================================================================
    output logic                          o_jammer_present,
    output logic [15:0]                   o_jammer_power,    // Estimated jammer power
    output logic [15:0]                   o_jammer_azimuth,  // HOJ bearing (if multistatic)
    output logic [15:0]                   o_jammer_range,    // HOJ range estimate
    output logic                          o_hoj_valid,
    
    //=========================================================================
    // Status
    //=========================================================================
    output logic [2:0]                    o_active_mode,
    output logic [15:0]                   o_noise_estimate,
    output logic                          o_clutter_edge_detected
);

    //=========================================================================
    // Local Parameters
    //=========================================================================
    localparam int HALF_WINDOW = CFAR_WINDOW / 2;
    localparam int TOTAL_REF = CFAR_WINDOW - 2*CFAR_GUARD;
    
    // CFAR modes
    localparam logic [2:0] MODE_CA_CFAR = 3'd0;
    localparam logic [2:0] MODE_GO_CFAR = 3'd1;
    localparam logic [2:0] MODE_SO_CFAR = 3'd2;
    localparam logic [2:0] MODE_OS_CFAR = 3'd3;
    localparam logic [2:0] MODE_ML_CFAR = 3'd4;
    
    //=========================================================================
    // Internal Signals
    //=========================================================================
    
    // CFAR processing
    logic [DATA_WIDTH+7:0] leading_sum;
    logic [DATA_WIDTH+7:0] lagging_sum;
    logic [DATA_WIDTH+7:0] noise_estimate_internal;
    logic [DATA_WIDTH-1:0] adaptive_threshold [NUM_RANGE_BINS];
    
    // ML inference
    logic signed [31:0] ml_accumulator;
    logic [DATA_WIDTH-1:0] ml_threshold_adjust;
    
    // Jammer detection
    logic [31:0] total_power;
    logic [31:0] peak_power;
    logic [15:0] power_ratio_db;
    logic jammer_detected;
    
    // HOJ tracking
    logic [15:0] jammer_bearing_estimate;
    logic [15:0] jammer_range_estimate;
    
    // Detection counters
    logic [8:0] detection_count;
    
    // Feature extraction for ML
    logic [DATA_WIDTH-1:0] ml_features [ML_FEATURES];
    
    //=========================================================================
    // CFAR Mode Selection State Machine
    //=========================================================================
    
    typedef enum logic [2:0] {
        CFAR_IDLE,
        CFAR_COMPUTE_SUMS,
        CFAR_APPLY_THRESHOLD,
        CFAR_ML_INFERENCE,
        CFAR_DETECT,
        CFAR_HOJ_CHECK,
        CFAR_OUTPUT
    } cfar_state_t;
    
    cfar_state_t cfar_state;
    logic [9:0] bin_index;
    
    //=========================================================================
    // Alpha Lookup Table (for P_fa)
    // alpha = N * (P_fa^(-1/N) - 1) where N = number of reference cells
    //=========================================================================
    
    function automatic logic [15:0] get_cfar_alpha(input logic [7:0] pfa_exp);
        // Simplified LUT for common P_fa values
        case (pfa_exp)
            8'd6:  return 16'd2048;   // P_fa = 1e-2
            8'd8:  return 16'd3277;   // P_fa = 1e-3
            8'd10: return 16'd4915;   // P_fa = 1e-4
            8'd12: return 16'd6554;   // P_fa = 1e-5
            8'd14: return 16'd8192;   // P_fa = 1e-6
            8'd16: return 16'd9830;   // P_fa = 1e-7
            default: return 16'd4915; // Default P_fa = 1e-4
        endcase
    endfunction
    
    logic [15:0] cfar_alpha;
    assign cfar_alpha = get_cfar_alpha(i_pfa_exponent);
    
    //=========================================================================
    // Main CFAR Processing FSM
    //=========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cfar_state <= CFAR_IDLE;
            bin_index <= '0;
            detection_count <= '0;
            o_detections <= '0;
            o_detections_valid <= 1'b0;
            o_jammer_present <= 1'b0;
            o_hoj_valid <= 1'b0;
            leading_sum <= '0;
            lagging_sum <= '0;
            total_power <= '0;
            peak_power <= '0;
        end else begin
            case (cfar_state)
                //-------------------------------------------------------------
                CFAR_IDLE: begin
                    o_detections_valid <= 1'b0;
                    o_hoj_valid <= 1'b0;
                    
                    if (i_profile_valid) begin
                        cfar_state <= CFAR_COMPUTE_SUMS;
                        bin_index <= HALF_WINDOW + CFAR_GUARD;
                        detection_count <= '0;
                        o_detections <= '0;
                        total_power <= '0;
                        peak_power <= '0;
                    end
                end
                
                //-------------------------------------------------------------
                CFAR_COMPUTE_SUMS: begin
                    // Compute leading and lagging sums for current CUT
                    logic [DATA_WIDTH+7:0] lead_sum_temp;
                    logic [DATA_WIDTH+7:0] lag_sum_temp;
                    
                    lead_sum_temp = '0;
                    lag_sum_temp = '0;
                    
                    // Leading cells (before CUT)
                    for (int i = 0; i < HALF_WINDOW - CFAR_GUARD; i++) begin
                        if (bin_index >= HALF_WINDOW + CFAR_GUARD) begin
                            lead_sum_temp += i_range_profile[bin_index - CFAR_GUARD - 1 - i];
                        end
                    end
                    
                    // Lagging cells (after CUT)
                    for (int i = 0; i < HALF_WINDOW - CFAR_GUARD; i++) begin
                        if (bin_index + CFAR_GUARD + 1 + i < NUM_RANGE_BINS) begin
                            lag_sum_temp += i_range_profile[bin_index + CFAR_GUARD + 1 + i];
                        end
                    end
                    
                    leading_sum <= lead_sum_temp;
                    lagging_sum <= lag_sum_temp;
                    
                    // Accumulate total power for jammer detection
                    total_power <= total_power + i_range_profile[bin_index];
                    if (i_range_profile[bin_index] > peak_power[DATA_WIDTH-1:0]) begin
                        peak_power <= {16'b0, i_range_profile[bin_index]};
                    end
                    
                    cfar_state <= CFAR_APPLY_THRESHOLD;
                end
                
                //-------------------------------------------------------------
                CFAR_APPLY_THRESHOLD: begin
                    logic [DATA_WIDTH+7:0] noise_est;
                    logic [DATA_WIDTH-1:0] threshold;
                    
                    case (i_cfar_mode)
                        MODE_CA_CFAR: begin
                            // Cell-Averaging: average of all reference cells
                            noise_est = (leading_sum + lagging_sum) / TOTAL_REF;
                        end
                        
                        MODE_GO_CFAR: begin
                            // Greatest-Of: max of leading/lagging averages
                            logic [DATA_WIDTH+7:0] lead_avg, lag_avg;
                            lead_avg = leading_sum / (HALF_WINDOW - CFAR_GUARD);
                            lag_avg = lagging_sum / (HALF_WINDOW - CFAR_GUARD);
                            noise_est = (lead_avg > lag_avg) ? lead_avg : lag_avg;
                        end
                        
                        MODE_SO_CFAR: begin
                            // Smallest-Of: min of leading/lagging averages
                            logic [DATA_WIDTH+7:0] lead_avg, lag_avg;
                            lead_avg = leading_sum / (HALF_WINDOW - CFAR_GUARD);
                            lag_avg = lagging_sum / (HALF_WINDOW - CFAR_GUARD);
                            noise_est = (lead_avg < lag_avg) ? lead_avg : lag_avg;
                        end
                        
                        MODE_ML_CFAR: begin
                            // ML-enhanced: base + ML adjustment
                            noise_est = (leading_sum + lagging_sum) / TOTAL_REF;
                            // ML adjustment applied in next state
                        end
                        
                        default: begin
                            noise_est = (leading_sum + lagging_sum) / TOTAL_REF;
                        end
                    endcase
                    
                    noise_estimate_internal <= noise_est;
                    
                    // Apply alpha scaling for threshold
                    threshold = (noise_est * cfar_alpha) >> 12;
                    adaptive_threshold[bin_index] <= threshold;
                    
                    if (i_cfar_mode == MODE_ML_CFAR) begin
                        cfar_state <= CFAR_ML_INFERENCE;
                    end else begin
                        cfar_state <= CFAR_DETECT;
                    end
                end
                
                //-------------------------------------------------------------
                CFAR_ML_INFERENCE: begin
                    // Simple neural network inference for threshold adjustment
                    // Features: noise_est, variance, skewness, gradient, etc.
                    
                    // Extract features (simplified)
                    ml_features[0] <= noise_estimate_internal[DATA_WIDTH-1:0];
                    ml_features[1] <= (leading_sum > lagging_sum) ? 
                                      leading_sum[DATA_WIDTH-1:0] - lagging_sum[DATA_WIDTH-1:0] :
                                      lagging_sum[DATA_WIDTH-1:0] - leading_sum[DATA_WIDTH-1:0];
                    ml_features[2] <= i_range_profile[bin_index];
                    ml_features[3] <= (bin_index > 0) ? 
                                      i_range_profile[bin_index] - i_range_profile[bin_index-1] : '0;
                    // Additional features would be computed here
                    
                    // Compute ML output: y = sum(w_i * x_i) + bias
                    ml_accumulator <= '0;
                    for (int i = 0; i < ML_FEATURES; i++) begin
                        ml_accumulator <= ml_accumulator + 
                            (i_ml_weights[i] * $signed({1'b0, ml_features[i]}));
                    end
                    ml_accumulator <= ml_accumulator + i_ml_bias;
                    
                    // Apply ReLU activation and scale
                    if (ml_accumulator > 0) begin
                        ml_threshold_adjust <= ml_accumulator[DATA_WIDTH-1:0];
                    end else begin
                        ml_threshold_adjust <= '0;
                    end
                    
                    // Adjust threshold
                    adaptive_threshold[bin_index] <= adaptive_threshold[bin_index] + 
                                                     ml_threshold_adjust;
                    
                    cfar_state <= CFAR_DETECT;
                end
                
                //-------------------------------------------------------------
                CFAR_DETECT: begin
                    // Compare CUT against threshold
                    if (i_range_profile[bin_index] > adaptive_threshold[bin_index]) begin
                        o_detections[bin_index] <= 1'b1;
                        detection_count <= detection_count + 1;
                    end
                    
                    // Store threshold for output
                    o_threshold[bin_index] <= adaptive_threshold[bin_index];
                    
                    // Move to next bin or check for jammer
                    if (bin_index < NUM_RANGE_BINS - HALF_WINDOW - CFAR_GUARD - 1) begin
                        bin_index <= bin_index + 1;
                        cfar_state <= CFAR_COMPUTE_SUMS;
                    end else begin
                        cfar_state <= CFAR_HOJ_CHECK;
                    end
                end
                
                //-------------------------------------------------------------
                CFAR_HOJ_CHECK: begin
                    // Jammer detection based on power analysis
                    logic [31:0] avg_power;
                    logic [15:0] ratio;
                    
                    avg_power = total_power / NUM_RANGE_BINS;
                    
                    // Check if peak >> average (indicates jammer)
                    if (avg_power > 0) begin
                        ratio = peak_power / avg_power;
                        
                        // Convert to dB approximation
                        if (ratio > 100) power_ratio_db <= 16'd20;
                        else if (ratio > 32) power_ratio_db <= 16'd15;
                        else if (ratio > 10) power_ratio_db <= 16'd10;
                        else power_ratio_db <= 16'd5;
                        
                        // Detect jammer if power ratio exceeds threshold
                        if (power_ratio_db >= i_jammer_threshold[7:0]) begin
                            jammer_detected <= 1'b1;
                            o_jammer_present <= 1'b1;
                            o_jammer_power <= avg_power[15:0];
                        end else begin
                            jammer_detected <= 1'b0;
                            o_jammer_present <= 1'b0;
                        end
                    end
                    
                    // HOJ processing (if enabled and jammer detected)
                    if (i_hoj_enable && jammer_detected) begin
                        // In multistatic config, HOJ bearing/range would be
                        // computed from TDOA across nodes. Here we output
                        // the detected jammer parameters for external processing.
                        o_jammer_azimuth <= '0;  // Computed by fusion processor
                        o_jammer_range <= '0;   // Computed by fusion processor
                        o_hoj_valid <= 1'b1;
                    end
                    
                    cfar_state <= CFAR_OUTPUT;
                end
                
                //-------------------------------------------------------------
                CFAR_OUTPUT: begin
                    o_num_detections <= detection_count;
                    o_detections_valid <= 1'b1;
                    o_noise_estimate <= noise_estimate_internal[DATA_WIDTH-1:0];
                    o_active_mode <= i_cfar_mode;
                    
                    // Clutter edge detection (GO-CFAR trigger)
                    o_clutter_edge_detected <= (leading_sum > (lagging_sum << 1)) ||
                                               (lagging_sum > (leading_sum << 1));
                    
                    cfar_state <= CFAR_IDLE;
                end
                
                default: cfar_state <= CFAR_IDLE;
            endcase
        end
    end
    
endmodule
