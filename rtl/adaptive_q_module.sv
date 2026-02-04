//==============================================================================
// NX-MIMOSA v2.0 — Adaptive Process Noise Module
// [REQ-V2-AQ-01] NIS (Normalized Innovation Squared) computation
// [REQ-V2-AQ-02] Dynamic Q scaling based on filter consistency
// [REQ-V2-AQ-03] Exponential smoothing for stability
//==============================================================================
// Reference: Bar-Shalom "Estimation with Applications to Tracking" Ch. 5.4
// Author: Dr. Mladen Mešter / Nexellum d.o.o.
//==============================================================================

`timescale 1ns/1ps

module adaptive_q_module
    import nx_mimosa_pkg_v2::*;
(
    input  logic        clk,
    input  logic        rst_n,
    
    // Inputs from Kalman filter
    input  logic        update_valid,
    input  fp_t         innovation [MEAS_DIM],     // y = z - H*xp
    input  fp_t         S_inv [MEAS_DIM][MEAS_DIM], // Innovation covariance inverse
    
    // Configuration
    input  fp_t         q_base,                    // Base process noise
    input  fp_t         chi2_thresh,               // Chi-squared threshold
    input  fp_t         alpha_smooth,              // Smoothing factor (0.1-0.3)
    
    // Output
    output fp_t         q_adapted,                 // Adapted process noise
    output fp_t         nis_value,                 // NIS for diagnostics
    output logic        maneuver_detected          // High NIS indicator
);

    //--------------------------------------------------------------------------
    // Internal Signals
    //--------------------------------------------------------------------------
    fp_t nis_raw;
    fp_t nis_smoothed;
    fp_t q_scale;
    fp_t q_scale_smoothed;
    
    // NIS history for averaging (improves stability)
    parameter int NIS_HISTORY_LEN = 8;
    fp_t nis_history [NIS_HISTORY_LEN];
    logic [$clog2(NIS_HISTORY_LEN)-1:0] nis_idx;
    
    //--------------------------------------------------------------------------
    // NIS Computation: NIS = y' * S^(-1) * y
    //--------------------------------------------------------------------------
    // For 2D measurement: NIS = y[0]²*S_inv[0][0] + 2*y[0]*y[1]*S_inv[0][1] + y[1]²*S_inv[1][1]
    //--------------------------------------------------------------------------
    always_comb begin
        fp_t term1, term2, term3;
        
        // y[0]² * S_inv[0][0]
        term1 = fp_mul(fp_mul(innovation[0], innovation[0]), S_inv[0][0]);
        
        // 2 * y[0] * y[1] * S_inv[0][1]
        term2 = fp_mul(fp_mul(innovation[0], innovation[1]), S_inv[0][1]) <<< 1;
        
        // y[1]² * S_inv[1][1]
        term3 = fp_mul(fp_mul(innovation[1], innovation[1]), S_inv[1][1]);
        
        nis_raw = term1 + term2 + term3;
    end
    
    //--------------------------------------------------------------------------
    // NIS Smoothing (Exponential Moving Average)
    //--------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            nis_smoothed <= FP_TWO;  // Initialize to expected value (MEAS_DIM)
            nis_idx <= '0;
            for (int i = 0; i < NIS_HISTORY_LEN; i++)
                nis_history[i] <= FP_TWO;
        end else if (update_valid) begin
            // Update history
            nis_history[nis_idx] <= nis_raw;
            nis_idx <= nis_idx + 1;
            
            // Exponential smoothing: nis_smooth = α*nis_raw + (1-α)*nis_smooth
            nis_smoothed <= fp_mul(alpha_smooth, nis_raw) + 
                           fp_mul(FP_ONE - alpha_smooth, nis_smoothed);
        end
    end
    
    //--------------------------------------------------------------------------
    // Q Scale Computation
    //--------------------------------------------------------------------------
    // If NIS > threshold: Q needs to increase (filter is too optimistic)
    // If NIS < threshold: Q can decrease (filter is conservative)
    //--------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            q_scale <= FP_ONE;
            q_scale_smoothed <= FP_ONE;
            maneuver_detected <= 1'b0;
        end else if (update_valid) begin
            // Determine scaling based on NIS
            if (nis_smoothed > chi2_thresh) begin
                // High NIS → increase Q
                q_scale <= Q_INCREASE_FACTOR;  // 1.5x
                maneuver_detected <= 1'b1;
            end else if (nis_smoothed > (chi2_thresh >>> 1)) begin
                // Medium NIS → slight increase
                q_scale <= 32'h0001_2000;  // 1.125x
                maneuver_detected <= 1'b0;
            end else begin
                // Low NIS → decrease Q slowly
                q_scale <= Q_DECREASE_FACTOR;  // 0.95x
                maneuver_detected <= 1'b0;
            end
            
            // Smooth the scale factor for stability
            q_scale_smoothed <= fp_mul(alpha_smooth, q_scale) +
                               fp_mul(FP_ONE - alpha_smooth, q_scale_smoothed);
        end
    end
    
    //--------------------------------------------------------------------------
    // Output Q Computation
    //--------------------------------------------------------------------------
    always_comb begin
        fp_t q_raw;
        q_raw = fp_mul(q_base, q_scale_smoothed);
        
        // Clamp to valid range
        q_adapted = fp_clamp(q_raw, 
                            fp_mul(q_base, Q_MIN_SCALE),  // 0.1 * q_base
                            fp_mul(q_base, Q_MAX_SCALE)); // 5.0 * q_base
    end
    
    assign nis_value = nis_smoothed;

endmodule
