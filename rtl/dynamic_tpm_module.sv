//==============================================================================
// NX-MIMOSA v2.0 — Dynamic Transition Probability Matrix (VS-IMM)
// [REQ-V2-TPM-01] Variable Structure IMM with dynamic persistence
// [REQ-V2-TPM-02] Confidence-based switching rate
// [REQ-V2-TPM-03] Model activation/deactivation
//==============================================================================
// Reference: Li & Jilkov "Survey of Maneuvering Target Tracking" IEEE TAES
// Author: Dr. Mladen Mešter / Nexellum d.o.o.
//==============================================================================

`timescale 1ns/1ps

module dynamic_tpm_module
    import nx_mimosa_pkg_v2::*;
(
    input  logic        clk,
    input  logic        rst_n,
    
    // Mode probabilities from IMM
    input  fp_t         mu [N_MODELS],
    input  logic        mu_valid,
    
    // Maneuver detection from Adaptive Q
    input  logic        maneuver_detected,
    
    // Innovation norms (for each model)
    input  fp_t         nis_per_model [N_MODELS],
    
    // Configuration
    input  fp_t         p_stay_base,               // Base stay probability
    input  logic        vs_imm_enable,             // Enable VS-IMM
    
    // Output: Dynamic TPM
    output fp_t         PI_dynamic [N_MODELS][N_MODELS],
    output logic [N_MODELS-1:0] model_active       // Which models are active
);

    //--------------------------------------------------------------------------
    // Internal Signals
    //--------------------------------------------------------------------------
    fp_t max_mu;
    logic [$clog2(N_MODELS)-1:0] dominant_model;
    fp_t p_stay_current;
    fp_t p_switch;
    
    // Confidence level
    typedef enum logic [1:0] {
        CONF_LOW    = 2'b00,  // max(μ) < 0.7
        CONF_MEDIUM = 2'b01,  // 0.7 ≤ max(μ) < 0.9
        CONF_HIGH   = 2'b10   // max(μ) ≥ 0.9
    } confidence_t;
    
    confidence_t confidence;
    
    //--------------------------------------------------------------------------
    // Find Maximum Mode Probability
    //--------------------------------------------------------------------------
    always_comb begin
        max_mu = mu[0];
        dominant_model = '0;
        
        for (int i = 1; i < N_MODELS; i++) begin
            if (mu[i] > max_mu) begin
                max_mu = mu[i];
                dominant_model = i[$clog2(N_MODELS)-1:0];
            end
        end
    end
    
    //--------------------------------------------------------------------------
    // Determine Confidence Level
    //--------------------------------------------------------------------------
    always_comb begin
        if (max_mu >= MU_HIGH_CONF)  // ≥ 0.9
            confidence = CONF_HIGH;
        else if (max_mu >= 32'h0000_B333)  // ≥ 0.7
            confidence = CONF_MEDIUM;
        else
            confidence = CONF_LOW;
    end
    
    //--------------------------------------------------------------------------
    // Dynamic Stay Probability
    //--------------------------------------------------------------------------
    // High confidence → high persistence (stay longer in current model)
    // Low confidence → allow more switching
    // Maneuver detected → force lower persistence for CV model
    //--------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p_stay_current <= P_STAY_MED;
        end else if (mu_valid && vs_imm_enable) begin
            case (confidence)
                CONF_HIGH: begin
                    // Very confident → high persistence
                    if (maneuver_detected && dominant_model == 0) begin
                        // CV dominant but maneuver detected → reduce CV persistence
                        p_stay_current <= P_STAY_LOW;  // 0.8
                    end else begin
                        p_stay_current <= P_STAY_HIGH; // 0.99
                    end
                end
                
                CONF_MEDIUM: begin
                    // Moderately confident
                    p_stay_current <= P_STAY_MED;  // 0.9
                end
                
                CONF_LOW: begin
                    // Low confidence → encourage exploration
                    p_stay_current <= P_STAY_LOW;  // 0.8
                end
                
                default: begin
                    p_stay_current <= p_stay_base;
                end
            endcase
        end else begin
            p_stay_current <= p_stay_base;
        end
    end
    
    //--------------------------------------------------------------------------
    // Build Dynamic TPM
    //--------------------------------------------------------------------------
    always_comb begin
        // Calculate switch probability
        // p_switch = (1 - p_stay) / (N_MODELS - 1)
        p_switch = fp_div(FP_ONE - p_stay_current, 
                          (N_MODELS - 1) << FRAC_BITS);
        
        // Build symmetric TPM
        for (int i = 0; i < N_MODELS; i++) begin
            for (int j = 0; j < N_MODELS; j++) begin
                if (i == j) begin
                    PI_dynamic[i][j] = p_stay_current;
                end else begin
                    PI_dynamic[i][j] = p_switch;
                end
            end
        end
        
        // VS-IMM: Asymmetric adjustments based on model performance
        if (vs_imm_enable) begin
            // If a model has very high NIS, reduce transitions TO that model
            for (int j = 0; j < N_MODELS; j++) begin
                if (nis_per_model[j] > CHI2_THRESHOLD) begin
                    // This model is performing poorly → reduce incoming transitions
                    for (int i = 0; i < N_MODELS; i++) begin
                        if (i != j) begin
                            PI_dynamic[i][j] = PI_dynamic[i][j] >>> 1;  // Halve
                            // Redistribute to diagonal
                            PI_dynamic[i][i] = PI_dynamic[i][i] + (PI_dynamic[i][j] >>> 1);
                        end
                    end
                end
            end
        end
    end
    
    //--------------------------------------------------------------------------
    // Model Activation (VS-IMM feature)
    //--------------------------------------------------------------------------
    // Deactivate models with very low probability for efficiency
    //--------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            model_active <= {N_MODELS{1'b1}};  // All active initially
        end else if (mu_valid && vs_imm_enable) begin
            for (int m = 0; m < N_MODELS; m++) begin
                // Deactivate if μ < 0.01 for extended period
                // For now, keep all active (full VS-IMM in future)
                model_active[m] <= (mu[m] > 32'h0000_028F);  // > 0.01
            end
        end else begin
            model_active <= {N_MODELS{1'b1}};
        end
    end

endmodule
