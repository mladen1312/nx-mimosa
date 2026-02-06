//==============================================================================
// NX-MIMOSA v4.0.2 — Feature Extractor for Platform Classifier
//==============================================================================
// Extracts kinematic and IMM dynamics features from forward filter output:
//   - Speed average (from mixed state velocity)
//   - Turn rate (omega) peak and average
//   - CV NIS peak and average
//   - Mode probability peaks (mu_CT, mu_CA)
//   - Maneuvering/high-dynamics flags
//
// Features sent to PS (ARM) via AXI-Lite for Python-based platform ID.
// [REQ-CLF-001] Hardware feature extraction for real-time classification
//==============================================================================

`timescale 1ns/1ps

module nx_mimosa_v40_feature_ext
    import nx_mimosa_v40_pkg::*;
(
    input  logic                   clk,
    input  logic                   rst_n,
    
    // IMM outputs
    input  fp_t                    x_mixed [STATE_DIM],
    input  fp_t                    mu [N_MODELS],
    input  fp_t                    nis_cv,
    input  fp_t                    nis_ca,
    input  fp_t                    omega,
    input  logic                   in_valid,
    
    // Feature output
    output classifier_features_t   features,
    output logic                   features_valid
);

    //--------------------------------------------------------------------------
    // Running statistics (EMA-based)
    //--------------------------------------------------------------------------
    localparam fp_t ALPHA     = 32'h0000_0CCD;  // 0.05 (slow EMA)
    localparam fp_t ONE_ALPHA = 32'h0000_F333;  // 0.95
    
    fp_t  spd_ema;
    fp_t  omega_ema;
    fp_t  nis_cv_ema;
    fp_t  omega_peak_r;
    fp_t  nis_cv_peak_r;
    fp_t  mu_ct_peak_r;
    fp_t  mu_ca_peak_r;
    
    logic [15:0] sample_count;
    logic initialized;
    
    // Speed computation: sqrt(vx² + vy²) ≈ max(|vx|,|vy|) + 0.4*min(|vx|,|vy|)
    // Alpha-max-beta-min approximation (good to ~4%)
    fp_t spd_cur;
    
    always_comb begin
        fp_t avx, avy, spd_max, spd_min;
        avx = fp_abs(x_mixed[2]);  // |vx|
        avy = fp_abs(x_mixed[3]);  // |vy|
        spd_max = (avx > avy) ? avx : avy;
        spd_min = (avx > avy) ? avy : avx;
        spd_cur = spd_max + fp_mul(spd_min, 32'h0000_6666);  // max + 0.4*min
    end
    
    // mu_CT combined = mu_CT+ + mu_CT-
    fp_t mu_ct_combined;
    assign mu_ct_combined = mu[MDL_CTP] + mu[MDL_CTN];
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spd_ema       <= FP_ZERO;
            omega_ema     <= FP_ZERO;
            nis_cv_ema    <= FP_ZERO;
            omega_peak_r  <= FP_ZERO;
            nis_cv_peak_r <= FP_ZERO;
            mu_ct_peak_r  <= FP_ZERO;
            mu_ca_peak_r  <= FP_ZERO;
            sample_count  <= 16'h0;
            initialized   <= 1'b0;
            features_valid <= 1'b0;
        end else if (in_valid) begin
            sample_count <= sample_count + 1'b1;
            
            if (!initialized) begin
                // First sample: initialize
                spd_ema    <= spd_cur;
                omega_ema  <= fp_abs(omega);
                nis_cv_ema <= nis_cv;
                initialized <= 1'b1;
            end else begin
                // EMA update
                spd_ema    <= fp_mul(ONE_ALPHA, spd_ema) + fp_mul(ALPHA, spd_cur);
                omega_ema  <= fp_mul(ONE_ALPHA, omega_ema) + fp_mul(ALPHA, fp_abs(omega));
                nis_cv_ema <= fp_mul(ONE_ALPHA, nis_cv_ema) + fp_mul(ALPHA, nis_cv);
            end
            
            // Peak tracking (lifetime)
            if (fp_abs(omega) > omega_peak_r)
                omega_peak_r <= fp_abs(omega);
            if (nis_cv > nis_cv_peak_r)
                nis_cv_peak_r <= nis_cv;
            if (mu_ct_combined > mu_ct_peak_r)
                mu_ct_peak_r <= mu_ct_combined;
            if (mu[MDL_CA] > mu_ca_peak_r)
                mu_ca_peak_r <= mu[MDL_CA];
            
            // Output features every 10 samples (avoid bus flooding)
            if (sample_count[3:0] == 4'h9) begin
                features.spd_avg      <= spd_ema;
                features.omega_peak   <= omega_peak_r;
                features.omega_avg    <= omega_ema;
                features.nis_cv_avg   <= nis_cv_ema;
                features.nis_cv_peak  <= nis_cv_peak_r;
                features.mu_ct_peak   <= mu_ct_peak_r;
                features.mu_ca_peak   <= mu_ca_peak_r;
                
                // Maneuvering flag: mu_CT_peak > 0.5 OR omega_peak > 0.1
                features.is_maneuvering <= (mu_ct_peak_r > 32'h0000_8000) |
                                           (omega_peak_r > 32'h0000_199A) |
                                           (nis_cv_peak_r > 32'h0032_0000);  // 50.0
                
                // High dynamics flag: mu_CT_peak > 0.8 OR omega_peak > 0.3
                features.is_high_dynamics <= (mu_ct_peak_r > 32'h0000_CCCD) |
                                             (omega_peak_r > 32'h0000_4CCD) |
                                             (nis_cv_peak_r > 32'h01F4_0000); // 500.0
                
                features_valid <= 1'b1;
            end else begin
                features_valid <= 1'b0;
            end
        end else begin
            features_valid <= 1'b0;
        end
    end

endmodule
