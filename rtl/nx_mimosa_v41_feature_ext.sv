//==============================================================================
// NX-MIMOSA v4.1 — Feature Extractor with Micro-Doppler & ECM Detection
//==============================================================================
// Expanded feature set for multi-domain classification (111 platforms):
//   FROM IMM:  speed, omega, NIS, mode probabilities
//   FROM RF:   RCS amplitude, SNR, Doppler spectrum, micro-Doppler flags
//   DERIVED:   ECM indicators, classification features, threat flags
//
// [REQ-FE-001] Kinematic features (speed, omega, g-load) from IMM state
// [REQ-FE-002] RF observable features (RCS, SNR, Doppler) from front-end
// [REQ-FE-003] Micro-Doppler classification (flapping, rotor, jet, static)
// [REQ-FE-004] ECM detection flags (SNR drop, RCS anomaly, Doppler spread)
// [REQ-FE-005] Feature output to PS via AXI-Lite for Python classifier
//
// Target: Xilinx RFSoC ZU48DR @ 250MHz
// Author: Dr. Mladen Mešter / Nexellum d.o.o.
//==============================================================================

`timescale 1ns/1ps

module nx_mimosa_v41_feature_ext
    import nx_mimosa_v40_pkg::*;
(
    input  logic                   clk,
    input  logic                   rst_n,
    
    // === IMM tracker outputs ===
    input  fp_t                    x_mixed [STATE_DIM],   // Mixed state [x,y,vx,vy,ax,ay]
    input  fp_t                    mu [N_MODELS],          // Mode probabilities
    input  fp_t                    nis_cv,                 // CV model NIS
    input  fp_t                    nis_ca,                 // CA model NIS
    input  fp_t                    omega,                  // Turn rate estimate
    input  logic                   imm_valid,              // IMM output valid strobe
    
    // === RF front-end observables ===
    input  fp_t                    rcs_amplitude,           // RCS estimate (linear, m²)
    input  fp_t                    snr_db,                  // Signal-to-noise ratio (dB)
    input  fp_t                    doppler_center_hz,       // Doppler centroid
    input  fp_t                    doppler_spread_hz,       // Doppler bandwidth/spread
    input  fp_t                    micro_doppler_peak_hz,   // Strongest micro-Doppler line
    input  fp_t                    micro_doppler_period,    // Periodicity of μD (0=aperiodic)
    input  fp_t                    range_m,                 // Slant range (m)
    input  logic                   rf_valid,                // RF data valid strobe
    
    // === Feature output (to PS via AXI-Lite) ===
    output classifier_features_v41_t  features,
    output logic                      features_valid,
    
    // === ECM alert flags (directly to alert logic) ===
    output logic                   ecm_noise_flag,          // Noise jamming detected
    output logic                   ecm_deception_flag,      // Deception (RGPO/VGPO/DRFM)
    output logic                   ecm_chaff_flag,          // Chaff corridor detected
    output logic                   false_target_flag        // Bird/balloon/clutter
);

    //==========================================================================
    // [REQ-FE-001] Kinematic Feature Extraction (from IMM)
    //==========================================================================
    
    // EMA parameters
    localparam fp_t ALPHA_FAST = 32'h0000_2666;  // 0.15 (fast EMA, ~10 sample conv.)
    localparam fp_t ALPHA_SLOW = 32'h0000_0CCD;  // 0.05 (slow EMA, ~30 sample conv.)
    localparam fp_t ONE_FAST   = 32'h0000_D99A;  // 1.0 - 0.15
    localparam fp_t ONE_SLOW   = 32'h0000_F333;  // 1.0 - 0.05
    
    // Speed: alpha-max-beta-min ≈ sqrt(vx² + vy²), error < 4%
    fp_t spd_cur;
    always_comb begin
        fp_t avx, avy, spd_max, spd_min;
        avx = fp_abs(x_mixed[2]);
        avy = fp_abs(x_mixed[3]);
        spd_max = (avx > avy) ? avx : avy;
        spd_min = (avx > avy) ? avy : avx;
        spd_cur = spd_max + fp_mul(spd_min, 32'h0000_6666);  // max + 0.4*min
    end
    
    // G-load approximation from acceleration state: g ≈ sqrt(ax² + ay²) / 9.81
    fp_t g_load_cur;
    always_comb begin
        fp_t aax, aay, a_max, a_min, a_mag;
        aax = fp_abs(x_mixed[4]);
        aay = fp_abs(x_mixed[5]);
        a_max = (aax > aay) ? aax : aay;
        a_min = (aax > aay) ? aay : aax;
        a_mag = a_max + fp_mul(a_min, 32'h0000_6666);
        // Divide by gravity: 9.81 ≈ 0x0009_CF5C (Q15.16)
        g_load_cur = fp_div(a_mag, 32'h0009_CF5C);
    end
    
    // mu_CT combined = mu_CT+ + mu_CT-
    fp_t mu_ct_combined;
    assign mu_ct_combined = mu[MDL_CTP] + mu[MDL_CTN];
    
    // Heading from velocity: atan2 approximation (CORDIC-free)
    // Use simple octant-based approximation for heading change detection
    fp_t heading_octant;
    always_comb begin
        fp_t vx_s, vy_s;
        vx_s = x_mixed[2];
        vy_s = x_mixed[3];
        // Encode as 3-bit octant: enough for heading reversal detection
        if (vx_s >= 0 && vy_s >= 0)
            heading_octant = (vy_s > vx_s) ? 32'h0002_0000 : 32'h0001_0000;
        else if (vx_s < 0 && vy_s >= 0)
            heading_octant = (vy_s > fp_abs(vx_s)) ? 32'h0003_0000 : 32'h0004_0000;
        else if (vx_s < 0 && vy_s < 0)
            heading_octant = (fp_abs(vy_s) > fp_abs(vx_s)) ? 32'h0006_0000 : 32'h0005_0000;
        else
            heading_octant = (fp_abs(vy_s) > vx_s) ? 32'h0007_0000 : 32'h0008_0000;
    end
    
    //==========================================================================
    // [REQ-FE-003] Micro-Doppler Classification
    //==========================================================================
    // Types detected:
    //   0 = none/static      (wind turbine, ground vehicle)
    //   1 = flapping_wings   (birds — periodic, 2-15 Hz, amplitude modulated)
    //   2 = multi_rotor      (drones — periodic, 50-300 Hz, harmonic comb)
    //   3 = rotor_blade      (helicopter — periodic, 10-50 Hz, strong harmonics)
    //   4 = jet_turbine       (fighter/airliner — broadband, non-periodic)
    //   5 = propeller         (UAV/GA — periodic, 30-100 Hz)
    //   6 = plasma_noise      (hypersonic — broadband, very high spread)
    //   7 = unknown
    //==========================================================================
    
    localparam int MD_NONE         = 0;
    localparam int MD_FLAPPING     = 1;
    localparam int MD_MULTI_ROTOR  = 2;
    localparam int MD_ROTOR_BLADE  = 3;
    localparam int MD_JET_TURBINE  = 4;
    localparam int MD_PROPELLER    = 5;
    localparam int MD_PLASMA       = 6;
    localparam int MD_UNKNOWN      = 7;
    
    // Thresholds (Q15.16)
    localparam fp_t MD_FLAP_LO     = 32'h0002_0000;  //  2 Hz
    localparam fp_t MD_FLAP_HI     = 32'h000F_0000;  // 15 Hz
    localparam fp_t MD_ROTOR_LO    = 32'h000A_0000;  // 10 Hz
    localparam fp_t MD_ROTOR_HI    = 32'h0032_0000;  // 50 Hz
    localparam fp_t MD_MROTOR_LO   = 32'h0032_0000;  // 50 Hz
    localparam fp_t MD_MROTOR_HI   = 32'h012C_0000;  // 300 Hz
    localparam fp_t MD_PROP_LO     = 32'h001E_0000;  // 30 Hz
    localparam fp_t MD_PROP_HI     = 32'h0064_0000;  // 100 Hz
    localparam fp_t MD_SPREAD_JET  = 32'h00C8_0000;  // 200 Hz spread → jet
    localparam fp_t MD_SPREAD_PLSM = 32'h03E8_0000;  // 1000 Hz spread → plasma
    localparam fp_t PERIODIC_THRESH = 32'h0000_4000;  // 0.25 (periodicity > this = periodic)
    
    logic [2:0] micro_doppler_type_r;
    fp_t        md_confidence_r;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            micro_doppler_type_r <= MD_UNKNOWN;
            md_confidence_r      <= FP_ZERO;
        end else if (rf_valid) begin
            // Decision tree for micro-Doppler classification
            if (doppler_spread_hz > MD_SPREAD_PLSM) begin
                // Very high spread = plasma (hypersonic ionization)
                micro_doppler_type_r <= MD_PLASMA;
                md_confidence_r      <= 32'h0000_E000;  // 0.875
            end else if (doppler_spread_hz > MD_SPREAD_JET && micro_doppler_period < PERIODIC_THRESH) begin
                // High spread + aperiodic = jet turbine
                micro_doppler_type_r <= MD_JET_TURBINE;
                md_confidence_r      <= 32'h0000_C000;  // 0.75
            end else if (micro_doppler_period > PERIODIC_THRESH) begin
                // Periodic: classify by frequency
                if (micro_doppler_peak_hz >= MD_MROTOR_LO && micro_doppler_peak_hz <= MD_MROTOR_HI) begin
                    micro_doppler_type_r <= MD_MULTI_ROTOR;
                    md_confidence_r      <= 32'h0000_D000;  // 0.8125
                end else if (micro_doppler_peak_hz >= MD_PROP_LO && micro_doppler_peak_hz <= MD_PROP_HI) begin
                    micro_doppler_type_r <= MD_PROPELLER;
                    md_confidence_r      <= 32'h0000_B000;  // 0.6875
                end else if (micro_doppler_peak_hz >= MD_ROTOR_LO && micro_doppler_peak_hz <= MD_ROTOR_HI) begin
                    micro_doppler_type_r <= MD_ROTOR_BLADE;
                    md_confidence_r      <= 32'h0000_C000;  // 0.75
                end else if (micro_doppler_peak_hz >= MD_FLAP_LO && micro_doppler_peak_hz <= MD_FLAP_HI) begin
                    micro_doppler_type_r <= MD_FLAPPING;
                    md_confidence_r      <= 32'h0000_A000;  // 0.625
                end else begin
                    micro_doppler_type_r <= MD_UNKNOWN;
                    md_confidence_r      <= 32'h0000_4000;  // 0.25
                end
            end else if (doppler_spread_hz < 32'h0005_0000 && spd_cur < 32'h0005_0000) begin
                // Very low spread + stationary = ground/static
                micro_doppler_type_r <= MD_NONE;
                md_confidence_r      <= 32'h0000_E000;  // 0.875
            end else begin
                micro_doppler_type_r <= MD_UNKNOWN;
                md_confidence_r      <= 32'h0000_2000;  // 0.125
            end
        end
    end
    
    //==========================================================================
    // [REQ-FE-004] ECM Detection (hardware-accelerated)
    //==========================================================================
    // Monitors: SNR drop, RCS variance, Doppler spread explosion, NIS spikes
    // Feeds PS classifier with ECM flags for Q-scale adaptation
    //==========================================================================
    
    // SNR baseline tracking (slow EMA)
    fp_t snr_baseline;
    fp_t rcs_ema;
    fp_t rcs_var_ema;      // Running variance of RCS
    fp_t prev_rcs;
    fp_t doppler_spread_ema;
    
    // ECM thresholds
    localparam fp_t SNR_DROP_THRESH   = 32'h000A_0000;  // 10 dB drop from baseline
    localparam fp_t RCS_VAR_THRESH    = 32'h0005_0000;  // 5x nominal variance
    localparam fp_t DSPREAD_THRESH    = 32'h0064_0000;  // 100 Hz spread threshold
    localparam fp_t NIS_SPIKE_THRESH  = 32'h0032_0000;  // NIS > 50 = anomaly
    
    logic [15:0] baseline_count;
    logic baseline_established;
    
    // ECM score accumulator (0-8 range, maps to confidence)
    fp_t ecm_score;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            snr_baseline        <= 32'h0019_0000;  // 25 dB default
            rcs_ema             <= FP_ONE;
            rcs_var_ema         <= FP_ZERO;
            prev_rcs            <= FP_ONE;
            doppler_spread_ema  <= FP_ZERO;
            baseline_count      <= 16'h0;
            baseline_established <= 1'b0;
            ecm_score           <= FP_ZERO;
        end else if (rf_valid) begin
            baseline_count <= baseline_count + 1'b1;
            
            // Slow EMA for baseline (first 20 samples establish nominal)
            if (!baseline_established) begin
                snr_baseline <= fp_mul(ONE_SLOW, snr_baseline) + fp_mul(ALPHA_SLOW, snr_db);
                rcs_ema      <= fp_mul(ONE_SLOW, rcs_ema) + fp_mul(ALPHA_SLOW, rcs_amplitude);
                if (baseline_count >= 16'd20)
                    baseline_established <= 1'b1;
            end
            
            // RCS variance: |RCS - RCS_ema|
            rcs_var_ema <= fp_mul(ONE_FAST, rcs_var_ema) + 
                           fp_mul(ALPHA_FAST, fp_abs(rcs_amplitude - rcs_ema));
            
            // Doppler spread EMA
            doppler_spread_ema <= fp_mul(ONE_FAST, doppler_spread_ema) + 
                                  fp_mul(ALPHA_FAST, doppler_spread_hz);
            
            prev_rcs <= rcs_amplitude;
            
            // ECM score computation (combinatorial below, registered here)
            ecm_score <= FP_ZERO;  // Will be overridden by combinatorial
        end
    end
    
    // ECM score: combinatorial aggregation
    fp_t ecm_score_comb;
    always_comb begin
        ecm_score_comb = FP_ZERO;
        
        if (baseline_established) begin
            // SNR drop from baseline
            if ((snr_baseline - snr_db) > SNR_DROP_THRESH)
                ecm_score_comb = ecm_score_comb + 32'h0003_0000;  // +3.0
            
            // RCS variance anomaly (5x nominal)
            if (rcs_var_ema > fp_mul(RCS_VAR_THRESH, rcs_ema))
                ecm_score_comb = ecm_score_comb + 32'h0002_0000;  // +2.0
            
            // Doppler spread explosion
            if (doppler_spread_ema > DSPREAD_THRESH)
                ecm_score_comb = ecm_score_comb + 32'h0002_8000;  // +2.5
            
            // NIS spike (from IMM — indicates measurement corruption)
            if (nis_cv > NIS_SPIKE_THRESH)
                ecm_score_comb = ecm_score_comb + 32'h0001_8000;  // +1.5
        end
    end
    
    // ECM flag generation
    assign ecm_noise_flag     = baseline_established && 
                                 ((snr_baseline - snr_db) > SNR_DROP_THRESH) &&
                                 (doppler_spread_ema > DSPREAD_THRESH);
    
    assign ecm_deception_flag = baseline_established &&
                                 (rcs_var_ema > fp_mul(RCS_VAR_THRESH, rcs_ema));
    
    assign ecm_chaff_flag     = baseline_established &&
                                 (rcs_var_ema > fp_mul(32'h000A_0000, rcs_ema)) &&  // 10x
                                 (doppler_spread_ema > 32'h0032_0000);  // 50 Hz
    
    //==========================================================================
    // [REQ-FE-005] False Target Detection (birds/balloons/clutter)
    //==========================================================================
    // Birds:    flapping μD + slow speed + low altitude + erratic omega
    // Balloons: near-zero speed + very high altitude + isotropic RCS
    // Clutter:  stationary + high RCS variance
    //==========================================================================
    
    logic bird_flag, balloon_flag, clutter_flag;
    
    assign bird_flag    = (micro_doppler_type_r == MD_FLAPPING) &&
                          (spd_cur < 32'h0019_0000) &&    // < 25 m/s
                          (fp_abs(omega) > 32'h0000_8000); // omega > 0.5 rad/s
    
    assign balloon_flag = (spd_cur < 32'h000A_0000) &&    // < 10 m/s
                          (g_load_cur < 32'h0000_8000) &&  // < 0.5 g
                          (micro_doppler_type_r == MD_NONE || micro_doppler_type_r == MD_UNKNOWN);
    
    assign clutter_flag = (spd_cur < 32'h0005_0000) &&    // < 5 m/s
                          (rcs_var_ema > fp_mul(32'h0003_0000, rcs_ema)); // high var
    
    assign false_target_flag = bird_flag | balloon_flag | clutter_flag;
    
    //==========================================================================
    // Running Statistics (EMA-based)
    //==========================================================================
    
    fp_t  spd_ema_r;
    fp_t  omega_ema_r;
    fp_t  nis_cv_ema_r;
    fp_t  g_load_ema_r;
    fp_t  rcs_dbsm_ema_r;      // RCS in dBsm (log domain)
    fp_t  snr_ema_r;
    
    // Peaks (lifetime)
    fp_t  omega_peak_r;
    fp_t  nis_cv_peak_r;
    fp_t  mu_ct_peak_r;
    fp_t  mu_ca_peak_r;
    fp_t  g_load_peak_r;
    fp_t  spd_peak_r;
    
    // Heading reversal counter
    fp_t  prev_heading_octant;
    logic [7:0] heading_reversal_cnt;
    
    logic [15:0] sample_count;
    logic initialized;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spd_ema_r           <= FP_ZERO;
            omega_ema_r         <= FP_ZERO;
            nis_cv_ema_r        <= FP_ZERO;
            g_load_ema_r        <= FP_ZERO;
            rcs_dbsm_ema_r      <= FP_ZERO;
            snr_ema_r           <= FP_ZERO;
            omega_peak_r        <= FP_ZERO;
            nis_cv_peak_r       <= FP_ZERO;
            mu_ct_peak_r        <= FP_ZERO;
            mu_ca_peak_r        <= FP_ZERO;
            g_load_peak_r       <= FP_ZERO;
            spd_peak_r          <= FP_ZERO;
            prev_heading_octant <= FP_ZERO;
            heading_reversal_cnt <= 8'h0;
            sample_count        <= 16'h0;
            initialized         <= 1'b0;
            features_valid      <= 1'b0;
        end else if (imm_valid) begin
            sample_count <= sample_count + 1'b1;
            
            if (!initialized) begin
                spd_ema_r    <= spd_cur;
                omega_ema_r  <= fp_abs(omega);
                nis_cv_ema_r <= nis_cv;
                g_load_ema_r <= g_load_cur;
                snr_ema_r    <= snr_db;
                prev_heading_octant <= heading_octant;
                initialized  <= 1'b1;
            end else begin
                // Fast EMA for all features
                spd_ema_r    <= fp_mul(ONE_FAST, spd_ema_r)    + fp_mul(ALPHA_FAST, spd_cur);
                omega_ema_r  <= fp_mul(ONE_FAST, omega_ema_r)  + fp_mul(ALPHA_FAST, fp_abs(omega));
                nis_cv_ema_r <= fp_mul(ONE_FAST, nis_cv_ema_r) + fp_mul(ALPHA_FAST, nis_cv);
                g_load_ema_r <= fp_mul(ONE_FAST, g_load_ema_r) + fp_mul(ALPHA_FAST, g_load_cur);
                snr_ema_r    <= fp_mul(ONE_SLOW, snr_ema_r)    + fp_mul(ALPHA_SLOW, snr_db);
                
                // Heading reversal detection (octant change by ≥ 4 = reversal)
                if (fp_abs(heading_octant - prev_heading_octant) >= 32'h0004_0000)
                    heading_reversal_cnt <= heading_reversal_cnt + 1'b1;
                prev_heading_octant <= heading_octant;
            end
            
            // Peak tracking (lifetime)
            if (fp_abs(omega) > omega_peak_r)   omega_peak_r <= fp_abs(omega);
            if (nis_cv > nis_cv_peak_r)         nis_cv_peak_r <= nis_cv;
            if (mu_ct_combined > mu_ct_peak_r)  mu_ct_peak_r <= mu_ct_combined;
            if (mu[MDL_CA] > mu_ca_peak_r)      mu_ca_peak_r <= mu[MDL_CA];
            if (g_load_cur > g_load_peak_r)     g_load_peak_r <= g_load_cur;
            if (spd_cur > spd_peak_r)           spd_peak_r <= spd_cur;
            
            // Output features every 10 samples (100ms @ 10Hz tracker rate)
            if (sample_count[3:0] == 4'h9) begin
                // --- Kinematic features ---
                features.spd_avg            <= spd_ema_r;
                features.spd_peak           <= spd_peak_r;
                features.omega_peak         <= omega_peak_r;
                features.omega_avg          <= omega_ema_r;
                features.g_load_avg         <= g_load_ema_r;
                features.g_load_peak        <= g_load_peak_r;
                features.nis_cv_avg         <= nis_cv_ema_r;
                features.nis_cv_peak        <= nis_cv_peak_r;
                features.mu_ct_peak         <= mu_ct_peak_r;
                features.mu_ca_peak         <= mu_ca_peak_r;
                features.heading_reversals  <= {24'h0, heading_reversal_cnt};
                
                // --- RF observable features ---
                features.rcs_amplitude      <= rcs_ema;
                features.rcs_variance       <= rcs_var_ema;
                features.snr_db             <= snr_ema_r;
                features.doppler_spread     <= doppler_spread_ema;
                features.range_m            <= range_m;
                
                // --- Micro-Doppler features ---
                features.micro_doppler_type <= {29'h0, micro_doppler_type_r};
                features.micro_doppler_conf <= md_confidence_r;
                features.micro_doppler_freq <= micro_doppler_peak_hz;
                
                // --- Derived flags ---
                features.is_maneuvering     <= (mu_ct_peak_r > 32'h0000_8000) |
                                               (omega_peak_r > 32'h0000_199A) |
                                               (nis_cv_peak_r > 32'h0032_0000);
                
                features.is_high_dynamics   <= (mu_ct_peak_r > 32'h0000_CCCD) |
                                               (omega_peak_r > 32'h0000_4CCD) |
                                               (nis_cv_peak_r > 32'h01F4_0000);
                
                features.ecm_detected       <= ecm_noise_flag | ecm_deception_flag | ecm_chaff_flag;
                features.ecm_score          <= ecm_score_comb;
                features.false_target       <= false_target_flag;
                features.bird_flag          <= bird_flag;
                features.balloon_flag       <= balloon_flag;
                
                features_valid <= 1'b1;
            end else begin
                features_valid <= 1'b0;
            end
        end else begin
            features_valid <= 1'b0;
        end
    end

endmodule
