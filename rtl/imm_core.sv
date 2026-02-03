//==============================================================================
// QEDMMA v3.1 Pro — IMM (Interacting Multiple Model) Core
// [REQ-RTL-IMM-01] 3-model mixing (CV, CT+, CT-)
// [REQ-RTL-IMM-02] Mode probability update
// [REQ-RTL-IMM-03] Parallel per-model Kalman filters
//==============================================================================

`timescale 1ns/1ps

module imm_core
    import qedmma_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,
    
    // Control
    input  logic        meas_valid,
    input  logic        init,
    output logic        done,
    
    // Measurement
    input  fp_t         z [MEAS_DIM],
    
    // Configuration
    input  fp_t         omega,              // Turn rate for CT models
    input  fp_t         dt,                 // Time step
    input  fp_t         q_cv,               // Process noise CV
    input  fp_t         q_ct,               // Process noise CT
    input  fp_t         r,                  // Measurement noise variance
    input  fp_t         p_stay,             // Stay probability (0.85-0.95)
    
    // Initial state
    input  fp_t         x_init [STATE_DIM],
    input  fp_t         P_init [STATE_DIM][STATE_DIM],
    
    // Outputs
    output fp_t         x_filt [STATE_DIM],             // Combined filtered state
    output fp_t         x_filt_model [N_MODELS][STATE_DIM],  // Per-model filtered
    output fp_t         P_filt_model [N_MODELS][STATE_DIM][STATE_DIM],
    output fp_t         x_pred_model [N_MODELS][STATE_DIM],  // Per-model predicted (for smoother)
    output fp_t         P_pred_model [N_MODELS][STATE_DIM][STATE_DIM],
    output fp_t         mu [N_MODELS],                  // Mode probabilities
    output logic        filt_valid
);

    //--------------------------------------------------------------------------
    // Model Matrices
    //--------------------------------------------------------------------------
    fp_t F [N_MODELS][STATE_DIM][STATE_DIM];
    fp_t Q [N_MODELS][STATE_DIM][STATE_DIM];
    fp_t H [MEAS_DIM][STATE_DIM];
    fp_t R_mat [MEAS_DIM][MEAS_DIM];
    
    // Transition probability matrix
    fp_t PI [N_MODELS][N_MODELS];
    
    //--------------------------------------------------------------------------
    // Per-Model State
    //--------------------------------------------------------------------------
    fp_t x_model [N_MODELS][STATE_DIM];
    fp_t P_model [N_MODELS][STATE_DIM][STATE_DIM];
    fp_t likelihood [N_MODELS];
    
    // Mixed states for each model
    fp_t x_mixed [N_MODELS][STATE_DIM];
    fp_t P_mixed [N_MODELS][STATE_DIM][STATE_DIM];
    
    // Mixing weights
    fp_t mu_ij [N_MODELS][N_MODELS];  // μ(i|j)
    fp_t c_bar [N_MODELS];            // Normalization
    
    //--------------------------------------------------------------------------
    // Sin/Cos for CT Models
    //--------------------------------------------------------------------------
    fp_t sin_omega_dt, cos_omega_dt;
    fp_t sin_neg_omega_dt, cos_neg_omega_dt;
    
    sincos_lut sincos_pos (
        .clk(clk),
        .rst_n(rst_n),
        .angle(fp_mul(omega, dt)),
        .sin_out(sin_omega_dt),
        .cos_out(cos_omega_dt),
        .valid()
    );
    
    // Negative omega
    sincos_lut sincos_neg (
        .clk(clk),
        .rst_n(rst_n),
        .angle(-fp_mul(omega, dt)),
        .sin_out(sin_neg_omega_dt),
        .cos_out(cos_neg_omega_dt),
        .valid()
    );
    
    //--------------------------------------------------------------------------
    // Build F Matrices
    //--------------------------------------------------------------------------
    always_comb begin
        // CV Model: F[0]
        // [1, 0, dt, 0]
        // [0, 1, 0, dt]
        // [0, 0, 1,  0]
        // [0, 0, 0,  1]
        F[0][0][0] = FP_ONE;  F[0][0][1] = FP_ZERO; F[0][0][2] = dt;      F[0][0][3] = FP_ZERO;
        F[0][1][0] = FP_ZERO; F[0][1][1] = FP_ONE;  F[0][1][2] = FP_ZERO; F[0][1][3] = dt;
        F[0][2][0] = FP_ZERO; F[0][2][1] = FP_ZERO; F[0][2][2] = FP_ONE;  F[0][2][3] = FP_ZERO;
        F[0][3][0] = FP_ZERO; F[0][3][1] = FP_ZERO; F[0][3][2] = FP_ZERO; F[0][3][3] = FP_ONE;
        
        // CT+ Model: F[1] (positive omega - right turn)
        // [1,      0,      sin(ωdt)/ω,    -(1-cos(ωdt))/ω]
        // [0,      1,      (1-cos(ωdt))/ω, sin(ωdt)/ω    ]
        // [0,      0,      cos(ωdt),      -sin(ωdt)      ]
        // [0,      0,      sin(ωdt),       cos(ωdt)      ]
        begin
            fp_t sin_div_omega = (omega != 0) ? fp_div(sin_omega_dt, omega) : dt;
            fp_t one_minus_cos_div_omega = (omega != 0) ? 
                fp_div(FP_ONE - cos_omega_dt, omega) : FP_ZERO;
            
            F[1][0][0] = FP_ONE;  F[1][0][1] = FP_ZERO; 
            F[1][0][2] = sin_div_omega; F[1][0][3] = -one_minus_cos_div_omega;
            
            F[1][1][0] = FP_ZERO; F[1][1][1] = FP_ONE;
            F[1][1][2] = one_minus_cos_div_omega; F[1][1][3] = sin_div_omega;
            
            F[1][2][0] = FP_ZERO; F[1][2][1] = FP_ZERO;
            F[1][2][2] = cos_omega_dt; F[1][2][3] = -sin_omega_dt;
            
            F[1][3][0] = FP_ZERO; F[1][3][1] = FP_ZERO;
            F[1][3][2] = sin_omega_dt; F[1][3][3] = cos_omega_dt;
        end
        
        // CT- Model: F[2] (negative omega - left turn)
        begin
            fp_t sin_div_omega_neg = (omega != 0) ? fp_div(sin_neg_omega_dt, -omega) : dt;
            fp_t one_minus_cos_div_omega_neg = (omega != 0) ? 
                fp_div(FP_ONE - cos_neg_omega_dt, -omega) : FP_ZERO;
            
            F[2][0][0] = FP_ONE;  F[2][0][1] = FP_ZERO;
            F[2][0][2] = sin_div_omega_neg; F[2][0][3] = -one_minus_cos_div_omega_neg;
            
            F[2][1][0] = FP_ZERO; F[2][1][1] = FP_ONE;
            F[2][1][2] = one_minus_cos_div_omega_neg; F[2][1][3] = sin_div_omega_neg;
            
            F[2][2][0] = FP_ZERO; F[2][2][1] = FP_ZERO;
            F[2][2][2] = cos_neg_omega_dt; F[2][2][3] = -sin_neg_omega_dt;
            
            F[2][3][0] = FP_ZERO; F[2][3][1] = FP_ZERO;
            F[2][3][2] = sin_neg_omega_dt; F[2][3][3] = cos_neg_omega_dt;
        end
    end
    
    //--------------------------------------------------------------------------
    // Build Q, H, R Matrices
    //--------------------------------------------------------------------------
    always_comb begin
        // Q matrix (discretized continuous white noise)
        // q * [dt^4/4, 0, dt^3/2, 0; ...]
        fp_t dt2 = fp_mul(dt, dt);
        fp_t dt3 = fp_mul(dt2, dt);
        fp_t dt4 = fp_mul(dt3, dt);
        
        fp_t q_dt4_4 = fp_mul(q_cv, dt4) >>> 2;
        fp_t q_dt3_2 = fp_mul(q_cv, dt3) >>> 1;
        fp_t q_dt2 = fp_mul(q_cv, dt2);
        
        Q[0][0][0] = q_dt4_4; Q[0][0][1] = FP_ZERO; Q[0][0][2] = q_dt3_2; Q[0][0][3] = FP_ZERO;
        Q[0][1][0] = FP_ZERO; Q[0][1][1] = q_dt4_4; Q[0][1][2] = FP_ZERO; Q[0][1][3] = q_dt3_2;
        Q[0][2][0] = q_dt3_2; Q[0][2][1] = FP_ZERO; Q[0][2][2] = q_dt2;   Q[0][2][3] = FP_ZERO;
        Q[0][3][0] = FP_ZERO; Q[0][3][1] = q_dt3_2; Q[0][3][2] = FP_ZERO; Q[0][3][3] = q_dt2;
        
        // Q for CT models (higher process noise)
        fp_t qct_dt4_4 = fp_mul(q_ct, dt4) >>> 2;
        fp_t qct_dt3_2 = fp_mul(q_ct, dt3) >>> 1;
        fp_t qct_dt2 = fp_mul(q_ct, dt2);
        
        Q[1][0][0] = qct_dt4_4; Q[1][0][1] = FP_ZERO; Q[1][0][2] = qct_dt3_2; Q[1][0][3] = FP_ZERO;
        Q[1][1][0] = FP_ZERO; Q[1][1][1] = qct_dt4_4; Q[1][1][2] = FP_ZERO; Q[1][1][3] = qct_dt3_2;
        Q[1][2][0] = qct_dt3_2; Q[1][2][1] = FP_ZERO; Q[1][2][2] = qct_dt2;   Q[1][2][3] = FP_ZERO;
        Q[1][3][0] = FP_ZERO; Q[1][3][1] = qct_dt3_2; Q[1][3][2] = FP_ZERO; Q[1][3][3] = qct_dt2;
        
        Q[2] = Q[1];  // CT- same as CT+
        
        // H matrix: [1,0,0,0; 0,1,0,0]
        H[0][0] = FP_ONE;  H[0][1] = FP_ZERO; H[0][2] = FP_ZERO; H[0][3] = FP_ZERO;
        H[1][0] = FP_ZERO; H[1][1] = FP_ONE;  H[1][2] = FP_ZERO; H[1][3] = FP_ZERO;
        
        // R matrix: r^2 * I
        R_mat[0][0] = fp_mul(r, r); R_mat[0][1] = FP_ZERO;
        R_mat[1][0] = FP_ZERO;      R_mat[1][1] = fp_mul(r, r);
        
        // Transition probability matrix
        fp_t p_switch = fp_div(FP_ONE - p_stay, 32'h0002_0000);  // (1-p_stay)/2
        PI[0][0] = p_stay;   PI[0][1] = p_switch; PI[0][2] = p_switch;
        PI[1][0] = p_switch; PI[1][1] = p_stay;   PI[1][2] = p_switch;
        PI[2][0] = p_switch; PI[2][1] = p_switch; PI[2][2] = p_stay;
    end
    
    //--------------------------------------------------------------------------
    // FSM
    //--------------------------------------------------------------------------
    typedef enum logic [3:0] {
        S_IDLE,
        S_INIT,
        S_COMPUTE_CBAR,      // c_bar[j] = sum_i(PI[i][j] * mu[i])
        S_COMPUTE_MU_IJ,     // mu_ij[i][j] = PI[i][j] * mu[i] / c_bar[j]
        S_MIX_STATES,        // x_mixed[j] = sum_i(mu_ij[i][j] * x[i])
        S_MIX_COVS,          // P_mixed[j] = sum_i(mu_ij * (P[i] + (x[i]-x_mixed[j])*(...)^T))
        S_RUN_FILTERS,       // Run Kalman filters in parallel
        S_WAIT_FILTERS,
        S_UPDATE_MU,         // mu[j] = c_bar[j] * likelihood[j] / normalization
        S_COMBINE,           // x_filt = sum(mu[j] * x[j])
        S_DONE
    } state_t;
    
    state_t state, next_state;
    
    // Kalman filter instances control
    logic kf_start [N_MODELS];
    logic kf_init [N_MODELS];
    logic kf_done [N_MODELS];
    logic all_kf_done;
    
    assign all_kf_done = kf_done[0] && kf_done[1] && kf_done[2];
    
    //--------------------------------------------------------------------------
    // Kalman Filter Instances (one per model)
    //--------------------------------------------------------------------------
    generate
        for (genvar m = 0; m < N_MODELS; m++) begin : kf_gen
            kalman_filter_core kf_inst (
                .clk(clk),
                .rst_n(rst_n),
                .start(kf_start[m]),
                .init(kf_init[m]),
                .done(kf_done[m]),
                .z(z),
                .F(F[m]),
                .Q(Q[m]),
                .H(H),
                .R(R_mat),
                .x_init(x_mixed[m]),
                .P_init(P_mixed[m]),
                .x_filt(x_filt_model[m]),
                .P_filt(P_filt_model[m]),
                .x_pred(x_pred_model[m]),
                .P_pred(P_pred_model[m]),
                .likelihood(likelihood[m])
            );
        end
    endgenerate
    
    //--------------------------------------------------------------------------
    // FSM Sequential
    //--------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            done <= 1'b0;
            filt_valid <= 1'b0;
            
            // Initialize mode probabilities
            mu[0] <= 32'h0000_CCCD;  // 0.8
            mu[1] <= 32'h0000_199A;  // 0.1
            mu[2] <= 32'h0000_199A;  // 0.1
            
            for (int m = 0; m < N_MODELS; m++) begin
                kf_start[m] <= 1'b0;
                kf_init[m] <= 1'b0;
                for (int i = 0; i < STATE_DIM; i++) begin
                    x_model[m][i] <= '0;
                    x_filt[i] <= '0;
                end
            end
        end else begin
            state <= next_state;
            done <= 1'b0;
            filt_valid <= 1'b0;
            
            for (int m = 0; m < N_MODELS; m++) begin
                kf_start[m] <= 1'b0;
                kf_init[m] <= 1'b0;
            end
            
            case (state)
                S_IDLE: begin
                    // Wait for measurement
                end
                
                S_INIT: begin
                    // Initialize all models with same state
                    for (int m = 0; m < N_MODELS; m++) begin
                        x_model[m] <= x_init;
                        P_model[m] <= P_init;
                        x_mixed[m] <= x_init;
                        P_mixed[m] <= P_init;
                    end
                    mu[0] <= 32'h0000_CCCD;  // 0.8
                    mu[1] <= 32'h0000_199A;  // 0.1
                    mu[2] <= 32'h0000_199A;  // 0.1
                end
                
                //--------------------------------------------------------------
                // Mixing Step 1: c_bar[j] = sum_i(PI[i][j] * mu[i])
                //--------------------------------------------------------------
                S_COMPUTE_CBAR: begin
                    for (int j = 0; j < N_MODELS; j++) begin
                        fp_t sum;
                        sum = FP_ZERO;
                        for (int i = 0; i < N_MODELS; i++) begin
                            sum = sum + fp_mul(PI[i][j], mu[i]);
                        end
                        c_bar[j] <= sum;
                    end
                end
                
                //--------------------------------------------------------------
                // Mixing Step 2: mu_ij[i][j] = PI[i][j] * mu[i] / c_bar[j]
                //--------------------------------------------------------------
                S_COMPUTE_MU_IJ: begin
                    for (int i = 0; i < N_MODELS; i++) begin
                        for (int j = 0; j < N_MODELS; j++) begin
                            fp_t num = fp_mul(PI[i][j], mu[i]);
                            mu_ij[i][j] <= (c_bar[j] != 0) ? fp_div(num, c_bar[j]) : FP_ZERO;
                        end
                    end
                end
                
                //--------------------------------------------------------------
                // Mixing Step 3: x_mixed[j] = sum_i(mu_ij[i][j] * x_model[i])
                //--------------------------------------------------------------
                S_MIX_STATES: begin
                    for (int j = 0; j < N_MODELS; j++) begin
                        for (int s = 0; s < STATE_DIM; s++) begin
                            fp_t sum;
                            sum = FP_ZERO;
                            for (int i = 0; i < N_MODELS; i++) begin
                                sum = sum + fp_mul(mu_ij[i][j], x_model[i][s]);
                            end
                            x_mixed[j][s] <= sum;
                        end
                    end
                end
                
                //--------------------------------------------------------------
                // Mixing Step 4: P_mixed[j] = sum_i(mu_ij[i][j] * (P[i] + spread))
                //--------------------------------------------------------------
                S_MIX_COVS: begin
                    for (int j = 0; j < N_MODELS; j++) begin
                        for (int r_idx = 0; r_idx < STATE_DIM; r_idx++) begin
                            for (int c_idx = 0; c_idx < STATE_DIM; c_idx++) begin
                                fp_t sum;
                                sum = FP_ZERO;
                                for (int i = 0; i < N_MODELS; i++) begin
                                    // Spread term: (x[i] - x_mixed[j]) * (x[i] - x_mixed[j]).T
                                    fp_t dx_r = x_model[i][r_idx] - x_mixed[j][r_idx];
                                    fp_t dx_c = x_model[i][c_idx] - x_mixed[j][c_idx];
                                    fp_t spread = fp_mul(dx_r, dx_c);
                                    fp_t P_plus_spread = P_model[i][r_idx][c_idx] + spread;
                                    sum = sum + fp_mul(mu_ij[i][j], P_plus_spread);
                                end
                                P_mixed[j][r_idx][c_idx] <= sum;
                            end
                        end
                    end
                end
                
                //--------------------------------------------------------------
                // Run Kalman Filters
                //--------------------------------------------------------------
                S_RUN_FILTERS: begin
                    for (int m = 0; m < N_MODELS; m++) begin
                        kf_start[m] <= 1'b1;
                    end
                end
                
                S_WAIT_FILTERS: begin
                    // Wait for all filters to complete
                    if (all_kf_done) begin
                        // Copy results
                        for (int m = 0; m < N_MODELS; m++) begin
                            x_model[m] <= x_filt_model[m];
                            P_model[m] <= P_filt_model[m];
                        end
                    end
                end
                
                //--------------------------------------------------------------
                // Update Mode Probabilities
                //--------------------------------------------------------------
                S_UPDATE_MU: begin
                    // mu[j] = c_bar[j] * likelihood[j]
                    fp_t mu_unnorm [N_MODELS];
                    fp_t sum_mu;
                    
                    sum_mu = FP_ZERO;
                    for (int j = 0; j < N_MODELS; j++) begin
                        mu_unnorm[j] = fp_mul(c_bar[j], likelihood[j]);
                        sum_mu = sum_mu + mu_unnorm[j];
                    end
                    
                    // Normalize
                    for (int j = 0; j < N_MODELS; j++) begin
                        mu[j] <= (sum_mu != 0) ? fp_div(mu_unnorm[j], sum_mu) : 
                                 (j == 0) ? FP_ONE : FP_ZERO;
                    end
                end
                
                //--------------------------------------------------------------
                // Combine Estimates
                //--------------------------------------------------------------
                S_COMBINE: begin
                    for (int s = 0; s < STATE_DIM; s++) begin
                        fp_t sum;
                        sum = FP_ZERO;
                        for (int j = 0; j < N_MODELS; j++) begin
                            sum = sum + fp_mul(mu[j], x_filt_model[j][s]);
                        end
                        x_filt[s] <= sum;
                    end
                    filt_valid <= 1'b1;
                end
                
                S_DONE: begin
                    done <= 1'b1;
                end
            endcase
        end
    end
    
    //--------------------------------------------------------------------------
    // FSM Combinational
    //--------------------------------------------------------------------------
    always_comb begin
        next_state = state;
        case (state)
            S_IDLE:         if (meas_valid) next_state = init ? S_INIT : S_COMPUTE_CBAR;
            S_INIT:         next_state = S_DONE;
            S_COMPUTE_CBAR: next_state = S_COMPUTE_MU_IJ;
            S_COMPUTE_MU_IJ: next_state = S_MIX_STATES;
            S_MIX_STATES:   next_state = S_MIX_COVS;
            S_MIX_COVS:     next_state = S_RUN_FILTERS;
            S_RUN_FILTERS:  next_state = S_WAIT_FILTERS;
            S_WAIT_FILTERS: if (all_kf_done) next_state = S_UPDATE_MU;
            S_UPDATE_MU:    next_state = S_COMBINE;
            S_COMBINE:      next_state = S_DONE;
            S_DONE:         next_state = S_IDLE;
        endcase
    end

endmodule
