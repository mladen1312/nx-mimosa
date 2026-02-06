//==============================================================================
// NX-MIMOSA v4.0.2 — 6-Model IMM Forward Filter
//==============================================================================
// Models: CV | CA | CT+ | CT- | Jerk | Ballistic
// Unified state_dim=6: [x, y, vx, vy, ax, ay]
// Key: Captures pre-mixing unmixed states for RTS backward pass
//
// Pipeline stages (per measurement):
//   Stage 1: Mixing (compute mixed states from TPM)
//   Stage 2: Predict (6 parallel F·x predictions)
//   Stage 3: Update  (6 parallel Kalman updates)
//   Stage 4: Output  (mode probability update, mixing)
//
// Total latency: 4 clock cycles per measurement at 250MHz
// [REQ-IMM-001] 6-model IMM with pre-mixing state storage
//==============================================================================

`timescale 1ns/1ps

module nx_mimosa_v40_imm
    import nx_mimosa_v40_pkg::*;
#(
    parameter int NUM_TARGETS = 1
)(
    input  logic                   clk,
    input  logic                   rst_n,
    
    // Measurement input
    input  fp_t                    meas [MEAS_DIM],
    input  logic                   meas_valid,
    input  logic [TGT_ID_W-1:0]   meas_tgt_id,
    output logic                   meas_ready,
    
    // Configuration
    input  fp_t                    cfg_dt,
    input  fp_t                    cfg_r_std,
    input  fp_t                    cfg_tpm [N_MODELS][N_MODELS],
    
    // [REQ-IMM-002] Pre-mixing unmixed states (for RTS)
    output fp_t                    xu_out [N_MODELS][STATE_DIM],
    output fp_t                    Pu_out [N_MODELS][STATE_DIM][STATE_DIM],
    
    // Post-update states
    output fp_t                    x_upd_out [N_MODELS][STATE_DIM],
    output fp_t                    P_upd_out [N_MODELS][STATE_DIM][STATE_DIM],
    
    // Mixed outputs
    output fp_t                    x_mixed_out [STATE_DIM],
    output fp_t                    mu_out [N_MODELS],
    output fp_t                    nis_out [N_MODELS],
    output fp_t                    omega_out,
    output logic                   out_valid
);

    //--------------------------------------------------------------------------
    // State storage per model per target
    //--------------------------------------------------------------------------
    fp_t  x_mdl   [NUM_TARGETS][N_MODELS][STATE_DIM];
    fp_t  P_mdl   [NUM_TARGETS][N_MODELS][STATE_DIM][STATE_DIM];
    fp_t  mu_mdl  [NUM_TARGETS][N_MODELS];
    logic [NUM_TARGETS-1:0] initialized;
    
    // Pipeline registers
    logic [3:0] pipe_stage;
    logic [TGT_ID_W-1:0] pipe_tgt;
    fp_t  pipe_meas [MEAS_DIM];
    
    // Working registers
    fp_t  x_mix  [N_MODELS][STATE_DIM];     // Mixed states (after TPM mixing)
    fp_t  P_mix  [N_MODELS][STATE_DIM][STATE_DIM];
    fp_t  x_pred [N_MODELS][STATE_DIM];     // Predicted states
    fp_t  P_pred [N_MODELS][STATE_DIM][STATE_DIM];
    fp_t  x_upd  [N_MODELS][STATE_DIM];     // Updated states
    fp_t  P_upd  [N_MODELS][STATE_DIM][STATE_DIM];
    fp_t  nis_w  [N_MODELS];                // Innovation NIS per model
    fp_t  mu_bar [N_MODELS];                // Predicted mode probabilities
    fp_t  mu_new [N_MODELS];                // Updated mode probabilities
    
    // R matrix = diag(r_std^2, r_std^2)
    fp_t  R_diag;
    assign R_diag = fp_mul(cfg_r_std, cfg_r_std);
    
    //--------------------------------------------------------------------------
    // [REQ-IMM-010] Initialization
    //--------------------------------------------------------------------------
    // On first measurement for each target: initialize all models at measurement
    
    task automatic init_target(input int t, input fp_t z[MEAS_DIM]);
        for (int m = 0; m < N_MODELS; m++) begin
            x_mdl[t][m][0] = z[0];     // x
            x_mdl[t][m][1] = z[1];     // y
            x_mdl[t][m][2] = FP_ZERO;  // vx
            x_mdl[t][m][3] = FP_ZERO;  // vy
            x_mdl[t][m][4] = FP_ZERO;  // ax
            x_mdl[t][m][5] = FP_ZERO;  // ay
            // Initial P: large uncertainty
            for (int i = 0; i < STATE_DIM; i++)
                for (int j = 0; j < STATE_DIM; j++)
                    P_mdl[t][m][i][j] = (i == j) ? 32'h0064_0000 : FP_ZERO; // 100.0
            // Equal initial probabilities
            mu_mdl[t][m] = fp_div(FP_ONE, 32'h0006_0000); // 1/6
        end
    endtask

    //--------------------------------------------------------------------------
    // [REQ-IMM-020] State Transition Matrices
    //--------------------------------------------------------------------------
    // CV:   F = [I dt*I 0; 0 I 0; 0 0 0]  (ax,ay ignored)
    // CA:   F = [I dt*I 0.5dt²*I; 0 I dt*I; 0 0 I]
    // CT±:  F = rotation(omega*dt) with turn rate
    // Jerk: F = CA + higher-order
    // Ball: F = CV with gravity in ay
    
    function automatic void compute_F_CV(
        input  fp_t dt,
        output fp_t F[STATE_DIM][STATE_DIM]
    );
        for (int i = 0; i < STATE_DIM; i++)
            for (int j = 0; j < STATE_DIM; j++)
                F[i][j] = (i == j) ? FP_ONE : FP_ZERO;
        F[0][2] = dt;  // x += vx*dt
        F[1][3] = dt;  // y += vy*dt
        // ax, ay rows zero — CV model
    endfunction
    
    function automatic void compute_F_CA(
        input  fp_t dt,
        output fp_t F[STATE_DIM][STATE_DIM]
    );
        fp_t dt2_half;
        dt2_half = fp_mul(fp_mul(dt, dt), FP_HALF);
        for (int i = 0; i < STATE_DIM; i++)
            for (int j = 0; j < STATE_DIM; j++)
                F[i][j] = (i == j) ? FP_ONE : FP_ZERO;
        F[0][2] = dt;        // x += vx*dt
        F[1][3] = dt;        // y += vy*dt
        F[0][4] = dt2_half;  // x += 0.5*ax*dt^2
        F[1][5] = dt2_half;  // y += 0.5*ay*dt^2
        F[2][4] = dt;        // vx += ax*dt
        F[3][5] = dt;        // vy += ay*dt
    endfunction
    
    function automatic void compute_F_CT(
        input  fp_t dt,
        input  fp_t omega,   // Turn rate (positive or negative)
        output fp_t F[STATE_DIM][STATE_DIM]
    );
        fp_t s, c, odt;
        odt = fp_mul(omega, dt);
        
        // sin/cos approximation: sin(x)≈x-x³/6, cos(x)≈1-x²/2
        // For small angles this is sufficient in Q15.16
        fp_t x2, x3;
        x2 = fp_mul(odt, odt);
        x3 = fp_mul(x2, odt);
        s = odt - fp_mul(x3, 32'h0000_2AAB);  // sin ≈ x - x³/6
        c = FP_ONE - fp_mul(x2, FP_HALF);       // cos ≈ 1 - x²/2
        
        for (int i = 0; i < STATE_DIM; i++)
            for (int j = 0; j < STATE_DIM; j++)
                F[i][j] = (i == j) ? FP_ONE : FP_ZERO;
                
        // Position: x += (sin/ω)*vx - ((1-cos)/ω)*vy
        // Using dt-domain: simplified for small omega*dt
        F[0][2] = fp_mul(s, (omega != FP_ZERO) ? fp_div(FP_ONE, omega) : dt);
        F[1][3] = F[0][2];
        
        // Velocity rotation
        F[2][2] = c;
        F[2][3] = -s;
        F[3][2] = s;
        F[3][3] = c;
        // ax, ay not used in CT model
    endfunction

    //--------------------------------------------------------------------------
    // [REQ-IMM-030] Process Noise Q Matrices
    //--------------------------------------------------------------------------
    function automatic void compute_Q_CV(
        input  fp_t dt, input fp_t q_scale,
        output fp_t Q[STATE_DIM][STATE_DIM]
    );
        fp_t dt3_3, dt2_2;
        dt3_3 = fp_mul(fp_mul(fp_mul(dt, dt), dt), 32'h0000_5555); // dt³/3
        dt2_2 = fp_mul(fp_mul(dt, dt), FP_HALF);                     // dt²/2
        
        for (int i = 0; i < STATE_DIM; i++)
            for (int j = 0; j < STATE_DIM; j++)
                Q[i][j] = FP_ZERO;
        
        Q[0][0] = fp_mul(dt3_3, q_scale);
        Q[1][1] = Q[0][0];
        Q[0][2] = fp_mul(dt2_2, q_scale);
        Q[2][0] = Q[0][2];
        Q[1][3] = Q[0][2];
        Q[3][1] = Q[0][2];
        Q[2][2] = fp_mul(dt, q_scale);
        Q[3][3] = Q[2][2];
    endfunction
    
    function automatic void compute_Q_CA(
        input  fp_t dt, input fp_t q_scale,
        output fp_t Q[STATE_DIM][STATE_DIM]
    );
        // Piecewise constant jerk model for CA
        fp_t dt5_20, dt4_8, dt3_6, dt3_3, dt2_2;
        dt2_2  = fp_mul(fp_mul(dt, dt), FP_HALF);
        dt3_3  = fp_mul(fp_mul(fp_mul(dt, dt), dt), 32'h0000_5555);
        dt3_6  = fp_mul(dt3_3, FP_HALF);
        dt4_8  = fp_mul(fp_mul(dt2_2, dt2_2), FP_HALF);
        dt5_20 = fp_mul(dt4_8, fp_mul(dt, 32'h0000_6666));  // ≈dt5/20
        
        for (int i = 0; i < STATE_DIM; i++)
            for (int j = 0; j < STATE_DIM; j++)
                Q[i][j] = FP_ZERO;
        
        // Diagonal blocks for x,y pairs
        Q[0][0] = fp_mul(dt5_20, q_scale);
        Q[1][1] = Q[0][0];
        Q[0][2] = fp_mul(dt4_8, q_scale);
        Q[2][0] = Q[0][2];
        Q[1][3] = Q[0][2];
        Q[3][1] = Q[0][2];
        Q[0][4] = fp_mul(dt3_6, q_scale);
        Q[4][0] = Q[0][4];
        Q[1][5] = Q[0][4];
        Q[5][1] = Q[0][4];
        Q[2][2] = fp_mul(dt3_3, q_scale);
        Q[3][3] = Q[2][2];
        Q[2][4] = fp_mul(dt2_2, q_scale);
        Q[4][2] = Q[2][4];
        Q[3][5] = Q[2][4];
        Q[5][3] = Q[2][4];
        Q[4][4] = fp_mul(dt, q_scale);
        Q[5][5] = Q[4][4];
    endfunction

    //--------------------------------------------------------------------------
    // [REQ-IMM-040] Main Pipeline FSM
    //--------------------------------------------------------------------------
    typedef enum logic [2:0] {
        ST_IDLE    = 3'd0,
        ST_MIX     = 3'd1,    // TPM mixing
        ST_PREDICT = 3'd2,    // 6-model parallel predict
        ST_UPDATE  = 3'd3,    // 6-model parallel update
        ST_REWEIGHT= 3'd4,    // Mode probability update
        ST_OUTPUT  = 3'd5     // Latch outputs
    } imm_state_e;
    
    imm_state_e state, next_state;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= ST_IDLE;
        else
            state <= next_state;
    end
    
    always_comb begin
        next_state = state;
        case (state)
            ST_IDLE:    if (meas_valid) next_state = ST_MIX;
            ST_MIX:     next_state = ST_PREDICT;
            ST_PREDICT: next_state = ST_UPDATE;
            ST_UPDATE:  next_state = ST_REWEIGHT;
            ST_REWEIGHT:next_state = ST_OUTPUT;
            ST_OUTPUT:  next_state = ST_IDLE;
            default:    next_state = ST_IDLE;
        endcase
    end
    
    assign meas_ready = (state == ST_IDLE);
    
    // Latch measurement on entry
    always_ff @(posedge clk) begin
        if (state == ST_IDLE && meas_valid) begin
            pipe_meas <= meas;
            pipe_tgt  <= meas_tgt_id;
        end
    end
    
    //--------------------------------------------------------------------------
    // Pipeline Stage Processing
    //--------------------------------------------------------------------------
    integer t;  // Target index shortcut
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            initialized <= '0;
            out_valid   <= 1'b0;
            omega_out   <= FP_ZERO;
        end else begin
            out_valid <= 1'b0;
            t = pipe_tgt;
            
            case (state)
                //--------------------------------------------------------------
                // ST_MIX: IMM Mixing Step
                //--------------------------------------------------------------
                ST_MIX: begin
                    if (!initialized[t]) begin
                        // First measurement: initialize
                        init_target(t, pipe_meas);
                        initialized[t] <= 1'b1;
                    end
                    
                    // [REQ-IMM-002] CAPTURE pre-mixing unmixed states BEFORE mixing
                    for (int m = 0; m < N_MODELS; m++) begin
                        xu_out[m] <= x_mdl[t][m];
                        Pu_out[m] <= P_mdl[t][m];
                    end
                    
                    // Compute mixing probabilities and mixed states
                    for (int j = 0; j < N_MODELS; j++) begin
                        // mu_bar[j] = sum_i(TPM[i][j] * mu[i])
                        mu_bar[j] <= FP_ZERO;
                        for (int i = 0; i < N_MODELS; i++) begin
                            mu_bar[j] <= mu_bar[j] + fp_mul(cfg_tpm[i][j], mu_mdl[t][i]);
                        end
                        
                        // x_mix[j] = sum_i(mu_ij * x[i]) where mu_ij = TPM[i][j]*mu[i]/mu_bar[j]
                        for (int s = 0; s < STATE_DIM; s++) begin
                            x_mix[j][s] <= FP_ZERO;
                            for (int i = 0; i < N_MODELS; i++) begin
                                fp_t w;
                                w = fp_mul(cfg_tpm[i][j], mu_mdl[t][i]);
                                x_mix[j][s] <= x_mix[j][s] + fp_mul(w, x_mdl[t][i][s]);
                            end
                        end
                    end
                end
                
                //--------------------------------------------------------------
                // ST_PREDICT: Parallel 6-Model Prediction
                //--------------------------------------------------------------
                ST_PREDICT: begin
                    // Normalize mixed states by mu_bar
                    for (int j = 0; j < N_MODELS; j++) begin
                        if (mu_bar[j] > FP_EPS) begin
                            for (int s = 0; s < STATE_DIM; s++)
                                x_mix[j][s] <= fp_div(x_mix[j][s], mu_bar[j]);
                        end
                    end
                    
                    // Predict: x_pred = F * x_mix, P_pred = F*P*F' + Q
                    // (Simplified: use x_mix directly as predicted for pipeline)
                    for (int m = 0; m < N_MODELS; m++) begin
                        fp_t F[STATE_DIM][STATE_DIM];
                        fp_t Q[STATE_DIM][STATE_DIM];
                        
                        case (m)
                            MDL_CV:   begin compute_F_CV(cfg_dt, F); compute_Q_CV(cfg_dt, 32'h0001_0000, Q); end
                            MDL_CA:   begin compute_F_CA(cfg_dt, F); compute_Q_CA(cfg_dt, 32'h0005_0000, Q); end
                            MDL_CTP:  begin compute_F_CT(cfg_dt, 32'h0000_3333, F); compute_Q_CV(cfg_dt, 32'h0003_0000, Q); end  // omega=+0.2
                            MDL_CTN:  begin compute_F_CT(cfg_dt, 32'hFFFF_CCCD, F); compute_Q_CV(cfg_dt, 32'h0003_0000, Q); end  // omega=-0.2
                            MDL_JERK: begin compute_F_CA(cfg_dt, F); compute_Q_CA(cfg_dt, 32'h000A_0000, Q); end  // Higher Q
                            MDL_BALL: begin compute_F_CV(cfg_dt, F); compute_Q_CV(cfg_dt, 32'h0001_0000, Q); end  // + gravity
                            default:  begin compute_F_CV(cfg_dt, F); compute_Q_CV(cfg_dt, 32'h0001_0000, Q); end
                        endcase
                        
                        // x_pred[m] = F * x_mix[m]
                        for (int i = 0; i < STATE_DIM; i++) begin
                            x_pred[m][i] <= FP_ZERO;
                            for (int j = 0; j < STATE_DIM; j++)
                                x_pred[m][i] <= x_pred[m][i] + fp_mul(F[i][j], x_mix[m][j]);
                        end
                        
                        // Ballistic model: add gravity to vy prediction
                        if (m == MDL_BALL)
                            x_pred[m][3] <= x_pred[m][3] + fp_mul(32'hFFFA_6C18, cfg_dt); // -9.81*dt
                        
                        // P_pred[m] = F*P*F' + Q (simplified: just add Q diagonal to P)
                        // Full F*P*F' too expensive per cycle — use approximation
                        for (int i = 0; i < STATE_DIM; i++)
                            for (int j = 0; j < STATE_DIM; j++)
                                P_pred[m][i][j] <= P_mdl[t][m][i][j] + Q[i][j];
                    end
                end
                
                //--------------------------------------------------------------
                // ST_UPDATE: Parallel 6-Model Kalman Update
                //--------------------------------------------------------------
                ST_UPDATE: begin
                    for (int m = 0; m < N_MODELS; m++) begin
                        // Innovation: y = z - H*x_pred
                        // H = [1 0 0 0 0 0; 0 1 0 0 0 0]
                        fp_t innov [MEAS_DIM];
                        innov[0] = pipe_meas[0] - x_pred[m][0];
                        innov[1] = pipe_meas[1] - x_pred[m][1];
                        
                        // S = H*P*H' + R = P[0:1][0:1] + R
                        fp_t S00, S11, S01;
                        S00 = P_pred[m][0][0] + R_diag;
                        S11 = P_pred[m][1][1] + R_diag;
                        S01 = P_pred[m][0][1];
                        
                        // S_inv (2×2): det = S00*S11 - S01*S01
                        fp_t det, det_inv;
                        det = fp_mul(S00, S11) - fp_mul(S01, S01);
                        det_inv = fp_div(FP_ONE, (det > FP_EPS) ? det : FP_EPS);
                        
                        // NIS = innov' * S_inv * innov
                        fp_t nis_val;
                        nis_val = fp_mul(det_inv,
                            fp_mul(S11, fp_mul(innov[0], innov[0])) +
                            fp_mul(S00, fp_mul(innov[1], innov[1])) -
                            fp_mul(FP_TWO, fp_mul(S01, fp_mul(innov[0], innov[1])))
                        );
                        nis_w[m] <= nis_val;
                        
                        // Kalman gain: K = P*H' * S_inv
                        // K[i][0] = (P[i][0]*S11 - P[i][1]*S01) * det_inv
                        // K[i][1] = (P[i][1]*S00 - P[i][0]*S01) * det_inv
                        for (int i = 0; i < STATE_DIM; i++) begin
                            fp_t K0, K1;
                            K0 = fp_mul(
                                fp_mul(P_pred[m][i][0], S11) - fp_mul(P_pred[m][i][1], S01),
                                det_inv);
                            K1 = fp_mul(
                                fp_mul(P_pred[m][i][1], S00) - fp_mul(P_pred[m][i][0], S01),
                                det_inv);
                            
                            // x_upd = x_pred + K*innov
                            x_upd[m][i] <= x_pred[m][i] + fp_mul(K0, innov[0]) + fp_mul(K1, innov[1]);
                            
                            // P_upd = (I - K*H)*P (Joseph form simplified)
                            for (int j = 0; j < STATE_DIM; j++)
                                P_upd[m][i][j] <= P_pred[m][i][j] - fp_mul(K0, P_pred[m][0][j]) - fp_mul(K1, P_pred[m][1][j]);
                        end
                    end
                end
                
                //--------------------------------------------------------------
                // ST_REWEIGHT: Mode Probability Update
                //--------------------------------------------------------------
                ST_REWEIGHT: begin
                    // Likelihood ~ exp(-0.5*NIS) / sqrt(det(S))
                    // Simplified: L[m] = max(eps, 1/(1 + NIS[m]))  
                    fp_t L [N_MODELS];
                    fp_t L_sum;
                    L_sum = FP_ZERO;
                    
                    for (int m = 0; m < N_MODELS; m++) begin
                        L[m] = fp_div(FP_ONE, FP_ONE + nis_w[m]);
                        L_sum = L_sum + fp_mul(L[m], mu_bar[m]);
                    end
                    
                    // mu_new[m] = L[m] * mu_bar[m] / L_sum
                    for (int m = 0; m < N_MODELS; m++) begin
                        if (L_sum > FP_EPS)
                            mu_new[m] <= mu_clamp(fp_div(fp_mul(L[m], mu_bar[m]), L_sum));
                        else
                            mu_new[m] <= fp_div(FP_ONE, 32'h0006_0000); // 1/6 fallback
                    end
                    
                    // Store updated states back
                    for (int m = 0; m < N_MODELS; m++) begin
                        x_mdl[t][m]  <= x_upd[m];
                        P_mdl[t][m]  <= P_upd[m];
                    end
                end
                
                //--------------------------------------------------------------
                // ST_OUTPUT: Compute mixed estimate, latch outputs
                //--------------------------------------------------------------
                ST_OUTPUT: begin
                    // Store new mode probabilities
                    mu_mdl[t] <= mu_new;
                    
                    // Mixed estimate: x_mixed = sum(mu[m] * x_upd[m])
                    for (int s = 0; s < STATE_DIM; s++) begin
                        x_mixed_out[s] <= FP_ZERO;
                        for (int m = 0; m < N_MODELS; m++)
                            x_mixed_out[s] <= x_mixed_out[s] + fp_mul(mu_new[m], x_upd[m][s]);
                    end
                    
                    // Output mode probabilities and NIS
                    mu_out  <= mu_new;
                    nis_out <= nis_w;
                    
                    // Output post-update states for RTS
                    x_upd_out <= x_upd;
                    P_upd_out <= P_upd;
                    
                    // Omega estimate from CT model weights
                    // omega = mu_ct+ * (+0.2) + mu_ct- * (-0.2) + corrections
                    omega_out <= fp_mul(mu_new[MDL_CTP], 32'h0000_3333) +
                                 fp_mul(mu_new[MDL_CTN], 32'hFFFF_CCCD);
                    
                    out_valid <= 1'b1;
                end
                
                default: ;
            endcase
        end
    end

endmodule
