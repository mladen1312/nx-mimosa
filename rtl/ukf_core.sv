//==============================================================================
// NX-MIMOSA v2.0 — Unscented Kalman Filter Core
// [REQ-V2-UKF-01] Sigma point generation
// [REQ-V2-UKF-02] Unscented transform for prediction
// [REQ-V2-UKF-03] Unscented transform for update
// [REQ-V2-UKF-04] Cholesky decomposition for sqrt(P)
//==============================================================================
// Reference: Wan & van der Merwe "The Unscented Kalman Filter" 2000
// Author: Dr. Mladen Mešter / Nexellum d.o.o.
//==============================================================================

`timescale 1ns/1ps

module ukf_core
    import nx_mimosa_pkg_v2::*;
(
    input  logic        clk,
    input  logic        rst_n,
    
    // Control
    input  logic        start,
    input  logic        init,
    output logic        done,
    
    // Measurement input
    input  fp_t         z [MEAS_DIM],
    
    // Model configuration
    input  logic        is_ct_model,       // CT model flag (for nonlinear F)
    input  fp_t         omega,             // Turn rate (for CT)
    input  fp_t         dt,
    input  fp_t         q_process,
    input  fp_t         r_meas,
    
    // Measurement model selection
    input  logic        polar_meas,        // 0: Cartesian, 1: Polar (r, θ)
    
    // Initial state
    input  fp_t         x_init [STATE_DIM],
    input  fp_t         P_init [STATE_DIM][STATE_DIM],
    
    // Outputs
    output fp_t         x_filt [STATE_DIM],
    output fp_t         P_filt [STATE_DIM][STATE_DIM],
    output fp_t         x_pred [STATE_DIM],
    output fp_t         P_pred [STATE_DIM][STATE_DIM],
    output fp_t         likelihood,
    output fp_t         innovation [MEAS_DIM],
    output fp_t         S_inv [MEAS_DIM][MEAS_DIM]
);

    //--------------------------------------------------------------------------
    // UKF Parameters
    //--------------------------------------------------------------------------
    localparam int N = STATE_DIM;
    localparam int L = 2 * N + 1;  // Number of sigma points
    
    // Scaling parameters (computed at elaboration)
    // lambda = alpha² * (n + kappa) - n
    // For alpha=0.001, kappa=0, n=4: lambda = 0.000001*(4+0)-4 ≈ -4
    // We use alpha=0.5 for stability: lambda = 0.25*(4+3)-4 = -2.25
    localparam fp_t LAMBDA = -32'h0002_4000;  // ≈ -2.25 for n=4
    localparam fp_t N_PLUS_LAMBDA = 32'h0001_C000;  // n + lambda ≈ 1.75
    
    // Weights
    fp_t Wm [L];  // Mean weights
    fp_t Wc [L];  // Covariance weights
    
    //--------------------------------------------------------------------------
    // Internal State
    //--------------------------------------------------------------------------
    fp_t x [STATE_DIM];
    fp_t P [STATE_DIM][STATE_DIM];
    
    // Sigma points
    fp_t chi [L][STATE_DIM];           // State sigma points
    fp_t chi_pred [L][STATE_DIM];      // Predicted sigma points
    fp_t gamma [L][MEAS_DIM];          // Measurement sigma points
    
    // Square root of P (via Cholesky)
    fp_t sqrt_P [STATE_DIM][STATE_DIM];
    
    // Intermediate values
    fp_t xp [STATE_DIM];
    fp_t Pp [STATE_DIM][STATE_DIM];
    fp_t zp [MEAS_DIM];
    fp_t Pzz [MEAS_DIM][MEAS_DIM];
    fp_t Pxz [STATE_DIM][MEAS_DIM];
    fp_t K [STATE_DIM][MEAS_DIM];
    fp_t y [MEAS_DIM];
    fp_t S [MEAS_DIM][MEAS_DIM];
    
    //--------------------------------------------------------------------------
    // FSM
    //--------------------------------------------------------------------------
    typedef enum logic [4:0] {
        S_IDLE,
        S_INIT_STATE,
        S_COMPUTE_WEIGHTS,
        S_CHOLESKY,
        S_GEN_SIGMA,
        S_PREDICT_SIGMA,
        S_PREDICT_MEAN,
        S_PREDICT_COV,
        S_MEAS_SIGMA,
        S_MEAS_MEAN,
        S_INNOVATION_COV,
        S_CROSS_COV,
        S_KALMAN_GAIN,
        S_UPDATE_X,
        S_UPDATE_P,
        S_LIKELIHOOD,
        S_DONE
    } state_t;
    
    state_t state, next_state;
    
    //--------------------------------------------------------------------------
    // Weight Computation (constant for given N)
    //--------------------------------------------------------------------------
    // Wm[0] = lambda / (n + lambda)
    // Wc[0] = Wm[0] + (1 - alpha² + beta)
    // Wm[i] = Wc[i] = 1 / (2*(n + lambda))  for i = 1..2n
    //--------------------------------------------------------------------------
    initial begin
        // For n=4, lambda=-2.25, n+lambda=1.75
        // Wm[0] = -2.25 / 1.75 ≈ -1.286
        Wm[0] = -32'h0001_4924;  // ≈ -1.286
        // Wc[0] = Wm[0] + 1 - 0.001 + 2 ≈ 1.713
        Wc[0] = 32'h0001_B6DB;   // ≈ 1.713
        
        // Wm[i] = Wc[i] = 1/(2*1.75) = 0.286 for i=1..8
        for (int i = 1; i < L; i++) begin
            Wm[i] = 32'h0000_4924;  // ≈ 0.286
            Wc[i] = 32'h0000_4924;
        end
    end
    
    //--------------------------------------------------------------------------
    // Cholesky Decomposition: P = L * L^T
    //--------------------------------------------------------------------------
    // Simple column-by-column algorithm for 4x4 matrix
    //--------------------------------------------------------------------------
    task automatic cholesky_decomp(
        input  fp_t A [STATE_DIM][STATE_DIM],
        output fp_t L_mat [STATE_DIM][STATE_DIM]
    );
        for (int i = 0; i < STATE_DIM; i++) begin
            for (int j = 0; j < STATE_DIM; j++) begin
                L_mat[i][j] = FP_ZERO;
            end
        end
        
        for (int j = 0; j < STATE_DIM; j++) begin
            fp_t sum_sq;
            sum_sq = FP_ZERO;
            
            // Compute diagonal element
            for (int k = 0; k < j; k++) begin
                sum_sq = sum_sq + fp_mul(L_mat[j][k], L_mat[j][k]);
            end
            fp_t diag_val = A[j][j] - sum_sq;
            L_mat[j][j] = (diag_val > FP_ZERO) ? fp_sqrt_approx(diag_val) : 32'h0000_1000;
            
            // Compute off-diagonal elements
            for (int i = j + 1; i < STATE_DIM; i++) begin
                fp_t sum_prod;
                sum_prod = FP_ZERO;
                for (int k = 0; k < j; k++) begin
                    sum_prod = sum_prod + fp_mul(L_mat[i][k], L_mat[j][k]);
                end
                if (L_mat[j][j] != FP_ZERO)
                    L_mat[i][j] = fp_div(A[i][j] - sum_prod, L_mat[j][j]);
                else
                    L_mat[i][j] = FP_ZERO;
            end
        end
    endtask
    
    //--------------------------------------------------------------------------
    // Motion Model: f(x, dt, omega)
    //--------------------------------------------------------------------------
    function automatic void motion_model(
        input  fp_t x_in [STATE_DIM],
        input  logic is_ct,
        input  fp_t dt_in,
        input  fp_t omega_in,
        output fp_t x_out [STATE_DIM]
    );
        if (!is_ct) begin
            // CV Model: x' = x + v*dt
            x_out[0] = x_in[0] + fp_mul(x_in[2], dt_in);  // x + vx*dt
            x_out[1] = x_in[1] + fp_mul(x_in[3], dt_in);  // y + vy*dt
            x_out[2] = x_in[2];  // vx unchanged
            x_out[3] = x_in[3];  // vy unchanged
        end else begin
            // CT Model: Coordinated turn
            fp_t omega_dt = fp_mul(omega_in, dt_in);
            fp_t sin_wdt, cos_wdt;
            // Simplified: use small angle approximation or LUT
            sin_wdt = omega_dt;  // sin(x) ≈ x for small x
            cos_wdt = FP_ONE - (fp_mul(omega_dt, omega_dt) >>> 1);  // cos(x) ≈ 1 - x²/2
            
            if (omega_in != FP_ZERO) begin
                fp_t inv_omega = fp_div(FP_ONE, omega_in);
                fp_t sin_div_w = fp_mul(sin_wdt, inv_omega);
                fp_t one_minus_cos_div_w = fp_mul(FP_ONE - cos_wdt, inv_omega);
                
                x_out[0] = x_in[0] + fp_mul(sin_div_w, x_in[2]) - fp_mul(one_minus_cos_div_w, x_in[3]);
                x_out[1] = x_in[1] + fp_mul(one_minus_cos_div_w, x_in[2]) + fp_mul(sin_div_w, x_in[3]);
                x_out[2] = fp_mul(cos_wdt, x_in[2]) - fp_mul(sin_wdt, x_in[3]);
                x_out[3] = fp_mul(sin_wdt, x_in[2]) + fp_mul(cos_wdt, x_in[3]);
            end else begin
                // omega ≈ 0, use CV
                x_out[0] = x_in[0] + fp_mul(x_in[2], dt_in);
                x_out[1] = x_in[1] + fp_mul(x_in[3], dt_in);
                x_out[2] = x_in[2];
                x_out[3] = x_in[3];
            end
        end
    endfunction
    
    //--------------------------------------------------------------------------
    // Measurement Model: h(x)
    //--------------------------------------------------------------------------
    function automatic void meas_model(
        input  fp_t x_in [STATE_DIM],
        input  logic polar,
        output fp_t z_out [MEAS_DIM]
    );
        if (!polar) begin
            // Cartesian: h(x) = [x, y]
            z_out[0] = x_in[0];
            z_out[1] = x_in[1];
        end else begin
            // Polar: h(x) = [sqrt(x²+y²), atan2(y,x)]
            fp_t x_sq = fp_mul(x_in[0], x_in[0]);
            fp_t y_sq = fp_mul(x_in[1], x_in[1]);
            z_out[0] = fp_sqrt_approx(x_sq + y_sq);  // Range
            // Angle approximation: atan2(y,x) ≈ y/x for |y| < |x|
            z_out[1] = (x_in[0] != FP_ZERO) ? fp_div(x_in[1], x_in[0]) : FP_ZERO;
        end
    endfunction
    
    //--------------------------------------------------------------------------
    // Main FSM
    //--------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            done <= 1'b0;
            likelihood <= FP_ONE;
            
            for (int i = 0; i < STATE_DIM; i++) begin
                x[i] <= FP_ZERO;
                x_filt[i] <= FP_ZERO;
                x_pred[i] <= FP_ZERO;
                for (int j = 0; j < STATE_DIM; j++) begin
                    P[i][j] <= FP_ZERO;
                    P_filt[i][j] <= FP_ZERO;
                    P_pred[i][j] <= FP_ZERO;
                end
            end
            for (int i = 0; i < MEAS_DIM; i++) begin
                innovation[i] <= FP_ZERO;
                for (int j = 0; j < MEAS_DIM; j++) begin
                    S_inv[i][j] <= FP_ZERO;
                end
            end
        end else begin
            state <= next_state;
            done <= (next_state == S_DONE);
            
            case (state)
                S_INIT_STATE: begin
                    x <= x_init;
                    P <= P_init;
                end
                
                S_CHOLESKY: begin
                    // Compute sqrt((n+lambda)*P) for sigma points
                    fp_t scaled_P [STATE_DIM][STATE_DIM];
                    for (int i = 0; i < STATE_DIM; i++)
                        for (int j = 0; j < STATE_DIM; j++)
                            scaled_P[i][j] = fp_mul(N_PLUS_LAMBDA, P[i][j]);
                    cholesky_decomp(scaled_P, sqrt_P);
                end
                
                S_GEN_SIGMA: begin
                    // chi[0] = x
                    chi[0] <= x;
                    // chi[1..n] = x + sqrt_P[:,i]
                    // chi[n+1..2n] = x - sqrt_P[:,i]
                    for (int i = 0; i < STATE_DIM; i++) begin
                        for (int j = 0; j < STATE_DIM; j++) begin
                            chi[1+i][j] <= x[j] + sqrt_P[j][i];
                            chi[1+STATE_DIM+i][j] <= x[j] - sqrt_P[j][i];
                        end
                    end
                end
                
                S_PREDICT_SIGMA: begin
                    // Propagate each sigma point through motion model
                    for (int i = 0; i < L; i++) begin
                        motion_model(chi[i], is_ct_model, dt, omega, chi_pred[i]);
                    end
                end
                
                S_PREDICT_MEAN: begin
                    // xp = sum(Wm[i] * chi_pred[i])
                    for (int j = 0; j < STATE_DIM; j++) begin
                        fp_t sum;
                        sum = FP_ZERO;
                        for (int i = 0; i < L; i++) begin
                            sum = sum + fp_mul(Wm[i], chi_pred[i][j]);
                        end
                        xp[j] <= sum;
                        x_pred[j] <= sum;
                    end
                end
                
                S_PREDICT_COV: begin
                    // Pp = sum(Wc[i] * (chi_pred[i] - xp) * (chi_pred[i] - xp)') + Q
                    for (int r = 0; r < STATE_DIM; r++) begin
                        for (int c = 0; c < STATE_DIM; c++) begin
                            fp_t sum;
                            sum = FP_ZERO;
                            for (int i = 0; i < L; i++) begin
                                fp_t dx_r = chi_pred[i][r] - xp[r];
                                fp_t dx_c = chi_pred[i][c] - xp[c];
                                sum = sum + fp_mul(Wc[i], fp_mul(dx_r, dx_c));
                            end
                            // Add process noise (diagonal)
                            if (r == c)
                                Pp[r][c] <= sum + q_process;
                            else
                                Pp[r][c] <= sum;
                        end
                    end
                    P_pred <= Pp;
                end
                
                S_MEAS_SIGMA: begin
                    // Transform sigma points through measurement model
                    for (int i = 0; i < L; i++) begin
                        meas_model(chi_pred[i], polar_meas, gamma[i]);
                    end
                end
                
                S_MEAS_MEAN: begin
                    // zp = sum(Wm[i] * gamma[i])
                    for (int j = 0; j < MEAS_DIM; j++) begin
                        fp_t sum;
                        sum = FP_ZERO;
                        for (int i = 0; i < L; i++) begin
                            sum = sum + fp_mul(Wm[i], gamma[i][j]);
                        end
                        zp[j] <= sum;
                    end
                end
                
                S_INNOVATION_COV: begin
                    // Pzz = sum(Wc[i] * (gamma[i] - zp) * (gamma[i] - zp)') + R
                    // S = Pzz
                    // y = z - zp
                    for (int r = 0; r < MEAS_DIM; r++) begin
                        y[r] <= z[r] - zp[r];
                        innovation[r] <= z[r] - zp[r];
                        
                        for (int c = 0; c < MEAS_DIM; c++) begin
                            fp_t sum;
                            sum = FP_ZERO;
                            for (int i = 0; i < L; i++) begin
                                fp_t dz_r = gamma[i][r] - zp[r];
                                fp_t dz_c = gamma[i][c] - zp[c];
                                sum = sum + fp_mul(Wc[i], fp_mul(dz_r, dz_c));
                            end
                            if (r == c)
                                S[r][c] <= sum + fp_mul(r_meas, r_meas);
                            else
                                S[r][c] <= sum;
                        end
                    end
                end
                
                S_CROSS_COV: begin
                    // Pxz = sum(Wc[i] * (chi_pred[i] - xp) * (gamma[i] - zp)')
                    for (int r = 0; r < STATE_DIM; r++) begin
                        for (int c = 0; c < MEAS_DIM; c++) begin
                            fp_t sum;
                            sum = FP_ZERO;
                            for (int i = 0; i < L; i++) begin
                                fp_t dx = chi_pred[i][r] - xp[r];
                                fp_t dz = gamma[i][c] - zp[c];
                                sum = sum + fp_mul(Wc[i], fp_mul(dx, dz));
                            end
                            Pxz[r][c] <= sum;
                        end
                    end
                    
                    // Compute S_inv (2x2 inline)
                    begin
                        fp_t det = fp_mul(S[0][0], S[1][1]) - fp_mul(S[0][1], S[1][0]);
                        fp_t det_inv = (det != FP_ZERO) ? fp_div(FP_ONE, det) : FP_ONE;
                        S_inv[0][0] <= fp_mul(S[1][1], det_inv);
                        S_inv[0][1] <= fp_mul(-S[0][1], det_inv);
                        S_inv[1][0] <= fp_mul(-S[1][0], det_inv);
                        S_inv[1][1] <= fp_mul(S[0][0], det_inv);
                    end
                end
                
                S_KALMAN_GAIN: begin
                    // K = Pxz * S_inv
                    for (int r = 0; r < STATE_DIM; r++) begin
                        for (int c = 0; c < MEAS_DIM; c++) begin
                            K[r][c] <= fp_mul(Pxz[r][0], S_inv[0][c]) + 
                                      fp_mul(Pxz[r][1], S_inv[1][c]);
                        end
                    end
                end
                
                S_UPDATE_X: begin
                    // x = xp + K * y
                    for (int i = 0; i < STATE_DIM; i++) begin
                        x[i] <= xp[i] + fp_mul(K[i][0], y[0]) + fp_mul(K[i][1], y[1]);
                        x_filt[i] <= xp[i] + fp_mul(K[i][0], y[0]) + fp_mul(K[i][1], y[1]);
                    end
                end
                
                S_UPDATE_P: begin
                    // P = Pp - K * S * K'
                    for (int r = 0; r < STATE_DIM; r++) begin
                        for (int c = 0; c < STATE_DIM; c++) begin
                            fp_t KSKt_elem;
                            KSKt_elem = FP_ZERO;
                            for (int i = 0; i < MEAS_DIM; i++) begin
                                for (int j = 0; j < MEAS_DIM; j++) begin
                                    KSKt_elem = KSKt_elem + 
                                               fp_mul(fp_mul(K[r][i], S[i][j]), K[c][j]);
                                end
                            end
                            P[r][c] <= Pp[r][c] - KSKt_elem;
                            P_filt[r][c] <= Pp[r][c] - KSKt_elem;
                        end
                    end
                end
                
                S_LIKELIHOOD: begin
                    // likelihood = exp(-0.5 * y' * S_inv * y) / sqrt(det(2*pi*S))
                    // Simplified: use NIS directly
                    fp_t nis = fp_mul(fp_mul(y[0], S_inv[0][0]), y[0]) +
                              fp_mul(fp_mul(y[0], S_inv[0][1]), y[1]) * 2 +
                              fp_mul(fp_mul(y[1], S_inv[1][1]), y[1]);
                    // likelihood ≈ 1/(1 + nis) for simplicity
                    likelihood <= fp_div(FP_ONE, FP_ONE + (nis >>> 2));
                end
                
                default: ;
            endcase
        end
    end
    
    //--------------------------------------------------------------------------
    // FSM Combinational
    //--------------------------------------------------------------------------
    always_comb begin
        next_state = state;
        case (state)
            S_IDLE:           if (start) next_state = init ? S_INIT_STATE : S_CHOLESKY;
            S_INIT_STATE:     next_state = S_DONE;
            S_CHOLESKY:       next_state = S_GEN_SIGMA;
            S_GEN_SIGMA:      next_state = S_PREDICT_SIGMA;
            S_PREDICT_SIGMA:  next_state = S_PREDICT_MEAN;
            S_PREDICT_MEAN:   next_state = S_PREDICT_COV;
            S_PREDICT_COV:    next_state = S_MEAS_SIGMA;
            S_MEAS_SIGMA:     next_state = S_MEAS_MEAN;
            S_MEAS_MEAN:      next_state = S_INNOVATION_COV;
            S_INNOVATION_COV: next_state = S_CROSS_COV;
            S_CROSS_COV:      next_state = S_KALMAN_GAIN;
            S_KALMAN_GAIN:    next_state = S_UPDATE_X;
            S_UPDATE_X:       next_state = S_UPDATE_P;
            S_UPDATE_P:       next_state = S_LIKELIHOOD;
            S_LIKELIHOOD:     next_state = S_DONE;
            S_DONE:           next_state = S_IDLE;
        endcase
    end

endmodule
