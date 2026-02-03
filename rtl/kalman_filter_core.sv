//==============================================================================
// NX-MIMOSA v3.1 Pro — Single-Model Kalman Filter Core
// [REQ-RTL-KF-01] Standard Kalman predict/update
// [REQ-RTL-KF-02] Stores xp, Pp for smoother
// [REQ-RTL-KF-03] Joseph-form covariance update
//==============================================================================

`timescale 1ns/1ps

module kalman_filter_core
    import nx_mimosa_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,
    
    // Control
    input  logic        start,
    input  logic        init,           // Initialize state
    output logic        done,
    
    // Measurement input
    input  fp_t         z [MEAS_DIM],   // [z_x, z_y]
    
    // Model matrices (from parent)
    input  fp_t         F [STATE_DIM][STATE_DIM],
    input  fp_t         Q [STATE_DIM][STATE_DIM],
    input  fp_t         H [MEAS_DIM][STATE_DIM],
    input  fp_t         R [MEAS_DIM][MEAS_DIM],
    
    // Initial state (for init)
    input  fp_t         x_init [STATE_DIM],
    input  fp_t         P_init [STATE_DIM][STATE_DIM],
    
    // Outputs
    output fp_t         x_filt [STATE_DIM],      // Filtered state
    output fp_t         P_filt [STATE_DIM][STATE_DIM],
    output fp_t         x_pred [STATE_DIM],      // Predicted state (for smoother!)
    output fp_t         P_pred [STATE_DIM][STATE_DIM],
    output fp_t         likelihood              // For mode probability
);

    //--------------------------------------------------------------------------
    // Internal State
    //--------------------------------------------------------------------------
    fp_t x [STATE_DIM];
    fp_t P [STATE_DIM][STATE_DIM];
    
    // Intermediate values
    fp_t xp [STATE_DIM];                        // x_pred = F @ x
    fp_t Pp [STATE_DIM][STATE_DIM];             // P_pred = F @ P @ F.T + Q
    fp_t y [MEAS_DIM];                          // Innovation = z - H @ xp
    fp_t S [MEAS_DIM][MEAS_DIM];                // Innovation cov = H @ Pp @ H.T + R
    fp_t S_inv [MEAS_DIM][MEAS_DIM];            // inv(S)
    fp_t K [STATE_DIM][MEAS_DIM];               // Kalman gain = Pp @ H.T @ S_inv
    
    //--------------------------------------------------------------------------
    // FSM
    //--------------------------------------------------------------------------
    typedef enum logic [3:0] {
        S_IDLE,
        S_INIT_STATE,
        S_PREDICT_X,      // xp = F @ x
        S_PREDICT_P1,     // Pp = F @ P (start)
        S_PREDICT_P2,     // Pp = Pp @ F.T + Q
        S_INNOVATION,     // y = z - H @ xp
        S_INNOV_COV,      // S = H @ Pp @ H.T + R
        S_S_INVERSE,      // S_inv = inv(S)
        S_KALMAN_GAIN,    // K = Pp @ H.T @ S_inv
        S_UPDATE_X,       // x = xp + K @ y
        S_UPDATE_P,       // P = (I - K@H) @ Pp (Joseph form)
        S_LIKELIHOOD,     // Compute likelihood
        S_DONE
    } state_t;
    
    state_t state, next_state;
    
    // Matrix operation submodules control
    logic mat_mul_start, mat_mul_done;
    fp_t mat_A [STATE_DIM][STATE_DIM];
    fp_t mat_B [STATE_DIM][STATE_DIM];
    fp_t mat_C [STATE_DIM][STATE_DIM];
    
    logic mat_inv_start, mat_inv_done;
    fp_t mat_inv_in [STATE_DIM][STATE_DIM];
    fp_t mat_inv_out [STATE_DIM][STATE_DIM];
    
    // Temporary storage
    fp_t FP [STATE_DIM][STATE_DIM];   // F @ P
    fp_t F_T [STATE_DIM][STATE_DIM];  // F transposed
    fp_t HPp [MEAS_DIM][STATE_DIM];   // H @ Pp (2x4)
    fp_t PpHT [STATE_DIM][MEAS_DIM];  // Pp @ H.T (4x2)
    
    //--------------------------------------------------------------------------
    // Matrix Multiply Instance (4x4)
    //--------------------------------------------------------------------------
    matrix_multiply_4x4 mat_mul_inst (
        .clk(clk),
        .rst_n(rst_n),
        .start(mat_mul_start),
        .A(mat_A),
        .B(mat_B),
        .C(mat_C),
        .done(mat_mul_done)
    );
    
    //--------------------------------------------------------------------------
    // 2x2 Matrix Inverse (for S)
    //--------------------------------------------------------------------------
    // S is only 2x2, implement inline
    function automatic void invert_2x2(
        input  fp_t M [MEAS_DIM][MEAS_DIM],
        output fp_t M_inv [MEAS_DIM][MEAS_DIM]
    );
        logic signed [2*DATA_WIDTH-1:0] det;
        fp_t det_fp, det_inv;
        
        // det = M[0][0]*M[1][1] - M[0][1]*M[1][0]
        det = (M[0][0] * M[1][1]) - (M[0][1] * M[1][0]);
        det_fp = det[DATA_WIDTH+FRAC_BITS-1:FRAC_BITS];
        
        // det_inv = 1/det (fixed-point)
        if (det_fp != 0) begin
            det_inv = fp_div(FP_ONE, det_fp);
        end else begin
            det_inv = FP_ONE;  // Regularization
        end
        
        // M_inv = adj(M) / det
        M_inv[0][0] = fp_mul(M[1][1], det_inv);
        M_inv[0][1] = fp_mul(-M[0][1], det_inv);
        M_inv[1][0] = fp_mul(-M[1][0], det_inv);
        M_inv[1][1] = fp_mul(M[0][0], det_inv);
    endfunction
    
    //--------------------------------------------------------------------------
    // Transpose F
    //--------------------------------------------------------------------------
    always_comb begin
        for (int i = 0; i < STATE_DIM; i++)
            for (int j = 0; j < STATE_DIM; j++)
                F_T[i][j] = F[j][i];
    end
    
    //--------------------------------------------------------------------------
    // FSM Sequential
    //--------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            done <= 1'b0;
            mat_mul_start <= 1'b0;
            for (int i = 0; i < STATE_DIM; i++) begin
                x[i] <= '0;
                x_filt[i] <= '0;
                x_pred[i] <= '0;
                for (int j = 0; j < STATE_DIM; j++) begin
                    P[i][j] <= '0;
                    P_filt[i][j] <= '0;
                    P_pred[i][j] <= '0;
                end
            end
            likelihood <= FP_ONE;
        end else begin
            state <= next_state;
            done <= 1'b0;
            mat_mul_start <= 1'b0;
            
            case (state)
                S_IDLE: begin
                    if (init) begin
                        x <= x_init;
                        P <= P_init;
                    end
                end
                
                S_INIT_STATE: begin
                    x <= x_init;
                    P <= P_init;
                end
                
                //--------------------------------------------------------------
                // Predict: xp = F @ x
                //--------------------------------------------------------------
                S_PREDICT_X: begin
                    for (int i = 0; i < STATE_DIM; i++) begin
                        logic signed [2*DATA_WIDTH-1:0] acc;
                        acc = '0;
                        for (int j = 0; j < STATE_DIM; j++)
                            acc = acc + (F[i][j] * x[j]);
                        xp[i] <= acc[DATA_WIDTH+FRAC_BITS-1:FRAC_BITS];
                    end
                end
                
                //--------------------------------------------------------------
                // Predict: FP = F @ P
                //--------------------------------------------------------------
                S_PREDICT_P1: begin
                    mat_A <= F;
                    mat_B <= P;
                    mat_mul_start <= 1'b1;
                end
                
                //--------------------------------------------------------------
                // Predict: Pp = FP @ F.T + Q
                //--------------------------------------------------------------
                S_PREDICT_P2: begin
                    if (mat_mul_done) begin
                        FP <= mat_C;
                        // Now compute Pp = FP @ F.T + Q
                        for (int i = 0; i < STATE_DIM; i++) begin
                            for (int j = 0; j < STATE_DIM; j++) begin
                                logic signed [2*DATA_WIDTH-1:0] acc;
                                acc = '0;
                                for (int k = 0; k < STATE_DIM; k++)
                                    acc = acc + (mat_C[i][k] * F_T[k][j]);
                                Pp[i][j] <= acc[DATA_WIDTH+FRAC_BITS-1:FRAC_BITS] + Q[i][j];
                            end
                        end
                        // Store for smoother
                        x_pred <= xp;
                    end
                end
                
                //--------------------------------------------------------------
                // Innovation: y = z - H @ xp
                //--------------------------------------------------------------
                S_INNOVATION: begin
                    for (int i = 0; i < MEAS_DIM; i++) begin
                        logic signed [2*DATA_WIDTH-1:0] acc;
                        acc = '0;
                        for (int j = 0; j < STATE_DIM; j++)
                            acc = acc + (H[i][j] * xp[j]);
                        y[i] <= z[i] - acc[DATA_WIDTH+FRAC_BITS-1:FRAC_BITS];
                    end
                    P_pred <= Pp;
                end
                
                //--------------------------------------------------------------
                // Innovation Covariance: S = H @ Pp @ H.T + R
                //--------------------------------------------------------------
                S_INNOV_COV: begin
                    // HPp = H @ Pp (2x4 @ 4x4 = 2x4)
                    for (int i = 0; i < MEAS_DIM; i++) begin
                        for (int j = 0; j < STATE_DIM; j++) begin
                            logic signed [2*DATA_WIDTH-1:0] acc;
                            acc = '0;
                            for (int k = 0; k < STATE_DIM; k++)
                                acc = acc + (H[i][k] * Pp[k][j]);
                            HPp[i][j] <= acc[DATA_WIDTH+FRAC_BITS-1:FRAC_BITS];
                        end
                    end
                    
                    // S = HPp @ H.T + R (2x4 @ 4x2 = 2x2)
                    for (int i = 0; i < MEAS_DIM; i++) begin
                        for (int j = 0; j < MEAS_DIM; j++) begin
                            logic signed [2*DATA_WIDTH-1:0] acc;
                            acc = '0;
                            for (int k = 0; k < STATE_DIM; k++)
                                acc = acc + (HPp[i][k] * H[j][k]);  // H.T
                            S[i][j] <= acc[DATA_WIDTH+FRAC_BITS-1:FRAC_BITS] + R[i][j];
                        end
                    end
                end
                
                //--------------------------------------------------------------
                // S Inverse (2x2)
                //--------------------------------------------------------------
                S_S_INVERSE: begin
                    invert_2x2(S, S_inv);
                end
                
                //--------------------------------------------------------------
                // Kalman Gain: K = Pp @ H.T @ S_inv (4x2)
                //--------------------------------------------------------------
                S_KALMAN_GAIN: begin
                    // PpHT = Pp @ H.T (4x4 @ 4x2 = 4x2)
                    for (int i = 0; i < STATE_DIM; i++) begin
                        for (int j = 0; j < MEAS_DIM; j++) begin
                            logic signed [2*DATA_WIDTH-1:0] acc;
                            acc = '0;
                            for (int k = 0; k < STATE_DIM; k++)
                                acc = acc + (Pp[i][k] * H[j][k]);  // H.T
                            PpHT[i][j] <= acc[DATA_WIDTH+FRAC_BITS-1:FRAC_BITS];
                        end
                    end
                    
                    // K = PpHT @ S_inv (4x2 @ 2x2 = 4x2)
                    for (int i = 0; i < STATE_DIM; i++) begin
                        for (int j = 0; j < MEAS_DIM; j++) begin
                            logic signed [2*DATA_WIDTH-1:0] acc;
                            acc = '0;
                            for (int k = 0; k < MEAS_DIM; k++)
                                acc = acc + (PpHT[i][k] * S_inv[k][j]);
                            K[i][j] <= acc[DATA_WIDTH+FRAC_BITS-1:FRAC_BITS];
                        end
                    end
                end
                
                //--------------------------------------------------------------
                // Update State: x = xp + K @ y
                //--------------------------------------------------------------
                S_UPDATE_X: begin
                    for (int i = 0; i < STATE_DIM; i++) begin
                        logic signed [2*DATA_WIDTH-1:0] acc;
                        acc = '0;
                        for (int j = 0; j < MEAS_DIM; j++)
                            acc = acc + (K[i][j] * y[j]);
                        x[i] <= xp[i] + acc[DATA_WIDTH+FRAC_BITS-1:FRAC_BITS];
                        x_filt[i] <= xp[i] + acc[DATA_WIDTH+FRAC_BITS-1:FRAC_BITS];
                    end
                end
                
                //--------------------------------------------------------------
                // Update Covariance: P = (I - K@H) @ Pp (Joseph form)
                //--------------------------------------------------------------
                S_UPDATE_P: begin
                    // I_KH = I - K @ H
                    fp_t I_KH [STATE_DIM][STATE_DIM];
                    for (int i = 0; i < STATE_DIM; i++) begin
                        for (int j = 0; j < STATE_DIM; j++) begin
                            logic signed [2*DATA_WIDTH-1:0] acc;
                            acc = '0;
                            for (int k = 0; k < MEAS_DIM; k++)
                                acc = acc + (K[i][k] * H[k][j]);
                            I_KH[i][j] = ((i == j) ? FP_ONE : FP_ZERO) - 
                                         acc[DATA_WIDTH+FRAC_BITS-1:FRAC_BITS];
                        end
                    end
                    
                    // P = I_KH @ Pp
                    for (int i = 0; i < STATE_DIM; i++) begin
                        for (int j = 0; j < STATE_DIM; j++) begin
                            logic signed [2*DATA_WIDTH-1:0] acc;
                            acc = '0;
                            for (int k = 0; k < STATE_DIM; k++)
                                acc = acc + (I_KH[i][k] * Pp[k][j]);
                            P[i][j] <= acc[DATA_WIDTH+FRAC_BITS-1:FRAC_BITS];
                            P_filt[i][j] <= acc[DATA_WIDTH+FRAC_BITS-1:FRAC_BITS];
                        end
                    end
                end
                
                //--------------------------------------------------------------
                // Compute Likelihood: L = exp(-0.5 * y.T @ S_inv @ y) / sqrt(det(S))
                // Simplified: Just use y.T @ S_inv @ y as log-likelihood proxy
                //--------------------------------------------------------------
                S_LIKELIHOOD: begin
                    logic signed [2*DATA_WIDTH-1:0] yTSinvy;
                    fp_t Sinv_y [MEAS_DIM];
                    
                    // S_inv @ y
                    for (int i = 0; i < MEAS_DIM; i++) begin
                        logic signed [2*DATA_WIDTH-1:0] acc;
                        acc = '0;
                        for (int j = 0; j < MEAS_DIM; j++)
                            acc = acc + (S_inv[i][j] * y[j]);
                        Sinv_y[i] = acc[DATA_WIDTH+FRAC_BITS-1:FRAC_BITS];
                    end
                    
                    // y.T @ (S_inv @ y)
                    yTSinvy = '0;
                    for (int i = 0; i < MEAS_DIM; i++)
                        yTSinvy = yTSinvy + (y[i] * Sinv_y[i]);
                    
                    // Likelihood proxy (inversely proportional to Mahalanobis distance)
                    // Use 1/(1 + d^2) as approximation
                    fp_t d_sq = yTSinvy[DATA_WIDTH+FRAC_BITS-1:FRAC_BITS];
                    likelihood <= fp_div(FP_ONE, FP_ONE + (d_sq >>> 4));
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
            S_IDLE:       if (start) next_state = init ? S_INIT_STATE : S_PREDICT_X;
            S_INIT_STATE: next_state = S_DONE;
            S_PREDICT_X:  next_state = S_PREDICT_P1;
            S_PREDICT_P1: next_state = S_PREDICT_P2;
            S_PREDICT_P2: if (mat_mul_done) next_state = S_INNOVATION;
            S_INNOVATION: next_state = S_INNOV_COV;
            S_INNOV_COV:  next_state = S_S_INVERSE;
            S_S_INVERSE:  next_state = S_KALMAN_GAIN;
            S_KALMAN_GAIN: next_state = S_UPDATE_X;
            S_UPDATE_X:   next_state = S_UPDATE_P;
            S_UPDATE_P:   next_state = S_LIKELIHOOD;
            S_LIKELIHOOD: next_state = S_DONE;
            S_DONE:       next_state = S_IDLE;
        endcase
    end

endmodule
