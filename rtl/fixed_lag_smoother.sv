//==============================================================================
// QEDMMA v3.1 Pro — Fixed-Lag IMM Smoother (COMPLETE IMPLEMENTATION)
// [REQ-RTL-SMOOTH-01] Per-model RTS backward pass
// [REQ-RTL-SMOOTH-02] REAL G = Pf @ F.T @ inv(Pp) computation
// [REQ-RTL-SMOOTH-03] xs = xf + G @ (xs[k+1] - xp[k+1])
// [REQ-RTL-SMOOTH-04] Circular buffer with LAG_DEPTH samples
//==============================================================================
// Author: Dr. Mladen Mešter / Nexellum d.o.o.
// License: Commercial (qedmma-pro)
//==============================================================================

`timescale 1ns/1ps

module fixed_lag_smoother
    import qedmma_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,
    
    // Forward pass data input (from IMM core)
    input  logic        fwd_valid,
    input  fp_t         fwd_xf [N_MODELS][STATE_DIM],
    input  fp_t         fwd_Pf [N_MODELS][STATE_DIM][STATE_DIM],
    input  fp_t         fwd_xp [N_MODELS][STATE_DIM],      // CRITICAL: predictions from forward
    input  fp_t         fwd_Pp [N_MODELS][STATE_DIM][STATE_DIM],
    input  fp_t         fwd_mu [N_MODELS],
    input  fp_t         F_mat [N_MODELS][STATE_DIM][STATE_DIM],
    
    // Configuration
    input  logic        enable,
    
    // Smoothed output
    output logic        smooth_valid,
    output fp_t         x_smooth [STATE_DIM],
    output fp_t         x_smooth_model [N_MODELS][STATE_DIM]
);

    //--------------------------------------------------------------------------
    // Circular Buffer for Forward Pass Data
    //--------------------------------------------------------------------------
    localparam int PTR_BITS = $clog2(LAG_DEPTH);
    
    typedef struct packed {
        fp_t xf [N_MODELS][STATE_DIM];
        fp_t Pf [N_MODELS][STATE_DIM][STATE_DIM];
        fp_t xp [N_MODELS][STATE_DIM];
        fp_t Pp [N_MODELS][STATE_DIM][STATE_DIM];
        fp_t mu [N_MODELS];
    } forward_data_t;
    
    (* ram_style = "block" *) forward_data_t buffer [LAG_DEPTH];
    
    logic [PTR_BITS-1:0] wr_ptr;
    logic [PTR_BITS-1:0] rd_ptr;
    logic [PTR_BITS-1:0] fill_count;
    logic buffer_full;
    
    assign buffer_full = (fill_count == LAG_DEPTH);
    
    //--------------------------------------------------------------------------
    // Matrix Operation Modules
    //--------------------------------------------------------------------------
    // Matrix multiply for G computation
    logic mat_mul_start, mat_mul_done;
    fp_t mat_mul_A [STATE_DIM][STATE_DIM];
    fp_t mat_mul_B [STATE_DIM][STATE_DIM];
    fp_t mat_mul_C [STATE_DIM][STATE_DIM];
    
    matrix_multiply_4x4 g_mat_mul (
        .clk(clk),
        .rst_n(rst_n),
        .start(mat_mul_start),
        .A(mat_mul_A),
        .B(mat_mul_B),
        .C(mat_mul_C),
        .done(mat_mul_done)
    );
    
    // Matrix inverse for Pp^-1
    logic mat_inv_start, mat_inv_done, mat_inv_singular;
    fp_t mat_inv_A [STATE_DIM][STATE_DIM];
    fp_t mat_inv_result [STATE_DIM][STATE_DIM];
    
    matrix_inverse_4x4 pp_inverse (
        .clk(clk),
        .rst_n(rst_n),
        .start(mat_inv_start),
        .A(mat_inv_A),
        .A_inv(mat_inv_result),
        .done(mat_inv_done),
        .singular(mat_inv_singular)
    );
    
    // Matrix-vector multiply for correction
    logic matvec_start, matvec_done;
    fp_t matvec_M [STATE_DIM][STATE_DIM];
    fp_t matvec_x [STATE_DIM];
    fp_t matvec_y [STATE_DIM];
    
    matrix_vector_mult g_matvec (
        .clk(clk),
        .rst_n(rst_n),
        .start(matvec_start),
        .M(matvec_M),
        .x(matvec_x),
        .y(matvec_y),
        .done(matvec_done)
    );
    
    //--------------------------------------------------------------------------
    // Working Registers
    //--------------------------------------------------------------------------
    // Per-model smoothed states
    fp_t xs [N_MODELS][STATE_DIM];
    
    // Intermediate matrices
    fp_t F_T [STATE_DIM][STATE_DIM];           // F transposed
    fp_t Pf_FT [STATE_DIM][STATE_DIM];         // Pf @ F.T
    fp_t Pp_inv [STATE_DIM][STATE_DIM];        // inv(Pp)
    fp_t G [STATE_DIM][STATE_DIM];             // Smoother gain
    fp_t innovation [STATE_DIM];               // xs[k+1] - xp[k+1]
    fp_t correction [STATE_DIM];               // G @ innovation
    
    // Current processing indices
    logic [PTR_BITS-1:0] smooth_idx;
    logic [1:0] model_idx;
    logic [PTR_BITS-1:0] next_idx;
    
    // F matrix register (loaded per model)
    fp_t curr_F [STATE_DIM][STATE_DIM];
    
    //--------------------------------------------------------------------------
    // FSM
    //--------------------------------------------------------------------------
    typedef enum logic [4:0] {
        S_IDLE,
        S_BUFFER_WRITE,
        S_START_SMOOTH,
        S_LOAD_DATA,
        S_TRANSPOSE_F,
        S_START_PF_FT,      // Pf @ F.T
        S_WAIT_PF_FT,
        S_START_PP_INV,     // inv(Pp)
        S_WAIT_PP_INV,
        S_START_G,          // G = Pf_FT @ Pp_inv
        S_WAIT_G,
        S_COMPUTE_INNOV,    // innovation = xs[k+1] - xp[k+1]
        S_START_CORRECT,    // correction = G @ innovation
        S_WAIT_CORRECT,
        S_UPDATE_XS,        // xs[k] = xf[k] + correction
        S_NEXT_MODEL,
        S_NEXT_TIMESTEP,
        S_COMBINE,
        S_OUTPUT
    } state_t;
    
    state_t state, next_state;
    
    //--------------------------------------------------------------------------
    // Buffer Write (always active when data arrives)
    //--------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= '0;
            fill_count <= '0;
        end else if (fwd_valid && enable) begin
            buffer[wr_ptr].xf <= fwd_xf;
            buffer[wr_ptr].Pf <= fwd_Pf;
            buffer[wr_ptr].xp <= fwd_xp;
            buffer[wr_ptr].Pp <= fwd_Pp;
            buffer[wr_ptr].mu <= fwd_mu;
            
            wr_ptr <= (wr_ptr == LAG_DEPTH - 1) ? '0 : wr_ptr + 1;
            
            if (fill_count < LAG_DEPTH)
                fill_count <= fill_count + 1;
        end
    end
    
    //--------------------------------------------------------------------------
    // Compute next_idx (circular buffer)
    //--------------------------------------------------------------------------
    always_comb begin
        next_idx = (smooth_idx == LAG_DEPTH - 1) ? '0 : smooth_idx + 1;
    end
    
    //--------------------------------------------------------------------------
    // FSM Sequential
    //--------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            smooth_valid <= 1'b0;
            mat_mul_start <= 1'b0;
            mat_inv_start <= 1'b0;
            matvec_start <= 1'b0;
            rd_ptr <= '0;
            smooth_idx <= '0;
            model_idx <= '0;
            
            for (int m = 0; m < N_MODELS; m++)
                for (int i = 0; i < STATE_DIM; i++) begin
                    xs[m][i] <= '0;
                    x_smooth_model[m][i] <= '0;
                end
            for (int i = 0; i < STATE_DIM; i++)
                x_smooth[i] <= '0;
        end else begin
            state <= next_state;
            smooth_valid <= 1'b0;
            mat_mul_start <= 1'b0;
            mat_inv_start <= 1'b0;
            matvec_start <= 1'b0;
            
            case (state)
                //--------------------------------------------------------------
                // Idle: Wait for buffer full and new data
                //--------------------------------------------------------------
                S_IDLE: begin
                    // Ready to start smoothing when buffer full
                end
                
                //--------------------------------------------------------------
                // Start Smoothing: Initialize with last filtered state
                //--------------------------------------------------------------
                S_START_SMOOTH: begin
                    // Initialize: xs[T-1] = xf[T-1] for all models
                    automatic logic [PTR_BITS-1:0] last_idx;
                    last_idx = (wr_ptr == 0) ? LAG_DEPTH - 1 : wr_ptr - 1;
                    
                    for (int m = 0; m < N_MODELS; m++)
                        xs[m] <= buffer[last_idx].xf[m];
                    
                    smooth_idx <= (last_idx == 0) ? LAG_DEPTH - 1 : last_idx - 1;
                    model_idx <= '0;
                    rd_ptr <= (wr_ptr == 0) ? LAG_DEPTH - 1 : wr_ptr - 1;
                end
                
                //--------------------------------------------------------------
                // Load Data: Get Pf, xf, Pp, xp for current timestep
                //--------------------------------------------------------------
                S_LOAD_DATA: begin
                    // Load current model's F matrix
                    curr_F <= F_mat[model_idx];
                end
                
                //--------------------------------------------------------------
                // Transpose F
                //--------------------------------------------------------------
                S_TRANSPOSE_F: begin
                    for (int i = 0; i < STATE_DIM; i++)
                        for (int j = 0; j < STATE_DIM; j++)
                            F_T[i][j] <= curr_F[j][i];
                end
                
                //--------------------------------------------------------------
                // Start: Pf @ F.T
                //--------------------------------------------------------------
                S_START_PF_FT: begin
                    mat_mul_A <= buffer[smooth_idx].Pf[model_idx];
                    mat_mul_B <= F_T;
                    mat_mul_start <= 1'b1;
                end
                
                S_WAIT_PF_FT: begin
                    if (mat_mul_done)
                        Pf_FT <= mat_mul_C;
                end
                
                //--------------------------------------------------------------
                // Start: inv(Pp[k+1])
                //--------------------------------------------------------------
                S_START_PP_INV: begin
                    // Add regularization
                    for (int i = 0; i < STATE_DIM; i++) begin
                        for (int j = 0; j < STATE_DIM; j++) begin
                            mat_inv_A[i][j] <= buffer[next_idx].Pp[model_idx][i][j];
                            if (i == j)
                                mat_inv_A[i][j] <= buffer[next_idx].Pp[model_idx][i][j] + FP_EPS;
                        end
                    end
                    mat_inv_start <= 1'b1;
                end
                
                S_WAIT_PP_INV: begin
                    if (mat_inv_done)
                        Pp_inv <= mat_inv_result;
                end
                
                //--------------------------------------------------------------
                // Start: G = Pf_FT @ Pp_inv
                //--------------------------------------------------------------
                S_START_G: begin
                    mat_mul_A <= Pf_FT;
                    mat_mul_B <= Pp_inv;
                    mat_mul_start <= 1'b1;
                end
                
                S_WAIT_G: begin
                    if (mat_mul_done)
                        G <= mat_mul_C;
                end
                
                //--------------------------------------------------------------
                // Compute Innovation: xs[k+1] - xp[k+1]
                //--------------------------------------------------------------
                S_COMPUTE_INNOV: begin
                    for (int i = 0; i < STATE_DIM; i++)
                        innovation[i] <= xs[model_idx][i] - buffer[next_idx].xp[model_idx][i];
                end
                
                //--------------------------------------------------------------
                // Start: correction = G @ innovation
                //--------------------------------------------------------------
                S_START_CORRECT: begin
                    matvec_M <= G;
                    matvec_x <= innovation;
                    matvec_start <= 1'b1;
                end
                
                S_WAIT_CORRECT: begin
                    if (matvec_done)
                        correction <= matvec_y;
                end
                
                //--------------------------------------------------------------
                // Update: xs[k] = xf[k] + correction
                //--------------------------------------------------------------
                S_UPDATE_XS: begin
                    for (int i = 0; i < STATE_DIM; i++)
                        xs[model_idx][i] <= buffer[smooth_idx].xf[model_idx][i] + correction[i];
                end
                
                //--------------------------------------------------------------
                // Next Model
                //--------------------------------------------------------------
                S_NEXT_MODEL: begin
                    if (model_idx == N_MODELS - 1) begin
                        model_idx <= '0;
                    end else begin
                        model_idx <= model_idx + 1;
                    end
                end
                
                //--------------------------------------------------------------
                // Next Timestep
                //--------------------------------------------------------------
                S_NEXT_TIMESTEP: begin
                    if (smooth_idx != rd_ptr) begin
                        smooth_idx <= (smooth_idx == 0) ? LAG_DEPTH - 1 : smooth_idx - 1;
                    end
                end
                
                //--------------------------------------------------------------
                // Combine: x_smooth = sum(mu[j] * xs[j])
                //--------------------------------------------------------------
                S_COMBINE: begin
                    for (int i = 0; i < STATE_DIM; i++) begin
                        fp_t sum;
                        sum = FP_ZERO;
                        for (int m = 0; m < N_MODELS; m++) begin
                            sum = sum + fp_mul(buffer[rd_ptr].mu[m], xs[m][i]);
                            x_smooth_model[m][i] <= xs[m][i];
                        end
                        x_smooth[i] <= sum;
                    end
                    
                    // Advance read pointer
                    rd_ptr <= (rd_ptr == LAG_DEPTH - 1) ? '0 : rd_ptr + 1;
                end
                
                //--------------------------------------------------------------
                // Output
                //--------------------------------------------------------------
                S_OUTPUT: begin
                    smooth_valid <= 1'b1;
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
            S_IDLE: begin
                if (buffer_full && fwd_valid && enable)
                    next_state = S_START_SMOOTH;
            end
            
            S_START_SMOOTH:  next_state = S_LOAD_DATA;
            S_LOAD_DATA:     next_state = S_TRANSPOSE_F;
            S_TRANSPOSE_F:   next_state = S_START_PF_FT;
            S_START_PF_FT:   next_state = S_WAIT_PF_FT;
            S_WAIT_PF_FT:    if (mat_mul_done) next_state = S_START_PP_INV;
            S_START_PP_INV:  next_state = S_WAIT_PP_INV;
            S_WAIT_PP_INV:   if (mat_inv_done) next_state = S_START_G;
            S_START_G:       next_state = S_WAIT_G;
            S_WAIT_G:        if (mat_mul_done) next_state = S_COMPUTE_INNOV;
            S_COMPUTE_INNOV: next_state = S_START_CORRECT;
            S_START_CORRECT: next_state = S_WAIT_CORRECT;
            S_WAIT_CORRECT:  if (matvec_done) next_state = S_UPDATE_XS;
            S_UPDATE_XS:     next_state = S_NEXT_MODEL;
            
            S_NEXT_MODEL: begin
                if (model_idx == N_MODELS - 1)
                    next_state = S_NEXT_TIMESTEP;
                else
                    next_state = S_LOAD_DATA;
            end
            
            S_NEXT_TIMESTEP: begin
                if (smooth_idx == rd_ptr)
                    next_state = S_COMBINE;
                else
                    next_state = S_LOAD_DATA;
            end
            
            S_COMBINE:  next_state = S_OUTPUT;
            S_OUTPUT:   next_state = S_IDLE;
        endcase
    end

endmodule
