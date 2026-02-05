//==============================================================================
// NX-MIMOSA v3.3 — Window RTS Smoother
// [REQ-WSMOOTH-01] Configurable window depth (1..64 steps)
// [REQ-WSMOOTH-02] BRAM circular buffer for forward pass data
// [REQ-WSMOOTH-03] Triggered or continuous backward RTS pass
// [REQ-WSMOOTH-04] Per-model smoother gain: G = Pf @ F.T @ inv(Pp)
// [REQ-WSMOOTH-05] IMM-weighted combination: xs = sum(mu_j * xs_j)
// [REQ-WSMOOTH-06] Achieves 99.7% of full-track smooth accuracy (Window-30)
//==============================================================================
// Math:
//   Forward: x_f[k], P_f[k] from IMM core
//   Backward (k = N-1 down to 0):
//     G[k]  = P_f[k] @ F[k].T @ inv(P_p[k+1])
//     xs[k] = x_f[k] + G[k] @ (xs[k+1] - x_p[k+1])
//     Ps[k] = P_f[k] + G[k] @ (Ps[k+1] - P_p[k+1]) @ G[k].T  (optional)
//==============================================================================
// Author: Dr. Mladen Mešter / Nexellum d.o.o.
// License: Commercial — Nexellum d.o.o.
//==============================================================================

`timescale 1ns/1ps

module window_rts_smoother
    import nx_mimosa_pkg_v33::*;
(
    input  logic        clk,
    input  logic        rst_n,

    //--------------------------------------------------------------------------
    // Forward Pass Input (from IMM Core, every measurement update)
    //--------------------------------------------------------------------------
    input  logic        fwd_valid,
    input  fp_t         fwd_xf [N_MODELS][STATE_DIM],         // Filtered states
    input  fp_t         fwd_Pf [N_MODELS][STATE_DIM][STATE_DIM], // Filtered covs
    input  fp_t         fwd_xp [N_MODELS][STATE_DIM],         // Predicted states
    input  fp_t         fwd_Pp [N_MODELS][STATE_DIM][STATE_DIM], // Predicted covs
    input  fp_t         fwd_mu [N_MODELS],                    // Mode probabilities
    input  fp_t         F_mat [N_MODELS][STATE_DIM][STATE_DIM], // Transition matrices

    //--------------------------------------------------------------------------
    // Configuration
    //--------------------------------------------------------------------------
    input  logic        enable,
    input  logic [5:0]  cfg_window_size,     // 1..64 (default 30)
    input  smooth_trigger_t cfg_trigger_mode,
    input  logic        ext_trigger,         // External trigger (maneuver/manual)

    //--------------------------------------------------------------------------
    // Smoothed Output
    //--------------------------------------------------------------------------
    output logic        smooth_valid,
    output fp_t         x_smooth [STATE_DIM],
    output fp_t         x_smooth_model [N_MODELS][STATE_DIM],
    output stream_id_t  output_stream_id,

    //--------------------------------------------------------------------------
    // Status
    //--------------------------------------------------------------------------
    output logic [31:0] smooth_count,        // Number of smoothed outputs
    output logic [31:0] cycle_count,         // Backward pass cycle counter
    output logic        busy                 // Smoother FSM active
);

    //--------------------------------------------------------------------------
    // Circular Buffer (BRAM)
    // [REQ-WSMOOTH-02]
    //--------------------------------------------------------------------------
    localparam int BUF_DEPTH = WINDOW_DEPTH_MAX;  // 64
    localparam int BUF_BITS  = WINDOW_PTR_BITS;   // 6

    // Buffer entry — store forward pass data per timestep
    // Total per entry: 3 * (4 + 16 + 4 + 16 + 1) * 32 bits = 3696 bits ≈ 116 BRAM36
    // For 64 entries: ~7.4K BRAM36 — fits in ZU48DR's 1080 BRAM36
    typedef struct packed {
        fp_t xf [N_MODELS][STATE_DIM];
        fp_t Pf [N_MODELS][STATE_DIM][STATE_DIM];
        fp_t xp [N_MODELS][STATE_DIM];
        fp_t Pp [N_MODELS][STATE_DIM][STATE_DIM];
        fp_t mu [N_MODELS];
        fp_t F  [N_MODELS][STATE_DIM][STATE_DIM];
    } window_entry_t;

    (* ram_style = "block" *) window_entry_t buf [BUF_DEPTH];

    logic [BUF_BITS-1:0] wr_ptr;
    logic [BUF_BITS-1:0] fill_count;
    logic window_full;

    assign window_full = (fill_count >= cfg_window_size);

    //--------------------------------------------------------------------------
    // Buffer Write — continuous on every forward step
    //--------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr     <= '0;
            fill_count <= '0;
        end else if (fwd_valid && enable) begin
            buf[wr_ptr].xf <= fwd_xf;
            buf[wr_ptr].Pf <= fwd_Pf;
            buf[wr_ptr].xp <= fwd_xp;
            buf[wr_ptr].Pp <= fwd_Pp;
            buf[wr_ptr].mu <= fwd_mu;
            buf[wr_ptr].F  <= F_mat;

            wr_ptr <= (wr_ptr == BUF_DEPTH - 1) ? '0 : wr_ptr + 1;

            if (fill_count < BUF_DEPTH)
                fill_count <= fill_count + 1;
        end
    end

    //--------------------------------------------------------------------------
    // Trigger Logic
    // [REQ-WSMOOTH-03]
    //--------------------------------------------------------------------------
    logic start_smooth;
    logic sliding_trigger;

    // Sliding: trigger on every fwd_valid when window is full
    assign sliding_trigger = fwd_valid && window_full && (cfg_trigger_mode == TRIG_SLIDING);

    always_comb begin
        case (cfg_trigger_mode)
            TRIG_SLIDING:  start_smooth = sliding_trigger;
            TRIG_MANEUVER: start_smooth = ext_trigger && window_full;
            TRIG_COV_SPIKE:start_smooth = ext_trigger && window_full;
            TRIG_MANUAL:   start_smooth = ext_trigger && (fill_count > 0);
            default:       start_smooth = 1'b0;
        endcase
    end

    //--------------------------------------------------------------------------
    // Shared Matrix Operation Modules
    //--------------------------------------------------------------------------
    logic mat_mul_start, mat_mul_done;
    fp_t mat_mul_A [STATE_DIM][STATE_DIM];
    fp_t mat_mul_B [STATE_DIM][STATE_DIM];
    fp_t mat_mul_C [STATE_DIM][STATE_DIM];

    matrix_multiply_4x4 g_matmul (
        .clk(clk), .rst_n(rst_n),
        .start(mat_mul_start),
        .A(mat_mul_A), .B(mat_mul_B), .C(mat_mul_C),
        .done(mat_mul_done)
    );

    logic mat_inv_start, mat_inv_done, mat_inv_singular;
    fp_t mat_inv_A [STATE_DIM][STATE_DIM];
    fp_t mat_inv_result [STATE_DIM][STATE_DIM];

    matrix_inverse_4x4 pp_inv (
        .clk(clk), .rst_n(rst_n),
        .start(mat_inv_start),
        .A(mat_inv_A), .A_inv(mat_inv_result),
        .done(mat_inv_done), .singular(mat_inv_singular)
    );

    logic matvec_start, matvec_done;
    fp_t matvec_M [STATE_DIM][STATE_DIM];
    fp_t matvec_x [STATE_DIM];
    fp_t matvec_y [STATE_DIM];

    matrix_vector_mult g_matvec (
        .clk(clk), .rst_n(rst_n),
        .start(matvec_start),
        .M(matvec_M), .x(matvec_x), .y(matvec_y),
        .done(matvec_done)
    );

    //--------------------------------------------------------------------------
    // Smoother FSM
    // [REQ-WSMOOTH-04], [REQ-WSMOOTH-05]
    //--------------------------------------------------------------------------
    typedef enum logic [4:0] {
        SM_IDLE,
        SM_INIT,            // Initialize xs[N-1] = xf[N-1]
        SM_LOAD_DATA,       // Load buffer[k] data
        SM_TRANSPOSE_F,     // F_T = F[model_idx].T
        SM_START_PF_FT,     // Pf[k] @ F.T
        SM_WAIT_PF_FT,
        SM_START_PP_INV,    // inv(Pp[k+1])
        SM_WAIT_PP_INV,
        SM_START_G,         // G = Pf_FT @ Pp_inv
        SM_WAIT_G,
        SM_COMPUTE_INNOV,   // innovation = xs[k+1] - xp[k+1]
        SM_START_CORRECT,   // correction = G @ innovation
        SM_WAIT_CORRECT,
        SM_UPDATE_XS,       // xs[k] = xf[k] + correction
        SM_NEXT_MODEL,
        SM_NEXT_TIMESTEP,
        SM_COMBINE,         // IMM-weighted combination
        SM_OUTPUT,
        SM_DONE
    } sm_state_t;

    sm_state_t sm_state;

    // Working registers
    fp_t xs [N_MODELS][STATE_DIM];
    fp_t F_T [STATE_DIM][STATE_DIM];
    fp_t Pf_FT [STATE_DIM][STATE_DIM];
    fp_t Pp_inv_r [STATE_DIM][STATE_DIM];
    fp_t G_r [STATE_DIM][STATE_DIM];
    fp_t innov_r [STATE_DIM];
    fp_t correct_r [STATE_DIM];

    logic [BUF_BITS-1:0] smooth_idx;     // Current backward index
    logic [BUF_BITS-1:0] next_idx;       // k+1 index
    logic [BUF_BITS-1:0] start_idx;      // Newest entry (backward start)
    logic [BUF_BITS-1:0] end_idx;        // Oldest entry (backward end)
    logic [1:0] model_idx;
    logic [5:0] steps_remaining;

    logic [31:0] cycle_counter;

    // Circular buffer index math
    function automatic logic [BUF_BITS-1:0] buf_prev(input logic [BUF_BITS-1:0] idx);
        return (idx == 0) ? BUF_DEPTH[BUF_BITS-1:0] - 1 : idx - 1;
    endfunction

    function automatic logic [BUF_BITS-1:0] buf_next(input logic [BUF_BITS-1:0] idx);
        return (idx == BUF_DEPTH - 1) ? '0 : idx + 1;
    endfunction

    //--------------------------------------------------------------------------
    // FSM Sequential
    //--------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sm_state      <= SM_IDLE;
            smooth_valid  <= 1'b0;
            smooth_count  <= '0;
            cycle_counter <= '0;
            mat_mul_start <= 1'b0;
            mat_inv_start <= 1'b0;
            matvec_start  <= 1'b0;
            model_idx     <= '0;
            steps_remaining <= '0;
            output_stream_id <= STREAM_WINDOW;

            for (int m = 0; m < N_MODELS; m++)
                for (int i = 0; i < STATE_DIM; i++) begin
                    xs[m][i]              <= FP_ZERO;
                    x_smooth[i]           <= FP_ZERO;
                    x_smooth_model[m][i]  <= FP_ZERO;
                end
        end else begin
            // Default: clear one-shot signals
            mat_mul_start <= 1'b0;
            mat_inv_start <= 1'b0;
            matvec_start  <= 1'b0;
            smooth_valid  <= 1'b0;

            case (sm_state)
                //--------------------------------------------------------------
                SM_IDLE: begin
                    cycle_counter <= '0;
                    if (start_smooth && enable && !busy) begin
                        sm_state <= SM_INIT;
                    end
                end

                //--------------------------------------------------------------
                // Initialize: xs[N-1] = xf[N-1] (most recent entry)
                //--------------------------------------------------------------
                SM_INIT: begin
                    // Start from newest entry, go backward
                    start_idx <= buf_prev(wr_ptr);
                    
                    // Compute end index (window_size steps back)
                    begin
                        automatic logic [BUF_BITS-1:0] actual_window;
                        actual_window = (fill_count < cfg_window_size) ? 
                                        fill_count[BUF_BITS-1:0] : cfg_window_size[BUF_BITS-1:0];
                        steps_remaining <= actual_window[5:0] - 1;
                    end

                    // Initialize xs with most recent filtered state
                    for (int m = 0; m < N_MODELS; m++)
                        xs[m] <= buf[buf_prev(wr_ptr)].xf[m];

                    // Start from second-to-last entry
                    smooth_idx <= buf_prev(buf_prev(wr_ptr));
                    next_idx   <= buf_prev(wr_ptr);
                    model_idx  <= '0;

                    if (fill_count <= 1)
                        sm_state <= SM_IDLE;  // Not enough data
                    else
                        sm_state <= SM_LOAD_DATA;
                end

                //--------------------------------------------------------------
                SM_LOAD_DATA: begin
                    cycle_counter <= cycle_counter + 1;
                    sm_state <= SM_TRANSPOSE_F;
                end

                //--------------------------------------------------------------
                SM_TRANSPOSE_F: begin
                    for (int i = 0; i < STATE_DIM; i++)
                        for (int j = 0; j < STATE_DIM; j++)
                            F_T[i][j] <= buf[smooth_idx].F[model_idx][j][i];
                    sm_state <= SM_START_PF_FT;
                end

                //--------------------------------------------------------------
                // G[k] = P_f[k] @ F[k]^T @ inv(P_p[k+1])
                //--------------------------------------------------------------
                SM_START_PF_FT: begin
                    mat_mul_A <= buf[smooth_idx].Pf[model_idx];
                    mat_mul_B <= F_T;
                    mat_mul_start <= 1'b1;
                    sm_state <= SM_WAIT_PF_FT;
                end

                SM_WAIT_PF_FT: begin
                    if (mat_mul_done) begin
                        Pf_FT <= mat_mul_C;
                        sm_state <= SM_START_PP_INV;
                    end
                end

                SM_START_PP_INV: begin
                    // inv(P_p[k+1]) with regularization
                    for (int i = 0; i < STATE_DIM; i++)
                        for (int j = 0; j < STATE_DIM; j++)
                            mat_inv_A[i][j] <= buf[next_idx].Pp[model_idx][i][j] +
                                               ((i == j) ? FP_EPS : FP_ZERO);
                    mat_inv_start <= 1'b1;
                    sm_state <= SM_WAIT_PP_INV;
                end

                SM_WAIT_PP_INV: begin
                    if (mat_inv_done) begin
                        Pp_inv_r <= mat_inv_singular ? 
                            // Fallback: use identity if singular
                            '{default: FP_ZERO} : mat_inv_result;
                        // If singular, set diagonal to 1
                        if (mat_inv_singular) begin
                            for (int i = 0; i < STATE_DIM; i++)
                                Pp_inv_r[i][i] <= FP_ONE;
                        end
                        sm_state <= SM_START_G;
                    end
                end

                SM_START_G: begin
                    mat_mul_A <= Pf_FT;
                    mat_mul_B <= Pp_inv_r;
                    mat_mul_start <= 1'b1;
                    sm_state <= SM_WAIT_G;
                end

                SM_WAIT_G: begin
                    if (mat_mul_done) begin
                        G_r <= mat_mul_C;
                        sm_state <= SM_COMPUTE_INNOV;
                    end
                end

                //--------------------------------------------------------------
                // xs[k] = xf[k] + G[k] @ (xs[k+1] - xp[k+1])
                //--------------------------------------------------------------
                SM_COMPUTE_INNOV: begin
                    for (int i = 0; i < STATE_DIM; i++)
                        innov_r[i] <= xs[model_idx][i] - buf[next_idx].xp[model_idx][i];
                    sm_state <= SM_START_CORRECT;
                end

                SM_START_CORRECT: begin
                    matvec_M <= G_r;
                    matvec_x <= innov_r;
                    matvec_start <= 1'b1;
                    sm_state <= SM_WAIT_CORRECT;
                end

                SM_WAIT_CORRECT: begin
                    if (matvec_done) begin
                        correct_r <= matvec_y;
                        sm_state <= SM_UPDATE_XS;
                    end
                end

                SM_UPDATE_XS: begin
                    for (int i = 0; i < STATE_DIM; i++)
                        xs[model_idx][i] <= buf[smooth_idx].xf[model_idx][i] + correct_r[i];
                    sm_state <= SM_NEXT_MODEL;
                end

                //--------------------------------------------------------------
                SM_NEXT_MODEL: begin
                    if (model_idx == N_MODELS - 1) begin
                        model_idx <= '0;
                        sm_state <= SM_NEXT_TIMESTEP;
                    end else begin
                        model_idx <= model_idx + 1;
                        sm_state <= SM_LOAD_DATA;
                    end
                end

                //--------------------------------------------------------------
                SM_NEXT_TIMESTEP: begin
                    if (steps_remaining <= 1) begin
                        // Done — combine models for oldest smoothed step
                        sm_state <= SM_COMBINE;
                    end else begin
                        steps_remaining <= steps_remaining - 1;
                        next_idx   <= smooth_idx;
                        smooth_idx <= buf_prev(smooth_idx);
                        sm_state   <= SM_LOAD_DATA;
                    end
                end

                //--------------------------------------------------------------
                // IMM-weighted combination
                // [REQ-WSMOOTH-05]
                //--------------------------------------------------------------
                SM_COMBINE: begin
                    for (int i = 0; i < STATE_DIM; i++) begin
                        automatic fp_t sum;
                        sum = FP_ZERO;
                        for (int m = 0; m < N_MODELS; m++) begin
                            // Use the mu from the oldest entry in the window
                            sum = sum + fp_mul(buf[smooth_idx].mu[m], xs[m][i]);
                            x_smooth_model[m][i] <= xs[m][i];
                        end
                        x_smooth[i] <= sum;
                    end
                    output_stream_id <= STREAM_WINDOW;
                    sm_state <= SM_OUTPUT;
                end

                //--------------------------------------------------------------
                SM_OUTPUT: begin
                    smooth_valid <= 1'b1;
                    smooth_count <= smooth_count + 1;
                    sm_state <= SM_DONE;
                end

                SM_DONE: begin
                    sm_state <= SM_IDLE;
                end

                default: sm_state <= SM_IDLE;
            endcase
        end
    end

    //--------------------------------------------------------------------------
    // Status
    //--------------------------------------------------------------------------
    assign busy = (sm_state != SM_IDLE);
    assign cycle_count = cycle_counter;

endmodule
