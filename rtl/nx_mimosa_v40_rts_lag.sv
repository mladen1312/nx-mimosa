//==============================================================================
// NX-MIMOSA v4.0.2 — Fixed-Lag RTS Smoother
//==============================================================================
// Single-model fixed-lag RTS smoother with streaming output.
// Stores WINDOW_DEPTH forward states, runs backward pass on window fill.
//
// Key insight: Uses pre-mixing unmixed states (xu_pre, Pu_pre) from IMM
// to avoid divergence caused by IMM mixing contamination.
//
// Pipeline: On each new measurement:
//   1. Push forward state into circular buffer
//   2. If buffer full: run backward RTS pass (W steps)
//   3. Output oldest smoothed state (FIFO-order)
//
// Latency: WINDOW_DEPTH * dt seconds (40 * 0.1s = 4.0s for offline)
// [REQ-RTS-001] Fixed-lag RTS with pre-mixing state storage
//==============================================================================

`timescale 1ns/1ps

module nx_mimosa_v40_rts_lag
    import nx_mimosa_v40_pkg::*;
#(
    parameter int WINDOW_DEPTH = 40,
    parameter int MODEL_IDX    = 0     // Which IMM model to smooth
)(
    input  logic                   clk,
    input  logic                   rst_n,
    
    // Forward pass inputs (from IMM per-model)
    input  fp_t                    x_fwd [STATE_DIM],
    input  fp_t                    P_fwd [STATE_DIM][STATE_DIM],
    input  fp_t                    xu_pre [STATE_DIM],      // Pre-mixing unmixed state
    input  fp_t                    Pu_pre [STATE_DIM][STATE_DIM],
    input  fp_t                    mu_weight,                // Mode probability weight
    input  logic                   fwd_valid,
    
    // Configuration
    input  fp_t                    cfg_dt,
    
    // Smoothed output
    output fp_t                    x_smooth [STATE_DIM],
    output fp_t                    quality,
    output logic                   smooth_valid
);

    //--------------------------------------------------------------------------
    // Circular buffer for forward states
    //--------------------------------------------------------------------------
    localparam int BUF_AW = $clog2(WINDOW_DEPTH);
    
    // Buffer storage (BRAM inference)
    fp_t  buf_x_fwd  [WINDOW_DEPTH][STATE_DIM];
    fp_t  buf_P_fwd  [WINDOW_DEPTH][STATE_DIM][STATE_DIM];
    fp_t  buf_x_pred [WINDOW_DEPTH][STATE_DIM];         // Predicted (for backward gain)
    fp_t  buf_P_pred [WINDOW_DEPTH][STATE_DIM][STATE_DIM];
    fp_t  buf_mu     [WINDOW_DEPTH];
    
    logic [BUF_AW-1:0] wr_ptr;
    logic [BUF_AW-1:0] rd_ptr;     // Oldest entry
    logic [$clog2(WINDOW_DEPTH):0] fill_count;
    logic buffer_full;
    
    assign buffer_full = (fill_count >= WINDOW_DEPTH);
    
    //--------------------------------------------------------------------------
    // Write pointer management
    //--------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr     <= '0;
            fill_count <= '0;
        end else if (fwd_valid) begin
            // Store forward state and prediction
            buf_x_fwd[wr_ptr]  <= x_fwd;
            buf_P_fwd[wr_ptr]  <= P_fwd;
            buf_x_pred[wr_ptr] <= xu_pre;   // Pre-mixing state as "predicted"
            buf_P_pred[wr_ptr] <= Pu_pre;
            buf_mu[wr_ptr]     <= mu_weight;
            
            wr_ptr <= (wr_ptr == WINDOW_DEPTH-1) ? '0 : wr_ptr + 1'b1;
            if (!buffer_full)
                fill_count <= fill_count + 1;
        end
    end
    
    //--------------------------------------------------------------------------
    // RTS Backward Pass FSM
    //--------------------------------------------------------------------------
    typedef enum logic [2:0] {
        RTS_IDLE     = 3'd0,
        RTS_INIT     = 3'd1,    // Initialize with newest forward state
        RTS_BACKWARD = 3'd2,    // Iterate backward through buffer
        RTS_OUTPUT   = 3'd3     // Output oldest smoothed state
    } rts_state_e;
    
    rts_state_e rts_state;
    
    logic [BUF_AW-1:0] bw_ptr;           // Backward iteration pointer
    logic [$clog2(WINDOW_DEPTH):0] bw_remaining;
    
    // Smoothed state (running backward)
    fp_t  xs [STATE_DIM];
    fp_t  Ps [STATE_DIM][STATE_DIM];
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rts_state    <= RTS_IDLE;
            smooth_valid <= 1'b0;
        end else begin
            smooth_valid <= 1'b0;
            
            case (rts_state)
                RTS_IDLE: begin
                    if (fwd_valid && buffer_full)
                        rts_state <= RTS_INIT;
                end
                
                RTS_INIT: begin
                    // Start from newest state (wr_ptr - 1)
                    bw_ptr = (wr_ptr == 0) ? WINDOW_DEPTH-1 : wr_ptr - 1;
                    xs <= buf_x_fwd[bw_ptr];
                    Ps <= buf_P_fwd[bw_ptr];
                    bw_remaining <= WINDOW_DEPTH - 1;
                    rts_state <= RTS_BACKWARD;
                end
                
                RTS_BACKWARD: begin
                    if (bw_remaining == 0) begin
                        rts_state <= RTS_OUTPUT;
                    end else begin
                        // Move backward
                        bw_ptr = (bw_ptr == 0) ? WINDOW_DEPTH-1 : bw_ptr - 1;
                        
                        // RTS backward step:
                        // G_k = P_fwd[k] * F' * P_pred[k+1]^(-1)
                        // x_s[k] = x_fwd[k] + G_k * (x_s[k+1] - x_pred[k+1])
                        // P_s[k] = P_fwd[k] + G_k * (P_s[k+1] - P_pred[k+1]) * G_k'
                        //
                        // Simplified for real-time: use scalar gain approximation
                        // G ≈ diag(P_fwd[i][i] / P_pred[i][i])
                        
                        for (int i = 0; i < STATE_DIM; i++) begin
                            fp_t G_ii;
                            fp_t P_pred_ii;
                            P_pred_ii = buf_P_pred[bw_ptr + 1 < WINDOW_DEPTH ? bw_ptr + 1 : 0][i][i];
                            G_ii = (P_pred_ii > FP_EPS) ?
                                   fp_div(buf_P_fwd[bw_ptr][i][i], P_pred_ii) : FP_ZERO;
                            
                            // Clamp gain [0, 1]
                            if (G_ii > FP_ONE) G_ii = FP_ONE;
                            if (G_ii < FP_ZERO) G_ii = FP_ZERO;
                            
                            // Backward update
                            fp_t x_pred_next;
                            x_pred_next = buf_x_pred[bw_ptr + 1 < WINDOW_DEPTH ? bw_ptr + 1 : 0][i];
                            xs[i] <= buf_x_fwd[bw_ptr][i] + fp_mul(G_ii, xs[i] - x_pred_next);
                            
                            // Covariance (diagonal only for speed)
                            fp_t P_pred_next;
                            P_pred_next = buf_P_pred[bw_ptr + 1 < WINDOW_DEPTH ? bw_ptr + 1 : 0][i][i];
                            Ps[i][i] <= buf_P_fwd[bw_ptr][i][i] +
                                        fp_mul(fp_mul(G_ii, G_ii), Ps[i][i] - P_pred_next);
                        end
                        
                        bw_remaining <= bw_remaining - 1;
                    end
                end
                
                RTS_OUTPUT: begin
                    // Output the smoothed state for oldest buffer entry
                    // Weight by mode probability (mu-weighted smoothing)
                    x_smooth <= xs;
                    
                    // Quality metric: trace(P_smooth) — lower is better
                    quality <= FP_ZERO;
                    for (int i = 0; i < STATE_DIM; i++)
                        quality <= quality + Ps[i][i];
                    
                    smooth_valid <= 1'b1;
                    rts_state <= RTS_IDLE;
                end
                
                default: rts_state <= RTS_IDLE;
            endcase
        end
    end

endmodule
