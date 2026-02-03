//==============================================================================
// QEDMMA v3.1 — Fixed-Lag IMM Smoother
// [REQ-RTL-01] Per-model RTS with circular buffer
// [REQ-RTL-02] Configurable lag depth (default 50 samples)
// [REQ-RTL-03] Real-time output with LAG_DEPTH latency
//==============================================================================
// Author: Dr. Mladen Mešter / Nexellum d.o.o.
// License: Commercial (qedmma-pro)
//==============================================================================

`timescale 1ns/1ps

module imm_fixed_lag_smoother #(
    parameter int STATE_DIM  = 4,           // [x, y, vx, vy]
    parameter int N_MODELS   = 3,           // CV, CT+, CT-
    parameter int LAG_DEPTH  = 50,          // Smoothing lag samples
    parameter int DATA_WIDTH = 32,          // Fixed-point width
    parameter int FRAC_BITS  = 16           // Fractional bits
)(
    input  logic                    clk,
    input  logic                    rst_n,
    
    // Forward pass interface (from IMM filter)
    input  logic                    fwd_valid,
    output logic                    fwd_ready,
    input  logic [DATA_WIDTH-1:0]   fwd_xf [N_MODELS][STATE_DIM],    // Filtered states
    input  logic [DATA_WIDTH-1:0]   fwd_Pf [N_MODELS][STATE_DIM][STATE_DIM],
    input  logic [DATA_WIDTH-1:0]   fwd_xp [N_MODELS][STATE_DIM],    // Predicted (from mixed)
    input  logic [DATA_WIDTH-1:0]   fwd_Pp [N_MODELS][STATE_DIM][STATE_DIM],
    input  logic [DATA_WIDTH-1:0]   fwd_mu [N_MODELS],               // Mode probabilities
    
    // Model F matrices (preloaded)
    input  logic [DATA_WIDTH-1:0]   F_mat [N_MODELS][STATE_DIM][STATE_DIM],
    
    // Smoothed output
    output logic                    out_valid,
    output logic [DATA_WIDTH-1:0]   out_xs [STATE_DIM],              // Combined smoothed
    output logic [DATA_WIDTH-1:0]   out_xs_model [N_MODELS][STATE_DIM]
);

    //--------------------------------------------------------------------------
    // Circular Buffer for Forward Data
    //--------------------------------------------------------------------------
    typedef struct packed {
        logic [DATA_WIDTH-1:0] xf [N_MODELS][STATE_DIM];
        logic [DATA_WIDTH-1:0] Pf [N_MODELS][STATE_DIM][STATE_DIM];
        logic [DATA_WIDTH-1:0] xp [N_MODELS][STATE_DIM];
        logic [DATA_WIDTH-1:0] Pp [N_MODELS][STATE_DIM][STATE_DIM];
        logic [DATA_WIDTH-1:0] mu [N_MODELS];
    } forward_data_t;
    
    forward_data_t buffer [LAG_DEPTH];
    
    logic [$clog2(LAG_DEPTH)-1:0] wr_ptr, rd_ptr;
    logic [$clog2(LAG_DEPTH)-1:0] fill_count;
    logic buffer_full;
    
    assign buffer_full = (fill_count == LAG_DEPTH);
    assign fwd_ready = 1'b1;  // Always ready (overwrite oldest)
    
    //--------------------------------------------------------------------------
    // Buffer Write (Forward Data Input)
    //--------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= '0;
            fill_count <= '0;
        end else if (fwd_valid) begin
            buffer[wr_ptr].xf <= fwd_xf;
            buffer[wr_ptr].Pf <= fwd_Pf;
            buffer[wr_ptr].xp <= fwd_xp;
            buffer[wr_ptr].Pp <= fwd_Pp;
            buffer[wr_ptr].mu <= fwd_mu;
            
            wr_ptr <= (wr_ptr == LAG_DEPTH-1) ? '0 : wr_ptr + 1;
            
            if (fill_count < LAG_DEPTH)
                fill_count <= fill_count + 1;
        end
    end
    
    //--------------------------------------------------------------------------
    // RTS Smoother State Machine
    //--------------------------------------------------------------------------
    typedef enum logic [2:0] {
        S_IDLE,
        S_INIT_LAST,
        S_COMPUTE_G,
        S_SMOOTH_STATE,
        S_COMBINE,
        S_OUTPUT
    } state_t;
    
    state_t state, next_state;
    
    logic [$clog2(LAG_DEPTH)-1:0] smooth_idx;
    logic [$clog2(N_MODELS)-1:0] model_idx;
    
    // Per-model smoothed states (working registers)
    logic [DATA_WIDTH-1:0] xs_work [N_MODELS][STATE_DIM];
    
    // Smoother gain G (4x4 for current model)
    logic [DATA_WIDTH-1:0] G_mat [STATE_DIM][STATE_DIM];
    
    // Intermediate computation registers
    logic [DATA_WIDTH-1:0] innovation [STATE_DIM];
    logic [DATA_WIDTH-1:0] correction [STATE_DIM];
    
    //--------------------------------------------------------------------------
    // FSM Sequential
    //--------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            out_valid <= 1'b0;
        end else begin
            state <= next_state;
        end
    end
    
    //--------------------------------------------------------------------------
    // FSM Combinational
    //--------------------------------------------------------------------------
    always_comb begin
        next_state = state;
        
        case (state)
            S_IDLE: begin
                if (buffer_full && fwd_valid)
                    next_state = S_INIT_LAST;
            end
            
            S_INIT_LAST: begin
                next_state = S_COMPUTE_G;
            end
            
            S_COMPUTE_G: begin
                // Wait for G computation (pipelined matrix ops)
                next_state = S_SMOOTH_STATE;
            end
            
            S_SMOOTH_STATE: begin
                if (model_idx == N_MODELS - 1) begin
                    if (smooth_idx == 0)
                        next_state = S_COMBINE;
                    else
                        next_state = S_COMPUTE_G;
                end
            end
            
            S_COMBINE: begin
                next_state = S_OUTPUT;
            end
            
            S_OUTPUT: begin
                next_state = S_IDLE;
            end
        endcase
    end
    
    //--------------------------------------------------------------------------
    // Smoother Index Management
    //--------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            smooth_idx <= '0;
            model_idx <= '0;
        end else begin
            case (state)
                S_INIT_LAST: begin
                    // Start from T-1 (newest in buffer)
                    smooth_idx <= (wr_ptr == 0) ? LAG_DEPTH - 1 : wr_ptr - 1;
                    model_idx <= '0;
                    
                    // Initialize xs at T-1 = xf at T-1
                    for (int m = 0; m < N_MODELS; m++)
                        xs_work[m] <= buffer[(wr_ptr == 0) ? LAG_DEPTH-1 : wr_ptr-1].xf[m];
                end
                
                S_SMOOTH_STATE: begin
                    if (model_idx == N_MODELS - 1) begin
                        model_idx <= '0;
                        smooth_idx <= (smooth_idx == 0) ? LAG_DEPTH - 1 : smooth_idx - 1;
                    end else begin
                        model_idx <= model_idx + 1;
                    end
                end
            endcase
        end
    end
    
    //--------------------------------------------------------------------------
    // RTS Computation (Simplified — production needs DSP pipeline)
    //--------------------------------------------------------------------------
    // G = Pf[k] @ F.T @ inv(Pp[k+1])
    // xs[k] = xf[k] + G @ (xs[k+1] - xp[k+1])
    
    // For synthesis, this would be a multi-cycle pipelined operation
    // Here we show the structure; actual DSP instantiation in production
    
    logic [$clog2(LAG_DEPTH)-1:0] next_idx;
    assign next_idx = (smooth_idx == LAG_DEPTH - 1) ? 0 : smooth_idx + 1;
    
    always_ff @(posedge clk) begin
        if (state == S_SMOOTH_STATE) begin
            // Innovation: xs[k+1] - xp[k+1]
            for (int i = 0; i < STATE_DIM; i++) begin
                innovation[i] <= xs_work[model_idx][i] - buffer[next_idx].xp[model_idx][i];
            end
            
            // Correction: G @ innovation (requires matrix multiply)
            // Placeholder — actual impl uses DSP48 cascade
            for (int i = 0; i < STATE_DIM; i++) begin
                correction[i] <= innovation[i];  // Simplified
            end
            
            // Update smoothed state
            for (int i = 0; i < STATE_DIM; i++) begin
                xs_work[model_idx][i] <= buffer[smooth_idx].xf[model_idx][i] + correction[i];
            end
        end
    end
    
    //--------------------------------------------------------------------------
    // Output Combination (weighted by forward mu)
    //--------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (state == S_COMBINE) begin
            for (int i = 0; i < STATE_DIM; i++) begin
                logic [2*DATA_WIDTH-1:0] sum;
                sum = '0;
                for (int m = 0; m < N_MODELS; m++) begin
                    sum += xs_work[m][i] * buffer[rd_ptr].mu[m];
                end
                out_xs[i] <= sum[DATA_WIDTH+FRAC_BITS-1:FRAC_BITS];
            end
            
            // Also output per-model
            out_xs_model <= xs_work;
        end
        
        out_valid <= (state == S_OUTPUT);
    end
    
    // Update read pointer for next output
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_ptr <= '0;
        end else if (state == S_OUTPUT) begin
            rd_ptr <= (rd_ptr == LAG_DEPTH - 1) ? '0 : rd_ptr + 1;
        end
    end

endmodule
