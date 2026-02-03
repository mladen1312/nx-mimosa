//==============================================================================
// NX-MIMOSA v3.1 Pro — 4x4 Matrix Multiply (Pipelined)
// [REQ-RTL-MAT-01] C = A @ B with DSP48E2 inference
// [REQ-RTL-MAT-02] 4-cycle latency, fully pipelined
//==============================================================================
// Author: Dr. Mladen Mešter / Nexellum d.o.o.
//==============================================================================

`timescale 1ns/1ps

module matrix_multiply_4x4
    import nx_mimosa_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  fp_t         A [STATE_DIM][STATE_DIM],
    input  fp_t         B [STATE_DIM][STATE_DIM],
    output fp_t         C [STATE_DIM][STATE_DIM],
    output logic        done
);

    //--------------------------------------------------------------------------
    // Pipeline Registers
    //--------------------------------------------------------------------------
    // Stage 0: Input capture
    // Stage 1-4: MAC operations (4 columns in parallel)
    
    typedef enum logic [2:0] {
        S_IDLE,
        S_ROW0,
        S_ROW1,
        S_ROW2,
        S_ROW3,
        S_DONE
    } state_t;
    
    state_t state, next_state;
    
    // Registered inputs
    fp_t A_reg [STATE_DIM][STATE_DIM];
    fp_t B_reg [STATE_DIM][STATE_DIM];
    
    // Accumulator for each output element
    logic signed [2*DATA_WIDTH-1:0] acc [STATE_DIM][STATE_DIM];
    
    // Current row being computed
    logic [1:0] row_idx;
    
    //--------------------------------------------------------------------------
    // FSM Sequential
    //--------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            row_idx <= '0;
            done <= 1'b0;
            for (int i = 0; i < STATE_DIM; i++)
                for (int j = 0; j < STATE_DIM; j++)
                    C[i][j] <= '0;
        end else begin
            state <= next_state;
            done <= 1'b0;
            
            case (state)
                S_IDLE: begin
                    if (start) begin
                        // Capture inputs
                        A_reg <= A;
                        B_reg <= B;
                        row_idx <= '0;
                        // Clear accumulators
                        for (int i = 0; i < STATE_DIM; i++)
                            for (int j = 0; j < STATE_DIM; j++)
                                acc[i][j] <= '0;
                    end
                end
                
                S_ROW0, S_ROW1, S_ROW2, S_ROW3: begin
                    // Compute one row: C[row_idx][j] = sum_k(A[row_idx][k] * B[k][j])
                    for (int j = 0; j < STATE_DIM; j++) begin
                        logic signed [2*DATA_WIDTH-1:0] sum;
                        sum = '0;
                        for (int k = 0; k < STATE_DIM; k++) begin
                            sum = sum + (A_reg[row_idx][k] * B_reg[k][j]);
                        end
                        // Store with fixed-point scaling
                        C[row_idx][j] <= sum[DATA_WIDTH+FRAC_BITS-1:FRAC_BITS];
                    end
                    row_idx <= row_idx + 1;
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
            S_IDLE: if (start) next_state = S_ROW0;
            S_ROW0: next_state = S_ROW1;
            S_ROW1: next_state = S_ROW2;
            S_ROW2: next_state = S_ROW3;
            S_ROW3: next_state = S_DONE;
            S_DONE: next_state = S_IDLE;
        endcase
    end

endmodule
