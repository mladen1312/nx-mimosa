//==============================================================================
// NX-MIMOSA v3.1 Pro — 4x4 Matrix Inverse (Gauss-Jordan)
// [REQ-RTL-INV-01] A_inv = inv(A) using row reduction
// [REQ-RTL-INV-02] 16-cycle latency (4 pivot + 12 elimination)
// [REQ-RTL-INV-03] Handles near-singular with regularization
//==============================================================================
// Author: Dr. Mladen Mešter / Nexellum d.o.o.
//==============================================================================

`timescale 1ns/1ps

module matrix_inverse_4x4
    import nx_mimosa_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  fp_t         A [STATE_DIM][STATE_DIM],
    output fp_t         A_inv [STATE_DIM][STATE_DIM],
    output logic        done,
    output logic        singular  // Set if matrix is singular
);

    //--------------------------------------------------------------------------
    // Augmented Matrix [A | I] stored as 4x8
    //--------------------------------------------------------------------------
    localparam int AUG_COLS = 2 * STATE_DIM;  // 8 columns
    
    // Extended precision for intermediate calculations
    localparam int EXT_WIDTH = DATA_WIDTH + FRAC_BITS;  // 48 bits
    
    logic signed [EXT_WIDTH-1:0] aug [STATE_DIM][AUG_COLS];
    
    //--------------------------------------------------------------------------
    // FSM States
    //--------------------------------------------------------------------------
    typedef enum logic [4:0] {
        S_IDLE,
        S_INIT,           // Initialize [A | I]
        S_PIVOT_0,        // Find and scale pivot row 0
        S_ELIM_0_1,       // Eliminate column 0 in rows 1,2,3
        S_ELIM_0_2,
        S_ELIM_0_3,
        S_PIVOT_1,        // Pivot row 1
        S_ELIM_1_0,       // Eliminate column 1 in rows 0,2,3
        S_ELIM_1_2,
        S_ELIM_1_3,
        S_PIVOT_2,        // Pivot row 2
        S_ELIM_2_0,       // Eliminate column 2 in rows 0,1,3
        S_ELIM_2_1,
        S_ELIM_2_3,
        S_PIVOT_3,        // Pivot row 3
        S_ELIM_3_0,       // Eliminate column 3 in rows 0,1,2
        S_ELIM_3_1,
        S_ELIM_3_2,
        S_EXTRACT,        // Extract inverse from right half
        S_DONE
    } state_t;
    
    state_t state, next_state;
    
    // Pivot value (with regularization)
    logic signed [EXT_WIDTH-1:0] pivot;
    logic signed [EXT_WIDTH-1:0] factor;
    
    // Current pivot column
    logic [1:0] piv_col;
    
    // Row being modified
    logic [1:0] mod_row;
    
    //--------------------------------------------------------------------------
    // Fixed-point division for pivot scaling
    //--------------------------------------------------------------------------
    function automatic logic signed [EXT_WIDTH-1:0] fp_div_ext(
        input logic signed [EXT_WIDTH-1:0] num,
        input logic signed [EXT_WIDTH-1:0] den
    );
        logic signed [2*EXT_WIDTH-1:0] num_scaled;
        num_scaled = num <<< FRAC_BITS;
        return (den != 0) ? (num_scaled / den) : {EXT_WIDTH{1'b0}};
    endfunction
    
    //--------------------------------------------------------------------------
    // Extended precision multiply
    //--------------------------------------------------------------------------
    function automatic logic signed [EXT_WIDTH-1:0] fp_mul_ext(
        input logic signed [EXT_WIDTH-1:0] a,
        input logic signed [EXT_WIDTH-1:0] b
    );
        logic signed [2*EXT_WIDTH-1:0] prod;
        prod = a * b;
        return prod[EXT_WIDTH+FRAC_BITS-1:FRAC_BITS];
    endfunction
    
    //--------------------------------------------------------------------------
    // FSM Sequential
    //--------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            done <= 1'b0;
            singular <= 1'b0;
            for (int i = 0; i < STATE_DIM; i++)
                for (int j = 0; j < STATE_DIM; j++)
                    A_inv[i][j] <= '0;
        end else begin
            state <= next_state;
            done <= 1'b0;
            
            case (state)
                //--------------------------------------------------------------
                // Initialize augmented matrix [A | I]
                //--------------------------------------------------------------
                S_IDLE: begin
                    singular <= 1'b0;
                end
                
                S_INIT: begin
                    for (int i = 0; i < STATE_DIM; i++) begin
                        for (int j = 0; j < STATE_DIM; j++) begin
                            // Left half: A (sign-extended to EXT_WIDTH)
                            aug[i][j] <= {{(EXT_WIDTH-DATA_WIDTH){A[i][j][DATA_WIDTH-1]}}, A[i][j]};
                            // Right half: Identity
                            aug[i][j + STATE_DIM] <= (i == j) ? 
                                {{(EXT_WIDTH-DATA_WIDTH){1'b0}}, FP_ONE} : '0;
                        end
                        // Add regularization to diagonal
                        aug[i][i] <= {{(EXT_WIDTH-DATA_WIDTH){A[i][i][DATA_WIDTH-1]}}, A[i][i]} + 
                                     {{(EXT_WIDTH-DATA_WIDTH){1'b0}}, FP_EPS};
                    end
                    piv_col <= '0;
                end
                
                //--------------------------------------------------------------
                // Pivot Operations: Scale row so diagonal = 1
                //--------------------------------------------------------------
                S_PIVOT_0, S_PIVOT_1, S_PIVOT_2, S_PIVOT_3: begin
                    // Get pivot element
                    pivot <= aug[piv_col][piv_col];
                    
                    // Check for singularity
                    if (aug[piv_col][piv_col] == '0 || 
                        (aug[piv_col][piv_col] < {{(EXT_WIDTH-4){1'b0}}, 4'h1} &&
                         aug[piv_col][piv_col] > -{{(EXT_WIDTH-4){1'b0}}, 4'h1})) begin
                        singular <= 1'b1;
                    end
                    
                    // Scale entire row by 1/pivot
                    for (int j = 0; j < AUG_COLS; j++) begin
                        if (aug[piv_col][piv_col] != '0)
                            aug[piv_col][j] <= fp_div_ext(aug[piv_col][j], aug[piv_col][piv_col]);
                    end
                end
                
                //--------------------------------------------------------------
                // Elimination: Zero out column in other rows
                //--------------------------------------------------------------
                S_ELIM_0_1: begin eliminate_row(0, 1); end
                S_ELIM_0_2: begin eliminate_row(0, 2); end
                S_ELIM_0_3: begin eliminate_row(0, 3); piv_col <= 2'd1; end
                
                S_ELIM_1_0: begin eliminate_row(1, 0); end
                S_ELIM_1_2: begin eliminate_row(1, 2); end
                S_ELIM_1_3: begin eliminate_row(1, 3); piv_col <= 2'd2; end
                
                S_ELIM_2_0: begin eliminate_row(2, 0); end
                S_ELIM_2_1: begin eliminate_row(2, 1); end
                S_ELIM_2_3: begin eliminate_row(2, 3); piv_col <= 2'd3; end
                
                S_ELIM_3_0: begin eliminate_row(3, 0); end
                S_ELIM_3_1: begin eliminate_row(3, 1); end
                S_ELIM_3_2: begin eliminate_row(3, 2); end
                
                //--------------------------------------------------------------
                // Extract inverse from right half
                //--------------------------------------------------------------
                S_EXTRACT: begin
                    for (int i = 0; i < STATE_DIM; i++) begin
                        for (int j = 0; j < STATE_DIM; j++) begin
                            // Truncate back to DATA_WIDTH
                            A_inv[i][j] <= aug[i][j + STATE_DIM][DATA_WIDTH-1:0];
                        end
                    end
                end
                
                S_DONE: begin
                    done <= 1'b1;
                end
            endcase
        end
    end
    
    //--------------------------------------------------------------------------
    // Elimination Task: row[target] -= factor * row[pivot_col]
    //--------------------------------------------------------------------------
    task automatic eliminate_row(input int pivot_c, input int target_r);
        logic signed [EXT_WIDTH-1:0] f;
        f = aug[target_r][pivot_c];  // Factor to eliminate
        for (int j = 0; j < AUG_COLS; j++) begin
            aug[target_r][j] <= aug[target_r][j] - fp_mul_ext(f, aug[pivot_c][j]);
        end
    endtask
    
    //--------------------------------------------------------------------------
    // FSM Combinational
    //--------------------------------------------------------------------------
    always_comb begin
        next_state = state;
        case (state)
            S_IDLE:     if (start) next_state = S_INIT;
            S_INIT:     next_state = S_PIVOT_0;
            S_PIVOT_0:  next_state = S_ELIM_0_1;
            S_ELIM_0_1: next_state = S_ELIM_0_2;
            S_ELIM_0_2: next_state = S_ELIM_0_3;
            S_ELIM_0_3: next_state = S_PIVOT_1;
            S_PIVOT_1:  next_state = S_ELIM_1_0;
            S_ELIM_1_0: next_state = S_ELIM_1_2;
            S_ELIM_1_2: next_state = S_ELIM_1_3;
            S_ELIM_1_3: next_state = S_PIVOT_2;
            S_PIVOT_2:  next_state = S_ELIM_2_0;
            S_ELIM_2_0: next_state = S_ELIM_2_1;
            S_ELIM_2_1: next_state = S_ELIM_2_3;
            S_ELIM_2_3: next_state = S_PIVOT_3;
            S_PIVOT_3:  next_state = S_ELIM_3_0;
            S_ELIM_3_0: next_state = S_ELIM_3_1;
            S_ELIM_3_1: next_state = S_ELIM_3_2;
            S_ELIM_3_2: next_state = S_EXTRACT;
            S_EXTRACT:  next_state = S_DONE;
            S_DONE:     next_state = S_IDLE;
        endcase
    end

endmodule
