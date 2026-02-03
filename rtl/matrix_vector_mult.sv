//==============================================================================
// NX-MIMOSA v3.1 Pro — Matrix-Vector Multiply
// [REQ-RTL-MV-01] y = M @ x (4x4 matrix, 4x1 vector)
// [REQ-RTL-MV-02] Single-cycle combinational with DSP inference
//==============================================================================

`timescale 1ns/1ps

module matrix_vector_mult
    import nx_mimosa_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  fp_t         M [STATE_DIM][STATE_DIM],
    input  fp_t         x [STATE_DIM],
    output fp_t         y [STATE_DIM],
    output logic        done
);

    logic [1:0] cycle_cnt;
    logic running;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < STATE_DIM; i++)
                y[i] <= '0;
            done <= 1'b0;
            running <= 1'b0;
            cycle_cnt <= '0;
        end else begin
            done <= 1'b0;
            
            if (start && !running) begin
                running <= 1'b1;
                cycle_cnt <= '0;
                
                // Compute y = M @ x (all rows in parallel)
                for (int i = 0; i < STATE_DIM; i++) begin
                    logic signed [2*DATA_WIDTH-1:0] acc;
                    acc = '0;
                    for (int j = 0; j < STATE_DIM; j++) begin
                        acc = acc + (M[i][j] * x[j]);
                    end
                    y[i] <= acc[DATA_WIDTH+FRAC_BITS-1:FRAC_BITS];
                end
            end else if (running) begin
                cycle_cnt <= cycle_cnt + 1;
                if (cycle_cnt == 2'd1) begin  // 2-cycle latency
                    done <= 1'b1;
                    running <= 1'b0;
                end
            end
        end
    end

endmodule


//==============================================================================
// NX-MIMOSA v3.1 Pro — Vector Subtraction
// y = a - b
//==============================================================================
module vector_subtract
    import nx_mimosa_pkg::*;
#(
    parameter int VEC_DIM = STATE_DIM
)(
    input  fp_t  a [VEC_DIM],
    input  fp_t  b [VEC_DIM],
    output fp_t  y [VEC_DIM]
);
    generate
        for (genvar i = 0; i < VEC_DIM; i++) begin : sub_gen
            assign y[i] = a[i] - b[i];
        end
    endgenerate
endmodule


//==============================================================================
// NX-MIMOSA v3.1 Pro — Vector Addition
// y = a + b
//==============================================================================
module vector_add
    import nx_mimosa_pkg::*;
#(
    parameter int VEC_DIM = STATE_DIM
)(
    input  fp_t  a [VEC_DIM],
    input  fp_t  b [VEC_DIM],
    output fp_t  y [VEC_DIM]
);
    generate
        for (genvar i = 0; i < VEC_DIM; i++) begin : add_gen
            assign y[i] = a[i] + b[i];
        end
    endgenerate
endmodule
