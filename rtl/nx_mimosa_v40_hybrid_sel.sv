//==============================================================================
// NX-MIMOSA v4.0.2 — NIS-Based Hybrid Stream Selector
//==============================================================================
// Selects between CV-RTS and CA-RTS streams based on running NIS comparison.
// For fire control applications: bounded 1.5s latency.
//
// Algorithm:
//   nis_cv_avg = EMA(NIS_CV, alpha=0.1)
//   nis_ca_avg = EMA(NIS_CA, alpha=0.1)
//   if nis_cv_avg < nis_ca_avg: select CV-RTS
//   else: select CA-RTS
//
// [REQ-HYB-001] NIS-based stream selection for fire control
//==============================================================================

`timescale 1ns/1ps

module nx_mimosa_v40_hybrid_sel
    import nx_mimosa_v40_pkg::*;
#(
    parameter int NIS_WINDOW = 20
)(
    input  logic                   clk,
    input  logic                   rst_n,
    
    // CV-RTS stream
    input  fp_t                    cvr_x [STATE_DIM],
    input  fp_t                    cvr_quality,
    input  logic                   cvr_valid,
    
    // CA-RTS stream
    input  fp_t                    car_x [STATE_DIM],
    input  fp_t                    car_quality,
    input  logic                   car_valid,
    
    // Forward NIS for online comparison
    input  fp_t                    nis_cv,
    input  fp_t                    nis_ca,
    input  logic                   nis_valid,
    
    // Hybrid output
    output fp_t                    hyb_x [STATE_DIM],
    output fp_t                    hyb_quality,
    output logic                   hyb_valid
);

    //--------------------------------------------------------------------------
    // NIS Exponential Moving Average
    //--------------------------------------------------------------------------
    // alpha = 0.1 = 0x0000_199A in Q15.16
    // (1-alpha) = 0.9 = 0x0000_E666
    localparam fp_t EMA_ALPHA     = 32'h0000_199A;  // 0.1
    localparam fp_t EMA_ONE_ALPHA = 32'h0000_E666;  // 0.9
    
    fp_t  nis_cv_ema;
    fp_t  nis_ca_ema;
    logic ema_initialized;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            nis_cv_ema      <= FP_ZERO;
            nis_ca_ema      <= FP_ZERO;
            ema_initialized <= 1'b0;
        end else if (nis_valid) begin
            if (!ema_initialized) begin
                nis_cv_ema      <= nis_cv;
                nis_ca_ema      <= nis_ca;
                ema_initialized <= 1'b1;
            end else begin
                // EMA update: avg = (1-alpha)*avg + alpha*new
                nis_cv_ema <= fp_mul(EMA_ONE_ALPHA, nis_cv_ema) + fp_mul(EMA_ALPHA, nis_cv);
                nis_ca_ema <= fp_mul(EMA_ONE_ALPHA, nis_ca_ema) + fp_mul(EMA_ALPHA, nis_ca);
            end
        end
    end
    
    //--------------------------------------------------------------------------
    // Stream Selection Logic
    //--------------------------------------------------------------------------
    logic select_cv;      // 1 = use CV-RTS, 0 = use CA-RTS
    logic select_cv_r;    // Registered for stability
    
    // Hysteresis to prevent rapid switching
    localparam fp_t HYST_MARGIN = 32'h0000_3333;  // 0.2 (20% hysteresis)
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            select_cv_r <= 1'b1;  // Default to CV-RTS
        end else if (nis_valid && ema_initialized) begin
            if (select_cv_r) begin
                // Currently using CV: switch to CA only if CA significantly better
                if (nis_ca_ema + HYST_MARGIN < nis_cv_ema)
                    select_cv_r <= 1'b0;
            end else begin
                // Currently using CA: switch to CV only if CV significantly better
                if (nis_cv_ema + HYST_MARGIN < nis_ca_ema)
                    select_cv_r <= 1'b1;
            end
        end
    end
    
    //--------------------------------------------------------------------------
    // Output MUX
    //--------------------------------------------------------------------------
    // Use whichever stream is valid, prefer selected stream
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hyb_valid <= 1'b0;
        end else begin
            hyb_valid <= 1'b0;
            
            if (select_cv_r && cvr_valid) begin
                hyb_x       <= cvr_x;
                hyb_quality <= cvr_quality;
                hyb_valid   <= 1'b1;
            end else if (!select_cv_r && car_valid) begin
                hyb_x       <= car_x;
                hyb_quality <= car_quality;
                hyb_valid   <= 1'b1;
            end else if (cvr_valid) begin
                // Fallback: use whatever is available
                hyb_x       <= cvr_x;
                hyb_quality <= cvr_quality;
                hyb_valid   <= 1'b1;
            end else if (car_valid) begin
                hyb_x       <= car_x;
                hyb_quality <= car_quality;
                hyb_valid   <= 1'b1;
            end
        end
    end

endmodule
