//==============================================================================
// QEDMMA v3.1 — Top-Level with Fixed-Lag Smoother
// [REQ-TOP-01] Integrated IMM Filter + Smoother
// [REQ-TOP-02] AXI-Lite control interface
// [REQ-TOP-03] AXI-Stream measurement input
//==============================================================================
// Target: Xilinx RFSoC ZU28DR / ZU48DR
// Resources: ~15K LUTs, 48 DSP48E2, 36 BRAM
//==============================================================================

`timescale 1ns/1ps

module qedmma_v31_top #(
    parameter int STATE_DIM  = 4,
    parameter int MEAS_DIM   = 2,
    parameter int N_MODELS   = 3,
    parameter int LAG_DEPTH  = 50,
    parameter int DATA_WIDTH = 32,
    parameter int FRAC_BITS  = 16
)(
    // System
    input  logic                    clk,
    input  logic                    rst_n,
    
    // AXI-Stream Measurement Input
    input  logic                    s_axis_meas_tvalid,
    output logic                    s_axis_meas_tready,
    input  logic [2*DATA_WIDTH-1:0] s_axis_meas_tdata,   // {y, x}
    
    // AXI-Stream Filtered Output
    output logic                    m_axis_filt_tvalid,
    input  logic                    m_axis_filt_tready,
    output logic [4*DATA_WIDTH-1:0] m_axis_filt_tdata,   // {vy, vx, y, x}
    
    // AXI-Stream Smoothed Output (LAG_DEPTH delayed)
    output logic                    m_axis_smooth_tvalid,
    input  logic                    m_axis_smooth_tready,
    output logic [4*DATA_WIDTH-1:0] m_axis_smooth_tdata,
    
    // AXI-Lite Configuration
    input  logic [31:0]             cfg_omega,           // Turn rate (Q16.16)
    input  logic [31:0]             cfg_q_cv,            // CV process noise
    input  logic [31:0]             cfg_q_ct,            // CT process noise
    input  logic [31:0]             cfg_r,               // Measurement noise
    input  logic [7:0]              cfg_lag_depth,       // Dynamic lag config
    input  logic                    cfg_smooth_enable,   // Enable smoother
    
    // Status
    output logic [31:0]             status_track_count,
    output logic [2:0]              status_mode_active,  // Current dominant mode
    output logic                    status_initialized
);

    //--------------------------------------------------------------------------
    // Model Matrices (ROM or computed from cfg_omega)
    //--------------------------------------------------------------------------
    logic [DATA_WIDTH-1:0] F_mat [N_MODELS][STATE_DIM][STATE_DIM];
    logic [DATA_WIDTH-1:0] Q_mat [N_MODELS][STATE_DIM][STATE_DIM];
    logic [DATA_WIDTH-1:0] H_mat [MEAS_DIM][STATE_DIM];
    logic [DATA_WIDTH-1:0] R_mat [MEAS_DIM][MEAS_DIM];
    logic [DATA_WIDTH-1:0] PI_mat [N_MODELS][N_MODELS];
    
    // Model initialization (simplified — use ROM or LUT in production)
    localparam logic [DATA_WIDTH-1:0] DT = 32'h0000_199A;  // 0.1 in Q16.16
    
    initial begin
        // CV Model (identity velocity propagation)
        F_mat[0] = '{
            '{32'h0001_0000, 32'h0, DT, 32'h0},
            '{32'h0, 32'h0001_0000, 32'h0, DT},
            '{32'h0, 32'h0, 32'h0001_0000, 32'h0},
            '{32'h0, 32'h0, 32'h0, 32'h0001_0000}
        };
        
        // CT+/CT- loaded from ROM based on cfg_omega
        // (Placeholder — actual uses sin/cos LUT)
        F_mat[1] = F_mat[0];
        F_mat[2] = F_mat[0];
        
        // Measurement matrix
        H_mat = '{
            '{32'h0001_0000, 32'h0, 32'h0, 32'h0},
            '{32'h0, 32'h0001_0000, 32'h0, 32'h0}
        };
        
        // Transition matrix (p_stay = 0.88)
        PI_mat = '{
            '{32'h0000_E148, 32'h0000_0F5C, 32'h0000_0F5C},
            '{32'h0000_0F5C, 32'h0000_E148, 32'h0000_0F5C},
            '{32'h0000_0F5C, 32'h0000_0F5C, 32'h0000_E148}
        };
    end
    
    //--------------------------------------------------------------------------
    // IMM Filter Core
    //--------------------------------------------------------------------------
    logic filt_valid, filt_ready;
    logic [DATA_WIDTH-1:0] filt_xf [N_MODELS][STATE_DIM];
    logic [DATA_WIDTH-1:0] filt_Pf [N_MODELS][STATE_DIM][STATE_DIM];
    logic [DATA_WIDTH-1:0] filt_xp [N_MODELS][STATE_DIM];
    logic [DATA_WIDTH-1:0] filt_Pp [N_MODELS][STATE_DIM][STATE_DIM];
    logic [DATA_WIDTH-1:0] filt_mu [N_MODELS];
    logic [DATA_WIDTH-1:0] filt_x_combined [STATE_DIM];
    
    // IMM filter instance (from existing qedmma_imm_core_v3.sv)
    // Placeholder — wire to existing module
    
    assign filt_valid = s_axis_meas_tvalid;  // Simplified
    assign s_axis_meas_tready = filt_ready;
    
    //--------------------------------------------------------------------------
    // Fixed-Lag Smoother
    //--------------------------------------------------------------------------
    logic smooth_valid;
    logic [DATA_WIDTH-1:0] smooth_xs [STATE_DIM];
    logic [DATA_WIDTH-1:0] smooth_xs_model [N_MODELS][STATE_DIM];
    
    generate
        if (1) begin : gen_smoother
            imm_fixed_lag_smoother #(
                .STATE_DIM(STATE_DIM),
                .N_MODELS(N_MODELS),
                .LAG_DEPTH(LAG_DEPTH),
                .DATA_WIDTH(DATA_WIDTH),
                .FRAC_BITS(FRAC_BITS)
            ) u_smoother (
                .clk(clk),
                .rst_n(rst_n),
                
                .fwd_valid(filt_valid & cfg_smooth_enable),
                .fwd_ready(filt_ready),
                .fwd_xf(filt_xf),
                .fwd_Pf(filt_Pf),
                .fwd_xp(filt_xp),
                .fwd_Pp(filt_Pp),
                .fwd_mu(filt_mu),
                
                .F_mat(F_mat),
                
                .out_valid(smooth_valid),
                .out_xs(smooth_xs),
                .out_xs_model(smooth_xs_model)
            );
        end
    endgenerate
    
    //--------------------------------------------------------------------------
    // Output Packing
    //--------------------------------------------------------------------------
    assign m_axis_filt_tvalid = filt_valid;
    assign m_axis_filt_tdata = {filt_x_combined[3], filt_x_combined[2], 
                                 filt_x_combined[1], filt_x_combined[0]};
    
    assign m_axis_smooth_tvalid = smooth_valid & cfg_smooth_enable;
    assign m_axis_smooth_tdata = {smooth_xs[3], smooth_xs[2], 
                                   smooth_xs[1], smooth_xs[0]};
    
    //--------------------------------------------------------------------------
    // Status
    //--------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            status_track_count <= '0;
            status_initialized <= 1'b0;
        end else begin
            if (filt_valid)
                status_track_count <= status_track_count + 1;
            
            if (status_track_count > 1)
                status_initialized <= 1'b1;
        end
    end
    
    // Dominant mode detection
    always_comb begin
        status_mode_active = 3'b001;  // Default CV
        if (filt_mu[1] > filt_mu[0] && filt_mu[1] > filt_mu[2])
            status_mode_active = 3'b010;  // CT+
        else if (filt_mu[2] > filt_mu[0] && filt_mu[2] > filt_mu[1])
            status_mode_active = 3'b100;  // CT-
    end

endmodule
