//==============================================================================
// NX-MIMOSA v3.1 Pro — Top-Level Integration
// [REQ-RTL-TOP-01] AXI-Stream measurement input
// [REQ-RTL-TOP-02] AXI-Stream filtered/smoothed output
// [REQ-RTL-TOP-03] AXI-Lite configuration
// [REQ-RTL-TOP-04] IMM Core + Fixed-Lag Smoother integration
//==============================================================================
// Target: RFSoC 4x2 Board ZU48DR (RFSoC 4x2 Board) @ 250MHz
// Author: Dr. Mladen Mešter / Nexellum d.o.o.
// License: Commercial (nx_mimosa-pro)
//==============================================================================

`timescale 1ns/1ps

module nx_mimosa_top
    import nx_mimosa_pkg::*;
#(
    parameter int C_S_AXI_DATA_WIDTH = 32,
    parameter int C_S_AXI_ADDR_WIDTH = 8
)(
    // System
    input  logic        aclk,
    input  logic        aresetn,
    
    //--------------------------------------------------------------------------
    // AXI-Stream Measurement Input
    //--------------------------------------------------------------------------
    input  logic [63:0] s_axis_meas_tdata,     // {z_y[31:0], z_x[31:0]}
    input  logic        s_axis_meas_tvalid,
    output logic        s_axis_meas_tready,
    
    //--------------------------------------------------------------------------
    // AXI-Stream Filtered Output
    //--------------------------------------------------------------------------
    output logic [127:0] m_axis_filt_tdata,    // {vy, vx, y, x}
    output logic         m_axis_filt_tvalid,
    input  logic         m_axis_filt_tready,
    
    //--------------------------------------------------------------------------
    // AXI-Stream Smoothed Output
    //--------------------------------------------------------------------------
    output logic [127:0] m_axis_smooth_tdata,  // {vy, vx, y, x}
    output logic         m_axis_smooth_tvalid,
    input  logic         m_axis_smooth_tready,
    
    //--------------------------------------------------------------------------
    // AXI-Lite Configuration Interface
    //--------------------------------------------------------------------------
    input  logic [C_S_AXI_ADDR_WIDTH-1:0]  s_axi_awaddr,
    input  logic                           s_axi_awvalid,
    output logic                           s_axi_awready,
    input  logic [C_S_AXI_DATA_WIDTH-1:0]  s_axi_wdata,
    input  logic                           s_axi_wvalid,
    output logic                           s_axi_wready,
    output logic [1:0]                     s_axi_bresp,
    output logic                           s_axi_bvalid,
    input  logic                           s_axi_bready,
    input  logic [C_S_AXI_ADDR_WIDTH-1:0]  s_axi_araddr,
    input  logic                           s_axi_arvalid,
    output logic                           s_axi_arready,
    output logic [C_S_AXI_DATA_WIDTH-1:0]  s_axi_rdata,
    output logic [1:0]                     s_axi_rresp,
    output logic                           s_axi_rvalid,
    input  logic                           s_axi_rready,
    
    //--------------------------------------------------------------------------
    // Status
    //--------------------------------------------------------------------------
    output logic [2:0]  dominant_mode,        // Which model is most likely
    output logic        track_initialized,
    output logic [31:0] track_count
);

    //--------------------------------------------------------------------------
    // Configuration Registers (AXI-Lite accessible)
    //--------------------------------------------------------------------------
    // Address map:
    // 0x00: Control (bit 0: enable, bit 1: smoother_enable, bit 2: reset)
    // 0x04: Omega (turn rate, Q15.16)
    // 0x08: dt (time step, Q15.16)
    // 0x0C: q_cv (process noise CV)
    // 0x10: q_ct (process noise CT)
    // 0x14: r (measurement noise std)
    // 0x18: p_stay (mode stay probability)
    // 0x1C: Status (read-only)
    // 0x20-0x2C: x_init[0..3]
    // 0x30-0x6C: P_init (diagonal only for simplicity)
    
    logic        cfg_enable;
    logic        cfg_smoother_enable;
    logic        cfg_reset;
    fp_t         cfg_omega;
    fp_t         cfg_dt;
    fp_t         cfg_q_cv;
    fp_t         cfg_q_ct;
    fp_t         cfg_r;
    fp_t         cfg_p_stay;
    fp_t         cfg_x_init [STATE_DIM];
    fp_t         cfg_P_init [STATE_DIM][STATE_DIM];
    
    // Default values
    initial begin
        cfg_enable = 1'b1;
        cfg_smoother_enable = 1'b1;
        cfg_reset = 1'b0;
        cfg_omega = 32'h0000_3298;        // 0.196 rad/s (6g @ 300m/s)
        cfg_dt = 32'h0000_199A;           // 0.1s
        cfg_q_cv = 32'h0000_8000;         // 0.5
        cfg_q_ct = 32'h0001_0000;         // 1.0
        cfg_r = 32'h0002_8000;            // 2.5m
        cfg_p_stay = 32'h0000_E148;       // 0.88
        
        for (int i = 0; i < STATE_DIM; i++) begin
            cfg_x_init[i] = FP_ZERO;
            for (int j = 0; j < STATE_DIM; j++)
                cfg_P_init[i][j] = (i == j) ? 32'h0064_0000 : FP_ZERO;  // 100.0
        end
    end
    
    //--------------------------------------------------------------------------
    // AXI-Lite State Machine
    //--------------------------------------------------------------------------
    logic [C_S_AXI_ADDR_WIDTH-1:0] axi_awaddr_reg;
    logic axi_aw_done, axi_w_done;
    
    // Write Address
    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            s_axi_awready <= 1'b0;
            axi_awaddr_reg <= '0;
            axi_aw_done <= 1'b0;
        end else begin
            if (!axi_aw_done && s_axi_awvalid && !s_axi_awready) begin
                s_axi_awready <= 1'b1;
                axi_awaddr_reg <= s_axi_awaddr;
                axi_aw_done <= 1'b1;
            end else begin
                s_axi_awready <= 1'b0;
                if (s_axi_bvalid && s_axi_bready)
                    axi_aw_done <= 1'b0;
            end
        end
    end
    
    // Write Data
    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            s_axi_wready <= 1'b0;
            axi_w_done <= 1'b0;
            cfg_reset <= 1'b0;
        end else begin
            cfg_reset <= 1'b0;  // Auto-clear
            
            if (!axi_w_done && s_axi_wvalid && !s_axi_wready) begin
                s_axi_wready <= 1'b1;
                axi_w_done <= 1'b1;
                
                // Write to registers
                case (axi_awaddr_reg[7:2])
                    6'h00: begin
                        cfg_enable <= s_axi_wdata[0];
                        cfg_smoother_enable <= s_axi_wdata[1];
                        cfg_reset <= s_axi_wdata[2];
                    end
                    6'h01: cfg_omega <= s_axi_wdata;
                    6'h02: cfg_dt <= s_axi_wdata;
                    6'h03: cfg_q_cv <= s_axi_wdata;
                    6'h04: cfg_q_ct <= s_axi_wdata;
                    6'h05: cfg_r <= s_axi_wdata;
                    6'h06: cfg_p_stay <= s_axi_wdata;
                    6'h08: cfg_x_init[0] <= s_axi_wdata;
                    6'h09: cfg_x_init[1] <= s_axi_wdata;
                    6'h0A: cfg_x_init[2] <= s_axi_wdata;
                    6'h0B: cfg_x_init[3] <= s_axi_wdata;
                    6'h0C: cfg_P_init[0][0] <= s_axi_wdata;
                    6'h0D: cfg_P_init[1][1] <= s_axi_wdata;
                    6'h0E: cfg_P_init[2][2] <= s_axi_wdata;
                    6'h0F: cfg_P_init[3][3] <= s_axi_wdata;
                endcase
            end else begin
                s_axi_wready <= 1'b0;
                if (s_axi_bvalid && s_axi_bready)
                    axi_w_done <= 1'b0;
            end
        end
    end
    
    // Write Response
    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            s_axi_bvalid <= 1'b0;
            s_axi_bresp <= 2'b00;
        end else begin
            if (axi_aw_done && axi_w_done && !s_axi_bvalid) begin
                s_axi_bvalid <= 1'b1;
                s_axi_bresp <= 2'b00;  // OKAY
            end else if (s_axi_bready && s_axi_bvalid) begin
                s_axi_bvalid <= 1'b0;
            end
        end
    end
    
    // Read
    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            s_axi_arready <= 1'b0;
            s_axi_rvalid <= 1'b0;
            s_axi_rresp <= 2'b00;
            s_axi_rdata <= '0;
        end else begin
            if (s_axi_arvalid && !s_axi_arready) begin
                s_axi_arready <= 1'b1;
                
                case (s_axi_araddr[7:2])
                    6'h00: s_axi_rdata <= {29'b0, cfg_reset, cfg_smoother_enable, cfg_enable};
                    6'h01: s_axi_rdata <= cfg_omega;
                    6'h02: s_axi_rdata <= cfg_dt;
                    6'h03: s_axi_rdata <= cfg_q_cv;
                    6'h04: s_axi_rdata <= cfg_q_ct;
                    6'h05: s_axi_rdata <= cfg_r;
                    6'h06: s_axi_rdata <= cfg_p_stay;
                    6'h07: s_axi_rdata <= {track_initialized, 28'b0, dominant_mode};
                    default: s_axi_rdata <= '0;
                endcase
            end else begin
                s_axi_arready <= 1'b0;
            end
            
            if (s_axi_arready) begin
                s_axi_rvalid <= 1'b1;
                s_axi_rresp <= 2'b00;
            end else if (s_axi_rready && s_axi_rvalid) begin
                s_axi_rvalid <= 1'b0;
            end
        end
    end
    
    //--------------------------------------------------------------------------
    // Measurement Parsing
    //--------------------------------------------------------------------------
    fp_t z [MEAS_DIM];
    logic meas_valid_internal;
    
    assign z[0] = s_axis_meas_tdata[31:0];    // z_x
    assign z[1] = s_axis_meas_tdata[63:32];   // z_y
    assign meas_valid_internal = s_axis_meas_tvalid && s_axis_meas_tready && cfg_enable;
    
    //--------------------------------------------------------------------------
    // IMM Core Instance
    //--------------------------------------------------------------------------
    fp_t x_filt [STATE_DIM];
    fp_t x_filt_model [N_MODELS][STATE_DIM];
    fp_t P_filt_model [N_MODELS][STATE_DIM][STATE_DIM];
    fp_t x_pred_model [N_MODELS][STATE_DIM];
    fp_t P_pred_model [N_MODELS][STATE_DIM][STATE_DIM];
    fp_t mu [N_MODELS];
    logic imm_done, imm_filt_valid;
    logic imm_init;
    
    assign imm_init = cfg_reset || (!track_initialized && meas_valid_internal);
    
    imm_core imm_inst (
        .clk(aclk),
        .rst_n(aresetn),
        .meas_valid(meas_valid_internal),
        .init(imm_init),
        .done(imm_done),
        .z(z),
        .omega(cfg_omega),
        .dt(cfg_dt),
        .q_cv(cfg_q_cv),
        .q_ct(cfg_q_ct),
        .r(cfg_r),
        .p_stay(cfg_p_stay),
        .x_init(cfg_x_init),
        .P_init(cfg_P_init),
        .x_filt(x_filt),
        .x_filt_model(x_filt_model),
        .P_filt_model(P_filt_model),
        .x_pred_model(x_pred_model),
        .P_pred_model(P_pred_model),
        .mu(mu),
        .filt_valid(imm_filt_valid)
    );
    
    //--------------------------------------------------------------------------
    // Fixed-Lag Smoother Instance
    //--------------------------------------------------------------------------
    fp_t x_smooth [STATE_DIM];
    fp_t x_smooth_model [N_MODELS][STATE_DIM];
    logic smooth_valid;
    
    // F matrices (generated from IMM core parameters)
    fp_t F_mat [N_MODELS][STATE_DIM][STATE_DIM];
    
    // F matrix generation (CV model)
    always_comb begin
        // CV: [1, 0, dt, 0; 0, 1, 0, dt; 0, 0, 1, 0; 0, 0, 0, 1]
        F_mat[0][0][0] = FP_ONE;  F_mat[0][0][1] = FP_ZERO; F_mat[0][0][2] = cfg_dt;   F_mat[0][0][3] = FP_ZERO;
        F_mat[0][1][0] = FP_ZERO; F_mat[0][1][1] = FP_ONE;  F_mat[0][1][2] = FP_ZERO;  F_mat[0][1][3] = cfg_dt;
        F_mat[0][2][0] = FP_ZERO; F_mat[0][2][1] = FP_ZERO; F_mat[0][2][2] = FP_ONE;   F_mat[0][2][3] = FP_ZERO;
        F_mat[0][3][0] = FP_ZERO; F_mat[0][3][1] = FP_ZERO; F_mat[0][3][2] = FP_ZERO;  F_mat[0][3][3] = FP_ONE;
        
        // CT+ and CT- get F from IMM core (simplified: use CV for smoother)
        // Full implementation would route F from sincos_lut
        F_mat[1] = F_mat[0];
        F_mat[2] = F_mat[0];
    end
    
    fixed_lag_smoother smoother_inst (
        .clk(aclk),
        .rst_n(aresetn),
        .fwd_valid(imm_filt_valid),
        .fwd_xf(x_filt_model),
        .fwd_Pf(P_filt_model),
        .fwd_xp(x_pred_model),
        .fwd_Pp(P_pred_model),
        .fwd_mu(mu),
        .F_mat(F_mat),
        .enable(cfg_smoother_enable),
        .smooth_valid(smooth_valid),
        .x_smooth(x_smooth),
        .x_smooth_model(x_smooth_model)
    );
    
    //--------------------------------------------------------------------------
    // Output AXI-Stream
    //--------------------------------------------------------------------------
    // Filtered output
    assign s_axis_meas_tready = cfg_enable;
    
    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            m_axis_filt_tvalid <= 1'b0;
            m_axis_filt_tdata <= '0;
        end else begin
            if (imm_filt_valid) begin
                m_axis_filt_tvalid <= 1'b1;
                m_axis_filt_tdata <= {x_filt[3], x_filt[2], x_filt[1], x_filt[0]};
            end else if (m_axis_filt_tready) begin
                m_axis_filt_tvalid <= 1'b0;
            end
        end
    end
    
    // Smoothed output
    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            m_axis_smooth_tvalid <= 1'b0;
            m_axis_smooth_tdata <= '0;
        end else begin
            if (smooth_valid) begin
                m_axis_smooth_tvalid <= 1'b1;
                m_axis_smooth_tdata <= {x_smooth[3], x_smooth[2], x_smooth[1], x_smooth[0]};
            end else if (m_axis_smooth_tready) begin
                m_axis_smooth_tvalid <= 1'b0;
            end
        end
    end
    
    //--------------------------------------------------------------------------
    // Status
    //--------------------------------------------------------------------------
    // Find dominant mode
    always_comb begin
        if (mu[0] >= mu[1] && mu[0] >= mu[2])
            dominant_mode = 3'd0;  // CV
        else if (mu[1] >= mu[2])
            dominant_mode = 3'd1;  // CT+
        else
            dominant_mode = 3'd2;  // CT-
    end
    
    // Track initialization and count
    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            track_initialized <= 1'b0;
            track_count <= '0;
        end else begin
            if (cfg_reset) begin
                track_initialized <= 1'b0;
                track_count <= '0;
            end else if (imm_filt_valid) begin
                track_initialized <= 1'b1;
                track_count <= track_count + 1;
            end
        end
    end

endmodule
