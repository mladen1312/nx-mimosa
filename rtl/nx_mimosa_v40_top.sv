//==============================================================================
// NX-MIMOSA v4.0.2 SENTINEL — Multi-Pipeline Top Module
//==============================================================================
// Architecture:
//   ┌─────────────────────────────────────────────────────────────────────┐
//   │                     AXI-Stream Measurement Input                    │
//   └──────────────────────────────┬──────────────────────────────────────┘
//                                  │
//                    ┌─────────────▼─────────────┐
//                    │    6-Model IMM Forward     │  ← Stream 0: FWD (0 lag)
//                    │  CV|CA|CT+|CT-|Jerk|Ball   │
//                    └──┬──────────┬──────────┬──┘
//                       │          │          │
//         ┌─────────────▼─┐  ┌────▼────┐  ┌──▼──────────────┐
//         │ CV-RTS Fixed   │  │ CA-RTS  │  │ Feature Extract │
//         │ Lag Smoother   │  │ Fixed   │  │ (omega, NIS,    │
//         │ (W=40 steps)   │  │ Lag     │  │  mu peaks)      │
//         └───────┬───────┘  └────┬────┘  └───────┬─────────┘
//                 │               │               │
//         Stream 2: CVR    Stream 3: CAR    ┌─────▼─────────┐
//         (offline)        (offline)        │ NIS Hybrid     │
//                                           │ Selector       │
//                                           │ min(NIS_CV,    │
//                                           │     NIS_CA)    │
//                                           └───────┬───────┘
//                                                   │
//                                            Stream 1: HYB
//                                            (fire control,
//                                             1.5s lag)
//                                                   │
//                    ┌──────────────────────────────────────────┐
//                    │    Stream MUX → AXI-Stream Output        │
//                    │    (selectable via REG_STREAM_SEL)       │
//                    └──────────────────────────────────────────┘
//
// Target: Xilinx RFSoC ZU48DR @ 250MHz
// Author: Dr. Mladen Mešter / Nexellum d.o.o.
// [REQ-TOP-001] Multi-pipeline with 4 output streams
//==============================================================================

`timescale 1ns/1ps

module nx_mimosa_v40_top
    import nx_mimosa_v40_pkg::*;
#(
    parameter int NUM_TARGETS    = 1,        // [REQ-TOP-002] 1-8 simultaneous tracks
    parameter int RTS_WINDOW     = 40,       // [REQ-TOP-003] Fixed-lag window depth
    parameter int HYBRID_LOOKBACK = 15       // [REQ-TOP-004] Hybrid selector lookback
)(
    // Clock & Reset
    input  logic                      clk,
    input  logic                      rst_n,
    
    // === AXI-Stream Measurement Input ===
    // [REQ-IF-001] Packed: {tgt_id[2:0], meas_y[31:0], meas_x[31:0]}
    input  logic [66:0]               s_axis_meas_tdata,
    input  logic                      s_axis_meas_tvalid,
    output logic                      s_axis_meas_tready,
    
    // === AXI-Stream Track Output ===
    // [REQ-IF-002] Packed: {stream_id[1:0], tgt_id[2:0], quality[31:0],
    //                       vy[31:0], vx[31:0], y[31:0], x[31:0]}
    output logic [197:0]              m_axis_trk_tdata,
    output logic                      m_axis_trk_tvalid,
    input  logic                      m_axis_trk_tready,
    
    // === AXI-Lite Configuration ===
    // [REQ-IF-003] Register interface for TPM, thresholds, stream select
    input  logic [11:0]               cfg_addr,
    input  logic [31:0]               cfg_wdata,
    input  logic                      cfg_wen,
    output logic [31:0]               cfg_rdata,
    input  logic                      cfg_ren,
    
    // === Platform Classifier Output ===
    // [REQ-IF-004] Feature vector to PS for platform identification
    output classifier_features_t      clf_features,
    output logic                      clf_features_valid,
    
    // === Status ===
    output logic                      ready,
    output logic [7:0]                active_targets
);

    //==========================================================================
    // Configuration Registers
    //==========================================================================
    logic                    sys_enable;
    logic                    sys_reset;
    fp_t                     cfg_dt;
    fp_t                     cfg_r_std;
    logic [STREAM_ID_W-1:0]  cfg_stream_sel;
    logic [MAX_TARGETS-1:0]  cfg_tgt_mask;
    fp_t                     cfg_tpm [N_MODELS][N_MODELS];
    
    // Register write
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sys_enable     <= 1'b0;
            sys_reset      <= 1'b0;
            cfg_dt         <= 32'h0000_199A;  // 0.1s default
            cfg_r_std      <= 32'h0002_8000;  // 2.5m default
            cfg_stream_sel <= STREAM_FWD;
            cfg_tgt_mask   <= 8'h01;          // Track 0 active
            cfg_tpm        <= DEFAULT_TPM;
        end else if (cfg_wen) begin
            case (cfg_addr)
                REG_CTRL:       {sys_reset, sys_enable} <= cfg_wdata[1:0];
                REG_DT:         cfg_dt         <= cfg_wdata;
                REG_R_STD:      cfg_r_std      <= cfg_wdata;
                REG_STREAM_SEL: cfg_stream_sel <= cfg_wdata[STREAM_ID_W-1:0];
                REG_TGT_MASK:   cfg_tgt_mask   <= cfg_wdata[MAX_TARGETS-1:0];
                default: begin
                    // TPM write: addr = 0x100 + model_i*24 + model_j*4
                    if (cfg_addr >= REG_TPM_BASE && cfg_addr < REG_PLATFORM_BASE) begin
                        automatic int offset = (cfg_addr - REG_TPM_BASE) >> 2;
                        automatic int mi = offset / N_MODELS;
                        automatic int mj = offset % N_MODELS;
                        if (mi < N_MODELS && mj < N_MODELS)
                            cfg_tpm[mi][mj] <= cfg_wdata;
                    end
                end
            endcase
        end
    end
    
    // Register read
    always_ff @(posedge clk) begin
        if (cfg_ren) begin
            case (cfg_addr)
                REG_CTRL:       cfg_rdata <= {30'b0, sys_reset, sys_enable};
                REG_STATUS:     cfg_rdata <= {24'b0, active_targets};
                REG_DT:         cfg_rdata <= cfg_dt;
                REG_R_STD:      cfg_rdata <= cfg_r_std;
                REG_STREAM_SEL: cfg_rdata <= {{(32-STREAM_ID_W){1'b0}}, cfg_stream_sel};
                REG_TGT_MASK:   cfg_rdata <= {{(32-MAX_TARGETS){1'b0}}, cfg_tgt_mask};
                default:        cfg_rdata <= 32'hDEAD_BEEF;
            endcase
        end
    end

    //==========================================================================
    // Input Unpacking
    //==========================================================================
    logic [TGT_ID_W-1:0]  in_tgt_id;
    fp_t                   in_meas [MEAS_DIM];
    logic                  in_valid;
    
    assign in_tgt_id   = s_axis_meas_tdata[66:64];
    assign in_meas[0]  = s_axis_meas_tdata[31:0];    // x
    assign in_meas[1]  = s_axis_meas_tdata[63:32];   // y
    assign in_valid     = s_axis_meas_tvalid & sys_enable & !sys_reset;
    assign s_axis_meas_tready = sys_enable;

    //==========================================================================
    // [REQ-TOP-010] 6-Model IMM Forward Filter
    //==========================================================================
    // Outputs: per-model states, mixed estimate, mode probabilities, NIS
    
    // Per-model unmixed states (stored BEFORE mixing for correct RTS)
    fp_t  xu_h  [N_MODELS][STATE_DIM];        // [REQ-RTS-001] Pre-mixing states
    fp_t  Pu_h  [N_MODELS][STATE_DIM][STATE_DIM];
    
    // Per-model post-update states
    fp_t  x_upd [N_MODELS][STATE_DIM];
    fp_t  P_upd [N_MODELS][STATE_DIM][STATE_DIM];
    
    // IMM outputs
    fp_t  x_mixed  [STATE_DIM];
    fp_t  mu       [N_MODELS];
    fp_t  nis_per_model [N_MODELS];
    fp_t  omega_est;
    logic imm_valid;
    logic imm_ready;
    
    nx_mimosa_v40_imm #(
        .NUM_TARGETS(NUM_TARGETS)
    ) u_imm (
        .clk            (clk),
        .rst_n          (rst_n & !sys_reset),
        
        // Measurement input
        .meas           (in_meas),
        .meas_valid     (in_valid),
        .meas_tgt_id    (in_tgt_id),
        .meas_ready     (imm_ready),
        
        // Configuration
        .cfg_dt         (cfg_dt),
        .cfg_r_std      (cfg_r_std),
        .cfg_tpm        (cfg_tpm),
        
        // Pre-mixing unmixed states (for RTS backward pass)
        .xu_out         (xu_h),
        .Pu_out         (Pu_h),
        
        // Post-update states
        .x_upd_out      (x_upd),
        .P_upd_out      (P_upd),
        
        // Mixed outputs
        .x_mixed_out    (x_mixed),
        .mu_out         (mu),
        .nis_out        (nis_per_model),
        .omega_out      (omega_est),
        .out_valid      (imm_valid)
    );

    //==========================================================================
    // [REQ-TOP-020] Stream 0: Forward Output (zero latency)
    //==========================================================================
    fp_t  fwd_x [STATE_DIM];
    fp_t  fwd_quality;
    logic fwd_valid;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fwd_valid <= 1'b0;
        end else begin
            fwd_valid <= imm_valid;
            if (imm_valid) begin
                fwd_x       <= x_mixed;
                fwd_quality <= nis_per_model[MDL_CV];  // Forward quality = CV NIS
            end
        end
    end

    //==========================================================================
    // [REQ-TOP-030] CV-RTS Fixed-Lag Smoother (Stream 2)
    //==========================================================================
    fp_t  cvr_x [STATE_DIM];
    fp_t  cvr_quality;
    logic cvr_valid;
    
    nx_mimosa_v40_rts_lag #(
        .WINDOW_DEPTH (RTS_WINDOW),
        .MODEL_IDX    (MDL_CV)
    ) u_cv_rts (
        .clk          (clk),
        .rst_n        (rst_n & !sys_reset),
        
        // Forward pass inputs (from IMM)
        .x_fwd        (x_upd[MDL_CV]),
        .P_fwd        (P_upd[MDL_CV]),
        .xu_pre       (xu_h[MDL_CV]),       // Pre-mixing state
        .Pu_pre       (Pu_h[MDL_CV]),
        .mu_weight    (mu[MDL_CV]),
        .fwd_valid    (imm_valid),
        
        // Configuration
        .cfg_dt       (cfg_dt),
        
        // Smoothed output
        .x_smooth     (cvr_x),
        .quality      (cvr_quality),
        .smooth_valid (cvr_valid)
    );

    //==========================================================================
    // [REQ-TOP-040] CA-RTS Fixed-Lag Smoother (Stream 3)
    //==========================================================================
    fp_t  car_x [STATE_DIM];
    fp_t  car_quality;
    logic car_valid;
    
    nx_mimosa_v40_rts_lag #(
        .WINDOW_DEPTH (RTS_WINDOW),
        .MODEL_IDX    (MDL_CA)
    ) u_ca_rts (
        .clk          (clk),
        .rst_n        (rst_n & !sys_reset),
        
        .x_fwd        (x_upd[MDL_CA]),
        .P_fwd        (P_upd[MDL_CA]),
        .xu_pre       (xu_h[MDL_CA]),
        .Pu_pre       (Pu_h[MDL_CA]),
        .mu_weight    (mu[MDL_CA]),
        .fwd_valid    (imm_valid),
        
        .cfg_dt       (cfg_dt),
        
        .x_smooth     (car_x),
        .quality      (car_quality),
        .smooth_valid (car_valid)
    );

    //==========================================================================
    // [REQ-TOP-050] NIS-Based Hybrid Selector (Stream 1)
    //==========================================================================
    // Select between CV-RTS and CA-RTS based on running NIS comparison
    // For fire control: bounded 1.5s latency
    
    fp_t  hyb_x [STATE_DIM];
    fp_t  hyb_quality;
    logic hyb_valid;
    
    nx_mimosa_v40_hybrid_sel #(
        .NIS_WINDOW(NIS_WINDOW)
    ) u_hybrid (
        .clk           (clk),
        .rst_n         (rst_n & !sys_reset),
        
        // CV-RTS stream
        .cvr_x         (cvr_x),
        .cvr_quality   (cvr_quality),
        .cvr_valid     (cvr_valid),
        
        // CA-RTS stream
        .car_x         (car_x),
        .car_quality   (car_quality),
        .car_valid     (car_valid),
        
        // Forward NIS for online comparison
        .nis_cv        (nis_per_model[MDL_CV]),
        .nis_ca        (nis_per_model[MDL_CA]),
        .nis_valid     (imm_valid),
        
        // Hybrid output
        .hyb_x         (hyb_x),
        .hyb_quality   (hyb_quality),
        .hyb_valid     (hyb_valid)
    );

    //==========================================================================
    // [REQ-TOP-060] Feature Extractor for Platform Classifier
    //==========================================================================
    nx_mimosa_v40_feature_ext u_feat (
        .clk             (clk),
        .rst_n           (rst_n & !sys_reset),
        
        .x_mixed         (x_mixed),
        .mu              (mu),
        .nis_cv          (nis_per_model[MDL_CV]),
        .nis_ca          (nis_per_model[MDL_CA]),
        .omega           (omega_est),
        .in_valid        (imm_valid),
        
        .features        (clf_features),
        .features_valid  (clf_features_valid)
    );

    //==========================================================================
    // [REQ-TOP-070] Output Stream MUX
    //==========================================================================
    fp_t                   out_x [STATE_DIM];
    fp_t                   out_quality;
    logic [STREAM_ID_W-1:0] out_stream_id;
    logic                  out_valid;
    
    always_comb begin
        case (cfg_stream_sel)
            STREAM_FWD: begin
                out_x         = fwd_x;
                out_quality   = fwd_quality;
                out_stream_id = STREAM_FWD;
                out_valid     = fwd_valid;
            end
            STREAM_HYB: begin
                out_x         = hyb_x;
                out_quality   = hyb_quality;
                out_stream_id = STREAM_HYB;
                out_valid     = hyb_valid;
            end
            STREAM_CVR: begin
                out_x         = cvr_x;
                out_quality   = cvr_quality;
                out_stream_id = STREAM_CVR;
                out_valid     = cvr_valid;
            end
            STREAM_CAR: begin
                out_x         = car_x;
                out_quality   = car_quality;
                out_stream_id = STREAM_CAR;
                out_valid     = car_valid;
            end
            default: begin
                out_x         = fwd_x;
                out_quality   = fwd_quality;
                out_stream_id = STREAM_FWD;
                out_valid     = fwd_valid;
            end
        endcase
    end
    
    // AXI-Stream output packing
    // [197:195] = stream_id, [194:192] = tgt_id, [191:160] = quality,
    // [159:128] = vy, [127:96] = vx, [95:64] = y, [63:32] = x, [31:0] = reserved
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_axis_trk_tvalid <= 1'b0;
        end else begin
            m_axis_trk_tvalid <= out_valid & m_axis_trk_tready;
            if (out_valid) begin
                m_axis_trk_tdata <= {
                    out_stream_id,           // [197:196]
                    in_tgt_id,               // [195:193]  
                    out_quality,             // [192:161] — shifted for alignment
                    out_x[3],               // [160:129] vy
                    out_x[2],               // [128:97]  vx
                    out_x[1],               // [96:65]   y
                    out_x[0],               // [64:33]   x
                    32'h0                    // [32:1]    reserved/padding
                };
            end
        end
    end

    //==========================================================================
    // Status
    //==========================================================================
    assign ready = sys_enable & !sys_reset & imm_ready;
    
    // Count active targets
    always_comb begin
        active_targets = 8'h0;
        for (int i = 0; i < MAX_TARGETS; i++)
            active_targets[i] = cfg_tgt_mask[i];
    end

endmodule
