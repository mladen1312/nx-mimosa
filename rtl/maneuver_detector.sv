//==============================================================================
// NX-MIMOSA v3.3 — Maneuver Detector
// [REQ-MNV-01] Innovation-based chi-squared detection
// [REQ-MNV-02] Covariance trace spike monitoring
// [REQ-MNV-03] IMM mode transition detector (CT→CV)
// [REQ-MNV-04] Configurable thresholds via AXI-Lite
// [REQ-MNV-05] Maneuver end trigger for window smoother
//==============================================================================
// Author: Dr. Mladen Mešter / Nexellum d.o.o.
// License: Commercial — Nexellum d.o.o.
//==============================================================================

`timescale 1ns/1ps

module maneuver_detector
    import nx_mimosa_pkg_v33::*;
(
    input  logic        clk,
    input  logic        rst_n,

    //--------------------------------------------------------------------------
    // Inputs from IMM Core
    //--------------------------------------------------------------------------
    input  logic        filt_valid,          // New filter output ready
    input  fp_t         innovation [MEAS_DIM], // Innovation vector (nu)
    input  fp_t         S_diag [MEAS_DIM],   // Innovation covariance diagonal
    input  fp_t         P_combined [STATE_DIM][STATE_DIM], // Combined covariance
    input  fp_t         mu [N_MODELS],       // Mode probabilities [CV, CT+, CT-]

    //--------------------------------------------------------------------------
    // Configuration (from AXI-Lite)
    //--------------------------------------------------------------------------
    input  fp_t         cfg_innov_threshold, // NIS threshold (default 4.0)
    input  fp_t         cfg_cov_threshold,   // trace(P) threshold (default 100.0)
    input  logic        cfg_enable,

    //--------------------------------------------------------------------------
    // Outputs
    //--------------------------------------------------------------------------
    output maneuver_state_t  maneuver_state,
    output logic             smooth_trigger,   // Pulse: trigger window smoother
    output logic             cov_spike,        // Covariance spike detected
    output fp_t              nis_value,        // Current NIS for diagnostics
    output fp_t              cov_trace_value   // Current trace(P) for diagnostics
);

    //--------------------------------------------------------------------------
    // Internal State
    //--------------------------------------------------------------------------
    maneuver_state_t state_r, state_next;
    logic [3:0] cv_consecutive_count;  // Count consecutive CV-dominant steps
    fp_t mu_cv_prev;                   // Previous CV probability
    fp_t nis_r;
    fp_t cov_trace_r;
    logic trigger_r;
    logic spike_r;

    // Dominant model detection
    logic cv_dominant;
    logic ct_dominant;
    fp_t mu_ct_total;  // mu[CT+] + mu[CT-]

    assign mu_ct_total = mu[1] + mu[2];
    assign cv_dominant = (mu[0] >= mu_ct_total);
    assign ct_dominant = !cv_dominant;

    //--------------------------------------------------------------------------
    // NIS and Covariance Trace Computation
    // [REQ-MNV-01], [REQ-MNV-02]
    //--------------------------------------------------------------------------
    fp_t nis_computed;
    fp_t trace_computed;

    always_comb begin
        // NIS = sum(nu_i^2 / S_ii)
        nis_computed = fp_nis_diag(innovation, S_diag);
        // trace(P)
        trace_computed = fp_trace4(P_combined);
    end

    //--------------------------------------------------------------------------
    // Main FSM
    // [REQ-MNV-03], [REQ-MNV-05]
    //--------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_r             <= MNV_IDLE;
            cv_consecutive_count <= '0;
            mu_cv_prev          <= FP_ZERO;
            nis_r               <= FP_ZERO;
            cov_trace_r         <= FP_ZERO;
            trigger_r           <= 1'b0;
            spike_r             <= 1'b0;
        end else if (filt_valid && cfg_enable) begin
            // Latch computed values
            nis_r       <= nis_computed;
            cov_trace_r <= trace_computed;
            trigger_r   <= 1'b0;  // Default: no trigger pulse
            spike_r     <= (trace_computed > cfg_cov_threshold);
            mu_cv_prev  <= mu[0];

            case (state_r)
                //--------------------------------------------------------------
                MNV_IDLE: begin
                    cv_consecutive_count <= '0;
                    if (ct_dominant && nis_computed > cfg_innov_threshold) begin
                        // Maneuver onset: CT dominant + high NIS
                        state_r <= MNV_ONSET;
                    end else if (trace_computed > cfg_cov_threshold) begin
                        // Covariance spike without clear CT — still flag
                        spike_r <= 1'b1;
                    end
                end

                //--------------------------------------------------------------
                MNV_ONSET: begin
                    cv_consecutive_count <= '0;
                    if (ct_dominant) begin
                        state_r <= MNV_SUSTAINED;
                    end else begin
                        // False alarm — quick return to CV
                        state_r <= MNV_IDLE;
                    end
                end

                //--------------------------------------------------------------
                MNV_SUSTAINED: begin
                    cv_consecutive_count <= '0;
                    if (cv_dominant) begin
                        // Mode transitioning back
                        state_r <= MNV_ENDING;
                        cv_consecutive_count <= 4'd1;
                    end
                end

                //--------------------------------------------------------------
                MNV_ENDING: begin
                    if (cv_dominant) begin
                        if (cv_consecutive_count >= MANEUVER_END_COUNT[3:0]) begin
                            // Confirmed maneuver end
                            state_r   <= MNV_ENDED;
                            trigger_r <= 1'b1;  // Trigger window smoother!
                        end else begin
                            cv_consecutive_count <= cv_consecutive_count + 1;
                        end
                    end else begin
                        // False ending — back to sustained
                        state_r <= MNV_SUSTAINED;
                        cv_consecutive_count <= '0;
                    end
                end

                //--------------------------------------------------------------
                MNV_ENDED: begin
                    // Auto-return to idle after trigger pulse
                    state_r   <= MNV_IDLE;
                    trigger_r <= 1'b0;
                    cv_consecutive_count <= '0;
                end

                default: state_r <= MNV_IDLE;
            endcase
        end else begin
            trigger_r <= 1'b0;  // Clear trigger if not valid
        end
    end

    //--------------------------------------------------------------------------
    // Output Assignments
    //--------------------------------------------------------------------------
    assign maneuver_state  = state_r;
    assign smooth_trigger  = trigger_r;
    assign cov_spike       = spike_r;
    assign nis_value       = nis_r;
    assign cov_trace_value = cov_trace_r;

endmodule
