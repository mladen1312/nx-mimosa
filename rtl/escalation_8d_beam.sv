// ═══════════════════════════════════════════════════════════════════════════════════════════════════
// NX-MIMOSA 8D Beam Steering Escalation Module
// ═══════════════════════════════════════════════════════════════════════════════════════════════════
//
// Extends hybrid_eccm_6d7d.sv with 8th dimension: Adaptive Beam Steering
// for countering hypersonic threats with spatial decorrelation.
//
// Escalation Levels:
//   REG_ECCM_LEVEL[1:0]:
//     00 = 6D (baseline)
//     01 = 7D (+ code diversity)
//     10 = 8D (+ code diversity + beam steering)
//     11 = Reserved
//
// 8D Beam Steering:
//   - ±20° azimuth/elevation offset in 16 quantized steps
//   - Breaks spatial jammer lock (DRFM replay becomes spatially incoherent)
//   - Critical for Mach 6+ targets where angular rate > beam dwell
//
// Traceability:
//   [REQ-RL-HYPERSONIC-8D-ESC-001] 8D escalation under hypersonic threat
//   [REQ-HYBRID-001] Runtime mode configuration
//   [REQ-SAFETY-01] Manual override capability
//
// Author: Dr. Mladen Mešter / Nexellum d.o.o.
// Version: 4.0.0
// License: AGPL v3 / Commercial
// ═══════════════════════════════════════════════════════════════════════════════════════════════════

`timescale 1ns/1ps
`default_nettype none

package eccm_8d_pkg;

    // Escalation levels
    typedef enum logic [1:0] {
        LEVEL_6D = 2'b00,
        LEVEL_7D = 2'b01,
        LEVEL_8D = 2'b10,
        LEVEL_RESERVED = 2'b11
    } escalation_level_t;

    // Threat categories for auto-escalation
    typedef enum logic [2:0] {
        THREAT_NONE       = 3'b000,
        THREAT_NOISE      = 3'b001,
        THREAT_SPOT       = 3'b010,
        THREAT_DRFM       = 3'b011,   // → triggers 7D
        THREAT_SPATIAL     = 3'b100,   // → triggers 8D (spatial deception)
        THREAT_HYPERSONIC = 3'b101    // → triggers 8D (high angular rate)
    } threat_category_t;

    // Beam steering configuration
    localparam int BEAM_AZ_STEPS     = 16;      // ±20° in 16 steps = 2.5°/step
    localparam int BEAM_EL_STEPS     = 16;
    localparam int BEAM_MAX_DEG_Q8   = 5120;    // 20.0° in Q8.8 format
    localparam int BEAM_STEP_DEG_Q8  = 640;     // 2.5° in Q8.8

endpackage


// =============================================================================
// BEAM STEERING CONTROLLER
// =============================================================================
// [REQ-RL-HYPERSONIC-8D-ESC-001]

module beam_steering_controller
    import eccm_8d_pkg::*;
#(
    parameter int DATA_WIDTH = 16
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // Enable (from escalation controller)
    input  wire                     beam_enable,

    // Target tracking input (from tracker)
    input  wire signed [DATA_WIDTH-1:0]  target_az_deg,     // Q8.8 azimuth
    input  wire signed [DATA_WIDTH-1:0]  target_el_deg,     // Q8.8 elevation
    input  wire signed [DATA_WIDTH-1:0]  target_az_rate,    // Q8.8 deg/s
    input  wire signed [DATA_WIDTH-1:0]  target_el_rate,    // Q8.8 deg/s
    input  wire                     target_valid,

    // Jammer direction estimate (from classifier)
    input  wire signed [DATA_WIDTH-1:0]  jammer_az_est,
    input  wire signed [DATA_WIDTH-1:0]  jammer_el_est,
    input  wire                     jammer_dir_valid,

    // Beam steering output (to phased array / beamformer)
    output logic signed [DATA_WIDTH-1:0] beam_az_offset,    // Q8.8
    output logic signed [DATA_WIDTH-1:0] beam_el_offset,    // Q8.8
    output logic [3:0]              beam_az_step,           // Quantized step index
    output logic [3:0]              beam_el_step,
    output logic                    beam_valid,

    // Telemetry
    output logic [15:0]             stat_beam_switches,
    output logic                    stat_beam_active
);

    // Internal state
    logic signed [DATA_WIDTH-1:0] current_az_offset;
    logic signed [DATA_WIDTH-1:0] current_el_offset;
    logic [3:0] az_step_reg;
    logic [3:0] el_step_reg;
    logic [15:0] switch_counter;

    // Pseudo-random offset generator (for spatial decorrelation)
    logic [15:0] lfsr;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lfsr <= 16'hBEEF;
        end else begin
            lfsr <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
        end
    end

    // ─────────────────────────────────────────────────────────────────────────
    // BEAM STEERING ALGORITHM
    // ─────────────────────────────────────────────────────────────────────────
    // Strategy: Combine target-predictive steering with anti-jammer offset
    //
    // 1. Predict target position at next PRI (angular rate extrapolation)
    // 2. Add pseudo-random spatial offset to break jammer spatial lock
    // 3. Offset is anti-correlated with jammer direction estimate
    // ─────────────────────────────────────────────────────────────────────────

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_az_offset <= '0;
            current_el_offset <= '0;
            az_step_reg       <= '0;
            el_step_reg       <= '0;
            switch_counter    <= '0;
            beam_valid        <= 1'b0;
        end else if (beam_enable && target_valid) begin
            logic signed [DATA_WIDTH-1:0] az_predict;
            logic signed [DATA_WIDTH-1:0] el_predict;
            logic signed [DATA_WIDTH-1:0] az_anti_jam;
            logic signed [DATA_WIDTH-1:0] el_anti_jam;
            logic signed [DATA_WIDTH-1:0] az_random;
            logic signed [DATA_WIDTH-1:0] el_random;

            // Step 1: Predictive component (extrapolate angular rate)
            // T_pred ≈ 1/PRF, use rate * dt (simplified as rate >> 2)
            az_predict = target_az_rate >>> 2;
            el_predict = target_el_rate >>> 2;

            // Step 2: Anti-jammer offset (steer away from jammer direction)
            if (jammer_dir_valid) begin
                // Offset perpendicular to jammer bearing
                az_anti_jam = -(jammer_az_est >>> 3);
                el_anti_jam = -(jammer_el_est >>> 3);
            end else begin
                az_anti_jam = '0;
                el_anti_jam = '0;
            end

            // Step 3: Random spatial decorrelation
            az_random = {{(DATA_WIDTH-6){lfsr[7]}}, lfsr[7:2]};  // Small random offset
            el_random = {{(DATA_WIDTH-6){lfsr[15]}}, lfsr[15:10]};

            // Combine: predictive + anti-jammer + random
            current_az_offset <= az_predict + az_anti_jam + az_random;
            current_el_offset <= el_predict + el_anti_jam + el_random;

            // Clamp to ±20° (±BEAM_MAX_DEG_Q8 in Q8.8)
            if (current_az_offset > BEAM_MAX_DEG_Q8)
                current_az_offset <= BEAM_MAX_DEG_Q8;
            else if (current_az_offset < -BEAM_MAX_DEG_Q8)
                current_az_offset <= -BEAM_MAX_DEG_Q8;

            if (current_el_offset > BEAM_MAX_DEG_Q8)
                current_el_offset <= BEAM_MAX_DEG_Q8;
            else if (current_el_offset < -BEAM_MAX_DEG_Q8)
                current_el_offset <= -BEAM_MAX_DEG_Q8;

            // Quantize to beam steps
            az_step_reg <= 4'(8 + (current_az_offset / BEAM_STEP_DEG_Q8));
            el_step_reg <= 4'(8 + (current_el_offset / BEAM_STEP_DEG_Q8));

            // Update counter
            switch_counter <= switch_counter + 1;
            beam_valid <= 1'b1;

        end else if (!beam_enable) begin
            // 8D disabled → zero offset
            current_az_offset <= '0;
            current_el_offset <= '0;
            az_step_reg       <= 4'd8;  // Center
            el_step_reg       <= 4'd8;
            beam_valid        <= 1'b0;
        end else begin
            beam_valid <= 1'b0;
        end
    end

    assign beam_az_offset  = current_az_offset;
    assign beam_el_offset  = current_el_offset;
    assign beam_az_step    = az_step_reg;
    assign beam_el_step    = el_step_reg;
    assign stat_beam_switches = switch_counter;
    assign stat_beam_active   = beam_enable;

endmodule


// =============================================================================
// 8D ESCALATION CONTROLLER (wraps 6D/7D hybrid + beam steering)
// =============================================================================
// [REQ-RL-HYPERSONIC-8D-ESC-001] [REQ-HYBRID-001]

module escalation_controller_8d
    import eccm_8d_pkg::*;
#(
    parameter int DATA_WIDTH = 16
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // Configuration
    input  wire [1:0]               reg_eccm_level,     // 00:6D, 01:7D, 10:8D
    input  wire                     reg_auto_escalate,  // 1 = auto-escalation enabled
    input  wire [15:0]              reg_deesc_hold,     // Hold timer for de-escalation

    // Classifier inputs
    input  wire [2:0]               threat_category,    // From jammer/target classifier
    input  wire [DATA_WIDTH-1:0]    coherence_metric,   // Jammer coherence
    input  wire [DATA_WIDTH-1:0]    target_velocity,    // Target radial velocity (m/s Q8.8)
    input  wire [DATA_WIDTH-1:0]    target_accel,       // Target acceleration (g Q8.8)
    input  wire                     classifier_valid,

    // Escalation outputs
    output logic                    code_diversity_en,  // → 7D code generator
    output logic                    beam_steering_en,   // → 8D beam controller
    output logic [1:0]              current_level,      // Active escalation level
    output logic [31:0]             stat_escalations,   // Total escalation events
    output logic [31:0]             stat_deescalations  // Total de-escalation events
);

    // FSM states
    typedef enum logic [2:0] {
        S_IDLE,
        S_6D_ACTIVE,
        S_ESCALATE_TO_7D,
        S_7D_ACTIVE,
        S_ESCALATE_TO_8D,
        S_8D_ACTIVE,
        S_DEESCALATING
    } state_t;

    state_t state, state_next;
    logic [15:0] deesc_counter;
    logic [31:0] esc_count, deesc_count;

    // Hypersonic detection thresholds
    localparam int VELOCITY_HYPERSONIC_Q8 = 512_000;  // 2000 m/s in Q8.8
    localparam int ACCEL_HIGH_Q8 = 7680;               // 30g in Q8.8

    // Escalation conditions
    logic need_7d, need_8d;

    always_comb begin
        // 7D needed: DRFM detected or high coherence
        need_7d = (threat_category >= THREAT_DRFM) ||
                  (coherence_metric > 16'h00C8);  // > 0.78 in Q8.8

        // 8D needed: spatial/hypersonic threat OR extreme velocity/acceleration
        need_8d = (threat_category >= THREAT_SPATIAL) ||
                  (target_velocity > VELOCITY_HYPERSONIC_Q8) ||
                  (target_accel > ACCEL_HIGH_Q8);
    end

    // FSM: Sequential
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            deesc_counter <= '0;
            esc_count    <= '0;
            deesc_count  <= '0;
        end else begin
            state <= state_next;

            // Counters
            if (state == S_ESCALATE_TO_7D || state == S_ESCALATE_TO_8D)
                esc_count <= esc_count + 1;

            if (state == S_DEESCALATING && state_next != S_DEESCALATING)
                deesc_count <= deesc_count + 1;

            // De-escalation timer
            if (state == S_DEESCALATING)
                deesc_counter <= deesc_counter + 1;
            else
                deesc_counter <= '0;
        end
    end

    // FSM: Next-state
    always_comb begin
        state_next = state;

        if (!reg_auto_escalate) begin
            // Manual mode: directly map register to level
            case (reg_eccm_level)
                2'b00:   state_next = S_6D_ACTIVE;
                2'b01:   state_next = S_7D_ACTIVE;
                2'b10:   state_next = S_8D_ACTIVE;
                default: state_next = S_6D_ACTIVE;
            endcase
        end else begin
            // Auto-escalation
            case (state)
                S_IDLE: state_next = S_6D_ACTIVE;

                S_6D_ACTIVE: begin
                    if (need_8d && classifier_valid)
                        state_next = S_ESCALATE_TO_8D;
                    else if (need_7d && classifier_valid)
                        state_next = S_ESCALATE_TO_7D;
                end

                S_ESCALATE_TO_7D: state_next = S_7D_ACTIVE;

                S_7D_ACTIVE: begin
                    if (need_8d && classifier_valid)
                        state_next = S_ESCALATE_TO_8D;
                    else if (!need_7d && classifier_valid)
                        state_next = S_DEESCALATING;
                end

                S_ESCALATE_TO_8D: state_next = S_8D_ACTIVE;

                S_8D_ACTIVE: begin
                    if (!need_8d && !need_7d && classifier_valid)
                        state_next = S_DEESCALATING;
                    else if (!need_8d && need_7d && classifier_valid)
                        state_next = S_7D_ACTIVE;
                end

                S_DEESCALATING: begin
                    // Re-escalate immediately if threat returns
                    if (need_8d)
                        state_next = S_8D_ACTIVE;
                    else if (need_7d)
                        state_next = S_7D_ACTIVE;
                    else if (deesc_counter >= reg_deesc_hold)
                        state_next = S_6D_ACTIVE;
                end

                default: state_next = S_IDLE;
            endcase
        end
    end

    // Output decoding
    always_comb begin
        case (state)
            S_6D_ACTIVE, S_IDLE, S_ESCALATE_TO_7D: begin
                code_diversity_en = 1'b0;
                beam_steering_en  = 1'b0;
                current_level     = LEVEL_6D;
            end

            S_7D_ACTIVE: begin
                code_diversity_en = 1'b1;
                beam_steering_en  = 1'b0;
                current_level     = LEVEL_7D;
            end

            S_ESCALATE_TO_8D, S_8D_ACTIVE: begin
                code_diversity_en = 1'b1;
                beam_steering_en  = 1'b1;
                current_level     = LEVEL_8D;
            end

            S_DEESCALATING: begin
                // Keep current capabilities during de-escalation
                code_diversity_en = 1'b1;
                beam_steering_en  = 1'b1;
                current_level     = LEVEL_8D;
            end

            default: begin
                code_diversity_en = 1'b0;
                beam_steering_en  = 1'b0;
                current_level     = LEVEL_6D;
            end
        endcase
    end

    assign stat_escalations   = esc_count;
    assign stat_deescalations = deesc_count;

endmodule

`default_nettype wire
