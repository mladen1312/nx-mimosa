/**
 * ═══════════════════════════════════════════════════════════════════════════════
 * NX-MIMOSA 7D PPO Policy Weights (INT8 Quantized)
 * ═══════════════════════════════════════════════════════════════════════════════
 * 
 * Auto-generated header for HLS/RTL integration
 * 
 * Architecture: 
 *   Input: 10D state → Shared(256) → Shared(256) → Actor(7D action)
 * 
 * Traceability:
 *   [REQ-RL-7D-001] 7D waveform agility
 *   [REQ-DEPLOY-7D-001] FPGA deployment
 * 
 * Author: Dr. Mladen Mešter / Nexellum d.o.o.
 * ═══════════════════════════════════════════════════════════════════════════════
 */

#ifndef PPO_7D_WEIGHTS_H
#define PPO_7D_WEIGHTS_H

#include <stdint.h>

// ═══════════════════════════════════════════════════════════════════════════════
// Network Configuration
// ═══════════════════════════════════════════════════════════════════════════════

#define PPO_7D_STATE_DIM     10
#define PPO_7D_ACTION_DIM    7
#define PPO_7D_HIDDEN_DIM    256

// Quantization parameters
#define PPO_7D_SCALE_INPUT   128    // Q7.8 for input
#define PPO_7D_SCALE_WEIGHT  128    // Q7.8 for weights
#define PPO_7D_SCALE_OUTPUT  256    // Q8.8 for output

// ═══════════════════════════════════════════════════════════════════════════════
// Placeholder Weights (Replace with trained values via Vitis AI export)
// ═══════════════════════════════════════════════════════════════════════════════

// Layer 1: Input(10) → Hidden(256)
// Shape: [256, 10] = 2560 weights
static const int8_t w_shared1[PPO_7D_HIDDEN_DIM][PPO_7D_STATE_DIM] = {
    // Placeholder - actual weights from training
    {0}
};

static const int8_t b_shared1[PPO_7D_HIDDEN_DIM] = {0};

// Layer 2: Hidden(256) → Hidden(256)
// Shape: [256, 256] = 65536 weights
static const int8_t w_shared2[PPO_7D_HIDDEN_DIM][PPO_7D_HIDDEN_DIM] = {
    {0}
};

static const int8_t b_shared2[PPO_7D_HIDDEN_DIM] = {0};

// Actor output: Hidden(256) → Action(7)
// Shape: [7, 256] = 1792 weights
static const int8_t w_actor[PPO_7D_ACTION_DIM][PPO_7D_HIDDEN_DIM] = {
    {0}
};

static const int8_t b_actor[PPO_7D_ACTION_DIM] = {0};

// ═══════════════════════════════════════════════════════════════════════════════
// HLS Inference Function Prototype
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief INT8 quantized 7D policy inference
 * 
 * @param state_in  Input state vector (10D, Q8.8 fixed-point)
 * @param action_out Output action vector (7D, Q8.8 fixed-point)
 */
void ppo_7d_inference_int8(
    const int16_t state_in[PPO_7D_STATE_DIM],
    int16_t action_out[PPO_7D_ACTION_DIM]
);

#endif // PPO_7D_WEIGHTS_H
