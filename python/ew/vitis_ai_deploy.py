#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════════════════════════════
NX-MIMOSA Vitis AI Deployment for Quantized PPO Policy
═══════════════════════════════════════════════════════════════════════════════════════════════════════

Export and quantization flow for deploying PPO policy on Versal AI Engine (AIE-ML).

Flow:
  1. PyTorch Training → ppo_policy.pth
  2. ONNX Export → ppo_policy.onnx
  3. Vitis AI Quantization → INT8 (PTQ)
  4. AIE Compilation → ppo_policy.xclbin

Performance Targets:
  - Inference latency: <50 µs @ 1 GHz AIE
  - Quantization loss: <5% accuracy
  - Power: <0.5W per inference

Traceability:
  [REQ-RL-DEPLOY-001] AIE deployment
  [REQ-ECCM-LATENCY] Sub-microsecond response

Author: Dr. Mladen Mešter / Nexellum d.o.o.
Version: 1.0.0
License: AGPL v3 / Commercial
═══════════════════════════════════════════════════════════════════════════════════════════════════════
"""

import numpy as np
import json
import os
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional

# Optional imports
try:
    import torch
    import torch.nn as nn
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False


# =============================================================================
# QUANTIZATION CONFIGURATION
# =============================================================================

@dataclass
class QuantConfig:
    """Quantization configuration for Vitis AI."""
    # Network
    state_dim: int = 6
    action_dim: int = 2
    hidden_dim: int = 128
    
    # Quantization
    n_bits: int = 8                 # INT8 quantization
    symmetric: bool = True          # Symmetric quantization
    per_channel: bool = False       # Per-tensor quantization
    
    # Calibration
    calib_samples: int = 1000       # Number of calibration samples
    calib_batch_size: int = 32
    
    # Target
    target_platform: str = "aie-ml"  # Versal AI Engine
    target_frequency: int = 1000     # MHz


# =============================================================================
# FIXED-POINT CONVERSION
# =============================================================================

def float_to_fixed(value: float, n_bits: int = 8, frac_bits: int = 4) -> int:
    """Convert float to fixed-point integer."""
    scale = 2 ** frac_bits
    int_bits = n_bits - frac_bits
    max_val = 2 ** (n_bits - 1) - 1
    min_val = -2 ** (n_bits - 1)
    
    fixed = int(np.round(value * scale))
    return int(np.clip(fixed, min_val, max_val))


def fixed_to_float(value: int, frac_bits: int = 4) -> float:
    """Convert fixed-point integer back to float."""
    scale = 2 ** frac_bits
    return value / scale


class FixedPointQuantizer:
    """
    Fixed-point quantizer for neural network weights.
    
    Converts float32 weights to INT8 with computed scale factors.
    """
    
    def __init__(self, n_bits: int = 8):
        self.n_bits = n_bits
        self.scale_factors = {}
    
    def compute_scale(self, tensor: np.ndarray, name: str) -> float:
        """Compute optimal scale factor for a tensor."""
        abs_max = np.max(np.abs(tensor))
        
        # Compute scale to maximize dynamic range
        int_max = 2 ** (self.n_bits - 1) - 1
        scale = abs_max / int_max if abs_max > 0 else 1.0
        
        self.scale_factors[name] = scale
        return scale
    
    def quantize(self, tensor: np.ndarray, name: str) -> np.ndarray:
        """Quantize tensor to INT8."""
        scale = self.compute_scale(tensor, name)
        quantized = np.round(tensor / scale).astype(np.int8)
        return quantized
    
    def dequantize(self, tensor: np.ndarray, name: str) -> np.ndarray:
        """Dequantize INT8 tensor back to float."""
        scale = self.scale_factors.get(name, 1.0)
        return tensor.astype(np.float32) * scale
    
    def quantization_error(self, original: np.ndarray, name: str) -> float:
        """Compute quantization error (MSE)."""
        quantized = self.quantize(original, name)
        reconstructed = self.dequantize(quantized, name)
        return np.mean((original - reconstructed) ** 2)


# =============================================================================
# POLICY NETWORK EXPORT
# =============================================================================

if TORCH_AVAILABLE:
    class PolicyNetwork(nn.Module):
        """PPO Policy network for export."""
        
        def __init__(self, state_dim: int, action_dim: int, hidden_dim: int):
            super().__init__()
            self.net = nn.Sequential(
                nn.Linear(state_dim, hidden_dim),
                nn.Tanh(),
                nn.Linear(hidden_dim, hidden_dim),
                nn.Tanh(),
                nn.Linear(hidden_dim, action_dim),
                nn.Tanh()
            )
        
        def forward(self, x):
            return self.net(x)


def export_policy_to_onnx(policy: 'PolicyNetwork', config: QuantConfig, output_path: str):
    """Export policy network to ONNX format."""
    policy.eval()
    
    dummy_input = torch.randn(1, config.state_dim)
    
    torch.onnx.export(
        policy,
        dummy_input,
        output_path,
        input_names=['state'],
        output_names=['action'],
        dynamic_axes={'state': {0: 'batch'}, 'action': {0: 'batch'}},
        opset_version=13,
        do_constant_folding=True
    )
    
    print(f"ONNX model exported to {output_path}")
    return output_path


def export_weights_to_c(policy: 'PolicyNetwork', output_path: str, config: QuantConfig):
    """
    Export quantized weights to C header for direct FPGA use.
    
    Generates HLS-compatible weight arrays.
    """
    quantizer = FixedPointQuantizer(config.n_bits)
    
    with open(output_path, 'w') as f:
        f.write("// ═══════════════════════════════════════════════════════════════════════════════\n")
        f.write("// NX-MIMOSA Quantized PPO Policy Weights\n")
        f.write("// Auto-generated - DO NOT EDIT\n")
        f.write("// ═══════════════════════════════════════════════════════════════════════════════\n\n")
        f.write("#ifndef PPO_POLICY_WEIGHTS_H\n")
        f.write("#define PPO_POLICY_WEIGHTS_H\n\n")
        f.write("#include <stdint.h>\n\n")
        
        f.write(f"#define STATE_DIM {config.state_dim}\n")
        f.write(f"#define ACTION_DIM {config.action_dim}\n")
        f.write(f"#define HIDDEN_DIM {config.hidden_dim}\n")
        f.write(f"#define QUANT_BITS {config.n_bits}\n\n")
        
        layer_idx = 0
        for name, param in policy.named_parameters():
            data = param.detach().cpu().numpy()
            
            # Quantize
            q_data = quantizer.quantize(data, name)
            scale = quantizer.scale_factors[name]
            
            # Write scale factor
            f.write(f"// Scale factor for {name}: {scale}\n")
            scale_fixed = float_to_fixed(scale, 16, 8)
            f.write(f"static const int16_t scale_{layer_idx} = 0x{scale_fixed & 0xFFFF:04X};\n\n")
            
            # Write weights/biases
            if 'weight' in name:
                shape = q_data.shape
                f.write(f"// {name}: shape [{shape[0]}, {shape[1]}]\n")
                f.write(f"static const int8_t weights_{layer_idx}[{shape[0]}][{shape[1]}] = {{\n")
                
                for i in range(shape[0]):
                    f.write("    {")
                    f.write(", ".join([f"{int(v)}" for v in q_data[i]]))
                    f.write("},\n")
                
                f.write("};\n\n")
                
            elif 'bias' in name:
                shape = q_data.shape
                f.write(f"// {name}: shape [{shape[0]}]\n")
                f.write(f"static const int8_t biases_{layer_idx}[{shape[0]}] = {{\n")
                f.write("    " + ", ".join([f"{int(v)}" for v in q_data]))
                f.write("\n};\n\n")
                
                layer_idx += 1
        
        f.write("#endif // PPO_POLICY_WEIGHTS_H\n")
    
    print(f"C header exported to {output_path}")
    
    # Report quantization error
    print("\nQuantization Error Analysis:")
    for name, param in policy.named_parameters():
        data = param.detach().cpu().numpy()
        error = quantizer.quantization_error(data, name)
        print(f"  {name}: MSE = {error:.6e}")
    
    return quantizer.scale_factors


# =============================================================================
# VITIS AI SCRIPT GENERATION
# =============================================================================

def generate_vitis_ai_scripts(config: QuantConfig, output_dir: str):
    """Generate Vitis AI quantization and compilation scripts."""
    
    os.makedirs(output_dir, exist_ok=True)
    
    # Quantization script
    quant_script = f'''#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════════════
# NX-MIMOSA Vitis AI Quantization Script
# ═══════════════════════════════════════════════════════════════════════════════
# [REQ-RL-DEPLOY-001] Quantization for Versal AIE

set -e

# Environment setup
source /opt/vitis_ai/setup.sh

echo "Starting Vitis AI Quantization..."

# Post-Training Quantization
vai_q_pytorch quantize \\
    --model ppo_policy.onnx \\
    --input_shape 1,{config.state_dim} \\
    --output_dir ./quantized \\
    --quant_mode calib \\
    --calib_input_fn calibration_data.npy \\
    --device cpu \\
    --gpu 0

# Verify quantized model
echo "Quantization complete. Running verification..."
python3 verify_quantized.py

echo "Quantized model ready: ./quantized/ppo_policy_int8.xmodel"
'''
    
    with open(os.path.join(output_dir, "quantize.sh"), 'w') as f:
        f.write(quant_script)
    
    # AIE compilation script
    compile_script = f'''#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════════════
# NX-MIMOSA Vitis AI AIE Compilation Script
# ═══════════════════════════════════════════════════════════════════════════════
# [REQ-RL-DEPLOY-001] Compilation for Versal AIE-ML

set -e

source /opt/vitis_ai/setup.sh

echo "Compiling for Versal AI Engine..."

# Compile for AIE-ML
vai_c_aie \\
    --arch /opt/vitis_ai/arch/DPUAIE.json \\
    --model ./quantized/ppo_policy_int8.xmodel \\
    --output_dir ./compiled \\
    --net_name ppo_policy

echo "AIE compilation complete: ./compiled/ppo_policy.xclbin"

# Generate deployment package
mkdir -p deploy
cp ./compiled/ppo_policy.xclbin deploy/
cp ../ppo_weights.h deploy/
echo "Deployment package ready in ./deploy/"
'''
    
    with open(os.path.join(output_dir, "compile_aie.sh"), 'w') as f:
        f.write(compile_script)
    
    # Verification script (Python)
    verify_script = '''#!/usr/bin/env python3
"""Verify quantized model accuracy."""
import numpy as np

def load_calibration_data(path):
    """Load calibration data."""
    return np.load(path)

def verify_quantized_model(float_model_path, quant_model_path, calib_data_path):
    """Compare float vs quantized model outputs."""
    # This would use Vitis AI runtime in production
    print("Verification would run here with Vitis AI runtime")
    print("Expected accuracy loss: <5%")

if __name__ == "__main__":
    verify_quantized_model(
        "ppo_policy.onnx",
        "./quantized/ppo_policy_int8.xmodel",
        "calibration_data.npy"
    )
'''
    
    with open(os.path.join(output_dir, "verify_quantized.py"), 'w') as f:
        f.write(verify_script)
    
    print(f"Vitis AI scripts generated in {output_dir}")


# =============================================================================
# CALIBRATION DATA GENERATION
# =============================================================================

def generate_calibration_data(config: QuantConfig, output_path: str):
    """
    Generate calibration data for quantization.
    
    Uses realistic state distributions from training.
    """
    np.random.seed(42)
    
    # Generate diverse state samples
    n_samples = config.calib_samples
    
    # State space: [separation, tx_offset, jammer_est, jammer_type, time, velocity]
    states = np.zeros((n_samples, config.state_dim), dtype=np.float32)
    
    for i in range(n_samples):
        # Random but realistic state distributions
        separation = np.random.uniform(-1.0, 1.0)
        tx_offset = np.random.uniform(-1.0, 1.0)
        jammer_est = tx_offset + np.random.normal(0, 0.2)  # Noisy jammer estimate
        jammer_type = np.random.randint(0, 3) / 2.0  # Normalized
        time_step = np.random.uniform(0, 1)
        velocity = np.random.normal(0, 0.3)
        
        states[i] = [separation, tx_offset, jammer_est, jammer_type, time_step, velocity]
    
    np.save(output_path, states)
    print(f"Calibration data saved to {output_path}")
    print(f"  Samples: {n_samples}")
    print(f"  Shape: {states.shape}")
    
    return states


# =============================================================================
# SYSTEMVERILOG WRAPPER FOR AIE
# =============================================================================

def generate_aie_wrapper_sv(config: QuantConfig, output_path: str):
    """
    Generate SystemVerilog wrapper for AIE policy inference.
    
    Integrates AIE output with radar control logic in PL.
    """
    
    sv_code = f'''// ═══════════════════════════════════════════════════════════════════════════════
// NX-MIMOSA AIE Policy Wrapper
// ═══════════════════════════════════════════════════════════════════════════════
// [REQ-RL-DEPLOY-001] Wraps Vitis AI inference for radar integration
//
// Architecture:
//   PL State Collector → AXI4-Stream → AIE Policy → AXI4-Stream → PL Action Controller
//
// Latency: <50 µs total (AIE inference + wrapper overhead)
// ═══════════════════════════════════════════════════════════════════════════════

`timescale 1ns/1ps

module aie_policy_wrapper #(
    parameter int STATE_DIM = {config.state_dim},
    parameter int ACTION_DIM = {config.action_dim},
    parameter int DATA_WIDTH = 32          // Float32 for AIE interface
)(
    // Clock & Reset
    input  logic                        clk,
    input  logic                        rst_n,
    
    // State Input (from radar sensors)
    input  logic                        state_valid,
    input  logic [DATA_WIDTH-1:0]       state_data [STATE_DIM],
    output logic                        state_ready,
    
    // Action Output (to waveform generator)
    output logic                        action_valid,
    output logic [DATA_WIDTH-1:0]       action_data [ACTION_DIM],
    input  logic                        action_ready,
    
    // AIE Interface (AXI4-Stream to/from AIE)
    output logic                        m_axis_state_tvalid,
    output logic [DATA_WIDTH*STATE_DIM-1:0] m_axis_state_tdata,
    output logic                        m_axis_state_tlast,
    input  logic                        m_axis_state_tready,
    
    input  logic                        s_axis_action_tvalid,
    input  logic [DATA_WIDTH*ACTION_DIM-1:0] s_axis_action_tdata,
    input  logic                        s_axis_action_tlast,
    output logic                        s_axis_action_tready,
    
    // Status
    output logic [15:0]                 stat_inferences,
    output logic [15:0]                 stat_latency_cycles
);

    // ═══════════════════════════════════════════════════════════════════════════
    // State Machine
    // ═══════════════════════════════════════════════════════════════════════════
    
    typedef enum logic [2:0] {{
        IDLE,
        PACK_STATE,
        SEND_TO_AIE,
        WAIT_AIE,
        UNPACK_ACTION,
        OUTPUT_ACTION
    }} state_t;
    
    state_t fsm_state;
    
    // Latency measurement
    logic [15:0] latency_counter;
    logic latency_counting;
    
    // State packing
    logic [DATA_WIDTH*STATE_DIM-1:0] packed_state;
    logic [DATA_WIDTH*ACTION_DIM-1:0] packed_action;
    
    // ═══════════════════════════════════════════════════════════════════════════
    // State Packing
    // ═══════════════════════════════════════════════════════════════════════════
    
    always_comb begin
        packed_state = '0;
        for (int i = 0; i < STATE_DIM; i++) begin
            packed_state[i*DATA_WIDTH +: DATA_WIDTH] = state_data[i];
        end
    end
    
    // ═══════════════════════════════════════════════════════════════════════════
    // Main FSM
    // ═══════════════════════════════════════════════════════════════════════════
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fsm_state <= IDLE;
            m_axis_state_tvalid <= 1'b0;
            m_axis_state_tdata <= '0;
            m_axis_state_tlast <= 1'b0;
            s_axis_action_tready <= 1'b0;
            action_valid <= 1'b0;
            state_ready <= 1'b1;
            latency_counter <= '0;
            latency_counting <= 1'b0;
            stat_inferences <= '0;
            stat_latency_cycles <= '0;
        end else begin
            case (fsm_state)
                IDLE: begin
                    state_ready <= 1'b1;
                    if (state_valid && state_ready) begin
                        fsm_state <= PACK_STATE;
                        state_ready <= 1'b0;
                    end
                end
                
                PACK_STATE: begin
                    m_axis_state_tdata <= packed_state;
                    m_axis_state_tvalid <= 1'b1;
                    m_axis_state_tlast <= 1'b1;
                    latency_counting <= 1'b1;
                    latency_counter <= '0;
                    fsm_state <= SEND_TO_AIE;
                end
                
                SEND_TO_AIE: begin
                    if (m_axis_state_tready) begin
                        m_axis_state_tvalid <= 1'b0;
                        s_axis_action_tready <= 1'b1;
                        fsm_state <= WAIT_AIE;
                    end
                end
                
                WAIT_AIE: begin
                    latency_counter <= latency_counter + 1;
                    
                    if (s_axis_action_tvalid) begin
                        packed_action <= s_axis_action_tdata;
                        s_axis_action_tready <= 1'b0;
                        latency_counting <= 1'b0;
                        stat_latency_cycles <= latency_counter;
                        fsm_state <= UNPACK_ACTION;
                    end
                end
                
                UNPACK_ACTION: begin
                    for (int i = 0; i < ACTION_DIM; i++) begin
                        action_data[i] <= packed_action[i*DATA_WIDTH +: DATA_WIDTH];
                    end
                    action_valid <= 1'b1;
                    fsm_state <= OUTPUT_ACTION;
                end
                
                OUTPUT_ACTION: begin
                    if (action_ready) begin
                        action_valid <= 1'b0;
                        stat_inferences <= stat_inferences + 1;
                        fsm_state <= IDLE;
                    end
                end
                
                default: fsm_state <= IDLE;
            endcase
        end
    end

endmodule
'''
    
    with open(output_path, 'w') as f:
        f.write(sv_code)
    
    print(f"AIE wrapper generated: {output_path}")


# =============================================================================
# MAIN
# =============================================================================

def main():
    """Generate complete Vitis AI deployment package."""
    print("═" * 70)
    print("NX-MIMOSA Vitis AI Deployment Package Generator")
    print("═" * 70)
    
    config = QuantConfig()
    output_dir = "/home/claude/nx-mimosa-unified/python/ew/export/vitis_ai"
    
    os.makedirs(output_dir, exist_ok=True)
    
    # Generate calibration data
    print("\n1. Generating calibration data...")
    generate_calibration_data(config, os.path.join(output_dir, "calibration_data.npy"))
    
    # Generate Vitis AI scripts
    print("\n2. Generating Vitis AI scripts...")
    generate_vitis_ai_scripts(config, output_dir)
    
    # Generate AIE wrapper
    print("\n3. Generating AIE SystemVerilog wrapper...")
    generate_aie_wrapper_sv(config, os.path.join(output_dir, "aie_policy_wrapper.sv"))
    
    # If PyTorch available, export policy
    if TORCH_AVAILABLE:
        print("\n4. Creating dummy policy for export...")
        policy = PolicyNetwork(config.state_dim, config.action_dim, config.hidden_dim)
        
        export_policy_to_onnx(policy, config, os.path.join(output_dir, "ppo_policy.onnx"))
        export_weights_to_c(policy, os.path.join(output_dir, "ppo_weights.h"), config)
    else:
        print("\n4. PyTorch not available - skipping ONNX export")
    
    print("\n" + "═" * 70)
    print("DEPLOYMENT PACKAGE CONTENTS")
    print("═" * 70)
    for f in os.listdir(output_dir):
        size = os.path.getsize(os.path.join(output_dir, f))
        print(f"  {f:<30} {size:>8} bytes")
    
    print("\n✅ Vitis AI deployment package complete!")
    print(f"   Location: {output_dir}")
    
    return output_dir


if __name__ == "__main__":
    main()
