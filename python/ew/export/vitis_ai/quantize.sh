#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════════════
# NX-MIMOSA Vitis AI Quantization Script
# ═══════════════════════════════════════════════════════════════════════════════
# [REQ-RL-DEPLOY-001] Quantization for Versal AIE

set -e

# Environment setup
source /opt/vitis_ai/setup.sh

echo "Starting Vitis AI Quantization..."

# Post-Training Quantization
vai_q_pytorch quantize \
    --model ppo_policy.onnx \
    --input_shape 1,6 \
    --output_dir ./quantized \
    --quant_mode calib \
    --calib_input_fn calibration_data.npy \
    --device cpu \
    --gpu 0

# Verify quantized model
echo "Quantization complete. Running verification..."
python3 verify_quantized.py

echo "Quantized model ready: ./quantized/ppo_policy_int8.xmodel"
