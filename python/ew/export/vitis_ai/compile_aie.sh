#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════════════
# NX-MIMOSA Vitis AI AIE Compilation Script
# ═══════════════════════════════════════════════════════════════════════════════
# [REQ-RL-DEPLOY-001] Compilation for Versal AIE-ML

set -e

source /opt/vitis_ai/setup.sh

echo "Compiling for Versal AI Engine..."

# Compile for AIE-ML
vai_c_aie \
    --arch /opt/vitis_ai/arch/DPUAIE.json \
    --model ./quantized/ppo_policy_int8.xmodel \
    --output_dir ./compiled \
    --net_name ppo_policy

echo "AIE compilation complete: ./compiled/ppo_policy.xclbin"

# Generate deployment package
mkdir -p deploy
cp ./compiled/ppo_policy.xclbin deploy/
cp ../ppo_weights.h deploy/
echo "Deployment package ready in ./deploy/"
