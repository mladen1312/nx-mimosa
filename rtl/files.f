# NX-MIMOSA RTL Compilation Order
# Multi-model IMM Optimal Smoothing Algorithm
# Target: ZU48DR (RFSoC 4x2 / ZCU208)

# Package (must be first)
nx_mimosa_pkg.sv

# Utility modules
sincos_lut.sv
matrix_multiply_4x4.sv
matrix_inverse_4x4.sv
matrix_vector_mult.sv

# Core modules
kalman_filter_core.sv
imm_core.sv
fixed_lag_smoother.sv

# Top-level
nx_mimosa_top.sv
