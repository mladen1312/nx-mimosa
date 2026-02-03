# QEDMMA v3.1 Pro RTL Compilation Order
# Target: ZU48DR (RFSoC 4x2)

# Package (must be first)
qedmma_pkg.sv

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
qedmma_v31_top.sv
