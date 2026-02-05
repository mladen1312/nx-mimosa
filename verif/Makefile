# NX-MIMOSA v3.3 — Cocotb Verification Makefile
# Run: make SIM=verilator

TOPLEVEL_LANG = verilog
VERILOG_SOURCES = \
    $(shell pwd)/../rtl/nx_mimosa_pkg_v33.sv \
    $(shell pwd)/../rtl/matrix_multiply_4x4.sv \
    $(shell pwd)/../rtl/matrix_inverse_4x4.sv \
    $(shell pwd)/../rtl/matrix_vector_mult.sv \
    $(shell pwd)/../rtl/sincos_lut.sv \
    $(shell pwd)/../rtl/imm_core.sv \
    $(shell pwd)/../rtl/kalman_filter_core.sv \
    $(shell pwd)/../rtl/maneuver_detector.sv \
    $(shell pwd)/../rtl/window_rts_smoother.sv \
    $(shell pwd)/../rtl/fixed_lag_smoother.sv \
    $(shell pwd)/../rtl/nx_mimosa_v33_top.sv

TOPLEVEL = nx_mimosa_v33_top
MODULE = test_nx_mimosa_v33

# Verilator flags
EXTRA_ARGS += --timing
EXTRA_ARGS += -Wno-fatal

SIM ?= verilator

include $(shell cocotb-config --makefiles)/Makefile.sim
