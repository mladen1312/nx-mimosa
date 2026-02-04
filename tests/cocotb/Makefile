# ═══════════════════════════════════════════════════════════════════════════════
# NX-MIMOSA Cocotb RTL Testbench Makefile
# Target: Verilator or Icarus Verilog simulation
# ═══════════════════════════════════════════════════════════════════════════════

# Simulator selection (verilator, icarus, questa, vcs)
SIM ?= verilator
TOPLEVEL_LANG ?= verilog

# Project paths
PROJ_ROOT = $(shell cd ../.. && pwd)
RTL_DIR = $(PROJ_ROOT)/rtl
RESULTS_DIR = results

# Verilog sources
VERILOG_SOURCES = \
    $(RTL_DIR)/nx_mimosa_pkg.sv \
    $(RTL_DIR)/matrix_multiply_4x4.sv \
    $(RTL_DIR)/matrix_inverse_4x4.sv \
    $(RTL_DIR)/matrix_vector_mult.sv \
    $(RTL_DIR)/sincos_lut.sv \
    $(RTL_DIR)/kalman_filter_core.sv \
    $(RTL_DIR)/ukf_core.sv \
    $(RTL_DIR)/imm_core.sv \
    $(RTL_DIR)/adaptive_q_module.sv \
    $(RTL_DIR)/dynamic_tpm_module.sv \
    $(RTL_DIR)/frequency_agility_controller.sv \
    $(RTL_DIR)/nx_mimosa_axi_wrapper.sv \
    $(RTL_DIR)/nx_mimosa_top.sv

# Top-level module for simulation
TOPLEVEL = nx_mimosa_axi_wrapper

# Python test module
MODULE = test_nx_mimosa_rtl

# Verilator-specific options
EXTRA_ARGS += --trace --trace-structs
VERILATOR_ARGS += -Wno-fatal -Wno-WIDTH -Wno-STMTDLY

# Coverage options
ifeq ($(COVERAGE),1)
    EXTRA_ARGS += --coverage
endif

# Include cocotb makefile
include $(shell cocotb-config --makefiles)/Makefile.sim

# ═══════════════════════════════════════════════════════════════════════════════
# Custom targets
# ═══════════════════════════════════════════════════════════════════════════════

.PHONY: all clean results lint help

all: sim

# Create results directory
$(RESULTS_DIR):
	mkdir -p $(RESULTS_DIR)

# Run simulation with results capture
sim: $(RESULTS_DIR)
	$(MAKE) SIM=$(SIM)
	@echo "Results saved to $(RESULTS_DIR)/"

# Run with coverage
coverage:
	$(MAKE) COVERAGE=1 SIM=$(SIM)
	@echo "Coverage report generated"

# Lint RTL before simulation
lint:
	@echo "Running Verilator lint..."
	verilator --lint-only -Wall $(VERILOG_SOURCES) \
		-Wno-fatal -Wno-WIDTH -Wno-STMTDLY \
		--top-module $(TOPLEVEL)
	@echo "Lint complete"

# Clean build artifacts
clean::
	rm -rf $(RESULTS_DIR)
	rm -rf sim_build
	rm -rf __pycache__
	rm -f results.xml
	rm -f *.vcd
	rm -f *.fst

# Generate waveform viewer command
waves:
	@echo "Opening waveform viewer..."
	gtkwave sim_build/dump.vcd &

# Quick test (single test)
quick:
	$(MAKE) TESTCASE=test_reset SIM=$(SIM)

# Help
help:
	@echo "NX-MIMOSA Cocotb Testbench"
	@echo ""
	@echo "Targets:"
	@echo "  make sim        - Run full simulation"
	@echo "  make coverage   - Run with coverage"
	@echo "  make lint       - Lint RTL only"
	@echo "  make quick      - Run single quick test"
	@echo "  make waves      - Open waveform viewer"
	@echo "  make clean      - Clean build artifacts"
	@echo ""
	@echo "Variables:"
	@echo "  SIM=verilator   - Use Verilator (default)"
	@echo "  SIM=icarus      - Use Icarus Verilog"
	@echo "  COVERAGE=1      - Enable coverage"
	@echo ""
	@echo "Examples:"
	@echo "  make sim SIM=icarus"
	@echo "  make coverage SIM=verilator"
