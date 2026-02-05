# ═══════════════════════════════════════════════════════════════════════════════
# NX-MIMOSA Cocotb Makefile
# ═══════════════════════════════════════════════════════════════════════════════

TOPLEVEL_LANG = verilog

# RTL Sources
VERILOG_SOURCES = $(PWD)/../../rtl/cognitive_cfar.sv
VERILOG_SOURCES += $(PWD)/../../rtl/vgpo_continuity_filter.sv

# Include paths
VERILOG_INCLUDE_DIRS = $(PWD)/../../rtl $(PWD)/../../generated/sv

# Simulator
SIM ?= verilator

# Top level module
TOPLEVEL = cognitive_cfar

# Python test module
MODULE = test_cognitive_cfar

# Verilator-specific options
EXTRA_ARGS += --timing
EXTRA_ARGS += -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNSIGNED
EXTRA_ARGS += $(addprefix -I,$(VERILOG_INCLUDE_DIRS))

# Coverage
COVERAGE = 1

include $(shell cocotb-config --makefiles)/Makefile.sim

# Custom targets
.PHONY: test_cfar test_vgpo all clean_all

test_cfar:
	$(MAKE) TOPLEVEL=cognitive_cfar MODULE=test_cognitive_cfar

test_vgpo:
	$(MAKE) TOPLEVEL=vgpo_continuity_filter MODULE=test_vgpo_filter

all: test_cfar test_vgpo

clean_all:
	$(MAKE) clean
	rm -rf __pycache__ sim_build *.vcd *.xml
