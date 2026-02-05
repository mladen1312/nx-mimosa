#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════════════════════════════
NX-MIMOSA Configuration Code Generator
═══════════════════════════════════════════════════════════════════════════════════════════════════════

Generates code from SSOT config.yaml:
- SystemVerilog packages (.sv)
- C headers (.h)
- Python modules (.py)
- Device Tree overlays (.dtsi)

Traceability:
  [REQ-SSOT-002] Automated code generation from YAML

Usage:
  python config_generator.py --config system_config.yaml --output ./generated

Author: Dr. Mladen Mešter / Nexellum d.o.o.
Version: 1.1.0
License: AGPL v3 / Commercial
═══════════════════════════════════════════════════════════════════════════════════════════════════════
"""

import yaml
import argparse
from pathlib import Path
from datetime import datetime
from typing import Dict, Any, List


# =============================================================================
# UTILITIES
# =============================================================================

def load_config(config_path: str) -> Dict[str, Any]:
    """Load YAML configuration file."""
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def ensure_dir(path: Path):
    """Ensure directory exists."""
    path.mkdir(parents=True, exist_ok=True)


def get_header(format_type: str, filename: str, config: Dict[str, Any]) -> str:
    """Generate file header comment."""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    system = config.get('system', {})
    
    if format_type == 'sv':
        return f"""// ═══════════════════════════════════════════════════════════════════════════════
// {filename} - Auto-generated from system_config.yaml
// ═══════════════════════════════════════════════════════════════════════════════
// System: {system.get('name', 'NX-MIMOSA')} v{system.get('version', '1.0.0')}
// Generated: {timestamp}
// DO NOT EDIT MANUALLY - Regenerate from YAML
// ═══════════════════════════════════════════════════════════════════════════════
"""
    
    elif format_type == 'c':
        return f"""/**
 * @file {filename}
 * @brief Auto-generated from system_config.yaml
 * 
 * System: {system.get('name', 'NX-MIMOSA')} v{system.get('version', '1.0.0')}
 * Generated: {timestamp}
 * 
 * DO NOT EDIT MANUALLY - Regenerate from YAML
 */
"""
    
    elif format_type == 'py':
        return f'''"""
{filename} - Auto-generated from system_config.yaml

System: {system.get('name', 'NX-MIMOSA')} v{system.get('version', '1.0.0')}
Generated: {timestamp}

DO NOT EDIT MANUALLY - Regenerate from YAML
"""
'''
    
    return ""


# =============================================================================
# SYSTEMVERILOG GENERATOR
# =============================================================================

def generate_systemverilog_params(config: Dict[str, Any], output_dir: Path):
    """Generate SystemVerilog parameter package."""
    filename = "nx_mimosa_params_pkg.sv"
    filepath = output_dir / filename
    
    radar = config.get('radar', {})
    fpga = config.get('fpga', {})
    tracking = config.get('tracking', {})
    cfar = config.get('cfar', {})
    cognitive = config.get('cognitive', {})
    eccm = config.get('eccm', {})
    
    with open(filepath, 'w') as f:
        f.write(get_header('sv', filename, config))
        f.write("""
`timescale 1ns/1ps

package nx_mimosa_params_pkg;

    // ═══════════════════════════════════════════════════════════════════════════
    // RADAR PARAMETERS
    // ═══════════════════════════════════════════════════════════════════════════
""")
        f.write(f"    localparam RADAR_FS = {radar.get('fs', 100000000)};\n")
        f.write(f"    localparam RADAR_FC = {radar.get('fc', 10000000000)};\n")
        f.write(f"    localparam RADAR_BANDWIDTH = {radar.get('bandwidth', 10000000)};\n")
        f.write(f"    localparam RADAR_PRF = {radar.get('prf', 1000)};\n")
        f.write(f"    localparam RADAR_MAX_RANGE = {radar.get('max_range', 150000)};\n")
        
        f.write("""
    // ═══════════════════════════════════════════════════════════════════════════
    // FPGA PARAMETERS
    // ═══════════════════════════════════════════════════════════════════════════
""")
        f.write(f"    localparam CLOCK_FREQ = {fpga.get('clock_freq', 250000000)};\n")
        f.write(f"    localparam AXI_DATA_WIDTH = {fpga.get('axi_data_width', 32)};\n")
        f.write(f"    localparam AXI_ADDR_WIDTH = {fpga.get('axi_addr_width', 32)};\n")
        f.write(f"    localparam FIFO_DEPTH = {fpga.get('fifo_depth', 1024)};\n")
        
        f.write("""
    // ═══════════════════════════════════════════════════════════════════════════
    // TRACKING PARAMETERS
    // ═══════════════════════════════════════════════════════════════════════════
""")
        ukf = tracking.get('ukf', {})
        f.write(f"    localparam UKF_DIM_X = {ukf.get('dim_x', 6)};\n")
        f.write(f"    localparam UKF_DIM_Z = {ukf.get('dim_z', 3)};\n")
        
        imm = tracking.get('imm', {})
        f.write(f"    localparam IMM_N_MODES = {imm.get('n_modes', 3)};\n")
        
        f.write("""
    // ═══════════════════════════════════════════════════════════════════════════
    // CFAR PARAMETERS
    // ═══════════════════════════════════════════════════════════════════════════
""")
        f.write(f"    localparam CFAR_WINDOW_SIZE = {cfar.get('window_size', 20)};\n")
        f.write(f"    localparam CFAR_GUARD_CELLS = {cfar.get('guard_cells', 4)};\n")
        
        f.write("""
    // ═══════════════════════════════════════════════════════════════════════════
    // COGNITIVE CFAR PARAMETERS
    // ═══════════════════════════════════════════════════════════════════════════
""")
        svm = cognitive.get('svm', {})
        fp = cognitive.get('fixed_point', {})
        f.write(f"    localparam COGNITIVE_ENABLED = {1 if cognitive.get('enabled') else 0};\n")
        f.write(f"    localparam SVM_NUM_SV = {svm.get('num_support_vectors', 64)};\n")
        f.write(f"    localparam SVM_PROB_THRESH = 16'h{int(svm.get('prob_threshold', 0.95) * 32768):04X};\n")
        f.write(f"    localparam FEATURE_WIDTH = {fp.get('total_bits', 16)};\n")
        f.write(f"    localparam FRAC_BITS = {fp.get('frac_bits', 8)};\n")
        
        f.write("""
    // ═══════════════════════════════════════════════════════════════════════════
    // ECCM PARAMETERS
    // ═══════════════════════════════════════════════════════════════════════════
""")
        fa = eccm.get('frequency_agility', {})
        f.write(f"    localparam ECCM_ENABLED = {1 if eccm.get('enabled') else 0};\n")
        f.write(f"    localparam FA_N_CHANNELS = {fa.get('n_channels', 64)};\n")
        f.write(f"    localparam FA_HOP_INTERVAL = {fa.get('hop_interval_us', 100)};\n")
        
        jd = eccm.get('jammer_detection', {})
        f.write(f"    localparam JAMMER_POWER_RATIO = 16'h{int(jd.get('power_ratio_threshold', 5.0) * 256):04X};\n")
        
        f.write("""
endpackage
""")
    
    print(f"  Generated: {filepath}")


def generate_systemverilog_regs(config: Dict[str, Any], output_dir: Path):
    """Generate SystemVerilog register definitions."""
    filename = "nx_mimosa_regs_pkg.sv"
    filepath = output_dir / filename
    
    regs = config.get('registers', {})
    base_addr = regs.get('base_address', 0x80000000)
    
    with open(filepath, 'w') as f:
        f.write(get_header('sv', filename, config))
        f.write(f"""
`timescale 1ns/1ps

package nx_mimosa_regs_pkg;

    // Base address
    localparam logic [31:0] BASE_ADDR = 32'h{base_addr:08X};

    // Register offsets
""")
        
        for reg_name, reg_def in regs.items():
            if reg_name == 'base_address':
                continue
            if isinstance(reg_def, dict) and 'offset' in reg_def:
                offset = reg_def['offset']
                f.write(f"    localparam logic [31:0] {reg_name}_OFFSET = 32'h{offset:08X};\n")
        
        f.write("""
    // Field definitions
""")
        
        for reg_name, reg_def in regs.items():
            if reg_name == 'base_address':
                continue
            if isinstance(reg_def, dict) and 'fields' in reg_def:
                f.write(f"\n    // {reg_name} fields\n")
                for field_name, field_def in reg_def['fields'].items():
                    bits = field_def.get('bits', '0')
                    if ':' in str(bits):
                        hi, lo = map(int, str(bits).split(':'))
                    else:
                        hi = lo = int(bits)
                    f.write(f"    localparam {reg_name}_{field_name.upper()}_HI = {hi};\n")
                    f.write(f"    localparam {reg_name}_{field_name.upper()}_LO = {lo};\n")
        
        f.write("""
endpackage
""")
    
    print(f"  Generated: {filepath}")


# =============================================================================
# C HEADER GENERATOR
# =============================================================================

def generate_c_header(config: Dict[str, Any], output_dir: Path):
    """Generate C header file."""
    filename = "nx_mimosa_config.h"
    filepath = output_dir / filename
    
    radar = config.get('radar', {})
    fpga = config.get('fpga', {})
    cfar = config.get('cfar', {})
    cognitive = config.get('cognitive', {})
    regs = config.get('registers', {})
    
    with open(filepath, 'w') as f:
        f.write(get_header('c', filename, config))
        f.write("""
#ifndef NX_MIMOSA_CONFIG_H
#define NX_MIMOSA_CONFIG_H

#include <stdint.h>

// ═══════════════════════════════════════════════════════════════════════════════
// RADAR PARAMETERS
// ═══════════════════════════════════════════════════════════════════════════════
""")
        f.write(f"#define RADAR_FS           {radar.get('fs', 100000000)}UL\n")
        f.write(f"#define RADAR_FC           {radar.get('fc', 10000000000)}ULL\n")
        f.write(f"#define RADAR_BANDWIDTH    {radar.get('bandwidth', 10000000)}UL\n")
        f.write(f"#define RADAR_PRF          {radar.get('prf', 1000)}U\n")
        f.write(f"#define RADAR_MAX_RANGE    {radar.get('max_range', 150000)}U\n")
        
        f.write("""
// ═══════════════════════════════════════════════════════════════════════════════
// FPGA PARAMETERS
// ═══════════════════════════════════════════════════════════════════════════════
""")
        f.write(f"#define AXI_DATA_WIDTH     {fpga.get('axi_data_width', 32)}U\n")
        f.write(f"#define FIFO_DEPTH         {fpga.get('fifo_depth', 1024)}U\n")
        
        f.write("""
// ═══════════════════════════════════════════════════════════════════════════════
// CFAR PARAMETERS
// ═══════════════════════════════════════════════════════════════════════════════
""")
        f.write(f"#define CFAR_WINDOW_SIZE   {cfar.get('window_size', 20)}U\n")
        f.write(f"#define CFAR_GUARD_CELLS   {cfar.get('guard_cells', 4)}U\n")
        
        f.write("""
// ═══════════════════════════════════════════════════════════════════════════════
// COGNITIVE CFAR
// ═══════════════════════════════════════════════════════════════════════════════
""")
        svm = cognitive.get('svm', {})
        f.write(f"#define COGNITIVE_ENABLED  {1 if cognitive.get('enabled') else 0}\n")
        f.write(f"#define SVM_NUM_SV         {svm.get('num_support_vectors', 64)}U\n")
        
        f.write("""
// ═══════════════════════════════════════════════════════════════════════════════
// REGISTER MAP
// ═══════════════════════════════════════════════════════════════════════════════
""")
        base_addr = regs.get('base_address', 0x80000000)
        f.write(f"#define NX_MIMOSA_BASE_ADDR  0x{base_addr:08X}UL\n\n")
        
        for reg_name, reg_def in regs.items():
            if reg_name == 'base_address':
                continue
            if isinstance(reg_def, dict) and 'offset' in reg_def:
                offset = reg_def['offset']
                f.write(f"#define {reg_name}_OFFSET   0x{offset:02X}U\n")
        
        f.write("""
// Register access macros
#define REG_WRITE(reg, val)  (*(volatile uint32_t*)(NX_MIMOSA_BASE_ADDR + reg##_OFFSET) = (val))
#define REG_READ(reg)        (*(volatile uint32_t*)(NX_MIMOSA_BASE_ADDR + reg##_OFFSET))

#endif // NX_MIMOSA_CONFIG_H
""")
    
    print(f"  Generated: {filepath}")


# =============================================================================
# PYTHON MODULE GENERATOR
# =============================================================================

def generate_python_config(config: Dict[str, Any], output_dir: Path):
    """Generate Python configuration module."""
    filename = "nx_mimosa_config.py"
    filepath = output_dir / filename
    
    with open(filepath, 'w') as f:
        f.write(get_header('py', filename, config))
        f.write("""
from dataclasses import dataclass
from typing import List, Dict, Any

""")
        
        # Radar config
        radar = config.get('radar', {})
        f.write(f"""
# ═══════════════════════════════════════════════════════════════════════════════
# RADAR PARAMETERS
# ═══════════════════════════════════════════════════════════════════════════════
RADAR_FS = {radar.get('fs', 100000000)}
RADAR_FC = {radar.get('fc', 10000000000)}
RADAR_BANDWIDTH = {radar.get('bandwidth', 10000000)}
RADAR_PRF = {radar.get('prf', 1000)}
RADAR_MAX_RANGE = {radar.get('max_range', 150000)}
""")
        
        # CFAR config
        cfar = config.get('cfar', {})
        f.write(f"""
# ═══════════════════════════════════════════════════════════════════════════════
# CFAR PARAMETERS
# ═══════════════════════════════════════════════════════════════════════════════
CFAR_WINDOW_SIZE = {cfar.get('window_size', 20)}
CFAR_GUARD_CELLS = {cfar.get('guard_cells', 4)}
CFAR_THRESHOLD_FACTOR = {cfar.get('threshold_factor', 5.0)}
""")
        
        # Cognitive config
        cognitive = config.get('cognitive', {})
        svm = cognitive.get('svm', {})
        f.write(f"""
# ═══════════════════════════════════════════════════════════════════════════════
# COGNITIVE CFAR
# ═══════════════════════════════════════════════════════════════════════════════
COGNITIVE_ENABLED = {cognitive.get('enabled', True)}
SVM_NUM_SUPPORT_VECTORS = {svm.get('num_support_vectors', 64)}
SVM_GAMMA = {svm.get('gamma', 0.5)}
SVM_PROB_THRESHOLD = {svm.get('prob_threshold', 0.95)}
""")
        
        # Register map
        regs = config.get('registers', {})
        f.write(f"""
# ═══════════════════════════════════════════════════════════════════════════════
# REGISTER MAP
# ═══════════════════════════════════════════════════════════════════════════════
BASE_ADDRESS = 0x{regs.get('base_address', 0x80000000):08X}

REGISTERS = {{
""")
        for reg_name, reg_def in regs.items():
            if reg_name == 'base_address':
                continue
            if isinstance(reg_def, dict) and 'offset' in reg_def:
                f.write(f"    '{reg_name}': {{'offset': 0x{reg_def['offset']:02X}}},\n")
        f.write("}\n")
    
    print(f"  Generated: {filepath}")


# =============================================================================
# DEVICE TREE GENERATOR
# =============================================================================

def generate_device_tree(config: Dict[str, Any], output_dir: Path):
    """Generate Device Tree overlay."""
    filename = "nx_mimosa.dtsi"
    filepath = output_dir / filename
    
    regs = config.get('registers', {})
    base_addr = regs.get('base_address', 0x80000000)
    
    with open(filepath, 'w') as f:
        f.write(f"""/*
 * NX-MIMOSA Device Tree Include
 * Auto-generated from system_config.yaml
 */

/ {{
    nx_mimosa: nx-mimosa@{base_addr:08x} {{
        compatible = "nexellum,nx-mimosa-1.1";
        reg = <0x0 0x{base_addr:08x} 0x0 0x1000>;
        interrupt-parent = <&gic>;
        interrupts = <0 89 4>;
        clocks = <&zynqmp_clk 71>;
        clock-names = "axi_clk";
        
        /* DMA channels */
        dmas = <&fpd_dma_chan0 0>;
        dma-names = "rx";
        
        /* Configuration */
        nexellum,cfar-window = <{config.get('cfar', {}).get('window_size', 20)}>;
        nexellum,cfar-guard = <{config.get('cfar', {}).get('guard_cells', 4)}>;
        nexellum,cognitive-enable = <{1 if config.get('cognitive', {}).get('enabled') else 0}>;
    }};
}};
""")
    
    print(f"  Generated: {filepath}")


# =============================================================================
# MAIN
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="NX-MIMOSA Configuration Code Generator"
    )
    parser.add_argument(
        '--config', '-c',
        default='config/system_config.yaml',
        help='Path to YAML configuration file'
    )
    parser.add_argument(
        '--output', '-o',
        default='./generated',
        help='Output directory for generated files'
    )
    parser.add_argument(
        '--formats', '-f',
        nargs='+',
        default=['all'],
        choices=['all', 'sv', 'c', 'py', 'dts'],
        help='Output formats to generate'
    )
    
    args = parser.parse_args()
    
    print("═" * 70)
    print("NX-MIMOSA Configuration Code Generator")
    print("═" * 70)
    
    # Load config
    print(f"\nLoading: {args.config}")
    config = load_config(args.config)
    
    # Create output directory
    output_dir = Path(args.output)
    ensure_dir(output_dir)
    
    print(f"Output directory: {output_dir}\n")
    
    formats = args.formats
    if 'all' in formats:
        formats = ['sv', 'c', 'py', 'dts']
    
    # Generate files
    if 'sv' in formats:
        print("Generating SystemVerilog...")
        sv_dir = output_dir / 'sv'
        ensure_dir(sv_dir)
        generate_systemverilog_params(config, sv_dir)
        generate_systemverilog_regs(config, sv_dir)
    
    if 'c' in formats:
        print("\nGenerating C headers...")
        c_dir = output_dir / 'c'
        ensure_dir(c_dir)
        generate_c_header(config, c_dir)
    
    if 'py' in formats:
        print("\nGenerating Python modules...")
        py_dir = output_dir / 'py'
        ensure_dir(py_dir)
        generate_python_config(config, py_dir)
    
    if 'dts' in formats:
        print("\nGenerating Device Tree...")
        dts_dir = output_dir / 'dts'
        ensure_dir(dts_dir)
        generate_device_tree(config, dts_dir)
    
    print("\n" + "═" * 70)
    print("✅ Code generation complete!")
    print("═" * 70)


if __name__ == "__main__":
    main()
