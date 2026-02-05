#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════════════════════════════
NX-MIMOSA SVM Model Exporter for FPGA
═══════════════════════════════════════════════════════════════════════════════════════════════════════

Exports trained scikit-learn SVM models to FPGA-compatible formats:
- Fixed-point Q8.8 support vectors
- Hex files for Verilog $readmemh
- SystemVerilog package with constants
- Vivado COE files for BRAM initialization

Traceability:
  [REQ-COG-001] ML model export for FPGA
  [REQ-SSOT-001] Single Source of Truth generation

Author: Dr. Mladen Mešter / Nexellum d.o.o.
Version: 1.1.0
License: AGPL v3 / Commercial
═══════════════════════════════════════════════════════════════════════════════════════════════════════
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional
from pathlib import Path
import pickle
import json

# Optional sklearn import
try:
    from sklearn.svm import SVC
    from sklearn.preprocessing import StandardScaler
    SKLEARN_AVAILABLE = True
except ImportError:
    SKLEARN_AVAILABLE = False


# =============================================================================
# FIXED-POINT CONFIGURATION
# =============================================================================

@dataclass
class FixedPointConfig:
    """Fixed-point number format configuration."""
    total_bits: int = 16
    frac_bits: int = 8
    signed: bool = True
    
    @property
    def int_bits(self) -> int:
        return self.total_bits - self.frac_bits - (1 if self.signed else 0)
    
    @property
    def max_value(self) -> float:
        if self.signed:
            return (2 ** (self.total_bits - 1) - 1) / (2 ** self.frac_bits)
        else:
            return (2 ** self.total_bits - 1) / (2 ** self.frac_bits)
    
    @property
    def min_value(self) -> float:
        if self.signed:
            return -(2 ** (self.total_bits - 1)) / (2 ** self.frac_bits)
        else:
            return 0.0
    
    def to_fixed(self, value: float) -> int:
        """Convert float to fixed-point integer."""
        scaled = int(round(value * (2 ** self.frac_bits)))
        max_int = 2 ** (self.total_bits - 1) - 1 if self.signed else 2 ** self.total_bits - 1
        min_int = -(2 ** (self.total_bits - 1)) if self.signed else 0
        return max(min_int, min(max_int, scaled))
    
    def from_fixed(self, value: int) -> float:
        """Convert fixed-point integer to float."""
        return value / (2 ** self.frac_bits)
    
    def to_hex(self, value: float, width: int = None) -> str:
        """Convert float to hex string."""
        if width is None:
            width = self.total_bits // 4
        fixed = self.to_fixed(value)
        if fixed < 0:
            fixed = fixed + (1 << self.total_bits)  # Two's complement
        return f"{fixed:0{width}X}"


# =============================================================================
# SVM EXPORTER
# =============================================================================

class SVMExporter:
    """
    Export trained SVM models to FPGA-compatible formats.
    
    Supports:
    - RBF kernel approximation
    - Fixed-point quantization
    - Multiple output formats
    """
    
    def __init__(self, fp_config: FixedPointConfig = None):
        """
        Initialize exporter.
        
        Args:
            fp_config: Fixed-point configuration (default Q8.8)
        """
        self.fp = fp_config or FixedPointConfig()
        self.support_vectors = None
        self.alphas = None
        self.bias = None
        self.gamma = None
        self.scaler_mean = None
        self.scaler_std = None
        self.n_features = None
        self.n_support_vectors = None
    
    def load_sklearn_model(self, model: 'SVC', scaler: 'StandardScaler' = None):
        """
        Load parameters from trained sklearn SVC model.
        
        Args:
            model: Trained sklearn SVC with RBF kernel
            scaler: Optional StandardScaler used for feature normalization
        """
        if not SKLEARN_AVAILABLE:
            raise ImportError("scikit-learn required for this function")
        
        if model.kernel != 'rbf':
            raise ValueError(f"Only RBF kernel supported, got {model.kernel}")
        
        # Extract support vectors
        self.support_vectors = model.support_vectors_.copy()
        self.n_support_vectors = len(self.support_vectors)
        self.n_features = self.support_vectors.shape[1]
        
        # Extract dual coefficients (alphas)
        self.alphas = model.dual_coef_.flatten().copy()
        
        # Bias term
        self.bias = float(model.intercept_[0])
        
        # RBF gamma
        if model.gamma == 'scale':
            self.gamma = 1.0 / (self.n_features * self.support_vectors.var())
        elif model.gamma == 'auto':
            self.gamma = 1.0 / self.n_features
        else:
            self.gamma = float(model.gamma)
        
        # Scaler parameters
        if scaler is not None:
            self.scaler_mean = scaler.mean_.copy()
            self.scaler_std = scaler.scale_.copy()
        else:
            self.scaler_mean = np.zeros(self.n_features)
            self.scaler_std = np.ones(self.n_features)
        
        print(f"Loaded SVM model:")
        print(f"  Support vectors: {self.n_support_vectors}")
        print(f"  Features: {self.n_features}")
        print(f"  Gamma: {self.gamma:.6f}")
        print(f"  Bias: {self.bias:.6f}")
    
    def load_from_arrays(self, support_vectors: np.ndarray, alphas: np.ndarray,
                         bias: float, gamma: float):
        """
        Load parameters directly from numpy arrays.
        
        Args:
            support_vectors: (n_sv, n_features) array
            alphas: (n_sv,) array of weights
            bias: Scalar bias term
            gamma: RBF kernel gamma parameter
        """
        self.support_vectors = np.array(support_vectors)
        self.alphas = np.array(alphas).flatten()
        self.bias = float(bias)
        self.gamma = float(gamma)
        
        self.n_support_vectors = len(self.support_vectors)
        self.n_features = self.support_vectors.shape[1]
        
        self.scaler_mean = np.zeros(self.n_features)
        self.scaler_std = np.ones(self.n_features)
    
    def reduce_support_vectors(self, max_sv: int = 64):
        """
        Reduce number of support vectors using k-means clustering.
        
        Args:
            max_sv: Maximum number of support vectors
        """
        if self.n_support_vectors <= max_sv:
            print(f"No reduction needed ({self.n_support_vectors} <= {max_sv})")
            return
        
        try:
            from sklearn.cluster import KMeans
        except ImportError:
            print("Warning: sklearn not available for clustering")
            # Simple reduction: take highest weight SVs
            top_indices = np.argsort(np.abs(self.alphas))[-max_sv:]
            self.support_vectors = self.support_vectors[top_indices]
            self.alphas = self.alphas[top_indices]
            self.n_support_vectors = max_sv
            return
        
        # Cluster support vectors
        kmeans = KMeans(n_clusters=max_sv, random_state=42)
        labels = kmeans.fit_predict(self.support_vectors)
        
        # New support vectors are cluster centers
        new_sv = kmeans.cluster_centers_
        
        # New alphas are sum of alphas in each cluster
        new_alphas = np.zeros(max_sv)
        for i in range(max_sv):
            mask = labels == i
            new_alphas[i] = np.sum(self.alphas[mask])
        
        self.support_vectors = new_sv
        self.alphas = new_alphas
        self.n_support_vectors = max_sv
        
        print(f"Reduced to {max_sv} support vectors via clustering")
    
    def export_hex(self, output_dir: str, prefix: str = "svm"):
        """
        Export to Verilog hex format files.
        
        Args:
            output_dir: Output directory
            prefix: Filename prefix
        """
        out_path = Path(output_dir)
        out_path.mkdir(parents=True, exist_ok=True)
        
        # Support vectors: sv_features.hex
        # Format: Each line is one SV, features concatenated
        sv_file = out_path / f"{prefix}_sv_features.hex"
        with open(sv_file, 'w') as f:
            f.write(f"// NX-MIMOSA SVM Support Vectors (Q{self.fp.total_bits-self.fp.frac_bits}.{self.fp.frac_bits})\n")
            f.write(f"// {self.n_support_vectors} vectors x {self.n_features} features\n")
            for sv in self.support_vectors:
                hex_vals = [self.fp.to_hex(v) for v in sv]
                f.write(" ".join(hex_vals) + "\n")
        print(f"  Written: {sv_file}")
        
        # Alpha weights: sv_alphas.hex
        alpha_file = out_path / f"{prefix}_sv_alphas.hex"
        with open(alpha_file, 'w') as f:
            f.write(f"// NX-MIMOSA SVM Alpha Weights\n")
            for alpha in self.alphas:
                f.write(self.fp.to_hex(alpha) + "\n")
        print(f"  Written: {alpha_file}")
        
        # Parameters: svm_params.hex
        params_file = out_path / f"{prefix}_params.hex"
        with open(params_file, 'w') as f:
            f.write(f"// Bias\n{self.fp.to_hex(self.bias)}\n")
            f.write(f"// Gamma\n{self.fp.to_hex(self.gamma)}\n")
            f.write(f"// Scaler mean\n")
            for m in self.scaler_mean:
                f.write(self.fp.to_hex(m) + "\n")
            f.write(f"// Scaler std\n")
            for s in self.scaler_std:
                f.write(self.fp.to_hex(s) + "\n")
        print(f"  Written: {params_file}")
    
    def export_systemverilog(self, output_dir: str, module_name: str = "svm_params_pkg"):
        """
        Export as SystemVerilog package.
        
        Args:
            output_dir: Output directory
            module_name: Package name
        """
        out_path = Path(output_dir)
        out_path.mkdir(parents=True, exist_ok=True)
        
        sv_file = out_path / f"{module_name}.sv"
        
        with open(sv_file, 'w') as f:
            f.write(f"""// ═══════════════════════════════════════════════════════════════════════════════
// {module_name.upper()} - Auto-generated SVM Parameters
// ═══════════════════════════════════════════════════════════════════════════════
// Generated by NX-MIMOSA svm_exporter.py
// DO NOT EDIT MANUALLY
// ═══════════════════════════════════════════════════════════════════════════════

`timescale 1ns/1ps

package {module_name};

    // Fixed-point format: Q{self.fp.total_bits-self.fp.frac_bits}.{self.fp.frac_bits}
    localparam FEATURE_WIDTH = {self.fp.total_bits};
    localparam FRAC_BITS = {self.fp.frac_bits};
    
    // Model dimensions
    localparam NUM_SUPPORT_VECTORS = {self.n_support_vectors};
    localparam NUM_FEATURES = {self.n_features};
    
    // RBF kernel parameters
    localparam logic [{self.fp.total_bits-1}:0] SVM_GAMMA = {self.fp.total_bits}'h{self.fp.to_hex(self.gamma)};
    localparam logic [{self.fp.total_bits-1}:0] SVM_BIAS = {self.fp.total_bits}'h{self.fp.to_hex(self.bias)};
    
    // Support vectors
    localparam logic [{self.fp.total_bits-1}:0] SUPPORT_VECTORS [{self.n_support_vectors}][{self.n_features}] = '{{
""")
            for i, sv in enumerate(self.support_vectors):
                hex_vals = [f"{self.fp.total_bits}'h{self.fp.to_hex(v)}" for v in sv]
                comma = "," if i < self.n_support_vectors - 1 else ""
                f.write(f"        '{{{', '.join(hex_vals)}}}{comma}\n")
            
            f.write(f"""    }};
    
    // Alpha weights
    localparam logic signed [{self.fp.total_bits-1}:0] SV_ALPHAS [{self.n_support_vectors}] = '{{
""")
            for i, alpha in enumerate(self.alphas):
                comma = "," if i < self.n_support_vectors - 1 else ""
                f.write(f"        {self.fp.total_bits}'h{self.fp.to_hex(alpha)}{comma}\n")
            
            f.write(f"""    }};
    
    // Scaler parameters (for input normalization)
    localparam logic [{self.fp.total_bits-1}:0] SCALER_MEAN [{self.n_features}] = '{{
        {', '.join([f"{self.fp.total_bits}'h{self.fp.to_hex(m)}" for m in self.scaler_mean])}
    }};
    
    localparam logic [{self.fp.total_bits-1}:0] SCALER_STD [{self.n_features}] = '{{
        {', '.join([f"{self.fp.total_bits}'h{self.fp.to_hex(s)}" for s in self.scaler_std])}
    }};

endpackage
""")
        print(f"  Written: {sv_file}")
    
    def export_vivado_coe(self, output_dir: str, prefix: str = "svm"):
        """
        Export as Vivado COE files for BRAM initialization.
        
        Args:
            output_dir: Output directory
            prefix: Filename prefix
        """
        out_path = Path(output_dir)
        out_path.mkdir(parents=True, exist_ok=True)
        
        # Support vectors COE
        sv_coe = out_path / f"{prefix}_sv.coe"
        with open(sv_coe, 'w') as f:
            f.write("; NX-MIMOSA SVM Support Vectors\n")
            f.write("memory_initialization_radix=16;\n")
            f.write("memory_initialization_vector=\n")
            
            lines = []
            for sv in self.support_vectors:
                for v in sv:
                    lines.append(self.fp.to_hex(v))
            f.write(",\n".join(lines) + ";\n")
        print(f"  Written: {sv_coe}")
        
        # Alphas COE
        alpha_coe = out_path / f"{prefix}_alpha.coe"
        with open(alpha_coe, 'w') as f:
            f.write("; NX-MIMOSA SVM Alpha Weights\n")
            f.write("memory_initialization_radix=16;\n")
            f.write("memory_initialization_vector=\n")
            f.write(",\n".join([self.fp.to_hex(a) for a in self.alphas]) + ";\n")
        print(f"  Written: {alpha_coe}")
    
    def export_c_header(self, output_dir: str, header_name: str = "svm_params"):
        """
        Export as C header for embedded systems.
        
        Args:
            output_dir: Output directory
            header_name: Header filename (without .h)
        """
        out_path = Path(output_dir)
        out_path.mkdir(parents=True, exist_ok=True)
        
        h_file = out_path / f"{header_name}.h"
        
        with open(h_file, 'w') as f:
            guard = header_name.upper() + "_H"
            f.write(f"""/**
 * @file {header_name}.h
 * @brief NX-MIMOSA SVM Parameters (Auto-generated)
 * 
 * DO NOT EDIT MANUALLY
 */

#ifndef {guard}
#define {guard}

#include <stdint.h>

#define SVM_NUM_SUPPORT_VECTORS {self.n_support_vectors}
#define SVM_NUM_FEATURES {self.n_features}
#define SVM_FRAC_BITS {self.fp.frac_bits}

// Fixed-point type
typedef int16_t fixed_t;

// Convert float to fixed
#define FLOAT_TO_FIXED(x) ((fixed_t)((x) * (1 << SVM_FRAC_BITS)))

// Convert fixed to float
#define FIXED_TO_FLOAT(x) ((float)(x) / (1 << SVM_FRAC_BITS))

// Model parameters
static const fixed_t SVM_GAMMA = 0x{self.fp.to_hex(self.gamma)};
static const fixed_t SVM_BIAS = 0x{self.fp.to_hex(self.bias)};

// Support vectors
static const fixed_t SUPPORT_VECTORS[SVM_NUM_SUPPORT_VECTORS][SVM_NUM_FEATURES] = {{
""")
            for sv in self.support_vectors:
                hex_vals = [f"0x{self.fp.to_hex(v)}" for v in sv]
                f.write(f"    {{{', '.join(hex_vals)}}},\n")
            
            f.write(f"""}};

// Alpha weights
static const fixed_t SV_ALPHAS[SVM_NUM_SUPPORT_VECTORS] = {{
    {', '.join([f"0x{self.fp.to_hex(a)}" for a in self.alphas])}
}};

#endif // {guard}
""")
        print(f"  Written: {h_file}")
    
    def export_all(self, output_dir: str, prefix: str = "svm"):
        """Export to all supported formats."""
        print(f"\nExporting SVM model to {output_dir}/")
        self.export_hex(output_dir, prefix)
        self.export_systemverilog(output_dir, f"{prefix}_params_pkg")
        self.export_vivado_coe(output_dir, prefix)
        self.export_c_header(output_dir, f"{prefix}_params")
        print("\nExport complete!")


# =============================================================================
# TANH/EXP LUT GENERATOR
# =============================================================================

def generate_exp_lut(output_dir: str, bits: int = 8, scale: float = 32.0):
    """
    Generate exponential LUT for RBF kernel: exp(-x/scale).
    
    Args:
        output_dir: Output directory
        bits: LUT address bits (256 entries for 8 bits)
        scale: Scaling factor for input
    """
    out_path = Path(output_dir)
    out_path.mkdir(parents=True, exist_ok=True)
    
    n_entries = 2 ** bits
    
    # Generate exp(-x/scale) * 255
    lut = []
    for i in range(n_entries):
        val = int(255 * np.exp(-i / scale))
        lut.append(val)
    
    # Hex file
    hex_file = out_path / "exp_lut.hex"
    with open(hex_file, 'w') as f:
        f.write("// exp(-x/32) * 255 LUT\n")
        for v in lut:
            f.write(f"{v:02X}\n")
    print(f"  Written: {hex_file}")
    
    # SystemVerilog
    sv_file = out_path / "exp_lut_pkg.sv"
    with open(sv_file, 'w') as f:
        f.write(f"""// Exponential LUT for RBF kernel
package exp_lut_pkg;
    localparam logic [7:0] EXP_LUT [0:{n_entries-1}] = '{{
        {', '.join([f"8'h{v:02X}" for v in lut])}
    }};
endpackage
""")
    print(f"  Written: {sv_file}")


def generate_tanh_lut(output_dir: str, bits: int = 8, scale: float = 32.0):
    """
    Generate tanh LUT for SVM output.
    
    Args:
        output_dir: Output directory
        bits: LUT address bits
        scale: Scaling factor
    """
    out_path = Path(output_dir)
    out_path.mkdir(parents=True, exist_ok=True)
    
    n_entries = 2 ** bits
    
    # Generate tanh((x-128)/scale) * 127 + 128 (mapped to 0-255)
    lut = []
    for i in range(n_entries):
        x = (i - 128) / scale
        val = int(127 * np.tanh(x) + 128)
        lut.append(max(0, min(255, val)))
    
    # Hex file
    hex_file = out_path / "tanh_lut.hex"
    with open(hex_file, 'w') as f:
        f.write("// tanh((x-128)/32) LUT\n")
        for v in lut:
            f.write(f"{v:02X}\n")
    print(f"  Written: {hex_file}")


# =============================================================================
# MAIN / DEMO
# =============================================================================

if __name__ == "__main__":
    print("═" * 70)
    print("NX-MIMOSA SVM Exporter v1.1.0")
    print("═" * 70)
    
    # Create example SVM model (for demo without sklearn)
    np.random.seed(42)
    
    # Synthetic support vectors (3 features: magnitude, mean, variance)
    n_sv = 32
    support_vectors = np.random.randn(n_sv, 3) * np.array([0.5, 0.3, 0.2])
    alphas = np.random.randn(n_sv) * 0.1
    bias = 0.0
    gamma = 0.5
    
    # Create exporter
    fp_config = FixedPointConfig(total_bits=16, frac_bits=8, signed=True)
    exporter = SVMExporter(fp_config)
    
    # Load model
    exporter.load_from_arrays(support_vectors, alphas, bias, gamma)
    
    # Export all formats
    output_dir = "/home/claude/nx-mimosa-unified/python/cognitive/export"
    exporter.export_all(output_dir, prefix="cfar_svm")
    
    # Generate LUTs
    print("\nGenerating LUTs...")
    generate_exp_lut(output_dir)
    generate_tanh_lut(output_dir)
    
    print("\n✅ Export complete!")
