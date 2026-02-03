# NX-MIMOSA RTL

**Multi-model IMM Optimal Smoothing Algorithm**

## Supported Platforms

Both boards use **XCZU48DR** Gen 3 RFSoC — same RTL, different build targets.

| Feature | RFSoC 4x2 | ZCU208 |
|---------|-----------|--------|
| **Device** | XCZU48DR-1 | XCZU48DR-2 |
| **Price** | **$2,499** (academic) | $13,194 |
| **ADC** | 4× SMA | 8× via XM650 |
| **DAC** | 2× SMA | 8× via XM650 |
| **Ethernet** | **100 Gbps** QSFP28 | 10 Gbps SFP+ |

## Resource Utilization (ZU48DR)

| Resource | Used | Available | % |
|----------|------|-----------|---|
| LUT | 15,000 | 425,280 | **3.5%** |
| FF | 11,000 | 850,560 | **1.3%** |
| DSP48E2 | 48 | 4,272 | **1.1%** |
| BRAM36 | 40 | 1,080 | **3.7%** |

**Headroom: 89× — supports 8+ parallel trackers!**

## Module Hierarchy

```
nx_mimosa_top.sv (390 LOC)
├── imm_core.sv (458 LOC)
│   ├── kalman_filter_core.sv (400 LOC) × 3 models
│   │   └── matrix_multiply_4x4.sv (115 LOC)
│   └── sincos_lut.sv (137 LOC) × 2
├── fixed_lag_smoother.sv (427 LOC)
│   ├── matrix_multiply_4x4.sv
│   ├── matrix_inverse_4x4.sv (234 LOC)
│   └── matrix_vector_mult.sv (99 LOC)
└── nx_mimosa_pkg.sv (166 LOC)

Total: ~2,400 LOC SystemVerilog
```

## File List

| File | Lines | Description |
|------|-------|-------------|
| `nx_mimosa_pkg.sv` | 166 | Types, dual-board config |
| `nx_mimosa_top.sv` | 390 | Top-level AXI interfaces |
| `imm_core.sv` | 458 | IMM filter + mixing |
| `kalman_filter_core.sv` | 400 | Single-model Kalman |
| `fixed_lag_smoother.sv` | 427 | Per-model RTS smoother |
| `matrix_inverse_4x4.sv` | 234 | Gauss-Jordan inverse |
| `matrix_multiply_4x4.sv` | 115 | Pipelined multiply |
| `matrix_vector_mult.sv` | 99 | Vector operations |
| `sincos_lut.sv` | 137 | CT model trig |

## Build

```bash
# RFSoC 4x2
cd scripts && vivado -mode batch -source build_rfsoc4x2.tcl

# ZCU208
cd scripts && vivado -mode batch -source build_zcu208.tcl
```

## PYNQ Integration

```python
from pynq import Overlay

ol = Overlay('nx_mimosa_rfsoc4x2.bit')
mimosa = ol.nx_mimosa_top_0

mimosa.write(0x04, 0x00003298)  # omega
mimosa.write(0x08, 0x0000199A)  # dt
mimosa.write(0x00, 0x00000003)  # enable
```

---

**Commercial Use: Contact mladen@nexellum.com for exemptions.**

© 2026 Nexellum d.o.o. | mladen@nexellum.com
