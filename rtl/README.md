# QEDMMA v3.1 Pro RTL

## Target Device

**Xilinx RFSoC ZU48DR** (RFSoC 4x2 Board)
- Gen 3 RFSoC
- 4× RF-ADC @ 5 GSPS, 14-bit, 6 GHz bandwidth
- 2× RF-DAC @ 9.85 GSPS, 14-bit
- ~930K logic cells, 4,272 DSP48E2, 1,080 BRAM36
- 100G Ethernet (QSFP28)

## Files

| File | Lines | Description |
|------|-------|-------------|
| `qedmma_pkg.sv` | 113 | Package with types, constants, functions |
| `qedmma_v31_top.sv` | 390 | Top-level with AXI interfaces |
| `imm_core.sv` | 452 | IMM filter with mixing |
| `kalman_filter_core.sv` | 400 | Single-model Kalman filter |
| `fixed_lag_smoother.sv` | 427 | Per-model RTS smoother |
| `matrix_multiply_4x4.sv` | 115 | Pipelined matrix multiply |
| `matrix_inverse_4x4.sv` | 234 | Gauss-Jordan inverse |
| `matrix_vector_mult.sv` | 99 | Matrix-vector operations |
| `sincos_lut.sv` | 137 | Sin/cos lookup table |

**Total: ~2,367 LOC**

## Resource Estimate (ZU48DR)

| Resource | Used | Available | % |
|----------|------|-----------|---|
| LUT | ~15,000 | 425,280 | 3.5% |
| FF | ~11,000 | 850,560 | 1.3% |
| DSP48E2 | ~48 | 4,272 | 1.1% |
| BRAM36 | ~40 | 1,080 | 3.7% |

**Headroom for 8+ parallel trackers!**

## Build

```bash
cd scripts
vivado -mode batch -source build_qedmma_v31.tcl
```

## PYNQ Integration

The build generates `qedmma_v31.xsa` for use with PYNQ overlay system.

```python
from pynq import Overlay
ol = Overlay('qedmma_v31.bit')
qedmma = ol.qedmma_v31_top_0
```

---
© 2026 Nexellum d.o.o. | Commercial License
