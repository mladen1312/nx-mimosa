# QEDMMA v3.1 Pro RTL

## Supported Platforms

Both boards use the same **XCZU48DR** Gen 3 RFSoC — identical RTL, different build targets.

### Board Comparison

| Feature | RFSoC 4x2 | ZCU208 |
|---------|-----------|--------|
| **Device** | XCZU48DR-1FFVG1517E | XCZU48DR-2FSVG1517E |
| **Price** | **$2,499** (academic) | $13,194 |
| **Speed Grade** | -1 | -2 (faster) |
| **RF-ADC Exposed** | 4× SMA | 8× via XM650 |
| **RF-DAC Exposed** | 2× SMA | 8× via XM650 |
| **Ethernet** | **100 Gbps** QSFP28 | 10 Gbps SFP+ |
| **PYNQ** | Native support | Supported |
| **Availability** | Academic only | General |
| **Baluns** | Integrated | External (XM650) |
| **Open Source PCB** | Yes | No |

### Recommendation

| Use Case | Recommended Board |
|----------|-------------------|
| **Development/Prototyping** | RFSoC 4x2 ($2,499) |
| **Academic Research** | RFSoC 4x2 |
| **8-channel ADC/DAC** | ZCU208 |
| **Production System** | ZCU208 (general availability) |
| **100G Data Offload** | RFSoC 4x2 |

---

## Device Resources (ZU48DR)

| Resource | Available | QEDMMA Used | Utilization |
|----------|-----------|-------------|-------------|
| LUTs | 425,280 | ~15,000 | **3.5%** |
| FFs | 850,560 | ~11,000 | **1.3%** |
| DSP48E2 | 4,272 | ~48 | **1.1%** |
| BRAM36 | 1,080 | ~40 | **3.7%** |
| URAM | 80 | 0 | 0% |

**Headroom: 89× — supports 8+ parallel trackers!**

### RF Specifications (Gen 3)

| Parameter | Value |
|-----------|-------|
| RF-ADC Resolution | 14-bit |
| RF-ADC Sample Rate | 5 GSPS |
| RF-ADC Bandwidth | 6 GHz |
| RF-DAC Resolution | 14-bit |
| RF-DAC Sample Rate | 9.85 GSPS (10 GSPS capable) |
| SD-FEC Cores | 8 |

---

## Build Instructions

### RFSoC 4x2 (Recommended for development)

```bash
cd scripts
vivado -mode batch -source build_rfsoc4x2.tcl
```

Output:
- `qedmma_v31_rfsoc4x2.bit`
- `qedmma_v31_rfsoc4x2.xsa` (PYNQ overlay)

### ZCU208

```bash
cd scripts
vivado -mode batch -source build_zcu208.tcl
```

Output:
- `qedmma_v31_zcu208.bit`
- `qedmma_v31_zcu208.xsa`

---

## Board Selection in RTL

The package (`qedmma_pkg.sv`) auto-configures based on defines:

```systemverilog
// Default: RFSoC 4x2
`define TARGET_RFSOC_4X2

// Or for ZCU208:
`define TARGET_ZCU208
```

Build scripts set this automatically. Key differences:

| Parameter | RFSoC 4x2 | ZCU208 |
|-----------|-----------|--------|
| `NUM_ADC_CHANNELS` | 4 | 8 |
| `NUM_DAC_CHANNELS` | 2 | 8 |
| `HAS_100G_ETH` | true | false |

---

## Module Hierarchy

```
qedmma_v31_top.sv (390 LOC)
├── imm_core.sv (458 LOC)
│   ├── kalman_filter_core.sv (400 LOC) × 3 models
│   │   └── matrix_multiply_4x4.sv (115 LOC)
│   └── sincos_lut.sv (137 LOC) × 2
├── fixed_lag_smoother.sv (427 LOC)
│   ├── matrix_multiply_4x4.sv
│   ├── matrix_inverse_4x4.sv (234 LOC)
│   └── matrix_vector_mult.sv (99 LOC)
└── qedmma_pkg.sv (182 LOC)

Total: ~2,442 LOC SystemVerilog
```

---

## File List

| File | Lines | Description |
|------|-------|-------------|
| `qedmma_pkg.sv` | 182 | Types, dual-board config |
| `qedmma_v31_top.sv` | 390 | Top-level AXI interfaces |
| `imm_core.sv` | 458 | IMM filter + mixing |
| `kalman_filter_core.sv` | 400 | Single-model Kalman |
| `fixed_lag_smoother.sv` | 427 | Per-model RTS smoother |
| `matrix_inverse_4x4.sv` | 234 | Gauss-Jordan inverse |
| `matrix_multiply_4x4.sv` | 115 | Pipelined multiply |
| `matrix_vector_mult.sv` | 99 | Vector operations |
| `sincos_lut.sv` | 137 | CT model trig |

---

## PYNQ Integration

### RFSoC 4x2

```python
from pynq import Overlay

ol = Overlay('qedmma_v31_rfsoc4x2.bit')
qedmma = ol.qedmma_v31_top_0

# Configure tracker
qedmma.write(0x04, 0x00003298)  # omega
qedmma.write(0x08, 0x0000199A)  # dt
qedmma.write(0x00, 0x00000003)  # enable
```

### ZCU208

```python
from pynq import Overlay

ol = Overlay('qedmma_v31_zcu208.bit')
qedmma = ol.qedmma_v31_top_0
# Same API
```

---

## Contact

**Nexellum d.o.o.**
- Dr. Mladen Mešter
- mladen@nexellum.com
- +385 99 737 5100

---

© 2026 Nexellum d.o.o. | Commercial License

---

## ⚖️ License

**Commercial Use: Contact mladen@nexellum.com for licensing and exemptions.**
