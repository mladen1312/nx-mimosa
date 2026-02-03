# QEDMMA v3.1 Pro RTL — Target: ZU48DR (RFSoC 4x2)

## Device Specifications

| Parameter | Value |
|-----------|-------|
| **Part Number** | XCZU48DR-1FFVG1517E |
| **Board** | RFSoC 4x2 by Real Digital |
| **Generation** | Gen 3 RFSoC |
| **Price** | $2,499 (academic) |

### PL Resources

| Resource | Available | QEDMMA Used | Utilization |
|----------|-----------|-------------|-------------|
| LUTs | 425,280 | ~15,000 | 3.5% |
| FFs | 850,560 | ~11,000 | 1.3% |
| DSP48E2 | 4,272 | ~48 | 1.1% |
| BRAM36 | 1,080 | ~40 | 3.7% |

**Headroom: 89× — supports 8+ parallel trackers!**

### RF Resources

| Resource | Specification |
|----------|---------------|
| RF-ADC | 4× 14-bit @ 5 GSPS, 6 GHz BW |
| RF-DAC | 2× 14-bit @ 9.85 GSPS |
| Ethernet | 100 Gbps QSFP28 |
| Memory | 8 GB DDR4 |

## Module Hierarchy

```
qedmma_v31_top.sv (390 LOC)
├── imm_core.sv (452 LOC)
│   ├── kalman_filter_core.sv (400 LOC) × 3
│   │   └── matrix_multiply_4x4.sv (115 LOC)
│   └── sincos_lut.sv (137 LOC) × 2
├── fixed_lag_smoother.sv (427 LOC)
│   ├── matrix_multiply_4x4.sv
│   ├── matrix_inverse_4x4.sv (234 LOC)
│   └── matrix_vector_mult.sv (99 LOC)
└── qedmma_pkg.sv (72 LOC)

Total: ~2,300 LOC
```

## Build

```bash
cd scripts
vivado -mode batch -source build_zu48dr.tcl
```

## PYNQ Integration

```python
from pynq import Overlay, allocate
import numpy as np

# Load overlay
ol = Overlay('qedmma_v31_zu48dr.bit')
qedmma = ol.qedmma_v31_top_0

# Configure
qedmma.write(0x04, 0x00003298)  # omega = 0.196 rad/s
qedmma.write(0x08, 0x0000199A)  # dt = 0.1s
qedmma.write(0x00, 0x00000003)  # enable + smoother
```

---
© 2026 Nexellum d.o.o. | Commercial License
