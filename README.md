# NX-MIMOSA v4.0 - Unified Multi-Domain Tracking System

[![License: AGPL v3](https://img.shields.io/badge/License-AGPL%20v3-blue.svg)](https://www.gnu.org/licenses/agpl-3.0)
[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![FPGA Ready](https://img.shields.io/badge/FPGA-RFSoC%20ZU48DR-green.svg)](https://www.xilinx.com/products/silicon-devices/soc/rfsoc.html)

> **One Core Engine • Multiple Industry Verticals • Full Compliance**

NX-MIMOSA (Nexellum Multi-model IMM Optimal Smoothing Algorithm) is a production-grade multi-target tracking system designed for the most demanding applications across aviation, automotive, defense, space, and maritime domains.

## 🎯 Key Features

| Feature | Description |
|---------|-------------|
| **Unified Architecture** | Single codebase supporting 6 industry verticals |
| **VS-IMM Core** | Variable-Structure Interacting Multiple Model filter |
| **Multiple Filters** | UKF, CKF, EKF with automatic selection |
| **Adaptive Estimation** | Real-time Q and R adaptation from innovation sequence |
| **ECCM Resilience** | Anti-jamming for defense applications |
| **Industry Compliance** | Pre-configured profiles for certification requirements |

## 📊 Performance Summary

```
┌─────────────────────────────────────────────────────────────────────────────┐
│ NX-MIMOSA v4.0 PERFORMANCE BENCHMARKS                                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  Scenario                      RMSE        vs Standard IMM   Improvement   │
│  ──────────────────────────────────────────────────────────────────────    │
│  Maneuvering Target (9g)       45 m        vs 180 m          +75%          │
│  Hypersonic (Mach 10)          120 m       vs 450 m          +73%          │
│  EW Jamming (90% intensity)    1,125 m     vs 2,165 m        +48%          │
│  Multi-Sensor Fusion           35 m        vs 95 m           +63%          │
│                                                                             │
│  OVERALL IMPROVEMENT: +59% average across all scenarios                    │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

## 🏭 Supported Industries

### 1. Civil Aviation (ATC/ATM)
- **Standards**: EUROCONTROL EASSP, DO-178C DAL-C, ASTERIX CAT062
- **Requirements**: ≤500m RMS (en-route), ≤150m RMS (TMA), 99.5% continuity
- **Sensors**: PSR, SSR, Mode-S, ADS-B, WAM fusion

### 2. Automotive (ADAS/AD)
- **Standards**: ISO 26262 ASIL-D, Euro NCAP
- **Requirements**: ≤10cm @ 100m, 20 Hz update, 50ms latency
- **Features**: Multi-object tracking, classification support

### 3. Defense (Military Radar)
- **Standards**: MIL-STD, DO-254 DAL-A, Link-16
- **Requirements**: Track through ECM/ECCM, hypersonic capability
- **Features**: Anti-jamming, DRFM RGPO countermeasures

### 4. Space (SSA/STM)
- **Standards**: ECSS-E-ST-60-20C, CCSDS
- **Requirements**: ≤1km @ GEO, debris tracking
- **Features**: Orbit determination, conjunction assessment

### 5. Maritime (VTS/VTMS)
- **Standards**: IMO Resolution A.857(20), SOLAS Chapter V
- **Requirements**: ≤30m accuracy, AIS fusion
- **Features**: NMEA 2000 output

## 🚀 Quick Start

```python
from nx_mimosa_v4_unified import create_tracker

# Create industry-specific tracker
tracker = create_tracker('aviation')  # or 'automotive', 'defense', 'space', 'maritime'

# Initialize with first measurement
tracker.initialize(position=[50000, 10000, 5000], velocity=[-400, -100, 20])

# Process measurements
for measurement in measurements:
    tracker.predict(dt=1.0)
    state = tracker.update(measurement, sigma=50.0)
    
    print(f"Position: {state[:3]}")
    print(f"Velocity: {state[3:6]}")
    print(f"Mode probabilities: {tracker.get_mode_probabilities()}")
```

## 📁 Repository Structure

```
nx-mimosa/
├── python/
│   ├── nx_mimosa_v4_unified.py      # Unified tracker (all industries)
│   ├── atc_compliance_validation.py  # ATC/ATM compliance testing
│   ├── qedmma_pro/                   # PRO version modules
│   │   ├── core/
│   │   │   ├── ukf.py               # Unscented Kalman Filter
│   │   │   ├── ckf.py               # Cubature Kalman Filter
│   │   │   ├── adaptive_noise.py    # Q/R adaptation
│   │   │   └── zero_dsp_correlator.py
│   │   ├── exclusive/
│   │   │   ├── anomaly_hunter.py    # Threat detection
│   │   │   └── multi_fusion.py      # Multi-sensor fusion
│   │   └── layer2a/
│   │       └── micro_doppler_classifier.py
│   └── eccm/
│       └── ew_resilience.py         # EW countermeasures
├── rtl/                              # FPGA implementation
│   ├── nx_mimosa_top.sv
│   ├── ukf_pipeline.sv
│   └── imm_controller.sv
├── fpga/                             # Build scripts
│   ├── vivado_project.tcl
│   └── constraints/
├── docs/
│   ├── ARCHITECTURE.md
│   ├── COMPLIANCE.md
│   └── API_REFERENCE.md
└── benchmarks/
    └── performance_results/
```

## ⚙️ Algorithm Details

### VS-IMM (Variable-Structure IMM)

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                     VS-IMM ARCHITECTURE                                     │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   ┌──────────┐     ┌──────────┐     ┌──────────┐                           │
│   │  Mode 1  │     │  Mode 2  │     │  Mode 3  │                           │
│   │   CV     │     │   CT-L   │     │   CT-H   │                           │
│   │ (cruise) │     │ (light)  │     │ (heavy)  │                           │
│   └────┬─────┘     └────┬─────┘     └────┬─────┘                           │
│        │                │                │                                  │
│        └────────────────┼────────────────┘                                  │
│                         │                                                   │
│                    ┌────▼────┐                                              │
│                    │ Mixing  │ ← Adaptive TPM based on μ_cv                 │
│                    └────┬────┘                                              │
│                         │                                                   │
│                    ┌────▼────┐                                              │
│                    │Combined │                                              │
│                    │Estimate │                                              │
│                    └─────────┘                                              │
│                                                                             │
│   Transition Probability Matrix (Dynamic):                                  │
│   μ_cv > 0.8 → p = 0.95 (stable, low transition)                           │
│   μ_cv > 0.5 → p = 0.90 (moderate)                                          │
│   μ_cv < 0.5 → p = 0.85 (maneuvering, high transition)                     │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Adaptive Noise Estimation

```python
# Innovation-based R adaptation
actual_cov = np.cov(innovations.T)
r_scale = trace(actual_cov) / trace(predicted_S)
R_adapted = R_nominal * r_scale

# NIS-based Q adaptation
if NIS > χ²_threshold:
    Q_scale *= 1.2  # Increase for maneuvering
else:
    Q_scale *= 0.95  # Decrease for stable flight
```

## 🛡️ ECCM Capabilities (Defense Version)

| Threat | Detection Method | Countermeasure | Effectiveness |
|--------|-----------------|----------------|---------------|
| **Noise Jamming** | R estimation | Adaptive R inflation | 95% |
| **DRFM VGPO** | Velocity inconsistency | Soft gating | 99% |
| **Cross-Eye** | Angle jitter | R inflation only | 48% |
| **False Targets** | Track divergence | Innovation gating | 99% |
| **DRFM RGPO** | *Requires hardware* | Frequency agility | 70%* |

*Requires FPGA frequency hopping implementation

## 🔧 FPGA Implementation

Target platforms:
- **RFSoC 4x2** ($2,499) - Development/prototype
- **ZCU208** ($13,194) - Production evaluation
- **ZU48DR** - Production deployment

Resources (estimated for ZU48DR):
- LUTs: ~45,000 (12%)
- DSP48: ~120 (8%)
- BRAM: ~80 (15%)
- Clock: 250 MHz

## 📜 License

**Dual License:**

1. **Open Source**: AGPL v3 - Free for open-source projects
2. **Commercial**: Contact licensing@nexellum.com

## 🤝 Support & Contact

- **Email**: mladen@nexellum.com
- **Phone**: +385 99 737 5100
- **GitHub Issues**: For bug reports and feature requests

## 📚 References

1. Bar-Shalom, Y., Li, X. R., & Kirubarajan, T. (2001). *Estimation with Applications to Tracking and Navigation*.
2. EUROCONTROL. (2022). *Specification for ATM Surveillance System Performance (EASSP)*.
3. Blackman, S. S., & Popoli, R. (1999). *Design and Analysis of Modern Tracking Systems*.

---

**© 2024-2026 Nexellum d.o.o. All rights reserved.**

*Dr. Mladen Mešter - Radar Systems Architect*
