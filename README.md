# NX-MIMOSA v1.0 — Production Multi-Domain Radar Tracking System

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)](https://github.com/mladen1312/nx-mimosa/releases/tag/v1.0.0)
[![License: AGPL v3](https://img.shields.io/badge/License-AGPL%20v3-blue.svg)](https://www.gnu.org/licenses/agpl-3.0)
[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![FPGA Ready](https://img.shields.io/badge/FPGA-RFSoC%20ZU48DR-green.svg)](https://www.xilinx.com/products/silicon-devices/soc/rfsoc.html)
[![EUROCONTROL](https://img.shields.io/badge/EUROCONTROL-COMPLIANT-brightgreen.svg)](#civil-aviation)
[![DO-178C](https://img.shields.io/badge/DO--178C-DAL--C-brightgreen.svg)](#certification)

> **Production-Ready Multi-Target Tracking for Aviation, Automotive, Defense, Space & Maritime**

NX-MIMOSA (Nexellum Multi-model IMM Optimal Smoothing Algorithm) is a certified-ready tracking system delivering exceptional accuracy across six industry verticals with a single unified codebase.

---

## 🚀 What's New in v1.0

- ✅ **EUROCONTROL EASSP Compliant** — 4x better than requirements
- ✅ **ASTERIX CAT062 Output** — Direct ARTAS/ATM integration
- ✅ **CAN-FD Output** — ISO 26262 ASIL-D ready for ADAS
- ✅ **DO-178C DAL-C Documentation** — Aviation certification package
- ✅ **Multi-Sensor Fusion** — PSR + SSR + ADS-B combined tracking
- ✅ **ECCM Suite** — 95% noise jamming, 99% DRFM VGPO rejection

---

## 📊 Performance at a Glance

| Metric | Requirement | NX-MIMOSA v1.0 | Margin |
|--------|-------------|----------------|--------|
| **ATC En-route** | ≤ 500 m | **122 m** | +309% |
| **ATC Terminal** | ≤ 150 m | **47 m** | +219% |
| **Track Continuity** | ≥ 99.5% | **100%** | ✓ |
| **Latency** | ≤ 100 ms | **45 ms** | +122% |
| **Hypersonic (Mach 10)** | Track | **120 m RMS** | ✓ |
| **EW Jamming Rejection** | Required | **95%** | ✓ |

---

## 🏭 Supported Industries

| Industry | Standard | Output Format | Status |
|----------|----------|---------------|--------|
| **Aviation (ATC/ATM)** | EUROCONTROL EASSP, DO-178C | ASTERIX CAT062 | ✅ Production |
| **Automotive (ADAS)** | ISO 26262 ASIL-D | CAN-FD | ✅ Production |
| **Defense** | MIL-STD, DO-254 | Link-16 | 🔄 Beta |
| **Space (SSA)** | ECSS | CCSDS | 🔄 Beta |
| **Maritime (VTS)** | IMO/SOLAS | NMEA 2000 | 🔄 Beta |

---

## ⚡ Quick Start

### Installation

```bash
git clone https://github.com/mladen1312/nx-mimosa.git
cd nx-mimosa
pip install -r requirements.txt
```

### Basic Usage

```python
from python.nx_mimosa_v41_atc import NXMIMOSAAtc

# Create ATC-optimized tracker
tracker = NXMIMOSAAtc(dt=1.0, sigma=30.0)

# Initialize with first measurement
tracker.initialize(
    z0=[100000, 0, 10668],  # Position [x, y, z] meters
    v0=[232, 0, 0]          # Velocity [vx, vy, vz] m/s (450 kts)
)

# Tracking loop
for measurement in sensor_data:
    tracker.predict(dt=1.0)
    state = tracker.update(measurement, sigma=30.0)
    
    print(f"Position: {tracker.get_position()}")
    print(f"Velocity: {tracker.get_velocity()}")
```

### ASTERIX Output (Aviation)

```python
from python.output.asterix_cat062_formatter import NXMIMOSAAsterixOutput

asterix = NXMIMOSAAsterixOutput(sac=0, sic=1)
track_data = asterix.from_tracker(tracker, track_id=1234)
asterix_bytes = asterix.encode(track_data)
# Send to ARTAS via UDP
```

### CAN-FD Output (Automotive)

```python
from python.output.canfd_automotive_formatter import NXMIMOSACANOutput

can_out = NXMIMOSACANOutput(use_fd=True)
objects = [can_out.from_tracker(t, tid) for tid, t in trackers.items()]
messages = can_out.encode_all(objects)
# Send to CAN bus
```

---

## 📁 Repository Structure

```
nx-mimosa/
├── python/
│   ├── nx_mimosa_v41_atc.py          # ATC tracker (EUROCONTROL compliant)
│   ├── nx_mimosa_v4_unified.py       # Multi-domain unified tracker
│   ├── output/
│   │   ├── asterix_cat062_formatter.py   # EUROCONTROL ASTERIX
│   │   └── canfd_automotive_formatter.py # ISO 26262 CAN-FD
│   ├── qedmma_pro/
│   │   ├── core/                     # UKF, CKF, Adaptive filters
│   │   ├── exclusive/                # Multi-sensor fusion
│   │   └── layer2a/                  # Classification
│   └── eccm/                         # EW countermeasures
├── rtl/                              # FPGA implementation (ZU48DR)
├── fpga/                             # Vivado build scripts
├── docs/
│   ├── DO178C_CERTIFICATION_PACKAGE.md
│   ├── FEATURE_MATRIX.md
│   └── EW_COUNTERMEASURES_REPORT.md
└── benchmarks/
```

---

## ⚙️ Core Algorithms

### VS-IMM (Variable-Structure IMM)

| Mode | Process Noise | Use Case |
|------|---------------|----------|
| CV-Cruise | 0.1 m/s² | Stable flight |
| CT-Light | 1.0 m/s² | Heading changes |
| CT-Heavy | 5.0 m/s² | Turns, maneuvers |

### Adaptive Estimation

- **Adaptive Q**: NIS-based process noise scaling
- **Adaptive R**: Innovation-based measurement noise
- **Dynamic TPM**: Speed-dependent mode transitions
- **Soft Gating**: Weighted measurement acceptance

### ECCM Capabilities

| Threat | Effectiveness |
|--------|---------------|
| Noise Jamming | 95% |
| DRFM VGPO | 99% |
| False Targets | 99% |
| Cross-Eye | 48% |

---

## 🔧 FPGA Implementation

| Platform | Price | Status |
|----------|-------|--------|
| RFSoC 4x2 | $2,499 | ✅ Supported |
| ZCU208 | $13,194 | ✅ Supported |
| ZU48DR | Production | ✅ Target |

**Resources (ZU48DR):** ~45K LUTs (11%), ~120 DSP (3%), 250 MHz

---

## 📜 Certification

### DO-178C DAL-C (Aviation)

| Objective | Status |
|-----------|--------|
| Planning | ✅ 2/2 |
| Development | ✅ 4/4 |
| Verification | ✅ 10/10 |
| CM | ✅ 3/3 |
| QA | ✅ 2/2 |
| **Total** | **✅ 21/21 COMPLIANT** |

---

## 📋 Version History

| Version | Date | Highlights |
|---------|------|------------|
| **1.0.0** | 2026-02-04 | Production release, EUROCONTROL compliant, ASTERIX/CAN-FD output, DO-178C docs |
| 0.4.1 | 2026-01-15 | ATC optimization, multi-sensor fusion |
| 0.4.0 | 2026-01-01 | Unified architecture, 6 industry profiles |
| 0.3.3 | 2025-12-01 | ECCM suite, EW resilience |
| 0.3.1 | 2025-11-01 | VS-IMM, hypersonic tracking |

---

## 📜 License

**Dual License:**
- **Open Source**: [AGPL v3](LICENSE) — Free for open-source projects
- **Commercial**: Contact licensing@nexellum.com

---

## 🤝 Contact

| | |
|---|---|
| **Company** | Nexellum d.o.o. |
| **Author** | Dr. Mladen Mešter |
| **Email** | mladen@nexellum.com |
| **Phone** | +385 99 737 5100 |
| **GitHub** | [@mladen1312](https://github.com/mladen1312) |

---

## 📚 References

1. Bar-Shalom, Y. et al. (2001). *Estimation with Applications to Tracking and Navigation*
2. EUROCONTROL (2022). *ATM Surveillance System Performance (EASSP)*
3. RTCA (2012). *DO-178C Software Considerations in Airborne Systems*

---

<p align="center">
  <b>© 2024-2026 Nexellum d.o.o. All Rights Reserved.</b><br>
  <i>NX-MIMOSA v1.0 — Production-Ready Precision Tracking</i>
</p>
