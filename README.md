# QEDMMA-PRO v3.0

> **Production-Ready Radar Tracking & Signal Processing Suite**

[![License](https://img.shields.io/badge/license-Commercial-gold.svg)](LICENSE_COMMERCIAL.md)
[![Version](https://img.shields.io/badge/version-3.0.0-blue.svg)]()

---

## 🎯 Overview

**QEDMMA-PRO** is the commercial extension of [QEDMMA-Lite](https://github.com/mladen1312/qedmma-lite), providing enterprise-grade tracking algorithms and FPGA IP cores for:

- 🚗 **Automotive** - ADAS, autonomous vehicles, sensor fusion
- ✈️ **Aerospace/Defense** - Air defense, missile tracking, ECCM
- 🚢 **Maritime** - VTS, collision avoidance, AIS integration
- 🤖 **Robotics** - Warehouse tracking, drone fleets
- 🛰️ **Space** - Debris tracking, conjunction assessment

---

## 📊 Performance Comparison

| Target Type | Industry Standard | QEDMMA-Lite | **QEDMMA-PRO** |
|-------------|:-----------------:|:-----------:|:--------------:|
| Fighter Aircraft | 150m | 33m | **< 15m** |
| Cruise Missile | 200m | 41m | **< 20m** |
| Hypersonic (M5+) | 2500m | 95m | **< 50m** |
| Skip-Glide Maneuver | Track Loss | Track Loss | **< 100m** ✨ |
| Multi-target (1000+) | ❌ | ~100 | **1024+** |

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                              QEDMMA-PRO LAYERS                                  │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                 │
│  LAYER 5: C2 INTEGRATION                                                        │
│  ├── Link-16 Interface (NATO tactical data link)                               │
│  ├── ASTERIX Parser (EUROCONTROL surveillance)                                 │
│  └── AIS Integration (Maritime)                                                │
│                                                                                 │
│  LAYER 4: MULTI-SENSOR FUSION                                                  │
│  ├── Track Fusion Engine (1024 simultaneous tracks)                            │
│  ├── Covariance Intersection                                                   │
│  ├── Global Nearest Neighbor association                                       │
│  └── JDL Fusion Levels 0-4                                                     │
│                                                                                 │
│  LAYER 3: TRACKING (Enhanced)                                                   │
│  ├── UKF-Pro (SR-UKF, IUKF, State Constraints)                                │
│  ├── CKF-Pro (Higher-order cubature)                                           │
│  ├── IMM (CV/CA/CT motion models)                                              │
│  └── GPU Acceleration (CUDA/CuPy)                                              │
│                                                                                 │
│  LAYER 2B: ANOMALY TRACKING 🔒 PRO EXCLUSIVE                                   │
│  ├── Anomaly Hunter™ - Physics-agnostic tracking                              │
│  ├── Pattern learning for unconventional targets                               │
│  └── Auto physics↔learned handoff                                             │
│                                                                                 │
│  LAYER 2A: DETECTION & CLASSIFICATION                                          │
│  ├── ML-CFAR Engine (ML-assisted detection)                                    │
│  ├── Micro-Doppler AI Classifier                                               │
│  ├── Jammer Localizer (HOJ capability)                                         │
│  └── DRFM/Decoy rejection                                                      │
│                                                                                 │
│  LAYER 1: SIGNAL PROCESSING                                                     │
│  ├── Zero-DSP Correlator (parallel streaming)                                  │
│  ├── Coherent Integrator                                                       │
│  ├── Digital AGC                                                               │
│  └── Polyphase Decimator                                                       │
│                                                                                 │
│  LAYER 0: RF FRONTEND                                                           │
│  ├── BladeRF driver                                                            │
│  ├── PlutoSDR driver                                                           │
│  └── USRP driver                                                               │
│                                                                                 │
└─────────────────────────────────────────────────────────────────────────────────┘
```

---

## 🔒 PRO-Exclusive Features

### Anomaly Hunter™ (Layer 2B)

Physics-agnostic tracking for unconventional targets:

```python
from qedmma_pro import AnomalyHunter, AnomalyConfig

hunter = AnomalyHunter(AnomalyConfig(blend_alpha=0.7))

for measurement in radar_data:
    state = hunter.process(
        target_id=1,
        measured_pos=measurement[:3],
        physics_pred=kalman_prediction,
        pred_cov=covariance,
        timestamp=t
    )
    
    if state.l2b_active:
        # Physics failed - using learned pattern
        prediction = state.blended_pred
```

**Results on hypersonic skip-glide:**
- Standard trackers: Track loss during skip
- Anomaly Hunter: Maintains track, RMSE < 100m

### Enhanced UKF-Pro

```python
from qedmma_pro import UKFPro, UKFProParams

params = UKFProParams(
    adaptive_scaling=True,      # Auto-tune sigma points
    iterated_updates=True,      # IUKF for nonlinear h()
    constraints_enabled=True,   # Enforce physical bounds
)

ukf = UKFPro(f, h, n_states=9, n_meas=3, params=params)
state, metrics = ukf.update(state, z)

# Diagnostics
print(f"NEES: {metrics['nees']:.2f}, Health: {metrics['health']:.2f}")
```

---

## ⚡ FPGA IP Cores

22+ production-ready SystemVerilog modules:

| Category | Modules | Description |
|----------|---------|-------------|
| **Correlator** | 4 | Zero-DSP, parallel, coherent integration |
| **ECCM** | 4 | ML-CFAR, jammer localizer, HOJ |
| **Fusion** | 5 | Track fusion, Link-16, ASTERIX |
| **Sync** | 3 | White Rabbit PTP (<1ns precision) |
| **Frontend** | 2 | Digital AGC, polyphase decimation |
| **Comm** | 4 | Controller, adapters, failover |

Pre-built bitstreams available for:
- Xilinx RFSoC 4x2
- Xilinx ZCU111
- Intel Arria 10 SoC

---

## 💰 Pricing

| Edition | Features | Price |
|---------|----------|-------|
| **Lite** | Open-source baseline | Free (MIT) |
| **PRO Starter** | Python algorithms | $25,000 |
| **PRO Professional** | + FPGA IP cores | $75,000 |
| **PRO Enterprise** | + ECCM + Fusion + C2 | $150,000 |
| **PRO Defense** | + Certification + SLA | Contact |
| **Automotive OEM** | Per-unit licensing | Contact |

Volume discounts available. Academic pricing on request.

---

## 📦 Package Contents

```
qedmma-pro/
├── python/
│   └── qedmma_pro/
│       ├── core/           # UKF-Pro, CKF-Pro, Zero-DSP
│       ├── layer2a/        # ML-CFAR, Micro-Doppler AI
│       ├── layer2b/        # Anomaly Hunter™
│       ├── layer4_fusion/  # Track fusion, JDL
│       └── drivers/        # BladeRF, PlutoSDR, USRP
├── fpga/
│   ├── correlator/         # 4 IP cores
│   ├── eccm/               # 4 IP cores
│   ├── fusion/             # 5 IP cores
│   ├── sync/               # 3 IP cores
│   ├── frontend/           # 2 IP cores
│   └── comm/               # 4 IP cores
├── bitstreams/             # Pre-built for RFSoC
├── docs/
│   ├── API.md
│   ├── FPGA_INTEGRATION.md
│   └── CERTIFICATION.md
└── examples/
    ├── automotive_adas/
    ├── defense_tracking/
    └── maritime_vts/
```

---

## 📜 Certification Support

| Standard | Coverage |
|----------|----------|
| **DO-254** | Design artifacts for DAL-C |
| **DO-178C** | Software artifacts for DAL-C |
| **ISO 26262** | ASIL-B support |
| **MIL-STD-882E** | Safety analysis templates |

---

## 📧 Contact

| | |
|---|---|
| 📧 Email | [mladen@nexellum.com](mailto:mladen@nexellum.com) |
| 🌐 Web | [www.nexellum.com](https://www.nexellum.com) |
| 📱 Phone | +385 99 737 5100 |

---

## 🔗 Related

- **QEDMMA-Lite** (Open Source): [github.com/mladen1312/qedmma-lite](https://github.com/mladen1312/qedmma-lite)
- **Documentation**: Coming soon
- **Blog**: Coming soon

---

<div align="center">

**Built with 🔬 by [Dr. Mladen Mešter](mailto:mladen@nexellum.com)**

*Enterprise radar solutions for automotive, defense, and beyond*

© 2026 Nexellum d.o.o. All rights reserved.

</div>
