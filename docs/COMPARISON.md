# QEDMMA Product Comparison

## Open Source vs Commercial Editions

| Feature | QEDMMA-Lite (Open Source) | QEDMMA-Pro (Commercial) |
|---------|---------------------------|-------------------------|
| **License** | MIT | Commercial / AGPL-3.0 |
| **Price** | Free | Starting $50,000 |
| **Support** | Community | Dedicated Engineering |

---

## Tracking Algorithms

| Algorithm | Lite | Pro |
|-----------|:----:|:---:|
| Extended Kalman Filter (EKF) | ✅ | ✅ |
| Unscented Kalman Filter (UKF) | ✅ | ✅ |
| Cubature Kalman Filter (CKF) | ✅ | ✅ |
| Square-Root UKF/CKF | ❌ | ✅ |
| IMM (4 models) | ✅ | ✅ |
| IMM (N configurable models) | ❌ | ✅ |
| Variable Structure IMM (VS-IMM) | ❌ | ✅ |
| Particle Filter | ❌ | ✅ |
| **Anomaly Hunter™** (Physics-Agnostic) | ❌ | ✅ |

### Anomaly Hunter™ Details

| Capability | Description |
|------------|-------------|
| Input | Raw sensor data (no physics model required) |
| Learning | Online adaptation to target behavior |
| Performance | Tracks UAV swarms, hypersonic vehicles, space debris |
| Latency | < 1ms per update |

---

## Adaptive Noise Estimation

| Method | Lite | Pro |
|--------|:----:|:---:|
| Mehra (Innovation-based) | ✅ | ✅ |
| Sage-Husa | ✅ | ✅ |
| Variational Bayesian | ✅ | ✅ |
| Covariance Matching | ✅ | ✅ |
| IMM-Adaptive | ✅ | ✅ |
| Deep Learning Noise Estimator | ❌ | ✅ |
| Clutter Map Integration | ❌ | ✅ |

---

## FPGA Implementation

| Component | Lite | Pro |
|-----------|:----:|:---:|
| Zero-DSP Correlator | ✅ | ✅ |
| Pulse Compression | ✅ | ✅ |
| Doppler Processing | ❌ | ✅ |
| CFAR Detection | ❌ | ✅ |
| Beamformer | ❌ | ✅ |
| Channelizer | ❌ | ✅ |
| Full Signal Chain IP | ❌ | ✅ |

### FPGA Resource Comparison

| Metric | Lite (Zero-DSP only) | Pro (Full Chain) |
|--------|---------------------|------------------|
| DSP48 Slices | 0 | ~200 |
| LUTs | ~2,000 | ~50,000 |
| BRAM | 0 | ~100 |
| Estimated Clock | 200 MHz | 300 MHz |

---

## Multi-Sensor Fusion

| Capability | Lite | Pro |
|------------|:----:|:---:|
| Single Radar | ✅ | ✅ |
| Multi-Radar Fusion | ❌ | ✅ |
| Radar + EO/IR Fusion | ❌ | ✅ |
| Async Multi-Static | ❌ | ✅ |
| JDL Fusion Levels 0-4 | ❌ | ✅ |
| Edge ML Deployment | ❌ | ✅ |

---

## Data Association

| Algorithm | Lite | Pro |
|-----------|:----:|:---:|
| Global Nearest Neighbor (GNN) | ✅ | ✅ |
| Joint Probabilistic (JPDA) | ❌ | ✅ |
| Multi-Hypothesis (MHT) | ❌ | ✅ |
| Random Finite Sets (RFS) | ❌ | ✅ |

---

## Deployment & Integration

| Feature | Lite | Pro |
|---------|:----:|:---:|
| Python API | ✅ | ✅ |
| C++ API | ❌ | ✅ |
| ROS/ROS2 Integration | ❌ | ✅ |
| DDS Integration | ❌ | ✅ |
| STANAG 4607/4609 | ❌ | ✅ |
| Docker Images | ❌ | ✅ |
| Kubernetes Helm Charts | ❌ | ✅ |

---

## Verification & Certification

| Aspect | Lite | Pro |
|--------|:----:|:---:|
| Unit Tests | ✅ | ✅ |
| Integration Tests | ✅ | ✅ |
| Cocotb RTL Tests | ✅ | ✅ |
| DO-254 Artifacts | ❌ | ✅ |
| DO-178C Artifacts | ❌ | ✅ |
| Formal Verification | ❌ | ✅ |
| MISRA Compliance | ❌ | ✅ |

---

## Support & Services

| Service | Lite | Pro |
|---------|:----:|:---:|
| GitHub Issues | ✅ | ✅ |
| Email Support | ❌ | ✅ |
| Phone Support | ❌ | ✅ |
| Slack Channel | ❌ | ✅ |
| Dedicated Engineer | ❌ | ✅ |
| On-site Training | ❌ | ✅ |
| Custom Development | ❌ | ✅ |

---

## Pricing Tiers

| Edition | Price | Use Case |
|---------|-------|----------|
| **Lite** | Free | Research, Education, Prototyping |
| **Pro Starter** | $50,000 | Single project, 1 FPGA target |
| **Pro Team** | $150,000 | Team license, 5 FPGA targets |
| **Pro Enterprise** | $350,000 | Unlimited, full source, support |

### Volume Discounts

| Quantity | Discount |
|----------|----------|
| 2-5 licenses | 10% |
| 6-10 licenses | 20% |
| 11+ licenses | Contact us |

---

## Upgrade Path

```
┌─────────────────────────────────────────────────────────────────┐
│                    QEDMMA UPGRADE PATH                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  QEDMMA-Lite (Free)                                            │
│       │                                                         │
│       ├──▶ Evaluation of Pro features (30-day trial)           │
│       │                                                         │
│       ▼                                                         │
│  QEDMMA-Pro Starter ($50K)                                     │
│       │                                                         │
│       ├──▶ Add more FPGA targets                               │
│       ├──▶ Add sensor types                                    │
│       │                                                         │
│       ▼                                                         │
│  QEDMMA-Pro Enterprise ($350K)                                 │
│       │                                                         │
│       ├──▶ Full source code                                    │
│       ├──▶ Unlimited deployment                                │
│       └──▶ Custom development available                        │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## Technical Specifications

### Performance Benchmarks

| Scenario | Lite | Pro |
|----------|------|-----|
| Single target, 100 Hz update | 0.2 ms | 0.1 ms |
| 100 targets, 10 Hz update | 15 ms | 5 ms |
| 80g maneuver tracking error | 30 m RMS | 15 m RMS |
| Clutter density 10^6 /km³ | N/A | < 1% false tracks |

### Supported Platforms

| Platform | Lite | Pro |
|----------|:----:|:---:|
| Linux (x86_64) | ✅ | ✅ |
| Windows | ✅ | ✅ |
| macOS | ✅ | ✅ |
| ARM64 (Jetson, RPi) | ✅ | ✅ |
| AMD RFSoC | ✅ | ✅ |
| Intel/Altera FPGA | ❌ | ✅ |
| Microchip PolarFire | ❌ | ✅ |

---

## Contact

**Sales & Licensing**
- Email: mladen@nexellum.com
- Phone: +385 99 737 5100
- Web: https://www.nexellum.com

**Technical Support**
- GitHub: https://github.com/mladen1312/qedmma-lite/issues
- Email: support@nexellum.com

---

**© 2026 Nexellum d.o.o. All rights reserved.**

*QEDMMA, Anomaly Hunter, and Nexellum are trademarks of Nexellum d.o.o.*
