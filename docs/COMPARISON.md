# QEDMMA Feature Comparison: Lite vs Pro

## Overview

| Feature | QEDMMA-Lite (AGPL) | QEDMMA-Pro (Commercial) |
|---------|-------------------|------------------------|
| **License** | AGPL-3.0 (copyleft) | Commercial (proprietary OK) |
| **Price** | Free | Contact for pricing |
| **Support** | Community | Priority support |
| **Updates** | Public releases | Early access + hotfixes |

---

## Algorithm Comparison

### Kalman Filters

| Algorithm | Lite | Pro |
|-----------|------|-----|
| Extended Kalman Filter (EKF) | ✅ | ✅ |
| Unscented Kalman Filter (UKF) | ✅ | ✅ |
| Cubature Kalman Filter (CKF) | ✅ | ✅ |
| Square-Root CKF | ✅ | ✅ |
| **Gaussian Process UKF (GP-UKF)** | ❌ | ✅ |
| **Particle Filter (PF)** | ❌ | ✅ |
| **Rao-Blackwellized PF** | ❌ | ✅ |

### Adaptive Estimation

| Feature | Lite | Pro |
|---------|------|-----|
| Innovation-based R estimation | ✅ | ✅ |
| Covariance matching | ✅ | ✅ |
| Sage-Husa adaptive | ✅ | ✅ |
| **ML hyperparameter optimization** | ❌ | ✅ |
| **Online model selection** | ❌ | ✅ |

### Multi-Target Tracking

| Algorithm | Lite | Pro |
|-----------|------|-----|
| Nearest Neighbor (NN) | ✅ | ✅ |
| Global Nearest Neighbor (GNN) | ✅ | ✅ |
| **Joint Probabilistic Data Association (JPDA)** | ❌ | ✅ |
| **Multi-Hypothesis Tracking (MHT)** | ❌ | ✅ |
| **Poisson Multi-Bernoulli Mixture (PMBM)** | ❌ | ✅ |

### Track Management

| Feature | Lite | Pro |
|---------|------|-----|
| Basic track initiation/deletion | ✅ | ✅ |
| M-of-N logic | ✅ | ✅ |
| **Integrated Track Scoring** | ❌ | ✅ |
| **Track-to-Track Fusion** | ❌ | ✅ |
| **Distributed Tracking** | ❌ | ✅ |

---

## FPGA Features

| Feature | Lite | Pro |
|---------|------|-----|
| Zero-DSP Correlator (VHDL) | ✅ | ✅ |
| Zero-DSP Correlator (HLS) | ✅ | ✅ |
| **Pipelined UKF Core** | ❌ | ✅ |
| **Hardware CKF** | ❌ | ✅ |
| **AXI4-Stream Interface** | ❌ | ✅ |
| **Multi-Channel Tracker** | ❌ | ✅ |
| **Real-Time Scheduler** | ❌ | ✅ |

### Resource Comparison (Zynq UltraScale+)

| IP Core | Lite Available | Pro Features | LUTs | DSPs | Fmax |
|---------|---------------|--------------|------|------|------|
| Zero-DSP Correlator | ✅ | - | 2.8K | 0 | 1.2 GHz |
| UKF Core (4-state) | ❌ | ✅ | 8.5K | 24 | 250 MHz |
| CKF Core (9-state) | ❌ | ✅ | 15.2K | 48 | 200 MHz |
| Multi-Target (8 tracks) | ❌ | ✅ | 42K | 96 | 150 MHz |

---

## Software Features

| Feature | Lite | Pro |
|---------|------|-----|
| Python API | ✅ | ✅ |
| NumPy/SciPy only dependencies | ✅ | ✅ |
| **C++ High-Performance Library** | ❌ | ✅ |
| **MATLAB/Simulink Integration** | ❌ | ✅ |
| **ROS2 Node** | ❌ | ✅ |

### Performance (100 targets, 10 Hz)

| Metric | Lite (Python) | Pro (C++) |
|--------|---------------|-----------|
| UKF cycle time | 2.4 ms | 0.12 ms |
| CKF cycle time | 1.9 ms | 0.09 ms |
| JPDA cycle time | N/A | 0.8 ms |
| Memory usage | 45 MB | 8 MB |

---

## Documentation & Support

| Item | Lite | Pro |
|------|------|-----|
| Algorithm documentation | ✅ | ✅ |
| API reference | ✅ | ✅ |
| **Application notes** | ❌ | ✅ |
| **Integration guides** | ❌ | ✅ |
| **Training videos** | ❌ | ✅ |
| Email support | Community | Priority (24h response) |
| **Dedicated Slack channel** | ❌ | ✅ |
| **On-site training** | ❌ | ✅ (additional fee) |

---

## Licensing Terms

### QEDMMA-Lite (AGPL-3.0)

- ✅ Free for open-source projects
- ✅ Free for academic research
- ⚠️ Derivative works must be AGPL-licensed
- ⚠️ Network use triggers copyleft
- ❌ Cannot be used in proprietary products

### QEDMMA-Pro (Commercial)

- ✅ Use in proprietary products
- ✅ No source code disclosure
- ✅ Sublicensing available
- ✅ Patent indemnification
- ✅ Export compliance support

---

## Pricing

| Tier | Annual License | Includes |
|------|----------------|----------|
| **Startup** | €5,000 | 1 product, 5 developers |
| **Professional** | €15,000 | 3 products, 20 developers |
| **Enterprise** | €50,000 | Unlimited products/developers |
| **Defense/Gov** | Custom | ITAR/EAR compliance, on-premise |

**Volume discounts available for multi-year commitments.**

---

## Contact

**Nexellum d.o.o.**

- 📧 Email: mladen@nexellum.com
- 🌐 Web: [www.nexellum.com](https://www.nexellum.com)
- 📱 Phone: +385 99 737 5100
- 📍 Location: Zagreb, Croatia

---

*© 2026 Dr. Mladen Mešter / Nexellum. All rights reserved.*
