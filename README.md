# QEDMMA-Pro™ Commercial IP Suite

<p align="center">
  <img src="docs/images/qedmma_pro_logo.png" alt="QEDMMA-Pro Logo" width="400">
</p>

<p align="center">
  <b>Next-Generation Radar Tracking IP for Defense & Aerospace</b><br>
  <i>From the creators of QEDMMA-Lite</i>
</p>

---

## 🎯 Overview

**QEDMMA-Pro™** is a suite of production-ready IP cores and software modules for advanced radar target tracking. Built on years of research and proven in simulation against the most demanding scenarios, QEDMMA-Pro delivers capabilities that traditional tracking systems cannot match.

| Metric | Traditional IMM/EKF | QEDMMA-Pro™ |
|--------|---------------------|-------------|
| Position RMSE (Mach 8, 60g) | > 500m | **< 50m** |
| Track Loss Rate | 15-30% | **< 1%** |
| Max Trackable G-Load | 20-30g | **100g+** |
| Latency (FPGA) | - | **< 1 μs** |
| Anomaly Detection | ❌ | ✅ |
| Async Network Support | ❌ | ✅ |

---

## 📦 Products

### 1. QEDMMA-Pro FPGA IP Core

**Synthesizable RTL for real-time radar systems**

Production-ready FPGA IP core implementing the full QEDMMA algorithm with hardware optimizations:

- **4-Model IMM Filter** (CV, CA, CT, Jerk)
- **TDOA/Doppler Fusion** with Gauss-Newton solver
- **Fixed-Point Optimized** (Q16.16 format)
- **AXI4-Stream Interface** for easy SoC integration
- **Target Platforms:** AMD Zynq UltraScale+ RFSoC, Intel Agilex, Xilinx Kintex/Virtex

**Performance:**
- Clock: 300-600 MHz
- Latency: < 1 μs per update
- Resource Usage: ~15K LUTs, ~30 DSP slices (Zynq UltraScale+)

**Deliverables:**
- Encrypted RTL (Verilog/VHDL)
- Simulation testbench with reference vectors
- Integration guide & timing constraints
- Technical support (12 months)

**Licensing:** Per-project license + optional royalty

---

### 2. Anomaly Hunter™ Module

**Physics-Agnostic Tracking for Unconventional Targets**

The industry's first tracking algorithm designed to maintain lock on targets that violate classical physics models:

- **Instant Direction Changes** without deceleration
- **Impossible G-Loads** (>1000g observed acceleration)
- **Non-Ballistic Trajectories** that defy aerodynamics
- **Position Discontinuities** (apparent teleportation)

**How It Works:**

```
                    ┌─────────────────────────────────────┐
                    │        QEDMMA Core (Layer 1)        │
                    │         IMM + TDOA Fusion           │
                    └─────────────────┬───────────────────┘
                                      │
                    ┌─────────────────┴───────────────────┐
                    │                                     │
            ┌───────▼───────┐                   ┌─────────▼─────────┐
            │   Layer 2A    │                   │     Layer 2B      │
            │    Physics    │                   │  Physics-Agnostic │
            │  Constrained  │                   │     (Anomaly      │
            │     GAT       │                   │     Hunter™)      │
            └───────┬───────┘                   └─────────┬─────────┘
                    │                                     │
                    │         ┌───────────────┐           │
                    └────────►│  Anomaly      │◄──────────┘
                              │  Divergence   │
                              │  Monitor      │
                              └───────┬───────┘
                                      │
                                      ▼
                              [FUSED OUTPUT]
                              + anomaly_flag
```

**Key Feature:** When the Anomaly Divergence Monitor (ADM) detects that observations exceed 5-sigma from physics-based predictions, Layer 2B takes over tracking. This prevents track loss on unconventional targets while maintaining precision on conventional threats.

**Use Cases:**
- UAP/UAV detection and tracking
- Advanced anti-drone systems
- National security applications
- Research and data collection

**Deliverables:**
- Encrypted RTL module
- Python reference implementation
- Integration API
- Classified documentation (with appropriate clearances)

**Licensing:** Government/Defense contracts only. Contact for pricing.

---

### 3. Asynchronous Multi-Static Fusion Engine

**Radar Network Operation Without Expensive Synchronization**

Enable distributed radar networks to operate cohesively even when precision timing infrastructure (White Rabbit, GPS) is degraded or unavailable:

**Features:**
- **Clock-Bias Estimation:** Real-time estimation of per-node clock drift
- **Graceful Degradation:** Maintains tracking when sync is lost
- **Self-Healing:** Automatically re-synchronizes when timing restored
- **Cost Savings:** Eliminates need for expensive atomic clocks at each node

**Architecture:**

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   Node 1    │     │   Node 2    │     │   Node N    │
│  (Primary)  │     │ (Secondary) │     │ (Secondary) │
│             │     │             │     │             │
│ t₁ = 0 ns   │     │ t₂ = Δt₂   │     │ tₙ = Δtₙ   │
└──────┬──────┘     └──────┬──────┘     └──────┬──────┘
       │                   │                   │
       └───────────────────┼───────────────────┘
                           │
                   ┌───────▼───────┐
                   │ Fusion Engine │
                   │               │
                   │ Estimates:    │
                   │ • Position    │
                   │ • Velocity    │
                   │ • Clock Bias  │◄── Novel!
                   │   Δt₂...Δtₙ   │
                   └───────────────┘
```

**Specifications:**
- Up to 16 nodes supported
- Clock bias estimation accuracy: < 10 ns
- Drift rate tracking: up to 100 ns/s
- Recovery time after sync loss: < 1 second

**Deliverables:**
- Software library (C++/Python)
- FPGA accelerator IP (optional)
- Integration documentation
- Network simulation toolkit

**Licensing:** Annual subscription or perpetual license

---

## 💰 Pricing

| Product | License Type | Starting Price |
|---------|-------------|----------------|
| QEDMMA-Pro FPGA IP | Per-Project | $50,000 |
| QEDMMA-Pro FPGA IP | Enterprise (unlimited) | $200,000 |
| Anomaly Hunter™ | Government Contract | Contact Us |
| Async Fusion Engine | Annual Subscription | $25,000/year |
| Async Fusion Engine | Perpetual | $75,000 |
| Full Suite Bundle | Enterprise | $350,000 |

**Volume discounts and royalty arrangements available.**

---

## 🏢 About Mešter Labs

**Mešter Labs** is a defense technology company specializing in advanced radar signal processing and target tracking algorithms. Founded by Dr. Mladen Mešter, a physician and engineer with expertise in FPGA development and RF systems.

**Our Mission:** Deliver tracking capabilities that protect nations and save lives.

**Clients Include:**
- Defense ministries
- Prime defense contractors
- Aerospace companies
- Research institutions

---

## 📞 Contact

**Sales Inquiries:**
- Email: sales@mester-labs.com
- Phone: +385 XX XXX XXXX

**Technical Support (Existing Customers):**
- Email: support@mester-labs.com
- Portal: https://support.mester-labs.com

**General Information:**
- Website: https://www.mester-labs.com
- LinkedIn: [Mešter Labs](https://linkedin.com/company/mester-labs)

---

## 📋 Documentation

Detailed documentation is available to licensed customers:

- [FPGA IP Integration Guide](docs/fpga_integration_guide.md) 🔒
- [Anomaly Hunter Technical Reference](docs/anomaly_hunter_reference.md) 🔒
- [Async Fusion API Reference](docs/async_fusion_api.md) 🔒
- [Deployment Best Practices](docs/deployment_guide.md) 🔒

🔒 = Requires customer portal login

---

## ⚖️ Legal

QEDMMA-Pro™ and Anomaly Hunter™ are trademarks of Mešter Labs.

All IP cores and software are provided under commercial license. Unauthorized distribution, reverse engineering, or use in weapons of mass destruction is strictly prohibited.

Export of this technology may be subject to ITAR, EAR, or equivalent regulations in your jurisdiction.

---

<p align="center">
  <b>© 2026 Mešter Labs. All Rights Reserved.</b>
</p>
