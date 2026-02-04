# NX-MIMOSA — Advanced Radar Tracking Algorithm

[![License](https://img.shields.io/badge/License-AGPL%20v3-blue.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.8%2B-green.svg)](https://www.python.org/)
[![Benchmark](https://img.shields.io/badge/Benchmark-Verified-brightgreen.svg)](#benchmark-results)

**NX-MIMOSA** (Nexellum Multi-Model Interacting Multiple Model with Optimal Smoother Architecture) is a state-of-the-art radar tracking algorithm for defense applications. It provides **+36% accuracy improvement** over standard IMM and **+93% over EKF** in challenging maneuvering target scenarios.

## 🏆 Benchmark Results

Run the benchmark yourself to verify:

```bash
python3 benchmarks/fair_benchmark.py --runs 50 --seed 42
```

| Scenario | EKF-CV | EKF-CA | α-β-γ | IMM-3 | **IMM-Smooth** | Winner |
|----------|--------|--------|-------|-------|----------------|--------|
| Missile Terminal (M4, 9g) | 165.87 | 2.72 | 7.51 | 2.29 | **1.01** | 🥇 IMM-Smooth |
| Hypersonic Glide (M5, 2g) | 25.62 | 4.18 | 15.11 | 3.14 | **1.37** | 🥇 IMM-Smooth |
| Fighter Aircraft (8g) | 99.61 | 1.89 | 4.53 | 1.46 | **0.67** | 🥇 IMM-Smooth |
| Cruise Missile (3g) | 47.84 | 10.06 | 22.23 | 8.58 | **4.02** | 🥇 IMM-Smooth |
| Ballistic RV (M7, 1g) | 72.40 | 27.23 | 75.91 | 38.13 | 29.16 | EKF-CA |
| UAV Swarm (2g) | 9.73 | 1.79 | 3.00 | 1.50 | **0.85** | 🥇 IMM-Smooth |
| Stealth Aircraft (4g) | 340.12 | 29.29 | 36.43 | 21.35 | **12.11** | 🥇 IMM-Smooth |
| **AVERAGE** | 108.74 | 11.02 | 23.53 | 10.92 | **7.03** | 🏆 **IMM-Smooth** |

**Summary: IMM-Smooth wins 6/7 scenarios with +36% average improvement over standard IMM.**

## 📊 Improvement Analysis

| vs Algorithm | Improvement |
|--------------|-------------|
| vs EKF-CV | **+93.5%** |
| vs α-β-γ | **+70.1%** |
| vs EKF-CA | **+36.2%** |
| vs IMM-3 (Forward) | **+35.7%** |

## 🔬 Key Innovations

### 1. True IMM Smoother (v3.1 Bugfix)

The critical insight: RTS smoother backward pass must use `F @ x_mixed` (the state used for forward prediction), not `F @ x_filt`. This eliminates numerical instability during mode transitions.

```python
# ❌ WRONG (causes divergence during maneuvers)
x_pred_backward = F @ x_filt[k+1]

# ✅ CORRECT (stable and accurate)
x_pred_backward = F @ x_mixed[k]  # Stored during forward pass
```

### 2. Variable-Structure IMM (VS-IMM)

Dynamic transition probability matrix based on mode confidence:
- High CV probability → Higher persistence (p_stay = 0.98)
- Mixed modes → Lower persistence (p_stay = 0.90)
- Faster response to maneuver onset

### 3. Adaptive Process Noise

NIS-based Q scaling:
```python
if NIS > χ²_threshold:
    Q_scale = min(Q_scale * 1.3, 3.0)  # Increase for unexpected maneuver
elif NIS < χ²_threshold / 2:
    Q_scale = max(Q_scale * 0.98, 0.3)  # Decrease during steady flight
```

### 4. Joseph Form Covariance Update

Numerically stable covariance update:
```python
I_KH = I - K @ H
P = I_KH @ P_pred @ I_KH.T + K @ R @ K.T  # Joseph form
```

## 📁 Repository Structure

```
nx-mimosa/
├── benchmarks/
│   └── fair_benchmark.py      # Reproducible benchmark (run this!)
├── rtl/
│   ├── nx_mimosa_v31_top.sv   # FPGA implementation (ZU48DR)
│   ├── adaptive_q_module.sv   # Adaptive process noise
│   ├── dynamic_tpm_module.sv  # VS-IMM transition matrix
│   └── ukf_core.sv            # UKF for nonlinear measurements
├── python/
│   └── v31_hypersonic_validation.py  # Validation simulations
├── docs/
│   ├── BENCHMARK_REPORT.md    # Detailed results
│   └── BOM_SEEKER_BOARD_V31.md # Hardware cost analysis
└── README.md
```

## 🚀 Quick Start

### Requirements

- Python 3.8+
- NumPy only (no other dependencies!)

### Run Benchmark

```bash
# Clone repository
git clone https://github.com/mladen1312/nx-mimosa.git
cd nx-mimosa

# Run benchmark (default: 50 Monte Carlo runs)
python3 benchmarks/fair_benchmark.py

# Quick test (10 runs)
python3 benchmarks/fair_benchmark.py --quick

# Custom configuration
python3 benchmarks/fair_benchmark.py --runs 100 --seed 12345
```

### Expected Output

```
BENCHMARK RESULTS — Position RMSE (meters, lower is better)
====================================================================================================
Scenario                                  EKF-CV       EKF-CA        α-β-γ        IMM-3   IMM-Smooth
----------------------------------------------------------------------------------------------------
Missile Terminal (Mach 4, 9g)             165.87         2.72         7.51         2.29         1.01 🥇
...
AVERAGE                                   108.74        11.02        23.53        10.92         7.03
====================================================================================================

SUMMARY
Wins per algorithm:
  🏆 IMM-Smooth     : 6/7 scenarios
```

## ⚡ Real-Time Performance

### FPGA Implementation (ZU48DR @ 250 MHz)

| Parameter | Value |
|-----------|-------|
| Latency | **0.76 μs** per update |
| Throughput | **1.3 MHz** max rate |
| DSP Usage | 2.3% (96 of 4272) |
| BRAM Usage | 2.2% (24 of 1080) |

**Verdict: 13,000× faster than 100 Hz requirement. Suitable for terminal guidance.**

### Smoother Latency Tradeoff

| Lag (samples) | Delay @50Hz | Accuracy Gain |
|---------------|-------------|---------------|
| 5 | 100 ms | +15% |
| 10 | 200 ms | +30% |
| 25 | 500 ms | +36% |

Recommendation: Use lag=25 for mid-course, reduce to lag=5 in terminal phase.

## 💰 Commercial Information

### Licensing Options

| License | Features | Price |
|---------|----------|-------|
| **Open Source** (AGPL v3) | Algorithm, benchmark | Free |
| **Enterprise** | RTL, support, custom features | Contact us |
| **OEM** | White-label, volume production | Contact us |

### Hardware BOM (1000 units)

| Component | Unit Cost |
|-----------|-----------|
| FPGA (XCZU48DR) | $3,395 |
| DDR4 + Power | $442 |
| RF Frontend | $123 |
| PCB + Assembly | $150 |
| **Total Hardware** | **$4,110** |

**Target sell price: $25,000/unit → 52% gross margin**

## 📄 Publications

*Patent pending:* "Multi-Model Interacting Multiple Model Tracking with Per-Model RTS Smoothing and Adaptive Process Noise"

Key citations:
- Bar-Shalom, Y., Li, X.R., "Estimation with Applications to Tracking and Navigation"
- Blackman, S., Popoli, R., "Design and Analysis of Modern Tracking Systems"

## 📞 Contact

**Nexellum d.o.o.**
- Email: mladen@nexellum.com
- Phone: +385 99 737 5100
- GitHub: [@mladen1312](https://github.com/mladen1312)

---

## Reproducibility Statement

This benchmark is designed to be **100% reproducible**. Running with the same seed will produce identical results:

```bash
python3 benchmarks/fair_benchmark.py --runs 50 --seed 42
```

All random number generation uses NumPy with fixed seeds. No external data dependencies.

---

**© 2026 Nexellum d.o.o. All rights reserved.**

Open source components licensed under AGPL v3. Enterprise features available under commercial license.
