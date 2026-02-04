# NX-MIMOSA v3.1

**Nexellum eXtended - Multiple IMM with Optimal Smoothing Algorithm**

[![License](https://img.shields.io/badge/License-AGPL%20v3-blue.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.8%2B-green.svg)](https://python.org)
[![Benchmark](https://img.shields.io/badge/Benchmark-8%2F8%20Wins-brightgreen.svg)](#benchmark-results)

> **State-of-the-art maneuvering target tracking for defense applications**

NX-MIMOSA is an advanced tracking algorithm that outperforms all standard approaches across defense-relevant scenarios. It combines Variable-Structure IMM, adaptive process noise, and a mathematically correct RTS smoother.

## 🏆 Benchmark Results

Run the benchmark yourself to verify:

```bash
pip install numpy
python benchmark.py --runs 30 --seed 42
```

### Position RMSE (meters) — Lower is Better

| Scenario | EKF-CV | EKF-CA | α-β-γ | IMM-Fwd | MIMOSA-v2 | **MIMOSA-v3** |
|:---------|-------:|-------:|------:|--------:|----------:|--------------:|
| Missile Terminal (M4, 9g) | 165.96 | 2.72 | 7.71 | 2.33 | 2.45 | **1.03** |
| Hypersonic Glide (M5, 2g) | 25.68 | 4.25 | 15.10 | 4.07 | 4.45 | **2.42** |
| SAM Engagement (6g) | 71.85 | 6.69 | 12.29 | 4.69 | 5.05 | **2.47** |
| Dogfight BFM (8g) | 99.65 | 1.92 | 4.56 | 1.48 | 1.63 | **0.70** |
| Cruise Missile (3g) | 94.75 | 10.45 | 22.80 | 8.49 | 8.80 | **4.45** |
| Ballistic Reentry (M7) | 236.11 | 26.63 | 75.08 | 26.24 | 27.20 | **17.03** |
| UAV Swarm (2g) | 22.24 | 1.79 | 3.03 | 1.31 | 1.41 | **0.72** |
| Stealth Aircraft (4g) | 340.62 | 28.56 | 33.53 | 20.64 | 21.28 | **12.34** |
| **AVERAGE** | 132.11 | 10.38 | 21.76 | 8.65 | 9.03 | **5.14** |

### Summary

| Algorithm | Win Rate | Improvement vs MIMOSA-v3 |
|:----------|:--------:|-------------------------:|
| **MIMOSA-v3** | **8/8** | — |
| IMM-Forward | 0/8 | +40.6% |
| MIMOSA-v2 | 0/8 | +43.1% |
| EKF-CA | 0/8 | +50.4% |
| α-β-γ | 0/8 | +76.4% |
| EKF-CV | 0/8 | +96.1% |

## ✨ Key Features

### Variable-Structure IMM (VS-IMM)
Dynamic transition probability matrix that adapts based on mode confidence:
- High CV confidence (μ > 0.9): p_stay = 0.98 (stable tracking)
- Medium confidence: p_stay = 0.95 (balanced)
- Low confidence: p_stay = 0.90 (fast mode switching)

### NIS-Based Adaptive Process Noise
Real-time process noise scaling using Normalized Innovation Squared:
```python
if NIS > χ²(0.95, 2):  # Filter is overconfident
    q_scale *= 1.3    # Increase process noise
elif NIS < χ²(0.95, 2) / 2:  # Filter is underconfident
    q_scale *= 0.98   # Decrease process noise
```

### True IMM RTS Smoother (BUGFIX)
The critical insight that makes v3.1 work:

```python
# ❌ WRONG (causes divergence):
x_pred = F @ x_filt[k]

# ✅ CORRECT (stable):
x_pred = F @ x_mixed[k]  # Use the state that was ACTUALLY predicted from
```

This bugfix maintains mathematical consistency with the RTS smoother derivation and provides +43% improvement over forward-only filtering.

### Joseph Form Covariance Update
Numerically stable covariance update:
```python
I_KH = np.eye(4) - K @ H
P = I_KH @ P_pred @ I_KH.T + K @ R @ K.T
```

## 🚀 Quick Start

### Installation

```bash
git clone https://github.com/mladen1312/nx-mimosa.git
cd nx-mimosa
pip install numpy
```

### Basic Usage

```python
from nx_mimosa import NX_MIMOSA_V3

# Initialize tracker
tracker = NX_MIMOSA_V3(dt=0.02, sigma=5.0, omega=0.15)
tracker.initialize(z_initial, v_initial)

# Forward pass (real-time)
for measurement in measurements:
    estimate = tracker.update(measurement)
    # Use estimate for display/control

# Offline smoothing (batch processing)
smoothed_estimates = tracker.smooth()
```

### Run Benchmark

```bash
# Default settings (30 runs, seed=42)
python benchmark.py

# More statistical significance
python benchmark.py --runs 100

# Different seed for verification
python benchmark.py --seed 12345
```

## 📊 Scenarios Tested

| Scenario | Speed | Max G | Update Rate | Noise σ |
|:---------|------:|------:|------------:|--------:|
| Missile Terminal | Mach 4 | 9g | 50 Hz | 5 m |
| Hypersonic Glide | Mach 5 | 2g | 50 Hz | 10 m |
| SAM Engagement | 300 m/s | 6g | 20 Hz | 8 m |
| Dogfight BFM | 250 m/s | 8g | 50 Hz | 3 m |
| Cruise Missile | 250 m/s | 3g | 10 Hz | 15 m |
| Ballistic Reentry | Mach 7 | 1g | 10 Hz | 50 m |
| UAV Swarm | 50 m/s | 2g | 10 Hz | 2 m |
| Stealth Aircraft | 200 m/s | 4g | 2 Hz | 25 m |

## ⚡ Real-Time Performance

### FPGA Implementation (ZU48DR @ 250 MHz)

| Metric | Value |
|:-------|------:|
| Latency | 0.76 μs |
| Throughput | 1.3 MHz |
| DSP48E2 | 96 (2.25%) |
| BRAM36 | 24 (2.22%) |
| LUT | 15,000 (3.53%) |

### Timing Budget

| Phase | Requirement | FPGA Latency | Margin |
|:------|------------:|-------------:|-------:|
| Search (20 Hz) | 50 ms | 0.76 μs | 65,000× |
| Track (50 Hz) | 20 ms | 0.76 μs | 26,000× |
| Terminal (100 Hz) | 10 ms | 0.76 μs | 13,000× |
| Extreme (1 kHz) | 1 ms | 0.76 μs | 1,300× |

## 📁 Repository Structure

```
nx-mimosa/
├── README.md           # This file
├── LICENSE             # AGPL v3.0
├── benchmark.py        # Fair, reproducible benchmark
├── nx_mimosa/
│   ├── __init__.py
│   ├── tracker.py      # NX-MIMOSA v3.1 implementation
│   ├── models.py       # Motion models (CV, CT)
│   └── utils.py        # Helper functions
├── tests/
│   └── test_tracker.py # Unit tests
└── docs/
    ├── BENCHMARK_REPORT.md
    └── ALGORITHM.md    # Mathematical derivation
```

## 📜 License

**Open Source (AGPL v3.0):**
- Free for research and evaluation
- Modifications must be open-sourced
- Commercial use requires license

**Enterprise License:**
- Full source code
- No AGPL obligations
- Priority support
- Custom integration assistance

Contact: mladen@nexellum.com

## 🎯 Applications

- **Missile Seekers**: Sub-meter accuracy for terminal guidance
- **Air Defense Radars**: Superior track quality for engagement
- **Fire Control Systems**: Real-time tracking for weapon systems
- **Counter-UAS**: Handles erratic drone maneuvers
- **Space Surveillance**: Long-range tracking with sparse updates

## 📚 References

1. Bar-Shalom, Y., Li, X.R., Kirubarajan, T. (2001). *Estimation with Applications to Tracking and Navigation*
2. Blom, H.A.P., Bar-Shalom, Y. (1988). "The Interacting Multiple Model Algorithm for Systems with Markovian Switching Coefficients"
3. Rauch, H.E., Tung, F., Striebel, C.T. (1965). "Maximum Likelihood Estimates of Linear Dynamic Systems"

## 📧 Contact

**Nexellum d.o.o.**
- Email: mladen@nexellum.com
- Phone: +385 99 737 5100
- GitHub: [@mladen1312](https://github.com/mladen1312)

---

*Built with ❤️ in Croatia for defense applications worldwide*
