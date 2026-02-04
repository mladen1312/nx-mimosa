# NX-MIMOSA — Advanced Multi-Model Tracking Algorithm

[![License: AGPL v3](https://img.shields.io/badge/License-AGPL%20v3-blue.svg)](https://www.gnu.org/licenses/agpl-3.0)
[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![Benchmark](https://img.shields.io/badge/benchmark-reproducible-green.svg)](#fair-benchmark)

**NX-MIMOSA** (Nexellum Multi-Model Optimal State Approximation) is a state-of-the-art target tracking algorithm designed for defense applications including missile guidance, air defense, and fire control systems.

## 🎯 Key Features

- **Interacting Multiple Model (IMM)** filter with CV/CT+/CT- motion models
- **Variable-Structure IMM (VS-IMM)** with dynamic transition probability matrix
- **Adaptive Process Noise** via NIS-based Q-scaling
- **True IMM Smoother** with critical bugfix for offline processing
- **Joseph Form** covariance update for numerical stability

## 📊 Performance Summary

**8/8 scenario wins** against all compared algorithms:

| Algorithm | Avg RMSE | vs MIMOSA-v3 |
|-----------|----------|--------------|
| EKF-CV | 93.31 m | +96.7% worse |
| Alpha-Beta | 15.56 m | +80.0% worse |
| EKF-CA | 7.51 m | +58.6% worse |
| IMM-Forward | 5.62 m | +44.8% worse |
| MIMOSA-v2 | 5.90 m | +47.3% worse |
| **MIMOSA-v3** | **3.11 m** | **baseline** |

## 🚀 Quick Start

```bash
# Clone repository
git clone https://github.com/mladen1312/nx-mimosa.git
cd nx-mimosa

# Run benchmark (requires only NumPy!)
python benchmark.py

# Custom settings
python benchmark.py --runs 100 --seed 12345
```

## 📈 Fair Benchmark

The benchmark is designed to be **fair and reproducible**:

- **Same measurements**: All algorithms receive identical noisy observations
- **Same initialization**: All algorithms start with same initial state estimate
- **Reproducible**: Fixed random seed ensures identical results across runs
- **Monte Carlo**: 30 runs per scenario for statistical significance
- **8 Defense Scenarios**:
  - Missile Terminal Guidance (Mach 4, 9g)
  - Hypersonic Glide Vehicle (Mach 5, 2g)
  - SAM Engagement (300 m/s, 6g)
  - Dogfight BFM (250 m/s, 8g)
  - Cruise Missile (250 m/s, 3g)
  - Ballistic Reentry (Mach 7, 1g)
  - UAV Swarm (50 m/s, 2g)
  - Stealth Aircraft (200 m/s, 4g, sparse 2Hz)

### Verify Results Yourself

```bash
python benchmark.py --runs 30 --seed 42
```

Expected output:
```
==========================================================================================
NX-MIMOSA FAIR BENCHMARK v1.1 — Reproducible Comparison
==========================================================================================
Settings: 30 Monte Carlo runs, seed=42
Algorithms: EKF-CV, EKF-CA, Alpha-Beta, IMM-Fwd, MIMOSA-v2, MIMOSA-v3

====================================================================================================
RESULTS — Position RMSE (meters), lower is better
====================================================================================================
Scenario                             EKF-CV     EKF-CA Alpha-Beta    IMM-Fwd  MIMOSA-v2  MIMOSA-v3    Winner
----------------------------------------------------------------------------------------------------
Missile Terminal (M4, 9g)            117.19       1.95       5.33       1.63       1.70       0.72    MIMOSA-v3
Hypersonic Glide (M5, 2g)             18.19       3.01      10.68       2.68       2.85       1.44    MIMOSA-v3
SAM Engagement (6g)                   50.77       4.78       8.59       3.21       3.39       1.55    MIMOSA-v3
Dogfight BFM (8g)                     70.44       1.34       3.15       1.03       1.15       0.51    MIMOSA-v3
Cruise Missile (3g)                   67.00       7.47      15.75       6.01       6.20       3.02    MIMOSA-v3
Ballistic Reentry (M7)               166.47      19.10      52.68      14.58      15.50       7.94    MIMOSA-v3
UAV Swarm (2g)                        15.74       1.27       2.15       0.94       1.01       0.50    MIMOSA-v3
Stealth Aircraft (4g)                240.66      21.15      26.17      14.91      15.36       9.16    MIMOSA-v3
----------------------------------------------------------------------------------------------------
AVERAGE                               93.31       7.51      15.56       5.62       5.90       3.11
====================================================================================================

SUMMARY:
  EKF-CV      :   93.31m avg, 0/8 wins, MIMOSA-v3 +96.7% better
  EKF-CA      :    7.51m avg, 0/8 wins, MIMOSA-v3 +58.6% better
  Alpha-Beta  :   15.56m avg, 0/8 wins, MIMOSA-v3 +80.0% better
  IMM-Fwd     :    5.62m avg, 0/8 wins, MIMOSA-v3 +44.8% better
  MIMOSA-v2   :    5.90m avg, 0/8 wins, MIMOSA-v3 +47.3% better
  MIMOSA-v3   :    3.11m avg, 8/8 wins <-- BEST
```

## 🔬 Algorithm Details

### v3.1 Innovations

1. **True IMM Smoother Bugfix**: The backward pass now correctly uses `F @ x_mixed` instead of `F @ x_filt`, maintaining mathematical consistency with the RTS smoother derivation.

2. **Variable-Structure TPM**: Transition probabilities adapt based on mode confidence:
   - High CV confidence (>90%): p_stay = 0.98
   - Medium (70-90%): p_stay = 0.95  
   - Low (<70%): p_stay = 0.90

3. **NIS-Based Adaptive Q**: Process noise scales based on Normalized Innovation Squared:
   - NIS > χ²(2, 0.95): Increase Q by 30%
   - NIS < χ²(2, 0.95)/2: Decrease Q by 2%

### Motion Models

- **CV (Constant Velocity)**: For straight-line flight
- **CT+ (Coordinated Turn, positive)**: For right turns
- **CT- (Coordinated Turn, negative)**: For left turns

Turn rate ω is computed from scenario parameters: `ω = max_g × 9.81 / speed`

## 📁 Repository Structure

```
nx-mimosa/
├── README.md           # This file
├── benchmark.py        # Fair, reproducible benchmark (single file, NumPy only)
├── LICENSE             # AGPL v3
└── docs/
    └── BENCHMARK_REPORT.md
```

## 🔧 Requirements

**Minimal**: Python 3.8+ and NumPy only!

```bash
pip install numpy
```

## 📜 License

This project is licensed under **AGPL v3** for open-source use.

**Commercial License**: For proprietary applications or to avoid AGPL requirements, contact us for commercial licensing.

## 💼 Enterprise Edition

The Enterprise Edition includes:
- Real-time FPGA implementation (0.76 μs latency @ 250 MHz)
- Multi-target tracking with track management
- Integration support for Xilinx RFSoC platforms (ZU48DR, ZU28DR)
- Comprehensive test suite and documentation
- Technical support and maintenance

**Pricing**: Contact for quote

## 📚 References

1. Blom, H.A.P. & Bar-Shalom, Y. (1988). "The interacting multiple model algorithm for systems with Markovian switching coefficients." IEEE TAC.
2. Li, X.R. & Jilkov, V.P. (2005). "Survey of maneuvering target tracking. Part V: Multiple-model methods." IEEE TAES.
3. Rauch, H.E., Tung, F., & Striebel, C.T. (1965). "Maximum likelihood estimates of linear dynamic systems." AIAA Journal.

## 🏢 About Nexellum

**Nexellum d.o.o.** develops advanced radar systems and tracking algorithms for defense applications.

- **Website**: https://nexellum.com
- **Email**: mladen@nexellum.com
- **Phone**: +385 99 737 5100
- **GitHub**: https://github.com/mladen1312

---

*"Precision tracking for a safer world."*
