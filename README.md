# NX-MIMOSA

## Multi-Domain Radar Tracking System

[![Version](https://img.shields.io/badge/version-3.2.0-blue.svg)](https://github.com/mladen1312/nx-mimosa/releases)
[![License](https://img.shields.io/badge/license-AGPL%20v3-green.svg)](LICENSE)
[![Python](https://img.shields.io/badge/python-3.8%2B-blue.svg)](https://python.org)
[![Tests](https://img.shields.io/badge/tests-70%20passed-brightgreen.svg)](#testing)
[![Benchmark](https://img.shields.io/badge/benchmark-reproducible-green.svg)](#gold-standard-benchmark)

**NX-MIMOSA** (Nexellum Multi-Model Optimal State Approximation) is a production-grade radar tracking algorithm featuring:

- **4-Model IMM** (CV, CT+, CT-, CV-HIGH-Q) with Variable-Structure TPM
- **True Per-Model RTS Smoother** with covariance-weighted combination
- **Adaptive Process Noise** via NIS exponential moving average
- **Joseph Form** covariance update for numerical stability
- **FPGA-Ready** SystemVerilog RTL for ZU48DR RFSoC

### Commercial Use

**Contact mladen@nexellum.com for licensing and exemptions.**

---

## Gold Standard Benchmark

### Methodology

Honest, reproducible comparison against the tracking community's gold standards:

- **[Stone Soup](https://github.com/dstl/Stone-Soup)** (UK DSTL) — Production-grade tracking framework
- **[FilterPy](https://github.com/rlabbe/filterpy)** (Roger Labbe) — Most widely used Python Kalman library

**Fairness Guarantees:**
- All algorithms receive **identical** noisy measurements (same RNG seed)
- All algorithms use **identical** initial state and covariance
- FilterPy IMM uses the **same 3-model structure** (CV/CT+/CT-) as NX-MIMOSA
- Monte Carlo: 30 runs per scenario, seed=42
- No cherry-picking: ALL scenarios and ALL algorithms reported

### Results (v3.2, 30 MC runs, seed=42)

| Scenario | StoneSoup EKF+RTS | FilterPy IMM | NX-MIMOSA v3.2 | Winner |
|----------|:-----------------:|:------------:|:--------------:|:------:|
| Missile Terminal (Mach 4, 9g) | 13.50 m | 40.27 m | **8.80 m** | **v3.2** |
| Hypersonic Glide (Mach 5, 2g) | 46.86 m | 80.28 m | **22.56 m** | **v3.2** |
| SAM Engagement (300m/s, 6g) | **4.97 m** | 17.28 m | 5.66 m | StoneSoup |
| Dogfight BFM (250m/s, 8g) | 4.17 m | 14.76 m | **3.70 m** | **v3.2** |
| Cruise Missile (250m/s, 3g) | 21.36 m | 33.01 m | **9.31 m** | **v3.2** |
| Ballistic Reentry (Mach 7) | 65.70 m | 198.41 m | **61.64 m** | **v3.2** |
| UAV Swarm (50m/s, 2g) | 2.69 m | 11.01 m | **2.61 m** | **v3.2** |
| Stealth Aircraft (200m/s, 4g) | 42.12 m | 86.91 m | **22.35 m** | **v3.2** |
| **GRAND AVERAGE** | **25.17 m** | **60.24 m** | **17.08 m** | **v3.2** |

**Win Rate: 7/8 scenarios** | **+32% better than Stone Soup** | **+72% better than FilterPy IMM**

### Honest Disclosure

**Known advantages of NX-MIMOSA (disclosed for fairness):**
- IMM (multi-model) inherently outperforms single-model filters on maneuvers
- Smoothing uses future data — not available in real-time applications
- Adaptive Q and VS-TPM provide additional tuning beyond standard IMM

**Known limitations:**
- Stone Soup has no built-in IMM — comparison is CV-smoother vs IMM-smoother
- FilterPy IMM is forward-only — no smoother available for direct comparison
- NX-MIMOSA smoother requires offline/batch processing (not causal)

**Fair forward-only comparison (real-time capable):**
- NX-MIMOSA v3.2 (fwd): **24.72 m** vs FilterPy IMM: 60.24 m (**+59% better**)

### Reproduce

```bash
# Install dependencies
pip install numpy scipy filterpy stonesoup

# Run benchmark
python benchmarks/benchmark_vs_goldstandard.py --runs 30 --seed 42

# Run v3.2 comparison
python benchmarks/benchmark_v32_upgrade.py
```

---

## Algorithm Details

### v3.2 Innovations

1. **4-Model IMM**: Added CV-HIGH-Q model (5× base process noise) as "catch-all" for unknown maneuver types, inspired by Singer maneuvering target model
2. **Aggressive Adaptive Q**: NIS-based scaling up to 25× (was 10×) with exponential moving average for stability
3. **NIS-aware TPM**: Transition probability matrix biases toward high-Q model when innovation statistics indicate model mismatch
4. **Covariance-weighted Smoother**: Combines smoothed per-model estimates using `mode_prob / trace(P)` weighting instead of pure mode probabilities

### Architecture

```
                     ┌─────────────┐
Measurements ──────►│  IMM Mixer   │
                     └──────┬──────┘
                            │
              ┌─────────────┼─────────────┬─────────────┐
              ▼             ▼             ▼             ▼
         ┌────────┐   ┌────────┐   ┌────────┐   ┌────────────┐
         │  CV    │   │  CT+   │   │  CT-   │   │ CV-HIGH-Q  │
         │(q=q₀) │   │(ω=+0.1)│   │(ω=-0.1)│   │ (q=5×q₀)  │
         └───┬────┘   └───┬────┘   └───┬────┘   └─────┬──────┘
             │             │             │              │
             │    Adaptive Q (NIS EMA)   │              │
             │    Joseph Form Update     │              │
              ▼             ▼             ▼             ▼
         ┌────────────────────────────────────────────────┐
         │           Mode Probability Update               │
         │         VS-TPM (NIS-aware biasing)              │
         └───────────────────┬────────────────────────────┘
                             │
                    ┌────────▼────────┐
                    │  Per-Model RTS  │
                    │    Smoother     │
                    └────────┬────────┘
                             │
                    ┌────────▼────────┐
                    │   Cov-Weighted  │
                    │   Combination   │
                    └─────────────────┘
```

---

## Supported Platforms

| Board | Price | ADC | DAC | Best For |
|-------|-------|-----|-----|----------|
| **RFSoC 4x2** | **$2,499** | 4× 5GSPS | 2× 9.85GSPS | Development, PYNQ |
| **ZCU208** | $13,194 | 8× 5GSPS | 8× 10GSPS | Production, 8-ch |

Both boards use **XCZU48DR** Gen 3 RFSoC — same RTL, different build targets.

---

## Quick Start

```bash
git clone https://github.com/mladen1312/nx-mimosa.git
cd nx-mimosa
pip install -e .

# Run tests
pytest tests/ -v

# Run benchmark
python benchmarks/benchmark_vs_goldstandard.py
```

---

## License

AGPL v3 — see [LICENSE](LICENSE)

**Commercial licensing available.** Contact: mladen@nexellum.com | +385 99 737 5100

© 2024-2026 Nexellum d.o.o. All rights reserved.
