# NX-MIMOSA

## Multi-Domain Radar Tracking System

[![Version](https://img.shields.io/badge/version-3.1.0-blue.svg)](https://github.com/mladen1312/nx-mimosa/releases)
[![License](https://img.shields.io/badge/license-AGPL%20v3-green.svg)](LICENSE)
[![Python](https://img.shields.io/badge/python-3.8%2B-blue.svg)](https://python.org)
[![Tests](https://img.shields.io/badge/tests-70%20passed-brightgreen.svg)](#testing)
[![Benchmark](https://img.shields.io/badge/benchmark-reproducible-green.svg)](#gold-standard-benchmark)

**NX-MIMOSA** (Nexellum Multi-Model Optimal State Approximation) is a production-grade radar tracking algorithm featuring:

- **3-Model IMM** (CV, CT+, CT-) with Variable-Structure TPM
- **True Per-Model RTS Smoother** with critical bugfix for offline processing
- **Adaptive Process Noise** via NIS-based Q scaling
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
- Monte Carlo: 50 runs per scenario, seed=42
- No cherry-picking: ALL scenarios and ALL algorithms reported

### Results (v3.1, 50 MC runs, seed=42)

| Scenario | StoneSoup EKF+RTS | FilterPy IMM | NX-MIMOSA v3.1 | Winner |
|----------|:-----------------:|:------------:|:--------------:|:------:|
| Missile Terminal (Mach 4, 9g) | 13.54 m | 40.20 m | **9.67 m** | **v3.1** |
| Hypersonic Glide (Mach 5, 2g) | 46.77 m | 80.47 m | **28.60 m** | **v3.1** |
| SAM Engagement (300m/s, 6g) | **5.03 m** | 17.25 m | 5.47 m | StoneSoup |
| Dogfight BFM (250m/s, 8g) | **4.18 m** | 14.69 m | 4.43 m | StoneSoup |
| Cruise Missile (250m/s, 3g) | 21.33 m | 33.09 m | **12.35 m** | **v3.1** |
| Ballistic Reentry (Mach 7) | **65.54 m** | 198.92 m | 73.96 m | StoneSoup |
| UAV Swarm (50m/s, 2g) | **2.68 m** | 11.03 m | 3.42 m | StoneSoup |
| Stealth Aircraft (200m/s, 4g) | 42.08 m | 86.79 m | **28.74 m** | **v3.1** |
| **GRAND AVERAGE** | **25.14 m** | **60.30 m** | **20.83 m** | **v3.1** |

**Win Rate: 4/8 scenarios** | **+17% better than Stone Soup on average** | **+65% better than FilterPy IMM**

### Analysis

NX-MIMOSA wins with **larger margins (29-42%)** on scenarios with distinct mode transitions (missile evasion, S-weave, pop-up, sparse data). Stone Soup wins with **smaller margins (6-22%)** on scenarios where a high-Q CV model with RTS smoothing suffices (sustained turns, ballistic, moderate maneuvers).

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
- NX-MIMOSA v3.1 (fwd): **38.46 m** vs FilterPy IMM: 60.30 m (**+36% better**)

### Reproduce

```bash
pip install numpy scipy filterpy stonesoup
python benchmarks/benchmark_vs_goldstandard.py --runs 50 --seed 42
```

---

## Algorithm Details

### v3.1 Architecture

```
                     +-------------+
Measurements ------->|  IMM Mixer  |
                     +------+------+
                            |
              +-------------+-------------+
              v             v             v
         +--------+   +--------+   +--------+
         |  CV    |   |  CT+   |   |  CT-   |
         |(q=q0)  |   |(w=+0.1)|   |(w=-0.1)|
         +---+----+   +---+----+   +---+----+
             |             |             |
             |    Adaptive Q (NIS)       |
             |    Joseph Form Update     |
              v             v             v
         +----------------------------------+
         |   Mode Probability Update        |
         |   VS-TPM (confidence-based)      |
         +----------------+-----------------+
                          |
                 +--------v--------+
                 |  Per-Model RTS  |
                 |    Smoother     |
                 +--------+--------+
                          |
                 +--------v--------+
                 |  Mode-Weighted  |
                 |  Combination    |
                 +-----------------+
```

### Key Innovation

Standard IMM smoothers smooth the combined state, mixing incompatible dynamics. NX-MIMOSA's **per-model RTS** smooths each model independently:

```
For each model j:
    G[j] = P_filt[j] @ F[j].T @ inv(P_pred[j])
    x_smooth[j] = x_filt[j] + G[j] @ (x_smooth[k+1] - x_pred[k+1])

Combined: x_smooth = sum( mu[j] * x_smooth[j] )
```

### v3.2c Adaptive Omega — Attempted and Rejected

Attempted to adapt CT model turn rates online. Result: regression on 6/8 scenarios (+1.6% average). Root cause: IMM mode probability weighting already compensates for model mismatch; omega adaptation injects noise that destabilizes the smoother. See `benchmarks/benchmark_v32c_adaptive_omega.py`.

---

## Supported Platforms

| Board | Price | ADC | DAC | Best For |
|-------|-------|-----|-----|----------|
| **RFSoC 4x2** | **$2,499** | 4x 5GSPS | 2x 9.85GSPS | Development, PYNQ |
| **ZCU208** | $13,194 | 8x 5GSPS | 8x 10GSPS | Production, 8-ch |

Both boards use **XCZU48DR** Gen 3 RFSoC — same RTL, different build targets.

---

## Quick Start

```bash
git clone https://github.com/mladen1312/nx-mimosa.git
cd nx-mimosa
pip install -e .
pytest tests/ -v
python benchmarks/benchmark_vs_goldstandard.py
```

---

## License

AGPL v3 — see [LICENSE](LICENSE)

**Commercial licensing available.** Contact: mladen@nexellum.com | +385 99 737 5100

(c) 2024-2026 Nexellum d.o.o. All rights reserved.
