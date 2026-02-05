# NX-MIMOSA

## Multi-Domain Radar Tracking System

[![Version](https://img.shields.io/badge/version-3.3.0-blue.svg)](https://github.com/mladen1312/nx-mimosa/releases)
[![License](https://img.shields.io/badge/license-AGPL%20v3-green.svg)](LICENSE)
[![Python](https://img.shields.io/badge/python-3.8%2B-blue.svg)](https://python.org)
[![Benchmark](https://img.shields.io/badge/benchmark-reproducible-green.svg)](#gold-standard-benchmark)

**NX-MIMOSA** (Nexellum Multi-Model Optimal State Approximation) is a production-grade radar tracking algorithm featuring:

- **3-Model IMM** (CV, CT+, CT-) with Variable-Structure TPM
- **Dual-Mode Output** (v3.3): Real-time + Delayed Refined + Offline Smooth
- **Per-Model RTS Smoother** with proper backward pass
- **Adaptive Process Noise** via NIS-based Q scaling
- **FPGA-Ready** SystemVerilog RTL for ZU48DR RFSoC

### Commercial Use

**Contact mladen@nexellum.com for licensing and exemptions.**

---

## NEW: v3.3 Dual-Mode Architecture

v3.3 provides **three output streams** for different operational needs:

```
                    REAL-TIME PATH
Measurements ──────►│ Forward IMM │──────► x_realtime (0 ms latency)
        │           └─────────────┘
        │                 │
        │           ┌─────▼─────┐
        │           │  N-Step   │
        └──────────►│  Buffer   │
                    └─────┬─────┘
                          │
                    ┌─────▼─────┐
                    │ Sliding   │──────► x_refined (N×dt latency)
                    │ Window    │
                    │ Smooth    │
                    └───────────┘
                          │
                    ┌─────▼─────┐
                    │ Full RTS  │──────► x_offline (full track)
                    └───────────┘
```

### Performance (Dogfight BFM scenario)

| Output Stream | Latency | RMSE | Use Case |
|---------------|---------|------|----------|
| `x_realtime` | 0 ms | 8.66 m | Display, situational awareness |
| `x_refined` (N=30) | 1500 ms | **4.22 m** | Fire control, weapon guidance |
| `x_offline` | Full track | 4.18 m | Post-mission analysis |

**Window-30 achieves 99.7% of full smooth accuracy at 1.5s latency!**

---

## Gold Standard Benchmark (v3.1)

### Methodology

Honest, reproducible comparison against gold standard libraries:

- **[Stone Soup](https://github.com/dstl/Stone-Soup)** (UK DSTL)
- **[FilterPy](https://github.com/rlabbe/filterpy)** (Roger Labbe)

### Results (50 MC runs, seed=42)

| Scenario | StoneSoup | FilterPy | NX-MIMOSA v3.1 | Winner |
|----------|:---------:|:--------:|:--------------:|:------:|
| Missile Terminal (M4, 9g) | 13.54 m | 40.20 m | **9.67 m** | **v3.1** |
| Hypersonic Glide (M5, 2g) | 46.77 m | 80.47 m | **28.60 m** | **v3.1** |
| SAM Engagement (300m/s, 6g) | **5.03 m** | 17.25 m | 5.47 m | StoneSoup |
| Dogfight BFM (250m/s, 8g) | **4.18 m** | 14.69 m | 4.43 m | StoneSoup |
| Cruise Missile (250m/s, 3g) | 21.33 m | 33.09 m | **12.35 m** | **v3.1** |
| Ballistic Reentry (M7) | **65.54 m** | 198.92 m | 73.96 m | StoneSoup |
| UAV Swarm (50m/s, 2g) | **2.68 m** | 11.03 m | 3.42 m | StoneSoup |
| Stealth Aircraft (200m/s) | 42.08 m | 86.79 m | **28.74 m** | **v3.1** |
| **GRAND AVERAGE** | **25.14 m** | **60.30 m** | **20.83 m** | **v3.1** |

**Win Rate: 4/8 scenarios** | **+17% better than Stone Soup on average**

### Reproduce

```bash
pip install numpy scipy filterpy stonesoup
python benchmarks/benchmark_vs_goldstandard.py --runs 50 --seed 42
```

---

## Quick Start

```bash
git clone https://github.com/mladen1312/nx-mimosa.git
cd nx-mimosa
pip install -e .

# v3.3 dual-mode example
from python.nx_mimosa_v33_dual_mode import NxMimosaV33Dual

tracker = NxMimosaV33Dual(dt=0.05, sigma_pos=10, sigma_vel=5, window_size=30)

for measurement in measurements:
    x_realtime = tracker.update(measurement)  # real-time output
    
x_refined = tracker.get_window_smoothed_estimates(30)  # delayed refined
x_offline = tracker.get_smoothed_estimates()           # full smooth
```

---

## Supported Platforms

| Board | Price | ADC | Best For |
|-------|-------|-----|----------|
| **RFSoC 4x2** | **$2,499** | 4x 5GSPS | Development |
| **ZCU208** | $13,194 | 8x 5GSPS | Production |

---

## License

AGPL v3 — see [LICENSE](LICENSE)

**Commercial licensing available.** Contact: mladen@nexellum.com | +385 99 737 5100

(c) 2024-2026 Nexellum d.o.o. All rights reserved.
