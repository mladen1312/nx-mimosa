# NX-MIMOSA v3.1 — Gold Standard Benchmark Report

**Date:** 2026-02-05  
**Author:** Dr. Mladen Mešter / Nexellum d.o.o.  
**License:** AGPL v3  

## Executive Summary

NX-MIMOSA v3.1 was benchmarked against the tracking community's gold standard libraries:

- **Stone Soup 1.8** (UK DSTL) — EKF + RTS Smoother
- **FilterPy 1.4.5** (Roger Labbe) — IMMEstimator, 3-model

**Result: NX-MIMOSA v3.1 outperforms both on grand average.**

| Algorithm | Grand Avg RMSE | vs MIMOSA |
|-----------|---------------|-----------|
| **NX-MIMOSA v3.1 (smooth)** | **20.83 m** | **baseline** |
| Stone Soup EKF+RTS | 25.14 m | +17.2% worse |
| EKF-CA (5×Q) | 38.30 m | +83.9% worse |
| NX-MIMOSA v3.1 (fwd) | 38.46 m | +84.7% worse |
| FilterPy IMM | 60.30 m | +189.6% worse |

## Per-Scenario Results (50 MC runs, seed=42)

| Scenario | MIMOSA v3.1 | Stone Soup | Gap | Winner |
|----------|------------|------------|-----|--------|
| Missile Terminal (M4, 9g) | **9.67 m** | 13.54 m | +28.6% | MIMOSA |
| Hypersonic Glide (M5, 2g) | **28.60 m** | 46.77 m | +38.8% | MIMOSA |
| SAM Engagement (300m/s, 6g) | 5.47 m | **5.03 m** | -8.1% | Stone Soup |
| Dogfight BFM (250m/s, 8g) | 4.43 m | **4.18 m** | -5.6% | Stone Soup |
| Cruise Missile (250m/s, 3g) | **12.35 m** | 21.33 m | +42.1% | MIMOSA |
| Ballistic Reentry (M7) | 73.96 m | **65.54 m** | -11.4% | Stone Soup |
| UAV Swarm (50m/s, 2g) | 3.42 m | **2.68 m** | -21.6% | Stone Soup |
| Stealth Aircraft (200m/s, 4g) | **28.74 m** | 42.08 m | +31.7% | MIMOSA |

**Win distribution: MIMOSA 4/8, Stone Soup 4/8**

Key pattern:
- MIMOSA wins with **larger margins (29-42%)** on scenarios with distinct mode transitions
- Stone Soup wins with **smaller margins (6-22%)** on scenarios where high-Q CV + RTS suffices

## v3.2c Adaptive Omega — REJECTED

Attempted fix: adapt CT model omega online from state velocity cross-product.  
**Result: regression on 6/8 scenarios (+1.6% average).** Rejected.

Root cause: IMM's mode probability weighting already compensates for model mismatch.  
Adding omega adaptation injects noise into the model bank, destabilizing the RTS smoother.

**Lesson: Do not micro-manage the IMM framework. It IS the adaptive mechanism.**

## Fairness Declaration

- All algorithms receive identical noisy measurements (same RNG seed)
- All algorithms use identical initial state and covariance
- FilterPy IMM uses same 3-model structure (CV/CT±0.1) with same TPM
- Stone Soup uses their production Kalman + RTS smoother
- 50 Monte Carlo runs per scenario, all reported (no cherry-picking)
- Fully reproducible: `pip install numpy scipy filterpy stonesoup`

### Known advantages of NX-MIMOSA (disclosed):
- IMM (multi-model) inherently outperforms single-model on mode transitions
- Per-model RTS smoother uses future data (offline, not causal)
- Adaptive Q + VS-TPM tune to innovation statistics

### Known limitations (disclosed):
- Stone Soup has no built-in IMM — comparison is CV-smoother vs IMM-smoother
- FilterPy IMM is forward-only — no smoother available
- MIMOSA smoother is offline (not real-time)
- For real-time: compare MIMOSA (fwd) vs competitors

## Reproduce

```bash
pip install numpy scipy filterpy stonesoup
python benchmark_vs_goldstandard.py --runs 50 --seed 42
```
