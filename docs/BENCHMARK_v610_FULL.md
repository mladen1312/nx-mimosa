# NX-MIMOSA v6.1.0 — Full-Scale C++ Benchmark

**Date:** 2026-02-07 18:53 UTC  
**Engine:** C++ `_nx_core` (Eigen3 + OpenMP + KDTree + Bertsekas Auction)  
**IMM:** 4-model (CV/CA/CT+/CT−) per track  
**ECCM:** Python ML-CFAR overlay (6-feature classifier, not in timing loop)  
**Platform:** x86_64, 4 cores  

## Summary

| Metric | 1,000 AC | 2,000 AC | 5,000 AC |
|--------|----------|----------|----------|
| Mean scan (ms) | 9.4 | 18.9 | 53.2 |
| P95 scan (ms) | 11.2 | 20.9 | 65.9 |
| Max scan (ms) | 13.9 | 25.6 | 66.2 |
| µs / target | 9.4 | 9.4 | 10.6 |
| GOSPA (m) | 34445 | 51939 | 80102 |
| RMS error (m) | 377 | 362 | 386 |
| Detection rate | 99.9% | 99.9% | 99.9% |
| Final confirmed | 1,024 | 2,059 | 5,139 |
| ECCM Pd | 65.7% | 61.0% | 50.3% |

## v6.0.1 → v6.1.0 Capability Delta

| Capability | v6.0.1 | v6.1.0 | QEDMMA Source |
|-----------|--------|--------|---------------|
| ECM classification | 4 binary detectors | 6-feature ML + 4 environment classes | ml_cfar_engine.sv |
| Features extracted | NIS, range-rate, jitter | power ratio, guard ratio, kurtosis, lag-1 corr, range/Doppler deriv | ml_cfar_engine.sv |
| R-matrix adaptation | None | Auto 1×–10× by environment | integration_controller.sv |
| Gate adaptation | Fixed 4σ | Dynamic 3σ–8σ | integration_controller.sv |
| Coast adaptation | Fixed | Dynamic 8–20 scans | integration_controller.sv |
| Bearing-only mode | No | Auto in heavy jamming | integration_controller.sv |
| Jammer localization | No | TDOA (Chan-Ho + Gauss-Newton + KF) | jammer_localizer.sv |
| Integration levels | 1 | 4 (NORMAL/ELEVATED/HIGH/MAXIMUM) | integration_controller.sv |
| Classes | 71 | 78 | — |
| LOC | 11,493 | 12,980 | — |
| Tests | 352 | 397 | — |

## Scaling Analysis

T ∝ N^α:  1K→2K α=1.01  |  2K→5K α=1.13  |  1K→5K α=1.08

Average α = 1.08 → sub-quadratic (KDTree + sparse auction effective).
Projected 10K targets: ~112 ms/scan.

## 1,000 Aircraft Detail

Mix: {'commercial': 838, 'ga': 107, 'military': 34, 'helicopter': 21}  |  Jammed: 50

```
  Scan  1:     9.1 ms  ████████████████████████      0 trk
  Scan  2:    10.5 ms  ████████████████████████████      0 trk
  Scan  3:    14.7 ms  ████████████████████████████████████████    998 trk
  Scan  4:     8.5 ms  ███████████████████████   1000 trk
  Scan  5:     8.9 ms  ████████████████████████   1000 trk
  Scan  6:     8.5 ms  ███████████████████████   1000 trk
  Scan  7:     8.5 ms  ███████████████████████   1000 trk
  Scan  8:     8.7 ms  ███████████████████████   1000 trk
  Scan  9:     9.5 ms  █████████████████████████   1005 trk
  Scan 10:    13.9 ms  █████████████████████████████████████   1005 trk
  Scan 11:     8.8 ms  ███████████████████████   1013 trk
  Scan 12:     8.9 ms  ████████████████████████   1011 trk
  Scan 13:     8.9 ms  ████████████████████████   1017 trk
  Scan 14:     8.8 ms  ████████████████████████   1023 trk
  Scan 15:     9.7 ms  ██████████████████████████   1025 trk
  Scan 16:     9.6 ms  ██████████████████████████   1021 trk
  Scan 17:    10.5 ms  ████████████████████████████   1023 trk
  Scan 18:     9.4 ms  █████████████████████████   1022 trk
  Scan 19:     9.3 ms  █████████████████████████   1020 trk
  Scan 20:     9.1 ms  ████████████████████████   1024 trk
```

## 2,000 Aircraft Detail

Mix: {'commercial': 1648, 'ga': 217, 'military': 87, 'helicopter': 48}  |  Jammed: 100

```
  Scan  1:    15.0 ms  ███████████████████████      0 trk
  Scan  2:    17.8 ms  ███████████████████████████      0 trk
  Scan  3:    17.3 ms  ██████████████████████████   1999 trk
  Scan  4:    17.8 ms  ███████████████████████████   2000 trk
  Scan  5:    25.6 ms  ████████████████████████████████████████   2000 trk
  Scan  6:    19.4 ms  ██████████████████████████████   2002 trk
  Scan  7:    17.9 ms  ███████████████████████████   2002 trk
  Scan  8:    17.0 ms  ██████████████████████████   2002 trk
  Scan  9:    19.7 ms  ██████████████████████████████   2004 trk
  Scan 10:    17.8 ms  ███████████████████████████   2002 trk
  Scan 11:    18.2 ms  ████████████████████████████   2033 trk
  Scan 12:    18.4 ms  ████████████████████████████   2030 trk
  Scan 13:    18.1 ms  ████████████████████████████   2042 trk
  Scan 14:    18.2 ms  ████████████████████████████   2059 trk
  Scan 15:    18.9 ms  █████████████████████████████   2057 trk
  Scan 16:    19.5 ms  ██████████████████████████████   2059 trk
  Scan 17:    18.0 ms  ████████████████████████████   2058 trk
  Scan 18:    18.9 ms  █████████████████████████████   2058 trk
  Scan 19:    19.3 ms  ██████████████████████████████   2060 trk
  Scan 20:    18.4 ms  ████████████████████████████   2059 trk
```

## 5,000 Aircraft Detail

Mix: {'commercial': 4121, 'ga': 545, 'military': 211, 'helicopter': 123}  |  Jammed: 250

```
  Scan  1:    39.8 ms  ████████████████████████      0 trk
  Scan  2:    49.9 ms  ██████████████████████████████      0 trk
  Scan  3:    44.1 ms  ██████████████████████████   4998 trk
  Scan  4:    44.1 ms  ██████████████████████████   5001 trk
  Scan  5:    45.3 ms  ███████████████████████████   5001 trk
  Scan  6:    46.9 ms  ████████████████████████████   5007 trk
  Scan  7:    45.6 ms  ███████████████████████████   5008 trk
  Scan  8:    44.3 ms  ██████████████████████████   5015 trk
  Scan  9:    65.8 ms  ███████████████████████████████████████   5008 trk
  Scan 10:    49.6 ms  █████████████████████████████   5007 trk
  Scan 11:    56.3 ms  ██████████████████████████████████   5060 trk
  Scan 12:    51.0 ms  ██████████████████████████████   5041 trk
  Scan 13:    55.6 ms  █████████████████████████████████   5078 trk
  Scan 14:    58.3 ms  ███████████████████████████████████   5107 trk
  Scan 15:    58.2 ms  ███████████████████████████████████   5112 trk
  Scan 16:    53.8 ms  ████████████████████████████████   5133 trk
  Scan 17:    52.8 ms  ███████████████████████████████   5142 trk
  Scan 18:    53.1 ms  ████████████████████████████████   5134 trk
  Scan 19:    66.2 ms  ████████████████████████████████████████   5141 trk
  Scan 20:    57.8 ms  ██████████████████████████████████   5139 trk
```

## Methodology

- **Tracking engine**: C++ `_nx_core` — Eigen3, OpenMP, KDTree O(N log N) gating,
  Bertsekas sparse auction O(N·k) assignment, 4-model IMM per track
- **ECCM analysis**: Python overlay (NOT included in timing) — ML-CFAR 6-feature
  environment classification + adaptive parameter recommendation
- **ECM injection**: 5% targets jammed from scan 8 (NOISE 8×, RGPO 300m/s, DRFM ghost)
- **GOSPA**: c=10 km, p=2, greedy assignment | **Seed**: 42

---
*NX-MIMOSA v6.1.0 · Copyright © 2026 Nexellum d.o.o.*