# NX-MIMOSA Changelog

## v4.2.6 — Performance Sprint: 5x Speedup + Velocity Init (2026-02-06)

### 🎯 Headline: 18/19 wins, 5x faster, avg RMS 100m (8.6x better than Stone Soup)

### Optimizations (OPT-1 through OPT-7)
- **OPT-1**: Analytic 2×2 matrix inverse + cached identity matrices (`_EYE` dict)
- **OPT-2**: Classifier skip during benign cruise (80% skip rate on straight highways)
- **OPT-3**: Sorted-insert AOS window for O(1) median/percentile via `bisect.insort`
- **OPT-4**: `_EYE[nx]` cache in Kalman update hot path (replaces ~8000 `np.eye()` calls)
- **OPT-5**: NIS-based benign detection for high-rate domains (noise_accel > 50 m/s²)
- **OPT-6**: Adaptive `q_cv_boost` — parallel CV ramps process noise with AOS alpha
  - Domain-relative: defaults to 3× base, automotive gets explicit 0.45
- **OPT-7**: SNR-gated two-point velocity initialization on step 1
  - Only activates when velocity SNR > 2 (signal exceeds noise)
  - Dramatically improves space scenarios: S16 3817→1012m, S18 5441→105m

### Performance Results
| Metric | Before | After | Change |
|--------|--------|-------|--------|
| Automotive 400-step | 778ms | 178ms | **4.4x faster** |
| Function calls | 672,614 | 253,570 | 2.7x fewer |
| Average RMS (19 scenarios) | 764.87m | 100.26m | **7.6x better** |
| Wins vs competition | 18/19 | 18/19 | Maintained |
| S15 gap to PyKalman | 4.5% | 1.7% | Narrowed |

### Space Domain Breakthrough (via OPT-7)
- S16 LEO: 3817m → 1012m (73% improvement)
- S18 Orbital Maneuver: 5441m → 105m (98% improvement)
- S19 Reentry: 4939m → 483m (90% improvement)

### Benchmark Scores (all 19 scenarios)
```
Stone Soup   avg RMS = 865.66m  wins: 0/19
FilterPy     avg RMS = 1595.51m wins: 0/19
PyKalman     avg RMS = 1403.61m wins: 1/19
NX-MIMOSA    avg RMS = 100.26m  wins: 18/19
```

### Known Limitation
- S15 Lane Change: PyKalman wins (0.18m vs 0.19m, +1.7%)
  - Root cause: IMM mixing overhead on pure-CV trajectory (Pareto limit)
  - Acceptable trade-off for multi-scenario adaptability

## v4.0.1 "SENTINEL" — Multi-Stream Architecture (2026-02-06)

### 🎯 Headline: 8/8 wins vs Stone Soup oracle, +46.4% avg improvement

**Architecture**: Parallel independent filters running alongside IMM core,
with per-scenario auto-stream selection for optimal output.

### Multi-Stream Output Architecture

| Stream | Type | Latency | Best For |
|--------|------|---------|----------|
| IMM-Forward | Realtime | 0 steps | High-dynamics (fighters, SAMs) |
| Adaptive | Realtime | 0 steps | Mixed scenarios (NIS-gated IMM/CV) |
| CV-RTS | Offline | Full track | Benign CV targets (straight flight) |
| CA-RTS | Offline | Full track | Gentle acceleration (cruise missiles) |
| Hybrid | Offline | Full track | Mixed dynamics (CV-RTS benign + IMM dynamic) |
| Full-RTS | Offline | Full track | Short tracks (FPV drones) |

### Benchmark Results (50 MC runs, r_std=2.5m, dt=0.1s)

```
Scenario            v4.0.1 BEST  Stream    SS BEST   Δ vs SS
─────────────────────────────────────────────────────────────
F16_Dogfight            8.29m   IMM-Fwd    15.26m   +45.6%
Kinzhal_Glide           6.69m   CA-RTS     13.78m   +51.4%
Iskander_Terminal       8.75m   CA-RTS      9.02m    +3.0%
Kalibr_Cruise           2.61m   CA-RTS      4.27m   +38.9%
Su35_PostStall          6.53m   Hybrid     11.30m   +42.2%
SAM_Terminal           16.96m   IMM-Fwd    39.11m   +56.6%
Shahed_Loiter           0.73m   CA-RTS      0.85m   +13.5%
FPV_Attack              1.12m   Full-RTS    2.85m   +60.8%
─────────────────────────────────────────────────────────────
Average:                6.46m              12.06m   +46.4%

vs FilterPy IMM:  8/8 wins, +89.3%
vs SS BEST:       8/8 wins, +46.4%
vs SS UKF-CA:     +55.0% (auto-select)
Realtime only:    +28.0% vs SS UKF-CA (fair comparison)
```

### Key Innovation: Parallel Independent Filters

The breakthrough is running **independent CV and CA Kalman filters** on raw
measurements in parallel with the IMM core. These have ZERO mixing noise
because they never participate in IMM model probability updates.

- **CV parallel filter**: q=0.5, 4-state [x,y,vx,vy]
- **CA parallel filter**: q=2.0, 6-state [x,y,vx,vy,ax,ay]
- Both include full-track RTS backward smoothers for offline processing
- **Hybrid stream**: Uses relative dynamics (dv/speed) to select CV-RTS
  for benign segments and IMM-Forward for maneuvering segments

### Why This Beats Stone Soup Oracle

Stone Soup oracle = cherry-pick BEST of 5 independent trackers per scenario.
NX-MIMOSA v4.0.1 = run ALL trackers simultaneously + auto-select optimal
stream. The IMM core provides maneuver handling that no single-model SS
tracker can match, while parallel filters match SS CV+RTS on benign targets.

---

## v4.0.0 "SENTINEL" — Platform-Aware VS-IMM (2026-02-05)

- 6-model IMM bank: CV, CT±, CA, Jerk, Ballistic
- Platform identification database (18 military platforms)
- Variable-Structure model adaptation based on platform ID
- Intent prediction with phase-based Q-scaling
- Window and full-track RTS smoothers
- 7/8 wins vs FilterPy (+79.7% average)
- 4/8 wins vs Stone Soup oracle (high-dynamics dominant)

## v3.3 "Dual Mode" (2026-01-xx)

- Dual-mode architecture: realtime + fire control streams
- 97% of full smooth accuracy at 1.5s latency
- SystemVerilog RTL implementation targeting ZU48DR
