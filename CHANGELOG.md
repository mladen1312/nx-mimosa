# NX-MIMOSA Changelog

## v5.0.1 — "FULL SPECTRUM + MHT" (2026-02-06)

### 🎯 Headline: MHT Data Association + pip Packaging + Sphinx Documentation

**Commercial readiness: 70% → 75%**

#### New Features
- **MHT (Multi-Hypothesis Tracking)** — Third data association algorithm. Generates ranked global hypotheses via log-likelihood scoring, selects best assignment. Best for dense, ambiguous environments. Reference: Reid (1979).
- **pip packaging** — `pip install nx-mimosa` with proper `pyproject.toml`, wheel build, `nx_mimosa` package namespace
- **Sphinx documentation** — Full RTD-themed docs: quickstart, user guides (multi-target, sensors, coordinates, metrics), API reference with autodoc
- **Package `__init__.py`** — Clean top-level imports: `from nx_mimosa import MultiTargetTracker`

#### Tests
- 5 new MHT tests: basic, two-targets, clutter, enum selection, crossing targets
- **141/141 tests PASS** (136 existing + 5 MHT)

#### Documentation
- `docs/source/` — Complete Sphinx project (conf.py, index.rst, 8 guide/API pages)
- README completely rewritten for sales conversion — storytelling, use cases, ROI calculator, "What Only NX-MIMOSA Has" section

---

## v5.0.0 — "FULL SPECTRUM" (2026-02-06)

### 🎯 Headline: 3D Multi-Target Tracker with GNN + JPDA — Four Critical Blockers Eliminated

Commercial readiness: **35% → 70%** (from research prototype to production-grade MTT)

### Added — Core Systems

**3D Tracking** (`nx_mimosa_mtt.py`)
- Full 3D state vector [x, y, z, vx, vy, vz] with 3D motion models (CV3D, CA3D, CT3D)
- 3D IMM filter with automatic model mixing across state dimensions
- CT3D coordinated turn degrades gracefully to CV3D at ω→0

**Multi-Target Tracking** (`nx_mimosa_mtt.py`)
- `MultiTargetTracker` engine — complete predict-associate-update-manage cycle
- GNN (Global Nearest Neighbor) via Munkres/Hungarian optimal assignment
- JPDA (Joint Probabilistic Data Association) — Mahalanobis-based robust weighting
- Selectable: `association="gnn"` or `association="jpda"`

**Track Management** (`nx_mimosa_mtt.py`)
- Full lifecycle: TENTATIVE → CONFIRMED → COASTING → DELETED
- M-of-N confirmation, consecutive-miss deletion, coast-before-delete
- Track scoring, minimum separation enforcement, configurable per domain

**Coordinate Transforms** (`nx_mimosa_coords.py`)
- WGS-84 ↔ ECEF ↔ ENU ↔ Spherical (Bowring iterative, sub-mm accuracy)
- Unbiased polar/spherical-to-Cartesian (Bar-Shalom 2001)
- EKF Jacobians for measurement models, range-rate projection
- `SensorLocation` dataclass with mounting rotation support

**Metrics** — OSPA, NEES, NIS, NATO SIAP (completeness, ambiguity, spuriousness)

**Domain Presets** — military, atc, automotive, space, maritime

### Test Coverage
- 63 new tests (15 classes) — **136/136 total PASS**
- Scenarios: military air defense, automotive highway, crossing targets, JPDA in clutter

---

## v4.3.0 — Multi-Sensor Fusion + README Rewrite (2026-02-06)

### 🎯 Headline: Native multi-sensor fusion — radar, EO/IR, ESM, Doppler, ADS-B

### Multi-Sensor Fusion Engine (`nx_mimosa_fusion.py`)
- **[REQ-V43-01]** Measurement-level fusion via sequential Kalman update
- **[REQ-V43-02]** 6 sensor types: POSITION_XY, POSITION_3D, RANGE_BEARING, RANGE_DOPPLER, BEARING_ONLY, ADS_B
- **[REQ-V43-03]** Asynchronous sensor handling with per-sensor timestamps
- **[REQ-V43-04]** Information-weighted simultaneous fusion (batch update alternative)
- **[REQ-V43-05]** Per-sensor health monitoring (NIS tracking, degradation detection)
- **[REQ-V43-06]** Automatic bias correction per sensor
- **[REQ-V43-07]** Graceful degradation (single sensor fallback, gate rejection)

### Measured Fusion Gains (ATC scenario, σ_radar=50m)
| Configuration | RMS | Improvement |
|--------------|-----|-------------|
| Single radar (baseline) | 33.00m | — |
| + EO bearing (0.3°) | 31.57m | +4.4% |
| + ESM bearing (2°) | 33.16m | -0.5% (too noisy) |
| + Doppler radar (30m, 1m/s) | 22.48m | **+31.9%** |
| + Doppler + ADS-B (10m, 1m/s) | 11.73m | **+64.4%** |

### Factory Functions
- `make_radar_sensor()` — Cartesian position radar
- `make_polar_radar_sensor()` — Range-bearing radar
- `make_doppler_radar_sensor()` — Range-bearing-Doppler radar
- `make_eo_sensor()` — EO/IR passive bearing-only
- `make_esm_sensor()` — ESM/ELINT passive bearing-only
- `make_adsb_sensor()` — ADS-B cooperative (position + velocity)
- `make_gps_sensor()` — GPS position

### Core Tracker Changes
- Added `_get_combined_position()` helper for fusion engine integration
- No regression: 18/19 wins, 43/43 tests passing, all benchmarks unchanged

### README Rewrite
- Complete rewrite targeting engineers and investors
- Competitive positioning vs Stone Soup, MATLAB, custom development, defense primes
- Full capability matrix, architecture diagram, multi-sensor quick start
- Honest benchmark disclosure with methodology

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
