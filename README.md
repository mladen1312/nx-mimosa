# NX-MIMOSA v5.9.3

**Multi-target tracker with platform intelligence for defence radar systems.**

11,493 lines across 9 modules. 62 unique classes. 340 tests. Drop into any radar pipeline and get: 8 filter types (KF/EKF/UKF/IMM/PF/GM-PHD/CPHD/LMB), three association engines (GNN/JPDA/MHT), 6-sensor fusion engine, automatic ECM detection (5 types), military aircraft identification (30+ forces), platform classification (111 types), intent prediction (16 behaviours), coordinate transforms (geodetic/ENU/ECEF/polar), dataset adapters (nuScenes/CARLA/RADIATE), dual-mode NATO outputs (ASTERIX Cat048/Link-16), and synthetic scenario generation — with zero mandatory dependencies beyond NumPy and SciPy.

The core tracker is a single 4,236-line Python file you can deploy by copying. The full library adds intelligence, fusion, coordinates, and dataset modules.

## Why NX-MIMOSA

**761 aircraft tracked simultaneously from live ADS-B data.** OpenSky Network, Central Europe, February 2026. Detection rate 99.8%. Mean scan processing 519 ms. Five military jets autonomously identified across four NATO air forces.

That test ran on a laptop. The data and scripts are in this repository. Run it yourself.

**18 of 19 benchmark scenarios won** against Stone Soup v1.9, FilterPy, and PyKalman at seed = 42. The one loss (S15 lane change) is by 1 cm. Full benchmark table below. We show every number, including the loss.

## Architecture

NX-MIMOSA is a *tracking product*, not a toolkit. Where Stone Soup gives you building blocks to assemble your own tracker, NX-MIMOSA gives you a complete, tested pipeline that works out of the box. Both approaches have value — pick the one that matches your integration timeline.

**Filtering pipeline:**
- **6-model IMM** (Interacting Multiple Model): Constant Velocity, Constant Acceleration, Coordinated Turn, Jerk, Ballistic, Orbital — running in parallel per target with automatic mode switching via Markov transition matrix
- **4 Kalman variants**: Standard KF, Extended KF for polar measurements, Unscented KF for polar measurements, and Particle Filter (SIR with systematic resampling) for non-Gaussian scenarios
- **RFS filters**: GM-PHD (Vo & Ma 2006) and CPHD (Vo, Vo & Cantoni 2007) for unknown/variable target count without explicit data association
- **3 association engines**: Global Nearest Neighbour (GNN) for speed, Joint Probabilistic Data Association (JPDA) for dense clutter, Multiple Hypothesis Tracking (MHT) with N-scan pruning for crossing targets

**Intelligence layer:**
- **Platform identification**: 111 aircraft types classified from kinematics alone — no IFF transponder needed. 66.5% fine-grain accuracy, 99.8% top-3
- **Military identification**: ICAO24 hex database + 35 callsign regex patterns. NATO, USAF, RAF, Luftwaffe, French AF, Polish AF, and 25+ more forces identified automatically
- **ECM detection**: Automatic NIS-based anomaly detection for 5 ECM types — DRFM/repeater jamming, RGPO (range gate pull-off), noise jamming, chaff discrimination, and ghost track identification. Returns per-track ECM classification with confidence scores and recommended countermeasure actions
- **Intent prediction**: 16 tactical behaviours classified (holding, intercept, terrain following, SAM evasion, aerial refuelling, etc.)

**Sensor fusion:**
- **6 measurement types**: Primary radar, Doppler radar, EO/IR, ESM, ADS-B, secondary radar — each with appropriate measurement model
- **Track-to-track association**: Statistical distance-based Hungarian assignment for multi-radar fusion (Bar-Shalom & Chen, 2004). Fuse tracks from different sensors via covariance intersection
- **Out-of-sequence measurements**: Bar-Shalom one-step-lag retrodiction handles late-arriving data from distributed sensors without reprocessing
- **Online sensor bias estimation**: Detects and corrects systematic range/bearing/elevation biases from individual sensors

**Operational features:**
- **Dual-mode output**: Real-time forward estimates (zero latency for displays) + sliding window smoothed estimates (1.5 s latency for fire control)
- **Track quality scoring**: Automated assessment of each track's reliability based on update rate, innovation consistency, covariance condition, and age
- **Track coasting**: Intelligent extrapolation during measurement dropouts with exponentially decaying confidence
- **ASTERIX Cat048 + Link-16 J3.2**: Standard NATO output formats
- **5 domain presets**: Air surveillance, maritime, ground, space, ballistic — each with tuned process noise, gating, and M/N confirmation logic
- **Clutter resilience**: Gate-level filtering + association-level scoring. Tested up to 50 false alarms per scan

**Coordinate transforms** (`nx_mimosa_coords.py`):
- Geodetic (WGS-84) ↔ ECEF ↔ ENU ↔ Cartesian ↔ Polar (range/azimuth/elevation)
- Radar-centric and ground-truth alignment utilities
- 27 functions covering all standard conversions for multi-sensor integration

**Dataset adapters** (`nx_mimosa_datasets.py`):
- **nuScenes**: Autonomous driving radar+lidar dataset (1,000 scenes, 1.4M 3D annotations)
- **CARLA**: Synthetic autonomous driving simulator output
- **RADIATE**: All-weather radar dataset from Heriot-Watt University
- **GenericCSV**: Any timestamped CSV with position columns
- **SyntheticScenarioGenerator**: Configurable multi-target scenarios with programmable maneuvers, ECM, and clutter for Monte Carlo testing

**Module structure** (9 modules, 11,493 total lines):

| Module | Lines | Purpose |
|--------|-------|---------|
| `nx_mimosa_mtt.py` | 4,236 | Core tracker — filters, association, metrics, RFS |
| `nx_mimosa_intent_classifier.py` | 1,212 | Improved platform + intent + ECM classifier pipeline |
| `nx_mimosa_intelligence.py` | 675 | Intelligence pipeline (classification → intent → threat) |
| `nx_mimosa_coords.py` | 643 | Coordinate transforms (WGS-84/ECEF/ENU/polar) |
| `nx_mimosa_fusion.py` | 616 | Multi-sensor fusion engine (6 sensor types) |
| `nx_mimosa_datasets.py` | 595 | Dataset adapters (nuScenes/CARLA/RADIATE/CSV) |
| `nx_mimosa_v40_sentinel.py` | 2,688 | Sentinel pipeline (tracker + intelligence integrated) |
| `nx_mimosa_v33_dual_mode.py` | 466 | Dual-mode output (display + fire control) |
| `nx_mimosa_v33_driver.py` | 362 | FPGA register driver (SystemVerilog bridge) |

## Live Benchmark: 761 Aircraft

Tested against the OpenSky Network ADS-B feed covering Central Europe (43–55°N, 5–30°E), February 2026. Eight consecutive radar scans processed. No synthetic data — every measurement is a real aircraft reporting its position via ADS-B.

**Scalability results:**

| Metric | Value |
|--------|-------|
| Peak simultaneous targets | **761** |
| Mean targets per scan | 753 |
| Confirmed tracks | 751 |
| Detection rate | **99.8%** |
| Mean scan processing time | **519 ms** |
| P95 scan processing time | 864 ms |
| Max scan processing time | 974 ms |
| Total time (8 scans) | 4.36 s |

Real-time capable for any surveillance radar with rotation period ≥ 2 s. The scan time is dominated by the 761×761 assignment matrix — the KDTree spatial pre-gate (O(n log n)) reduces the dense cost matrix to a sparse one, and SciPy's C-optimized LAPJV solver handles the assignment in ~200 ms at this scale.

**Military aircraft autonomously identified:**

| ICAO24 | Callsign | Force | Altitude | Speed | KF RMS | Improvement |
|--------|----------|-------|----------|-------|--------|-------------|
| 4784c2 | NSZ21U | NATO | FL340 | 443 kt | 224 m | 1.11× |
| ae123a | RCH4539 | USAF AMC | FL360 | 398 kt | 252 m | 1.06× |
| 3b776f | CTM2004 | French AF | FL305 | 397 kt | 187 m | 1.17× |
| 48d960 | PLF105 | Polish AF | FL182 | 384 kt | 214 m | 1.29× |
| 479227 | NSZ3YT | NATO | FL162 | 338 kt | 161 m | 1.24× |
| | | **MEAN** | | | **207 m** | **1.17×** |

Identification uses two methods: ICAO24 hex range lookup (military blocks per ICAO Annex 10) and callsign regex matching against 35 patterns covering 30+ air forces. No IFF transponder required.

Improvement measured against constant-velocity baseline. The Cramér-Rao lower bound at σ = 150 m gives a theoretical maximum around 1.4×. Our 1.17× on cruise flight is consistent with published IMM results (1.2–1.3× in literature). Maneuvering targets show larger gains.

**GOSPA metric decomposition (full test):**

| Component | Value | Interpretation |
|-----------|-------|----------------|
| Total GOSPA | 35,262 m | Combined multi-target error |
| Localisation | 8,374 m | Position accuracy of matched tracks |
| Missed targets | 6,243 m | Penalty for untracked aircraft |
| False tracks | 31,072 m | Penalty for tracks without measurements |

The false track component dominates because GOSPA penalises coasting tracks during initialisation — with 761 targets and a 3-scan M/N confirmation window, this is expected. Localisation error per matched track is 11.1 m, confirming filter convergence.

Full benchmark report with methodology and reproduction instructions: [`docs/BENCHMARK_v591_500plus.md`](docs/BENCHMARK_v591_500plus.md)

## 19-Scenario Benchmark

All scenarios use seed = 42. RMS position error in metres. Lower is better.

| # | Scenario | Stone Soup | FilterPy | PyKalman | NX-MIMOSA |
|---|----------|-----------|----------|----------|-----------|
| S01 | ATC Enroute | 69.98 | 96.71 | 109.75 | **32.20** ★ |
| S02 | Holding Pattern | 72.23 | 56.50 | 77.18 | **55.56** ★ |
| S03 | Approach | 51.21 | 66.54 | 78.35 | **20.62** ★ |
| S04 | Go-Around | 72.94 | 69.97 | 83.79 | **28.53** ★ |
| S05 | Departure SID | 89.78 | 110.22 | 127.35 | **48.97** ★ |
| S06 | Racing | 23.74 | 31.15 | 33.96 | **16.55** ★ |
| S07 | TCAS RA | 24.79 | 31.22 | 33.50 | **11.48** ★ |
| S08 | Fighter Intercept | 23.60 | 13.86 | 23.78 | **7.79** ★ |
| S09 | SAM Engagement | 99.91 | 45.86 | 100.04 | **11.37** ★ |
| S10 | Cruise Missile | 7.39 | 6.89 | 7.58 | **3.88** ★ |
| S11 | Ballistic Missile | 86.33 | 166.30 | 119.97 | **51.04** ★ |
| S12 | UAV Loiter | 14.73 | 20.43 | 22.52 | **8.63** ★ |
| S13 | Helicopter NOE | 22.46 | 31.38 | 33.52 | **20.65** ★ |
| S14 | Ship Transit | 50.19 | 73.98 | 78.62 | **33.57** ★ |
| S15 | Lane Change | 0.18 | 0.18 | **0.18** | 0.19 |
| S16 | LEO Satellite | 4194.55 | 6450.92 | 6916.74 | **1011.93** ★ |
| S17 | Ground Vehicle | 4.21 | 5.81 | 6.32 | **2.81** ★ |
| S18 | Orbital Manoeuvre | 5670.49 | 8655.95 | 9863.84 | **105.14** ★ |
| S19 | Atmospheric Reentry | 6077.77 | 14724.27 | 9297.91 | **483.48** ★ |
| | **AVERAGE** | 865.66 | 1595.51 | 1403.61 | **100.26** |

**Score: NX-MIMOSA 18 — Competitors 1** (PyKalman wins S15 by 1 cm).

**Fair context on maneuvering scenarios.** Stone Soup, FilterPy, and PyKalman use single-model Kalman filters in these benchmarks. NX-MIMOSA uses a 6-model IMM. Any well-tuned IMM would outperform single-model KFs on high-maneuver scenarios (S09, S11, S16, S18, S19) — that is the fundamental value of IMM, not a unique NX-MIMOSA advantage. The meaningful comparison is on cruise-flight scenarios (S01–S06, S12–S14, S17) where improvement is 1.1–2.2×, the physically correct result. We plan to publish IMM-versus-IMM comparisons in a future release.

## Competitive Comparison

|  | NX-MIMOSA v5.9.3 | Stone Soup v1.9 | FilterPy | MATLAB SFT |
|--|------------------|----------------|----------|------------|
| Multi-model IMM | ✓ 6 models | Composable | ✗ | ✓ |
| EKF / UKF | ✓ both (polar) | ✓ both | ✓ EKF | ✓ both |
| Particle filter | ✓ SIR | ✓ (extensive) | ✗ | ✓ |
| GNN association | ✓ | ✓ | ✗ | ✓ |
| JPDA | ✓ | ✓ | ✗ | ✓ |
| MHT | ✓ N-scan | ✓ (extensive) | ✗ | ✓ |
| Multi-sensor fusion | ✓ T2TA + CI | ✓ feeders | ✗ | ✓ |
| OOSM handling | ✓ Bar-Shalom | ✗ | ✗ | ✗ |
| Sensor bias estimation | ✓ online | ✗ | ✗ | ✗ |
| Platform identification | ✓ 111 types | ✗ | ✗ | ✗ |
| Military ICAO ID | ✓ auto 30+ forces | ✗ | ✗ | ✗ |
| ECM detection | ✓ auto (5 types) | ✗ | ✗ | ✗ |
| Intent prediction | ✓ 16 behaviours | ✗ | ✗ | ✗ |
| Track coasting | ✓ with confidence | ✗ | ✗ | ✓ |
| Track quality scoring | ✓ auto | ✗ | ✗ | ✓ |
| ASTERIX / Link-16 | ✓ | ✗ | ✗ | ✗ |
| GOSPA metrics | ✓ decomposed | ✓ | ✗ | ✓ |
| **PHD / CPHD / LMB** | ✓ GM-PHD + CPHD + LMB | ✓ (+ GLMB) | ✗ | ✓ |
| Coordinate transforms | ✓ WGS84/ECEF/ENU/polar | ✓ | ✗ | ✓ |
| Dataset adapters | ✓ nuScenes/CARLA/RADIATE | ✗ | ✗ | ✗ |
| Synthetic scenario gen | ✓ Monte Carlo ready | ✓ | ✗ | ✓ |
| Dual-mode output | ✓ display + fire control | ✗ | ✗ | ✗ |
| FPGA register bridge | ✓ SystemVerilog driver | ✗ | ✗ | ✓ via HDL Coder |
| Code generation C/HDL | ✗ (on roadmap) | ✗ | ✗ | ✓ |
| Certification path | ✗ (on roadmap) | ✗ | ✗ | ✓ via Coder |
| Live 761-target test | ✓ published | not published | N/A | not published |
| Deployment | single .py file | pip package | pip package | MATLAB runtime |
| Price | AGPL / $50K+ | Apache-2 (free) | MIT (free) | ~$5K/yr/seat |

### Where Stone Soup is stronger

Stone Soup is a *tracking framework* with a composable plugin architecture — you pick a filter, an associator, a deleter, and wire them together. This gives extreme flexibility and makes it the best choice for research and algorithm comparison. It offers particle filter variants beyond our bootstrap SIR (Extended, Auxiliary, Rao-Blackwellised), Generalized Labeled Multi-Bernoulli (GLMB) beyond our LMB, 50+ active contributors, and UK DSTL government backing. If you need to experiment with novel filter combinations or publish academic comparisons, Stone Soup is excellent.

NX-MIMOSA's advantage is that it ships a complete, integrated pipeline with 13 intelligence features Stone Soup doesn't provide (platform ID, military ID, ECM auto-detection, intent prediction, sensor bias estimation, OOSM handling, track coasting, track quality scoring, ASTERIX/Link-16 output, multi-sensor T2TA fusion, LMB with cardinality distribution, and dual-mode display/fire-control output) — and that you can deploy by copying a single file.

### Where MATLAB Sensor Fusion Toolbox is stronger

MATLAB SFT offers code generation to C, C++, and HDL through MATLAB Coder, a certification path (DO-254, DO-178C) through the MathWorks toolchain, beam scheduling for phased array control, and deep integration with the wider MATLAB/Simulink ecosystem. At ~$5K/year/seat it is the right choice for organisations already committed to MATLAB with certification requirements.

NX-MIMOSA's advantage is intelligence features MATLAB SFT does not offer (platform ID, military ID, ECM detection, intent prediction), open-source transparency under AGPL, and a price point that starts at zero.

### A note on RFS methods

NX-MIMOSA v5.9.3 includes GM-PHD (Vo & Ma 2006), CPHD (Vo, Vo & Cantoni 2007), and LMB (Reuter, Vo, Vo & Dietmayer 2014) filters for scenarios with unknown and variable target count. These handle target birth, death, and clutter without explicit data association. The LMB filter provides explicit track identity through labeled Bernoulli components with existence probabilities and a Poisson binomial cardinality distribution. For most radar applications with thresholded detections, the IMM+GNN/JPDA/MHT pipeline remains the recommended approach — it provides better single-target accuracy with lower computational cost.

Stone Soup additionally offers Generalized Labeled Multi-Bernoulli (GLMB) which extends LMB with joint prediction and update — useful for very dense scenarios. If your application specifically requires GLMB, consider Stone Soup.

## Quick Start

```python
from nx_mimosa_mtt import MultiTargetTracker
import numpy as np

# Create tracker (air surveillance defaults)
tracker = MultiTargetTracker(dt=5.0, r_std=150.0, domain='air')

# Feed measurements scan by scan
for scan in radar_scans:
    measurements = np.array(scan)  # shape (N, 3) — x, y, z in metres
    tracker.process_scan(measurements)
    
    for track in tracker.confirmed_tracks:
        print(f"Track {track.track_id}: {track.position}")
```

**With JPDA for dense clutter:**
```python
tracker = MultiTargetTracker(dt=5.0, r_std=150.0, association='jpda', domain='air')
```

**With MHT for crossing targets:**
```python
tracker = MultiTargetTracker(dt=5.0, r_std=150.0, association='mht', domain='air')
```

**With automatic ECM detection:**
```python
from nx_mimosa_mtt import MultiTargetTracker, ECMDetector

tracker = MultiTargetTracker(dt=5.0, r_std=150.0, domain='air')
ecm = ECMDetector()

for track in tracker.confirmed_tracks:
    result = ecm.update(
        track_id=track.track_id,
        nis=track.nis if hasattr(track, 'nis') else 1.0,
        position=track.position,
        velocity=track.velocity,
        was_hit=track.consecutive_misses == 0
    )
    if result['ecm_detected']:
        print(f"Track {track.track_id}: ECM {result['ecm_types']} → {result['recommended_action']}")
```

**Multi-sensor fusion:**
```python
from nx_mimosa_mtt import MultiTargetTracker, t2ta_associate, fuse_tracks

tracker_radar = MultiTargetTracker(dt=5.0, r_std=150.0)
tracker_eo = MultiTargetTracker(dt=1.0, r_std=10.0)

# Get tracks from each sensor, then fuse
tracks_r = [{'id': t.track_id, 'x': t.imm.combined_state()[0], 'P': t.imm.combined_state()[1]} 
            for t in tracker_radar.confirmed_tracks]
tracks_e = [{'id': t.track_id, 'x': t.imm.combined_state()[0], 'P': t.imm.combined_state()[1]} 
            for t in tracker_eo.confirmed_tracks]

pairs, unmatched_r, unmatched_e = t2ta_associate(tracks_r, tracks_e)
for pair in pairs:
    fused_state, fused_cov = fuse_tracks(
        {'x': pair.state_a, 'P': pair.cov_a},
        {'x': pair.state_b, 'P': pair.cov_b}
    )
```

## Metrics and Verification

NX-MIMOSA includes five metric families for tracking performance evaluation:

- **NEES** (Normalised Estimation Error Squared) — filter consistency check
- **NIS** (Normalised Innovation Squared) — measurement consistency, drives automatic ECM detection
- **SIAP** (Single Integrated Air Picture) — completeness, ambiguity, spuriousness, timeliness
- **OSPA** (Optimal Sub-Pattern Assignment) — combined localisation and cardinality error
- **GOSPA** (Generalised OSPA) — decomposed into localisation, missed targets, and false tracks (García-Fernández et al., 2020)

The test suite contains 340 tests covering filter convergence (KF/EKF/UKF/IMM/PF/GM-PHD/CPHD/LMB), association correctness, metric computation, ECM detection, track lifecycle, domain presets, and feature inventory.

## Roadmap

| Version | Feature | Target |
|---------|---------|--------|
| v5.9.2 | Particle filter, auto ECM detection, track coasting | ✓ Done |
| v5.9.3 | LMB filter, 62 classes, 11,493 LOC, 340 tests | ✓ Done |
| v5.9.4 | PyPI package (`pip install nx-mimosa`), API docs | Q1 2026 |
| v6.0 | C++ core — target <50 ms at 1,000 simultaneous tracks | Q1 2026 |
| v6.1 | FPGA proof-of-concept (Xilinx Versal AI Core) | Q2 2026 |
| v6.2 | Multi-radar track fusion service (distributed) | Q3 2026 |
| v7.0 | DO-254 certification path | Q4 2026 |

## Licensing

| Tier | Price | Includes |
|------|-------|----------|
| **Open Source** | Free | Full tracker engine, all algorithms, AGPL v3. Use freely, share derivatives. |
| **Lite** | $50,000 | MTT + IMM + coords. GOSPA metrics. Private modifications. 12 months support. |
| **Pro** | $150,000 | Everything in Lite + fusion, platform ID, ECM, intent, military ID, ASTERIX, Link-16. |
| **Enterprise** | $350,000 | Everything in Pro + FPGA SystemVerilog, DO-254 certification support, integration engineering. |

**Build vs. buy context**: 3–5 engineers × 12–18 months = $1.5M–$2.5M loaded cost to build an equivalent tracker in-house. That covers basic tracking only — before intelligence features.

## Known Limitations

1. **Python performance ceiling** — 519 ms per scan at 761 targets. Adequate for surveillance radar (4–12 s rotation). C++ core targeting <50 ms on roadmap (Q1 2026).
2. **No certification** — not yet qualified for DO-254 or DO-178C. Certification path planned for Q4 2026 via SystemVerilog FPGA implementation.
3. **Core tracker is a single file** — The 4,236-line `nx_mimosa_mtt.py` contains all filters, association, and metrics. Intelligence, fusion, and dataset modules are separate files. Deliberate for zero-dependency deployment: `scp` one file and you have a complete tracker. Full modular packaging available for teams that prefer it.
4. **GLMB not implemented** — GM-PHD, CPHD, and LMB cover most unknown target count scenarios. Generalized Labeled Multi-Bernoulli (GLMB) with joint prediction/update is not yet implemented — Stone Soup offers this for very dense environments.
5. **ECM detection is statistical** — NIS-based anomaly detection across 5 ECM types (DRFM, RGPO, noise, chaff, ghost). Requires 3–5 scan baseline. Cannot detect first-scan ECM onset.
6. **Benchmark context** — 19-scenario comparison uses single-model KF competitors. IMM-vs-IMM comparison forthcoming. Cruise-flight improvement (1.1–2.2×) is the honest metric.
7. **No PyPI package yet** — Install via `git clone` or file copy. `pip install nx-mimosa` targeting Q1 2026.
8. **API documentation** — Docstrings are comprehensive (149 docstrings, 72 REQ IDs), but generated Sphinx/mkdocs site not yet published.

## Contact

Dr. Mladen Mešter — Nexellum d.o.o., Zagreb, Croatia

mladen@nexellum.com · +385 99 737 5100

Every number in this document is reproducible. Run the benchmarks or ask us to demonstrate.
