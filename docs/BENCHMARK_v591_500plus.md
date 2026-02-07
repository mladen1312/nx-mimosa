# NX-MIMOSA v5.9.1 — 500+ Target Real-Data Benchmark Report

## Executive Summary

NX-MIMOSA Multi-Target Tracker successfully tracked **761 simultaneous aircraft** from live OpenSky ADS-B data over Central Europe, processing each scan in **519ms mean** (real-time capable). Five military jets were autonomously identified and tracked alongside 756 civil aircraft with **99.8% detection rate**.

---

## Test Configuration

| Parameter | Value |
|-----------|-------|
| Data Source | OpenSky Network (live ADS-B) |
| Region | Central Europe (43–55°N, 5–30°E) |
| Snapshots | 8 scans |
| Aircraft per scan | 748–761 (mean 753) |
| Measurement noise | σ = 150 m (simulated L-band radar) |
| Association | GNN + KDTree pre-gate + scipy LAPJV |
| Filter | IMM (CV3D + CA3D + CT3D) per track |

---

## Results: Large-Scale MTT

### Scalability

| Metric | Value |
|--------|-------|
| Max simultaneous targets | **761** |
| Mean simultaneous targets | 753 |
| Final confirmed tracks | 751 |
| Total tracks created | 791 |
| Detection rate | **99.8%** |

### Timing (per scan)

| Metric | Value |
|--------|-------|
| Mean | **519 ms** |
| P95 | 864 ms |
| Max | 974 ms |
| Total (8 scans) | 4.36 s |
| Real-time capable (<1s/scan) | **✅ YES** |

### Scalability Architecture

The O(n³) Hungarian assignment is the bottleneck for large-scale tracking. NX-MIMOSA addresses this with:

1. **KDTree spatial pre-gate** — O(n log n) candidate pair selection (20 km radius), eliminating >99% of impossible track-measurement pairs before cost computation.

2. **scipy.optimize.linear_sum_assignment** — C-optimized LAPJV algorithm, 10-100× faster than pure Python Hungarian. Handles 1000×1000 matrices in ~50ms.

3. **Sparse cost matrix** — Only Mahalanobis distances for KDTree-selected candidates are computed; remainder is set to 1e9 (impossible).

Measured scipy performance on this hardware:
- 500×500: ~25 ms
- 1000×1000: ~50 ms
- 1500×1500: ~114 ms

---

## Results: Military Aircraft Identification

### Detection Method

Military aircraft are identified via two complementary methods:

1. **ICAO24 hex range** (high confidence): US DoD (AE-AF), NATO (478-47F), RAF (43C), Luftwaffe (3F4-3F7), Armée de l'Air (3A8-3AF), Aeronautica Militare (33F)

2. **Callsign pattern** (medium confidence): 35+ prefixes mapped to service/platform (RCH→USAF AMC, FORTE→RQ-4, GAF→Luftwaffe, HRZ→Croatian AF, etc.)

### Military Aircraft Tracked

| ICAO24 | Callsign | Affiliation | Alt | Speed | KF RMS | Raw RMS | Improvement |
|--------|----------|-------------|-----|-------|--------|---------|-------------|
| 4784c2 | NSZ21U | NATO | FL340 | 443 kt | 224 m | 248 m | 1.11× |
| ae123a | RCH4539 | US DoD (AMC) | FL360 | 398 kt | 252 m | 268 m | 1.06× |
| 3b776f | CTM2004 | French AF (COTAM) | FL305 | 397 kt | 187 m | 219 m | 1.17× |
| 48d960 | PLF105 | Polish AF | FL182 | 384 kt | 214 m | 276 m | 1.29× |
| 479227 | NSZ3YT | NATO | FL162 | 338 kt | 161 m | 199 m | 1.24× |

### Military Tracking Summary

| Metric | Value |
|--------|-------|
| Military detected | 5 |
| Military jets (>150 m/s) | 5 |
| Jet mean KF RMS | **207 m** |
| Jet mean improvement vs raw | **1.17×** |

**Note on RCH4539**: USAF Air Mobility Command (C-17 or KC-135), observed FL360 at 398 kt crossing Central Europe eastbound — consistent with ongoing NATO logistics.

**Note on NSZ21U/NSZ3YT**: NATO callsign prefix, likely operating from NATO's Component Command or AWACS mission in the region.

---

## GOSPA Metric Decomposition

GOSPA (Generalized OSPA) with c=10 km cutoff:

| Component | Value | Interpretation |
|-----------|-------|----------------|
| **Total GOSPA** | 35,262 m | Aggregate tracking quality |
| Localization | 8,374 m | Accuracy of assigned tracks |
| Missed | 6,243 m (2.2 targets) | True targets with no track |
| False | 31,072 m (22.0 tracks) | Tracks with no true target |

**Analysis**: The false track component dominates GOSPA at this scale. With 761 measurements per scan and a minimum separation constraint of 1 km, the tracker creates ~22 extra tentative tracks from measurement noise. These are not confirmed long-term tracks but transient initiations. The localization component (8.4 km) is dominated by initial convergence on new tracks — once converged, individual track accuracy is 161–252 m as shown in the military jet results.

---

## Pytest Results

```
12 passed in 41.86s

TestLargeScaleMTT::test_500plus_targets          PASSED
TestLargeScaleMTT::test_scan_time_under_2s        PASSED
TestLargeScaleMTT::test_detection_rate             PASSED
TestLargeScaleMTT::test_gospa_bounded              PASSED
TestMilitaryTracking::test_military_identified     PASSED
TestMilitaryTracking::test_military_kf_improves    PASSED
TestMilitaryIdentification::test_us_dod_icao       PASSED
TestMilitaryIdentification::test_nato_icao         PASSED
TestMilitaryIdentification::test_callsign_rch      PASSED
TestMilitaryIdentification::test_callsign_gaf      PASSED
TestMilitaryIdentification::test_civil_not_detected PASSED
TestMilitaryIdentification::test_airline_exclusion  PASSED
```

---

## Competitive Position

| Capability | NX-MIMOSA v5.9.1 | Stone Soup | FilterPy |
|------------|-------------------|------------|----------|
| Max simultaneous targets | **761 (tested)** | ~100 (typical) | Single-target |
| Real-time at 500+ targets | **✅ 519ms** | ❌ No benchmark | N/A |
| Military ID | **✅ ICAO + callsign** | ❌ | ❌ |
| IMM (multi-model) | **✅ CV+CA+CT** | ✅ | ❌ |
| GOSPA metric | **✅** | ✅ | ❌ |
| EKF/UKF polar | **✅** | ✅ | ✅ (basic) |
| Live ADS-B validation | **✅ 761 aircraft** | Synthetic only | N/A |

---

## Files

- `python/nx_mimosa_mtt.py` — Core library (EKF3D, UKF3D, GOSPA, KDTree GNN, clutter gen)
- `tests/test_500plus_benchmark.py` — 500+ target benchmark + military ID + pytest suite
- `tests/test_v59_benchmark.py` — EKF/UKF polar + GOSPA + clutter resilience benchmark

---

*Benchmark executed 2025-02-07 | Author: Dr. Mladen Mešter, Nexellum d.o.o.*
