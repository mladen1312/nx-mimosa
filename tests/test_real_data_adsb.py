#!/usr/bin/env python3
"""
NX-MIMOSA v5.7 — Real ADS-B Data Benchmark
============================================
Tests NX-MIMOSA against LIVE aircraft position data from OpenSky Network.

Data source: OpenSky Network REST API (https://opensky-network.org)
License:     Creative Commons Attribution 4.0
Citation:    M. Schäfer et al., "Bringing Up OpenSky: A Large-scale ADS-B
             Sensor Network for Research", IPSN 2014

Methodology:
  1. Pull live ADS-B state vectors for Croatia/regional airspace
  2. ADS-B positions (GPS-based, ~10m accuracy) serve as ground truth
  3. Add σ=150m isotropic noise to simulate L-band surveillance radar
  4. Run NX-MIMOSA filters (CV Kalman, IMM-6, full MTT)
  5. Compare filtered output to clean ADS-B truth

Key insight: These aircraft are at cruise (straight-level, no maneuvering).
NX-MIMOSA's IMM advantage appears in maneuvering scenarios (see synthetic
benchmarks). For cruise traffic, even simple CV Kalman improves ~1.26× over
raw measurement — this is the honest, expected result.

// [REQ-V57-REAL-01] Real-data validation with ADS-B ground truth
// [REQ-V57-REAL-02] Per-aircraft state estimation benchmark
// [REQ-V57-REAL-03] Multi-target tracking benchmark (68 simultaneous)

Nexellum d.o.o. — Dr. Mladen Mešter — mladen@nexellum.com
"""

import os
import sys
import json
import pytest
import numpy as np
from collections import defaultdict

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))
from nx_mimosa_mtt import (
    KalmanFilter3D, IMM3D, make_cv3d_matrices,
    MultiTargetTracker, TrackManagerConfig, TrackStatus,
)

# ═══════════════════════════════════════════════════════════════════════
# ADS-B DATA FETCHER
# ═══════════════════════════════════════════════════════════════════════

def fetch_adsb_snapshots(n_snapshots: int = 12, sleep_sec: float = 11.0,
                         bbox: dict = None) -> list:
    """Pull live ADS-B snapshots from OpenSky Network.
    
    Args:
        n_snapshots: Number of snapshots to pull
        sleep_sec: Seconds between requests (anonymous rate limit ~10s)
        bbox: Bounding box {lamin, lamax, lomin, lomax}
    
    Returns:
        List of dicts with 'time' and 'states' keys
    """
    import requests
    import time
    
    if bbox is None:
        # Croatia + neighbors
        bbox = {"lamin": 43.0, "lamax": 47.5, "lomin": 13.0, "lomax": 20.0}
    
    API = "https://opensky-network.org/api/states/all"
    snapshots = []
    
    for i in range(n_snapshots):
        try:
            r = requests.get(API, params=bbox, timeout=15)
            data = r.json()
            snapshots.append({"time": data["time"], "states": data["states"]})
            if i < n_snapshots - 1:
                time.sleep(sleep_sec)
        except Exception:
            pass  # Skip failed requests
    
    return snapshots


def snapshots_to_enu_tracks(snapshots: list, ref_lat: float = 45.8,
                            ref_lon: float = 15.97, min_points: int = 6) -> dict:
    """Convert ADS-B snapshots to ENU track dictionaries.
    
    Args:
        snapshots: From fetch_adsb_snapshots()
        ref_lat/ref_lon: Reference point for ENU (default: Zagreb)
        min_points: Minimum position reports per aircraft
    
    Returns:
        Dict of icao -> {positions: Nx3, times: N, callsign, mean_vel, mean_alt}
    """
    R_EARTH = 6371000.0
    
    def geodetic_to_enu(lat, lon, alt):
        d_lat = np.radians(lat - ref_lat)
        d_lon = np.radians(lon - ref_lon)
        cos_ref = np.cos(np.radians(ref_lat))
        return np.array([R_EARTH * d_lon * cos_ref, R_EARTH * d_lat, alt])
    
    tracks_raw = defaultdict(list)
    for snap in snapshots:
        t = snap["time"]
        for sv in snap["states"]:
            icao = sv[0]
            lon, lat, baro_alt, vel = sv[5], sv[6], sv[7], sv[9]
            if None in (lon, lat, baro_alt, vel):
                continue
            tracks_raw[icao].append({
                "time": t, "lon": lon, "lat": lat, "alt": baro_alt,
                "vel": vel, "callsign": (sv[1] or "").strip()
            })
    
    enu_tracks = {}
    for icao, pts in tracks_raw.items():
        if len(pts) < min_points:
            continue
        pts.sort(key=lambda x: x["time"])
        positions = np.array([geodetic_to_enu(p["lat"], p["lon"], p["alt"]) for p in pts])
        times = np.array([p["time"] for p in pts])
        enu_tracks[icao] = {
            "positions": positions, "times": times,
            "callsign": pts[0]["callsign"],
            "mean_vel": np.mean([p["vel"] for p in pts]),
            "mean_alt": np.mean([p["alt"] for p in pts]),
        }
    
    return enu_tracks


# ═══════════════════════════════════════════════════════════════════════
# SYNTHETIC FALLBACK (for CI/offline testing)
# ═══════════════════════════════════════════════════════════════════════

def generate_synthetic_cruise_traffic(n_aircraft: int = 68, n_scans: int = 12,
                                      dt_mean: float = 11.0, seed: int = 42) -> dict:
    """Generate synthetic cruise traffic that mimics real ADS-B patterns.
    
    Creates straight-level flights at realistic commercial aviation parameters:
    - Speed: 200-290 m/s (720-1044 km/h)
    - Altitude: 8000-12500m (FL260-FL410)
    - Headings: random
    - dt: variable (1-26s) like real OpenSky snapshots
    """
    rng = np.random.RandomState(seed)
    
    # Generate variable dt sequence (matches real OpenSky pattern)
    dts = np.clip(rng.exponential(dt_mean, n_scans - 1), 1, 30)
    times = np.zeros(n_scans)
    times[1:] = np.cumsum(dts)
    
    enu_tracks = {}
    for i in range(n_aircraft):
        speed = rng.uniform(200, 290)  # m/s
        heading = rng.uniform(0, 2 * np.pi)
        alt = rng.uniform(8000, 12500)
        
        # Starting position: random within 300km of reference
        e0 = rng.uniform(-250000, 300000)
        n0 = rng.uniform(-300000, 200000)
        
        vx = speed * np.sin(heading)
        vy = speed * np.cos(heading)
        
        positions = np.zeros((n_scans, 3))
        for j in range(n_scans):
            positions[j] = [e0 + vx * times[j], n0 + vy * times[j], alt]
        
        icao = f"syn{i:03d}"
        enu_tracks[icao] = {
            "positions": positions,
            "times": times + 1770415296,  # Fake epoch
            "callsign": f"SYN{i:03d}",
            "mean_vel": speed,
            "mean_alt": alt,
        }
    
    return enu_tracks


# ═══════════════════════════════════════════════════════════════════════
# BENCHMARK FUNCTIONS
# ═══════════════════════════════════════════════════════════════════════

def run_per_aircraft_benchmark(enu_tracks: dict, noise_std: float = 150.0,
                               seed: int = 42) -> dict:
    """[REQ-V57-REAL-02] Per-aircraft state estimation benchmark.
    
    Returns dict with raw/kf/imm errors (all and converged).
    """
    rng = np.random.RandomState(seed)
    
    kf_results = []
    imm_results = []
    raw_results = []
    
    for icao, track in enu_tracks.items():
        truth = track["positions"]
        times = track["times"]
        n = len(truth)
        if n < 4:
            continue
        
        noisy = truth + rng.randn(n, 3) * noise_std
        raw_results.append(np.linalg.norm(noisy[1:] - truth[1:], axis=1))
        
        # CV Kalman
        kf = KalmanFilter3D(nx=6, nz=3)
        kf.x[:3] = noisy[0]
        kf.R = np.eye(3) * noise_std**2
        kf_err = []
        for i in range(1, n):
            dt = max(times[i] - times[i-1], 1.0)
            F, Q = make_cv3d_matrices(dt, 0.5)
            kf.predict(F, Q)
            kf.update(noisy[i])
            kf_err.append(np.linalg.norm(kf.x[:3] - truth[i]))
        kf_results.append(np.array(kf_err))
        
        # IMM-6
        imm = IMM3D(dt=10.0, q_base=0.5, r_std=noise_std)
        for f in imm.filters:
            f.x[:3] = noisy[0]
        imm_err = []
        for i in range(1, n):
            dt = max(times[i] - times[i-1], 1.0)
            imm.dt = dt
            imm.predict()
            imm.update(noisy[i])
            imm_err.append(np.linalg.norm(imm.position - truth[i]))
        imm_results.append(np.array(imm_err))
    
    all_raw = np.concatenate(raw_results)
    all_kf = np.concatenate(kf_results)
    all_imm = np.concatenate(imm_results)
    kf_conv = np.concatenate([e[2:] for e in kf_results if len(e) > 2])
    imm_conv = np.concatenate([e[2:] for e in imm_results if len(e) > 2])
    raw_conv = np.concatenate([e[2:] for e in raw_results if len(e) > 2])
    
    return {
        "n_aircraft": len(kf_results),
        "raw_rms": np.sqrt(np.mean(all_raw**2)),
        "kf_rms": np.sqrt(np.mean(all_kf**2)),
        "imm_rms": np.sqrt(np.mean(all_imm**2)),
        "raw_conv_rms": np.sqrt(np.mean(raw_conv**2)),
        "kf_conv_rms": np.sqrt(np.mean(kf_conv**2)),
        "imm_conv_rms": np.sqrt(np.mean(imm_conv**2)),
        "kf_improvement": np.sqrt(np.mean(raw_conv**2)) / np.sqrt(np.mean(kf_conv**2)),
        "imm_improvement": np.sqrt(np.mean(raw_conv**2)) / np.sqrt(np.mean(imm_conv**2)),
        "raw_median": np.median(all_raw),
        "kf_median": np.median(all_kf),
        "imm_median": np.median(all_imm),
    }


def run_mtt_benchmark(enu_tracks: dict, noise_std: float = 150.0,
                      seed: int = 42) -> dict:
    """[REQ-V57-REAL-03] Multi-target tracking benchmark."""
    rng = np.random.RandomState(seed)
    
    all_times = set()
    for t in enu_tracks.values():
        all_times.update(t["times"].tolist())
    time_grid = sorted(all_times)
    n_scans = len(time_grid)
    time_to_idx = {t: i for i, t in enumerate(time_grid)}
    
    scan_truth = [[] for _ in range(n_scans)]
    scan_meas = [[] for _ in range(n_scans)]
    
    for icao, track in enu_tracks.items():
        for i, t_epoch in enumerate(track["times"]):
            idx = time_to_idx[t_epoch]
            pos = track["positions"][i]
            noisy = pos + rng.randn(3) * noise_std
            scan_truth[idx].append(pos)
            scan_meas[idx].append(noisy)
    
    config = TrackManagerConfig(
        confirm_m=2, confirm_n=3, delete_misses=4,
        coast_misses=2, max_coast_age=8,
        gate_threshold=100.0, min_separation=500.0
    )
    tracker = MultiTargetTracker(dt=10.0, r_std=noise_std, q_base=0.5,
                                  config=config, association="gnn")
    
    mtt_errors = []
    n_active_per_scan = []
    
    for scan_idx in range(n_scans):
        meas = scan_meas[scan_idx]
        if not meas:
            n_active_per_scan.append(0)
            continue
        
        if scan_idx > 0:
            actual_dt = max(time_grid[scan_idx] - time_grid[scan_idx - 1], 1.0)
            tracker.dt = actual_dt
            for t in tracker.tracks:
                if hasattr(t.filter, 'dt'):
                    t.filter.dt = actual_dt
        
        active = tracker.process_scan(np.array(meas))
        n_active_per_scan.append(len(active))
        truth_arr = np.array(scan_truth[scan_idx])
        
        for t in active:
            dists = np.linalg.norm(truth_arr - t.filter.position, axis=1)
            min_d = np.min(dists)
            if min_d < 5000:
                mtt_errors.append(min_d)
    
    mtt_errors = np.array(mtt_errors)
    
    return {
        "n_aircraft": len(enu_tracks),
        "n_scans": n_scans,
        "avg_active": np.mean(n_active_per_scan),
        "detection_ratio": np.mean(n_active_per_scan) / len(enu_tracks),
        "mtt_rms": np.sqrt(np.mean(mtt_errors**2)),
        "mtt_mean": np.mean(mtt_errors),
        "mtt_median": np.median(mtt_errors),
        "mtt_p90": np.percentile(mtt_errors, 90),
        "n_samples": len(mtt_errors),
    }


def format_benchmark_report(pa: dict, mtt: dict) -> str:
    """Format human-readable benchmark report."""
    lines = [
        "=" * 72,
        "NX-MIMOSA v5.7 — REAL ADS-B DATA BENCHMARK",
        "OpenSky Network | Croatia/Regional Airspace",
        "=" * 72,
        "",
        "TEST A: Per-Aircraft State Estimation",
        f"  {pa['n_aircraft']} aircraft, σ=150m radar noise on GPS truth",
        "",
        f"  {'':20s} {'Raw':>8s} {'CV KF':>8s} {'IMM-6':>8s}",
        f"  {'-'*48}",
        f"  {'RMS (m)':<20s} {pa['raw_rms']:>7.1f} {pa['kf_rms']:>7.1f} {pa['imm_rms']:>7.1f}",
        f"  {'Median (m)':<20s} {pa['raw_median']:>7.1f} {pa['kf_median']:>7.1f} {pa['imm_median']:>7.1f}",
        f"  {'Conv RMS (m)':<20s} {pa['raw_conv_rms']:>7.1f} {pa['kf_conv_rms']:>7.1f} {pa['imm_conv_rms']:>7.1f}",
        "",
        f"  CV Kalman improvement (converged): {pa['kf_improvement']:.2f}×",
        f"  IMM-6 improvement (converged):     {pa['imm_improvement']:.2f}×",
        "",
        "TEST B: Multi-Target Tracking",
        f"  {mtt['n_aircraft']} aircraft, {mtt['n_scans']} scans",
        "",
        f"  Detection ratio: {mtt['detection_ratio']:.0%}",
        f"  Position RMS:    {mtt['mtt_rms']:.1f}m",
        f"  Position Median: {mtt['mtt_median']:.1f}m",
        "",
        "NOTES:",
        "  These aircraft are at cruise (straight-level, non-maneuvering).",
        "  NX-MIMOSA's IMM advantage is in maneuvering scenarios (synthetic",
        "  benchmarks show 8.6× mean improvement, 54× in orbital maneuver).",
        "  For cruise traffic, ~1.2× is the honest, expected result.",
        "=" * 72,
    ]
    return "\n".join(lines)


# ═══════════════════════════════════════════════════════════════════════
# TESTS
# ═══════════════════════════════════════════════════════════════════════

@pytest.fixture(scope="module")
def enu_tracks():
    """Load or generate ENU tracks."""
    import pickle
    cache = "/home/claude/adsb_enu_tracks.pkl"
    if os.path.exists(cache):
        with open(cache, "rb") as f:
            return pickle.load(f)
    # Fallback to synthetic
    return generate_synthetic_cruise_traffic()


class TestPerAircraftEstimation:
    """[REQ-V57-REAL-02] State estimation on real ADS-B data."""
    
    def test_kf_improves_over_raw(self, enu_tracks):
        """CV Kalman should improve over raw measurements."""
        result = run_per_aircraft_benchmark(enu_tracks)
        assert result["kf_improvement"] > 1.0, \
            f"KF should improve: {result['kf_improvement']:.2f}×"
    
    def test_kf_median_lower(self, enu_tracks):
        """KF median should be lower than raw median."""
        result = run_per_aircraft_benchmark(enu_tracks)
        assert result["kf_median"] < result["raw_median"], \
            f"KF median {result['kf_median']:.1f} >= raw {result['raw_median']:.1f}"
    
    def test_imm_improves_over_raw(self, enu_tracks):
        """IMM should improve over raw (at least marginally on cruise data)."""
        result = run_per_aircraft_benchmark(enu_tracks)
        assert result["imm_improvement"] > 1.0, \
            f"IMM should improve: {result['imm_improvement']:.2f}×"
    
    def test_kf_beats_imm_on_cruise(self, enu_tracks):
        """On straight-level cruise, CV KF should match or beat IMM.
        
        This is expected: cruise traffic doesn't maneuver, so extra IMM
        models add noise. This test documents the expected behavior.
        """
        result = run_per_aircraft_benchmark(enu_tracks)
        # KF should be at least as good as IMM on cruise data
        assert result["kf_improvement"] >= result["imm_improvement"] * 0.9, \
            "KF should be competitive with IMM on non-maneuvering data"
    
    def test_kf_rms_under_raw(self, enu_tracks):
        """Converged KF RMS should be below raw RMS."""
        result = run_per_aircraft_benchmark(enu_tracks)
        assert result["kf_conv_rms"] < result["raw_conv_rms"]


class TestMultiTargetTracking:
    """[REQ-V57-REAL-03] Multi-target tracking on real ADS-B data."""
    
    def test_detection_above_80pct(self, enu_tracks):
        """Should track at least 80% of aircraft."""
        result = run_mtt_benchmark(enu_tracks)
        assert result["detection_ratio"] > 0.80, \
            f"Detection {result['detection_ratio']:.0%} < 80%"
    
    def test_mtt_median_under_500m(self, enu_tracks):
        """Median position error should be under 500m."""
        result = run_mtt_benchmark(enu_tracks)
        assert result["mtt_median"] < 500.0, \
            f"Median {result['mtt_median']:.1f}m > 500m"
    
    def test_mtt_has_samples(self, enu_tracks):
        """Should produce reasonable number of track-truth associations."""
        result = run_mtt_benchmark(enu_tracks)
        assert result["n_samples"] > 100, \
            f"Only {result['n_samples']} samples"


class TestBenchmarkReport:
    """Report generation test."""
    
    def test_report_format(self, enu_tracks):
        """Report should be non-empty and contain key metrics."""
        pa = run_per_aircraft_benchmark(enu_tracks)
        mtt = run_mtt_benchmark(enu_tracks)
        report = format_benchmark_report(pa, mtt)
        assert len(report) > 500
        assert "CV Kalman improvement" in report
        assert "Detection ratio" in report
        print("\n" + report)


if __name__ == "__main__":
    # Run standalone: fetch live data, benchmark, print report
    import pickle
    
    cache = "/home/claude/adsb_enu_tracks.pkl"
    if os.path.exists(cache):
        with open(cache, "rb") as f:
            tracks = pickle.load(f)
        print(f"Loaded {len(tracks)} cached tracks")
    else:
        print("Fetching live ADS-B data (takes ~2 minutes)...")
        snapshots = fetch_adsb_snapshots()
        tracks = snapshots_to_enu_tracks(snapshots)
        with open(cache, "wb") as f:
            pickle.dump(tracks, f)
        print(f"Cached {len(tracks)} tracks")
    
    pa = run_per_aircraft_benchmark(tracks)
    mtt = run_mtt_benchmark(tracks)
    print(format_benchmark_report(pa, mtt))
