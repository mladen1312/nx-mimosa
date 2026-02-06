#!/usr/bin/env python3
"""
NX-MIMOSA v5.7 — Real ADS-B Data Validation Test
===================================================

Tests tracker against REAL aircraft trajectories from OpenSky Network.
ADS-B positions (GPS, ~10m) serve as ground truth.
Radar noise is added synthetically per Bar-Shalom methodology.

This test downloads live ADS-B data, so requires network access.
Results are non-deterministic but statistical bounds are tested.

[REQ-V57-REAL-01] Real-data validation against ADS-B ground truth
[REQ-V57-REAL-02] Track confirmation rate >= 95%  
[REQ-V57-REAL-03] Tracker beats raw measurements on >= 80% of aircraft
[REQ-V57-REAL-04] Median noise reduction factor >= 1.1×
"""
import pytest
import json
import os
import sys
import numpy as np
from scipy.interpolate import interp1d
from dataclasses import dataclass
from typing import List, Dict, Optional

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))
from nx_mimosa_mtt import MultiTargetTracker, TrackStatus
from nx_mimosa_coords import geodetic_to_enu

# ============================================================
# Data collection / caching
# ============================================================
CACHE_FILE = os.path.join(os.path.dirname(__file__), ".adsb_cache.json")
REF_LAT, REF_LON, REF_ALT = 48.0, 15.0, 0.0
UNIFORM_DT = 10.0
MIN_OBS = 10
BBOX = dict(lamin=45.0, lomin=10.0, lamax=52.0, lomax=20.0)


@dataclass
class AircraftTrack:
    icao24: str
    callsign: str
    times: np.ndarray
    truth_enu: np.ndarray
    velocities: np.ndarray
    headings: np.ndarray


def collect_adsb(n_polls: int = 20, interval: int = 10) -> List[dict]:
    """Collect ADS-B snapshots from OpenSky Network."""
    import requests, time
    url = "https://opensky-network.org/api/states/all"
    snapshots = []
    for i in range(n_polls):
        try:
            resp = requests.get(url, params=BBOX, timeout=15)
            data = resp.json()
            states = []
            for s in data.get("states", []):
                icao24, callsign, origin, tp, lc, lon, lat, baro, on_gnd, vel, hdg, vr, sens, geo, sq, spi, src = s[:17]
                if on_gnd or lat is None or lon is None or geo is None or vel is None:
                    continue
                states.append(dict(icao24=icao24, callsign=(callsign or "").strip(),
                                   lat=lat, lon=lon, alt=geo, vel=vel,
                                   heading=hdg or 0, vert_rate=vr or 0))
            snapshots.append(dict(time=data["time"], states=states))
            if i < n_polls - 1:
                time.sleep(interval)
        except Exception:
            if i < n_polls - 1:
                time.sleep(interval)
    return snapshots


def load_or_collect() -> List[dict]:
    """Use cached data if available, otherwise collect live."""
    if os.path.exists(CACHE_FILE):
        with open(CACHE_FILE) as f:
            return json.load(f)
    
    # Also check if the benchmark file exists
    alt_cache = "/home/claude/adsb_raw.json"
    if os.path.exists(alt_cache):
        with open(alt_cache) as f:
            return json.load(f)
    
    snapshots = collect_adsb(n_polls=20, interval=10)
    with open(CACHE_FILE, "w") as f:
        json.dump(snapshots, f)
    return snapshots


def build_uniform_tracks(snapshots: List[dict]) -> List[AircraftTrack]:
    """Build uniform-dt trajectories from ADS-B snapshots."""
    raw_trajs = {}
    callsigns = {}
    
    for snap in snapshots:
        t = snap["time"]
        for s in snap["states"]:
            icao = s["icao24"]
            if s["lat"] is None or s["lon"] is None or s["alt"] is None:
                continue
            r = geodetic_to_enu(s["lat"], s["lon"], s["alt"], REF_LAT, REF_LON, REF_ALT)
            e, n, u = float(r[0]), float(r[1]), float(r[2])
            if icao not in raw_trajs:
                raw_trajs[icao] = []
            raw_trajs[icao].append((t, e, n, u, s["vel"], s["heading"]))
            if s.get("callsign"):
                callsigns[icao] = s["callsign"]
    
    tracks = []
    for icao, traj in raw_trajs.items():
        traj.sort(key=lambda x: x[0])
        clean = [traj[0]]
        for i in range(1, len(traj)):
            if traj[i][0] > clean[-1][0]:
                clean.append(traj[i])
        if len(clean) < 8:
            continue
        
        ts = np.array([p[0] for p in clean], dtype=float)
        n_steps = int((ts[-1] - ts[0]) / UNIFORM_DT)
        if n_steps < MIN_OBS:
            continue
        
        t_uni = np.arange(n_steps) * UNIFORM_DT + ts[0]
        e_i = interp1d(ts, [p[1] for p in clean], fill_value='extrapolate')(t_uni)
        n_i = interp1d(ts, [p[2] for p in clean], fill_value='extrapolate')(t_uni)
        u_i = interp1d(ts, [p[3] for p in clean], fill_value='extrapolate')(t_uni)
        v_i = interp1d(ts, [p[4] for p in clean], fill_value='extrapolate')(t_uni)
        h_i = interp1d(ts, [p[5] for p in clean], fill_value='extrapolate')(t_uni)
        
        tracks.append(AircraftTrack(
            icao24=icao,
            callsign=callsigns.get(icao, ""),
            times=t_uni - t_uni[0],
            truth_enu=np.column_stack([e_i, n_i, u_i]),
            velocities=v_i,
            headings=h_i,
        ))
    return tracks


def track_aircraft(track: AircraftTrack, r_std: float, seed: int,
                   domain: str = "atc") -> Dict:
    """Run NX-MIMOSA tracker on a single aircraft."""
    rng = np.random.RandomState(seed)
    measurements = track.truth_enu + rng.randn(*track.truth_enu.shape) * r_std
    
    tracker = MultiTargetTracker(dt=UNIFORM_DT, r_std=r_std, domain=domain)
    
    pos_errors = []
    raw_errors = []
    confirmed = False
    
    for i in range(len(measurements)):
        raw_errors.append(np.linalg.norm(measurements[i] - track.truth_enu[i]))
        tracks = tracker.process_scan(measurements[i:i+1])
        
        if tracks:
            best = min(
                (t for t in tracks if t.status in (TrackStatus.CONFIRMED, TrackStatus.COASTING)),
                key=lambda t: np.linalg.norm(t.filter.position - track.truth_enu[i]),
                default=None,
            )
            if best is not None:
                confirmed = True
                pos_errors.append(np.linalg.norm(best.filter.position - track.truth_enu[i]))
    
    pos = np.array(pos_errors) if pos_errors else np.array([])
    raw = np.array(raw_errors)
    
    return dict(
        confirmed=confirmed,
        rms_pos=float(np.sqrt(np.mean(pos**2))) if len(pos) > 0 else float('inf'),
        raw_rms=float(np.sqrt(np.mean(raw**2))),
        improvement=float(np.sqrt(np.mean(raw**2)) / np.sqrt(np.mean(pos**2))) if len(pos) > 0 and np.mean(pos**2) > 0 else 0,
    )


# ============================================================
# Fixtures
# ============================================================
@pytest.fixture(scope="module")
def adsb_tracks():
    """Load or collect ADS-B tracks."""
    snapshots = load_or_collect()
    tracks = build_uniform_tracks(snapshots)
    assert len(tracks) >= 20, f"Need >= 20 aircraft, got {len(tracks)}"
    return tracks


@pytest.fixture(scope="module")
def benchmark_results(adsb_tracks):
    """Run tracker on all aircraft at σ=100m."""
    results = []
    for i, track in enumerate(adsb_tracks):
        results.append(track_aircraft(track, r_std=100.0, seed=42+i))
    return results


# ============================================================
# Tests
# ============================================================
class TestRealADSB:
    """[REQ-V57-REAL] Real-data validation suite."""
    
    def test_sufficient_aircraft(self, adsb_tracks):
        """[REQ-V57-REAL-00] Enough aircraft for statistical validity."""
        assert len(adsb_tracks) >= 50, \
            f"Need >= 50 aircraft, got {len(adsb_tracks)}"
    
    def test_track_confirmation_rate(self, benchmark_results):
        """[REQ-V57-REAL-02] >= 95% of real aircraft confirmed."""
        confirmed = sum(1 for r in benchmark_results if r["confirmed"])
        rate = confirmed / len(benchmark_results)
        assert rate >= 0.95, \
            f"Confirmation rate {rate:.1%} < 95% ({confirmed}/{len(benchmark_results)})"
    
    def test_beats_raw_majority(self, benchmark_results):
        """[REQ-V57-REAL-03] Tracker beats raw on >= 80% of aircraft."""
        confirmed = [r for r in benchmark_results if r["confirmed"]]
        wins = sum(1 for r in confirmed if r["improvement"] > 1.0)
        rate = wins / len(confirmed) if confirmed else 0
        assert rate >= 0.80, \
            f"Win rate {rate:.1%} < 80% ({wins}/{len(confirmed)})"
    
    def test_median_improvement(self, benchmark_results):
        """[REQ-V57-REAL-04] Median noise reduction >= 1.1×."""
        confirmed = [r for r in benchmark_results if r["confirmed"]]
        improvements = [r["improvement"] for r in confirmed]
        median = np.median(improvements)
        assert median >= 1.1, \
            f"Median improvement {median:.2f}× < 1.1×"
    
    def test_no_catastrophic_failures(self, benchmark_results, adsb_tracks):
        """[REQ-V57-REAL-05] < 5% catastrophic failures (10× worse than raw)."""
        confirmed = [r for r in benchmark_results if r["confirmed"]]
        catastrophic = sum(1 for r in confirmed if r["improvement"] < 0.1)
        rate = catastrophic / len(confirmed) if confirmed else 0
        assert rate < 0.05, \
            f"Catastrophic failure rate {rate:.1%} >= 5% ({catastrophic}/{len(confirmed)})"
    
    def test_summary_report(self, benchmark_results, adsb_tracks):
        """Print human-readable summary (always passes)."""
        confirmed = [r for r in benchmark_results if r["confirmed"]]
        rms_vals = [r["rms_pos"] for r in confirmed]
        raw_vals = [r["raw_rms"] for r in confirmed]
        imprs = [r["improvement"] for r in confirmed]
        wins = sum(1 for i in imprs if i > 1.0)
        
        print(f"\n{'=' * 60}")
        print(f"NX-MIMOSA REAL-DATA BENCHMARK (σ=100m, dt={UNIFORM_DT}s)")
        print(f"{'=' * 60}")
        print(f"Aircraft:       {len(adsb_tracks)}")
        print(f"Confirmed:      {len(confirmed)} ({100*len(confirmed)/len(benchmark_results):.0f}%)")
        print(f"Raw median:     {np.median(raw_vals):.1f}m")
        print(f"Tracker median: {np.median(rms_vals):.1f}m")
        print(f"Improvement:    {np.median(imprs):.2f}× (median)")
        print(f"Win rate:       {wins}/{len(confirmed)} ({100*wins/len(confirmed):.0f}%)")
        print(f"{'=' * 60}")
