#!/usr/bin/env python3
"""
NX-MIMOSA v6.1.0 — Full-Scale ECCM Benchmark
==============================================

Tests 1,000 / 2,000 / 5,000 simultaneous aircraft from live OpenSky ADS-B.
Measures: tracking performance, ECCM detection accuracy, timing.

Three test phases per scale:
  Phase A: Clean tracking (baseline)
  Phase B: Injected jamming (4 ECM types on 10% of tracks)
  Phase C: Full ECCM pipeline (ML-CFAR + adaptive integration)

Usage: python scripts/benchmark_v610_eccm.py
"""

import sys, os, time, json, pickle, requests
import numpy as np
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))

from nx_mimosa_mtt import (
    MultiTargetTracker, TrackManagerConfig, ECMDetector,
)
from nx_mimosa_eccm import (
    MLCFARDetector, AdaptiveIntegrator, ECCMPipeline, EnvironmentClass,
)

SEED = 42
DT = 15.0          # OpenSky update interval
R_STD = 150.0      # ADS-B noise std (meters)
CLUTTER_PER_SCAN = 5

# ════════════════════════════════════════════════════════════════════
# DATA FETCH & PROCESSING
# ════════════════════════════════════════════════════════════════════

def fetch_opensky_snapshots(n_snaps: int = 12, interval: float = 16.0) -> List[dict]:
    """Fetch global ADS-B snapshots with caching."""
    cache = f"/tmp/nx_v610_global_{n_snaps}s.pkl"
    if os.path.exists(cache):
        age_min = (time.time() - os.path.getmtime(cache)) / 60
        if age_min < 60:  # Cache valid for 1 hour
            print(f"  [CACHE HIT] {cache} ({age_min:.0f} min old)")
            with open(cache, 'rb') as f:
                return pickle.load(f)
        print(f"  [CACHE STALE] {age_min:.0f} min, refetching...")

    API = "https://opensky-network.org/api/states/all"
    snaps = []

    for i in range(n_snaps):
        for attempt in range(3):
            try:
                r = requests.get(API, timeout=30)
                if r.status_code == 429:
                    wait = 15 + attempt * 15
                    print(f"  Scan {i+1}: RATE LIMITED, wait {wait}s...")
                    time.sleep(wait)
                    continue
                if r.status_code == 200:
                    data = r.json()
                    states = data.get("states") or []
                    valid = sum(1 for sv in states if sv[5] is not None and sv[6] is not None)
                    snaps.append({"time": data["time"], "states": states})
                    print(f"  Scan {i+1}/{n_snaps}: {len(states)} total, {valid} positioned")
                    break
                else:
                    print(f"  Scan {i+1}: HTTP {r.status_code}, retry...")
                    time.sleep(5)
            except Exception as e:
                print(f"  Scan {i+1}: {e}, retry...")
                time.sleep(5)

        if i < n_snaps - 1:
            time.sleep(interval)

    if snaps:
        with open(cache, 'wb') as f:
            pickle.dump(snaps, f)
        print(f"  Cached to {cache}")

    return snaps


def geodetic_to_enu(lat, lon, alt, ref_lat, ref_lon, ref_alt=0):
    """Geodetic (deg) → local ENU (meters)."""
    d2r = np.pi / 180.0
    R_EARTH = 6371000.0
    dlat = (lat - ref_lat) * d2r
    dlon = (lon - ref_lon) * d2r
    cos_ref = np.cos(ref_lat * d2r)
    e = dlon * cos_ref * R_EARTH
    n = dlat * R_EARTH
    u = alt - ref_alt
    return np.array([e, n, u])


def build_aircraft_tracks(snaps: List[dict], max_aircraft: int = 9999) -> Tuple[dict, list, float, float]:
    """Build per-aircraft track histories from OpenSky snapshots.
    
    Returns: (tracks_dict, unique_times, ref_lat, ref_lon)
    """
    # Collect all positioned aircraft
    all_obs = defaultdict(list)  # icao24 → [(time, lat, lon, alt, vel, heading, callsign)]

    for snap in snaps:
        t = snap["time"]
        for sv in snap.get("states") or []:
            icao24 = sv[0]
            lon, lat, alt = sv[5], sv[6], sv[7]
            vel = sv[9]
            heading = sv[10]
            callsign = (sv[1] or "").strip()
            on_ground = sv[8]

            if lat is None or lon is None or on_ground:
                continue
            if alt is None:
                alt = 10000.0  # Default cruise
            if vel is None:
                vel = 200.0

            all_obs[icao24].append((t, lat, lon, alt, vel, heading, callsign))

    # Filter: need at least 3 observations for trackable aircraft
    valid_icaos = [k for k, v in all_obs.items() if len(v) >= 3]
    valid_icaos.sort(key=lambda k: -len(all_obs[k]))  # Most observations first

    if len(valid_icaos) > max_aircraft:
        valid_icaos = valid_icaos[:max_aircraft]

    # Reference point: centroid of all aircraft
    all_lats = [obs[1] for k in valid_icaos for obs in all_obs[k]]
    all_lons = [obs[2] for k in valid_icaos for obs in all_obs[k]]
    ref_lat = np.mean(all_lats)
    ref_lon = np.mean(all_lons)

    # Build ENU tracks
    tracks = {}
    for icao24 in valid_icaos:
        obs = sorted(all_obs[icao24], key=lambda x: x[0])
        # Deduplicate by time
        seen_t = set()
        deduped = []
        for o in obs:
            if o[0] not in seen_t:
                seen_t.add(o[0])
                deduped.append(o)
        obs = deduped

        positions_enu = []
        times = []
        for o in obs:
            t, lat, lon, alt, vel, heading, cs = o
            enu = geodetic_to_enu(lat, lon, alt, ref_lat, ref_lon)
            positions_enu.append(enu)
            times.append(t)

        tracks[icao24] = {
            "positions": positions_enu,
            "times": times,
            "callsign": obs[-1][6],
            "n_obs": len(obs),
        }

    unique_times = sorted(set(t for trk in tracks.values() for t in trk["times"]))
    return tracks, unique_times, ref_lat, ref_lon


# ════════════════════════════════════════════════════════════════════
# JAMMING INJECTION
# ════════════════════════════════════════════════════════════════════

@dataclass
class JammingScenario:
    """Defines synthetic jamming applied to a subset of tracks."""
    target_icaos: List[str]
    jam_type: str  # 'NOISE', 'RGPO', 'DRFM', 'SCREENING'
    start_scan: int
    intensity: float = 1.0

def inject_jamming(measurement: np.ndarray, jam_type: str, scan_idx: int,
                   start_scan: int, intensity: float, rng) -> Tuple[np.ndarray, bool]:
    """Corrupt a measurement according to jamming type.
    
    Returns: (corrupted_measurement, was_hit)
    """
    if scan_idx < start_scan:
        return measurement, True

    elapsed = scan_idx - start_scan

    if jam_type == 'NOISE':
        # Barrage noise: inflate noise 5-20×, sometimes miss
        noise_mult = 5.0 + 15.0 * intensity
        corrupted = measurement + rng.randn(3) * R_STD * noise_mult
        was_hit = rng.random() > 0.3 * intensity  # 30% dropout at full power
        return corrupted, was_hit

    elif jam_type == 'RGPO':
        # Range gate pull-off: range drifts away progressively
        pull_rate = 500.0 * intensity  # 500 m/scan at full power
        range_offset = pull_rate * elapsed
        direction = measurement[:2] / (np.linalg.norm(measurement[:2]) + 1e-6)
        corrupted = measurement.copy()
        corrupted[:2] += direction * range_offset
        return corrupted, True

    elif jam_type == 'DRFM':
        # Digital RF Memory: creates coherent false target offset
        offset = rng.randn(3) * 2000.0 * intensity
        offset[2] *= 0.3
        corrupted = measurement + offset
        # Also inject ghost track measurement
        return corrupted, True

    elif jam_type == 'SCREENING':
        # Screening jammer: periodic blanking
        period = max(2, int(4 / (intensity + 0.01)))
        was_hit = (elapsed % period) != 0
        return measurement, was_hit

    return measurement, True


# ════════════════════════════════════════════════════════════════════
# GOSPA METRIC
# ════════════════════════════════════════════════════════════════════

def compute_gospa(track_positions, truth_positions, c=10000.0, p=2):
    """Compute GOSPA metric."""
    n_tracks = len(track_positions)
    n_truth = len(truth_positions)

    if n_tracks == 0 and n_truth == 0:
        return {"gospa": 0, "localization": 0, "missed": 0, "false": 0,
                "n_assigned": 0, "n_missed": 0, "n_false": 0}
    if n_truth == 0:
        false_cost = n_tracks * (c ** p / 2)
        return {"gospa": false_cost ** (1/p), "localization": 0, "missed": 0,
                "false": false_cost ** (1/p), "n_assigned": 0, "n_missed": 0, "n_false": n_tracks}
    if n_tracks == 0:
        miss_cost = n_truth * (c ** p / 2)
        return {"gospa": miss_cost ** (1/p), "localization": 0,
                "missed": miss_cost ** (1/p), "false": 0,
                "n_assigned": 0, "n_missed": n_truth, "n_false": 0}

    T = np.array(track_positions)
    G = np.array(truth_positions)

    # Distance matrix
    dist = np.linalg.norm(T[:, None, :] - G[None, :, :], axis=2)
    clipped = np.minimum(dist, c)

    # Greedy assignment
    assigned_t = set()
    assigned_g = set()
    loc_cost = 0.0

    flat = [(clipped[i, j], i, j) for i in range(n_tracks) for j in range(n_truth)]
    flat.sort()

    for d, i, j in flat:
        if i not in assigned_t and j not in assigned_g:
            loc_cost += d ** p
            assigned_t.add(i)
            assigned_g.add(j)

    n_assigned = len(assigned_t)
    n_missed = n_truth - n_assigned
    n_false = n_tracks - n_assigned

    miss_cost = n_missed * (c ** p / 2)
    false_cost = n_false * (c ** p / 2)
    total = loc_cost + miss_cost + false_cost

    n_total = max(n_truth, n_tracks)
    gospa = (total / n_total) ** (1/p) if n_total > 0 else 0

    return {
        "gospa": gospa,
        "localization": (loc_cost / max(n_assigned, 1)) ** (1/p),
        "missed": (miss_cost / max(1, n_total)) ** (1/p),
        "false": (false_cost / max(1, n_total)) ** (1/p),
        "n_assigned": n_assigned,
        "n_missed": n_missed,
        "n_false": n_false,
    }


# ════════════════════════════════════════════════════════════════════
# BENCHMARK ENGINE
# ════════════════════════════════════════════════════════════════════

def run_benchmark(tracks: dict, unique_times: list, ref_lat: float, ref_lon: float,
                  n_aircraft: int, label: str,
                  jamming_fraction: float = 0.0,
                  use_eccm: bool = False) -> dict:
    """Run one benchmark configuration.
    
    Args:
        tracks: all available tracks
        unique_times: sorted epoch times
        n_aircraft: how many to use
        label: test label
        jamming_fraction: fraction of tracks to jam (0 = clean)
        use_eccm: whether to use ML-CFAR ECCM pipeline
    """
    rng = np.random.RandomState(SEED)
    
    # Select n_aircraft with most observations
    selected = sorted(tracks.keys(), key=lambda k: -tracks[k]["n_obs"])[:n_aircraft]
    
    # Build scan-indexed measurements
    t2i = {t: i for i, t in enumerate(unique_times)}
    n_scans = len(unique_times)
    
    scan_truth = [[] for _ in range(n_scans)]
    scan_meas = [[] for _ in range(n_scans)]
    scan_icaos = [[] for _ in range(n_scans)]
    
    # Setup jamming
    n_jammed = int(len(selected) * jamming_fraction)
    jammed_icaos = set(rng.choice(selected, size=n_jammed, replace=False)) if n_jammed > 0 else set()
    jam_types = ['NOISE', 'RGPO', 'DRFM', 'SCREENING']
    jam_assignment = {}
    for i, icao in enumerate(jammed_icaos):
        jam_assignment[icao] = jam_types[i % 4]
    
    jam_start_scan = 4  # Start jamming after track establishment
    
    for icao in selected:
        trk = tracks[icao]
        for i, t_epoch in enumerate(trk["times"]):
            if t_epoch not in t2i:
                continue
            si = t2i[t_epoch]
            pos = trk["positions"][i]
            
            # Add measurement noise
            noisy = pos + rng.randn(3) * R_STD
            noisy[2] += rng.randn() * R_STD * 0.3  # Less altitude noise
            
            # Apply jamming if applicable
            was_hit = True
            if icao in jam_assignment:
                noisy, was_hit = inject_jamming(
                    noisy, jam_assignment[icao], si, jam_start_scan,
                    intensity=0.8, rng=rng)
            
            scan_truth[si].append(pos)
            if was_hit:
                scan_meas[si].append(noisy)
            scan_icaos[si].append(icao)
    
    # Add clutter (scaled to scene size)
    n_clutter = max(2, CLUTTER_PER_SCAN * n_aircraft // 1000)
    for si in range(n_scans):
        for _ in range(n_clutter):
            scan_meas[si].append(rng.uniform(-5e5, 5e5, 3))
    
    # Configure tracker — aggressive management for large scale
    gate_thresh = 30.0 if n_aircraft >= 2000 else 40.0
    config = TrackManagerConfig(
        confirm_m=2, confirm_n=3,
        delete_misses=3,         # Delete faster
        coast_misses=1, max_coast_age=4,  # Short coast
        gate_threshold=gate_thresh,
        min_separation=2000.0,   # Wider dedup for dense scenes
    )
    
    mtt = MultiTargetTracker(
        dt=DT, r_std=R_STD, q_base=0.5,
        config=config, association="gnn",
    )
    
    # ECCM setup
    ecm_legacy = ECMDetector()
    eccm_pipeline = ECCMPipeline(ml_window=10) if use_eccm else None
    
    # ─── RUN TRACKING ───
    scan_times_ms = []
    gospa_scores = []
    ecm_detections_legacy = defaultdict(set)
    ecm_detections_ml = defaultdict(set)
    eccm_env_counts = defaultdict(int)
    
    for si in range(n_scans):
        meas = scan_meas[si]
        truth = scan_truth[si]
        
        if not meas:
            continue
        
        # Actual dt
        if si > 0 and si < len(unique_times):
            actual_dt = max(unique_times[si] - unique_times[min(si-1, len(unique_times)-1)], 1.0)
            mtt.dt = min(actual_dt, 60.0)
        
        # Track
        t0 = time.perf_counter()
        active = mtt.process_scan(np.array(meas))
        t_ms = (time.perf_counter() - t0) * 1000
        scan_times_ms.append(t_ms)
        
        # GOSPA
        track_pos = [t.filter.position for t in active]
        g = compute_gospa(track_pos, truth, c=10000.0, p=2)
        gospa_scores.append(g)
        
        # ECM analysis on each confirmed track
        for trk in active:
            tid = trk.track_id
            nis = trk.avg_nis if trk.avg_nis > 0 else 3.0
                
            innovation = None  # Not stored on TrackState; use NIS as proxy
            pos = trk.filter.position
            velocity = trk.filter.velocity if hasattr(trk.filter, 'velocity') else np.zeros(3)
            was_associated = trk.hit_count > 0
            
            # Legacy ECM detector
            leg_result = ecm_legacy.update(
                track_id=tid, nis=nis,
                innovation=innovation, position=pos,
                velocity=velocity, was_hit=was_associated)
            if leg_result['ecm_detected']:
                for etype in leg_result['ecm_types']:
                    ecm_detections_legacy[etype].add(tid)
            
            # ML-CFAR ECCM
            if eccm_pipeline is not None:
                innov_vec = innovation if innovation is not None else np.zeros(3)
                vel_vec = velocity if velocity is not None else np.zeros(3)
                eccm_result = eccm_pipeline.update(
                    track_id=tid, nis=nis,
                    innovation=innov_vec, velocity=vel_vec,
                    was_hit=was_associated)
                env = eccm_result['env_class']
                eccm_env_counts[env.name] += 1
                if env != EnvironmentClass.CLEAR:
                    ecm_detections_ml[env.name].add(tid)
        
        if si % 3 == 0 or si == n_scans - 1:
            n_meas = len(meas)
            n_trk = len(active)
            print(f"  [{label}] Scan {si+1}/{n_scans}: {n_meas} meas → "
                  f"{n_trk} tracks | {t_ms:.0f}ms | GOSPA={g['gospa']:.0f}m")
    
    # ─── AGGREGATE ───
    skip = min(3, len(gospa_scores) - 1)
    valid_g = gospa_scores[skip:]
    valid_t = scan_times_ms[skip:]
    
    n_per_scan = [len(m) for m in scan_meas if m]
    
    result = {
        "label": label,
        "n_aircraft": n_aircraft,
        "n_jammed": n_jammed,
        "use_eccm": use_eccm,
        "n_scans": n_scans,
        "max_simultaneous": max(n_per_scan) if n_per_scan else 0,
        "mean_simultaneous": np.mean(n_per_scan) if n_per_scan else 0,
        # Timing
        "mean_scan_ms": float(np.mean(valid_t)) if valid_t else 0,
        "max_scan_ms": float(np.max(valid_t)) if valid_t else 0,
        "p95_scan_ms": float(np.percentile(valid_t, 95)) if valid_t else 0,
        "total_time_s": sum(scan_times_ms) / 1000,
        # GOSPA
        "mean_gospa": float(np.mean([g["gospa"] for g in valid_g])) if valid_g else 0,
        "mean_localization": float(np.mean([g["localization"] for g in valid_g])) if valid_g else 0,
        "mean_missed": float(np.mean([g["n_missed"] for g in valid_g])) if valid_g else 0,
        "mean_false": float(np.mean([g["n_false"] for g in valid_g])) if valid_g else 0,
        "mean_assigned": float(np.mean([g["n_assigned"] for g in valid_g])) if valid_g else 0,
        # Tracker stats
        "final_confirmed": mtt.stats.get("current_confirmed", len(mtt.confirmed_tracks)),
        "total_created": mtt.stats.get("total_tracks_created", 0),
        # ECM
        "legacy_ecm_detections": {k: len(v) for k, v in ecm_detections_legacy.items()},
        "ml_cfar_detections": {k: len(v) for k, v in ecm_detections_ml.items()},
        "eccm_env_counts": dict(eccm_env_counts),
        "jam_assignment": {k: v for k, v in jam_assignment.items()},
        # Per scan
        "scan_times_ms": scan_times_ms,
        "gospa_per_scan": [g["gospa"] for g in gospa_scores],
    }
    
    return result


# ════════════════════════════════════════════════════════════════════
# MAIN
# ════════════════════════════════════════════════════════════════════

def main():
    print("=" * 72)
    print("NX-MIMOSA v6.1.0 — Full-Scale ECCM Benchmark")
    print("OpenSky ADS-B Real Data | ML-CFAR + TDOA + Adaptive Integration")
    print("=" * 72)
    
    # ─── PHASE 1: Fetch data ───
    print("\n[1/4] Fetching OpenSky global ADS-B data...")
    snaps = fetch_opensky_snapshots(n_snaps=12, interval=16.0)
    
    if not snaps:
        print("FATAL: No data fetched. Check internet/API.")
        sys.exit(1)
    
    print(f"\n  Got {len(snaps)} snapshots")
    
    # ─── PHASE 2: Build tracks ───
    print("\n[2/4] Building aircraft tracks...")
    tracks, unique_times, ref_lat, ref_lon = build_aircraft_tracks(snaps, max_aircraft=6000)
    print(f"  {len(tracks)} trackable aircraft (≥3 observations)")
    print(f"  {len(unique_times)} unique time steps")
    print(f"  Reference: ({ref_lat:.2f}°, {ref_lon:.2f}°)")
    
    obs_counts = [trk["n_obs"] for trk in tracks.values()]
    print(f"  Observations per aircraft: median={np.median(obs_counts):.0f}, "
          f"max={max(obs_counts)}, min={min(obs_counts)}")
    
    available = len(tracks)
    scales = []
    for n in [1000, 2000, 5000]:
        if n <= available:
            scales.append(n)
        else:
            print(f"  ⚠ Only {available} aircraft available, capping at {available} instead of {n}")
            scales.append(available)
            break
    
    # ─── PHASE 3: Run benchmarks ───
    print(f"\n[3/4] Running benchmarks at scales: {scales}")
    all_results = []
    
    for n_ac in scales:
        print(f"\n{'─'*60}")
        print(f"  SCALE: {n_ac} aircraft")
        print(f"{'─'*60}")
        
        # Phase A: Clean tracking
        print(f"\n  [A] Clean tracking (no jamming)...")
        r_clean = run_benchmark(tracks, unique_times, ref_lat, ref_lon,
                                n_ac, f"{n_ac}-CLEAN", jamming_fraction=0.0, use_eccm=False)
        all_results.append(r_clean)
        
        # Phase B: Jamming WITHOUT ECCM
        print(f"\n  [B] Jammed (10% tracks, no ECCM protection)...")
        r_jammed = run_benchmark(tracks, unique_times, ref_lat, ref_lon,
                                 n_ac, f"{n_ac}-JAMMED", jamming_fraction=0.10, use_eccm=False)
        all_results.append(r_jammed)
        
        # Phase C: Jamming WITH ECCM
        print(f"\n  [C] Jammed + ECCM pipeline (ML-CFAR + adaptive)...")
        r_eccm = run_benchmark(tracks, unique_times, ref_lat, ref_lon,
                               n_ac, f"{n_ac}-ECCM", jamming_fraction=0.10, use_eccm=True)
        all_results.append(r_eccm)
    
    # ─── PHASE 4: Analysis ───
    print(f"\n{'='*72}")
    print("  RESULTS ANALYSIS")
    print(f"{'='*72}")
    
    # Print results table
    print(f"\n{'Label':<16} {'Targets':>8} {'Mean ms':>8} {'P95 ms':>8} "
          f"{'GOSPA':>8} {'RMS':>8} {'Missed':>8} {'False':>8} {'Conf':>8}")
    print("─" * 96)
    
    for r in all_results:
        print(f"{r['label']:<16} {r['max_simultaneous']:>8} {r['mean_scan_ms']:>8.1f} "
              f"{r['p95_scan_ms']:>8.1f} {r['mean_gospa']:>8.0f} "
              f"{r['mean_localization']:>8.0f} {r['mean_missed']:>8.1f} "
              f"{r['mean_false']:>8.1f} {r['final_confirmed']:>8}")
    
    # ECM detection analysis
    print(f"\n{'='*72}")
    print("  ECM DETECTION ANALYSIS")
    print(f"{'='*72}")
    
    for r in all_results:
        if r['n_jammed'] > 0:
            print(f"\n  [{r['label']}] — {r['n_jammed']} jammed tracks")
            print(f"    Legacy ECM: {r['legacy_ecm_detections']}")
            if r['use_eccm']:
                print(f"    ML-CFAR:    {r['ml_cfar_detections']}")
                print(f"    Env counts: {r['eccm_env_counts']}")
    
    # Save JSON
    output = {
        "version": "6.1.0",
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S UTC", time.gmtime()),
        "data_source": "OpenSky Network ADS-B",
        "n_snapshots": len(snaps),
        "total_aircraft_available": len(tracks),
        "ref_lat": ref_lat,
        "ref_lon": ref_lon,
        "results": [{k: v for k, v in r.items() 
                     if k not in ('scan_times_ms', 'gospa_per_scan', 'jam_assignment')} 
                    for r in all_results],
        "full_results": all_results,
    }
    
    json_path = "/tmp/nx_v610_benchmark_results.json"
    with open(json_path, 'w') as f:
        json.dump(output, f, indent=2, default=str)
    print(f"\n  Results saved: {json_path}")
    
    return output, all_results


if __name__ == "__main__":
    output, results = main()
