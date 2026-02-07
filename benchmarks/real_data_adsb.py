#!/usr/bin/env python3
"""
NX-MIMOSA Real-Data Benchmark — Live ADS-B Aircraft Tracking
=============================================================

Pulls LIVE aircraft positions from OpenSky Network API,
runs NX-MIMOSA multi-target tracker on real measurements,
and computes tracking accuracy against ADS-B ground truth.

This is the REAL-DATA validation that proves NX-MIMOSA works
on actual aircraft — not just simulations.

Usage:
    python benchmarks/real_data_adsb.py                    # Full run
    python benchmarks/real_data_adsb.py --snapshots 30     # More data
    python benchmarks/real_data_adsb.py --save             # Save plots

Nexellum d.o.o. — Dr. Mladen Mešter — mladen@nexellum.com
"""

import sys
import os
import time
import json
import urllib.request
import urllib.error
import argparse
import math
import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from python.nx_mimosa_mtt import MultiTargetTracker, TrackStatus
from python.nx_mimosa_coords import geodetic_to_enu, haversine_distance


# ═══════════════════════════════════════════════════════════════════
# DATA COLLECTION — OpenSky Network REST API
# ═══════════════════════════════════════════════════════════════════

@dataclass
class AircraftState:
    """Single ADS-B state vector."""
    icao24: str
    callsign: str
    origin_country: str
    longitude: float
    latitude: float
    baro_altitude: float  # meters
    velocity: float       # m/s ground speed
    true_track: float     # degrees from north
    vertical_rate: float  # m/s
    geo_altitude: float   # meters
    timestamp: float      # unix epoch
    on_ground: bool
    category: int


@dataclass
class Snapshot:
    """One time-slice of all aircraft."""
    timestamp: float
    aircraft: List[AircraftState]


def fetch_opensky_states(
    lamin: float = 43.0, lomin: float = 14.0,
    lamax: float = 47.0, lomax: float = 20.0,
) -> Optional[Snapshot]:
    """Fetch live aircraft states from OpenSky Network.
    
    Default bounding box: Croatia + surrounding airspace.
    """
    url = (f"https://opensky-network.org/api/states/all"
           f"?lamin={lamin}&lomin={lomin}&lamax={lamax}&lomax={lomax}")
    try:
        req = urllib.request.Request(url, headers={"User-Agent": "NX-MIMOSA-Benchmark/5.7"})
        with urllib.request.urlopen(req, timeout=15) as resp:
            data = json.loads(resp.read().decode())
    except (urllib.error.URLError, json.JSONDecodeError) as e:
        print(f"  [WARN] API error: {e}")
        return None

    states = data.get("states", [])
    if not states:
        return None

    aircraft = []
    for s in states:
        # Skip aircraft with missing position data
        if s[5] is None or s[6] is None or s[7] is None:
            continue
        if s[8]:  # on_ground — skip
            continue
        aircraft.append(AircraftState(
            icao24=s[0] or "",
            callsign=(s[1] or "").strip(),
            origin_country=s[2] or "",
            longitude=float(s[5]),
            latitude=float(s[6]),
            baro_altitude=float(s[7]) if s[7] else 0.0,
            velocity=float(s[9]) if s[9] else 0.0,
            true_track=float(s[10]) if s[10] else 0.0,
            vertical_rate=float(s[11]) if s[11] else 0.0,
            geo_altitude=float(s[13]) if s[13] else float(s[7] or 0),
            timestamp=float(s[3] or s[4] or data["time"]),
            on_ground=bool(s[8]),
            category=int(s[16]) if s[16] else 0,
        ))

    return Snapshot(timestamp=float(data["time"]), aircraft=aircraft)


def collect_snapshots(
    n_snapshots: int = 20,
    interval_s: float = 10.0,
    **kwargs,
) -> List[Snapshot]:
    """Collect multiple time snapshots from OpenSky."""
    snapshots = []
    print(f"\n{'='*70}")
    print(f"COLLECTING {n_snapshots} SNAPSHOTS @ {interval_s}s INTERVAL")
    print(f"{'='*70}")

    for i in range(n_snapshots):
        t0 = time.time()
        snap = fetch_opensky_states(**kwargs)
        elapsed = time.time() - t0

        if snap and snap.aircraft:
            snapshots.append(snap)
            print(f"  [{i+1:3d}/{n_snapshots}] t={snap.timestamp:.0f} "
                  f"aircraft={len(snap.aircraft):3d} ({elapsed:.1f}s)")
        else:
            print(f"  [{i+1:3d}/{n_snapshots}] EMPTY or ERROR ({elapsed:.1f}s)")

        if i < n_snapshots - 1:
            sleep_time = max(0, interval_s - elapsed)
            time.sleep(sleep_time)

    print(f"\nCollected {len(snapshots)} valid snapshots")
    return snapshots


# ═══════════════════════════════════════════════════════════════════
# CONVERSION — WGS-84 → ENU for tracking
# ═══════════════════════════════════════════════════════════════════

def compute_reference_point(snapshots: List[Snapshot]) -> Tuple[float, float, float]:
    """Compute centroid of all observations as ENU reference."""
    lats, lons, alts = [], [], []
    for snap in snapshots:
        for ac in snap.aircraft:
            lats.append(ac.latitude)
            lons.append(ac.longitude)
            alts.append(ac.baro_altitude)
    return np.mean(lats), np.mean(lons), np.mean(alts)


def wgs84_to_enu_pos(lat: float, lon: float, alt: float,
                      ref_lat: float, ref_lon: float, ref_alt: float) -> np.ndarray:
    """Convert WGS-84 to ENU coordinates."""
    return geodetic_to_enu(lat, lon, alt, ref_lat, ref_lon, ref_alt)


# ═══════════════════════════════════════════════════════════════════
# GROUND TRUTH EXTRACTION — Build per-aircraft trajectories
# ═══════════════════════════════════════════════════════════════════

@dataclass
class AircraftTrajectory:
    """Ground truth trajectory for one aircraft."""
    icao24: str
    callsign: str
    timestamps: List[float] = field(default_factory=list)
    positions_enu: List[np.ndarray] = field(default_factory=list)
    positions_wgs84: List[Tuple[float, float, float]] = field(default_factory=list)
    velocities: List[float] = field(default_factory=list)


def extract_trajectories(
    snapshots: List[Snapshot],
    ref_lat: float, ref_lon: float, ref_alt: float,
    min_observations: int = 5,
) -> Dict[str, AircraftTrajectory]:
    """Extract per-aircraft trajectories from snapshots."""
    trajs: Dict[str, AircraftTrajectory] = {}

    for snap in snapshots:
        for ac in snap.aircraft:
            key = ac.icao24
            if key not in trajs:
                trajs[key] = AircraftTrajectory(
                    icao24=ac.icao24,
                    callsign=ac.callsign,
                )
            traj = trajs[key]
            if ac.callsign and not traj.callsign:
                traj.callsign = ac.callsign

            enu = wgs84_to_enu_pos(
                ac.latitude, ac.longitude, ac.baro_altitude,
                ref_lat, ref_lon, ref_alt,
            )
            traj.timestamps.append(snap.timestamp)
            traj.positions_enu.append(enu)
            traj.positions_wgs84.append((ac.latitude, ac.longitude, ac.baro_altitude))
            traj.velocities.append(ac.velocity)

    # Filter: keep only aircraft with enough observations
    filtered = {k: v for k, v in trajs.items() if len(v.timestamps) >= min_observations}
    print(f"\nTrajectories: {len(trajs)} total, {len(filtered)} with ≥{min_observations} observations")
    return filtered


# ═══════════════════════════════════════════════════════════════════
# TRACKER EXECUTION — Run NX-MIMOSA on real measurements
# ═══════════════════════════════════════════════════════════════════

@dataclass
class TrackResult:
    """Per-scan tracker output for one track."""
    scan_idx: int
    track_id: int
    position: np.ndarray
    status: str


def run_tracker_on_snapshots(
    snapshots: List[Snapshot],
    ref_lat: float, ref_lon: float, ref_alt: float,
    noise_std: float = 0.0,
    seed: int = 42,
) -> Tuple[List[List[TrackResult]], List[float]]:
    """Run NX-MIMOSA multi-target tracker on real ADS-B data.
    
    Args:
        noise_std: Additional measurement noise (m). 0 = pure ADS-B.
                   Set >0 to simulate radar-grade noise on real trajectories.
    """
    rng = np.random.default_rng(seed)

    # Compute dt from first two snapshots
    if len(snapshots) < 2:
        raise ValueError("Need ≥2 snapshots")
    dt = snapshots[1].timestamp - snapshots[0].timestamp
    if dt <= 0:
        dt = 10.0
    print(f"\nTracker config: dt={dt:.1f}s, noise_std={noise_std:.1f}m, domain=atc")

    tracker = MultiTargetTracker(
        dt=dt,
        r_std=max(50.0, noise_std) if noise_std > 0 else 50.0,
        domain="atc",
        association="gnn",
    )

    all_results = []
    scan_timestamps = []

    for scan_idx, snap in enumerate(snapshots):
        # Convert all aircraft positions to ENU measurements
        measurements = []
        for ac in snap.aircraft:
            enu = wgs84_to_enu_pos(
                ac.latitude, ac.longitude, ac.baro_altitude,
                ref_lat, ref_lon, ref_alt,
            )
            if noise_std > 0:
                enu = enu + rng.normal(0, noise_std, size=3)
            measurements.append(enu)

        if not measurements:
            all_results.append([])
            scan_timestamps.append(snap.timestamp)
            continue

        meas_array = np.array(measurements)
        tracks = tracker.process_scan(meas_array)

        scan_results = []
        for t in tracks:
            scan_results.append(TrackResult(
                scan_idx=scan_idx,
                track_id=t.track_id,
                position=t.filter.position.copy(),
                status=t.status.name,
            ))
        all_results.append(scan_results)
        scan_timestamps.append(snap.timestamp)

    return all_results, scan_timestamps


# ═══════════════════════════════════════════════════════════════════
# METRICS — Real-data tracking performance
# ═══════════════════════════════════════════════════════════════════

@dataclass
class TrackingMetrics:
    """Comprehensive real-data tracking metrics."""
    n_aircraft: int
    n_snapshots: int
    n_confirmed_tracks: int
    n_truth_trajectories: int

    # Position accuracy (for associated tracks)
    mean_position_error_m: float
    median_position_error_m: float
    p95_position_error_m: float
    max_position_error_m: float

    # Track management
    track_continuity: float     # % of truth observations covered by a track
    false_track_rate: float     # fraction of tracks not matching any truth
    track_fragmentation: float  # avg fragments per truth trajectory

    # Velocity accuracy
    mean_velocity_error_ms: float

    # Per-aircraft details
    per_aircraft: Dict[str, dict] = field(default_factory=dict)


def associate_tracks_to_truth(
    all_results: List[List[TrackResult]],
    scan_timestamps: List[float],
    trajectories: Dict[str, AircraftTrajectory],
    snapshots: List[Snapshot],
    ref_lat: float, ref_lon: float, ref_alt: float,
    gate_m: float = 5000.0,
) -> TrackingMetrics:
    """Associate tracker outputs to ground truth and compute metrics."""

    # Build truth position lookup: scan_idx -> list of (icao24, enu_pos)
    truth_lookup = {}
    for scan_idx, snap in enumerate(snapshots):
        truth_lookup[scan_idx] = []
        for ac in snap.aircraft:
            if ac.icao24 in trajectories:
                enu = wgs84_to_enu_pos(
                    ac.latitude, ac.longitude, ac.baro_altitude,
                    ref_lat, ref_lon, ref_alt,
                )
                truth_lookup[scan_idx].append((ac.icao24, enu, ac.velocity))

    # Greedy association: for each track at each scan, find nearest truth
    position_errors = []
    velocity_errors = []
    truth_covered = {k: set() for k in trajectories}  # scan indices covered
    track_to_truth = {}  # track_id -> set of icao24s it matched

    for scan_idx, scan_results in enumerate(all_results):
        truths = truth_lookup.get(scan_idx, [])
        if not truths or not scan_results:
            continue

        truth_positions = np.array([t[1] for t in truths])
        truth_icaos = [t[0] for t in truths]
        truth_vels = [t[2] for t in truths]

        for tr in scan_results:
            if tr.status not in ("CONFIRMED", "COASTING"):
                continue

            # Find nearest truth
            dists = np.linalg.norm(truth_positions - tr.position, axis=1)
            best_idx = np.argmin(dists)
            best_dist = dists[best_idx]

            if best_dist < gate_m:
                position_errors.append(best_dist)
                icao = truth_icaos[best_idx]
                truth_covered[icao].add(scan_idx)

                if tr.track_id not in track_to_truth:
                    track_to_truth[tr.track_id] = set()
                track_to_truth[tr.track_id].add(icao)

    # Count confirmed tracks at final scan
    final_confirmed = 0
    if all_results:
        final_confirmed = sum(1 for tr in all_results[-1]
                              if tr.status in ("CONFIRMED", "COASTING"))

    # Track continuity: fraction of truth observations covered
    total_truth_obs = sum(len(t.timestamps) for t in trajectories.values())
    total_covered = sum(len(scans) for scans in truth_covered.values())
    continuity = total_covered / max(total_truth_obs, 1)

    # Fragmentation: how many distinct tracks matched each truth
    truth_to_tracks: Dict[str, set] = {k: set() for k in trajectories}
    for tid, icaos in track_to_truth.items():
        for icao in icaos:
            if icao in truth_to_tracks:
                truth_to_tracks[icao].add(tid)
    frags = [len(tids) for tids in truth_to_tracks.values() if tids]
    avg_frag = np.mean(frags) if frags else 0.0

    # False tracks: tracks that never matched any truth
    all_track_ids = set()
    for scan_results in all_results:
        for tr in scan_results:
            if tr.status in ("CONFIRMED", "COASTING"):
                all_track_ids.add(tr.track_id)
    matched_track_ids = set(track_to_truth.keys())
    false_tracks = all_track_ids - matched_track_ids
    false_rate = len(false_tracks) / max(len(all_track_ids), 1)

    pos_errors = np.array(position_errors) if position_errors else np.array([0.0])

    # Per-aircraft breakdown
    per_aircraft = {}
    for icao, traj in trajectories.items():
        n_obs = len(traj.timestamps)
        n_covered = len(truth_covered[icao])
        n_tracks = len(truth_to_tracks[icao])
        per_aircraft[icao] = {
            "callsign": traj.callsign,
            "observations": n_obs,
            "covered": n_covered,
            "continuity": n_covered / max(n_obs, 1),
            "n_tracks": n_tracks,
            "mean_velocity_ms": np.mean(traj.velocities) if traj.velocities else 0,
        }

    return TrackingMetrics(
        n_aircraft=len(set(ac.icao24 for snap in snapshots for ac in snap.aircraft)),
        n_snapshots=len(snapshots),
        n_confirmed_tracks=final_confirmed,
        n_truth_trajectories=len(trajectories),
        mean_position_error_m=float(np.mean(pos_errors)),
        median_position_error_m=float(np.median(pos_errors)),
        p95_position_error_m=float(np.percentile(pos_errors, 95)),
        max_position_error_m=float(np.max(pos_errors)),
        track_continuity=continuity,
        false_track_rate=false_rate,
        track_fragmentation=avg_frag,
        mean_velocity_error_ms=0.0,  # TODO: velocity comparison
        per_aircraft=per_aircraft,
    )


# ═══════════════════════════════════════════════════════════════════
# REPORTING
# ═══════════════════════════════════════════════════════════════════

def print_report(metrics: TrackingMetrics, noise_label: str = ""):
    """Print human-readable tracking report."""
    print(f"\n{'='*70}")
    print(f"NX-MIMOSA REAL-DATA BENCHMARK — ADS-B AIRCRAFT TRACKING {noise_label}")
    print(f"{'='*70}")
    print(f"Data source:          OpenSky Network (live ADS-B)")
    print(f"Snapshots:            {metrics.n_snapshots}")
    print(f"Unique aircraft seen: {metrics.n_aircraft}")
    print(f"Truth trajectories:   {metrics.n_truth_trajectories} (≥5 observations)")
    print(f"Confirmed tracks:     {metrics.n_confirmed_tracks} (at final scan)")
    print()
    print(f"POSITION ACCURACY (ENU, confirmed tracks vs ADS-B truth)")
    print(f"  Mean error:         {metrics.mean_position_error_m:.1f} m")
    print(f"  Median error:       {metrics.median_position_error_m:.1f} m")
    print(f"  95th percentile:    {metrics.p95_position_error_m:.1f} m")
    print(f"  Max error:          {metrics.max_position_error_m:.1f} m")
    print()
    print(f"TRACK MANAGEMENT")
    print(f"  Track continuity:   {metrics.track_continuity*100:.1f}%")
    print(f"  False track rate:   {metrics.false_track_rate*100:.1f}%")
    print(f"  Avg fragmentation:  {metrics.track_fragmentation:.2f} tracks/truth")
    print()

    # Top-10 aircraft by observations
    sorted_ac = sorted(metrics.per_aircraft.items(),
                       key=lambda x: x[1]["observations"], reverse=True)
    print(f"TOP TRACKED AIRCRAFT")
    print(f"{'ICAO24':<10s} {'Callsign':<10s} {'Obs':>4s} {'Covered':>7s} "
          f"{'Cont%':>6s} {'Trks':>4s} {'Speed':>8s}")
    print(f"{'-'*55}")
    for icao, info in sorted_ac[:15]:
        print(f"{icao:<10s} {info['callsign']:<10s} {info['observations']:>4d} "
              f"{info['covered']:>7d} {info['continuity']*100:>5.1f}% "
              f"{info['n_tracks']:>4d} {info['mean_velocity_ms']:>7.1f}m/s")

    print(f"\n{'='*70}")


# ═══════════════════════════════════════════════════════════════════
# PLOTTING
# ═══════════════════════════════════════════════════════════════════

def plot_results(
    snapshots: List[Snapshot],
    all_results: List[List[TrackResult]],
    trajectories: Dict[str, AircraftTrajectory],
    ref_lat: float, ref_lon: float, ref_alt: float,
    metrics: TrackingMetrics,
    noise_label: str = "",
    save_path: Optional[str] = None,
):
    """Plot tracking results — truth vs tracker output."""
    import matplotlib
    if save_path:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 3, figsize=(20, 7))
    fig.suptitle(
        f"NX-MIMOSA Real-Data Benchmark — {metrics.n_aircraft} Aircraft, "
        f"{metrics.n_snapshots} Scans {noise_label}\n"
        f"Mean Error: {metrics.mean_position_error_m:.1f}m | "
        f"Continuity: {metrics.track_continuity*100:.1f}% | "
        f"False Rate: {metrics.false_track_rate*100:.1f}%",
        fontsize=12, fontweight="bold",
    )

    # --- Panel 1: XY plan view (truth + tracks) ---
    ax = axes[0]
    ax.set_title("Plan View (East-North)")
    ax.set_xlabel("East (km)")
    ax.set_ylabel("North (km)")

    # Truth trajectories
    for icao, traj in trajectories.items():
        if len(traj.positions_enu) >= 2:
            pts = np.array(traj.positions_enu)
            ax.plot(pts[:, 0]/1000, pts[:, 1]/1000, 'b-', alpha=0.3, linewidth=0.8)
            ax.plot(pts[-1, 0]/1000, pts[-1, 1]/1000, 'b.', markersize=3)

    # Tracker output (confirmed only)
    track_history: Dict[int, List[np.ndarray]] = {}
    for scan_results in all_results:
        for tr in scan_results:
            if tr.status in ("CONFIRMED", "COASTING"):
                if tr.track_id not in track_history:
                    track_history[tr.track_id] = []
                track_history[tr.track_id].append(tr.position)

    for tid, positions in track_history.items():
        if len(positions) >= 2:
            pts = np.array(positions)
            ax.plot(pts[:, 0]/1000, pts[:, 1]/1000, 'r-', alpha=0.5, linewidth=1.2)
            ax.plot(pts[-1, 0]/1000, pts[-1, 1]/1000, 'r^', markersize=4)

    ax.legend(["ADS-B truth", "", "NX-MIMOSA tracks"], loc="upper left", fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal")

    # --- Panel 2: Altitude profile ---
    ax = axes[1]
    ax.set_title("Altitude Profile")
    ax.set_xlabel("Time index")
    ax.set_ylabel("Altitude (km)")

    for icao, traj in list(trajectories.items())[:20]:
        if len(traj.positions_enu) >= 2:
            alts = [p[2]/1000 for p in traj.positions_enu]
            ax.plot(range(len(alts)), alts, 'b-', alpha=0.3, linewidth=0.8)

    for tid, positions in list(track_history.items())[:30]:
        if len(positions) >= 2:
            alts = [p[2]/1000 for p in positions]
            ax.plot(range(len(alts)), alts, 'r-', alpha=0.5, linewidth=1.0)

    ax.grid(True, alpha=0.3)

    # --- Panel 3: Error histogram ---
    ax = axes[2]
    ax.set_title("Position Error Distribution")

    # Recompute errors for histogram
    errors = []
    for scan_idx, scan_results in enumerate(all_results):
        if scan_idx >= len(snapshots):
            break
        snap = snapshots[scan_idx]
        truth_map = {}
        for ac in snap.aircraft:
            if ac.icao24 in trajectories:
                enu = wgs84_to_enu_pos(ac.latitude, ac.longitude, ac.baro_altitude,
                                        ref_lat, ref_lon, ref_alt)
                truth_map[ac.icao24] = enu

        if not truth_map:
            continue
        truth_arr = np.array(list(truth_map.values()))
        for tr in scan_results:
            if tr.status in ("CONFIRMED", "COASTING"):
                dists = np.linalg.norm(truth_arr - tr.position, axis=1)
                errors.append(np.min(dists))

    if errors:
        errors = np.array(errors)
        ax.hist(errors[errors < np.percentile(errors, 98)], bins=50,
                color="steelblue", edgecolor="white", alpha=0.8)
        ax.axvline(np.mean(errors), color="red", linestyle="--",
                   label=f"Mean: {np.mean(errors):.0f}m")
        ax.axvline(np.median(errors), color="orange", linestyle="--",
                   label=f"Median: {np.median(errors):.0f}m")
        ax.legend(fontsize=9)
    ax.set_xlabel("Error (m)")
    ax.set_ylabel("Count")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"  Plot saved: {save_path}")
    else:
        plt.show()
    plt.close()


# ═══════════════════════════════════════════════════════════════════
# NOISE SWEEP — Test with increasing radar-grade noise
# ═══════════════════════════════════════════════════════════════════

def noise_sweep(
    snapshots: List[Snapshot],
    ref_lat: float, ref_lon: float, ref_alt: float,
    trajectories: Dict[str, AircraftTrajectory],
    noise_levels: List[float] = [0, 25, 50, 100, 200, 500],
) -> List[Tuple[float, TrackingMetrics]]:
    """Run tracker at multiple noise levels to show degradation curve."""
    results = []
    print(f"\n{'='*70}")
    print(f"NOISE SWEEP — {len(noise_levels)} levels")
    print(f"{'='*70}")
    print(f"{'Noise σ':>10s} {'MeanErr':>10s} {'P95Err':>10s} "
          f"{'Contin%':>10s} {'FalseRate':>10s} {'Tracks':>8s}")
    print(f"{'-'*62}")

    for noise in noise_levels:
        all_results, timestamps = run_tracker_on_snapshots(
            snapshots, ref_lat, ref_lon, ref_alt,
            noise_std=noise, seed=42,
        )
        m = associate_tracks_to_truth(
            all_results, timestamps, trajectories,
            snapshots, ref_lat, ref_lon, ref_alt,
        )
        results.append((noise, m))
        print(f"{noise:>8.0f}m {m.mean_position_error_m:>10.1f} "
              f"{m.p95_position_error_m:>10.1f} "
              f"{m.track_continuity*100:>9.1f}% "
              f"{m.false_track_rate*100:>9.1f}% "
              f"{m.n_confirmed_tracks:>8d}")

    return results


# ═══════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="NX-MIMOSA Real-Data ADS-B Benchmark")
    parser.add_argument("--snapshots", type=int, default=20,
                        help="Number of snapshots to collect (default: 20)")
    parser.add_argument("--interval", type=float, default=10.0,
                        help="Seconds between snapshots (default: 10)")
    parser.add_argument("--noise", type=float, default=0.0,
                        help="Additional noise σ in meters (default: 0 = pure ADS-B)")
    parser.add_argument("--sweep", action="store_true",
                        help="Run noise sweep (0-500m)")
    parser.add_argument("--save", action="store_true",
                        help="Save plots as PNG")
    parser.add_argument("--data", type=str, default=None,
                        help="Load cached data from JSON file")
    parser.add_argument("--cache", type=str, default=None,
                        help="Save collected data to JSON file")
    args = parser.parse_args()

    print("""
╔══════════════════════════════════════════════════════════════╗
║    NX-MIMOSA v5.7 — REAL-DATA BENCHMARK                    ║
║    Live ADS-B Aircraft Tracking via OpenSky Network         ║
║                                                              ║
║    "When Tracking Fails, People Die."                       ║
║    Nexellum d.o.o. — Dr. Mladen Mešter                     ║
╚══════════════════════════════════════════════════════════════╝
""")

    # --- Collect or load data ---
    if args.data:
        print(f"Loading cached data from {args.data}...")
        with open(args.data) as f:
            cache = json.load(f)
        snapshots = []
        for s in cache["snapshots"]:
            aircraft = [AircraftState(**a) for a in s["aircraft"]]
            snapshots.append(Snapshot(timestamp=s["timestamp"], aircraft=aircraft))
    else:
        snapshots = collect_snapshots(
            n_snapshots=args.snapshots,
            interval_s=args.interval,
        )

    if len(snapshots) < 3:
        print("ERROR: Not enough snapshots collected. Try again or check network.")
        sys.exit(1)

    # --- Cache data ---
    if args.cache:
        cache = {"snapshots": []}
        for s in snapshots:
            cache["snapshots"].append({
                "timestamp": s.timestamp,
                "aircraft": [vars(a) for a in s.aircraft],
            })
        with open(args.cache, "w") as f:
            json.dump(cache, f)
        print(f"Data cached to {args.cache}")

    # --- Reference point & trajectories ---
    ref_lat, ref_lon, ref_alt = compute_reference_point(snapshots)
    print(f"\nReference point: ({ref_lat:.4f}°N, {ref_lon:.4f}°E, {ref_alt:.0f}m)")

    trajectories = extract_trajectories(
        snapshots, ref_lat, ref_lon, ref_alt, min_observations=5,
    )

    # --- Run tracker ---
    noise_label = f"(+{args.noise:.0f}m noise)" if args.noise > 0 else "(pure ADS-B)"
    all_results, timestamps = run_tracker_on_snapshots(
        snapshots, ref_lat, ref_lon, ref_alt,
        noise_std=args.noise,
    )

    # --- Compute metrics ---
    metrics = associate_tracks_to_truth(
        all_results, timestamps, trajectories,
        snapshots, ref_lat, ref_lon, ref_alt,
    )
    print_report(metrics, noise_label)

    # --- Plot ---
    save_path = None
    if args.save:
        os.makedirs("benchmarks/results", exist_ok=True)
        save_path = "benchmarks/results/real_data_adsb.png"
    plot_results(
        snapshots, all_results, trajectories,
        ref_lat, ref_lon, ref_alt, metrics,
        noise_label=noise_label, save_path=save_path,
    )

    # --- Noise sweep ---
    if args.sweep:
        sweep_results = noise_sweep(
            snapshots, ref_lat, ref_lon, ref_alt, trajectories,
        )

    print("\n✅ REAL-DATA BENCHMARK COMPLETE")
    return metrics


if __name__ == "__main__":
    main()
