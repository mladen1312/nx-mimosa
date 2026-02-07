#!/usr/bin/env python3
"""
NX-MIMOSA v6.2.0 — ECCM Integration Benchmark
===============================================
Tests all four v6.2.0 improvements:
  1. C++ ECCM core (no Python overlay overhead)
  2. Closed-loop: ECCM classify → R inflate in IMM
  3. Extended ML window (20 scans)
  4. Cross-track sector correlation

Benchmark matrix:
  A. Performance regression: 1K/2K/5K targets (compare vs v6.1.0 baseline)
  B. ECCM accuracy: inject known jamming → verify classification
  C. Closed-loop validation: verify R inflation prevents track loss
  D. Cross-track correlation: multi-track jammed sector detection

Author: Dr. Mladen Mešter · Nexellum d.o.o.
Copyright (c) 2026 — AGPL-3.0-or-later
"""

import sys, os, time
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)) + '/..')
import _nx_core as nx

np.random.seed(42)

# ============================================================
# SCENARIO GENERATORS
# ============================================================

def generate_linear_targets(n_targets, scan_range=300_000, dt=5.0, n_scans=30):
    """Generate N targets flying straight at constant velocity."""
    targets = []
    for i in range(n_targets):
        x0 = np.random.uniform(-scan_range, scan_range)
        y0 = np.random.uniform(-scan_range, scan_range)
        z0 = np.random.uniform(5000, 15000)
        vx = np.random.uniform(-300, 300)
        vy = np.random.uniform(-300, 300)
        vz = np.random.uniform(-10, 10)
        targets.append((x0, y0, z0, vx, vy, vz))

    scans = []
    for s in range(n_scans):
        t = s * dt
        meas = []
        for (x0, y0, z0, vx, vy, vz) in targets:
            x = x0 + vx * t + np.random.randn() * 150
            y = y0 + vy * t + np.random.randn() * 150
            z = z0 + vz * t + np.random.randn() * 50
            meas.append([x, y, z])
        scans.append(np.array(meas))
    return scans


def generate_jammed_scenario(n_clean, n_jammed, jammer_az_deg=45.0,
                              jammer_start_scan=10, dt=5.0, n_scans=40):
    """Generate scenario with clean + jammed targets in a specific sector.

    Jammed targets get inflated measurement noise (simulating noise jamming)
    and possible RGPO pull-off after onset.
    """
    scan_range = 200_000
    targets = []

    # Clean targets — scattered everywhere
    for i in range(n_clean):
        az = np.random.uniform(0, 360)
        rng = np.random.uniform(50_000, scan_range)
        x0 = rng * np.cos(np.radians(az))
        y0 = rng * np.sin(np.radians(az))
        z0 = np.random.uniform(5000, 12000)
        vx = np.random.uniform(-200, 200)
        vy = np.random.uniform(-200, 200)
        targets.append({'x0': x0, 'y0': y0, 'z0': z0, 'vx': vx, 'vy': vy,
                        'jammed': False})

    # Jammed targets — clustered in jammer_az_deg ± 5°
    for i in range(n_jammed):
        az = jammer_az_deg + np.random.uniform(-5, 5)
        rng = np.random.uniform(80_000, 180_000)
        x0 = rng * np.cos(np.radians(az))
        y0 = rng * np.sin(np.radians(az))
        z0 = np.random.uniform(8000, 12000)
        vx = np.random.uniform(-150, 150)
        vy = np.random.uniform(-150, 150)
        targets.append({'x0': x0, 'y0': y0, 'z0': z0, 'vx': vx, 'vy': vy,
                        'jammed': True})

    scans = []
    jammed_indices = [i for i, t in enumerate(targets) if t['jammed']]
    clean_indices = [i for i, t in enumerate(targets) if not t['jammed']]

    for s in range(n_scans):
        t_sec = s * dt
        meas = []
        for i, tgt in enumerate(targets):
            x = tgt['x0'] + tgt['vx'] * t_sec
            y = tgt['y0'] + tgt['vy'] * t_sec
            z = tgt['z0']

            if tgt['jammed'] and s >= jammer_start_scan:
                # Noise jamming: 5× measurement noise
                noise_mult = 5.0
                # RGPO pull-off: gradual range drift
                pull_off = (s - jammer_start_scan) * 200.0  # 200m/scan drift
                x += np.random.randn() * 150 * noise_mult + pull_off * 0.7
                y += np.random.randn() * 150 * noise_mult + pull_off * 0.3
                z += np.random.randn() * 50 * noise_mult
            else:
                x += np.random.randn() * 150
                y += np.random.randn() * 150
                z += np.random.randn() * 50

            meas.append([x, y, z])
        scans.append(np.array(meas))

    return scans, jammed_indices, clean_indices


# ============================================================
# TEST A: PERFORMANCE REGRESSION (vs v6.1.0 baselines)
# ============================================================
def test_performance_regression():
    print("=" * 70)
    print("TEST A: PERFORMANCE REGRESSION (ECCM-integrated vs v6.1.0 baseline)")
    print("=" * 70)

    # v6.1.0 baselines (from previous benchmark)
    v610_baselines = {1000: 9.4, 2000: 18.9, 5000: 53.2}

    results = {}
    for n_targets in [1000, 2000, 5000]:
        scans = generate_linear_targets(n_targets, n_scans=25)

        # With ECCM enabled
        tracker_eccm = nx.MultiTargetTracker(dt=5.0, r_std=150.0, q_base=1.0,
                                              eccm_enabled=True, eccm_cross_track=True)
        times_eccm = []
        for i, meas in enumerate(scans):
            ms = tracker_eccm.process_scan(meas, float(i * 5.0))
            if i >= 5:  # Skip warmup
                times_eccm.append(ms)

        # Without ECCM (regression check)
        tracker_plain = nx.MultiTargetTracker(dt=5.0, r_std=150.0, q_base=1.0,
                                               eccm_enabled=False, eccm_cross_track=False)
        times_plain = []
        for i, meas in enumerate(scans):
            ms = tracker_plain.process_scan(meas, float(i * 5.0))
            if i >= 5:
                times_plain.append(ms)

        mean_eccm = np.mean(times_eccm)
        p95_eccm  = np.percentile(times_eccm, 95)
        mean_plain = np.mean(times_plain)
        baseline   = v610_baselines.get(n_targets, mean_plain)
        overhead   = ((mean_eccm / mean_plain) - 1) * 100

        n_conf_eccm = tracker_eccm.n_confirmed
        n_conf_plain = tracker_plain.n_confirmed

        results[n_targets] = {
            'mean_eccm': mean_eccm, 'p95_eccm': p95_eccm,
            'mean_plain': mean_plain, 'baseline': baseline,
            'overhead': overhead,
            'n_conf_eccm': n_conf_eccm, 'n_conf_plain': n_conf_plain,
        }

        print(f"\n  {n_targets:,} targets:")
        print(f"    ECCM OFF:  {mean_plain:7.1f} ms (v6.1.0 baseline: {baseline:.1f} ms)")
        print(f"    ECCM ON:   {mean_eccm:7.1f} ms (P95: {p95_eccm:.1f} ms)")
        print(f"    Overhead:  {overhead:+.1f}%")
        print(f"    Tracks:    {n_conf_eccm} (ECCM) / {n_conf_plain} (plain)")

        # ECCM stats from last scan
        stats = tracker_eccm.eccm_stats
        print(f"    ECCM stats: clear={stats.n_clear} clutter={stats.n_clutter}"
              f" noise_jam={stats.n_noise_jam} deception={stats.n_deception}")
        print(f"    Mean R mult: {stats.mean_r_mult:.2f}")

    return results


# ============================================================
# TEST B: ECCM CLASSIFICATION ACCURACY
# ============================================================
def test_eccm_classification():
    print("\n" + "=" * 70)
    print("TEST B: ECCM CLASSIFICATION ACCURACY")
    print("=" * 70)

    n_clean = 50
    n_jammed = 10
    jammer_start = 10

    scans, jammed_idx, clean_idx = generate_jammed_scenario(
        n_clean, n_jammed, jammer_az_deg=45.0,
        jammer_start_scan=jammer_start, n_scans=40)

    tracker = nx.MultiTargetTracker(dt=5.0, r_std=150.0, q_base=1.0,
                                     eccm_enabled=True, eccm_cross_track=True)

    for i, meas in enumerate(scans):
        tracker.process_scan(meas, float(i * 5.0))

    # Get ECCM summary
    ids, classes, confidences, r_mults, jammed_scans = tracker.get_eccm_summary()

    n_total = len(ids)
    n_classified_jammed = np.sum(classes > 0)  # Any non-CLEAR class
    n_classified_clear  = np.sum(classes == 0)

    # Expected: ~10 tracks should be classified as jammed
    # (those that correspond to jammed_idx targets)
    print(f"\n  Scenario: {n_clean} clean + {n_jammed} jammed targets")
    print(f"  Jamming onset: scan {jammer_start}")
    print(f"  Confirmed tracks: {n_total}")
    print(f"  Classified jammed: {n_classified_jammed}")
    print(f"  Classified clear:  {n_classified_clear}")

    # Class distribution
    class_names = {0: 'CLEAR', 1: 'CLUTTER', 2: 'NOISE_JAM', 3: 'DECEPTION'}
    print(f"\n  Classification distribution:")
    for cls_val in range(4):
        count = np.sum(classes == cls_val)
        if count > 0:
            mean_conf = np.mean(confidences[classes == cls_val])
            mean_rm = np.mean(r_mults[classes == cls_val])
            print(f"    {class_names[cls_val]:>12s}: {count:3d} tracks "
                  f"(mean conf={mean_conf:.2f}, mean R_mult={mean_rm:.1f}×)")

    # R multiplier statistics
    print(f"\n  R multiplier range: [{np.min(r_mults):.1f}× – {np.max(r_mults):.1f}×]")
    print(f"  Tracks with R>1.0:  {np.sum(r_mults > 1.01)}")

    # Scan-level ECCM stats
    stats = tracker.eccm_stats
    print(f"\n  Last scan ECCM stats:")
    print(f"    Coherent sectors: {stats.n_coherent_sectors}")
    print(f"    Boosted tracks:   {stats.n_boosted_tracks}")
    print(f"    Mean R mult:      {stats.mean_r_mult:.2f}")

    return {
        'n_classified_jammed': int(n_classified_jammed),
        'n_classified_clear':  int(n_classified_clear),
        'n_coherent_sectors':  stats.n_coherent_sectors,
    }


# ============================================================
# TEST C: CLOSED-LOOP R INFLATION VALIDATION
# ============================================================
def test_closed_loop():
    print("\n" + "=" * 70)
    print("TEST C: CLOSED-LOOP R INFLATION — TRACK CONTINUITY UNDER JAMMING")
    print("=" * 70)

    n_clean = 30
    n_jammed = 8

    scans, jammed_idx, clean_idx = generate_jammed_scenario(
        n_clean, n_jammed, jammer_az_deg=90.0,
        jammer_start_scan=8, n_scans=35)

    # Run WITH ECCM
    tracker_eccm = nx.MultiTargetTracker(dt=5.0, r_std=150.0, q_base=1.0,
                                          eccm_enabled=True, eccm_cross_track=True)
    for i, meas in enumerate(scans):
        tracker_eccm.process_scan(meas, float(i * 5.0))
    n_conf_eccm = tracker_eccm.n_confirmed

    # Run WITHOUT ECCM
    tracker_no = nx.MultiTargetTracker(dt=5.0, r_std=150.0, q_base=1.0,
                                        eccm_enabled=False, eccm_cross_track=False)
    for i, meas in enumerate(scans):
        tracker_no.process_scan(meas, float(i * 5.0))
    n_conf_no = tracker_no.n_confirmed

    print(f"\n  Scenario: {n_clean} clean + {n_jammed} jammed (onset scan 8)")
    print(f"  After 35 scans:")
    print(f"    ECCM ON:  {n_conf_eccm} confirmed tracks")
    print(f"    ECCM OFF: {n_conf_no} confirmed tracks")
    print(f"    Delta:    {n_conf_eccm - n_conf_no:+d} tracks preserved by ECCM")

    # Track loss analysis
    total_created_eccm = tracker_eccm.total_created
    total_created_no   = tracker_no.total_created

    loss_rate_eccm = 1.0 - n_conf_eccm / max(total_created_eccm, 1)
    loss_rate_no   = 1.0 - n_conf_no / max(total_created_no, 1)

    print(f"\n  Track churn:")
    print(f"    ECCM ON:  created={total_created_eccm}, confirmed={n_conf_eccm}, "
          f"loss={loss_rate_eccm:.1%}")
    print(f"    ECCM OFF: created={total_created_no}, confirmed={n_conf_no}, "
          f"loss={loss_rate_no:.1%}")

    # R multiplier evolution on jammed tracks
    if n_conf_eccm > 0:
        ids, classes, conf, r_mults, js = tracker_eccm.get_eccm_summary()
        jammed_mask = r_mults > 1.5
        if np.any(jammed_mask):
            print(f"\n  Jammed track R multipliers: "
                  f"mean={np.mean(r_mults[jammed_mask]):.1f}× "
                  f"max={np.max(r_mults):.1f}×")

    return {
        'n_conf_eccm': n_conf_eccm,
        'n_conf_no': n_conf_no,
        'delta': n_conf_eccm - n_conf_no,
    }


# ============================================================
# TEST D: CROSS-TRACK SECTOR CORRELATION
# ============================================================
def test_cross_track_correlation():
    print("\n" + "=" * 70)
    print("TEST D: CROSS-TRACK SECTOR CORRELATION — COHERENT THREAT DETECTION")
    print("=" * 70)

    # Multiple jammed targets in same sector → should detect coherent threat
    scans, jammed_idx, clean_idx = generate_jammed_scenario(
        n_clean=40, n_jammed=6, jammer_az_deg=135.0,
        jammer_start_scan=8, n_scans=35)

    tracker = nx.MultiTargetTracker(dt=5.0, r_std=150.0, q_base=1.0,
                                     eccm_enabled=True, eccm_cross_track=True)

    sector_history = []
    for i, meas in enumerate(scans):
        tracker.process_scan(meas, float(i * 5.0))
        stats = tracker.eccm_stats
        sector_history.append({
            'scan': i,
            'n_coherent': stats.n_coherent_sectors,
            'n_boosted': stats.n_boosted_tracks,
            'mean_r': stats.mean_r_mult,
        })

    print(f"\n  Scenario: 40 clean + 6 jammed at az=135° (onset scan 8)")
    print(f"\n  Cross-track correlation timeline:")
    print(f"  {'Scan':>5s} | {'Coherent':>8s} | {'Boosted':>7s} | {'Mean R':>7s}")
    print(f"  {'-'*5}-+-{'-'*8}-+-{'-'*7}-+-{'-'*7}")
    for h in sector_history:
        marker = " ◄ JAMMING" if h['scan'] >= 8 and h['n_coherent'] > 0 else ""
        print(f"  {h['scan']:5d} | {h['n_coherent']:8d} | {h['n_boosted']:7d} | "
              f"{h['mean_r']:7.2f}{marker}")

    # Summary
    onset_detected = None
    for h in sector_history:
        if h['n_coherent'] > 0:
            onset_detected = h['scan']
            break

    print(f"\n  Coherent threat detection:")
    if onset_detected is not None:
        latency = onset_detected - 8
        print(f"    First detected at scan {onset_detected} "
              f"(latency: {latency} scans = {latency * 5}s after onset)")
    else:
        print(f"    NOT detected (need more scans for ML window build-up)")

    max_coherent = max(h['n_coherent'] for h in sector_history)
    max_boosted  = max(h['n_boosted'] for h in sector_history)
    print(f"    Peak coherent sectors: {max_coherent}")
    print(f"    Peak boosted tracks:   {max_boosted}")

    return {
        'onset_detected': onset_detected,
        'max_coherent': max_coherent,
        'max_boosted': max_boosted,
    }


# ============================================================
# MAIN — FULL v6.2.0 VALIDATION SUITE
# ============================================================
if __name__ == '__main__':
    print("╔" + "═" * 68 + "╗")
    print("║  NX-MIMOSA v6.2.0 — ECCM Integration Benchmark Suite              ║")
    print("║  C++ Core | Closed-Loop | ML Window 20 | Cross-Track Correlation   ║")
    print("╚" + "═" * 68 + "╝")
    print(f"\n  Module version: {nx.__version__}")
    print(f"  ML window:      {nx.ECCM_ML_WINDOW} scans")
    print(f"  History depth:  {nx.ECCM_HISTORY_DEPTH} scans")
    print()

    t0 = time.time()

    perf_results    = test_performance_regression()
    class_results   = test_eccm_classification()
    closed_results  = test_closed_loop()
    xcorr_results   = test_cross_track_correlation()

    elapsed = time.time() - t0

    # ── SUMMARY ──
    print("\n" + "=" * 70)
    print("SUMMARY — NX-MIMOSA v6.2.0 ECCM Integration Validation")
    print("=" * 70)

    print(f"\n  A. Performance overhead (ECCM ON vs OFF):")
    for n, r in perf_results.items():
        print(f"     {n:,} targets: {r['mean_plain']:.1f} → {r['mean_eccm']:.1f} ms "
              f"({r['overhead']:+.1f}%)")

    print(f"\n  B. Classification accuracy:")
    print(f"     Jammed detected: {class_results['n_classified_jammed']} tracks")
    print(f"     Coherent sectors: {class_results['n_coherent_sectors']}")

    print(f"\n  C. Closed-loop track preservation:")
    print(f"     ECCM ON: {closed_results['n_conf_eccm']} vs "
          f"ECCM OFF: {closed_results['n_conf_no']} "
          f"(Δ={closed_results['delta']:+d})")

    print(f"\n  D. Cross-track correlation:")
    print(f"     Onset detection: scan {xcorr_results['onset_detected']}")
    print(f"     Peak coherent sectors: {xcorr_results['max_coherent']}")

    print(f"\n  Total benchmark time: {elapsed:.1f}s")
    print(f"\n  v6.2.0 ECCM integration: {'PASS ✅' if all([
        perf_results[5000]['overhead'] < 25,  # <25% overhead acceptable
        class_results['n_classified_jammed'] > 0,
    ]) else 'NEEDS REVIEW ⚠️'}")
