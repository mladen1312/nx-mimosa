#!/usr/bin/env python3
"""
NX-MIMOSA v4.0 SENTINEL — Comprehensive Benchmark
====================================================
Tests Platform-Aware VS-IMM against:
  1. NX-MIMOSA v3.3 (our previous best)
  2. Stone Soup (UK DSTL gold standard)
  3. FilterPy (community standard)

Scenarios with known platform types for intent verification.

Author: Dr. Mladen Mešter / Nexellum d.o.o.
License: AGPL v3 | Commercial available
"""

import numpy as np
import sys
import os
import time

# Add parent to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'python'))


# =============================================================================
# Scenario Generator — Platform-Aware
# =============================================================================
def generate_scenario(name: str, dt: float, N: int, seed: int = 42):
    """Generate ground truth trajectory for named scenario."""
    rng = np.random.RandomState(seed)

    if name == "F16_Dogfight":
        # 9g BFM engagement
        speed = 250.0
        states = []
        x, y, vx, vy = 0, 0, speed, 0
        for k in range(N):
            t = k * dt
            if t < 3:
                omega = 0  # Straight approach
            elif t < 7:
                omega = 0.35  # Hard right turn (9g)
            elif t < 9:
                omega = -0.35  # Break left
            elif t < 12:
                omega = 0.25  # Sustained turn
            else:
                omega = 0  # Egress
            ax = -omega * vy
            ay = omega * vx
            vx += ax * dt
            vy += ay * dt
            x += vx * dt
            y += vy * dt
            states.append([x, y, vx, vy])
        return np.array(states), "fighter", 9.0

    elif name == "Kinzhal_Glide":
        # Hypersonic glide vehicle — Mach 5+, mild maneuvers
        speed = 1700.0  # ~M5
        states = []
        x, y = 0, 50000  # Start at 50km altitude
        vx, vy = speed, -200  # Descending
        for k in range(N):
            t = k * dt
            # Gradual descent with periodic adjustments
            ax = 0
            ay = -9.81 + 5.0 * np.sin(0.2 * t)  # Gravity + lift modulation
            if 5 < t < 8:
                ax = -50.0  # Terminal maneuver
                ay = -100.0
            vx += ax * dt
            vy += ay * dt
            x += vx * dt
            y += vy * dt
            states.append([x, y, vx, vy])
        return np.array(states), "hypersonic_missile", 25.0

    elif name == "Iskander_Terminal":
        # Ballistic missile terminal phase — high speed, terminal dive
        speed = 2000.0
        states = []
        x, y = 0, 40000
        vx, vy = speed * 0.7, -speed * 0.7
        for k in range(N):
            t = k * dt
            ax = 0
            ay = -9.81  # Pure ballistic
            if t > N * dt * 0.7:
                # Terminal maneuver (skip-glide)
                ax = 100 * np.sin(3 * t)
                ay = -9.81 + 150 * np.cos(3 * t)
            vx += ax * dt
            vy += ay * dt
            x += vx * dt
            y += vy * dt
            states.append([x, y, vx, vy])
        return np.array(states), "ballistic_missile", 20.0

    elif name == "Kalibr_Cruise":
        # Cruise missile with waypoint turns
        speed = 250.0
        states = []
        x, y, vx, vy = 0, 0, speed, 0
        for k in range(N):
            t = k * dt
            if t < 5:
                omega = 0
            elif t < 7:
                omega = 0.12  # Waypoint turn
            elif t < 12:
                omega = 0
            elif t < 14:
                omega = -0.10  # Another waypoint
            elif t < 18:
                omega = 0
            else:
                omega = 0.05  # Terminal approach correction
            ax = -omega * vy
            ay = omega * vx
            vx += ax * dt
            vy += ay * dt
            x += vx * dt
            y += vy * dt
            states.append([x, y, vx, vy])
        return np.array(states), "cruise_missile", 5.0

    elif name == "Su35_PostStall":
        # Su-35 cobra maneuver + high-alpha
        speed = 200.0
        states = []
        x, y, vx, vy = 0, 0, speed, 0
        for k in range(N):
            t = k * dt
            if t < 2:
                omega = 0
            elif t < 3:
                omega = 0.40  # Pull up into cobra
            elif t < 4:
                # Near stall — jerk-dominated
                omega = -0.3 + 0.8 * np.sin(5 * t)
            elif t < 6:
                omega = -0.35  # Recovery turn
            elif t < 8:
                omega = 0.25  # Re-engage
            else:
                omega = 0
            ax = -omega * vy + rng.normal(0, 5)  # turbulence
            ay = omega * vx + rng.normal(0, 5)
            vx += ax * dt
            vy += ay * dt
            sp = np.sqrt(vx**2 + vy**2)
            if sp < 50:  # Min speed clamp (stall protection)
                vx *= 50/sp; vy *= 50/sp
            x += vx * dt
            y += vy * dt
            states.append([x, y, vx, vy])
        return np.array(states), "fighter", 9.5

    elif name == "SAM_Terminal":
        # S-300 class SAM — high-g proportional navigation
        speed = 1200.0
        states = []
        x, y, vx, vy = 0, 0, speed * 0.8, speed * 0.6
        for k in range(N):
            t = k * dt
            if t < 3:
                omega = 0  # Boost/midcourse
            elif t < 5:
                omega = 0.15  # Course correction
            elif t < 8:
                omega = 0.25  # Terminal homing — high g
            elif t < 10:
                omega = -0.20  # Last-second correction
            else:
                omega = 0
            ax = -omega * vy
            ay = omega * vx
            vx += ax * dt
            vy += ay * dt
            x += vx * dt
            y += vy * dt
            states.append([x, y, vx, vy])
        return np.array(states), "sam", 25.0

    elif name == "Shahed_Loiter":
        # Shahed-136 loitering munition — slow, predictable, then terminal dive
        speed = 46.0
        states = []
        x, y, vx, vy = 0, 0, speed, 0
        for k in range(N):
            t = k * dt
            if t < 10:
                omega = 0  # Straight cruise
            elif t < 15:
                omega = 0.03  # Gentle course correction
            elif t < 18:
                omega = 0  # Final approach
            else:
                # Terminal dive
                omega = 0
                vy -= 5.0 * dt  # Dive acceleration
            ax = -omega * vy
            ay = omega * vx
            vx += ax * dt
            vy += ay * dt
            x += vx * dt
            y += vy * dt
            states.append([x, y, vx, vy])
        return np.array(states), "loitering_munition", 3.0

    elif name == "FPV_Attack":
        # FPV kamikaze drone — erratic, fast direction changes
        speed = 25.0
        states = []
        x, y, vx, vy = 0, 0, speed, 0
        for k in range(N):
            t = k * dt
            if t < 3:
                omega = 0.5 * np.sin(2 * t)  # Search pattern
            elif t < 6:
                omega = -0.8  # Spotted target, approach
            elif t < 8:
                omega = 1.2 * np.sin(8 * t)  # Evasive jinking
            else:
                omega = 0  # Final dive
                vy -= 3.0 * dt
            ax = -omega * vy + rng.normal(0, 3)
            ay = omega * vx + rng.normal(0, 3)
            vx += ax * dt
            vy += ay * dt
            sp = np.sqrt(vx**2 + vy**2)
            if sp > 40: vx *= 40/sp; vy *= 40/sp
            if sp < 5: vx *= 5/sp; vy *= 5/sp
            x += vx * dt
            y += vy * dt
            states.append([x, y, vx, vy])
        return np.array(states), "fpv_kamikaze", 4.0

    else:
        raise ValueError(f"Unknown scenario: {name}")


# =============================================================================
# NX-MIMOSA v4.0 SENTINEL Runner
# =============================================================================
def run_v40_sentinel(measurements, dt, r_std, platform_db_path=None):
    """Run v4.0 tracker and return estimates + intent log."""
    from nx_mimosa_v40_sentinel import NxMimosaV40Sentinel

    tracker = NxMimosaV40Sentinel(
        dt=dt, r_std=r_std,
        platform_db_path=platform_db_path,
        window_size=30, prune_threshold=0.03
    )

    forward_est = []
    intent_log = []

    for z in measurements:
        x_rt, P_rt, intent = tracker.update(z)
        forward_est.append(x_rt.copy())
        intent_log.append({
            'platform': intent.platform_type,
            'category': intent.platform_category,
            'confidence': intent.confidence,
            'phase': intent.phase,
            'threat': intent.threat_level,
            'n_models': intent.n_active_models,
            'active': list(intent.active_models),
        })

    window_est = tracker.get_window_smoothed_estimates(30)
    full_est = tracker.get_smoothed_estimates()
    best_est, best_stream = tracker.get_best_estimates()

    return forward_est, window_est, full_est, intent_log, best_est, best_stream


# =============================================================================
# v3.3 Baseline Runner
# =============================================================================
def run_v33_baseline(measurements, dt, r_std):
    """Run v3.3 dual-mode for comparison."""
    try:
        from nx_mimosa_v33_dual_mode import NxMimosaV33DualMode
        tracker = NxMimosaV33DualMode(dt=dt, r_std=r_std)
        for z in measurements:
            tracker.update(z)
        fwd = tracker.get_forward_estimates()
        win = tracker.get_window_smoothed_estimates(30)
        full = tracker.get_smoothed_estimates()
        return fwd, win, full
    except ImportError:
        return None, None, None


# =============================================================================
# Stone Soup / FilterPy Runners
# =============================================================================
def run_stonesoup(measurements, dt, r_std):
    """Run Stone Soup CV+Smoother for comparison."""
    try:
        from stonesoup.models.transition.linear import CombinedLinearGaussianTransitionModel, \
            ConstantVelocity
        from stonesoup.models.measurement.linear import LinearGaussian
        from stonesoup.predictor.kalman import KalmanPredictor
        from stonesoup.updater.kalman import KalmanUpdater
        from stonesoup.smoother.kalman import KalmanSmoother
        from stonesoup.types.state import GaussianState
        from stonesoup.types.detection import Detection
        from stonesoup.types.array import StateVector, CovarianceMatrix
        import datetime

        transition = CombinedLinearGaussianTransitionModel([
            ConstantVelocity(1.0), ConstantVelocity(1.0)
        ])
        meas_model = LinearGaussian(ndim_state=4, mapping=(0, 2),
                                     noise_covar=np.eye(2) * r_std**2)
        predictor = KalmanPredictor(transition)
        updater = KalmanUpdater(meas_model)
        smoother = KalmanSmoother(transition)

        t0 = datetime.datetime.now()
        prior = GaussianState(
            StateVector([[measurements[0][0]], [0], [measurements[0][1]], [0]]),
            CovarianceMatrix(np.eye(4) * 100),
            timestamp=t0
        )

        track = []
        state = prior
        for k, z in enumerate(measurements):
            timestamp = t0 + datetime.timedelta(seconds=(k+1)*dt)
            det = Detection(StateVector([[z[0]], [z[1]]]),
                           timestamp=timestamp, measurement_model=meas_model)
            pred = predictor.predict(state, timestamp=timestamp)
            state = updater.update(pred, det)
            track.append(state)

        forward_est = [np.array([s.state_vector[0, 0], s.state_vector[2, 0]]) for s in track]
        smoothed_track = smoother.smooth(track)
        smooth_est = [np.array([s.state_vector[0, 0], s.state_vector[2, 0]]) for s in smoothed_track]

        return forward_est, smooth_est
    except Exception:
        return None, None


def run_filterpy(measurements, dt, r_std):
    """Run FilterPy IMM for comparison."""
    try:
        from filterpy.kalman import IMMEstimator, KalmanFilter
        # CV filter
        kf_cv = KalmanFilter(dim_x=4, dim_z=2)
        kf_cv.F = np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]])
        kf_cv.H = np.array([[1,0,0,0],[0,1,0,0]])
        kf_cv.R = np.eye(2) * r_std**2
        kf_cv.Q = 0.5 * np.array([
            [dt**4/4,0,dt**3/2,0],[0,dt**4/4,0,dt**3/2],
            [dt**3/2,0,dt**2,0],[0,dt**3/2,0,dt**2]])
        kf_cv.P *= 100
        # CA filter
        kf_ca = KalmanFilter(dim_x=4, dim_z=2)
        kf_ca.F = kf_cv.F.copy()
        kf_ca.H = kf_cv.H.copy()
        kf_ca.R = kf_cv.R.copy()
        kf_ca.Q = kf_cv.Q * 5.0
        kf_ca.P *= 100

        M = np.array([[0.95, 0.05], [0.05, 0.95]])
        mu = np.array([0.5, 0.5])
        imm = IMMEstimator([kf_cv, kf_ca], mu, M)

        estimates = []
        for z in measurements:
            imm.predict()
            imm.update(z)
            estimates.append(imm.x[:2, 0].copy())

        return estimates
    except Exception:
        return None


# =============================================================================
# RMSE Computation
# =============================================================================
def compute_rmse(truth, estimates, skip=5):
    """Compute position RMSE, skipping initial transient."""
    truth = np.array(truth)
    estimates = np.array(estimates)
    n = min(len(truth), len(estimates))
    if n <= skip:
        return float('inf')
    errors = truth[skip:n, :2] - estimates[skip:n, :2]
    return np.sqrt(np.mean(np.sum(errors**2, axis=1)))


# =============================================================================
# Main Benchmark
# =============================================================================
def main():
    print("=" * 80)
    print("  NX-MIMOSA v4.0 SENTINEL — Platform-Aware VS-IMM Benchmark")
    print("  50 Monte Carlo runs per scenario")
    print("=" * 80)
    print()

    dt = 0.1
    r_std = 2.5
    N = 200  # 20 seconds
    n_runs = 50

    scenarios = [
        "F16_Dogfight", "Kinzhal_Glide", "Iskander_Terminal",
        "Kalibr_Cruise", "Su35_PostStall", "SAM_Terminal",
        "Shahed_Loiter", "FPV_Attack"
    ]

    # Platform DB path
    db_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data', 'platform_db.json')
    if not os.path.exists(db_path):
        db_path = None
        print("WARNING: platform_db.json not found, using fallback")

    results = {}

    for scenario in scenarios:
        print(f"\n{'─'*60}")
        print(f"  Scenario: {scenario}")
        print(f"{'─'*60}")

        rmse_v40_fwd = []
        rmse_v40_win = []
        rmse_v40_full = []
        rmse_v40_best = []
        rmse_v33_fwd = []
        rmse_v33_win = []
        rmse_ss = []
        rmse_fp = []
        intent_correct = 0
        platform_ids = {}
        avg_n_models = []
        stream_picks = {}

        for run in range(n_runs):
            seed = 42 + run
            truth, true_category, max_g = generate_scenario(scenario, dt, N, seed=seed)
            rng = np.random.RandomState(seed + 10000)
            measurements = truth[:, :2] + rng.normal(0, r_std, (N, 2))

            # v4.0 SENTINEL
            try:
                fwd, win, full, intent_log, best, best_stream = run_v40_sentinel(
                    measurements, dt, r_std, db_path)
                rmse_v40_fwd.append(compute_rmse(truth, fwd))
                rmse_v40_win.append(compute_rmse(truth, win))
                rmse_v40_full.append(compute_rmse(truth, full))
                rmse_v40_best.append(compute_rmse(truth, best))
                stream_picks[best_stream] = stream_picks.get(best_stream, 0) + 1

                # Check intent accuracy
                if intent_log:
                    last = intent_log[-1]
                    avg_n_models.append(last['n_models'])
                    plat = last['platform']
                    platform_ids[plat] = platform_ids.get(plat, 0) + 1
                    if last['category'] == true_category:
                        intent_correct += 1
            except Exception as e:
                if run == 0:
                    print(f"  v4.0 error: {e}")
                rmse_v40_fwd.append(float('inf'))
                rmse_v40_win.append(float('inf'))
                rmse_v40_full.append(float('inf'))

            # v3.3 baseline
            try:
                fwd33, win33, full33 = run_v33_baseline(measurements, dt, r_std)
                if fwd33:
                    rmse_v33_fwd.append(compute_rmse(truth, fwd33))
                    rmse_v33_win.append(compute_rmse(truth, win33))
            except Exception:
                pass

            # Stone Soup
            try:
                ss_fwd, ss_smooth = run_stonesoup(measurements, dt, r_std)
                if ss_fwd:
                    rmse_ss.append(compute_rmse(truth, ss_smooth or ss_fwd))
            except Exception:
                pass

            # FilterPy
            try:
                fp_est = run_filterpy(measurements, dt, r_std)
                if fp_est:
                    rmse_fp.append(compute_rmse(truth, fp_est))
            except Exception:
                pass

        # Results
        def fmt(arr): return f"{np.mean(arr):.2f}" if arr and np.isfinite(np.mean(arr)) else "N/A"

        print(f"  v4.0 Forward:   {fmt(rmse_v40_fwd)} m")
        print(f"  v4.0 Window-30: {fmt(rmse_v40_win)} m")
        print(f"  v4.0 Full:      {fmt(rmse_v40_full)} m")
        print(f"  v4.0 BEST:      {fmt(rmse_v40_best)} m  ← adaptive stream")
        if rmse_v33_fwd:
            print(f"  v3.3 Forward:   {fmt(rmse_v33_fwd)} m")
            print(f"  v3.3 Window-30: {fmt(rmse_v33_win)} m")
        if rmse_ss:
            print(f"  Stone Soup:     {fmt(rmse_ss)} m")
        if rmse_fp:
            print(f"  FilterPy:       {fmt(rmse_fp)} m")

        print(f"  Intent category accuracy: {intent_correct}/{n_runs} ({100*intent_correct/n_runs:.0f}%)")
        print(f"  Platform IDs: {dict(sorted(platform_ids.items(), key=lambda x:-x[1]))}")
        print(f"  Avg active models: {np.mean(avg_n_models):.1f}" if avg_n_models else "")
        print(f"  Stream picks: {stream_picks}")

        results[scenario] = {
            'v40_fwd': np.mean(rmse_v40_fwd) if rmse_v40_fwd else float('inf'),
            'v40_win': np.mean(rmse_v40_win) if rmse_v40_win else float('inf'),
            'v40_full': np.mean(rmse_v40_full) if rmse_v40_full else float('inf'),
            'v40_best': np.mean(rmse_v40_best) if rmse_v40_best else float('inf'),
            'v33_fwd': np.mean(rmse_v33_fwd) if rmse_v33_fwd else float('inf'),
            'v33_win': np.mean(rmse_v33_win) if rmse_v33_win else float('inf'),
            'ss': np.mean(rmse_ss) if rmse_ss else float('inf'),
            'fp': np.mean(rmse_fp) if rmse_fp else float('inf'),
            'intent_acc': intent_correct / n_runs,
        }

    # =========================================================================
    # Summary Table
    # =========================================================================
    print("\n" + "=" * 80)
    print("  SUMMARY — v4.0 SENTINEL vs All")
    print("=" * 80)
    print(f"{'Scenario':<22} {'v4.0 BEST':>10} {'v4.0 Fwd':>10} {'v4.0 W30':>10} {'FilterPy':>10} {'Intent%':>8}")
    print("─" * 80)

    v40_wins = 0
    total = 0
    for sc in scenarios:
        r = results[sc]
        competitors = [r['v33_win'], r['ss'], r['fp']]
        competitors = [c for c in competitors if np.isfinite(c)]
        best_comp = min(competitors) if competitors else float('inf')
        winner = "✅" if r['v40_best'] < best_comp else "  "

        if np.isfinite(r['v40_best']) and competitors:
            total += 1
            if r['v40_best'] <= best_comp:
                v40_wins += 1

        def fv(v): return f"{v:.2f}" if np.isfinite(v) else "N/A"

        print(f"{sc:<22} {fv(r['v40_best']):>10} {fv(r['v40_fwd']):>10} "
              f"{fv(r['v40_win']):>10} {fv(r['fp']):>10} "
              f"{r['intent_acc']*100:>7.0f}% {winner}")

    print("─" * 80)
    print(f"  v4.0 SENTINEL wins: {v40_wins}/{total}")

    if total > 0:
        # Compute average improvements
        v40_avg = np.mean([results[s]['v40_best'] for s in scenarios if np.isfinite(results[s]['v40_best'])])
        ss_avg = np.mean([results[s]['ss'] for s in scenarios if np.isfinite(results[s]['ss'])]) if any(np.isfinite(results[s]['ss']) for s in scenarios) else float('inf')
        fp_avg = np.mean([results[s]['fp'] for s in scenarios if np.isfinite(results[s]['fp'])]) if any(np.isfinite(results[s]['fp']) for s in scenarios) else float('inf')

        print(f"\n  v4.0 Grand Average: {v40_avg:.2f} m")
        if np.isfinite(ss_avg):
            imp = (ss_avg - v40_avg) / ss_avg * 100
            print(f"  vs Stone Soup:      {imp:+.1f}%")
        if np.isfinite(fp_avg):
            imp = (fp_avg - v40_avg) / fp_avg * 100
            print(f"  vs FilterPy:        {imp:+.1f}%")

    print()


if __name__ == "__main__":
    main()
