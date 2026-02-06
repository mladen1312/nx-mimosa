#!/usr/bin/env python3
"""
NX-MIMOSA v4.2 OPEN BENCHMARK — Reproducible Multi-Library Comparison
======================================================================

Compares NX-MIMOSA v4.2 against three open-source tracking libraries:
  - Stone Soup v1.8  (UK DSTL)       https://github.com/dstl/Stone-Soup
  - FilterPy v1.4.5  (Roger Labbe)   https://github.com/rlabbe/filterpy
  - PyKalman v0.11.2  (D. Duckworth) https://github.com/pykalman/pykalman

FAIRNESS PRINCIPLES:
  1. FIXED SEED (42)         — identical truth + noise for ALL trackers
  2. SAME R_std = 5m         — measurement noise identical
  3. FAIR Q TUNING           — reasonable defaults, NOT sabotaged
  4. BEST-OF-CLASS reporting — Stone Soup picks BEST model per scenario
  5. MULTIPLE METRICS        — RMS, Max, P95, Mean, computation time
  6. NO ORACLE INFO          — no tracker gets true omega or acceleration
  7. STANDARD SCENARIOS      — from tracking literature, not cherry-picked

SCENARIOS (7 standard tracking challenges):
  1. Constant Velocity     — baseline, ~300 m/s straight line
  2. Gentle Turn (2 deg/s) — mild coordinated turn
  3. Hard Turn (8 deg/s)   — 4g fighter turn at 300 m/s
  4. Acceleration Burst    — 5g onset, 100 to 500 m/s
  5. Fighter Dogfight      — multi-segment: CV, 6g turn, CV, 4g reverse, CV
  6. Jinking Evasion       — +/-5 deg/s alternating turns every 3s
  7. Ballistic Arc         — parabolic trajectory with gravity (9.81 m/s^2)

HOW TO REPRODUCE:
  pip install stonesoup filterpy pykalman numpy
  cd nx-mimosa
  python benchmarks/open_benchmark.py

Author: Dr. Mladen Mester / Nexellum d.o.o.
License: MIT (benchmark code) — each library under its own license
"""

import numpy as np
import time
import sys
import os
import json
from dataclasses import dataclass, field
from typing import List, Optional
from datetime import datetime, timedelta

# ============================================================================
# GLOBAL PARAMETERS — identical for ALL trackers
# ============================================================================
SEED       = 42
DT         = 0.1      # 10 Hz update rate
R_STD      = 5.0      # Measurement noise sigma = 5m (both x and y)
Q_CV       = 0.5      # Process noise for CV models
Q_CA       = 2.0      # Process noise for CA models
Q_CT       = 1.0      # Process noise for CT models
Q_SINGER   = 2.0      # Singer spectral density

# ============================================================================
# TRUTH TRAJECTORY GENERATORS
# ============================================================================

def make_truth_cv(n, dt):
    """Scenario 1: Constant velocity — straight line at ~291 m/s heading NE."""
    vx, vy = 250.0, 150.0
    return np.array([[vx * i * dt, vy * i * dt] for i in range(n)])

def make_truth_gentle_turn(n, dt):
    """Scenario 2: Gentle coordinated turn at 2 deg/s, 300 m/s."""
    truth = np.zeros((n, 2))
    speed, omega, heading = 300.0, np.radians(2.0), 0.0
    x, y = 0.0, 0.0
    for i in range(n):
        truth[i] = [x, y]
        heading += omega * dt
        x += speed * np.cos(heading) * dt
        y += speed * np.sin(heading) * dt
    return truth

def make_truth_hard_turn(n, dt):
    """Scenario 3: Hard turn at 8 deg/s (~4.2g at 300 m/s)."""
    truth = np.zeros((n, 2))
    speed, omega, heading = 300.0, np.radians(8.0), 0.0
    x, y = 0.0, 0.0
    for i in range(n):
        truth[i] = [x, y]
        heading += omega * dt
        x += speed * np.cos(heading) * dt
        y += speed * np.sin(heading) * dt
    return truth

def make_truth_acceleration(n, dt):
    """Scenario 4: Acceleration burst — 100 to 500 m/s, 5g onset, heading East."""
    truth = np.zeros((n, 2))
    vx, ax = 100.0, 50.0
    x, y = 0.0, 5000.0
    for i in range(n):
        truth[i] = [x, y]
        x += vx * dt + 0.5 * ax * dt**2
        if i * dt < 8.0:
            vx += ax * dt
    return truth

def make_truth_dogfight(n, dt):
    """Scenario 5: Fighter dogfight — multi-segment realistic profile.
    0-10s: CV at 350 m/s NE | 10-18s: 6g break turn | 18-28s: CV
    28-35s: 4g reverse turn | 35-40s: CV recovery"""
    truth = np.zeros((n, 2))
    speed, heading = 350.0, np.radians(45)
    x, y = 0.0, 0.0
    for i in range(n):
        t = i * dt
        truth[i] = [x, y]
        if t < 10:       omega = 0.0
        elif t < 18:     omega = np.radians(10.5)
        elif t < 28:     omega = 0.0
        elif t < 35:     omega = np.radians(-7.0)
        else:            omega = 0.0
        heading += omega * dt
        x += speed * np.cos(heading) * dt
        y += speed * np.sin(heading) * dt
    return truth

def make_truth_jinking(n, dt):
    """Scenario 6: Jinking evasion — +/-5 deg/s alternating every 3s."""
    truth = np.zeros((n, 2))
    speed, heading = 280.0, 0.0
    x, y = 0.0, 0.0
    for i in range(n):
        t = i * dt
        truth[i] = [x, y]
        period = int(t / 3.0)
        omega = np.radians(5.0) if period % 2 == 0 else np.radians(-5.0)
        heading += omega * dt
        x += speed * np.cos(heading) * dt
        y += speed * np.sin(heading) * dt
    return truth

def make_truth_ballistic(n, dt):
    """Scenario 7: Ballistic arc — parabolic with gravity.
    Launch at 500 m/s, 60 deg elevation, x-y plane (y = up)."""
    truth = np.zeros((n, 2))
    vx = 500.0 * np.cos(np.radians(60))
    vy = 500.0 * np.sin(np.radians(60))
    g = 9.81
    x, y = 0.0, 0.0
    for i in range(n):
        truth[i] = [x, max(y, 0)]
        x += vx * dt
        y += vy * dt - 0.5 * g * dt**2
        vy -= g * dt
    return truth

# ============================================================================
# MEASUREMENT GENERATOR — shared noise for all trackers
# ============================================================================

def generate_measurements(truth, r_std, seed):
    rng = np.random.RandomState(seed)
    return truth + rng.randn(len(truth), 2) * r_std

# ============================================================================
# TRACKER: Stone Soup (UK DSTL)
# ============================================================================

def _stonesoup_run(measurements, dt, r_std, transition_model, ndim, pos_map):
    """Generic Stone Soup single-model Kalman filter runner."""
    from stonesoup.models.measurement.linear import LinearGaussian
    from stonesoup.predictor.kalman import KalmanPredictor
    from stonesoup.updater.kalman import KalmanUpdater
    from stonesoup.types.state import GaussianState
    from stonesoup.types.detection import Detection
    from stonesoup.types.hypothesis import SingleHypothesis
    from stonesoup.types.array import StateVector, CovarianceMatrix

    meas_model = LinearGaussian(ndim_state=ndim, mapping=pos_map,
                                 noise_covar=np.eye(2) * r_std**2)
    predictor = KalmanPredictor(transition_model)
    updater = KalmanUpdater(meas_model)

    z0 = measurements[0]
    x0 = np.zeros(ndim)
    x0[pos_map[0]] = z0[0]; x0[pos_map[1]] = z0[1]
    P0 = np.eye(ndim) * 100
    P0[pos_map[0], pos_map[0]] = r_std**2
    P0[pos_map[1], pos_map[1]] = r_std**2

    state = GaussianState(
        state_vector=StateVector(x0.reshape(-1, 1)),
        covar=CovarianceMatrix(P0), timestamp=datetime.now())

    t0 = datetime.now()
    estimates = []
    for i in range(1, len(measurements)):
        ts = t0 + timedelta(seconds=i * dt)
        det = Detection(state_vector=StateVector(measurements[i].reshape(-1, 1)),
                        timestamp=ts, measurement_model=meas_model)
        pred = predictor.predict(state, timestamp=ts)
        hyp = SingleHypothesis(pred, det)
        state = updater.update(hyp)
        sv = state.state_vector
        estimates.append([float(sv[pos_map[0]]), float(sv[pos_map[1]])])
    return np.array(estimates)

def run_ss_cv(meas, dt, r_std, q):
    from stonesoup.models.transition.linear import (
        CombinedLinearGaussianTransitionModel, ConstantVelocity)
    tm = CombinedLinearGaussianTransitionModel(
        [ConstantVelocity(q), ConstantVelocity(q)])
    return _stonesoup_run(meas, dt, r_std, tm, ndim=4, pos_map=(0, 2))

def run_ss_ca(meas, dt, r_std, q):
    from stonesoup.models.transition.linear import (
        CombinedLinearGaussianTransitionModel, ConstantAcceleration)
    tm = CombinedLinearGaussianTransitionModel(
        [ConstantAcceleration(q), ConstantAcceleration(q)])
    return _stonesoup_run(meas, dt, r_std, tm, ndim=6, pos_map=(0, 3))

def run_ss_singer(meas, dt, r_std, q):
    from stonesoup.models.transition.linear import (
        CombinedLinearGaussianTransitionModel, Singer)
    tm = CombinedLinearGaussianTransitionModel(
        [Singer(damping_coeff=1.0/3.0, noise_diff_coeff=q),
         Singer(damping_coeff=1.0/3.0, noise_diff_coeff=q)])
    return _stonesoup_run(meas, dt, r_std, tm, ndim=6, pos_map=(0, 3))

# ============================================================================
# TRACKER: FilterPy (Roger Labbe)
# ============================================================================

def _make_fp_cv(dt, r_std, q):
    from filterpy.kalman import KalmanFilter
    kf = KalmanFilter(dim_x=4, dim_z=2)
    kf.F = np.array([[1,dt,0,0],[0,1,0,0],[0,0,1,dt],[0,0,0,1]])
    kf.H = np.array([[1,0,0,0],[0,0,1,0]])
    kf.R = np.eye(2) * r_std**2
    kf.Q = np.array([[dt**3/3,dt**2/2,0,0],[dt**2/2,dt,0,0],
                      [0,0,dt**3/3,dt**2/2],[0,0,dt**2/2,dt]]) * q
    kf.P = np.diag([r_std**2, 100, r_std**2, 100])
    return kf

def _make_fp_ct(dt, r_std, q, omega):
    from filterpy.kalman import KalmanFilter
    kf = KalmanFilter(dim_x=4, dim_z=2)
    co, so = np.cos(omega*dt), np.sin(omega*dt)
    if abs(omega) > 1e-6:
        kf.F = np.array([
            [1, so/omega, 0, -(1-co)/omega], [0, co, 0, -so],
            [0, (1-co)/omega, 1, so/omega], [0, so, 0, co]])
    else:
        kf.F = np.array([[1,dt,0,0],[0,1,0,0],[0,0,1,dt],[0,0,0,1]])
    kf.H = np.array([[1,0,0,0],[0,0,1,0]])
    kf.R = np.eye(2) * r_std**2
    kf.Q = np.array([[dt**3/3,dt**2/2,0,0],[dt**2/2,dt,0,0],
                      [0,0,dt**3/3,dt**2/2],[0,0,dt**2/2,dt]]) * q
    kf.P = np.diag([r_std**2, 100, r_std**2, 100])
    return kf

def run_fp_cv(meas, dt, r_std, q):
    kf = _make_fp_cv(dt, r_std, q)
    kf.x = np.array([meas[0][0], 0, meas[0][1], 0])
    estimates = []
    for i in range(1, len(meas)):
        kf.predict(); kf.update(meas[i])
        estimates.append([kf.x[0], kf.x[2]])
    return np.array(estimates)

def run_fp_imm(meas, dt, r_std, q_cv, q_ct):
    """FilterPy IMM — 3 models: CV + CT(+3 deg/s) + CT(-3 deg/s)."""
    from filterpy.kalman import IMMEstimator
    cv = _make_fp_cv(dt, r_std, q_cv)
    ct_p = _make_fp_ct(dt, r_std, q_ct, omega=np.radians(3.0))
    ct_m = _make_fp_ct(dt, r_std, q_ct, omega=np.radians(-3.0))
    z0 = meas[0]
    for kf in [cv, ct_p, ct_m]:
        kf.x = np.array([z0[0], 0, z0[1], 0])
    M = np.array([[0.90,0.05,0.05],[0.10,0.85,0.05],[0.10,0.05,0.85]])
    mu = np.array([0.6, 0.2, 0.2])
    imm = IMMEstimator([cv, ct_p, ct_m], mu, M)
    estimates = []
    for i in range(1, len(meas)):
        imm.predict(); imm.update(meas[i])
        estimates.append([imm.x[0], imm.x[2]])
    return np.array(estimates)

# ============================================================================
# TRACKER: PyKalman (D. Duckworth)
# ============================================================================

def run_pk_cv(meas, dt, r_std, q):
    from pykalman import KalmanFilter
    F = np.array([[1,dt,0,0],[0,1,0,0],[0,0,1,dt],[0,0,0,1]])
    H = np.array([[1,0,0,0],[0,0,1,0]])
    Q = np.array([[dt**3/3,dt**2/2,0,0],[dt**2/2,dt,0,0],
                   [0,0,dt**3/3,dt**2/2],[0,0,dt**2/2,dt]]) * q
    R = np.eye(2) * r_std**2
    P0 = np.diag([r_std**2, 100, r_std**2, 100])
    kf = KalmanFilter(transition_matrices=F, observation_matrices=H,
                       transition_covariance=Q, observation_covariance=R,
                       initial_state_mean=np.array([meas[0][0],0,meas[0][1],0]),
                       initial_state_covariance=P0)
    state = np.array([meas[0][0],0,meas[0][1],0]); cov = P0.copy()
    estimates = []
    for i in range(1, len(meas)):
        state, cov = kf.filter_update(state, cov, meas[i])
        estimates.append([state[0], state[2]])
    return np.array(estimates)

def run_pk_ca(meas, dt, r_std, q):
    from pykalman import KalmanFilter
    F = np.array([[1,dt,dt**2/2,0,0,0],[0,1,dt,0,0,0],[0,0,1,0,0,0],
                   [0,0,0,1,dt,dt**2/2],[0,0,0,0,1,dt],[0,0,0,0,0,1]])
    H = np.array([[1,0,0,0,0,0],[0,0,0,1,0,0]])
    blk = np.array([[dt**5/20,dt**4/8,dt**3/6],[dt**4/8,dt**3/3,dt**2/2],
                     [dt**3/6,dt**2/2,dt]]) * q
    Q = np.zeros((6,6)); Q[:3,:3]=blk; Q[3:,3:]=blk
    R = np.eye(2)*r_std**2; P0 = np.diag([r_std**2,100,10,r_std**2,100,10])
    kf = KalmanFilter(transition_matrices=F, observation_matrices=H,
                       transition_covariance=Q, observation_covariance=R,
                       initial_state_mean=np.array([meas[0][0],0,0,meas[0][1],0,0]),
                       initial_state_covariance=P0)
    state = np.array([meas[0][0],0,0,meas[0][1],0,0]); cov = P0.copy()
    estimates = []
    for i in range(1, len(meas)):
        state, cov = kf.filter_update(state, cov, meas[i])
        estimates.append([state[0], state[3]])
    return np.array(estimates)

# ============================================================================
# TRACKER: NX-MIMOSA v4.2 GUARDIAN
# ============================================================================

def run_nx_mimosa(meas, dt, r_std, q_base=1.0):
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                     '..', 'python'))
    from nx_mimosa_v40_sentinel import NxMimosaV40Sentinel
    tracker = NxMimosaV40Sentinel(dt=dt, r_std=r_std, q_base=q_base,
        initial_models=["CV", "CT_plus", "CT_minus", "CA", "Jerk"])
    tracker.update(meas[0])
    estimates = []
    for i in range(1, len(meas)):
        pos, cov, intent = tracker.update(meas[i])
        estimates.append(pos[:2])
    return np.array(estimates)

# ============================================================================
# METRICS
# ============================================================================

@dataclass
class Metrics:
    name: str = ""
    rms: float = 0.0
    max_err: float = 0.0
    p95: float = 0.0
    mean: float = 0.0
    time_ms: float = 0.0
    n: int = 0
    failed: bool = False

def calc_metrics(name, truth, est, elapsed):
    n = min(len(truth)-1, len(est))
    errs = np.linalg.norm(truth[1:n+1] - est[:n], axis=1)
    return Metrics(name=name, rms=float(np.sqrt(np.mean(errs**2))),
        max_err=float(np.max(errs)), p95=float(np.percentile(errs, 95)),
        mean=float(np.mean(errs)), time_ms=elapsed*1000, n=n)

@dataclass
class ScenarioResult:
    name: str
    n_steps: int
    duration_s: float
    metrics: List[Metrics] = field(default_factory=list)

# ============================================================================
# SCENARIO RUNNER
# ============================================================================

def run_scenario(name, truth_fn, n_steps, dt, r_std):
    truth = truth_fn(n_steps, dt)
    meas = generate_measurements(truth, r_std, SEED)
    trackers = [
        ("SS KF-CV",       lambda: run_ss_cv(meas, dt, r_std, Q_CV)),
        ("SS KF-CA",       lambda: run_ss_ca(meas, dt, r_std, Q_CA)),
        ("SS Singer",      lambda: run_ss_singer(meas, dt, r_std, Q_SINGER)),
        ("FP KF-CV",       lambda: run_fp_cv(meas, dt, r_std, Q_CV)),
        ("FP IMM(CV+CT)",  lambda: run_fp_imm(meas, dt, r_std, Q_CV, Q_CT)),
        ("PK KF-CV",       lambda: run_pk_cv(meas, dt, r_std, Q_CV)),
        ("PK KF-CA",       lambda: run_pk_ca(meas, dt, r_std, Q_CA)),
        ("NX-MIMOSA v4.2", lambda: run_nx_mimosa(meas, dt, r_std, 1.0)),
    ]
    sr = ScenarioResult(name=name, n_steps=n_steps, duration_s=n_steps*dt)
    for tname, tfn in trackers:
        try:
            t0 = time.perf_counter(); est = tfn(); elapsed = time.perf_counter()-t0
            sr.metrics.append(calc_metrics(tname, truth, est, elapsed))
        except Exception as e:
            print(f"  WARNING: {tname} FAILED: {e}")
            sr.metrics.append(Metrics(name=tname, rms=float('inf'), failed=True))
    return sr

def best_of(metrics_list, prefix):
    cands = [m for m in metrics_list if m.name.startswith(prefix) and not m.failed]
    return min(cands, key=lambda m: m.rms) if cands else None

# ============================================================================
# REPORT
# ============================================================================

def print_report(all_results):
    W = 105
    print("\n" + "="*W)
    print("NX-MIMOSA v4.2 OPEN BENCHMARK — Multi-Library Tracking Comparison")
    print("="*W)
    print()
    print("Libraries under test:")
    print("  Stone Soup v1.8  (UK DSTL)       — KF-CV, KF-CA, Singer  [best-of-3 reported]")
    print("  FilterPy v1.4.5  (Roger Labbe)   — KF-CV, IMM (CV + CT+/- at +/-3 deg/s)")
    print("  PyKalman v0.11.2 (D. Duckworth)  — KF-CV, KF-CA  [best-of-2 reported]")
    print("  NX-MIMOSA v4.2   (Nexellum)      — VS-IMM (CV, CA, CT+/-, Jerk) + GUARDIAN")
    print()
    print(f"Parameters: dt={DT}s | R_std={R_STD}m | seed={SEED}")
    print(f"Q values:   CV={Q_CV}, CA={Q_CA}, CT={Q_CT}, Singer={Q_SINGER}")
    print()

    for sr in all_results:
        print("-"*W)
        print(f"SCENARIO: {sr.name}  ({sr.duration_s:.0f}s, {sr.n_steps} steps)")
        print("-"*W)
        print(f"  {'Tracker':<20} {'RMS(m)':>8} {'Max(m)':>8} {'P95(m)':>8} {'Mean(m)':>8} {'ms':>8}")
        print(f"  {'-'*19} {'-'*8} {'-'*8} {'-'*8} {'-'*8} {'-'*8}")
        valid = [m for m in sr.metrics if not m.failed]
        best_rms = min(m.rms for m in valid) if valid else 999
        for m in sr.metrics:
            tag = " << BEST" if m.rms == best_rms and not m.failed else ""
            if m.failed:
                print(f"  {m.name:<20} {'FAIL':>8}")
            else:
                print(f"  {m.name:<20} {m.rms:>8.1f} {m.max_err:>8.1f} "
                      f"{m.p95:>8.1f} {m.mean:>8.1f} {m.time_ms:>7.0f}{tag}")
        print()

    # Best-of-class summary
    print("\n" + "="*W)
    print("BEST-OF-CLASS SUMMARY — RMS Position Error (meters)")
    print("  Stone Soup = best of {CV, CA, Singer} per scenario")
    print("  FilterPy   = best of {CV, IMM} per scenario")
    print("  PyKalman   = best of {CV, CA} per scenario")
    print("  NX-MIMOSA  = single VS-IMM config (no per-scenario tuning)")
    print("="*W + "\n")

    cn = ["Stone Soup", "FilterPy", "PyKalman", "NX-MIMOSA v4.2"]
    pf = ["SS", "FP", "PK", "NX-MIMOSA"]
    sn = [sr.name.split("—")[0].strip()[:13] for sr in all_results]
    ns = len(all_results)

    rms = {}
    model_sel = {}
    for c, p in zip(cn, pf):
        rms[c] = []; model_sel[c] = []
        for sr in all_results:
            if p == "NX-MIMOSA":
                m = next((x for x in sr.metrics if x.name.startswith("NX")), None)
            else:
                m = best_of(sr.metrics, p)
            rms[c].append(m.rms if m and not m.failed else float('inf'))
            model_sel[c].append(m.name if m and not m.failed else "FAIL")

    hdr = f"  {'Library':<20}"
    for s in sn: hdr += f" {s:>13}"
    hdr += f" {'AVG':>8} {'WINS':>5}"
    print(hdr)
    print(f"  {'-'*19}" + (" "+"-"*13)*ns + f" {'-'*8} {'-'*5}")

    for c in cn:
        row = rms[c]
        valid = [v for v in row if v<float('inf')]
        avg = np.mean(valid) if valid else float('inf')
        wins = sum(1 for si in range(ns)
                   if row[si]==min(rms[cc][si] for cc in cn) and row[si]<float('inf'))
        line = f"  {c:<20}"
        for si, v in enumerate(row):
            is_best = v==min(rms[cc][si] for cc in cn) and v<float('inf')
            if v==float('inf'): line += f" {'FAIL':>13}"
            elif is_best: line += f" {v:>10.1f} * "
            else: line += f" {v:>13.1f}"
        line += f" {avg:>8.1f} {wins:>3}/{ns}"
        print(line)

    # Model selection
    print()
    print("Models selected (best-of-class per scenario):")
    for c in cn:
        if c == "NX-MIMOSA v4.2": continue
        models = model_sel[c]
        if len(set(models)) > 1:
            detail = " | ".join(f"S{i+1}={m}" for i, m in enumerate(models))
            print(f"  {c:<20} {detail}")

    # Head-to-head
    nx_rms = rms["NX-MIMOSA v4.2"]
    print()
    print("-"*W)
    print("HEAD-TO-HEAD vs NX-MIMOSA v4.2 (positive % = NX-MIMOSA better)")
    print("-"*W)

    for c in cn:
        if c == "NX-MIMOSA v4.2": continue
        other = rms[c]
        imps = []
        for si in range(ns):
            if nx_rms[si]<float('inf') and other[si]<float('inf') and other[si]>0:
                imps.append(((other[si]-nx_rms[si])/other[si])*100)
        if imps:
            wns = sum(1 for i in imps if i > 0)
            lss = sum(1 for i in imps if i < 0)
            print(f"  vs {c:<20} avg: {np.mean(imps):+.1f}%  W:{wns} L:{lss}  "
                  f"range: [{min(imps):+.1f}%, {max(imps):+.1f}%]")

    # Honest disclosure
    print()
    print("="*W)
    print("HONEST DISCLOSURE — Where NX-MIMOSA Loses:")
    found = False
    for si in range(ns):
        for c in cn:
            if c=="NX-MIMOSA v4.2": continue
            if rms[c][si] < nx_rms[si] and rms[c][si] < float('inf'):
                gap = (nx_rms[si]-rms[c][si])/rms[c][si]*100
                print(f"  S{si+1} {sn[si]}: {c} ({model_sel[c][si]}) wins "
                      f"— {rms[c][si]:.1f}m vs NX {nx_rms[si]:.1f}m (+{gap:.1f}% worse)")
                found = True
    if not found:
        print("  None — NX-MIMOSA wins all scenarios.")

    print()
    print("="*W)
    print("METHODOLOGY:")
    print("  All trackers: identical truth + noise (seed=42)")
    print("  Q values: reasonable defaults, NOT per-tracker/per-scenario optimized")
    print("  Stone Soup: 3 models tested, best RMS per scenario (generous)")
    print("  FilterPy: IMM with 3 models (CV, CT+, CT-)")
    print("  PyKalman: 2 models tested, best per scenario")
    print("  NX-MIMOSA: ONE fixed 5-model VS-IMM config for ALL scenarios")
    print("  No oracle info (true omega/acceleration) given to any tracker")
    print(f"  Reproduce: pip install stonesoup filterpy pykalman numpy")
    print(f"             python benchmarks/open_benchmark.py")
    print("="*W)

    return {"class_names": cn, "rms": rms, "model_sel": model_sel,
            "scenarios": [sr.name for sr in all_results], "short_names": sn}


def generate_markdown(all_results, data):
    cn = data["class_names"]; rms = data["rms"]
    sn = data["short_names"]; ns = len(all_results)

    L = []
    L.append("## Open Benchmark: NX-MIMOSA vs Stone Soup vs FilterPy vs PyKalman\n")
    L.append("**Fully reproducible** — anyone can verify:\n")
    L.append("```bash")
    L.append("pip install stonesoup filterpy pykalman numpy")
    L.append("python benchmarks/open_benchmark.py")
    L.append("```\n")

    L.append("### Fairness Principles\n")
    L.append("| Principle | Implementation |")
    L.append("|-----------|---------------|")
    L.append(f"| Fixed seed | `seed={SEED}` — identical truth + noise for ALL |")
    L.append(f"| Same noise | R_std = {R_STD}m for all trackers |")
    L.append(f"| Fair Q tuning | CV={Q_CV}, CA={Q_CA}, CT={Q_CT} — not per-tracker optimized |")
    L.append("| Best-of-class | Stone Soup picks BEST of 3 models per scenario |")
    L.append("| No oracle info | No tracker receives true turn rate or acceleration |")
    L.append("| Single config | NX-MIMOSA uses ONE fixed 5-model config for ALL scenarios |")
    L.append("")

    L.append("### Results — RMS Position Error (meters)\n")
    hdr = "| Library |"
    sep = "|---------|"
    for s in sn: hdr += f" {s} |"; sep += "---:|"
    hdr += " **AVG** | **Wins** |"; sep += "---:|---:|"
    L.append(hdr); L.append(sep)

    for c in cn:
        row = rms[c]
        valid = [v for v in row if v<float('inf')]
        avg = np.mean(valid) if valid else float('inf')
        wins = sum(1 for si in range(ns)
                   if row[si]==min(rms[cc][si] for cc in cn) and row[si]<float('inf'))
        line = f"| {c} |"
        for si, v in enumerate(row):
            is_best = v==min(rms[cc][si] for cc in cn) and v<float('inf')
            if v==float('inf'): line += " FAIL |"
            elif is_best: line += f" **{v:.1f}** ★ |"
            else: line += f" {v:.1f} |"
        line += f" **{avg:.1f}** | **{wins}/{ns}** |"
        L.append(line)

    L.append("")
    L.append("### Scenarios\n")
    for i, sr in enumerate(all_results):
        L.append(f"{i+1}. **{sr.name}** ({sr.duration_s:.0f}s, {sr.n_steps} steps)")
    L.append("")

    L.append("### Honest Disclosure\n")
    nx_rms = rms["NX-MIMOSA v4.2"]
    found = False
    for si in range(ns):
        for c in cn:
            if c=="NX-MIMOSA v4.2": continue
            if rms[c][si]<nx_rms[si] and rms[c][si]<float('inf'):
                gap = (nx_rms[si]-rms[c][si])/rms[c][si]*100
                L.append(f"- **S{si+1} {sn[si]}**: {c} wins ({rms[c][si]:.1f}m vs "
                         f"NX {nx_rms[si]:.1f}m, +{gap:.1f}%). "
                         f"Pure acceleration/ballistic scenarios favor dedicated CA model; "
                         f"IMM splits probability across 5 models.")
                found = True
    if not found:
        L.append("NX-MIMOSA wins all scenarios.")

    nx = rms["NX-MIMOSA v4.2"]
    L.append("\n### Head-to-Head\n")
    for c in cn:
        if c=="NX-MIMOSA v4.2": continue
        o = rms[c]
        imps = [((o[si]-nx[si])/o[si])*100 for si in range(ns)
                if nx[si]<float('inf') and o[si]<float('inf') and o[si]>0]
        if imps:
            L.append(f"- **vs {c}**: avg {np.mean(imps):+.1f}% "
                     f"(range {min(imps):+.1f}% to {max(imps):+.1f}%)")
    L.append("")
    return "\n".join(L)


# ============================================================================
# MAIN
# ============================================================================

def run_all():
    scenarios = [
        ("1. Constant Velocity — straight line ~291 m/s",     make_truth_cv,           300),
        ("2. Gentle Turn — 2 deg/s coordinated turn",         make_truth_gentle_turn,  200),
        ("3. Hard Turn — 8 deg/s (~4g at 300 m/s)",           make_truth_hard_turn,    150),
        ("4. Acceleration — 5g burst (100 to 500 m/s)",       make_truth_acceleration, 200),
        ("5. Fighter Dogfight — multi-segment profile",       make_truth_dogfight,     400),
        ("6. Jinking Evasion — +/-5 deg/s alternating",       make_truth_jinking,      300),
        ("7. Ballistic Arc — parabolic with gravity",         make_truth_ballistic,    250),
    ]
    all_results = []
    for name, truth_fn, n_steps in scenarios:
        print(f"Running: {name}...", flush=True)
        sr = run_scenario(name, truth_fn, n_steps, DT, R_STD)
        ok = len([m for m in sr.metrics if not m.failed])
        print(f"  {ok}/{len(sr.metrics)} trackers OK")
        all_results.append(sr)

    data = print_report(all_results)

    # Save JSON
    jpath = os.path.join(os.path.dirname(os.path.abspath(__file__)), "benchmark_results.json")
    with open(jpath, "w") as f:
        rms_ser = {k: [float(v) for v in vals] for k, vals in data["rms"].items()}
        json.dump({"seed": SEED, "dt": DT, "r_std": R_STD, "rms": rms_ser,
                    "scenarios": data["scenarios"]}, f, indent=2)
    print(f"\nJSON: {jpath}")

    # Save Markdown
    md = generate_markdown(all_results, data)
    mdpath = os.path.join(os.path.dirname(os.path.abspath(__file__)), "BENCHMARK_RESULTS.md")
    with open(mdpath, "w") as f: f.write(md)
    print(f"Markdown: {mdpath}")

    return all_results, data

if __name__ == "__main__":
    run_all()
