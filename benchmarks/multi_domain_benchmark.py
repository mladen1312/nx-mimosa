#!/usr/bin/env python3
"""
NX-MIMOSA v4.2 MULTI-DOMAIN OPEN BENCHMARK
============================================
Independent, unbiased comparison across 5 operational domains:

  DOMAIN 1 — CIVILIAN ATC (Air Traffic Control)
    S01: Enroute cruise (FL350, 480kt)
    S02: Holding pattern (standard rate, 220kt)
    S03: ILS approach (3° glideslope, decelerating)
    S04: Missed approach / go-around (climb + turn)

  DOMAIN 2 — COMMERCIAL AVIATION
    S05: Oceanic cruise with wind shear
    S06: Turbulence encounter (random acceleration bursts)
    S07: TCAS RA climb (1500 ft/min → 2500 ft/min)

  DOMAIN 3 — MILITARY
    S08: Fighter intercept (Mach 1.6, 7g break turn)
    S09: SAM engagement (Mach 3, terminal dive)
    S10: Cruise missile sea-skimmer (30m altitude, weave)
    S11: Helicopter NOE (low/slow, pop-up maneuver)

  DOMAIN 4 — AUTOMOTIVE
    S12: Highway cruise (120 km/h straight)
    S13: Urban intersection (stop, turn, accelerate)
    S14: Emergency braking (1g deceleration)
    S15: Highway lane change (lateral maneuver)

  DOMAIN 5 — SPACE
    S16: LEO satellite pass (7.5 km/s, predictable)
    S17: GEO stationkeeping (micro-corrections)
    S18: Orbital maneuver (delta-v burn)
    S19: Reentry vehicle (deceleration + S-turn)

FAIRNESS:
  • Fixed seed (42) — identical noise for all trackers
  • Domain-appropriate R_std and dt for each scenario
  • Same Q tuning principles across all trackers
  • Stone Soup/FilterPy/PyKalman get BEST-of-class per scenario
  • NX-MIMOSA uses ONE config per domain (no per-scenario tuning)
  • No oracle information given to any tracker

REPRODUCE:
  pip install stonesoup filterpy pykalman numpy
  python benchmarks/multi_domain_benchmark.py

Author: Dr. Mladen Mešter / Nexellum d.o.o.
License: MIT (benchmark code)
"""

import numpy as np
import time
import sys
import os
import json
from dataclasses import dataclass, field
from typing import List, Tuple, Callable, Optional, Dict
from datetime import datetime, timedelta

SEED = 42

# ============================================================================
# Domain Parameters — realistic per-domain sensor characteristics
# ============================================================================
DOMAINS = {
    "ATC": {
        "dt": 4.0,        # SSR/Mode-S rotation: 4s
        "r_std": 50.0,    # SSR accuracy ~50m range, ~0.06° azimuth
        "q_cv": 0.01,     # Very low — aircraft are predictable
        "q_ca": 0.1,
        "q_ct": 0.05,
        "q_singer": 0.1,
        "nx_q_base": 0.05,
        "nx_domain": "atc",
    },
    "AVIATION": {
        "dt": 1.0,        # ADS-B: 1s update
        "r_std": 15.0,    # ADS-B GPS accuracy ~15m
        "q_cv": 0.05,
        "q_ca": 0.5,
        "q_ct": 0.2,
        "q_singer": 0.5,
        "nx_q_base": 0.2,
        "nx_domain": "aviation",
    },
    "MILITARY": {
        "dt": 0.1,        # Military radar: 10 Hz track update
        "r_std": 5.0,     # Pulse-Doppler: ~5m range accuracy
        "q_cv": 0.5,
        "q_ca": 2.0,
        "q_ct": 1.0,
        "q_singer": 2.0,
        "nx_q_base": 1.0,
        "nx_domain": "military",
    },
    "AUTOMOTIVE": {
        "dt": 0.05,       # Automotive radar: 20 Hz
        "r_std": 0.3,     # Automotive radar: ~0.3m accuracy
        "q_cv": 0.5,
        "q_ca": 5.0,
        "q_ct": 2.0,
        "q_singer": 5.0,
        "nx_q_base": 2.0,
        "nx_domain": "automotive",
    },
    "SPACE": {
        "dt": 10.0,       # Space surveillance: 10s between measurements
        "r_std": 100.0,   # Ground radar: ~100m at LEO range
        "q_cv": 0.0001,   # Orbital mechanics: extremely predictable
        "q_ca": 0.001,
        "q_ct": 0.0005,
        "q_singer": 0.001,
        "nx_q_base": 0.001,
        "nx_domain": "space",
    },
}

# ============================================================================
# Truth Trajectory Generators — Domain-specific
# ============================================================================

# --- DOMAIN 1: ATC ---

def atc_enroute(n, dt):
    """S01: Enroute cruise FL350, 480kt (247 m/s) heading 090."""
    truth = np.zeros((n, 2))
    v = 247.0  # 480 kt
    for i in range(n):
        truth[i] = [v * i * dt, 0.0]
    return truth

def atc_holding(n, dt):
    """S02: Standard holding pattern — 1-min legs, standard rate turns (3°/s).
    Racetrack: straight 1 min → 180° turn → straight 1 min → 180° turn."""
    truth = np.zeros((n, 2))
    speed = 113.0  # 220 kt
    omega_turn = np.radians(3.0)  # Standard rate
    heading = 0.0
    x, y = 0.0, 0.0
    leg_time = 60.0  # 1 minute legs
    turn_time = 60.0  # 180° at 3°/s = 60s
    for i in range(n):
        t = i * dt
        cycle_t = t % (2 * leg_time + 2 * turn_time)
        truth[i] = [x, y]
        if cycle_t < leg_time:
            omega = 0.0
        elif cycle_t < leg_time + turn_time:
            omega = omega_turn
        elif cycle_t < 2 * leg_time + turn_time:
            omega = 0.0
        else:
            omega = omega_turn
        heading += omega * dt
        x += speed * np.cos(heading) * dt
        y += speed * np.sin(heading) * dt
    return truth

def atc_ils_approach(n, dt):
    """S03: ILS approach — 3° glideslope, decelerating from 140kt to 130kt."""
    truth = np.zeros((n, 2))
    speed = 72.0  # 140 kt initial
    decel = -0.1  # Gentle deceleration
    heading = 0.0
    x, y = 0.0, 0.0
    for i in range(n):
        truth[i] = [x, y]
        x += speed * dt
        speed = max(67.0, speed + decel * dt)  # Floor at 130kt
    return truth

def atc_missed_approach(n, dt):
    """S04: Missed approach — climb + 15° heading change."""
    truth = np.zeros((n, 2))
    speed = 77.0  # 150kt
    heading = 0.0
    x, y = 0.0, 0.0
    omega = 0.0
    for i in range(n):
        t = i * dt
        truth[i] = [x, y]
        if t < 20:
            omega = 0.0
            speed = min(speed + 0.5 * dt, 93.0)  # Accelerate to 180kt
        elif t < 40:
            omega = np.radians(1.5)  # Gentle turn
        else:
            omega = 0.0
        heading += omega * dt
        x += speed * np.cos(heading) * dt
        y += speed * np.sin(heading) * dt
    return truth

# --- DOMAIN 2: COMMERCIAL AVIATION ---

def avia_cruise_windshear(n, dt):
    """S05: Oceanic cruise with gradual wind shear — apparent velocity change."""
    truth = np.zeros((n, 2))
    base_speed = 257.0  # Mach 0.85 at FL390
    heading = np.radians(45)
    x, y = 0.0, 0.0
    for i in range(n):
        t = i * dt
        truth[i] = [x, y]
        # Wind component varies sinusoidally
        wind_x = 15.0 * np.sin(2 * np.pi * t / 300)  # 15 m/s amplitude, 5-min period
        wind_y = 8.0 * np.cos(2 * np.pi * t / 200)
        vx = base_speed * np.cos(heading) + wind_x
        vy = base_speed * np.sin(heading) + wind_y
        x += vx * dt
        y += vy * dt
    return truth

def avia_turbulence(n, dt):
    """S06: Turbulence encounter — random acceleration bursts (moderate CAT)."""
    rng_turb = np.random.RandomState(123)  # Separate seed for truth
    truth = np.zeros((n, 2))
    speed = 240.0
    x, y = 0.0, 0.0
    vx, vy = speed, 0.0
    for i in range(n):
        truth[i] = [x, y]
        # Turbulence: random acceleration, ~2 m/s² RMS
        ax = rng_turb.randn() * 2.0
        ay = rng_turb.randn() * 2.0
        vx += ax * dt
        vy += ay * dt
        # Dampen back to cruise speed (drag-like restoring force)
        sp = max(np.sqrt(vx**2 + vy**2), 1e-6)
        restore = 0.98  # Per-step damping toward cruise speed
        vx = vx * restore * (speed / sp) + vx * (1.0 - restore)
        vy = vy * restore * (speed / sp) + vy * (1.0 - restore)
        x += vx * dt
        y += vy * dt
    return truth

def avia_tcas_ra(n, dt):
    """S07: TCAS RA — level flight then sudden climb command (1500→2500 ft/min).
    Modeled as 2D: sudden lateral offset acceleration then back to straight."""
    truth = np.zeros((n, 2))
    speed = 220.0
    x, y = 0.0, 0.0
    vy = 0.0
    for i in range(n):
        t = i * dt
        truth[i] = [x, y]
        x += speed * dt
        if 10.0 < t < 15.0:
            vy += 2.0 * dt  # Climb acceleration
        elif 15.0 < t < 25.0:
            vy *= 0.98  # Level off
        y += vy * dt
    return truth

# --- DOMAIN 3: MILITARY ---

def mil_intercept(n, dt):
    """S08: Fighter intercept — Mach 1.6 (490 m/s) with 7g break turn."""
    truth = np.zeros((n, 2))
    speed = 490.0
    heading = 0.0
    x, y = 0.0, 0.0
    for i in range(n):
        t = i * dt
        truth[i] = [x, y]
        if t < 8.0:
            omega = 0.0
        elif t < 13.0:
            omega = np.radians(12.7)  # ~7g at 490 m/s
        elif t < 20.0:
            omega = 0.0
        elif t < 25.0:
            omega = np.radians(-8.0)  # 4g reverse
        else:
            omega = 0.0
        heading += omega * dt
        x += speed * np.cos(heading) * dt
        y += speed * np.sin(heading) * dt
    return truth

def mil_sam(n, dt):
    """S09: SAM engagement — Mach 3 (1020 m/s), terminal acceleration + dive."""
    truth = np.zeros((n, 2))
    speed = 600.0  # Initial
    accel = 60.0  # 6g acceleration phase
    heading = np.radians(30)
    x, y = 0.0, 0.0
    for i in range(n):
        t = i * dt
        truth[i] = [x, y]
        if t < 5.0:
            speed = min(speed + accel * dt, 1020.0)
            omega = 0.0
        elif t < 10.0:
            omega = np.radians(15.0)  # Terminal guidance correction
        elif t < 15.0:
            omega = np.radians(-20.0)  # Dive
        else:
            omega = 0.0
        heading += omega * dt
        x += speed * np.cos(heading) * dt
        y += speed * np.sin(heading) * dt
    return truth

def mil_cruise_missile(n, dt):
    """S10: Sea-skimming cruise missile — 250 m/s with weave pattern."""
    truth = np.zeros((n, 2))
    speed = 250.0
    heading = 0.0
    x, y = 0.0, 0.0
    for i in range(n):
        t = i * dt
        truth[i] = [x, y]
        # Sinusoidal weave: ±2°/s, period 5s
        omega = np.radians(2.0) * np.sin(2 * np.pi * t / 5.0)
        heading += omega * dt
        x += speed * np.cos(heading) * dt
        y += speed * np.sin(heading) * dt
    return truth

def mil_helicopter_noe(n, dt):
    """S11: Helicopter NOE — slow (60 m/s), pop-up + break + re-mask."""
    truth = np.zeros((n, 2))
    speed = 60.0
    heading = 0.0
    x, y = 0.0, 0.0
    for i in range(n):
        t = i * dt
        truth[i] = [x, y]
        if t < 10.0:
            omega = 0.0
            speed = 60.0
        elif t < 13.0:
            omega = 0.0
            speed = 80.0  # Pop-up acceleration
        elif t < 16.0:
            omega = np.radians(8.0)  # Break turn
            speed = 70.0
        elif t < 22.0:
            omega = np.radians(-5.0)  # Reverse to re-mask
            speed = 55.0
        else:
            omega = 0.0
            speed = 50.0
        heading += omega * dt
        x += speed * np.cos(heading) * dt
        y += speed * np.sin(heading) * dt
    return truth

# --- DOMAIN 4: AUTOMOTIVE ---

def auto_highway(n, dt):
    """S12: Highway cruise — 120 km/h (33.3 m/s) straight."""
    truth = np.zeros((n, 2))
    v = 33.3
    for i in range(n):
        truth[i] = [v * i * dt, 0.0]
    return truth

def auto_intersection(n, dt):
    """S13: Urban intersection — approach, stop, turn 90°, accelerate."""
    truth = np.zeros((n, 2))
    x, y = 0.0, 0.0
    speed = 13.9  # 50 km/h
    heading = 0.0
    for i in range(n):
        t = i * dt
        truth[i] = [x, y]
        if t < 3.0:
            pass  # Cruise
        elif t < 5.0:
            speed = max(0.5, speed - 7.0 * dt)  # Brake
        elif t < 5.5:
            speed = max(0.5, speed)  # Near-stop
        elif t < 8.0:
            omega = np.radians(36.0)  # 90° turn in ~2.5s
            heading += omega * dt
            speed = min(speed + 3.0 * dt, 8.0)
        elif t < 12.0:
            speed = min(speed + 2.5 * dt, 13.9)  # Re-accelerate
        x += speed * np.cos(heading) * dt
        y += speed * np.sin(heading) * dt
    return truth

def auto_emergency_brake(n, dt):
    """S14: Emergency braking — 120 km/h → 0 at ~1g (9.8 m/s²)."""
    truth = np.zeros((n, 2))
    speed = 33.3
    x = 0.0
    for i in range(n):
        truth[i] = [x, 0.0]
        if i * dt > 1.0:  # Brake after 1s
            speed = max(0, speed - 9.8 * dt)
        x += speed * dt
    return truth

def auto_lane_change(n, dt):
    """S15: Highway lane change — 3.5m lateral shift over 3s at 100 km/h."""
    truth = np.zeros((n, 2))
    vx = 27.8  # 100 km/h
    x, y = 0.0, 0.0
    for i in range(n):
        t = i * dt
        truth[i] = [x, y]
        x += vx * dt
        if 2.0 < t < 5.0:
            # Sinusoidal lateral: y = 3.5/2 * (1 - cos(pi*(t-2)/3))
            vy = 3.5 / 2 * np.pi / 3.0 * np.sin(np.pi * (t - 2.0) / 3.0)
            y += vy * dt
    return truth

# --- DOMAIN 5: SPACE ---

def space_leo(n, dt):
    """S16: LEO satellite — circular orbit at 400km altitude (7.67 km/s)."""
    truth = np.zeros((n, 2))
    R = 6771e3  # Earth radius + 400km
    v = 7670.0  # Orbital velocity
    omega = v / R  # Angular rate
    for i in range(n):
        t = i * dt
        # Project to 2D pass overhead (as seen from ground station)
        angle = omega * t
        truth[i] = [R * np.sin(angle), R * (np.cos(angle) - 1)]
    return truth

def space_geo_stationkeep(n, dt):
    """S17: GEO stationkeeping — nearly stationary, micro-corrections."""
    rng_space = np.random.RandomState(777)
    truth = np.zeros((n, 2))
    x, y = 42164e3, 0.0
    vx, vy = 0.0, 0.0
    for i in range(n):
        truth[i] = [x, y]
        # Micro-corrections every ~100s
        if i > 0 and i % 10 == 0:
            vx += rng_space.randn() * 0.1  # ~0.1 m/s correction
            vy += rng_space.randn() * 0.1
        x += vx * dt
        y += vy * dt
        # Solar pressure drift
        vx += 0.001 * dt
    return truth

def space_orbital_maneuver(n, dt):
    """S18: Orbital maneuver — delta-v burn at t=100s (10 m/s over 20s)."""
    truth = np.zeros((n, 2))
    speed = 7670.0
    heading = 0.0
    x, y = 0.0, 0.0
    for i in range(n):
        t = i * dt
        truth[i] = [x, y]
        if 100.0 < t < 120.0:
            speed += 0.5 * dt  # 10 m/s delta-v over 20s
            heading += np.radians(0.01) * dt  # Slight plane change
        x += speed * np.cos(heading) * dt
        y += speed * np.sin(heading) * dt
    return truth

def space_reentry(n, dt):
    """S19: Reentry vehicle — deceleration from 7 km/s + S-turn maneuver."""
    truth = np.zeros((n, 2))
    speed = 7000.0
    heading = np.radians(-10)  # Slight descent angle
    x, y = 0.0, 0.0
    for i in range(n):
        t = i * dt
        truth[i] = [x, y]
        # Atmospheric deceleration (exponential)
        drag = 5.0 * np.exp(t / 100.0)  # Growing drag
        speed = max(1000.0, speed - drag * dt)
        # S-turn for cross-range
        if 50 < t < 80:
            omega = np.radians(0.5)
        elif 80 < t < 110:
            omega = np.radians(-0.5)
        else:
            omega = 0.0
        heading += omega * dt
        x += speed * np.cos(heading) * dt
        y += speed * np.sin(heading) * dt
    return truth

# ============================================================================
# Measurement Generator
# ============================================================================

def generate_measurements(truth, r_std, seed=42):
    rng = np.random.RandomState(seed)
    return truth + rng.randn(len(truth), 2) * r_std

# ============================================================================
# TRACKERS — Stone Soup, FilterPy, PyKalman, NX-MIMOSA
# ============================================================================

def run_stonesoup_cv(meas, dt, r_std, q):
    from stonesoup.models.transition.linear import CombinedLinearGaussianTransitionModel, ConstantVelocity
    from stonesoup.models.measurement.linear import LinearGaussian
    from stonesoup.predictor.kalman import KalmanPredictor
    from stonesoup.updater.kalman import KalmanUpdater
    from stonesoup.types.state import GaussianState
    from stonesoup.types.detection import Detection
    from stonesoup.types.hypothesis import SingleHypothesis
    from stonesoup.types.array import StateVector, CovarianceMatrix

    trans = CombinedLinearGaussianTransitionModel([ConstantVelocity(q), ConstantVelocity(q)])
    mm = LinearGaussian(ndim_state=4, mapping=(0, 2), noise_covar=np.eye(2)*r_std**2)
    pred, upd = KalmanPredictor(trans), KalmanUpdater(mm)
    z0 = meas[0]
    state = GaussianState(StateVector([z0[0],0,z0[1],0]),
                          CovarianceMatrix(np.diag([r_std**2,100,r_std**2,100])),
                          timestamp=datetime.now())
    est = []
    t0 = datetime.now()
    for i in range(1, len(meas)):
        ts = t0 + timedelta(seconds=i*dt)
        det = Detection(StateVector(meas[i].reshape(-1,1)), timestamp=ts, measurement_model=mm)
        p = pred.predict(state, timestamp=ts)
        state = upd.update(SingleHypothesis(p, det))
        est.append([float(state.state_vector[0]), float(state.state_vector[2])])
    return np.array(est)

def run_stonesoup_ca(meas, dt, r_std, q):
    from stonesoup.models.transition.linear import CombinedLinearGaussianTransitionModel, ConstantAcceleration
    from stonesoup.models.measurement.linear import LinearGaussian
    from stonesoup.predictor.kalman import KalmanPredictor
    from stonesoup.updater.kalman import KalmanUpdater
    from stonesoup.types.state import GaussianState
    from stonesoup.types.detection import Detection
    from stonesoup.types.hypothesis import SingleHypothesis
    from stonesoup.types.array import StateVector, CovarianceMatrix

    trans = CombinedLinearGaussianTransitionModel([ConstantAcceleration(q), ConstantAcceleration(q)])
    mm = LinearGaussian(ndim_state=6, mapping=(0, 3), noise_covar=np.eye(2)*r_std**2)
    pred, upd = KalmanPredictor(trans), KalmanUpdater(mm)
    z0 = meas[0]
    state = GaussianState(StateVector([z0[0],0,0,z0[1],0,0]),
                          CovarianceMatrix(np.diag([r_std**2,100,10,r_std**2,100,10])),
                          timestamp=datetime.now())
    est = []
    t0 = datetime.now()
    for i in range(1, len(meas)):
        ts = t0 + timedelta(seconds=i*dt)
        det = Detection(StateVector(meas[i].reshape(-1,1)), timestamp=ts, measurement_model=mm)
        p = pred.predict(state, timestamp=ts)
        state = upd.update(SingleHypothesis(p, det))
        est.append([float(state.state_vector[0]), float(state.state_vector[3])])
    return np.array(est)

def run_stonesoup_singer(meas, dt, r_std, q):
    from stonesoup.models.transition.linear import CombinedLinearGaussianTransitionModel, Singer
    from stonesoup.models.measurement.linear import LinearGaussian
    from stonesoup.predictor.kalman import KalmanPredictor
    from stonesoup.updater.kalman import KalmanUpdater
    from stonesoup.types.state import GaussianState
    from stonesoup.types.detection import Detection
    from stonesoup.types.hypothesis import SingleHypothesis
    from stonesoup.types.array import StateVector, CovarianceMatrix

    trans = CombinedLinearGaussianTransitionModel(
        [Singer(damping_coeff=1.0/3.0, noise_diff_coeff=q),
         Singer(damping_coeff=1.0/3.0, noise_diff_coeff=q)])
    mm = LinearGaussian(ndim_state=6, mapping=(0, 3), noise_covar=np.eye(2)*r_std**2)
    pred, upd = KalmanPredictor(trans), KalmanUpdater(mm)
    z0 = meas[0]
    state = GaussianState(StateVector([z0[0],0,0,z0[1],0,0]),
                          CovarianceMatrix(np.diag([r_std**2,100,10,r_std**2,100,10])),
                          timestamp=datetime.now())
    est = []
    t0 = datetime.now()
    for i in range(1, len(meas)):
        ts = t0 + timedelta(seconds=i*dt)
        det = Detection(StateVector(meas[i].reshape(-1,1)), timestamp=ts, measurement_model=mm)
        p = pred.predict(state, timestamp=ts)
        state = upd.update(SingleHypothesis(p, det))
        est.append([float(state.state_vector[0]), float(state.state_vector[3])])
    return np.array(est)

def _make_fp_cv(dt, r_std, q):
    from filterpy.kalman import KalmanFilter
    kf = KalmanFilter(dim_x=4, dim_z=2)
    kf.F = np.array([[1,dt,0,0],[0,1,0,0],[0,0,1,dt],[0,0,0,1]])
    kf.H = np.array([[1,0,0,0],[0,0,1,0]])
    kf.R = np.eye(2)*r_std**2
    kf.Q = np.array([[dt**3/3,dt**2/2,0,0],[dt**2/2,dt,0,0],
                      [0,0,dt**3/3,dt**2/2],[0,0,dt**2/2,dt]])*q
    kf.P = np.diag([r_std**2,100,r_std**2,100])
    return kf

def _make_fp_ct(dt, r_std, q, omega):
    from filterpy.kalman import KalmanFilter
    kf = KalmanFilter(dim_x=4, dim_z=2)
    co, so = np.cos(omega*dt), np.sin(omega*dt)
    if abs(omega) > 1e-6:
        kf.F = np.array([[1,so/omega,0,-(1-co)/omega],[0,co,0,-so],
                          [0,(1-co)/omega,1,so/omega],[0,so,0,co]])
    else:
        kf.F = np.array([[1,dt,0,0],[0,1,0,0],[0,0,1,dt],[0,0,0,1]])
    kf.H = np.array([[1,0,0,0],[0,0,1,0]])
    kf.R = np.eye(2)*r_std**2
    kf.Q = np.array([[dt**3/3,dt**2/2,0,0],[dt**2/2,dt,0,0],
                      [0,0,dt**3/3,dt**2/2],[0,0,dt**2/2,dt]])*q
    kf.P = np.diag([r_std**2,100,r_std**2,100])
    return kf

def run_filterpy_cv(meas, dt, r_std, q):
    kf = _make_fp_cv(dt, r_std, q)
    kf.x = np.array([meas[0,0],0,meas[0,1],0])
    est = []
    for i in range(1, len(meas)):
        kf.predict(); kf.update(meas[i])
        est.append([kf.x[0], kf.x[2]])
    return np.array(est)

def run_filterpy_imm(meas, dt, r_std, q_cv, q_ct, omega_ct=None):
    from filterpy.kalman import IMMEstimator
    if omega_ct is None:
        omega_ct = np.radians(3.0)
    cv = _make_fp_cv(dt, r_std, q_cv)
    ct_p = _make_fp_ct(dt, r_std, q_ct, omega_ct)
    ct_m = _make_fp_ct(dt, r_std, q_ct, -omega_ct)
    z0 = meas[0]
    for kf in [cv, ct_p, ct_m]:
        kf.x = np.array([z0[0],0,z0[1],0])
    M = np.array([[0.90,0.05,0.05],[0.10,0.85,0.05],[0.10,0.05,0.85]])
    mu = np.array([0.6,0.2,0.2])
    imm = IMMEstimator([cv, ct_p, ct_m], mu, M)
    est = []
    for i in range(1, len(meas)):
        imm.predict(); imm.update(meas[i])
        est.append([imm.x[0], imm.x[2]])
    return np.array(est)

def run_pykalman_cv(meas, dt, r_std, q):
    from pykalman import KalmanFilter
    F = np.array([[1,dt,0,0],[0,1,0,0],[0,0,1,dt],[0,0,0,1]])
    H = np.array([[1,0,0,0],[0,0,1,0]])
    Q = np.array([[dt**3/3,dt**2/2,0,0],[dt**2/2,dt,0,0],
                   [0,0,dt**3/3,dt**2/2],[0,0,dt**2/2,dt]])*q
    R = np.eye(2)*r_std**2
    kf = KalmanFilter(transition_matrices=F, observation_matrices=H,
                      transition_covariance=Q, observation_covariance=R,
                      initial_state_mean=np.array([meas[0,0],0,meas[0,1],0]),
                      initial_state_covariance=np.diag([r_std**2,100,r_std**2,100]))
    states, _ = kf.filter(meas[1:])
    return states[:, [0,2]]

def run_pykalman_ca(meas, dt, r_std, q):
    from pykalman import KalmanFilter
    F = np.eye(6)
    F[0,1]=dt; F[0,2]=dt**2/2; F[1,2]=dt; F[3,4]=dt; F[3,5]=dt**2/2; F[4,5]=dt
    H = np.zeros((2,6)); H[0,0]=1; H[1,3]=1
    b = np.array([[dt**5/20,dt**4/8,dt**3/6],[dt**4/8,dt**3/3,dt**2/2],[dt**3/6,dt**2/2,dt]])*q
    Q = np.zeros((6,6)); Q[:3,:3]=b; Q[3:,3:]=b
    R = np.eye(2)*r_std**2
    kf = KalmanFilter(transition_matrices=F, observation_matrices=H,
                      transition_covariance=Q, observation_covariance=R,
                      initial_state_mean=np.array([meas[0,0],0,0,meas[0,1],0,0]),
                      initial_state_covariance=np.diag([r_std**2,100,10,r_std**2,100,10]))
    states, _ = kf.filter(meas[1:])
    return states[:, [0,3]]

def run_nx_mimosa(meas, dt, r_std, q_base, models=None, domain=None):
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))
    from nx_mimosa_v40_sentinel import NxMimosaV40Sentinel
    if models is None and domain is None:
        models = ["CV", "CT_plus", "CT_minus", "CA", "Jerk"]
    tracker = NxMimosaV40Sentinel(dt=dt, r_std=r_std, q_base=q_base,
                                   initial_models=models, domain=domain)
    tracker.update(meas[0])
    est = []
    for i in range(1, len(meas)):
        pos, cov, intent = tracker.update(meas[i])
        est.append(pos[:2])
    return np.array(est)

# ============================================================================
# Metrics
# ============================================================================

@dataclass
class Metrics:
    name: str = ""
    rms: float = 0.0
    max_err: float = 0.0
    p95: float = 0.0
    mean_err: float = 0.0
    time_ms: float = 0.0

def calc_metrics(name, truth, est, elapsed):
    n = min(len(truth)-1, len(est))
    errs = np.linalg.norm(truth[1:n+1] - est[:n], axis=1)
    return Metrics(name, float(np.sqrt(np.mean(errs**2))), float(np.max(errs)),
                   float(np.percentile(errs, 95)), float(np.mean(errs)), elapsed*1000)

# ============================================================================
# Scenario Definition
# ============================================================================

@dataclass
class Scenario:
    id: str
    name: str
    domain: str
    truth_fn: Callable
    n_steps: int
    description: str = ""

SCENARIOS = [
    # ATC
    Scenario("S01","Enroute Cruise","ATC",atc_enroute,75,"FL350, 480kt, straight"),
    Scenario("S02","Holding Pattern","ATC",atc_holding,120,"Standard rate turns, 220kt"),
    Scenario("S03","ILS Approach","ATC",atc_ils_approach,50,"3deg glideslope, decelerating"),
    Scenario("S04","Missed Approach","ATC",atc_missed_approach,30,"Climb + heading change"),
    # Aviation
    Scenario("S05","Cruise + Wind Shear","AVIATION",avia_cruise_windshear,300,"Mach 0.85, sinusoidal wind"),
    Scenario("S06","Turbulence","AVIATION",avia_turbulence,200,"Moderate CAT, random accel"),
    Scenario("S07","TCAS RA Climb","AVIATION",avia_tcas_ra,60,"Sudden climb command"),
    # Military
    Scenario("S08","Fighter Intercept","MILITARY",mil_intercept,300,"Mach 1.6, 7g break turn"),
    Scenario("S09","SAM Engagement","MILITARY",mil_sam,200,"Mach 3, terminal dive"),
    Scenario("S10","Cruise Missile","MILITARY",mil_cruise_missile,300,"250 m/s, weave pattern"),
    Scenario("S11","Helicopter NOE","MILITARY",mil_helicopter_noe,300,"Pop-up + break + re-mask"),
    # Automotive
    Scenario("S12","Highway Cruise","AUTOMOTIVE",auto_highway,400,"120 km/h straight"),
    Scenario("S13","Urban Intersection","AUTOMOTIVE",auto_intersection,300,"Stop, turn, accelerate"),
    Scenario("S14","Emergency Brake","AUTOMOTIVE",auto_emergency_brake,200,"1g deceleration"),
    Scenario("S15","Lane Change","AUTOMOTIVE",auto_lane_change,200,"3.5m lateral at 100 km/h"),
    # Space
    Scenario("S16","LEO Satellite","SPACE",space_leo,60,"7.67 km/s circular orbit"),
    Scenario("S17","GEO Stationkeeping","SPACE",space_geo_stationkeep,50,"Micro-corrections"),
    Scenario("S18","Orbital Maneuver","SPACE",space_orbital_maneuver,30,"10 m/s delta-v burn"),
    Scenario("S19","Reentry Vehicle","SPACE",space_reentry,30,"Deceleration + S-turn"),
]

# ============================================================================
# Domain-appropriate CT omega for FilterPy IMM
# ============================================================================
DOMAIN_CT_OMEGA = {
    "ATC": np.radians(3.0),        # Standard rate turn
    "AVIATION": np.radians(1.5),   # Gentle commercial
    "MILITARY": np.radians(5.0),   # Fighter-grade
    "AUTOMOTIVE": np.radians(15.0),# Tight intersection turns
    "SPACE": np.radians(0.01),     # Orbital: nearly zero
}

# ============================================================================
# Runner
# ============================================================================

def run_scenario(sc: Scenario) -> Tuple[str, List[Metrics]]:
    dp = DOMAINS[sc.domain]
    dt, r_std = dp["dt"], dp["r_std"]
    truth = sc.truth_fn(sc.n_steps, dt)
    meas = generate_measurements(truth, r_std, SEED)

    trackers = [
        ("SS CV",     lambda: run_stonesoup_cv(meas, dt, r_std, dp["q_cv"])),
        ("SS CA",     lambda: run_stonesoup_ca(meas, dt, r_std, dp["q_ca"])),
        ("SS Singer", lambda: run_stonesoup_singer(meas, dt, r_std, dp["q_singer"])),
        ("FP CV",     lambda: run_filterpy_cv(meas, dt, r_std, dp["q_cv"])),
        ("FP IMM",    lambda: run_filterpy_imm(meas, dt, r_std, dp["q_cv"], dp["q_ct"],
                                                DOMAIN_CT_OMEGA[sc.domain])),
        ("PK CV",     lambda: run_pykalman_cv(meas, dt, r_std, dp["q_cv"])),
        ("PK CA",     lambda: run_pykalman_ca(meas, dt, r_std, dp["q_ca"])),
        ("NX v4.2",   lambda: run_nx_mimosa(meas, dt, r_std, dp["nx_q_base"],
                                              domain=dp.get("nx_domain"))),
    ]

    results = []
    for tname, tfn in trackers:
        try:
            t0 = time.perf_counter()
            est = tfn()
            elapsed = time.perf_counter() - t0
            m = calc_metrics(tname, truth, est, elapsed)
            results.append(m)
        except Exception as e:
            results.append(Metrics(name=tname, rms=float('inf')))
    return sc.id, results

# ============================================================================
# Report
# ============================================================================

def best_of_lib(results: List[Metrics], prefixes: List[str]) -> Metrics:
    """Return best (lowest RMS) among trackers matching any prefix."""
    matches = [r for r in results if any(r.name.startswith(p) for p in prefixes)]
    valid = [r for r in matches if r.rms < float('inf')]
    if not valid: return Metrics(name="FAIL", rms=float('inf'))
    return min(valid, key=lambda r: r.rms)

def print_domain_report(domain: str, scenarios: List[Tuple[Scenario, List[Metrics]]]):
    dp = DOMAINS[domain]
    print(f"\n{'='*100}")
    print(f"DOMAIN: {domain}  |  dt={dp['dt']}s  R_std={dp['r_std']}m  seed={SEED}")
    print(f"{'='*100}")
    
    for sc, results in scenarios:
        dur = sc.n_steps * dp["dt"]
        print(f"\n  {sc.id}: {sc.name} — {sc.description}  ({dur:.0f}s, {sc.n_steps} steps)")
        print(f"  {'Tracker':<12} {'RMS':>10} {'Max':>10} {'P95':>10} {'Mean':>10} {'ms':>8}")
        print(f"  {'-'*11} {'-'*10} {'-'*10} {'-'*10} {'-'*10} {'-'*8}")
        sorted_r = sorted(results, key=lambda r: r.rms)
        best = sorted_r[0].rms if sorted_r and sorted_r[0].rms < float('inf') else 999
        for r in results:
            if r.rms == float('inf'):
                print(f"  {r.name:<12} {'FAIL':>10}")
            else:
                mark = " *" if r.rms == best else ""
                print(f"  {r.name:<12} {r.rms:>10.2f} {r.max_err:>10.2f} "
                      f"{r.p95:>10.2f} {r.mean_err:>10.2f} {r.time_ms:>7.0f}{mark}")

def run_all():
    print("NX-MIMOSA v4.2 MULTI-DOMAIN OPEN BENCHMARK")
    print("=" * 100)
    print("Libraries: Stone Soup v1.8 | FilterPy v1.4.5 | PyKalman v0.11 | NX-MIMOSA v4.2")
    print(f"Seed: {SEED} | All trackers get identical truth + noise per scenario")
    print()

    all_results = {}  # {scenario_id: (Scenario, [Metrics])}
    
    for sc in SCENARIOS:
        print(f"  Running {sc.id}: {sc.name} ({sc.domain})...", flush=True)
        sid, results = run_scenario(sc)
        all_results[sid] = (sc, results)

    # Per-domain detailed reports
    for domain in ["ATC", "AVIATION", "MILITARY", "AUTOMOTIVE", "SPACE"]:
        domain_results = [(sc, res) for sc, res in
                          [(all_results[s.id][0], all_results[s.id][1])
                           for s in SCENARIOS if s.domain == domain]]
        print_domain_report(domain, domain_results)

    # ── Grand Summary: Best-of-class per library ──
    print(f"\n{'='*100}")
    print("GRAND SUMMARY — Best-of-class RMS (meters)")
    print(f"{'='*100}")
    print(f"  Stone Soup = best of {{CV, CA, Singer}}")
    print(f"  FilterPy   = best of {{CV, IMM}}")
    print(f"  PyKalman   = best of {{CV, CA}}")
    print(f"  NX-MIMOSA  = domain preset per domain (auto-config model bank + AOS + Q)\n")

    header = f"  {'ID':<5} {'Scenario':<25} {'Domain':<10} {'SS best':>10} {'FP best':>10} {'PK best':>10} {'NX v4.2':>10} {'Winner':<12}"
    print(header)
    print(f"  {'-'*4} {'-'*24} {'-'*9} {'-'*10} {'-'*10} {'-'*10} {'-'*10} {'-'*11}")

    wins = {"SS": 0, "FP": 0, "PK": 0, "NX": 0}
    domain_wins = {d: {"SS":0,"FP":0,"PK":0,"NX":0} for d in DOMAINS}
    all_rms = {"SS": [], "FP": [], "PK": [], "NX": []}

    for sc in SCENARIOS:
        _, results = all_results[sc.id]
        ss = best_of_lib(results, ["SS"])
        fp = best_of_lib(results, ["FP"])
        pk = best_of_lib(results, ["PK"])
        nx = best_of_lib(results, ["NX"])

        vals = {"SS": ss.rms, "FP": fp.rms, "PK": pk.rms, "NX": nx.rms}
        valid_vals = {k: v for k, v in vals.items() if v < float('inf')}
        winner = min(valid_vals, key=valid_vals.get) if valid_vals else "—"

        for k in vals:
            if vals[k] < float('inf'):
                all_rms[k].append(vals[k])

        if winner != "—":
            wins[winner] += 1
            domain_wins[sc.domain][winner] += 1

        def fmt(v): return f"{v:>10.2f}" if v < float('inf') else f"{'FAIL':>10}"
        star = {k: " *" if k == winner else "" for k in vals}
        line = (f"  {sc.id:<5} {sc.name:<25} {sc.domain:<10} "
                f"{fmt(ss.rms)}{star['SS']:<2}{fmt(fp.rms)}{star['FP']:<2}"
                f"{fmt(pk.rms)}{star['PK']:<2}{fmt(nx.rms)}{star['NX']:<2}")
        win_name = {"SS":"StoneSoup","FP":"FilterPy","PK":"PyKalman","NX":"NX-MIMOSA"}.get(winner, "—")
        line += f" {win_name}"
        print(line)

    # Domain summary
    print(f"\n{'='*100}")
    print("WINS PER DOMAIN")
    print(f"{'='*100}")
    print(f"  {'Domain':<12} {'SS':>6} {'FP':>6} {'PK':>6} {'NX':>6} {'Total':>6}")
    for d in DOMAINS:
        dw = domain_wins[d]
        total = sum(dw.values())
        print(f"  {d:<12} {dw['SS']:>6} {dw['FP']:>6} {dw['PK']:>6} {dw['NX']:>6} {total:>6}")
    print(f"  {'TOTAL':<12} {wins['SS']:>6} {wins['FP']:>6} {wins['PK']:>6} {wins['NX']:>6} {sum(wins.values()):>6}")

    # Overall averages
    print(f"\n{'='*100}")
    print("OVERALL AVERAGES")
    print(f"{'='*100}")
    for lib, label in [("SS","Stone Soup"),("FP","FilterPy"),("PK","PyKalman"),("NX","NX-MIMOSA v4.2")]:
        if all_rms[lib]:
            print(f"  {label:<20} avg RMS = {np.mean(all_rms[lib]):>10.2f}m  "
                  f"(across {len(all_rms[lib])} scenarios)  wins: {wins[lib]}/19")

    # Honest disclosure
    print(f"\n{'='*100}")
    print("HONEST DISCLOSURE — Where NX-MIMOSA Loses")
    print(f"{'='*100}")
    for sc in SCENARIOS:
        _, results = all_results[sc.id]
        ss = best_of_lib(results, ["SS"])
        fp = best_of_lib(results, ["FP"])
        pk = best_of_lib(results, ["PK"])
        nx = best_of_lib(results, ["NX"])
        if nx.rms == float('inf'): continue
        for lib, label, val in [("SS","Stone Soup",ss.rms),("FP","FilterPy",fp.rms),("PK","PyKalman",pk.rms)]:
            if val < nx.rms and val < float('inf'):
                pct = (nx.rms - val) / val * 100
                print(f"  {sc.id} {sc.name}: {label} wins — {val:.2f}m vs NX {nx.rms:.2f}m (+{pct:.1f}% worse)")
    
    nx_no_loss = all(
        best_of_lib(all_results[sc.id][1], ["NX"]).rms <=
        min(best_of_lib(all_results[sc.id][1], [p]).rms
            for p in ["SS","FP","PK"])
        for sc in SCENARIOS
    )
    if nx_no_loss:
        print("  None — NX-MIMOSA wins or ties all scenarios.")

    print(f"\n{'='*100}")
    print("METHODOLOGY")
    print(f"{'='*100}")
    print("  - All trackers get identical truth + measurement noise (seed=42)")
    print("  - Q values are domain-appropriate defaults, NOT per-scenario optimized")
    print("  - Stone Soup: best of 3 models (CV, CA, Singer) per scenario")
    print("  - FilterPy: best of 2 (CV, IMM with domain-matched omega)")
    print("  - PyKalman: best of 2 (CV, CA)")
    print("  - NX-MIMOSA: ONE domain preset per domain (domain= parameter)")
    print("    Each preset auto-configures model bank, AOS, TPM, Q scaling")
    print("  - FilterPy IMM CT omega adapted per domain (generous)")
    print("  - No oracle info given to any tracker")
    print(f"\nReproduce: pip install stonesoup filterpy pykalman numpy")
    print(f"           python benchmarks/multi_domain_benchmark.py")
    print(f"{'='*100}")

    # Save JSON
    json_out = {}
    for sc in SCENARIOS:
        _, results = all_results[sc.id]
        json_out[sc.id] = {
            "name": sc.name, "domain": sc.domain,
            "trackers": {r.name: {"rms": round(r.rms, 3), "max": round(r.max_err, 3),
                                   "p95": round(r.p95, 3), "time_ms": round(r.time_ms, 1)}
                         for r in results if r.rms < float('inf')}
        }
    json_path = os.path.join(os.path.dirname(__file__), "multi_domain_results.json")
    with open(json_path, "w") as f:
        json.dump(json_out, f, indent=2)
    print(f"\nJSON: {json_path}")

    # Save Markdown
    md_path = os.path.join(os.path.dirname(__file__), "MULTI_DOMAIN_RESULTS.md")
    with open(md_path, "w") as f:
        f.write("## Multi-Domain Open Benchmark: NX-MIMOSA v4.2\n\n")
        f.write("**19 scenarios across 5 domains** — fully reproducible.\n\n")
        f.write("```bash\npip install stonesoup filterpy pykalman numpy\n")
        f.write("python benchmarks/multi_domain_benchmark.py\n```\n\n")
        f.write("### Results — Best-of-class RMS (meters)\n\n")
        f.write("| ID | Scenario | Domain | Stone Soup | FilterPy | PyKalman | NX-MIMOSA | Winner |\n")
        f.write("|---|---|---|---:|---:|---:|---:|---|\n")
        for sc in SCENARIOS:
            _, results = all_results[sc.id]
            ss = best_of_lib(results, ["SS"])
            fp = best_of_lib(results, ["FP"])
            pk = best_of_lib(results, ["PK"])
            nx = best_of_lib(results, ["NX"])
            vals = {"SS": ss.rms, "FP": fp.rms, "PK": pk.rms, "NX": nx.rms}
            valid = {k: v for k, v in vals.items() if v < float('inf')}
            winner = min(valid, key=valid.get) if valid else "—"
            def mfmt(v, k):
                if v == float('inf'): return "FAIL"
                s = f"{v:.2f}"
                return f"**{s}** ★" if k == winner else s
            wname = {"SS":"Stone Soup","FP":"FilterPy","PK":"PyKalman","NX":"**NX-MIMOSA**"}.get(winner,"—")
            f.write(f"| {sc.id} | {sc.name} | {sc.domain} | "
                    f"{mfmt(ss.rms,'SS')} | {mfmt(fp.rms,'FP')} | {mfmt(pk.rms,'PK')} | "
                    f"{mfmt(nx.rms,'NX')} | {wname} |\n")
        f.write(f"\n**Overall: Stone Soup {wins['SS']}/19 | FilterPy {wins['FP']}/19 | "
                f"PyKalman {wins['PK']}/19 | NX-MIMOSA {wins['NX']}/19**\n")
    print(f"Markdown: {md_path}")

    return all_results, wins

if __name__ == "__main__":
    run_all()
