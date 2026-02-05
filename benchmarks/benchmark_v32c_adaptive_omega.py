#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════════
NX-MIMOSA v3.2c BENCHMARK — Adaptive Omega Fix
═══════════════════════════════════════════════════════════════════════════════════

v3.2c = STRICTLY v3.1 + adaptive omega estimation. Nothing else changed.

Root cause of v3.1 losses on 4/8 scenarios:
  Fixed omega=±0.1 rad/s mismatched for:
  - Dogfight:  needs ±0.31 rad/s (3× off)
  - UAV Swarm: needs ±0.39 rad/s (4× off)
  - SAM:       needs ±0.20 rad/s (2× off)
  - Ballistic: needs ±0.004 rad/s (25× off)

Fix: estimate omega online from state velocity cross-product,
     EMA smooth, clip to [0.02, 0.5] rad/s.

COMPARED AGAINST:
  1. NX-MIMOSA v3.1          (3-model, fixed omega=±0.1)
  2. NX-MIMOSA v3.2c         (3-model, adaptive omega)
  3. FilterPy IMM            (Roger Labbe, 3-model, fixed omega=±0.1)
  4. Stone Soup EKF+RTS      (UK DSTL, CV model + Kalman smoother)
  5. EKF-CA                  (single model, high-Q baseline)

100 Monte Carlo runs, seed=42, 8 defense scenarios.

Author: Dr. Mladen Mešter / Nexellum d.o.o.
License: AGPL v3
═══════════════════════════════════════════════════════════════════════════════════
"""

import numpy as np
from numpy.linalg import inv
from scipy.linalg import block_diag
import time
import argparse
from dataclasses import dataclass, field
from typing import List, Tuple, Dict
import warnings
warnings.filterwarnings('ignore')


# ═══════════════════════════════════════════════════════════════════════════════
# SHARED PHYSICS (identical for all algorithms)
# ═══════════════════════════════════════════════════════════════════════════════

def cv_matrices(dt, q):
    F = np.array([[1,dt,0,0],[0,1,0,0],[0,0,1,dt],[0,0,0,1]])
    q2 = q**2
    Qb = np.array([[dt**3/3, dt**2/2],[dt**2/2, dt]]) * q2
    return F, block_diag(Qb, Qb)

def ct_matrices(dt, q, omega):
    if abs(omega) < 1e-8:
        return cv_matrices(dt, q)
    s, c = np.sin(omega*dt), np.cos(omega*dt)
    F = np.array([
        [1, s/omega, 0, -(1-c)/omega],
        [0, c, 0, -s],
        [0, (1-c)/omega, 1, s/omega],
        [0, s, 0, c]
    ])
    q2 = q**2
    Qb = np.array([[dt**3/3, dt**2/2],[dt**2/2, dt]]) * q2
    return F, block_diag(Qb, Qb)

H_MAT = np.array([[1,0,0,0],[0,0,1,0]])

def R_mat(sigma):
    return np.eye(2) * sigma**2


# ═══════════════════════════════════════════════════════════════════════════════
# SCENARIOS
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class Scenario:
    name: str
    duration: float
    dt: float
    sigma_pos: float
    sigma_vel: float
    segments: list  # [(dur, ax, ay), ...]

    def gen_truth(self):
        states = []
        state = np.array([0.0, 300.0, 0.0, 0.0])
        for seg_dur, ax, ay in self.segments:
            for _ in range(int(seg_dur / self.dt)):
                states.append(state.copy())
                x, vx, y, vy = state
                state = np.array([
                    x + vx*self.dt + 0.5*ax*self.dt**2,
                    vx + ax*self.dt,
                    y + vy*self.dt + 0.5*ay*self.dt**2,
                    vy + ay*self.dt
                ])
        return np.array(states)

    def gen_meas(self, truth, rng):
        N = len(truth)
        noise = rng.normal(0, self.sigma_pos, (N, 2))
        return np.column_stack([truth[:,0], truth[:,2]]) + noise


def get_scenarios():
    g = 9.81
    return [
        Scenario("Missile Terminal (Mach 4, 9g)", 20, 0.05, 15, 5,
                 [(5,0,0),(5,0,9*g),(5,-5*g,-9*g),(5,5*g,4*g)]),
        Scenario("Hypersonic Glide (Mach 5, 2g)", 30, 0.1, 50, 3,
                 [(7.5,0,2*g),(7.5,0,-2*g),(7.5,0,2*g),(7.5,0,-2*g)]),
        Scenario("SAM Engagement (300m/s, 6g)", 15, 0.05, 10, 8,
                 [(3,0,0),(4,3*g,6*g),(4,-6*g,-3*g),(4,0,0)]),
        Scenario("Dogfight BFM (250m/s, 8g)", 20, 0.05, 8, 10,
                 [(5,0,8*g),(5,-4*g,-8*g),(5,8*g,0),(5,-8*g,4*g)]),
        Scenario("Cruise Missile (250m/s, 3g)", 30, 0.1, 20, 2,
                 [(10,0,0),(5,0,3*g),(5,0,-3*g),(10,0,0)]),
        Scenario("Ballistic Reentry (Mach 7)", 40, 0.2, 100, 1,
                 [(15,0,0),(5,0,-1*g),(10,0,0),(10,0.5*g,0.5*g)]),
        Scenario("UAV Swarm (50m/s, 2g)", 30, 0.1, 5, 3,
                 [(7.5,2*g,0),(7.5,-2*g,2*g),(7.5,0,-2*g),(7.5,2*g,2*g)]),
        Scenario("Stealth Aircraft (200m/s, 4g)", 30, 0.5, 30, 5,
                 [(10,0,0),(5,0,4*g),(5,-4*g,0),(10,0,-2*g)]),
    ]


# ═══════════════════════════════════════════════════════════════════════════════
# NX-MIMOSA v3.1 (BASELINE — fixed omega=±0.1)
# ═══════════════════════════════════════════════════════════════════════════════

class NxMimosaV31:
    """Exact v3.1: 3-model IMM + VS-TPM + Adaptive Q + Per-Model RTS."""

    def __init__(self, dt, sigma, q, omega_fixed=0.1):
        self.dt = dt
        self.H = H_MAT
        self.R = R_mat(sigma)
        self.q = q
        self.omegas = [0.0, omega_fixed, -omega_fixed]
        self.nm = 3
        self.mu0 = np.array([0.8, 0.1, 0.1])
        self.TPM_base = np.array([
            [0.95, 0.025, 0.025],
            [0.05, 0.90, 0.05],
            [0.05, 0.05, 0.90]
        ])

    def _FQ(self, j, q_scale=1.0):
        return ct_matrices(self.dt, self.q * q_scale, self.omegas[j])

    def _vs_tpm(self, mu):
        mx = np.max(mu)
        ps = 0.98 if mx > 0.9 else (0.95 if mx > 0.7 else 0.90)
        T = np.full((3,3), (1-ps)/2)
        np.fill_diagonal(T, ps)
        return T

    def _nis_q_scale(self, innov, S):
        nis = innov @ inv(S) @ innov
        r = nis / len(innov)
        if r > 3: return min(r, 10.0)
        if r < 0.3: return max(r, 0.1)
        return 1.0

    def run(self, meas):
        N = len(meas)
        nx, nm = 4, self.nm
        xf = np.zeros((nm, N, nx))
        Pf = np.zeros((nm, N, nx, nx))
        xp = np.zeros((nm, N, nx))
        Pp = np.zeros((nm, N, nx, nx))
        mu_h = np.zeros((N, nm))

        x0 = np.array([meas[0,0], 0, meas[0,1], 0])
        P0 = np.diag([self.R[0,0], 100, self.R[1,1], 100])
        for j in range(nm):
            xf[j,0] = x0.copy(); Pf[j,0] = P0.copy()
            xp[j,0] = x0.copy(); Pp[j,0] = P0.copy()

        mu = self.mu0.copy()
        mu_h[0] = mu

        for k in range(1, N):
            z = meas[k]
            T = self._vs_tpm(mu)
            c_bar = T.T @ mu
            c_bar = np.maximum(c_bar, 1e-30)
            mm = np.zeros((nm, nm))
            for i in range(nm):
                for j in range(nm):
                    mm[i,j] = T[i,j] * mu[i] / c_bar[j]

            xm = np.zeros((nm, nx)); Pm = np.zeros((nm, nx, nx))
            for j in range(nm):
                for i in range(nm):
                    xm[j] += mm[i,j] * xf[i,k-1]
                for i in range(nm):
                    d = xf[i,k-1] - xm[j]
                    Pm[j] += mm[i,j] * (Pf[i,k-1] + np.outer(d,d))

            lik = np.zeros(nm)
            for j in range(nm):
                Fj, Qj = self._FQ(j)
                xp_j = Fj @ xm[j]
                Pp_j = Fj @ Pm[j] @ Fj.T + Qj
                inn = z - self.H @ xp_j
                S = self.H @ Pp_j @ self.H.T + self.R
                qs = self._nis_q_scale(inn, S)
                if qs != 1.0:
                    _, Qa = self._FQ(j, qs)
                    Pp_j = Fj @ Pm[j] @ Fj.T + Qa
                    S = self.H @ Pp_j @ self.H.T + self.R
                xp[j,k] = xp_j; Pp[j,k] = Pp_j
                K = Pp_j @ self.H.T @ inv(S)
                xf[j,k] = xp_j + K @ inn
                IKH = np.eye(nx) - K @ self.H
                Pf[j,k] = IKH @ Pp_j @ IKH.T + K @ self.R @ K.T
                sgn, ld = np.linalg.slogdet(S)
                ll = -0.5 * (inn @ inv(S) @ inn + ld + 2*np.log(2*np.pi))
                lik[j] = np.exp(np.clip(ll, -500, 500))

            mu = c_bar * lik
            s = np.sum(mu)
            mu = mu / s if s > 1e-300 else np.ones(nm)/nm
            mu_h[k] = mu

        # Forward combined
        xc_fwd = np.zeros((N, nx))
        for k in range(N):
            for j in range(nm):
                xc_fwd[k] += mu_h[k,j] * xf[j,k]

        # Per-model RTS backward
        xs_j = np.zeros((nm, N, nx))
        Ps_j = np.zeros((nm, N, nx, nx))
        for j in range(nm):
            xs_j[j, N-1] = xf[j, N-1].copy()
            Ps_j[j, N-1] = Pf[j, N-1].copy()
            for k in range(N-2, -1, -1):
                Fj, _ = self._FQ(j)
                Pi = inv(Pp[j,k+1] + np.eye(nx)*1e-10)
                G = Pf[j,k] @ Fj.T @ Pi
                xs_j[j,k] = xf[j,k] + G @ (xs_j[j,k+1] - xp[j,k+1])
                Ps_j[j,k] = Pf[j,k] + G @ (Ps_j[j,k+1] - Pp[j,k+1]) @ G.T

        xs = np.zeros((N, nx)); Ps = np.zeros((N, nx, nx))
        for k in range(N):
            for j in range(nm):
                xs[k] += mu_h[k,j] * xs_j[j,k]
            for j in range(nm):
                d = xs_j[j,k] - xs[k]
                Ps[k] += mu_h[k,j] * (Ps_j[j,k] + np.outer(d,d))

        return xs, Ps, xc_fwd


# ═══════════════════════════════════════════════════════════════════════════════
# NX-MIMOSA v3.2c (v3.1 + ADAPTIVE OMEGA — ONLY CHANGE)
# ═══════════════════════════════════════════════════════════════════════════════

class NxMimosaV32c(NxMimosaV31):
    """
    v3.2c = v3.1 + online adaptive omega estimation.

    ONLY CHANGE: omegas[1] and omegas[2] are updated each step based on
    estimated turn rate from the combined state velocity cross-product.

    Physics: omega ≈ |v × a| / |v|² where a = dv/dt estimated from state.
    Implementation: EMA-smoothed, clipped to [0.02, 0.5] rad/s.
    """

    def __init__(self, dt, sigma, q, omega_init=0.1):
        super().__init__(dt, sigma, q, omega_init)
        self.omega_init = omega_init
        self.omega_ema_alpha = 0.12   # EMA smoothing factor
        self.omega_min = 0.02         # minimum omega (near-straight)
        self.omega_max = 0.50         # maximum omega (extreme turn)
        self.warmup_steps = 10        # steps before adaptation starts

    def run(self, meas):
        N = len(meas)
        nx, nm = 4, self.nm
        xf = np.zeros((nm, N, nx))
        Pf = np.zeros((nm, N, nx, nx))
        xp = np.zeros((nm, N, nx))
        Pp = np.zeros((nm, N, nx, nx))
        mu_h = np.zeros((N, nm))

        x0 = np.array([meas[0,0], 0, meas[0,1], 0])
        P0 = np.diag([self.R[0,0], 100, self.R[1,1], 100])
        for j in range(nm):
            xf[j,0] = x0.copy(); Pf[j,0] = P0.copy()
            xp[j,0] = x0.copy(); Pp[j,0] = P0.copy()

        mu = self.mu0.copy()
        mu_h[0] = mu

        # Adaptive omega state
        omega_current = self.omega_init
        omega_ema = self.omega_init
        prev_combined = x0.copy()

        # Storage for F matrices used (needed for correct backward pass)
        F_used = np.zeros((nm, N, nx, nx))
        for j in range(nm):
            F_used[j, 0] = np.eye(nx)

        for k in range(1, N):
            z = meas[k]

            # ─── ADAPTIVE OMEGA (the ONLY difference from v3.1) ───
            if k > self.warmup_steps:
                # Combined estimate from previous step
                x_prev = np.zeros(nx)
                for j in range(nm):
                    x_prev += mu_h[k-1, j] * xf[j, k-1]

                vx, vy = x_prev[1], x_prev[3]
                v_sq = vx**2 + vy**2

                if v_sq > 100.0:  # minimum speed² threshold (10 m/s)
                    # Estimate acceleration from state change
                    dvx = x_prev[1] - prev_combined[1]
                    dvy = x_prev[3] - prev_combined[3]
                    ax_est = dvx / self.dt
                    ay_est = dvy / self.dt

                    # omega = |v × a| / |v|²  (2D cross product magnitude)
                    omega_inst = abs(vx * ay_est - vy * ax_est) / v_sq

                    # EMA smoothing
                    omega_ema = (self.omega_ema_alpha * omega_inst +
                                 (1 - self.omega_ema_alpha) * omega_ema)

                    # Clip for stability
                    omega_current = np.clip(omega_ema, self.omega_min, self.omega_max)

                prev_combined = x_prev.copy()

            # Update model bank with adapted omega
            self.omegas = [0.0, omega_current, -omega_current]
            # ─── END ADAPTIVE OMEGA ───

            T = self._vs_tpm(mu)
            c_bar = T.T @ mu
            c_bar = np.maximum(c_bar, 1e-30)
            mm = np.zeros((nm, nm))
            for i in range(nm):
                for j in range(nm):
                    mm[i,j] = T[i,j] * mu[i] / c_bar[j]

            xm = np.zeros((nm, nx)); Pm = np.zeros((nm, nx, nx))
            for j in range(nm):
                for i in range(nm):
                    xm[j] += mm[i,j] * xf[i,k-1]
                for i in range(nm):
                    d = xf[i,k-1] - xm[j]
                    Pm[j] += mm[i,j] * (Pf[i,k-1] + np.outer(d,d))

            lik = np.zeros(nm)
            for j in range(nm):
                Fj, Qj = self._FQ(j)
                F_used[j, k] = Fj  # store for backward pass
                xp_j = Fj @ xm[j]
                Pp_j = Fj @ Pm[j] @ Fj.T + Qj
                inn = z - self.H @ xp_j
                S = self.H @ Pp_j @ self.H.T + self.R
                qs = self._nis_q_scale(inn, S)
                if qs != 1.0:
                    _, Qa = self._FQ(j, qs)
                    Pp_j = Fj @ Pm[j] @ Fj.T + Qa
                    S = self.H @ Pp_j @ self.H.T + self.R
                xp[j,k] = xp_j; Pp[j,k] = Pp_j
                K = Pp_j @ self.H.T @ inv(S)
                xf[j,k] = xp_j + K @ inn
                IKH = np.eye(nx) - K @ self.H
                Pf[j,k] = IKH @ Pp_j @ IKH.T + K @ self.R @ K.T
                sgn, ld = np.linalg.slogdet(S)
                ll = -0.5 * (inn @ inv(S) @ inn + ld + 2*np.log(2*np.pi))
                lik[j] = np.exp(np.clip(ll, -500, 500))

            mu = c_bar * lik
            s = np.sum(mu)
            mu = mu / s if s > 1e-300 else np.ones(nm)/nm
            mu_h[k] = mu

        # Forward combined
        xc_fwd = np.zeros((N, nx))
        for k in range(N):
            for j in range(nm):
                xc_fwd[k] += mu_h[k,j] * xf[j,k]

        # Per-model RTS backward (uses stored F_used for correctness)
        xs_j = np.zeros((nm, N, nx))
        Ps_j = np.zeros((nm, N, nx, nx))
        for j in range(nm):
            xs_j[j, N-1] = xf[j, N-1].copy()
            Ps_j[j, N-1] = Pf[j, N-1].copy()
            for k in range(N-2, -1, -1):
                Fj = F_used[j, k+1]  # use the F that was actually used at step k+1
                Pi = inv(Pp[j,k+1] + np.eye(nx)*1e-10)
                G = Pf[j,k] @ Fj.T @ Pi
                xs_j[j,k] = xf[j,k] + G @ (xs_j[j,k+1] - xp[j,k+1])
                Ps_j[j,k] = Pf[j,k] + G @ (Ps_j[j,k+1] - Pp[j,k+1]) @ G.T

        xs = np.zeros((N, nx)); Ps = np.zeros((N, nx, nx))
        for k in range(N):
            for j in range(nm):
                xs[k] += mu_h[k,j] * xs_j[j,k]
            for j in range(nm):
                d = xs_j[j,k] - xs[k]
                Ps[k] += mu_h[k,j] * (Ps_j[j,k] + np.outer(d,d))

        return xs, Ps, xc_fwd


# ═══════════════════════════════════════════════════════════════════════════════
# COMPETITORS (Gold Standard)
# ═══════════════════════════════════════════════════════════════════════════════

def run_filterpy_imm(meas, dt, sigma, q):
    """FilterPy IMMEstimator — Roger Labbe, 3-model, fixed omega=±0.1."""
    from filterpy.kalman import KalmanFilter, IMMEstimator
    filters = []
    for omega in [0.0, 0.1, -0.1]:
        f = KalmanFilter(dim_x=4, dim_z=2)
        F, Q = ct_matrices(dt, q, omega)
        f.F = F; f.Q = Q; f.H = H_MAT; f.R = R_mat(sigma)
        f.x = np.array([[meas[0,0]], [0], [meas[0,1]], [0]])
        f.P = np.diag([sigma**2, 100, sigma**2, 100])
        filters.append(f)
    mu = np.array([0.8, 0.1, 0.1])
    M = np.array([[0.95,0.025,0.025],[0.05,0.90,0.05],[0.05,0.05,0.90]])
    imm = IMMEstimator(filters, mu, M)
    N = len(meas)
    states = np.zeros((N, 4))
    states[0] = np.array([meas[0,0], 0, meas[0,1], 0])
    for k in range(1, N):
        imm.predict()
        imm.update(meas[k].reshape(2,1))
        states[k] = imm.x.flatten()
    return states


def run_stonesoup_ekf_rts(meas, dt, sigma, q):
    """Stone Soup EKF + RTS Smoother — UK DSTL production framework."""
    from stonesoup.types.state import GaussianState
    from stonesoup.types.detection import Detection
    from stonesoup.types.track import Track
    from stonesoup.types.array import StateVector, CovarianceMatrix
    from stonesoup.types.hypothesis import SingleHypothesis
    from stonesoup.models.transition.linear import (
        CombinedLinearGaussianTransitionModel, ConstantVelocity)
    from stonesoup.models.measurement.linear import LinearGaussian
    from stonesoup.predictor.kalman import KalmanPredictor
    from stonesoup.updater.kalman import KalmanUpdater
    from stonesoup.smoother.kalman import KalmanSmoother
    from datetime import datetime, timedelta

    tm = CombinedLinearGaussianTransitionModel([
        ConstantVelocity(q**2), ConstantVelocity(q**2)])
    mm = LinearGaussian(ndim_state=4, mapping=(0,2),
                        noise_covar=CovarianceMatrix(np.eye(2)*sigma**2))
    pred = KalmanPredictor(tm)
    upd = KalmanUpdater(mm)
    sm = KalmanSmoother(tm)

    t0 = datetime.now()
    prior = GaussianState(
        state_vector=StateVector([[meas[0,0]],[0],[meas[0,1]],[0]]),
        covar=CovarianceMatrix(np.diag([sigma**2,100,sigma**2,100])),
        timestamp=t0)
    track = Track([prior])

    N = len(meas)
    for k in range(1, N):
        tk = t0 + timedelta(seconds=k*dt)
        det = Detection(state_vector=StateVector([[meas[k,0]],[meas[k,1]]]),
                        timestamp=tk, measurement_model=mm)
        prediction = pred.predict(track[-1], timestamp=tk)
        hyp = SingleHypothesis(prediction, det)
        track.append(upd.update(hyp))

    smoothed = sm.smooth(track)
    states = np.zeros((N, 4))
    for k, st in enumerate(smoothed):
        states[k] = st.state_vector.flatten()
    return states


def run_ekf_ca(meas, dt, sigma, q):
    """EKF with 5× process noise (maneuvering baseline)."""
    F, Q = cv_matrices(dt, q * 5.0)
    R = R_mat(sigma)
    N = len(meas)
    x = np.array([meas[0,0], 0, meas[0,1], 0])
    P = np.diag([sigma**2, 100, sigma**2, 100])
    states = np.zeros((N, 4))
    states[0] = x
    for k in range(1, N):
        x = F @ x; P = F @ P @ F.T + Q
        inn = meas[k] - H_MAT @ x
        S = H_MAT @ P @ H_MAT.T + R
        K = P @ H_MAT.T @ inv(S)
        x = x + K @ inn
        P = (np.eye(4) - K @ H_MAT) @ P
        states[k] = x
    return states


# ═══════════════════════════════════════════════════════════════════════════════
# METRICS
# ═══════════════════════════════════════════════════════════════════════════════

def pos_rmse(est, truth):
    dx = est[:,0] - truth[:,0]
    dy = est[:,2] - truth[:,2]
    return np.sqrt(np.mean(dx**2 + dy**2))

def vel_rmse(est, truth):
    dvx = est[:,1] - truth[:,1]
    dvy = est[:,3] - truth[:,3]
    return np.sqrt(np.mean(dvx**2 + dvy**2))


# ═══════════════════════════════════════════════════════════════════════════════
# BENCHMARK RUNNER
# ═══════════════════════════════════════════════════════════════════════════════

def run_benchmark(n_runs=100, seed=42):
    scenarios = get_scenarios()

    algo_names = [
        "NX-MIMOSA v3.1 (smooth)",
        "NX-MIMOSA v3.2c (smooth)",
        "NX-MIMOSA v3.1 (fwd)",
        "NX-MIMOSA v3.2c (fwd)",
        "FilterPy IMM",
        "StoneSoup EKF+RTS",
        "EKF-CA (5×Q)",
    ]

    # {scenario: {algo: [rmse_per_run]}}
    results = {s.name: {a: [] for a in algo_names} for s in scenarios}
    vresults = {s.name: {a: [] for a in algo_names} for s in scenarios}
    timings = {s.name: {a: [] for a in algo_names} for s in scenarios}

    print("=" * 110)
    print(f"NX-MIMOSA v3.2c BENCHMARK — Adaptive Omega Fix")
    print(f"Monte Carlo: {n_runs} runs | Seed: {seed}")
    print("=" * 110)
    print()
    print("CHANGE LOG v3.1 → v3.2c:")
    print("  ONLY CHANGE: omega[CT±] adapted online via EMA(|v×a|/v², α=0.12)")
    print("  Clipped to [0.02, 0.5] rad/s. Warmup: 10 steps. Everything else IDENTICAL.")
    print()

    for si, sc in enumerate(scenarios):
        truth = sc.gen_truth()
        N = len(truth)

        print(f"{'─'*110}")
        print(f"SCENARIO {si+1}/8: {sc.name}  (N={N}, dt={sc.dt}s)")
        print(f"{'─'*110}")

        for run in range(n_runs):
            rng = np.random.default_rng(seed * 1000 + si * 100 + run)
            meas = sc.gen_meas(truth, rng)

            # v3.1 smooth + fwd
            m31 = NxMimosaV31(sc.dt, sc.sigma_pos, sc.sigma_vel)
            t0 = time.perf_counter()
            xs31, _, xf31 = m31.run(meas)
            t31 = (time.perf_counter() - t0) * 1000
            results[sc.name]["NX-MIMOSA v3.1 (smooth)"].append(pos_rmse(xs31, truth))
            results[sc.name]["NX-MIMOSA v3.1 (fwd)"].append(pos_rmse(xf31, truth))
            vresults[sc.name]["NX-MIMOSA v3.1 (smooth)"].append(vel_rmse(xs31, truth))
            vresults[sc.name]["NX-MIMOSA v3.1 (fwd)"].append(vel_rmse(xf31, truth))
            timings[sc.name]["NX-MIMOSA v3.1 (smooth)"].append(t31)
            timings[sc.name]["NX-MIMOSA v3.1 (fwd)"].append(t31)

            # v3.2c smooth + fwd
            m32c = NxMimosaV32c(sc.dt, sc.sigma_pos, sc.sigma_vel)
            t0 = time.perf_counter()
            xs32c, _, xf32c = m32c.run(meas)
            t32c = (time.perf_counter() - t0) * 1000
            results[sc.name]["NX-MIMOSA v3.2c (smooth)"].append(pos_rmse(xs32c, truth))
            results[sc.name]["NX-MIMOSA v3.2c (fwd)"].append(pos_rmse(xf32c, truth))
            vresults[sc.name]["NX-MIMOSA v3.2c (smooth)"].append(vel_rmse(xs32c, truth))
            vresults[sc.name]["NX-MIMOSA v3.2c (fwd)"].append(vel_rmse(xf32c, truth))
            timings[sc.name]["NX-MIMOSA v3.2c (smooth)"].append(t32c)
            timings[sc.name]["NX-MIMOSA v3.2c (fwd)"].append(t32c)

            # FilterPy IMM
            t0 = time.perf_counter()
            ef = run_filterpy_imm(meas, sc.dt, sc.sigma_pos, sc.sigma_vel)
            tf = (time.perf_counter() - t0) * 1000
            results[sc.name]["FilterPy IMM"].append(pos_rmse(ef, truth))
            vresults[sc.name]["FilterPy IMM"].append(vel_rmse(ef, truth))
            timings[sc.name]["FilterPy IMM"].append(tf)

            # Stone Soup
            t0 = time.perf_counter()
            try:
                ess = run_stonesoup_ekf_rts(meas, sc.dt, sc.sigma_pos, sc.sigma_vel)
                tss = (time.perf_counter() - t0) * 1000
                results[sc.name]["StoneSoup EKF+RTS"].append(pos_rmse(ess, truth))
                vresults[sc.name]["StoneSoup EKF+RTS"].append(vel_rmse(ess, truth))
            except Exception:
                tss = (time.perf_counter() - t0) * 1000
                results[sc.name]["StoneSoup EKF+RTS"].append(np.nan)
                vresults[sc.name]["StoneSoup EKF+RTS"].append(np.nan)
            timings[sc.name]["StoneSoup EKF+RTS"].append(tss)

            # EKF-CA
            t0 = time.perf_counter()
            eca = run_ekf_ca(meas, sc.dt, sc.sigma_pos, sc.sigma_vel)
            tca = (time.perf_counter() - t0) * 1000
            results[sc.name]["EKF-CA (5×Q)"].append(pos_rmse(eca, truth))
            vresults[sc.name]["EKF-CA (5×Q)"].append(vel_rmse(eca, truth))
            timings[sc.name]["EKF-CA (5×Q)"].append(tca)

            if run == 0 and si == 0:
                print(f"  First run timing: v3.1={t31:.0f}ms v3.2c={t32c:.0f}ms "
                      f"FilterPy={tf:.0f}ms SS={tss:.0f}ms EKF-CA={tca:.1f}ms")

        # Per-scenario results
        print(f"\n  {'Algorithm':<32} {'Pos RMSE':>10} {'Vel RMSE':>12} {'Time':>10}")
        print(f"  {'─'*66}")
        sorted_a = sorted(algo_names, key=lambda a: np.nanmean(results[sc.name][a]))
        for rank, a in enumerate(sorted_a):
            r = np.nanmean(results[sc.name][a])
            v = np.nanmean(vresults[sc.name][a])
            t = np.nanmean(timings[sc.name][a])
            medal = " 🥇" if rank == 0 else " 🥈" if rank == 1 else ""
            print(f"  {a:<32} {r:>8.2f} m {v:>10.2f} m/s {t:>8.1f} ms{medal}")

    # ═══════════════════════════════════════════════════════════════════════
    # GRAND SUMMARY
    # ═══════════════════════════════════════════════════════════════════════
    print("\n" + "=" * 130)
    print("GRAND SUMMARY — Position RMSE (m)")
    print("=" * 130)

    # Header
    short_names = {
        "NX-MIMOSA v3.1 (smooth)": "v3.1-S",
        "NX-MIMOSA v3.2c (smooth)": "v3.2c-S",
        "NX-MIMOSA v3.1 (fwd)": "v3.1-F",
        "NX-MIMOSA v3.2c (fwd)": "v3.2c-F",
        "FilterPy IMM": "FPy-IMM",
        "StoneSoup EKF+RTS": "SS-RTS",
        "EKF-CA (5×Q)": "EKF-CA",
    }

    hdr = f"{'Scenario':<42}"
    for a in algo_names:
        hdr += f" {short_names[a]:>10}"
    hdr += f"  {'WINNER':>14}"
    print(hdr)
    print("─" * 130)

    wins = {a: 0 for a in algo_names}
    grand = {a: [] for a in algo_names}

    for sc in scenarios:
        row = f"{sc.name[:40]:<42}"
        best_r, best_a = 1e9, ""
        for a in algo_names:
            r = np.nanmean(results[sc.name][a])
            grand[a].append(r)
            row += f" {r:>10.2f}"
            if r < best_r:
                best_r, best_a = r, a
        wins[best_a] += 1
        row += f"  {short_names[best_a]:>14}"
        print(row)

    print("─" * 130)
    row = f"{'GRAND AVERAGE':<42}"
    for a in algo_names:
        row += f" {np.nanmean(grand[a]):>10.2f}"
    print(row)

    row = f"{'WINS':<42}"
    for a in algo_names:
        row += f" {wins[a]:>10}"
    print(row)

    # ═══════════════════════════════════════════════════════════════════════
    # v3.1 vs v3.2c DELTA ANALYSIS
    # ═══════════════════════════════════════════════════════════════════════
    print("\n" + "=" * 100)
    print("v3.1 → v3.2c DELTA ANALYSIS (smoothed)")
    print("=" * 100)
    print(f"\n{'Scenario':<42} {'v3.1':>10} {'v3.2c':>10} {'Delta':>10} {'Change':>10}")
    print("─" * 84)

    for sc in scenarios:
        r31 = np.nanmean(results[sc.name]["NX-MIMOSA v3.1 (smooth)"])
        r32 = np.nanmean(results[sc.name]["NX-MIMOSA v3.2c (smooth)"])
        delta = r32 - r31
        pct = (r32 - r31) / r31 * 100
        marker = "✅ BETTER" if delta < -0.01 else ("⚠️  REGRESS" if delta > 0.01 else "≈ SAME")
        print(f"{sc.name[:40]:<42} {r31:>8.2f} m {r32:>8.2f} m {delta:>+8.2f} m {pct:>+8.1f}%  {marker}")

    g31 = np.nanmean(grand["NX-MIMOSA v3.1 (smooth)"])
    g32 = np.nanmean(grand["NX-MIMOSA v3.2c (smooth)"])
    print("─" * 84)
    print(f"{'GRAND AVERAGE':<42} {g31:>8.2f} m {g32:>8.2f} m {g32-g31:>+8.2f} m {(g32-g31)/g31*100:>+8.1f}%")

    # ═══════════════════════════════════════════════════════════════════════
    # v3.2c vs STONE SOUP (the real question)
    # ═══════════════════════════════════════════════════════════════════════
    print("\n" + "=" * 100)
    print("v3.2c (smooth) vs STONE SOUP EKF+RTS (the critical comparison)")
    print("=" * 100)
    print(f"\n{'Scenario':<42} {'v3.2c':>10} {'StoneSoup':>10} {'Δ':>10} {'Winner':>14}")
    print("─" * 88)

    v32c_wins, ss_wins = 0, 0
    for sc in scenarios:
        r32 = np.nanmean(results[sc.name]["NX-MIMOSA v3.2c (smooth)"])
        rss = np.nanmean(results[sc.name]["StoneSoup EKF+RTS"])
        delta = r32 - rss
        if r32 < rss:
            v32c_wins += 1
            w = "v3.2c ✅"
        else:
            ss_wins += 1
            w = "StoneSoup ✅"
        print(f"{sc.name[:40]:<42} {r32:>8.2f} m {rss:>8.2f} m {delta:>+8.2f} m {w:>14}")

    gss = np.nanmean(grand["StoneSoup EKF+RTS"])
    print("─" * 88)
    print(f"{'GRAND AVERAGE':<42} {g32:>8.2f} m {gss:>8.2f} m {g32-gss:>+8.2f} m")
    print(f"\n  Win count: v3.2c = {v32c_wins}/8, StoneSoup = {ss_wins}/8")
    print(f"  Grand avg improvement: {(gss-g32)/gss*100:+.1f}% ({'v3.2c better' if g32 < gss else 'StoneSoup better'})")

    # ═══════════════════════════════════════════════════════════════════════
    # CATEGORY ANALYSIS
    # ═══════════════════════════════════════════════════════════════════════
    print("\n" + "=" * 100)
    print("FAIR CATEGORY COMPARISONS")
    print("=" * 100)

    print("\n📡 CATEGORY 1: Forward-Only (Real-Time)")
    print("─" * 60)
    for a in ["EKF-CA (5×Q)", "FilterPy IMM", "NX-MIMOSA v3.1 (fwd)", "NX-MIMOSA v3.2c (fwd)"]:
        print(f"  {a:<32} {np.nanmean(grand[a]):>8.2f} m")

    print("\n📊 CATEGORY 2: Smoothed (Offline)")
    print("─" * 60)
    for a in ["StoneSoup EKF+RTS", "NX-MIMOSA v3.1 (smooth)", "NX-MIMOSA v3.2c (smooth)"]:
        print(f"  {a:<32} {np.nanmean(grand[a]):>8.2f} m")

    print("\n🎯 CATEGORY 3: IMM-to-IMM")
    print("─" * 60)
    for a in ["FilterPy IMM", "NX-MIMOSA v3.1 (fwd)", "NX-MIMOSA v3.2c (fwd)",
              "NX-MIMOSA v3.1 (smooth)", "NX-MIMOSA v3.2c (smooth)"]:
        print(f"  {a:<32} {np.nanmean(grand[a]):>8.2f} m")

    # ═══════════════════════════════════════════════════════════════════════
    # FAIRNESS DECLARATION
    # ═══════════════════════════════════════════════════════════════════════
    print("\n" + "=" * 100)
    print("FAIRNESS & REPRODUCIBILITY")
    print("=" * 100)
    print(f"""
✅ Identical measurements per run (same RNG seed)
✅ Identical initialization for all algorithms
✅ FilterPy IMM uses same 3-model structure (CV/CT±) with same TPM
✅ Stone Soup uses their production RTS smoother
✅ {n_runs} Monte Carlo runs, seed={seed}
✅ All 8 scenarios, all algorithms reported (no cherry-picking)
✅ Reproducible: pip install numpy scipy filterpy stonesoup

⚠️  DISCLOSED ADVANTAGES of v3.2c:
  - Adaptive omega matches CT model to actual turn rate
  - Adaptive Q (NIS-based) — also in v3.1
  - VS-TPM — also in v3.1
  - Per-model smoother uses future data (not causal)

⚠️  DISCLOSED LIMITATIONS:
  - Stone Soup = CV-only smoother, no IMM (unfair class comparison)
  - FilterPy IMM = forward-only, no smoother (different capability class)
  - v3.2c smoother not causal — for real-time, compare fwd variants
""")

    return results, grand


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs", type=int, default=100)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    run_benchmark(args.runs, args.seed)
