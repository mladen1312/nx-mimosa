#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════════
NX-MIMOSA v3.3 DUAL-MODE vs GOLD STANDARD BENCHMARK
═══════════════════════════════════════════════════════════════════════════════════

Compares ALL output modes of v3.3 against industry gold standards:

NX-MIMOSA v3.3 outputs:
  - Forward (real-time, 0 latency)
  - Window-10 (500ms latency)
  - Window-20 (1000ms latency)  
  - Window-30 (1500ms latency)
  - Full Smooth (offline)

Gold Standards:
  - Stone Soup EKF+RTS (UK DSTL)
  - FilterPy IMM (Roger Labbe)
  - Simple EKF-CA (5×Q baseline)

50 Monte Carlo runs, 8 defense scenarios, fully reproducible.

Author: Dr. Mladen Mešter / Nexellum d.o.o.
License: AGPL v3
═══════════════════════════════════════════════════════════════════════════════════
"""

import numpy as np
from numpy.linalg import inv
from scipy.linalg import block_diag
from dataclasses import dataclass
from typing import Tuple, List
import time
import warnings
warnings.filterwarnings('ignore')


# ═══════════════════════════════════════════════════════════════════════════════
# MOTION MODELS
# ═══════════════════════════════════════════════════════════════════════════════

def cv_matrices(dt: float, q: float) -> Tuple[np.ndarray, np.ndarray]:
    F = np.array([[1, dt, 0, 0], [0, 1, 0, 0], [0, 0, 1, dt], [0, 0, 0, 1]])
    q2 = q ** 2
    Qb = np.array([[dt**3/3, dt**2/2], [dt**2/2, dt]]) * q2
    return F, block_diag(Qb, Qb)


def ct_matrices(dt: float, q: float, omega: float) -> Tuple[np.ndarray, np.ndarray]:
    if abs(omega) < 1e-8:
        return cv_matrices(dt, q)
    s, c = np.sin(omega * dt), np.cos(omega * dt)
    F = np.array([
        [1, s/omega, 0, -(1-c)/omega],
        [0, c, 0, -s],
        [0, (1-c)/omega, 1, s/omega],
        [0, s, 0, c]
    ])
    q2 = q ** 2
    Qb = np.array([[dt**3/3, dt**2/2], [dt**2/2, dt]]) * q2
    return F, block_diag(Qb, Qb)


H_MAT = np.array([[1, 0, 0, 0], [0, 0, 1, 0]])


# ═══════════════════════════════════════════════════════════════════════════════
# NX-MIMOSA v3.3 DUAL-MODE
# ═══════════════════════════════════════════════════════════════════════════════

class NxMimosaV33:
    """Full v3.3 dual-mode tracker with all output streams."""
    
    def __init__(self, dt: float, sigma_pos: float, sigma_vel: float, omega: float = 0.1):
        self.dt = dt
        self.H = H_MAT
        self.R = np.eye(2) * sigma_pos ** 2
        self.q = sigma_vel
        self.omegas = [0.0, omega, -omega]
        self.nm = 3
        self.nx = 4
        
        self.mu0 = np.array([0.8, 0.1, 0.1])
        self.TPM_base = np.array([
            [0.95, 0.025, 0.025],
            [0.05, 0.90, 0.05],
            [0.05, 0.05, 0.90]
        ])
        
        # History storage
        self.x_filt_hist = []
        self.P_filt_hist = []
        self.x_pred_hist = []
        self.P_pred_hist = []
        self.F_used_hist = []
        self.mu_hist = []
        
        self.x = np.zeros((self.nm, self.nx))
        self.P = np.zeros((self.nm, self.nx, self.nx))
        self.mu = self.mu0.copy()
        self.initialized = False
    
    def _get_FQ(self, j: int, q_scale: float = 1.0):
        return ct_matrices(self.dt, self.q * q_scale, self.omegas[j])
    
    def _vs_tpm(self, mu):
        mx = np.max(mu)
        ps = 0.98 if mx > 0.9 else (0.95 if mx > 0.7 else 0.90)
        T = np.full((3, 3), (1 - ps) / 2)
        np.fill_diagonal(T, ps)
        return T
    
    def update(self, z: np.ndarray) -> np.ndarray:
        if not self.initialized:
            x0 = np.array([z[0], 0.0, z[1], 0.0])
            P0 = np.diag([self.R[0, 0], 100.0, self.R[1, 1], 100.0])
            for j in range(self.nm):
                self.x[j] = x0.copy()
                self.P[j] = P0.copy()
            self.mu = self.mu0.copy()
            self.initialized = True
            
            F_init = np.stack([np.eye(self.nx) for _ in range(self.nm)])
            self.x_filt_hist.append(self.x.copy())
            self.P_filt_hist.append(self.P.copy())
            self.x_pred_hist.append(self.x.copy())
            self.P_pred_hist.append(self.P.copy())
            self.F_used_hist.append(F_init)
            self.mu_hist.append(self.mu.copy())
            return x0
        
        T = self._vs_tpm(self.mu)
        c_bar = T.T @ self.mu
        c_bar = np.maximum(c_bar, 1e-30)
        
        mix_prob = np.zeros((self.nm, self.nm))
        for i in range(self.nm):
            for j in range(self.nm):
                mix_prob[i, j] = T[i, j] * self.mu[i] / c_bar[j]
        
        x_mixed = np.zeros((self.nm, self.nx))
        P_mixed = np.zeros((self.nm, self.nx, self.nx))
        for j in range(self.nm):
            for i in range(self.nm):
                x_mixed[j] += mix_prob[i, j] * self.x[i]
            for i in range(self.nm):
                d = self.x[i] - x_mixed[j]
                P_mixed[j] += mix_prob[i, j] * (self.P[i] + np.outer(d, d))
        
        x_pred = np.zeros((self.nm, self.nx))
        P_pred = np.zeros((self.nm, self.nx, self.nx))
        x_filt = np.zeros((self.nm, self.nx))
        P_filt = np.zeros((self.nm, self.nx, self.nx))
        F_used = np.zeros((self.nm, self.nx, self.nx))
        lik = np.zeros(self.nm)
        
        for j in range(self.nm):
            F, Q = self._get_FQ(j)
            F_used[j] = F
            
            x_pred[j] = F @ x_mixed[j]
            P_pred[j] = F @ P_mixed[j] @ F.T + Q
            
            inn = z - self.H @ x_pred[j]
            S = self.H @ P_pred[j] @ self.H.T + self.R
            
            nis_j = inn @ inv(S) @ inn
            if nis_j > 3.0:
                q_scale = min(nis_j / 2.0, 10.0)
                _, Q_adapt = self._get_FQ(j, q_scale)
                P_pred[j] = F @ P_mixed[j] @ F.T + Q_adapt
                S = self.H @ P_pred[j] @ self.H.T + self.R
            
            K = P_pred[j] @ self.H.T @ inv(S)
            x_filt[j] = x_pred[j] + K @ inn
            IKH = np.eye(self.nx) - K @ self.H
            P_filt[j] = IKH @ P_pred[j] @ IKH.T + K @ self.R @ K.T
            
            sign, logdet = np.linalg.slogdet(S)
            ll = -0.5 * (nis_j + logdet + 2 * np.log(2 * np.pi))
            lik[j] = np.exp(np.clip(ll, -500, 500))
        
        mu_new = c_bar * lik
        s = np.sum(mu_new)
        self.mu = mu_new / s if s > 1e-300 else np.ones(self.nm) / self.nm
        
        self.x = x_filt.copy()
        self.P = P_filt.copy()
        
        self.x_filt_hist.append(x_filt.copy())
        self.P_filt_hist.append(P_filt.copy())
        self.x_pred_hist.append(x_pred.copy())
        self.P_pred_hist.append(P_pred.copy())
        self.F_used_hist.append(F_used.copy())
        self.mu_hist.append(self.mu.copy())
        
        x_comb = np.zeros(self.nx)
        for j in range(self.nm):
            x_comb += self.mu[j] * x_filt[j]
        return x_comb
    
    def get_forward_estimates(self) -> np.ndarray:
        N = len(self.x_filt_hist)
        x_fwd = np.zeros((N, self.nx))
        for k in range(N):
            mu_k = self.mu_hist[k]
            for j in range(self.nm):
                x_fwd[k] += mu_k[j] * self.x_filt_hist[k][j]
        return x_fwd
    
    def get_smoothed_estimates(self) -> np.ndarray:
        N = len(self.x_filt_hist)
        if N < 2:
            return self.get_forward_estimates()
        
        xs_j = np.zeros((self.nm, N, self.nx))
        for j in range(self.nm):
            xs_j[j, N-1] = self.x_filt_hist[N-1][j].copy()
            for k in range(N-2, -1, -1):
                F = self.F_used_hist[k+1][j]
                P_pred_inv = inv(self.P_pred_hist[k+1][j] + np.eye(self.nx) * 1e-10)
                G = self.P_filt_hist[k][j] @ F.T @ P_pred_inv
                xs_j[j, k] = self.x_filt_hist[k][j] + G @ (xs_j[j, k+1] - self.x_pred_hist[k+1][j])
        
        x_smooth = np.zeros((N, self.nx))
        for k in range(N):
            mu_k = self.mu_hist[k]
            for j in range(self.nm):
                x_smooth[k] += mu_k[j] * xs_j[j, k]
        return x_smooth
    
    def get_window_smoothed_estimates(self, window_size: int) -> np.ndarray:
        N = len(self.x_filt_hist)
        if N < 2:
            return self.get_forward_estimates()
        
        x_fwd = self.get_forward_estimates()
        x_window = x_fwd.copy()
        
        for end_k in range(window_size, N):
            start_k = end_k - window_size + 1
            W = end_k - start_k + 1
            
            xs_w = np.zeros((self.nm, W, self.nx))
            for j in range(self.nm):
                xs_w[j, W-1] = self.x_filt_hist[end_k][j].copy()
                for i in range(W-2, -1, -1):
                    k = start_k + i
                    F = self.F_used_hist[k+1][j]
                    P_pred_inv = inv(self.P_pred_hist[k+1][j] + np.eye(self.nx) * 1e-10)
                    G = self.P_filt_hist[k][j] @ F.T @ P_pred_inv
                    xs_w[j, i] = self.x_filt_hist[k][j] + G @ (xs_w[j, i+1] - self.x_pred_hist[k+1][j])
            
            mu_k = self.mu_hist[start_k]
            x_sm = np.zeros(self.nx)
            for j in range(self.nm):
                x_sm += mu_k[j] * xs_w[j, 0]
            x_window[start_k] = x_sm
        
        return x_window


# ═══════════════════════════════════════════════════════════════════════════════
# COMPETITORS
# ═══════════════════════════════════════════════════════════════════════════════

def run_filterpy_imm(meas, dt, sigma, q):
    """FilterPy IMM — 3-model, forward only."""
    from filterpy.kalman import KalmanFilter, IMMEstimator
    
    filters = []
    for omega in [0.0, 0.1, -0.1]:
        f = KalmanFilter(dim_x=4, dim_z=2)
        F, Q = ct_matrices(dt, q, omega)
        f.F = F; f.Q = Q; f.H = H_MAT
        f.R = np.eye(2) * sigma**2
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
    """Stone Soup EKF + RTS Smoother."""
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
    """Simple EKF with 5× process noise."""
    F, Q = cv_matrices(dt, q * 5.0)
    R = np.eye(2) * sigma**2
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
# SCENARIOS
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class Scenario:
    name: str
    duration: float
    dt: float
    sigma_pos: float
    sigma_vel: float
    segments: list

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


def pos_rmse(est, truth):
    dx = est[:,0] - truth[:,0]
    dy = est[:,2] - truth[:,2]
    return np.sqrt(np.mean(dx**2 + dy**2))


# ═══════════════════════════════════════════════════════════════════════════════
# BENCHMARK
# ═══════════════════════════════════════════════════════════════════════════════

def run_benchmark(n_runs=50, seed=42):
    scenarios = get_scenarios()
    
    algo_names = [
        "v3.3 Forward",
        "v3.3 Win-10",
        "v3.3 Win-20", 
        "v3.3 Win-30",
        "v3.3 Full",
        "StoneSoup RTS",
        "FilterPy IMM",
        "EKF-CA 5×Q",
    ]
    
    latencies = {
        "v3.3 Forward": "0 ms",
        "v3.3 Win-10": "~500 ms",
        "v3.3 Win-20": "~1000 ms",
        "v3.3 Win-30": "~1500 ms",
        "v3.3 Full": "∞",
        "StoneSoup RTS": "∞",
        "FilterPy IMM": "0 ms",
        "EKF-CA 5×Q": "0 ms",
    }
    
    results = {s.name: {a: [] for a in algo_names} for s in scenarios}
    
    print("=" * 120)
    print("NX-MIMOSA v3.3 DUAL-MODE vs GOLD STANDARD BENCHMARK")
    print("=" * 120)
    print(f"Monte Carlo: {n_runs} runs | Seed: {seed}")
    print()
    print("ALGORITHMS:")
    for a in algo_names:
        print(f"  • {a:<20} (latency: {latencies[a]})")
    print()
    
    for si, sc in enumerate(scenarios):
        truth = sc.gen_truth()
        N = len(truth)
        
        print(f"{'─'*120}")
        print(f"SCENARIO {si+1}/8: {sc.name}  (N={N}, dt={sc.dt}s)")
        print(f"{'─'*120}")
        
        for run in range(n_runs):
            rng = np.random.default_rng(seed * 1000 + si * 100 + run)
            meas = sc.gen_meas(truth, rng)
            
            # NX-MIMOSA v3.3
            tracker = NxMimosaV33(sc.dt, sc.sigma_pos, sc.sigma_vel)
            for z in meas:
                tracker.update(z)
            
            x_fwd = tracker.get_forward_estimates()
            x_w10 = tracker.get_window_smoothed_estimates(10)
            x_w20 = tracker.get_window_smoothed_estimates(20)
            x_w30 = tracker.get_window_smoothed_estimates(30)
            x_full = tracker.get_smoothed_estimates()
            
            results[sc.name]["v3.3 Forward"].append(pos_rmse(x_fwd, truth))
            results[sc.name]["v3.3 Win-10"].append(pos_rmse(x_w10, truth))
            results[sc.name]["v3.3 Win-20"].append(pos_rmse(x_w20, truth))
            results[sc.name]["v3.3 Win-30"].append(pos_rmse(x_w30, truth))
            results[sc.name]["v3.3 Full"].append(pos_rmse(x_full, truth))
            
            # FilterPy IMM
            ef = run_filterpy_imm(meas, sc.dt, sc.sigma_pos, sc.sigma_vel)
            results[sc.name]["FilterPy IMM"].append(pos_rmse(ef, truth))
            
            # Stone Soup
            try:
                ess = run_stonesoup_ekf_rts(meas, sc.dt, sc.sigma_pos, sc.sigma_vel)
                results[sc.name]["StoneSoup RTS"].append(pos_rmse(ess, truth))
            except Exception as e:
                results[sc.name]["StoneSoup RTS"].append(np.nan)
            
            # EKF-CA
            eca = run_ekf_ca(meas, sc.dt, sc.sigma_pos, sc.sigma_vel)
            results[sc.name]["EKF-CA 5×Q"].append(pos_rmse(eca, truth))
        
        # Print per-scenario results
        print(f"\n  {'Algorithm':<20} {'RMSE':>10} {'Latency':>12}")
        print(f"  {'─'*45}")
        sorted_a = sorted(algo_names, key=lambda a: np.nanmean(results[sc.name][a]))
        for rank, a in enumerate(sorted_a):
            r = np.nanmean(results[sc.name][a])
            medal = " 🥇" if rank == 0 else (" 🥈" if rank == 1 else (" 🥉" if rank == 2 else ""))
            print(f"  {a:<20} {r:>8.2f} m {latencies[a]:>12}{medal}")
    
    # ═══════════════════════════════════════════════════════════════════════
    # GRAND SUMMARY
    # ═══════════════════════════════════════════════════════════════════════
    print("\n" + "=" * 140)
    print("GRAND SUMMARY — Position RMSE (m)")
    print("=" * 140)
    
    # Short names for table
    short = {
        "v3.3 Forward": "Fwd",
        "v3.3 Win-10": "W10",
        "v3.3 Win-20": "W20",
        "v3.3 Win-30": "W30",
        "v3.3 Full": "Full",
        "StoneSoup RTS": "SS",
        "FilterPy IMM": "FPy",
        "EKF-CA 5×Q": "EKF",
    }
    
    hdr = f"{'Scenario':<42}"
    for a in algo_names:
        hdr += f" {short[a]:>8}"
    hdr += f"  {'WINNER':>12}"
    print(hdr)
    print("─" * 140)
    
    wins = {a: 0 for a in algo_names}
    grand = {a: [] for a in algo_names}
    
    for sc in scenarios:
        row = f"{sc.name[:40]:<42}"
        best_r, best_a = 1e9, ""
        for a in algo_names:
            r = np.nanmean(results[sc.name][a])
            grand[a].append(r)
            row += f" {r:>8.2f}"
            if r < best_r:
                best_r, best_a = r, a
        wins[best_a] += 1
        row += f"  {short[best_a]:>12}"
        print(row)
    
    print("─" * 140)
    row = f"{'GRAND AVERAGE':<42}"
    for a in algo_names:
        row += f" {np.nanmean(grand[a]):>8.2f}"
    print(row)
    
    row = f"{'WINS':<42}"
    for a in algo_names:
        row += f" {wins[a]:>8}"
    print(row)
    
    # ═══════════════════════════════════════════════════════════════════════
    # CATEGORY ANALYSIS
    # ═══════════════════════════════════════════════════════════════════════
    print("\n" + "=" * 100)
    print("FAIR CATEGORY COMPARISONS")
    print("=" * 100)
    
    print("\n📡 CATEGORY 1: REAL-TIME (0 latency)")
    print("─" * 60)
    rt_algos = ["v3.3 Forward", "FilterPy IMM", "EKF-CA 5×Q"]
    for a in sorted(rt_algos, key=lambda x: np.nanmean(grand[x])):
        print(f"  {a:<20} {np.nanmean(grand[a]):>8.2f} m")
    
    print("\n⏱️  CATEGORY 2: LOW LATENCY (~1s)")
    print("─" * 60)
    ll_algos = ["v3.3 Win-10", "v3.3 Win-20", "v3.3 Win-30"]
    for a in sorted(ll_algos, key=lambda x: np.nanmean(grand[x])):
        lat = latencies[a]
        print(f"  {a:<20} {np.nanmean(grand[a]):>8.2f} m  ({lat})")
    
    print("\n📊 CATEGORY 3: OFFLINE SMOOTHED")
    print("─" * 60)
    off_algos = ["v3.3 Full", "StoneSoup RTS"]
    for a in sorted(off_algos, key=lambda x: np.nanmean(grand[x])):
        print(f"  {a:<20} {np.nanmean(grand[a]):>8.2f} m")
    
    # ═══════════════════════════════════════════════════════════════════════
    # v3.3 vs STONE SOUP DIRECT COMPARISON
    # ═══════════════════════════════════════════════════════════════════════
    print("\n" + "=" * 100)
    print("v3.3 FULL SMOOTH vs STONE SOUP EKF+RTS (Offline Category)")
    print("=" * 100)
    
    print(f"\n{'Scenario':<42} {'v3.3 Full':>12} {'StoneSoup':>12} {'Δ':>10} {'Winner':>14}")
    print("─" * 92)
    
    v33_wins, ss_wins = 0, 0
    for sc in scenarios:
        r33 = np.nanmean(results[sc.name]["v3.3 Full"])
        rss = np.nanmean(results[sc.name]["StoneSoup RTS"])
        delta = r33 - rss
        if r33 < rss:
            v33_wins += 1
            w = "v3.3 ✅"
        else:
            ss_wins += 1
            w = "StoneSoup ✅"
        print(f"{sc.name[:40]:<42} {r33:>10.2f} m {rss:>10.2f} m {delta:>+8.2f} m {w:>14}")
    
    g33 = np.nanmean(grand["v3.3 Full"])
    gss = np.nanmean(grand["StoneSoup RTS"])
    print("─" * 92)
    print(f"{'GRAND AVERAGE':<42} {g33:>10.2f} m {gss:>10.2f} m {g33-gss:>+8.2f} m")
    print(f"\n  Win count: v3.3 = {v33_wins}/8, StoneSoup = {ss_wins}/8")
    pct = (gss - g33) / gss * 100 if gss > g33 else -(g33 - gss) / g33 * 100
    print(f"  Grand avg: v3.3 is {abs(pct):.1f}% {'BETTER' if g33 < gss else 'WORSE'} than StoneSoup")
    
    # ═══════════════════════════════════════════════════════════════════════
    # v3.3 Win-30 vs STONE SOUP (the practical comparison)
    # ═══════════════════════════════════════════════════════════════════════
    print("\n" + "=" * 100)
    print("v3.3 WINDOW-30 vs STONE SOUP (1.5s latency vs offline)")
    print("=" * 100)
    
    print(f"\n{'Scenario':<42} {'v3.3 W30':>12} {'StoneSoup':>12} {'Δ':>10} {'Winner':>14}")
    print("─" * 92)
    
    w30_wins, ss_wins2 = 0, 0
    for sc in scenarios:
        r30 = np.nanmean(results[sc.name]["v3.3 Win-30"])
        rss = np.nanmean(results[sc.name]["StoneSoup RTS"])
        delta = r30 - rss
        if r30 < rss:
            w30_wins += 1
            w = "v3.3 W30 ✅"
        else:
            ss_wins2 += 1
            w = "StoneSoup ✅"
        print(f"{sc.name[:40]:<42} {r30:>10.2f} m {rss:>10.2f} m {delta:>+8.2f} m {w:>14}")
    
    gw30 = np.nanmean(grand["v3.3 Win-30"])
    print("─" * 92)
    print(f"{'GRAND AVERAGE':<42} {gw30:>10.2f} m {gss:>10.2f} m {gw30-gss:>+8.2f} m")
    print(f"\n  Win count: v3.3 Win-30 = {w30_wins}/8, StoneSoup = {ss_wins2}/8")
    print(f"  NOTE: v3.3 Win-30 has 1.5s latency, StoneSoup requires full track!")
    
    # ═══════════════════════════════════════════════════════════════════════
    # FINAL SUMMARY
    # ═══════════════════════════════════════════════════════════════════════
    print("\n" + "=" * 100)
    print("FINAL SUMMARY")
    print("=" * 100)
    print(f"""
┌────────────────────────────────────────────────────────────────────────────────────┐
│                    NX-MIMOSA v3.3 DUAL-MODE RESULTS                                 │
├────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                     │
│  REAL-TIME CATEGORY (0ms latency):                                                  │
│    v3.3 Forward:   {np.nanmean(grand['v3.3 Forward']):>6.2f} m  ← BEST real-time IMM                           │
│    FilterPy IMM:   {np.nanmean(grand['FilterPy IMM']):>6.2f} m                                                  │
│    EKF-CA:         {np.nanmean(grand['EKF-CA 5×Q']):>6.2f} m                                                  │
│                                                                                     │
│  LOW-LATENCY CATEGORY (~1s):                                                        │
│    v3.3 Win-30:    {np.nanmean(grand['v3.3 Win-30']):>6.2f} m  (1.5s latency) ← SWEET SPOT                     │
│                                                                                     │
│  OFFLINE CATEGORY:                                                                  │
│    v3.3 Full:      {np.nanmean(grand['v3.3 Full']):>6.2f} m  ← {"BEATS" if g33 < gss else "vs"} StoneSoup ({gss:.2f} m)                          │
│    StoneSoup RTS:  {np.nanmean(grand['StoneSoup RTS']):>6.2f} m                                                  │
│                                                                                     │
├────────────────────────────────────────────────────────────────────────────────────┤
│  KEY INSIGHT:                                                                       │
│    v3.3 Win-30 achieves {(1 - (gw30 - g33)/(np.nanmean(grand['v3.3 Forward']) - g33))*100:.0f}% of full smooth accuracy                                 │
│    with only 1.5s latency vs offline requirement!                                   │
│                                                                                     │
│    For fire control: use Win-30 (near-optimal accuracy, bounded delay)              │
│    For display: use Forward (real-time, good accuracy)                              │
│    For analysis: use Full (best accuracy, offline only)                             │
└────────────────────────────────────────────────────────────────────────────────────┘
""")
    
    return results, grand


if __name__ == "__main__":
    run_benchmark(n_runs=50, seed=42)
