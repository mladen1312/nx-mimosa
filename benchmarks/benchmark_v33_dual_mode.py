#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════════
NX-MIMOSA v3.3 DUAL-MODE — FULL GOLD STANDARD BENCHMARK
═══════════════════════════════════════════════════════════════════════════════════

Compares all three v3.3 output streams against competitors:

  NX-MIMOSA v3.3 outputs:
    - Forward (real-time, 0 latency)
    - Window-30 (refined, 1.5s latency)  
    - Full smooth (offline)

  Competitors:
    - Stone Soup EKF+RTS (UK DSTL gold standard)
    - FilterPy IMM (Roger Labbe, 3-model)
    - EKF-CA (5×Q baseline)

50 Monte Carlo runs, 8 defense scenarios, seed=42.

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

def ct_matrices(dt: float, q: float, omega: float) -> Tuple[np.ndarray, np.ndarray]:
    if abs(omega) < 1e-8:
        F = np.array([[1, dt, 0, 0], [0, 1, 0, 0], [0, 0, 1, dt], [0, 0, 0, 1]])
    else:
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
    def __init__(self, dt, sigma_pos, sigma_vel, omega=0.1):
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
    
    def _get_FQ(self, j, q_scale=1.0):
        return ct_matrices(self.dt, self.q * q_scale, self.omegas[j])
    
    def _vs_tpm(self, mu):
        mx = np.max(mu)
        ps = 0.98 if mx > 0.9 else (0.95 if mx > 0.7 else 0.90)
        T = np.full((3, 3), (1 - ps) / 2)
        np.fill_diagonal(T, ps)
        return T
    
    def update(self, z):
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
    
    def get_forward_estimates(self):
        N = len(self.x_filt_hist)
        x_fwd = np.zeros((N, self.nx))
        for k in range(N):
            mu_k = self.mu_hist[k]
            for j in range(self.nm):
                x_fwd[k] += mu_k[j] * self.x_filt_hist[k][j]
        return x_fwd
    
    def get_smoothed_estimates(self):
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
    
    def get_window_smoothed_estimates(self, window_size=30):
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
    from filterpy.kalman import KalmanFilter, IMMEstimator
    filters = []
    for omega in [0.0, 0.1, -0.1]:
        f = KalmanFilter(dim_x=4, dim_z=2)
        F, Q = ct_matrices(dt, q, omega)
        f.F = F; f.Q = Q; f.H = H_MAT; f.R = np.eye(2) * sigma**2
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
    from stonesoup.types.state import GaussianState
    from stonesoup.types.detection import Detection
    from stonesoup.types.track import Track
    from stonesoup.types.array import StateVector, CovarianceMatrix
    from stonesoup.types.hypothesis import SingleHypothesis
    from stonesoup.models.transition.linear import CombinedLinearGaussianTransitionModel, ConstantVelocity
    from stonesoup.models.measurement.linear import LinearGaussian
    from stonesoup.predictor.kalman import KalmanPredictor
    from stonesoup.updater.kalman import KalmanUpdater
    from stonesoup.smoother.kalman import KalmanSmoother
    from datetime import datetime, timedelta

    tm = CombinedLinearGaussianTransitionModel([ConstantVelocity(q**2), ConstantVelocity(q**2)])
    mm = LinearGaussian(ndim_state=4, mapping=(0,2), noise_covar=CovarianceMatrix(np.eye(2)*sigma**2))
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
        det = Detection(state_vector=StateVector([[meas[k,0]],[meas[k,1]]]), timestamp=tk, measurement_model=mm)
        prediction = pred.predict(track[-1], timestamp=tk)
        hyp = SingleHypothesis(prediction, det)
        track.append(upd.update(hyp))

    smoothed = sm.smooth(track)
    states = np.zeros((N, 4))
    for k, st in enumerate(smoothed):
        states[k] = st.state_vector.flatten()
    return states


def run_ekf_ca(meas, dt, sigma, q):
    F, Q = ct_matrices(dt, q * 5.0, 0.0)
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
# MAIN BENCHMARK
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    n_runs = 50
    seed = 42
    window_size = 30
    
    scenarios = get_scenarios()
    
    algo_names = [
        "v3.3 Forward",
        "v3.3 Window-30", 
        "v3.3 Full Smooth",
        "FilterPy IMM",
        "StoneSoup RTS",
        "EKF-CA (5×Q)",
    ]
    
    results = {s.name: {a: [] for a in algo_names} for s in scenarios}
    
    print("=" * 120)
    print("NX-MIMOSA v3.3 DUAL-MODE — GOLD STANDARD BENCHMARK")
    print(f"Monte Carlo: {n_runs} runs | Seed: {seed} | Window: {window_size} steps")
    print("=" * 120)
    
    for si, sc in enumerate(scenarios):
        truth = sc.gen_truth()
        N = len(truth)
        latency_ms = window_size * sc.dt * 1000
        
        print(f"\n{'─'*120}")
        print(f"SCENARIO {si+1}/8: {sc.name}  (N={N}, dt={sc.dt}s, window latency={latency_ms:.0f}ms)")
        print(f"{'─'*120}")
        
        for run in range(n_runs):
            rng = np.random.default_rng(seed * 1000 + si * 100 + run)
            meas = sc.gen_meas(truth, rng)
            
            # v3.3 all modes
            tracker = NxMimosaV33(sc.dt, sc.sigma_pos, sc.sigma_vel)
            for z in meas:
                tracker.update(z)
            
            x_fwd = tracker.get_forward_estimates()
            x_win = tracker.get_window_smoothed_estimates(window_size)
            x_full = tracker.get_smoothed_estimates()
            
            results[sc.name]["v3.3 Forward"].append(pos_rmse(x_fwd, truth))
            results[sc.name]["v3.3 Window-30"].append(pos_rmse(x_win, truth))
            results[sc.name]["v3.3 Full Smooth"].append(pos_rmse(x_full, truth))
            
            # FilterPy
            ef = run_filterpy_imm(meas, sc.dt, sc.sigma_pos, sc.sigma_vel)
            results[sc.name]["FilterPy IMM"].append(pos_rmse(ef, truth))
            
            # Stone Soup
            try:
                ess = run_stonesoup_ekf_rts(meas, sc.dt, sc.sigma_pos, sc.sigma_vel)
                results[sc.name]["StoneSoup RTS"].append(pos_rmse(ess, truth))
            except:
                results[sc.name]["StoneSoup RTS"].append(np.nan)
            
            # EKF-CA
            eca = run_ekf_ca(meas, sc.dt, sc.sigma_pos, sc.sigma_vel)
            results[sc.name]["EKF-CA (5×Q)"].append(pos_rmse(eca, truth))
        
        # Per-scenario summary
        print(f"\n  {'Algorithm':<25} {'RMSE':>10} {'vs Forward':>12} {'vs StoneSoup':>14}")
        print(f"  {'─'*65}")
        
        fwd_rmse = np.nanmean(results[sc.name]["v3.3 Forward"])
        ss_rmse = np.nanmean(results[sc.name]["StoneSoup RTS"])
        
        sorted_algos = sorted(algo_names, key=lambda a: np.nanmean(results[sc.name][a]))
        for rank, a in enumerate(sorted_algos):
            r = np.nanmean(results[sc.name][a])
            vs_fwd = (r - fwd_rmse) / fwd_rmse * 100
            vs_ss = (r - ss_rmse) / ss_rmse * 100
            medal = " 🥇" if rank == 0 else (" 🥈" if rank == 1 else "")
            print(f"  {a:<25} {r:>8.2f} m  {vs_fwd:>+10.1f}%  {vs_ss:>+12.1f}%{medal}")
    
    # ═══════════════════════════════════════════════════════════════════════
    # GRAND SUMMARY
    # ═══════════════════════════════════════════════════════════════════════
    print("\n" + "=" * 140)
    print("GRAND SUMMARY — Position RMSE (m)")
    print("=" * 140)
    
    short = {
        "v3.3 Forward": "v3.3-FWD",
        "v3.3 Window-30": "v3.3-W30",
        "v3.3 Full Smooth": "v3.3-FULL",
        "FilterPy IMM": "FPy-IMM",
        "StoneSoup RTS": "SS-RTS",
        "EKF-CA (5×Q)": "EKF-CA",
    }
    
    hdr = f"{'Scenario':<42}"
    for a in algo_names:
        hdr += f" {short[a]:>10}"
    hdr += f"  {'BEST':>10}"
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
            row += f" {r:>10.2f}"
            if r < best_r:
                best_r, best_a = r, a
        wins[best_a] += 1
        row += f"  {short[best_a]:>10}"
        print(row)
    
    print("─" * 140)
    row = f"{'GRAND AVERAGE':<42}"
    for a in algo_names:
        row += f" {np.nanmean(grand[a]):>10.2f}"
    print(row)
    
    row = f"{'WINS':<42}"
    for a in algo_names:
        row += f" {wins[a]:>10}"
    print(row)
    
    # ═══════════════════════════════════════════════════════════════════════
    # DUAL-MODE ANALYSIS
    # ═══════════════════════════════════════════════════════════════════════
    print("\n" + "=" * 100)
    print("v3.3 DUAL-MODE ANALYSIS")
    print("=" * 100)
    
    print(f"\n{'Scenario':<42} {'Forward':>10} {'Window-30':>12} {'Full':>10} {'W30 % of Full':>15}")
    print("─" * 92)
    
    for sc in scenarios:
        fwd = np.nanmean(results[sc.name]["v3.3 Forward"])
        win = np.nanmean(results[sc.name]["v3.3 Window-30"])
        full = np.nanmean(results[sc.name]["v3.3 Full Smooth"])
        
        # What % of improvement does Window-30 capture?
        if fwd != full:
            pct = (fwd - win) / (fwd - full) * 100
        else:
            pct = 100.0
        pct = max(0, min(100, pct))
        
        print(f"{sc.name[:40]:<42} {fwd:>8.2f} m  {win:>10.2f} m  {full:>8.2f} m  {pct:>12.1f}%")
    
    # Grand average
    fwd_avg = np.nanmean(grand["v3.3 Forward"])
    win_avg = np.nanmean(grand["v3.3 Window-30"])
    full_avg = np.nanmean(grand["v3.3 Full Smooth"])
    pct_avg = (fwd_avg - win_avg) / (fwd_avg - full_avg) * 100 if fwd_avg != full_avg else 100
    
    print("─" * 92)
    print(f"{'GRAND AVERAGE':<42} {fwd_avg:>8.2f} m  {win_avg:>10.2f} m  {full_avg:>8.2f} m  {pct_avg:>12.1f}%")
    
    # ═══════════════════════════════════════════════════════════════════════
    # KEY FINDINGS
    # ═══════════════════════════════════════════════════════════════════════
    print("\n" + "=" * 100)
    print("KEY FINDINGS")
    print("=" * 100)
    
    ss_avg = np.nanmean(grand["StoneSoup RTS"])
    fpy_avg = np.nanmean(grand["FilterPy IMM"])
    
    print(f"""
┌─────────────────────────────────────────────────────────────────────────────────┐
│  NX-MIMOSA v3.3 DUAL-MODE PERFORMANCE                                            │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                  │
│  v3.3 Forward (real-time):     {fwd_avg:>6.2f} m   Latency: 0 ms                         │
│  v3.3 Window-30 (refined):     {win_avg:>6.2f} m   Latency: 30×dt (~1.5s)                │
│  v3.3 Full Smooth (offline):   {full_avg:>6.2f} m   Latency: full track                  │
│                                                                                  │
│  Window-30 captures {pct_avg:.1f}% of full smooth improvement                           │
│                                                                                  │
├─────────────────────────────────────────────────────────────────────────────────┤
│  VS COMPETITORS (using v3.3 Full Smooth for fair smoother comparison):           │
│                                                                                  │
│  vs Stone Soup EKF+RTS:  {(ss_avg - full_avg) / ss_avg * 100:>+6.1f}% ({full_avg:.2f}m vs {ss_avg:.2f}m)                       │
│  vs FilterPy IMM:        {(fpy_avg - fwd_avg) / fpy_avg * 100:>+6.1f}% ({fwd_avg:.2f}m vs {fpy_avg:.2f}m, forward-only)             │
│                                                                                  │
│  Win distribution: v3.3 Full = {wins["v3.3 Full Smooth"]}/8, v3.3 W30 = {wins["v3.3 Window-30"]}/8, SS = {wins["StoneSoup RTS"]}/8        │
│                                                                                  │
└─────────────────────────────────────────────────────────────────────────────────┘
""")
    
    print("\n✅ OPERATIONAL RECOMMENDATION:")
    print(f"   Use Window-30 for fire control: {win_avg:.2f}m RMSE at 1.5s latency")
    print(f"   Achieves {pct_avg:.0f}% of offline accuracy with bounded delay")


if __name__ == "__main__":
    main()
