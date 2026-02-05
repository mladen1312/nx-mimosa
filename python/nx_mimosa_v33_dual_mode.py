#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════════
NX-MIMOSA v3.3 — DUAL-MODE TRACKING (Fixed Implementation)
═══════════════════════════════════════════════════════════════════════════════════

Dual output streams for different operational needs:

  x_realtime: Zero latency, forward-only (~38m RMSE)
  x_refined:  N×dt latency, sliding window smooth (~25m RMSE)  
  x_offline:  Full track latency, complete RTS (~21m RMSE)

Author: Dr. Mladen Mešter / Nexellum d.o.o.
License: AGPL v3
═══════════════════════════════════════════════════════════════════════════════════
"""

import numpy as np
from numpy.linalg import inv
from scipy.linalg import block_diag
from typing import Tuple, List, Optional
from enum import Enum, auto


def ct_matrices(dt: float, q: float, omega: float) -> Tuple[np.ndarray, np.ndarray]:
    """Coordinated Turn model F and Q matrices."""
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


class NxMimosaV33Dual:
    """
    Dual-mode IMM tracker: real-time + delayed refined output.
    
    Stores full filter history for flexible smoothing.
    """
    
    def __init__(self, dt: float, sigma_pos: float, sigma_vel: float, 
                 omega: float = 0.1, window_size: int = 20):
        self.dt = dt
        self.H = H_MAT
        self.R = np.eye(2) * sigma_pos ** 2
        self.q = sigma_vel
        self.omegas = [0.0, omega, -omega]
        self.nm = 3
        self.nx = 4
        self.window_size = window_size
        
        # IMM parameters
        self.mu0 = np.array([0.8, 0.1, 0.1])
        self.TPM_base = np.array([
            [0.95, 0.025, 0.025],
            [0.05, 0.90, 0.05],
            [0.05, 0.05, 0.90]
        ])
        
        # Storage for full track (needed for proper smoothing)
        self.x_filt_hist = []   # List of (nm, nx) arrays
        self.P_filt_hist = []   # List of (nm, nx, nx) arrays
        self.x_pred_hist = []
        self.P_pred_hist = []
        self.F_used_hist = []   # List of (nm, nx, nx) arrays
        self.mu_hist = []       # List of (nm,) arrays
        
        # Current state
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
    
    def reset(self):
        """Clear all history."""
        self.x_filt_hist.clear()
        self.P_filt_hist.clear()
        self.x_pred_hist.clear()
        self.P_pred_hist.clear()
        self.F_used_hist.clear()
        self.mu_hist.clear()
        self.initialized = False
    
    def update(self, z: np.ndarray) -> np.ndarray:
        """
        Process one measurement, return real-time (forward) estimate.
        Stores state for later smoothing.
        """
        if not self.initialized:
            x0 = np.array([z[0], 0.0, z[1], 0.0])
            P0 = np.diag([self.R[0, 0], 100.0, self.R[1, 1], 100.0])
            for j in range(self.nm):
                self.x[j] = x0.copy()
                self.P[j] = P0.copy()
            self.mu = self.mu0.copy()
            self.initialized = True
            
            # Store initial
            F_init = np.stack([np.eye(self.nx) for _ in range(self.nm)])
            self.x_filt_hist.append(self.x.copy())
            self.P_filt_hist.append(self.P.copy())
            self.x_pred_hist.append(self.x.copy())
            self.P_pred_hist.append(self.P.copy())
            self.F_used_hist.append(F_init)
            self.mu_hist.append(self.mu.copy())
            
            return x0
        
        # VS-TPM
        T = self._vs_tpm(self.mu)
        c_bar = T.T @ self.mu
        c_bar = np.maximum(c_bar, 1e-30)
        
        # Mixing probabilities
        mix_prob = np.zeros((self.nm, self.nm))
        for i in range(self.nm):
            for j in range(self.nm):
                mix_prob[i, j] = T[i, j] * self.mu[i] / c_bar[j]
        
        # Mixed states
        x_mixed = np.zeros((self.nm, self.nx))
        P_mixed = np.zeros((self.nm, self.nx, self.nx))
        for j in range(self.nm):
            for i in range(self.nm):
                x_mixed[j] += mix_prob[i, j] * self.x[i]
            for i in range(self.nm):
                d = self.x[i] - x_mixed[j]
                P_mixed[j] += mix_prob[i, j] * (self.P[i] + np.outer(d, d))
        
        # Per-model predict/update
        x_pred = np.zeros((self.nm, self.nx))
        P_pred = np.zeros((self.nm, self.nx, self.nx))
        x_filt = np.zeros((self.nm, self.nx))
        P_filt = np.zeros((self.nm, self.nx, self.nx))
        F_used = np.zeros((self.nm, self.nx, self.nx))
        lik = np.zeros(self.nm)
        
        for j in range(self.nm):
            F, Q = self._get_FQ(j)
            F_used[j] = F
            
            # Predict
            x_pred[j] = F @ x_mixed[j]
            P_pred[j] = F @ P_mixed[j] @ F.T + Q
            
            # Innovation
            inn = z - self.H @ x_pred[j]
            S = self.H @ P_pred[j] @ self.H.T + self.R
            
            # NIS for adaptive Q
            nis_j = inn @ inv(S) @ inn
            if nis_j > 3.0:
                q_scale = min(nis_j / 2.0, 10.0)
                _, Q_adapt = self._get_FQ(j, q_scale)
                P_pred[j] = F @ P_mixed[j] @ F.T + Q_adapt
                S = self.H @ P_pred[j] @ self.H.T + self.R
            
            # Update (Joseph form)
            K = P_pred[j] @ self.H.T @ inv(S)
            x_filt[j] = x_pred[j] + K @ inn
            IKH = np.eye(self.nx) - K @ self.H
            P_filt[j] = IKH @ P_pred[j] @ IKH.T + K @ self.R @ K.T
            
            # Likelihood
            sign, logdet = np.linalg.slogdet(S)
            ll = -0.5 * (nis_j + logdet + 2 * np.log(2 * np.pi))
            lik[j] = np.exp(np.clip(ll, -500, 500))
        
        # Update mode probabilities
        mu_new = c_bar * lik
        s = np.sum(mu_new)
        self.mu = mu_new / s if s > 1e-300 else np.ones(self.nm) / self.nm
        
        # Store
        self.x = x_filt.copy()
        self.P = P_filt.copy()
        
        self.x_filt_hist.append(x_filt.copy())
        self.P_filt_hist.append(P_filt.copy())
        self.x_pred_hist.append(x_pred.copy())
        self.P_pred_hist.append(P_pred.copy())
        self.F_used_hist.append(F_used.copy())
        self.mu_hist.append(self.mu.copy())
        
        # Combined forward estimate
        x_comb = np.zeros(self.nx)
        for j in range(self.nm):
            x_comb += self.mu[j] * x_filt[j]
        
        return x_comb
    
    def get_forward_estimates(self) -> np.ndarray:
        """Return all forward (real-time) estimates."""
        N = len(self.x_filt_hist)
        x_fwd = np.zeros((N, self.nx))
        for k in range(N):
            mu_k = self.mu_hist[k]
            for j in range(self.nm):
                x_fwd[k] += mu_k[j] * self.x_filt_hist[k][j]
        return x_fwd
    
    def get_smoothed_estimates(self) -> np.ndarray:
        """Full offline RTS smooth over entire track."""
        N = len(self.x_filt_hist)
        if N < 2:
            return self.get_forward_estimates()
        
        # Per-model RTS backward pass
        xs_j = np.zeros((self.nm, N, self.nx))
        
        for j in range(self.nm):
            xs_j[j, N-1] = self.x_filt_hist[N-1][j].copy()
            
            for k in range(N-2, -1, -1):
                F = self.F_used_hist[k+1][j]
                P_pred_inv = inv(self.P_pred_hist[k+1][j] + np.eye(self.nx) * 1e-10)
                G = self.P_filt_hist[k][j] @ F.T @ P_pred_inv
                
                xs_j[j, k] = self.x_filt_hist[k][j] + G @ (xs_j[j, k+1] - self.x_pred_hist[k+1][j])
        
        # Combine with mode probabilities
        x_smooth = np.zeros((N, self.nx))
        for k in range(N):
            mu_k = self.mu_hist[k]
            for j in range(self.nm):
                x_smooth[k] += mu_k[j] * xs_j[j, k]
        
        return x_smooth
    
    def get_window_smoothed_estimates(self, window_size: int = None) -> np.ndarray:
        """
        Sliding window smooth: at step k, smooth over [k-W+1, k].
        Output for step (k-W+1) available at time k.
        
        Returns array where x_refined[k] is the smoothed estimate for step k,
        computed using data up to step (k + window_size - 1).
        """
        if window_size is None:
            window_size = self.window_size
        
        N = len(self.x_filt_hist)
        if N < 2:
            return self.get_forward_estimates()
        
        x_fwd = self.get_forward_estimates()
        x_window = x_fwd.copy()  # Start with forward, replace with smoothed
        
        # For each window position
        for end_k in range(window_size, N):
            start_k = end_k - window_size + 1
            W = end_k - start_k + 1  # window length
            
            # Per-model RTS over window
            xs_w = np.zeros((self.nm, W, self.nx))
            
            for j in range(self.nm):
                # Initialize at end of window
                xs_w[j, W-1] = self.x_filt_hist[end_k][j].copy()
                
                # Backward pass
                for i in range(W-2, -1, -1):
                    k = start_k + i
                    F = self.F_used_hist[k+1][j]
                    P_pred_inv = inv(self.P_pred_hist[k+1][j] + np.eye(self.nx) * 1e-10)
                    G = self.P_filt_hist[k][j] @ F.T @ P_pred_inv
                    
                    xs_w[j, i] = self.x_filt_hist[k][j] + G @ (xs_w[j, i+1] - self.x_pred_hist[k+1][j])
            
            # Combine at start of window (fully smoothed point)
            mu_k = self.mu_hist[start_k]
            x_sm = np.zeros(self.nx)
            for j in range(self.nm):
                x_sm += mu_k[j] * xs_w[j, 0]
            
            x_window[start_k] = x_sm
        
        return x_window


def run_dual_benchmark():
    """Benchmark dual-mode tracker on defense scenarios."""
    from dataclasses import dataclass
    
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
            return np.column_stack([truth[:, 0], truth[:, 2]]) + noise
    
    g = 9.81
    scenarios = [
        Scenario("Missile Terminal (Mach 4, 9g)", 20, 0.05, 15, 5,
                 [(5,0,0), (5,0,9*g), (5,-5*g,-9*g), (5,5*g,4*g)]),
        Scenario("Dogfight BFM (250m/s, 8g)", 20, 0.05, 8, 10,
                 [(5,0,8*g), (5,-4*g,-8*g), (5,8*g,0), (5,-8*g,4*g)]),
        Scenario("Cruise Missile (250m/s, 3g)", 30, 0.1, 20, 2,
                 [(10,0,0), (5,0,3*g), (5,0,-3*g), (10,0,0)]),
        Scenario("UAV Swarm (50m/s, 2g)", 30, 0.1, 5, 3,
                 [(7.5,2*g,0), (7.5,-2*g,2*g), (7.5,0,-2*g), (7.5,2*g,2*g)]),
    ]
    
    def pos_rmse(est, truth):
        dx = est[:, 0] - truth[:, 0]
        dy = est[:, 2] - truth[:, 2]
        return np.sqrt(np.mean(dx**2 + dy**2))
    
    print("=" * 100)
    print("NX-MIMOSA v3.3 — DUAL-MODE BENCHMARK")
    print("=" * 100)
    
    # Test different window sizes
    print("\n📈 WINDOW SIZE OPTIMIZATION")
    print("-" * 90)
    
    # Use Dogfight scenario for optimization
    sc = scenarios[1]
    truth = sc.gen_truth()
    rng = np.random.default_rng(42)
    meas = sc.gen_meas(truth, rng)
    
    tracker = NxMimosaV33Dual(sc.dt, sc.sigma_pos, sc.sigma_vel)
    for z in meas:
        tracker.update(z)
    
    x_fwd = tracker.get_forward_estimates()
    x_full = tracker.get_smoothed_estimates()
    
    rmse_fwd = pos_rmse(x_fwd, truth)
    rmse_full = pos_rmse(x_full, truth)
    
    print(f"Baseline — {sc.name}")
    print(f"  Forward RMSE:     {rmse_fwd:.2f} m  (latency: 0 ms)")
    print(f"  Full smooth RMSE: {rmse_full:.2f} m  (latency: ∞)")
    print(f"  Improvement:      {(rmse_fwd - rmse_full) / rmse_fwd * 100:.1f}%")
    print()
    
    print(f"{'Window':<10} {'Latency':<12} {'RMSE':<12} {'% of Full Smooth':<20}")
    print("-" * 60)
    
    for window in [5, 10, 20, 30, 50, 75, 100]:
        x_win = tracker.get_window_smoothed_estimates(window)
        rmse_win = pos_rmse(x_win, truth)
        latency = window * sc.dt * 1000
        
        # Percentage toward full smooth (100% = equals full smooth)
        pct = (rmse_fwd - rmse_win) / (rmse_fwd - rmse_full) * 100 if rmse_fwd != rmse_full else 100
        pct = max(0, min(100, pct))
        
        print(f"{window:<10} {latency:>8.0f} ms   {rmse_win:>8.2f} m   {pct:>10.1f}%")
    
    # Full scenario benchmark
    print("\n" + "=" * 100)
    print("FULL SCENARIO BENCHMARK (10 MC runs)")
    print("=" * 100)
    
    n_runs = 10
    window_size = 30  # Use 30-step window as compromise
    
    print(f"\n{'Scenario':<40} {'Forward':<12} {'Window-30':<12} {'Full Smooth':<12}")
    print("-" * 80)
    
    for sc in scenarios:
        truth = sc.gen_truth()
        
        rmse_fwd_avg, rmse_win_avg, rmse_full_avg = 0, 0, 0
        
        for run in range(n_runs):
            rng = np.random.default_rng(42 * 1000 + run)
            meas = sc.gen_meas(truth, rng)
            
            tracker = NxMimosaV33Dual(sc.dt, sc.sigma_pos, sc.sigma_vel, window_size=window_size)
            for z in meas:
                tracker.update(z)
            
            x_fwd = tracker.get_forward_estimates()
            x_win = tracker.get_window_smoothed_estimates(window_size)
            x_full = tracker.get_smoothed_estimates()
            
            rmse_fwd_avg += pos_rmse(x_fwd, truth)
            rmse_win_avg += pos_rmse(x_win, truth)
            rmse_full_avg += pos_rmse(x_full, truth)
        
        rmse_fwd_avg /= n_runs
        rmse_win_avg /= n_runs
        rmse_full_avg /= n_runs
        
        print(f"{sc.name:<40} {rmse_fwd_avg:>8.2f} m   {rmse_win_avg:>8.2f} m   {rmse_full_avg:>8.2f} m")
    
    # Summary
    print("\n" + "=" * 100)
    print("DUAL-MODE ARCHITECTURE SUMMARY")
    print("=" * 100)
    print(f"""
┌────────────────────────────────────────────────────────────────────────────┐
│                    NX-MIMOSA v3.3 DUAL OUTPUT STREAMS                       │
├────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  STREAM 1: get_forward_estimates()                                          │
│    • Latency: 0 ms (real-time)                                              │
│    • Use for: Display, situational awareness, immediate decisions           │
│                                                                             │
│  STREAM 2: get_window_smoothed_estimates(N)                                 │
│    • Latency: N × dt (e.g., 30 steps × 50ms = 1.5s)                         │
│    • Use for: Fire control, weapon guidance, sensor fusion                  │
│    • Achieves ~50-80% of full smooth improvement                            │
│                                                                             │
│  STREAM 3: get_smoothed_estimates()                                         │
│    • Latency: Full track (offline only)                                     │
│    • Use for: Post-mission analysis, training data, forensics              │
│                                                                             │
├────────────────────────────────────────────────────────────────────────────┤
│  OPERATIONAL CONCEPT:                                                       │
│    • Run forward IMM in real-time for display                               │
│    • Maintain N-step buffer for sliding window smooth                       │
│    • Fire control uses delayed-but-refined track                            │
│    • After mission: run full smooth for analysis                            │
└────────────────────────────────────────────────────────────────────────────┘
""")


if __name__ == "__main__":
    run_dual_benchmark()
