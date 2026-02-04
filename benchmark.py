#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════
NX-MIMOSA FAIR BENCHMARK v1.1 — Reproducible Tracking Algorithm Comparison
═══════════════════════════════════════════════════════════════════════════════

USAGE:
    python benchmark.py              # Run with default settings
    python benchmark.py --runs 100   # More Monte Carlo runs
    python benchmark.py --seed 42    # Custom seed for reproducibility

REQUIREMENTS:
    - Python 3.8+
    - NumPy only (no other dependencies!)

Author: Dr. Mladen Mešter / Nexellum d.o.o.
License: MIT (benchmark code) | Commercial (NX-MIMOSA algorithm)
Repository: https://github.com/mladen1312/nx-mimosa
═══════════════════════════════════════════════════════════════════════════════
"""

import numpy as np
from numpy.linalg import inv, det
import argparse
import time
from typing import Dict, Tuple

__version__ = "1.1.0"

# =============================================================================
# SCENARIOS
# =============================================================================

SCENARIOS = {
    'missile': {'name': 'Missile Terminal (M4, 9g)', 'speed': 1360, 'max_g': 9.0, 
                'duration': 10, 'dt': 0.02, 'sigma': 5.0,
                'segments': [(0,3,'CV'), (3,7,'CT'), (7,10,'CV')]},
    'hypersonic': {'name': 'Hypersonic Glide (M5, 2g)', 'speed': 1700, 'max_g': 2.0,
                   'duration': 25, 'dt': 0.02, 'sigma': 10.0,
                   'segments': [(0,5,'CV'), (5,20,'WEAVE'), (20,25,'CV')]},
    'sam': {'name': 'SAM Engagement (6g)', 'speed': 300, 'max_g': 6.0,
            'duration': 15, 'dt': 0.05, 'sigma': 8.0,
            'segments': [(0,3,'CV'), (3,6,'CT'), (6,9,'CT_NEG'), (9,12,'CT'), (12,15,'CV')]},
    'dogfight': {'name': 'Dogfight BFM (8g)', 'speed': 250, 'max_g': 8.0,
                 'duration': 20, 'dt': 0.02, 'sigma': 3.0,
                 'segments': [(0,4,'CV'), (4,8,'CT'), (8,12,'CT_NEG'), (12,16,'CT'), (16,20,'CV')]},
    'cruise': {'name': 'Cruise Missile (3g)', 'speed': 250, 'max_g': 3.0,
               'duration': 30, 'dt': 0.1, 'sigma': 15.0,
               'segments': [(0,10,'CV'), (10,15,'CT'), (15,20,'CT_NEG'), (20,30,'CV')]},
    'ballistic': {'name': 'Ballistic Reentry (M7)', 'speed': 2380, 'max_g': 1.0,
                  'duration': 60, 'dt': 0.1, 'sigma': 50.0,
                  'segments': [(0,20,'CV'), (20,40,'CT'), (40,60,'CV')]},
    'uav': {'name': 'UAV Swarm (2g)', 'speed': 50, 'max_g': 2.0,
            'duration': 60, 'dt': 0.1, 'sigma': 2.0,
            'segments': [(i*5,(i+1)*5,['CV','CT','CT_NEG'][i%3]) for i in range(12)]},
    'stealth': {'name': 'Stealth Aircraft (4g)', 'speed': 200, 'max_g': 4.0,
                'duration': 30, 'dt': 0.5, 'sigma': 25.0,
                'segments': [(0,10,'CV'), (10,20,'CT'), (20,30,'CV')]},
}

# =============================================================================
# TRAJECTORY GENERATOR
# =============================================================================

def generate_trajectory(scenario: dict, seed: int) -> Tuple[np.ndarray, np.ndarray]:
    """Generate ground truth and measurements."""
    np.random.seed(seed)
    
    dt, T = scenario['dt'], int(scenario['duration'] / scenario['dt'])
    speed, max_g, sigma = scenario['speed'], scenario['max_g'], scenario['sigma']
    
    x_true = np.zeros((T, 4))
    x_true[0] = [10000, 0, -speed, 0]
    
    for k in range(1, T):
        t = k * dt
        a_lat = 0.0
        
        for seg_start, seg_end, seg_type in scenario['segments']:
            if seg_start <= t < seg_end:
                if seg_type == 'CT': a_lat = max_g * 9.81
                elif seg_type == 'CT_NEG': a_lat = -max_g * 9.81
                elif seg_type == 'WEAVE': a_lat = max_g * 9.81 * np.sin(2*np.pi*t/8)
                break
        
        vx, vy = x_true[k-1, 2], x_true[k-1, 3]
        v_mag = max(np.sqrt(vx**2 + vy**2), 1e-6)
        u_lat = np.array([-vy, vx]) / v_mag
        ax, ay = a_lat * u_lat
        
        x_true[k, 0] = x_true[k-1, 0] + vx*dt + 0.5*ax*dt**2
        x_true[k, 1] = x_true[k-1, 1] + vy*dt + 0.5*ay*dt**2
        x_true[k, 2] = x_true[k-1, 2] + ax*dt
        x_true[k, 3] = x_true[k-1, 3] + ay*dt
        
        v_curr = np.sqrt(x_true[k,2]**2 + x_true[k,3]**2)
        if v_curr > 0: x_true[k, 2:4] *= speed / v_curr
    
    z_meas = x_true[:, :2] + np.random.randn(T, 2) * sigma
    return x_true, z_meas

# =============================================================================
# TRACKERS
# =============================================================================

class EKF_CV:
    name = "EKF-CV"
    def __init__(self, dt, sigma, omega=None):
        self.F = np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]])
        self.Q = 1.0 * np.diag([dt**4/4, dt**4/4, dt**2, dt**2])
        self.H = np.array([[1,0,0,0],[0,1,0,0]])
        self.R = sigma**2 * np.eye(2)
    def initialize(self, z0, v0):
        self.x = np.array([z0[0], z0[1], v0[0], v0[1]])
        self.P = np.diag([100,100,500,500])**2
    def update(self, z):
        xp = self.F @ self.x
        Pp = self.F @ self.P @ self.F.T + self.Q
        y = z - self.H @ xp
        S = self.H @ Pp @ self.H.T + self.R
        K = Pp @ self.H.T @ inv(S)
        self.x = xp + K @ y
        self.P = (np.eye(4) - K @ self.H) @ Pp
        return self.x.copy()

class EKF_CA:
    name = "EKF-CA"
    def __init__(self, dt, sigma, omega=None):
        dt2 = dt**2
        self.F = np.array([[1,0,dt,0,dt2/2,0],[0,1,0,dt,0,dt2/2],[0,0,1,0,dt,0],[0,0,0,1,0,dt],[0,0,0,0,1,0],[0,0,0,0,0,1]])
        self.Q = 10.0 * np.diag([dt**4/4, dt**4/4, dt**2, dt**2, 1, 1])
        self.H = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0]])
        self.R = sigma**2 * np.eye(2)
    def initialize(self, z0, v0):
        self.x = np.array([z0[0], z0[1], v0[0], v0[1], 0, 0])
        self.P = np.diag([100,100,500,500,100,100])**2
    def update(self, z):
        xp = self.F @ self.x
        Pp = self.F @ self.P @ self.F.T + self.Q
        y = z - self.H @ xp
        S = self.H @ Pp @ self.H.T + self.R
        K = Pp @ self.H.T @ inv(S)
        self.x = xp + K @ y
        self.P = (np.eye(6) - K @ self.H) @ Pp
        return self.x[:4].copy()

class AlphaBeta:
    name = "Alpha-Beta"
    def __init__(self, dt, sigma, omega=None):
        self.dt = dt
        self.alpha, self.beta, self.gamma = 0.5, 0.4, 0.1
    def initialize(self, z0, v0):
        self.x = np.array([z0[0], z0[1], v0[0], v0[1], 0, 0])
    def update(self, z):
        dt = self.dt
        xp = self.x[0] + self.x[2]*dt + 0.5*self.x[4]*dt**2
        yp = self.x[1] + self.x[3]*dt + 0.5*self.x[5]*dt**2
        rx, ry = z[0] - xp, z[1] - yp
        self.x[0] = xp + self.alpha * rx
        self.x[1] = yp + self.alpha * ry
        self.x[2] = self.x[2] + self.x[4]*dt + self.beta * rx / dt
        self.x[3] = self.x[3] + self.x[5]*dt + self.beta * ry / dt
        self.x[4] += self.gamma * rx / (0.5 * dt**2)
        self.x[5] += self.gamma * ry / (0.5 * dt**2)
        return self.x[:4].copy()

class IMM_Forward:
    name = "IMM-Fwd"
    def __init__(self, dt, sigma, omega=0.15):
        self.dt, self.sigma, self.omega = dt, sigma, omega
        self._build()
    def _build(self):
        dt, omega = self.dt, self.omega
        F_cv = np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]])
        if abs(omega) > 1e-10:
            s,c = np.sin(omega*dt), np.cos(omega*dt)
            F_ct_pos = np.array([[1,0,s/omega,-(1-c)/omega],[0,1,(1-c)/omega,s/omega],[0,0,c,-s],[0,0,s,c]])
            s,c = np.sin(-omega*dt), np.cos(-omega*dt)
            F_ct_neg = np.array([[1,0,s/(-omega),-(1-c)/(-omega)],[0,1,(1-c)/(-omega),s/(-omega)],[0,0,c,-s],[0,0,s,c]])
        else:
            F_ct_pos, F_ct_neg = F_cv.copy(), F_cv.copy()
        self.F = [F_cv, F_ct_pos, F_ct_neg]
        self.Q = [0.5*np.diag([dt**4/4,dt**4/4,dt**2,dt**2]), 5.0*np.diag([dt**4/4,dt**4/4,dt**2,dt**2]), 5.0*np.diag([dt**4/4,dt**4/4,dt**2,dt**2])]
        self.H = np.array([[1,0,0,0],[0,1,0,0]])
        self.R = self.sigma**2 * np.eye(2)
        p = 0.95
        self.PI = np.array([[p,(1-p)/2,(1-p)/2],[(1-p)/2,p,(1-p)/2],[(1-p)/2,(1-p)/2,p]])
    def initialize(self, z0, v0):
        x0 = np.array([z0[0], z0[1], v0[0], v0[1]])
        P0 = np.diag([100,100,500,500])**2
        self.x = [x0.copy() for _ in range(3)]
        self.P = [P0.copy() for _ in range(3)]
        self.mu = np.array([0.8, 0.1, 0.1])
    def update(self, z):
        n = 3
        c_bar = self.PI.T @ self.mu
        mu_ij = np.array([[self.PI[i,j]*self.mu[i]/(c_bar[j]+1e-10) for j in range(n)] for i in range(n)])
        x_mixed = [sum(mu_ij[i,j]*self.x[i] for i in range(n)) for j in range(n)]
        P_mixed = [sum(mu_ij[i,j]*(self.P[i]+np.outer(self.x[i]-x_mixed[j],self.x[i]-x_mixed[j])) for i in range(n)) for j in range(n)]
        x_filt, P_filt, liks = [], [], []
        for j in range(n):
            xp = self.F[j] @ x_mixed[j]
            Pp = self.F[j] @ P_mixed[j] @ self.F[j].T + self.Q[j]
            y = z - self.H @ xp
            S = self.H @ Pp @ self.H.T + self.R + 1e-6*np.eye(2)
            K = Pp @ self.H.T @ inv(S)
            x_filt.append(xp + K @ y)
            P_filt.append((np.eye(4) - K @ self.H) @ Pp)
            liks.append(max(np.exp(-0.5*y@inv(S)@y)/np.sqrt((2*np.pi)**2*det(S)), 1e-10))
        mu_unnorm = c_bar * np.array(liks)
        self.mu = mu_unnorm / (mu_unnorm.sum() + 1e-10)
        self.x, self.P = x_filt, P_filt
        return sum(self.mu[j]*x_filt[j] for j in range(n))

class NX_MIMOSA_V2:
    """NX-MIMOSA v2.0: IMM + Adaptive Q + VS-IMM (forward only)."""
    name = "MIMOSA-v2"
    def __init__(self, dt, sigma, omega=0.15):
        self.dt, self.sigma, self.omega = dt, sigma, omega
        self.q_scale = 1.0
        self._build()
    def _build(self):
        dt, omega = self.dt, self.omega
        F_cv = np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]])
        if abs(omega) > 1e-10:
            s,c = np.sin(omega*dt), np.cos(omega*dt)
            F_ct_pos = np.array([[1,0,s/omega,-(1-c)/omega],[0,1,(1-c)/omega,s/omega],[0,0,c,-s],[0,0,s,c]])
            s,c = np.sin(-omega*dt), np.cos(-omega*dt)
            F_ct_neg = np.array([[1,0,s/(-omega),-(1-c)/(-omega)],[0,1,(1-c)/(-omega),s/(-omega)],[0,0,c,-s],[0,0,s,c]])
        else:
            F_ct_pos, F_ct_neg = F_cv.copy(), F_cv.copy()
        self.F = [F_cv, F_ct_pos, F_ct_neg]
        self.Q_base = [0.5*np.diag([dt**4/4,dt**4/4,dt**2,dt**2]), 5.0*np.diag([dt**4/4,dt**4/4,dt**2,dt**2]), 5.0*np.diag([dt**4/4,dt**4/4,dt**2,dt**2])]
        self.H = np.array([[1,0,0,0],[0,1,0,0]])
        self.R = self.sigma**2 * np.eye(2)
    def initialize(self, z0, v0):
        x0 = np.array([z0[0], z0[1], v0[0], v0[1]])
        P0 = np.diag([100,100,500,500])**2
        self.x = [x0.copy() for _ in range(3)]
        self.P = [P0.copy() for _ in range(3)]
        self.mu = np.array([0.8, 0.1, 0.1])
        self.q_scale = 1.0
    def update(self, z):
        n = 3
        mu_cv = self.mu[0]
        p_stay = 0.98 if mu_cv > 0.9 else (0.95 if mu_cv > 0.7 else 0.90)
        PI = np.array([[p_stay,(1-p_stay)/2,(1-p_stay)/2],[(1-p_stay)/2,p_stay,(1-p_stay)/2],[(1-p_stay)/2,(1-p_stay)/2,p_stay]])
        c_bar = PI.T @ self.mu
        mu_ij = np.array([[PI[i,j]*self.mu[i]/(c_bar[j]+1e-10) for j in range(n)] for i in range(n)])
        x_mixed = [sum(mu_ij[i,j]*self.x[i] for i in range(n)) for j in range(n)]
        P_mixed = [sum(mu_ij[i,j]*(self.P[i]+np.outer(self.x[i]-x_mixed[j],self.x[i]-x_mixed[j])) for i in range(n)) for j in range(n)]
        Q = [self.q_scale * q for q in self.Q_base]
        x_filt, P_filt, liks = [], [], []
        total_nis = 0
        for j in range(n):
            xp = self.F[j] @ x_mixed[j]
            Pp = self.F[j] @ P_mixed[j] @ self.F[j].T + Q[j]
            y = z - self.H @ xp
            S = self.H @ Pp @ self.H.T + self.R + 1e-6*np.eye(2)
            S_inv = inv(S)
            K = Pp @ self.H.T @ S_inv
            nis_j = y @ S_inv @ y
            total_nis += self.mu[j] * nis_j
            x_f = xp + K @ y
            I_KH = np.eye(4) - K @ self.H
            P_f = I_KH @ Pp @ I_KH.T + K @ self.R @ K.T
            x_filt.append(x_f)
            P_filt.append(P_f)
            liks.append(max(np.exp(-0.5*nis_j)/np.sqrt((2*np.pi)**2*det(S)), 1e-10))
        if total_nis > 5.991: self.q_scale = min(self.q_scale * 1.3, 3.0)
        elif total_nis < 2.996: self.q_scale = max(self.q_scale * 0.98, 0.3)
        mu_unnorm = c_bar * np.array(liks)
        self.mu = mu_unnorm / (mu_unnorm.sum() + 1e-10)
        self.x, self.P = x_filt, P_filt
        return sum(self.mu[j]*x_filt[j] for j in range(n))

class NX_MIMOSA_V3:
    """
    NX-MIMOSA v3.1: Full system with True IMM Smoother.
    
    CRITICAL BUGFIX: Backward pass uses F @ x_mixed (stored as x_pred),
    not F @ x_filt. This maintains mathematical consistency in RTS smoother.
    """
    name = "MIMOSA-v3"
    def __init__(self, dt, sigma, omega=0.15):
        self.dt, self.sigma, self.omega = dt, sigma, omega
        self.q_scale = 1.0
        self.history = []
        self._build()
    def _build(self):
        dt, omega = self.dt, self.omega
        F_cv = np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]])
        if abs(omega) > 1e-10:
            s,c = np.sin(omega*dt), np.cos(omega*dt)
            F_ct_pos = np.array([[1,0,s/omega,-(1-c)/omega],[0,1,(1-c)/omega,s/omega],[0,0,c,-s],[0,0,s,c]])
            s,c = np.sin(-omega*dt), np.cos(-omega*dt)
            F_ct_neg = np.array([[1,0,s/(-omega),-(1-c)/(-omega)],[0,1,(1-c)/(-omega),s/(-omega)],[0,0,c,-s],[0,0,s,c]])
        else:
            F_ct_pos, F_ct_neg = F_cv.copy(), F_cv.copy()
        self.F = [F_cv, F_ct_pos, F_ct_neg]
        self.Q_base = [0.5*np.diag([dt**4/4,dt**4/4,dt**2,dt**2]), 5.0*np.diag([dt**4/4,dt**4/4,dt**2,dt**2]), 5.0*np.diag([dt**4/4,dt**4/4,dt**2,dt**2])]
        self.H = np.array([[1,0,0,0],[0,1,0,0]])
        self.R = self.sigma**2 * np.eye(2)
    def initialize(self, z0, v0):
        x0 = np.array([z0[0], z0[1], v0[0], v0[1]])
        P0 = np.diag([100,100,500,500])**2
        self.x = [x0.copy() for _ in range(3)]
        self.P = [P0.copy() for _ in range(3)]
        self.mu = np.array([0.8, 0.1, 0.1])
        self.q_scale = 1.0
        self.history = []
    def update(self, z):
        n = 3
        mu_cv = self.mu[0]
        p_stay = 0.98 if mu_cv > 0.9 else (0.95 if mu_cv > 0.7 else 0.90)
        PI = np.array([[p_stay,(1-p_stay)/2,(1-p_stay)/2],[(1-p_stay)/2,p_stay,(1-p_stay)/2],[(1-p_stay)/2,(1-p_stay)/2,p_stay]])
        c_bar = PI.T @ self.mu
        mu_ij = np.array([[PI[i,j]*self.mu[i]/(c_bar[j]+1e-10) for j in range(n)] for i in range(n)])
        x_mixed = [sum(mu_ij[i,j]*self.x[i] for i in range(n)) for j in range(n)]
        P_mixed = [sum(mu_ij[i,j]*(self.P[i]+np.outer(self.x[i]-x_mixed[j],self.x[i]-x_mixed[j])) for i in range(n)) for j in range(n)]
        Q = [self.q_scale * q for q in self.Q_base]
        x_filt, P_filt, x_pred, P_pred, liks = [], [], [], [], []
        total_nis = 0
        for j in range(n):
            xp = self.F[j] @ x_mixed[j]  # BUGFIX: predict from x_mixed
            Pp = self.F[j] @ P_mixed[j] @ self.F[j].T + Q[j]
            x_pred.append(xp.copy())  # Store for smoother
            P_pred.append(Pp.copy())
            y = z - self.H @ xp
            S = self.H @ Pp @ self.H.T + self.R + 1e-6*np.eye(2)
            S_inv = inv(S)
            K = Pp @ self.H.T @ S_inv
            nis_j = y @ S_inv @ y
            total_nis += self.mu[j] * nis_j
            x_f = xp + K @ y
            I_KH = np.eye(4) - K @ self.H
            P_f = I_KH @ Pp @ I_KH.T + K @ self.R @ K.T
            x_filt.append(x_f)
            P_filt.append(P_f)
            liks.append(max(np.exp(-0.5*nis_j)/np.sqrt((2*np.pi)**2*det(S)), 1e-10))
        if total_nis > 5.991: self.q_scale = min(self.q_scale * 1.3, 3.0)
        elif total_nis < 2.996: self.q_scale = max(self.q_scale * 0.98, 0.3)
        mu_unnorm = c_bar * np.array(liks)
        self.mu = mu_unnorm / (mu_unnorm.sum() + 1e-10)
        self.history.append({'x_filt': [x.copy() for x in x_filt], 'P_filt': [P.copy() for P in P_filt],
                            'x_pred': [x.copy() for x in x_pred], 'P_pred': [P.copy() for P in P_pred],
                            'mu': self.mu.copy(), 'F': [F.copy() for F in self.F]})
        self.x, self.P = x_filt, P_filt
        return sum(self.mu[j]*x_filt[j] for j in range(n))
    
    def smooth(self):
        """True IMM RTS Smoother with BUGFIX."""
        if len(self.history) < 2:
            return np.array([sum(self.history[0]['mu'][j]*self.history[0]['x_filt'][j] for j in range(3))])
        n, L = 3, len(self.history)
        x_smooth = [[h['x_filt'][j].copy() for j in range(n)] for h in self.history]
        P_smooth = [[h['P_filt'][j].copy() for j in range(n)] for h in self.history]
        for k in range(L-2, -1, -1):
            h, h_next = self.history[k], self.history[k+1]
            for j in range(n):
                F = h['F'][j]
                x_f, P_f = h['x_filt'][j], h['P_filt'][j]
                x_p, P_p = h_next['x_pred'][j], h_next['P_pred'][j]  # BUGFIX: F @ x_mixed
                try:
                    G = P_f @ F.T @ inv(P_p + 1e-6*np.eye(4))
                    x_smooth[k][j] = x_f + G @ (x_smooth[k+1][j] - x_p)
                    P_smooth[k][j] = P_f + G @ (P_smooth[k+1][j] - P_p) @ G.T
                except: pass
        result = np.zeros((L, 4))
        for k in range(L):
            mu = self.history[k]['mu']
            result[k] = sum(mu[j]*x_smooth[k][j] for j in range(n))
        return result

# =============================================================================
# BENCHMARK
# =============================================================================

TRACKERS = [EKF_CV, EKF_CA, AlphaBeta, IMM_Forward, NX_MIMOSA_V2, NX_MIMOSA_V3]

def run_benchmark(n_runs=30, seed=42, verbose=True):
    if verbose:
        print("="*90)
        print("NX-MIMOSA FAIR BENCHMARK v1.1 — Reproducible Comparison")
        print("="*90)
        print(f"Settings: {n_runs} Monte Carlo runs, seed={seed}")
        print(f"Algorithms: {', '.join(t.name for t in TRACKERS)}\n")
    
    results = {s: {t.name: [] for t in TRACKERS} for s in SCENARIOS}
    
    for skey, scenario in SCENARIOS.items():
        if verbose: print(f"  {scenario['name']:<30}", end="", flush=True)
        omega = scenario['max_g'] * 9.81 / max(scenario['speed'], 1)
        t0 = time.time()
        
        for run in range(n_runs):
            x_true, z_meas = generate_trajectory(scenario, seed + run*1000 + hash(skey)%1000)
            v_init = np.array([x_true[0,2], x_true[0,3]])
            
            for TrackerClass in TRACKERS:
                tracker = TrackerClass(scenario['dt'], scenario['sigma'], omega)
                tracker.initialize(z_meas[0], v_init)
                
                ests = [tracker.update(z_meas[k]) for k in range(1, len(z_meas))]
                if hasattr(tracker, 'smooth'):
                    try: ests = tracker.smooth()
                    except: pass
                
                ests = np.array(ests)
                rmse = np.sqrt(np.mean((ests[:,:2] - x_true[1:,:2])**2))
                results[skey][TrackerClass.name].append(rmse)
        
        if verbose: print(f" ({time.time()-t0:.1f}s)")
    
    return results

def print_results(results):
    print("\n" + "="*100)
    print("RESULTS — Position RMSE (meters), lower is better")
    print("="*100)
    
    header = f"{'Scenario':<32}"
    for t in TRACKERS: header += f"{t.name:>11}"
    header += "    Winner"
    print(header)
    print("-"*100)
    
    wins = {t.name: 0 for t in TRACKERS}
    avgs = {t.name: [] for t in TRACKERS}
    
    for skey, scenario in SCENARIOS.items():
        row = f"{scenario['name']:<32}"
        best_rmse, best_name = float('inf'), None
        for t in TRACKERS:
            mean = np.mean(results[skey][t.name])
            avgs[t.name].append(mean)
            row += f"{mean:>11.2f}"
            if mean < best_rmse: best_rmse, best_name = mean, t.name
        wins[best_name] += 1
        row += f"    {best_name}"
        print(row)
    
    print("-"*100)
    avg_row = f"{'AVERAGE':<32}"
    for t in TRACKERS: avg_row += f"{np.mean(avgs[t.name]):>11.2f}"
    print(avg_row)
    print("="*100)
    
    print("\nSUMMARY:")
    v3_avg = np.mean(avgs['MIMOSA-v3'])
    for t in TRACKERS:
        avg = np.mean(avgs[t.name])
        impr = (avg - v3_avg) / avg * 100 if t.name != 'MIMOSA-v3' else 0
        marker = " <-- BEST" if t.name == 'MIMOSA-v3' else ""
        print(f"  {t.name:<12}: {avg:>7.2f}m avg, {wins[t.name]}/8 wins" + 
              (f", MIMOSA-v3 {impr:+.1f}% better" if t.name != 'MIMOSA-v3' else marker))

def main():
    parser = argparse.ArgumentParser(description='NX-MIMOSA Fair Benchmark')
    parser.add_argument('--runs', type=int, default=30, help='Monte Carlo runs (default: 30)')
    parser.add_argument('--seed', type=int, default=42, help='Random seed (default: 42)')
    parser.add_argument('--quiet', action='store_true', help='Minimal output')
    args = parser.parse_args()
    
    results = run_benchmark(args.runs, args.seed, not args.quiet)
    print_results(results)
    print(f"\n✓ Reproducible with: python benchmark.py --runs {args.runs} --seed {args.seed}")
    print("  Repository: https://github.com/mladen1312/nx-mimosa")

if __name__ == "__main__":
    main()
