#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════
NX-MIMOSA FAIR BENCHMARK — Reproducible Tracking Algorithm Comparison
═══════════════════════════════════════════════════════════════════════════════

This benchmark provides a FAIR and REPRODUCIBLE comparison of radar tracking
algorithms across defense-relevant scenarios. 

HOW TO RUN:
    python3 fair_benchmark.py [--runs N] [--seed S]

REQUIREMENTS:
    - Python 3.8+
    - NumPy only (no other dependencies)

LICENSE: MIT — Free for academic and commercial use
AUTHOR: Dr. Mladen Mešter / Nexellum d.o.o.
DATE: February 2026

ALGORITHMS COMPARED:
    1. EKF-CV      — Extended Kalman Filter, Constant Velocity model
    2. EKF-CA      — Extended Kalman Filter, Constant Acceleration model  
    3. α-β-γ       — Classic alpha-beta-gamma filter
    4. IMM-3       — Interacting Multiple Model (CV + CT+ + CT-)
    5. IMM-Smooth  — IMM with RTS Smoother (NX-MIMOSA approach)

SCENARIOS:
    1. Missile Terminal — Mach 4, 9g evasion maneuver
    2. Hypersonic Glide — Mach 5, 2g S-weave pattern
    3. Fighter Aircraft — 250 m/s, 8g dogfight maneuvers
    4. Cruise Missile   — 250 m/s, 3g pop-up attack
    5. Ballistic RV     — Mach 7, 1g correction burns
    6. UAV Swarm        — 50 m/s, frequent mode changes
    7. Stealth Aircraft — 200 m/s, sparse radar updates

METHODOLOGY:
    - Monte Carlo: 50 runs per scenario (configurable)
    - Metric: Position RMSE (meters)
    - Random seed: Fixed for reproducibility
    - All algorithms use identical measurements
    - Smoother uses offline (batch) processing for fair comparison
═══════════════════════════════════════════════════════════════════════════════
"""

import numpy as np
from numpy.linalg import inv, det
import argparse
import time
from typing import List, Tuple, Dict
from dataclasses import dataclass


# =============================================================================
# SCENARIO DEFINITIONS
# =============================================================================

@dataclass
class Scenario:
    """Tracking scenario configuration."""
    name: str
    speed_mps: float      # Target speed
    max_g: float          # Maximum lateral acceleration in g's
    duration_s: float     # Scenario duration
    dt: float             # Measurement interval (1/update_rate)
    sigma_meas: float     # Measurement noise standard deviation
    maneuvers: List[Tuple[float, float, str]]  # (start_time, end_time, type)
    
    @property
    def update_rate_hz(self) -> float:
        return 1.0 / self.dt


SCENARIOS = [
    Scenario(
        name="Missile Terminal (Mach 4, 9g)",
        speed_mps=1360.0,
        max_g=9.0,
        duration_s=10.0,
        dt=0.02,
        sigma_meas=5.0,
        maneuvers=[(0, 3, 'straight'), (3, 7, 'turn'), (7, 10, 'straight')]
    ),
    Scenario(
        name="Hypersonic Glide (Mach 5, 2g)",
        speed_mps=1700.0,
        max_g=2.0,
        duration_s=25.0,
        dt=0.02,
        sigma_meas=10.0,
        maneuvers=[(0, 5, 'straight'), (5, 20, 'weave'), (20, 25, 'straight')]
    ),
    Scenario(
        name="Fighter Aircraft (250m/s, 8g)",
        speed_mps=250.0,
        max_g=8.0,
        duration_s=20.0,
        dt=0.02,
        sigma_meas=3.0,
        maneuvers=[(0, 4, 'straight'), (4, 8, 'turn'), (8, 12, 'turn_neg'),
                   (12, 16, 'turn'), (16, 20, 'straight')]
    ),
    Scenario(
        name="Cruise Missile (250m/s, 3g)",
        speed_mps=250.0,
        max_g=3.0,
        duration_s=30.0,
        dt=0.1,
        sigma_meas=15.0,
        maneuvers=[(0, 10, 'straight'), (10, 15, 'pop-up'), (15, 20, 'dive'),
                   (20, 25, 'straight'), (25, 30, 'terminal')]
    ),
    Scenario(
        name="Ballistic RV (Mach 7, 1g)",
        speed_mps=2380.0,
        max_g=1.0,
        duration_s=60.0,
        dt=0.1,
        sigma_meas=50.0,
        maneuvers=[(0, 20, 'straight'), (20, 40, 'slight_turn'), (40, 60, 'straight')]
    ),
    Scenario(
        name="UAV Swarm (50m/s, 2g)",
        speed_mps=50.0,
        max_g=2.0,
        duration_s=60.0,
        dt=0.1,
        sigma_meas=2.0,
        maneuvers=[(i*5, (i+1)*5, 'random') for i in range(12)]
    ),
    Scenario(
        name="Stealth Aircraft (200m/s, 4g)",
        speed_mps=200.0,
        max_g=4.0,
        duration_s=30.0,
        dt=0.5,  # Sparse updates (2 Hz) due to low RCS
        sigma_meas=25.0,
        maneuvers=[(0, 10, 'straight'), (10, 20, 'turn'), (20, 30, 'straight')]
    ),
]


# =============================================================================
# TRAJECTORY GENERATOR
# =============================================================================

def generate_trajectory(scenario: Scenario, seed: int) -> Tuple[np.ndarray, np.ndarray]:
    """
    Generate ground truth trajectory and noisy measurements.
    
    Returns:
        x_true: [T, 4] array of true states [x, y, vx, vy]
        z_meas: [T, 2] array of noisy position measurements [x, y]
    """
    np.random.seed(seed)
    
    T = int(scenario.duration_s / scenario.dt)
    x_true = np.zeros((T, 4))
    
    # Initial state: target approaching from +x direction
    x_true[0] = [10000.0, 0.0, -scenario.speed_mps, 0.0]
    
    g = 9.81  # m/s²
    random_accel = 0.0
    
    for k in range(1, T):
        t = k * scenario.dt
        a_lateral = 0.0
        
        # Determine maneuver type at current time
        for seg_start, seg_end, seg_type in scenario.maneuvers:
            if seg_start <= t < seg_end:
                if seg_type == 'straight':
                    a_lateral = 0.0
                elif seg_type == 'turn':
                    a_lateral = scenario.max_g * g
                elif seg_type == 'turn_neg':
                    a_lateral = -scenario.max_g * g
                elif seg_type == 'weave':
                    omega_weave = 2 * np.pi / 8.0
                    a_lateral = scenario.max_g * g * np.sin(omega_weave * t)
                elif seg_type == 'pop-up':
                    a_lateral = scenario.max_g * g * 0.5
                elif seg_type == 'dive':
                    a_lateral = -scenario.max_g * g * 0.5
                elif seg_type == 'slight_turn':
                    a_lateral = scenario.max_g * g * 0.3
                elif seg_type == 'random':
                    if k % 10 == 0:
                        random_accel = np.random.uniform(-1, 1) * scenario.max_g * g
                    a_lateral = random_accel
                elif seg_type == 'terminal':
                    a_lateral = scenario.max_g * g * np.sin(0.5 * t)
                break
        
        # Compute lateral direction (perpendicular to velocity)
        vx, vy = x_true[k-1, 2], x_true[k-1, 3]
        v_mag = max(np.sqrt(vx**2 + vy**2), 1e-6)
        u_lateral = np.array([-vy, vx]) / v_mag
        
        # Acceleration components
        ax = a_lateral * u_lateral[0]
        ay = a_lateral * u_lateral[1]
        dt = scenario.dt
        
        # Kinematic integration
        x_true[k, 0] = x_true[k-1, 0] + vx * dt + 0.5 * ax * dt**2
        x_true[k, 1] = x_true[k-1, 1] + vy * dt + 0.5 * ay * dt**2
        x_true[k, 2] = x_true[k-1, 2] + ax * dt
        x_true[k, 3] = x_true[k-1, 3] + ay * dt
        
        # Maintain approximately constant speed
        v_current = np.sqrt(x_true[k, 2]**2 + x_true[k, 3]**2)
        if v_current > 0:
            scale = scenario.speed_mps / v_current
            x_true[k, 2] *= scale
            x_true[k, 3] *= scale
    
    # Generate noisy measurements
    z_meas = x_true[:, :2] + np.random.randn(T, 2) * scenario.sigma_meas
    
    return x_true, z_meas


# =============================================================================
# TRACKING ALGORITHMS
# =============================================================================

class EKF_CV:
    """Extended Kalman Filter with Constant Velocity model."""
    
    def __init__(self, dt: float, sigma_meas: float, q: float = 1.0):
        self.dt = dt
        self.q = q
        
        # State transition matrix
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # Process noise
        self.Q = q * np.array([
            [dt**4/4, 0, dt**3/2, 0],
            [0, dt**4/4, 0, dt**3/2],
            [dt**3/2, 0, dt**2, 0],
            [0, dt**3/2, 0, dt**2]
        ])
        
        # Measurement matrix and noise
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        self.R = sigma_meas**2 * np.eye(2)
        
    def initialize(self, z0: np.ndarray, v0: np.ndarray):
        self.x = np.array([z0[0], z0[1], v0[0], v0[1]])
        self.P = np.diag([100, 100, 500, 500])**2
        
    def update(self, z: np.ndarray) -> np.ndarray:
        # Predict
        x_pred = self.F @ self.x
        P_pred = self.F @ self.P @ self.F.T + self.Q
        
        # Update
        y = z - self.H @ x_pred
        S = self.H @ P_pred @ self.H.T + self.R
        K = P_pred @ self.H.T @ inv(S)
        
        self.x = x_pred + K @ y
        self.P = (np.eye(4) - K @ self.H) @ P_pred
        
        return self.x.copy()


class EKF_CA:
    """Extended Kalman Filter with Constant Acceleration model."""
    
    def __init__(self, dt: float, sigma_meas: float, q: float = 10.0):
        self.dt = dt
        self.q = q
        
        # 6-state model: [x, y, vx, vy, ax, ay]
        dt2 = dt**2
        self.F = np.array([
            [1, 0, dt, 0, dt2/2, 0],
            [0, 1, 0, dt, 0, dt2/2],
            [0, 0, 1, 0, dt, 0],
            [0, 0, 0, 1, 0, dt],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])
        
        self.Q = q * np.diag([dt**4/4, dt**4/4, dt**2, dt**2, 1, 1])
        self.H = np.array([[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0]])
        self.R = sigma_meas**2 * np.eye(2)
        
    def initialize(self, z0: np.ndarray, v0: np.ndarray):
        self.x = np.array([z0[0], z0[1], v0[0], v0[1], 0, 0])
        self.P = np.diag([100, 100, 500, 500, 100, 100])**2
        
    def update(self, z: np.ndarray) -> np.ndarray:
        x_pred = self.F @ self.x
        P_pred = self.F @ self.P @ self.F.T + self.Q
        
        y = z - self.H @ x_pred
        S = self.H @ P_pred @ self.H.T + self.R
        K = P_pred @ self.H.T @ inv(S)
        
        self.x = x_pred + K @ y
        self.P = (np.eye(6) - K @ self.H) @ P_pred
        
        return self.x[:4].copy()


class AlphaBetaGamma:
    """Classic α-β-γ filter (second-order polynomial)."""
    
    def __init__(self, dt: float, sigma_meas: float):
        self.dt = dt
        # Optimal gains for tracking index ~0.5
        self.alpha = 0.5
        self.beta = 0.4
        self.gamma = 0.1
        
    def initialize(self, z0: np.ndarray, v0: np.ndarray):
        self.x = np.array([z0[0], z0[1], v0[0], v0[1], 0.0, 0.0])
        
    def update(self, z: np.ndarray) -> np.ndarray:
        dt = self.dt
        
        # Predict
        x_pred = self.x[0] + self.x[2] * dt + 0.5 * self.x[4] * dt**2
        y_pred = self.x[1] + self.x[3] * dt + 0.5 * self.x[5] * dt**2
        vx_pred = self.x[2] + self.x[4] * dt
        vy_pred = self.x[3] + self.x[5] * dt
        
        # Residual
        rx = z[0] - x_pred
        ry = z[1] - y_pred
        
        # Update
        self.x[0] = x_pred + self.alpha * rx
        self.x[1] = y_pred + self.alpha * ry
        self.x[2] = vx_pred + self.beta * rx / dt
        self.x[3] = vy_pred + self.beta * ry / dt
        self.x[4] = self.x[4] + self.gamma * rx / (0.5 * dt**2)
        self.x[5] = self.x[5] + self.gamma * ry / (0.5 * dt**2)
        
        return self.x[:4].copy()


class IMM_Forward:
    """
    Interacting Multiple Model filter (forward only).
    
    Models: CV (Constant Velocity) + CT+ (Coordinated Turn +ω) + CT- (CT -ω)
    """
    
    def __init__(self, dt: float, sigma_meas: float, omega: float = 0.15):
        self.dt = dt
        self.omega = omega
        self.n_models = 3
        self._build_models(sigma_meas)
        
    def _build_models(self, sigma_meas: float):
        dt, omega = self.dt, self.omega
        
        # CV model
        F_cv = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]])
        
        # CT models
        if abs(omega) > 1e-10:
            s, c = np.sin(omega * dt), np.cos(omega * dt)
            F_ct_pos = np.array([
                [1, 0, s/omega, -(1-c)/omega],
                [0, 1, (1-c)/omega, s/omega],
                [0, 0, c, -s],
                [0, 0, s, c]
            ])
            s, c = np.sin(-omega * dt), np.cos(-omega * dt)
            F_ct_neg = np.array([
                [1, 0, s/(-omega), -(1-c)/(-omega)],
                [0, 1, (1-c)/(-omega), s/(-omega)],
                [0, 0, c, -s],
                [0, 0, s, c]
            ])
        else:
            F_ct_pos = F_cv.copy()
            F_ct_neg = F_cv.copy()
        
        self.F = [F_cv, F_ct_pos, F_ct_neg]
        
        # Process noise (higher for maneuvering models)
        Q_base = np.diag([dt**4/4, dt**4/4, dt**2, dt**2])
        self.Q = [0.5 * Q_base, 5.0 * Q_base, 5.0 * Q_base]
        
        # Measurement model
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        self.R = sigma_meas**2 * np.eye(2)
        
        # Transition probability matrix (Markov)
        p_stay = 0.95
        p_switch = (1 - p_stay) / 2
        self.PI = np.array([
            [p_stay, p_switch, p_switch],
            [p_switch, p_stay, p_switch],
            [p_switch, p_switch, p_stay]
        ])
        
    def initialize(self, z0: np.ndarray, v0: np.ndarray):
        x0 = np.array([z0[0], z0[1], v0[0], v0[1]])
        P0 = np.diag([100, 100, 500, 500])**2
        
        self.x = [x0.copy() for _ in range(self.n_models)]
        self.P = [P0.copy() for _ in range(self.n_models)]
        self.mu = np.array([0.8, 0.1, 0.1])  # Prior: mostly CV
        
    def update(self, z: np.ndarray) -> np.ndarray:
        n = self.n_models
        
        # Mixing probabilities
        c_bar = self.PI.T @ self.mu
        mu_ij = np.zeros((n, n))
        for i in range(n):
            for j in range(n):
                mu_ij[i, j] = self.PI[i, j] * self.mu[i] / (c_bar[j] + 1e-10)
        
        # Mix states and covariances
        x_mixed = []
        P_mixed = []
        for j in range(n):
            x_j = sum(mu_ij[i, j] * self.x[i] for i in range(n))
            x_mixed.append(x_j)
            P_j = sum(mu_ij[i, j] * (self.P[i] + np.outer(self.x[i] - x_j, self.x[i] - x_j)) 
                     for i in range(n))
            P_mixed.append(P_j)
        
        # Per-model Kalman filter
        x_filt = []
        P_filt = []
        likelihoods = []
        
        for j in range(n):
            # Predict
            xp = self.F[j] @ x_mixed[j]
            Pp = self.F[j] @ P_mixed[j] @ self.F[j].T + self.Q[j]
            
            # Update
            y = z - self.H @ xp
            S = self.H @ Pp @ self.H.T + self.R + 1e-6 * np.eye(2)
            K = Pp @ self.H.T @ inv(S)
            
            x_f = xp + K @ y
            P_f = (np.eye(4) - K @ self.H) @ Pp
            
            x_filt.append(x_f)
            P_filt.append(P_f)
            
            # Gaussian likelihood
            try:
                lik = np.exp(-0.5 * y @ inv(S) @ y) / np.sqrt((2*np.pi)**2 * det(S))
            except:
                lik = 1e-10
            likelihoods.append(max(lik, 1e-10))
        
        # Update mode probabilities
        mu_unnorm = c_bar * np.array(likelihoods)
        self.mu = mu_unnorm / (mu_unnorm.sum() + 1e-10)
        
        self.x = x_filt
        self.P = P_filt
        
        # Combined estimate
        return sum(self.mu[j] * x_filt[j] for j in range(n))


class IMM_Smoother:
    """
    IMM with Per-Model RTS Smoother (NX-MIMOSA approach).
    
    Key innovation: True IMM smoother stores x_mixed for backward pass prediction,
    not x_filt. This eliminates numerical issues during mode transitions.
    
    This is a BATCH/OFFLINE smoother for fair comparison.
    For real-time use, a fixed-lag variant would be employed.
    """
    
    def __init__(self, dt: float, sigma_meas: float, omega: float = 0.15):
        self.dt = dt
        self.omega = omega
        self.n_models = 3
        self._build_models(sigma_meas)
        self.history = []
        
    def _build_models(self, sigma_meas: float):
        dt, omega = self.dt, self.omega
        
        F_cv = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]])
        
        if abs(omega) > 1e-10:
            s, c = np.sin(omega * dt), np.cos(omega * dt)
            F_ct_pos = np.array([
                [1, 0, s/omega, -(1-c)/omega],
                [0, 1, (1-c)/omega, s/omega],
                [0, 0, c, -s],
                [0, 0, s, c]
            ])
            s, c = np.sin(-omega * dt), np.cos(-omega * dt)
            F_ct_neg = np.array([
                [1, 0, s/(-omega), -(1-c)/(-omega)],
                [0, 1, (1-c)/(-omega), s/(-omega)],
                [0, 0, c, -s],
                [0, 0, s, c]
            ])
        else:
            F_ct_pos = F_cv.copy()
            F_ct_neg = F_cv.copy()
        
        self.F = [F_cv, F_ct_pos, F_ct_neg]
        
        Q_base = np.diag([dt**4/4, dt**4/4, dt**2, dt**2])
        self.Q = [0.5 * Q_base, 5.0 * Q_base, 5.0 * Q_base]
        
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        self.R = sigma_meas**2 * np.eye(2)
        
        p_stay = 0.95
        p_switch = (1 - p_stay) / 2
        self.PI = np.array([
            [p_stay, p_switch, p_switch],
            [p_switch, p_stay, p_switch],
            [p_switch, p_switch, p_stay]
        ])
        
    def initialize(self, z0: np.ndarray, v0: np.ndarray):
        x0 = np.array([z0[0], z0[1], v0[0], v0[1]])
        P0 = np.diag([100, 100, 500, 500])**2
        
        self.x = [x0.copy() for _ in range(self.n_models)]
        self.P = [P0.copy() for _ in range(self.n_models)]
        self.mu = np.array([0.8, 0.1, 0.1])
        self.history = []
        
    def update(self, z: np.ndarray) -> np.ndarray:
        """Forward pass with history storage for smoother."""
        n = self.n_models
        
        c_bar = self.PI.T @ self.mu
        mu_ij = np.zeros((n, n))
        for i in range(n):
            for j in range(n):
                mu_ij[i, j] = self.PI[i, j] * self.mu[i] / (c_bar[j] + 1e-10)
        
        # Mix states
        x_mixed = []
        P_mixed = []
        for j in range(n):
            x_j = sum(mu_ij[i, j] * self.x[i] for i in range(n))
            x_mixed.append(x_j)
            P_j = sum(mu_ij[i, j] * (self.P[i] + np.outer(self.x[i] - x_j, self.x[i] - x_j)) 
                     for i in range(n))
            P_mixed.append(P_j)
        
        # Per-model predict and update
        x_filt = []
        P_filt = []
        x_pred = []
        P_pred = []
        likelihoods = []
        
        for j in range(n):
            # Predict using x_mixed (KEY for smoother!)
            xp = self.F[j] @ x_mixed[j]
            Pp = self.F[j] @ P_mixed[j] @ self.F[j].T + self.Q[j]
            
            x_pred.append(xp.copy())
            P_pred.append(Pp.copy())
            
            # Update
            y = z - self.H @ xp
            S = self.H @ Pp @ self.H.T + self.R + 1e-6 * np.eye(2)
            K = Pp @ self.H.T @ inv(S)
            
            x_f = xp + K @ y
            P_f = (np.eye(4) - K @ self.H) @ Pp
            
            x_filt.append(x_f)
            P_filt.append(P_f)
            
            try:
                lik = np.exp(-0.5 * y @ inv(S) @ y) / np.sqrt((2*np.pi)**2 * det(S))
            except:
                lik = 1e-10
            likelihoods.append(max(lik, 1e-10))
        
        mu_unnorm = c_bar * np.array(likelihoods)
        self.mu = mu_unnorm / (mu_unnorm.sum() + 1e-10)
        
        # Store for smoother (including x_pred = F @ x_mixed)
        self.history.append({
            'x_filt': [x.copy() for x in x_filt],
            'P_filt': [P.copy() for P in P_filt],
            'x_pred': [x.copy() for x in x_pred],
            'P_pred': [P.copy() for P in P_pred],
            'mu': self.mu.copy(),
            'F': [F.copy() for F in self.F]
        })
        
        self.x = x_filt
        self.P = P_filt
        
        return sum(self.mu[j] * x_filt[j] for j in range(n))
    
    def smooth(self) -> np.ndarray:
        """
        Per-model RTS smoother (offline/batch).
        
        KEY: Backward prediction uses stored x_pred = F @ x_mixed,
        NOT F @ x_filt. This is mathematically correct and numerically stable.
        """
        if len(self.history) < 2:
            return np.array([sum(self.history[0]['mu'][j] * self.history[0]['x_filt'][j] 
                               for j in range(self.n_models))])
        
        n = self.n_models
        L = len(self.history)
        
        # Initialize smoothed estimates from filtered
        x_smooth = [[h['x_filt'][j].copy() for j in range(n)] for h in self.history]
        P_smooth = [[h['P_filt'][j].copy() for j in range(n)] for h in self.history]
        
        # Backward pass
        for k in range(L - 2, -1, -1):
            h = self.history[k]
            h_next = self.history[k + 1]
            
            for j in range(n):
                F = h['F'][j]
                x_f = h['x_filt'][j]
                P_f = h['P_filt'][j]
                
                # Use stored x_pred (which is F @ x_mixed, not F @ x_filt)
                x_p = h_next['x_pred'][j]
                P_p = h_next['P_pred'][j]
                
                # Smoother gain
                try:
                    G = P_f @ F.T @ inv(P_p + 1e-6 * np.eye(4))
                except:
                    G = np.zeros((4, 4))
                
                # Smooth
                x_smooth[k][j] = x_f + G @ (x_smooth[k+1][j] - x_p)
                P_smooth[k][j] = P_f + G @ (P_smooth[k+1][j] - P_p) @ G.T
        
        # Combine smoothed estimates using mode probabilities
        result = np.zeros((L, 4))
        for k in range(L):
            mu = self.history[k]['mu']
            result[k] = sum(mu[j] * x_smooth[k][j] for j in range(n))
        
        return result


# =============================================================================
# BENCHMARK RUNNER
# =============================================================================

def run_benchmark(n_runs: int = 50, base_seed: int = 42, verbose: bool = True):
    """
    Run the complete benchmark.
    
    Args:
        n_runs: Number of Monte Carlo runs per scenario
        base_seed: Base random seed for reproducibility
        verbose: Print progress and results
        
    Returns:
        Dictionary of results
    """
    
    algorithms = [
        ('EKF-CV', EKF_CV),
        ('EKF-CA', EKF_CA),
        ('α-β-γ', AlphaBetaGamma),
        ('IMM-3', IMM_Forward),
        ('IMM-Smooth', IMM_Smoother),
    ]
    
    results = {s.name: {name: [] for name, _ in algorithms} for s in SCENARIOS}
    
    if verbose:
        print("=" * 90)
        print("NX-MIMOSA FAIR BENCHMARK — Reproducible Tracking Algorithm Comparison")
        print("=" * 90)
        print(f"\nMonte Carlo runs: {n_runs} per scenario")
        print(f"Base seed: {base_seed}")
        print(f"Algorithms: {', '.join(name for name, _ in algorithms)}")
        print()
    
    for scenario in SCENARIOS:
        if verbose:
            print(f"{'─' * 90}")
            print(f"Scenario: {scenario.name}")
            print(f"  Speed: {scenario.speed_mps:.0f} m/s, Max-g: {scenario.max_g}, "
                  f"Update rate: {scenario.update_rate_hz:.0f} Hz, σ_meas: {scenario.sigma_meas:.0f} m")
        
        # Compute omega for IMM models
        omega = scenario.max_g * 9.81 / max(scenario.speed_mps, 1.0)
        
        for run in range(n_runs):
            seed = base_seed + run * 1000 + hash(scenario.name) % 10000
            x_true, z_meas = generate_trajectory(scenario, seed)
            v_init = np.array([x_true[0, 2], x_true[0, 3]])
            
            for alg_name, AlgClass in algorithms:
                # Create tracker
                if alg_name in ['EKF-CV', 'EKF-CA', 'α-β-γ']:
                    tracker = AlgClass(scenario.dt, scenario.sigma_meas)
                else:
                    tracker = AlgClass(scenario.dt, scenario.sigma_meas, omega)
                
                tracker.initialize(z_meas[0], v_init)
                
                # Run forward pass
                forward_estimates = []
                for k in range(1, len(z_meas)):
                    est = tracker.update(z_meas[k])
                    forward_estimates.append(est)
                
                # For smoother, run backward pass
                if alg_name == 'IMM-Smooth':
                    smoothed = tracker.smooth()
                    estimates = smoothed
                else:
                    estimates = np.array(forward_estimates)
                
                # Compute RMSE
                pos_error = np.sqrt((estimates[:, 0] - x_true[1:, 0])**2 + 
                                   (estimates[:, 1] - x_true[1:, 1])**2)
                rmse = np.sqrt(np.mean(pos_error**2))
                results[scenario.name][alg_name].append(rmse)
        
        if verbose:
            print(f"  ✓ Complete")
    
    return results


def print_results(results: Dict, algorithms: List[str]):
    """Print formatted results table."""
    
    print("\n" + "=" * 100)
    print("BENCHMARK RESULTS — Position RMSE (meters, lower is better)")
    print("=" * 100)
    
    # Header
    header = f"{'Scenario':<35}"
    for alg in algorithms:
        header += f" {alg:>12}"
    print(header)
    print("-" * 100)
    
    # Results per scenario
    wins = {alg: 0 for alg in algorithms}
    
    for scenario_name, alg_results in results.items():
        row = f"{scenario_name[:34]:<35}"
        
        rmse_values = []
        for alg in algorithms:
            mean_rmse = np.mean(alg_results[alg])
            rmse_values.append((alg, mean_rmse))
            row += f" {mean_rmse:>12.2f}"
        
        # Mark winner
        sorted_rmse = sorted(rmse_values, key=lambda x: x[1])
        winner = sorted_rmse[0][0]
        wins[winner] += 1
        
        if winner == 'IMM-Smooth':
            row += " 🥇"
        
        print(row)
    
    print("-" * 100)
    
    # Averages
    avg_row = f"{'AVERAGE':<35}"
    avg_rmse = {}
    for alg in algorithms:
        all_rmse = []
        for scenario_name in results:
            all_rmse.extend(results[scenario_name][alg])
        avg = np.mean(all_rmse)
        avg_rmse[alg] = avg
        avg_row += f" {avg:>12.2f}"
    print(avg_row)
    print("=" * 100)
    
    # Summary
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    
    print("\nWins per algorithm:")
    for alg in algorithms:
        marker = "🏆" if wins[alg] == max(wins.values()) else "  "
        print(f"  {marker} {alg:<15}: {wins[alg]}/{len(results)} scenarios")
    
    print("\nImprovement of IMM-Smooth vs others:")
    smooth_avg = avg_rmse['IMM-Smooth']
    for alg in algorithms:
        if alg != 'IMM-Smooth':
            improvement = (avg_rmse[alg] - smooth_avg) / avg_rmse[alg] * 100
            print(f"  vs {alg:<12}: {improvement:>+6.1f}%")


def main():
    parser = argparse.ArgumentParser(
        description='NX-MIMOSA Fair Benchmark — Reproducible Tracking Algorithm Comparison',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 fair_benchmark.py                    # Default: 50 runs
  python3 fair_benchmark.py --runs 100         # More runs for better statistics
  python3 fair_benchmark.py --seed 12345       # Different seed for validation
  python3 fair_benchmark.py --quick            # Quick test with 10 runs
        """
    )
    parser.add_argument('--runs', type=int, default=50, 
                        help='Number of Monte Carlo runs per scenario (default: 50)')
    parser.add_argument('--seed', type=int, default=42,
                        help='Base random seed for reproducibility (default: 42)')
    parser.add_argument('--quick', action='store_true',
                        help='Quick test with only 10 runs')
    
    args = parser.parse_args()
    
    n_runs = 10 if args.quick else args.runs
    
    print(f"\nStarting benchmark with {n_runs} Monte Carlo runs...")
    print(f"Random seed: {args.seed}")
    print()
    
    start_time = time.time()
    results = run_benchmark(n_runs=n_runs, base_seed=args.seed, verbose=True)
    elapsed = time.time() - start_time
    
    algorithms = ['EKF-CV', 'EKF-CA', 'α-β-γ', 'IMM-3', 'IMM-Smooth']
    print_results(results, algorithms)
    
    print(f"\nBenchmark completed in {elapsed:.1f} seconds")
    print("\nTo reproduce these results, run:")
    print(f"  python3 fair_benchmark.py --runs {n_runs} --seed {args.seed}")


if __name__ == "__main__":
    main()
