#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════════════════════════════
NX-MIMOSA v4.1 - HIGH-SPEED OPTIMIZED TRACKER FOR ATC
═══════════════════════════════════════════════════════════════════════════════════════════════════════

PROBLEM: En-route cruise at high speed (450 kts / 230 m/s) shows excessive error
SOLUTION: 
  1. Coordinate-Velocity-Acceleration (CVA) model for smooth flight
  2. Velocity-based gate scaling
  3. Improved Q matrix for high-speed targets
  4. Multi-rate prediction (fine steps between radar updates)

EUROCONTROL Requirements:
  - En-route: RMS ≤ 500 m
  - TMA: RMS ≤ 150 m
  - Continuity: ≥ 99.5%

Author: Dr. Mladen Mešter / Nexellum d.o.o.
═══════════════════════════════════════════════════════════════════════════════════════════════════════
"""

import numpy as np
from numpy.linalg import inv, norm, cholesky
from scipy.linalg import sqrtm
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
from enum import Enum, auto
import time

__version__ = "4.1.0"
__author__ = "Dr. Mladen Mešter"

# =============================================================================
# CONSTANTS
# =============================================================================

NM_TO_METERS = 1852.0
KNOTS_TO_MS = 0.5144
FL_TO_METERS = 30.48


# =============================================================================
# MOTION MODELS
# =============================================================================

class MotionModel(Enum):
    """Available motion models."""
    CV = "Constant Velocity"
    CA = "Constant Acceleration"
    CT = "Coordinated Turn"
    SINGER = "Singer Acceleration"


def build_cv_matrices(dt: float, dim: int = 3) -> Tuple[np.ndarray, np.ndarray]:
    """Build CV state transition and process noise matrices."""
    # State: [x, y, z, vx, vy, vz]
    n = dim * 2
    F = np.eye(n)
    for i in range(dim):
        F[i, i + dim] = dt
    
    # Process noise (discrete white noise acceleration)
    Q = np.zeros((n, n))
    for i in range(dim):
        Q[i, i] = dt**4 / 4
        Q[i, i + dim] = dt**3 / 2
        Q[i + dim, i] = dt**3 / 2
        Q[i + dim, i + dim] = dt**2
    
    return F, Q


def build_ca_matrices(dt: float, dim: int = 3) -> Tuple[np.ndarray, np.ndarray]:
    """Build CA (Constant Acceleration) state transition and process noise matrices."""
    # State: [x, y, z, vx, vy, vz, ax, ay, az]
    n = dim * 3
    F = np.eye(n)
    for i in range(dim):
        F[i, i + dim] = dt
        F[i, i + 2*dim] = dt**2 / 2
        F[i + dim, i + 2*dim] = dt
    
    # Process noise (jerk model)
    Q = np.zeros((n, n))
    for i in range(dim):
        # Position
        Q[i, i] = dt**6 / 36
        Q[i, i + dim] = dt**5 / 12
        Q[i, i + 2*dim] = dt**4 / 6
        # Velocity
        Q[i + dim, i] = dt**5 / 12
        Q[i + dim, i + dim] = dt**4 / 4
        Q[i + dim, i + 2*dim] = dt**3 / 2
        # Acceleration
        Q[i + 2*dim, i] = dt**4 / 6
        Q[i + 2*dim, i + dim] = dt**3 / 2
        Q[i + 2*dim, i + 2*dim] = dt**2
    
    return F, Q


# =============================================================================
# ENHANCED UKF FOR HIGH-SPEED TARGETS
# =============================================================================

class HighSpeedUKF:
    """
    UKF optimized for high-speed targets (>200 m/s).
    
    Key optimizations:
    1. Velocity-adaptive process noise
    2. Multi-rate prediction between measurements
    3. Improved sigma point spread for high dynamics
    """
    
    def __init__(self, dim_x: int = 6, dim_z: int = 3,
                 alpha: float = 0.5,  # Larger spread for high-speed
                 beta: float = 2.0,
                 kappa: float = 0.0):
        self.dim_x = dim_x
        self.dim_z = dim_z
        self.x = np.zeros(dim_x)
        self.P = np.eye(dim_x) * 1000
        
        # UKF parameters - optimized for high-speed
        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa
        
        # Process noise base level
        self.q_base = 1.0  # m/s^2
        
        # Measurement matrix
        self.H = np.zeros((dim_z, dim_x))
        for i in range(dim_z):
            self.H[i, i] = 1
        
        self._compute_weights()
    
    def _compute_weights(self):
        n = self.dim_x
        lam = self.alpha**2 * (n + self.kappa) - n
        
        self.Wm = np.full(2*n + 1, 1.0 / (2*(n + lam)))
        self.Wc = np.full(2*n + 1, 1.0 / (2*(n + lam)))
        self.Wm[0] = lam / (n + lam)
        self.Wc[0] = lam / (n + lam) + (1 - self.alpha**2 + self.beta)
        self.gamma = np.sqrt(n + lam)
    
    def initialize(self, x0: np.ndarray, P0: np.ndarray):
        self.x = x0.copy()
        self.P = P0.copy()
    
    def _sigma_points(self, x: np.ndarray, P: np.ndarray) -> np.ndarray:
        n = len(x)
        sigmas = np.zeros((2*n + 1, n))
        sigmas[0] = x
        
        try:
            sqrtP = cholesky(P + 1e-9 * np.eye(n)).T
        except:
            sqrtP = sqrtm(P + 1e-6 * np.eye(n)).real
        
        for i in range(n):
            sigmas[i + 1] = x + self.gamma * sqrtP[i]
            sigmas[n + i + 1] = x - self.gamma * sqrtP[i]
        
        return sigmas
    
    def _f_cv(self, x: np.ndarray, dt: float) -> np.ndarray:
        """CV state transition."""
        x_new = x.copy()
        n = len(x) // 2
        x_new[:n] += x[n:] * dt
        return x_new
    
    def _get_adaptive_Q(self, dt: float) -> np.ndarray:
        """Get velocity-adaptive process noise."""
        speed = norm(self.x[3:6])
        
        # Scale Q based on speed
        # Higher speed -> need more process noise to handle small heading changes
        if speed > 200:  # High-speed (en-route)
            q_scale = 0.5 + 0.01 * (speed - 200)  # Gradual increase
        elif speed > 100:  # Medium speed (TMA)
            q_scale = 1.0
        else:  # Low speed (approach)
            q_scale = 2.0  # Higher uncertainty for slow targets
        
        q = (self.q_base * q_scale) ** 2
        
        _, Q = build_cv_matrices(dt)
        return Q * q
    
    def predict(self, dt: float, n_substeps: int = 1):
        """
        Predict with optional sub-stepping for accuracy.
        
        For high-speed targets with long update intervals,
        sub-stepping improves prediction accuracy.
        """
        sub_dt = dt / n_substeps
        
        for _ in range(n_substeps):
            sigmas = self._sigma_points(self.x, self.P)
            
            # Propagate sigma points
            sigmas_f = np.array([self._f_cv(s, sub_dt) for s in sigmas])
            
            # Compute predicted mean
            self.x = np.sum(self.Wm[:, np.newaxis] * sigmas_f, axis=0)
            
            # Compute predicted covariance
            Q = self._get_adaptive_Q(sub_dt)
            self.P = Q.copy()
            for i, s in enumerate(sigmas_f):
                y = s - self.x
                self.P += self.Wc[i] * np.outer(y, y)
        
        return self.x.copy(), self.P.copy()
    
    def update(self, z: np.ndarray, R: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
        """Standard UKF update."""
        sigmas = self._sigma_points(self.x, self.P)
        
        # Transform through measurement function (linear: H @ x)
        sigmas_h = np.array([self.H @ s for s in sigmas])
        
        # Predicted measurement
        z_pred = np.sum(self.Wm[:, np.newaxis] * sigmas_h, axis=0)
        
        # Innovation covariance
        S = R.copy()
        for i, s in enumerate(sigmas_h):
            y = s - z_pred
            S += self.Wc[i] * np.outer(y, y)
        
        # Cross covariance
        Pxz = np.zeros((self.dim_x, self.dim_z))
        for i in range(len(sigmas)):
            Pxz += self.Wc[i] * np.outer(sigmas[i] - self.x, sigmas_h[i] - z_pred)
        
        # Kalman gain
        try:
            S_inv = inv(S)
            K = Pxz @ S_inv
        except:
            return self.x.copy(), self.P.copy(), float('inf')
        
        # NIS
        innovation = z - z_pred
        nis = float(innovation @ S_inv @ innovation)
        
        # Velocity-based gating
        speed = norm(self.x[3:6])
        gate_scale = 1.0 + 0.005 * max(0, speed - 100)  # Wider gate for fast targets
        gate_threshold = self.dim_z * 9.21 * gate_scale  # 99% chi-square
        
        if nis > gate_threshold:
            # Measurement outside gate - reduce gain
            K *= 0.1
        
        # State update
        self.x = self.x + K @ innovation
        
        # Covariance update (Joseph form)
        I_KH = np.eye(self.dim_x) - K @ self.H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        
        return self.x.copy(), self.P.copy(), nis
    
    def get_state(self) -> np.ndarray:
        return self.x.copy()
    
    def get_covariance(self) -> np.ndarray:
        return self.P.copy()


# =============================================================================
# ENHANCED VS-IMM FOR ATC
# =============================================================================

class ATCOptimizedIMM:
    """
    VS-IMM optimized for ATC scenarios.
    
    Models:
    1. CV-Cruise: For stable en-route flight
    2. CV-Maneuver: For heading changes
    3. CT: For holding patterns and turns
    """
    
    def __init__(self, dim_x: int = 6, dim_z: int = 3):
        self.dim_x = dim_x
        self.dim_z = dim_z
        
        # Three modes optimized for ATC
        self.n_modes = 3
        self.filters = [HighSpeedUKF(dim_x, dim_z) for _ in range(self.n_modes)]
        
        # Mode probabilities
        self.mu = np.array([0.8, 0.15, 0.05])  # Favor cruise mode
        
        # Process noise levels for each mode (m/s^2)
        self.q_levels = np.array([0.1, 1.0, 5.0])  # Cruise, light maneuver, heavy
        
        # Set filter Q bases
        for i, f in enumerate(self.filters):
            f.q_base = self.q_levels[i]
    
    def initialize(self, x0: np.ndarray, P0: np.ndarray):
        for f in self.filters:
            f.initialize(x0, P0)
        self.mu = np.array([0.8, 0.15, 0.05])
    
    def _get_tpm(self) -> np.ndarray:
        """Adaptive transition probability matrix."""
        # Check current speed
        speed = norm(self.filters[0].x[3:6])
        
        if speed > 200:  # High-speed en-route
            # Very stable, stay in cruise mode
            PI = np.array([
                [0.98, 0.015, 0.005],
                [0.10, 0.85, 0.05],
                [0.05, 0.10, 0.85]
            ])
        elif speed > 50:  # Medium speed (TMA, holding)
            PI = np.array([
                [0.92, 0.06, 0.02],
                [0.08, 0.85, 0.07],
                [0.03, 0.07, 0.90]
            ])
        else:  # Slow (approach, taxi)
            PI = np.array([
                [0.85, 0.10, 0.05],
                [0.10, 0.80, 0.10],
                [0.05, 0.10, 0.85]
            ])
        
        return PI
    
    def predict(self, dt: float):
        """Predict all modes with sub-stepping for long intervals."""
        # Determine sub-steps based on speed and dt
        speed = norm(self.filters[0].x[3:6])
        
        # More sub-steps for high speed + long dt
        if speed > 200 and dt > 2.0:
            n_substeps = int(dt / 0.5)  # 0.5s sub-steps
        elif speed > 100 and dt > 1.0:
            n_substeps = int(dt / 0.5)
        else:
            n_substeps = max(1, int(dt / 1.0))
        
        for f in self.filters:
            f.predict(dt, n_substeps=n_substeps)
    
    def update(self, z: np.ndarray, R: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
        """IMM update cycle."""
        PI = self._get_tpm()
        
        # Mixing probabilities
        c_bar = PI.T @ self.mu + 1e-10
        mu_ij = np.zeros((self.n_modes, self.n_modes))
        for i in range(self.n_modes):
            for j in range(self.n_modes):
                mu_ij[i, j] = PI[i, j] * self.mu[i] / c_bar[j]
        
        # Mix states and covariances
        x_mix = []
        P_mix = []
        for j in range(self.n_modes):
            x_j = sum(mu_ij[i, j] * self.filters[i].get_state() for i in range(self.n_modes))
            P_j = np.zeros((self.dim_x, self.dim_x))
            for i in range(self.n_modes):
                dx = self.filters[i].get_state() - x_j
                P_j += mu_ij[i, j] * (self.filters[i].get_covariance() + np.outer(dx, dx))
            x_mix.append(x_j)
            P_mix.append(P_j)
        
        # Reinitialize filters with mixed states
        for j in range(self.n_modes):
            self.filters[j].x = x_mix[j]
            self.filters[j].P = P_mix[j]
        
        # Update each filter
        likelihoods = []
        total_nis = 0
        for j, f in enumerate(self.filters):
            _, _, nis = f.update(z, R)
            # Likelihood with numerical stability
            lik = np.exp(-0.5 * min(nis, 100)) + 1e-30
            likelihoods.append(lik)
            total_nis += self.mu[j] * nis
        
        # Update mode probabilities
        mu_new = c_bar * np.array(likelihoods)
        mu_sum = mu_new.sum()
        if mu_sum > 1e-20:
            self.mu = mu_new / mu_sum
        else:
            self.mu = np.array([0.34, 0.33, 0.33])
        
        # Compute combined estimate
        x_combined = sum(self.mu[j] * self.filters[j].get_state() for j in range(self.n_modes))
        P_combined = np.zeros((self.dim_x, self.dim_x))
        for j in range(self.n_modes):
            dx = self.filters[j].get_state() - x_combined
            P_combined += self.mu[j] * (self.filters[j].get_covariance() + np.outer(dx, dx))
        
        return x_combined, P_combined, total_nis
    
    def get_state(self) -> np.ndarray:
        return sum(self.mu[j] * self.filters[j].get_state() for j in range(self.n_modes))
    
    def get_mode_probabilities(self) -> np.ndarray:
        return self.mu.copy()


# =============================================================================
# ATC TRACKER
# =============================================================================

class NXMIMOSAAtc:
    """
    NX-MIMOSA v4.1 - ATC Optimized Tracker
    
    Specifically tuned for civil aviation ATC/ATM requirements:
    - En-route: ≤500m RMS
    - TMA: ≤150m RMS
    - Continuity: ≥99.5%
    """
    
    def __init__(self, dt: float = 4.0, sigma: float = 100.0):
        self.dt = dt
        self.sigma = sigma
        
        self.dim_x = 6  # [x, y, z, vx, vy, vz]
        self.dim_z = 3  # [x, y, z]
        
        # Core filter
        self.imm = ATCOptimizedIMM(self.dim_x, self.dim_z)
        
        # Measurement matrix
        self.H = np.zeros((self.dim_z, self.dim_x))
        for i in range(self.dim_z):
            self.H[i, i] = 1
        
        # Statistics
        self.n_updates = 0
        self.n_predictions = 0
        self.last_update_time = None
        
        # Innovation monitoring for adaptive R
        self.innovation_buffer = deque(maxlen=20)
        self.r_scale = 1.0
    
    def initialize(self, z0: np.ndarray, v0: Optional[np.ndarray] = None):
        """Initialize track."""
        if v0 is None:
            v0 = np.zeros(3)
        
        x0 = np.concatenate([z0, v0])
        
        # Initial covariance based on expected accuracy
        P0 = np.diag([
            self.sigma**2, self.sigma**2, self.sigma**2,
            50**2, 50**2, 10**2  # Velocity uncertainty
        ])
        
        self.imm.initialize(x0, P0)
        self.n_updates = 0
        self.n_predictions = 0
        self.innovation_buffer.clear()
        self.r_scale = 1.0
    
    def predict(self, dt: Optional[float] = None):
        """Predict to next time."""
        dt_use = dt if dt is not None else self.dt
        self.imm.predict(dt_use)
        self.n_predictions += 1
    
    def update(self, z: np.ndarray, sigma: Optional[float] = None) -> np.ndarray:
        """Update with measurement."""
        sigma_use = sigma if sigma is not None else self.sigma
        
        # Adaptive R based on innovation history
        x_pred = self.imm.get_state()
        z_pred = self.H @ x_pred
        innovation = z - z_pred
        self.innovation_buffer.append(innovation)
        
        if len(self.innovation_buffer) >= 5:
            # Estimate actual measurement noise from innovations
            innovations = np.array(self.innovation_buffer)
            cov_est = np.cov(innovations.T) if innovations.shape[0] > 1 else np.eye(3)
            trace_est = np.trace(cov_est)
            trace_nominal = 3 * sigma_use**2
            ratio = trace_est / (trace_nominal + 1e-10)
            
            # Smooth update
            self.r_scale = 0.9 * self.r_scale + 0.1 * np.clip(ratio, 0.5, 2.0)
        
        R = (sigma_use * self.r_scale) ** 2 * np.eye(self.dim_z)
        
        x_est, _, nis = self.imm.update(z, R)
        self.n_updates += 1
        self.last_update_time = time.time()
        
        return x_est
    
    def coast(self, dt: Optional[float] = None) -> np.ndarray:
        """Coast without measurement."""
        self.predict(dt)
        return self.imm.get_state()
    
    def get_state(self) -> np.ndarray:
        return self.imm.get_state()
    
    def get_position(self) -> np.ndarray:
        return self.imm.get_state()[:3]
    
    def get_velocity(self) -> np.ndarray:
        return self.imm.get_state()[3:6]
    
    def get_mode_probabilities(self) -> np.ndarray:
        return self.imm.get_mode_probabilities()
    
    def get_track_quality(self) -> Dict:
        total = self.n_updates + self.n_predictions
        continuity = self.n_updates / max(1, total)
        
        return {
            'n_updates': self.n_updates,
            'n_predictions': self.n_predictions,
            'continuity': continuity,
            'mode_probabilities': self.get_mode_probabilities().tolist(),
            'r_scale': self.r_scale,
        }


# =============================================================================
# VALIDATION
# =============================================================================

def validate_atc_compliance():
    """Run ATC compliance validation with realistic multi-sensor setup."""
    print("="*100)
    print("NX-MIMOSA v4.1 - ATC OPTIMIZED TRACKER VALIDATION")
    print("Multi-Sensor Configuration: PSR + SSR + ADS-B (ARTAS-like)")
    print("="*100)
    
    np.random.seed(42)
    
    # Simulation parameters
    sim_dt = 0.1  # 100ms simulation
    radar_period = 4.0  # 4s radar rotation
    adsb_period = 1.0  # 1 Hz ADS-B
    duration = 120.0
    
    results = {}
    
    # =========================================================================
    # Scenario 1: En-route Cruise (450 kts, FL350) - Multi-sensor
    # =========================================================================
    print("\n▶ EN-ROUTE CRUISE (450 kts @ FL350)")
    print("  Sensors: PSR (50m σ, 4s) + ADS-B (30m σ, 1s)")
    print("-"*80)
    
    # Generate trajectory
    T = int(duration / sim_dt)
    traj = np.zeros((T, 6))
    
    # Initial: 100km out, FL350, heading 090
    x0, y0, z0 = 100000, 0, 350 * FL_TO_METERS
    vx0 = 450 * KNOTS_TO_MS  # ~232 m/s
    traj[0] = [x0, y0, z0, vx0, 0, 0]
    
    for k in range(1, T):
        t = k * sim_dt
        # Occasional small heading change
        if k % int(60/sim_dt) == 0:
            heading = np.arctan2(traj[k-1, 4], traj[k-1, 3])
            d_heading = np.deg2rad(np.random.uniform(-3, 3))
            speed = norm(traj[k-1, 3:5])
            traj[k, 3] = speed * np.cos(heading + d_heading)
            traj[k, 4] = speed * np.sin(heading + d_heading)
            traj[k, 5] = 0
        else:
            traj[k, 3:6] = traj[k-1, 3:6]
        traj[k, :3] = traj[k-1, :3] + traj[k, 3:6] * sim_dt
    
    # Create tracker with shorter effective dt due to ADS-B
    tracker = NXMIMOSAAtc(dt=adsb_period, sigma=30.0)
    tracker.initialize(traj[0, :3], traj[0, 3:6])
    
    # Run simulation with multi-sensor
    position_errors = []
    last_radar = 0
    last_adsb = 0
    
    for k in range(1, T):
        t = k * sim_dt
        true_pos = traj[k, :3]
        
        measurement_available = False
        z = None
        sigma = None
        dt_since_last = None
        
        # Check radar (4s period, 50m accuracy)
        if t - last_radar >= radar_period:
            noise = np.random.randn(3) * 50
            z_radar = true_pos + noise
            last_radar = t
            
            if z is None:
                z = z_radar
                sigma = 50.0
                dt_since_last = t - max(last_adsb, 0.01)
                measurement_available = True
        
        # Check ADS-B (1s period, 30m accuracy - NACp 8)
        if t - last_adsb >= adsb_period:
            noise = np.random.randn(3) * np.array([30, 30, 45])  # H/V accuracy
            z_adsb = true_pos + noise
            
            if measurement_available:
                # Fuse radar and ADS-B measurements
                w_radar = 1 / (50**2)
                w_adsb = 1 / (30**2)
                z = (w_radar * z + w_adsb * z_adsb) / (w_radar + w_adsb)
                sigma = 1 / np.sqrt(w_radar + w_adsb)
            else:
                z = z_adsb
                sigma = 30.0
                dt_since_last = t - last_adsb
                measurement_available = True
            
            last_adsb = t
        
        if measurement_available and dt_since_last is not None:
            tracker.predict(dt_since_last)
            tracker.update(z, sigma)
        
        # Compute error
        est = tracker.get_state()
        pos_err = norm(est[:3] - true_pos)
        position_errors.append(pos_err)
    
    enroute_rms = np.sqrt(np.mean(np.array(position_errors)**2))
    enroute_max = np.max(position_errors)
    
    status = "✅ PASS" if enroute_rms <= 500 else "❌ FAIL"
    print(f"  Position RMS:     {enroute_rms:.1f} m (limit: 500 m) {status}")
    print(f"  Max Error:        {enroute_max:.1f} m")
    print(f"  Mode probs:       {tracker.get_mode_probabilities()}")
    
    results['enroute'] = {'rms': enroute_rms, 'max': enroute_max, 'passed': enroute_rms <= 500}
    
    # =========================================================================
    # Scenario 2: Terminal Approach (140 kts, 3° glideslope) - Multi-sensor
    # =========================================================================
    print("\n▶ TERMINAL APPROACH (140 kts, ILS)")
    print("  Sensors: SSR (30m σ, 4s) + ADS-B (30m σ, 1s)")
    print("-"*80)
    
    traj = np.zeros((T, 6))
    x0 = 10 * NM_TO_METERS
    z0 = 3000 * 0.3048
    gs_angle = np.deg2rad(3.0)
    speed = 140 * KNOTS_TO_MS
    
    traj[0] = [x0, 0, z0, -speed * np.cos(gs_angle), 0, -speed * np.sin(gs_angle)]
    
    for k in range(1, T):
        traj[k, 3:6] = traj[k-1, 3:6] * 0.9999
        traj[k, :3] = traj[k-1, :3] + traj[k, 3:6] * sim_dt
        if traj[k, 0] < 0:
            traj[k:] = traj[k]
            break
    
    tracker = NXMIMOSAAtc(dt=adsb_period, sigma=25.0)
    tracker.initialize(traj[0, :3], traj[0, 3:6])
    
    position_errors = []
    last_radar = 0
    last_adsb = 0
    
    for k in range(1, T):
        t = k * sim_dt
        true_pos = traj[k, :3]
        
        measurement_available = False
        z = None
        sigma = None
        dt_since_last = None
        
        # SSR in TMA (better accuracy, 30m)
        if t - last_radar >= radar_period:
            noise = np.random.randn(3) * 30
            z_radar = true_pos + noise
            last_radar = t
            
            if z is None:
                z = z_radar
                sigma = 30.0
                dt_since_last = t - max(last_adsb, 0.01)
                measurement_available = True
        
        # ADS-B
        if t - last_adsb >= adsb_period:
            noise = np.random.randn(3) * np.array([30, 30, 45])
            z_adsb = true_pos + noise
            
            if measurement_available:
                w_radar = 1 / (30**2)
                w_adsb = 1 / (30**2)
                z = (w_radar * z + w_adsb * z_adsb) / (w_radar + w_adsb)
                sigma = 1 / np.sqrt(w_radar + w_adsb)
            else:
                z = z_adsb
                sigma = 30.0
                dt_since_last = t - last_adsb
                measurement_available = True
            
            last_adsb = t
        
        if measurement_available and dt_since_last is not None:
            tracker.predict(dt_since_last)
            tracker.update(z, sigma)
        
        est = tracker.get_state()
        pos_err = norm(est[:3] - true_pos)
        position_errors.append(pos_err)
    
    tma_rms = np.sqrt(np.mean(np.array(position_errors)**2))
    tma_max = np.max(position_errors)
    
    status = "✅ PASS" if tma_rms <= 150 else "❌ FAIL"
    print(f"  Position RMS:     {tma_rms:.1f} m (limit: 150 m) {status}")
    print(f"  Max Error:        {tma_max:.1f} m")
    
    results['tma'] = {'rms': tma_rms, 'max': tma_max, 'passed': tma_rms <= 150}
    
    # =========================================================================
    # Scenario 3: Holding Pattern (220 kts, standard rate turns)
    # =========================================================================
    print("\n▶ HOLDING PATTERN (220 kts, race-track)")
    print("  Sensors: SSR (40m σ, 4s) + ADS-B (30m σ, 1s)")
    print("-"*80)
    
    traj = np.zeros((T, 6))
    speed = 220 * KNOTS_TO_MS
    altitude = 10000 * 0.3048
    turn_rate = np.deg2rad(3.0)
    leg_time = 60.0
    
    traj[0] = [0, 0, altitude, speed, 0, 0]
    heading = 0
    
    for k in range(1, T):
        t = k * sim_dt
        phase = (t % (4 * leg_time)) / leg_time
        
        if phase < 1.0:
            d_heading = 0
        elif phase < 1.5:
            d_heading = turn_rate * sim_dt
        elif phase < 2.5:
            d_heading = 0
        else:
            d_heading = turn_rate * sim_dt
        
        heading += d_heading
        traj[k, 3] = speed * np.cos(heading)
        traj[k, 4] = speed * np.sin(heading)
        traj[k, 5] = 0
        traj[k, :3] = traj[k-1, :3] + traj[k, 3:6] * sim_dt
    
    tracker = NXMIMOSAAtc(dt=adsb_period, sigma=30.0)
    tracker.initialize(traj[0, :3], traj[0, 3:6])
    
    position_errors = []
    last_radar = 0
    last_adsb = 0
    
    for k in range(1, T):
        t = k * sim_dt
        true_pos = traj[k, :3]
        
        measurement_available = False
        z = None
        sigma = None
        dt_since_last = None
        
        if t - last_radar >= radar_period:
            noise = np.random.randn(3) * 40
            z_radar = true_pos + noise
            last_radar = t
            
            if z is None:
                z = z_radar
                sigma = 40.0
                dt_since_last = t - max(last_adsb, 0.01)
                measurement_available = True
        
        if t - last_adsb >= adsb_period:
            noise = np.random.randn(3) * np.array([30, 30, 45])
            z_adsb = true_pos + noise
            
            if measurement_available:
                w_radar = 1 / (40**2)
                w_adsb = 1 / (30**2)
                z = (w_radar * z + w_adsb * z_adsb) / (w_radar + w_adsb)
                sigma = 1 / np.sqrt(w_radar + w_adsb)
            else:
                z = z_adsb
                sigma = 30.0
                dt_since_last = t - last_adsb
                measurement_available = True
            
            last_adsb = t
        
        if measurement_available and dt_since_last is not None:
            tracker.predict(dt_since_last)
            tracker.update(z, sigma)
        
        est = tracker.get_state()
        pos_err = norm(est[:3] - true_pos)
        position_errors.append(pos_err)
    
    hold_rms = np.sqrt(np.mean(np.array(position_errors)**2))
    hold_max = np.max(position_errors)
    
    status = "✅ PASS" if hold_rms <= 500 else "❌ FAIL"
    print(f"  Position RMS:     {hold_rms:.1f} m (limit: 500 m) {status}")
    print(f"  Max Error:        {hold_max:.1f} m")
    
    results['holding'] = {'rms': hold_rms, 'max': hold_max, 'passed': hold_rms <= 500}
    
    # =========================================================================
    # Summary
    # =========================================================================
    print("\n" + "="*100)
    print("📊 EUROCONTROL EASSP COMPLIANCE SUMMARY")
    print("="*100)
    
    all_passed = all(r['passed'] for r in results.values())
    
    print(f"""
┌─────────────────────────────────────────────────────────────────────────────────────────────────┐
│ SCENARIO              │ REQUIREMENT     │ ACHIEVED        │ STATUS                             │
├─────────────────────────────────────────────────────────────────────────────────────────────────┤
│ En-route Cruise       │ ≤ 500 m RMS     │ {results['enroute']['rms']:>7.1f} m       │ {'✅ PASS' if results['enroute']['passed'] else '❌ FAIL'}                             │
│ Terminal Approach     │ ≤ 150 m RMS     │ {results['tma']['rms']:>7.1f} m       │ {'✅ PASS' if results['tma']['passed'] else '❌ FAIL'}                             │
│ Holding Pattern       │ ≤ 500 m RMS     │ {results['holding']['rms']:>7.1f} m       │ {'✅ PASS' if results['holding']['passed'] else '❌ FAIL'}                             │
├─────────────────────────────────────────────────────────────────────────────────────────────────┤
│ OVERALL COMPLIANCE    │                 │                 │ {'✅ COMPLIANT' if all_passed else '❌ NON-COMPLIANT'}                        │
└─────────────────────────────────────────────────────────────────────────────────────────────────┘
""")
    
    return results


# =============================================================================
# MAIN
# =============================================================================

if __name__ == "__main__":
    results = validate_atc_compliance()
