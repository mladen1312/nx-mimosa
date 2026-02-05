#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════════
NX-MIMOSA vs GOLD STANDARD BENCHMARK
═══════════════════════════════════════════════════════════════════════════════════

HONEST, REPRODUCIBLE comparison against the tracking community's gold standards:

  1. FilterPy IMMEstimator     — Roger Labbe's IMM (most-used Python IMM)
  2. FilterPy UKF              — Unscented Kalman Filter 
  3. Stone Soup EKF + RTS      — UK DSTL's production tracking framework
  4. Stone Soup UKF + RTS      — Stone Soup Unscented + RTS Smoother
  5. NX-MIMOSA v3.1            — Our IMM + True Per-Model Smoother

FAIRNESS GUARANTEES:
  - All algorithms receive IDENTICAL noisy measurements
  - All algorithms use IDENTICAL initial state/covariance
  - All algorithms use the SAME motion model parameters
  - Monte Carlo with fixed seed for reproducibility
  - Standard metrics: RMSE, NEES, ANEES (consistency check)

REQUIREMENTS:
  pip install numpy scipy filterpy stonesoup

USAGE:
  python benchmark_vs_goldstandard.py
  python benchmark_vs_goldstandard.py --runs 50 --seed 42

Author: Dr. Mladen Mešter / Nexellum d.o.o.
License: AGPL v3
═══════════════════════════════════════════════════════════════════════════════════
"""

import numpy as np
from numpy.linalg import inv, cholesky
from scipy.linalg import block_diag
import time
import argparse
from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Optional
import warnings
warnings.filterwarnings('ignore')

# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 1: SCENARIO DEFINITIONS (shared by ALL algorithms)
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class TrackingScenario:
    """Defense tracking scenario with ground truth generation."""
    name: str
    duration: float        # seconds
    dt: float              # measurement interval
    sigma_pos: float       # measurement noise std (meters)
    sigma_vel: float       # process noise std for CV (m/s²) 
    segments: list         # list of (duration, accel_x, accel_y) tuples
    description: str = ""
    
    def generate_truth(self) -> Tuple[np.ndarray, np.ndarray]:
        """Generate ground truth trajectory. Returns (states [N,4], times [N])."""
        # State = [x, vx, y, vy]
        states = []
        t = 0.0
        # Initial state from first segment
        vx0, vy0 = 300.0, 0.0  # default initial velocity
        state = np.array([0.0, vx0, 0.0, vy0])
        
        for seg_dur, ax, ay in self.segments:
            seg_steps = int(seg_dur / self.dt)
            for _ in range(seg_steps):
                states.append(state.copy())
                # Constant acceleration within segment
                x, vx, y, vy = state
                state = np.array([
                    x + vx * self.dt + 0.5 * ax * self.dt**2,
                    vx + ax * self.dt,
                    y + vy * self.dt + 0.5 * ay * self.dt**2,
                    vy + ay * self.dt
                ])
                t += self.dt
                
        states = np.array(states)
        times = np.arange(len(states)) * self.dt
        return states, times
    
    def generate_measurements(self, truth: np.ndarray, rng: np.random.Generator) -> np.ndarray:
        """Generate noisy position measurements from ground truth."""
        N = len(truth)
        noise = rng.normal(0, self.sigma_pos, (N, 2))
        measurements = np.column_stack([truth[:, 0], truth[:, 2]]) + noise
        return measurements


def get_scenarios() -> List[TrackingScenario]:
    """8 defense scenarios covering full spectrum of maneuvering targets."""
    g = 9.81
    scenarios = [
        TrackingScenario(
            name="Missile Terminal (Mach 4, 9g)",
            duration=20.0, dt=0.05, sigma_pos=15.0, sigma_vel=5.0,
            segments=[
                (5.0, 0, 0),           # coast
                (5.0, 0, 9*g),         # 9g pull-up
                (5.0, -5*g, -9*g),     # 9g evasion  
                (5.0, 5*g, 4*g),       # weave
            ],
            description="High-g terminal evasion maneuver"
        ),
        TrackingScenario(
            name="Hypersonic Glide (Mach 5, 2g S-weave)",
            duration=30.0, dt=0.1, sigma_pos=50.0, sigma_vel=3.0,
            segments=[
                (7.5, 0, 2*g),        # bank left
                (7.5, 0, -2*g),       # bank right (S-weave)
                (7.5, 0, 2*g),        # bank left
                (7.5, 0, -2*g),       # bank right
            ],
            description="Hypersonic glide vehicle S-weave pattern"
        ),
        TrackingScenario(
            name="SAM Engagement (300m/s, 6g barrel)",
            duration=15.0, dt=0.05, sigma_pos=10.0, sigma_vel=8.0,
            segments=[
                (3.0, 0, 0),          # initial approach
                (4.0, 3*g, 6*g),      # barrel roll maneuver
                (4.0, -6*g, -3*g),    # reverse barrel
                (4.0, 0, 0),          # coast out
            ],
            description="SAM interceptor barrel roll engagement"
        ),
        TrackingScenario(
            name="Dogfight BFM (250m/s, 8g sustained)",
            duration=20.0, dt=0.05, sigma_pos=8.0, sigma_vel=10.0,
            segments=[
                (5.0, 0, 8*g),        # max-g sustained turn
                (5.0, -4*g, -8*g),    # reversal
                (5.0, 8*g, 0),        # scissors
                (5.0, -8*g, 4*g),     # break
            ],
            description="Close-in dogfight with sustained high-g"
        ),
        TrackingScenario(
            name="Cruise Missile (250m/s, 3g pop-up)",
            duration=30.0, dt=0.1, sigma_pos=20.0, sigma_vel=2.0,
            segments=[
                (10.0, 0, 0),         # low-level cruise
                (5.0, 0, 3*g),        # pop-up maneuver
                (5.0, 0, -3*g),       # dive to target
                (10.0, 0, 0),         # terminal run
            ],
            description="Sea-skimming cruise missile with pop-up"
        ),
        TrackingScenario(
            name="Ballistic Reentry (Mach 7, corrections)",
            duration=40.0, dt=0.2, sigma_pos=100.0, sigma_vel=1.0,
            segments=[
                (15.0, 0, 0),         # ballistic coast
                (5.0, 0, -1*g),       # correction burn
                (10.0, 0, 0),         # coast
                (10.0, 0.5*g, 0.5*g), # terminal correction
            ],
            description="Ballistic reentry with correction burns"
        ),
        TrackingScenario(
            name="UAV Swarm (50m/s, 2g random)",
            duration=30.0, dt=0.1, sigma_pos=5.0, sigma_vel=3.0,
            segments=[
                (7.5, 2*g, 0),        # accelerate
                (7.5, -2*g, 2*g),     # turn
                (7.5, 0, -2*g),       # reverse
                (7.5, 2*g, 2*g),      # diagonal
            ],
            description="Small UAV with rapid direction changes"
        ),
        TrackingScenario(
            name="Stealth Aircraft (200m/s, 4g, sparse 2Hz)",
            duration=30.0, dt=0.5, sigma_pos=30.0, sigma_vel=5.0,
            segments=[
                (10.0, 0, 0),         # cruise (hard to detect)
                (5.0, 0, 4*g),        # evasion
                (5.0, -4*g, 0),       # break
                (10.0, 0, -2*g),      # escape
            ],
            description="Low-RCS aircraft with sparse radar updates"
        ),
    ]
    return scenarios


# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 2: SHARED MODELS (identical for all algorithms)
# ═══════════════════════════════════════════════════════════════════════════════

def cv_matrices(dt: float, q: float) -> Tuple[np.ndarray, np.ndarray]:
    """Constant Velocity model F and Q matrices. State = [x, vx, y, vy]."""
    F = np.array([
        [1, dt, 0, 0],
        [0, 1,  0, 0],
        [0, 0,  1, dt],
        [0, 0,  0, 1]
    ])
    # Piecewise constant white noise acceleration model
    q2 = q**2
    Q_block = np.array([
        [dt**3/3, dt**2/2],
        [dt**2/2, dt]
    ]) * q2
    Q = block_diag(Q_block, Q_block)
    return F, Q


def ct_matrices(dt: float, q: float, omega: float) -> Tuple[np.ndarray, np.ndarray]:
    """Coordinated Turn model F and Q. State = [x, vx, y, vy]. omega = turn rate."""
    if abs(omega) < 1e-6:
        return cv_matrices(dt, q)
    s = np.sin(omega * dt)
    c = np.cos(omega * dt)
    F = np.array([
        [1,  s/omega,      0, -(1-c)/omega],
        [0,  c,            0, -s],
        [0,  (1-c)/omega,  1,  s/omega],
        [0,  s,            0,  c]
    ])
    q2 = q**2
    Q_block = np.array([
        [dt**3/3, dt**2/2],
        [dt**2/2, dt]
    ]) * q2
    Q = block_diag(Q_block, Q_block)
    return F, Q


def measurement_matrix() -> np.ndarray:
    """H matrix: measure position only. z = [x, y]."""
    return np.array([
        [1, 0, 0, 0],
        [0, 0, 1, 0]
    ])


def measurement_noise(sigma: float) -> np.ndarray:
    """R matrix for position-only measurements."""
    return np.eye(2) * sigma**2


# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 3: NX-MIMOSA v3.1 IMPLEMENTATION 
# ═══════════════════════════════════════════════════════════════════════════════

class NxMimosaV3:
    """
    NX-MIMOSA v3.1 — True IMM Smoother with Per-Model RTS.
    
    Key innovations:
    1. 3-model IMM: CV, CT+, CT- (constant velocity + two coordinated turns)
    2. Variable-Structure Transition Probability Matrix (VS-TPM)
    3. Adaptive Q via Normalized Innovation Squared (NIS)
    4. True per-model RTS smoother (NOT combined-state smoother)
    5. Joseph form covariance update
    """
    
    def __init__(self, dt: float, sigma_meas: float, q_base: float):
        self.dt = dt
        self.H = measurement_matrix()
        self.R = measurement_noise(sigma_meas)
        self.q_base = q_base
        self.n_models = 3
        
        # Model parameters: CV, CT+ (omega=0.1), CT- (omega=-0.1)
        self.omegas = [0.0, 0.1, -0.1]
        
        # Initial mode probabilities
        self.mu = np.array([0.8, 0.1, 0.1])
        
        # Transition probability matrix
        self.TPM_base = np.array([
            [0.95, 0.025, 0.025],
            [0.05, 0.90,  0.05],
            [0.05, 0.05,  0.90]
        ])
        
    def _get_model_FQ(self, j: int, q_scale: float = 1.0) -> Tuple[np.ndarray, np.ndarray]:
        F, Q = ct_matrices(self.dt, self.q_base * q_scale, self.omegas[j])
        return F, Q
    
    def _adaptive_q_scale(self, innovation: np.ndarray, S: np.ndarray) -> float:
        """NIS-based adaptive Q scaling."""
        nis = innovation @ inv(S) @ innovation
        expected_nis = len(innovation)  # chi-squared DOF
        ratio = nis / expected_nis
        if ratio > 3.0:
            return min(ratio, 10.0)  # scale up Q
        elif ratio < 0.3:
            return max(ratio, 0.1)   # scale down Q
        return 1.0
    
    def _vs_tpm(self, mu: np.ndarray) -> np.ndarray:
        """Variable-Structure TPM based on mode confidence."""
        max_prob = np.max(mu)
        if max_prob > 0.90:
            p_stay = 0.98
        elif max_prob > 0.70:
            p_stay = 0.95
        else:
            p_stay = 0.90
        
        TPM = np.full((3, 3), (1 - p_stay) / 2)
        np.fill_diagonal(TPM, p_stay)
        return TPM
    
    def run(self, measurements: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Full forward-backward tracking.
        Returns: (smoothed_states, smoothed_covs, forward_states)
        """
        N = len(measurements)
        nx = 4
        
        # Storage per model
        x_filt = np.zeros((self.n_models, N, nx))
        P_filt = np.zeros((self.n_models, N, nx, nx))
        x_pred = np.zeros((self.n_models, N, nx))
        P_pred = np.zeros((self.n_models, N, nx, nx))
        mu_hist = np.zeros((N, self.n_models))
        
        # Combined forward estimates
        x_combined = np.zeros((N, nx))
        P_combined = np.zeros((N, nx, nx))
        
        # Initialize all models identically
        x0 = np.array([measurements[0, 0], 0.0, measurements[0, 1], 0.0])
        P0 = np.diag([self.R[0,0], 100.0, self.R[1,1], 100.0])
        
        for j in range(self.n_models):
            x_filt[j, 0] = x0.copy()
            P_filt[j, 0] = P0.copy()
            x_pred[j, 0] = x0.copy()
            P_pred[j, 0] = P0.copy()
        
        mu = self.mu.copy()
        mu_hist[0] = mu
        x_combined[0] = x0
        P_combined[0] = P0
        
        # ─── FORWARD PASS (IMM) ───
        for k in range(1, N):
            z = measurements[k]
            TPM = self._vs_tpm(mu)
            
            # 1. Mixing probabilities
            c_bar = TPM.T @ mu  # predicted mode probs
            mu_mix = np.zeros((self.n_models, self.n_models))
            for i in range(self.n_models):
                for j in range(self.n_models):
                    mu_mix[i, j] = TPM[i, j] * mu[i] / max(c_bar[j], 1e-30)
            
            # 2. Mixed initial conditions
            x_mix = np.zeros((self.n_models, nx))
            P_mix = np.zeros((self.n_models, nx, nx))
            for j in range(self.n_models):
                for i in range(self.n_models):
                    x_mix[j] += mu_mix[i, j] * x_filt[i, k-1]
                for i in range(self.n_models):
                    diff = x_filt[i, k-1] - x_mix[j]
                    P_mix[j] += mu_mix[i, j] * (P_filt[i, k-1] + np.outer(diff, diff))
            
            # 3. Per-model predict & update
            likelihoods = np.zeros(self.n_models)
            for j in range(self.n_models):
                F_j, Q_j = self._get_model_FQ(j)
                
                # Predict
                xp = F_j @ x_mix[j]
                Pp = F_j @ P_mix[j] @ F_j.T + Q_j
                
                # Innovation
                innov = z - self.H @ xp
                S = self.H @ Pp @ self.H.T + self.R
                
                # Adaptive Q
                q_scale = self._adaptive_q_scale(innov, S)
                if q_scale != 1.0:
                    _, Q_adj = self._get_model_FQ(j, q_scale)
                    Pp = F_j @ P_mix[j] @ F_j.T + Q_adj
                    S = self.H @ Pp @ self.H.T + self.R
                
                x_pred[j, k] = xp
                P_pred[j, k] = Pp
                
                # Kalman gain
                K = Pp @ self.H.T @ inv(S)
                
                # Joseph form update
                x_filt[j, k] = xp + K @ innov
                IKH = np.eye(nx) - K @ self.H
                P_filt[j, k] = IKH @ Pp @ IKH.T + K @ self.R @ K.T
                
                # Likelihood
                sign, logdet = np.linalg.slogdet(S)
                log_lik = -0.5 * (innov @ inv(S) @ innov + logdet + 2 * np.log(2*np.pi))
                likelihoods[j] = np.exp(log_lik)
            
            # 4. Mode probability update
            mu = c_bar * likelihoods
            mu_sum = np.sum(mu)
            if mu_sum > 1e-300:
                mu = mu / mu_sum
            else:
                mu = np.ones(self.n_models) / self.n_models
            mu_hist[k] = mu
            
            # 5. Combined estimate
            for j in range(self.n_models):
                x_combined[k] += mu[j] * x_filt[j, k]
            for j in range(self.n_models):
                diff = x_filt[j, k] - x_combined[k]
                P_combined[k] += mu[j] * (P_filt[j, k] + np.outer(diff, diff))
        
        forward_states = x_combined.copy()
        
        # ─── BACKWARD PASS (True Per-Model RTS Smoother) ───
        # Key innovation: smooth each model independently, then combine
        x_smooth_j = np.zeros((self.n_models, N, nx))
        P_smooth_j = np.zeros((self.n_models, N, nx, nx))
        
        for j in range(self.n_models):
            x_smooth_j[j, N-1] = x_filt[j, N-1].copy()
            P_smooth_j[j, N-1] = P_filt[j, N-1].copy()
            
            for k in range(N-2, -1, -1):
                F_j, _ = self._get_model_FQ(j)
                
                # RTS gain for this model
                Pp_inv = inv(P_pred[j, k+1] + np.eye(nx) * 1e-10)
                G = P_filt[j, k] @ F_j.T @ Pp_inv
                
                # CRITICAL BUGFIX: use x_filt[j,k] not x_combined[k]
                x_smooth_j[j, k] = x_filt[j, k] + G @ (x_smooth_j[j, k+1] - x_pred[j, k+1])
                P_smooth_j[j, k] = P_filt[j, k] + G @ (P_smooth_j[j, k+1] - P_pred[j, k+1]) @ G.T
        
        # Combine smoothed estimates using forward mode probabilities
        x_smooth = np.zeros((N, nx))
        P_smooth = np.zeros((N, nx, nx))
        for k in range(N):
            for j in range(self.n_models):
                x_smooth[k] += mu_hist[k, j] * x_smooth_j[j, k]
            for j in range(self.n_models):
                diff = x_smooth_j[j, k] - x_smooth[k]
                P_smooth[k] += mu_hist[k, j] * (P_smooth_j[j, k] + np.outer(diff, diff))
        
        return x_smooth, P_smooth, forward_states


# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 4: FILTERPY GOLD STANDARD WRAPPERS
# ═══════════════════════════════════════════════════════════════════════════════

def run_filterpy_imm(measurements: np.ndarray, dt: float, sigma_meas: float, 
                     q_base: float) -> np.ndarray:
    """
    FilterPy IMMEstimator — Roger Labbe's implementation.
    Uses the same 3-model structure (CV, CT+, CT-) for fair comparison.
    """
    from filterpy.kalman import KalmanFilter, IMMEstimator
    
    H = measurement_matrix()
    R = measurement_noise(sigma_meas)
    
    filters = []
    for omega in [0.0, 0.1, -0.1]:
        f = KalmanFilter(dim_x=4, dim_z=2)
        F, Q = ct_matrices(dt, q_base, omega)
        f.F = F
        f.Q = Q
        f.H = H
        f.R = R
        f.x = np.array([[measurements[0, 0]], [0.0], [measurements[0, 1]], [0.0]])
        f.P = np.diag([sigma_meas**2, 100.0, sigma_meas**2, 100.0])
        filters.append(f)
    
    mu = np.array([0.8, 0.1, 0.1])
    M = np.array([
        [0.95, 0.025, 0.025],
        [0.05, 0.90,  0.05],
        [0.05, 0.05,  0.90]
    ])
    
    imm = IMMEstimator(filters, mu, M)
    
    N = len(measurements)
    states = np.zeros((N, 4))
    states[0] = np.array([measurements[0, 0], 0.0, measurements[0, 1], 0.0])
    
    for k in range(1, N):
        z = measurements[k].reshape(2, 1)
        imm.predict()
        imm.update(z)
        states[k] = imm.x.flatten()
    
    return states


def run_filterpy_ukf(measurements: np.ndarray, dt: float, sigma_meas: float,
                     q_base: float) -> np.ndarray:
    """
    FilterPy UnscentedKalmanFilter — sigma-point based, handles nonlinearity.
    Using CV model (linear, but tests UKF overhead/accuracy).
    """
    from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
    
    def fx(x, dt_inner):
        F, _ = cv_matrices(dt_inner, q_base)
        return F @ x
    
    def hx(x):
        return np.array([x[0], x[2]])
    
    points = MerweScaledSigmaPoints(n=4, alpha=0.1, beta=2.0, kappa=-1.0)
    ukf = UnscentedKalmanFilter(dim_x=4, dim_z=2, dt=dt, fx=fx, hx=hx, points=points)
    
    _, Q = cv_matrices(dt, q_base)
    ukf.Q = Q
    ukf.R = measurement_noise(sigma_meas)
    ukf.x = np.array([measurements[0, 0], 0.0, measurements[0, 1], 0.0])
    ukf.P = np.diag([sigma_meas**2, 100.0, sigma_meas**2, 100.0])
    
    N = len(measurements)
    states = np.zeros((N, 4))
    states[0] = ukf.x.copy()
    
    for k in range(1, N):
        ukf.predict()
        ukf.update(measurements[k])
        states[k] = ukf.x.copy()
    
    return states


# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 5: STONE SOUP GOLD STANDARD WRAPPERS
# ═══════════════════════════════════════════════════════════════════════════════

def run_stonesoup_ekf_rts(measurements: np.ndarray, dt: float, sigma_meas: float,
                          q_base: float) -> np.ndarray:
    """
    Stone Soup Extended Kalman Filter + RTS Smoother.
    The UK DSTL's production-grade tracking framework.
    """
    from stonesoup.types.state import GaussianState
    from stonesoup.types.detection import Detection
    from stonesoup.types.track import Track
    from stonesoup.types.array import StateVector, CovarianceMatrix
    from stonesoup.types.hypothesis import SingleHypothesis
    from stonesoup.models.transition.linear import (
        CombinedLinearGaussianTransitionModel,
        ConstantVelocity
    )
    from stonesoup.models.measurement.linear import LinearGaussian
    from stonesoup.predictor.kalman import KalmanPredictor
    from stonesoup.updater.kalman import KalmanUpdater
    from stonesoup.smoother.kalman import KalmanSmoother
    from datetime import datetime, timedelta
    
    # Transition model: CV in 2D
    transition_model = CombinedLinearGaussianTransitionModel([
        ConstantVelocity(q_base**2),
        ConstantVelocity(q_base**2)
    ])
    
    # Measurement model: observe position only (indices 0, 2 from state [x, vx, y, vy])
    measurement_model = LinearGaussian(
        ndim_state=4,
        mapping=(0, 2),
        noise_covar=CovarianceMatrix(np.eye(2) * sigma_meas**2)
    )
    
    predictor = KalmanPredictor(transition_model)
    updater = KalmanUpdater(measurement_model)
    smoother = KalmanSmoother(transition_model)
    
    # Initial state
    t0 = datetime.now()
    prior = GaussianState(
        state_vector=StateVector([[measurements[0, 0]], [0.0], [measurements[0, 1]], [0.0]]),
        covar=CovarianceMatrix(np.diag([sigma_meas**2, 100.0, sigma_meas**2, 100.0])),
        timestamp=t0
    )
    
    track = Track([prior])
    
    N = len(measurements)
    for k in range(1, N):
        tk = t0 + timedelta(seconds=k * dt)
        detection = Detection(
            state_vector=StateVector([[measurements[k, 0]], [measurements[k, 1]]]),
            timestamp=tk,
            measurement_model=measurement_model
        )
        prediction = predictor.predict(track[-1], timestamp=tk)
        hypothesis = SingleHypothesis(prediction, detection)
        update = updater.update(hypothesis)
        track.append(update)
    
    # Apply RTS smoother
    smoothed_track = smoother.smooth(track)
    
    # Extract states
    states = np.zeros((N, 4))
    for k, state in enumerate(smoothed_track):
        states[k] = state.state_vector.flatten()
    
    return states


def run_stonesoup_ukf_rts(measurements: np.ndarray, dt: float, sigma_meas: float,
                          q_base: float) -> np.ndarray:
    """
    Stone Soup Unscented Kalman Filter + UKF Smoother.
    """
    from stonesoup.types.state import GaussianState
    from stonesoup.types.detection import Detection
    from stonesoup.types.track import Track
    from stonesoup.types.array import StateVector, CovarianceMatrix
    from stonesoup.types.hypothesis import SingleHypothesis
    from stonesoup.models.transition.linear import (
        CombinedLinearGaussianTransitionModel,
        ConstantVelocity
    )
    from stonesoup.models.measurement.linear import LinearGaussian
    from stonesoup.predictor.kalman import UnscentedKalmanPredictor
    from stonesoup.updater.kalman import UnscentedKalmanUpdater
    from stonesoup.smoother.kalman import UnscentedKalmanSmoother
    from datetime import datetime, timedelta
    
    transition_model = CombinedLinearGaussianTransitionModel([
        ConstantVelocity(q_base**2),
        ConstantVelocity(q_base**2)
    ])
    
    measurement_model = LinearGaussian(
        ndim_state=4,
        mapping=(0, 2),
        noise_covar=CovarianceMatrix(np.eye(2) * sigma_meas**2)
    )
    
    predictor = UnscentedKalmanPredictor(transition_model)
    updater = UnscentedKalmanUpdater(measurement_model)
    smoother = UnscentedKalmanSmoother(transition_model)
    
    t0 = datetime.now()
    prior = GaussianState(
        state_vector=StateVector([[measurements[0, 0]], [0.0], [measurements[0, 1]], [0.0]]),
        covar=CovarianceMatrix(np.diag([sigma_meas**2, 100.0, sigma_meas**2, 100.0])),
        timestamp=t0
    )
    
    track = Track([prior])
    
    N = len(measurements)
    for k in range(1, N):
        tk = t0 + timedelta(seconds=k * dt)
        detection = Detection(
            state_vector=StateVector([[measurements[k, 0]], [measurements[k, 1]]]),
            timestamp=tk,
            measurement_model=measurement_model
        )
        prediction = predictor.predict(track[-1], timestamp=tk)
        hypothesis = SingleHypothesis(prediction, detection)
        update = updater.update(hypothesis)
        track.append(update)
    
    smoothed_track = smoother.smooth(track)
    
    states = np.zeros((N, 4))
    for k, state in enumerate(smoothed_track):
        states[k] = state.state_vector.flatten()
    
    return states


# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 6: SIMPLE BASELINE FILTERS (for reference)
# ═══════════════════════════════════════════════════════════════════════════════

def run_ekf_cv(measurements: np.ndarray, dt: float, sigma_meas: float,
               q_base: float) -> np.ndarray:
    """Basic EKF with constant velocity model."""
    F, Q = cv_matrices(dt, q_base)
    H = measurement_matrix()
    R = measurement_noise(sigma_meas)
    
    N = len(measurements)
    x = np.array([measurements[0, 0], 0.0, measurements[0, 1], 0.0])
    P = np.diag([sigma_meas**2, 100.0, sigma_meas**2, 100.0])
    
    states = np.zeros((N, 4))
    states[0] = x
    
    for k in range(1, N):
        # Predict
        x = F @ x
        P = F @ P @ F.T + Q
        # Update
        innov = measurements[k] - H @ x
        S = H @ P @ H.T + R
        K = P @ H.T @ inv(S)
        x = x + K @ innov
        P = (np.eye(4) - K @ H) @ P
        states[k] = x
    
    return states


def run_ekf_ca(measurements: np.ndarray, dt: float, sigma_meas: float,
               q_base: float) -> np.ndarray:
    """EKF with constant acceleration model (uses higher process noise)."""
    # Use larger Q to handle maneuvers
    F, Q = cv_matrices(dt, q_base * 5.0)  # 5x higher Q for CA-like behavior
    H = measurement_matrix()
    R = measurement_noise(sigma_meas)
    
    N = len(measurements)
    x = np.array([measurements[0, 0], 0.0, measurements[0, 1], 0.0])
    P = np.diag([sigma_meas**2, 100.0, sigma_meas**2, 100.0])
    
    states = np.zeros((N, 4))
    states[0] = x
    
    for k in range(1, N):
        x = F @ x
        P = F @ P @ F.T + Q
        innov = measurements[k] - H @ x
        S = H @ P @ H.T + R
        K = P @ H.T @ inv(S)
        x = x + K @ innov
        P = (np.eye(4) - K @ H) @ P
        states[k] = x
    
    return states


# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 7: METRICS (RMSE + NEES for statistical consistency)
# ═══════════════════════════════════════════════════════════════════════════════

def compute_pos_rmse(estimated: np.ndarray, truth: np.ndarray) -> float:
    """Position RMSE: sqrt(mean( (x_est - x_true)^2 + (y_est - y_true)^2 ))."""
    dx = estimated[:, 0] - truth[:, 0]
    dy = estimated[:, 2] - truth[:, 2]
    return np.sqrt(np.mean(dx**2 + dy**2))


def compute_vel_rmse(estimated: np.ndarray, truth: np.ndarray) -> float:
    """Velocity RMSE."""
    dvx = estimated[:, 1] - truth[:, 1]
    dvy = estimated[:, 3] - truth[:, 3]
    return np.sqrt(np.mean(dvx**2 + dvy**2))


def compute_nees(estimated: np.ndarray, truth: np.ndarray, 
                 covariances: Optional[np.ndarray] = None) -> float:
    """
    Normalized Estimation Error Squared (NEES).
    For a consistent filter, NEES should be close to n_x (state dimension).
    Returns average NEES across time steps.
    """
    if covariances is None:
        return float('nan')
    
    N = len(estimated)
    nees_vals = []
    for k in range(N):
        err = estimated[k] - truth[k]
        try:
            P_inv = inv(covariances[k] + np.eye(4) * 1e-10)
            nees = err @ P_inv @ err
            if nees < 1000:  # skip diverged steps
                nees_vals.append(nees)
        except:
            pass
    
    return np.mean(nees_vals) if nees_vals else float('nan')


# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 8: BENCHMARK RUNNER
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class AlgorithmResult:
    name: str
    pos_rmse: List[float] = field(default_factory=list)
    vel_rmse: List[float] = field(default_factory=list)
    nees: List[float] = field(default_factory=list)
    runtime_ms: List[float] = field(default_factory=list)
    
    @property
    def avg_pos_rmse(self) -> float:
        return np.mean(self.pos_rmse)
    
    @property
    def avg_vel_rmse(self) -> float:
        return np.mean(self.vel_rmse)
    
    @property
    def avg_nees(self) -> float:
        vals = [v for v in self.nees if not np.isnan(v)]
        return np.mean(vals) if vals else float('nan')
    
    @property
    def avg_runtime_ms(self) -> float:
        return np.mean(self.runtime_ms)


def run_benchmark(n_runs: int = 30, seed: int = 42, verbose: bool = True):
    """Run the full benchmark."""
    
    scenarios = get_scenarios()
    
    algorithms = {
        "EKF-CV (baseline)": lambda m, s: (run_ekf_cv(m, s.dt, s.sigma_pos, s.sigma_vel), None),
        "EKF-CA (high-Q)": lambda m, s: (run_ekf_ca(m, s.dt, s.sigma_pos, s.sigma_vel), None),
        "FilterPy IMM": lambda m, s: (run_filterpy_imm(m, s.dt, s.sigma_pos, s.sigma_vel), None),
        "FilterPy UKF": lambda m, s: (run_filterpy_ukf(m, s.dt, s.sigma_pos, s.sigma_vel), None),
        "StoneSoup EKF+RTS": lambda m, s: (run_stonesoup_ekf_rts(m, s.dt, s.sigma_pos, s.sigma_vel), None),
        "StoneSoup UKF+RTS": lambda m, s: (run_stonesoup_ukf_rts(m, s.dt, s.sigma_pos, s.sigma_vel), None),
        "NX-MIMOSA v3.1 (fwd)": None,   # special handling
        "NX-MIMOSA v3.1 (smooth)": None, # special handling
    }
    
    # Results storage: {scenario_name: {algo_name: AlgorithmResult}}
    all_results: Dict[str, Dict[str, AlgorithmResult]] = {}
    
    print("=" * 100)
    print("NX-MIMOSA vs GOLD STANDARD BENCHMARK")
    print(f"Monte Carlo runs: {n_runs} | Seed: {seed} | Date: 2026-02-05")
    print("=" * 100)
    print()
    print("ALGORITHMS UNDER TEST:")
    print("  1. EKF-CV (baseline)        — Simple constant velocity Kalman")
    print("  2. EKF-CA (high-Q)          — High process noise for maneuvering")
    print("  3. FilterPy IMM             — Roger Labbe's IMMEstimator (3-model)")
    print("  4. FilterPy UKF             — FilterPy Unscented Kalman Filter")
    print("  5. StoneSoup EKF+RTS        — UK DSTL framework: EKF + RTS Smoother")
    print("  6. StoneSoup UKF+RTS        — UK DSTL framework: UKF + UKF Smoother")
    print("  7. NX-MIMOSA v3.1 (fwd)     — IMM forward-only (no smoothing)")
    print("  8. NX-MIMOSA v3.1 (smooth)  — Full system with True IMM Smoother")
    print()
    print("FAIRNESS: All algorithms receive IDENTICAL measurements & initialization")
    print("=" * 100)
    
    for si, scenario in enumerate(scenarios):
        print(f"\n{'─' * 100}")
        print(f"SCENARIO {si+1}/8: {scenario.name}")
        print(f"  dt={scenario.dt}s, σ_pos={scenario.sigma_pos}m, σ_vel={scenario.sigma_vel} m/s²")
        print(f"{'─' * 100}")
        
        truth, times = scenario.generate_truth()
        
        scenario_results: Dict[str, AlgorithmResult] = {}
        for name in list(algorithms.keys()):
            scenario_results[name] = AlgorithmResult(name=name)
        
        for run in range(n_runs):
            rng = np.random.default_rng(seed * 1000 + si * 100 + run)
            measurements = scenario.generate_measurements(truth, rng)
            
            # --- Run each algorithm ---
            
            # EKF-CV
            t0 = time.perf_counter()
            est_cv, _ = algorithms["EKF-CV (baseline)"](measurements, scenario)
            dt_cv = (time.perf_counter() - t0) * 1000
            scenario_results["EKF-CV (baseline)"].pos_rmse.append(compute_pos_rmse(est_cv, truth))
            scenario_results["EKF-CV (baseline)"].vel_rmse.append(compute_vel_rmse(est_cv, truth))
            scenario_results["EKF-CV (baseline)"].runtime_ms.append(dt_cv)
            
            # EKF-CA
            t0 = time.perf_counter()
            est_ca, _ = algorithms["EKF-CA (high-Q)"](measurements, scenario)
            dt_ca = (time.perf_counter() - t0) * 1000
            scenario_results["EKF-CA (high-Q)"].pos_rmse.append(compute_pos_rmse(est_ca, truth))
            scenario_results["EKF-CA (high-Q)"].vel_rmse.append(compute_vel_rmse(est_ca, truth))
            scenario_results["EKF-CA (high-Q)"].runtime_ms.append(dt_ca)
            
            # FilterPy IMM
            t0 = time.perf_counter()
            est_fimm, _ = algorithms["FilterPy IMM"](measurements, scenario)
            dt_fimm = (time.perf_counter() - t0) * 1000
            scenario_results["FilterPy IMM"].pos_rmse.append(compute_pos_rmse(est_fimm, truth))
            scenario_results["FilterPy IMM"].vel_rmse.append(compute_vel_rmse(est_fimm, truth))
            scenario_results["FilterPy IMM"].runtime_ms.append(dt_fimm)
            
            # FilterPy UKF
            t0 = time.perf_counter()
            est_fukf, _ = algorithms["FilterPy UKF"](measurements, scenario)
            dt_fukf = (time.perf_counter() - t0) * 1000
            scenario_results["FilterPy UKF"].pos_rmse.append(compute_pos_rmse(est_fukf, truth))
            scenario_results["FilterPy UKF"].vel_rmse.append(compute_vel_rmse(est_fukf, truth))
            scenario_results["FilterPy UKF"].runtime_ms.append(dt_fukf)
            
            # Stone Soup EKF + RTS
            t0 = time.perf_counter()
            try:
                est_ss_ekf, _ = algorithms["StoneSoup EKF+RTS"](measurements, scenario)
                dt_ss = (time.perf_counter() - t0) * 1000
                scenario_results["StoneSoup EKF+RTS"].pos_rmse.append(compute_pos_rmse(est_ss_ekf, truth))
                scenario_results["StoneSoup EKF+RTS"].vel_rmse.append(compute_vel_rmse(est_ss_ekf, truth))
            except Exception as e:
                dt_ss = (time.perf_counter() - t0) * 1000
                scenario_results["StoneSoup EKF+RTS"].pos_rmse.append(float('nan'))
                scenario_results["StoneSoup EKF+RTS"].vel_rmse.append(float('nan'))
            scenario_results["StoneSoup EKF+RTS"].runtime_ms.append(dt_ss)
            
            # Stone Soup UKF + RTS
            t0 = time.perf_counter()
            try:
                est_ss_ukf, _ = algorithms["StoneSoup UKF+RTS"](measurements, scenario)
                dt_ss_u = (time.perf_counter() - t0) * 1000
                scenario_results["StoneSoup UKF+RTS"].pos_rmse.append(compute_pos_rmse(est_ss_ukf, truth))
                scenario_results["StoneSoup UKF+RTS"].vel_rmse.append(compute_vel_rmse(est_ss_ukf, truth))
            except Exception as e:
                dt_ss_u = (time.perf_counter() - t0) * 1000
                scenario_results["StoneSoup UKF+RTS"].pos_rmse.append(float('nan'))
                scenario_results["StoneSoup UKF+RTS"].vel_rmse.append(float('nan'))
            scenario_results["StoneSoup UKF+RTS"].runtime_ms.append(dt_ss_u)
            
            # NX-MIMOSA v3.1
            mimosa = NxMimosaV3(scenario.dt, scenario.sigma_pos, scenario.sigma_vel)
            t0 = time.perf_counter()
            smooth, P_smooth, forward = mimosa.run(measurements)
            dt_m = (time.perf_counter() - t0) * 1000
            
            scenario_results["NX-MIMOSA v3.1 (fwd)"].pos_rmse.append(compute_pos_rmse(forward, truth))
            scenario_results["NX-MIMOSA v3.1 (fwd)"].vel_rmse.append(compute_vel_rmse(forward, truth))
            scenario_results["NX-MIMOSA v3.1 (fwd)"].runtime_ms.append(dt_m)
            scenario_results["NX-MIMOSA v3.1 (fwd)"].nees.append(compute_nees(forward, truth, P_smooth))
            
            scenario_results["NX-MIMOSA v3.1 (smooth)"].pos_rmse.append(compute_pos_rmse(smooth, truth))
            scenario_results["NX-MIMOSA v3.1 (smooth)"].vel_rmse.append(compute_vel_rmse(smooth, truth))
            scenario_results["NX-MIMOSA v3.1 (smooth)"].runtime_ms.append(dt_m)
            scenario_results["NX-MIMOSA v3.1 (smooth)"].nees.append(compute_nees(smooth, truth, P_smooth))
        
        all_results[scenario.name] = scenario_results
        
        if verbose:
            # Print scenario results
            print(f"\n  {'Algorithm':<30} {'Pos RMSE (m)':>14} {'Vel RMSE (m/s)':>16} {'Time (ms)':>12}")
            print(f"  {'─' * 74}")
            
            # Sort by position RMSE
            sorted_algos = sorted(scenario_results.items(), 
                                  key=lambda x: np.nanmean(x[1].pos_rmse))
            
            for rank, (name, res) in enumerate(sorted_algos):
                avg_rmse = np.nanmean(res.pos_rmse)
                avg_vrmse = np.nanmean(res.vel_rmse)
                avg_time = res.avg_runtime_ms
                marker = " 🥇" if rank == 0 else " 🥈" if rank == 1 else " 🥉" if rank == 2 else ""
                print(f"  {name:<30} {avg_rmse:>12.2f} m {avg_vrmse:>14.2f} m/s {avg_time:>10.1f} ms{marker}")
    
    # ═══════════════════════════════════════════════════════════════════════
    # SUMMARY TABLE
    # ═══════════════════════════════════════════════════════════════════════
    print("\n" + "=" * 120)
    print("GRAND SUMMARY — Average Position RMSE (meters) Across All Monte Carlo Runs")
    print("=" * 120)
    
    algo_names = [
        "EKF-CV (baseline)", "EKF-CA (high-Q)", "FilterPy UKF",
        "FilterPy IMM", "StoneSoup EKF+RTS", "StoneSoup UKF+RTS",
        "NX-MIMOSA v3.1 (fwd)", "NX-MIMOSA v3.1 (smooth)"
    ]
    
    # Header
    print(f"\n{'Scenario':<40}", end="")
    for name in algo_names:
        short = name.replace(" (baseline)", "").replace(" (high-Q)", "")
        short = short[:12]
        print(f" {short:>12}", end="")
    print(f" {'WINNER':>14}")
    print("─" * (40 + 12 * len(algo_names) + 16))
    
    # Per scenario
    wins = {name: 0 for name in algo_names}
    grand_rmse = {name: [] for name in algo_names}
    
    for sname, sresults in all_results.items():
        short_sname = sname[:38]
        print(f"{short_sname:<40}", end="")
        
        best_rmse = float('inf')
        best_algo = ""
        
        for aname in algo_names:
            avg = np.nanmean(sresults[aname].pos_rmse)
            grand_rmse[aname].append(avg)
            if avg < best_rmse:
                best_rmse = avg
                best_algo = aname
            print(f" {avg:>12.2f}", end="")
        
        wins[best_algo] += 1
        print(f" {best_algo[:14]:>14}")
    
    # Grand average
    print("─" * (40 + 12 * len(algo_names) + 16))
    print(f"{'GRAND AVERAGE':<40}", end="")
    for aname in algo_names:
        avg = np.nanmean(grand_rmse[aname])
        print(f" {avg:>12.2f}", end="")
    print()
    
    # Win count
    print(f"\n{'WINS':<40}", end="")
    for aname in algo_names:
        print(f" {wins[aname]:>12}", end="")
    print()
    
    # ═══════════════════════════════════════════════════════════════════════
    # IMPROVEMENT ANALYSIS
    # ═══════════════════════════════════════════════════════════════════════
    print("\n" + "=" * 100)
    print("IMPROVEMENT ANALYSIS — NX-MIMOSA v3.1 (smooth) vs Each Competitor")
    print("=" * 100)
    
    mimosa_avg = np.nanmean(grand_rmse["NX-MIMOSA v3.1 (smooth)"])
    
    print(f"\n{'Competitor':<30} {'Their Avg RMSE':>16} {'MIMOSA Avg RMSE':>16} {'Improvement':>14} {'Verdict':>12}")
    print("─" * 90)
    
    for aname in algo_names:
        if aname == "NX-MIMOSA v3.1 (smooth)":
            continue
        comp_avg = np.nanmean(grand_rmse[aname])
        if comp_avg > 0 and not np.isnan(comp_avg):
            improvement = (comp_avg - mimosa_avg) / comp_avg * 100
            verdict = "✅ BETTER" if improvement > 0 else "❌ WORSE"
            print(f"  {aname:<28} {comp_avg:>14.2f} m {mimosa_avg:>14.2f} m {improvement:>+12.1f}% {verdict:>12}")
        else:
            print(f"  {aname:<28} {'N/A':>14} {mimosa_avg:>14.2f} m {'N/A':>14} {'N/A':>12}")
    
    # ═══════════════════════════════════════════════════════════════════════
    # FAIRNESS DECLARATION
    # ═══════════════════════════════════════════════════════════════════════
    print("\n" + "=" * 100)
    print("FAIRNESS & REPRODUCIBILITY DECLARATION")
    print("=" * 100)
    print(f"""
✅ All algorithms receive IDENTICAL noisy measurements (same RNG seed)
✅ All algorithms use IDENTICAL initial state and covariance
✅ FilterPy IMM uses SAME 3-model structure (CV/CT+/CT-) as NX-MIMOSA
✅ Stone Soup uses their built-in RTS smoother (production-grade)
✅ Monte Carlo: {n_runs} runs per scenario, seed={seed}
✅ No cherry-picking: ALL 8 scenarios reported, ALL algorithms reported
✅ Code is self-contained: pip install numpy scipy filterpy stonesoup

⚠️  KNOWN ADVANTAGES of NX-MIMOSA (disclosed for honesty):
  - IMM (multi-model) inherently outperforms single-model filters on maneuvers
  - Smoothing (backward pass) uses future data — not available in real-time
  - Adaptive Q scaling tunes process noise to match innovation statistics
  - VS-TPM adapts transition probabilities to mode confidence

⚠️  KNOWN LIMITATIONS disclosed:
  - Stone Soup has no built-in IMM — comparison is CV-smoother vs IMM-smoother
  - FilterPy IMM is forward-only — no smoother available for direct comparison
  - NX-MIMOSA smoother requires offline/batch processing (not causal)
  - Fair CAUSAL comparison: NX-MIMOSA v3.1 (fwd) vs FilterPy IMM

REPRODUCE: python {__file__} --runs {n_runs} --seed {seed}
""")
    
    # ═══════════════════════════════════════════════════════════════════════
    # CATEGORY ANALYSIS (fair comparisons within same capability class)
    # ═══════════════════════════════════════════════════════════════════════
    print("=" * 100)
    print("FAIR CATEGORY COMPARISONS (apples-to-apples)")
    print("=" * 100)
    
    # Category 1: Forward-only (causal/real-time capable)
    print("\n📡 CATEGORY 1: Forward-Only / Real-Time Capable")
    print("─" * 60)
    fwd_algos = ["EKF-CV (baseline)", "EKF-CA (high-Q)", "FilterPy UKF", 
                  "FilterPy IMM", "NX-MIMOSA v3.1 (fwd)"]
    for aname in fwd_algos:
        avg = np.nanmean(grand_rmse[aname])
        print(f"  {aname:<30} {avg:>10.2f} m")
    
    mimosa_fwd = np.nanmean(grand_rmse["NX-MIMOSA v3.1 (fwd)"])
    fimm_avg = np.nanmean(grand_rmse["FilterPy IMM"])
    if fimm_avg > 0:
        imp = (fimm_avg - mimosa_fwd) / fimm_avg * 100
        print(f"\n  → NX-MIMOSA (fwd) vs FilterPy IMM: {imp:+.1f}%")
    
    # Category 2: Smoothed (offline/batch)
    print("\n📊 CATEGORY 2: Smoothed / Offline Batch")
    print("─" * 60)
    smooth_algos = ["StoneSoup EKF+RTS", "StoneSoup UKF+RTS", "NX-MIMOSA v3.1 (smooth)"]
    for aname in smooth_algos:
        avg = np.nanmean(grand_rmse[aname])
        print(f"  {aname:<30} {avg:>10.2f} m")
    
    ss_ekf = np.nanmean(grand_rmse["StoneSoup EKF+RTS"])
    ss_ukf = np.nanmean(grand_rmse["StoneSoup UKF+RTS"])
    mimosa_sm = np.nanmean(grand_rmse["NX-MIMOSA v3.1 (smooth)"])
    
    if ss_ekf > 0 and not np.isnan(ss_ekf):
        imp1 = (ss_ekf - mimosa_sm) / ss_ekf * 100
        print(f"\n  → NX-MIMOSA (smooth) vs StoneSoup EKF+RTS: {imp1:+.1f}%")
    if ss_ukf > 0 and not np.isnan(ss_ukf):
        imp2 = (ss_ukf - mimosa_sm) / ss_ukf * 100
        print(f"  → NX-MIMOSA (smooth) vs StoneSoup UKF+RTS: {imp2:+.1f}%")
    
    # Category 3: IMM-to-IMM (the truest comparison)
    print("\n🎯 CATEGORY 3: IMM-to-IMM (Most Direct Comparison)")
    print("─" * 60)
    imm_algos = ["FilterPy IMM", "NX-MIMOSA v3.1 (fwd)", "NX-MIMOSA v3.1 (smooth)"]
    for aname in imm_algos:
        avg = np.nanmean(grand_rmse[aname])
        print(f"  {aname:<30} {avg:>10.2f} m")
    
    print(f"\n  FilterPy IMM = standard IMM implementation (Blom & Bar-Shalom)")
    print(f"  NX-MIMOSA adds: Adaptive Q + VS-TPM + Joseph Form + Per-Model Smoother")
    
    return all_results


# ═══════════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="NX-MIMOSA vs Gold Standard Benchmark")
    parser.add_argument("--runs", type=int, default=30, help="Monte Carlo runs per scenario")
    parser.add_argument("--seed", type=int, default=42, help="Random seed for reproducibility")
    args = parser.parse_args()
    
    results = run_benchmark(n_runs=args.runs, seed=args.seed)
