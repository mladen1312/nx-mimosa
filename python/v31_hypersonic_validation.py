"""
═══════════════════════════════════════════════════════════════════════════════
NX-MIMOSA v3.1 — HYPERSONIC WEAVE VALIDATION
═══════════════════════════════════════════════════════════════════════════════
Scenario: Mach 5+ hypersonic glide vehicle with S-weave evasion
Validates: Bugfix (F @ x_mixed) + Joseph stability + high-speed tracking

Target Profile:
- Speed: 1700 m/s (Mach 5 at altitude)
- Maneuver: S-weave pattern, ±2g lateral (pull-up/push-over limits at hypersonic)
- Altitude: 30 km (boost-glide trajectory)
- Range: 50 km initial

Author: Dr. Mladen Mešter / Nexellum d.o.o.
[REQ-V31-HYPERSONIC-01] Validate on M5+ weave scenario
═══════════════════════════════════════════════════════════════════════════════
"""

import numpy as np
from numpy.linalg import inv, det, norm
from dataclasses import dataclass
from typing import List, Tuple, Optional
import warnings
warnings.filterwarnings('ignore')


@dataclass
class HypersonicScenarioConfig:
    """Hypersonic glide vehicle scenario configuration."""
    # Target parameters
    speed_mps: float = 1700.0       # Mach 5 at 30km altitude
    weave_g: float = 2.0            # ±2g S-weave (hypersonic structural limit)
    weave_period_s: float = 8.0     # Period of S-weave maneuver
    initial_range_m: float = 50000  # 50 km
    altitude_m: float = 30000       # 30 km
    
    # Tracker parameters  
    dt: float = 0.02                # 50 Hz update rate
    smoother_lag_s: float = 0.5     # 500 ms fixed-lag
    sigma_meas_m: float = 10.0      # Higher noise at long range
    
    # IMM parameters
    omega_turn: float = 0.05        # Lower turn rate for hypersonic (2g at 1700 m/s)
    q_cv: float = 0.5               # Process noise CV
    q_ct: float = 5.0               # Process noise CT
    p_stay: float = 0.95            # Mode persistence


class HypersonicTrajectoryGenerator:
    """Generate hypersonic S-weave trajectory."""
    
    def __init__(self, config: HypersonicScenarioConfig):
        self.config = config
        
    def generate(self, T_steps: int) -> np.ndarray:
        """
        Generate S-weave trajectory.
        
        S-weave: Sinusoidal lateral acceleration
        a_lateral(t) = a_max * sin(2π * t / T_period)
        
        Returns: [T, 4] array of [x, y, vx, vy]
        """
        c = self.config
        dt = c.dt
        
        # Initialize
        x_true = np.zeros((T_steps, 4))
        
        # Initial state: approaching from +x direction
        x_true[0] = [c.initial_range_m, 0, -c.speed_mps, 0]
        
        # Integrate trajectory with S-weave
        g = 9.81
        a_max = c.weave_g * g  # Max lateral acceleration
        omega_weave = 2 * np.pi / c.weave_period_s
        
        for k in range(1, T_steps):
            t = k * dt
            
            # S-weave lateral acceleration (perpendicular to flight path)
            a_lateral = a_max * np.sin(omega_weave * t)
            
            # Flight direction (approximately -x for approaching target)
            vx, vy = x_true[k-1, 2], x_true[k-1, 3]
            v_mag = np.sqrt(vx**2 + vy**2)
            
            # Unit vectors: along track and lateral
            if v_mag > 0:
                u_along = np.array([vx, vy]) / v_mag
                u_lateral = np.array([-u_along[1], u_along[0]])  # Perpendicular
            else:
                u_along = np.array([-1, 0])
                u_lateral = np.array([0, 1])
            
            # Acceleration components
            ax = a_lateral * u_lateral[0]
            ay = a_lateral * u_lateral[1]
            
            # Integrate
            x_true[k, 0] = x_true[k-1, 0] + x_true[k-1, 2] * dt + 0.5 * ax * dt**2
            x_true[k, 1] = x_true[k-1, 1] + x_true[k-1, 3] * dt + 0.5 * ay * dt**2
            x_true[k, 2] = x_true[k-1, 2] + ax * dt
            x_true[k, 3] = x_true[k-1, 3] + ay * dt
            
            # Speed maintenance (hypersonic maintains ~constant speed)
            v_current = np.sqrt(x_true[k, 2]**2 + x_true[k, 3]**2)
            if v_current > 0:
                scale = c.speed_mps / v_current
                x_true[k, 2] *= scale
                x_true[k, 3] *= scale
        
        return x_true


class IMMTrackerV31:
    """
    NX-MIMOSA v3.1 IMM Tracker with BUGFIX.
    
    BUGFIX: x_pred in backward pass uses F @ x_mixed (not F @ x_filt)
    This eliminates singular matrix issues during turn transitions.
    """
    
    def __init__(self, config: HypersonicScenarioConfig):
        self.config = config
        self.dt = config.dt
        self.omega = config.omega_turn
        self.n_models = 3  # CV, CT+, CT-
        
        self._build_models()
        
        # State
        self.x = None
        self.P = None
        self.mu = None
        
        # History for smoother
        self.history = []
        self.lag_samples = int(config.smoother_lag_s / config.dt)
        
    def _build_models(self):
        dt = self.dt
        omega = self.omega
        
        # CV Model
        F_cv = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # CT+ Model (positive turn)
        if omega != 0:
            s, c = np.sin(omega*dt), np.cos(omega*dt)
            F_ct_pos = np.array([
                [1, 0, s/omega, -(1-c)/omega],
                [0, 1, (1-c)/omega, s/omega],
                [0, 0, c, -s],
                [0, 0, s, c]
            ])
            s_n, c_n = np.sin(-omega*dt), np.cos(-omega*dt)
            F_ct_neg = np.array([
                [1, 0, s_n/(-omega), -(1-c_n)/(-omega)],
                [0, 1, (1-c_n)/(-omega), s_n/(-omega)],
                [0, 0, c_n, -s_n],
                [0, 0, s_n, c_n]
            ])
        else:
            F_ct_pos = F_cv.copy()
            F_ct_neg = F_cv.copy()
        
        self.F = [F_cv, F_ct_pos, F_ct_neg]
        
        # Q matrices
        c = self.config
        Q_base = np.diag([dt**4/4, dt**4/4, dt**2, dt**2])
        self.Q = [c.q_cv * Q_base, c.q_ct * Q_base, c.q_ct * Q_base]
        
        # H, R
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        self.R = c.sigma_meas_m**2 * np.eye(2)
        
        # TPM
        p = c.p_stay
        p_sw = (1 - p) / 2
        self.PI = np.array([[p, p_sw, p_sw], [p_sw, p, p_sw], [p_sw, p_sw, p]])
    
    def initialize(self, z0: np.ndarray, v_init: np.ndarray = None):
        """Initialize tracker with first measurement."""
        if v_init is None:
            v_init = np.array([-self.config.speed_mps, 0])  # Assume approaching
        
        x0 = np.array([z0[0], z0[1], v_init[0], v_init[1]])
        P0 = np.diag([100, 100, 500, 500])**2  # High velocity uncertainty
        
        self.x = [x0.copy() for _ in range(self.n_models)]
        self.P = [P0.copy() for _ in range(self.n_models)]
        self.mu = np.array([0.8, 0.1, 0.1])
        self.history = []
    
    def update(self, z: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        IMM update with BUGFIX storage for smoother.
        
        BUGFIX: Store x_mixed for backward pass prediction.
        """
        n = self.n_models
        
        # Mixing
        c_bar = self.PI.T @ self.mu
        mu_ij = np.zeros((n, n))
        for i in range(n):
            for j in range(n):
                mu_ij[i, j] = self.PI[i, j] * self.mu[i] / (c_bar[j] + 1e-10)
        
        # Mixed states (BUGFIX: store these for smoother)
        x_mixed = []
        P_mixed = []
        for j in range(n):
            x_j = sum(mu_ij[i, j] * self.x[i] for i in range(n))
            x_mixed.append(x_j)
            P_j = sum(mu_ij[i, j] * (self.P[i] + np.outer(self.x[i] - x_j, self.x[i] - x_j)) 
                     for i in range(n))
            P_mixed.append(P_j)
        
        # Predict & Update per model
        x_filt = []
        P_filt = []
        x_pred = []
        P_pred = []
        likelihoods = []
        
        for j in range(n):
            # Predict
            # BUGFIX: Use x_mixed[j] for prediction, store for smoother
            xp = self.F[j] @ x_mixed[j]
            Pp = self.F[j] @ P_mixed[j] @ self.F[j].T + self.Q[j]
            
            x_pred.append(xp)
            P_pred.append(Pp)
            
            # Update
            y = z - self.H @ xp
            S = self.H @ Pp @ self.H.T + self.R
            
            # Regularization for numerical stability
            S = S + 1e-6 * np.eye(2)
            
            try:
                K = Pp @ self.H.T @ inv(S)
            except:
                K = np.zeros((4, 2))
            
            x_f = xp + K @ y
            
            # Joseph form for stability
            I_KH = np.eye(4) - K @ self.H
            P_f = I_KH @ Pp @ I_KH.T + K @ self.R @ K.T
            
            x_filt.append(x_f)
            P_filt.append(P_f)
            
            # Likelihood
            try:
                lik = np.exp(-0.5 * y @ inv(S) @ y) / np.sqrt((2*np.pi)**2 * det(S))
            except:
                lik = 1e-10
            likelihoods.append(max(lik, 1e-10))
        
        # Update mode probabilities
        mu_unnorm = c_bar * np.array(likelihoods)
        self.mu = mu_unnorm / (mu_unnorm.sum() + 1e-10)
        
        # Store for smoother
        # BUGFIX: Store x_mixed for backward pass prediction
        self.history.append({
            'x_filt': [x.copy() for x in x_filt],
            'P_filt': [P.copy() for P in P_filt],
            'x_pred': [x.copy() for x in x_pred],  # F @ x_mixed
            'P_pred': [P.copy() for P in P_pred],
            'x_mixed': [x.copy() for x in x_mixed],  # BUGFIX: store mixed states
            'mu': self.mu.copy(),
            'F': [F.copy() for F in self.F]
        })
        
        # Update state
        self.x = x_filt
        self.P = P_filt
        
        # Combined estimate
        x_comb = sum(self.mu[j] * x_filt[j] for j in range(n))
        
        return x_comb, self.mu
    
    def smooth(self) -> List[Tuple[np.ndarray, np.ndarray]]:
        """
        Per-model RTS smoother with BUGFIX.
        
        BUGFIX: Backward prediction uses F @ x_mixed (stored), not F @ x_filt.
        This is mathematically correct because the forward prediction used x_mixed.
        """
        if len(self.history) < 2:
            return []
        
        n = self.n_models
        L = len(self.history)
        
        # Initialize with last filtered
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
                
                # BUGFIX: Use stored x_pred which was computed from x_mixed
                x_p = h_next['x_pred'][j]  # = F @ x_mixed[j]
                P_p = h_next['P_pred'][j]
                
                # Regularize for inversion
                P_p_reg = P_p + 1e-6 * np.eye(4)
                
                try:
                    G = P_f @ F.T @ inv(P_p_reg)
                except:
                    G = np.zeros((4, 4))
                
                # Smooth
                x_smooth[k][j] = x_f + G @ (x_smooth[k+1][j] - x_p)
                P_smooth[k][j] = P_f + G @ (P_smooth[k+1][j] - P_p) @ G.T
        
        # Combine smoothed estimates
        results = []
        for k in range(L):
            mu = self.history[k]['mu']
            x_comb = sum(mu[j] * x_smooth[k][j] for j in range(n))
            results.append((x_comb, mu))
        
        return results


class IMMTrackerPreBugfix:
    """
    Pre-bugfix IMM Tracker for comparison.
    
    BUG: Backward prediction uses F @ x_filt instead of F @ x_mixed.
    This causes singular matrix issues during turn transitions.
    """
    
    def __init__(self, config: HypersonicScenarioConfig):
        self.config = config
        self.dt = config.dt
        self.omega = config.omega_turn
        self.n_models = 3
        
        self._build_models()
        self.x = None
        self.P = None
        self.mu = None
        self.history = []
        
    def _build_models(self):
        dt = self.dt
        omega = self.omega
        
        F_cv = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]])
        
        if omega != 0:
            s, c = np.sin(omega*dt), np.cos(omega*dt)
            F_ct_pos = np.array([[1, 0, s/omega, -(1-c)/omega], [0, 1, (1-c)/omega, s/omega],
                                [0, 0, c, -s], [0, 0, s, c]])
            s_n, c_n = np.sin(-omega*dt), np.cos(-omega*dt)
            F_ct_neg = np.array([[1, 0, s_n/(-omega), -(1-c_n)/(-omega)], 
                                [0, 1, (1-c_n)/(-omega), s_n/(-omega)],
                                [0, 0, c_n, -s_n], [0, 0, s_n, c_n]])
        else:
            F_ct_pos = F_cv.copy()
            F_ct_neg = F_cv.copy()
        
        self.F = [F_cv, F_ct_pos, F_ct_neg]
        
        c = self.config
        Q_base = np.diag([dt**4/4, dt**4/4, dt**2, dt**2])
        self.Q = [c.q_cv * Q_base, c.q_ct * Q_base, c.q_ct * Q_base]
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        self.R = c.sigma_meas_m**2 * np.eye(2)
        
        p = c.p_stay
        p_sw = (1 - p) / 2
        self.PI = np.array([[p, p_sw, p_sw], [p_sw, p, p_sw], [p_sw, p_sw, p]])
    
    def initialize(self, z0: np.ndarray, v_init: np.ndarray = None):
        if v_init is None:
            v_init = np.array([-self.config.speed_mps, 0])
        x0 = np.array([z0[0], z0[1], v_init[0], v_init[1]])
        P0 = np.diag([100, 100, 500, 500])**2
        self.x = [x0.copy() for _ in range(self.n_models)]
        self.P = [P0.copy() for _ in range(self.n_models)]
        self.mu = np.array([0.8, 0.1, 0.1])
        self.history = []
    
    def update(self, z: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        n = self.n_models
        c_bar = self.PI.T @ self.mu
        
        mu_ij = np.zeros((n, n))
        for i in range(n):
            for j in range(n):
                mu_ij[i, j] = self.PI[i, j] * self.mu[i] / (c_bar[j] + 1e-10)
        
        x_mixed, P_mixed = [], []
        for j in range(n):
            x_j = sum(mu_ij[i, j] * self.x[i] for i in range(n))
            x_mixed.append(x_j)
            P_j = sum(mu_ij[i, j] * (self.P[i] + np.outer(self.x[i] - x_j, self.x[i] - x_j)) 
                     for i in range(n))
            P_mixed.append(P_j)
        
        x_filt, P_filt, likelihoods = [], [], []
        
        for j in range(n):
            xp = self.F[j] @ x_mixed[j]
            Pp = self.F[j] @ P_mixed[j] @ self.F[j].T + self.Q[j]
            y = z - self.H @ xp
            S = self.H @ Pp @ self.H.T + self.R + 1e-6 * np.eye(2)
            
            try:
                K = Pp @ self.H.T @ inv(S)
            except:
                K = np.zeros((4, 2))
            
            x_f = xp + K @ y
            I_KH = np.eye(4) - K @ self.H
            P_f = I_KH @ Pp @ I_KH.T + K @ self.R @ K.T
            
            x_filt.append(x_f)
            P_filt.append(P_f)
            
            try:
                lik = np.exp(-0.5 * y @ inv(S) @ y) / np.sqrt((2*np.pi)**2 * det(S))
            except:
                lik = 1e-10
            likelihoods.append(max(lik, 1e-10))
        
        mu_unnorm = c_bar * np.array(likelihoods)
        self.mu = mu_unnorm / (mu_unnorm.sum() + 1e-10)
        
        # BUG: Store x_filt instead of x_mixed for prediction
        self.history.append({
            'x_filt': [x.copy() for x in x_filt],
            'P_filt': [P.copy() for P in P_filt],
            'mu': self.mu.copy(),
            'F': [F.copy() for F in self.F]
        })
        
        self.x = x_filt
        self.P = P_filt
        
        return sum(self.mu[j] * x_filt[j] for j in range(n)), self.mu
    
    def smooth(self) -> List[Tuple[np.ndarray, np.ndarray]]:
        """Pre-bugfix smoother with INCORRECT prediction."""
        if len(self.history) < 2:
            return []
        
        n = self.n_models
        L = len(self.history)
        
        x_smooth = [[h['x_filt'][j].copy() for j in range(n)] for h in self.history]
        P_smooth = [[h['P_filt'][j].copy() for j in range(n)] for h in self.history]
        
        for k in range(L - 2, -1, -1):
            h = self.history[k]
            h_next = self.history[k + 1]
            
            for j in range(n):
                F = h['F'][j]
                x_f = h['x_filt'][j]
                P_f = h['P_filt'][j]
                
                # BUG: Use F @ x_filt instead of F @ x_mixed
                x_p = F @ h_next['x_filt'][j]  # WRONG!
                P_p = F @ h_next['P_filt'][j] @ F.T
                
                P_p_reg = P_p + 1e-4 * np.eye(4)  # Need more regularization due to bug
                
                try:
                    G = P_f @ F.T @ inv(P_p_reg)
                except:
                    G = np.zeros((4, 4))
                
                x_smooth[k][j] = x_f + G @ (x_smooth[k+1][j] - x_p)
                P_smooth[k][j] = P_f + G @ (P_smooth[k+1][j] - P_p) @ G.T
        
        results = []
        for k in range(L):
            mu = self.history[k]['mu']
            x_comb = sum(mu[j] * x_smooth[k][j] for j in range(n))
            results.append((x_comb, mu))
        
        return results


def run_hypersonic_validation():
    """
    Validate v3.1 on hypersonic weave scenario.
    
    Monte Carlo: 50 runs
    Compare: Forward IMM vs Pre-bugfix Smoother vs v3.1 Smoother
    """
    print("=" * 70)
    print("NX-MIMOSA v3.1 HYPERSONIC WEAVE VALIDATION")
    print("[REQ-V31-HYPERSONIC-01] Mach 5 S-weave scenario")
    print("=" * 70)
    
    config = HypersonicScenarioConfig()
    traj_gen = HypersonicTrajectoryGenerator(config)
    
    # Simulation parameters
    T_seconds = 25  # 25 second engagement
    T_steps = int(T_seconds / config.dt)
    n_monte_carlo = 50
    
    # Results storage
    rmse_forward = []
    rmse_prebugfix = []
    rmse_v31 = []
    
    print(f"\nRunning {n_monte_carlo} Monte Carlo trials...")
    print(f"Target: Mach {config.speed_mps/340:.1f}, ±{config.weave_g}g S-weave")
    print(f"Range: {config.initial_range_m/1000:.0f} km, Altitude: {config.altitude_m/1000:.0f} km")
    print()
    
    for run in range(n_monte_carlo):
        np.random.seed(run + 1000)
        
        # Generate trajectory
        x_true = traj_gen.generate(T_steps)
        
        # Generate measurements
        z_meas = x_true[:, :2] + np.random.randn(T_steps, 2) * config.sigma_meas_m
        
        # Initialize trackers
        tracker_v31 = IMMTrackerV31(config)
        tracker_prebugfix = IMMTrackerPreBugfix(config)
        
        v_init = np.array([x_true[0, 2], x_true[0, 3]])
        tracker_v31.initialize(z_meas[0], v_init)
        tracker_prebugfix.initialize(z_meas[0], v_init)
        
        # Track
        for k in range(1, T_steps):
            tracker_v31.update(z_meas[k])
            tracker_prebugfix.update(z_meas[k])
        
        # Get smoothed estimates
        smoothed_v31 = tracker_v31.smooth()
        smoothed_prebugfix = tracker_prebugfix.smooth()
        
        # Compute errors
        # Forward-only (from v31 tracker, no smoothing)
        forward_est = np.array([h['x_filt'][0] for h in tracker_v31.history])  # CV model
        forward_comb = np.array([sum(h['mu'][j] * h['x_filt'][j] for j in range(3)) 
                                for h in tracker_v31.history])
        
        # Smoothed estimates
        if smoothed_v31:
            v31_est = np.array([s[0] for s in smoothed_v31])
            prebugfix_est = np.array([s[0] for s in smoothed_prebugfix])
            
            # RMSE computation (position only)
            err_forward = np.sqrt(np.mean((forward_comb[:, :2] - x_true[1:, :2])**2))
            err_prebugfix = np.sqrt(np.mean((prebugfix_est[:, :2] - x_true[1:T_steps, :2])**2))
            err_v31 = np.sqrt(np.mean((v31_est[:, :2] - x_true[1:T_steps, :2])**2))
            
            rmse_forward.append(err_forward)
            rmse_prebugfix.append(err_prebugfix)
            rmse_v31.append(err_v31)
        
        if (run + 1) % 10 == 0:
            print(f"  Completed {run + 1}/{n_monte_carlo} runs...")
    
    # Compute statistics
    rmse_forward = np.array(rmse_forward)
    rmse_prebugfix = np.array(rmse_prebugfix)
    rmse_v31 = np.array(rmse_v31)
    
    mean_forward = np.mean(rmse_forward)
    mean_prebugfix = np.mean(rmse_prebugfix)
    mean_v31 = np.mean(rmse_v31)
    
    improvement_vs_forward = (mean_forward - mean_v31) / mean_forward * 100
    improvement_vs_prebugfix = (mean_prebugfix - mean_v31) / mean_prebugfix * 100
    
    print()
    print("=" * 70)
    print("RESULTS")
    print("=" * 70)
    print(f"\n{'Tracker':<30} {'RMSE (m)':<15} {'Std (m)':<15} {'Improvement':<15}")
    print("-" * 70)
    print(f"{'Forward IMM (baseline)':<30} {mean_forward:<15.2f} {np.std(rmse_forward):<15.2f} {'-':<15}")
    print(f"{'Smoother (pre-bugfix)':<30} {mean_prebugfix:<15.2f} {np.std(rmse_prebugfix):<15.2f} "
          f"{(mean_forward - mean_prebugfix)/mean_forward*100:+.1f}%")
    print(f"{'v3.1 Smoother (BUGFIX)':<30} {mean_v31:<15.2f} {np.std(rmse_v31):<15.2f} "
          f"{improvement_vs_forward:+.1f}%")
    print("-" * 70)
    
    print(f"\n🎯 KEY RESULTS:")
    print(f"   v3.1 vs Forward:    {improvement_vs_forward:+.1f}% improvement")
    print(f"   v3.1 vs Pre-bugfix: {improvement_vs_prebugfix:+.1f}% improvement")
    print(f"   Scenario: Mach {config.speed_mps/340:.1f}, ±{config.weave_g}g S-weave")
    
    # Validation check
    print()
    if improvement_vs_forward > 25:
        print("✅ VALIDATION PASSED: >25% improvement on hypersonic scenario")
    else:
        print("⚠️ VALIDATION MARGINAL: <25% improvement")
    
    if mean_v31 < mean_prebugfix:
        print("✅ BUGFIX CONFIRMED: v3.1 outperforms pre-bugfix smoother")
    else:
        print("❌ BUGFIX ISSUE: Pre-bugfix performing better (unexpected)")
    
    return {
        'mean_forward': mean_forward,
        'mean_prebugfix': mean_prebugfix,
        'mean_v31': mean_v31,
        'improvement_vs_forward': improvement_vs_forward,
        'improvement_vs_prebugfix': improvement_vs_prebugfix
    }


if __name__ == "__main__":
    results = run_hypersonic_validation()
    print("\n" + "=" * 70)
    print("HYPERSONIC VALIDATION COMPLETE")
    print("=" * 70)
