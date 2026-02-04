"""
NX-MIMOSA v2.0 — Reference Implementation
=========================================
Multi-model IMM Optimal Smoothing Algorithm with Adaptive Features

Features:
- UKF/CKF/EKF selectable filter cores
- Adaptive process noise (NIS-based)
- Dynamic mode persistence (VS-IMM)
- 4-model set (CV, CA, CT+, CT-)
- Per-model RTS smoother

Author: Dr. Mladen Mešter / Nexellum d.o.o.
License: Commercial
Contact: mladen@nexellum.com | +385 99 737 5100
"""

import numpy as np
from numpy.linalg import inv, cholesky, det
from scipy.stats import chi2
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Literal
from enum import Enum


class FilterType(Enum):
    EKF = "ekf"
    UKF = "ukf"
    CKF = "ckf"


@dataclass
class NXMimosaConfig:
    """Configuration for NX-MIMOSA v2.0 tracker."""
    
    # Time step
    dt: float = 0.1
    
    # Filter type
    filter_type: FilterType = FilterType.UKF
    
    # Model selection
    models: List[str] = field(default_factory=lambda: ["CV", "CA", "CT+", "CT-"])
    
    # Turn rate for CT models (rad/s)
    omega: float = 0.196  # ~11.2 deg/s, typical fighter turn
    
    # Process noise (base values)
    q_cv: float = 0.15    # Tight CV for straight segments
    q_ca: float = 2.0     # Higher for acceleration
    q_ct: float = 1.0     # Medium for turns
    
    # Measurement noise std (meters)
    r: float = 5.0
    
    # Transition probability (base stay probability)
    p_stay: float = 0.95
    
    # Adaptive features
    adaptive_q: bool = True
    vs_imm: bool = True
    
    # UKF parameters
    ukf_alpha: float = 0.5
    ukf_beta: float = 2.0
    ukf_kappa: float = 3 - 4  # 3 - n
    
    # Smoothing lag
    lag_depth: int = 50
    
    # Chi-squared threshold (2 DOF, 95% confidence)
    chi2_threshold: float = 5.991


@dataclass
class TrackState:
    """State estimate from tracker."""
    x: np.ndarray           # Combined state estimate
    P: np.ndarray           # Combined covariance
    mu: np.ndarray          # Mode probabilities
    x_models: List[np.ndarray]  # Per-model estimates
    P_models: List[np.ndarray]  # Per-model covariances
    nis: float              # Normalized innovation squared
    q_scale: float          # Current Q scaling factor
    dominant_model: str     # Most likely model


class AdaptiveQManager:
    """NIS-based adaptive process noise scaling."""
    
    def __init__(self, config: NXMimosaConfig):
        self.config = config
        self.nis_history = []
        self.q_scale = 1.0
        self.alpha_smooth = 0.2
        
    def update(self, innovation: np.ndarray, S: np.ndarray) -> float:
        """
        Compute NIS and update Q scaling factor.
        
        NIS = y' * S^(-1) * y
        If NIS > threshold: increase Q (filter too optimistic)
        If NIS < threshold: decrease Q slowly (filter conservative)
        """
        S_inv = inv(S)
        nis = float(innovation.T @ S_inv @ innovation)
        
        self.nis_history.append(nis)
        if len(self.nis_history) > 20:
            self.nis_history.pop(0)
        
        # Smoothed NIS
        nis_smooth = np.mean(self.nis_history[-8:]) if len(self.nis_history) >= 8 else nis
        
        # Determine scaling
        if nis_smooth > self.config.chi2_threshold:
            target_scale = 1.5  # Increase Q
        elif nis_smooth > self.config.chi2_threshold / 2:
            target_scale = 1.125  # Slight increase
        else:
            target_scale = 0.95  # Slow decrease
        
        # Smooth transition
        self.q_scale = self.alpha_smooth * target_scale + (1 - self.alpha_smooth) * self.q_scale
        
        # Clamp
        self.q_scale = np.clip(self.q_scale, 0.1, 5.0)
        
        return nis_smooth
    
    def get_scaled_q(self, q_base: float) -> float:
        """Get scaled process noise."""
        return q_base * self.q_scale


class DynamicTPMManager:
    """VS-IMM dynamic transition probability matrix."""
    
    def __init__(self, config: NXMimosaConfig):
        self.config = config
        self.n_models = len(config.models)
        
    def compute_tpm(self, mu: np.ndarray, maneuver_detected: bool) -> np.ndarray:
        """
        Compute dynamic TPM based on mode confidence.
        
        High confidence → high persistence (stay in current model)
        Maneuver detected → reduce CV persistence
        """
        max_mu = np.max(mu)
        dominant = np.argmax(mu)
        
        # Determine stay probability
        if max_mu >= 0.9:
            if maneuver_detected and dominant == 0:  # CV dominant but maneuvering
                p_stay = 0.8
            else:
                p_stay = 0.99
        elif max_mu >= 0.7:
            p_stay = 0.9
        else:
            p_stay = 0.8
        
        # Build symmetric TPM
        p_switch = (1 - p_stay) / (self.n_models - 1)
        PI = np.full((self.n_models, self.n_models), p_switch)
        np.fill_diagonal(PI, p_stay)
        
        return PI


class UnscentedTransform:
    """Unscented transform for UKF."""
    
    def __init__(self, n: int, alpha: float = 0.5, beta: float = 2.0, kappa: float = -1):
        self.n = n
        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa
        
        # Compute lambda and weights
        self.lambda_ = alpha**2 * (n + kappa) - n
        self.gamma = np.sqrt(n + self.lambda_)
        
        # Weights
        self.Wm = np.zeros(2*n + 1)
        self.Wc = np.zeros(2*n + 1)
        
        self.Wm[0] = self.lambda_ / (n + self.lambda_)
        self.Wc[0] = self.Wm[0] + (1 - alpha**2 + beta)
        
        for i in range(1, 2*n + 1):
            self.Wm[i] = 1 / (2 * (n + self.lambda_))
            self.Wc[i] = self.Wm[i]
    
    def sigma_points(self, x: np.ndarray, P: np.ndarray) -> np.ndarray:
        """Generate sigma points."""
        n = self.n
        L = 2*n + 1
        chi = np.zeros((L, n))
        
        # Cholesky decomposition
        try:
            sqrt_P = cholesky((n + self.lambda_) * P)
        except np.linalg.LinAlgError:
            # Regularization if not positive definite
            P_reg = P + 1e-6 * np.eye(n)
            sqrt_P = cholesky((n + self.lambda_) * P_reg)
        
        chi[0] = x
        for i in range(n):
            chi[i+1] = x + sqrt_P[:, i]
            chi[n+i+1] = x - sqrt_P[:, i]
        
        return chi
    
    def unscented_transform(self, chi: np.ndarray, f: callable, 
                           Q: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray]:
        """Apply unscented transform through function f."""
        L = chi.shape[0]
        
        # Transform sigma points
        gamma = np.array([f(chi[i]) for i in range(L)])
        m = gamma.shape[1]
        
        # Mean
        y = np.sum(self.Wm[:, None] * gamma, axis=0)
        
        # Covariance
        Pyy = np.zeros((m, m))
        for i in range(L):
            dy = gamma[i] - y
            Pyy += self.Wc[i] * np.outer(dy, dy)
        
        if Q is not None:
            Pyy += Q
        
        return y, Pyy


class NXMimosaV2:
    """
    NX-MIMOSA v2.0 — Multi-model IMM Optimal Smoothing Algorithm
    
    Features:
    - UKF/CKF/EKF selectable cores
    - Adaptive Q (NIS-based)
    - VS-IMM (dynamic TPM)
    - 4-model set (CV, CA, CT+, CT-)
    - Per-model RTS smoother
    """
    
    def __init__(self, config: NXMimosaConfig = None):
        self.config = config or NXMimosaConfig()
        self.n_models = len(self.config.models)
        self.state_dim = 6 if "CA" in self.config.models else 4
        self.meas_dim = 2
        
        # Initialize managers
        self.adaptive_q = AdaptiveQManager(self.config) if self.config.adaptive_q else None
        self.dynamic_tpm = DynamicTPMManager(self.config) if self.config.vs_imm else None
        
        # UKF transform
        if self.config.filter_type == FilterType.UKF:
            self.ut = UnscentedTransform(
                self.state_dim,
                self.config.ukf_alpha,
                self.config.ukf_beta,
                self.config.ukf_kappa
            )
        else:
            self.ut = None
        
        # State storage
        self.x = None  # Per-model states
        self.P = None  # Per-model covariances
        self.mu = None  # Mode probabilities
        
        # History for smoother
        self.history = []
        
        self._build_models()
    
    def _build_models(self):
        """Build motion and measurement models."""
        dt = self.config.dt
        omega = self.config.omega
        
        self.F = {}
        self.Q = {}
        
        # CV Model (4D or 6D with zero accel)
        if self.state_dim == 4:
            F_cv = np.array([
                [1, 0, dt, 0],
                [0, 1, 0, dt],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])
            q = self.config.q_cv
            Q_cv = q * np.array([
                [dt**4/4, 0, dt**3/2, 0],
                [0, dt**4/4, 0, dt**3/2],
                [dt**3/2, 0, dt**2, 0],
                [0, dt**3/2, 0, dt**2]
            ])
        else:  # 6D
            F_cv = np.array([
                [1, 0, dt, 0, dt**2/2, 0],
                [0, 1, 0, dt, 0, dt**2/2],
                [0, 0, 1, 0, dt, 0],
                [0, 0, 0, 1, 0, dt],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1]
            ])
            q = self.config.q_cv
            Q_cv = q * np.eye(6) * np.array([dt**4/4, dt**4/4, dt**2, dt**2, 1, 1])
        
        self.F["CV"] = F_cv
        self.Q["CV"] = Q_cv
        
        # CA Model (only for 6D)
        if "CA" in self.config.models and self.state_dim == 6:
            self.F["CA"] = F_cv.copy()  # Same structure
            q = self.config.q_ca
            self.Q["CA"] = q * np.eye(6) * np.array([dt**4/4, dt**4/4, dt**2, dt**2, 1, 1])
        
        # CT+ Model (right turn)
        sin_wdt = np.sin(omega * dt)
        cos_wdt = np.cos(omega * dt)
        if omega != 0:
            F_ct_pos = np.array([
                [1, 0, sin_wdt/omega, -(1-cos_wdt)/omega],
                [0, 1, (1-cos_wdt)/omega, sin_wdt/omega],
                [0, 0, cos_wdt, -sin_wdt],
                [0, 0, sin_wdt, cos_wdt]
            ])
        else:
            F_ct_pos = F_cv[:4, :4] if self.state_dim == 4 else F_cv
        
        if self.state_dim == 6:
            F_ct_pos_6d = np.eye(6)
            F_ct_pos_6d[:4, :4] = F_ct_pos
            self.F["CT+"] = F_ct_pos_6d
        else:
            self.F["CT+"] = F_ct_pos
        
        q = self.config.q_ct
        if self.state_dim == 4:
            self.Q["CT+"] = q * np.array([
                [dt**4/4, 0, dt**3/2, 0],
                [0, dt**4/4, 0, dt**3/2],
                [dt**3/2, 0, dt**2, 0],
                [0, dt**3/2, 0, dt**2]
            ])
        else:
            self.Q["CT+"] = q * np.eye(6) * np.array([dt**4/4, dt**4/4, dt**2, dt**2, 1, 1])
        
        # CT- Model (left turn)
        sin_wdt_neg = np.sin(-omega * dt)
        cos_wdt_neg = np.cos(-omega * dt)
        if omega != 0:
            F_ct_neg = np.array([
                [1, 0, sin_wdt_neg/(-omega), -(1-cos_wdt_neg)/(-omega)],
                [0, 1, (1-cos_wdt_neg)/(-omega), sin_wdt_neg/(-omega)],
                [0, 0, cos_wdt_neg, -sin_wdt_neg],
                [0, 0, sin_wdt_neg, cos_wdt_neg]
            ])
        else:
            F_ct_neg = F_cv[:4, :4] if self.state_dim == 4 else F_cv
        
        if self.state_dim == 6:
            F_ct_neg_6d = np.eye(6)
            F_ct_neg_6d[:4, :4] = F_ct_neg
            self.F["CT-"] = F_ct_neg_6d
        else:
            self.F["CT-"] = F_ct_neg
        
        self.Q["CT-"] = self.Q["CT+"].copy()
        
        # Measurement model: H = [I_2, 0_2x2] or [I_2, 0_2x4]
        if self.state_dim == 4:
            self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        else:
            self.H = np.array([[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0]])
        
        # Measurement noise
        self.R = self.config.r**2 * np.eye(2)
    
    def initialize(self, x0: np.ndarray, P0: np.ndarray = None):
        """Initialize tracker with state estimate."""
        if P0 is None:
            P0 = np.diag([100, 100, 10, 10, 5, 5][:self.state_dim])**2
        
        # Initialize all models with same state
        self.x = [x0.copy() for _ in range(self.n_models)]
        self.P = [P0.copy() for _ in range(self.n_models)]
        
        # Initial mode probabilities (CV-biased)
        self.mu = np.array([0.7] + [0.1]*(self.n_models-1))
        self.mu /= self.mu.sum()
        
        self.history = []
    
    def _predict_ekf(self, x: np.ndarray, P: np.ndarray, 
                     F: np.ndarray, Q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """EKF prediction step."""
        xp = F @ x
        Pp = F @ P @ F.T + Q
        return xp, Pp
    
    def _predict_ukf(self, x: np.ndarray, P: np.ndarray,
                     model: str) -> Tuple[np.ndarray, np.ndarray]:
        """UKF prediction step using unscented transform."""
        F = self.F[model]
        Q = self.Q[model]
        
        # Generate sigma points
        chi = self.ut.sigma_points(x, P)
        
        # Motion model function
        def f(xi):
            return F @ xi
        
        # Unscented transform
        xp, Pp = self.ut.unscented_transform(chi, f, Q)
        
        return xp, Pp
    
    def _update(self, xp: np.ndarray, Pp: np.ndarray, 
                z: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float, np.ndarray, np.ndarray]:
        """Kalman update step."""
        H = self.H
        R = self.R
        
        # Innovation
        y = z - H @ xp
        S = H @ Pp @ H.T + R
        
        # Kalman gain
        K = Pp @ H.T @ inv(S)
        
        # Update
        x_filt = xp + K @ y
        
        # Joseph form for numerical stability
        I_KH = np.eye(self.state_dim) - K @ H
        P_filt = I_KH @ Pp @ I_KH.T + K @ R @ K.T
        
        # Likelihood
        S_inv = inv(S)
        nis = float(y.T @ S_inv @ y)
        likelihood = np.exp(-0.5 * nis) / np.sqrt((2*np.pi)**self.meas_dim * det(S))
        likelihood = max(likelihood, 1e-10)  # Prevent zero
        
        return x_filt, P_filt, likelihood, y, S
    
    def update(self, z: np.ndarray) -> TrackState:
        """
        Process measurement and return state estimate.
        
        IMM Algorithm:
        1. Mixing: Combine model states based on TPM
        2. Predict: Run each filter's prediction
        3. Update: Run each filter's measurement update
        4. Combine: Weight estimates by mode probabilities
        """
        if self.x is None:
            raise ValueError("Tracker not initialized. Call initialize() first.")
        
        models = self.config.models
        n = self.n_models
        
        # Get TPM
        maneuver_detected = False
        if self.adaptive_q:
            maneuver_detected = self.adaptive_q.q_scale > 1.2
        
        if self.dynamic_tpm:
            PI = self.dynamic_tpm.compute_tpm(self.mu, maneuver_detected)
        else:
            p_stay = self.config.p_stay
            p_switch = (1 - p_stay) / (n - 1)
            PI = np.full((n, n), p_switch)
            np.fill_diagonal(PI, p_stay)
        
        #----------------------------------------------------------------------
        # 1. Mixing
        #----------------------------------------------------------------------
        # c_bar[j] = sum_i(PI[i,j] * mu[i])
        c_bar = PI.T @ self.mu
        
        # mu_ij[i,j] = PI[i,j] * mu[i] / c_bar[j]
        mu_ij = np.zeros((n, n))
        for i in range(n):
            for j in range(n):
                mu_ij[i, j] = PI[i, j] * self.mu[i] / (c_bar[j] + 1e-10)
        
        # Mixed states
        x_mixed = []
        P_mixed = []
        for j in range(n):
            x_j = sum(mu_ij[i, j] * self.x[i] for i in range(n))
            x_mixed.append(x_j)
            
            P_j = np.zeros_like(self.P[0], dtype=np.float64)
            for i in range(n):
                dx = self.x[i] - x_j
                P_j += mu_ij[i, j] * (self.P[i] + np.outer(dx, dx))
            P_mixed.append(P_j)
        
        #----------------------------------------------------------------------
        # 2. Predict & Update per model
        #----------------------------------------------------------------------
        x_filt_models = []
        P_filt_models = []
        x_pred_models = []
        P_pred_models = []
        likelihoods = []
        innovations = []
        S_matrices = []
        
        for j, model in enumerate(models):
            # Get Q (possibly scaled by adaptive module)
            if self.adaptive_q:
                q_base = self.config.q_cv if model == "CV" else \
                        self.config.q_ca if model == "CA" else self.config.q_ct
                q_scaled = self.adaptive_q.get_scaled_q(q_base)
                Q = self.Q[model] * (q_scaled / q_base)
            else:
                Q = self.Q[model]
            
            # Predict
            if self.config.filter_type == FilterType.UKF and self.ut:
                xp, Pp = self._predict_ukf(x_mixed[j], P_mixed[j], model)
            else:
                F = self.F[model]
                xp, Pp = self._predict_ekf(x_mixed[j], P_mixed[j], F, Q)
            
            x_pred_models.append(xp)
            P_pred_models.append(Pp)
            
            # Update
            x_filt, P_filt, likelihood, y, S = self._update(xp, Pp, z)
            
            x_filt_models.append(x_filt)
            P_filt_models.append(P_filt)
            likelihoods.append(likelihood)
            innovations.append(y)
            S_matrices.append(S)
        
        #----------------------------------------------------------------------
        # 3. Update Mode Probabilities
        #----------------------------------------------------------------------
        # mu[j] = c_bar[j] * likelihood[j]
        mu_unnorm = c_bar * np.array(likelihoods)
        self.mu = mu_unnorm / (mu_unnorm.sum() + 1e-10)
        
        #----------------------------------------------------------------------
        # 4. Combine Estimates
        #----------------------------------------------------------------------
        x_combined = sum(self.mu[j] * x_filt_models[j] for j in range(n))
        
        P_combined = np.zeros_like(self.P[0], dtype=np.float64)
        for j in range(n):
            dx = x_filt_models[j] - x_combined
            P_combined += self.mu[j] * (P_filt_models[j] + np.outer(dx, dx))
        
        # Store for next iteration
        self.x = x_filt_models
        self.P = P_filt_models
        
        #----------------------------------------------------------------------
        # 5. Update Adaptive Q
        #----------------------------------------------------------------------
        nis = 0.0
        if self.adaptive_q:
            # Use combined innovation
            y_combined = sum(self.mu[j] * innovations[j] for j in range(n))
            S_combined = sum(self.mu[j] * S_matrices[j] for j in range(n))
            nis = self.adaptive_q.update(y_combined, S_combined)
        
        #----------------------------------------------------------------------
        # 6. Store history for smoother
        #----------------------------------------------------------------------
        self.history.append({
            'x_filt': x_filt_models.copy(),
            'P_filt': P_filt_models.copy(),
            'x_pred': x_pred_models.copy(),
            'P_pred': P_pred_models.copy(),
            'mu': self.mu.copy(),
            'F': [self.F[m] for m in models]
        })
        
        if len(self.history) > self.config.lag_depth:
            self.history.pop(0)
        
        # Dominant model
        dominant_idx = np.argmax(self.mu)
        dominant_model = models[dominant_idx]
        
        return TrackState(
            x=x_combined,
            P=P_combined,
            mu=self.mu.copy(),
            x_models=x_filt_models,
            P_models=P_filt_models,
            nis=nis,
            q_scale=self.adaptive_q.q_scale if self.adaptive_q else 1.0,
            dominant_model=dominant_model
        )
    
    def smooth(self) -> List[TrackState]:
        """
        Run RTS smoother over history.
        
        Per-model smoothing (NX-MIMOSA innovation):
        For each model j:
            G[j] = P_filt[j] @ F[j].T @ inv(P_pred[j])
            x_smooth[j] = x_filt[j] + G[j] @ (x_smooth[k+1] - x_pred[k+1])
        
        Combined: x_smooth = sum(mu[j] * x_smooth[j])
        """
        if len(self.history) < 2:
            return []
        
        n = self.n_models
        L = len(self.history)
        
        # Initialize with last filtered estimates
        x_smooth = [h['x_filt'].copy() for h in self.history]
        P_smooth = [h['P_filt'].copy() for h in self.history]
        
        # Backward pass
        for k in range(L - 2, -1, -1):
            h = self.history[k]
            h_next = self.history[k + 1]
            
            x_smooth_k = []
            P_smooth_k = []
            
            for j in range(n):
                F = h['F'][j]
                x_filt = h['x_filt'][j]
                P_filt = h['P_filt'][j]
                x_pred_next = h_next['x_pred'][j]
                P_pred_next = h_next['P_pred'][j]
                
                # Smoother gain
                try:
                    G = P_filt @ F.T @ inv(P_pred_next)
                except:
                    G = np.zeros_like(P_filt)
                
                # Smoothed state
                x_s = x_filt + G @ (x_smooth[k + 1][j] - x_pred_next)
                P_s = P_filt + G @ (P_smooth[k + 1][j] - P_pred_next) @ G.T
                
                x_smooth_k.append(x_s)
                P_smooth_k.append(P_s)
            
            x_smooth[k] = x_smooth_k
            P_smooth[k] = P_smooth_k
        
        # Combine smoothed estimates
        smoothed_states = []
        for k in range(L):
            mu = self.history[k]['mu']
            x_comb = sum(mu[j] * x_smooth[k][j] for j in range(n))
            
            P_comb = np.zeros_like(P_smooth[k][0], dtype=np.float64)
            for j in range(n):
                dx = x_smooth[k][j] - x_comb
                P_comb += mu[j] * (P_smooth[k][j] + np.outer(dx, dx))
            
            smoothed_states.append(TrackState(
                x=x_comb,
                P=P_comb,
                mu=mu,
                x_models=x_smooth[k],
                P_models=P_smooth[k],
                nis=0.0,
                q_scale=1.0,
                dominant_model=self.config.models[np.argmax(mu)]
            ))
        
        return smoothed_states


#==============================================================================
# Demo / Validation
#==============================================================================
def demo_tracking():
    """Demonstrate NX-MIMOSA v2.0 tracker."""
    import matplotlib.pyplot as plt
    
    print("=" * 60)
    print("NX-MIMOSA v2.0 — Demo Tracking Scenario")
    print("=" * 60)
    
    # Configuration
    config = NXMimosaConfig(
        dt=0.1,
        filter_type=FilterType.UKF,
        models=["CV", "CT+", "CT-"],  # 3-model for this demo
        adaptive_q=True,
        vs_imm=True,
        omega=0.2,  # ~11.5 deg/s turn rate
    )
    
    # Initialize tracker
    tracker = NXMimosaV2(config)
    
    # Generate ground truth trajectory
    # Straight → Turn → Straight
    np.random.seed(42)
    T = 100  # 10 seconds at dt=0.1
    dt = config.dt
    
    x_true = np.zeros((T, 4))  # [x, y, vx, vy]
    x_true[0] = [0, 0, 100, 50]  # Start position and velocity
    
    for k in range(1, T):
        if k < 30:  # Straight
            F = tracker.F["CV"][:4, :4]
        elif k < 70:  # Turn
            F = tracker.F["CT+"][:4, :4]
        else:  # Straight
            F = tracker.F["CV"][:4, :4]
        
        x_true[k] = F @ x_true[k-1]
    
    # Generate measurements
    R = config.r**2 * np.eye(2)
    z_meas = x_true[:, :2] + np.random.multivariate_normal([0, 0], R, T)
    
    # Initialize tracker
    x0 = np.array([z_meas[0, 0], z_meas[0, 1], 100, 50])
    P0 = np.diag([100, 100, 50, 50])**2
    tracker.initialize(x0, P0)
    
    # Track
    estimates = []
    mode_probs = []
    nis_values = []
    
    for k in range(1, T):
        state = tracker.update(z_meas[k])
        estimates.append(state.x)
        mode_probs.append(state.mu)
        nis_values.append(state.nis)
    
    estimates = np.array(estimates)
    mode_probs = np.array(mode_probs)
    
    # Compute RMSE
    rmse_pos = np.sqrt(np.mean((estimates[:, 0] - x_true[1:, 0])**2 + 
                                (estimates[:, 1] - x_true[1:, 1])**2))
    
    print(f"\nResults:")
    print(f"  Position RMSE: {rmse_pos:.2f} m")
    print(f"  Mean NIS: {np.mean(nis_values):.2f} (expected: ~2.0)")
    
    # Plot
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    # Trajectory
    ax = axes[0, 0]
    ax.plot(x_true[:, 0], x_true[:, 1], 'k-', lw=2, label='Truth')
    ax.plot(z_meas[:, 0], z_meas[:, 1], 'r.', alpha=0.3, label='Measurements')
    ax.plot(estimates[:, 0], estimates[:, 1], 'b-', lw=1.5, label='NX-MIMOSA v2.0')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.legend()
    ax.set_title('Trajectory')
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    
    # Position error
    ax = axes[0, 1]
    pos_err = np.sqrt((estimates[:, 0] - x_true[1:, 0])**2 + 
                      (estimates[:, 1] - x_true[1:, 1])**2)
    ax.plot(pos_err, 'b-')
    ax.axhline(rmse_pos, color='r', ls='--', label=f'RMSE={rmse_pos:.2f}m')
    ax.set_xlabel('Time step')
    ax.set_ylabel('Position Error (m)')
    ax.legend()
    ax.set_title('Position Error')
    ax.grid(True, alpha=0.3)
    
    # Mode probabilities
    ax = axes[1, 0]
    for j, model in enumerate(config.models):
        ax.plot(mode_probs[:, j], label=model)
    ax.axvline(30, color='k', ls=':', alpha=0.5)
    ax.axvline(70, color='k', ls=':', alpha=0.5)
    ax.set_xlabel('Time step')
    ax.set_ylabel('Mode Probability')
    ax.legend()
    ax.set_title('Mode Probabilities (VS-IMM)')
    ax.grid(True, alpha=0.3)
    ax.set_ylim([0, 1])
    
    # NIS
    ax = axes[1, 1]
    ax.plot(nis_values, 'g-', alpha=0.7)
    ax.axhline(config.chi2_threshold, color='r', ls='--', label=f'χ² threshold')
    ax.axhline(2.0, color='k', ls=':', label='Expected NIS')
    ax.set_xlabel('Time step')
    ax.set_ylabel('NIS')
    ax.legend()
    ax.set_title('Normalized Innovation Squared (Adaptive Q)')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('/home/claude/nx-mimosa-v2/demo_results.png', dpi=150)
    print(f"\n✅ Plot saved to demo_results.png")
    
    return rmse_pos


if __name__ == "__main__":
    rmse = demo_tracking()
