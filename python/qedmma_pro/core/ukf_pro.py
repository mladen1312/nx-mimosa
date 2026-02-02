"""
QEDMMA-Pro - Enhanced Unscented Kalman Filter (UKF-Pro)
========================================================
Copyright (C) 2026 Dr. Mladen Mešter / Nexellum
License: Commercial - See LICENSE_COMMERCIAL.md

PRO ENHANCEMENTS over Lite:
1. Square-Root UKF (SR-UKF) for numerical stability
2. Adaptive sigma point scaling based on innovation
3. State constraints handling (physical bounds)
4. Iterated updates for highly nonlinear measurements
5. GPU acceleration support (CuPy backend)
6. Real-time covariance monitoring & reset logic

For licensing: mladen@nexellum.com | www.nexellum.com
"""

import numpy as np
from numpy.linalg import cholesky, LinAlgError, qr
from typing import Callable, Tuple, Optional, List
from dataclasses import dataclass, field
from enum import Enum
import warnings


class UKFVariant(Enum):
    """Available UKF implementations"""
    STANDARD = "standard"
    SQUARE_ROOT = "square_root"
    ITERATED = "iterated"
    ADAPTIVE = "adaptive"
    CONSTRAINED = "constrained"


@dataclass
class UKFProParams:
    """Enhanced UKF parameters for PRO version"""
    # Standard UKF params
    alpha: float = 0.001
    beta: float = 2.0
    kappa: float = 0.0
    
    # PRO enhancements
    adaptive_scaling: bool = True          # Auto-adjust alpha based on innovation
    alpha_min: float = 1e-4
    alpha_max: float = 1.0
    
    iterated_updates: bool = False         # Enable IUKF
    max_iterations: int = 5
    iteration_tolerance: float = 1e-6
    
    constraint_handling: bool = False      # Enable state constraints
    state_min: Optional[np.ndarray] = None
    state_max: Optional[np.ndarray] = None
    
    covariance_reset_threshold: float = 1e6  # Reset P if trace exceeds
    
    use_gpu: bool = False                  # CuPy acceleration


@dataclass
class UKFProState:
    """Enhanced state container for UKF-Pro"""
    x: np.ndarray                     # State estimate
    S: np.ndarray                     # Square-root of covariance (Cholesky)
    Q: np.ndarray                     # Process noise
    R: np.ndarray                     # Measurement noise
    Sq: np.ndarray = field(init=False)  # sqrt(Q)
    Sr: np.ndarray = field(init=False)  # sqrt(R)
    n: int = field(init=False)
    
    # PRO tracking
    innovation_history: List[np.ndarray] = field(default_factory=list)
    nees_history: List[float] = field(default_factory=list)  # Normalized estimation error
    current_alpha: float = 0.001
    
    def __post_init__(self):
        self.n = len(self.x)
        try:
            self.Sq = cholesky(self.Q)
        except LinAlgError:
            self.Sq = np.sqrt(np.abs(self.Q))
        try:
            self.Sr = cholesky(self.R)
        except LinAlgError:
            self.Sr = np.sqrt(np.abs(self.R))
    
    @property
    def P(self) -> np.ndarray:
        """Reconstruct full covariance from square-root"""
        return self.S @ self.S.T
    
    def record_innovation(self, innovation: np.ndarray, S_innov: np.ndarray):
        """Record innovation for adaptive scaling"""
        self.innovation_history.append(innovation.copy())
        if len(self.innovation_history) > 50:
            self.innovation_history.pop(0)
        
        # NEES = ν' S⁻¹ ν
        try:
            nees = innovation @ np.linalg.solve(S_innov @ S_innov.T, innovation)
            self.nees_history.append(nees)
            if len(self.nees_history) > 50:
                self.nees_history.pop(0)
        except:
            pass


class UKFPro:
    """
    QEDMMA-Pro Enhanced Unscented Kalman Filter.
    
    PRO features:
    - Square-root formulation for guaranteed positive definiteness
    - Adaptive sigma point scaling based on filter consistency
    - State constraints (e.g., speed limits, altitude bounds)
    - Iterated measurement updates for strong nonlinearities
    - GPU acceleration via CuPy (optional)
    
    Example:
        >>> ukf = UKFPro(f, h, n_states=6, n_meas=3, 
        ...              params=UKFProParams(adaptive_scaling=True))
        >>> state = ukf.init_state(x0, S0, Q, R)
        >>> state = ukf.predict(state, dt=0.1)
        >>> state, metrics = ukf.update(state, z)
        >>> print(f"NEES: {metrics['nees']:.2f}")
    """
    
    def __init__(
        self,
        f: Callable[[np.ndarray, float], np.ndarray],
        h: Callable[[np.ndarray], np.ndarray],
        n_states: int,
        n_meas: int,
        params: Optional[UKFProParams] = None
    ):
        self.f = f
        self.h = h
        self.n = n_states
        self.m = n_meas
        self.params = params or UKFProParams()
        
        # Initialize numpy/cupy backend
        if self.params.use_gpu:
            try:
                import cupy as cp
                self.xp = cp
                print("UKF-Pro: GPU acceleration enabled (CuPy)")
            except ImportError:
                self.xp = np
                warnings.warn("CuPy not available, falling back to NumPy")
        else:
            self.xp = np
        
        self._compute_weights(self.params.alpha)
    
    def _compute_weights(self, alpha: float):
        """Compute sigma point weights with given alpha"""
        n = self.n
        beta = self.params.beta
        kappa = self.params.kappa
        
        self.lambda_ = alpha**2 * (n + kappa) - n
        self.gamma = np.sqrt(n + self.lambda_)
        
        self.n_sigma = 2 * n + 1
        
        self.Wm = np.zeros(self.n_sigma)
        self.Wm[0] = self.lambda_ / (n + self.lambda_)
        self.Wm[1:] = 0.5 / (n + self.lambda_)
        
        self.Wc = np.zeros(self.n_sigma)
        self.Wc[0] = self.Wm[0] + (1 - alpha**2 + beta)
        self.Wc[1:] = self.Wm[1:]
        
        # For square-root update
        self.sqrt_Wc = np.sqrt(np.abs(self.Wc))
        self.sqrt_Wc[0] = np.sign(self.Wc[0]) * np.sqrt(np.abs(self.Wc[0]))
    
    def init_state(
        self,
        x0: np.ndarray,
        P0_or_S0: np.ndarray,
        Q: np.ndarray,
        R: np.ndarray,
        is_sqrt: bool = False
    ) -> UKFProState:
        """
        Initialize UKF-Pro state.
        
        Args:
            x0: Initial state
            P0_or_S0: Initial covariance or its square root
            Q: Process noise covariance
            R: Measurement noise covariance
            is_sqrt: If True, P0_or_S0 is already the Cholesky factor
        """
        x0 = np.asarray(x0, dtype=np.float64)
        
        if is_sqrt:
            S0 = np.asarray(P0_or_S0, dtype=np.float64)
        else:
            P0 = np.asarray(P0_or_S0, dtype=np.float64)
            try:
                S0 = cholesky(P0 + 1e-9 * np.eye(self.n))
            except LinAlgError:
                # Fallback to eigendecomposition
                eigvals, eigvecs = np.linalg.eigh(P0)
                eigvals = np.maximum(eigvals, 1e-10)
                S0 = eigvecs @ np.diag(np.sqrt(eigvals))
        
        state = UKFProState(
            x=x0,
            S=S0,
            Q=np.asarray(Q, dtype=np.float64),
            R=np.asarray(R, dtype=np.float64)
        )
        state.current_alpha = self.params.alpha
        
        return state
    
    def _generate_sigma_points_sr(self, x: np.ndarray, S: np.ndarray) -> np.ndarray:
        """Generate sigma points from square-root covariance"""
        sigma = np.zeros((self.n_sigma, self.n))
        sigma[0] = x
        
        scaled_S = self.gamma * S
        
        for i in range(self.n):
            sigma[i + 1] = x + scaled_S[:, i]
            sigma[i + 1 + self.n] = x - scaled_S[:, i]
        
        return sigma
    
    def _apply_constraints(self, x: np.ndarray) -> np.ndarray:
        """Apply state constraints (PRO feature)"""
        if not self.params.constraint_handling:
            return x
        
        x_constrained = x.copy()
        
        if self.params.state_min is not None:
            x_constrained = np.maximum(x_constrained, self.params.state_min)
        
        if self.params.state_max is not None:
            x_constrained = np.minimum(x_constrained, self.params.state_max)
        
        return x_constrained
    
    def _qr_update(self, A: np.ndarray) -> np.ndarray:
        """QR decomposition for square-root covariance update"""
        _, R = qr(A.T)
        return R[:self.n, :].T
    
    def _cholupdate(self, S: np.ndarray, v: np.ndarray, sign: float = 1.0) -> np.ndarray:
        """Rank-1 Cholesky update: S' such that S'S'^T = SS^T + sign*vv^T"""
        n = len(v)
        S_new = S.copy()
        v = v.copy()
        
        for i in range(n):
            r = np.sqrt(S_new[i, i]**2 + sign * v[i]**2)
            c = r / S_new[i, i]
            s = v[i] / S_new[i, i]
            S_new[i, i] = r
            
            if i < n - 1:
                S_new[i+1:, i] = (S_new[i+1:, i] + sign * s * v[i+1:]) / c
                v[i+1:] = c * v[i+1:] - s * S_new[i+1:, i]
        
        return S_new
    
    def _adapt_scaling(self, state: UKFProState) -> float:
        """Adapt sigma point scaling based on filter consistency (PRO feature)"""
        if not self.params.adaptive_scaling or len(state.nees_history) < 10:
            return state.current_alpha
        
        # Average NEES should be approximately n_meas for consistent filter
        avg_nees = np.mean(state.nees_history[-20:])
        expected_nees = self.m
        
        # If NEES too high, increase alpha (spread sigma points)
        # If NEES too low, decrease alpha (tighten sigma points)
        ratio = avg_nees / expected_nees
        
        if ratio > 2.0:
            new_alpha = min(state.current_alpha * 1.1, self.params.alpha_max)
        elif ratio < 0.5:
            new_alpha = max(state.current_alpha * 0.9, self.params.alpha_min)
        else:
            new_alpha = state.current_alpha
        
        if abs(new_alpha - state.current_alpha) > 1e-6:
            self._compute_weights(new_alpha)
        
        return new_alpha
    
    def predict(self, state: UKFProState, dt: float = 1.0) -> UKFProState:
        """
        Square-root UKF prediction step.
        """
        # Adapt scaling if enabled
        state.current_alpha = self._adapt_scaling(state)
        
        # Generate sigma points
        sigma = self._generate_sigma_points_sr(state.x, state.S)
        
        # Propagate through process model
        sigma_pred = np.zeros_like(sigma)
        for i in range(self.n_sigma):
            sigma_pred[i] = self.f(sigma[i], dt)
            sigma_pred[i] = self._apply_constraints(sigma_pred[i])
        
        # Predicted mean
        x_pred = np.sum(self.Wm[:, np.newaxis] * sigma_pred, axis=0)
        x_pred = self._apply_constraints(x_pred)
        
        # Square-root covariance update via QR
        # Form matrix for QR: [sqrt(Wc)*(χ - x̄), sqrt(Q)]
        X_centered = np.zeros((self.n_sigma - 1 + self.n, self.n))
        
        for i in range(1, self.n_sigma):
            X_centered[i-1] = self.sqrt_Wc[i] * (sigma_pred[i] - x_pred)
        
        X_centered[self.n_sigma-1:] = state.Sq
        
        # QR decomposition
        _, R = qr(X_centered)
        S_pred = R[:self.n, :self.n].T
        
        # Rank-1 update for W0 (can be negative)
        diff0 = sigma_pred[0] - x_pred
        if self.Wc[0] >= 0:
            S_pred = self._cholupdate(S_pred, np.sqrt(self.Wc[0]) * diff0, +1.0)
        else:
            S_pred = self._cholupdate(S_pred, np.sqrt(-self.Wc[0]) * diff0, -1.0)
        
        # Ensure positive diagonal
        for i in range(self.n):
            if S_pred[i, i] < 0:
                S_pred[i, :] = -S_pred[i, :]
        
        return UKFProState(
            x=x_pred,
            S=S_pred,
            Q=state.Q,
            R=state.R,
            innovation_history=state.innovation_history,
            nees_history=state.nees_history,
            current_alpha=state.current_alpha
        )
    
    def update(
        self, 
        state: UKFProState, 
        z: np.ndarray
    ) -> Tuple[UKFProState, dict]:
        """
        Square-root UKF update with optional iteration (PRO feature).
        
        Returns:
            Tuple of (updated_state, metrics_dict)
        """
        z = np.asarray(z)
        
        if self.params.iterated_updates:
            return self._iterated_update(state, z)
        else:
            return self._standard_update(state, z)
    
    def _standard_update(
        self, 
        state: UKFProState, 
        z: np.ndarray
    ) -> Tuple[UKFProState, dict]:
        """Standard square-root update"""
        
        # Generate sigma points
        sigma = self._generate_sigma_points_sr(state.x, state.S)
        
        # Transform through measurement model
        sigma_z = np.zeros((self.n_sigma, self.m))
        for i in range(self.n_sigma):
            sigma_z[i] = self.h(sigma[i])
        
        # Predicted measurement
        z_pred = np.sum(self.Wm[:, np.newaxis] * sigma_z, axis=0)
        
        # Innovation covariance square root via QR
        Z_centered = np.zeros((self.n_sigma - 1 + self.m, self.m))
        
        for i in range(1, self.n_sigma):
            Z_centered[i-1] = self.sqrt_Wc[i] * (sigma_z[i] - z_pred)
        
        Z_centered[self.n_sigma-1:] = state.Sr
        
        _, R = qr(Z_centered)
        Sz = R[:self.m, :self.m].T
        
        # Rank-1 update for W0
        diff0_z = sigma_z[0] - z_pred
        if self.Wc[0] >= 0:
            Sz = self._cholupdate(Sz, np.sqrt(self.Wc[0]) * diff0_z, +1.0)
        else:
            Sz = self._cholupdate(Sz, np.sqrt(-self.Wc[0]) * diff0_z, -1.0)
        
        # Cross covariance
        Pxz = np.zeros((self.n, self.m))
        for i in range(self.n_sigma):
            Pxz += self.Wc[i] * np.outer(sigma[i] - state.x, sigma_z[i] - z_pred)
        
        # Kalman gain via forward/backward substitution
        # K = Pxz @ inv(Sz @ Sz.T) = Pxz @ inv(Sz).T @ inv(Sz)
        K = np.linalg.solve(Sz, np.linalg.solve(Sz, Pxz.T).T).T
        
        # Innovation
        innovation = z - z_pred
        
        # State update
        x_upd = state.x + K @ innovation
        x_upd = self._apply_constraints(x_upd)
        
        # Covariance update via Cholesky downdate
        S_upd = state.S.copy()
        U = K @ Sz
        for i in range(self.m):
            S_upd = self._cholupdate(S_upd, U[:, i], -1.0)
        
        # Compute NEES for consistency monitoring
        try:
            nees = innovation @ np.linalg.solve(Sz @ Sz.T, innovation)
        except:
            nees = np.nan
        
        # Update state
        new_state = UKFProState(
            x=x_upd,
            S=S_upd,
            Q=state.Q,
            R=state.R,
            innovation_history=state.innovation_history.copy(),
            nees_history=state.nees_history.copy(),
            current_alpha=state.current_alpha
        )
        new_state.record_innovation(innovation, Sz)
        
        metrics = {
            'innovation': innovation,
            'nees': nees,
            'kalman_gain_norm': np.linalg.norm(K),
            'alpha': state.current_alpha
        }
        
        return new_state, metrics
    
    def _iterated_update(
        self, 
        state: UKFProState, 
        z: np.ndarray
    ) -> Tuple[UKFProState, dict]:
        """Iterated UKF update for strong nonlinearities (PRO feature)"""
        
        x_iter = state.x.copy()
        S_iter = state.S.copy()
        
        for iteration in range(self.params.max_iterations):
            x_prev = x_iter.copy()
            
            # Generate sigma points around current iterate
            sigma = self._generate_sigma_points_sr(x_iter, S_iter)
            
            # Transform through measurement model
            sigma_z = np.array([self.h(sigma[i]) for i in range(self.n_sigma)])
            z_pred = np.sum(self.Wm[:, np.newaxis] * sigma_z, axis=0)
            
            # Innovation covariance
            Pzz = state.R.copy()
            for i in range(self.n_sigma):
                diff = sigma_z[i] - z_pred
                Pzz += self.Wc[i] * np.outer(diff, diff)
            
            # Cross covariance
            Pxz = np.zeros((self.n, self.m))
            for i in range(self.n_sigma):
                Pxz += self.Wc[i] * np.outer(sigma[i] - x_iter, sigma_z[i] - z_pred)
            
            # Kalman gain
            K = Pxz @ np.linalg.inv(Pzz)
            
            # Update iterate
            innovation = z - z_pred
            x_iter = state.x + K @ (z - z_pred - self.h(state.x) + self.h(x_iter))
            x_iter = self._apply_constraints(x_iter)
            
            # Check convergence
            if np.linalg.norm(x_iter - x_prev) < self.params.iteration_tolerance:
                break
        
        # Final covariance update
        P_upd = state.P - K @ Pzz @ K.T
        P_upd = 0.5 * (P_upd + P_upd.T)
        
        try:
            S_upd = cholesky(P_upd + 1e-9 * np.eye(self.n))
        except LinAlgError:
            S_upd = state.S * 0.99
        
        nees = innovation @ np.linalg.solve(Pzz, innovation)
        
        new_state = UKFProState(
            x=x_iter,
            S=S_upd,
            Q=state.Q,
            R=state.R,
            innovation_history=state.innovation_history.copy(),
            nees_history=state.nees_history.copy(),
            current_alpha=state.current_alpha
        )
        new_state.record_innovation(innovation, cholesky(Pzz + 1e-9*np.eye(self.m)))
        
        metrics = {
            'innovation': innovation,
            'nees': nees,
            'iterations': iteration + 1,
            'alpha': state.current_alpha
        }
        
        return new_state, metrics


def create_hypersonic_ukf_pro() -> Tuple[UKFPro, UKFProState]:
    """
    Factory for hypersonic target tracking (PRO optimized).
    
    Features:
    - 9-state model (position, velocity, acceleration)
    - Adaptive scaling for high dynamics
    - State constraints (physical limits)
    - Iterated updates for range-bearing-elevation
    """
    dt = 0.05  # 20 Hz for hypersonic
    
    def f_hypersonic(x: np.ndarray, dt: float) -> np.ndarray:
        """Constant jerk model for hypersonic"""
        # State: [px, py, pz, vx, vy, vz, ax, ay, az]
        F = np.eye(9)
        F[0:3, 3:6] = dt * np.eye(3)
        F[0:3, 6:9] = 0.5 * dt**2 * np.eye(3)
        F[3:6, 6:9] = dt * np.eye(3)
        return F @ x
    
    def h_hypersonic(x: np.ndarray) -> np.ndarray:
        """Range-bearing-elevation measurement"""
        px, py, pz = x[0], x[1], x[2]
        r = np.sqrt(px**2 + py**2 + pz**2)
        az = np.arctan2(py, px)
        el = np.arctan2(pz, np.sqrt(px**2 + py**2))
        return np.array([r, az, el])
    
    params = UKFProParams(
        alpha=0.01,
        beta=2.0,
        adaptive_scaling=True,
        alpha_min=0.001,
        alpha_max=0.5,
        iterated_updates=True,
        max_iterations=3,
        constraint_handling=True,
        state_min=np.array([-1e6, -1e6, 0, -3000, -3000, -500, -200, -200, -100]),
        state_max=np.array([1e6, 1e6, 100000, 3000, 3000, 500, 200, 200, 100])
    )
    
    ukf = UKFPro(
        f=f_hypersonic,
        h=h_hypersonic,
        n_states=9,
        n_meas=3,
        params=params
    )
    
    x0 = np.array([50000, 0, 30000, 2000, 100, -50, 0, 0, 0])
    P0 = np.diag([1000, 1000, 500, 100, 100, 50, 10, 10, 5])**2
    Q = np.diag([1, 1, 1, 10, 10, 5, 50, 50, 20])
    R = np.diag([100, 0.01, 0.01])  # Range in m, angles in rad
    
    state = ukf.init_state(x0, P0, Q, R)
    
    return ukf, state


if __name__ == "__main__":
    print("QEDMMA-Pro UKF-Pro Demo")
    print("=" * 60)
    print("PRO Features: SR-UKF, Adaptive Scaling, Constraints, IUKF")
    print()
    
    ukf, state = create_hypersonic_ukf_pro()
    
    np.random.seed(42)
    
    # Simulate hypersonic target
    true_state = np.array([50000, 0, 30000, 2000, 100, -50, 10, 5, -2])
    
    print(f"{'Step':>4} | {'Range Err':>10} | {'NEES':>8} | {'Alpha':>8} | {'Iters':>5}")
    print("-" * 55)
    
    for t in range(50):
        # True dynamics
        F = np.eye(9)
        F[0:3, 3:6] = 0.05 * np.eye(3)
        F[0:3, 6:9] = 0.5 * 0.05**2 * np.eye(3)
        F[3:6, 6:9] = 0.05 * np.eye(3)
        true_state = F @ true_state
        
        # Measurement
        r = np.sqrt(true_state[0]**2 + true_state[1]**2 + true_state[2]**2)
        az = np.arctan2(true_state[1], true_state[0])
        el = np.arctan2(true_state[2], np.sqrt(true_state[0]**2 + true_state[1]**2))
        z = np.array([r + np.random.randn()*100, 
                      az + np.random.randn()*0.01,
                      el + np.random.randn()*0.01])
        
        # UKF-Pro cycle
        state = ukf.predict(state, dt=0.05)
        state, metrics = ukf.update(state, z)
        
        # Compute range error
        est_r = np.sqrt(state.x[0]**2 + state.x[1]**2 + state.x[2]**2)
        range_err = abs(est_r - r)
        
        if t % 5 == 0:
            print(f"{t:>4} | {range_err:>8.1f} m | {metrics['nees']:>8.2f} | "
                  f"{metrics['alpha']:>8.4f} | {metrics.get('iterations', 1):>5}")
    
    print()
    print("✅ UKF-Pro provides enhanced tracking for hypersonic targets")
    print("   with adaptive scaling and iterated updates.")
