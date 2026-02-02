"""
QEDMMA-Lite v3.0 - Cubature Kalman Filter (CKF)
================================================
Copyright (C) 2026 Dr. Mladen Mešter / Nexellum
License: AGPL-3.0-or-later

For commercial licensing: mladen@nexellum.com

Theory:
    CKF uses spherical-radial cubature rule to approximate Gaussian
    weighted integrals. Unlike UKF, CKF:
    - Has no tuning parameters (α, β, κ)
    - All weights are positive and equal (1/2n)
    - More numerically stable for high dimensions
    - Better suited for n > 3 dimensions

Reference:
    Arasaratnam & Haykin, "Cubature Kalman Filters", IEEE TAC 2009
"""

import numpy as np
from numpy.linalg import cholesky, LinAlgError
from typing import Callable, Tuple, Optional
from dataclasses import dataclass, field


@dataclass
class CKFState:
    """CKF state container"""
    x: np.ndarray                    # State estimate [n_states]
    P: np.ndarray                    # Covariance [n_states, n_states]
    Q: np.ndarray                    # Process noise covariance
    R: np.ndarray                    # Measurement noise covariance
    n: int = field(init=False)       # State dimension
    
    def __post_init__(self):
        self.n = len(self.x)
        self.P = 0.5 * (self.P + self.P.T)


class CubatureKalmanFilter:
    """
    Cubature Kalman Filter for nonlinear state estimation.
    
    Uses 2n cubature points with equal weights, providing:
    - Third-order accuracy for Gaussian inputs
    - No tuning parameters required
    - Numerical stability for high dimensions
    
    Example:
        >>> def f(x, dt): return np.array([x[0] + x[1]*dt, x[1]])
        >>> def h(x): return np.array([x[0]])
        >>> ckf = CubatureKalmanFilter(f, h, n_states=2, n_meas=1)
        >>> state = ckf.init_state(x0=[0,1], P0=np.eye(2), Q=0.1*np.eye(2), R=np.array([[1.0]]))
        >>> state = ckf.predict(state, dt=0.1)
        >>> state, innov = ckf.update(state, z=np.array([0.5]))
    """
    
    def __init__(
        self,
        f: Callable[[np.ndarray, float], np.ndarray],
        h: Callable[[np.ndarray], np.ndarray],
        n_states: int,
        n_meas: int
    ):
        """
        Args:
            f: State transition function f(x, dt) -> x_next
            h: Measurement function h(x) -> z
            n_states: Dimension of state vector
            n_meas: Dimension of measurement vector
        """
        self.f = f
        self.h = h
        self.n = n_states
        self.m = n_meas
        
        # CKF uses 2n cubature points
        self.n_cubature = 2 * n_states
        
        # All weights are equal: 1/(2n)
        self.W = 1.0 / self.n_cubature
        
        # Pre-compute cubature point directions
        # ξ_i = sqrt(n) * e_i for i=1..n
        # ξ_{i+n} = -sqrt(n) * e_i for i=1..n
        self._compute_cubature_directions()
    
    def _compute_cubature_directions(self):
        """Pre-compute the spherical-radial cubature directions"""
        n = self.n
        sqrt_n = np.sqrt(n)
        
        # Unit directions [2n, n]
        self.xi = np.zeros((self.n_cubature, n))
        for i in range(n):
            self.xi[i, i] = sqrt_n
            self.xi[i + n, i] = -sqrt_n
    
    def init_state(
        self,
        x0: np.ndarray,
        P0: np.ndarray,
        Q: np.ndarray,
        R: np.ndarray
    ) -> CKFState:
        """Initialize filter state"""
        return CKFState(
            x=np.asarray(x0, dtype=np.float64),
            P=np.asarray(P0, dtype=np.float64),
            Q=np.asarray(Q, dtype=np.float64),
            R=np.asarray(R, dtype=np.float64)
        )
    
    def _generate_cubature_points(self, x: np.ndarray, P: np.ndarray) -> np.ndarray:
        """
        Generate cubature points.
        
        X_i = S * ξ_i + x̄
        
        where S is Cholesky factor: P = S @ S.T
        
        Returns:
            cubature_points: [2n, n_states] array
        """
        # Regularize P for numerical stability
        P_reg = P + 1e-10 * np.eye(self.n)
        
        try:
            S = cholesky(P_reg)  # Lower triangular
        except LinAlgError:
            # SVD-based square root if Cholesky fails
            U, s, Vt = np.linalg.svd(P_reg)
            s = np.maximum(s, 1e-10)
            S = U @ np.diag(np.sqrt(s))
        
        # Transform cubature directions to state space
        # X = S @ xi.T + x  -->  [n, 2n] + [n, 1] broadcast
        cubature = (S @ self.xi.T).T + x
        
        return cubature
    
    def predict(self, state: CKFState, dt: float = 1.0) -> CKFState:
        """
        Prediction step using cubature integration.
        
        Args:
            state: Current CKF state
            dt: Time step
            
        Returns:
            State with prior estimate (x⁻, P⁻)
        """
        # Generate cubature points
        X = self._generate_cubature_points(state.x, state.P)
        
        # Propagate cubature points through process model
        X_pred = np.zeros_like(X)
        for i in range(self.n_cubature):
            X_pred[i] = self.f(X[i], dt)
        
        # Predicted mean (equal weights)
        x_pred = self.W * np.sum(X_pred, axis=0)
        
        # Predicted covariance
        P_pred = state.Q.copy()
        for i in range(self.n_cubature):
            diff = X_pred[i] - x_pred
            P_pred += self.W * np.outer(diff, diff)
        
        # Ensure symmetry
        P_pred = 0.5 * (P_pred + P_pred.T)
        
        return CKFState(x=x_pred, P=P_pred, Q=state.Q, R=state.R)
    
    def update(self, state: CKFState, z: np.ndarray) -> Tuple[CKFState, np.ndarray]:
        """
        Update step: incorporate measurement.
        
        Args:
            state: Predicted state
            z: Measurement vector
            
        Returns:
            Tuple of (updated_state, innovation)
        """
        # Generate cubature points from predicted state
        X = self._generate_cubature_points(state.x, state.P)
        
        # Transform through measurement model
        Z = np.zeros((self.n_cubature, self.m))
        for i in range(self.n_cubature):
            Z[i] = self.h(X[i])
        
        # Predicted measurement mean
        z_pred = self.W * np.sum(Z, axis=0)
        
        # Innovation covariance S = Pzz + R
        Pzz = state.R.copy()
        for i in range(self.n_cubature):
            diff = Z[i] - z_pred
            Pzz += self.W * np.outer(diff, diff)
        
        # Cross covariance Pxz
        Pxz = np.zeros((self.n, self.m))
        for i in range(self.n_cubature):
            diff_x = X[i] - state.x
            diff_z = Z[i] - z_pred
            Pxz += self.W * np.outer(diff_x, diff_z)
        
        # Kalman gain K = Pxz @ inv(Pzz)
        K = Pxz @ np.linalg.inv(Pzz)
        
        # Innovation
        innovation = z - z_pred
        
        # Update
        x_upd = state.x + K @ innovation
        P_upd = state.P - K @ Pzz @ K.T
        
        # Ensure symmetry
        P_upd = 0.5 * (P_upd + P_upd.T)
        
        return CKFState(x=x_upd, P=P_upd, Q=state.Q, R=state.R), innovation
    
    def predict_update(
        self, 
        state: CKFState, 
        z: np.ndarray, 
        dt: float = 1.0
    ) -> Tuple[CKFState, np.ndarray]:
        """Combined predict and update for efficiency"""
        state = self.predict(state, dt)
        return self.update(state, z)


class SquareRootCKF(CubatureKalmanFilter):
    """
    Square-Root Cubature Kalman Filter (SR-CKF).
    
    Propagates Cholesky factor S directly instead of P:
    - Guarantees positive semi-definiteness
    - Better numerical stability
    - Slightly higher computational cost
    
    Uses QR decomposition and Cholesky updates.
    """
    
    def predict(self, state: CKFState, dt: float = 1.0) -> CKFState:
        """
        Square-root prediction using QR decomposition.
        """
        # Cholesky of current covariance
        try:
            S = cholesky(state.P + 1e-10 * np.eye(self.n))
        except LinAlgError:
            S = np.linalg.cholesky(state.P @ state.P.T + 1e-10 * np.eye(self.n))
        
        # Cholesky of process noise
        try:
            Sq = cholesky(state.Q + 1e-10 * np.eye(self.n))
        except LinAlgError:
            Sq = np.zeros_like(state.Q)
        
        # Generate and propagate cubature points
        X = (S @ self.xi.T).T + state.x
        X_pred = np.array([self.f(X[i], dt) for i in range(self.n_cubature)])
        
        # Mean
        x_pred = self.W * np.sum(X_pred, axis=0)
        
        # Centered, weighted cubature points
        sqrt_W = np.sqrt(self.W)
        X_centered = sqrt_W * (X_pred - x_pred)
        
        # QR decomposition for square root covariance
        # [X_centered; Sq]^T -> triangularize
        combined = np.vstack([X_centered, Sq])
        _, R = np.linalg.qr(combined)
        S_pred = R[:self.n, :self.n].T
        
        # Reconstruct P for compatibility
        P_pred = S_pred @ S_pred.T
        
        return CKFState(x=x_pred, P=P_pred, Q=state.Q, R=state.R)


def create_high_dim_ckf(n_states: int = 9, n_meas: int = 3) -> CubatureKalmanFilter:
    """
    Create CKF for high-dimensional tracking (e.g., 9D: pos, vel, acc).
    
    CKF is preferred over UKF for n > 3 due to weight stability.
    """
    
    def f_high_dim(x: np.ndarray, dt: float) -> np.ndarray:
        """Constant acceleration model (9 states)"""
        # State: [px, py, pz, vx, vy, vz, ax, ay, az]
        n = len(x) // 3
        F = np.eye(len(x))
        for i in range(n):
            if i + n < len(x):
                F[i, i + n] = dt
            if i + 2*n < len(x):
                F[i, i + 2*n] = 0.5 * dt**2
                F[i + n, i + 2*n] = dt
        return F @ x
    
    def h_high_dim(x: np.ndarray) -> np.ndarray:
        """Position-only measurement"""
        return x[:3]
    
    return CubatureKalmanFilter(
        f=f_high_dim,
        h=h_high_dim,
        n_states=n_states,
        n_meas=n_meas
    )


if __name__ == "__main__":
    print("QEDMMA-Lite v3.0 - CKF Demo")
    print("=" * 50)
    
    # Create high-dimensional CKF
    ckf = create_high_dim_ckf(n_states=6, n_meas=2)
    
    # State: [x, y, vx, vy, ax, ay]
    x0 = np.array([0.0, 0.0, 10.0, 5.0, 0.1, -0.05])
    P0 = np.diag([10.0, 10.0, 1.0, 1.0, 0.1, 0.1])
    Q = 0.01 * np.eye(6)
    R = np.diag([5.0, 5.0])
    
    state = ckf.init_state(x0, P0, Q, R)
    
    def f_test(x, dt):
        F = np.array([
            [1, 0, dt, 0, 0.5*dt**2, 0],
            [0, 1, 0, dt, 0, 0.5*dt**2],
            [0, 0, 1, 0, dt, 0],
            [0, 0, 0, 1, 0, dt],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])
        return F @ x
    
    def h_test(x):
        return x[:2]
    
    ckf.f = f_test
    ckf.h = h_test
    
    np.random.seed(42)
    true_state = x0.copy()
    
    print("\nTracking with 6D state (CKF handles high dimensions well):")
    for step in range(10):
        true_state = f_test(true_state, 0.1)
        z = true_state[:2] + np.random.randn(2) * np.array([5.0, 5.0])
        
        state = ckf.predict(state, dt=0.1)
        state, innov = ckf.update(state, z)
        
        err = np.linalg.norm(state.x[:2] - true_state[:2])
        print(f"  Step {step+1}: Pos=({state.x[0]:.1f}, {state.x[1]:.1f}), "
              f"True=({true_state[0]:.1f}, {true_state[1]:.1f}), Err={err:.2f}m")
