"""
QEDMMA-Lite v3.0 - Unscented Kalman Filter (UKF)
================================================
Copyright (C) 2026 Dr. Mladen Mešter / Nexellum
License: AGPL-3.0-or-later

For commercial licensing: mladen@nexellum.com

Theory:
    UKF uses sigma points to capture mean and covariance through
    nonlinear transformations without explicit Jacobian calculation.
    
    Key advantages over EKF:
    - No Jacobian required
    - Captures higher-order moments
    - Better accuracy for highly nonlinear systems
"""

import numpy as np
from numpy.linalg import cholesky, LinAlgError
from typing import Callable, Tuple, Optional
from dataclasses import dataclass, field


@dataclass
class UKFParams:
    """UKF tuning parameters (Van der Merwe formulation)"""
    alpha: float = 1e-3  # Spread of sigma points (1e-4 to 1)
    beta: float = 2.0    # Prior knowledge (2 optimal for Gaussian)
    kappa: float = 0.0   # Secondary scaling (usually 0 or 3-n)


@dataclass
class UKFState:
    """UKF state container"""
    x: np.ndarray                    # State estimate [n_states]
    P: np.ndarray                    # Covariance [n_states, n_states]
    Q: np.ndarray                    # Process noise covariance
    R: np.ndarray                    # Measurement noise covariance
    n: int = field(init=False)       # State dimension
    
    def __post_init__(self):
        self.n = len(self.x)
        # Ensure symmetric positive definite
        self.P = 0.5 * (self.P + self.P.T)
        

class UnscentedKalmanFilter:
    """
    Unscented Kalman Filter for nonlinear state estimation.
    
    Implements the scaled unscented transform (Van der Merwe, 2004).
    
    Example:
        >>> def f(x, dt): return np.array([x[0] + x[1]*dt, x[1]])  # CV model
        >>> def h(x): return np.array([x[0]])  # Position only
        >>> ukf = UnscentedKalmanFilter(f, h, n_states=2, n_meas=1)
        >>> state = ukf.init_state(x0=[0, 1], P0=np.eye(2), Q=0.1*np.eye(2), R=np.array([[1.0]]))
        >>> state = ukf.predict(state, dt=0.1)
        >>> state = ukf.update(state, z=np.array([0.5]))
    """
    
    def __init__(
        self,
        f: Callable[[np.ndarray, float], np.ndarray],
        h: Callable[[np.ndarray], np.ndarray],
        n_states: int,
        n_meas: int,
        params: Optional[UKFParams] = None
    ):
        """
        Args:
            f: State transition function f(x, dt) -> x_next
            h: Measurement function h(x) -> z
            n_states: Dimension of state vector
            n_meas: Dimension of measurement vector
            params: UKF tuning parameters
        """
        self.f = f
        self.h = h
        self.n = n_states
        self.m = n_meas
        self.params = params or UKFParams()
        
        # Compute derived parameters
        self._compute_weights()
    
    def _compute_weights(self):
        """Compute sigma point weights"""
        n = self.n
        alpha = self.params.alpha
        beta = self.params.beta
        kappa = self.params.kappa
        
        # Lambda parameter
        self.lambda_ = alpha**2 * (n + kappa) - n
        
        # Number of sigma points
        self.n_sigma = 2 * n + 1
        
        # Weights for mean
        self.Wm = np.zeros(self.n_sigma)
        self.Wm[0] = self.lambda_ / (n + self.lambda_)
        self.Wm[1:] = 1.0 / (2 * (n + self.lambda_))
        
        # Weights for covariance
        self.Wc = np.zeros(self.n_sigma)
        self.Wc[0] = self.Wm[0] + (1 - alpha**2 + beta)
        self.Wc[1:] = self.Wm[1:]
    
    def init_state(
        self,
        x0: np.ndarray,
        P0: np.ndarray,
        Q: np.ndarray,
        R: np.ndarray
    ) -> UKFState:
        """Initialize filter state"""
        return UKFState(
            x=np.asarray(x0, dtype=np.float64),
            P=np.asarray(P0, dtype=np.float64),
            Q=np.asarray(Q, dtype=np.float64),
            R=np.asarray(R, dtype=np.float64)
        )
    
    def _generate_sigma_points(self, x: np.ndarray, P: np.ndarray) -> np.ndarray:
        """
        Generate sigma points using scaled unscented transform.
        
        Returns:
            sigma_points: [n_sigma, n_states] array
        """
        n = self.n
        sigma = np.zeros((self.n_sigma, n))
        
        # Ensure P is positive definite with regularization
        P_reg = P + 1e-9 * np.eye(n)
        
        try:
            # Cholesky decomposition: P = L @ L.T
            sqrt_P = cholesky((n + self.lambda_) * P_reg)
        except LinAlgError:
            # Fallback to eigenvalue decomposition if Cholesky fails
            eigvals, eigvecs = np.linalg.eigh(P_reg)
            eigvals = np.maximum(eigvals, 1e-10)
            sqrt_P = eigvecs @ np.diag(np.sqrt((n + self.lambda_) * eigvals))
        
        # Central point
        sigma[0] = x
        
        # Spread points
        for i in range(n):
            sigma[i + 1] = x + sqrt_P[i]
            sigma[i + 1 + n] = x - sqrt_P[i]
        
        return sigma
    
    def predict(self, state: UKFState, dt: float = 1.0) -> UKFState:
        """
        Prediction step: propagate sigma points through process model.
        
        Args:
            state: Current UKF state
            dt: Time step
            
        Returns:
            Updated state with prior estimate
        """
        # Generate sigma points
        sigma = self._generate_sigma_points(state.x, state.P)
        
        # Propagate through process model
        sigma_pred = np.zeros_like(sigma)
        for i in range(self.n_sigma):
            sigma_pred[i] = self.f(sigma[i], dt)
        
        # Predicted mean
        x_pred = np.sum(self.Wm[:, np.newaxis] * sigma_pred, axis=0)
        
        # Predicted covariance
        P_pred = state.Q.copy()
        for i in range(self.n_sigma):
            diff = sigma_pred[i] - x_pred
            P_pred += self.Wc[i] * np.outer(diff, diff)
        
        # Ensure symmetry
        P_pred = 0.5 * (P_pred + P_pred.T)
        
        return UKFState(x=x_pred, P=P_pred, Q=state.Q, R=state.R)
    
    def update(self, state: UKFState, z: np.ndarray) -> Tuple[UKFState, np.ndarray]:
        """
        Update step: incorporate measurement.
        
        Args:
            state: Predicted state
            z: Measurement vector
            
        Returns:
            Tuple of (updated_state, innovation)
        """
        # Generate sigma points from predicted state
        sigma = self._generate_sigma_points(state.x, state.P)
        
        # Transform sigma points through measurement model
        sigma_z = np.zeros((self.n_sigma, self.m))
        for i in range(self.n_sigma):
            sigma_z[i] = self.h(sigma[i])
        
        # Predicted measurement mean
        z_pred = np.sum(self.Wm[:, np.newaxis] * sigma_z, axis=0)
        
        # Innovation covariance
        Pzz = state.R.copy()
        for i in range(self.n_sigma):
            diff_z = sigma_z[i] - z_pred
            Pzz += self.Wc[i] * np.outer(diff_z, diff_z)
        
        # Cross covariance
        Pxz = np.zeros((self.n, self.m))
        for i in range(self.n_sigma):
            diff_x = sigma[i] - state.x
            diff_z = sigma_z[i] - z_pred
            Pxz += self.Wc[i] * np.outer(diff_x, diff_z)
        
        # Kalman gain
        K = Pxz @ np.linalg.inv(Pzz)
        
        # Innovation
        innovation = z - z_pred
        
        # Update state
        x_upd = state.x + K @ innovation
        P_upd = state.P - K @ Pzz @ K.T
        
        # Ensure symmetry
        P_upd = 0.5 * (P_upd + P_upd.T)
        
        return UKFState(x=x_upd, P=P_upd, Q=state.Q, R=state.R), innovation


# Convenience function for radar tracking
def create_radar_ukf(
    dt: float = 0.1,
    process_noise: float = 0.1,
    measurement_noise_range: float = 10.0,
    measurement_noise_bearing: float = 0.01
) -> Tuple[UnscentedKalmanFilter, UKFState]:
    """
    Create UKF configured for radar tracking (range + bearing).
    
    State: [x, y, vx, vy]
    Measurement: [range, bearing]
    """
    
    def f_radar(x: np.ndarray, dt: float) -> np.ndarray:
        """Constant velocity model"""
        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        return F @ x
    
    def h_radar(x: np.ndarray) -> np.ndarray:
        """Cartesian to polar (range, bearing)"""
        px, py = x[0], x[1]
        r = np.sqrt(px**2 + py**2)
        theta = np.arctan2(py, px)
        return np.array([r, theta])
    
    ukf = UnscentedKalmanFilter(
        f=f_radar,
        h=h_radar,
        n_states=4,
        n_meas=2,
        params=UKFParams(alpha=0.1, beta=2.0, kappa=0.0)
    )
    
    # Initial state
    x0 = np.array([1000.0, 1000.0, 10.0, 5.0])
    P0 = np.diag([100.0, 100.0, 10.0, 10.0])
    Q = process_noise * np.array([
        [dt**4/4, 0, dt**3/2, 0],
        [0, dt**4/4, 0, dt**3/2],
        [dt**3/2, 0, dt**2, 0],
        [0, dt**3/2, 0, dt**2]
    ])
    R = np.diag([measurement_noise_range**2, measurement_noise_bearing**2])
    
    state = ukf.init_state(x0, P0, Q, R)
    
    return ukf, state


if __name__ == "__main__":
    # Demo
    print("QEDMMA-Lite v3.0 - UKF Demo")
    print("=" * 50)
    
    ukf, state = create_radar_ukf()
    
    # Simulate target
    np.random.seed(42)
    true_pos = np.array([1000.0, 1000.0])
    true_vel = np.array([10.0, 5.0])
    
    for step in range(10):
        # True position
        true_pos += true_vel * 0.1
        
        # Noisy measurement
        r = np.sqrt(true_pos[0]**2 + true_pos[1]**2)
        theta = np.arctan2(true_pos[1], true_pos[0])
        z = np.array([r + np.random.randn()*10, theta + np.random.randn()*0.01])
        
        # UKF cycle
        state = ukf.predict(state, dt=0.1)
        state, innov = ukf.update(state, z)
        
        print(f"Step {step+1}: Est=({state.x[0]:.1f}, {state.x[1]:.1f}), "
              f"True=({true_pos[0]:.1f}, {true_pos[1]:.1f}), "
              f"Err={np.linalg.norm(state.x[:2] - true_pos):.2f}m")
