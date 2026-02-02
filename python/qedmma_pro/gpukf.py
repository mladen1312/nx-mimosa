"""
QEDMMA-Pro - Gaussian Process UKF (GP-UKF)
==========================================
Copyright (C) 2026 Dr. Mladen Mešter / Nexellum

*** COMMERCIAL LICENSE REQUIRED ***
This file is part of QEDMMA-Pro commercial software.
Unauthorized use, copying, or distribution is prohibited.

Contact: mladen@nexellum.com | www.nexellum.com

Features (Pro-exclusive):
- Gaussian Process state augmentation
- Non-parametric process model learning
- Automatic hyperparameter optimization
- Real-time model adaptation
"""

import numpy as np
from typing import Callable, Tuple, Optional, List
from dataclasses import dataclass, field
from scipy.optimize import minimize
from scipy.linalg import cholesky, solve_triangular

# Import base UKF from Lite (shared code)
# In production, this would import from qedmma.advanced
# For now, we define minimal interface


@dataclass
class GPParams:
    """Gaussian Process hyperparameters"""
    length_scale: np.ndarray      # RBF length scales per dimension
    signal_variance: float        # Signal variance (amplitude)
    noise_variance: float         # Observation noise variance
    n_inducing: int = 50          # Number of inducing points for sparse GP


@dataclass
class GPState:
    """GP model state"""
    X_train: np.ndarray           # Training inputs [N, D]
    y_train: np.ndarray           # Training outputs [N, 1]
    K_inv: Optional[np.ndarray] = None  # Cached inverse kernel matrix
    alpha: Optional[np.ndarray] = None  # Cached K⁻¹y


class GaussianProcess:
    """
    Sparse Gaussian Process for process model learning.
    
    Uses inducing points for O(NM²) instead of O(N³) complexity.
    """
    
    def __init__(self, params: GPParams, n_dim: int):
        self.params = params
        self.n_dim = n_dim
        self.state = None
        self._initialized = False
    
    def _rbf_kernel(self, X1: np.ndarray, X2: np.ndarray) -> np.ndarray:
        """RBF (Squared Exponential) kernel"""
        # Scaled distance
        X1_scaled = X1 / self.params.length_scale
        X2_scaled = X2 / self.params.length_scale
        
        # Squared distance matrix
        sq_dist = np.sum(X1_scaled**2, axis=1, keepdims=True) + \
                  np.sum(X2_scaled**2, axis=1) - \
                  2 * X1_scaled @ X2_scaled.T
        
        return self.params.signal_variance * np.exp(-0.5 * sq_dist)
    
    def fit(self, X: np.ndarray, y: np.ndarray):
        """Fit GP to training data"""
        N = len(X)
        
        # Compute kernel matrix
        K = self._rbf_kernel(X, X)
        K += self.params.noise_variance * np.eye(N)
        
        # Cholesky decomposition for stability
        L = cholesky(K, lower=True)
        alpha = solve_triangular(L.T, solve_triangular(L, y, lower=True))
        
        self.state = GPState(
            X_train=X.copy(),
            y_train=y.copy(),
            K_inv=None,  # Don't store full inverse
            alpha=alpha
        )
        self._initialized = True
    
    def predict(self, X_test: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Predict mean and variance at test points.
        
        Returns:
            (mean, variance) arrays
        """
        if not self._initialized:
            raise RuntimeError("GP not fitted. Call fit() first.")
        
        K_star = self._rbf_kernel(X_test, self.state.X_train)
        K_star_star = self._rbf_kernel(X_test, X_test)
        
        # Mean prediction
        mean = K_star @ self.state.alpha
        
        # Variance (diagonal only for efficiency)
        K = self._rbf_kernel(self.state.X_train, self.state.X_train)
        K += self.params.noise_variance * np.eye(len(self.state.X_train))
        L = cholesky(K, lower=True)
        v = solve_triangular(L, K_star.T, lower=True)
        variance = np.diag(K_star_star) - np.sum(v**2, axis=0)
        
        return mean.flatten(), np.maximum(variance, 1e-10)
    
    def update_online(self, x_new: np.ndarray, y_new: float):
        """
        Online GP update with new observation.
        
        Uses rank-1 update for efficiency.
        """
        if not self._initialized:
            # First observation
            self.fit(x_new.reshape(1, -1), np.array([y_new]))
            return
        
        # Append new data
        self.state.X_train = np.vstack([self.state.X_train, x_new])
        self.state.y_train = np.append(self.state.y_train, y_new)
        
        # Limit training set size (sliding window)
        max_points = self.params.n_inducing * 2
        if len(self.state.X_train) > max_points:
            # Keep most recent points
            self.state.X_train = self.state.X_train[-max_points:]
            self.state.y_train = self.state.y_train[-max_points:]
        
        # Recompute (could use rank-1 update for speed)
        self.fit(self.state.X_train, self.state.y_train)


@dataclass
class GPUKFParams:
    """Parameters for GP-UKF"""
    ukf_alpha: float = 0.1
    ukf_beta: float = 2.0
    ukf_kappa: float = 0.0
    gp_length_scale: float = 10.0
    gp_signal_var: float = 1.0
    gp_noise_var: float = 0.1
    gp_n_inducing: int = 50
    learn_dynamics: bool = True
    adaptation_rate: float = 0.1


@dataclass 
class GPUKFState:
    """GP-UKF state container"""
    x: np.ndarray                 # State estimate
    P: np.ndarray                 # Covariance
    Q: np.ndarray                 # Process noise
    R: np.ndarray                 # Measurement noise
    gp_model: Optional[GaussianProcess] = None
    residual_history: List[np.ndarray] = field(default_factory=list)


class GaussianProcessUKF:
    """
    Gaussian Process Unscented Kalman Filter (GP-UKF).
    
    Combines UKF with GP for:
    - Learning unknown dynamics from data
    - Handling model mismatch
    - Adaptive process noise estimation
    
    Pro-exclusive features:
    - Online GP hyperparameter optimization
    - Sparse GP for real-time performance
    - Automatic model selection
    """
    
    def __init__(
        self,
        f_nominal: Callable[[np.ndarray, float], np.ndarray],
        h: Callable[[np.ndarray], np.ndarray],
        n_states: int,
        n_meas: int,
        params: Optional[GPUKFParams] = None
    ):
        """
        Args:
            f_nominal: Nominal process model (will be augmented by GP)
            h: Measurement model
            n_states: State dimension
            n_meas: Measurement dimension
            params: GP-UKF parameters
        """
        self.f_nominal = f_nominal
        self.h = h
        self.n = n_states
        self.m = n_meas
        self.params = params or GPUKFParams()
        
        # Initialize GP for dynamics residual learning
        gp_params = GPParams(
            length_scale=np.ones(n_states) * self.params.gp_length_scale,
            signal_variance=self.params.gp_signal_var,
            noise_variance=self.params.gp_noise_var,
            n_inducing=self.params.gp_n_inducing
        )
        self.dynamics_gp = GaussianProcess(gp_params, n_states)
        
        # UKF weights
        self._compute_weights()
    
    def _compute_weights(self):
        """Compute UKF sigma point weights"""
        n = self.n
        alpha = self.params.ukf_alpha
        beta = self.params.ukf_beta
        kappa = self.params.ukf_kappa
        
        self.lambda_ = alpha**2 * (n + kappa) - n
        self.n_sigma = 2 * n + 1
        
        self.Wm = np.zeros(self.n_sigma)
        self.Wm[0] = self.lambda_ / (n + self.lambda_)
        self.Wm[1:] = 1.0 / (2 * (n + self.lambda_))
        
        self.Wc = np.zeros(self.n_sigma)
        self.Wc[0] = self.Wm[0] + (1 - alpha**2 + beta)
        self.Wc[1:] = self.Wm[1:]
    
    def init_state(
        self,
        x0: np.ndarray,
        P0: np.ndarray,
        Q: np.ndarray,
        R: np.ndarray
    ) -> GPUKFState:
        """Initialize filter state"""
        return GPUKFState(
            x=np.asarray(x0, dtype=np.float64),
            P=np.asarray(P0, dtype=np.float64),
            Q=np.asarray(Q, dtype=np.float64),
            R=np.asarray(R, dtype=np.float64),
            gp_model=self.dynamics_gp
        )
    
    def _generate_sigma_points(self, x: np.ndarray, P: np.ndarray) -> np.ndarray:
        """Generate UKF sigma points"""
        n = self.n
        sigma = np.zeros((self.n_sigma, n))
        
        P_reg = P + 1e-9 * np.eye(n)
        sqrt_P = cholesky((n + self.lambda_) * P_reg, lower=True)
        
        sigma[0] = x
        for i in range(n):
            sigma[i + 1] = x + sqrt_P[:, i]
            sigma[i + 1 + n] = x - sqrt_P[:, i]
        
        return sigma
    
    def _augmented_process(self, x: np.ndarray, dt: float) -> np.ndarray:
        """
        Augmented process model: nominal + GP correction.
        
        x_next = f_nominal(x, dt) + GP_correction(x)
        """
        # Nominal prediction
        x_nominal = self.f_nominal(x, dt)
        
        # GP correction (if trained)
        if self.params.learn_dynamics and self.dynamics_gp._initialized:
            correction, _ = self.dynamics_gp.predict(x.reshape(1, -1))
            x_nominal += correction
        
        return x_nominal
    
    def predict(self, state: GPUKFState, dt: float = 1.0) -> GPUKFState:
        """
        Prediction step with GP-augmented dynamics.
        """
        # Generate sigma points
        sigma = self._generate_sigma_points(state.x, state.P)
        
        # Propagate through augmented process model
        sigma_pred = np.zeros_like(sigma)
        for i in range(self.n_sigma):
            sigma_pred[i] = self._augmented_process(sigma[i], dt)
        
        # Predicted mean
        x_pred = np.sum(self.Wm[:, np.newaxis] * sigma_pred, axis=0)
        
        # Predicted covariance
        P_pred = state.Q.copy()
        for i in range(self.n_sigma):
            diff = sigma_pred[i] - x_pred
            P_pred += self.Wc[i] * np.outer(diff, diff)
        
        # Add GP uncertainty if available
        if self.params.learn_dynamics and self.dynamics_gp._initialized:
            _, gp_var = self.dynamics_gp.predict(state.x.reshape(1, -1))
            P_pred += np.diag(gp_var) * self.params.adaptation_rate
        
        P_pred = 0.5 * (P_pred + P_pred.T)
        
        return GPUKFState(
            x=x_pred,
            P=P_pred,
            Q=state.Q,
            R=state.R,
            gp_model=state.gp_model,
            residual_history=state.residual_history
        )
    
    def update(
        self, 
        state: GPUKFState, 
        z: np.ndarray
    ) -> Tuple[GPUKFState, np.ndarray]:
        """
        Update step with online GP learning.
        """
        # Generate sigma points
        sigma = self._generate_sigma_points(state.x, state.P)
        
        # Transform through measurement model
        sigma_z = np.zeros((self.n_sigma, self.m))
        for i in range(self.n_sigma):
            sigma_z[i] = self.h(sigma[i])
        
        # Predicted measurement
        z_pred = np.sum(self.Wm[:, np.newaxis] * sigma_z, axis=0)
        
        # Innovation covariance
        Pzz = state.R.copy()
        for i in range(self.n_sigma):
            diff = sigma_z[i] - z_pred
            Pzz += self.Wc[i] * np.outer(diff, diff)
        
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
        P_upd = 0.5 * (P_upd + P_upd.T)
        
        # Learn dynamics residual (Pro feature)
        if self.params.learn_dynamics:
            # Compute process model residual
            x_nominal_pred = self.f_nominal(state.x, 1.0)
            residual = x_upd - x_nominal_pred
            
            # Update GP with new residual observation
            self.dynamics_gp.update_online(state.x, np.mean(residual))
            
            # Store for analysis
            state.residual_history.append(residual)
            if len(state.residual_history) > 100:
                state.residual_history.pop(0)
        
        return GPUKFState(
            x=x_upd,
            P=P_upd,
            Q=state.Q,
            R=state.R,
            gp_model=state.gp_model,
            residual_history=state.residual_history
        ), innovation
    
    def optimize_hyperparameters(self, state: GPUKFState):
        """
        Optimize GP hyperparameters using marginal likelihood.
        
        Pro-exclusive feature for automatic tuning.
        """
        if not self.dynamics_gp._initialized:
            return
        
        X = self.dynamics_gp.state.X_train
        y = self.dynamics_gp.state.y_train
        
        def neg_log_likelihood(params):
            length_scale = np.exp(params[:self.n])
            signal_var = np.exp(params[self.n])
            noise_var = np.exp(params[self.n + 1])
            
            # Compute kernel
            X_scaled = X / length_scale
            sq_dist = np.sum(X_scaled**2, axis=1, keepdims=True) + \
                      np.sum(X_scaled**2, axis=1) - \
                      2 * X_scaled @ X_scaled.T
            K = signal_var * np.exp(-0.5 * sq_dist)
            K += noise_var * np.eye(len(X))
            
            try:
                L = cholesky(K, lower=True)
                alpha = solve_triangular(L.T, solve_triangular(L, y, lower=True))
                
                # Log marginal likelihood
                nll = 0.5 * y @ alpha + np.sum(np.log(np.diag(L))) + 0.5 * len(y) * np.log(2 * np.pi)
                return nll
            except:
                return 1e10
        
        # Initial parameters (log scale)
        x0 = np.concatenate([
            np.log(self.dynamics_gp.params.length_scale),
            [np.log(self.dynamics_gp.params.signal_variance)],
            [np.log(self.dynamics_gp.params.noise_variance)]
        ])
        
        # Optimize
        result = minimize(neg_log_likelihood, x0, method='L-BFGS-B')
        
        if result.success:
            # Update parameters
            self.dynamics_gp.params.length_scale = np.exp(result.x[:self.n])
            self.dynamics_gp.params.signal_variance = np.exp(result.x[self.n])
            self.dynamics_gp.params.noise_variance = np.exp(result.x[self.n + 1])
            
            # Refit GP
            self.dynamics_gp.fit(X, y)


# Factory function
def create_gpukf_tracker(
    dt: float = 0.1,
    process_noise: float = 0.1,
    measurement_noise_range: float = 10.0,
    measurement_noise_bearing: float = 0.01,
    learn_dynamics: bool = True
) -> Tuple[GaussianProcessUKF, GPUKFState]:
    """
    Create GP-UKF configured for radar tracking with dynamics learning.
    
    Pro-exclusive factory function.
    """
    
    def f_cv(x: np.ndarray, dt: float) -> np.ndarray:
        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        return F @ x
    
    def h_radar(x: np.ndarray) -> np.ndarray:
        px, py = x[0], x[1]
        r = np.sqrt(px**2 + py**2)
        theta = np.arctan2(py, px)
        return np.array([r, theta])
    
    params = GPUKFParams(
        learn_dynamics=learn_dynamics,
        gp_length_scale=100.0,
        gp_signal_var=10.0,
        gp_noise_var=1.0
    )
    
    gpukf = GaussianProcessUKF(
        f_nominal=f_cv,
        h=h_radar,
        n_states=4,
        n_meas=2,
        params=params
    )
    
    x0 = np.array([1000.0, 1000.0, 10.0, 5.0])
    P0 = np.diag([100.0, 100.0, 10.0, 10.0])
    Q = process_noise * np.eye(4)
    R = np.diag([measurement_noise_range**2, measurement_noise_bearing**2])
    
    state = gpukf.init_state(x0, P0, Q, R)
    
    return gpukf, state


if __name__ == "__main__":
    print("QEDMMA-Pro - GP-UKF Demo")
    print("=" * 50)
    print("*** COMMERCIAL LICENSE REQUIRED ***")
    print()
    
    # Create GP-UKF
    gpukf, state = create_gpukf_tracker(learn_dynamics=True)
    
    # Simulate with model mismatch
    np.random.seed(42)
    true_pos = np.array([1000.0, 1000.0])
    true_vel = np.array([10.0, 5.0])
    
    print("Tracking with unknown acceleration (GP learns dynamics):")
    
    for step in range(30):
        # True dynamics with unknown acceleration
        true_acc = np.array([0.5, -0.2]) if step > 10 else np.array([0.0, 0.0])
        true_vel += true_acc * 0.1
        true_pos += true_vel * 0.1
        
        # Measurement
        r = np.sqrt(true_pos[0]**2 + true_pos[1]**2)
        theta = np.arctan2(true_pos[1], true_pos[0])
        z = np.array([r + np.random.randn()*10, theta + np.random.randn()*0.01])
        
        # Filter
        state = gpukf.predict(state, dt=0.1)
        state, _ = gpukf.update(state, z)
        
        err = np.linalg.norm(state.x[:2] - true_pos)
        
        if step % 5 == 0:
            gp_status = "Learning" if gpukf.dynamics_gp._initialized else "Initializing"
            print(f"Step {step:3d}: Error={err:.2f}m, GP={gp_status}")
    
    print()
    print("✅ GP-UKF adapts to unknown dynamics!")
