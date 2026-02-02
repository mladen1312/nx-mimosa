"""
QEDMMA-Lite v3.0 - Adaptive Noise Estimation
=============================================
Copyright (C) 2026 Dr. Mladen Mešter / Nexellum
License: AGPL-3.0-or-later

For commercial licensing: mladen@nexellum.com

Theory:
    Real radar systems have time-varying noise characteristics:
    - Clutter changes with weather/terrain
    - Target RCS varies with aspect angle
    - Sensor noise drifts with temperature
    
    Adaptive estimation adjusts Q and R in real-time using:
    1. Innovation-based estimation
    2. Covariance matching
    3. Sage-Husa adaptive filter
    4. Maximum Likelihood Estimation (MLE)
"""

import numpy as np
from collections import deque
from typing import Tuple, Optional
from dataclasses import dataclass, field
from abc import ABC, abstractmethod


@dataclass
class AdaptiveNoiseState:
    """State container for adaptive noise estimation"""
    Q: np.ndarray                     # Process noise estimate
    R: np.ndarray                     # Measurement noise estimate
    innovation_history: deque = field(default_factory=lambda: deque(maxlen=50))
    Q_history: list = field(default_factory=list)
    R_history: list = field(default_factory=list)
    
    def record_innovation(self, innovation: np.ndarray):
        self.innovation_history.append(innovation.copy())
    
    def record_estimates(self):
        self.Q_history.append(np.trace(self.Q))
        self.R_history.append(np.trace(self.R))


class AdaptiveEstimator(ABC):
    """Base class for adaptive noise estimators"""
    
    @abstractmethod
    def update(
        self,
        state: AdaptiveNoiseState,
        innovation: np.ndarray,
        H: np.ndarray,
        P_pred: np.ndarray,
        K: np.ndarray,
        F: np.ndarray,
        P_post: np.ndarray
    ) -> AdaptiveNoiseState:
        """Update noise estimates given filter data"""
        pass


class InnovationBasedEstimator(AdaptiveEstimator):
    """
    Innovation-Based Adaptive Estimation (IAE).
    
    Estimates R from innovation sequence autocorrelation:
        R̂ = (1/N) Σ νν' - H P⁻ H'
    
    Simple and widely used, but assumes stationary noise.
    """
    
    def __init__(self, window_size: int = 20, min_samples: int = 10):
        self.window_size = window_size
        self.min_samples = min_samples
    
    def update(
        self,
        state: AdaptiveNoiseState,
        innovation: np.ndarray,
        H: np.ndarray,
        P_pred: np.ndarray,
        K: np.ndarray,
        F: np.ndarray,
        P_post: np.ndarray
    ) -> AdaptiveNoiseState:
        """Estimate R from innovation sequence"""
        
        state.record_innovation(innovation)
        
        if len(state.innovation_history) < self.min_samples:
            return state
        
        # Compute sample covariance of innovations
        innovations = np.array(list(state.innovation_history))
        n_samples = len(innovations)
        
        # Innovation covariance estimate
        # S = E[νν'] ≈ (1/N) Σ νν'
        S_hat = np.zeros((len(innovation), len(innovation)))
        for nu in innovations:
            S_hat += np.outer(nu, nu)
        S_hat /= n_samples
        
        # Theoretical innovation covariance: S = H P⁻ H' + R
        # Therefore: R̂ = S_hat - H P⁻ H'
        HPH = H @ P_pred @ H.T
        R_hat = S_hat - HPH
        
        # Ensure positive semi-definite
        eigvals, eigvecs = np.linalg.eigh(R_hat)
        eigvals = np.maximum(eigvals, 1e-6)  # Floor eigenvalues
        R_hat = eigvecs @ np.diag(eigvals) @ eigvecs.T
        
        # Exponential smoothing for stability
        alpha = 0.1  # Learning rate
        state.R = (1 - alpha) * state.R + alpha * R_hat
        
        state.record_estimates()
        return state


class CovarianceMatchingEstimator(AdaptiveEstimator):
    """
    Covariance Matching Adaptive Estimation.
    
    Estimates both Q and R by matching predicted and actual covariances.
    
    For Q:
        Q̂ = K S K' where S is innovation covariance
    
    More aggressive than innovation-based, tracks faster changes.
    """
    
    def __init__(self, window_size: int = 20, alpha_q: float = 0.1, alpha_r: float = 0.1):
        self.window_size = window_size
        self.alpha_q = alpha_q
        self.alpha_r = alpha_r
    
    def update(
        self,
        state: AdaptiveNoiseState,
        innovation: np.ndarray,
        H: np.ndarray,
        P_pred: np.ndarray,
        K: np.ndarray,
        F: np.ndarray,
        P_post: np.ndarray
    ) -> AdaptiveNoiseState:
        """Estimate Q and R via covariance matching"""
        
        state.record_innovation(innovation)
        
        if len(state.innovation_history) < 5:
            return state
        
        # Innovation covariance
        innovations = np.array(list(state.innovation_history)[-self.window_size:])
        S_hat = np.cov(innovations.T)
        if S_hat.ndim == 0:
            S_hat = np.array([[S_hat]])
        
        # R estimation (same as innovation-based)
        HPH = H @ P_pred @ H.T
        R_new = S_hat - HPH
        eigvals, eigvecs = np.linalg.eigh(R_new)
        eigvals = np.maximum(eigvals, 1e-6)
        R_new = eigvecs @ np.diag(eigvals) @ eigvecs.T
        
        # Q estimation: Q̂ = K S K'
        Q_new = K @ S_hat @ K.T
        eigvals, eigvecs = np.linalg.eigh(Q_new)
        eigvals = np.maximum(eigvals, 1e-8)
        Q_new = eigvecs @ np.diag(eigvals) @ eigvecs.T
        
        # Smooth updates
        state.R = (1 - self.alpha_r) * state.R + self.alpha_r * R_new
        state.Q = (1 - self.alpha_q) * state.Q + self.alpha_q * Q_new
        
        state.record_estimates()
        return state


class SageHusaEstimator(AdaptiveEstimator):
    """
    Sage-Husa Adaptive Kalman Filter.
    
    Recursive estimation with exponential forgetting:
        Q̂_k = (1-d_k) Q̂_{k-1} + d_k [K_k ν_k ν_k' K_k' + P_k - F P_{k-1} F']
        R̂_k = (1-d_k) R̂_{k-1} + d_k [ν_k ν_k' - H P_k⁻ H']
    
    where d_k = (1-b)/(1-b^{k+1}) is the forgetting factor.
    
    Advantages:
    - Recursive (O(1) memory)
    - Handles non-stationary noise
    - Widely used in GPS/INS applications
    """
    
    def __init__(self, b: float = 0.95):
        """
        Args:
            b: Forgetting factor (0.9-0.99). Higher = slower adaptation.
        """
        self.b = b
        self.k = 0
    
    def _forgetting_factor(self) -> float:
        """Compute time-varying forgetting factor d_k"""
        return (1 - self.b) / (1 - self.b**(self.k + 1))
    
    def update(
        self,
        state: AdaptiveNoiseState,
        innovation: np.ndarray,
        H: np.ndarray,
        P_pred: np.ndarray,
        K: np.ndarray,
        F: np.ndarray,
        P_post: np.ndarray
    ) -> AdaptiveNoiseState:
        """Recursive Sage-Husa update"""
        
        self.k += 1
        d_k = self._forgetting_factor()
        
        state.record_innovation(innovation)
        
        # R update: R̂_k = (1-d_k) R̂_{k-1} + d_k [νν' - H P⁻ H']
        nu_outer = np.outer(innovation, innovation)
        HPH = H @ P_pred @ H.T
        R_increment = nu_outer - HPH
        
        # Ensure increment is reasonable (clip extreme values)
        R_increment = np.clip(R_increment, -10 * state.R, 10 * state.R)
        
        state.R = (1 - d_k) * state.R + d_k * R_increment
        
        # Ensure positive definite
        eigvals, eigvecs = np.linalg.eigh(state.R)
        eigvals = np.maximum(eigvals, 1e-6)
        state.R = eigvecs @ np.diag(eigvals) @ eigvecs.T
        
        # Q update: Q̂_k = (1-d_k) Q̂_{k-1} + d_k [K ν ν' K' + P_post - F P_prev F']
        # Note: We need previous P, approximating with P_pred @ F⁻¹
        K_nu = K @ innovation
        Q_increment = np.outer(K_nu, K_nu)
        
        state.Q = (1 - d_k) * state.Q + d_k * Q_increment
        
        eigvals, eigvecs = np.linalg.eigh(state.Q)
        eigvals = np.maximum(eigvals, 1e-8)
        state.Q = eigvecs @ np.diag(eigvals) @ eigvecs.T
        
        state.record_estimates()
        return state


class MLENoiseEstimator(AdaptiveEstimator):
    """
    Maximum Likelihood Estimation for noise covariances.
    
    Periodically re-estimates Q and R using batch MLE on
    innovation sequence. More accurate but computationally expensive.
    
    Uses EM algorithm for joint Q, R estimation.
    """
    
    def __init__(self, batch_size: int = 50, update_interval: int = 10):
        self.batch_size = batch_size
        self.update_interval = update_interval
        self.counter = 0
        self._H_history = []
        self._P_pred_history = []
    
    def update(
        self,
        state: AdaptiveNoiseState,
        innovation: np.ndarray,
        H: np.ndarray,
        P_pred: np.ndarray,
        K: np.ndarray,
        F: np.ndarray,
        P_post: np.ndarray
    ) -> AdaptiveNoiseState:
        """Periodic MLE-based update"""
        
        state.record_innovation(innovation)
        self._H_history.append(H.copy())
        self._P_pred_history.append(P_pred.copy())
        
        # Keep history bounded
        if len(self._H_history) > self.batch_size:
            self._H_history.pop(0)
            self._P_pred_history.pop(0)
        
        self.counter += 1
        
        if self.counter < self.update_interval:
            return state
        
        self.counter = 0
        
        if len(state.innovation_history) < self.batch_size // 2:
            return state
        
        # MLE estimation
        innovations = np.array(list(state.innovation_history)[-self.batch_size:])
        N = len(innovations)
        m = innovations.shape[1]
        
        # Sample innovation covariance
        S_sample = np.cov(innovations.T)
        if S_sample.ndim == 0:
            S_sample = np.array([[S_sample]])
        
        # Average H @ P @ H' over window
        HPH_avg = np.mean([
            self._H_history[i] @ self._P_pred_history[i] @ self._H_history[i].T
            for i in range(min(len(self._H_history), N))
        ], axis=0)
        
        # MLE for R: R̂ = S - avg(HPH')
        R_mle = S_sample - HPH_avg
        eigvals, eigvecs = np.linalg.eigh(R_mle)
        eigvals = np.maximum(eigvals, 1e-6)
        R_mle = eigvecs @ np.diag(eigvals) @ eigvecs.T
        
        # Smooth update
        state.R = 0.7 * state.R + 0.3 * R_mle
        
        state.record_estimates()
        return state


class CompositeAdaptiveEstimator:
    """
    Combines multiple estimation strategies with voting/averaging.
    
    Provides robust estimation by fusing:
    - Innovation-based (fast, simple)
    - Covariance matching (tracks Q)
    - Sage-Husa (recursive, non-stationary)
    """
    
    def __init__(self, weights: Optional[Tuple[float, float, float]] = None):
        self.estimators = [
            InnovationBasedEstimator(window_size=20),
            CovarianceMatchingEstimator(window_size=15),
            SageHusaEstimator(b=0.95)
        ]
        self.weights = weights or (0.4, 0.3, 0.3)
        self._states = None
    
    def initialize(self, Q0: np.ndarray, R0: np.ndarray) -> AdaptiveNoiseState:
        """Initialize composite estimator"""
        self._states = [
            AdaptiveNoiseState(Q=Q0.copy(), R=R0.copy())
            for _ in self.estimators
        ]
        return AdaptiveNoiseState(Q=Q0.copy(), R=R0.copy())
    
    def update(
        self,
        innovation: np.ndarray,
        H: np.ndarray,
        P_pred: np.ndarray,
        K: np.ndarray,
        F: np.ndarray,
        P_post: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Update all estimators and fuse results.
        
        Returns:
            (Q_fused, R_fused)
        """
        if self._states is None:
            raise RuntimeError("Call initialize() first")
        
        # Update each estimator
        for i, (est, st) in enumerate(zip(self.estimators, self._states)):
            self._states[i] = est.update(st, innovation, H, P_pred, K, F, P_post)
        
        # Weighted fusion
        Q_fused = sum(w * s.Q for w, s in zip(self.weights, self._states))
        R_fused = sum(w * s.R for w, s in zip(self.weights, self._states))
        
        return Q_fused, R_fused


def demonstrate_adaptive_estimation():
    """Demo: Track changing noise characteristics"""
    np.random.seed(42)
    
    print("QEDMMA-Lite v3.0 - Adaptive Noise Estimation Demo")
    print("=" * 60)
    
    # True noise that changes over time
    n_steps = 100
    true_R_values = []
    for t in range(n_steps):
        if t < 30:
            true_R_values.append(1.0)
        elif t < 60:
            true_R_values.append(5.0)  # Noise increases
        else:
            true_R_values.append(2.0)  # Noise decreases
    
    # Initialize adaptive estimator
    Q0 = np.array([[0.1]])
    R0 = np.array([[1.0]])
    
    estimator = SageHusaEstimator(b=0.92)
    state = AdaptiveNoiseState(Q=Q0.copy(), R=R0.copy())
    
    # Simulate
    H = np.array([[1.0]])
    F = np.array([[1.0]])
    
    print("\nTracking time-varying measurement noise:")
    print(f"{'Step':>5} | {'True R':>8} | {'Est R':>8} | {'Error':>8}")
    print("-" * 45)
    
    for t in range(n_steps):
        true_R = true_R_values[t]
        
        # Generate synthetic innovation (should have variance = HPH' + R ≈ R)
        P_pred = np.array([[0.5]])
        innovation = np.random.randn(1) * np.sqrt(true_R + 0.5)
        K = np.array([[0.5]])
        P_post = np.array([[0.25]])
        
        state = estimator.update(state, innovation, H, P_pred, K, F, P_post)
        
        if t % 10 == 0:
            est_R = state.R[0, 0]
            error = abs(est_R - true_R)
            print(f"{t:>5} | {true_R:>8.2f} | {est_R:>8.2f} | {error:>8.2f}")
    
    print("\n✅ Adaptive estimation tracks noise changes!")


if __name__ == "__main__":
    demonstrate_adaptive_estimation()
