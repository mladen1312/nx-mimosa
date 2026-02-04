#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════════════════════════════
NX-MIMOSA Particle Filter Module
═══════════════════════════════════════════════════════════════════════════════════════════════════════

Sequential Monte Carlo (Particle Filter) for:
- Non-Gaussian noise distributions
- Multimodal posterior (multiple hypotheses)
- Severe nonlinearities
- Bearing-only tracking

Author: Dr. Mladen Mešter / Nexellum d.o.o.
Version: 1.1.0
License: AGPL v3 / Commercial
═══════════════════════════════════════════════════════════════════════════════════════════════════════
"""

import numpy as np
from typing import Tuple, Optional, Callable, List
from dataclasses import dataclass
from enum import IntEnum


# =============================================================================
# CONFIGURATION
# =============================================================================

@dataclass
class ParticleFilterConfig:
    """Particle filter configuration."""
    n_particles: int = 1000
    resample_threshold: float = 0.5  # Effective sample size ratio
    process_noise_std: np.ndarray = None
    roughening: bool = True
    roughening_factor: float = 0.01


class ResamplingMethod(IntEnum):
    """Resampling algorithms."""
    MULTINOMIAL = 0
    SYSTEMATIC = 1
    STRATIFIED = 2
    RESIDUAL = 3


# =============================================================================
# PARTICLE FILTER
# =============================================================================

class ParticleFilter:
    """
    Sequential Monte Carlo (Particle Filter) implementation.
    
    State: [x, y, z, vx, vy, vz]
    
    [REQ-PF-001] Non-Gaussian tracking
    [REQ-PF-002] Multimodal posterior
    [REQ-PF-003] Bearing-only capability
    """
    
    def __init__(self, config: ParticleFilterConfig = None):
        """Initialize particle filter."""
        self.config = config or ParticleFilterConfig()
        self.n_particles = self.config.n_particles
        
        # Particles and weights
        self.particles = None  # (n_particles, 6)
        self.weights = None    # (n_particles,)
        
        # State dimension
        self.dim_x = 6
        
        # Process noise
        if self.config.process_noise_std is None:
            self.q_std = np.array([10.0, 10.0, 10.0, 5.0, 5.0, 5.0])
        else:
            self.q_std = self.config.process_noise_std
        
        # Resampling method
        self.resample_method = ResamplingMethod.SYSTEMATIC
    
    def initialize(self, x0: np.ndarray, P0: np.ndarray):
        """
        Initialize particles from Gaussian distribution.
        
        Args:
            x0: Mean state
            P0: Covariance matrix
        """
        x0 = np.asarray(x0)
        P0 = np.asarray(P0)
        
        # Sample particles from Gaussian
        self.particles = np.random.multivariate_normal(x0, P0, self.n_particles)
        self.weights = np.ones(self.n_particles) / self.n_particles
    
    def initialize_uniform(self, bounds: List[Tuple[float, float]]):
        """
        Initialize particles uniformly within bounds.
        
        Args:
            bounds: List of (min, max) for each state dimension
        """
        self.particles = np.zeros((self.n_particles, self.dim_x))
        
        for i, (lo, hi) in enumerate(bounds):
            self.particles[:, i] = np.random.uniform(lo, hi, self.n_particles)
        
        self.weights = np.ones(self.n_particles) / self.n_particles
    
    def predict(self, dt: float, u: np.ndarray = None):
        """
        Predict particles forward using motion model.
        
        Args:
            dt: Time step
            u: Control input (optional)
        """
        # Constant velocity motion model
        F = np.eye(6)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        
        # Apply motion model to all particles
        self.particles = (F @ self.particles.T).T
        
        # Add process noise
        noise = np.random.randn(self.n_particles, 6) * self.q_std * np.sqrt(dt)
        self.particles += noise
    
    def update(self, z: np.ndarray, h: Callable, R: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Update weights based on measurement likelihood.
        
        Args:
            z: Measurement
            h: Measurement function h(x) -> z_pred
            R: Measurement noise covariance
        
        Returns:
            (mean_state, covariance)
        """
        z = np.asarray(z)
        R = np.asarray(R)
        
        # Compute likelihoods
        R_inv = np.linalg.inv(R)
        R_det = np.linalg.det(R)
        dim_z = len(z)
        norm_const = 1.0 / np.sqrt((2 * np.pi) ** dim_z * R_det)
        
        likelihoods = np.zeros(self.n_particles)
        for i in range(self.n_particles):
            z_pred = h(self.particles[i])
            innovation = z - z_pred
            
            # Handle angle wrapping if needed
            for j in range(len(innovation)):
                if abs(innovation[j]) > np.pi:
                    innovation[j] = np.arctan2(np.sin(innovation[j]), np.cos(innovation[j]))
            
            mahal = innovation.T @ R_inv @ innovation
            likelihoods[i] = norm_const * np.exp(-0.5 * mahal)
        
        # Update weights
        self.weights *= likelihoods
        
        # Normalize
        weight_sum = np.sum(self.weights)
        if weight_sum > 1e-20:
            self.weights /= weight_sum
        else:
            # All weights near zero - reset to uniform
            self.weights = np.ones(self.n_particles) / self.n_particles
        
        # Check if resampling needed
        n_eff = self.effective_sample_size()
        if n_eff < self.config.resample_threshold * self.n_particles:
            self._resample()
        
        return self.get_state(), self.get_covariance()
    
    def update_cartesian(self, z: np.ndarray, R: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Update with Cartesian position measurement."""
        def h(x):
            return x[:3]
        return self.update(z, h, R)
    
    def update_bearing_only(self, azimuth: float, sigma_az: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Update with bearing-only measurement.
        
        Args:
            azimuth: Measured azimuth in radians
            sigma_az: Azimuth standard deviation
        """
        def h(x):
            return np.array([np.arctan2(x[0], x[1])])  # atan2(East, North)
        
        R = np.array([[sigma_az ** 2]])
        return self.update(np.array([azimuth]), h, R)
    
    def update_range_azimuth(self, r: float, az: float, 
                              sigma_r: float, sigma_az: float) -> Tuple[np.ndarray, np.ndarray]:
        """Update with range-azimuth measurement."""
        def h(x):
            range_pred = np.sqrt(x[0]**2 + x[1]**2)
            az_pred = np.arctan2(x[0], x[1])
            return np.array([range_pred, az_pred])
        
        R = np.diag([sigma_r**2, sigma_az**2])
        return self.update(np.array([r, az]), h, R)
    
    def _resample(self):
        """Resample particles based on current method."""
        if self.resample_method == ResamplingMethod.SYSTEMATIC:
            indices = self._systematic_resample()
        elif self.resample_method == ResamplingMethod.STRATIFIED:
            indices = self._stratified_resample()
        elif self.resample_method == ResamplingMethod.RESIDUAL:
            indices = self._residual_resample()
        else:
            indices = self._multinomial_resample()
        
        self.particles = self.particles[indices]
        self.weights = np.ones(self.n_particles) / self.n_particles
        
        # Roughening to prevent particle degeneracy
        if self.config.roughening:
            self._roughen()
    
    def _multinomial_resample(self) -> np.ndarray:
        """Standard multinomial resampling."""
        cumsum = np.cumsum(self.weights)
        cumsum[-1] = 1.0  # Ensure last element is exactly 1
        
        u = np.random.rand(self.n_particles)
        indices = np.searchsorted(cumsum, u)
        
        return indices
    
    def _systematic_resample(self) -> np.ndarray:
        """Low-variance systematic resampling."""
        cumsum = np.cumsum(self.weights)
        cumsum[-1] = 1.0
        
        u0 = np.random.rand() / self.n_particles
        u = u0 + np.arange(self.n_particles) / self.n_particles
        
        indices = np.searchsorted(cumsum, u)
        
        return indices
    
    def _stratified_resample(self) -> np.ndarray:
        """Stratified resampling."""
        cumsum = np.cumsum(self.weights)
        cumsum[-1] = 1.0
        
        u = (np.arange(self.n_particles) + np.random.rand(self.n_particles)) / self.n_particles
        indices = np.searchsorted(cumsum, u)
        
        return indices
    
    def _residual_resample(self) -> np.ndarray:
        """Residual resampling."""
        n = self.n_particles
        
        # Deterministic part
        n_copies = (n * self.weights).astype(int)
        indices = np.repeat(np.arange(n), n_copies)
        
        # Residual part
        n_residual = n - len(indices)
        if n_residual > 0:
            residual_weights = (n * self.weights) - n_copies
            residual_weights /= residual_weights.sum()
            
            cumsum = np.cumsum(residual_weights)
            u = np.random.rand(n_residual)
            residual_indices = np.searchsorted(cumsum, u)
            
            indices = np.concatenate([indices, residual_indices])
        
        return indices
    
    def _roughen(self):
        """Add small noise to prevent particle collapse."""
        # Compute particle spread
        K = self.config.roughening_factor
        
        for d in range(self.dim_x):
            sigma = K * np.std(self.particles[:, d])
            self.particles[:, d] += np.random.randn(self.n_particles) * sigma
    
    def effective_sample_size(self) -> float:
        """Compute effective sample size."""
        return 1.0 / np.sum(self.weights ** 2)
    
    def get_state(self) -> np.ndarray:
        """Get weighted mean state estimate."""
        return np.sum(self.particles * self.weights[:, np.newaxis], axis=0)
    
    def get_covariance(self) -> np.ndarray:
        """Get weighted covariance estimate."""
        mean = self.get_state()
        diff = self.particles - mean
        P = np.zeros((self.dim_x, self.dim_x))
        
        for i in range(self.n_particles):
            P += self.weights[i] * np.outer(diff[i], diff[i])
        
        return P
    
    def get_map_estimate(self) -> np.ndarray:
        """Get Maximum A Posteriori (highest weight) particle."""
        idx = np.argmax(self.weights)
        return self.particles[idx].copy()
    
    def get_particles(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get particles and weights."""
        return self.particles.copy(), self.weights.copy()


# =============================================================================
# REGULARIZED PARTICLE FILTER (RPF)
# =============================================================================

class RegularizedParticleFilter(ParticleFilter):
    """
    Regularized Particle Filter with kernel smoothing.
    
    Reduces particle degeneracy through continuous resampling.
    """
    
    def __init__(self, config: ParticleFilterConfig = None):
        super().__init__(config)
        self.kernel_bandwidth = None
    
    def _resample(self):
        """Resample with kernel regularization."""
        # Standard resample first
        indices = self._systematic_resample()
        self.particles = self.particles[indices]
        
        # Compute optimal bandwidth (Silverman's rule)
        if self.kernel_bandwidth is None:
            std = np.std(self.particles, axis=0)
            self.kernel_bandwidth = std * (4 / (self.dim_x + 2) / self.n_particles) ** (1 / (self.dim_x + 4))
        
        # Add kernel noise
        noise = np.random.randn(self.n_particles, self.dim_x) * self.kernel_bandwidth
        self.particles += noise
        
        # Reset weights
        self.weights = np.ones(self.n_particles) / self.n_particles


# =============================================================================
# AUXILIARY PARTICLE FILTER (APF)
# =============================================================================

class AuxiliaryParticleFilter(ParticleFilter):
    """
    Auxiliary Particle Filter for improved proposal.
    
    Uses lookahead to focus particles on high-likelihood regions.
    """
    
    def update(self, z: np.ndarray, h: Callable, R: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Update with auxiliary variable."""
        z = np.asarray(z)
        R = np.asarray(R)
        
        R_inv = np.linalg.inv(R)
        R_det = np.linalg.det(R)
        dim_z = len(z)
        norm_const = 1.0 / np.sqrt((2 * np.pi) ** dim_z * R_det)
        
        # First stage: Compute auxiliary weights using predicted means
        aux_weights = np.zeros(self.n_particles)
        for i in range(self.n_particles):
            z_pred = h(self.particles[i])
            innovation = z - z_pred
            mahal = innovation.T @ R_inv @ innovation
            aux_weights[i] = self.weights[i] * norm_const * np.exp(-0.5 * mahal)
        
        # Normalize auxiliary weights
        aux_sum = np.sum(aux_weights)
        if aux_sum > 1e-20:
            aux_weights /= aux_sum
        else:
            aux_weights = np.ones(self.n_particles) / self.n_particles
        
        # Resample based on auxiliary weights
        cumsum = np.cumsum(aux_weights)
        u0 = np.random.rand() / self.n_particles
        u = u0 + np.arange(self.n_particles) / self.n_particles
        indices = np.searchsorted(cumsum, u)
        
        self.particles = self.particles[indices]
        
        # Second stage: Compute final weights
        final_weights = np.zeros(self.n_particles)
        for i in range(self.n_particles):
            z_pred = h(self.particles[i])
            innovation = z - z_pred
            mahal = innovation.T @ R_inv @ innovation
            likelihood = norm_const * np.exp(-0.5 * mahal)
            
            # Divide by auxiliary weight to correct
            if aux_weights[indices[i]] > 1e-20:
                final_weights[i] = likelihood / (aux_weights[indices[i]] * self.n_particles)
            else:
                final_weights[i] = likelihood
        
        # Normalize
        self.weights = final_weights / np.sum(final_weights)
        
        return self.get_state(), self.get_covariance()


# =============================================================================
# MAIN / DEMO
# =============================================================================

if __name__ == "__main__":
    print("NX-MIMOSA Particle Filter v1.1.0")
    print("=" * 60)
    
    # Demo: Bearing-only tracking (classic PF problem)
    config = ParticleFilterConfig(
        n_particles=2000,
        resample_threshold=0.5,
        roughening=True
    )
    
    pf = ParticleFilter(config)
    
    # True target: starts at (5000, 5000), moving North
    true_state = np.array([5000.0, 5000.0, 0.0, 0.0, 100.0, 0.0])
    
    # Initialize with uncertainty
    x0 = np.array([5000.0, 5000.0, 0.0, 0.0, 100.0, 0.0])
    P0 = np.diag([1000**2, 1000**2, 100**2, 50**2, 50**2, 10**2])
    pf.initialize(x0, P0)
    
    print(f"Initial ESS: {pf.effective_sample_size():.0f}")
    
    # Track with bearing-only measurements
    dt = 1.0
    sigma_az = np.radians(1.0)  # 1 degree accuracy
    
    errors = []
    
    for step in range(20):
        # True dynamics
        true_state[:3] += true_state[3:6] * dt
        
        # Predict
        pf.predict(dt)
        
        # Generate bearing measurement
        true_az = np.arctan2(true_state[0], true_state[1])
        measured_az = true_az + np.random.randn() * sigma_az
        
        # Update
        state, P = pf.update_bearing_only(measured_az, sigma_az)
        
        # Error
        error = np.linalg.norm(state[:2] - true_state[:2])
        errors.append(error)
        
        ess = pf.effective_sample_size()
        print(f"Step {step+1:2d}: Error = {error:7.1f} m, ESS = {ess:.0f}")
    
    print(f"\nFinal position error: {errors[-1]:.1f} m")
    print(f"Mean error: {np.mean(errors):.1f} m")
    
    print("\n✅ Particle Filter Demo Complete")
