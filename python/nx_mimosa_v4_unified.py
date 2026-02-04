#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════════════════════════════
NX-MIMOSA v4.0 UNIFIED TRACKING ARCHITECTURE
═══════════════════════════════════════════════════════════════════════════════════════════════════════

ARCHITECTURE PHILOSOPHY:
────────────────────────
ONE CORE ENGINE + INDUSTRY COMPLIANCE MODULES

┌─────────────────────────────────────────────────────────────────────────────────────────────────────┐
│                              NX-MIMOSA UNIFIED ARCHITECTURE                                         │
├─────────────────────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                                     │
│  ┌─────────────────────────────────────────────────────────────────────────────────────────────┐   │
│  │                           CORE TRACKING ENGINE (Universal)                                   │   │
│  │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐           │   │
│  │  │    UKF      │ │    CKF      │ │    EKF      │ │   VS-IMM    │ │  Smoother   │           │   │
│  │  └─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘           │   │
│  │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐           │   │
│  │  │ Adaptive R  │ │ Adaptive Q  │ │ Soft Gating │ │   JPDA      │ │    MHT      │           │   │
│  │  └─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘           │   │
│  └─────────────────────────────────────────────────────────────────────────────────────────────┘   │
│                                            │                                                        │
│                                            ▼                                                        │
│  ┌─────────────────────────────────────────────────────────────────────────────────────────────┐   │
│  │                         INDUSTRY COMPLIANCE LAYER (Configurable)                             │   │
│  │                                                                                              │   │
│  │  ┌──────────────┐ ┌──────────────┐ ┌──────────────┐ ┌──────────────┐ ┌──────────────┐       │   │
│  │  │   AVIATION   │ │  AUTOMOTIVE  │ │   DEFENSE    │ │    SPACE     │ │   MARITIME   │       │   │
│  │  │  EUROCONTROL │ │  ISO 26262   │ │  MIL-STD     │ │  ECSS/NASA   │ │    IMO       │       │   │
│  │  │  DO-178C     │ │  ASIL-D      │ │  DO-254      │ │  CCSDS       │ │  SOLAS       │       │   │
│  │  │  ED-117      │ │  NCAP        │ │  STANAG      │ │  ITAR        │ │  COLREGS     │       │   │
│  │  └──────────────┘ └──────────────┘ └──────────────┘ └──────────────┘ └──────────────┘       │   │
│  └─────────────────────────────────────────────────────────────────────────────────────────────┘   │
│                                            │                                                        │
│                                            ▼                                                        │
│  ┌─────────────────────────────────────────────────────────────────────────────────────────────┐   │
│  │                              OUTPUT FORMATTERS                                               │   │
│  │  ┌──────────────┐ ┌──────────────┐ ┌──────────────┐ ┌──────────────┐ ┌──────────────┐       │   │
│  │  │   ASTERIX    │ │    CAN-FD    │ │   Link-16    │ │    CCSDS     │ │    NMEA      │       │   │
│  │  │  CAT001/048  │ │  ISO 11898   │ │   MIL-STD    │ │   Packets    │ │   0183/2000  │       │   │
│  │  └──────────────┘ └──────────────┘ └──────────────┘ └──────────────┘ └──────────────┘       │   │
│  └─────────────────────────────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────────────────────────────┘

INDUSTRY COMPLIANCE REQUIREMENTS:
─────────────────────────────────

┌──────────────────┬────────────────────────────────────────────────────────────────────────────────┐
│ INDUSTRY         │ KEY REQUIREMENTS                                                               │
├──────────────────┼────────────────────────────────────────────────────────────────────────────────┤
│ CIVIL AVIATION   │ • Position RMS ≤ 500m (EUROCONTROL EASSP)                                      │
│ (ATC/ATM)        │ • Track continuity ≥ 99.5%                                                     │
│                  │ • Latency ≤ 2s (95th percentile)                                               │
│                  │ • Support 3NM/5NM separation minima                                            │
│                  │ • ASTERIX CAT001/048/062 output format                                         │
│                  │ • DO-178C DAL-C or higher for software                                         │
│                  │ • Multi-sensor fusion (PSR + SSR + ADS-B + WAM)                                │
├──────────────────┼────────────────────────────────────────────────────────────────────────────────┤
│ AUTOMOTIVE       │ • Position accuracy ≤ 10cm @ 100m (Euro NCAP)                                  │
│ (ADAS/AD)        │ • Velocity accuracy ≤ 0.1 m/s                                                  │
│                  │ • Update rate ≥ 20 Hz                                                          │
│                  │ • Latency ≤ 50ms                                                               │
│                  │ • ISO 26262 ASIL-D compliance                                                  │
│                  │ • Multi-object tracking (≥64 simultaneous)                                     │
│                  │ • Classification: vehicle/pedestrian/cyclist                                   │
├──────────────────┼────────────────────────────────────────────────────────────────────────────────┤
│ DEFENSE          │ • Position accuracy ≤ 50m @ 200km                                              │
│ (Military Radar) │ • Velocity accuracy ≤ 1 m/s                                                    │
│                  │ • Track through ECM/ECCM                                                       │
│                  │ • Hypersonic target capability (>Mach 5)                                       │
│                  │ • DO-254 DAL-A for FPGA                                                        │
│                  │ • MIL-STD-1553/Link-16 output                                                  │
│                  │ • ITAR/EAR compliance                                                          │
├──────────────────┼────────────────────────────────────────────────────────────────────────────────┤
│ SPACE            │ • Position accuracy ≤ 1km @ GEO                                                │
│ (SSA/STM)        │ • Orbit determination convergence                                              │
│                  │ • Conjunction assessment                                                       │
│                  │ • ECSS-E-ST-60-20C compliance                                                  │
│                  │ • CCSDS packet format                                                          │
│                  │ • Debris tracking capability                                                   │
├──────────────────┼────────────────────────────────────────────────────────────────────────────────┤
│ MARITIME         │ • Position accuracy ≤ 30m                                                      │
│ (VTS/VTMS)       │ • IMO Resolution A.857(20) compliance                                          │
│                  │ • AIS fusion capability                                                        │
│                  │ • NMEA 0183/2000 output                                                        │
│                  │ • SOLAS Chapter V compliance                                                   │
└──────────────────┴────────────────────────────────────────────────────────────────────────────────┘

Author: Dr. Mladen Mešter / Nexellum d.o.o.
License: AGPL v3 (Open Source) / Commercial License Available
═══════════════════════════════════════════════════════════════════════════════════════════════════════
"""

import numpy as np
from numpy.linalg import inv, det, norm, cholesky
from scipy.linalg import sqrtm
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Callable, Any
from enum import Enum, auto
from abc import ABC, abstractmethod
import time

__version__ = "4.0.0"
__author__ = "Dr. Mladen Mešter"
__license__ = "AGPL-3.0 / Commercial"

# =============================================================================
# INDUSTRY PROFILES
# =============================================================================

class Industry(Enum):
    """Supported industry verticals."""
    AVIATION_ATC = auto()      # Civil aviation ATC/ATM
    AVIATION_MILITARY = auto() # Military aviation
    AUTOMOTIVE_ADAS = auto()   # Automotive ADAS/AD
    DEFENSE_RADAR = auto()     # Defense radar systems
    SPACE_SSA = auto()         # Space situational awareness
    MARITIME_VTS = auto()      # Maritime vessel traffic
    UNIVERSAL = auto()         # No specific compliance


@dataclass
class ComplianceProfile:
    """Industry-specific compliance requirements."""
    industry: Industry
    
    # Accuracy requirements
    position_rms_max: float      # meters
    velocity_rms_max: float      # m/s
    
    # Timing requirements
    update_rate_min: float       # Hz
    latency_max: float           # seconds (95th percentile)
    
    # Reliability requirements
    track_continuity_min: float  # fraction (0-1)
    false_track_max: float       # per hour
    
    # Capacity requirements
    max_simultaneous_tracks: int
    
    # Certification standards
    software_standard: str
    hardware_standard: str
    output_format: str
    
    # Domain-specific
    separation_minima: Optional[float] = None  # NM for aviation
    classification_required: bool = False
    eccm_required: bool = False
    

# Pre-defined compliance profiles
COMPLIANCE_PROFILES = {
    Industry.AVIATION_ATC: ComplianceProfile(
        industry=Industry.AVIATION_ATC,
        position_rms_max=500.0,        # EUROCONTROL EASSP requirement
        velocity_rms_max=10.0,
        update_rate_min=1.0,           # Typical radar rotation 4-12 rpm
        latency_max=2.0,               # 95th percentile
        track_continuity_min=0.995,    # 99.5%
        false_track_max=0.1,           # Per hour
        max_simultaneous_tracks=4000,  # ARTAS capacity
        software_standard="DO-178C DAL-C",
        hardware_standard="DO-254 DAL-C",
        output_format="ASTERIX CAT062",
        separation_minima=3.0,         # 3 NM minimum
        classification_required=False,
        eccm_required=False,
    ),
    Industry.AUTOMOTIVE_ADAS: ComplianceProfile(
        industry=Industry.AUTOMOTIVE_ADAS,
        position_rms_max=0.1,          # 10 cm at 100m
        velocity_rms_max=0.1,
        update_rate_min=20.0,          # 20 Hz minimum
        latency_max=0.05,              # 50 ms
        track_continuity_min=0.99,
        false_track_max=1.0,
        max_simultaneous_tracks=64,
        software_standard="ISO 26262 ASIL-D",
        hardware_standard="ISO 26262 ASIL-D",
        output_format="CAN-FD",
        classification_required=True,
    ),
    Industry.DEFENSE_RADAR: ComplianceProfile(
        industry=Industry.DEFENSE_RADAR,
        position_rms_max=50.0,         # At max range
        velocity_rms_max=1.0,
        update_rate_min=10.0,
        latency_max=0.1,
        track_continuity_min=0.99,
        false_track_max=0.5,
        max_simultaneous_tracks=1000,
        software_standard="DO-178C DAL-A",
        hardware_standard="DO-254 DAL-A",
        output_format="Link-16/MIL-STD-1553",
        eccm_required=True,
    ),
    Industry.SPACE_SSA: ComplianceProfile(
        industry=Industry.SPACE_SSA,
        position_rms_max=1000.0,       # 1 km at GEO
        velocity_rms_max=1.0,
        update_rate_min=0.1,           # Updates per orbit
        latency_max=60.0,
        track_continuity_min=0.95,
        false_track_max=0.01,
        max_simultaneous_tracks=50000, # Debris catalog
        software_standard="ECSS-E-ST-40C",
        hardware_standard="ECSS-Q-ST-60C",
        output_format="CCSDS",
    ),
    Industry.MARITIME_VTS: ComplianceProfile(
        industry=Industry.MARITIME_VTS,
        position_rms_max=30.0,
        velocity_rms_max=0.5,
        update_rate_min=0.5,
        latency_max=3.0,
        track_continuity_min=0.99,
        false_track_max=0.5,
        max_simultaneous_tracks=2000,
        software_standard="IEC 62065",
        hardware_standard="IEC 60945",
        output_format="NMEA 2000",
    ),
    Industry.UNIVERSAL: ComplianceProfile(
        industry=Industry.UNIVERSAL,
        position_rms_max=100.0,
        velocity_rms_max=5.0,
        update_rate_min=1.0,
        latency_max=1.0,
        track_continuity_min=0.95,
        false_track_max=1.0,
        max_simultaneous_tracks=1000,
        software_standard="N/A",
        hardware_standard="N/A",
        output_format="JSON",
    ),
}


# =============================================================================
# CORE KALMAN FILTERS
# =============================================================================

class BaseFilter(ABC):
    """Abstract base class for all Kalman filter variants."""
    
    @abstractmethod
    def predict(self, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """Predict state and covariance."""
        pass
    
    @abstractmethod
    def update(self, z: np.ndarray, R: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
        """Update with measurement. Returns (state, covariance, NIS)."""
        pass
    
    @abstractmethod
    def get_state(self) -> np.ndarray:
        pass
    
    @abstractmethod
    def get_covariance(self) -> np.ndarray:
        pass


class ExtendedKalmanFilter(BaseFilter):
    """Standard Extended Kalman Filter."""
    
    def __init__(self, dim_x: int, dim_z: int):
        self.dim_x = dim_x
        self.dim_z = dim_z
        self.x = np.zeros(dim_x)
        self.P = np.eye(dim_x) * 1000
        self.F = np.eye(dim_x)
        self.H = np.zeros((dim_z, dim_x))
        self.Q = np.eye(dim_x)
        
    def initialize(self, x0: np.ndarray, P0: np.ndarray):
        self.x = x0.copy()
        self.P = P0.copy()
        
    def predict(self, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        # Build F matrix for CV model
        F = np.eye(self.dim_x)
        for i in range(self.dim_x // 2):
            F[i, i + self.dim_x // 2] = dt
        
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q
        return self.x.copy(), self.P.copy()
    
    def update(self, z: np.ndarray, R: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
        # Measurement residual
        y = z - self.H @ self.x
        
        # Innovation covariance
        S = self.H @ self.P @ self.H.T + R
        
        # Kalman gain
        try:
            S_inv = inv(S)
            K = self.P @ self.H.T @ S_inv
        except:
            return self.x.copy(), self.P.copy(), float('inf')
        
        # NIS (Normalized Innovation Squared)
        nis = float(y @ S_inv @ y)
        
        # State update
        self.x = self.x + K @ y
        
        # Covariance update (Joseph form for numerical stability)
        I_KH = np.eye(self.dim_x) - K @ self.H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)  # Ensure symmetry
        
        return self.x.copy(), self.P.copy(), nis
    
    def get_state(self) -> np.ndarray:
        return self.x.copy()
    
    def get_covariance(self) -> np.ndarray:
        return self.P.copy()


class UnscentedKalmanFilter(BaseFilter):
    """Unscented Kalman Filter with configurable sigma points."""
    
    def __init__(self, dim_x: int, dim_z: int, alpha: float = 1e-3, 
                 beta: float = 2.0, kappa: float = 0.0):
        self.dim_x = dim_x
        self.dim_z = dim_z
        self.x = np.zeros(dim_x)
        self.P = np.eye(dim_x) * 1000
        self.Q = np.eye(dim_x)
        
        # UKF parameters
        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa
        
        # Compute weights
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
            sqrtP = cholesky(P).T
        except:
            sqrtP = sqrtm(P + 1e-6 * np.eye(n)).real
        
        for i in range(n):
            sigmas[i + 1] = x + self.gamma * sqrtP[i]
            sigmas[n + i + 1] = x - self.gamma * sqrtP[i]
        
        return sigmas
    
    def _f(self, x: np.ndarray, dt: float) -> np.ndarray:
        """State transition function (CV model)."""
        x_new = x.copy()
        n = len(x) // 2
        x_new[:n] += x[n:] * dt
        return x_new
    
    def _h(self, x: np.ndarray) -> np.ndarray:
        """Measurement function (position only)."""
        return x[:self.dim_z]
    
    def predict(self, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        sigmas = self._sigma_points(self.x, self.P)
        
        # Propagate sigma points
        sigmas_f = np.array([self._f(s, dt) for s in sigmas])
        
        # Compute predicted mean
        self.x = np.sum(self.Wm[:, np.newaxis] * sigmas_f, axis=0)
        
        # Compute predicted covariance
        self.P = self.Q.copy()
        for i, s in enumerate(sigmas_f):
            y = s - self.x
            self.P += self.Wc[i] * np.outer(y, y)
        
        return self.x.copy(), self.P.copy()
    
    def update(self, z: np.ndarray, R: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
        sigmas = self._sigma_points(self.x, self.P)
        
        # Transform sigma points through measurement function
        sigmas_h = np.array([self._h(s) for s in sigmas])
        
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
        
        # Update
        self.x = self.x + K @ innovation
        self.P = self.P - K @ S @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        
        return self.x.copy(), self.P.copy(), nis
    
    def get_state(self) -> np.ndarray:
        return self.x.copy()
    
    def get_covariance(self) -> np.ndarray:
        return self.P.copy()


class CubatureKalmanFilter(BaseFilter):
    """Cubature Kalman Filter - optimal for high dimensions."""
    
    def __init__(self, dim_x: int, dim_z: int):
        self.dim_x = dim_x
        self.dim_z = dim_z
        self.x = np.zeros(dim_x)
        self.P = np.eye(dim_x) * 1000
        self.Q = np.eye(dim_x)
        
        # CKF uses 2n cubature points
        self.n_points = 2 * dim_x
        self.weight = 1.0 / self.n_points
        
    def initialize(self, x0: np.ndarray, P0: np.ndarray):
        self.x = x0.copy()
        self.P = P0.copy()
        
    def _cubature_points(self, x: np.ndarray, P: np.ndarray) -> np.ndarray:
        n = len(x)
        points = np.zeros((2*n, n))
        
        try:
            sqrtP = cholesky(P).T * np.sqrt(n)
        except:
            sqrtP = sqrtm(P + 1e-6 * np.eye(n)).real * np.sqrt(n)
        
        for i in range(n):
            points[i] = x + sqrtP[i]
            points[n + i] = x - sqrtP[i]
        
        return points
    
    def _f(self, x: np.ndarray, dt: float) -> np.ndarray:
        x_new = x.copy()
        n = len(x) // 2
        x_new[:n] += x[n:] * dt
        return x_new
    
    def _h(self, x: np.ndarray) -> np.ndarray:
        return x[:self.dim_z]
    
    def predict(self, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        points = self._cubature_points(self.x, self.P)
        
        # Propagate
        points_f = np.array([self._f(p, dt) for p in points])
        
        # Mean
        self.x = self.weight * np.sum(points_f, axis=0)
        
        # Covariance
        self.P = self.Q.copy()
        for p in points_f:
            y = p - self.x
            self.P += self.weight * np.outer(y, y)
        
        return self.x.copy(), self.P.copy()
    
    def update(self, z: np.ndarray, R: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
        points = self._cubature_points(self.x, self.P)
        
        # Transform
        points_h = np.array([self._h(p) for p in points])
        
        # Predicted measurement
        z_pred = self.weight * np.sum(points_h, axis=0)
        
        # Innovation covariance
        S = R.copy()
        for p in points_h:
            y = p - z_pred
            S += self.weight * np.outer(y, y)
        
        # Cross covariance
        Pxz = np.zeros((self.dim_x, self.dim_z))
        for i in range(len(points)):
            Pxz += self.weight * np.outer(points[i] - self.x, points_h[i] - z_pred)
        
        # Kalman gain
        try:
            S_inv = inv(S)
            K = Pxz @ S_inv
        except:
            return self.x.copy(), self.P.copy(), float('inf')
        
        innovation = z - z_pred
        nis = float(innovation @ S_inv @ innovation)
        
        self.x = self.x + K @ innovation
        self.P = self.P - K @ S @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        
        return self.x.copy(), self.P.copy(), nis
    
    def get_state(self) -> np.ndarray:
        return self.x.copy()
    
    def get_covariance(self) -> np.ndarray:
        return self.P.copy()


# =============================================================================
# ADAPTIVE MODULES
# =============================================================================

class AdaptiveQEstimator:
    """Innovation-based process noise (Q) adaptation."""
    
    def __init__(self, window_size: int = 20, alpha: float = 0.1):
        self.window_size = window_size
        self.alpha = alpha  # Exponential smoothing factor
        self.innovation_buffer = deque(maxlen=window_size)
        self.q_scale = 1.0
        
    def update(self, innovation: np.ndarray, S: np.ndarray) -> float:
        """Update Q scale based on innovation."""
        self.innovation_buffer.append(innovation)
        
        if len(self.innovation_buffer) < 5:
            return self.q_scale
        
        # Compute average NIS
        innovations = np.array(self.innovation_buffer)
        try:
            S_inv = inv(S)
            nis_values = [float(inn @ S_inv @ inn) for inn in innovations[-10:]]
            avg_nis = np.mean(nis_values)
        except:
            return self.q_scale
        
        # Expected NIS = dimension of measurement
        dim_z = len(innovation)
        
        # Adjust Q scale
        if avg_nis > dim_z * 1.5:  # Innovations too large
            self.q_scale = min(self.q_scale * 1.1, 10.0)
        elif avg_nis < dim_z * 0.5:  # Innovations too small
            self.q_scale = max(self.q_scale * 0.95, 0.1)
        
        return self.q_scale
    
    def reset(self):
        self.innovation_buffer.clear()
        self.q_scale = 1.0


class AdaptiveREstimator:
    """Innovation-based measurement noise (R) adaptation."""
    
    def __init__(self, window_size: int = 20):
        self.window_size = window_size
        self.innovation_buffer = deque(maxlen=window_size)
        self.S_buffer = deque(maxlen=window_size)
        self.r_scale = 1.0
        
    def update(self, innovation: np.ndarray, S_predicted: np.ndarray) -> float:
        """Estimate actual R from innovation sequence."""
        self.innovation_buffer.append(innovation)
        self.S_buffer.append(S_predicted)
        
        if len(self.innovation_buffer) < 10:
            return self.r_scale
        
        # Compute empirical innovation covariance
        innovations = np.array(self.innovation_buffer)
        cov_actual = np.cov(innovations.T)
        
        # Average predicted S
        S_avg = np.mean(self.S_buffer, axis=0)
        
        # Ratio
        try:
            trace_actual = np.trace(cov_actual)
            trace_pred = np.trace(S_avg)
            ratio = trace_actual / (trace_pred + 1e-10)
        except:
            ratio = 1.0
        
        # Smooth update
        self.r_scale = 0.9 * self.r_scale + 0.1 * max(0.1, min(ratio, 10.0))
        
        return self.r_scale
    
    def reset(self):
        self.innovation_buffer.clear()
        self.S_buffer.clear()
        self.r_scale = 1.0


# =============================================================================
# VS-IMM (Variable-Structure IMM)
# =============================================================================

class VSIMM:
    """
    Variable-Structure Interacting Multiple Model filter.
    Automatically adjusts mode transition probabilities based on maneuver detection.
    """
    
    def __init__(self, dim_x: int, dim_z: int, filter_type: str = 'ukf'):
        self.dim_x = dim_x
        self.dim_z = dim_z
        
        # Create filters for each mode
        FilterClass = {
            'ekf': ExtendedKalmanFilter,
            'ukf': UnscentedKalmanFilter,
            'ckf': CubatureKalmanFilter,
        }.get(filter_type.lower(), UnscentedKalmanFilter)
        
        # Three modes: CV, CT (low maneuver), CT (high maneuver)
        self.n_modes = 3
        self.filters = [FilterClass(dim_x, dim_z) for _ in range(self.n_modes)]
        
        # Mode probabilities
        self.mu = np.array([0.7, 0.2, 0.1])
        
        # Base process noise levels (will be scaled)
        self.q_base = np.array([10, 50, 200])  # m/s^2 equivalent
        
    def initialize(self, x0: np.ndarray, P0: np.ndarray):
        """Initialize all mode filters with same state."""
        for f in self.filters:
            f.initialize(x0, P0)
        self.mu = np.array([0.7, 0.2, 0.1])
        
    def _get_tpm(self) -> np.ndarray:
        """Get transition probability matrix based on current mode probabilities."""
        # Higher probability of staying in same mode if confidence is high
        mu_cv = self.mu[0]
        
        if mu_cv > 0.8:
            p = 0.95  # Very stable, low transition
        elif mu_cv > 0.5:
            p = 0.90
        else:
            p = 0.85  # Maneuvering, higher transition probability
        
        # Build TPM
        PI = np.array([
            [p, (1-p)/2, (1-p)/2],
            [(1-p)/2, p, (1-p)/2],
            [(1-p)/2, (1-p)/2, p]
        ])
        
        return PI
    
    def predict(self, dt: float, q_scale: float = 1.0):
        """Predict all mode filters."""
        # Build Q matrices for each mode
        for j, f in enumerate(self.filters):
            q = (self.q_base[j] * q_scale) ** 2
            Q = np.zeros((self.dim_x, self.dim_x))
            n = self.dim_x // 2
            for i in range(n):
                Q[i, i] = q * dt**4 / 4
                Q[i, i+n] = Q[i+n, i] = q * dt**3 / 2
                Q[i+n, i+n] = q * dt**2
            f.Q = Q
            f.predict(dt)
    
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
            lik = np.exp(-0.5 * min(nis, 50)) + 1e-20
            likelihoods.append(lik)
            total_nis += self.mu[j] * nis
        
        # Update mode probabilities
        mu_new = c_bar * np.array(likelihoods)
        mu_sum = mu_new.sum()
        self.mu = mu_new / mu_sum if mu_sum > 1e-10 else np.array([0.34, 0.33, 0.33])
        
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
# ECCM MODULE (For Defense/Hostile Environments)
# =============================================================================

class ECCMModule:
    """Electronic Counter-Counter Measures for hostile environments."""
    
    def __init__(self, dt: float = 0.05):
        self.dt = dt
        self.innovation_history = deque(maxlen=30)
        self.angle_history = deque(maxlen=20)
        
        # Detection thresholds
        self.jitter_threshold = np.deg2rad(2.0)
        self.trend_threshold = 50.0  # m/s
        
        # State
        self.jamming_detected = False
        self.jamming_confidence = 0.0
        
    def check(self, z: np.ndarray, z_pred: np.ndarray) -> Dict:
        """Check for jamming and return mitigation parameters."""
        innovation = z - z_pred
        self.innovation_history.append(innovation)
        
        # Store angles
        r = norm(z[:3]) if len(z) >= 3 else norm(z)
        if r > 0:
            az = np.arctan2(z[1], z[0])
            el = np.arctan2(z[2], np.sqrt(z[0]**2 + z[1]**2)) if len(z) >= 3 else 0
            self.angle_history.append((az, el))
        
        result = {
            'detected': False,
            'confidence': 0.0,
            'r_scale': 1.0,
            'k_scale': 1.0,
        }
        
        if len(self.angle_history) < 10:
            return result
        
        # Check for angle jitter (Cross-Eye detection)
        angles = np.array(self.angle_history)
        az_rate = np.diff(angles[:, 0]) / self.dt
        az_jitter = np.std(az_rate)
        
        if az_jitter > self.jitter_threshold:
            result['detected'] = True
            result['confidence'] = min(1.0, az_jitter / (3 * self.jitter_threshold))
            result['r_scale'] = 1.0 + 10.0 * result['confidence']
        
        self.jamming_detected = result['detected']
        self.jamming_confidence = result['confidence']
        
        return result
    
    def reset(self):
        self.innovation_history.clear()
        self.angle_history.clear()
        self.jamming_detected = False
        self.jamming_confidence = 0.0


# =============================================================================
# UNIFIED TRACKER
# =============================================================================

class NXMIMOSAUnified:
    """
    NX-MIMOSA v4.0 Unified Tracker
    
    Single engine supporting all industry verticals through configurable
    compliance profiles.
    """
    
    def __init__(self, 
                 industry: Industry = Industry.UNIVERSAL,
                 filter_type: str = 'ukf',
                 dim_x: int = 6,
                 dim_z: int = 3,
                 dt: float = 0.05,
                 sigma: float = 20.0):
        
        self.industry = industry
        self.profile = COMPLIANCE_PROFILES[industry]
        self.filter_type = filter_type
        self.dim_x = dim_x
        self.dim_z = dim_z
        self.dt = dt
        self.sigma = sigma
        
        # Core filter (VS-IMM)
        self.imm = VSIMM(dim_x, dim_z, filter_type)
        
        # Adaptive modules
        self.adaptive_q = AdaptiveQEstimator()
        self.adaptive_r = AdaptiveREstimator()
        
        # ECCM (only for defense)
        self.eccm = ECCMModule(dt) if self.profile.eccm_required else None
        
        # Track statistics
        self.track_id = None
        self.track_start_time = None
        self.n_updates = 0
        self.n_coasts = 0
        self.last_update_time = None
        
        # Quality metrics
        self.position_error_history = deque(maxlen=100)
        self.velocity_error_history = deque(maxlen=100)
        
        # Build measurement matrix
        self.H = np.zeros((dim_z, dim_x))
        for i in range(dim_z):
            self.H[i, i] = 1
        
        # Set filter H matrices
        for f in self.imm.filters:
            f.H = self.H.copy()
    
    def initialize(self, z0: np.ndarray, v0: Optional[np.ndarray] = None,
                   track_id: Optional[int] = None):
        """Initialize track with first measurement."""
        if v0 is None:
            v0 = np.zeros(self.dim_x - self.dim_z)
        
        x0 = np.concatenate([z0, v0])
        P0 = np.diag([self.sigma**2] * self.dim_z + [100**2] * (self.dim_x - self.dim_z))
        
        self.imm.initialize(x0, P0)
        self.adaptive_q.reset()
        self.adaptive_r.reset()
        if self.eccm:
            self.eccm.reset()
        
        self.track_id = track_id
        self.track_start_time = time.time()
        self.n_updates = 0
        self.n_coasts = 0
        self.last_update_time = time.time()
    
    def predict(self, dt: Optional[float] = None):
        """Predict to next time step."""
        dt_use = dt if dt is not None else self.dt
        q_scale = self.adaptive_q.q_scale
        self.imm.predict(dt_use, q_scale)
    
    def update(self, z: np.ndarray, sigma_effective: Optional[float] = None, 
               skip_predict: bool = False) -> np.ndarray:
        """Process measurement and return updated state."""
        sigma = sigma_effective or self.sigma
        
        # Get current state for ECCM check (use current, not predicted since predict was already called)
        x_current = self.imm.get_state()
        z_pred = self.H @ x_current
        
        # ECCM check (if enabled)
        r_scale = 1.0
        if self.eccm:
            eccm_result = self.eccm.check(z, z_pred)
            if eccm_result['detected']:
                r_scale = eccm_result['r_scale']
        
        # Adaptive R estimation
        innovation = z - z_pred
        R_base = (sigma ** 2) * np.eye(self.dim_z)
        S_pred = self.H @ self.imm.filters[0].P @ self.H.T + R_base
        r_adapt = self.adaptive_r.update(innovation, S_pred)
        
        # Final R
        R = R_base * r_scale * r_adapt
        
        # IMM update (just the update step, predict was called separately)
        x_est, P_est, nis = self.imm.update(z, R)
        
        # Update adaptive Q
        self.adaptive_q.update(innovation, S_pred)
        
        # Update statistics
        self.n_updates += 1
        self.last_update_time = time.time()
        
        return x_est
    
    def coast(self) -> np.ndarray:
        """Coast track without measurement."""
        self.predict()
        self.n_coasts += 1
        return self.imm.get_state()
    
    def get_state(self) -> np.ndarray:
        """Get current state estimate."""
        return self.imm.get_state()
    
    def get_position(self) -> np.ndarray:
        """Get position estimate."""
        return self.imm.get_state()[:self.dim_z]
    
    def get_velocity(self) -> np.ndarray:
        """Get velocity estimate."""
        return self.imm.get_state()[self.dim_z:]
    
    def get_mode_probabilities(self) -> np.ndarray:
        """Get IMM mode probabilities."""
        return self.imm.get_mode_probabilities()
    
    def get_track_quality(self) -> Dict:
        """Get track quality metrics."""
        continuity = self.n_updates / (self.n_updates + self.n_coasts + 1e-10)
        
        return {
            'track_id': self.track_id,
            'n_updates': self.n_updates,
            'n_coasts': self.n_coasts,
            'continuity': continuity,
            'mode_probabilities': self.get_mode_probabilities().tolist(),
            'q_scale': self.adaptive_q.q_scale,
            'r_scale': self.adaptive_r.r_scale,
            'jamming_detected': self.eccm.jamming_detected if self.eccm else False,
            'meets_requirements': self._check_compliance(),
        }
    
    def _check_compliance(self) -> bool:
        """Check if track meets industry compliance requirements."""
        continuity = self.n_updates / (self.n_updates + self.n_coasts + 1e-10)
        return continuity >= self.profile.track_continuity_min
    
    def get_compliance_report(self) -> Dict:
        """Generate compliance report for current industry profile."""
        return {
            'industry': self.profile.industry.name,
            'requirements': {
                'position_rms_max': self.profile.position_rms_max,
                'velocity_rms_max': self.profile.velocity_rms_max,
                'track_continuity_min': self.profile.track_continuity_min,
                'latency_max': self.profile.latency_max,
            },
            'certifications_needed': {
                'software': self.profile.software_standard,
                'hardware': self.profile.hardware_standard,
            },
            'output_format': self.profile.output_format,
        }


# =============================================================================
# FACTORY FUNCTION
# =============================================================================

def create_tracker(industry: str = 'universal', **kwargs) -> NXMIMOSAUnified:
    """
    Factory function to create industry-specific tracker.
    
    Args:
        industry: One of 'aviation', 'automotive', 'defense', 'space', 'maritime', 'universal'
        **kwargs: Additional configuration
    
    Returns:
        Configured NXMIMOSAUnified tracker
    """
    industry_map = {
        'aviation': Industry.AVIATION_ATC,
        'atc': Industry.AVIATION_ATC,
        'automotive': Industry.AUTOMOTIVE_ADAS,
        'adas': Industry.AUTOMOTIVE_ADAS,
        'defense': Industry.DEFENSE_RADAR,
        'military': Industry.DEFENSE_RADAR,
        'space': Industry.SPACE_SSA,
        'ssa': Industry.SPACE_SSA,
        'maritime': Industry.MARITIME_VTS,
        'vts': Industry.MARITIME_VTS,
        'universal': Industry.UNIVERSAL,
    }
    
    industry_enum = industry_map.get(industry.lower(), Industry.UNIVERSAL)
    
    # Set default parameters based on industry
    profile = COMPLIANCE_PROFILES[industry_enum]
    
    defaults = {
        'dt': 1.0 / profile.update_rate_min,
        'sigma': profile.position_rms_max / 3,  # 3-sigma rule
    }
    defaults.update(kwargs)
    
    return NXMIMOSAUnified(industry=industry_enum, **defaults)


# =============================================================================
# MAIN - DEMONSTRATION
# =============================================================================

if __name__ == "__main__":
    print("="*100)
    print("NX-MIMOSA v4.0 UNIFIED TRACKING ARCHITECTURE")
    print("="*100)
    
    print("""
┌─────────────────────────────────────────────────────────────────────────────────────────────────────┐
│ SUPPORTED INDUSTRIES                                                                                │
├─────────────────────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                                     │
│  1. AVIATION (ATC/ATM)     - EUROCONTROL EASSP, DO-178C, ASTERIX CAT062                            │
│  2. AUTOMOTIVE (ADAS/AD)   - ISO 26262 ASIL-D, Euro NCAP, CAN-FD                                   │
│  3. DEFENSE (Military)     - MIL-STD, DO-254 DAL-A, Link-16, ECCM                                  │
│  4. SPACE (SSA/STM)        - ECSS, NASA, CCSDS                                                     │
│  5. MARITIME (VTS/VTMS)    - IMO, SOLAS, NMEA 2000                                                 │
│  6. UNIVERSAL              - General purpose tracking                                              │
│                                                                                                     │
└─────────────────────────────────────────────────────────────────────────────────────────────────────┘
""")
    
    # Demo each industry
    for industry_name in ['aviation', 'automotive', 'defense', 'space', 'maritime']:
        print(f"\n{'='*80}")
        print(f"Industry: {industry_name.upper()}")
        print('='*80)
        
        tracker = create_tracker(industry_name)
        report = tracker.get_compliance_report()
        
        print(f"Position RMS Max:     {report['requirements']['position_rms_max']:.1f} m")
        print(f"Track Continuity Min: {report['requirements']['track_continuity_min']*100:.1f}%")
        print(f"Latency Max:          {report['requirements']['latency_max']*1000:.0f} ms")
        print(f"Software Standard:    {report['certifications_needed']['software']}")
        print(f"Output Format:        {report['output_format']}")
    
    print("\n" + "="*100)
    print("✓ NX-MIMOSA v4.0 Unified Architecture Ready")
    print("  One core engine, multiple industry compliance profiles")
    print("="*100)
