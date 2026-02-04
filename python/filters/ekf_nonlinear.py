#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════════════════════════════
NX-MIMOSA Extended Kalman Filter (EKF) Module
═══════════════════════════════════════════════════════════════════════════════════════════════════════

Extended Kalman Filter implementation for nonlinear measurement models:
- Radar (range, azimuth, elevation) → Cartesian conversion
- GPS (geodetic) → Local tangent plane
- Bearing-only tracking

Compliant with:
- EUROCONTROL ARTAS sensor integration
- ISO 26262 ASIL-D (automotive radar)

Author: Dr. Mladen Mešter / Nexellum d.o.o.
Version: 1.1.0
License: AGPL v3 / Commercial
═══════════════════════════════════════════════════════════════════════════════════════════════════════
"""

import numpy as np
from typing import Tuple, Optional, Callable, Dict, Any
from dataclasses import dataclass
from enum import IntEnum
import warnings


# =============================================================================
# CONSTANTS
# =============================================================================

# WGS-84 ellipsoid parameters
WGS84_A = 6378137.0  # Semi-major axis (m)
WGS84_F = 1 / 298.257223563  # Flattening
WGS84_B = WGS84_A * (1 - WGS84_F)  # Semi-minor axis
WGS84_E2 = 2 * WGS84_F - WGS84_F ** 2  # First eccentricity squared


# =============================================================================
# MEASUREMENT MODELS
# =============================================================================

class MeasurementModel(IntEnum):
    """Supported measurement models."""
    CARTESIAN = 0       # Direct x, y, z measurements
    RADAR_RAE = 1       # Range, Azimuth, Elevation
    RADAR_2D = 2        # Range, Azimuth (2D radar)
    BEARING_ONLY = 3    # Azimuth only (passive)
    GPS_GEODETIC = 4    # Latitude, Longitude, Altitude
    ADS_B = 5           # ADS-B position report


@dataclass
class SensorConfig:
    """Sensor configuration for measurement model."""
    model: MeasurementModel
    # Sensor position (for radar)
    sensor_x: float = 0.0
    sensor_y: float = 0.0
    sensor_z: float = 0.0
    # Reference point for geodetic (for GPS)
    ref_lat: float = 0.0  # degrees
    ref_lon: float = 0.0  # degrees
    ref_alt: float = 0.0  # meters
    # Noise parameters
    sigma_range: float = 50.0      # meters
    sigma_azimuth: float = 0.5     # degrees
    sigma_elevation: float = 0.5   # degrees


# =============================================================================
# COORDINATE TRANSFORMS
# =============================================================================

def geodetic_to_ecef(lat: float, lon: float, alt: float) -> np.ndarray:
    """
    Convert geodetic (WGS-84) to ECEF coordinates.
    
    Args:
        lat: Latitude in degrees
        lon: Longitude in degrees
        alt: Altitude in meters (above ellipsoid)
    
    Returns:
        ECEF coordinates [x, y, z] in meters
    """
    lat_rad = np.radians(lat)
    lon_rad = np.radians(lon)
    
    N = WGS84_A / np.sqrt(1 - WGS84_E2 * np.sin(lat_rad) ** 2)
    
    x = (N + alt) * np.cos(lat_rad) * np.cos(lon_rad)
    y = (N + alt) * np.cos(lat_rad) * np.sin(lon_rad)
    z = (N * (1 - WGS84_E2) + alt) * np.sin(lat_rad)
    
    return np.array([x, y, z])


def ecef_to_enu(ecef: np.ndarray, ref_lat: float, ref_lon: float, ref_alt: float) -> np.ndarray:
    """
    Convert ECEF to local ENU (East-North-Up) coordinates.
    
    Args:
        ecef: ECEF coordinates [x, y, z]
        ref_lat, ref_lon, ref_alt: Reference point (geodetic)
    
    Returns:
        ENU coordinates [e, n, u] in meters
    """
    ref_ecef = geodetic_to_ecef(ref_lat, ref_lon, ref_alt)
    d = ecef - ref_ecef
    
    lat_rad = np.radians(ref_lat)
    lon_rad = np.radians(ref_lon)
    
    # Rotation matrix ECEF → ENU
    R = np.array([
        [-np.sin(lon_rad), np.cos(lon_rad), 0],
        [-np.sin(lat_rad) * np.cos(lon_rad), -np.sin(lat_rad) * np.sin(lon_rad), np.cos(lat_rad)],
        [np.cos(lat_rad) * np.cos(lon_rad), np.cos(lat_rad) * np.sin(lon_rad), np.sin(lat_rad)]
    ])
    
    return R @ d


def rae_to_cartesian(r: float, az: float, el: float,
                     sensor_pos: np.ndarray = None) -> np.ndarray:
    """
    Convert Range-Azimuth-Elevation to Cartesian.
    
    Args:
        r: Range in meters
        az: Azimuth in radians (from North, clockwise)
        el: Elevation in radians (from horizontal)
        sensor_pos: Sensor position [x, y, z]
    
    Returns:
        Cartesian position [x, y, z]
    """
    if sensor_pos is None:
        sensor_pos = np.zeros(3)
    
    # Convert spherical to Cartesian
    x = r * np.cos(el) * np.sin(az)  # East
    y = r * np.cos(el) * np.cos(az)  # North
    z = r * np.sin(el)                # Up
    
    return sensor_pos + np.array([x, y, z])


def cartesian_to_rae(pos: np.ndarray, sensor_pos: np.ndarray = None) -> Tuple[float, float, float]:
    """
    Convert Cartesian to Range-Azimuth-Elevation.
    
    Args:
        pos: Position [x, y, z]
        sensor_pos: Sensor position [x, y, z]
    
    Returns:
        (range, azimuth, elevation) in (meters, radians, radians)
    """
    if sensor_pos is None:
        sensor_pos = np.zeros(3)
    
    d = pos - sensor_pos
    
    r = np.linalg.norm(d)
    az = np.arctan2(d[0], d[1])  # atan2(East, North)
    el = np.arcsin(d[2] / r) if r > 0 else 0.0
    
    return r, az, el


# =============================================================================
# EXTENDED KALMAN FILTER
# =============================================================================

class ExtendedKalmanFilter:
    """
    Extended Kalman Filter for nonlinear measurement models.
    
    State vector: [x, y, z, vx, vy, vz] (6D constant velocity)
    
    Supports multiple measurement models via sensor configuration.
    
    [REQ-EKF-001] Nonlinear measurement handling
    [REQ-EKF-002] Jacobian computation
    [REQ-EKF-003] Numerical stability
    """
    
    def __init__(self, sensor_config: SensorConfig):
        """
        Initialize EKF with sensor configuration.
        
        Args:
            sensor_config: Sensor model and parameters
        """
        self.config = sensor_config
        self.dim_x = 6  # State dimension
        
        # Set measurement dimension based on model
        if sensor_config.model == MeasurementModel.CARTESIAN:
            self.dim_z = 3
        elif sensor_config.model == MeasurementModel.RADAR_RAE:
            self.dim_z = 3
        elif sensor_config.model == MeasurementModel.RADAR_2D:
            self.dim_z = 2
        elif sensor_config.model == MeasurementModel.BEARING_ONLY:
            self.dim_z = 1
        elif sensor_config.model in (MeasurementModel.GPS_GEODETIC, MeasurementModel.ADS_B):
            self.dim_z = 3
        else:
            self.dim_z = 3
        
        # State and covariance
        self.x = np.zeros(self.dim_x)
        self.P = np.eye(self.dim_x) * 1e6
        
        # Process noise
        self.q_pos = 1.0    # Position process noise (m²/s³)
        self.q_vel = 10.0   # Velocity process noise (m²/s⁵)
        
        # Sensor position
        self.sensor_pos = np.array([
            sensor_config.sensor_x,
            sensor_config.sensor_y,
            sensor_config.sensor_z
        ])
    
    def initialize(self, x0: np.ndarray, P0: np.ndarray = None):
        """
        Initialize filter state.
        
        Args:
            x0: Initial state [x, y, z, vx, vy, vz]
            P0: Initial covariance (optional)
        """
        self.x = np.array(x0, dtype=float)
        if P0 is not None:
            self.P = np.array(P0, dtype=float)
        else:
            # Default initialization
            self.P = np.diag([1000.0, 1000.0, 1000.0, 100.0, 100.0, 100.0])
    
    def initialize_from_measurement(self, z: np.ndarray, R: np.ndarray = None):
        """
        Initialize state from first measurement.
        
        Args:
            z: Measurement in sensor coordinates
            R: Measurement noise covariance
        """
        # Convert measurement to Cartesian
        pos = self._measurement_to_cartesian(z)
        
        # Initialize with zero velocity
        self.x = np.array([pos[0], pos[1], pos[2], 0.0, 0.0, 0.0])
        
        # Large initial covariance
        self.P = np.diag([10000.0, 10000.0, 10000.0, 1000.0, 1000.0, 1000.0])
    
    def _measurement_to_cartesian(self, z: np.ndarray) -> np.ndarray:
        """Convert measurement to Cartesian based on model."""
        if self.config.model == MeasurementModel.CARTESIAN:
            return z[:3]
        
        elif self.config.model == MeasurementModel.RADAR_RAE:
            r, az, el = z[0], z[1], z[2]
            return rae_to_cartesian(r, az, el, self.sensor_pos)
        
        elif self.config.model == MeasurementModel.RADAR_2D:
            r, az = z[0], z[1]
            return rae_to_cartesian(r, az, 0.0, self.sensor_pos)
        
        elif self.config.model == MeasurementModel.GPS_GEODETIC:
            lat, lon, alt = z[0], z[1], z[2]
            ecef = geodetic_to_ecef(lat, lon, alt)
            return ecef_to_enu(ecef, self.config.ref_lat, 
                             self.config.ref_lon, self.config.ref_alt)
        
        else:
            return z[:3]
    
    def _h(self, x: np.ndarray) -> np.ndarray:
        """
        Measurement function h(x).
        
        Args:
            x: State vector [x, y, z, vx, vy, vz]
        
        Returns:
            Expected measurement in sensor coordinates
        """
        pos = x[:3]
        
        if self.config.model == MeasurementModel.CARTESIAN:
            return pos
        
        elif self.config.model == MeasurementModel.RADAR_RAE:
            r, az, el = cartesian_to_rae(pos, self.sensor_pos)
            return np.array([r, az, el])
        
        elif self.config.model == MeasurementModel.RADAR_2D:
            r, az, _ = cartesian_to_rae(pos, self.sensor_pos)
            return np.array([r, az])
        
        elif self.config.model == MeasurementModel.BEARING_ONLY:
            _, az, _ = cartesian_to_rae(pos, self.sensor_pos)
            return np.array([az])
        
        else:
            return pos
    
    def _H_jacobian(self, x: np.ndarray) -> np.ndarray:
        """
        Compute measurement Jacobian H = ∂h/∂x.
        
        Args:
            x: State vector
        
        Returns:
            Jacobian matrix (dim_z × dim_x)
        """
        pos = x[:3]
        d = pos - self.sensor_pos
        
        if self.config.model == MeasurementModel.CARTESIAN:
            # H = [I_3 | 0_3]
            H = np.zeros((3, 6))
            H[:3, :3] = np.eye(3)
            return H
        
        elif self.config.model == MeasurementModel.RADAR_RAE:
            r = np.linalg.norm(d)
            r_horiz = np.sqrt(d[0]**2 + d[1]**2)
            
            if r < 1e-6:
                r = 1e-6
            if r_horiz < 1e-6:
                r_horiz = 1e-6
            
            H = np.zeros((3, 6))
            
            # ∂r/∂x = d/r
            H[0, 0] = d[0] / r
            H[0, 1] = d[1] / r
            H[0, 2] = d[2] / r
            
            # ∂az/∂x = ∂atan2(x,y)/∂(x,y)
            H[1, 0] = d[1] / (r_horiz ** 2)
            H[1, 1] = -d[0] / (r_horiz ** 2)
            
            # ∂el/∂x = ∂asin(z/r)/∂(x,y,z)
            cos_el = r_horiz / r
            if abs(cos_el) > 1e-6:
                H[2, 0] = -d[0] * d[2] / (r ** 2 * r_horiz)
                H[2, 1] = -d[1] * d[2] / (r ** 2 * r_horiz)
                H[2, 2] = r_horiz / (r ** 2)
            
            return H
        
        elif self.config.model == MeasurementModel.RADAR_2D:
            r_horiz = np.sqrt(d[0]**2 + d[1]**2)
            if r_horiz < 1e-6:
                r_horiz = 1e-6
            
            H = np.zeros((2, 6))
            
            # ∂r/∂x (horizontal range only)
            H[0, 0] = d[0] / r_horiz
            H[0, 1] = d[1] / r_horiz
            
            # ∂az/∂x
            H[1, 0] = d[1] / (r_horiz ** 2)
            H[1, 1] = -d[0] / (r_horiz ** 2)
            
            return H
        
        elif self.config.model == MeasurementModel.BEARING_ONLY:
            r_horiz = np.sqrt(d[0]**2 + d[1]**2)
            if r_horiz < 1e-6:
                r_horiz = 1e-6
            
            H = np.zeros((1, 6))
            H[0, 0] = d[1] / (r_horiz ** 2)
            H[0, 1] = -d[0] / (r_horiz ** 2)
            
            return H
        
        else:
            H = np.zeros((3, 6))
            H[:3, :3] = np.eye(3)
            return H
    
    def _get_Q(self, dt: float) -> np.ndarray:
        """Get process noise covariance matrix."""
        q = self.q_vel
        
        Q = np.array([
            [dt**5/20, 0, 0, dt**4/8, 0, 0],
            [0, dt**5/20, 0, 0, dt**4/8, 0],
            [0, 0, dt**5/20, 0, 0, dt**4/8],
            [dt**4/8, 0, 0, dt**3/3, 0, 0],
            [0, dt**4/8, 0, 0, dt**3/3, 0],
            [0, 0, dt**4/8, 0, 0, dt**3/3]
        ]) * q
        
        return Q
    
    def _get_F(self, dt: float) -> np.ndarray:
        """Get state transition matrix for constant velocity model."""
        F = np.eye(6)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        return F
    
    def predict(self, dt: float):
        """
        Predict state forward by dt seconds.
        
        Args:
            dt: Time step in seconds
        """
        F = self._get_F(dt)
        Q = self._get_Q(dt)
        
        # Predict state
        self.x = F @ self.x
        
        # Predict covariance
        self.P = F @ self.P @ F.T + Q
        
        # Ensure symmetry
        self.P = 0.5 * (self.P + self.P.T)
    
    def update(self, z: np.ndarray, R: np.ndarray = None) -> Tuple[np.ndarray, np.ndarray, float]:
        """
        Update state with measurement.
        
        Args:
            z: Measurement in sensor coordinates
            R: Measurement noise covariance (optional)
        
        Returns:
            (updated_state, updated_covariance, NIS)
        """
        if R is None:
            R = self._get_default_R()
        
        # Predicted measurement
        z_pred = self._h(self.x)
        
        # Innovation
        y = z - z_pred
        
        # Handle angle wrapping for azimuth
        if self.config.model in (MeasurementModel.RADAR_RAE, 
                                  MeasurementModel.RADAR_2D,
                                  MeasurementModel.BEARING_ONLY):
            az_idx = 1 if self.config.model != MeasurementModel.BEARING_ONLY else 0
            while y[az_idx] > np.pi:
                y[az_idx] -= 2 * np.pi
            while y[az_idx] < -np.pi:
                y[az_idx] += 2 * np.pi
        
        # Jacobian
        H = self._H_jacobian(self.x)
        
        # Innovation covariance
        S = H @ self.P @ H.T + R
        
        # Kalman gain
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            K = self.P @ H.T @ np.linalg.pinv(S)
        
        # Update state
        self.x = self.x + K @ y
        
        # Update covariance (Joseph form for numerical stability)
        I_KH = np.eye(self.dim_x) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        
        # Ensure symmetry
        self.P = 0.5 * (self.P + self.P.T)
        
        # Normalized Innovation Squared
        try:
            nis = y.T @ np.linalg.inv(S) @ y
        except:
            nis = 0.0
        
        return self.x.copy(), self.P.copy(), float(nis)
    
    def _get_default_R(self) -> np.ndarray:
        """Get default measurement noise based on sensor config."""
        if self.config.model == MeasurementModel.CARTESIAN:
            return np.diag([50.0**2, 50.0**2, 100.0**2])
        
        elif self.config.model == MeasurementModel.RADAR_RAE:
            sigma_r = self.config.sigma_range
            sigma_az = np.radians(self.config.sigma_azimuth)
            sigma_el = np.radians(self.config.sigma_elevation)
            return np.diag([sigma_r**2, sigma_az**2, sigma_el**2])
        
        elif self.config.model == MeasurementModel.RADAR_2D:
            sigma_r = self.config.sigma_range
            sigma_az = np.radians(self.config.sigma_azimuth)
            return np.diag([sigma_r**2, sigma_az**2])
        
        elif self.config.model == MeasurementModel.BEARING_ONLY:
            sigma_az = np.radians(self.config.sigma_azimuth)
            return np.array([[sigma_az**2]])
        
        else:
            return np.diag([50.0**2, 50.0**2, 100.0**2])
    
    def get_state(self) -> np.ndarray:
        """Get current state estimate."""
        return self.x.copy()
    
    def get_position(self) -> np.ndarray:
        """Get position [x, y, z]."""
        return self.x[:3].copy()
    
    def get_velocity(self) -> np.ndarray:
        """Get velocity [vx, vy, vz]."""
        return self.x[3:6].copy()
    
    def get_covariance(self) -> np.ndarray:
        """Get state covariance."""
        return self.P.copy()


# =============================================================================
# EKF-IMM INTEGRATION
# =============================================================================

class EKFIMM:
    """
    Extended Kalman Filter with Interacting Multiple Model.
    
    Combines EKF for nonlinear measurements with IMM for maneuver detection.
    
    [REQ-EKFIMM-001] Multiple motion models
    [REQ-EKFIMM-002] Mode probability estimation
    """
    
    def __init__(self, sensor_config: SensorConfig, n_modes: int = 3):
        """
        Initialize EKF-IMM.
        
        Args:
            sensor_config: Sensor configuration
            n_modes: Number of motion modes (default: 3)
        """
        self.config = sensor_config
        self.n_modes = n_modes
        
        # Create filters for each mode
        self.filters = [ExtendedKalmanFilter(sensor_config) for _ in range(n_modes)]
        
        # Mode probabilities
        self.mu = np.ones(n_modes) / n_modes
        
        # Transition probability matrix
        self.tpm = self._default_tpm()
        
        # Process noise for each mode (increasing aggressiveness)
        self.q_multipliers = [1.0, 5.0, 20.0]
        for i, f in enumerate(self.filters):
            f.q_vel *= self.q_multipliers[i]
    
    def _default_tpm(self) -> np.ndarray:
        """Default transition probability matrix."""
        return np.array([
            [0.95, 0.04, 0.01],
            [0.05, 0.90, 0.05],
            [0.01, 0.09, 0.90]
        ])
    
    def initialize(self, x0: np.ndarray, P0: np.ndarray = None):
        """Initialize all mode filters."""
        for f in self.filters:
            f.initialize(x0.copy(), P0.copy() if P0 is not None else None)
        self.mu = np.array([0.8, 0.15, 0.05])  # Favor CV initially
    
    def initialize_from_measurement(self, z: np.ndarray):
        """Initialize from first measurement."""
        for f in self.filters:
            f.initialize_from_measurement(z)
        self.mu = np.array([0.8, 0.15, 0.05])
    
    def predict(self, dt: float):
        """Predict with IMM mixing."""
        # Compute mixing probabilities
        c_bar = self.tpm.T @ self.mu
        c_bar = np.maximum(c_bar, 1e-10)
        
        mu_ij = (self.tpm.T * self.mu) / c_bar[:, np.newaxis]
        
        # Mix states and covariances
        mixed_x = []
        mixed_P = []
        
        for j in range(self.n_modes):
            # Mixed state
            x_mix = np.zeros(6)
            for i in range(self.n_modes):
                x_mix += mu_ij[j, i] * self.filters[i].x
            
            # Mixed covariance
            P_mix = np.zeros((6, 6))
            for i in range(self.n_modes):
                dx = self.filters[i].x - x_mix
                P_mix += mu_ij[j, i] * (self.filters[i].P + np.outer(dx, dx))
            
            mixed_x.append(x_mix)
            mixed_P.append(P_mix)
        
        # Set mixed states and predict
        for j in range(self.n_modes):
            self.filters[j].x = mixed_x[j]
            self.filters[j].P = mixed_P[j]
            self.filters[j].predict(dt)
    
    def update(self, z: np.ndarray, R: np.ndarray = None) -> Tuple[np.ndarray, np.ndarray, float]:
        """Update all filters and combine."""
        likelihoods = np.zeros(self.n_modes)
        
        for j, f in enumerate(self.filters):
            _, _, nis = f.update(z.copy(), R)
            
            # Compute likelihood from NIS
            S = f._H_jacobian(f.x) @ f.P @ f._H_jacobian(f.x).T + (R if R is not None else f._get_default_R())
            det_S = np.linalg.det(S)
            if det_S > 0:
                likelihoods[j] = np.exp(-0.5 * nis) / np.sqrt((2 * np.pi) ** f.dim_z * det_S)
            else:
                likelihoods[j] = 1e-10
        
        # Update mode probabilities
        c_bar = self.tpm.T @ self.mu
        self.mu = likelihoods * c_bar
        mu_sum = np.sum(self.mu)
        if mu_sum > 1e-10:
            self.mu /= mu_sum
        else:
            self.mu = np.ones(self.n_modes) / self.n_modes
        
        # Combined estimate
        x_combined = np.zeros(6)
        P_combined = np.zeros((6, 6))
        
        for j in range(self.n_modes):
            x_combined += self.mu[j] * self.filters[j].x
        
        for j in range(self.n_modes):
            dx = self.filters[j].x - x_combined
            P_combined += self.mu[j] * (self.filters[j].P + np.outer(dx, dx))
        
        nis_combined = np.sum([self.mu[j] * nis for j, nis in enumerate(likelihoods)])
        
        return x_combined, P_combined, nis_combined
    
    def get_state(self) -> np.ndarray:
        """Get combined state estimate."""
        x = np.zeros(6)
        for j in range(self.n_modes):
            x += self.mu[j] * self.filters[j].x
        return x
    
    def get_mode_probabilities(self) -> np.ndarray:
        """Get current mode probabilities."""
        return self.mu.copy()


# =============================================================================
# MAIN / DEMO
# =============================================================================

if __name__ == "__main__":
    print("NX-MIMOSA EKF Module v1.1.0")
    print("=" * 60)
    
    # Demo: Radar tracking
    config = SensorConfig(
        model=MeasurementModel.RADAR_RAE,
        sensor_x=0.0,
        sensor_y=0.0,
        sensor_z=0.0,
        sigma_range=50.0,
        sigma_azimuth=0.5,
        sigma_elevation=0.5
    )
    
    ekf = ExtendedKalmanFilter(config)
    
    # Simulate target at 50km, moving north at 200 m/s
    true_pos = np.array([0.0, 50000.0, 10000.0])
    true_vel = np.array([0.0, 200.0, 0.0])
    
    # Initialize from first measurement
    r0, az0, el0 = cartesian_to_rae(true_pos, ekf.sensor_pos)
    z0 = np.array([r0 + np.random.randn() * 50, 
                   az0 + np.random.randn() * np.radians(0.5),
                   el0 + np.random.randn() * np.radians(0.5)])
    
    ekf.initialize_from_measurement(z0)
    
    print(f"Initial state: {ekf.get_state()[:3]}")
    print(f"True position: {true_pos}")
    
    # Track for 10 steps
    dt = 1.0
    for i in range(10):
        true_pos += true_vel * dt
        
        # Predict
        ekf.predict(dt)
        
        # Generate noisy measurement
        r, az, el = cartesian_to_rae(true_pos, ekf.sensor_pos)
        z = np.array([r + np.random.randn() * 50,
                      az + np.random.randn() * np.radians(0.5),
                      el + np.random.randn() * np.radians(0.5)])
        
        # Update
        state, P, nis = ekf.update(z)
        
        error = np.linalg.norm(state[:3] - true_pos)
        print(f"Step {i+1}: Error = {error:.1f} m, NIS = {nis:.2f}")
    
    print("\n✅ EKF Demo Complete")
