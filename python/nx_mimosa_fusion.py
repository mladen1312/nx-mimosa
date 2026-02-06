"""
NX-MIMOSA Multi-Sensor Fusion Module
=====================================
[REQ-V43-01] Measurement-level fusion via sequential Kalman update
[REQ-V43-02] Sensor types: position_xy, range_bearing, bearing_only, range_doppler, position_3d
[REQ-V43-03] Asynchronous sensor handling with per-sensor timestamps
[REQ-V43-04] Information-weighted simultaneous fusion (batch update)
[REQ-V43-05] Sensor health monitoring (NIS per sensor, degradation detection)
[REQ-V43-06] Automatic sensor registration (bias estimation)
[REQ-V43-07] Graceful degradation (single sensor fallback)

Architecture:
  Each sensor provides measurements in its native format.
  The fusion engine converts to a common state space via sensor-specific
  H matrices (Jacobians for nonlinear) and R matrices.
  Sequential Kalman update processes each measurement optimally.

  For N simultaneous measurements from N sensors:
    - Sequential update: process one at a time (order-independent for linear)
    - Information filter: sum information matrices (optimal for batch)

  Supported sensor types:
    POSITION_XY     : [x, y] in Cartesian (radar after coordinate conversion, GPS)
    POSITION_3D     : [x, y, z] in Cartesian (3D radar, GPS+alt)
    RANGE_BEARING   : [range, azimuth] polar from sensor position
    RANGE_DOPPLER   : [range, azimuth, range_rate] with Doppler
    BEARING_ONLY    : [azimuth] or [azimuth, elevation] (passive EO/IR, ESM)
    ADS_B           : [x, y, vx, vy] position + velocity (cooperative)

Author: Dr. Mladen Mešter / Nexellum d.o.o.
License: AGPL v3 (open-source) | Commercial license available
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any
from enum import Enum
import time


class SensorType(Enum):
    """Supported sensor measurement types."""
    POSITION_XY = "position_xy"       # [x, y] Cartesian
    POSITION_3D = "position_3d"       # [x, y, z] Cartesian
    RANGE_BEARING = "range_bearing"   # [r, θ] polar from sensor
    RANGE_DOPPLER = "range_doppler"   # [r, θ, ṙ] with range rate
    BEARING_ONLY = "bearing_only"     # [θ] or [θ, φ] passive
    ADS_B = "ads_b"                   # [x, y, vx, vy] cooperative


@dataclass
class SensorConfig:
    """Configuration for a single sensor in the fusion system.

    Attributes:
        sensor_id: Unique identifier for this sensor
        sensor_type: Type of measurements this sensor provides
        R: Measurement noise covariance matrix (sensor-specific)
        position: Sensor position [x, y] or [x, y, z] for range/bearing sensors
        bias: Known or estimated sensor bias vector
        max_range: Maximum detection range (meters), None = unlimited
        update_rate: Expected update rate (Hz)
        reliability: Sensor reliability score 0-1 (for weighted fusion)
        active: Whether this sensor is currently providing measurements
    """
    sensor_id: str
    sensor_type: SensorType
    R: np.ndarray
    position: np.ndarray = field(default_factory=lambda: np.zeros(2))
    bias: np.ndarray = field(default_factory=lambda: np.zeros(2))
    max_range: Optional[float] = None
    update_rate: float = 1.0
    reliability: float = 1.0
    active: bool = True


@dataclass
class SensorMeasurement:
    """A single measurement from a sensor.

    Attributes:
        sensor_id: Which sensor produced this measurement
        z: Raw measurement vector
        timestamp: Measurement time (seconds, monotonic)
        R_override: Optional per-measurement R (e.g., adaptive noise)
        metadata: Additional sensor-specific data (SNR, confidence, etc.)
    """
    sensor_id: str
    z: np.ndarray
    timestamp: float = 0.0
    R_override: Optional[np.ndarray] = None
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class SensorHealth:
    """Runtime health statistics for a sensor."""
    sensor_id: str
    nis_avg: float = 2.0        # Average NIS (should be ~dim_z for healthy)
    nis_history: List[float] = field(default_factory=list)
    measurement_count: int = 0
    last_update_time: float = 0.0
    consecutive_rejects: int = 0
    degraded: bool = False       # True if NIS indicates sensor issues
    bias_estimate: Optional[np.ndarray] = None


class MultiSensorFusionEngine:
    """
    Multi-sensor fusion engine for NX-MIMOSA tracker.

    Implements measurement-level centralized fusion using sequential
    Kalman updates. Each sensor measurement is processed through
    its own H/R matrices while sharing the common state estimate.

    Usage:
        engine = MultiSensorFusionEngine()

        # Register sensors
        engine.add_sensor(SensorConfig(
            sensor_id="radar_primary",
            sensor_type=SensorType.POSITION_XY,
            R=np.diag([50.0**2, 50.0**2]),  # 50m range noise
        ))
        engine.add_sensor(SensorConfig(
            sensor_id="eo_tracker",
            sensor_type=SensorType.BEARING_ONLY,
            R=np.array([[np.radians(0.5)**2]]),  # 0.5° bearing noise
            position=np.array([0.0, 0.0]),
        ))

        # In tracking loop, provide all available measurements:
        measurements = [
            SensorMeasurement("radar_primary", np.array([1000.0, 500.0])),
            SensorMeasurement("eo_tracker", np.array([np.radians(26.57)])),
        ]
        engine.fuse(tracker, measurements)
    """

    def __init__(self):
        self.sensors: Dict[str, SensorConfig] = {}
        self.health: Dict[str, SensorHealth] = {}
        self._nis_window = 20  # NIS averaging window

    def add_sensor(self, config: SensorConfig) -> None:
        """Register a sensor with the fusion engine."""
        self.sensors[config.sensor_id] = config
        self.health[config.sensor_id] = SensorHealth(sensor_id=config.sensor_id)

    def remove_sensor(self, sensor_id: str) -> None:
        """Remove a sensor from the fusion engine."""
        self.sensors.pop(sensor_id, None)
        self.health.pop(sensor_id, None)

    def get_active_sensors(self) -> List[str]:
        """List currently active sensor IDs."""
        return [sid for sid, s in self.sensors.items() if s.active]

    def _build_H_and_hx(self, sensor: SensorConfig, x: np.ndarray
                         ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Build measurement matrix H and predicted measurement h(x)
        for a given sensor type and current state.

        For linear models (POSITION_XY, ADS_B): H is constant, h(x) = H @ x
        For nonlinear models (RANGE_BEARING, etc.): H is Jacobian, h(x) is nonlinear

        State vector x = [px, py, vx, vy] (4D) or extended

        Returns:
            H: Measurement Jacobian matrix
            hx: Predicted measurement h(x)
        """
        px, py = x[0], x[1]
        vx = x[2] if len(x) > 2 else 0.0
        vy = x[3] if len(x) > 3 else 0.0

        sx, sy = sensor.position[0], sensor.position[1]
        dx, dy = px - sx, py - sy
        r = np.sqrt(dx**2 + dy**2)
        r = max(r, 1e-6)  # Avoid division by zero

        stype = sensor.sensor_type

        if stype == SensorType.POSITION_XY:
            # z = [x, y], H picks position from state
            n = len(x)
            H = np.zeros((2, n))
            H[0, 0] = 1.0  # x
            H[1, 1] = 1.0  # y
            hx = np.array([px, py])
            return H, hx

        elif stype == SensorType.POSITION_3D:
            # z = [x, y, z], needs 3D state (future extension)
            n = len(x)
            H = np.zeros((3, n))
            H[0, 0] = 1.0
            H[1, 1] = 1.0
            # z-component: if state has it (dim>=6 for 3D), use it
            if n >= 6:
                H[2, 4] = 1.0  # Assuming [px,py,vx,vy,pz,vz]
            hx = np.array([px, py, x[4] if n >= 6 else 0.0])
            return H, hx

        elif stype == SensorType.RANGE_BEARING:
            # z = [range, azimuth]
            # h(x) = [sqrt((px-sx)²+(py-sy)²), atan2(py-sy, px-sx)]
            theta = np.arctan2(dy, dx)
            hx = np.array([r, theta])

            n = len(x)
            H = np.zeros((2, n))
            # ∂r/∂px = dx/r, ∂r/∂py = dy/r
            H[0, 0] = dx / r
            H[0, 1] = dy / r
            # ∂θ/∂px = -dy/r², ∂θ/∂py = dx/r²
            H[1, 0] = -dy / (r**2)
            H[1, 1] = dx / (r**2)
            return H, hx

        elif stype == SensorType.RANGE_DOPPLER:
            # z = [range, azimuth, range_rate]
            theta = np.arctan2(dy, dx)
            # range rate = (dx*vx + dy*vy) / r
            rdot = (dx * vx + dy * vy) / r
            hx = np.array([r, theta, rdot])

            n = len(x)
            H = np.zeros((3, n))
            # Range partials (same as RANGE_BEARING)
            H[0, 0] = dx / r
            H[0, 1] = dy / r
            # Bearing partials
            H[1, 0] = -dy / (r**2)
            H[1, 1] = dx / (r**2)
            # Range-rate partials: ṙ = (dx·vx + dy·vy)/r
            # ∂ṙ/∂px = vx/r - (dx·vx+dy·vy)·dx/r³
            H[2, 0] = vx / r - rdot * dx / (r**2)
            H[2, 1] = vy / r - rdot * dy / (r**2)
            if n > 2:
                H[2, 2] = dx / r  # ∂ṙ/∂vx
            if n > 3:
                H[2, 3] = dy / r  # ∂ṙ/∂vy
            return H, hx

        elif stype == SensorType.BEARING_ONLY:
            # z = [azimuth] (passive sensor — no range info)
            theta = np.arctan2(dy, dx)
            hx = np.array([theta])

            n = len(x)
            H = np.zeros((1, n))
            H[0, 0] = -dy / (r**2)
            H[0, 1] = dx / (r**2)
            return H, hx

        elif stype == SensorType.ADS_B:
            # z = [x, y, vx, vy] — cooperative target, full state
            n = len(x)
            H = np.zeros((4, n))
            H[0, 0] = 1.0  # x
            H[1, 1] = 1.0  # y
            if n > 2:
                H[2, 2] = 1.0  # vx
            if n > 3:
                H[3, 3] = 1.0  # vy
            hx = np.array([px, py, vx, vy])
            return H, hx

        else:
            raise ValueError(f"Unknown sensor type: {stype}")

    def _wrap_angle(self, angle: float) -> float:
        """Wrap angle to [-π, π]."""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def _compute_innovation(self, z: np.ndarray, hx: np.ndarray,
                            sensor: SensorConfig) -> np.ndarray:
        """Compute innovation (measurement residual) with angle wrapping."""
        nu = z - hx

        # Wrap angular components for nonlinear sensor types
        if sensor.sensor_type == SensorType.RANGE_BEARING:
            nu[1] = self._wrap_angle(nu[1])
        elif sensor.sensor_type == SensorType.RANGE_DOPPLER:
            nu[1] = self._wrap_angle(nu[1])
        elif sensor.sensor_type == SensorType.BEARING_ONLY:
            nu[0] = self._wrap_angle(nu[0])

        return nu

    def _sequential_update(self, x: np.ndarray, P: np.ndarray,
                           z: np.ndarray, H: np.ndarray, R: np.ndarray,
                           hx: np.ndarray, sensor: SensorConfig
                           ) -> Tuple[np.ndarray, np.ndarray, float]:
        """
        Single-sensor Kalman update (EKF for nonlinear sensors).

        Returns updated (x, P, nis).
        """
        nu = self._compute_innovation(z, hx, sensor)
        S = H @ P @ H.T + R

        try:
            # Use analytic inverse for 1×1 and 2×2
            if S.shape == (1, 1):
                Si = np.array([[1.0 / S[0, 0]]])
            elif S.shape == (2, 2):
                det = S[0, 0] * S[1, 1] - S[0, 1] * S[1, 0]
                if abs(det) < 1e-30:
                    return x, P, 999.0
                Si = np.array([[S[1, 1], -S[0, 1]], [-S[1, 0], S[0, 0]]]) / det
            else:
                Si = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return x, P, 999.0

        # NIS for gating
        nis = float(nu @ Si @ nu)

        # Gate: reject if NIS too large (chi-squared threshold)
        dim_z = len(z)
        gate = {1: 10.83, 2: 13.82, 3: 16.27, 4: 18.47}.get(dim_z, 20.0)  # 99.9%
        if nis > gate:
            return x, P, nis

        K = P @ H.T @ Si
        x_new = x + K @ nu
        I_KH = np.eye(len(x)) - K @ H
        # Joseph form for numerical stability
        P_new = I_KH @ P @ I_KH.T + K @ R @ K.T

        return x_new, P_new, nis

    def fuse(self, tracker, measurements: List[SensorMeasurement]
             ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Fuse measurements from multiple sensors into the tracker state.

        This performs sequential Kalman updates on the tracker's internal
        state for each measurement. The tracker's IMM model probabilities
        and AOS system continue to operate on the primary sensor.

        Args:
            tracker: NxMimosaV40Sentinel instance (must be initialized)
            measurements: List of SensorMeasurement from different sensors

        Returns:
            (position, covariance) — fused state estimate
        """
        if not tracker.initialized or not measurements:
            return np.zeros(2), np.eye(2) * 999

        # Sort by timestamp for causal ordering
        measurements.sort(key=lambda m: m.timestamp)

        fused_nis = {}

        for meas in measurements:
            sid = meas.sensor_id
            if sid not in self.sensors:
                continue
            sensor = self.sensors[sid]
            if not sensor.active:
                continue

            z = meas.z.copy()
            # Apply bias correction (only up to available bias dimensions)
            bias = sensor.bias
            n_bias = min(len(bias), len(z))
            if n_bias > 0:
                z[:n_bias] -= bias[:n_bias]
            R = meas.R_override if meas.R_override is not None else sensor.R

            # Check range gate
            if sensor.max_range is not None:
                if sensor.sensor_type in (SensorType.RANGE_BEARING, SensorType.RANGE_DOPPLER):
                    if z[0] > sensor.max_range:
                        continue

            # === UPDATE EACH IMM MODEL ===
            for model_name in tracker.active_models:
                x_m = tracker.x[model_name]
                P_m = tracker.P[model_name]

                H, hx = self._build_H_and_hx(sensor, x_m)
                x_new, P_new, nis = self._sequential_update(
                    x_m, P_m, z, H, R, hx, sensor
                )

                # Only update if not rejected by gate
                if nis < 100:
                    tracker.x[model_name] = x_new
                    tracker.P[model_name] = P_new

            # === UPDATE PARALLEL CV FILTER ===
            H_cv, hx_cv = self._build_H_and_hx(sensor, tracker._cv_x)
            cv_x_new, cv_P_new, cv_nis = self._sequential_update(
                tracker._cv_x, tracker._cv_P, z, H_cv, R, hx_cv, sensor
            )
            if cv_nis < 100:
                tracker._cv_x = cv_x_new
                tracker._cv_P = cv_P_new

            # === UPDATE PARALLEL CA FILTER ===
            H_ca, hx_ca = self._build_H_and_hx(sensor, tracker._ca_x)
            ca_x_new, ca_P_new, ca_nis = self._sequential_update(
                tracker._ca_x, tracker._ca_P, z, H_ca, R, hx_ca, sensor
            )
            if ca_nis < 100:
                tracker._ca_x = ca_x_new
                tracker._ca_P = ca_P_new

            # === SENSOR HEALTH MONITORING ===
            health = self.health[sid]
            health.measurement_count += 1
            health.last_update_time = meas.timestamp
            health.nis_history.append(cv_nis)
            if len(health.nis_history) > self._nis_window:
                health.nis_history.pop(0)
            health.nis_avg = np.mean(health.nis_history)

            # Degradation detection: NIS >> expected means sensor issues
            dim_z = len(z)
            health.degraded = health.nis_avg > 3.0 * dim_z
            if cv_nis > 50:
                health.consecutive_rejects += 1
            else:
                health.consecutive_rejects = 0

            fused_nis[sid] = cv_nis

        # Return combined position estimate
        pos = tracker._get_combined_position()
        cov = tracker.P.get("CV", np.eye(4))[:2, :2]
        return pos, cov

    def fuse_information(self, tracker, measurements: List[SensorMeasurement]
                         ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Information-weighted fusion for simultaneous measurements.

        Converts to information space (Y = P⁻¹, y = P⁻¹·x),
        sums information contributions from all sensors,
        then converts back. Mathematically equivalent to sequential
        update but numerically better for many simultaneous sensors.

        Best when: all measurements arrive at same timestamp.
        """
        if not tracker.initialized or not measurements:
            return np.zeros(2), np.eye(2) * 999

        for model_name in tracker.active_models:
            x_m = tracker.x[model_name]
            P_m = tracker.P[model_name]
            n = len(x_m)

            try:
                Y = np.linalg.inv(P_m)  # Information matrix
                y = Y @ x_m             # Information vector
            except np.linalg.LinAlgError:
                continue

            for meas in measurements:
                sid = meas.sensor_id
                if sid not in self.sensors or not self.sensors[sid].active:
                    continue
                sensor = self.sensors[sid]
                z = meas.z.copy()
                n_bias = min(len(sensor.bias), len(z))
                if n_bias > 0:
                    z[:n_bias] -= sensor.bias[:n_bias]
                R = meas.R_override if meas.R_override is not None else sensor.R

                H, hx = self._build_H_and_hx(sensor, x_m)
                nu = self._compute_innovation(z, hx, sensor)

                try:
                    Ri = np.linalg.inv(R)
                except np.linalg.LinAlgError:
                    continue

                # Add information contribution
                Y += H.T @ Ri @ H
                y += H.T @ Ri @ (nu + H @ x_m)

            try:
                P_new = np.linalg.inv(Y)
                x_new = P_new @ y
                tracker.x[model_name] = x_new
                tracker.P[model_name] = P_new
            except np.linalg.LinAlgError:
                pass

        pos = tracker._get_combined_position()
        cov = tracker.P.get("CV", np.eye(4))[:2, :2]
        return pos, cov

    def get_health_report(self) -> Dict[str, Dict]:
        """Get health status of all sensors."""
        report = {}
        for sid, h in self.health.items():
            sensor = self.sensors.get(sid)
            report[sid] = {
                "type": sensor.sensor_type.value if sensor else "unknown",
                "active": sensor.active if sensor else False,
                "measurements": h.measurement_count,
                "nis_avg": round(h.nis_avg, 2),
                "degraded": h.degraded,
                "consecutive_rejects": h.consecutive_rejects,
                "last_update": h.last_update_time,
            }
        return report

    def __repr__(self) -> str:
        active = len(self.get_active_sensors())
        total = len(self.sensors)
        degraded = sum(1 for h in self.health.values() if h.degraded)
        return (f"MultiSensorFusionEngine({active}/{total} active"
                f"{f', {degraded} degraded' if degraded else ''})")


# === CONVENIENCE FACTORY FUNCTIONS ===

def make_radar_sensor(sensor_id: str, r_std: float = 50.0,
                      position: np.ndarray = None,
                      max_range: float = None,
                      update_rate: float = 1.0) -> SensorConfig:
    """Create a standard radar sensor (position measurement in Cartesian)."""
    return SensorConfig(
        sensor_id=sensor_id,
        sensor_type=SensorType.POSITION_XY,
        R=np.diag([r_std**2, r_std**2]),
        position=position if position is not None else np.zeros(2),
        max_range=max_range,
        update_rate=update_rate,
    )

def make_polar_radar_sensor(sensor_id: str, r_std: float = 50.0,
                            az_std_deg: float = 1.0,
                            position: np.ndarray = None,
                            max_range: float = None,
                            update_rate: float = 1.0) -> SensorConfig:
    """Create a radar sensor with range-bearing measurements."""
    return SensorConfig(
        sensor_id=sensor_id,
        sensor_type=SensorType.RANGE_BEARING,
        R=np.diag([r_std**2, np.radians(az_std_deg)**2]),
        position=position if position is not None else np.zeros(2),
        max_range=max_range,
        update_rate=update_rate,
    )

def make_doppler_radar_sensor(sensor_id: str, r_std: float = 50.0,
                              az_std_deg: float = 1.0,
                              rdot_std: float = 2.0,
                              position: np.ndarray = None,
                              max_range: float = None,
                              update_rate: float = 1.0) -> SensorConfig:
    """Create a Doppler radar sensor with range, bearing, and range-rate."""
    return SensorConfig(
        sensor_id=sensor_id,
        sensor_type=SensorType.RANGE_DOPPLER,
        R=np.diag([r_std**2, np.radians(az_std_deg)**2, rdot_std**2]),
        position=position if position is not None else np.zeros(2),
        max_range=max_range,
        update_rate=update_rate,
    )

def make_eo_sensor(sensor_id: str, az_std_deg: float = 0.1,
                   position: np.ndarray = None,
                   max_range: float = None,
                   update_rate: float = 30.0) -> SensorConfig:
    """Create an EO/IR passive sensor (bearing-only)."""
    return SensorConfig(
        sensor_id=sensor_id,
        sensor_type=SensorType.BEARING_ONLY,
        R=np.array([[np.radians(az_std_deg)**2]]),
        position=position if position is not None else np.zeros(2),
        max_range=max_range,
        update_rate=update_rate,
    )

def make_adsb_sensor(sensor_id: str = "adsb",
                     pos_std: float = 10.0,
                     vel_std: float = 1.0) -> SensorConfig:
    """Create an ADS-B cooperative sensor (position + velocity)."""
    return SensorConfig(
        sensor_id=sensor_id,
        sensor_type=SensorType.ADS_B,
        R=np.diag([pos_std**2, pos_std**2, vel_std**2, vel_std**2]),
        update_rate=1.0,
    )

def make_esm_sensor(sensor_id: str, az_std_deg: float = 2.0,
                    position: np.ndarray = None,
                    update_rate: float = 0.5) -> SensorConfig:
    """Create an ESM/ELINT passive sensor (bearing-only, lower accuracy)."""
    return SensorConfig(
        sensor_id=sensor_id,
        sensor_type=SensorType.BEARING_ONLY,
        R=np.array([[np.radians(az_std_deg)**2]]),
        position=position if position is not None else np.zeros(2),
        update_rate=update_rate,
    )

def make_gps_sensor(sensor_id: str = "gps",
                    pos_std: float = 3.0) -> SensorConfig:
    """Create a GPS sensor (Cartesian position)."""
    return SensorConfig(
        sensor_id=sensor_id,
        sensor_type=SensorType.POSITION_XY,
        R=np.diag([pos_std**2, pos_std**2]),
        update_rate=1.0,
    )
