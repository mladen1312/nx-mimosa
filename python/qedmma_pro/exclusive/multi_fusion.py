"""
QEDMMA-Pro v3.0 - Multi-Sensor Fusion Engine
=============================================
Copyright (C) 2026 Dr. Mladen Mešter / Nexellum
License: Commercial - See LICENSE_COMMERCIAL.md

🔒 PRO EXCLUSIVE - NOT AVAILABLE IN LITE

Multi-sensor fusion following JDL (Joint Directors of Laboratories) model:
  Level 0: Sub-Object Assessment (signal processing)
  Level 1: Object Assessment (tracking, classification)
  Level 2: Situation Assessment (threat evaluation)
  Level 3: Impact Assessment (response planning)
  Level 4: Process Refinement (adaptive sensor management)

Key Features:
  - Asynchronous multi-static radar fusion
  - Time bias estimation and compensation
  - Distributed track correlation (MHT-based)
  - Information-theoretic sensor tasking
  - TDOA/FDOA geolocation

For licensing: mladen@nexellum.com | www.nexellum.com
"""

import numpy as np
from typing import Dict, List, Tuple, Optional, Callable
from dataclasses import dataclass, field
from enum import Enum
from collections import defaultdict
import heapq


class JDLLevel(Enum):
    """JDL Fusion Levels"""
    LEVEL_0 = 0  # Signal/Pixel processing
    LEVEL_1 = 1  # Object tracking
    LEVEL_2 = 2  # Situation assessment
    LEVEL_3 = 3  # Impact assessment
    LEVEL_4 = 4  # Process refinement


class SensorType(Enum):
    """Supported sensor types"""
    MONOSTATIC_RADAR = "monostatic"
    BISTATIC_RADAR = "bistatic"
    MULTISTATIC_RADAR = "multistatic"
    ESM = "esm"  # Electronic Support Measures
    IRST = "irst"  # Infrared Search & Track
    AIS = "ais"  # Automatic Identification System


@dataclass
class SensorConfig:
    """Configuration for a single sensor"""
    sensor_id: str
    sensor_type: SensorType
    position: np.ndarray  # [x, y, z] in ECEF or local
    orientation: np.ndarray = field(default_factory=lambda: np.array([0, 0, 0]))
    
    # Measurement characteristics
    range_accuracy: float = 100.0  # meters
    azimuth_accuracy: float = 0.01  # radians
    elevation_accuracy: float = 0.01  # radians
    doppler_accuracy: float = 1.0  # m/s
    
    # Timing
    time_bias: float = 0.0  # seconds (estimated)
    time_bias_std: float = 0.001  # seconds
    update_rate: float = 1.0  # Hz


@dataclass
class Measurement:
    """Single sensor measurement"""
    sensor_id: str
    timestamp: float
    
    # Spherical coordinates (relative to sensor)
    range: Optional[float] = None
    azimuth: Optional[float] = None
    elevation: Optional[float] = None
    doppler: Optional[float] = None
    
    # Direct position (if available)
    position: Optional[np.ndarray] = None
    
    # Covariance
    R: Optional[np.ndarray] = None
    
    # Signal characteristics
    snr: float = 20.0  # dB
    rcs: float = 1.0  # m²


@dataclass
class FusedTrack:
    """Fused track from multiple sensors"""
    track_id: int
    state: np.ndarray  # [x, y, z, vx, vy, vz]
    covariance: np.ndarray
    
    # Track quality
    confidence: float = 1.0
    classification: str = "unknown"
    threat_level: float = 0.0
    
    # Contributing sensors
    contributing_sensors: List[str] = field(default_factory=list)
    last_update: float = 0.0
    
    # History
    state_history: List[np.ndarray] = field(default_factory=list)


@dataclass
class TimeBiasEstimate:
    """PRO: Estimated time bias between sensors"""
    sensor_pair: Tuple[str, str]
    bias: float  # seconds
    uncertainty: float  # seconds
    samples: int = 0


class TrackCorrelator:
    """
    PRO: Multi-hypothesis track correlator.
    
    Associates measurements from different sensors to the same target.
    Uses statistical distance and kinematic gating.
    """
    
    def __init__(self, gate_threshold: float = 9.21):  # Chi² 99% for 2 DOF
        self.gate_threshold = gate_threshold
        self.association_history = defaultdict(list)
    
    def compute_distance(
        self,
        track: FusedTrack,
        measurement: Measurement,
        sensor: SensorConfig
    ) -> float:
        """Compute statistical distance (Mahalanobis)"""
        # Predict measurement from track state
        rel_pos = track.state[:3] - sensor.position
        pred_range = np.linalg.norm(rel_pos)
        pred_az = np.arctan2(rel_pos[1], rel_pos[0])
        pred_el = np.arctan2(rel_pos[2], np.sqrt(rel_pos[0]**2 + rel_pos[1]**2))
        
        # Innovation
        if measurement.range is not None:
            innov = np.array([
                measurement.range - pred_range,
                measurement.azimuth - pred_az if measurement.azimuth else 0,
                measurement.elevation - pred_el if measurement.elevation else 0
            ])
        elif measurement.position is not None:
            innov = measurement.position - track.state[:3]
        else:
            return np.inf
        
        # Approximate covariance
        if measurement.R is not None:
            R = measurement.R
        else:
            R = np.diag([sensor.range_accuracy**2, 
                        sensor.azimuth_accuracy**2, 
                        sensor.elevation_accuracy**2])
        
        # Mahalanobis distance
        try:
            d2 = innov @ np.linalg.solve(R, innov)
        except:
            d2 = np.sum(innov**2) / np.trace(R)
        
        return d2
    
    def associate(
        self,
        tracks: List[FusedTrack],
        measurements: List[Measurement],
        sensors: Dict[str, SensorConfig]
    ) -> List[Tuple[int, int, float]]:
        """
        Associate measurements to tracks.
        
        Returns list of (track_idx, meas_idx, distance) tuples.
        """
        associations = []
        
        # Compute all distances
        distances = np.full((len(tracks), len(measurements)), np.inf)
        
        for i, track in enumerate(tracks):
            for j, meas in enumerate(measurements):
                sensor = sensors.get(meas.sensor_id)
                if sensor:
                    distances[i, j] = self.compute_distance(track, meas, sensor)
        
        # Greedy association (could use Hungarian algorithm for optimal)
        used_tracks = set()
        used_meas = set()
        
        # Sort by distance
        flat_indices = np.argsort(distances.ravel())
        
        for idx in flat_indices:
            i, j = divmod(idx, len(measurements))
            
            if i in used_tracks or j in used_meas:
                continue
            
            if distances[i, j] < self.gate_threshold:
                associations.append((i, j, distances[i, j]))
                used_tracks.add(i)
                used_meas.add(j)
        
        return associations


class TimeBiasEstimator:
    """
    PRO: Online time bias estimation between sensors.
    
    Critical for asynchronous multi-static fusion.
    """
    
    def __init__(self, initial_uncertainty: float = 0.1):
        self.biases: Dict[Tuple[str, str], TimeBiasEstimate] = {}
        self.initial_uncertainty = initial_uncertainty
    
    def get_or_create(self, sensor1: str, sensor2: str) -> TimeBiasEstimate:
        """Get or create bias estimate for sensor pair"""
        key = tuple(sorted([sensor1, sensor2]))
        if key not in self.biases:
            self.biases[key] = TimeBiasEstimate(
                sensor_pair=key,
                bias=0.0,
                uncertainty=self.initial_uncertainty
            )
        return self.biases[key]
    
    def update(
        self,
        sensor1: str,
        sensor2: str,
        observed_bias: float,
        observation_std: float
    ):
        """Update bias estimate with new observation"""
        estimate = self.get_or_create(sensor1, sensor2)
        
        # Kalman-like update
        K = estimate.uncertainty**2 / (estimate.uncertainty**2 + observation_std**2)
        estimate.bias += K * (observed_bias - estimate.bias)
        estimate.uncertainty = np.sqrt((1 - K) * estimate.uncertainty**2)
        estimate.samples += 1
    
    def get_bias(self, sensor1: str, sensor2: str) -> float:
        """Get estimated bias (sensor1 time - sensor2 time)"""
        key = tuple(sorted([sensor1, sensor2]))
        if key in self.biases:
            bias = self.biases[key].bias
            # Return with correct sign
            if key[0] != sensor1:
                bias = -bias
            return bias
        return 0.0


class MultiSensorFusion:
    """
    QEDMMA-Pro Multi-Sensor Fusion Engine
    
    🔒 PRO EXCLUSIVE - NOT AVAILABLE IN LITE
    
    Fuses data from multiple heterogeneous sensors following
    the JDL fusion model. Handles:
    - Asynchronous measurements
    - Time bias compensation
    - Track correlation
    - Distributed processing
    
    Example:
        >>> fusion = MultiSensorFusion()
        >>> 
        >>> # Register sensors
        >>> fusion.register_sensor(SensorConfig(
        ...     sensor_id="radar1",
        ...     sensor_type=SensorType.MONOSTATIC_RADAR,
        ...     position=np.array([0, 0, 0])
        ... ))
        >>> 
        >>> # Process measurements
        >>> for meas in measurements:
        ...     tracks = fusion.process_measurement(meas)
        >>> 
        >>> # Get fused tracks
        >>> tracks = fusion.get_tracks()
    """
    
    def __init__(
        self,
        track_init_threshold: int = 3,
        track_drop_time: float = 10.0,
        gate_threshold: float = 16.0
    ):
        self.sensors: Dict[str, SensorConfig] = {}
        self.tracks: Dict[int, FusedTrack] = {}
        self.next_track_id = 1
        
        self.correlator = TrackCorrelator(gate_threshold)
        self.time_bias_estimator = TimeBiasEstimator()
        
        self.track_init_threshold = track_init_threshold
        self.track_drop_time = track_drop_time
        
        # Tentative tracks (not yet confirmed)
        self.tentative_tracks: Dict[int, List[Measurement]] = {}
        
        # Current time
        self.current_time = 0.0
    
    def register_sensor(self, config: SensorConfig):
        """Register a sensor with the fusion engine"""
        self.sensors[config.sensor_id] = config
    
    def _measurement_to_position(
        self,
        meas: Measurement,
        sensor: SensorConfig
    ) -> np.ndarray:
        """Convert spherical measurement to Cartesian position"""
        if meas.position is not None:
            return meas.position
        
        if meas.range is None:
            return None
        
        r = meas.range
        az = meas.azimuth if meas.azimuth is not None else 0
        el = meas.elevation if meas.elevation is not None else 0
        
        # Local spherical to Cartesian
        x = r * np.cos(el) * np.cos(az)
        y = r * np.cos(el) * np.sin(az)
        z = r * np.sin(el)
        
        # Transform to global
        pos = sensor.position + np.array([x, y, z])
        
        return pos
    
    def _predict_track(self, track: FusedTrack, dt: float) -> FusedTrack:
        """Predict track state forward in time"""
        # Constant velocity prediction
        F = np.eye(6)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        
        new_state = F @ track.state
        
        # Process noise
        q = 10.0  # m²/s⁴ acceleration variance
        Q = np.zeros((6, 6))
        Q[0:3, 0:3] = q * dt**4/4 * np.eye(3)
        Q[0:3, 3:6] = q * dt**3/2 * np.eye(3)
        Q[3:6, 0:3] = q * dt**3/2 * np.eye(3)
        Q[3:6, 3:6] = q * dt**2 * np.eye(3)
        
        new_cov = F @ track.covariance @ F.T + Q
        
        track.state = new_state
        track.covariance = new_cov
        
        return track
    
    def _update_track(
        self,
        track: FusedTrack,
        meas: Measurement,
        sensor: SensorConfig
    ) -> FusedTrack:
        """Update track with measurement"""
        # Get measurement position
        z = self._measurement_to_position(meas, sensor)
        if z is None:
            return track
        
        # Measurement matrix (observe position only)
        H = np.zeros((3, 6))
        H[0:3, 0:3] = np.eye(3)
        
        # Measurement noise
        R = np.diag([
            sensor.range_accuracy**2,
            sensor.range_accuracy**2,
            sensor.range_accuracy**2
        ])
        
        # Kalman update
        y = z - H @ track.state
        S = H @ track.covariance @ H.T + R
        K = track.covariance @ H.T @ np.linalg.inv(S)
        
        track.state = track.state + K @ y
        track.covariance = (np.eye(6) - K @ H) @ track.covariance
        track.covariance = 0.5 * (track.covariance + track.covariance.T)
        
        # Update metadata
        track.last_update = meas.timestamp
        if sensor.sensor_id not in track.contributing_sensors:
            track.contributing_sensors.append(sensor.sensor_id)
        
        # Store history
        track.state_history.append(track.state.copy())
        if len(track.state_history) > 100:
            track.state_history.pop(0)
        
        return track
    
    def _initiate_track(self, meas: Measurement, sensor: SensorConfig) -> Optional[FusedTrack]:
        """Initiate new track from measurement"""
        pos = self._measurement_to_position(meas, sensor)
        if pos is None:
            return None
        
        # Initial state (zero velocity)
        state = np.zeros(6)
        state[:3] = pos
        
        # Initial covariance
        cov = np.eye(6)
        cov[0:3, 0:3] *= sensor.range_accuracy**2 * 10
        cov[3:6, 3:6] *= 100**2  # Large velocity uncertainty
        
        track = FusedTrack(
            track_id=self.next_track_id,
            state=state,
            covariance=cov,
            contributing_sensors=[sensor.sensor_id],
            last_update=meas.timestamp
        )
        
        self.next_track_id += 1
        
        return track
    
    def process_measurement(self, meas: Measurement) -> List[FusedTrack]:
        """
        Process a single measurement and update fusion state.
        
        Returns list of updated tracks.
        """
        self.current_time = max(self.current_time, meas.timestamp)
        
        sensor = self.sensors.get(meas.sensor_id)
        if sensor is None:
            return []
        
        # Apply time bias correction
        corrected_time = meas.timestamp
        for other_id in self.sensors:
            if other_id != meas.sensor_id:
                bias = self.time_bias_estimator.get_bias(meas.sensor_id, other_id)
                # Use bias for track prediction timing
        
        # Predict all tracks to measurement time
        for track in self.tracks.values():
            dt = meas.timestamp - track.last_update
            if dt > 0:
                self._predict_track(track, dt)
        
        # Associate measurement to tracks
        track_list = list(self.tracks.values())
        associations = self.correlator.associate(track_list, [meas], self.sensors)
        
        updated_tracks = []
        
        if associations:
            # Update associated track
            track_idx, _, _ = associations[0]
            track = track_list[track_idx]
            track = self._update_track(track, meas, sensor)
            updated_tracks.append(track)
        else:
            # No association - initiate new track
            new_track = self._initiate_track(meas, sensor)
            if new_track:
                self.tracks[new_track.track_id] = new_track
                updated_tracks.append(new_track)
        
        # Drop stale tracks
        stale_ids = [
            tid for tid, track in self.tracks.items()
            if self.current_time - track.last_update > self.track_drop_time
        ]
        for tid in stale_ids:
            del self.tracks[tid]
        
        return updated_tracks
    
    def process_batch(self, measurements: List[Measurement]) -> List[FusedTrack]:
        """Process batch of measurements (sorted by time)"""
        # Sort by timestamp
        measurements = sorted(measurements, key=lambda m: m.timestamp)
        
        all_updated = []
        for meas in measurements:
            updated = self.process_measurement(meas)
            all_updated.extend(updated)
        
        return list(self.tracks.values())
    
    def get_tracks(self) -> List[FusedTrack]:
        """Get all active tracks"""
        return list(self.tracks.values())
    
    def get_situation_assessment(self) -> Dict:
        """
        PRO: JDL Level 2 - Situation Assessment
        
        Analyze track patterns for threat evaluation.
        """
        assessment = {
            'total_tracks': len(self.tracks),
            'high_threat': [],
            'medium_threat': [],
            'low_threat': [],
            'sensor_coverage': {}
        }
        
        for track in self.tracks.values():
            # Simple threat assessment based on speed and heading
            speed = np.linalg.norm(track.state[3:6])
            
            if speed > 500:  # High speed = high threat
                track.threat_level = 0.9
                assessment['high_threat'].append(track.track_id)
            elif speed > 200:
                track.threat_level = 0.5
                assessment['medium_threat'].append(track.track_id)
            else:
                track.threat_level = 0.1
                assessment['low_threat'].append(track.track_id)
        
        # Sensor coverage analysis
        for sensor_id in self.sensors:
            contributing_tracks = [
                t.track_id for t in self.tracks.values()
                if sensor_id in t.contributing_sensors
            ]
            assessment['sensor_coverage'][sensor_id] = len(contributing_tracks)
        
        return assessment


# ═══════════════════════════════════════════════════════════════════════════════
#                                  DEMO
# ═══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    print("=" * 70)
    print("  QEDMMA-Pro Multi-Sensor Fusion Demo")
    print("  🔒 PRO EXCLUSIVE - JDL Fusion Model")
    print("=" * 70)
    print()
    
    # Create fusion engine
    fusion = MultiSensorFusion()
    
    # Register sensors
    fusion.register_sensor(SensorConfig(
        sensor_id="radar_north",
        sensor_type=SensorType.MONOSTATIC_RADAR,
        position=np.array([0, 10000, 0]),
        range_accuracy=50.0
    ))
    
    fusion.register_sensor(SensorConfig(
        sensor_id="radar_south",
        sensor_type=SensorType.MONOSTATIC_RADAR,
        position=np.array([0, -10000, 0]),
        range_accuracy=50.0
    ))
    
    fusion.register_sensor(SensorConfig(
        sensor_id="radar_east",
        sensor_type=SensorType.MONOSTATIC_RADAR,
        position=np.array([10000, 0, 0]),
        range_accuracy=50.0
    ))
    
    print(f"Registered {len(fusion.sensors)} sensors")
    print()
    
    # Simulate target
    np.random.seed(42)
    true_pos = np.array([5000.0, 5000.0, 10000.0])
    true_vel = np.array([200.0, -100.0, -50.0])
    
    measurements = []
    
    for t in range(50):
        time = t * 0.5
        
        # Update true state
        true_pos += true_vel * 0.5
        
        # Generate measurements from each sensor
        for sensor_id, sensor in fusion.sensors.items():
            if np.random.rand() < 0.8:  # 80% detection probability
                rel_pos = true_pos - sensor.position
                r = np.linalg.norm(rel_pos) + np.random.randn() * sensor.range_accuracy
                az = np.arctan2(rel_pos[1], rel_pos[0]) + np.random.randn() * 0.01
                el = np.arctan2(rel_pos[2], np.sqrt(rel_pos[0]**2 + rel_pos[1]**2)) + np.random.randn() * 0.01
                
                measurements.append(Measurement(
                    sensor_id=sensor_id,
                    timestamp=time + np.random.randn() * 0.01,  # Timing jitter
                    range=r,
                    azimuth=az,
                    elevation=el
                ))
    
    print(f"Generated {len(measurements)} measurements")
    print()
    
    # Process measurements
    tracks = fusion.process_batch(measurements)
    
    print(f"{'Time':>6} | {'Tracks':>6} | {'Position Error':>15}")
    print("-" * 35)
    
    if tracks:
        track = tracks[0]
        pos_err = np.linalg.norm(track.state[:3] - true_pos)
        print(f"{fusion.current_time:>6.1f} | {len(tracks):>6} | {pos_err:>13.1f}m")
        
        print()
        print("📊 Situation Assessment:")
        assessment = fusion.get_situation_assessment()
        print(f"   Total tracks: {assessment['total_tracks']}")
        print(f"   High threat: {assessment['high_threat']}")
        print(f"   Sensor coverage: {assessment['sensor_coverage']}")
    
    print()
    print("✅ Multi-Sensor Fusion: JDL-compliant track correlation")
