"""
NX-MIMOSA v6.1.0 — Advanced ECCM Module
Ported from QEDMMA Radar System FPGA implementation to Python/NumPy.

Three capabilities ported from QEDMMA hardware RTL:
1. MLCFARDetector — 6-feature ML environment classifier (from ml_cfar_engine.sv)
2. JammerLocalizer — TDOA-based emitter geolocation (from jammer_localizer.sv)
3. AdaptiveIntegrator — Dynamic SNR enhancement (from integration_controller.sv)

Author: Dr. Mladen Mešter
Copyright (c) 2026 Nexellum d.o.o. — AGPL v3

References:
- QEDMMA ECCM Architecture v2.0.1
- Neri, "Introduction to Electronic Defense Systems," Artech House
- Richards et al., "Principles of Modern Radar," SciTech, Ch. 16
"""

import numpy as np
from dataclasses import dataclass, field
from enum import IntEnum, auto
from typing import Dict, List, Optional, Tuple, Any
from collections import deque


# =============================================================================
# [REQ-V610-MLCFAR] ML-CFAR Environment Classifier
# Ported from: QEDMMA v2/rtl/eccm/ml_cfar_engine.sv (553 lines SV → Python)
# =============================================================================

class EnvironmentClass(IntEnum):
    """Environment classification from ML-CFAR feature analysis."""
    CLEAR = 0           # Normal operation
    CLUTTER = 1         # Heavy clutter, adapt threshold up
    NOISE_JAMMING = 2   # Barrage/spot noise jammer present
    DECEPTION = 3       # RGPO/VGPO/DRFM deception jamming


@dataclass
class MLCFARFeatures:
    """6-feature vector extracted per track, matching QEDMMA FPGA implementation.
    
    The FPGA ml_cfar_engine.sv extracts these from range-Doppler cells.
    This Python version extracts equivalent features from track-level statistics,
    enabling the same classification without access to raw range-Doppler data.
    """
    power_ratio: float = 0.0          # Feature 1: NIS / baseline NIS
    guard_ratio: float = 0.0          # Feature 2: Local NIS variance / global variance
    spectral_flatness: float = 0.0    # Feature 3: Kurtosis proxy of NIS sequence
    temporal_correlation: float = 0.0  # Feature 4: Autocorrelation lag-1 of innovations
    range_derivative: float = 0.0      # Feature 5: Range residual gradient
    doppler_derivative: float = 0.0    # Feature 6: Velocity residual gradient


class MLCFARDetector:
    """[REQ-V610-MLCFAR-01] ML-enhanced environment classifier.
    
    Replaces simple threshold-based ECM detection with 6-feature classification.
    Uses decision tree logic matching the QEDMMA FPGA inference engine.
    
    The decision tree thresholds were derived from QEDMMA ECCM scenario simulations:
    - 10,000 clear-sky sweeps (baseline statistics)
    - 5,000 barrage jamming scenarios (50 kW, 100 kW)
    - 5,000 deception scenarios (RGPO, VGPO, DRFM)
    - 3,000 heavy clutter scenarios (urban, mountain, sea)
    
    Feature extraction runs per-track per-scan. Classification latency: O(1).
    
    Example:
        detector = MLCFARDetector()
        for scan in radar_scans:
            for track in confirmed_tracks:
                features = detector.extract_features(track.track_id, nis, innovation, velocity)
                env_class, confidence = detector.classify(features)
                if env_class == EnvironmentClass.NOISE_JAMMING:
                    # Inflate measurement noise, switch to bearing-only
                    track.R *= detector.get_R_multiplier(env_class)
    """
    
    # Decision tree thresholds (from QEDMMA FPGA ml_weights[0:63])
    # These match the Q16.16 fixed-point values in ml_cfar_engine.sv
    THRESH_POWER_HIGH = 4.0       # NIS ratio above which jamming likely
    THRESH_POWER_LOW = 0.3        # NIS ratio below which blanking likely
    THRESH_KURTOSIS_FLAT = 2.0    # Kurtosis < 2.0 → flat spectrum (jamming)
    THRESH_KURTOSIS_SPIKY = 6.0   # Kurtosis > 6.0 → impulsive (deception)
    THRESH_TEMPORAL_CORR = 0.7    # Autocorrelation > 0.7 → coherent interference
    THRESH_RANGE_DERIV = 3.0      # Sigma multiplier for range pull-off
    THRESH_DOPPLER_DERIV = 3.0    # Sigma multiplier for velocity pull-off
    
    def __init__(self, window: int = 20, min_samples: int = 8):
        """Initialize ML-CFAR detector.
        
        Args:
            window: Feature extraction sliding window (scans)
            min_samples: Minimum samples before classification activates
        """
        self.window = window
        self.min_samples = min_samples
        self._history: Dict[int, Dict[str, deque]] = {}
    
    def _get_history(self, track_id: int) -> Dict[str, deque]:
        if track_id not in self._history:
            self._history[track_id] = {
                'nis': deque(maxlen=self.window * 3),
                'innovation_range': deque(maxlen=self.window * 3),
                'innovation_bearing': deque(maxlen=self.window * 3),
                'range_rate': deque(maxlen=self.window * 3),
                'was_hit': deque(maxlen=self.window * 3),
                'baseline_nis_mean': None,
                'baseline_nis_std': None,
                'n_updates': 0,
            }
        return self._history[track_id]
    
    def extract_features(self, track_id: int, nis: float,
                         innovation: Optional[np.ndarray] = None,
                         velocity: Optional[np.ndarray] = None,
                         was_hit: bool = True) -> MLCFARFeatures:
        """[REQ-V610-MLCFAR-02] Extract 6-feature vector from track statistics.
        
        Maps QEDMMA FPGA range-Doppler cell features to track-level equivalents:
        - FPGA Feature 1 (cell power / reference) → NIS / baseline NIS
        - FPGA Feature 2 (guard cell ratio) → local NIS variance / global
        - FPGA Feature 3 (spectral flatness) → kurtosis of NIS sequence
        - FPGA Feature 4 (temporal correlation) → lag-1 autocorrelation
        - FPGA Feature 5 (range derivative) → range innovation gradient
        - FPGA Feature 6 (Doppler derivative) → velocity innovation gradient
        """
        h = self._get_history(track_id)
        h['nis'].append(nis)
        h['was_hit'].append(was_hit)
        h['n_updates'] += 1
        
        if innovation is not None:
            h['innovation_range'].append(innovation[0] if len(innovation) > 0 else 0.0)
            if len(innovation) > 1:
                h['innovation_bearing'].append(innovation[1])
        
        if velocity is not None:
            speed = np.linalg.norm(velocity[:2]) if len(velocity) >= 2 else np.linalg.norm(velocity)
            h['range_rate'].append(speed)
        
        features = MLCFARFeatures()
        
        if h['n_updates'] < self.min_samples:
            return features
        
        nis_arr = np.array(h['nis'])
        recent = nis_arr[-self.window:]
        
        # Establish baseline from first window (clear-sky assumption)
        if h['baseline_nis_mean'] is None and len(nis_arr) >= self.window:
            h['baseline_nis_mean'] = float(np.mean(nis_arr[:self.window]))
            h['baseline_nis_std'] = float(max(np.std(nis_arr[:self.window]), 0.1))
        
        baseline_mean = h['baseline_nis_mean'] if h['baseline_nis_mean'] is not None else float(np.mean(recent))
        baseline_std = h['baseline_nis_std'] if h['baseline_nis_std'] is not None else max(float(np.std(recent)), 0.1)
        
        # Feature 1: Power ratio (NIS / baseline)
        features.power_ratio = float(np.mean(recent)) / max(baseline_mean, 0.01)
        
        # Feature 2: Guard ratio (local variance / global variance)
        if len(recent) >= 4:
            local_var = float(np.var(recent[-4:]))
            global_var = max(float(np.var(nis_arr)), 0.01)
            features.guard_ratio = local_var / global_var
        
        # Feature 3: Spectral flatness (kurtosis of NIS sequence)
        # Low kurtosis (~2) = flat/uniform → barrage jamming
        # High kurtosis (>6) = spiky → deception pulses
        # Normal (~3) = Gaussian → clear or clutter
        if len(recent) >= 8:
            mu = np.mean(recent)
            sigma = max(np.std(recent), 1e-6)
            features.spectral_flatness = float(np.mean(((recent - mu) / sigma) ** 4))
        
        # Feature 4: Temporal correlation (lag-1 autocorrelation of NIS)
        # High correlation → coherent interference (DRFM, CW jammer)
        # Low correlation → random (thermal noise, clutter)
        if len(recent) >= 4:
            centered = recent - np.mean(recent)
            var = np.var(recent)
            if var > 1e-10:
                autocorr = float(np.mean(centered[:-1] * centered[1:])) / var
                features.temporal_correlation = np.clip(autocorr, -1.0, 1.0)
        
        # Feature 5: Range derivative (innovation gradient → RGPO detection)
        # Normalized against INITIAL baseline innovation std (not contaminated by RGPO)
        if len(h['innovation_range']) >= 6:
            range_innov = np.array(list(h['innovation_range']))
            recent_ri = range_innov[-4:]
            # Use first window samples as clean baseline (before any RGPO onset)
            baseline_window = min(self.window, len(range_innov) - 4)
            baseline_ri = range_innov[:baseline_window] if baseline_window > 2 else range_innov
            baseline_ri_std = max(np.std(baseline_ri), 1.0)
            deriv = np.abs(np.mean(np.diff(recent_ri)))
            features.range_derivative = float(deriv / baseline_ri_std)
        
        # Feature 6: Doppler derivative (velocity gradient → VGPO detection)
        # Same approach: use initial window for baseline
        if len(h['range_rate']) >= 6:
            rates = np.array(list(h['range_rate']))
            recent_rr = rates[-4:]
            baseline_window = min(self.window, len(rates) - 4)
            baseline_rr = rates[:baseline_window] if baseline_window > 2 else rates
            baseline_rr_std = max(np.std(baseline_rr), 1.0)
            rate_diff = np.max(np.abs(np.diff(recent_rr)))
            features.doppler_derivative = float(rate_diff / baseline_rr_std)
        
        return features
    
    def classify(self, features: MLCFARFeatures) -> Tuple[EnvironmentClass, float]:
        """[REQ-V610-MLCFAR-03] Classify environment using decision tree.
        
        Matches QEDMMA FPGA inference logic (ml_cfar_engine.sv lines 380-420).
        Decision tree structure:
        
            power_ratio > 4.0?
            ├── YES → kurtosis < 2.0? 
            │         ├── YES → NOISE_JAMMING (flat spectrum)
            │         └── NO → temporal_corr > 0.7?
            │                  ├── YES → DECEPTION (coherent false signal)
            │                  └── NO → CLUTTER (high but random)
            └── NO → range_deriv > 3σ OR doppler_deriv > 3σ?
                      ├── YES → DECEPTION (pull-off detected)
                      └── NO → power_ratio < 0.3?
                               ├── YES → NOISE_JAMMING (blanking/desensitization)
                               └── NO → CLEAR
        
        Returns:
            (environment_class, confidence) where confidence is 0.0-1.0
        """
        f = features
        
        # Branch 1: High power ratio → jamming or clutter
        if f.power_ratio > self.THRESH_POWER_HIGH:
            if f.spectral_flatness < self.THRESH_KURTOSIS_FLAT:
                # Flat spectrum + high power = barrage noise jamming
                conf = min(1.0, f.power_ratio / (self.THRESH_POWER_HIGH * 2))
                return EnvironmentClass.NOISE_JAMMING, conf
            elif f.temporal_correlation > self.THRESH_TEMPORAL_CORR:
                # Correlated + high power = coherent deception
                conf = min(1.0, f.temporal_correlation)
                return EnvironmentClass.DECEPTION, conf
            else:
                # High power, not flat, not correlated → clutter
                conf = min(1.0, (f.power_ratio - self.THRESH_POWER_HIGH) / self.THRESH_POWER_HIGH)
                return EnvironmentClass.CLUTTER, conf
        
        # Branch 2: Pull-off detection (range or Doppler derivative anomaly)
        if (f.range_derivative > self.THRESH_RANGE_DERIV or 
            f.doppler_derivative > self.THRESH_DOPPLER_DERIV):
            conf = min(1.0, max(f.range_derivative, f.doppler_derivative) / 
                       max(self.THRESH_RANGE_DERIV, self.THRESH_DOPPLER_DERIV) / 2)
            return EnvironmentClass.DECEPTION, conf
        
        # Branch 3: Low power ratio → potential blanking/desensitization
        if f.power_ratio < self.THRESH_POWER_LOW:
            conf = min(1.0, (self.THRESH_POWER_LOW - f.power_ratio) / self.THRESH_POWER_LOW)
            return EnvironmentClass.NOISE_JAMMING, conf
        
        # Default: clear environment
        return EnvironmentClass.CLEAR, 1.0 - min(0.5, abs(f.power_ratio - 1.0))
    
    def get_R_multiplier(self, env_class: EnvironmentClass) -> float:
        """[REQ-V610-MLCFAR-04] Get measurement noise inflation factor.
        
        When jamming is detected, inflate R matrix to prevent filter divergence.
        Values match QEDMMA integration_controller.sv adaptive thresholds.
        """
        return {
            EnvironmentClass.CLEAR: 1.0,
            EnvironmentClass.CLUTTER: 2.0,
            EnvironmentClass.NOISE_JAMMING: 5.0,
            EnvironmentClass.DECEPTION: 10.0,
        }[env_class]
    
    def get_integration_pulses(self, env_class: EnvironmentClass) -> int:
        """Recommended integration pulses (from integration_controller.sv)."""
        return {
            EnvironmentClass.CLEAR: 10,
            EnvironmentClass.CLUTTER: 20,
            EnvironmentClass.NOISE_JAMMING: 50,
            EnvironmentClass.DECEPTION: 20,
        }[env_class]
    
    def clear_track(self, track_id: int):
        """Remove track from ML-CFAR monitoring."""
        self._history.pop(track_id, None)


# =============================================================================
# [REQ-V610-TDOA] Jammer Localizer — TDOA Emitter Geolocation
# Ported from: QEDMMA v2/rtl/eccm/jammer_localizer.sv (402 lines SV → Python)
# =============================================================================

@dataclass
class EmitterTrack:
    """Tracked emitter state from TDOA measurements."""
    emitter_id: int
    position: np.ndarray        # [x, y, z] in meters (ENU)
    velocity: np.ndarray        # [vx, vy, vz] in m/s
    covariance: np.ndarray      # 6×6 state covariance
    power_estimate_dBm: float   # Estimated radiated power
    confidence: float           # Track quality 0-1
    classification: str         # 'NOISE', 'DECEPTION', 'DRFM', 'UNKNOWN'
    n_updates: int = 0
    last_update_time: float = 0.0


class JammerLocalizer:
    """[REQ-V610-TDOA-01] TDOA-based emitter geolocation with Kalman tracking.
    
    Software port of QEDMMA jammer_localizer.sv FPGA module.
    
    Given 3+ receivers with known positions and time-of-arrival measurements,
    estimates emitter position via hyperbolic intersection and maintains
    a continuous Kalman track of the emitter.
    
    Algorithm:
    1. Compute TDOA pairs relative to reference receiver (node 0)
    2. Form hyperbolic equations: c·Δt_ij = |r - r_j| - |r - r_i|
    3. Linearize and solve via weighted least squares
    4. Feed position estimate into 6-state Kalman filter (pos + vel)
    5. Output: emitter position, velocity, confidence, classification
    
    Performance (from QEDMMA validation):
    - <1 km CEP at 500 km range with 6 nodes, 100 km baseline
    - Converges in 3-5 measurements
    - Tracks moving emitters (aircraft, ships)
    
    Cross-sector applications:
    - Spectrum enforcement (rogue transmitter geolocation)
    - Counter-UAS (drone controller localization)
    - GPS spoofing detection
    - Search and rescue (ELT/PLB localization)
    
    Example:
        localizer = JammerLocalizer()
        
        # Define receiver positions [x, y, z] in ENU meters
        rx_positions = [
            np.array([0, 0, 100]),          # Reference node
            np.array([50000, 0, 100]),       # 50 km east
            np.array([0, 80000, 150]),       # 80 km north
            np.array([-30000, 60000, 120]),  # Southwest
        ]
        
        # Process TOA measurements (seconds, synchronized via PTP)
        toa = [0.001234, 0.001267, 0.001198, 0.001245]
        result = localizer.update(rx_positions, toa, timestamp=scan_time)
        
        if result is not None:
            print(f"Emitter at {result.position}, power {result.power_estimate_dBm} dBm")
    """
    
    C = 299792458.0  # Speed of light, m/s
    
    def __init__(self, 
                 process_noise_pos: float = 100.0,
                 process_noise_vel: float = 10.0,
                 min_nodes: int = 3,
                 max_emitters: int = 8,
                 convergence_threshold: float = 1000.0):
        """Initialize jammer localizer.
        
        Args:
            process_noise_pos: Position process noise σ (meters)
            process_noise_vel: Velocity process noise σ (m/s)
            min_nodes: Minimum receiver nodes for localization
            max_emitters: Maximum simultaneous emitters to track
            convergence_threshold: Max residual for valid solution (meters)
        """
        self.process_noise_pos = process_noise_pos
        self.process_noise_vel = process_noise_vel
        self.min_nodes = min_nodes
        self.max_emitters = max_emitters
        self.convergence_threshold = convergence_threshold
        
        self._emitters: Dict[int, EmitterTrack] = {}
        self._next_id = 1
    
    def _compute_tdoa(self, toa: List[float]) -> Tuple[np.ndarray, np.ndarray]:
        """Compute TDOA pairs relative to reference receiver (node 0).
        
        Returns:
            tdoa_values: Array of c·Δt values (meters)
            valid_mask: Boolean mask for valid pairs
        """
        n = len(toa)
        tdoa = np.zeros(n - 1)
        valid = np.ones(n - 1, dtype=bool)
        
        for i in range(1, n):
            if toa[i] is not None and toa[0] is not None:
                tdoa[i - 1] = self.C * (toa[i] - toa[0])
            else:
                valid[i - 1] = False
        
        return tdoa, valid
    
    def _chan_ho_initial(self, rx_positions: List[np.ndarray],
                         tdoa: np.ndarray, valid: np.ndarray) -> Optional[np.ndarray]:
        """Chan-Ho closed-form TDOA solution for initialization.
        
        Reference: Y.T. Chan & K.C. Ho, "A simple and efficient estimator
        for hyperbolic location," IEEE Trans. Signal Processing, 1994.
        
        Handles coplanar receivers by solving 2D (x,y) when z-spread is small.
        """
        ref = rx_positions[0]
        n_valid = int(np.sum(valid))
        if n_valid < 2:
            return None
        
        # Check z-spread to decide 2D vs 3D
        z_values = [ref[2]] + [rx_positions[i+1][2] for i in range(len(tdoa)) if valid[i]]
        z_spread = max(z_values) - min(z_values)
        solve_2d = z_spread < 1000.0  # Less than 1 km altitude spread → 2D solve
        
        ndim = 2 if solve_2d else 3
        z_est = np.mean(z_values) + 5000.0  # Emitters are typically above receivers
        
        A = np.zeros((n_valid, ndim))
        b_vec = np.zeros(n_valid)
        
        row = 0
        for i in range(len(tdoa)):
            if not valid[i]:
                continue
            rx_i = rx_positions[i + 1]
            d_i0 = tdoa[i]
            
            A[row, 0] = 2.0 * (ref[0] - rx_i[0])
            A[row, 1] = 2.0 * (ref[1] - rx_i[1])
            if not solve_2d:
                A[row, 2] = 2.0 * (ref[2] - rx_i[2])
            
            r_i_sq = float(np.sum(rx_i ** 2))
            r_0_sq = float(np.sum(ref ** 2))
            
            b_vec[row] = d_i0 ** 2 + r_0_sq - r_i_sq
            if solve_2d:
                b_vec[row] -= 2.0 * (ref[2] - rx_i[2]) * z_est
            
            row += 1
        
        try:
            result, _, rank, _ = np.linalg.lstsq(A, b_vec, rcond=None)
            if solve_2d:
                return np.array([result[0], result[1], z_est])
            return result
        except np.linalg.LinAlgError:
            return None
    
    def _solve_tdoa_wls(self, rx_positions: List[np.ndarray],
                         tdoa: np.ndarray, valid: np.ndarray,
                         initial_guess: Optional[np.ndarray] = None) -> Optional[np.ndarray]:
        """Solve TDOA equations via Chan-Ho init + Gauss-Newton refinement.
        
        Two-stage approach (matches jammer_localizer.sv LOC_SOLVE state):
        1. Chan-Ho closed-form solution for robust initialization
        2. Gauss-Newton iterative refinement for accuracy
        """
        n_valid = int(np.sum(valid))
        if n_valid < self.min_nodes - 1:
            return None
        
        ref = rx_positions[0]
        
        # Stage 1: Get initial estimate
        if initial_guess is not None:
            r_est = initial_guess.copy()
        else:
            # Chan-Ho closed-form (much better than centroid)
            chan_result = self._chan_ho_initial(rx_positions, tdoa, valid)
            if chan_result is not None:
                r_est = chan_result
            else:
                r_est = np.mean(rx_positions, axis=0)
        
        # Stage 2: Gauss-Newton refinement
        for iteration in range(15):
            H = np.zeros((n_valid, 3))
            b = np.zeros(n_valid)
            
            d_ref = np.linalg.norm(r_est - ref)
            if d_ref < 1.0:
                d_ref = 1.0
            
            row = 0
            for i in range(len(tdoa)):
                if not valid[i]:
                    continue
                
                rx_i = rx_positions[i + 1]
                d_i = np.linalg.norm(r_est - rx_i)
                if d_i < 1.0:
                    d_i = 1.0
                
                # Jacobian row: ∂(d_i - d_ref)/∂r
                e_i = (r_est - rx_i) / d_i
                e_ref = (r_est - ref) / d_ref
                H[row] = e_i - e_ref
                
                # Residual
                predicted_tdoa = d_i - d_ref
                b[row] = tdoa[i] - predicted_tdoa
                
                row += 1
            
            # Weighted least squares solve
            try:
                delta, residuals, rank, sv = np.linalg.lstsq(H, b, rcond=None)
            except np.linalg.LinAlgError:
                return None
            
            r_est += delta
            
            if np.linalg.norm(delta) < 1.0:  # Converged (1 meter)
                break
        
        # Check residual
        total_residual = np.linalg.norm(b)
        if total_residual > self.convergence_threshold:
            return None
        
        return r_est
    
    def _kalman_predict(self, emitter: EmitterTrack, dt: float):
        """Predict emitter state forward by dt seconds."""
        # State transition: constant velocity model
        F = np.eye(6)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        
        emitter.position += emitter.velocity * dt
        
        # Process noise
        q_pos = self.process_noise_pos ** 2 * dt
        q_vel = self.process_noise_vel ** 2 * dt
        Q = np.diag([q_pos, q_pos, q_pos, q_vel, q_vel, q_vel])
        
        emitter.covariance = F @ emitter.covariance @ F.T + Q
    
    def _kalman_update(self, emitter: EmitterTrack, measured_pos: np.ndarray,
                       R: Optional[np.ndarray] = None):
        """Update emitter track with new position measurement."""
        # Measurement model: H = [I_3x3 | 0_3x3] (position only)
        H = np.zeros((3, 6))
        H[:3, :3] = np.eye(3)
        
        if R is None:
            R = np.diag([500.0 ** 2, 500.0 ** 2, 1000.0 ** 2])  # Default uncertainty
        
        # Innovation
        y = measured_pos - emitter.position
        S = H @ emitter.covariance @ H.T + R
        
        # Kalman gain
        try:
            K = emitter.covariance @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return
        
        # State update
        state = np.concatenate([emitter.position, emitter.velocity])
        state += K @ y
        emitter.position = state[:3]
        emitter.velocity = state[3:]
        
        # Covariance update (Joseph form for numerical stability)
        I_KH = np.eye(6) - K @ H
        emitter.covariance = I_KH @ emitter.covariance @ I_KH.T + K @ R @ K.T
        
        emitter.n_updates += 1
        emitter.confidence = min(1.0, emitter.n_updates / 10.0)
    
    def update(self, rx_positions: List[np.ndarray],
               toa: List[float],
               timestamp: float = 0.0,
               power_measurements: Optional[List[float]] = None,
               classification_hint: str = 'UNKNOWN') -> Optional[EmitterTrack]:
        """[REQ-V610-TDOA-02] Process TOA measurements and update emitter track.
        
        Args:
            rx_positions: List of receiver positions [x,y,z] in ENU meters
            toa: Time of arrival at each receiver (seconds, PTP-synchronized)
            timestamp: Current time (seconds)
            power_measurements: Optional received power at each node (dBm)
            classification_hint: ECM type from ML-CFAR ('NOISE', 'DECEPTION', etc.)
            
        Returns:
            Updated EmitterTrack or None if localization failed
        """
        if len(rx_positions) < self.min_nodes or len(toa) < self.min_nodes:
            return None
        
        # Compute TDOA
        tdoa, valid = self._compute_tdoa(toa)
        
        if np.sum(valid) < self.min_nodes - 1:
            return None
        
        # Find closest existing emitter or create new one
        best_emitter = None
        best_distance = float('inf')
        
        # Get initial position estimate
        initial = None
        if self._emitters:
            # Use existing emitter position as initial guess
            for eid, e in self._emitters.items():
                initial = e.position.copy()
                break
        
        measured_pos = self._solve_tdoa_wls(rx_positions, tdoa, valid, initial)
        if measured_pos is None:
            return None
        
        # Associate with existing emitter
        for eid, emitter in self._emitters.items():
            dist = np.linalg.norm(measured_pos - emitter.position)
            if dist < best_distance and dist < 10000.0:  # 10 km gate
                best_distance = dist
                best_emitter = emitter
        
        if best_emitter is None:
            # Create new emitter track
            emitter = EmitterTrack(
                emitter_id=self._next_id,
                position=measured_pos.copy(),
                velocity=np.zeros(3),
                covariance=np.diag([1000.0**2, 1000.0**2, 2000.0**2,
                                    50.0**2, 50.0**2, 10.0**2]),
                power_estimate_dBm=0.0,
                confidence=0.1,
                classification=classification_hint,
                n_updates=1,
                last_update_time=timestamp,
            )
            self._emitters[self._next_id] = emitter
            self._next_id += 1
            best_emitter = emitter
        else:
            # Predict and update existing track
            dt = timestamp - best_emitter.last_update_time if timestamp > best_emitter.last_update_time else 1.0
            self._kalman_predict(best_emitter, dt)
            self._kalman_update(best_emitter, measured_pos)
            best_emitter.last_update_time = timestamp
            best_emitter.classification = classification_hint
        
        # Power estimation (inverse square law from closest receiver)
        if power_measurements is not None and len(power_measurements) > 0:
            best_rx_idx = 0
            best_rx_dist = float('inf')
            for i, rxp in enumerate(rx_positions):
                d = np.linalg.norm(measured_pos - rxp)
                if d < best_rx_dist and i < len(power_measurements):
                    best_rx_dist = d
                    best_rx_idx = i
            
            if best_rx_dist > 100.0:
                # ERP = P_rx + 20·log10(d) + 20·log10(f) - 147.55
                # Simplified: ERP ≈ P_rx + 20·log10(d_km) + 32.4 + 20·log10(f_MHz)
                path_loss_dB = 20.0 * np.log10(best_rx_dist / 1000.0) + 90.0  # Rough @ VHF
                best_emitter.power_estimate_dBm = power_measurements[best_rx_idx] + path_loss_dB
        
        return best_emitter
    
    def get_all_emitters(self) -> List[EmitterTrack]:
        """Get all active emitter tracks."""
        return list(self._emitters.values())
    
    def clear(self):
        """Clear all emitter tracks."""
        self._emitters.clear()


# =============================================================================
# [REQ-V610-ADAPT] Adaptive Integration Controller
# Ported from: QEDMMA v2/rtl/eccm/integration_controller.sv (309 lines SV)
# =============================================================================

class AdaptiveIntegrator:
    """[REQ-V610-ADAPT-01] Dynamic SNR enhancement based on jamming level.
    
    Ported from QEDMMA integration_controller.sv.
    
    In FPGA: controls number of coherent pulses integrated.
    In NX-MIMOSA software: controls equivalent parameters:
    - Measurement noise inflation (R matrix scaling)
    - Track confirmation threshold adjustment
    - Coast timeout extension
    - Innovation gate widening
    
    When ML-CFAR detects jamming, this controller adapts tracker parameters
    to maintain track continuity at the cost of reduced accuracy.
    
    Integration levels (from QEDMMA):
        NORMAL:     10 pulses, +0 dB, baseline parameters
        ELEVATED:   20 pulses, +3 dB, inflated R, wider gate
        HIGH:       35 pulses, +5.4 dB, bearing-only mode available
        MAXIMUM:    50 pulses, +7 dB, maximum coast, emergency mode
    
    Example:
        integrator = AdaptiveIntegrator()
        ml_cfar = MLCFARDetector()
        
        env_class, conf = ml_cfar.classify(features)
        params = integrator.adapt(env_class, conf)
        
        tracker.r_std *= params['r_multiplier']
        tracker.gate_size = params['gate_sigma']
    """
    
    @dataclass
    class AdaptedParams:
        """Adapted tracker parameters based on jamming assessment."""
        r_multiplier: float = 1.0       # Measurement noise inflation
        gate_sigma: float = 4.0         # Innovation gate (sigma)
        max_coast_scans: int = 10       # Coast timeout
        confirm_threshold: int = 3      # M-of-N confirmation M
        confirm_window: int = 5         # M-of-N confirmation N
        integration_gain_dB: float = 0.0
        mode: str = 'NORMAL'
        bearing_only: bool = False      # Switch to bearing-only tracking
    
    def __init__(self):
        """Initialize adaptive integrator."""
        self._current_level = 'NORMAL'
        self._level_history: List[str] = []
    
    def adapt(self, env_class: EnvironmentClass, 
              confidence: float = 0.5) -> 'AdaptiveIntegrator.AdaptedParams':
        """[REQ-V610-ADAPT-02] Compute adapted parameters for current environment.
        
        Logic matches integration_controller.sv state machine (lines 120-200).
        
        Args:
            env_class: Environment classification from ML-CFAR
            confidence: Classification confidence (0-1)
            
        Returns:
            AdaptedParams with all tracker parameter adjustments
        """
        params = self.AdaptedParams()
        
        if env_class == EnvironmentClass.CLEAR:
            params.mode = 'NORMAL'
            params.r_multiplier = 1.0
            params.gate_sigma = 4.0
            params.max_coast_scans = 10
            params.confirm_threshold = 3
            params.confirm_window = 5
            params.integration_gain_dB = 0.0
        
        elif env_class == EnvironmentClass.CLUTTER:
            params.mode = 'ELEVATED'
            params.r_multiplier = 1.5 + confidence
            params.gate_sigma = 3.0  # Tighter gate to reject clutter
            params.max_coast_scans = 8
            params.confirm_threshold = 4  # Harder to confirm (anti-false-alarm)
            params.confirm_window = 6
            params.integration_gain_dB = 3.0
        
        elif env_class == EnvironmentClass.NOISE_JAMMING:
            if confidence > 0.7:
                params.mode = 'MAXIMUM'
                params.r_multiplier = 5.0 + 5.0 * confidence
                params.gate_sigma = 8.0  # Wide gate (measurements very noisy)
                params.max_coast_scans = 20  # Long coast (might lose measurements)
                params.confirm_threshold = 2  # Easy confirm (don't lose weak tracks)
                params.confirm_window = 5
                params.integration_gain_dB = 7.0
                params.bearing_only = True
            else:
                params.mode = 'HIGH'
                params.r_multiplier = 3.0 + 2.0 * confidence
                params.gate_sigma = 6.0
                params.max_coast_scans = 15
                params.confirm_threshold = 3
                params.confirm_window = 5
                params.integration_gain_dB = 5.4
        
        elif env_class == EnvironmentClass.DECEPTION:
            params.mode = 'ELEVATED'
            params.r_multiplier = 2.0 + 8.0 * confidence  # High inflation for deception
            params.gate_sigma = 5.0
            params.max_coast_scans = 12
            params.confirm_threshold = 4  # Harder to confirm (reject ghost tracks)
            params.confirm_window = 6
            params.integration_gain_dB = 3.0
        
        self._current_level = params.mode
        self._level_history.append(params.mode)
        
        return params
    
    @property
    def current_level(self) -> str:
        return self._current_level


# =============================================================================
# [REQ-V610-ECCM] Integrated ECCM Pipeline
# Combines ML-CFAR + Jammer Localization + Adaptive Integration
# =============================================================================

class ECCMPipeline:
    """[REQ-V610-ECCM-01] Integrated ECCM pipeline for NX-MIMOSA.
    
    Combines three QEDMMA modules into a single update() call:
    1. ML-CFAR feature extraction and environment classification
    2. Jammer localization (when multi-receiver data available)
    3. Adaptive parameter generation for tracker
    
    This is the main integration point between QEDMMA anti-jamming IP
    and NX-MIMOSA multi-target tracking.
    
    Example:
        eccm = ECCMPipeline()
        tracker = MultiTargetTracker(dt=5.0, r_std=150.0)
        
        for scan in radar_scans:
            tracks = tracker.process_scan(measurements)
            
            for track in tracks:
                # ECCM assessment
                result = eccm.update(
                    track_id=track.track_id,
                    nis=track.nis,
                    innovation=track.innovation,
                    velocity=track.velocity,
                    was_hit=track.was_associated,
                )
                
                # Apply adaptive parameters
                if result['env_class'] != EnvironmentClass.CLEAR:
                    track.R *= result['params'].r_multiplier
                    print(f"Track {track.track_id}: {result['env_class'].name}, "
                          f"mode={result['params'].mode}")
    """
    
    def __init__(self, 
                 ml_window: int = 20,
                 enable_localizer: bool = False,
                 min_nodes: int = 3):
        """Initialize ECCM pipeline.
        
        Args:
            ml_window: ML-CFAR feature extraction window
            enable_localizer: Enable TDOA jammer localization
            min_nodes: Minimum receivers for localization
        """
        self.ml_cfar = MLCFARDetector(window=ml_window)
        self.integrator = AdaptiveIntegrator()
        self.localizer = JammerLocalizer(min_nodes=min_nodes) if enable_localizer else None
        
        # Per-track environment state
        self._track_env: Dict[int, EnvironmentClass] = {}
    
    def update(self, track_id: int, nis: float,
               innovation: Optional[np.ndarray] = None,
               velocity: Optional[np.ndarray] = None,
               was_hit: bool = True,
               rx_positions: Optional[List[np.ndarray]] = None,
               toa: Optional[List[float]] = None,
               timestamp: float = 0.0) -> Dict[str, Any]:
        """[REQ-V610-ECCM-02] Run full ECCM assessment for one track.
        
        Returns dict with:
            'env_class': EnvironmentClass
            'confidence': float (0-1)
            'features': MLCFARFeatures
            'params': AdaptiveIntegrator.AdaptedParams
            'emitter': EmitterTrack or None (if localizer enabled)
        """
        # Step 1: ML-CFAR feature extraction
        features = self.ml_cfar.extract_features(
            track_id, nis, innovation, velocity, was_hit)
        
        # Step 2: Classify environment
        env_class, confidence = self.ml_cfar.classify(features)
        self._track_env[track_id] = env_class
        
        # Step 3: Adaptive parameter generation
        params = self.integrator.adapt(env_class, confidence)
        
        # Step 4: Jammer localization (optional)
        emitter = None
        if (self.localizer is not None and 
            env_class in (EnvironmentClass.NOISE_JAMMING, EnvironmentClass.DECEPTION) and
            rx_positions is not None and toa is not None):
            emitter = self.localizer.update(
                rx_positions, toa, timestamp,
                classification_hint=env_class.name)
        
        return {
            'env_class': env_class,
            'confidence': confidence,
            'features': features,
            'params': params,
            'emitter': emitter,
        }
    
    def get_environment_summary(self) -> Dict[str, int]:
        """Count tracks by environment classification."""
        summary = {e.name: 0 for e in EnvironmentClass}
        for env in self._track_env.values():
            summary[env.name] += 1
        return summary
    
    def clear_track(self, track_id: int):
        """Remove track from ECCM monitoring."""
        self.ml_cfar.clear_track(track_id)
        self._track_env.pop(track_id, None)
