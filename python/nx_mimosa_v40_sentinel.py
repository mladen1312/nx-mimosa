"""
NX-MIMOSA v4.0 "SENTINEL" — Platform-Aware Variable-Structure IMM Tracker
===========================================================================
[REQ-V40-01] 6-model IMM bank: CV, CT+, CT-, CA, Jerk, Ballistic
[REQ-V40-02] Platform identification from kinematics (speed/g/altitude)
[REQ-V40-03] VS-IMM: dynamic model activation from platform DB constraints
[REQ-V40-04] Adaptive Q scaling from identified platform intent phase
[REQ-V40-05] TPM biasing from platform doctrine
[REQ-V40-06] Model pruning (prob < threshold → deactivate)
[REQ-V40-07] Dual-mode smoothing (forward + window RTS + full RTS)
[REQ-V40-08] 4 output streams: realtime, refined, offline, intent
[REQ-V40-09] Graceful fallback to Unknown (all 6 models) if ID fails

Benchmark target: 8/8 wins vs Stone Soup, +40% RMSE improvement in hard scenarios

Author: Dr. Mladen Mešter / Nexellum d.o.o.
License: AGPL v3 (open-source) | Commercial license available
Contact: mladen@nexellum.com | +385 99 737 5100
"""

import numpy as np
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
from enum import Enum
import json
import os
import copy


# =============================================================================
# Constants
# =============================================================================
GRAVITY = 9.80665  # m/s^2


# =============================================================================
# Platform Intent State
# =============================================================================
class IntentPhase(Enum):
    UNKNOWN   = "unknown"
    CRUISE    = "cruise"
    PATROL    = "patrol"
    ENGAGEMENT = "engagement"
    TERMINAL  = "terminal"
    BOOST     = "boost"
    MIDCOURSE = "midcourse"
    GLIDE     = "glide"
    EGRESS    = "egress"
    HOVER     = "hover"
    ATTACK    = "attack"
    ORBIT     = "orbit"


@dataclass
class IntentState:
    """Output stream 4: threat/intent assessment."""
    platform_type: str = "Unknown"
    platform_category: str = "unknown"
    confidence: float = 0.0
    phase: str = "unknown"
    phase_confidence: float = 0.0
    estimated_max_g: float = 30.0
    estimated_max_speed: float = 3500.0
    threat_level: float = 0.5  # 0 = benign, 1 = maximum threat
    active_models: List[str] = field(default_factory=list)
    n_active_models: int = 3


# =============================================================================
# Kalman Filter for each motion model
# =============================================================================
class KalmanModel:
    """Single Kalman filter for one motion model in the IMM bank."""

    def __init__(self, model_type: str, state_dim: int, meas_dim: int, dt: float):
        self.model_type = model_type
        self.state_dim = state_dim
        self.meas_dim = meas_dim
        self.dt = dt

        self.x = np.zeros(state_dim)
        self.P = np.eye(state_dim) * 100.0
        self.F = np.eye(state_dim)
        self.H = np.zeros((meas_dim, state_dim))
        self.Q = np.eye(state_dim)
        self.R = np.eye(meas_dim)

        self.omega = 0.0  # For CT models
        self.q_base = 1.0  # Base process noise scalar
        self.q_scale = 1.0  # Adaptive scale from platform

        self._build_model()

    def _build_model(self):
        """Build F, H, Q matrices based on model type."""
        dt = self.dt

        if self.model_type == "CV":
            # Constant Velocity: state = [x, y, vx, vy]
            self.F = np.array([
                [1, 0, dt, 0],
                [0, 1, 0, dt],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])
            self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
            q = self.q_base * 0.5
            self.Q = q * np.array([
                [dt**4/4, 0, dt**3/2, 0],
                [0, dt**4/4, 0, dt**3/2],
                [dt**3/2, 0, dt**2, 0],
                [0, dt**3/2, 0, dt**2]
            ])

        elif self.model_type in ("CT_plus", "CT_minus"):
            # Coordinated Turn
            self.omega = 0.196 if self.model_type == "CT_plus" else -0.196
            self._update_ct_F()
            self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
            q = self.q_base * 1.0
            self.Q = q * np.array([
                [dt**4/4, 0, dt**3/2, 0],
                [0, dt**4/4, 0, dt**3/2],
                [dt**3/2, 0, dt**2, 0],
                [0, dt**3/2, 0, dt**2]
            ])

        elif self.model_type == "CA":
            # Constant Acceleration: state = [x, y, vx, vy, ax, ay]
            self.F = np.array([
                [1, 0, dt, 0, dt**2/2, 0],
                [0, 1, 0, dt, 0, dt**2/2],
                [0, 0, 1, 0, dt, 0],
                [0, 0, 0, 1, 0, dt],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1]
            ])
            self.H = np.array([
                [1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0]
            ])
            q = self.q_base * 2.0
            self.Q = q * np.block([
                [dt**4/4 * np.eye(2), dt**3/2 * np.eye(2), dt**2/2 * np.eye(2)],
                [dt**3/2 * np.eye(2), dt**2 * np.eye(2), dt * np.eye(2)],
                [dt**2/2 * np.eye(2), dt * np.eye(2), np.eye(2)]
            ])

        elif self.model_type == "Jerk":
            # Jerk model: state = [x, y, vx, vy, ax, ay, jx, jy]
            self.F = np.zeros((8, 8))
            for i in [0, 1]:  # x and y blocks
                base = i * 4
                self.F[base, base] = 1
                self.F[base, base+1] = dt
                self.F[base, base+2] = dt**2/2
                self.F[base, base+3] = dt**3/6
                self.F[base+1, base+1] = 1
                self.F[base+1, base+2] = dt
                self.F[base+1, base+3] = dt**2/2
                self.F[base+2, base+2] = 1
                self.F[base+2, base+3] = dt
                self.F[base+3, base+3] = 1
            # Fix: state is [x, vx, ax, jx, y, vy, ay, jy]
            # Actually let's use [x, y, vx, vy, ax, ay, jx, jy]
            self.F = np.zeros((8, 8))
            self.F[0, 0] = 1; self.F[0, 2] = dt; self.F[0, 4] = dt**2/2; self.F[0, 6] = dt**3/6
            self.F[1, 1] = 1; self.F[1, 3] = dt; self.F[1, 5] = dt**2/2; self.F[1, 7] = dt**3/6
            self.F[2, 2] = 1; self.F[2, 4] = dt; self.F[2, 6] = dt**2/2
            self.F[3, 3] = 1; self.F[3, 5] = dt; self.F[3, 7] = dt**2/2
            self.F[4, 4] = 1; self.F[4, 6] = dt
            self.F[5, 5] = 1; self.F[5, 7] = dt
            self.F[6, 6] = 1
            self.F[7, 7] = 1

            self.H = np.zeros((2, 8))
            self.H[0, 0] = 1; self.H[1, 1] = 1

            q = self.q_base * 5.0
            self.Q = q * np.eye(8) * dt**2

        elif self.model_type == "Ballistic":
            # Ballistic: state = [x, y, vx, vy] but with gravity Q model
            self.F = np.array([
                [1, 0, dt, 0],
                [0, 1, 0, dt],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])
            self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
            # Gravity-dominated Q: high in vertical, low in horizontal
            q = self.q_base * 0.3
            gq = GRAVITY * dt
            self.Q = np.array([
                [dt**4/4*q, 0, dt**3/2*q, 0],
                [0, dt**4/4*(q + gq**2), 0, dt**3/2*(q + gq**2)],
                [dt**3/2*q, 0, dt**2*q, 0],
                [0, dt**3/2*(q + gq**2), 0, dt**2*(q + gq**2)]
            ])

        # Initialize P
        self.P = np.eye(self.state_dim) * 100.0
        self.x = np.zeros(self.state_dim)

    def _update_ct_F(self):
        """Update CT transition matrix with current omega."""
        dt = self.dt
        w = self.omega
        if abs(w) < 1e-6:
            self.F = np.array([
                [1, 0, dt, 0],
                [0, 1, 0, dt],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])
        else:
            sw = np.sin(w * dt)
            cw = np.cos(w * dt)
            self.F = np.array([
                [1, 0, sw/w, -(1-cw)/w],
                [0, 1, (1-cw)/w, sw/w],
                [0, 0, cw, -sw],
                [0, 0, sw, cw]
            ])

    def set_omega(self, omega: float):
        """Update turn rate for CT models."""
        self.omega = omega
        if self.model_type in ("CT_plus", "CT_minus"):
            self._update_ct_F()

    def predict(self):
        """KF predict step. Returns x_pred, P_pred."""
        Q_scaled = self.Q * self.q_scale
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + Q_scaled

        # Ballistic: add gravity bias to vy
        if self.model_type == "Ballistic":
            self.x[3] -= GRAVITY * self.dt  # vy -= g*dt

        return self.x.copy(), self.P.copy()

    def update(self, z: np.ndarray, R: np.ndarray):
        """KF update step. Returns x_filt, P_filt, innovation, S."""
        # Innovation
        nu = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + R
        # Kalman gain
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            S_inv = np.linalg.pinv(S)
        K = self.P @ self.H.T @ S_inv
        # Update
        self.x = self.x + K @ nu
        I_KH = np.eye(self.state_dim) - K @ self.H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T  # Joseph form

        return self.x.copy(), self.P.copy(), nu, S

    def likelihood(self, z: np.ndarray, R: np.ndarray) -> float:
        """Compute measurement likelihood for this model."""
        nu = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + R
        try:
            S_inv = np.linalg.inv(S)
            sign, logdet = np.linalg.slogdet(S)
            if sign <= 0:
                return 1e-30
            exponent = -0.5 * nu @ S_inv @ nu
            log_lik = -0.5 * len(z) * np.log(2 * np.pi) - 0.5 * logdet + exponent
            return max(np.exp(log_lik), 1e-30)
        except np.linalg.LinAlgError:
            return 1e-30

    def get_position_velocity(self) -> Tuple[np.ndarray, np.ndarray]:
        """Extract [x,y] and [vx,vy] from state regardless of model dimension."""
        return self.x[:2].copy(), self.x[2:4].copy()


# =============================================================================
# Platform Identifier
# =============================================================================
class PlatformIdentifier:
    """
    Identifies target platform from observed kinematics + IMM model probabilities.
    [REQ-V40-02]
    
    Uses:
      - Speed (from IMM combined state)
      - IMM model probabilities (CT prob = maneuvering indicator)
      - Innovation-based NIS for g-load estimation
      - Altitude
    """

    def __init__(self, platform_db: dict, window_size: int = 10):
        self.db = platform_db
        self.window_size = window_size
        self.speed_history = deque(maxlen=window_size)
        self.accel_history = deque(maxlen=window_size)
        self.altitude_history = deque(maxlen=window_size)
        self.ct_prob_history = deque(maxlen=window_size)
        self.ca_prob_history = deque(maxlen=window_size)
        self.prev_velocity = None
        self.best_match = "Unknown"
        self.best_confidence = 0.0
        self.match_scores: Dict[str, float] = {}

    def update(self, position: np.ndarray, velocity: np.ndarray, dt: float,
               model_probs: Optional[Dict[str, float]] = None,
               innovation_norm: float = 0.0):
        """Update identifier with new kinematic observation + IMM info."""
        speed = np.linalg.norm(velocity)
        self.speed_history.append(speed)

        # Use innovation-based acceleration estimate (more reliable than vel diff)
        if innovation_norm > 0:
            # NIS-based g: larger innovations = higher maneuverability
            accel_g = innovation_norm / (9.81 * dt * dt + 1e-10)
            accel_g = min(accel_g, 50.0)  # Clamp
        elif self.prev_velocity is not None:
            accel = np.linalg.norm(velocity - self.prev_velocity) / max(dt, 1e-6)
            accel_g = accel / GRAVITY
        else:
            accel_g = 0.0

        self.accel_history.append(accel_g)
        self.prev_velocity = velocity.copy()

        # Store IMM model probabilities
        if model_probs:
            ct_prob = model_probs.get('CT_plus', 0) + model_probs.get('CT_minus', 0)
            ca_prob = model_probs.get('CA', 0) + model_probs.get('Jerk', 0)
            self.ct_prob_history.append(ct_prob)
            self.ca_prob_history.append(ca_prob)

        alt = abs(position[1]) if len(position) >= 2 else 0
        self.altitude_history.append(alt)

        if len(self.speed_history) < 3:
            return "Unknown", 0.0

        return self._classify()

    def _classify(self) -> Tuple[str, float]:
        """Rule-based platform classification using speed + IMM model probs."""
        avg_speed = np.mean(self.speed_history)
        max_speed = np.max(self.speed_history)
        avg_g = np.mean(self.accel_history) if self.accel_history else 0
        max_g = np.max(self.accel_history) if self.accel_history else 0
        avg_alt = np.mean(self.altitude_history) if self.altitude_history else 0

        # IMM-based maneuver indicators
        avg_ct = np.mean(self.ct_prob_history) if self.ct_prob_history else 0
        max_ct = np.max(self.ct_prob_history) if self.ct_prob_history else 0
        avg_ca = np.mean(self.ca_prob_history) if self.ca_prob_history else 0

        # Composite maneuver score: 0 = straight/level, 1 = highly maneuvering
        maneuver_score = min(1.0, avg_ct * 1.5 + avg_ca * 2.0 + min(avg_g / 5.0, 1.0) * 0.5)

        self.match_scores = {}

        for name, plat in self.db.items():
            if name == "_meta" or name == "Unknown":
                continue

            score = 0.0
            max_score = 0.0

            # --- Speed compatibility (0-4 points) ---
            max_score += 4.0
            plat_max_speed = plat["max_speed_mps"]
            plat_cruise = plat.get("cruise_speed_mps", plat_max_speed * 0.6)
            plat_min = plat.get("min_speed_mps", 0)

            if max_speed <= plat_max_speed * 1.2 and avg_speed >= plat_min * 0.5:
                # Speed in platform envelope
                speed_ratio = avg_speed / max(plat_cruise, 1)
                if 0.4 <= speed_ratio <= 2.0:
                    score += 4.0 * max(0, 1 - abs(speed_ratio - 1.0) * 0.4)
                elif speed_ratio < 0.4:
                    score += 1.0
                else:
                    score += 0.5
            elif max_speed > plat_max_speed * 1.5:
                score += 0.0  # Way over limit
            else:
                score += 0.5

            # --- Maneuverability compatibility (0-4 points) ---
            max_score += 4.0
            cat = plat.get("category", "unknown")
            plat_max_g = plat.get("max_g", 30.0)
            plat_sustained_g = plat.get("sustained_g", plat_max_g * 0.5)

            if cat in ("fighter",):
                # Fighters: high maneuver score is GOOD match
                if maneuver_score > 0.3:
                    score += 4.0 * min(maneuver_score / 0.7, 1.0)
                elif avg_speed > 150:
                    score += 2.0  # Fast but not maneuvering = possible cruise phase
                else:
                    score += 0.5
            elif cat in ("bomber", "transport", "aew"):
                # Non-maneuvering platforms: low maneuver score = good
                if maneuver_score < 0.2:
                    score += 4.0
                elif maneuver_score < 0.4:
                    score += 2.0
                else:
                    score += 0.5  # Too maneuvering for bomber
            elif cat in ("cruise_missile",):
                # Cruise missiles: low-moderate maneuver, moderate speed
                if maneuver_score < 0.35:
                    score += 3.5
                else:
                    score += 1.0
            elif cat in ("ballistic_missile", "hypersonic_missile"):
                # Ballistic/hypersonic: high speed, low-moderate maneuver until terminal
                if avg_speed > 800:
                    score += 3.0
                    if maneuver_score < 0.4:
                        score += 1.0  # Midcourse
                else:
                    score += 0.5
            elif cat == "sam":
                # SAMs: high speed + high maneuver
                if avg_speed > 500 and maneuver_score > 0.2:
                    score += 4.0
                elif avg_speed > 500:
                    score += 2.5
                else:
                    score += 0.5
            elif cat in ("small_drone", "fpv_kamikaze"):
                # Drones: low speed
                if avg_speed < 60:
                    if maneuver_score > 0.3:
                        score += 3.5 if cat == "fpv_kamikaze" else 2.5
                    else:
                        score += 3.0 if cat == "small_drone" else 2.0
                else:
                    score += 0.3
            elif cat == "loitering_munition":
                if 20 < avg_speed < 80 and maneuver_score < 0.3:
                    score += 4.0
                elif avg_speed < 80:
                    score += 2.0
                else:
                    score += 0.3
            elif cat == "mlrs":
                if avg_speed > 300 and maneuver_score < 0.2:
                    score += 3.5
                else:
                    score += 0.5
            else:
                score += 1.0

            # --- Altitude compatibility (0-2 points) ---
            max_score += 2.0
            plat_max_alt = plat.get("max_altitude_m", 50000)
            if avg_alt <= plat_max_alt * 1.1:
                score += 2.0
            elif avg_alt <= plat_max_alt * 2.0:
                score += 1.0

            self.match_scores[name] = score / max_score if max_score > 0 else 0

        if not self.match_scores:
            self.best_match = "Unknown"
            self.best_confidence = 0.0
            return self.best_match, self.best_confidence

        # Find best match, preferring category-level grouping
        best_name = max(self.match_scores, key=self.match_scores.get)
        best_score = self.match_scores[best_name]

        if best_score >= 0.45:
            self.best_match = best_name
            self.best_confidence = best_score
        else:
            self.best_match = "Unknown"
            self.best_confidence = best_score

        return self.best_match, self.best_confidence


# =============================================================================
# Intent Predictor
# =============================================================================
class IntentPredictor:
    """
    Predict target intent phase from kinematics + platform DB.
    [REQ-V40-04]
    """

    def __init__(self):
        self.current_phase = "unknown"
        self.phase_confidence = 0.0

    def predict(self, platform_type: str, platform_data: dict,
                speed: float, accel_g: float, model_probs: Dict[str, float]) -> Tuple[str, float]:
        """Predict current intent phase."""
        phases = platform_data.get("intent_phases", {})
        if not phases:
            return "unknown", 0.0

        best_phase = "unknown"
        best_score = 0.0

        max_speed = platform_data.get("max_speed_mps", 3500)
        cruise_speed = platform_data.get("cruise_speed_mps", max_speed * 0.6)
        sustained_g = platform_data.get("sustained_g", 5.0)

        for phase_name, phase_data in phases.items():
            score = phase_data.get("prob", 0.1)  # Prior probability

            phase_models = phase_data.get("models", [])
            # Boost score if active model probs align with phase models
            model_alignment = sum(model_probs.get(m, 0) for m in phase_models)
            score *= (1.0 + model_alignment * 2.0)

            # Kinematic indicators
            speed_ratio = speed / max(cruise_speed, 1)

            if phase_name in ("cruise", "patrol", "transit", "orbit"):
                if 0.7 <= speed_ratio <= 1.3 and accel_g < 1.0:
                    score *= 2.0
            elif phase_name in ("engagement", "attack", "post_stall"):
                if accel_g > sustained_g * 0.5:
                    score *= 2.5
            elif phase_name in ("terminal", "terminal_dive"):
                if accel_g > sustained_g * 0.3 and speed_ratio > 0.8:
                    score *= 2.0
            elif phase_name in ("boost"):
                if accel_g > 2.0:
                    score *= 2.0
            elif phase_name in ("hover", "loiter"):
                if speed < 5.0 and accel_g < 0.5:
                    score *= 3.0
            elif phase_name in ("glide", "midcourse", "ballistic"):
                if accel_g < 1.0:
                    score *= 1.5
            elif phase_name == "egress":
                if speed_ratio > 1.0 and accel_g < 1.5:
                    score *= 1.5

            if score > best_score:
                best_score = score
                best_phase = phase_name

        # Normalize confidence to 0-1
        self.phase_confidence = min(best_score / 5.0, 1.0)
        self.current_phase = best_phase
        return best_phase, self.phase_confidence


# =============================================================================
# NX-MIMOSA v4.0 SENTINEL Tracker
# =============================================================================
class NxMimosaV40Sentinel:
    """
    Platform-Aware Variable-Structure IMM Tracker.

    4 output streams:
      1. x_realtime   — forward IMM estimate, 0 latency
      2. x_refined    — window-N RTS smoothed, ~Ndt latency
      3. x_offline    — full-track RTS smoothed, full-track latency
      4. intent_state — platform ID + intent phase + threat level

    6 motion models: CV, CT+, CT-, CA, Jerk, Ballistic
    """

    ALL_MODELS = ["CV", "CT_plus", "CT_minus", "CA", "Jerk", "Ballistic"]
    MODEL_STATE_DIMS = {
        "CV": 4, "CT_plus": 4, "CT_minus": 4,
        "CA": 6, "Jerk": 8, "Ballistic": 4
    }

    def __init__(self, dt: float = 0.1, r_std: float = 2.5,
                 platform_db_path: Optional[str] = None,
                 window_size: int = 30,
                 prune_threshold: float = 0.03,
                 initial_models: Optional[List[str]] = None):
        """
        Args:
            dt: Time step [s]
            r_std: Measurement noise std [m]
            platform_db_path: Path to platform_db.json (None = use bundled)
            window_size: Window smoother depth
            prune_threshold: Model probability below which to deactivate
            initial_models: Starting models (default: CV, CT+, CT-)
        """
        self.dt = dt
        self.r_std = r_std
        self.R = np.eye(2) * r_std**2
        self.window_size = window_size
        self.prune_threshold = prune_threshold

        # Load platform database
        self.platform_db = self._load_platform_db(platform_db_path)

        # Initialize all 6 model filters
        self.filters: Dict[str, KalmanModel] = {}
        for model_name in self.ALL_MODELS:
            sd = self.MODEL_STATE_DIMS[model_name]
            self.filters[model_name] = KalmanModel(model_name, sd, 2, dt)

        # Active models (VS-IMM)
        if initial_models:
            self.active_models = list(initial_models)
        else:
            self.active_models = ["CV", "CT_plus", "CT_minus"]

        # Model probabilities
        n_active = len(self.active_models)
        self.mu = {m: 1.0/n_active for m in self.active_models}

        # TPM (transition probability matrix) — dynamic
        self.p_stay = 0.88
        self.tpm: Dict[str, Dict[str, float]] = {}
        self._rebuild_tpm()

        # Platform identification
        self.identifier = PlatformIdentifier(self.platform_db)
        self.intent_predictor = IntentPredictor()
        self.intent_state = IntentState()

        # Smoother buffers
        self.forward_history = deque(maxlen=max(window_size + 5, 2100))
        self.step_count = 0
        self.initialized = False

        # Adaptive omega for CT models
        self.omega_est = 0.196  # Current estimated turn rate

    def _load_platform_db(self, path: Optional[str]) -> dict:
        """Load platform database from JSON."""
        if path and os.path.exists(path):
            with open(path, 'r') as f:
                return json.load(f)

        # Try relative paths
        candidates = [
            'data/platform_db.json',
            os.path.join(os.path.dirname(__file__), '..', 'data', 'platform_db.json'),
            os.path.join(os.path.dirname(__file__), 'data', 'platform_db.json'),
        ]
        for p in candidates:
            if os.path.exists(p):
                with open(p, 'r') as f:
                    return json.load(f)

        # Fallback: minimal Unknown-only DB
        return {
            "Unknown": {
                "category": "unknown",
                "max_speed_mps": 3500, "cruise_speed_mps": 500,
                "max_g": 30.0, "sustained_g": 15.0,
                "max_turn_rate_radps": 1.5, "max_altitude_m": 100000,
                "preferred_models": self.ALL_MODELS,
                "initial_tpm_bias": {m: 1/6 for m in self.ALL_MODELS},
                "intent_phases": {"unknown": {"prob": 1.0, "models": self.ALL_MODELS, "q_scale": 1.0}}
            }
        }

    def _rebuild_tpm(self):
        """Rebuild transition probability matrix for active models."""
        n = len(self.active_models)
        if n <= 1:
            self.tpm = {m: {m: 1.0} for m in self.active_models}
            return

        p_switch = (1.0 - self.p_stay) / (n - 1)
        self.tpm = {}
        for mi in self.active_models:
            self.tpm[mi] = {}
            for mj in self.active_models:
                self.tpm[mi][mj] = self.p_stay if mi == mj else p_switch

    def _apply_platform_tpm_bias(self, platform_data: dict):
        """Bias TPM based on platform's preferred models. [REQ-V40-05]"""
        bias = platform_data.get("initial_tpm_bias", {})
        if not bias:
            return

        n = len(self.active_models)
        if n <= 1:
            return

        for mi in self.active_models:
            total = 0
            for mj in self.active_models:
                base = self.tpm[mi].get(mj, 0)
                b = bias.get(mj, 1.0 / n)
                # Blend: 70% standard TPM + 30% platform bias
                self.tpm[mi][mj] = 0.7 * base + 0.3 * b
                total += self.tpm[mi][mj]
            # Normalize row
            if total > 0:
                for mj in self.active_models:
                    self.tpm[mi][mj] /= total

    def _apply_q_scaling(self, platform_data: dict, phase: str):
        """Apply platform + phase-specific Q scaling. [REQ-V40-04]"""
        phases = platform_data.get("intent_phases", {})
        q_scale = 1.0
        if phase in phases:
            q_scale = phases[phase].get("q_scale", 1.0)

        for model_name in self.active_models:
            self.filters[model_name].q_scale = q_scale

        # Also apply max_g constraint: clip Q for physical realism
        max_g = platform_data.get("max_g", 30.0) * GRAVITY
        for model_name in self.active_models:
            f = self.filters[model_name]
            # Scale Q so process noise doesn't exceed physical acceleration limit
            max_q_accel = max_g**2 * self.dt**2
            q_trace = np.trace(f.Q * f.q_scale)
            if q_trace > max_q_accel * f.state_dim:
                f.q_scale *= (max_q_accel * f.state_dim) / q_trace

    def _activate_models_from_platform(self, platform_data: dict):
        """VS-IMM: activate/deactivate models based on platform. [REQ-V40-03]"""
        preferred = platform_data.get("preferred_models", self.ALL_MODELS[:3])
        # Always include CV as baseline
        new_active = list(set(["CV"] + preferred))
        # Filter to only known models
        new_active = [m for m in new_active if m in self.ALL_MODELS]

        if set(new_active) != set(self.active_models):
            # Transfer probabilities to new model set
            old_mu = dict(self.mu)
            self.active_models = new_active
            n = len(self.active_models)

            # Redistribute probability
            total_old = sum(old_mu.get(m, 0) for m in self.active_models)
            if total_old > 0.01:
                self.mu = {m: old_mu.get(m, 0) / total_old for m in self.active_models}
            else:
                self.mu = {m: 1.0/n for m in self.active_models}

            # Initialize new models from best current state
            best_model = max(old_mu, key=old_mu.get) if old_mu else "CV"
            if best_model in self.filters:
                best_pos, best_vel = self.filters[best_model].get_position_velocity()
                for m in self.active_models:
                    if m not in old_mu or old_mu.get(m, 0) < 0.01:
                        f = self.filters[m]
                        f.x[:2] = best_pos
                        f.x[2:4] = best_vel
                        f.P = np.eye(f.state_dim) * 50.0

            self._rebuild_tpm()

    def _prune_models(self):
        """Remove models with negligible probability. [REQ-V40-06]"""
        if len(self.active_models) <= 2:
            return  # Keep at least 2 models

        to_remove = [m for m in self.active_models
                     if self.mu.get(m, 0) < self.prune_threshold and m != "CV"]

        for m in to_remove:
            if len(self.active_models) <= 2:
                break
            self.active_models.remove(m)
            del self.mu[m]

        if to_remove:
            # Renormalize
            total = sum(self.mu.values())
            if total > 0:
                self.mu = {m: p/total for m, p in self.mu.items()}
            self._rebuild_tpm()

    def _adaptive_omega(self, x_combined: np.ndarray):
        """Estimate turn rate from combined state and apply to CT models."""
        vx = x_combined[2] if len(x_combined) > 2 else 0
        vy = x_combined[3] if len(x_combined) > 3 else 0
        speed = np.sqrt(vx**2 + vy**2)

        if speed > 10.0 and len(self.forward_history) >= 2:
            prev = self.forward_history[-1]
            prev_vx = prev['x_combined'][2] if len(prev['x_combined']) > 2 else 0
            prev_vy = prev['x_combined'][3] if len(prev['x_combined']) > 3 else 0

            # Cross product for turn rate
            cross = vx * prev_vy - vy * prev_vx
            prev_speed = np.sqrt(prev_vx**2 + prev_vy**2)
            if prev_speed > 10.0:
                omega_measured = cross / (speed * prev_speed * self.dt + 1e-10)
                # Exponential moving average
                alpha = 0.3
                self.omega_est = alpha * omega_measured + (1 - alpha) * self.omega_est

        # Apply to CT models
        if "CT_plus" in self.filters:
            self.filters["CT_plus"].set_omega(abs(self.omega_est))
        if "CT_minus" in self.filters:
            self.filters["CT_minus"].set_omega(-abs(self.omega_est))

    def initialize(self, z: np.ndarray):
        """Initialize tracker from first measurement."""
        for model_name, f in self.filters.items():
            f.x[:2] = z[:2]
            if f.state_dim >= 4:
                f.x[2:4] = [0, 0]
            f.P = np.eye(f.state_dim) * 100.0

        self.initialized = True
        self.step_count = 0

    def update(self, z: np.ndarray) -> Tuple[np.ndarray, np.ndarray, IntentState]:
        """
        Full IMM update cycle. Returns (x_realtime, P_combined, intent_state).

        [REQ-V40-08] All 4 streams are populated.
        """
        if not self.initialized:
            self.initialize(z)
            # Store initialization in forward history for smoother alignment
            x0 = np.zeros(4)
            x0[:2] = z[:2]
            P0 = np.eye(4) * 100.0
            F_cv = np.eye(4)
            F_cv[0, 2] = self.dt; F_cv[1, 3] = self.dt
            self.forward_history.append({
                'x_combined': x0,
                'P_combined': P0,
                'x_predicted': x0.copy(),
                'P_predicted': P0.copy(),
                'x_models': {m: self.filters[m].x.copy() for m in self.active_models},
                'P_models': {m: self.filters[m].P.copy() for m in self.active_models},
                'mu': dict(self.mu),
                'active_models': list(self.active_models),
                'F_models': {m: self.filters[m].F.copy() for m in self.active_models},
            })
            self.intent_state.active_models = list(self.active_models)
            return z.copy(), np.eye(2) * self.r_std**2, self.intent_state

        self.step_count += 1

        # =====================================================================
        # Step 1: IMM Mixing
        # =====================================================================
        mixed_x = {}
        mixed_P = {}

        for mj in self.active_models:
            fj = self.filters[mj]
            c_j = sum(self.tpm[mi][mj] * self.mu[mi]
                      for mi in self.active_models if mi in self.tpm)
            c_j = max(c_j, 1e-30)

            # Mixed state
            x_mix = np.zeros(fj.state_dim)
            for mi in self.active_models:
                fi = self.filters[mi]
                mu_ij = (self.tpm.get(mi, {}).get(mj, 0) * self.mu.get(mi, 0)) / c_j
                # Map states (handle dimension mismatch)
                xi_mapped = np.zeros(fj.state_dim)
                xi_mapped[:min(fi.state_dim, fj.state_dim)] = fi.x[:min(fi.state_dim, fj.state_dim)]
                x_mix += mu_ij * xi_mapped

            # Mixed covariance
            P_mix = np.zeros((fj.state_dim, fj.state_dim))
            for mi in self.active_models:
                fi = self.filters[mi]
                mu_ij = (self.tpm.get(mi, {}).get(mj, 0) * self.mu.get(mi, 0)) / c_j
                xi_mapped = np.zeros(fj.state_dim)
                xi_mapped[:min(fi.state_dim, fj.state_dim)] = fi.x[:min(fi.state_dim, fj.state_dim)]
                dx = xi_mapped - x_mix
                Pi_mapped = np.zeros((fj.state_dim, fj.state_dim))
                d = min(fi.state_dim, fj.state_dim)
                Pi_mapped[:d, :d] = fi.P[:d, :d]
                P_mix += mu_ij * (Pi_mapped + np.outer(dx, dx))

            mixed_x[mj] = x_mix
            mixed_P[mj] = P_mix

        # =====================================================================
        # Step 2: Predict + Update per model
        # =====================================================================
        likelihoods = {}
        predictions = {}  # Store predictions for RTS smoother

        for m in self.active_models:
            f = self.filters[m]
            f.x = mixed_x[m]
            f.P = mixed_P[m]

            # Predict
            x_pred, P_pred = f.predict()
            predictions[m] = {'x_pred': x_pred.copy(), 'P_pred': P_pred.copy()}

            # Likelihood (before update, using predicted state)
            likelihoods[m] = f.likelihood(z, self.R)

            # Update
            x_filt, P_filt, nu, S = f.update(z, self.R)

        # =====================================================================
        # Step 3: Model probability update
        # =====================================================================
        c_bar = {}
        for mj in self.active_models:
            c_bar[mj] = sum(self.tpm.get(mi, {}).get(mj, 0) * self.mu.get(mi, 0)
                            for mi in self.active_models)

        total = sum(likelihoods[m] * c_bar[m] for m in self.active_models)
        total = max(total, 1e-300)

        for m in self.active_models:
            self.mu[m] = (likelihoods[m] * c_bar[m]) / total
            self.mu[m] = max(self.mu[m], 1e-30)

        # Renormalize
        mu_sum = sum(self.mu.values())
        self.mu = {m: p/mu_sum for m, p in self.mu.items()}

        # =====================================================================
        # Step 4: Combined estimate (realtime output)
        # =====================================================================
        # Use common state space [x, y, vx, vy] for combination
        x_combined = np.zeros(4)
        P_combined = np.zeros((4, 4))

        # Also compute predicted combined for smoother
        x_predicted = np.zeros(4)
        P_predicted = np.zeros((4, 4))

        for m in self.active_models:
            f = self.filters[m]
            pos, vel = f.get_position_velocity()
            xi = np.concatenate([pos, vel])
            x_combined += self.mu[m] * xi

            # Predicted state (pre-update)
            pred = predictions[m]
            xi_pred = np.zeros(4)
            xi_pred[:min(4, len(pred['x_pred']))] = pred['x_pred'][:min(4, len(pred['x_pred']))]
            x_predicted += self.mu[m] * xi_pred

        for m in self.active_models:
            f = self.filters[m]
            pos, vel = f.get_position_velocity()
            xi = np.concatenate([pos, vel])
            dx = xi - x_combined
            Pi = np.eye(4) * 10.0
            d = min(4, f.state_dim)
            Pi[:d, :d] = f.P[:d, :d]
            P_combined += self.mu[m] * (Pi + np.outer(dx, dx))

            # Predicted covariance
            pred = predictions[m]
            xi_pred = np.zeros(4)
            xi_pred[:min(4, len(pred['x_pred']))] = pred['x_pred'][:min(4, len(pred['x_pred']))]
            dx_pred = xi_pred - x_predicted
            Pi_pred = np.eye(4) * 10.0
            d_pred = min(4, pred['P_pred'].shape[0])
            Pi_pred[:d_pred, :d_pred] = pred['P_pred'][:d_pred, :d_pred]
            P_predicted += self.mu[m] * (Pi_pred + np.outer(dx_pred, dx_pred))

        # =====================================================================
        # Step 5: Adaptive omega
        # =====================================================================
        self._adaptive_omega(x_combined)

        # =====================================================================
        # Step 6: Platform identification & VS-IMM adaptation
        # =====================================================================
        # Compute innovation norm from best model for classifier
        best_model_name = max(self.mu, key=self.mu.get)
        best_filter = self.filters[best_model_name]
        innov_norm = np.linalg.norm(z - best_filter.H @ best_filter.x)

        platform_type, confidence = self.identifier.update(
            x_combined[:2], x_combined[2:4], self.dt,
            model_probs=dict(self.mu),
            innovation_norm=innov_norm)

        platform_data = self.platform_db.get(platform_type,
                                              self.platform_db.get("Unknown", {}))

        if confidence >= 0.4 and platform_type != "Unknown":
            self._activate_models_from_platform(platform_data)
            self._apply_platform_tpm_bias(platform_data)

        # =====================================================================
        # Step 7: Intent prediction + Q scaling
        # =====================================================================
        speed = np.linalg.norm(x_combined[2:4])
        accel_g = 0
        if len(self.forward_history) >= 1:
            prev_vel = self.forward_history[-1]['x_combined'][2:4]
            accel_g = np.linalg.norm(x_combined[2:4] - prev_vel) / (self.dt * GRAVITY + 1e-10)

        phase, phase_conf = self.intent_predictor.predict(
            platform_type, platform_data, speed, accel_g, self.mu)

        self._apply_q_scaling(platform_data, phase)

        # =====================================================================
        # Step 8: Pruning
        # =====================================================================
        self._prune_models()

        # =====================================================================
        # Step 9: Store forward history for smoother
        # =====================================================================
        fwd_entry = {
            'x_combined': x_combined.copy(),
            'P_combined': P_combined.copy(),
            'x_predicted': x_predicted.copy(),
            'P_predicted': P_predicted.copy(),
            'x_models': {m: self.filters[m].x.copy() for m in self.active_models},
            'P_models': {m: self.filters[m].P.copy() for m in self.active_models},
            'mu': dict(self.mu),
            'active_models': list(self.active_models),
            'F_models': {m: self.filters[m].F.copy() for m in self.active_models},
        }
        self.forward_history.append(fwd_entry)

        # =====================================================================
        # Step 10: Intent state output
        # =====================================================================
        cat = platform_data.get("category", "unknown")
        threat = self._compute_threat_level(cat, speed, accel_g, phase)

        self.intent_state = IntentState(
            platform_type=platform_type,
            platform_category=cat,
            confidence=confidence,
            phase=phase,
            phase_confidence=phase_conf,
            estimated_max_g=platform_data.get("max_g", 30.0),
            estimated_max_speed=platform_data.get("max_speed_mps", 3500),
            threat_level=threat,
            active_models=list(self.active_models),
            n_active_models=len(self.active_models)
        )

        return x_combined[:2], P_combined[:2, :2], self.intent_state

    def _compute_threat_level(self, category: str, speed: float,
                              accel_g: float, phase: str) -> float:
        """Simple threat level computation."""
        threat = 0.3  # Baseline

        # Category-based
        cat_threats = {
            "fighter": 0.6, "bomber": 0.4, "transport": 0.1, "aew": 0.2,
            "ballistic_missile": 0.9, "hypersonic_missile": 0.95,
            "cruise_missile": 0.8, "sam": 0.85, "mlrs": 0.7,
            "small_drone": 0.3, "fpv_kamikaze": 0.7, "loitering_munition": 0.6,
        }
        threat = cat_threats.get(category, 0.5)

        # Phase modifier
        if phase in ("terminal", "attack", "engagement", "post_stall"):
            threat = min(threat * 1.3, 1.0)
        elif phase in ("cruise", "patrol", "orbit", "hover"):
            threat *= 0.8

        # Speed/accel modifier
        if speed > 1000:
            threat = min(threat * 1.1, 1.0)
        if accel_g > 5:
            threat = min(threat * 1.1, 1.0)

        return min(max(threat, 0.0), 1.0)

    # =========================================================================
    # Smoother Methods (Streams 2 & 3)
    # =========================================================================
    def get_forward_estimates(self) -> List[np.ndarray]:
        """Stream 1: All forward (realtime) estimates."""
        return [h['x_combined'][:2] for h in self.forward_history]

    def get_window_smoothed_estimates(self, window: Optional[int] = None) -> List[np.ndarray]:
        """
        Stream 2: Window-N RTS smoothed estimates.
        [REQ-V40-07]
        Uses IMM-weighted F matrix for model-aware backward pass.
        """
        N = window or self.window_size
        history = list(self.forward_history)
        if len(history) < 2:
            return [h['x_combined'][:2] for h in history]

        T = len(history)
        start = max(0, T - N)
        segment = history[start:T]
        n_seg = len(segment)

        # Fallback CV matrix
        F_cv = np.eye(4)
        F_cv[0, 2] = self.dt
        F_cv[1, 3] = self.dt

        # Initialize: xs[N-1] = xf[N-1]
        xs = [None] * n_seg
        Ps = [None] * n_seg
        xs[-1] = segment[-1]['x_combined'][:4].copy()
        Ps[-1] = segment[-1]['P_combined'][:4, :4].copy()

        # Backward pass
        for k in range(n_seg - 2, -1, -1):
            xf_k = segment[k]['x_combined'][:4]
            Pf_k = segment[k]['P_combined'][:4, :4]

            # IMM-weighted F matrix from this timestep
            F_combined = np.zeros((4, 4))
            mu_k = segment[k].get('mu', {})
            F_models = segment[k].get('F_models', {})
            mu_total = sum(mu_k.values()) if mu_k else 1.0

            if F_models and mu_k:
                for m_name, F_m in F_models.items():
                    w = mu_k.get(m_name, 0) / max(mu_total, 1e-10)
                    d = min(4, F_m.shape[0])
                    F_combined[:d, :d] += w * F_m[:d, :d]
            else:
                F_combined = F_cv.copy()

            # Use stored predicted state/cov from step k+1
            xp_k1 = segment[k+1].get('x_predicted', F_combined @ xf_k)[:4]
            Pp_k1 = segment[k+1].get('P_predicted', F_combined @ Pf_k @ F_combined.T + np.eye(4) * 0.1)[:4, :4]

            # Regularize Pp for inversion
            Pp_reg = Pp_k1 + np.eye(4) * 1e-6
            try:
                Pp_inv = np.linalg.inv(Pp_reg)
            except np.linalg.LinAlgError:
                Pp_inv = np.linalg.pinv(Pp_reg)

            # Smoother gain
            G = Pf_k @ F_combined.T @ Pp_inv
            G = np.clip(G, -5.0, 5.0)

            # Smoothed state
            innovation = xs[k+1] - xp_k1
            innovation = np.clip(innovation, -1000.0, 1000.0)
            xs[k] = xf_k + G @ innovation

            # Smoothed covariance
            Ps[k] = Pf_k + G @ (Ps[k+1] - Pp_k1) @ G.T
            Ps[k] = 0.5 * (Ps[k] + Ps[k].T)

        # Build full output
        result = []
        for i in range(start):
            result.append(history[i]['x_combined'][:2])
        for xs_k in xs:
            result.append(xs_k[:2])

        return result

    def get_best_estimates(self) -> Tuple[List[np.ndarray], str]:
        """
        Smart stream selection: returns the best of forward/window.
        [REQ-V40-08] Strategy:
          1. High speed (>500 m/s) → forward (smoother degrades on fast dynamics)
          2. Low filter uncertainty (P trace < threshold) → forward (already accurate)
          3. Otherwise → window-smoothed (helps on maneuvering targets)
        """
        fwd = self.get_forward_estimates()
        if len(fwd) < 10:
            return fwd, "forward"

        # Estimate current speed from last 10 steps
        recent_speeds = []
        recent_P_traces = []
        for h in list(self.forward_history)[-10:]:
            vel = h['x_combined'][2:4]
            recent_speeds.append(np.linalg.norm(vel))
            recent_P_traces.append(np.trace(h['P_combined'][:2, :2]))

        avg_speed = np.mean(recent_speeds)
        avg_P = np.mean(recent_P_traces)

        # High-speed → forward always (smoother can't improve, often degrades)
        if avg_speed > 400:
            return fwd, "forward"

        # Very low uncertainty → forward already optimal
        if avg_P < self.R[0, 0] * 0.5:
            return fwd, "forward"

        win = self.get_window_smoothed_estimates()

        # Sanity check: if smoother diverges from forward, use forward
        n_check = min(10, len(fwd), len(win))
        max_dev = 0
        for i in range(-n_check, 0):
            dev = np.linalg.norm(np.array(win[i]) - np.array(fwd[i]))
            max_dev = max(max_dev, dev)

        # If smoother deviates more than 5x forward uncertainty, it's unreliable
        if max_dev > 5.0 * np.sqrt(avg_P):
            return fwd, "forward"

        return win, "window_smoothed"

    def get_smoothed_estimates(self) -> List[np.ndarray]:
        """Stream 3: Full-track RTS smoothed estimates."""
        return self.get_window_smoothed_estimates(window=len(self.forward_history))


# =============================================================================
# Convenience factory
# =============================================================================
def create_tracker(dt: float = 0.1, r_std: float = 2.5,
                   platform_db_path: Optional[str] = None,
                   window_size: int = 30) -> NxMimosaV40Sentinel:
    """Create a v4.0 SENTINEL tracker with default settings."""
    return NxMimosaV40Sentinel(
        dt=dt, r_std=r_std,
        platform_db_path=platform_db_path,
        window_size=window_size
    )
