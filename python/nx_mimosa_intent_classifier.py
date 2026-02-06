"""
NX-MIMOSA v4.1 — Intent Prediction + Multi-Domain Classifier + ECM Detector
=============================================================================

Three integrated modules for platform-aware tracking:

1. ImprovedPlatformClassifier — Hierarchical coarse→fine classification
   using full multi-domain DB (RCS, micro-Doppler, HRR, kinematics, ECM, altitude)
   
2. ECMDetector — Real-time electronic countermeasure detection
   with SNR monitoring, RCS anomaly detection, and Doppler analysis
   
3. IntentPredictor — Behavioral intent prediction from kinematic features
   Outputs threat assessment streams ("terminal_dive_imminent", "evasion_break", etc.)

Copyright (c) 2025-2026 Nexellum d.o.o. — All rights reserved.
Author: Dr. Mladen Mešter, mladen@nexellum.com
License: AGPL v3 (open-source) / Commercial available
"""

import json
import math
import numpy as np
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, Dict, Tuple, List, Any
from enum import Enum

# =============================================================================
# [REQ-CLS-001] Data types and enumerations
# =============================================================================

class ThreatLevel(Enum):
    """Threat assessment level for intent prediction."""
    NONE = 0        # Benign (civil, false target)
    LOW = 1         # Non-threatening military
    MEDIUM = 2      # Potential threat, monitoring
    HIGH = 3        # Active threat, weapons engagement
    CRITICAL = 4    # Imminent impact / terminal phase


class IntentType(Enum):
    """Predicted behavioral intent categories."""
    CRUISE = "cruise"
    LOITER = "loiter"
    EVASION_BREAK = "evasion_break"
    ATTACK_RUN = "attack_run"
    TERMINAL_DIVE = "terminal_dive"
    SEA_SKIMMING = "sea_skimming"
    POP_UP = "pop_up"
    SKIP_GLIDE = "skip_glide"
    ORBIT_RACETRACK = "orbit_racetrack"
    BVR_INTERCEPT = "bvr_intercept"
    DOGFIGHT = "dogfight"
    TERRAIN_FOLLOWING = "terrain_following"
    JAMMER_STANDOFF = "jammer_standoff"
    REENTRY = "reentry"
    FALSE_TARGET = "false_target"
    UNKNOWN = "unknown"


class ECMStatus(Enum):
    """ECM environment assessment."""
    CLEAN = "clean"
    NOISE_JAMMING = "noise_jamming"
    DECEPTION_RGPO = "deception_rgpo"
    DECEPTION_VGPO = "deception_vgpo"
    DRFM_REPEATER = "drfm_repeater"
    CHAFF_CORRIDOR = "chaff_corridor"
    MULTI_SOURCE = "multi_source"


@dataclass
class ClassificationResult:
    """Output of the platform classifier."""
    platform_name: str = "Unknown"
    platform_class: str = "unknown"
    confidence: float = 0.0
    threat_level: ThreatLevel = ThreatLevel.NONE
    preferred_models: List[str] = field(default_factory=lambda: ["CV"])
    ecm_status: ECMStatus = ECMStatus.CLEAN
    intent: IntentType = IntentType.UNKNOWN
    intent_confidence: float = 0.0
    # Multi-domain features used
    features_used: Dict[str, float] = field(default_factory=dict)
    # Alert messages
    alerts: List[str] = field(default_factory=list)


@dataclass
class IntentState:
    """Running state for intent prediction per track."""
    intent: IntentType = IntentType.UNKNOWN
    intent_confidence: float = 0.0
    threat_level: ThreatLevel = ThreatLevel.NONE
    dive_angle_deg: float = 0.0
    altitude_rate_mps: float = 0.0
    time_to_impact_s: float = float('inf')
    closing_speed_mps: float = 0.0
    orbit_count: int = 0
    heading_reversal_count: int = 0
    alerts: List[str] = field(default_factory=list)


# =============================================================================
# [REQ-ECM-001] ECM Detector
# =============================================================================

class ECMDetector:
    """Real-time Electronic Countermeasure detection.
    
    Monitors SNR, RCS variance, Doppler spread, and track quality
    to detect noise jamming, deception (RGPO/VGPO), and DRFM repeaters.
    
    [REQ-ECM-002] Outputs ECM status and recommended Q scaling factor.
    """
    
    def __init__(self, 
                 snr_threshold_db: float = 10.0,
                 rcs_var_threshold: float = 5.0,
                 doppler_spread_threshold: float = 100.0,
                 nis_spike_threshold: float = 50.0,
                 history_len: int = 30):
        self.snr_threshold = snr_threshold_db
        self.rcs_var_threshold = rcs_var_threshold
        self.doppler_spread_threshold = doppler_spread_threshold
        self.nis_spike_threshold = nis_spike_threshold
        
        # Running statistics
        self.snr_history = deque(maxlen=history_len)
        self.rcs_history = deque(maxlen=history_len)
        self.doppler_history = deque(maxlen=history_len)
        self.nis_history = deque(maxlen=history_len)
        self.range_rate_history = deque(maxlen=history_len)
        
        # Baseline (established during clean period)
        self.nominal_snr = 25.0
        self.nominal_rcs_var = 1.0
        self.baseline_established = False
        self.clean_samples = 0
        
        # State
        self.status = ECMStatus.CLEAN
        self.q_scale_factor = 1.0
        self.confidence = 0.0
    
    def update(self, snr_db: float = 20.0, rcs_amplitude: float = 1.0,
               doppler_hz: float = 0.0, nis: float = 1.0,
               range_m: float = 0.0) -> Tuple[ECMStatus, float]:
        """Update ECM detector with new measurement.
        
        Returns:
            (ECMStatus, q_scale_factor) — status and recommended Q multiplier
        """
        self.snr_history.append(snr_db)
        self.rcs_history.append(rcs_amplitude)
        self.doppler_history.append(doppler_hz)
        self.nis_history.append(nis)
        self.range_rate_history.append(range_m)
        
        if len(self.snr_history) < 10:
            return ECMStatus.CLEAN, 1.0
        
        # Establish baseline from first clean period
        if not self.baseline_established and len(self.snr_history) >= 20:
            snr_arr = np.array(list(self.snr_history)[:20])
            rcs_arr = np.array(list(self.rcs_history)[:20])
            if np.std(snr_arr) < 5.0:  # reasonably stable
                self.nominal_snr = np.mean(snr_arr)
                self.nominal_rcs_var = np.var(rcs_arr)
                self.baseline_established = True
        
        # --- Detection logic ---
        snr_arr = np.array(self.snr_history)
        rcs_arr = np.array(self.rcs_history)
        dop_arr = np.array(self.doppler_history)
        nis_arr = np.array(self.nis_history)
        
        snr_mean = np.mean(snr_arr[-10:])
        rcs_var = np.var(rcs_arr[-10:])
        dop_spread = np.std(dop_arr[-10:])
        nis_mean = np.mean(nis_arr[-10:])
        nis_peak = np.max(nis_arr[-10:])
        
        ecm_score = 0.0
        detected_type = ECMStatus.CLEAN
        
        # [REQ-ECM-003] Noise jamming detection
        if snr_mean < self.snr_threshold:
            ecm_score += 3.0
            detected_type = ECMStatus.NOISE_JAMMING
        
        # [REQ-ECM-004] RCS anomaly (deception or chaff)
        if self.baseline_established and rcs_var > self.rcs_var_threshold * self.nominal_rcs_var:
            ecm_score += 2.0
            if detected_type == ECMStatus.CLEAN:
                detected_type = ECMStatus.CHAFF_CORRIDOR
        
        # [REQ-ECM-005] Doppler spread anomaly
        if dop_spread > self.doppler_spread_threshold:
            ecm_score += 2.5
            if detected_type == ECMStatus.CLEAN:
                detected_type = ECMStatus.NOISE_JAMMING
        
        # [REQ-ECM-006] NIS spike detection
        if nis_peak > self.nis_spike_threshold:
            ecm_score += 1.5
        
        # [REQ-ECM-007] RGPO/VGPO detection (range rate anomaly)
        if len(self.range_rate_history) >= 10:
            range_arr = np.array(list(self.range_rate_history)[-10:])
            range_accel = np.diff(np.diff(range_arr))
            if len(range_accel) > 0 and np.max(np.abs(range_accel)) > 500:
                ecm_score += 3.0
                detected_type = ECMStatus.DECEPTION_RGPO
        
        # [REQ-ECM-008] DRFM detection (correlated delayed returns)
        if len(self.rcs_history) >= 20:
            rcs_recent = np.array(list(self.rcs_history)[-20:])
            # Look for periodic correlation (DRFM typically has ~constant delay)
            autocorr = np.correlate(rcs_recent - np.mean(rcs_recent),
                                     rcs_recent - np.mean(rcs_recent), 'full')
            autocorr = autocorr[len(autocorr)//2:]
            if len(autocorr) > 5:
                # Secondary peak suggests repeater
                peaks = autocorr[3:] / (autocorr[0] + 1e-10)
                if np.any(peaks > 0.7):
                    ecm_score += 4.0
                    detected_type = ECMStatus.DRFM_REPEATER
        
        # Aggregate
        self.confidence = min(1.0, ecm_score / 8.0)  # normalize to [0, 1]
        
        if ecm_score >= 3.0:
            self.status = detected_type
            # Q scaling: higher ECM score → more process noise
            self.q_scale_factor = min(50.0, 1.0 + ecm_score * 5.0)
        else:
            self.status = ECMStatus.CLEAN
            self.q_scale_factor = 1.0
        
        return self.status, self.q_scale_factor


# =============================================================================
# [REQ-CLS-002] Improved Platform Classifier
# =============================================================================

class ImprovedPlatformClassifier:
    """Hierarchical multi-domain platform classifier.
    
    Pipeline:
    1. Feature extraction from kinematic + radar observables
    2. Coarse classification (rule-based, 31 classes)
    3. Fine classification (weighted scoring within class)
    4. Confidence thresholding with fallback to class defaults
    5. ECM-aware degradation (ignore radar features when jammed)
    
    [REQ-CLS-003] FPGA-friendly: Pure NumPy, no ML dependencies.
    """
    
    # Coarse class groupings for speed-based primary sort
    SPEED_CLASSES = {
        'stationary':      (0, 5),
        'very_slow':       (5, 30),
        'slow':            (30, 80),
        'subsonic_low':    (80, 200),
        'subsonic_medium': (200, 340),
        'subsonic_high':   (340, 600),
        'transonic':       (600, 900),
        'supersonic':      (900, 2000),
        'hypersonic':      (2000, 10000),
    }
    
    # Threat level mapping by class
    THREAT_MAP = {
        'false_target_bird': ThreatLevel.NONE,
        'false_target_balloon': ThreatLevel.NONE,
        'false_target_ground': ThreatLevel.NONE,
        'false_target_environment': ThreatLevel.NONE,
        'paraglider_ultralight': ThreatLevel.NONE,
        'space_object': ThreatLevel.NONE,
        'commercial_airliner': ThreatLevel.NONE,
        'general_aviation': ThreatLevel.NONE,
        'helicopter_civil': ThreatLevel.NONE,
        'cargo_military': ThreatLevel.LOW,
        'helicopter_military': ThreatLevel.MEDIUM,
        'uav_low_g': ThreatLevel.MEDIUM,
        'uav_micro': ThreatLevel.MEDIUM,
        'uav_stealth': ThreatLevel.HIGH,
        'uav_swarm': ThreatLevel.HIGH,
        'strategic_bomber': ThreatLevel.HIGH,
        '4th_gen_fighter': ThreatLevel.HIGH,
        '5th_gen_stealth': ThreatLevel.HIGH,
        '6th_gen_concept': ThreatLevel.CRITICAL,
        'subsonic_cruise': ThreatLevel.HIGH,
        'supersonic_cruise': ThreatLevel.CRITICAL,
        'hypersonic_cruise': ThreatLevel.CRITICAL,
        'hypersonic_glide': ThreatLevel.CRITICAL,
        'ballistic': ThreatLevel.CRITICAL,
        'anti_ship_ballistic': ThreatLevel.CRITICAL,
        'aam_short_range': ThreatLevel.CRITICAL,
        'aam_bvr': ThreatLevel.CRITICAL,
        'sam_terminal': ThreatLevel.CRITICAL,
        'unknown_future': ThreatLevel.HIGH,
        'unknown_civil': ThreatLevel.LOW,
        'unknown_low_g': ThreatLevel.MEDIUM,
    }
    
    def __init__(self, db_path: str = "data/platform_db_v3.json",
                 history_len: int = 30,
                 conf_threshold: float = 0.60,
                 ecm_detector: Optional[ECMDetector] = None):
        # Load platform database
        try:
            with open(db_path, 'r') as f:
                self.db: Dict[str, Dict] = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            self.db = {}
        
        self.history_len = history_len
        self.conf_threshold = conf_threshold
        self.ecm_detector = ecm_detector or ECMDetector()
        
        # Precompute class groups
        self.class_groups: Dict[str, List[str]] = {}
        for plat, data in self.db.items():
            cls = data.get("class", "unknown")
            self.class_groups.setdefault(cls, []).append(plat)
        
        # Feature accumulation history
        self.speed_history = deque(maxlen=history_len)
        self.accel_history = deque(maxlen=history_len)
        self.altitude_history = deque(maxlen=history_len)
        self.omega_history = deque(maxlen=history_len)
        self.rcs_history = deque(maxlen=history_len)
        self.nis_history = deque(maxlen=history_len)
        self.heading_history = deque(maxlen=history_len)
        
        # Running peaks and averages (EMA)
        self.alpha = 0.15  # moderate EMA — converges in ~15 samples
        self.spd_avg = 0.0
        self.spd_peak = 0.0
        self.omega_avg = 0.0
        self.omega_peak = 0.0
        self.alt_avg = 0.0
        self.g_peak = 0.0
        self.nis_avg = 1.0
        self.n_updates = 0
        
        # Previous classification (persistence)
        self._prev_result = ClassificationResult()
        self._class_hold_counter = 0
    
    def update_features(self, speed_mps: float, accel_g: float,
                        altitude_m: float, omega_radps: float,
                        heading_rad: float, nis_cv: float = 1.0,
                        rcs_dbsm: float = 0.0,
                        snr_db: float = 25.0, doppler_hz: float = 0.0,
                        range_m: float = 0.0):
        """Update all feature histories from tracker output.
        
        [REQ-CLS-004] Called once per measurement update.
        """
        self.speed_history.append(speed_mps)
        self.accel_history.append(accel_g)
        self.altitude_history.append(altitude_m)
        self.omega_history.append(abs(omega_radps))
        self.heading_history.append(heading_rad)
        self.nis_history.append(nis_cv)
        self.rcs_history.append(rcs_dbsm)
        
        # EMA updates
        a = self.alpha
        self.spd_avg = a * speed_mps + (1-a) * self.spd_avg
        self.spd_peak = max(self.spd_peak, speed_mps)
        self.omega_avg = a * abs(omega_radps) + (1-a) * self.omega_avg
        self.omega_peak = max(self.omega_peak, abs(omega_radps))
        self.alt_avg = a * altitude_m + (1-a) * self.alt_avg
        self.g_peak = max(self.g_peak, accel_g)
        self.nis_avg = a * nis_cv + (1-a) * self.nis_avg
        self.n_updates += 1
        
        # Update ECM detector
        self.ecm_detector.update(snr_db, 10**(rcs_dbsm/10) if rcs_dbsm != 0 else 1.0,
                                  doppler_hz, nis_cv, range_m)
    
    def _extract_features(self) -> Dict[str, float]:
        """Extract classification features from accumulated history."""
        if self.n_updates < 5:
            return {}
        
        spd_arr = np.array(self.speed_history)
        alt_arr = np.array(self.altitude_history)
        omega_arr = np.array(self.omega_history)
        accel_arr = np.array(self.accel_history)
        nis_arr = np.array(self.nis_history)
        
        # Altitude rate (vertical speed proxy)
        alt_rate = 0.0
        if len(alt_arr) >= 5:
            alt_rate = (alt_arr[-1] - alt_arr[-5]) / (5 * 0.1)  # assuming dt=0.1s
        
        # Heading reversal count (orbit detection)
        heading_arr = np.array(self.heading_history)
        reversals = 0
        if len(heading_arr) >= 10:
            dh = np.diff(heading_arr)
            # Wrap heading differences to [-pi, pi]
            dh = np.arctan2(np.sin(dh), np.cos(dh))
            sign_changes = np.diff(np.sign(dh))
            reversals = int(np.sum(np.abs(sign_changes) > 0))
        
        return {
            "spd_avg": self.spd_avg,
            "spd_max": self.spd_peak,
            "spd_std": float(np.std(spd_arr[-20:])) if len(spd_arr) >= 5 else 0.0,
            "alt_avg": self.alt_avg,
            "alt_rate": alt_rate,
            "alt_std": float(np.std(alt_arr[-20:])) if len(alt_arr) >= 5 else 0.0,
            "omega_avg": self.omega_avg,
            "omega_peak": self.omega_peak,
            "g_peak": self.g_peak,
            "g_avg": float(np.mean(accel_arr[-20:])) if len(accel_arr) >= 5 else 0.0,
            "nis_avg": self.nis_avg,
            "nis_peak": float(np.max(nis_arr[-20:])) if len(nis_arr) >= 5 else 1.0,
            "heading_reversals": reversals,
            "rcs_mean": float(np.mean(list(self.rcs_history)[-20:])) if self.rcs_history else 0.0,
            "rcs_var": float(np.var(list(self.rcs_history)[-20:])) if len(self.rcs_history) >= 5 else 0.0,
            "ecm_active": 1.0 if self.ecm_detector.status != ECMStatus.CLEAN else 0.0,
        }
    
    def _coarse_classify(self, f: Dict[str, float]) -> List[str]:
        """Coarse classification into candidate classes.
        
        [REQ-CLS-005] Rule-based, fast, returns 1-3 candidate classes.
        When ECM active, ignores radar features (RCS, micro-Doppler).
        """
        spd = f.get("spd_avg", 0)
        alt = f.get("alt_avg", 0)
        g_pk = f.get("g_peak", 0)
        omega = f.get("omega_peak", 0)
        alt_rate = f.get("alt_rate", 0)
        ecm = f.get("ecm_active", 0) > 0.5
        rcs = f.get("rcs_mean", 0) if not ecm else None  # ignore if jammed
        
        candidates = []
        
        # === Stationary / very slow → false targets ===
        if spd < 5:
            return ["false_target_ground", "false_target_environment", "false_target_balloon"]
        
        if spd < 25:
            if alt > 10000:
                candidates.append("false_target_balloon")
                candidates.append("space_object")
            else:
                candidates.append("false_target_bird")
                candidates.append("paraglider_ultralight")
                candidates.append("false_target_ground")
            return candidates[:3]
        
        # === Very slow with flutter → birds or paragliders ===
        if spd < 60:
            if alt < 300 and g_pk < 3:
                candidates.append("uav_micro")
                candidates.append("paraglider_ultralight")
            if omega > 0.5:
                candidates.append("false_target_bird")
            if alt < 5000:
                candidates.append("general_aviation")
                candidates.append("helicopter_civil")
            return candidates[:3] if candidates else ["unknown_low_g"]
        
        # === Slow subsonic (60-200) ===
        if spd < 200:
            if alt < 500:
                candidates.append("uav_swarm")
                candidates.append("helicopter_military")
            if alt < 5000:
                candidates.append("uav_low_g")
                candidates.append("helicopter_military")
            if 5000 <= alt <= 15000:
                candidates.append("uav_low_g")
                candidates.append("general_aviation")
            return candidates[:3] if candidates else ["uav_low_g"]
        
        # === Subsonic medium (200-340) ===
        if spd < 340:
            if alt < 200:
                candidates.append("subsonic_cruise")  # sea skimming missile
            if alt > 8000 and g_pk < 3:
                candidates.append("commercial_airliner")
                candidates.append("cargo_military")
                candidates.append("strategic_bomber")
            elif g_pk < 4:
                candidates.append("uav_stealth")
                candidates.append("cargo_military")
                candidates.append("strategic_bomber")
            else:
                candidates.append("subsonic_cruise")
            return candidates[:3]
        
        # === Subsonic high / transonic (340-900) ===
        if spd < 900:
            if g_pk > 7:
                candidates.append("4th_gen_fighter")
                candidates.append("5th_gen_stealth")
                candidates.append("6th_gen_concept")
            elif g_pk > 4:
                candidates.append("4th_gen_fighter")
                candidates.append("5th_gen_stealth")
                candidates.append("strategic_bomber")
            else:
                candidates.append("strategic_bomber")
                candidates.append("6th_gen_concept")
                if alt > 10000:
                    candidates.append("commercial_airliner")
            return candidates[:3]
        
        # === Supersonic (900-2000) ===
        if spd < 2000:
            if g_pk > 20:
                candidates.append("aam_bvr")
                candidates.append("sam_terminal")
                candidates.append("aam_short_range")
            elif g_pk > 10:
                candidates.append("supersonic_cruise")
                candidates.append("aam_bvr")
            else:
                candidates.append("supersonic_cruise")
                candidates.append("ballistic")
            return candidates[:3]
        
        # === Hypersonic (2000+) ===
        if alt_rate < -500:  # diving
            candidates.append("ballistic")
            candidates.append("anti_ship_ballistic")
            candidates.append("hypersonic_glide")
        else:
            candidates.append("hypersonic_glide")
            candidates.append("hypersonic_cruise")
            candidates.append("sam_terminal")
        
        return candidates[:3] if candidates else ["unknown_future"]
    
    def _parse_rcs_range(self, rcs_str: str) -> Tuple[float, float]:
        """Parse RCS string like '0.01-0.1' or '100+' into (low, high)."""
        s = str(rcs_str).replace('–', '-').replace(' ', '')
        if '+' in s:
            val = float(s.replace('+', ''))
            return val, val * 5
        parts = s.split('-')
        try:
            if len(parts) == 2:
                return float(parts[0]), float(parts[1])
            return float(parts[0]), float(parts[0])
        except (ValueError, IndexError):
            return 1.0, 1.0
    
    def _parse_altitude_range(self, alt_str: str) -> Tuple[float, float]:
        """Parse altitude string like '10000-15000' or '10000+' or 'variable'."""
        s = str(alt_str).replace('–', '-').replace(' ', '')
        if s == 'variable' or s == '0':
            return 0, 50000
        if '+' in s:
            val = float(s.replace('+', ''))
            return val, val * 2
        parts = s.split('-')
        try:
            return float(parts[0]), float(parts[-1])
        except ValueError:
            return 0, 50000
    
    def _score_platform(self, plat_data: Dict, f: Dict[str, float],
                         ecm_active: bool) -> float:
        """Score a specific platform against observed features.
        
        [REQ-CLS-006] Weighted feature matching.
        Returns score in range [-inf, 0] (higher = better match).
        """
        score = 0.0
        
        # --- Speed match (weight: 3) ---
        max_spd = plat_data.get("max_speed_mps", 500)
        if isinstance(max_spd, (int, float)):
            spd_ratio = f["spd_avg"] / (max_spd + 1e-6)
            # Best match: observed speed is 50-90% of max
            if 0.3 <= spd_ratio <= 1.0:
                score += 3.0 * (1.0 - abs(spd_ratio - 0.7) / 0.7)
            elif spd_ratio > 1.2:
                score -= 5.0  # exceeds max → poor match
            else:
                score += 1.0
        
        # --- G-load match (weight: 2) ---
        max_g = plat_data.get("max_g", 3.0)
        if isinstance(max_g, (int, float)):
            g_ratio = f["g_peak"] / (max_g + 1e-6)
            if g_ratio <= 1.0:
                score += 2.0 * (1.0 - abs(g_ratio - 0.5))
            elif g_ratio > 1.3:
                score -= 3.0  # exceeds structural limit
        
        # --- Turn rate match (weight: 2) ---
        max_omega = plat_data.get("max_turn_rate_radps", 0.2)
        if isinstance(max_omega, (int, float)):
            omega_ratio = f["omega_peak"] / (max_omega + 1e-6)
            if omega_ratio <= 1.0:
                score += 2.0 * (1.0 - abs(omega_ratio - 0.5))
            elif omega_ratio > 1.5:
                score -= 2.0
        
        # --- Altitude match (weight: 1.5) ---
        alt_str = plat_data.get("typical_altitude_m", "variable")
        alt_lo, alt_hi = self._parse_altitude_range(alt_str)
        if alt_lo <= f["alt_avg"] <= alt_hi:
            score += 1.5
        else:
            dist = min(abs(f["alt_avg"] - alt_lo), abs(f["alt_avg"] - alt_hi))
            score -= min(2.0, dist / 5000)
        
        # --- RCS match (weight: 1.5) — skip if ECM active ---
        if not ecm_active and f.get("rcs_mean", 0) != 0:
            rcs_lo, rcs_hi = self._parse_rcs_range(
                plat_data.get("average_rcs_m2", "1-10"))
            rcs_db_lo = 10 * math.log10(max(rcs_lo, 1e-6))
            rcs_db_hi = 10 * math.log10(max(rcs_hi, 1e-6))
            rcs_obs = f["rcs_mean"]
            if rcs_db_lo - 5 <= rcs_obs <= rcs_db_hi + 5:
                score += 1.5
            else:
                score -= 1.0
        
        # --- Behavior type bonus (weight: 1) ---
        behavior = plat_data.get("typical_behavior", "")
        if "cruise" in behavior and f["omega_avg"] < 0.05 and f["g_peak"] < 3:
            score += 1.0
        elif "maneuver" in behavior and (f["omega_peak"] > 0.2 or f["g_peak"] > 5):
            score += 1.0
        elif "attack" in behavior and f.get("alt_rate", 0) < -50:
            score += 1.0
        
        return score
    
    def classify(self) -> ClassificationResult:
        """Main classification pipeline.
        
        [REQ-CLS-007] Returns ClassificationResult with platform, confidence,
        threat level, and recommended IMM models.
        """
        features = self._extract_features()
        if not features:
            return ClassificationResult(
                platform_name="Unknown",
                platform_class="unknown",
                confidence=0.0,
                preferred_models=["CV", "CA", "CT+", "CT-"],
            )
        
        ecm_status = self.ecm_detector.status
        ecm_active = ecm_status != ECMStatus.CLEAN
        
        # Step 1: Coarse classification
        candidate_classes = self._coarse_classify(features)
        
        # Step 2: Gather all platforms in candidate classes
        candidates = []
        for cls in candidate_classes:
            candidates.extend(self.class_groups.get(cls, []))
        
        if not candidates:
            # Fallback: try all classes
            candidates = list(self.db.keys())
        
        # Step 3: Score each candidate
        scores = {}
        for plat_name in candidates:
            plat_data = self.db.get(plat_name, {})
            scores[plat_name] = self._score_platform(plat_data, features, ecm_active)
        
        if not scores:
            return ClassificationResult()
        
        # Step 4: Select best
        best_plat = max(scores, key=scores.get)
        best_score = scores[best_plat]
        
        # Confidence via softmax-like normalization
        all_scores = np.array(list(scores.values()))
        if len(all_scores) > 1:
            # Gap between best and second-best
            sorted_scores = np.sort(all_scores)[::-1]
            gap = sorted_scores[0] - sorted_scores[1] if len(sorted_scores) > 1 else 1.0
            confidence = min(1.0, max(0.0, 1.0 / (1.0 + math.exp(-gap))))
        else:
            confidence = 0.5
        
        # Step 5: Apply threshold
        plat_data = self.db.get(best_plat, {})
        plat_class = plat_data.get("class", "unknown")
        
        if confidence < self.conf_threshold:
            # Fallback to class defaults
            if ecm_active:
                plat_class = "unknown_future"
                best_plat = f"Unknown_ECM_{ecm_status.value}"
                preferred = ["CV", "CA", "CT+", "CT-", "Jerk", "Ballistic"]
            else:
                best_plat = f"Unknown_{plat_class}"
                preferred = plat_data.get("preferred_models", ["CV", "CA"])
        else:
            preferred = plat_data.get("preferred_models", ["CV"])
            # Expand "All models"
            if "All models" in preferred:
                preferred = ["CV", "CA", "CT+", "CT-", "Jerk", "Ballistic"]
        
        # Step 6: Threat level
        threat = self.THREAT_MAP.get(plat_class, ThreatLevel.MEDIUM)
        if ecm_active:
            # ECM presence → upgrade threat by 1 level
            threat = ThreatLevel(min(threat.value + 1, ThreatLevel.CRITICAL.value))
        
        # Step 7: Alerts
        alerts = []
        if ecm_active:
            alerts.append(f"ECM_DETECTED: {ecm_status.value}")
        if plat_data.get("plasma_effect", "no") != "no":
            alerts.append("PLASMA_EFFECT: RCS measurements unreliable")
        if plat_data.get("false_target_potential", "low") in ("high", "very_high"):
            alerts.append("FALSE_TARGET_WARNING: High clutter/false target probability")
        if plat_data.get("ecm_capability", "none") in ("high", "very_high"):
            alerts.append(f"PLATFORM_ECM: {best_plat} has {plat_data['ecm_capability']} ECM capability")
        
        # Classification persistence (prevent rapid oscillation)
        if best_plat == self._prev_result.platform_name:
            self._class_hold_counter += 1
        else:
            if self._class_hold_counter > 5 and confidence < 0.7:
                # Hold previous classification for stability
                best_plat = self._prev_result.platform_name
                plat_class = self._prev_result.platform_class
                confidence *= 0.9
            self._class_hold_counter = 0
        
        result = ClassificationResult(
            platform_name=best_plat,
            platform_class=plat_class,
            confidence=confidence,
            threat_level=threat,
            preferred_models=preferred,
            ecm_status=ecm_status,
            features_used=features,
            alerts=alerts,
        )
        
        self._prev_result = result
        return result


# =============================================================================
# [REQ-INT-001] Intent Predictor
# =============================================================================

class IntentPredictor:
    """Real-time behavioral intent prediction from kinematic features.
    
    Generates threat-relevant intent streams:
    - "terminal_dive_imminent" for ballistic/hypersonic threats
    - "sea_skimming" for anti-ship missiles
    - "evasion_break" for fighters
    - "pop_up" for terrain-following missiles
    - "orbit_racetrack" for ISR/loiter platforms
    
    [REQ-INT-002] Output feeds directly into operator display and 
    fire control priority queues.
    """
    
    # Intent rules by platform class
    # Each rule: (feature_test_fn, intent, confidence_weight)
    # Feature test functions take Dict[str, float] and return bool
    
    def __init__(self, dt: float = 0.1, history_len: int = 50):
        self.dt = dt
        self.history_len = history_len
        
        # Per-track state
        self.state = IntentState()
        
        # Smoothing
        self.intent_ema = {}  # intent_type → EMA confidence
        self.alpha = 0.15
        
        # History for pattern detection
        self.alt_history = deque(maxlen=history_len)
        self.spd_history = deque(maxlen=history_len)
        self.heading_history = deque(maxlen=history_len)
        self.g_history = deque(maxlen=history_len)
        self.vz_history = deque(maxlen=history_len)  # vertical velocity
    
    def update(self, speed_mps: float, altitude_m: float,
               heading_rad: float, g_load: float,
               vz_mps: float, omega_radps: float,
               platform_class: str = "unknown",
               platform_name: str = "Unknown") -> IntentState:
        """Update intent prediction with new track state.
        
        [REQ-INT-003] Called every tracker cycle.
        Returns IntentState with intent, threat level, and alerts.
        """
        self.alt_history.append(altitude_m)
        self.spd_history.append(speed_mps)
        self.heading_history.append(heading_rad)
        self.g_history.append(g_load)
        self.vz_history.append(vz_mps)
        
        n = len(self.alt_history)
        if n < 5:
            return self.state
        
        # === Derived features ===
        alt_arr = np.array(self.alt_history)
        spd_arr = np.array(self.spd_history)
        hdg_arr = np.array(self.heading_history)
        g_arr = np.array(self.g_history)
        vz_arr = np.array(self.vz_history)
        
        # Altitude rate (smoothed)
        if n >= 10:
            alt_rate = (alt_arr[-1] - alt_arr[-10]) / (10 * self.dt)
        else:
            alt_rate = vz_mps
        
        # Dive angle
        horiz_speed = max(speed_mps * math.cos(math.atan2(abs(vz_mps), max(speed_mps, 1))), 1.0)
        dive_angle_deg = math.degrees(math.atan2(-vz_mps, horiz_speed))
        
        # Time to impact (if descending)
        if vz_mps < -10 and altitude_m > 0:
            tti = altitude_m / (-vz_mps)
        else:
            tti = float('inf')
        
        # Heading variance (orbit detection)
        if n >= 20:
            dh = np.diff(hdg_arr[-20:])
            dh = np.arctan2(np.sin(dh), np.cos(dh))
            heading_var = float(np.var(dh))
            # Count sign changes → heading reversals
            sign_changes = np.sum(np.abs(np.diff(np.sign(dh))) > 0)
        else:
            heading_var = 0.0
            sign_changes = 0
        
        # Speed trend
        if n >= 10:
            spd_trend = (spd_arr[-1] - spd_arr[-10]) / (10 * self.dt)
        else:
            spd_trend = 0.0
        
        # === Intent scoring ===
        intent_scores = {}
        alerts = []
        
        # [REQ-INT-004] TERMINAL DIVE detection
        # Triggers: steep descent + high speed + specific platform classes
        terminal_classes = {"ballistic", "anti_ship_ballistic", "hypersonic_glide",
                           "hypersonic_cruise", "supersonic_cruise", "aam_bvr",
                           "aam_short_range", "sam_terminal"}
        if platform_class in terminal_classes:
            dive_score = 0.0
            if dive_angle_deg > 20:
                dive_score += 0.3
            if dive_angle_deg > 45:
                dive_score += 0.3
            if dive_angle_deg > 70:
                dive_score += 0.2
            if speed_mps > 500 and vz_mps < -100:
                dive_score += 0.2
            if tti < 30:
                dive_score += 0.3
                alerts.append(f"⚠️ TERMINAL_DIVE: TTI={tti:.1f}s, dive={dive_angle_deg:.0f}°")
            if tti < 10:
                alerts.append(f"🚨 IMPACT_IMMINENT: TTI={tti:.1f}s!")
            intent_scores[IntentType.TERMINAL_DIVE] = min(1.0, dive_score)
        
        # [REQ-INT-005] SEA SKIMMING detection
        sea_skim_classes = {"subsonic_cruise", "supersonic_cruise", "hypersonic_cruise",
                            "anti_ship_ballistic"}
        if platform_class in sea_skim_classes or altitude_m < 100:
            skim_score = 0.0
            if altitude_m < 50:
                skim_score += 0.4
            if altitude_m < 20:
                skim_score += 0.3
            if abs(alt_rate) < 5 and speed_mps > 200:
                skim_score += 0.3
            if skim_score > 0.3:
                alerts.append(f"⚠️ SEA_SKIMMING: alt={altitude_m:.0f}m, spd={speed_mps:.0f}m/s")
            intent_scores[IntentType.SEA_SKIMMING] = min(1.0, skim_score)
        
        # [REQ-INT-006] EVASION BREAK detection
        fighter_classes = {"4th_gen_fighter", "5th_gen_stealth", "6th_gen_concept"}
        if platform_class in fighter_classes:
            evasion_score = 0.0
            if g_load > 6:
                evasion_score += 0.3
            if abs(omega_radps) > 0.3:
                evasion_score += 0.3
            if n >= 5 and np.max(g_arr[-5:]) > 7:
                evasion_score += 0.2
            if spd_trend < -50:  # decelerating hard
                evasion_score += 0.2
            if evasion_score > 0.5:
                alerts.append(f"⚡ EVASION_BREAK: g={g_load:.1f}, ω={omega_radps:.2f}")
            intent_scores[IntentType.EVASION_BREAK] = min(1.0, evasion_score)
        
        # [REQ-INT-007] ATTACK RUN detection
        if platform_class in fighter_classes or platform_class in {"strategic_bomber", "uav_stealth"}:
            attack_score = 0.0
            if heading_var < 0.02 and speed_mps > 300:
                attack_score += 0.3  # steady heading + high speed
            if dive_angle_deg > 5 and dive_angle_deg < 30:
                attack_score += 0.3  # shallow dive
            if g_load < 3 and abs(omega_radps) < 0.1:
                attack_score += 0.2  # stable platform
            if attack_score > 0.4:
                alerts.append(f"🎯 ATTACK_RUN: hdg_var={heading_var:.3f}, dive={dive_angle_deg:.0f}°")
            intent_scores[IntentType.ATTACK_RUN] = min(1.0, attack_score)
        
        # [REQ-INT-008] POP-UP detection
        # Rapid altitude increase after terrain following
        if n >= 10:
            alt_10 = alt_arr[-10:]
            was_low = np.min(alt_10[:5]) < 200
            is_climbing = alt_rate > 50
            if was_low and is_climbing and speed_mps > 200:
                popup_score = min(1.0, alt_rate / 200)
                alerts.append(f"⬆️ POP_UP: alt_rate={alt_rate:.0f}m/s from low altitude")
                intent_scores[IntentType.POP_UP] = popup_score
        
        # [REQ-INT-009] ORBIT / RACETRACK detection
        loiter_classes = {"uav_low_g", "uav_stealth", "uav_micro", "helicopter_military",
                         "helicopter_civil", "general_aviation"}
        orbit_score = 0.0
        if heading_var > 0.1 and sign_changes >= 3:
            orbit_score += 0.3
        if abs(alt_rate) < 10 and abs(spd_trend) < 5:
            orbit_score += 0.2  # stable alt & speed
        if platform_class in loiter_classes:
            orbit_score += 0.2
        if sign_changes >= 4:
            orbit_score += 0.3
        intent_scores[IntentType.ORBIT_RACETRACK] = min(1.0, orbit_score)
        
        # [REQ-INT-010] BVR INTERCEPT detection
        if platform_class in {"aam_bvr", "aam_short_range", "sam_terminal"}:
            bvr_score = 0.0
            if speed_mps > 800:
                bvr_score += 0.3
            if g_load > 15:
                bvr_score += 0.3
            if abs(omega_radps) > 0.3:
                bvr_score += 0.2
            if spd_trend > 100:  # accelerating
                bvr_score += 0.2
            if bvr_score > 0.5:
                alerts.append(f"🚀 BVR_INTERCEPT: spd={speed_mps:.0f}, g={g_load:.1f}")
            intent_scores[IntentType.BVR_INTERCEPT] = min(1.0, bvr_score)
        
        # [REQ-INT-011] DOGFIGHT detection
        if platform_class in fighter_classes:
            df_score = 0.0
            if g_load > 5 and abs(omega_radps) > 0.3:
                df_score += 0.4
            if n >= 10 and np.mean(g_arr[-10:]) > 4:
                df_score += 0.3
            if sign_changes >= 3:
                df_score += 0.3
            intent_scores[IntentType.DOGFIGHT] = min(1.0, df_score)
        
        # [REQ-INT-012] TERRAIN FOLLOWING detection
        if n >= 20 and np.mean(alt_arr[-20:]) < 200:
            tf_score = 0.0
            if np.std(alt_arr[-20:]) > 10 and np.std(alt_arr[-20:]) < 100:
                tf_score += 0.4  # varying altitude but bounded
            if speed_mps > 200:
                tf_score += 0.3
            if heading_var < 0.05:
                tf_score += 0.3
            intent_scores[IntentType.TERRAIN_FOLLOWING] = min(1.0, tf_score)
        
        # [REQ-INT-013] SKIP-GLIDE detection
        if platform_class in {"hypersonic_glide"} and n >= 20:
            # Look for oscillating altitude at high speed
            alt_osc = np.std(alt_arr[-20:])
            if alt_osc > 5000 and speed_mps > 2000:
                skip_score = min(1.0, alt_osc / 20000 + speed_mps / 10000)
                alerts.append(f"〰️ SKIP_GLIDE: alt_osc={alt_osc:.0f}m, spd={speed_mps:.0f}")
                intent_scores[IntentType.SKIP_GLIDE] = skip_score
        
        # [REQ-INT-014] JAMMER STANDOFF detection
        if platform_class in {"4th_gen_fighter"} and sign_changes >= 2:
            # Orbiting at distance → possible EW platform
            if orbit_score > 0.4 and altitude_m > 5000:
                intent_scores[IntentType.JAMMER_STANDOFF] = orbit_score * 0.8
                alerts.append(f"📡 JAMMER_STANDOFF: orbiting at {altitude_m:.0f}m")
        
        # [REQ-INT-015] REENTRY detection
        if platform_class in {"ballistic", "hypersonic_glide", "space_object"}:
            if speed_mps > 3000 and vz_mps < -500:
                reentry_score = min(1.0, speed_mps / 7000 + abs(vz_mps) / 3000)
                alerts.append(f"☄️ REENTRY: spd={speed_mps:.0f}, vz={vz_mps:.0f}")
                intent_scores[IntentType.REENTRY] = reentry_score
        
        # [REQ-INT-016] FALSE TARGET assessment
        false_classes = {"false_target_bird", "false_target_balloon",
                        "false_target_ground", "false_target_environment"}
        if platform_class in false_classes:
            intent_scores[IntentType.FALSE_TARGET] = 0.9
        
        # [REQ-INT-017] CRUISE (default)
        if not intent_scores or max(intent_scores.values()) < 0.3:
            cruise_score = 0.5
            if heading_var < 0.01 and abs(alt_rate) < 5 and g_load < 2:
                cruise_score = 0.8
            intent_scores[IntentType.CRUISE] = cruise_score
        
        # === Select best intent with EMA smoothing ===
        for itype, score in intent_scores.items():
            prev = self.intent_ema.get(itype, 0.0)
            self.intent_ema[itype] = self.alpha * score + (1 - self.alpha) * prev
        
        best_intent = max(self.intent_ema, key=self.intent_ema.get)
        best_conf = self.intent_ema[best_intent]
        
        # === Threat level from intent ===
        threat = ThreatLevel.NONE
        if best_intent in {IntentType.TERMINAL_DIVE, IntentType.REENTRY}:
            if tti < 15:
                threat = ThreatLevel.CRITICAL
            elif tti < 30:
                threat = ThreatLevel.HIGH
            else:
                threat = ThreatLevel.MEDIUM
        elif best_intent in {IntentType.SEA_SKIMMING, IntentType.BVR_INTERCEPT,
                             IntentType.ATTACK_RUN, IntentType.POP_UP}:
            threat = ThreatLevel.HIGH
        elif best_intent in {IntentType.EVASION_BREAK, IntentType.DOGFIGHT,
                             IntentType.SKIP_GLIDE}:
            threat = ThreatLevel.HIGH
        elif best_intent in {IntentType.JAMMER_STANDOFF}:
            threat = ThreatLevel.MEDIUM
        elif best_intent in {IntentType.ORBIT_RACETRACK, IntentType.TERRAIN_FOLLOWING}:
            threat = ThreatLevel.MEDIUM
        elif best_intent in {IntentType.CRUISE}:
            threat = ThreatLevel.LOW
        
        # Update state
        self.state = IntentState(
            intent=best_intent,
            intent_confidence=best_conf,
            threat_level=threat,
            dive_angle_deg=dive_angle_deg,
            altitude_rate_mps=alt_rate,
            time_to_impact_s=tti,
            closing_speed_mps=speed_mps,  # TODO: compute vs. own position
            orbit_count=sign_changes // 2,
            heading_reversal_count=sign_changes,
            alerts=alerts,
        )
        
        return self.state


# =============================================================================
# [REQ-INT-018] Integrated Classification + Intent Pipeline
# =============================================================================

class NxMimosaClassifierPipeline:
    """Integrated classifier + intent predictor pipeline.
    
    Single update() call feeds both classifier and intent predictor,
    returning a unified ClassificationResult with intent.
    
    [REQ-INT-019] Designed for integration into NxMimosaV40Sentinel.
    """
    
    def __init__(self, db_path: str = "data/platform_db_v3.json",
                 dt: float = 0.1,
                 history_len: int = 30,
                 conf_threshold: float = 0.60):
        self.ecm_detector = ECMDetector()
        self.classifier = ImprovedPlatformClassifier(
            db_path=db_path,
            history_len=history_len,
            conf_threshold=conf_threshold,
            ecm_detector=self.ecm_detector,
        )
        self.intent_predictor = IntentPredictor(dt=dt, history_len=history_len * 2)
        self.dt = dt
        self._n_updates = 0
    
    def update(self, speed_mps: float, accel_g: float,
               altitude_m: float, omega_radps: float,
               heading_rad: float, vz_mps: float = 0.0,
               nis_cv: float = 1.0, rcs_dbsm: float = 0.0,
               snr_db: float = 25.0, doppler_hz: float = 0.0,
               range_m: float = 0.0) -> ClassificationResult:
        """Unified update for classification + intent prediction.
        
        [REQ-INT-020] Returns ClassificationResult with intent fields populated.
        """
        # Update classifier features
        self.classifier.update_features(
            speed_mps=speed_mps, accel_g=accel_g,
            altitude_m=altitude_m, omega_radps=omega_radps,
            heading_rad=heading_rad, nis_cv=nis_cv,
            rcs_dbsm=rcs_dbsm, snr_db=snr_db,
            doppler_hz=doppler_hz, range_m=range_m,
        )
        
        self._n_updates += 1
        
        # Classify every 5 updates (0.5s at dt=0.1)
        if self._n_updates % 5 == 0 or self._n_updates <= 10:
            result = self.classifier.classify()
        else:
            result = self.classifier._prev_result
        
        # Intent prediction every update
        intent_state = self.intent_predictor.update(
            speed_mps=speed_mps, altitude_m=altitude_m,
            heading_rad=heading_rad, g_load=accel_g,
            vz_mps=vz_mps, omega_radps=omega_radps,
            platform_class=result.platform_class,
            platform_name=result.platform_name,
        )
        
        # Merge intent into classification result
        result.intent = intent_state.intent
        result.intent_confidence = intent_state.intent_confidence
        
        # Upgrade threat level if intent suggests higher threat
        if intent_state.threat_level.value > result.threat_level.value:
            result.threat_level = intent_state.threat_level
        
        # Merge alerts
        result.alerts = list(set(result.alerts + intent_state.alerts))
        
        return result


# =============================================================================
# [REQ-TST-001] Self-test / demo
# =============================================================================

def demo_intent_scenarios():
    """Demonstrate intent prediction on simulated scenarios."""
    
    print("=" * 70)
    print("NX-MIMOSA v4.1 Intent Prediction Demo")
    print("=" * 70)
    
    dt = 0.1
    
    scenarios = [
        # (name, platform_class, trajectory_fn)
        ("Iskander Terminal Dive", "ballistic", 
         lambda t: (2000 - t*20, 50000 - t*1500, 0.0, 2.0, -1500 + t*10, 0.0)),
        ("Tomahawk Sea Skimming", "subsonic_cruise",
         lambda t: (240, 15, 0.1, 1.2, -0.5, 0.0)),
        ("F-16 Evasion Break", "4th_gen_fighter",
         lambda t: (500 - t*5, 8000, 0.5 + t*0.1, 7.0 + t*0.3, 0, 0.35)),
        ("MQ-9 Racetrack Orbit", "uav_low_g",
         lambda t: (100, 6000, math.sin(t*0.2)*0.5, 1.5, 0, 0.15*math.sin(t*0.3))),
        ("Kinzhal Skip-Glide", "hypersonic_glide",
         lambda t: (3400, 30000 + 10000*math.sin(t*0.1), 0.0, 3.0, 1000*math.cos(t*0.1), 0.05)),
        ("Boeing 737 Cruise", "commercial_airliner",
         lambda t: (250, 11000, 0.2, 1.0, 0, 0.001)),
    ]
    
    for name, plat_class, traj_fn in scenarios:
        print(f"\n--- Scenario: {name} ---")
        predictor = IntentPredictor(dt=dt)
        
        for step in range(50):
            t = step * dt
            spd, alt, hdg, g, vz, omega = traj_fn(t)
            state = predictor.update(
                speed_mps=spd, altitude_m=alt, heading_rad=hdg,
                g_load=g, vz_mps=vz, omega_radps=omega,
                platform_class=plat_class, platform_name=name,
            )
        
        # Print final state
        print(f"  Intent: {state.intent.value}")
        print(f"  Confidence: {state.intent_confidence:.2f}")
        print(f"  Threat: {state.threat_level.name}")
        print(f"  Dive angle: {state.dive_angle_deg:.1f}°")
        print(f"  Alt rate: {state.altitude_rate_mps:.0f} m/s")
        print(f"  TTI: {state.time_to_impact_s:.1f}s")
        if state.alerts:
            for a in state.alerts:
                print(f"  ALERT: {a}")
    
    print("\n" + "=" * 70)
    print("Demo complete.")


if __name__ == "__main__":
    demo_intent_scenarios()
