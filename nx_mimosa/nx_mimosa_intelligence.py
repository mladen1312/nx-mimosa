"""NX-MIMOSA v5.3 Intelligence Module — Platform ID, ECM Detection, Intent Prediction.

Ported from v4.x with embedded platform database (no external files needed).
Pure NumPy — FPGA-friendly, no ML dependencies.

Reference:
    - Platform classification: Kinematics-only, hierarchical speed→class→fine
    - ECM detection: SNR/RCS/Doppler/NIS anomaly detection
    - Intent prediction: State-machine + kinematic pattern matching

Author: Dr. Mladen Mešter — Nexellum d.o.o.
"""

from __future__ import annotations
import numpy as np
from enum import Enum
from dataclasses import dataclass, field
from collections import deque
from typing import Dict, List, Optional, Tuple


# ===== ENUMS =====

class ThreatLevel(Enum):
    """[REQ-V53-INT-01] Threat assessment levels."""
    NONE = 0
    LOW = 1
    MEDIUM = 2
    HIGH = 3
    CRITICAL = 4


class ECMStatus(Enum):
    """[REQ-V53-ECM-01] Electronic countermeasure status."""
    CLEAN = "clean"
    NOISE_JAMMING = "noise_jamming"
    DECEPTION = "deception"
    DRFM = "drfm"
    CHAFF = "chaff"


class IntentType(Enum):
    """[REQ-V53-INT-02] Target intent classification."""
    UNKNOWN = "unknown"
    TRANSIT = "transit"
    PATROL = "patrol"
    ORBIT = "orbit"
    INGRESS = "ingress"
    EGRESS = "egress"
    ATTACK_RUN = "attack_run"
    EVASION = "evasion"
    TERRAIN_FOLLOWING = "terrain_following"
    AERIAL_REFUEL = "aerial_refuel"
    FORMATION = "formation"
    SAM_LAUNCH = "sam_launch"
    MISSILE_TERMINAL = "missile_terminal"
    BALLISTIC_MIDCOURSE = "ballistic_midcourse"
    REENTRY = "reentry"
    LANDING = "landing"


# ===== DATA CLASSES =====

@dataclass
class PlatformClass:
    """[REQ-V53-PID-01] Platform classification result."""
    platform_type: str          # e.g., "4th_gen_fighter", "commercial_airliner"
    coarse_class: str           # e.g., "fighter", "airliner", "missile"
    threat_level: ThreatLevel
    confidence: float           # 0.0–1.0
    speed_class: str            # e.g., "subsonic_high", "supersonic"
    alt_candidates: List[Tuple[str, float]]  # Top-3 alternatives


@dataclass
class ECMReport:
    """[REQ-V53-ECM-02] ECM detection report."""
    status: ECMStatus
    confidence: float
    q_scale_factor: float       # Recommended process noise multiplier
    indicators: Dict[str, float]  # Individual indicator scores


@dataclass
class IntentReport:
    """[REQ-V53-INT-03] Intent prediction report."""
    primary_intent: IntentType
    confidence: float
    threat_level: ThreatLevel
    threat_score: float          # 0.0–1.0 composite threat
    secondary_intent: Optional[IntentType] = None
    time_to_threat: Optional[float] = None  # seconds, if applicable


@dataclass
class IntelligenceReport:
    """[REQ-V53-INT-04] Combined intelligence assessment for one track."""
    track_id: int
    platform: PlatformClass
    ecm: ECMReport
    intent: IntentReport
    composite_threat: float      # 0.0–1.0 overall threat score


# ===== EMBEDDED PLATFORM DATABASE =====

# 31 platform types organized by speed class
# Each: (type_name, coarse_class, speed_range_ms, alt_range_m, g_limit, threat)
PLATFORM_DB: List[Tuple[str, str, Tuple[float, float], Tuple[float, float], float, ThreatLevel]] = [
    # False targets
    ("bird", "false_target", (0, 30), (0, 3000), 0.5, ThreatLevel.NONE),
    ("balloon", "false_target", (0, 15), (0, 35000), 0.1, ThreatLevel.NONE),
    # Civil aviation
    ("general_aviation", "civil", (30, 130), (0, 8000), 2.0, ThreatLevel.NONE),
    ("helicopter_civil", "civil", (0, 80), (0, 5000), 1.5, ThreatLevel.NONE),
    ("commercial_airliner", "civil", (200, 280), (8000, 13000), 2.5, ThreatLevel.NONE),
    ("business_jet", "civil", (180, 280), (8000, 15000), 3.0, ThreatLevel.NONE),
    # UAVs
    ("uav_micro", "uav", (5, 30), (0, 500), 2.0, ThreatLevel.MEDIUM),
    ("uav_tactical", "uav", (20, 70), (0, 5000), 3.0, ThreatLevel.MEDIUM),
    ("uav_male", "uav", (30, 100), (3000, 15000), 2.5, ThreatLevel.MEDIUM),
    ("uav_hale", "uav", (50, 120), (15000, 25000), 2.0, ThreatLevel.HIGH),
    ("uav_ucav", "uav", (150, 300), (0, 15000), 6.0, ThreatLevel.HIGH),
    # Military aircraft
    ("helicopter_attack", "military", (0, 90), (0, 5000), 3.0, ThreatLevel.HIGH),
    ("cargo_military", "military", (100, 250), (3000, 12000), 2.5, ThreatLevel.LOW),
    ("4th_gen_fighter", "fighter", (150, 700), (0, 18000), 9.0, ThreatLevel.HIGH),
    ("5th_gen_stealth", "fighter", (150, 600), (0, 20000), 9.0, ThreatLevel.HIGH),
    ("strategic_bomber", "bomber", (200, 320), (5000, 15000), 3.0, ThreatLevel.HIGH),
    # Cruise missiles
    ("subsonic_cruise", "cruise_missile", (200, 320), (10, 500), 5.0, ThreatLevel.HIGH),
    ("supersonic_cruise", "cruise_missile", (400, 1000), (10, 20000), 15.0, ThreatLevel.CRITICAL),
    ("hypersonic_cruise", "cruise_missile", (1500, 3000), (20000, 50000), 5.0, ThreatLevel.CRITICAL),
    # Ballistic missiles
    ("srbm", "ballistic", (500, 2000), (0, 100000), 3.0, ThreatLevel.CRITICAL),
    ("mrbm", "ballistic", (1000, 4000), (0, 300000), 3.0, ThreatLevel.CRITICAL),
    ("icbm", "ballistic", (3000, 7000), (0, 1200000), 3.0, ThreatLevel.CRITICAL),
    # Air-to-air missiles
    ("aam_ir", "aam", (300, 1200), (0, 20000), 40.0, ThreatLevel.CRITICAL),
    ("aam_radar", "aam", (400, 1500), (0, 25000), 35.0, ThreatLevel.CRITICAL),
    # SAMs
    ("sam_manpad", "sam", (200, 700), (0, 5000), 20.0, ThreatLevel.CRITICAL),
    ("sam_short", "sam", (300, 1200), (0, 15000), 30.0, ThreatLevel.CRITICAL),
    ("sam_medium", "sam", (500, 2000), (0, 30000), 25.0, ThreatLevel.CRITICAL),
    ("sam_long", "sam", (800, 3000), (0, 40000), 20.0, ThreatLevel.CRITICAL),
    # Space
    ("satellite_leo", "space", (7000, 8000), (200000, 2000000), 0.1, ThreatLevel.NONE),
    ("reentry_vehicle", "space", (2000, 7000), (0, 400000), 10.0, ThreatLevel.CRITICAL),
    # Unknown
    ("unknown", "unknown", (0, 10000), (0, 1200000), 50.0, ThreatLevel.MEDIUM),
]


# ===== PLATFORM CLASSIFIER =====

class PlatformClassifier:
    """[REQ-V53-PID-02] Kinematics-only platform classification.
    
    Classifies targets by matching observed speed, altitude, and g-loading
    against the embedded platform database. No external files needed.
    Pure NumPy — suitable for FPGA HLS path.
    
    Args:
        history_len: Number of kinematic samples to buffer for statistics.
        confidence_threshold: Minimum confidence to report a classification.
    """
    
    def __init__(self, history_len: int = 30, confidence_threshold: float = 0.5):
        self.history_len = history_len
        self.conf_threshold = confidence_threshold
        self._speed_hist: deque = deque(maxlen=history_len)
        self._alt_hist: deque = deque(maxlen=history_len)
        self._g_hist: deque = deque(maxlen=history_len)
    
    def update(self, speed_ms: float, altitude_m: float,
               g_load: float = 1.0) -> PlatformClass:
        """[REQ-V53-PID-03] Classify from single kinematic observation.
        
        Args:
            speed_ms: Ground speed in m/s
            altitude_m: Altitude in meters (MSL)
            g_load: Normal acceleration in g's (1g = 9.81 m/s²)
        
        Returns:
            PlatformClass with type, threat level, and confidence.
        """
        self._speed_hist.append(speed_ms)
        self._alt_hist.append(altitude_m)
        self._g_hist.append(g_load)
        
        # Use median for robustness
        spd = float(np.median(list(self._speed_hist)))
        alt = float(np.median(list(self._alt_hist)))
        g_max = float(np.max(list(self._g_hist)))
        
        # Score each platform type
        scores: List[Tuple[str, float, int]] = []
        
        for idx, (name, coarse, spd_range, alt_range, g_lim, threat) in enumerate(PLATFORM_DB):
            s_lo, s_hi = spd_range
            a_lo, a_hi = alt_range
            
            # Speed score (Gaussian-like)
            s_mid = (s_lo + s_hi) / 2
            s_sigma = max((s_hi - s_lo) / 4, 1.0)
            spd_score = np.exp(-0.5 * ((spd - s_mid) / s_sigma) ** 2)
            
            # Altitude score
            a_mid = (a_lo + a_hi) / 2
            a_sigma = max((a_hi - a_lo) / 4, 1.0)
            alt_score = np.exp(-0.5 * ((alt - a_mid) / a_sigma) ** 2)
            
            # G-loading compatibility (penalty if exceeds platform limit)
            if g_max <= g_lim * 1.2:
                g_score = 1.0
            else:
                g_score = np.exp(-0.5 * ((g_max - g_lim) / max(g_lim * 0.3, 0.5)) ** 2)
            
            total = spd_score * 0.45 + alt_score * 0.35 + g_score * 0.20
            scores.append((name, total, idx))
        
        # Sort by score, take top
        scores.sort(key=lambda x: x[1], reverse=True)
        
        best_name, best_score, best_idx = scores[0]
        _, coarse, _, _, _, threat = PLATFORM_DB[best_idx]
        
        # Speed class
        speed_class = self._speed_class(spd)
        
        # Top-3 alternatives
        alts = [(n, float(s)) for n, s, _ in scores[1:4]]
        
        # Confidence scales with history
        n = len(self._speed_hist)
        hist_factor = min(n / self.history_len, 1.0)
        confidence = float(best_score * hist_factor)
        
        return PlatformClass(
            platform_type=best_name,
            coarse_class=coarse,
            threat_level=threat,
            confidence=confidence,
            speed_class=speed_class,
            alt_candidates=alts,
        )
    
    @staticmethod
    def _speed_class(speed_ms: float) -> str:
        if speed_ms < 5: return "stationary"
        if speed_ms < 30: return "very_slow"
        if speed_ms < 80: return "slow"
        if speed_ms < 200: return "subsonic_low"
        if speed_ms < 340: return "subsonic_high"
        if speed_ms < 900: return "transonic"
        if speed_ms < 2000: return "supersonic"
        return "hypersonic"
    
    def reset(self) -> None:
        self._speed_hist.clear()
        self._alt_hist.clear()
        self._g_hist.clear()


# ===== ECM DETECTOR =====

class ECMDetector:
    """[REQ-V53-ECM-03] Real-time ECM detection from radar observables.
    
    Monitors SNR, RCS variance, Doppler spread, and NIS for anomalies
    that indicate electronic countermeasures.
    
    Detection modes:
    - Noise jamming: SNR drop + elevated NIS
    - Deception (RGPO/VGPO): Range/velocity anomalies
    - DRFM: RCS stability anomaly + coherent Doppler
    - Chaff: RCS spike + rapid Doppler spread
    
    Args:
        history_len: Observation buffer length.
        snr_threshold_db: SNR drop threshold for noise jamming.
    """
    
    def __init__(self, history_len: int = 30, snr_threshold_db: float = 10.0):
        self.history_len = history_len
        self.snr_threshold = snr_threshold_db
        
        self._snr = deque(maxlen=history_len)
        self._rcs = deque(maxlen=history_len)
        self._doppler = deque(maxlen=history_len)
        self._nis = deque(maxlen=history_len)
        
        self._baseline_snr = 25.0
        self._baseline_set = False
    
    def update(self, snr_db: float = 25.0, rcs: float = 1.0,
               doppler_hz: float = 0.0, nis: float = 1.0) -> ECMReport:
        """[REQ-V53-ECM-04] Update ECM assessment with new observation.
        
        Args:
            snr_db: Signal-to-noise ratio in dB
            rcs: Radar cross section (normalized)
            doppler_hz: Doppler frequency in Hz
            nis: Normalized Innovation Squared from tracker
        
        Returns:
            ECMReport with status, confidence, and Q scale recommendation.
        """
        self._snr.append(snr_db)
        self._rcs.append(rcs)
        self._doppler.append(doppler_hz)
        self._nis.append(nis)
        
        if len(self._snr) < 5:
            return ECMReport(ECMStatus.CLEAN, 0.0, 1.0, {})
        
        # Establish baseline
        if not self._baseline_set and len(self._snr) >= 15:
            arr = np.array(list(self._snr)[:15])
            if np.std(arr) < 5.0:
                self._baseline_snr = float(np.mean(arr))
                self._baseline_set = True
        
        snr_arr = np.array(list(self._snr))
        rcs_arr = np.array(list(self._rcs))
        dop_arr = np.array(list(self._doppler))
        nis_arr = np.array(list(self._nis))
        
        # Indicator scores
        snr_drop = max(0, self._baseline_snr - np.mean(snr_arr[-10:]))
        rcs_var = float(np.std(rcs_arr[-10:])) if len(rcs_arr) >= 10 else 0.0
        dop_spread = float(np.std(dop_arr[-10:])) if len(dop_arr) >= 10 else 0.0
        nis_mean = float(np.mean(nis_arr[-10:])) if len(nis_arr) >= 10 else 1.0
        
        indicators = {
            "snr_drop_db": snr_drop,
            "rcs_variance": rcs_var,
            "doppler_spread_hz": dop_spread,
            "nis_mean": nis_mean,
        }
        
        # Decision logic
        status = ECMStatus.CLEAN
        confidence = 0.0
        q_scale = 1.0
        
        # Noise jamming: large SNR drop
        if snr_drop > self.snr_threshold:
            noise_conf = min(snr_drop / (self.snr_threshold * 2), 1.0)
            if noise_conf > confidence:
                status = ECMStatus.NOISE_JAMMING
                confidence = noise_conf
                q_scale = 3.0
        
        # Chaff: RCS spike + Doppler spread
        if rcs_var > 3.0 and dop_spread > 50.0:
            chaff_conf = min((rcs_var / 6.0 + dop_spread / 200.0) / 2, 1.0)
            if chaff_conf > confidence:
                status = ECMStatus.CHAFF
                confidence = chaff_conf
                q_scale = 2.0
        
        # DRFM: Very stable RCS + elevated NIS (coherent repeater)
        if rcs_var < 0.1 and nis_mean > 5.0 and len(rcs_arr) >= 15:
            drfm_conf = min(nis_mean / 15.0, 1.0)
            if drfm_conf > confidence:
                status = ECMStatus.DRFM
                confidence = drfm_conf
                q_scale = 4.0
        
        # Deception: NIS spikes without SNR drop
        if nis_mean > 10.0 and snr_drop < 3.0:
            deception_conf = min(nis_mean / 30.0, 1.0)
            if deception_conf > confidence:
                status = ECMStatus.DECEPTION
                confidence = deception_conf
                q_scale = 5.0
        
        return ECMReport(status, float(confidence), q_scale, indicators)
    
    def reset(self) -> None:
        self._snr.clear()
        self._rcs.clear()
        self._doppler.clear()
        self._nis.clear()
        self._baseline_set = False


# ===== INTENT PREDICTOR =====

class IntentPredictor:
    """[REQ-V53-INT-05] Kinematic-based intent prediction.
    
    Infers target behavior from speed/altitude/heading/g-loading patterns.
    State-machine approach with kinematic feature extraction.
    
    Args:
        history_len: Number of samples for pattern detection.
    """
    
    def __init__(self, history_len: int = 30):
        self.history_len = history_len
        self._speed: deque = deque(maxlen=history_len)
        self._alt: deque = deque(maxlen=history_len)
        self._heading: deque = deque(maxlen=history_len)
        self._g: deque = deque(maxlen=history_len)
        self._range_to_ref: deque = deque(maxlen=history_len)  # Range to defended asset
    
    def update(self, speed_ms: float, altitude_m: float,
               heading_rad: float = 0.0, g_load: float = 1.0,
               range_to_asset_m: float = 1e6,
               platform: Optional[PlatformClass] = None) -> IntentReport:
        """[REQ-V53-INT-06] Predict intent from current kinematics.
        
        Args:
            speed_ms: Ground speed in m/s
            altitude_m: Altitude MSL in m
            heading_rad: Heading in radians
            g_load: Normal acceleration in g's
            range_to_asset_m: Range to nearest defended asset
            platform: Platform classification (if available)
        
        Returns:
            IntentReport with predicted intent, threat level, and scores.
        """
        self._speed.append(speed_ms)
        self._alt.append(altitude_m)
        self._heading.append(heading_rad)
        self._g.append(g_load)
        self._range_to_ref.append(range_to_asset_m)
        
        n = len(self._speed)
        if n < 3:
            return IntentReport(IntentType.UNKNOWN, 0.0, ThreatLevel.LOW, 0.0)
        
        # Feature extraction
        spd_arr = np.array(list(self._speed))
        alt_arr = np.array(list(self._alt))
        hdg_arr = np.array(list(self._heading))
        g_arr = np.array(list(self._g))
        rng_arr = np.array(list(self._range_to_ref))
        
        spd_trend = spd_arr[-1] - spd_arr[0] if n > 1 else 0
        alt_trend = alt_arr[-1] - alt_arr[0] if n > 1 else 0
        rng_trend = rng_arr[-1] - rng_arr[0] if n > 1 else 0
        g_max = float(np.max(g_arr))
        hdg_var = float(np.std(hdg_arr)) if n > 3 else 0
        current_range = rng_arr[-1]
        current_alt = alt_arr[-1]
        current_speed = spd_arr[-1]
        
        # Intent scoring
        scores: Dict[IntentType, float] = {}
        
        # Transit: Steady speed, steady heading, no range closure
        scores[IntentType.TRANSIT] = 0.3
        if hdg_var < 0.05 and abs(spd_trend) < 20 and g_max < 2.0:
            scores[IntentType.TRANSIT] += 0.5
        
        # Patrol: Moderate heading variance (search pattern)
        scores[IntentType.PATROL] = 0.1
        if 0.05 < hdg_var < 0.5 and g_max < 3.0 and abs(alt_trend) < 500:
            scores[IntentType.PATROL] += 0.5
        
        # Orbit: High heading variance (circling)
        scores[IntentType.ORBIT] = 0.1
        if hdg_var > 0.3 and abs(spd_trend) < 20:
            scores[IntentType.ORBIT] += 0.5
        
        # Ingress: Closing range, stable heading toward asset
        scores[IntentType.INGRESS] = 0.1
        if rng_trend < -1000 and current_range < 100000:
            closure_factor = min(abs(rng_trend) / 50000, 1.0)
            scores[IntentType.INGRESS] += 0.5 * closure_factor
        
        # Attack run: Fast closure, low altitude, high g's
        scores[IntentType.ATTACK_RUN] = 0.0
        if rng_trend < -5000 and current_range < 50000 and g_max > 3.0:
            scores[IntentType.ATTACK_RUN] += 0.7
        if current_alt < 1000 and current_speed > 200:
            scores[IntentType.ATTACK_RUN] += 0.2
        
        # Evasion: High g, erratic heading
        scores[IntentType.EVASION] = 0.0
        if g_max > 5.0 and hdg_var > 0.2:
            scores[IntentType.EVASION] += 0.6
        if rng_trend > 5000:  # Running away
            scores[IntentType.EVASION] += 0.2
        
        # Terrain following: Low alt, moderate speed
        scores[IntentType.TERRAIN_FOLLOWING] = 0.0
        if current_alt < 300 and 100 < current_speed < 400:
            scores[IntentType.TERRAIN_FOLLOWING] += 0.5
            if np.std(alt_arr[-min(10, n):]) < 50:
                scores[IntentType.TERRAIN_FOLLOWING] += 0.3
        
        # Missile terminal: Very fast, very high g, closing fast
        scores[IntentType.MISSILE_TERMINAL] = 0.0
        if current_speed > 500 and g_max > 10 and rng_trend < -10000:
            scores[IntentType.MISSILE_TERMINAL] += 0.8
        
        # SAM launch: Rapid altitude gain + high acceleration from ground
        scores[IntentType.SAM_LAUNCH] = 0.0
        if alt_trend > 5000 and spd_trend > 200 and alt_arr[0] < 500:
            scores[IntentType.SAM_LAUNCH] += 0.7
        
        # Egress: Increasing range, after being close
        scores[IntentType.EGRESS] = 0.0
        if rng_trend > 2000 and current_range < 200000 and min(rng_arr) < current_range * 0.7:
            scores[IntentType.EGRESS] += 0.5
        
        # Landing: Descending, decelerating
        scores[IntentType.LANDING] = 0.0
        if alt_trend < -500 and spd_trend < -20 and current_alt < 3000:
            scores[IntentType.LANDING] += 0.6
        
        # Ballistic midcourse: Very high alt, high speed, minimal g
        scores[IntentType.BALLISTIC_MIDCOURSE] = 0.0
        if current_alt > 80000 and current_speed > 2000 and g_max < 2.0:
            scores[IntentType.BALLISTIC_MIDCOURSE] += 0.7
        
        # Reentry: Descending from high alt, very fast
        scores[IntentType.REENTRY] = 0.0
        if alt_trend < -10000 and current_speed > 2000 and alt_arr[0] > 50000:
            scores[IntentType.REENTRY] += 0.8
        
        # Select primary & secondary
        sorted_intents = sorted(scores.items(), key=lambda x: x[1], reverse=True)
        primary, primary_score = sorted_intents[0]
        secondary = sorted_intents[1][0] if len(sorted_intents) > 1 else None
        
        # Threat scoring
        threat_base = 0.0
        if primary in (IntentType.ATTACK_RUN, IntentType.MISSILE_TERMINAL,
                       IntentType.SAM_LAUNCH, IntentType.REENTRY):
            threat_base = 0.8
        elif primary in (IntentType.INGRESS, IntentType.TERRAIN_FOLLOWING):
            threat_base = 0.5
        elif primary in (IntentType.EVASION,):
            threat_base = 0.4
        elif primary in (IntentType.TRANSIT, IntentType.PATROL, IntentType.ORBIT):
            threat_base = 0.1
        
        # Range factor: closer = more threatening
        if current_range < 20000:
            range_factor = 1.0
        elif current_range < 100000:
            range_factor = 0.6
        else:
            range_factor = 0.2
        
        # Platform threat boost
        plat_factor = 0.0
        if platform and platform.threat_level.value >= ThreatLevel.HIGH.value:
            plat_factor = 0.2
        
        threat_score = min(threat_base * 0.5 + range_factor * 0.3 + plat_factor + primary_score * 0.2, 1.0)
        
        # Map to ThreatLevel
        if threat_score > 0.75:
            threat = ThreatLevel.CRITICAL
        elif threat_score > 0.5:
            threat = ThreatLevel.HIGH
        elif threat_score > 0.25:
            threat = ThreatLevel.MEDIUM
        elif threat_score > 0.1:
            threat = ThreatLevel.LOW
        else:
            threat = ThreatLevel.NONE
        
        # Time to threat (simple range/closure rate)
        ttt = None
        if rng_trend < -100:
            closure_rate = abs(rng_trend) / max(n * 1.0, 1.0)  # m/sample
            if closure_rate > 0:
                ttt = current_range / closure_rate
        
        confidence = float(min(primary_score * min(n / self.history_len, 1.0), 1.0))
        
        return IntentReport(
            primary_intent=primary,
            confidence=confidence,
            threat_level=threat,
            threat_score=float(threat_score),
            secondary_intent=secondary,
            time_to_threat=ttt,
        )
    
    def reset(self) -> None:
        self._speed.clear()
        self._alt.clear()
        self._heading.clear()
        self._g.clear()
        self._range_to_ref.clear()


# ===== INTELLIGENCE PIPELINE =====

class IntelligencePipeline:
    """[REQ-V53-INT-07] Combined intelligence pipeline for per-track assessment.
    
    Manages platform classifier, ECM detector, and intent predictor per track.
    Call `assess()` each scan with track kinematics and radar observables.
    
    Usage::
    
        intel = IntelligencePipeline()
        report = intel.assess(
            track_id=1,
            speed_ms=250, altitude_m=5000, heading_rad=0.5, g_load=2.0,
            range_to_asset_m=50000,
            snr_db=20, rcs=1.0, doppler_hz=500, nis=1.5,
        )
        print(report.platform.platform_type)  # "4th_gen_fighter"
        print(report.intent.primary_intent)   # IntentType.INGRESS
        print(report.composite_threat)        # 0.72
    """
    
    def __init__(self):
        self._classifiers: Dict[int, PlatformClassifier] = {}
        self._ecm_detectors: Dict[int, ECMDetector] = {}
        self._intent_predictors: Dict[int, IntentPredictor] = {}
    
    def assess(self, track_id: int,
               speed_ms: float, altitude_m: float,
               heading_rad: float = 0.0, g_load: float = 1.0,
               range_to_asset_m: float = 1e6,
               snr_db: float = 25.0, rcs: float = 1.0,
               doppler_hz: float = 0.0, nis: float = 1.0,
               ) -> IntelligenceReport:
        """[REQ-V53-INT-08] Full intelligence assessment for one track.
        
        Creates per-track classifiers on first call. All three sub-systems
        (platform, ECM, intent) are updated and results combined.
        
        Returns:
            IntelligenceReport with platform, ECM, intent, and composite threat.
        """
        # Lazy init per-track
        if track_id not in self._classifiers:
            self._classifiers[track_id] = PlatformClassifier()
            self._ecm_detectors[track_id] = ECMDetector()
            self._intent_predictors[track_id] = IntentPredictor()
        
        platform = self._classifiers[track_id].update(speed_ms, altitude_m, g_load)
        ecm = self._ecm_detectors[track_id].update(snr_db, rcs, doppler_hz, nis)
        intent = self._intent_predictors[track_id].update(
            speed_ms, altitude_m, heading_rad, g_load,
            range_to_asset_m, platform)
        
        # Composite threat: weighted combination
        plat_threat = platform.threat_level.value / 4.0
        ecm_threat = 0.3 if ecm.status != ECMStatus.CLEAN else 0.0
        intent_threat = intent.threat_score
        
        composite = plat_threat * 0.3 + ecm_threat * 0.2 + intent_threat * 0.5
        composite = min(composite, 1.0)
        
        return IntelligenceReport(
            track_id=track_id,
            platform=platform,
            ecm=ecm,
            intent=intent,
            composite_threat=float(composite),
        )
    
    def remove_track(self, track_id: int) -> None:
        """Remove per-track intelligence state for deleted track."""
        self._classifiers.pop(track_id, None)
        self._ecm_detectors.pop(track_id, None)
        self._intent_predictors.pop(track_id, None)
    
    @property
    def active_tracks(self) -> List[int]:
        return list(self._classifiers.keys())
