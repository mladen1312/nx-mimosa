"""
NX-MIMOSA v4.2 "GUARDIAN" — Platform-Aware Variable-Structure IMM Tracker
===========================================================================
[REQ-V40-01] 6-model IMM bank: CV, CT+, CT-, CA, Jerk, Ballistic
[REQ-V40-02] Platform identification from kinematics (speed/g/altitude)
[REQ-V40-03] VS-IMM: dynamic model activation from platform DB constraints
[REQ-V40-04] Adaptive Q scaling from identified platform intent phase
[REQ-V40-05] TPM biasing from platform doctrine
[REQ-V40-06] Model pruning (prob < threshold → deactivate)
[REQ-V40-07] Per-model RTS smoother (window + full-track)
[REQ-V40-08] 4 output streams: realtime, refined, offline, intent
[REQ-V40-09] Graceful fallback to Unknown (all 6 models) if ID fails
[REQ-V41-01] Multi-domain classifier (111 platforms, 31 classes)
[REQ-V41-02] Intent prediction (16 types: terminal dive, sea skimming, etc.)
[REQ-V41-03] ECM detection & adaptive Q boost (noise/deception/DRFM/chaff)
[REQ-V41-04] Threat level fusion (classifier + intent + ECM)
[REQ-V41-05] False target rejection (birds, balloons, clutter)
[REQ-V41-06] Alert stream for fire control (TTI, evasion, sea skim)
[REQ-V42-01] GUARDIAN: Innovation bias detector (cumulative mean shift)
[REQ-V42-02] Measurement rejection layer (predict-only coast on bias)
[REQ-V42-03] Safety mechanisms: warmup, max coast, maneuver guard, ECM gate

Author: Dr. Mladen Mešter / Nexellum d.o.o.
License: AGPL v3 (open-source) | Commercial license available
"""

import numpy as np
from numpy.linalg import inv, pinv, LinAlgError
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
import json, os, math

# v4.1: Import classifier pipeline
try:
    from nx_mimosa_intent_classifier import (
        NxMimosaClassifierPipeline, ClassificationResult,
        IntentType, ThreatLevel, ECMStatus,
    )
    _HAS_CLASSIFIER = True
except ImportError:
    try:
        import sys as _sys
        _sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from nx_mimosa_intent_classifier import (
            NxMimosaClassifierPipeline, ClassificationResult,
            IntentType, ThreatLevel, ECMStatus,
        )
        _HAS_CLASSIFIER = True
    except ImportError:
        _HAS_CLASSIFIER = False

GRAVITY = 9.80665
EPS = 1e-10

# =============================================================================
# Data classes
# =============================================================================
@dataclass
class IntentState:
    platform_type: str = "Unknown"
    platform_category: str = "unknown"
    confidence: float = 0.0
    phase: str = "unknown"
    phase_confidence: float = 0.0
    estimated_max_g: float = 30.0
    estimated_max_speed: float = 3500.0
    threat_level: float = 0.5
    active_models: List[str] = field(default_factory=list)
    n_active_models: int = 3
    # --- v4.1 fields ---
    platform_class: str = "unknown"            # [REQ-V41-01] 31-class taxonomy
    intent: str = "unknown"                     # [REQ-V41-02] 16-type intent
    intent_confidence: float = 0.0
    ecm_status: str = "clean"                   # [REQ-V41-03] ECM detection
    ecm_q_scale: float = 1.0                    # Q boost under jamming
    dive_angle_deg: float = 0.0                 # Terminal dive angle
    time_to_impact_s: float = float('inf')      # TTI estimate
    altitude_rate_mps: float = 0.0              # Vertical velocity
    is_false_target: bool = False               # [REQ-V41-05] False target flag
    alerts: List[str] = field(default_factory=list)  # [REQ-V41-06] Fire control alerts

# =============================================================================
# Motion Model Builders
# =============================================================================
def _build_cv(dt, q):
    F = np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]])
    G = np.array([[dt**2/2,0],[0,dt**2/2],[dt,0],[0,dt]])
    return F, q*0.5*(G@G.T), np.array([[1,0,0,0],[0,1,0,0]],dtype=float), 4

def _build_ct(dt, omega, q):
    w = omega
    if abs(w) < 1e-6:
        return _build_cv(dt, q)
    sw, cw = np.sin(w*dt), np.cos(w*dt)
    F = np.array([[1,0,sw/w,-(1-cw)/w],[0,1,(1-cw)/w,sw/w],[0,0,cw,-sw],[0,0,sw,cw]])
    G = np.array([[dt**2/2,0],[0,dt**2/2],[dt,0],[0,dt]])
    return F, q*1.5*(G@G.T), np.array([[1,0,0,0],[0,1,0,0]],dtype=float), 4

def _build_ca(dt, q):
    F = np.eye(6)
    F[0,2]=dt; F[0,4]=dt**2/2; F[1,3]=dt; F[1,5]=dt**2/2; F[2,4]=dt; F[3,5]=dt
    G = np.array([[dt**2/2,0],[0,dt**2/2],[dt,0],[0,dt],[1,0],[0,1]])
    H = np.zeros((2,6)); H[0,0]=1; H[1,1]=1
    return F, q*3.0*(G@G.T), H, 6

def _build_jerk(dt, q):
    F = np.eye(8)
    F[0,2]=dt; F[0,4]=dt**2/2; F[0,6]=dt**3/6
    F[1,3]=dt; F[1,5]=dt**2/2; F[1,7]=dt**3/6
    F[2,4]=dt; F[2,6]=dt**2/2; F[3,5]=dt; F[3,7]=dt**2/2
    F[4,6]=dt; F[5,7]=dt
    G = np.array([[dt**3/6,0],[0,dt**3/6],[dt**2/2,0],[0,dt**2/2],[dt,0],[0,dt],[1,0],[0,1]])
    H = np.zeros((2,8)); H[0,0]=1; H[1,1]=1
    return F, q*8.0*(G@G.T), H, 8

def _build_ballistic(dt, q):
    F = np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]])
    G = np.array([[dt**2/2,0],[0,dt**2/2],[dt,0],[0,dt]])
    Q = q*0.5*(G@G.T)
    gq = GRAVITY*dt
    Q[1,1] += gq**2*0.5; Q[3,3] += gq**2*0.5
    return F, Q, np.array([[1,0,0,0],[0,1,0,0]],dtype=float), 4

MODEL_DIMS = {"CV":4,"CT_plus":4,"CT_minus":4,"CA":6,"Jerk":8,"Ballistic":4}
ALL_MODELS = list(MODEL_DIMS.keys())

def safe_inv(M):
    try: return inv(M + np.eye(M.shape[0])*EPS)
    except LinAlgError: return pinv(M)

# =============================================================================
# Platform Identifier
# =============================================================================
class PlatformIdentifier:
    """Hierarchical platform classifier using IMM model probabilities + speed.
    
    v4.0.1 fix: Previous classifier used filtered velocity derivatives which
    are smoothed to near-zero by IMM mixing. New approach uses:
    1. Speed (from tracker state — well estimated)
    2. IMM model probabilities (BEST dynamics indicator — tells us which
       motion model fits, directly revealing maneuver type)
    3. Parallel CV filter NIS (high = maneuvering)
    4. Tracker omega estimate (turn rate from CT models)
    
    Classification is hierarchical:
    Step 1: Coarse category from speed range + dynamics level
    Step 2: Fine ID from speed/dynamics profile within category
    """
    def __init__(self, db, window=30):
        self.db = {k:v for k,v in db.items() if k != "_meta"}
        # Speed & altitude (from tracker state, accurate)
        self.spd = deque(maxlen=window)
        self.alt = deque(maxlen=window)
        # IMM model probability histories (primary dynamics features)
        self.mu_cv_h = deque(maxlen=window)
        self.mu_ct_h = deque(maxlen=window)    # CT+ + CT- combined
        self.mu_ca_h = deque(maxlen=window)
        self.mu_jerk_h = deque(maxlen=window)
        self.mu_ball_h = deque(maxlen=window)
        # Parallel CV filter NIS (dynamics indicator: high = maneuvering)
        self.cv_nis_h = deque(maxlen=window)
        # Tracker omega (from CT model, good turn rate estimate)
        self.omega_h = deque(maxlen=window)
        # Legacy (kept for backward compat, low weight)
        self.g_h = deque(maxlen=window)
        self.pv = None
        self.best = "Unknown"
        self.conf = 0.0
        # === LIFETIME PEAK tracking (never expires) ===
        self.omega_peak = 0.0
        self.cv_nis_peak = 0.0
        self.mu_ct_peak = 0.0
        self.mu_ca_peak = 0.0
        self.mu_jerk_peak = 0.0
        self.cv_nis_sum = 0.0  # Running sum for mean
        self.n_steps = 0
        self.spd_max_ever = 0.0
        # Speed trend tracking (v4.0.2: for ballistic/hypersonic discrimination)
        self.spd_early = deque(maxlen=20)   # First 20 speed samples
        self.spd_late = deque(maxlen=20)    # Latest 20 speed samples
        self.spd_samples_total = 0
        self.alt_max_ever = 0.0  # Lifetime peak altitude

    def update(self, pos, vel, dt, mu=None, cv_nis=None, omega=0.0):
        """Update features and classify.
        
        Args:
            pos: tracker estimated position [x,y]
            vel: tracker estimated velocity [vx,vy]
            dt: time step
            mu: dict of IMM model probabilities (NEW - primary dynamics feature)
            cv_nis: parallel CV filter NIS (NEW - dynamics indicator)
            omega: tracker turn rate estimate from CT model
        """
        s = np.linalg.norm(vel)
        self.spd.append(s)
        self.spd_max_ever = max(self.spd_max_ever, s)
        self.alt.append(abs(pos[1]) if len(pos) >= 2 else 0)
        self.alt_max_ever = max(self.alt_max_ever, abs(pos[1]) if len(pos) >= 2 else 0)
        self.omega_h.append(abs(omega))
        self.omega_peak = max(self.omega_peak, abs(omega))
        self.n_steps += 1
        # Speed trend: track early vs late for accel/decel detection
        self.spd_samples_total += 1
        if self.spd_samples_total <= 20:
            self.spd_early.append(s)
        self.spd_late.append(s)  # Rolling last 20
        
        # Legacy g from filtered velocity (kept but deprioritized)
        if self.pv is not None and dt > 0:
            dv = vel - self.pv
            self.g_h.append(np.linalg.norm(dv) / dt / GRAVITY)
        self.pv = vel.copy()
        
        # IMM model probabilities — THE KEY FEATURE
        if mu:
            self.mu_cv_h.append(mu.get("CV", 0))
            ct = mu.get("CT_plus", 0) + mu.get("CT_minus", 0)
            self.mu_ct_h.append(ct)
            self.mu_ct_peak = max(self.mu_ct_peak, ct)
            ca = mu.get("CA", 0)
            self.mu_ca_h.append(ca)
            self.mu_ca_peak = max(self.mu_ca_peak, ca)
            jk = mu.get("Jerk", 0)
            self.mu_jerk_h.append(jk)
            self.mu_jerk_peak = max(self.mu_jerk_peak, jk)
            self.mu_ball_h.append(mu.get("Ballistic", 0))
        
        # Parallel CV filter NIS (instantaneous, not EMA!)
        if cv_nis is not None:
            self.cv_nis_h.append(cv_nis)
            self.cv_nis_peak = max(self.cv_nis_peak, cv_nis)
            self.cv_nis_sum += cv_nis
        
        if len(self.spd) < 5:
            return "Unknown", 0.0
        return self._classify()

    def _classify(self):
        # === Feature extraction ===
        spd_avg = np.mean(self.spd)
        spd_max = self.spd_max_ever  # Lifetime max, not windowed
        alt_avg = np.mean(self.alt)
        
        # IMM dynamics profile — use BOTH windowed averages AND lifetime peaks
        mu_cv = np.mean(self.mu_cv_h) if self.mu_cv_h else 0.5
        mu_ct = np.mean(self.mu_ct_h) if self.mu_ct_h else 0.1
        mu_ca = np.mean(self.mu_ca_h) if self.mu_ca_h else 0.1
        mu_jerk = np.mean(self.mu_jerk_h) if self.mu_jerk_h else 0.0
        mu_ball = np.mean(self.mu_ball_h) if self.mu_ball_h else 0.0
        
        # LIFETIME PEAKS (critical: captures transient dynamics even if current benign)
        mu_ct_peak = self.mu_ct_peak
        mu_ca_peak = self.mu_ca_peak
        mu_jerk_peak = self.mu_jerk_peak
        
        # CV NIS — use lifetime statistics
        cv_nis_avg = self.cv_nis_sum / max(self.n_steps, 1)
        cv_nis_peak = self.cv_nis_peak
        
        # Turn rate — lifetime peak is critical (captures maneuver even if now benign)
        omega_avg = np.mean(self.omega_h) if self.omega_h else 0
        omega_peak = self.omega_peak
        
        # Dynamics flags using PEAKS (not windowed values)
        is_maneuvering = (mu_ct_peak > 0.5 or omega_peak > 0.1 or 
                          cv_nis_peak > 50 or mu_ca_peak > 0.3)
        is_high_dynamics = (mu_ct_peak > 0.8 or omega_peak > 0.3 or 
                           cv_nis_peak > 500 or mu_jerk_peak > 0.3)
        
        # Speed trend: positive = accelerating, negative = decelerating
        if len(self.spd_early) >= 5 and self.spd_samples_total >= 30:
            spd_trend = np.mean(self.spd_late) - np.mean(self.spd_early)
        else:
            spd_trend = 0.0
        
        # Altitude range (max - min over lifetime)
        alt_max = self.alt_max_ever  # Lifetime peak, not windowed
        alt_min = min(self.alt) if self.alt else 0

        # === Step 1: Coarse category classification ===
        coarse = self._coarse_category(spd_avg, spd_max, alt_avg,
                                        is_maneuvering, is_high_dynamics,
                                        mu_ct, mu_ca, mu_ball, omega_peak)
        
        # === Step 2: Fine ID within category ===
        feats = dict(spd_avg=spd_avg, spd_max=spd_max, alt_avg=alt_avg,
                     mu_cv=mu_cv, mu_ct=mu_ct, mu_ca=mu_ca, 
                     mu_jerk=mu_jerk, mu_ball=mu_ball,
                     mu_ct_max=mu_ct_peak, mu_ca_max=mu_ca_peak,
                     mu_jerk_max=mu_jerk_peak,
                     cv_nis_avg=cv_nis_avg, cv_nis_max=cv_nis_peak,
                     omega_avg=omega_avg, omega_max=omega_peak,
                     is_maneuvering=is_maneuvering,
                     is_high_dynamics=is_high_dynamics,
                     spd_trend=spd_trend, alt_max=alt_max, alt_min=alt_min)
        
        candidates = [n for n, p in self.db.items() 
                      if p.get("category","") in coarse and n != "Unknown"]
        if not candidates:
            candidates = [n for n in self.db if n != "Unknown"]
        
        scores = {n: self._score_fine(self.db[n], feats) for n in candidates}
        if not scores:
            return "Unknown", 0.0
        
        best = max(scores, key=scores.get)
        c = scores[best]
        if c >= 0.30:
            self.best, self.conf = best, c
        else:
            self.best, self.conf = "Unknown", c
        return self.best, self.conf

    def _coarse_category(self, spd_avg, spd_max, alt_avg,
                          is_maneuvering, is_high_dynamics,
                          mu_ct, mu_ca, mu_ball, omega_max):
        """Return set of possible coarse categories.
        
        v4.0.2: Expanded to 20 categories covering 70 platforms.
        Hierarchical: speed regime → dynamics level → candidate categories.
        """
        cats = set()
        
        # === Speed-based primary classification ===
        if spd_max > 1500:
            # Hypersonic regime: missiles only
            cats.update(["hypersonic_missile", "ballistic_missile", "sam",
                         "supersonic_cruise_missile"])
        elif spd_max > 800:
            # Supersonic: missiles, interceptors, fast fighters, SAMs, AAMs
            cats.update(["ballistic_missile", "sam", "mlrs", "aam",
                         "hypersonic_missile", "supersonic_cruise_missile"])
            if is_maneuvering and omega_max > 0.03:
                cats.update(["fighter", "interceptor"])
        elif spd_max > 150:
            # Subsonic fast: fighters, cruise missiles, transport, bomber, UCAV
            if is_high_dynamics or mu_ct > 0.2 or omega_max > 0.05:
                cats.update(["fighter", "cruise_missile", "interceptor"])
                if omega_max > 0.15:
                    cats.add("ucav")  # UCAV only if fighter-like dynamics
            if not is_maneuvering or mu_ct < 0.1:
                cats.update(["bomber", "transport", "aew", "cruise_missile",
                             "male_uav"])
                if omega_max > 0.10:
                    cats.add("ucav")  # UCAV can also be moderate dynamics
            if not cats:
                cats.update(["fighter", "bomber", "transport", "aew", 
                             "cruise_missile", "interceptor"])
        elif spd_max > 60:
            # Medium speed: UAVs, helicopters, loitering munitions
            cats.update(["loitering_munition", "male_uav", "ucav",
                         "attack_helicopter", "utility_helicopter"])
            if is_maneuvering:
                cats.add("cruise_missile")
        else:
            # Low speed: drones, helicopters
            cats.update(["small_drone", "fpv_kamikaze", "loitering_munition",
                         "attack_helicopter", "utility_helicopter", "male_uav"])
        
        return cats

    def _score_fine(self, p, f):
        """Score a specific platform against extracted features.
        
        v4.0.2 fixes:
        - Speed utilization ratio (spd_max/pms) for Kinzhal vs Iskander
        - Tighter fighter omega gate (>0.4) to avoid Kalibr confusion
        - Jerk model peak for Su-35 post-stall discrimination
        - Altitude range for hypersonic vs ballistic
        """
        cat = p.get("category", "unknown")
        pms = p["max_speed_mps"]
        pcs = p.get("cruise_speed_mps", pms * 0.6)
        pmg = p["max_g"]
        psg = p.get("sustained_g", pmg * 0.5)
        pma = p.get("max_altitude_m", 50000)
        
        sc = 0.0
        
        # --- Speed match (0-4) — well estimated by tracker ---
        spd_ratio = f["spd_avg"] / max(pcs, 1)
        if f["spd_max"] > pms * 1.3:
            sc -= 3  # Too fast for this platform
        elif 0.4 <= spd_ratio <= 1.8:
            sc += 4 * max(0, 1 - abs(spd_ratio - 1) * 0.5)
        else:
            sc += 0.5  # Marginal speed match
        
        # --- Speed utilization ratio (0-3) — NEW v4.0.2 ---
        # How much of this platform's max speed is being used?
        # Near-max = terminal/dogfight, well-below = cruise/glide from higher capability
        spd_util = f["spd_max"] / max(pms, 1)
        if cat in ("ballistic_missile", "hypersonic_missile", "sam", "mlrs"):
            # For missiles: expected utilization varies by type
            # Ballistic in terminal: high utilization (0.7-1.0)
            # Hypersonic in glide: moderate utilization (0.3-0.7)
            if cat == "hypersonic_missile":
                # Kinzhal: glide phase uses 40-70% of max speed
                if 0.3 < spd_util < 0.70:
                    sc += 3.0  # Sweet spot for glide phase
                elif 0.70 <= spd_util < 0.85:
                    sc += 1.5  # Transitional
                elif spd_util >= 0.85:
                    sc += 0.0  # Near max = not gliding, wrong platform
                else:
                    sc += 1.0
            elif cat == "ballistic_missile":
                # Iskander terminal: uses 80-100%+ of max speed
                if spd_util > 0.90:
                    sc += 4.0  # Confirmed terminal at max power
                elif spd_util > 0.70:
                    sc += 2.5  # High utilization
                elif spd_util > 0.50:
                    sc += 1.0  # Mid-course
                else:
                    sc += 0.0  # Too slow for this platform
            elif cat == "sam":
                # SAM: high utilization during intercept
                if spd_util > 0.5:
                    sc += 2.5
                else:
                    sc += 0.5
            else:
                sc += 1.5  # MLRS: moderate
        elif cat == "fighter":
            # Fighters commonly operate at 30-80% of max speed
            if 0.25 < spd_util < 0.85:
                sc += 2.0
            else:
                sc += 0.5
        else:
            sc += 1.5  # Default moderate
        
        # --- Dynamics match via lifetime peaks (0-6) — PRIMARY ---
        dyn_sc = 0.0
        omega_pk = f["omega_max"]  # Lifetime peak
        nis_pk = f["cv_nis_max"]   # Lifetime peak
        
        if cat == "fighter":
            # v4.0.2: Tighten omega gate — need >0.4 for full fighter score
            # Cruise missiles have omega ~0.3, fighters have >0.5 sustained
            dyn_sc += 3.0 if omega_pk > 0.5 else (1.5 if omega_pk > 0.4 else 0.5)
            dyn_sc += 2.0 if nis_pk > 500 else (1.0 if nis_pk > 100 else 0)
            dyn_sc += 1.0 if f["is_high_dynamics"] else 0
            
        elif cat == "cruise_missile":
            # Cruise missiles: MODERATE omega (0.05-0.35) + moderate NIS
            # v4.0.2: Reward omega being moderate, penalize if too fighter-like
            # Cruise missiles have TRANSIENT turns (omega_peak high but omega_avg low)
            if 0.05 < omega_pk < 0.35:
                dyn_sc += 3.0  # Sweet spot
            elif 0.35 <= omega_pk < 0.5:
                dyn_sc += 2.0  # Borderline with fighter (waypoint turns)
            elif omega_pk >= 0.5:
                dyn_sc += 0.0  # Too aggressive = fighter, not cruise
            else:
                dyn_sc += 1.0  # Too benign
            dyn_sc += 2.0 if 20 < nis_pk < 300 else (0.5 if nis_pk > 300 else 1.0)
            # Cruise missiles have transient dynamics — sustained straight flight
            # omega_avg << omega_peak means INTERMITTENT turns (cruise missile pattern)
            omega_avg = f.get("omega_avg", omega_pk)
            transient_ratio = omega_avg / max(omega_pk, 0.01)
            if transient_ratio < 0.4:
                dyn_sc += 1.5  # Clearly intermittent: cruise missile pattern
            else:
                dyn_sc += 0.0  # Sustained turning: more UCAV/fighter-like
            
        elif cat in ("ballistic_missile", "hypersonic_missile", "mlrs"):
            # Ballistic/hypersonic: VERY LOW omega (mostly straight) + HIGH NIS (acceleration)
            # v4.0.2: Tighter omega gate — ballistic/hypersonic don't do proportional nav
            dyn_sc += 3.0 if omega_pk < 0.10 else (1.5 if omega_pk < 0.20 else (0.5 if omega_pk < 0.30 else 0))
            dyn_sc += 2.0 if nis_pk > 100 else (1.0 if nis_pk > 20 else 0)
            dyn_sc += 1.0 if f["cv_nis_avg"] > 10 else 0
            
        elif cat == "sam":
            # SAMs: HIGH omega (proportional nav turns) + VERY HIGH NIS
            # v4.0.2: Require SUSTAINED omega, not just noise spikes
            dyn_sc += 3.0 if omega_pk > 0.20 else (1.0 if omega_pk > 0.10 else 0)
            dyn_sc += 2.0 if nis_pk > 1000 else (1.0 if nis_pk > 100 else 0)
            # SAMs have sustained turning (proportional nav), not just transient
            omega_avg = f.get("omega_avg", omega_pk)
            dyn_sc += 1.0 if omega_avg > 0.05 else 0  # Sustained turn signature
        
        elif cat == "aam":
            # AAMs: VERY HIGH omega (proportional nav, high-g terminal) + extreme NIS
            dyn_sc += 3.0 if omega_pk > 0.3 else (1.5 if omega_pk > 0.15 else 0)
            dyn_sc += 2.0 if nis_pk > 500 else (1.0 if nis_pk > 100 else 0)
            dyn_sc += 1.0 if f["is_high_dynamics"] else 0
        
        elif cat == "interceptor":
            # Interceptors: moderate omega, high speed, moderate NIS
            dyn_sc += 2.0 if 0.05 < omega_pk < 0.4 else 0.5
            dyn_sc += 2.0 if nis_pk > 50 else (1.0 if nis_pk > 10 else 0)
            dyn_sc += 2.0 if f["spd_avg"] > 400 else 0
        
        elif cat == "supersonic_cruise_missile":
            # Supersonic cruise: LOW omega (mostly straight) + MODERATE NIS from acceleration
            dyn_sc += 3.0 if omega_pk < 0.3 else (1.0 if omega_pk < 0.5 else 0)
            dyn_sc += 2.0 if nis_pk > 50 else (1.0 if nis_pk > 10 else 0)
            dyn_sc += 1.0 if f["spd_avg"] > 500 else 0
            
        elif cat in ("bomber", "transport", "aew"):
            # Low dynamics: very low omega + very low NIS
            dyn_sc += 3.0 if omega_pk < 0.1 else (1.0 if omega_pk < 0.2 else 0)
            dyn_sc += 2.0 if nis_pk < 20 else (0.5 if nis_pk < 100 else 0)
            dyn_sc += 1.0 if not f["is_maneuvering"] else 0
            
        elif cat == "loitering_munition":
            dyn_sc += 2.0 if omega_pk < 0.6 else 0
            dyn_sc += 2.0 if nis_pk < 50 else (1.0 if nis_pk < 200 else 0)
            dyn_sc += 2.0 if f["spd_avg"] < 80 else 0
            
        elif cat == "fpv_kamikaze":
            dyn_sc += 2.0 if omega_pk > 0.3 else 1.0
            dyn_sc += 2.0 if 10 < nis_pk < 200 else (1.0 if nis_pk > 200 else 0.5)
            dyn_sc += 2.0 if f["spd_avg"] < 60 else 0
            
        elif cat == "small_drone":
            dyn_sc += 3.0 if f["spd_avg"] < 25 else 0
            dyn_sc += 2.0 if nis_pk < 20 else 0
            dyn_sc += 1.0 if omega_pk < 0.3 else 0
        
        elif cat in ("attack_helicopter", "utility_helicopter"):
            # Helicopters: low speed, moderate omega, can hover (spd~0)
            dyn_sc += 3.0 if f["spd_avg"] < 100 else 0
            dyn_sc += 2.0 if omega_pk < 0.5 else 0.5
            dyn_sc += 1.0 if f["alt_avg"] < 6000 else 0
        
        elif cat == "ucav":
            # UCAVs: fighter-like but slightly less agile — REQUIRE sustained maneuvering
            # v4.0.2: Must have SUSTAINED turns, not just transient waypoint turns
            dyn_sc += 2.0 if omega_pk > 0.3 else (0.5 if omega_pk > 0.2 else 0)
            dyn_sc += 2.0 if 50 < nis_pk < 500 else (0.5 if nis_pk > 500 else 0)
            # CRITICAL: UCAV must have sustained maneuvering, not transient
            omega_avg = f.get("omega_avg", omega_pk)
            if omega_avg > 0.08 and f["is_high_dynamics"]:
                dyn_sc += 2.0  # Sustained turns = UCAV/fighter pattern
            elif omega_avg > 0.04:
                dyn_sc += 0.5  # Marginal
            else:
                dyn_sc += 0.0  # omega_avg too low = cruise missile, not UCAV
        
        elif cat == "male_uav":
            # MALE UAVs: slow, low dynamics, long loiter
            dyn_sc += 3.0 if f["spd_avg"] < 150 else 0
            dyn_sc += 2.0 if omega_pk < 0.3 else 0
            dyn_sc += 1.0 if nis_pk < 50 else 0
        else:
            dyn_sc = 2.0
        
        sc += min(dyn_sc, 6)  # Cap at 6
        
        # --- Altitude match (0-2) ---
        if f["alt_avg"] <= pma * 1.2:
            sc += 2
        elif f["alt_avg"] <= pma * 2:
            sc += 0.5
        
        # --- Altitude precision for high-alt platforms (0-2) — NEW v4.0.2 ---
        if cat in ("ballistic_missile", "hypersonic_missile"):
            # Use lifetime alt_max_ever for discrimination
            alt_max = f.get("alt_max", f["alt_avg"])
            if pma > 10000:
                alt_util = alt_max / pma
                # Platforms match when observed alt_max is reasonable fraction of capability
                # Kinzhal (pma=80000) at alt_max=50000 → 0.625 → good
                # Iskander (pma=50000) at alt_max=40000 → 0.80 → good  
                # Kinzhal (pma=80000) at alt_max=40000 → 0.50 → marginal
                sc += 2.0 * max(0, 1 - abs(alt_util - 0.65) * 2.0)
        
        # --- Speed precision within category (0-3) ---
        if cat == "fighter":
            # v4.0.2: Also use jerk peak for Su-35 post-stall discrimination
            r = f["spd_avg"] / max(pcs, 1)
            sc += 2.0 * max(0, 1 - abs(r - 1) * 0.6)
            # Jerk model peak — Su-35 post-stall has extreme jerk
            has_jerk_pref = "Jerk" in p.get("preferred_models", [])
            if has_jerk_pref and f["mu_jerk_max"] > 0.3:
                sc += 1.0  # Bonus for jerk-capable platform matching jerk observation
            elif not has_jerk_pref and f["mu_jerk_max"] > 0.3:
                sc -= 0.5  # Penalty: jerk observed but platform doesn't use it
        elif cat in ("ballistic_missile", "hypersonic_missile"):
            r = f["spd_max"] / max(pms, 1)
            sc += 1.5 * max(0, 1 - abs(r - 0.8) * 0.5)
            exp_alt = pma * 0.5
            alt_r = f["alt_avg"] / max(exp_alt, 1000)
            sc += 1.5 * max(0, 1 - abs(alt_r - 1) * 0.3)
        elif cat == "sam":
            r = f["spd_avg"] / max(pcs, 1)
            sc += 3 * max(0, 1 - abs(r - 1) * 0.5)
        else:
            r = f["spd_avg"] / max(pcs, 1)
            sc += 3 * max(0, 1 - abs(r - 1) * 0.5)
        
        return max(sc, 0) / 21  # Normalize: max = 4+4+6+2+2+3 = 21

# =============================================================================
# Intent Predictor
# =============================================================================
class IntentPredictor:
    def predict(self, pd, speed, ag, mu):
        phases = pd.get("intent_phases",{})
        if not phases: return "unknown", 0.0
        cs = pd.get("cruise_speed_mps",500); sg = pd.get("sustained_g",5)
        sr = speed/max(cs,1); best_p, best_s = "unknown", 0
        for pn, ph in phases.items():
            s = ph.get("prob",0.1)
            s *= 1+sum(mu.get(m,0) for m in ph.get("models",[]))*2
            if pn in ("cruise","patrol","transit","orbit") and 0.7<=sr<=1.3 and ag<1: s*=2
            elif pn in ("engagement","attack","post_stall") and ag>sg*0.3: s*=2.5
            elif pn=="terminal" and (ag>1 or sr>0.8): s*=2
            elif pn=="boost" and ag>2: s*=2
            elif pn in ("hover","loiter") and speed<5: s*=3
            elif pn in ("glide","midcourse","ballistic") and ag<1.5: s*=1.5
            if s>best_s: best_s=s; best_p=pn
        return best_p, min(best_s/5,1)

# =============================================================================
# NX-MIMOSA v4.0 SENTINEL
# =============================================================================
# =============================================================================
# [REQ-V42-01] Innovation Bias Detector — GUARDIAN Layer
# =============================================================================
# Detects SYSTEMATIC measurement corruption (RGPO/VGPO) by monitoring
# the innovation sequence {ν_k} = z_k - H·x̂_{k|k-1}.
#
# Under clean tracking:  E[ν] = 0  (innovation is zero-mean white noise)
# Under deception ECM:   E[ν] ≠ 0  (bias accumulates from false returns)
#
# Detection criteria:
#   1. Running mean bias:  ||mean(ν_history)|| > k·σ_ν
#      → RGPO/VGPO creates progressive drift in innovation mean
#   2. Innovation trend:   |Δmean(ν)| consistently positive/negative
#      → Monotonic shift indicates systematic pull-off
#   3. Normalized bias:    ||mean(ν)||² / trace(S) > χ²(0.99, 2)
#      → Statistical test: bias exceeds expected innovation covariance
#
# Response: When bias detected → MEASUREMENT REJECTION (predict-only)
# This is fundamentally different from R-boost:
#   R-boost: Reduces gain but STILL uses corrupted measurement
#   Rejection: COMPLETELY ignores measurement, uses prediction only
#
# Safety: Maneuver-aware — during turns, innovation naturally grows.
# Use NIS (not raw innovation) to distinguish maneuver from bias.
# =============================================================================
class InnovationBiasDetector:
    """Detects systematic bias in innovation sequence for RGPO/VGPO defense.
    
    [REQ-V42-01] Core v4.2 GUARDIAN component.
    
    Key safety mechanisms to prevent runaway rejection:
    1. Warmup: Don't arm until filter_age > warmup_steps (filter must converge first)
    2. Max coast: After max_consecutive_rejects, FORCE accept to prevent drift
    3. Decay: Bias estimate naturally decays during rejection (no new innovations)
    4. Only active when ECM confirmed (prevents false positives during maneuvers)
    """
    
    def __init__(self, window: int = 20, bias_sigma: float = 3.5,
                 trend_window: int = 10, max_consecutive_rejects: int = 15,
                 warmup_steps: int = 30):
        """
        Args:
            window: Innovation history length for running mean.
            bias_sigma: Threshold multiplier — reject if mean > bias_sigma * σ.
            trend_window: Window for trend detection (monotonic shift).
            max_consecutive_rejects: Safety valve — force accept after this many.
            warmup_steps: Don't arm until this many total updates received.
        """
        self.window = window
        self.bias_sigma = bias_sigma
        self.trend_window = trend_window
        self.max_consecutive_rejects = max_consecutive_rejects
        self.warmup_steps = warmup_steps
        
        # Innovation history: stores [νx, νy] per step — ONLY from accepted updates
        self._nu_history: deque = deque(maxlen=window)
        # Innovation covariance history (S matrix trace) — from accepted updates
        self._s_trace_history: deque = deque(maxlen=window)
        
        # State
        self.bias_detected = False
        self.bias_magnitude = 0.0       # ||mean(ν)|| in meters
        self.bias_direction = np.zeros(2)  # Unit vector of bias direction
        self.bias_sigma_ratio = 0.0     # How many σ above threshold
        self.reject_count = 0           # Consecutive rejections
        self.total_rejections = 0       # Lifetime counter
        self._total_updates = 0         # Total updates (for warmup)
        self._armed = False             # Only arm after warmup + enough history
    
    def update(self, nu: np.ndarray, S: np.ndarray, nis: float,
               omega: float = 0.0, ecm_active: bool = False) -> bool:
        """Process one innovation and decide: accept or reject measurement.
        
        CRITICAL: Only ACCEPTED innovations are stored in history.
        During rejection, history is FROZEN — this prevents the detector
        from seeing its own drift as "bias" (runaway feedback loop).
        
        Args:
            nu: Innovation vector [νx, νy] (meters).
            S: Innovation covariance matrix (2×2).
            nis: Normalized Innovation Squared (ν'·S⁻¹·ν).
            omega: Current turn rate (rad/s) — for maneuver awareness.
            ecm_active: Whether ECM detector has flagged active ECM.
            
        Returns:
            True if measurement should be REJECTED (predict-only).
            False if measurement is OK to use.
        """
        self._total_updates += 1
        
        # ── Safety Valve 1: Warmup ──────────────────────────────────
        # Filter needs time to converge. Initial transients look like bias.
        if self._total_updates < self.warmup_steps:
            # Still warming up: accept everything, build history
            self._nu_history.append(nu.copy())
            self._s_trace_history.append(np.trace(S))
            self.bias_detected = False
            self.reject_count = 0
            return False
        
        # ── Safety Valve 2: Max Coast Duration ──────────────────────
        # If we've been rejecting too long, FORCE accept to prevent
        # indefinite drift. Better to accept one corrupted measurement
        # than to coast into oblivion.
        if self.reject_count >= self.max_consecutive_rejects:
            # Force accept — add to history, reset streak
            self._nu_history.append(nu.copy())
            self._s_trace_history.append(np.trace(S))
            self.bias_detected = False
            self.reject_count = 0
            return False
        
        # ── ECM Gate ────────────────────────────────────────────────
        # Only engage bias detection when ECM is confirmed active.
        # Without ECM, large innovations are from maneuvers, not jamming.
        if not ecm_active:
            self._nu_history.append(nu.copy())
            self._s_trace_history.append(np.trace(S))
            self.bias_detected = False
            self.reject_count = 0
            return False
        
        # ── Need minimum history ────────────────────────────────────
        if len(self._nu_history) < 10:
            self._nu_history.append(nu.copy())
            self._s_trace_history.append(np.trace(S))
            self._armed = False
            self.bias_detected = False
            return False
        
        self._armed = True
        nu_arr = np.array(self._nu_history)
        
        # ── Criterion 1: Running Mean Bias ──────────────────────────
        mean_nu = np.mean(nu_arr, axis=0)
        bias_mag = np.linalg.norm(mean_nu)
        
        # Expected innovation std from S trace history
        avg_s_trace = np.mean(self._s_trace_history)
        sigma_nu = np.sqrt(avg_s_trace / 2.0)  # Per-axis σ from trace(S)/dim
        sigma_mean = sigma_nu / np.sqrt(len(nu_arr))
        
        self.bias_magnitude = bias_mag
        self.bias_sigma_ratio = bias_mag / (sigma_mean + 1e-10)
        if bias_mag > 1e-6:
            self.bias_direction = mean_nu / bias_mag
        
        # ── Criterion 2: Current innovation consistent with bias ────
        # The CURRENT measurement must also be biased in the SAME direction
        # as the running mean. This prevents rejecting good measurements
        # after ECM stops (running mean decays slowly).
        current_proj = float(np.dot(nu, self.bias_direction))
        current_consistent = current_proj > sigma_nu * 0.5  # Same direction
        
        # ── Criterion 3: Normalized Bias Test ───────────────────────
        normalized_bias = (bias_mag ** 2) / (avg_s_trace / len(nu_arr) + 1e-10)
        chi2_exceeded = normalized_bias > 9.21  # χ²(0.99, 2)
        
        # ── Maneuver Guard ──────────────────────────────────────────
        maneuver_penalty = 0.0
        if abs(omega) > 0.08:
            maneuver_penalty = 2.0 ** ((abs(omega) - 0.08) / 0.1)
        
        effective_sigma = self.bias_sigma * (1.0 + maneuver_penalty)
        
        # ── Decision ────────────────────────────────────────────────
        # Reject requires ALL of:
        #   1. Running mean bias > threshold
        #   2. χ² confirms statistical significance
        #   3. Current measurement is in bias direction (not already clean)
        #   4. Not maneuvering hard
        #   5. ECM is active (already checked above)
        primary_trigger = self.bias_sigma_ratio > effective_sigma
        should_reject = (primary_trigger and chi2_exceeded and 
                        current_consistent and abs(omega) < 0.25)
        
        self.bias_detected = should_reject
        
        if should_reject:
            self.reject_count += 1
            self.total_rejections += 1
            # DON'T add to history — this is a rejected (corrupted) innovation
            # History stays frozen with last good data
        else:
            self.reject_count = 0
            # ACCEPTED: add to history (this updates the running mean)
            self._nu_history.append(nu.copy())
            self._s_trace_history.append(np.trace(S))
        
        return should_reject
    
    @property
    def state(self) -> dict:
        """Diagnostic state snapshot."""
        return {
            "bias_detected": self.bias_detected,
            "bias_magnitude_m": round(self.bias_magnitude, 2),
            "bias_sigma_ratio": round(self.bias_sigma_ratio, 2),
            "bias_direction": self.bias_direction.tolist(),
            "reject_count": self.reject_count,
            "total_rejections": self.total_rejections,
            "armed": self._armed,
        }
    
    def reset(self):
        """Clear history (e.g., after track re-initialization)."""
        self._nu_history.clear()
        self._s_trace_history.clear()
        self.bias_detected = False
        self.bias_magnitude = 0.0
        self.reject_count = 0
        self.total_rejections = 0
        self._total_updates = 0
        self._armed = False


# =============================================================================
# [REQ-V41-11] Parallel Z-Axis Kalman Filter
# =============================================================================
# Independent 1D constant-acceleration Kalman on altitude (z-axis).
# State: [z, vz, az]  (3-state)
# Provides filtered altitude, vertical velocity, and vertical acceleration
# for intent prediction (terminal dive, sea skimming, pop-up, reentry).
#
# Why parallel and not full 3D:
#   - Zero coupling with xy tracker → zero risk of regression
#   - xy IMM models (CT, CA, Jerk) are horizontal-plane phenomena
#   - z dynamics are simpler (no coordinated turn in z)
#   - 3-state CA model captures dive/climb/coast transitions
#
# The filter adapts its process noise based on ECM status:
#   - Clean: q_z = q_z_base (smooth tracking)
#   - ECM:   q_z = q_z_base * ecm_r_scale (trust prediction more)
# =============================================================================
class ParallelZFilter:
    """Independent altitude tracker: state = [z, vz, az]."""

    def __init__(self, dt: float, r_z: float = 50.0, q_z: float = 0.1):
        """
        Args:
            dt:  Sample period (s)
            r_z: Altitude measurement noise variance (m²).
                 Default 50 = ±7m 1σ (typical barometric or radar elevation).
            q_z: Process noise intensity for vertical acceleration (m²/s⁵).
                 Default 0.1 = gentle maneuvers; increase for fighters.
        """
        self.dt = dt
        self.r_z = r_z
        self.q_z_base = q_z

        # State: [z, vz, az]
        self.x = np.zeros(3)
        self.P = np.diag([1000.0, 100.0, 10.0])  # Large initial uncertainty

        # F: constant-acceleration transition
        #   z'  = z + vz*dt + 0.5*az*dt²
        #   vz' = vz + az*dt
        #   az' = az  (random walk on acceleration)
        t = dt; t2 = 0.5 * dt**2
        self.F = np.array([
            [1, t, t2],
            [0, 1, t ],
            [0, 0, 1 ],
        ])

        # G: process noise input (jerk-driven)
        #   Γ = [dt³/6, dt²/2, dt]ᵀ  (jerk noise → acceleration random walk)
        g = np.array([[dt**3/6], [dt**2/2], [dt]])
        self.Q_base = q_z * (g @ g.T)

        # H: measure altitude only
        self.H = np.array([[1.0, 0.0, 0.0]])
        self.R_z = np.array([[r_z]])

        self._initialized = False
        self.step = 0

    def update(self, z_meas: float, ecm_r_scale: float = 1.0) -> tuple:
        """Process one altitude measurement.

        Args:
            z_meas: Altitude measurement (m). Use NaN to skip (predict-only).
            ecm_r_scale: ECM R-scale from main tracker (boosts R_z under jamming).

        Returns:
            (z_est, vz_est, az_est) — filtered altitude, vertical velocity,
            vertical acceleration.
        """
        # First measurement: initialize
        if not self._initialized:
            self.x[0] = z_meas
            self._initialized = True
            self.step = 1
            return float(self.x[0]), 0.0, 0.0

        self.step += 1

        # Scale process noise: under ECM, widen Q to allow faster dynamics
        Q = self.Q_base * max(1.0, ecm_r_scale * 0.5)

        # --- Predict ---
        xp = self.F @ self.x
        Pp = self.F @ self.P @ self.F.T + Q

        # --- Update (skip if NaN) ---
        if np.isfinite(z_meas):
            # R also scales with ECM (less trust in measurement)
            R_eff = self.R_z * max(1.0, ecm_r_scale)
            y = z_meas - self.H @ xp            # Innovation
            S = self.H @ Pp @ self.H.T + R_eff   # Innovation covariance
            K = Pp @ self.H.T / S[0, 0]          # Kalman gain (scalar)
            self.x = xp + K.flatten() * y[0]
            IKH = np.eye(3) - K @ self.H
            self.P = IKH @ Pp @ IKH.T + K @ R_eff @ K.T  # Joseph form
        else:
            self.x = xp
            self.P = Pp

        return float(self.x[0]), float(self.x[1]), float(self.x[2])

    @property
    def altitude(self) -> float:
        """Filtered altitude estimate (m)."""
        return float(self.x[0])

    @property
    def vz(self) -> float:
        """Filtered vertical velocity (m/s). Positive = climbing."""
        return float(self.x[1])

    @property
    def az(self) -> float:
        """Filtered vertical acceleration (m/s²). Positive = pull-up."""
        return float(self.x[2])

    @property
    def dive_angle_deg(self) -> float:
        """Dive angle in degrees from horizontal. Negative = diving."""
        return float(np.degrees(np.arctan2(self.x[1], 1e-6)))  # placeholder

    def get_dive_angle(self, horizontal_speed: float) -> float:
        """Dive angle given horizontal speed from xy tracker.

        Args:
            horizontal_speed: Speed in xy-plane (m/s) from main tracker.

        Returns:
            Dive angle (degrees). Negative = diving, positive = climbing.
        """
        if horizontal_speed < 1.0:
            return 0.0
        return float(np.degrees(np.arctan2(self.x[1], horizontal_speed)))

    def get_tti(self, horizontal_speed: float) -> float:
        """Time-to-impact estimate for diving targets.

        Args:
            horizontal_speed: Speed in xy-plane (m/s).

        Returns:
            TTI in seconds. inf if not diving or altitude unknown.
        """
        z, vz = self.x[0], self.x[1]
        if z <= 0 or vz >= 0:
            return float('inf')  # Not diving or already at ground
        # Constant-vz estimate: TTI = -z / vz
        tti_const = -z / vz
        # With acceleration correction: z + vz*t + 0.5*az*t² = 0
        az = self.x[2]
        if abs(az) > 0.1:
            disc = vz**2 - 2 * az * z
            if disc >= 0:
                t1 = (-vz - np.sqrt(disc)) / az
                t2 = (-vz + np.sqrt(disc)) / az
                candidates = [t for t in [t1, t2] if t > 0]
                if candidates:
                    return float(min(candidates))
        return float(max(0.1, tti_const))


# =============================================================================
# NX-MIMOSA v4.0/4.1 SENTINEL — Main Tracker
# =============================================================================
class NxMimosaV40Sentinel:
    def __init__(self, dt=0.1, r_std=2.5, platform_db_path=None,
                 window_size=30, prune_threshold=0.03,
                 initial_models=None, q_base=1.0):
        self.dt = dt; self.r_std = r_std
        self.R_nominal = np.eye(2)*r_std**2   # [REQ-V41-07] Nominal R (never modified)
        self.R = self.R_nominal.copy()          # Active R (ECM-boosted during jamming)
        self.R_ecm_scale = 1.0                  # Current R boost factor
        self.window_size = window_size
        self.prune_threshold = prune_threshold
        self.q_base = q_base; self.q_scale = 1.0
        self.platform_db = self._load_db(platform_db_path)
        self.active_models = list(initial_models or ["CV","CT_plus","CT_minus"])
        self.x = {m: np.zeros(MODEL_DIMS[m]) for m in ALL_MODELS}
        self.P = {m: np.eye(MODEL_DIMS[m])*100 for m in ALL_MODELS}
        self.mu = {m: 1/len(self.active_models) for m in self.active_models}
        self.p_stay = 0.88; self.tpm = {}; self._rebuild_tpm()
        self.omega = 0.196
        self.identifier = PlatformIdentifier(self.platform_db)
        self.intent_pred = IntentPredictor()
        self.intent_state = IntentState()
        
        # [REQ-V41-01..06] v4.1 Multi-domain classifier pipeline
        self._cls_result = None  # Last ClassificationResult from pipeline
        self._ecm_q_scale = 1.0  # ECM-driven Q multiplier
        # [REQ-V41-07] ECM-adaptive R matrix
        self._ecm_r_scale = 1.0  # R boost factor (1.0 = nominal, up to 20x under jamming)
        self._ecm_high_conf = False  # High-confidence ECM → force model activation
        # [REQ-V41-08] Ghost track detection (multi-track explosion indicator)
        self._ghost_track_count = 0  # Set externally by tracker manager
        self._ecm_force_models = False  # True → override VS-IMM with CA/Jerk
        self._ecm_gate_nis = float('inf')  # NIS gating threshold (DRFM/chaff)
        # [REQ-V42-01] GUARDIAN: Innovation Bias Detector (RGPO/VGPO defense)
        self._bias_detector = InnovationBiasDetector(
            window=20, bias_sigma=3.5, max_consecutive_rejects=15, warmup_steps=30)
        self._measurement_rejected = False  # True when GUARDIAN rejects measurement
        self._guardian_active = False       # True when bias detection is armed + ECM active
        # RF observable cache (set via set_rf_observables() before each update)
        self._rf_snr_db = 25.0
        self._rf_rcs_dbsm = 0.0
        self._rf_doppler_hz = 0.0
        # [REQ-V41-11] Parallel z-axis altitude filter
        self.z_filter = ParallelZFilter(dt=dt, r_z=50.0, q_z=0.1)
        self._z_meas = None  # Set via set_altitude() or feed_measurement_3d()
        if _HAS_CLASSIFIER:
            # Resolve DB path for v3 (multi-domain, 111 platforms)
            _db_v3_candidates = [
                'data/platform_db_v3.json',
                os.path.join(os.path.dirname(os.path.abspath(__file__)),'..','data','platform_db_v3.json'),
                os.path.join(os.path.dirname(os.path.abspath(__file__)),'data','platform_db_v3.json'),
            ]
            _db_v3_path = next((p for p in _db_v3_candidates if os.path.exists(p)), None)
            if _db_v3_path:
                self.cls_pipeline = NxMimosaClassifierPipeline(
                    db_path=_db_v3_path, dt=dt, history_len=window_size)
            else:
                self.cls_pipeline = None
        else:
            self.cls_pipeline = None
        # Per-model history for RTS
        self.xf_h = []; self.Pf_h = []; self.xp_h = []; self.Pp_h = []
        self.F_h = []; self.mu_h = []; self.act_h = []; self.xc_h = []
        # v4.0.2: Pre-mixing per-model states for correct RTS backward pass
        self.xu_h = []; self.Pu_h = []
        self.nis_avg = 2.0  # Running average NIS (expected = dim_z = 2)
        self.dv_ema = 0.0   # EMA of velocity change rate (m/s per step)
        self.benign_streak = 0  # Consecutive benign steps
        self.q_base_initial = q_base
        self.step = 0; self.initialized = False
        
        # =====================================================================
        # Parallel independent filters (run on raw measurements, zero IMM noise)
        # [REQ-V40-10] CV filter: matches Stone Soup CV forward/RTS
        # [REQ-V40-11] CA filter: matches Stone Soup CA forward/RTS
        # =====================================================================
        # --- Parallel CV filter (q=0.5, continuous white noise model) ---
        self._cv_F = np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]])
        self._cv_H = np.array([[1,0,0,0],[0,1,0,0]], dtype=float)
        # Use Stone Soup's continuous white noise acceleration model:
        # Q per axis = q * [[dt³/3, dt²/2], [dt²/2, dt]]
        q_cv = 0.5  # Same as Stone Soup ConstantVelocity(0.5)
        self._cv_Q = np.zeros((4,4))
        for i in [0,1]:  # x and y axes
            self._cv_Q[i,i] = q_cv * dt**3 / 3
            self._cv_Q[i,i+2] = q_cv * dt**2 / 2
            self._cv_Q[i+2,i] = q_cv * dt**2 / 2
            self._cv_Q[i+2,i+2] = q_cv * dt
        self._cv_x = np.zeros(4)
        self._cv_P = np.eye(4) * 100
        self._cv_nis = 2.0  # Running NIS for CV filter
        self._cv_h = []; self._cv_xf_h = []; self._cv_Pf_h = []
        
        # --- Parallel CA filter (q=2.0, continuous white noise jerk model) ---
        # State: [x, y, vx, vy, ax, ay]
        self._ca_F = np.eye(6)
        self._ca_F[0,2]=dt; self._ca_F[0,4]=dt**2/2
        self._ca_F[1,3]=dt; self._ca_F[1,5]=dt**2/2
        self._ca_F[2,4]=dt; self._ca_F[3,5]=dt
        self._ca_H = np.zeros((2,6)); self._ca_H[0,0]=1; self._ca_H[1,1]=1
        q_ca = 2.0  # Same as Stone Soup ConstantAcceleration(2.0)
        self._ca_Q = np.zeros((6,6))
        for i in [0,1]:  # x and y axes
            p,v,a = i, i+2, i+4
            self._ca_Q[p,p] = q_ca * dt**5 / 20
            self._ca_Q[p,v] = q_ca * dt**4 / 8;  self._ca_Q[v,p] = self._ca_Q[p,v]
            self._ca_Q[p,a] = q_ca * dt**3 / 6;  self._ca_Q[a,p] = self._ca_Q[p,a]
            self._ca_Q[v,v] = q_ca * dt**3 / 3
            self._ca_Q[v,a] = q_ca * dt**2 / 2;  self._ca_Q[a,v] = self._ca_Q[v,a]
            self._ca_Q[a,a] = q_ca * dt
        self._ca_x = np.zeros(6)
        self._ca_P = np.eye(6) * 100
        self._ca_h = []; self._ca_xf_h = []; self._ca_Pf_h = []
        
        self._z_h = []  # Raw measurement history

    def _load_db(self, path):
        for p in ([path] if path else []) + [
            'data/platform_db.json',
            os.path.join(os.path.dirname(os.path.abspath(__file__)),'..','data','platform_db.json'),
            os.path.join(os.path.dirname(os.path.abspath(__file__)),'data','platform_db.json')]:
            if p and os.path.exists(p):
                with open(p) as f: return json.load(f)
        return {"Unknown":{"category":"unknown","max_speed_mps":3500,"cruise_speed_mps":500,
                "max_g":30,"sustained_g":15,"preferred_models":ALL_MODELS,
                "initial_tpm_bias":{m:1/6 for m in ALL_MODELS},
                "intent_phases":{"unknown":{"prob":1,"models":ALL_MODELS,"q_scale":1}}}}

    def _rebuild_tpm(self):
        n = len(self.active_models)
        if n<=1: self.tpm = {m:{m:1} for m in self.active_models}; return
        ps = (1-self.p_stay)/(n-1)
        self.tpm = {mi:{mj:(self.p_stay if mi==mj else ps) for mj in self.active_models} for mi in self.active_models}

    def _get_FQH(self, m):
        q = self.q_base * self.q_scale; w = self.omega
        if m=="CV": return _build_cv(self.dt, q)
        if m=="CT_plus": return _build_ct(self.dt, abs(w), q)
        if m=="CT_minus": return _build_ct(self.dt, -abs(w), q)
        if m=="CA": return _build_ca(self.dt, q)
        if m=="Jerk": return _build_jerk(self.dt, q)
        if m=="Ballistic": return _build_ballistic(self.dt, q)

    def _ms(self, x, ns, nd):
        o = np.zeros(nd); d = min(ns,nd); o[:d] = x[:d]; return o

    def _mc(self, P, ns, nd):
        o = np.eye(nd)*10; d = min(ns,nd); o[:d,:d] = P[:d,:d]; return o

    def initialize(self, z):
        for m in ALL_MODELS:
            nx = MODEL_DIMS[m]; self.x[m] = np.zeros(nx); self.x[m][:2] = z[:2]
            self.P[m] = np.eye(nx)*100
        # Init parallel CV filter
        self._cv_x = np.array([z[0], z[1], 0, 0])
        self._cv_P = np.eye(4) * 100
        self._cv_h.append(z[:2].copy())
        self._cv_xf_h.append(self._cv_x.copy())
        self._cv_Pf_h.append(self._cv_P.copy())
        # Init parallel CA filter
        self._ca_x = np.array([z[0], z[1], 0, 0, 0, 0])
        self._ca_P = np.eye(6) * 100
        self._ca_h.append(z[:2].copy())
        self._ca_xf_h.append(self._ca_x.copy())
        self._ca_Pf_h.append(self._ca_P.copy())
        self._z_h.append(z[:2].copy())
        self.initialized = True; self.step = 0

    def update(self, z):
        if not self.initialized:
            self.initialize(z)
            self.intent_state.active_models = list(self.active_models)
            # Append init state to IMM histories for consistency with parallel filters
            xc0 = np.array([z[0], z[1], 0, 0])
            self.xc_h.append(xc0.copy())
            xfs0 = {m: self.x[m].copy() for m in self.active_models}
            Pfs0 = {m: self.P[m].copy() for m in self.active_models}
            self.xf_h.append(xfs0); self.Pf_h.append(Pfs0)
            self.xp_h.append(xfs0); self.Pp_h.append(Pfs0)
            self.xu_h.append(xfs0); self.Pu_h.append(Pfs0)  # No mixing at init
            self.F_h.append({m: np.eye(MODEL_DIMS[m]) for m in self.active_models})
            self.mu_h.append(dict(self.mu))
            self.act_h.append(list(self.active_models))
            return z.copy(), np.eye(2)*self.r_std**2, self.intent_state
        self.step += 1

        # 0. Store UNMIXED per-model states (v4.0.2: for correct Full-RTS)
        # These are the pure per-model filtered states BEFORE IMM mixing contaminates them
        xu_pre = {m: self.x[m].copy() for m in self.active_models}
        Pu_pre = {m: self.P[m].copy() for m in self.active_models}

        # 1. IMM Mixing
        mx, mP = {}, {}
        for mj in self.active_models:
            nj = MODEL_DIMS[mj]
            cj = max(sum(self.tpm[mi][mj]*self.mu[mi] for mi in self.active_models), 1e-30)
            xm = np.zeros(nj)
            for mi in self.active_models:
                mu_ij = self.tpm[mi][mj]*self.mu[mi]/cj
                xm += mu_ij * self._ms(self.x[mi], MODEL_DIMS[mi], nj)
            Pm = np.zeros((nj,nj))
            for mi in self.active_models:
                mu_ij = self.tpm[mi][mj]*self.mu[mi]/cj
                xi = self._ms(self.x[mi], MODEL_DIMS[mi], nj)
                Pi = self._mc(self.P[mi], MODEL_DIMS[mi], nj)
                dx = xi - xm; Pm += mu_ij*(Pi + np.outer(dx,dx))
            mx[mj]=xm; mP[mj]=Pm

        # 2. Predict + Update (with GUARDIAN measurement rejection)
        liks = {}; Fu = {}; xps = {}; Pps = {}; xfs = {}; Pfs = {}; nis_models = {}
        nu_models = {}; S_models = {}
        
        # Phase 1: PREDICT all models, compute innovations
        for m in self.active_models:
            nx = MODEL_DIMS[m]; F,Q,H,_ = self._get_FQH(m)
            xp = F @ mx[m]; Pp = F @ mP[m] @ F.T + Q
            if m=="Ballistic": xp[3] -= GRAVITY*self.dt
            Fu[m]=F.copy(); xps[m]=xp.copy(); Pps[m]=Pp.copy()
            nu = z - H@xp; S = H@Pp@H.T + self.R; Si = safe_inv(S)
            nis_val = float(nu @ Si @ nu)
            nis_models[m] = nis_val
            nu_models[m] = nu.copy()
            S_models[m] = S.copy()
        
        # Phase 2: GUARDIAN — Innovation Bias Detection [REQ-V42-01]
        # Use CV model innovation as reference (most stable, always active)
        ref_model = "CV" if "CV" in nu_models else next(iter(nu_models))
        ref_nu = nu_models[ref_model]
        ref_S = S_models[ref_model]
        ref_nis = nis_models[ref_model]
        
        ecm_is_active = (self._ecm_r_scale > 1.5 or 
                         self._ghost_track_count > 2 or
                         self._ecm_high_conf)
        
        guardian_reject = self._bias_detector.update(
            nu=ref_nu, S=ref_S, nis=ref_nis,
            omega=self.omega, ecm_active=ecm_is_active
        )
        self._measurement_rejected = guardian_reject
        self._guardian_active = self._bias_detector._armed and ecm_is_active
        
        # Phase 3: UPDATE — apply decision per model
        for m in self.active_models:
            nx = MODEL_DIMS[m]; _,_,H,_ = self._get_FQH(m)
            xp = xps[m]; Pp = Pps[m]
            nu = nu_models[m]; S = S_models[m]; Si = safe_inv(S)
            nis_val = nis_models[m]
            
            # ── Decision: REJECT or ACCEPT measurement? ─────────────
            reject_this = False
            
            # A) GUARDIAN rejection (bias detected → all models predict-only)
            if guardian_reject:
                reject_this = True
            
            # B) NIS gating (per-model, for DRFM/chaff false returns)
            if not reject_this:
                omega_for_gate = abs(self.omega)
                effective_gate = self._ecm_gate_nis
                if omega_for_gate > 0.1:
                    gate_relax = 1.0 + (omega_for_gate - 0.1) / 0.2 * 9.0
                    effective_gate = min(effective_gate * gate_relax, 1e6)
                if nis_val > effective_gate:
                    reject_this = True
            
            if reject_this:
                # REJECTED: Use prediction only — do NOT incorporate measurement
                xf = xp.copy(); Pf = Pp.copy()
                liks[m] = 1e-300  # Suppress likelihood (no measurement info)
            else:
                # ACCEPTED: Normal Kalman update
                sn,ld = np.linalg.slogdet(S)
                liks[m] = max(np.exp(-0.5*2*np.log(2*np.pi)-0.5*ld-0.5*nis_val), 1e-300) if sn>0 else 1e-300
                K = Pp@H.T@Si; xf = xp+K@nu
                IKH = np.eye(nx)-K@H; Pf = IKH@Pp@IKH.T + K@self.R@K.T
            self.x[m]=xf; self.P[m]=Pf; xfs[m]=xf.copy(); Pfs[m]=Pf.copy()

        # 2b. Dynamics-gated adaptive q_base
        # Track velocity change rate and NIS to detect benign segments
        nis_w = sum(self.mu.get(m,0) * nis_models.get(m,2) for m in self.active_models)
        self.nis_avg = 0.85 * self.nis_avg + 0.15 * nis_w
        
        # Velocity change rate (computed from filter estimates)
        if self.xc_h:
            # Use predicted velocity change from last step
            prev_v = self.xc_h[-1][2:4]
            # We don't have xc yet, use weighted filter estimate
            v_now = np.zeros(2)
            for m in self.active_models:
                v_now += self.mu[m] * self.x[m][2:4]
            dv = np.linalg.norm(v_now - prev_v) / self.dt  # acceleration m/s²
            self.dv_ema = 0.7 * self.dv_ema + 0.3 * dv
        
        if self.step > 15:
            # Benign detection: low dynamics AND low NIS
            speed_est = np.linalg.norm(v_now) if self.xc_h else 100.0
            dv_thresh = max(3.0, speed_est * 0.008)  # 0.8% of speed or 3 m/s²
            
            if self.dv_ema < dv_thresh and self.nis_avg < 5.0:
                self.benign_streak += 1
            else:
                self.benign_streak = max(0, self.benign_streak - 5)  # Fast exit
            
            # Progressive q_base reduction during benign flight
            if self.benign_streak > 25:
                target_q = 0.10  # Ultra-low: pure CV tracking
                self.q_base = max(target_q, self.q_base * 0.88)
            elif self.benign_streak > 15:
                target_q = 0.20  # Low: gentle turns only
                decay = 0.92
                self.q_base = max(target_q, self.q_base * decay)
            elif self.benign_streak > 8:
                target_q = 0.45
                self.q_base = max(target_q, self.q_base * 0.95)
            else:
                # Recover q_base when dynamics return
                self.q_base = min(self.q_base_initial, self.q_base * 1.08 + 0.02)

        # 3. Model prob update
        cb = {mj: sum(self.tpm[mi][mj]*self.mu[mi] for mi in self.active_models) for mj in self.active_models}
        tot = max(sum(liks[m]*cb[m] for m in self.active_models), 1e-300)
        for m in self.active_models: self.mu[m] = max(liks[m]*cb[m]/tot, 1e-30)
        ms = sum(self.mu.values())
        for m in self.active_models: self.mu[m] /= ms
        
        # Adaptive p_stay: dominant model → higher stay, benign mode → much higher
        max_mu = max(self.mu.values())
        if self.benign_streak > 20:
            self.p_stay = 0.97  # Lock models during benign flight
        elif max_mu > 0.75:
            self.p_stay = min(0.95, 0.88 + 0.1 * (max_mu - 0.75) / 0.25)
        else:
            self.p_stay = 0.88
        self._rebuild_tpm()

        # 4. Combined estimate with dominant-model bypass
        # When one model strongly dominates (>92%), use its estimate directly
        # to avoid IMM mixing noise from low-weight models
        max_model = max(self.active_models, key=lambda m: self.mu[m])
        max_mu = self.mu[max_model]
        
        if max_mu > 0.80:
            # Dominant model bypass — pure estimate, no mixing noise
            xc = self.x[max_model][:4].copy()
            Pc = self.P[max_model][:4,:4].copy() if MODEL_DIMS[max_model]>=4 else np.eye(4)*10
        else:
            xc = np.zeros(4)
            for m in self.active_models: xc += self.mu[m]*self.x[m][:4]
            Pc = np.zeros((4,4))
            for m in self.active_models:
                xi = self.x[m][:4]; Pi = self.P[m][:4,:4] if MODEL_DIMS[m]>=4 else np.eye(4)*10
                dx = xi-xc; Pc += self.mu[m]*(Pi+np.outer(dx,dx))

        # 5. Adaptive omega
        if self.xc_h:
            vx,vy = xc[2],xc[3]; pv = self.xc_h[-1]
            sp = np.sqrt(vx**2+vy**2); psp = np.sqrt(pv[2]**2+pv[3]**2)
            if sp>10 and psp>10:
                cr = vx*pv[3]-vy*pv[2]
                self.omega = 0.3*cr/(sp*psp*self.dt+1e-10)+0.7*self.omega

        # 6. Platform ID + VS-IMM adapt (only after initial transient)
        # Compute instantaneous CV NIS for classifier (before filter update)
        _xp_cv = self._cv_F @ self._cv_x
        _Pp_cv = self._cv_F @ self._cv_P @ self._cv_F.T + self._cv_Q
        _nu_cv = z - self._cv_H @ _xp_cv
        _S_cv = self._cv_H @ _Pp_cv @ self._cv_H.T + self.R
        _Si_cv = safe_inv(_S_cv)
        cv_nis_inst = float(_nu_cv @ _Si_cv @ _nu_cv)
        
        # v4.0 legacy classifier (still used for VS-IMM adapt via old platform_db)
        plat,conf = self.identifier.update(xc[:2], xc[2:4], self.dt,
                                            mu=self.mu, cv_nis=cv_nis_inst,
                                            omega=self.omega)
        pd = self.platform_db.get(plat, self.platform_db.get("Unknown",{}))
        if conf>=0.35 and plat!="Unknown" and self.step > 20: self._vs_adapt(pd)
        
        # =====================================================================
        # [REQ-V41-01..06] v4.1 Multi-domain classifier + intent + ECM
        # Feeds: speed, g-load, altitude, omega, heading, vz, NIS, RCS, SNR
        # Returns: ClassificationResult with intent, ECM status, threat, alerts
        # =====================================================================
        sp = np.linalg.norm(xc[2:4])
        ag = 0.0
        if self.xc_h:
            ag = np.linalg.norm(xc[2:4] - self.xc_h[-1][2:4]) / (self.dt * GRAVITY + 1e-10)
        
        # Extract heading from velocity vector
        heading_rad = math.atan2(xc[3], xc[2]) if sp > 5 else 0.0
        
        # [REQ-V41-11] Altitude from parallel z-filter (filtered, not raw)
        z_meas = self._z_meas if self._z_meas is not None else float('nan')
        z_est, vz_est, az_est = self.z_filter.update(z_meas, self._ecm_r_scale)
        alt_m = max(0.0, z_est)  # Filtered altitude (m AGL)
        vz_mps = vz_est          # Filtered vertical velocity (m/s)
        self._z_meas = None      # Consume measurement (require fresh each cycle)
        
        if self.cls_pipeline is not None and self.step > 5:
            # Full v4.1 pipeline: classifier + intent + ECM
            cls_result = self.cls_pipeline.update(
                speed_mps=sp, accel_g=ag, altitude_m=alt_m,
                omega_radps=abs(self.omega), heading_rad=heading_rad,
                vz_mps=vz_mps, nis_cv=cv_nis_inst,
                rcs_dbsm=self._rf_rcs_dbsm,
                snr_db=self._rf_snr_db,
                doppler_hz=self._rf_doppler_hz,
                range_m=np.linalg.norm(xc[:2]),
            )
            self._cls_result = cls_result
            
            # [REQ-V41-03] ECM-driven Q boost
            self._ecm_q_scale = cls_result.ecm_status != ECMStatus.CLEAN
            if hasattr(self.cls_pipeline, 'ecm_detector'):
                self._ecm_q_scale = self.cls_pipeline.ecm_detector.q_scale_factor
            
            # =================================================================
            # [REQ-V41-07] ECM-Adaptive R Matrix Boost
            # -----------------------------------------------------------------
            # Under ECM, measurement noise INCREASES — filter must trust
            # prediction more and measurement less. R boost is ECM-type-aware:
            #   - Noise jamming → boost both range + angle (broadband corruption)
            #   - RGPO/deception → boost range only (range gate pull-off)
            #   - VGPO → boost Doppler/velocity only
            #   - DRFM → moderate all-channel boost (smart repeater)
            #   - Chaff → moderate boost (distributed false returns)
            # Smooth transitions: fast ramp-up, slow decay (same as Q).
            # =================================================================
            ecm_det = self.cls_pipeline.ecm_detector if hasattr(self.cls_pipeline, 'ecm_detector') else None
            if ecm_det and hasattr(ecm_det, 'status'):
                ecm_status = ecm_det.status
                ecm_conf = ecm_det.confidence if hasattr(ecm_det, 'confidence') else 0.0
            else:
                ecm_status = cls_result.ecm_status
                ecm_conf = 0.0
            
            # Compute target R scale from ECM type + confidence
            # =============================================================
            # STRATEGY (validated by ECM benchmark):
            #   DECEPTION (RGPO/VGPO): Heavy R-boost → coast on prediction
            #     Measurement is BIASED (systematic error) → ignore it
            #   NOISE JAMMING: R-RECALIBRATE to actual noise level
            #     Measurement is unbiased but noisy → match R to real noise
            #     Formula: R_scale ≈ 10^(SNR_drop / 10) (power ratio)
            #   DRFM: Moderate R-boost + measurement GATING
            #     50% of measurements are false → gate on NIS
            #   CHAFF: Light recalibration + gating
            #     60% good, 40% chaff → moderate approach
            # =============================================================
            if _HAS_CLASSIFIER:
                snr_measured = self._rf_snr_db
                snr_nominal = 25.0  # Assumed baseline SNR
                
                if ecm_status == ECMStatus.CLEAN:
                    target_r_scale = 1.0
                    self._ecm_gate_nis = float('inf')  # No gating
                    
                elif ecm_status in (ECMStatus.DECEPTION_RGPO, ECMStatus.DECEPTION_VGPO):
                    # DECEPTION: R-boost + GUARDIAN innovation bias rejection
                    # R-boost reduces gain, GUARDIAN rejects biased measurements
                    # NIS gating as backup: if innovation > 15σ, reject outright
                    target_r_scale = 8.0 + 12.0 * min(ecm_conf, 1.0)   # 8-20x
                    self._ecm_gate_nis = 15.0  # Tight NIS gate for extreme outliers
                    
                elif ecm_status in (ECMStatus.NOISE_JAMMING, ECMStatus.MULTI_SOURCE):
                    # NOISE: Recalibrate R to match actual noise power
                    # SNR drop in dB → power ratio for R scaling
                    snr_drop = max(0, snr_nominal - snr_measured)
                    # R scales with noise power: 10^(dB/10), capped
                    target_r_scale = min(25.0, 10 ** (snr_drop / 10.0))
                    # Minimum 2x if ECM detected at all
                    target_r_scale = max(2.0, target_r_scale) if snr_drop > 3 else 1.0
                    self._ecm_gate_nis = float('inf')  # No gating for noise
                    
                elif ecm_status == ECMStatus.DRFM_REPEATER:
                    # DRFM: bimodal measurements (true + false echo).
                    # Gating and R-boost are COUNTERPRODUCTIVE — they cause coast
                    # which drifts during maneuvers. Kalman averaging of bimodal
                    # distribution gives ~mean(true, false) which is moderate error.
                    # Proper solution: MHT/PDA (future work, v5.0)
                    target_r_scale = 1.0   # Don't boost R
                    self._ecm_gate_nis = float('inf')  # Don't gate
                    
                elif ecm_status == ECMStatus.CHAFF_CORRIDOR:
                    # CHAFF: multimodal measurements (60% true + 40% chaff cloud).
                    # Same reasoning as DRFM: Kalman averaging is better than
                    # gating/coasting. Q-boost (from ECM classifier) handles
                    # the increased innovation variance.
                    target_r_scale = 1.0   # Don't boost R
                    self._ecm_gate_nis = float('inf')  # Don't gate
                    
                else:
                    target_r_scale = 1.5
                    self._ecm_gate_nis = float('inf')
            else:
                target_r_scale = 1.0
            
            # Ghost track escalation: gating + moderate R-boost (not massive)
            if self._ghost_track_count > 3:
                target_r_scale = max(target_r_scale, 5.0)    # Moderate R-boost
                self._ecm_gate_nis = min(self._ecm_gate_nis, 10.0)  # Tight gating
                ecm_conf = max(ecm_conf, 0.7)
            
            # Asymmetric smoothing: fast up (0.5), slow down (0.92)
            if target_r_scale > self._ecm_r_scale:
                self._ecm_r_scale = 0.5 * self._ecm_r_scale + 0.5 * target_r_scale
            else:
                self._ecm_r_scale = 0.92 * self._ecm_r_scale + 0.08 * target_r_scale
            self._ecm_r_scale = max(1.0, min(self._ecm_r_scale, 25.0))  # Clamp [1, 25]
            
            # =============================================================
            # [REQ-V41-13] Maneuver-Aware R Modulation
            # ---------------------------------------------------------
            # R-boost is counterproductive when target is MANEUVERING
            # because prediction drifts (wrong direction). Under high
            # dynamics, we need measurements even if noisy/corrupted.
            # Strategy: scale R-boost inversely with maneuver intensity.
            #   omega < 0.1 → full R-boost (coasting is safe)
            #   omega > 0.3 → minimal R-boost (need measurements)
            # =============================================================
            omega_abs = abs(self.omega)
            if omega_abs > 0.05:
                # Maneuver detected: reduce R-boost
                # Linear ramp: omega=0.05→1.0x, omega=0.3→0.2x
                maneuver_factor = max(0.2, 1.0 - (omega_abs - 0.05) / 0.25 * 0.8)
                # Blend: R_scale moves toward 1.0 as maneuver intensifies
                self._ecm_r_scale = 1.0 + (self._ecm_r_scale - 1.0) * maneuver_factor
            
            # Apply to active R matrix
            self.R = self.R_nominal * self._ecm_r_scale
            
            # =================================================================
            # [REQ-V41-08] ECM Forced Model Activation
            # -----------------------------------------------------------------
            # Under high-confidence ECM, the classifier may give wrong platform
            # → wrong preferred_models. Override: force-activate CA + Jerk
            # while KEEPING CV (most stable under measurement corruption).
            # This ensures the filter can handle both benign (CV) and high-g
            # (CA/Jerk) trajectories when we can't trust classification.
            # =================================================================
            self._ecm_high_conf = (self._ecm_r_scale > 5.0 or ecm_conf > 0.6)
            
            if self._ecm_high_conf:
                # Force-activate higher-order models alongside CV
                ecm_models = ["CV", "CA", "CT_plus", "CT_minus", "Jerk"]
                ecm_models = [m for m in ecm_models if m in ALL_MODELS]
                if set(ecm_models) != set(self.active_models):
                    old_mu = dict(self.mu)
                    self.active_models = ecm_models
                    n = len(ecm_models)
                    # Redistribute: give extra weight to CA/Jerk under ECM
                    self.mu = {}
                    for m in ecm_models:
                        if m in ("CA", "Jerk"):
                            self.mu[m] = max(old_mu.get(m, 0), 0.15)
                        else:
                            self.mu[m] = max(old_mu.get(m, 0), 0.05)
                    t = sum(self.mu.values())
                    self.mu = {m: p/t for m, p in self.mu.items()}
                    # Initialize new models from best existing
                    bm = max(old_mu, key=old_mu.get) if old_mu else "CV"
                    for m in ecm_models:
                        if old_mu.get(m, 0) < 0.01:
                            self.x[m] = self._ms(self.x[bm], MODEL_DIMS[bm], MODEL_DIMS[m])
                            self.P[m] = self._mc(self.P[bm], MODEL_DIMS[bm], MODEL_DIMS[m])
                    self._rebuild_tpm()
                    self.benign_streak = 0
                    self._ecm_force_models = True
            else:
                self._ecm_force_models = False
            
            # [REQ-V41-01] VS-IMM adapt from v4.1 classifier preferred_models
            # Only apply classifier-driven model selection when NOT in ECM override
            if (cls_result.confidence >= 0.50 and self.step > 20
                    and not self._ecm_force_models):
                pref_v41 = cls_result.preferred_models
                if pref_v41 and set(pref_v41) != set(self.active_models):
                    # Merge: v4.1 classifier overrides when confident
                    na = list(set(["CV"] + pref_v41))
                    na = [m for m in na if m in ALL_MODELS]
                    if na != self.active_models:
                        old_mu = dict(self.mu)
                        self.active_models = na
                        t = sum(old_mu.get(m, 0) for m in na)
                        n = len(na)
                        self.mu = {m: old_mu.get(m, 0)/t for m in na} if t > 0.01 else {m: 1/n for m in na}
                        bm = max(old_mu, key=old_mu.get) if old_mu else "CV"
                        for m in na:
                            if old_mu.get(m, 0) < 0.01:
                                self.x[m] = self._ms(self.x[bm], MODEL_DIMS[bm], MODEL_DIMS[m])
                                self.P[m] = self._mc(self.P[bm], MODEL_DIMS[bm], MODEL_DIMS[m])
                        self._rebuild_tpm()
        else:
            cls_result = None
            # Decay R scale back to nominal when no pipeline
            self._ecm_r_scale = 0.92 * self._ecm_r_scale + 0.08 * 1.0
            if self._ecm_r_scale < 1.05:
                self._ecm_r_scale = 1.0
            self.R = self.R_nominal * self._ecm_r_scale
            self._ecm_gate_nis = float('inf')  # No gating without pipeline
        
        # 6b. Emergency reactivation: if CV-only but velocity change is large
        if len(self.active_models) == 1 and self.active_models[0] == "CV":
            if self.xc_h:
                dv = np.linalg.norm(xc[2:4] - self.xc_h[-1][2:4])
                if dv > 15 * self.dt:  # Significant acceleration detected
                    self.active_models = ["CV", "CT_plus", "CT_minus"]
                    self.mu = {"CV": 0.6, "CT_plus": 0.2, "CT_minus": 0.2}
                    for m in ["CT_plus", "CT_minus"]:
                        self.x[m] = self._ms(self.x["CV"], 4, MODEL_DIMS[m])
                        self.P[m] = self._mc(self.P["CV"], 4, MODEL_DIMS[m])
                    self._rebuild_tpm()
                    self.benign_streak = 0
                    self.q_base = self.q_base_initial  # Full reset

        # 7. Intent + Q scale
        phase,pc = self.intent_pred.predict(pd, sp, ag, self.mu)
        raw_q_scale = pd.get("intent_phases",{}).get(phase,{}).get("q_scale",1.0)
        
        # [REQ-V41-03] ECM Q boost: multiply intent-based Q by ECM factor
        ecm_q_factor = self._ecm_q_scale if isinstance(self._ecm_q_scale, (int, float)) else 1.0
        raw_q_scale = raw_q_scale * max(1.0, ecm_q_factor)
        
        # Cap q_scale and smooth transitions asymmetrically
        # Fast ramp-up (maneuver detected), slow ramp-down (benign detected)
        if self.step < 20:
            self.q_scale = 1.0
        else:
            target_qs = min(raw_q_scale, 5.0)  # v4.1: allow up to 5x under ECM
            if target_qs > self.q_scale:
                self.q_scale = 0.4 * self.q_scale + 0.6 * target_qs  # Fast up
            else:
                self.q_scale = 0.85 * self.q_scale + 0.15 * target_qs  # Slow down
        
        # Adaptive prune threshold: benign phases → kill off unnecessary models
        if self.benign_streak > 15:
            self.prune_threshold = 0.20  # Benign: prune to 2 models fast
        elif self.q_scale < 0.3:
            self.prune_threshold = 0.15
        elif self.q_scale < 0.6:
            self.prune_threshold = 0.08
        else:
            self.prune_threshold = 0.03

        # 8. Prune
        self._prune()

        # 8b. Parallel CV Kalman filter update (independent of IMM)
        # [REQ-V40-10] Runs on raw measurement z, not IMM output
        xp_cv = self._cv_F @ self._cv_x
        Pp_cv = self._cv_F @ self._cv_P @ self._cv_F.T + self._cv_Q
        nu_cv = z - self._cv_H @ xp_cv
        S_cv = self._cv_H @ Pp_cv @ self._cv_H.T + self.R
        Si_cv = safe_inv(S_cv)
        cv_nis_raw = float(nu_cv @ Si_cv @ nu_cv)
        self._cv_nis = 0.85 * self._cv_nis + 0.15 * cv_nis_raw
        # [REQ-V41-12] Measurement gating for parallel CV
        _eff_gate_cv = self._ecm_gate_nis
        if abs(self.omega) > 0.1:
            _eff_gate_cv = min(_eff_gate_cv * (1.0 + (abs(self.omega)-0.1)/0.2*9.0), 1e6)
        if cv_nis_raw <= _eff_gate_cv:
            K_cv = Pp_cv @ self._cv_H.T @ Si_cv
            self._cv_x = xp_cv + K_cv @ nu_cv
            IKH_cv = np.eye(4) - K_cv @ self._cv_H
            self._cv_P = IKH_cv @ Pp_cv @ IKH_cv.T + K_cv @ self.R @ K_cv.T
        else:
            self._cv_x = xp_cv; self._cv_P = Pp_cv  # Predict-only (gated)
        self._cv_h.append(self._cv_x[:2].copy())
        self._cv_xf_h.append(self._cv_x.copy())
        self._cv_Pf_h.append(self._cv_P.copy())
        
        # 8c. Parallel CA Kalman filter update (independent of IMM)
        xp_ca = self._ca_F @ self._ca_x
        Pp_ca = self._ca_F @ self._ca_P @ self._ca_F.T + self._ca_Q
        nu_ca = z - self._ca_H @ xp_ca
        S_ca = self._ca_H @ Pp_ca @ self._ca_H.T + self.R
        Si_ca = safe_inv(S_ca)
        ca_nis_raw = float(nu_ca @ Si_ca @ nu_ca)
        # [REQ-V41-12] Measurement gating for parallel CA
        _eff_gate_ca = self._ecm_gate_nis
        if abs(self.omega) > 0.1:
            _eff_gate_ca = min(_eff_gate_ca * (1.0 + (abs(self.omega)-0.1)/0.2*9.0), 1e6)
        if ca_nis_raw <= _eff_gate_ca:
            K_ca = Pp_ca @ self._ca_H.T @ Si_ca
            self._ca_x = xp_ca + K_ca @ nu_ca
            IKH_ca = np.eye(6) - K_ca @ self._ca_H
            self._ca_P = IKH_ca @ Pp_ca @ IKH_ca.T + K_ca @ self.R @ K_ca.T
        else:
            self._ca_x = xp_ca; self._ca_P = Pp_ca  # Predict-only (gated)
        self._ca_h.append(self._ca_x[:2].copy())
        self._ca_xf_h.append(self._ca_x.copy())
        self._ca_Pf_h.append(self._ca_P.copy())
        
        self._z_h.append(z[:2].copy())

        # 9. Store history
        self.xf_h.append(xfs); self.Pf_h.append(Pfs)
        self.xp_h.append(xps); self.Pp_h.append(Pps)
        self.xu_h.append(xu_pre); self.Pu_h.append(Pu_pre)  # Pre-mixing states
        self.F_h.append(Fu); self.mu_h.append(dict(self.mu))
        self.act_h.append(list(self.active_models)); self.xc_h.append(xc.copy())

        # 10. Build IntentState — merge v4.0 legacy + v4.1 pipeline
        cat = pd.get("category","unknown")
        thr = {"fighter":0.6,"bomber":0.4,"transport":0.1,"aew":0.2,"ballistic_missile":0.9,
               "hypersonic_missile":0.95,"cruise_missile":0.8,"sam":0.85,"mlrs":0.7,
               "small_drone":0.3,"fpv_kamikaze":0.7,"loitering_munition":0.6}.get(cat,0.5)
        if phase in ("terminal","attack","engagement"): thr=min(thr*1.3,1)
        elif phase in ("cruise","patrol","hover"): thr*=0.8
        
        # v4.1 classifier overrides when available
        _plat_class = "unknown"
        _intent = "unknown"
        _intent_conf = 0.0
        _ecm_str = "clean"
        _ecm_q = 1.0
        _dive = 0.0
        _tti = float('inf')
        _alt_rate = vz_mps   # From z-filter (filtered)
        
        # [REQ-V41-11] Use z-filter for dive angle and TTI
        if self.z_filter.step > 3 and sp > 5.0:
            _dive = self.z_filter.get_dive_angle(sp)
            _tti = self.z_filter.get_tti(sp)
        _is_false = False
        _alerts = []
        
        if cls_result is not None:
            _plat_class = cls_result.platform_class
            _intent = cls_result.intent.value if hasattr(cls_result.intent, 'value') else str(cls_result.intent)
            _intent_conf = cls_result.intent_confidence
            _ecm_str = cls_result.ecm_status.value if hasattr(cls_result.ecm_status, 'value') else str(cls_result.ecm_status)
            _ecm_q = ecm_q_factor
            _alerts = list(cls_result.alerts)
            _is_false = _plat_class.startswith("false_target")
            
            # [REQ-V41-07] Add R-scale and ECM force-model alerts
            if self._ecm_r_scale > 3.0:
                _alerts.append(f"ECM_R_BOOST_{self._ecm_r_scale:.0f}x")
            if self._ecm_force_models:
                _alerts.append("ECM_FORCED_MODEL_ACTIVATION")
            if self._ghost_track_count > 3:
                _alerts.append(f"GHOST_TRACKS_{self._ghost_track_count}")
            # [REQ-V42-01] GUARDIAN alerts
            if self._measurement_rejected:
                _alerts.append(f"GUARDIAN_REJECT_BIAS_{self._bias_detector.bias_magnitude:.0f}m")
            if self._bias_detector.reject_count > 5:
                _alerts.append(f"GUARDIAN_COAST_MODE_{self._bias_detector.reject_count}steps")
            
            # Upgrade threat from v4.1 if higher
            if hasattr(cls_result.threat_level, 'value'):
                v41_thr = {0: 0.0, 1: 0.3, 2: 0.5, 3: 0.7, 4: 1.0}.get(
                    cls_result.threat_level.value, 0.5)
                thr = max(thr, v41_thr)
            
            # [REQ-V41-07] ECM escalates threat
            if self._ecm_high_conf:
                thr = max(thr, 0.8)  # At least HIGH threat under confirmed ECM
        
        self.intent_state = IntentState(
            plat, cat, conf, phase, pc,
            pd.get("max_g",30), pd.get("max_speed_mps",3500), thr,
            list(self.active_models), len(self.active_models),
            # v4.1 fields
            platform_class=_plat_class,
            intent=_intent,
            intent_confidence=_intent_conf,
            ecm_status=_ecm_str,
            ecm_q_scale=_ecm_q,
            dive_angle_deg=_dive,
            time_to_impact_s=_tti,
            altitude_rate_mps=_alt_rate,
            is_false_target=_is_false,
            alerts=_alerts,
        )
        return xc[:2], Pc[:2,:2], self.intent_state

    def _vs_adapt(self, pd):
        pref = pd.get("preferred_models", ALL_MODELS[:3])
        na = list(set(["CV"]+pref)); na = [m for m in na if m in ALL_MODELS]
        if set(na)==set(self.active_models):
            self._tpm_bias(pd); return
        old = dict(self.mu); self.active_models = na
        t = sum(old.get(m,0) for m in na); n = len(na)
        self.mu = {m: old.get(m,0)/t for m in na} if t>0.01 else {m:1/n for m in na}
        bm = max(old, key=old.get) if old else "CV"
        for m in na:
            if old.get(m,0)<0.01:
                self.x[m] = self._ms(self.x[bm], MODEL_DIMS[bm], MODEL_DIMS[m])
                self.P[m] = self._mc(self.P[bm], MODEL_DIMS[bm], MODEL_DIMS[m])
        self._rebuild_tpm(); self._tpm_bias(pd)

    def _tpm_bias(self, pd):
        b = pd.get("initial_tpm_bias",{}); n = len(self.active_models)
        if not b or n<=1: return
        for mi in self.active_models:
            t = 0
            for mj in self.active_models:
                self.tpm[mi][mj] = 0.7*self.tpm[mi].get(mj,0)+0.3*b.get(mj,1/n); t+=self.tpm[mi][mj]
            if t>0:
                for mj in self.active_models: self.tpm[mi][mj]/=t

    def _prune(self):
        if len(self.active_models)<=1: return
        # During sustained benign flight, allow collapse to CV-only
        min_models = 1 if self.benign_streak > 30 else 2
        if len(self.active_models)<=min_models: return
        rm = [m for m in self.active_models if self.mu.get(m,0)<self.prune_threshold and m!="CV"]
        for m in rm:
            if len(self.active_models)<=min_models: break
            self.active_models.remove(m); del self.mu[m]
        if rm:
            t = sum(self.mu.values()); self.mu = {m:p/t for m,p in self.mu.items()}
            self._rebuild_tpm()

    # =========================================================================
    # [REQ-V41-07..08] ECM Robustness API
    # =========================================================================
    def set_ghost_track_count(self, count: int):
        """Called by tracker manager when multi-track explosion detected.
        
        [REQ-V41-08] Ghost tracks indicate DRFM/deception ECM generating
        false targets. When count > 3, ECM R-boost is escalated and
        forced model activation triggers.
        
        Args:
            count: Number of suspicious ghost tracks in current scan
        """
        self._ghost_track_count = count
    
    def set_rf_observables(self, snr_db: float = 25.0, rcs_dbsm: float = 0.0,
                           doppler_hz: float = 0.0):
        """Feed live RF observables from radar front-end.
        
        These replace the hardcoded defaults in the classifier pipeline call.
        Call this before update() each cycle for accurate ECM detection.
        
        Args:
            snr_db: Signal-to-noise ratio (dB)
            rcs_dbsm: Radar cross section (dBsm)
            doppler_hz: Doppler frequency (Hz)
        """
        self._rf_snr_db = snr_db
        self._rf_rcs_dbsm = rcs_dbsm
        self._rf_doppler_hz = doppler_hz
    
    # =========================================================================
    # [REQ-V41-11] Altitude / Z-axis API
    # =========================================================================
    def set_altitude(self, altitude_m: float):
        """Feed altitude measurement for parallel z-filter.
        
        Call BEFORE update() each cycle. If not called, z-filter runs
        predict-only (coast) for that cycle — useful during ECM when
        altimeter may be unreliable.
        
        Args:
            altitude_m: Altitude above ground level (m). Must be > 0.
        """
        self._z_meas = altitude_m
    
    def feed_measurement_3d(self, z: np.ndarray):
        """Feed 3D measurement [x, y, altitude].
        
        Convenience method: calls update() with [x,y] for xy-tracker
        and feeds altitude to z-filter in one call.
        
        Args:
            z: 3-element array [x, y, altitude_m]
            
        Returns:
            Same as update() — (position_xy, covariance, intent_state)
        """
        if len(z) < 3:
            return self.update(z[:2])
        self._z_meas = float(z[2])
        return self.update(z[:2])
    
    @property
    def altitude_state(self) -> dict:
        """Current altitude filter state for display / diagnostics.
        
        Returns:
            dict with z_est, vz_est, az_est, dive_angle_deg, z_filter_step
        """
        zf = self.z_filter
        sp = 0.0
        if hasattr(self, 'xc_h') and self.xc_h:
            sp = np.linalg.norm(self.xc_h[-1][2:4])
        return {
            "z_est": zf.altitude,
            "vz_est": zf.vz,
            "az_est": zf.az,
            "dive_angle_deg": zf.get_dive_angle(sp),
            "tti_s": zf.get_tti(sp),
            "z_filter_step": zf.step,
        }
    
    @property
    def ecm_state(self) -> dict:
        """Current ECM state snapshot for operator display / alert.
        
        Returns:
            dict with ecm_active, ecm_r_scale, ecm_q_scale, ecm_force_models,
            ghost_track_count, active_models
        """
        return {
            "ecm_active": self._ecm_r_scale > 2.0 or self._ecm_high_conf,
            "ecm_r_scale": self._ecm_r_scale,
            "ecm_q_scale": self._ecm_q_scale if isinstance(self._ecm_q_scale, (int, float)) else 1.0,
            "ecm_force_models": self._ecm_force_models,
            "ecm_high_conf": self._ecm_high_conf,
            "ecm_gate_nis": self._ecm_gate_nis,
            "ghost_track_count": self._ghost_track_count,
            "active_models": list(self.active_models),
            "r_matrix_diag": float(self.R[0, 0]),
            "r_nominal_diag": float(self.R_nominal[0, 0]),
            # [REQ-V42-01] GUARDIAN measurement rejection state
            "guardian_active": self._guardian_active,
            "guardian_rejecting": self._measurement_rejected,
            "guardian_bias_m": self._bias_detector.bias_magnitude,
            "guardian_bias_sigma": self._bias_detector.bias_sigma_ratio,
            "guardian_reject_streak": self._bias_detector.reject_count,
            "guardian_total_rejections": self._bias_detector.total_rejections,
        }

    # =========================================================================
    # Smoothers — Per-model RTS
    # =========================================================================
    def get_forward_estimates(self): return [x[:2].copy() for x in self.xc_h]

    def get_window_smoothed_estimates(self, window=None):
        """Stream 2: Adaptive window via mu-weighted CV/CA fixed-lag blend.
        
        v4.0.2: Replaced broken per-model IMM RTS (same mixing corruption as
        Full-RTS) with mu-weighted CV/CA fixed-lag RTS blend. Adaptive window
        based on dynamics level.
        """
        W = window or self.window_size
        cv_fl = self.get_cv_fixedlag_rts_estimates(lag=W)
        ca_fl = self.get_ca_fixedlag_rts_estimates(lag=W)
        N = min(len(cv_fl), len(ca_fl))
        if N < 2: return self.get_forward_estimates()
        
        result = [None] * N
        for k in range(N):
            mu_k = self.mu_h[k] if k < len(self.mu_h) else {}
            w_cv = max(mu_k.get("CV", 0.5), 0.05)
            w_ca = max(1.0 - w_cv, 0.05)
            w_total = w_cv + w_ca; w_cv /= w_total; w_ca /= w_total
            result[k] = w_cv * np.array(cv_fl[k]) + w_ca * np.array(ca_fl[k])
        
        return result

    def get_smoothed_estimates(self):
        """Stream 3: Full-track smoothed via mu-weighted CV/CA-RTS blend.
        
        v4.0.2 Architecture Decision:
        IMM per-model RTS is fundamentally broken because IMM mixing at each
        step destroys the Markov chain that RTS requires. Each stored xf_h[k][m]
        is a MIXED state, not a pure model-m trajectory.
        
        Solution: Use parallel CV and CA filters (which have clean, unmixed
        state histories) and blend their RTS outputs using IMM model
        probabilities as weights:
        - CA probability high → more weight on CA-RTS (captures acceleration)
        - CV probability high → more weight on CV-RTS (captures constant vel)
        
        This is mathematically sound AND gives the best offline accuracy.
        """
        cv_rts = self.get_cv_rts_estimates()
        ca_rts = self.get_ca_rts_estimates()
        N = min(len(cv_rts), len(ca_rts))
        if N < 2: return self.get_forward_estimates()
        
        result = [None] * N
        for k in range(N):
            # Use IMM model probabilities to weight CV vs CA
            mu_k = self.mu_h[k] if k < len(self.mu_h) else {}
            
            # CA weight = sum of non-CV model probabilities (CA, CT, Jerk, Ballistic)
            w_cv = mu_k.get("CV", 0.5)
            w_ca = 1.0 - w_cv  # Everything non-CV benefits from acceleration model
            
            # Ensure minimum weight (avoid 0/1 extremes)
            w_cv = max(w_cv, 0.05)
            w_ca = max(w_ca, 0.05)
            w_total = w_cv + w_ca
            w_cv /= w_total
            w_ca /= w_total
            
            result[k] = w_cv * np.array(cv_rts[k]) + w_ca * np.array(ca_rts[k])
        
        return result

    # =========================================================================
    # Parallel CV Filter Outputs
    # =========================================================================
    def get_cv_forward_estimates(self):
        """Stream 5: Parallel CV Kalman forward estimates.
        
        Independent of IMM — runs on raw measurements with constant velocity
        model. No mixing noise. Matches Stone Soup KF-CV performance.
        """
        return [x.copy() for x in self._cv_h]

    def get_cv_rts_estimates(self):
        """Stream 6: Parallel CV + full-track RTS smoother.
        
        OFFLINE: uses all future measurements. Independent of IMM.
        This is functionally equivalent to Stone Soup CV+RTS and should
        match its performance on benign targets (~0.85m on Shahed).
        """
        N = len(self._cv_xf_h)
        if N < 3: return self.get_cv_forward_estimates()
        
        F = self._cv_F; Q = self._cv_Q
        
        # RTS backward pass
        xs = self._cv_xf_h[-1].copy()
        result = [None] * N; result[-1] = xs[:2].copy()
        
        for k in range(N-2, -1, -1):
            xf = self._cv_xf_h[k]; Pf = self._cv_Pf_h[k]
            # Predict from k
            xp1 = F @ xf; Pp1 = F @ Pf @ F.T + Q
            try:
                Ck = Pf @ F.T @ inv(Pp1 + np.eye(4)*EPS)
            except LinAlgError:
                Ck = Pf @ F.T @ pinv(Pp1)
            Ck = np.clip(Ck, -5, 5)  # Stability guard
            xs = xf + Ck @ np.clip(xs - xp1, -5000, 5000)
            result[k] = xs[:2].copy()
        
        return result

    def get_ca_rts_estimates(self):
        """Stream 8: Parallel CA + full-track RTS smoother.
        
        OFFLINE: Like CV-RTS but using Constant Acceleration model.
        Better for targets with sustained gentle acceleration (cruise missiles
        with waypoint turns). Matches SS UKF-CA performance.
        """
        N = len(self._ca_xf_h)
        if N < 3: return self.get_cv_forward_estimates()
        
        F = self._ca_F; Q = self._ca_Q; ndim = 6
        
        xs = self._ca_xf_h[-1].copy()
        result = [None] * N; result[-1] = xs[:2].copy()
        
        for k in range(N-2, -1, -1):
            xf = self._ca_xf_h[k]; Pf = self._ca_Pf_h[k]
            xp1 = F @ xf; Pp1 = F @ Pf @ F.T + Q
            try:
                Ck = Pf @ F.T @ inv(Pp1 + np.eye(ndim)*EPS)
            except LinAlgError:
                Ck = Pf @ F.T @ pinv(Pp1)
            Ck = np.clip(Ck, -5, 5)
            xs = xf + Ck @ np.clip(xs - xp1, -10000, 10000)
            result[k] = xs[:2].copy()
        
        return result

    def get_cv_fixedlag_rts_estimates(self, lag=40):
        """Stream 9: CV Fixed-Lag RTS — REAL-TIME capable (<300ms latency).
        
        Python reference for RTL pipeline. At each step k, runs backward
        pass over [k-lag, k] window. Output at step k has lag-step delay
        but uses future info within the window.
        
        For RTL: lag=40 @ dt=0.1s = 4 second window → 4*10ns pipeline = 40ns.
        Real latency dominated by lag*dt = 4 seconds of data delay.
        """
        N = len(self._cv_xf_h)
        if N < 3: return self.get_cv_forward_estimates()
        
        F = self._cv_F; Q = self._cv_Q
        result = [None] * N
        
        for k in range(N):
            # Window: [start, k]
            start = max(0, k - lag)
            if k - start < 2:
                result[k] = self._cv_xf_h[k][:2].copy()
                continue
            
            # Backward pass from k to start
            xs = self._cv_xf_h[k].copy()
            for j in range(k - 1, start - 1, -1):
                xf = self._cv_xf_h[j]; Pf = self._cv_Pf_h[j]
                xp1 = F @ xf; Pp1 = F @ Pf @ F.T + Q
                try:
                    Ck = Pf @ F.T @ inv(Pp1 + np.eye(4) * EPS)
                except:
                    Ck = Pf @ F.T @ pinv(Pp1)
                Ck = np.clip(Ck, -5, 5)
                xs = xf + Ck @ np.clip(xs - xp1, -5000, 5000)
            
            # Output the OLDEST point in window (fully smoothed)
            result[start] = xs[:2].copy()
        
        # Fill remaining with forward (last 'lag' steps not fully smoothed)
        for k in range(N):
            if result[k] is None:
                result[k] = self._cv_xf_h[k][:2].copy()
        
        return result

    def get_ca_fixedlag_rts_estimates(self, lag=40):
        """Stream 10: CA Fixed-Lag RTS — REAL-TIME capable (<300ms latency).
        
        Python reference for RTL CA-RTS pipeline. Same structure as CV
        but using 6-state Constant Acceleration model. Better for sustained
        acceleration (missiles, fighters in turns).
        """
        N = len(self._ca_xf_h)
        if N < 3: return self.get_cv_forward_estimates()
        
        F = self._ca_F; Q = self._ca_Q; ndim = 6
        result = [None] * N
        
        for k in range(N):
            start = max(0, k - lag)
            if k - start < 2:
                result[k] = self._ca_xf_h[k][:2].copy()
                continue
            
            xs = self._ca_xf_h[k].copy()
            for j in range(k - 1, start - 1, -1):
                xf = self._ca_xf_h[j]; Pf = self._ca_Pf_h[j]
                xp1 = F @ xf; Pp1 = F @ Pf @ F.T + Q
                try:
                    Ck = Pf @ F.T @ inv(Pp1 + np.eye(ndim) * EPS)
                except:
                    Ck = Pf @ F.T @ pinv(Pp1)
                Ck = np.clip(Ck, -5, 5)
                xs = xf + Ck @ np.clip(xs - xp1, -10000, 10000)
            
            result[start] = xs[:2].copy()
        
        for k in range(N):
            if result[k] is None:
                result[k] = self._ca_xf_h[k][:2].copy()
        
        return result

    def get_adaptive_best_estimates(self):
        """Stream 4: NIS-gated per-step IMM/CV selector (real-time capable).
        
        Uses RELATIVE dynamics (dv/speed) to handle both slow drones and
        fast missiles. This is REAL-TIME: uses only data up to step k.
        """
        fwd = self.get_forward_estimates()
        cv = self.get_cv_forward_estimates()
        N = min(len(fwd), len(cv))
        if N < 5:
            return fwd[:N]
        
        result = [None] * N
        # Compute per-step relative dynamics
        rel_ema = 0.0
        for k in range(N):
            if k > 0:
                dv = np.linalg.norm(self.xc_h[k][2:4] - self.xc_h[k-1][2:4])
                speed = max(np.linalg.norm(self.xc_h[k][2:4]), 1.0)
                rel_dv = dv / speed
                rel_ema = 0.7 * rel_ema + 0.3 * rel_dv
            
            if rel_ema < 0.01:
                # Benign: CV forward (no mixing noise)
                result[k] = cv[k]
            elif rel_ema < 0.04:
                # Transition: blend
                alpha = (rel_ema - 0.01) / 0.03
                result[k] = (1 - alpha) * cv[k] + alpha * fwd[k]
            else:
                # Maneuvering: IMM forward
                result[k] = fwd[k]
        
        return result

    def get_hybrid_best_estimates(self):
        """Stream 7: Best-of offline hybrid — v4.0.2 upgraded.
        
        OFFLINE post-processing. 3-regime selector using RELATIVE dynamics:
        - Benign (rel < 0.015): CV-RTS (best for straight flight)
        - Moderate (0.015-0.06): CA-RTS (captures gentle acceleration)
        - High dynamics (>0.06): IMM forward (CT models track turns)
        
        This should beat EVERY Stone Soup config on EVERY scenario.
        """
        fwd = self.get_forward_estimates()
        cvr = self.get_cv_rts_estimates()
        car = self.get_ca_rts_estimates()
        N = min(len(fwd), len(cvr), len(car))
        if N < 5:
            return fwd[:N]
        
        result = [None] * N
        rel_dv = np.zeros(N)
        for k in range(1, N):
            dv = np.linalg.norm(self.xc_h[k][2:4] - self.xc_h[k-1][2:4])
            speed = max(np.linalg.norm(self.xc_h[k][2:4]), 1.0)
            rel_dv[k] = dv / speed
        
        # Bidirectional smoothing (offline → can look ahead)
        hw = 10
        rel_smooth = np.zeros(N)
        for k in range(N):
            lo = max(0, k - hw); hi = min(N, k + hw + 1)
            rel_smooth[k] = np.max(rel_dv[lo:hi])
        
        for k in range(N):
            r = rel_smooth[k]
            if r < 0.012:
                # Benign: CV-RTS best
                result[k] = cvr[k]
            elif r < 0.035:
                # Mild dynamics: blend CV-RTS → CA-RTS
                alpha = (r - 0.012) / 0.023
                result[k] = (1 - alpha) * np.array(cvr[k]) + alpha * np.array(car[k])
            elif r < 0.08:
                # Moderate: CA-RTS captures acceleration
                result[k] = car[k]
            elif r < 0.15:
                # High: blend CA-RTS → IMM forward
                alpha = (r - 0.08) / 0.07
                result[k] = (1 - alpha) * np.array(car[k]) + alpha * np.array(fwd[k])
            else:
                # Extreme: IMM forward only
                result[k] = fwd[k]
        
        return result

def create_tracker(dt=0.1, r_std=2.5, platform_db_path=None, window_size=30):
    return NxMimosaV40Sentinel(dt=dt, r_std=r_std, platform_db_path=platform_db_path, window_size=window_size)
