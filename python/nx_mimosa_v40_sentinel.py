"""
NX-MIMOSA v4.0 "SENTINEL" — Platform-Aware Variable-Structure IMM Tracker
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

Author: Dr. Mladen Mešter / Nexellum d.o.o.
License: AGPL v3 (open-source) | Commercial license available
"""

import numpy as np
from numpy.linalg import inv, pinv, LinAlgError
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
import json, os

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
    def __init__(self, db, window=15):
        self.db = {k:v for k,v in db.items() if k != "_meta"}
        self.spd = deque(maxlen=window)
        self.g_h = deque(maxlen=window)
        self.alt = deque(maxlen=window)
        self.omega_h = deque(maxlen=window)  # Turn rate history
        self.pv = None
        self.best = "Unknown"
        self.conf = 0.0

    def update(self, pos, vel, dt):
        s = np.linalg.norm(vel)
        self.spd.append(s)
        if self.pv is not None and dt > 0:
            dv = vel - self.pv
            g_est = np.linalg.norm(dv) / dt / GRAVITY
            self.g_h.append(g_est)
            # Turn rate estimate
            sp = max(s, 1)
            cross = vel[0]*self.pv[1] - vel[1]*self.pv[0]
            psp = max(np.linalg.norm(self.pv), 1)
            omega = abs(cross / (sp * psp * dt + 1e-10))
            self.omega_h.append(omega)
        self.pv = vel.copy()
        self.alt.append(abs(pos[1]) if len(pos)>=2 else 0)
        if len(self.spd) < 5 or len(self.g_h) < 3:
            return "Unknown", 0.0
        return self._classify()

    def _classify(self):
        as_ = np.mean(self.spd); ms = np.max(self.spd)
        ag = np.mean(self.g_h); mg = np.max(self.g_h)
        g_var = np.std(self.g_h) if len(self.g_h) > 2 else 0  # Key discriminator
        aa = np.mean(self.alt)
        ao = np.mean(self.omega_h) if self.omega_h else 0
        scores = {}
        for n, p in self.db.items():
            if n == "Unknown": continue
            scores[n] = self._score(p, as_, ms, ag, mg, g_var, aa, ao)
        if not scores: return "Unknown", 0.0
        best = max(scores, key=scores.get)
        c = scores[best]
        if c >= 0.35:
            self.best, self.conf = best, c
        else:
            self.best, self.conf = "Unknown", c
        return self.best, self.conf

    def _score(self, p, as_, ms, ag, mg, g_var, aa, ao):
        cat = p.get("category","unknown")
        pms = p["max_speed_mps"]; pcs = p.get("cruise_speed_mps", pms*0.6)
        pmg = p["max_g"]; psg = p.get("sustained_g", pmg*0.5)
        pma = p.get("max_altitude_m", 50000)
        pmo = p.get("max_turn_rate_radps", 1.0)
        sc = 0.0
        
        # Speed (0-3)
        if ms > pms*1.2: sc -= 2
        else:
            r = as_/max(pcs,1)
            sc += 3*max(0, 1-abs(r-1)*0.4) if 0.3<=r<=2 else 0.5
        
        # G-load match (0-5) — STRONGEST discriminator
        if mg > pmg*1.3: sc -= 3
        else:
            r = mg/max(psg,0.1)
            if 0.1<=r<=1.5: sc += 5*max(0, 1-abs(r-0.7)*0.5)
            elif r<0.1: sc += (0 if psg>3 else 3)
            else: sc += 1
        
        # G variability (0-3) — fighters/FPV have high variance, missiles/transport low
        g_var_expected = 0.3 if cat in ("fighter","fpv_kamikaze") else 0.1 if cat in ("transport","bomber","aew","loitering_munition") else 0.2
        g_var_match = max(0, 3 - 5*abs(g_var - g_var_expected))
        sc += max(g_var_match, 0)
        
        # Turn rate match (0-2)
        if ao <= pmo * 1.3:
            omega_ratio = ao / max(pmo, 0.01)
            sc += 2 * max(0, 1 - abs(omega_ratio - 0.5) * 0.8)
        else:
            sc -= 1
        
        # Altitude (0-2)
        sc += 2 if aa<=pma*1.2 else (0.5 if aa<=pma*2 else 0)
        
        # Category-specific (0-3)
        cat_s = 0.3
        if cat=="fighter" and 80<as_<800 and mg>2: cat_s = 3
        elif cat=="fighter" and mg>3: cat_s = 2.5
        elif cat=="small_drone" and as_<50 and mg<3: cat_s = 3
        elif cat=="fpv_kamikaze" and as_<60 and g_var>0.3: cat_s = 3  # High g variability = FPV
        elif cat=="fpv_kamikaze" and as_<60 and mg>0.5: cat_s = 2.5
        elif cat=="loitering_munition" and 20<as_<80 and g_var<0.2: cat_s = 3  # Low g var = loiter
        elif cat=="cruise_missile" and 100<as_<400 and mg<3 and g_var<0.3: cat_s = 3
        elif cat=="cruise_missile" and 100<as_<400 and mg<6: cat_s = 2
        elif cat=="ballistic_missile" and as_>800 and aa>5000: cat_s = 3
        elif cat=="hypersonic_missile" and as_>1500: cat_s = 3
        elif cat=="sam" and as_>500 and mg>3: cat_s = 3
        elif cat in ("transport","bomber","aew") and as_<350 and mg<2 and g_var<0.15: cat_s = 3
        elif cat=="mlrs" and as_>200 and aa>3000: cat_s = 3
        sc += cat_s
        
        return max(sc,0)/18  # Normalized to total possible score

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
class NxMimosaV40Sentinel:
    def __init__(self, dt=0.1, r_std=2.5, platform_db_path=None,
                 window_size=30, prune_threshold=0.03,
                 initial_models=None, q_base=1.0):
        self.dt = dt; self.r_std = r_std
        self.R = np.eye(2)*r_std**2
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
        # Per-model history for RTS
        self.xf_h = []; self.Pf_h = []; self.xp_h = []; self.Pp_h = []
        self.F_h = []; self.mu_h = []; self.act_h = []; self.xc_h = []
        self.step = 0; self.initialized = False

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
        self.initialized = True; self.step = 0

    def update(self, z):
        if not self.initialized:
            self.initialize(z)
            self.intent_state.active_models = list(self.active_models)
            return z.copy(), np.eye(2)*self.r_std**2, self.intent_state
        self.step += 1

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

        # 2. Predict + Update
        liks = {}; Fu = {}; xps = {}; Pps = {}; xfs = {}; Pfs = {}
        for m in self.active_models:
            nx = MODEL_DIMS[m]; F,Q,H,_ = self._get_FQH(m)
            xp = F @ mx[m]; Pp = F @ mP[m] @ F.T + Q
            if m=="Ballistic": xp[3] -= GRAVITY*self.dt
            Fu[m]=F.copy(); xps[m]=xp.copy(); Pps[m]=Pp.copy()
            nu = z - H@xp; S = H@Pp@H.T + self.R; Si = safe_inv(S)
            sn,ld = np.linalg.slogdet(S)
            liks[m] = max(np.exp(-0.5*2*np.log(2*np.pi)-0.5*ld-0.5*nu@Si@nu), 1e-300) if sn>0 else 1e-300
            K = Pp@H.T@Si; xf = xp+K@nu
            IKH = np.eye(nx)-K@H; Pf = IKH@Pp@IKH.T + K@self.R@K.T
            self.x[m]=xf; self.P[m]=Pf; xfs[m]=xf.copy(); Pfs[m]=Pf.copy()

        # 3. Model prob update
        cb = {mj: sum(self.tpm[mi][mj]*self.mu[mi] for mi in self.active_models) for mj in self.active_models}
        tot = max(sum(liks[m]*cb[m] for m in self.active_models), 1e-300)
        for m in self.active_models: self.mu[m] = max(liks[m]*cb[m]/tot, 1e-30)
        ms = sum(self.mu.values())
        for m in self.active_models: self.mu[m] /= ms
        
        # Adaptive p_stay: when one model dominates, increase stay probability
        max_mu = max(self.mu.values())
        if max_mu > 0.75:
            self.p_stay = min(0.95, 0.88 + 0.1 * (max_mu - 0.75) / 0.25)
        else:
            self.p_stay = 0.88
        self._rebuild_tpm()

        # 4. Combined estimate
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

        # 6. Platform ID + VS-IMM adapt
        plat,conf = self.identifier.update(xc[:2], xc[2:4], self.dt)
        pd = self.platform_db.get(plat, self.platform_db.get("Unknown",{}))
        if conf>=0.35 and plat!="Unknown": self._vs_adapt(pd)
        
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

        # 7. Intent + Q scale
        sp = np.linalg.norm(xc[2:4]); ag = 0
        if self.xc_h: ag = np.linalg.norm(xc[2:4]-self.xc_h[-1][2:4])/(self.dt*GRAVITY+1e-10)
        phase,pc = self.intent_pred.predict(pd, sp, ag, self.mu)
        self.q_scale = pd.get("intent_phases",{}).get(phase,{}).get("q_scale",1.0)
        
        # Adaptive prune threshold: benign phases → kill off unnecessary models
        if self.q_scale < 0.3:
            self.prune_threshold = 0.15  # Aggressive: prune to CV-only
        elif self.q_scale < 0.6:
            self.prune_threshold = 0.08  # Moderate
        else:
            self.prune_threshold = 0.03  # Default: keep models for maneuvers

        # 8. Prune
        self._prune()

        # 9. Store history
        self.xf_h.append(xfs); self.Pf_h.append(Pfs)
        self.xp_h.append(xps); self.Pp_h.append(Pps)
        self.F_h.append(Fu); self.mu_h.append(dict(self.mu))
        self.act_h.append(list(self.active_models)); self.xc_h.append(xc.copy())

        # 10. Intent
        cat = pd.get("category","unknown")
        thr = {"fighter":0.6,"bomber":0.4,"transport":0.1,"aew":0.2,"ballistic_missile":0.9,
               "hypersonic_missile":0.95,"cruise_missile":0.8,"sam":0.85,"mlrs":0.7,
               "small_drone":0.3,"fpv_kamikaze":0.7,"loitering_munition":0.6}.get(cat,0.5)
        if phase in ("terminal","attack","engagement"): thr=min(thr*1.3,1)
        elif phase in ("cruise","patrol","hover"): thr*=0.8
        self.intent_state = IntentState(plat, cat, conf, phase, pc,
            pd.get("max_g",30), pd.get("max_speed_mps",3500), thr,
            list(self.active_models), len(self.active_models))
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
        if len(self.active_models)<=2: return
        rm = [m for m in self.active_models if self.mu.get(m,0)<self.prune_threshold and m!="CV"]
        for m in rm:
            if len(self.active_models)<=2: break
            self.active_models.remove(m); del self.mu[m]
        if rm:
            t = sum(self.mu.values()); self.mu = {m:p/t for m,p in self.mu.items()}
            self._rebuild_tpm()

    # =========================================================================
    # Smoothers — Per-model RTS
    # =========================================================================
    def get_forward_estimates(self): return [x[:2].copy() for x in self.xc_h]

    def get_window_smoothed_estimates(self, window=None):
        """Stream 2: Adaptive sliding window per-model RTS with divergence guard.
        
        Key: window shrinks automatically during high-dynamics segments,
        preventing oversmoothing during maneuvers while still gaining
        accuracy during benign segments.
        """
        W_max = window or self.window_size; N = len(self.xf_h)
        if N < 2: return self.get_forward_estimates()
        fwd = self.get_forward_estimates(); result = [x.copy() for x in fwd]

        for ek in range(min(5, W_max), N):
            # Adaptive window: measure dynamics in recent segment
            W = W_max
            if ek >= 5:
                dvs = []
                for j in range(max(0, ek-10), ek):
                    if j+1 < N:
                        dv = np.linalg.norm(self.xc_h[j+1][2:4] - self.xc_h[j][2:4])
                        dvs.append(dv)
                if dvs:
                    avg_dv = np.mean(dvs)
                    if avg_dv > 50:    W = min(W, 3)   # Extreme dynamics
                    elif avg_dv > 20:  W = min(W, 8)   # High dynamics
                    elif avg_dv > 10:  W = min(W, 15)  # Medium dynamics
            
            W = min(W, ek)
            if W < 2: continue

            sk = ek - W + 1; wl = ek - sk + 1
            models = self.act_h[ek]; mu_e = self.mu_h[ek]
            xc = np.zeros(2)
            for m in models:
                mu_m = mu_e.get(m,0)
                if mu_m < 1e-10: continue
                xs = self.xf_h[ek].get(m)
                if xs is None: continue
                xs = xs.copy()
                for i in range(wl-2, -1, -1):
                    k = sk+i
                    xf=self.xf_h[k].get(m); Pf=self.Pf_h[k].get(m)
                    xp1=self.xp_h[k+1].get(m); Pp1=self.Pp_h[k+1].get(m)
                    Fk=self.F_h[k+1].get(m)
                    if any(v is None for v in [xf,Pf,xp1,Pp1,Fk]):
                        xs = self.xf_h[k].get(m, xs); continue
                    G = np.clip(Pf@Fk.T@safe_inv(Pp1), -3, 3)
                    xs = xf + G @ np.clip(xs - xp1, -1000, 1000)
                xc += mu_m * xs[:2]
            # Divergence guard: tighter threshold  
            dev = np.linalg.norm(xc - fwd[sk])
            scale = max(np.linalg.norm(fwd[sk]), 50)
            result[sk] = xc if dev < scale * 0.15 else fwd[sk]
        return result

    def get_smoothed_estimates(self):
        """Stream 3: Full-track per-model RTS with tight divergence guard."""
        N = len(self.xf_h)
        if N < 2: return self.get_forward_estimates()
        fwd = self.get_forward_estimates()
        models = self.act_h[-1]; mu_f = self.mu_h[-1]
        result = [self.xc_h[k][:2].copy() for k in range(N)]

        for m in models:
            mu_m = mu_f.get(m,0)
            if mu_m < 1e-10: continue
            xs = self.xf_h[-1].get(m)
            if xs is None: continue
            xs = xs.copy()
            smoothed = [None]*N; smoothed[-1] = xs.copy()
            for k in range(N-2, -1, -1):
                xf=self.xf_h[k].get(m); Pf=self.Pf_h[k].get(m)
                xp1=self.xp_h[k+1].get(m); Pp1=self.Pp_h[k+1].get(m)
                Fk=self.F_h[k+1].get(m)
                if any(v is None for v in [xf,Pf,xp1,Pp1,Fk]):
                    xs = self.xf_h[k].get(m, xs); smoothed[k]=xs.copy(); continue
                G = np.clip(Pf@Fk.T@safe_inv(Pp1), -3, 3)
                xs = xf + G @ np.clip(xs-xp1, -1000, 1000)
                smoothed[k] = xs.copy()
            for k in range(N):
                if smoothed[k] is not None:
                    delta = mu_m*smoothed[k][:2] - mu_m*self.xc_h[k][:2]
                    candidate = result[k] + delta
                    dev = np.linalg.norm(candidate - fwd[k])
                    scale = max(np.linalg.norm(fwd[k]), 50)
                    if dev < scale * 0.15:
                        result[k] = candidate
        return result

    def get_adaptive_best_estimates(self):
        """Stream 4: Adaptive best-of per timestep.
        
        Uses smoothed during benign segments, forward during maneuvers.
        Selection based on local dynamics metric.
        """
        fwd = self.get_forward_estimates()
        N = len(fwd)
        if N < 10:
            return fwd
        
        win = self.get_window_smoothed_estimates()
        result = [None] * N
        
        for k in range(N):
            # Measure local dynamics (velocity change rate)
            dyn = 0
            for j in range(max(0, k-3), min(N-1, k+3)):
                if j+1 < N:
                    dyn += np.linalg.norm(self.xc_h[j+1][2:4] - self.xc_h[j][2:4])
            dyn /= min(6, k+3 - max(0,k-3))
            
            if k < len(win) and dyn < 10:
                # Benign segment: prefer smoothed
                result[k] = win[k]
            else:
                # Dynamic segment: use forward
                result[k] = fwd[k]
        
        return result

def create_tracker(dt=0.1, r_std=2.5, platform_db_path=None, window_size=30):
    return NxMimosaV40Sentinel(dt=dt, r_std=r_std, platform_db_path=platform_db_path, window_size=window_size)
