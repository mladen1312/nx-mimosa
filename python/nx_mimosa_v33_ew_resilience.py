#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════════════════════
NX-MIMOSA v3.3 - FUNDAMENTALLY IMPROVED ECCM
═══════════════════════════════════════════════════════════════════════════════════════════════

KEY INSIGHTS:

1. ANTI-RGPO PROBLEM:
   - Filter tracks spoofed target → innovations stay small → no detection!
   - SOLUTION: Compare tracked position vs DEAD-RECKONING from initial conditions
   - If |track - dead_reckoning| grows monotonically → RGPO detected

2. ANTI-CROSS-EYE PROBLEM:
   - Correcting angles makes things WORSE (averaging bad data)
   - SOLUTION: DON'T correct, just increase R (let filter handle uncertainty)
   - Detect via angle jitter, respond with R inflation only

Author: Dr. Mladen Mešter / Nexellum d.o.o.
═══════════════════════════════════════════════════════════════════════════════════════════════
"""

import numpy as np
from numpy.linalg import inv, det, norm
from collections import deque
from typing import Dict
import time

__version__ = "3.3.0"

# =============================================================================
# ANTI-RGPO: Dead-Reckoning Comparison
# =============================================================================

class AntiRGPO_DeadReckoning:
    """
    RGPO Detection via Dead-Reckoning Comparison
    
    Key insight: RGPO pulls the TRACKED position away from true position.
    But we don't know true position. However, we can:
    1. Keep a dead-reckoned estimate from initial state
    2. Compare tracked position against dead-reckoned
    3. If the discrepancy grows monotonically → RGPO detected
    
    Why this works: Dead-reckoning drifts randomly (process noise).
    RGPO drift is MONOTONIC (always increasing range).
    """
    
    def __init__(self, dt: float = 0.05):
        self.dt = dt
        self.buffer_size = 50
        
        # Dead-reckoning state (propagated without measurements)
        self.x_dr = None  # Dead-reckoned state [x,y,z,vx,vy,vz]
        
        # Discrepancy history
        self.range_discrepancy = deque(maxlen=self.buffer_size)
        self.step = 0
        
        # Thresholds
        self.min_samples = 20
        self.monotonic_threshold = 0.8  # 80% of changes same sign
        self.drift_rate_threshold = 20.0  # m/s drift rate
        
        # State
        self.detected = False
        self.confidence = 0.0
        
    def initialize(self, x0: np.ndarray):
        """Initialize dead-reckoning with initial state."""
        self.x_dr = x0.copy()
        self.range_discrepancy.clear()
        self.step = 0
        self.detected = False
        self.confidence = 0.0
        
    def reset(self):
        self.x_dr = None
        self.range_discrepancy.clear()
        self.step = 0
        self.detected = False
        self.confidence = 0.0
        
    def propagate_dr(self):
        """Propagate dead-reckoning state (constant velocity)."""
        if self.x_dr is None:
            return
        # Simple CV propagation
        self.x_dr[:3] += self.x_dr[3:6] * self.dt
        
    def check(self, x_tracked: np.ndarray) -> Dict:
        """
        Compare tracked state against dead-reckoning.
        
        Args:
            x_tracked: Current tracked state from filter [x,y,z,vx,vy,vz]
        """
        self.step += 1
        
        result = {
            'detected': False,
            'confidence': 0.0,
            'r_scale': 1.0,
            'k_scale': 1.0,
        }
        
        if self.x_dr is None:
            return result
        
        # Propagate dead-reckoning
        self.propagate_dr()
        
        # Update dead-reckoning velocity from tracked (but NOT position)
        # This is key: we trust velocity but not position
        self.x_dr[3:6] = 0.9 * self.x_dr[3:6] + 0.1 * x_tracked[3:6]
        
        # Compute range discrepancy
        r_tracked = norm(x_tracked[:3])
        r_dr = norm(self.x_dr[:3])
        discrepancy = r_tracked - r_dr  # Positive if tracked is farther
        
        self.range_discrepancy.append(discrepancy)
        
        if len(self.range_discrepancy) < self.min_samples:
            return result
        
        # Check for monotonic growth
        disc = np.array(self.range_discrepancy)
        diffs = np.diff(disc)
        
        # Fraction of positive changes
        positive_fraction = np.mean(diffs > 0) if len(diffs) > 0 else 0
        
        # Drift rate
        drift_rate = (disc[-1] - disc[0]) / (len(disc) * self.dt)
        
        # Detection: monotonic growth at significant rate
        is_monotonic = positive_fraction > self.monotonic_threshold
        is_significant = drift_rate > self.drift_rate_threshold
        
        if is_monotonic and is_significant:
            self.detected = True
            
            # Confidence based on monotonicity and drift rate
            mono_excess = (positive_fraction - self.monotonic_threshold) / (1 - self.monotonic_threshold)
            rate_excess = (drift_rate - self.drift_rate_threshold) / self.drift_rate_threshold
            
            self.confidence = min(1.0, mono_excess * 0.5 + min(rate_excess, 1.0) * 0.5)
            
            result['detected'] = True
            result['confidence'] = self.confidence
            
            # Response: Inflate R and reduce K
            result['r_scale'] = 1.0 + 30.0 * self.confidence  # Up to 31x
            result['k_scale'] = max(0.05, 1.0 - 0.9 * self.confidence)  # Down to 0.05x
        else:
            self.detected = False
            self.confidence = 0.0
            
        return result


# =============================================================================
# ANTI-CROSS-EYE: R Inflation Only (No Angle Correction)
# =============================================================================

class AntiCrossEye_RInflation:
    """
    Cross-Eye Detection with R Inflation Only
    
    Key insight: DON'T try to correct angles - that makes things worse.
    Just detect cross-eye and increase R, let the filter handle uncertainty.
    """
    
    def __init__(self, dt: float = 0.05):
        self.dt = dt
        self.buffer_size = 15
        
        # Angle history for jitter detection
        self.az_history = deque(maxlen=self.buffer_size)
        self.el_history = deque(maxlen=self.buffer_size)
        
        # Thresholds
        self.min_samples = 6
        self.jitter_threshold = np.deg2rad(2.0)  # 2 deg/s jitter
        
        # State
        self.detected = False
        self.confidence = 0.0
        
    def reset(self):
        self.az_history.clear()
        self.el_history.clear()
        self.detected = False
        self.confidence = 0.0
        
    def check(self, z_meas: np.ndarray) -> Dict:
        """
        Check for cross-eye via angle jitter.
        Returns R inflation factor only - NO angle correction.
        """
        # Convert to angles
        x, y = z_meas[0], z_meas[1]
        zc = z_meas[2] if len(z_meas) > 2 else 0
        r = np.sqrt(x**2 + y**2 + zc**2)
        az = np.arctan2(y, x)
        el = np.arctan2(zc, np.sqrt(x**2 + y**2) + 1e-10)
        
        self.az_history.append(az)
        self.el_history.append(el)
        
        result = {
            'detected': False,
            'confidence': 0.0,
            'r_scale': 1.0,
        }
        
        if len(self.az_history) < self.min_samples:
            return result
        
        # Unwrap azimuth
        az_arr = np.unwrap(list(self.az_history))
        el_arr = np.array(self.el_history)
        
        # Compute angle rates
        az_rate = np.diff(az_arr) / self.dt
        el_rate = np.diff(el_arr) / self.dt
        
        # Jitter = STD of angle rate
        az_jitter = np.std(az_rate)
        el_jitter = np.std(el_rate)
        total_jitter = np.sqrt(az_jitter**2 + el_jitter**2)
        
        if total_jitter > self.jitter_threshold:
            self.detected = True
            
            jitter_excess = (total_jitter - self.jitter_threshold) / self.jitter_threshold
            self.confidence = min(1.0, jitter_excess * 0.5)
            
            result['detected'] = True
            result['confidence'] = self.confidence
            
            # R inflation only - NO angle correction
            result['r_scale'] = 1.0 + 15.0 * self.confidence  # Up to 16x
        else:
            self.detected = False
            self.confidence = 0.0
            
        return result


# =============================================================================
# NX-MIMOSA v3.3 TRACKER
# =============================================================================

class NX_MIMOSA_v33:
    """
    NX-MIMOSA v3.3 with fundamentally improved ECCM.
    """
    
    def __init__(self, dt: float, sigma: float, max_g: float = 30,
                 anti_rgpo: bool = True, anti_crosseye: bool = True):
        self.dt = dt
        self.sigma = sigma
        self.max_g = max_g
        
        # ECCM Modules
        self.rgpo_module = AntiRGPO_DeadReckoning(dt) if anti_rgpo else None
        self.crosseye_module = AntiCrossEye_RInflation(dt) if anti_crosseye else None
        
        # Innovation-based R adaptation
        self.innovation_buffer = deque(maxlen=15)
        self.r_adapt = 1.0
        
        self._build_models()
        self.q_scale = 1.0
        
    def _build_models(self):
        dt = self.dt
        F = np.eye(6)
        F[0, 3] = F[1, 4] = F[2, 5] = dt
        self.F = [F.copy() for _ in range(3)]
        
        def make_Q(a):
            q = a**2
            Q = np.zeros((6, 6))
            Q[0,0] = Q[1,1] = Q[2,2] = q * dt**4 / 4
            Q[0,3] = Q[3,0] = Q[1,4] = Q[4,1] = Q[2,5] = Q[5,2] = q * dt**3 / 2
            Q[3,3] = Q[4,4] = Q[5,5] = q * dt**2
            return Q
        
        self.Q_base = [make_Q(10), make_Q(50), make_Q(self.max_g * 10)]
        self.H = np.zeros((3, 6))
        self.H[0,0] = self.H[1,1] = self.H[2,2] = 1
        
    def initialize(self, z0: np.ndarray, v0: np.ndarray):
        x0 = np.concatenate([z0, v0])
        P0 = np.diag([1000, 1000, 1000, 5000, 5000, 5000])**2
        self.x = [x0.copy() for _ in range(3)]
        self.P = [P0.copy() for _ in range(3)]
        self.mu = np.array([0.7, 0.2, 0.1])
        self.q_scale = 1.0
        self.r_adapt = 1.0
        self.innovation_buffer.clear()
        
        if self.rgpo_module:
            self.rgpo_module.initialize(x0)
        if self.crosseye_module:
            self.crosseye_module.reset()
        
    def _get_combined_estimate(self):
        return sum(self.mu[j] * self.x[j] for j in range(3))
    
    def update(self, z: np.ndarray, sigma_effective: float = None) -> np.ndarray:
        sigma = sigma_effective if sigma_effective else self.sigma
        R_base = sigma**2 * np.eye(3)
        
        # Get current tracked state
        x_tracked = self._get_combined_estimate()
        
        # === ECCM CHECKS ===
        r_scale = 1.0
        k_scale = 1.0
        
        # Anti-RGPO
        if self.rgpo_module:
            rgpo = self.rgpo_module.check(x_tracked)
            if rgpo['detected']:
                r_scale *= rgpo['r_scale']
                k_scale *= rgpo['k_scale']
        
        # Anti-Cross-Eye
        if self.crosseye_module:
            crosseye = self.crosseye_module.check(z)
            if crosseye['detected']:
                r_scale *= crosseye['r_scale']
        
        # Innovation-based R adaptation
        x_pred_simple = self.F[0] @ x_tracked
        y_simple = z - self.H @ x_pred_simple
        self.innovation_buffer.append(norm(y_simple))
        
        if len(self.innovation_buffer) >= 5:
            avg_inn = np.mean(self.innovation_buffer)
            expected_inn = np.sqrt(3) * sigma  # Expected for 3D
            if avg_inn > 2 * expected_inn:
                self.r_adapt = min(self.r_adapt * 1.1, 10.0)
            elif avg_inn < expected_inn:
                self.r_adapt = max(self.r_adapt * 0.95, 1.0)
        
        R = R_base * r_scale * self.r_adapt
        
        # === VS-IMM UPDATE ===
        n = 3
        
        mu_cv = self.mu[0]
        p = 0.95 if mu_cv > 0.8 else (0.90 if mu_cv > 0.5 else 0.85)
        PI = np.array([[p, (1-p)/2, (1-p)/2],
                      [(1-p)/2, p, (1-p)/2],
                      [(1-p)/2, (1-p)/2, p]])
        
        c_bar = PI.T @ self.mu + 1e-10
        mu_ij = np.zeros((n, n))
        for i in range(n):
            for j in range(n):
                mu_ij[i, j] = PI[i, j] * self.mu[i] / c_bar[j]
        
        x_mix, P_mix = [], []
        for j in range(n):
            x_j = sum(mu_ij[i, j] * self.x[i] for i in range(n))
            P_j = sum(mu_ij[i, j] * (self.P[i] + np.outer(self.x[i] - x_j, self.x[i] - x_j)) 
                     for i in range(n))
            x_mix.append(x_j)
            P_mix.append(P_j)
        
        Q = [self.q_scale * q for q in self.Q_base]
        
        x_filt, P_filt, liks = [], [], []
        total_nis = 0
        
        for j in range(n):
            xp = self.F[j] @ x_mix[j]
            Pp = self.F[j] @ P_mix[j] @ self.F[j].T + Q[j]
            
            y = z - self.H @ xp
            S = self.H @ Pp @ self.H.T + R + 1e-3 * np.eye(3)
            
            try:
                S_inv = inv(S)
                K = Pp @ self.H.T @ S_inv * k_scale
                
                nis = y @ S_inv @ y
                total_nis += self.mu[j] * nis
                
                x_f = xp + K @ y
                I_KH = np.eye(6) - K @ self.H
                P_f = I_KH @ Pp @ I_KH.T + K @ R @ K.T
                P_f = 0.5 * (P_f + P_f.T) + 1e-6 * np.eye(6)
                
                lik = np.exp(-0.5 * min(nis, 50)) / (np.sqrt((2*np.pi)**3 * max(det(S), 1e-10)) + 1e-10)
            except:
                x_f, P_f = xp, Pp
                lik = 1e-10
            
            x_filt.append(x_f)
            P_filt.append(P_f)
            liks.append(max(lik, 1e-20))
        
        # Adaptive Q
        if total_nis > 7.815:
            self.q_scale = min(self.q_scale * 1.2, 5.0)
        elif total_nis < 3.0:
            self.q_scale = max(self.q_scale * 0.95, 0.5)
        
        # Mode probability update
        mu_new = c_bar * np.array(liks)
        mu_sum = mu_new.sum()
        self.mu = mu_new / mu_sum if mu_sum > 1e-10 else np.array([0.34, 0.33, 0.33])
        
        self.x, self.P = x_filt, P_filt
        return self._get_combined_estimate()


# =============================================================================
# SIMULATION
# =============================================================================

class JammingEnvironment:
    def __init__(self, intensity: float = 1.0, seed: int = 42):
        self.intensity = intensity
        self.rng = np.random.RandomState(seed)
        self.rgpo_offset = 0.0
        
    def apply_rgpo(self, z, dt, pull_rate=100.0):
        self.rgpo_offset += pull_rate * dt * self.intensity
        r = norm(z[:3])
        if r > 0:
            return z + (z / r) * self.rgpo_offset
        return z
    
    def apply_cross_eye(self, z, angle_err_deg=5.0):
        x, y = z[0], z[1]
        zc = z[2] if len(z) > 2 else 0
        r = np.sqrt(x**2 + y**2 + zc**2)
        az = np.arctan2(y, x)
        el = np.arctan2(zc, np.sqrt(x**2 + y**2) + 1e-10)
        
        err = np.deg2rad(angle_err_deg * self.intensity)
        az += self.rng.randn() * err
        el += self.rng.randn() * err * 0.5
        
        return np.array([
            r * np.cos(el) * np.cos(az),
            r * np.cos(el) * np.sin(az),
            r * np.sin(el)
        ])[:len(z)]

def generate_target(dt, duration, seed=42):
    np.random.seed(seed)
    T = int(duration / dt)
    traj = np.zeros((T, 6))
    traj[0] = [50000, 10000, 5000, -400, -100, 20]
    for k in range(1, T):
        t = k * dt
        phase = (k * dt) % 20
        if phase < 5:
            ax, ay, az = 0, 0, 0
        elif phase < 10:
            ax, ay, az = 50*np.sin(0.5*t), 30*np.cos(0.3*t), 10
        else:
            ax, ay, az = 80*np.sin(t), 60*np.cos(1.2*t), -20*np.sin(0.8*t)
        traj[k, 3:6] = traj[k-1, 3:6] + np.array([ax, ay, az]) * dt
        traj[k, :3] = traj[k-1, :3] + traj[k, 3:6] * dt
    return traj

def run_simulation(attack_type, intensity, n_runs=20, seed=42):
    dt, sigma, duration = 0.05, 20, 50
    
    results = {
        'v3.3-DR-ECCM': {'rmse': [], 'tl': []},
        'v3.1-Basic': {'rmse': [], 'tl': []},
        'No-ECCM': {'rmse': [], 'tl': []},
    }
    
    for run in range(n_runs):
        traj = generate_target(dt, duration, seed + run)
        
        trackers = {
            'v3.3-DR-ECCM': NX_MIMOSA_v33(dt, sigma, anti_rgpo=True, anti_crosseye=True),
            'v3.1-Basic': NX_MIMOSA_v33(dt, sigma, anti_rgpo=False, anti_crosseye=False),
            'No-ECCM': NX_MIMOSA_v33(dt, sigma, anti_rgpo=False, anti_crosseye=False),
        }
        
        # Each tracker needs its own jammer (identical seeds for fair comparison)
        jammers = {name: JammingEnvironment(intensity, seed + run + 1000) for name in trackers}
        
        for t in trackers.values():
            t.initialize(traj[0, :3], traj[0, 3:6])
        
        estimates = {n: [] for n in trackers}
        
        for k in range(1, len(traj)):
            for name, tracker in trackers.items():
                jammer = jammers[name]
                
                # Apply attack
                z = traj[k, :3].copy()
                if attack_type == 'drfm_rgpo':
                    z = jammer.apply_rgpo(z, dt)
                elif attack_type == 'cross_eye':
                    z = jammer.apply_cross_eye(z)
                z += jammer.rng.randn(3) * sigma
                
                estimates[name].append(tracker.update(z, sigma))
        
        for name in trackers:
            ests = np.array(estimates[name])
            if len(ests) > 0:
                pos_err = np.sqrt(np.sum((ests[:, :3] - traj[1:len(ests)+1, :3])**2, axis=1))
                results[name]['rmse'].append(np.sqrt(np.mean(pos_err**2)))
                results[name]['tl'].append(np.mean(pos_err > 1000) * 100)
    
    return results

def main():
    print("="*100)
    print("NX-MIMOSA v3.3 - DEAD-RECKONING BASED ECCM")
    print("="*100)
    
    print("""
┌─────────────────────────────────────────────────────────────────────────────────────────────┐
│ FUNDAMENTAL IMPROVEMENTS IN v3.3                                                            │
├─────────────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                             │
│ ANTI-RGPO: Dead-Reckoning Comparison                                                        │
│ ─────────────────────────────────────                                                       │
│ PROBLEM: Filter tracks spoofed target → innovations stay small → no detection              │
│                                                                                             │
│ SOLUTION:                                                                                   │
│ 1. Maintain dead-reckoned state from initial conditions                                     │
│ 2. Compare tracked range vs dead-reckoned range                                             │
│ 3. If discrepancy grows MONOTONICALLY → RGPO detected                                       │
│ 4. Dead-reckoning drifts randomly, RGPO drifts monotonically                                │
│                                                                                             │
│ ANTI-CROSS-EYE: R Inflation Only                                                            │
│ ─────────────────────────────────                                                           │
│ PROBLEM: Angle correction makes things WORSE (averaging bad data)                           │
│                                                                                             │
│ SOLUTION:                                                                                   │
│ 1. Detect via angle jitter (STD of angle rate)                                              │
│ 2. DON'T correct angles                                                                     │
│ 3. Just inflate R → filter handles increased uncertainty                                    │
│                                                                                             │
└─────────────────────────────────────────────────────────────────────────────────────────────┘
""")
    
    scenarios = [
        ('drfm_rgpo', 'DRFM Range Gate Pull-Off'),
        ('cross_eye', 'Cross-Eye Angle Deception'),
    ]
    
    intensities = [0.3, 0.6, 0.9]
    all_results = []
    
    print("\n" + "="*100)
    print("🎯 PERFORMANCE RESULTS")
    print("="*100)
    
    for attack_type, attack_name in scenarios:
        print(f"\n▶ {attack_name}")
        print("-"*90)
        
        for intensity in intensities:
            label = ['Low', 'Med', 'High'][intensities.index(intensity)]
            results = run_simulation(attack_type, intensity, n_runs=20)
            
            print(f"\n  Intensity: {label} ({intensity*100:.0f}%)")
            print(f"  {'Tracker':<18} {'RMSE (m)':<12} {'Track Loss %'}")
            print(f"  {'-'*45}")
            
            for name in ['v3.3-DR-ECCM', 'v3.1-Basic', 'No-ECCM']:
                rmse = np.mean(results[name]['rmse'])
                tl = np.mean(results[name]['tl'])
                print(f"  {name:<18} {rmse:<12.1f} {tl:.1f}")
                all_results.append({
                    'attack': attack_type, 'intensity': intensity,
                    'tracker': name, 'rmse': rmse
                })
            
            # Improvements
            v33 = np.mean(results['v3.3-DR-ECCM']['rmse'])
            v31 = np.mean(results['v3.1-Basic']['rmse'])
            imp = (v31 - v33) / v31 * 100 if v31 > 0 else 0
            print(f"\n  ✓ v3.3 improvement vs v3.1: {imp:+.1f}%")
    
    # Summary
    print("\n" + "="*100)
    print("📊 FINAL SUMMARY")
    print("="*100)
    
    for attack in ['drfm_rgpo', 'cross_eye']:
        name = 'DRFM RGPO' if attack == 'drfm_rgpo' else 'Cross-Eye'
        v33_avg = np.mean([r['rmse'] for r in all_results if r['attack']==attack and r['tracker']=='v3.3-DR-ECCM'])
        v31_avg = np.mean([r['rmse'] for r in all_results if r['attack']==attack and r['tracker']=='v3.1-Basic'])
        
        imp = (v31_avg - v33_avg) / v31_avg * 100 if v31_avg > 0 else 0
        print(f"\n{name}:")
        print(f"  v3.3 DR-ECCM:  {v33_avg:.0f} m")
        print(f"  v3.1 Basic:    {v31_avg:.0f} m")
        print(f"  Improvement:   {imp:+.1f}%")
    
    # Overall
    v33_total = np.mean([r['rmse'] for r in all_results if r['tracker']=='v3.3-DR-ECCM'])
    v31_total = np.mean([r['rmse'] for r in all_results if r['tracker']=='v3.1-Basic'])
    
    print(f"\n{'='*60}")
    print(f"OVERALL v3.3 vs v3.1: {(v31_total - v33_total) / v31_total * 100:+.1f}%")
    print(f"{'='*60}")
    
    print("\n✓ Simulation complete")

if __name__ == "__main__":
    main()
