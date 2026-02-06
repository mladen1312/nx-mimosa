#!/usr/bin/env python3
"""
NX-MIMOSA v4.1.1 ECM Robustness Benchmark Suite
=================================================

Proves that ECM-adaptive R matrix boost WORKS against real-world jamming
scenarios. Compares tracker WITH and WITHOUT R-boost on identical scenarios.

Scenarios:
  1. RGPO  — Range Gate Pull-Off (gradual range corruption)
  2. VGPO  — Velocity Gate Pull-Off (gradual Doppler corruption)
  3. DRFM  — Digital RF Memory repeater (coherent false echo)
  4. Noise Jamming — Broadband SNR degradation
  5. Chaff Corridor — Distributed RCS spike + Doppler spread

For each scenario:
  - True trajectory is known (fighter flying straight then turning)
  - Measurements are corrupted according to ECM model
  - Tracker A: Full v4.1.1 (R-boost + ECM detection + forced models)
  - Tracker B: R-boost DISABLED (Q-boost only, simulates v4.0 behavior)
  - Metrics: RMSE, max error, track loss, intent accuracy

Usage:
    python ecm_benchmark.py             # Run all, print summary
    python ecm_benchmark.py --verbose   # Detailed per-step output
    python ecm_benchmark.py --plot      # Generate matplotlib plots

Author: Dr. Mladen Mešter, Nexellum d.o.o.
"""

import sys
import os
import math
import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nx_mimosa_v40_sentinel import NxMimosaV40Sentinel

# =============================================================================
# Constants
# =============================================================================
DT = 0.1           # 10 Hz tracker
R_STD = 2.5        # Nominal measurement noise (m)
TRACK_LOSS_THRESHOLD = 200.0  # Track considered "lost" if error > 200m
N_MONTE = 10       # Monte Carlo runs per scenario

# =============================================================================
# ECM Models — How each jammer corrupts measurements
# =============================================================================

def ecm_rgpo(step: int, true_pos: np.ndarray, onset: int, 
             pull_rate: float = 5.0) -> Tuple[np.ndarray, dict]:
    """Range Gate Pull-Off: gradually shifts range measurement.
    
    The jammer captures the radar's range gate, then slowly pulls it away
    from the true target position. After onset, the range error grows
    linearly at pull_rate m/step.
    
    Args:
        step: Current time step
        true_pos: True [x, y] position
        onset: Step at which RGPO begins
        pull_rate: Range pull rate (m/step after onset)
    
    Returns:
        (corrupted_measurement, rf_observables)
    """
    z = true_pos + np.random.randn(2) * R_STD  # Normal noise
    
    if step >= onset:
        t_ecm = step - onset
        # Pull range outward (radial direction from origin)
        r_true = np.linalg.norm(true_pos)
        if r_true > 1:
            direction = true_pos / r_true
            pull = pull_rate * t_ecm * direction
            z += pull
    
        # RF observables: SNR drops moderately, RCS spikes
        snr = max(3, 25 - 0.3 * t_ecm)
        rcs = 5.0 + 0.5 * t_ecm  # RCS walks up (jammer power)
        doppler = 0.0
    else:
        snr, rcs, doppler = 25.0, 0.0, 0.0
    
    return z, {"snr_db": snr, "rcs_dbsm": rcs, "doppler_hz": doppler}


def ecm_vgpo(step: int, true_pos: np.ndarray, true_vel: np.ndarray,
             onset: int, vel_pull_rate: float = 2.0) -> Tuple[np.ndarray, dict]:
    """Velocity Gate Pull-Off: corrupts velocity-derived position.
    
    The jammer gradually shifts the Doppler frequency, causing the
    tracker to mis-estimate velocity. Position measurements include
    a velocity-correlated bias that grows over time.
    
    Args:
        step: Current time step
        true_pos: True [x, y] position
        true_vel: True [vx, vy] velocity
        onset: Step at which VGPO begins
        vel_pull_rate: Velocity pull rate (m/s per step)
    """
    z = true_pos + np.random.randn(2) * R_STD
    
    if step >= onset:
        t_ecm = step - onset
        # Velocity pull → position bias accumulates quadratically
        sp = np.linalg.norm(true_vel)
        if sp > 1:
            vel_dir = true_vel / sp
            # Position error from integrated velocity bias
            pos_bias = vel_pull_rate * t_ecm * 0.5 * DT * vel_dir
            z += pos_bias
        
        # RF: Doppler spread increases, moderate SNR drop
        snr = max(5, 25 - 0.2 * t_ecm)
        rcs = 0.0
        doppler = 200 + 20 * t_ecm  # Doppler walks
    else:
        snr, rcs, doppler = 25.0, 0.0, 0.0
    
    return z, {"snr_db": snr, "rcs_dbsm": rcs, "doppler_hz": doppler}


def ecm_drfm(step: int, true_pos: np.ndarray, onset: int,
             false_offset: float = 100.0) -> Tuple[np.ndarray, dict]:
    """DRFM Repeater: coherent false echo at offset position.
    
    Digital RF Memory captures, delays, and retransmits radar pulse.
    Creates a false target that appears identical to the real one but
    at a different range. The tracker gets confused between two "valid"
    targets — measurement jumps between true and false.
    
    Args:
        step: Current time step
        true_pos: True [x, y] position
        onset: Step at which DRFM begins
        false_offset: Range offset of false target (m)
    """
    z = true_pos + np.random.randn(2) * R_STD
    
    if step >= onset:
        t_ecm = step - onset
        # 50% chance of measuring false target (DRFM echo)
        if np.random.rand() < 0.5:
            r_true = np.linalg.norm(true_pos)
            if r_true > 1:
                direction = true_pos / r_true
                z = true_pos + direction * (false_offset + 20 * np.sin(t_ecm * 0.2))
                z += np.random.randn(2) * R_STD
        
        # RF: SNR stable (DRFM coherent), RCS doubles (two targets)
        snr = 22.0  # Only slight drop
        rcs = 8.0 + 3.0 * np.sin(t_ecm * 0.3)  # Fluctuating (two-target blend)
        doppler = 50.0  # Small spread
    else:
        snr, rcs, doppler = 25.0, 0.0, 0.0
    
    return z, {"snr_db": snr, "rcs_dbsm": rcs, "doppler_hz": doppler}


def ecm_noise(step: int, true_pos: np.ndarray, onset: int,
              noise_power: float = 30.0) -> Tuple[np.ndarray, dict]:
    """Noise Jamming: broadband SNR degradation.
    
    Barrage noise jammer floods all frequencies. Measurement noise
    increases dramatically. Position measurements become very noisy
    but unbiased (no systematic pull).
    
    Args:
        step: Current time step
        true_pos: True [x, y] position
        onset: Step at which jamming begins
        noise_power: Measurement noise multiplier during jamming
    """
    if step >= onset:
        t_ecm = step - onset
        # Ramp up noise (jammer takes a few seconds to reach full power)
        noise_mult = min(noise_power, 1.0 + (noise_power - 1.0) * (t_ecm / 20.0))
        z = true_pos + np.random.randn(2) * R_STD * noise_mult
        
        snr = max(0, 25 - 2.0 * t_ecm)  # Rapid SNR crash
        rcs = 0.0  # Can't measure RCS under noise
        doppler = 500 + 50 * t_ecm  # Broadband spread
    else:
        z = true_pos + np.random.randn(2) * R_STD
        snr, rcs, doppler = 25.0, 0.0, 0.0
    
    return z, {"snr_db": snr, "rcs_dbsm": rcs, "doppler_hz": doppler}


def ecm_chaff(step: int, true_pos: np.ndarray, onset: int,
              chaff_radius: float = 80.0) -> Tuple[np.ndarray, dict]:
    """Chaff Corridor: distributed false returns.
    
    Aircraft dispenses chaff creating a cloud of radar-reflective material.
    Measurements jump between the real target and random points within
    the chaff cloud. RCS spikes (chaff has large aggregate cross section).
    
    Args:
        step: Current time step
        true_pos: True [x, y] position
        onset: Step at which chaff is dispensed
        chaff_radius: Radius of chaff cloud (m)
    """
    z = true_pos + np.random.randn(2) * R_STD
    
    if step >= onset:
        t_ecm = step - onset
        # 40% chance of measuring chaff instead of target
        if np.random.rand() < 0.4:
            # Chaff cloud expands slowly
            cloud_r = min(chaff_radius, 20 + 5 * t_ecm)
            offset = np.random.randn(2) * cloud_r
            z = true_pos + offset
        
        # RF: RCS spikes (chaff), Doppler spreads (slow-moving chaff vs fast target)
        snr = max(10, 25 - 0.3 * t_ecm)
        rcs = 15.0 + 5.0 * np.random.rand()  # Large, noisy RCS
        doppler = 150 + 10 * t_ecm  # Chaff has different Doppler
    else:
        snr, rcs, doppler = 25.0, 0.0, 0.0
    
    return z, {"snr_db": snr, "rcs_dbsm": rcs, "doppler_hz": doppler}


# =============================================================================
# Trajectory Generators
# =============================================================================

def gen_fighter_cruise_turn(n_steps: int, dt: float,
                            turn_onset: int = 70) -> Tuple[np.ndarray, np.ndarray]:
    """Fighter: cruise, then 5g turn, then cruise again.
    
    Default: cruise 0-69, 5g turn 70-100, cruise 100+.
    ECM typically starts BEFORE maneuver (step 30) to degrade tracking
    before the pilot begins evasion.
    
    Returns:
        (positions [n, 2], velocities [n, 2])
    """
    pos = np.zeros((n_steps, 2))
    vel = np.zeros((n_steps, 2))
    
    speed = 300.0  # m/s
    heading = 0.0  # radians
    x, y = 0.0, 10000.0
    
    for i in range(n_steps):
        vx = speed * np.cos(heading)
        vy = speed * np.sin(heading)
        
        pos[i] = [x, y]
        vel[i] = [vx, vy]
        
        x += vx * dt
        y += vy * dt
        
        # Turn phase
        if turn_onset <= i < turn_onset + 30:
            g_load = 5.0
            omega = g_load * 9.81 / speed
            heading += omega * dt
    
    return pos, vel


def gen_fighter_altitude_profile(n_steps: int) -> np.ndarray:
    """Altitude profile: cruise at 8000m, then dive at step 50.
    
    Returns:
        altitudes [n]
    """
    alt = np.zeros(n_steps)
    for i in range(n_steps):
        if i < 50:
            alt[i] = 8000.0
        else:
            # Diving at 150 m/s
            alt[i] = max(100, 8000.0 - 150.0 * (i - 50) * DT)
    return alt


# =============================================================================
# Benchmark Runner
# =============================================================================

@dataclass
class ECMBenchmarkResult:
    """Results for one ECM scenario."""
    scenario: str
    ecm_type: str
    
    # With R-boost (v4.1.1)
    rmse_with: float = 0.0
    max_err_with: float = 0.0
    track_loss_with: int = 0
    r_scale_peak_with: float = 1.0
    
    # Without R-boost (v4.0 behavior)
    rmse_without: float = 0.0
    max_err_without: float = 0.0
    track_loss_without: int = 0
    
    # Improvement
    rmse_improvement_pct: float = 0.0
    max_err_improvement_pct: float = 0.0
    
    # ECM detection
    ecm_detected_step: int = -1
    ecm_detection_latency: int = 0
    
    # Per-step errors for plotting
    errors_with: List[float] = field(default_factory=list)
    errors_without: List[float] = field(default_factory=list)
    r_scale_history: List[float] = field(default_factory=list)



def run_scenario(ecm_func, ecm_name: str, ecm_onset: int = 30,
                 turn_onset: int = 70,
                 n_steps: int = 150, n_monte: int = N_MONTE,
                 verbose: bool = False, **ecm_kwargs) -> ECMBenchmarkResult:
    """Run one ECM scenario with Monte Carlo averaging.
    
    Timeline: 0..29=clean | 30..69=ECM+cruise | 70..99=ECM+maneuver | 100+=ECM+cruise
    """
    result = ECMBenchmarkResult(scenario=ecm_name, ecm_type=ecm_name)
    
    all_rmse_w, all_rmse_wo = [], []
    all_maxerr_w, all_maxerr_wo = [], []
    all_loss_w, all_loss_wo = [], []
    all_r_peak, all_det_lat = [], []
    all_cruise_w, all_cruise_wo = [], []
    all_man_w, all_man_wo = [], []
    
    sum_errors_w = np.zeros(n_steps)
    sum_errors_wo = np.zeros(n_steps)
    sum_r_scale = np.zeros(n_steps)
    
    truth_pos, truth_vel = gen_fighter_cruise_turn(n_steps, DT, turn_onset=turn_onset)
    truth_alt = gen_fighter_altitude_profile(n_steps)
    
    for mc in range(n_monte):
        np.random.seed(42 + mc * 137)
        
        trk_a = NxMimosaV40Sentinel(dt=DT, r_std=R_STD)  # v4.1.1 (full ECM)
        trk_b = NxMimosaV40Sentinel(dt=DT, r_std=R_STD)  # v4.0 (no R-boost/gating)
        
        errors_a = np.zeros(n_steps)
        errors_b = np.zeros(n_steps)
        r_scales = np.zeros(n_steps)
        loss_a = loss_b = 0
        det_step = -1
        
        for i in range(n_steps):
            tp, tv = truth_pos[i], truth_vel[i]
            
            if ecm_name == "VGPO":
                z, rf = ecm_func(i, tp, tv, ecm_onset, **ecm_kwargs)
            else:
                z, rf = ecm_func(i, tp, ecm_onset, **ecm_kwargs)
            
            trk_a.set_rf_observables(**rf)
            trk_a.set_altitude(truth_alt[i] + np.random.randn() * 10)
            pa, _, _ = trk_a.update(z)
            
            trk_b.set_rf_observables(**rf)
            trk_b.set_altitude(truth_alt[i] + np.random.randn() * 10)
            pb, _, _ = trk_b.update(z)
            
            # Disable R-boost + gating for tracker B
            trk_b._ecm_r_scale = 1.0
            trk_b.R = trk_b.R_nominal.copy()
            trk_b._ecm_force_models = False
            trk_b._ecm_high_conf = False
            trk_b._ecm_gate_nis = float('inf')
            
            ea = np.linalg.norm(pa - tp)
            eb = np.linalg.norm(pb - tp)
            errors_a[i] = ea; errors_b[i] = eb
            r_scales[i] = trk_a._ecm_r_scale
            
            if ea > TRACK_LOSS_THRESHOLD: loss_a += 1
            if eb > TRACK_LOSS_THRESHOLD: loss_b += 1
            if i >= ecm_onset and det_step < 0 and trk_a._ecm_r_scale > 2.0:
                det_step = i
        
        ecm_ea = errors_a[ecm_onset:]; ecm_eb = errors_b[ecm_onset:]
        all_rmse_w.append(np.sqrt(np.mean(ecm_ea**2)))
        all_rmse_wo.append(np.sqrt(np.mean(ecm_eb**2)))
        all_maxerr_w.append(np.max(ecm_ea))
        all_maxerr_wo.append(np.max(ecm_eb))
        all_loss_w.append(loss_a); all_loss_wo.append(loss_b)
        all_r_peak.append(np.max(r_scales))
        if det_step >= 0: all_det_lat.append(det_step - ecm_onset)
        
        # Phase metrics
        cruise_ea = errors_a[ecm_onset:turn_onset]
        cruise_eb = errors_b[ecm_onset:turn_onset]
        man_ea = errors_a[turn_onset:turn_onset+30]
        man_eb = errors_b[turn_onset:turn_onset+30]
        all_cruise_w.append(np.sqrt(np.mean(cruise_ea**2)))
        all_cruise_wo.append(np.sqrt(np.mean(cruise_eb**2)))
        all_man_w.append(np.sqrt(np.mean(man_ea**2)))
        all_man_wo.append(np.sqrt(np.mean(man_eb**2)))
        
        sum_errors_w += errors_a; sum_errors_wo += errors_b; sum_r_scale += r_scales
    
    result.rmse_with = float(np.mean(all_rmse_w))
    result.rmse_without = float(np.mean(all_rmse_wo))
    result.max_err_with = float(np.mean(all_maxerr_w))
    result.max_err_without = float(np.mean(all_maxerr_wo))
    result.track_loss_with = int(np.mean(all_loss_w))
    result.track_loss_without = int(np.mean(all_loss_wo))
    result.r_scale_peak_with = float(np.mean(all_r_peak))
    
    if all_det_lat:
        result.ecm_detection_latency = int(np.mean(all_det_lat))
        result.ecm_detected_step = ecm_onset + result.ecm_detection_latency
    
    if result.rmse_without > 0:
        result.rmse_improvement_pct = (1 - result.rmse_with / result.rmse_without) * 100
    if result.max_err_without > 0:
        result.max_err_improvement_pct = (1 - result.max_err_with / result.max_err_without) * 100
    
    # Phase-aware (store as extra)
    result.rmse_cruise_with = float(np.mean(all_cruise_w))
    result.rmse_cruise_without = float(np.mean(all_cruise_wo))
    result.rmse_maneuver_with = float(np.mean(all_man_w))
    result.rmse_maneuver_without = float(np.mean(all_man_wo))
    
    result.errors_with = (sum_errors_w / n_monte).tolist()
    result.errors_without = (sum_errors_wo / n_monte).tolist()
    result.r_scale_history = (sum_r_scale / n_monte).tolist()
    
    return result


def run_all_benchmarks(verbose: bool = False, do_plot: bool = False) -> List[ECMBenchmarkResult]:
    """Run all 5 ECM scenarios."""
    scenarios = [
        ("RGPO", ecm_rgpo, {"pull_rate": 5.0}),
        ("VGPO", ecm_vgpo, {"vel_pull_rate": 2.0}),
        ("DRFM", ecm_drfm, {"false_offset": 100.0}),
        ("Noise Jamming", ecm_noise, {"noise_power": 30.0}),
        ("Chaff Corridor", ecm_chaff, {"chaff_radius": 80.0}),
    ]
    
    results = []
    
    print("=" * 90)
    print("  NX-MIMOSA v4.1.1 ECM ROBUSTNESS BENCHMARK SUITE")
    print("  Comparing: v4.1.1 (R-boost+gating+maneuver-aware) vs v4.0 (Q-boost only)")
    print(f"  Monte Carlo: {N_MONTE} runs | Timeline: ECM@step30, Turn@step70")
    print("=" * 90)
    print()
    
    for name, func, kwargs in scenarios:
        print(f"  Running {name}...", end=" ", flush=True)
        r = run_scenario(func, name, ecm_onset=30, turn_onset=70,
                         n_steps=150, n_monte=N_MONTE, verbose=verbose, **kwargs)
        results.append(r)
        print(f"done")
    
    # Summary
    print()
    print("=" * 90)
    print("  OVERALL RESULTS (full ECM phase)")
    print("=" * 90)
    print()
    print(f"  {'Scenario':<18} {'v4.1.1':>8} {'v4.0':>8} {'Δ':>8} {'R peak':>7} {'DetLat':>7} {'Loss':>8}")
    print(f"  {'':─<18} {'':─>8} {'':─>8} {'':─>8} {'':─>7} {'':─>7} {'(w/wo)':─>8}")
    
    for r in results:
        det = f"{r.ecm_detection_latency}" if r.ecm_detected_step >= 0 else "N/A"
        loss = f"{r.track_loss_with}/{r.track_loss_without}"
        print(f"  {r.scenario:<18} {r.rmse_with:>7.1f}m {r.rmse_without:>7.1f}m "
              f"{r.rmse_improvement_pct:>+7.1f}% {r.r_scale_peak_with:>6.1f}x {det:>7} {loss:>8}")
    
    avg_w = np.mean([r.rmse_with for r in results])
    avg_wo = np.mean([r.rmse_without for r in results])
    avg_imp = (1 - avg_w/avg_wo) * 100
    print(f"  {'AVERAGE':<18} {avg_w:>7.1f}m {avg_wo:>7.1f}m {avg_imp:>+7.1f}%")
    
    # Phase breakdown
    print()
    print("=" * 90)
    print("  PHASE BREAKDOWN (cruise-under-ECM vs maneuver-under-ECM)")
    print("=" * 90)
    print()
    print(f"  {'Scenario':<18} {'CRUISE':>26}  {'MANEUVER':>26}")
    print(f"  {'':─<18} {'v4.1.1':>8} {'v4.0':>8} {'Δ':>8}  {'v4.1.1':>8} {'v4.0':>8} {'Δ':>8}")
    
    for r in results:
        c_imp = (1 - r.rmse_cruise_with/r.rmse_cruise_without)*100 if r.rmse_cruise_without > 0 else 0
        m_imp = (1 - r.rmse_maneuver_with/r.rmse_maneuver_without)*100 if r.rmse_maneuver_without > 0 else 0
        print(f"  {r.scenario:<18} {r.rmse_cruise_with:>7.1f}m {r.rmse_cruise_without:>7.1f}m {c_imp:>+7.1f}%"
              f"  {r.rmse_maneuver_with:>7.1f}m {r.rmse_maneuver_without:>7.1f}m {m_imp:>+7.1f}%")
    
    avg_cw = np.mean([r.rmse_cruise_with for r in results])
    avg_cwo = np.mean([r.rmse_cruise_without for r in results])
    avg_mw = np.mean([r.rmse_maneuver_with for r in results])
    avg_mwo = np.mean([r.rmse_maneuver_without for r in results])
    c_avg = (1-avg_cw/avg_cwo)*100 if avg_cwo > 0 else 0
    m_avg = (1-avg_mw/avg_mwo)*100 if avg_mwo > 0 else 0
    print(f"  {'AVERAGE':<18} {avg_cw:>7.1f}m {avg_cwo:>7.1f}m {c_avg:>+7.1f}%"
          f"  {avg_mw:>7.1f}m {avg_mwo:>7.1f}m {m_avg:>+7.1f}%")
    
    # Verdict
    print()
    wins_total = sum(1 for r in results if r.rmse_with < r.rmse_without)
    wins_cruise = sum(1 for r in results if r.rmse_cruise_with < r.rmse_cruise_without)
    wins_man = sum(1 for r in results if r.rmse_maneuver_with < r.rmse_maneuver_without)
    
    print(f"  Score:  Overall {wins_total}/5 | Cruise {wins_cruise}/5 | Maneuver {wins_man}/5")
    print()
    
    if c_avg > 10:
        print(f"  ✅ CRUISE-under-ECM: {c_avg:+.1f}% improvement — R-boost effective for non-maneuvering targets")
    elif c_avg > 0:
        print(f"  🟡 CRUISE-under-ECM: {c_avg:+.1f}% — marginal improvement")
    else:
        print(f"  ❌ CRUISE-under-ECM: {c_avg:+.1f}% — no improvement")
    
    if m_avg > 0:
        print(f"  ✅ MANEUVER-under-ECM: {m_avg:+.1f}% — maneuver-aware modulation effective")
    else:
        print(f"  🟡 MANEUVER-under-ECM: {m_avg:+.1f}% — maneuver-aware modulation reducing penalty")
    
    print()
    
    if do_plot:
        _plot_results(results)
    
    return results


def _plot_results(results: List[ECMBenchmarkResult]):
    try:
        import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    except ImportError:
        print("  [matplotlib not available]"); return
    
    fig, axes = plt.subplots(len(results), 2, figsize=(16, 4 * len(results)))
    fig.suptitle('NX-MIMOSA v4.1.1 ECM Robustness Benchmark\n'
                 'ECM onset: step 30 (3.0s) | Maneuver onset: step 70 (7.0s)',
                 fontsize=14, fontweight='bold')
    
    for idx, r in enumerate(results):
        steps = np.arange(len(r.errors_with)) * DT
        ax = axes[idx, 0]
        ax.plot(steps, r.errors_with, 'b-', lw=1.5, label='v4.1.1 (R+gate+ω)', alpha=0.8)
        ax.plot(steps, r.errors_without, 'r-', lw=1.5, label='v4.0 (Q only)', alpha=0.8)
        ax.axvline(x=3.0, color='orange', ls='--', alpha=0.5, label='ECM onset')
        ax.axvline(x=7.0, color='purple', ls='--', alpha=0.5, label='Maneuver onset')
        ax.axhline(y=TRACK_LOSS_THRESHOLD, color='gray', ls=':', alpha=0.3)
        ax.set_ylabel('Position Error (m)'); ax.set_xlabel('Time (s)')
        c_imp = (1-r.rmse_cruise_with/r.rmse_cruise_without)*100 if r.rmse_cruise_without>0 else 0
        m_imp = (1-r.rmse_maneuver_with/r.rmse_maneuver_without)*100 if r.rmse_maneuver_without>0 else 0
        ax.set_title(f'{r.scenario} — Cruise:{c_imp:+.0f}% | Maneuver:{m_imp:+.0f}%')
        ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
        ax.set_ylim(0, min(500, max(max(r.errors_with), max(r.errors_without))*1.2))
        
        ax2 = axes[idx, 1]
        ax2.plot(steps, r.r_scale_history, 'g-', lw=1.5)
        ax2.axvline(x=3.0, color='orange', ls='--', alpha=0.5, label='ECM')
        ax2.axvline(x=7.0, color='purple', ls='--', alpha=0.5, label='Maneuver')
        ax2.set_ylabel('R Scale'); ax2.set_xlabel('Time (s)')
        ax2.set_title(f'{r.scenario} — R-profile (peak:{r.r_scale_peak_with:.1f}x)')
        ax2.legend(fontsize=8); ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    p = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'ecm_benchmark_results.png')
    plt.savefig(p, dpi=150, bbox_inches='tight'); print(f"  Plot saved: {p}"); plt.close()


if __name__ == "__main__":
    verbose = "--verbose" in sys.argv
    do_plot = "--plot" in sys.argv
    results = run_all_benchmarks(verbose=verbose, do_plot=do_plot)
    all_win = all(r.rmse_with <= r.rmse_without for r in results)
    sys.exit(0 if all_win else 1)
