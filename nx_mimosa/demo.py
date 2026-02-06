#!/usr/bin/env python3
"""
NX-MIMOSA Demo — Zero-Code Tracker Demonstration
==================================================

Run with:
    python -m nx_mimosa.demo              # All 3 scenarios
    python -m nx_mimosa.demo --scenario 1 # Fighter intercept only
    python -m nx_mimosa.demo --save       # Save PNGs instead of display

Shows NX-MIMOSA tracking in action on 3 scenarios:
  1. Fighter Intercept — high-g maneuvering (IMM model switching)
  2. Multi-Target Clutter — 3 targets + 10 false alarms per scan
  3. ECM Engagement — noise jamming mid-track (adaptive gating)

Nexellum d.o.o. — Dr. Mladen Mešter — mladen@nexellum.com
"""

import argparse
import sys
import os
import numpy as np

# Add parent paths for development
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


def _get_tracker():
    """Import tracker — handle both installed and development paths."""
    try:
        from nx_mimosa_mtt import MultiTargetTracker, TrackStatus
        from nx_mimosa_intelligence import (
            IntelligencePipeline, ECMDetector, ECMStatus,
        )
    except ImportError:
        from python.nx_mimosa_mtt import MultiTargetTracker, TrackStatus
        from python.nx_mimosa_intelligence import (
            IntelligencePipeline, ECMDetector, ECMStatus,
        )
    return MultiTargetTracker, TrackStatus, IntelligencePipeline, ECMDetector, ECMStatus


def generate_fighter_intercept(n_scans=80, dt=1.0, seed=42):
    """Scenario 1: Fighter intercept with 7g break turn.
    
    Phase 1 (0-30s):  Inbound at Mach 1.5, straight
    Phase 2 (30-50s): 7g break turn (evasive)
    Phase 3 (50-80s): Egress at Mach 1.2
    """
    rng = np.random.RandomState(seed)
    truth = np.zeros((n_scans, 3))
    pos = np.array([50000.0, 0.0, 10000.0])
    vel = np.array([-500.0, 0.0, 0.0])  # Mach 1.5 inbound
    
    for t in range(n_scans):
        if t < 30:
            # Straight inbound
            pass
        elif t < 50:
            # 7g break turn — turning left and climbing
            angle = (t - 30) * 0.08  # ~4.6 deg/s turn rate
            speed = np.linalg.norm(vel[:2])
            vel[0] = -speed * np.cos(angle)
            vel[1] = speed * np.sin(angle)
            vel[2] = 20.0  # Climb
        else:
            # Egress — reduce to Mach 1.2
            vel *= 0.998
        
        pos = pos + vel * dt
        truth[t] = pos.copy()
    
    # Generate noisy measurements
    measurements = []
    for t in range(n_scans):
        noise = rng.randn(3) * np.array([80.0, 80.0, 30.0])
        meas = truth[t] + noise
        measurements.append(meas.reshape(1, 3))
    
    return truth, measurements, {
        'name': 'Fighter Intercept',
        'description': 'Mach 1.5 inbound → 7g break turn → Mach 1.2 egress',
        'phases': [(0, 30, 'Inbound'), (30, 50, '7g Turn'), (50, 80, 'Egress')],
    }


def generate_multi_target_clutter(n_scans=60, dt=1.0, seed=42):
    """Scenario 2: 3 targets + dense Poisson clutter (λ=10)."""
    rng = np.random.RandomState(seed)
    
    # 3 targets
    targets = [
        {'pos': np.array([5000.0, -3000.0, 8000.0]),
         'vel': np.array([200.0, 100.0, 0.0])},
        {'pos': np.array([-2000.0, 10000.0, 5000.0]),
         'vel': np.array([150.0, -80.0, 10.0])},
        {'pos': np.array([15000.0, 5000.0, 12000.0]),
         'vel': np.array([-100.0, 50.0, -5.0])},
    ]
    
    truth = np.zeros((n_scans, 3, 3))  # (scans, targets, xyz)
    measurements = []
    
    for t in range(n_scans):
        scan_meas = []
        for i, tgt in enumerate(targets):
            pos = tgt['pos'] + tgt['vel'] * t * dt
            truth[t, i] = pos
            
            # Detection probability 0.9
            if rng.rand() < 0.9:
                noise = rng.randn(3) * 50.0
                scan_meas.append(pos + noise)
        
        # Poisson clutter (λ=10)
        n_clutter = rng.poisson(10)
        for _ in range(n_clutter):
            clutter = rng.uniform([-5000, -5000, 0], [30000, 20000, 15000])
            scan_meas.append(clutter)
        
        if scan_meas:
            measurements.append(np.array(scan_meas))
        else:
            measurements.append(np.zeros((0, 3)))
    
    return truth, measurements, {
        'name': 'Multi-Target Clutter',
        'description': '3 CV targets + Poisson(10) clutter per scan',
        'n_targets': 3,
    }


def generate_ecm_engagement(n_scans=100, dt=1.0, seed=42):
    """Scenario 3: Single target with noise jamming phase.
    
    Phase 1 (0-30s):  Clean tracking
    Phase 2 (30-70s): Noise jamming (5× measurement noise)
    Phase 3 (70-100s): Post-jamming recovery
    """
    rng = np.random.RandomState(seed)
    
    vel = np.array([250.0, 50.0, 0.0])
    pos0 = np.array([10000.0, 5000.0, 8000.0])
    truth = np.zeros((n_scans, 3))
    measurements = []
    ecm_active = np.zeros(n_scans, dtype=bool)
    
    for t in range(n_scans):
        truth[t] = pos0 + vel * t * dt
        
        in_ecm = 30 <= t < 70
        ecm_active[t] = in_ecm
        noise_mult = 5.0 if in_ecm else 1.0
        snr = 5.0 if in_ecm else 22.0
        
        noise = rng.randn(3) * 50.0 * noise_mult
        meas = truth[t] + noise
        measurements.append(meas.reshape(1, 3))
    
    return truth, measurements, {
        'name': 'ECM Engagement',
        'description': 'Clean → Noise Jamming (5×) → Recovery',
        'ecm_active': ecm_active,
        'phases': [(0, 30, 'Clean'), (30, 70, 'Jamming'), (70, 100, 'Recovery')],
    }


def run_tracker_on_scenario(measurements, meta, is_ecm=False):
    """Run MultiTargetTracker on a scenario, return track history."""
    MTT, TrackStatus, IntelPipe, ECMDet, ECMStat = _get_tracker()
    
    tracker = MTT(dt=1.0, r_std=50.0, domain="military")
    ecm_det = ECMDet(history_len=20, snr_threshold_db=8.0) if is_ecm else None
    
    track_history = {}  # {track_id: [(scan, x, y, z), ...]}
    
    for scan_idx, meas in enumerate(measurements):
        # ECM-aware gating
        if is_ecm and 'ecm_active' in meta:
            in_ecm = meta['ecm_active'][scan_idx]
            if in_ecm:
                tracker.set_ecm_state(True, gate_multiplier=3.0,
                                      coast_extension=5, r_scale=3.0)
                tracker.inflate_track_R(3.0)
            elif scan_idx > 75:
                tracker.set_ecm_state(False)
        
        tracker.process_scan(meas)
        
        for track in tracker.tracks:
            if track.status != TrackStatus.DELETED:
                tid = track.track_id
                if tid not in track_history:
                    track_history[tid] = []
                pos = track.filter.position
                track_history[tid].append((scan_idx, pos[0], pos[1], pos[2]))
    
    return track_history


def plot_scenario_1(truth, measurements, track_history, meta, save_path=None):
    """Plot Fighter Intercept — XY plane with phase markers."""
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    
    fig, axes = plt.subplots(1, 2, figsize=(16, 7))
    fig.suptitle(f'NX-MIMOSA Demo: {meta["name"]}', fontsize=14, fontweight='bold',
                 color='#E0E0E0')
    fig.patch.set_facecolor('#1a1a2e')
    
    for ax in axes:
        ax.set_facecolor('#16213e')
        ax.tick_params(colors='#A0A0A0')
        ax.xaxis.label.set_color('#C0C0C0')
        ax.yaxis.label.set_color('#C0C0C0')
        ax.title.set_color('#E0E0E0')
        for spine in ax.spines.values():
            spine.set_color('#333355')
    
    ax1, ax2 = axes
    
    # XY plane
    ax1.set_title('XY Plane — Track vs Truth')
    ax1.plot(truth[:, 0] / 1000, truth[:, 1] / 1000, 'w-', linewidth=2,
             alpha=0.4, label='Truth')
    
    # Phase markers
    colors_phase = ['#00ff88', '#ff4444', '#4488ff']
    for (start, end, label), color in zip(meta['phases'], colors_phase):
        ax1.plot(truth[start:end, 0] / 1000, truth[start:end, 1] / 1000,
                 '-', color=color, linewidth=2.5, label=label)
    
    # Tracks
    track_colors = ['#ffaa00', '#00ddff', '#ff66cc', '#88ff44']
    for i, (tid, pts) in enumerate(sorted(track_history.items())):
        if len(pts) > 3:
            pts_arr = np.array(pts)
            color = track_colors[i % len(track_colors)]
            ax1.plot(pts_arr[:, 1] / 1000, pts_arr[:, 2] / 1000,
                     '--', color=color, linewidth=1.5, alpha=0.9,
                     label=f'Track {tid}')
    
    ax1.set_xlabel('X (km)')
    ax1.set_ylabel('Y (km)')
    ax1.legend(loc='upper left', fontsize=8, facecolor='#1a1a2e',
               edgecolor='#444466', labelcolor='#C0C0C0')
    ax1.grid(True, alpha=0.15, color='#445577')
    
    # Position error over time
    ax2.set_title('Position Error (m)')
    for i, (tid, pts) in enumerate(sorted(track_history.items())):
        if len(pts) > 3:
            pts_arr = np.array(pts)
            errors = []
            for row in pts_arr:
                scan = int(row[0])
                if scan < len(truth):
                    err = np.linalg.norm(row[1:4] - truth[scan])
                    errors.append((scan, err))
            if errors:
                err_arr = np.array(errors)
                color = track_colors[i % len(track_colors)]
                ax2.plot(err_arr[:, 0], err_arr[:, 1], '-', color=color,
                         linewidth=1.5, label=f'Track {tid}')
    
    # Phase background
    for (start, end, label), color in zip(meta['phases'], colors_phase):
        ax2.axvspan(start, end, alpha=0.08, color=color)
        ax2.text((start + end) / 2, ax2.get_ylim()[1] * 0.9 if ax2.get_ylim()[1] > 0 else 500,
                 label, ha='center', fontsize=8, color=color, alpha=0.7)
    
    ax2.set_xlabel('Scan')
    ax2.set_ylabel('Error (m)')
    ax2.legend(loc='upper right', fontsize=8, facecolor='#1a1a2e',
               edgecolor='#444466', labelcolor='#C0C0C0')
    ax2.grid(True, alpha=0.15, color='#445577')
    
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, facecolor=fig.get_facecolor())
        print(f"  Saved: {save_path}")
    return fig


def plot_scenario_2(truth, measurements, track_history, meta, save_path=None):
    """Plot Multi-Target Clutter — 3D scatter + tracks."""
    import matplotlib.pyplot as plt
    
    fig = plt.figure(figsize=(14, 7))
    fig.suptitle(f'NX-MIMOSA Demo: {meta["name"]}', fontsize=14,
                 fontweight='bold', color='#E0E0E0')
    fig.patch.set_facecolor('#1a1a2e')
    
    ax1 = fig.add_subplot(121)
    ax2 = fig.add_subplot(122)
    
    for ax in [ax1, ax2]:
        ax.set_facecolor('#16213e')
        ax.tick_params(colors='#A0A0A0')
        ax.xaxis.label.set_color('#C0C0C0')
        ax.yaxis.label.set_color('#C0C0C0')
        ax.title.set_color('#E0E0E0')
        for spine in ax.spines.values():
            spine.set_color('#333355')
    
    # XY plane with truth + clutter + tracks
    ax1.set_title('XY Plane — Targets + Clutter')
    
    # Plot clutter from a few scans (sample)
    for scan_idx in range(0, len(measurements), 5):
        m = measurements[scan_idx]
        if len(m) > 3:
            ax1.scatter(m[3:, 0] / 1000, m[3:, 1] / 1000,
                        s=3, color='#ff4444', alpha=0.15, zorder=1)
    
    # Truth trajectories
    truth_colors = ['#00ff88', '#4488ff', '#ffaa00']
    for i in range(meta['n_targets']):
        ax1.plot(truth[:, i, 0] / 1000, truth[:, i, 1] / 1000,
                 '-', color=truth_colors[i], linewidth=2, alpha=0.5,
                 label=f'Truth {i+1}')
    
    # Tracks
    track_colors = ['#ffdd00', '#00ddff', '#ff66cc', '#88ff44', '#ff8800',
                    '#cc44ff', '#44ffcc', '#ff4488']
    for i, (tid, pts) in enumerate(sorted(track_history.items())):
        if len(pts) > 5:
            pts_arr = np.array(pts)
            color = track_colors[i % len(track_colors)]
            ax1.plot(pts_arr[:, 1] / 1000, pts_arr[:, 2] / 1000,
                     '--', color=color, linewidth=1.2, alpha=0.8,
                     label=f'Trk {tid}' if i < 6 else None)
    
    ax1.set_xlabel('X (km)')
    ax1.set_ylabel('Y (km)')
    ax1.legend(loc='upper left', fontsize=7, facecolor='#1a1a2e',
               edgecolor='#444466', labelcolor='#C0C0C0', ncol=2)
    ax1.grid(True, alpha=0.15, color='#445577')
    
    # Track count over time
    ax2.set_title('Track Count Over Time')
    scan_range = range(len(measurements))
    confirmed_counts = []
    
    for scan_idx in scan_range:
        count = 0
        for tid, pts in track_history.items():
            scans_in = [p[0] for p in pts]
            if scan_idx in scans_in:
                count += 1
        confirmed_counts.append(count)
    
    ax2.fill_between(scan_range, confirmed_counts, alpha=0.3, color='#00ff88')
    ax2.plot(scan_range, confirmed_counts, '-', color='#00ff88', linewidth=2)
    ax2.axhline(y=meta['n_targets'], color='#ffffff', linestyle='--',
                alpha=0.4, label=f'Truth: {meta["n_targets"]} targets')
    ax2.set_xlabel('Scan')
    ax2.set_ylabel('Active Tracks')
    ax2.legend(facecolor='#1a1a2e', edgecolor='#444466', labelcolor='#C0C0C0')
    ax2.grid(True, alpha=0.15, color='#445577')
    
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, facecolor=fig.get_facecolor())
        print(f"  Saved: {save_path}")
    return fig


def plot_scenario_3(truth, measurements, track_history, meta, save_path=None):
    """Plot ECM Engagement — error with jamming zone."""
    import matplotlib.pyplot as plt
    
    fig, axes = plt.subplots(1, 2, figsize=(16, 7))
    fig.suptitle(f'NX-MIMOSA Demo: {meta["name"]}', fontsize=14,
                 fontweight='bold', color='#E0E0E0')
    fig.patch.set_facecolor('#1a1a2e')
    
    for ax in axes:
        ax.set_facecolor('#16213e')
        ax.tick_params(colors='#A0A0A0')
        ax.xaxis.label.set_color('#C0C0C0')
        ax.yaxis.label.set_color('#C0C0C0')
        ax.title.set_color('#E0E0E0')
        for spine in ax.spines.values():
            spine.set_color('#333355')
    
    ax1, ax2 = axes
    
    # XY with ECM zone
    ax1.set_title('XY Track — ECM Adaptive Gating')
    ax1.plot(truth[:, 0] / 1000, truth[:, 1] / 1000, 'w-',
             linewidth=2, alpha=0.4, label='Truth')
    
    # Color truth by phase
    colors_phase = ['#00ff88', '#ff4444', '#4488ff']
    for (start, end, label), color in zip(meta['phases'], colors_phase):
        ax1.plot(truth[start:end, 0] / 1000, truth[start:end, 1] / 1000,
                 '-', color=color, linewidth=2.5, label=label)
    
    for i, (tid, pts) in enumerate(sorted(track_history.items())):
        if len(pts) > 3:
            pts_arr = np.array(pts)
            ax1.plot(pts_arr[:, 1] / 1000, pts_arr[:, 2] / 1000,
                     '--', color='#ffaa00', linewidth=1.5, alpha=0.9,
                     label=f'Track {tid}' if i == 0 else None)
    
    ax1.set_xlabel('X (km)')
    ax1.set_ylabel('Y (km)')
    ax1.legend(loc='upper left', fontsize=8, facecolor='#1a1a2e',
               edgecolor='#444466', labelcolor='#C0C0C0')
    ax1.grid(True, alpha=0.15, color='#445577')
    
    # Position error with jamming zone
    ax2.set_title('Position Error Under ECM')
    
    # Jamming zone
    ax2.axvspan(30, 70, alpha=0.15, color='#ff4444', label='Jamming Active')
    ax2.axvline(30, color='#ff4444', linestyle=':', alpha=0.5)
    ax2.axvline(70, color='#ff4444', linestyle=':', alpha=0.5)
    
    for i, (tid, pts) in enumerate(sorted(track_history.items())):
        if len(pts) > 3:
            pts_arr = np.array(pts)
            errors = []
            for row in pts_arr:
                scan = int(row[0])
                if scan < len(truth):
                    err = np.linalg.norm(row[1:4] - truth[scan])
                    errors.append((scan, err))
            if errors:
                err_arr = np.array(errors)
                ax2.plot(err_arr[:, 0], err_arr[:, 1], '-', color='#ffaa00',
                         linewidth=1.5, label=f'Track {tid}' if i == 0 else None)
    
    # Phase labels
    ax2.text(15, ax2.get_ylim()[1] * 0.85 if ax2.get_ylim()[1] > 0 else 800,
             'Clean', ha='center', fontsize=10, color='#00ff88', fontweight='bold')
    ax2.text(50, ax2.get_ylim()[1] * 0.85 if ax2.get_ylim()[1] > 0 else 800,
             'JAMMING', ha='center', fontsize=10, color='#ff4444', fontweight='bold')
    ax2.text(85, ax2.get_ylim()[1] * 0.85 if ax2.get_ylim()[1] > 0 else 800,
             'Recovery', ha='center', fontsize=10, color='#4488ff', fontweight='bold')
    
    ax2.set_xlabel('Scan')
    ax2.set_ylabel('Error (m)')
    ax2.legend(loc='upper right', fontsize=8, facecolor='#1a1a2e',
               edgecolor='#444466', labelcolor='#C0C0C0')
    ax2.grid(True, alpha=0.15, color='#445577')
    
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, facecolor=fig.get_facecolor())
        print(f"  Saved: {save_path}")
    return fig


def run_demo(scenario=None, save=False, output_dir='.'):
    """Run NX-MIMOSA demo scenarios."""
    import matplotlib
    if save:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    
    banner = """
╔══════════════════════════════════════════════════════════════╗
║           NX-MIMOSA — Adaptive Radar Tracker                ║
║         "When Tracking Fails, People Die."                  ║
║                                                              ║
║  Nexellum d.o.o. — Dr. Mladen Mešter                       ║
║  mladen@nexellum.com — https://github.com/mladen1312        ║
╚══════════════════════════════════════════════════════════════╝
"""
    print(banner)
    
    os.makedirs(output_dir, exist_ok=True)
    
    scenarios_to_run = []
    if scenario is None or scenario == 0:
        scenarios_to_run = [1, 2, 3]
    else:
        scenarios_to_run = [scenario]
    
    for s in scenarios_to_run:
        if s == 1:
            print("━━━ Scenario 1: Fighter Intercept (7g Break Turn) ━━━")
            truth, meas, meta = generate_fighter_intercept()
            history = run_tracker_on_scenario(meas, meta)
            
            n_scans = len(meas)
            mean_err = _compute_mean_error(history, truth, n_scans)
            print(f"  Scans: {n_scans} | Tracks: {len(history)} | Mean Error: {mean_err:.0f}m")
            
            path = os.path.join(output_dir, 'demo_fighter_intercept.png') if save else None
            plot_scenario_1(truth, meas, history, meta, save_path=path)
        
        elif s == 2:
            print("━━━ Scenario 2: Multi-Target Clutter (3 targets + λ=10) ━━━")
            truth, meas, meta = generate_multi_target_clutter()
            history = run_tracker_on_scenario(meas, meta)
            
            print(f"  Scans: {len(meas)} | Tracks created: {len(history)} | "
                  f"True targets: {meta['n_targets']}")
            
            path = os.path.join(output_dir, 'demo_multi_target.png') if save else None
            plot_scenario_2(truth, meas, history, meta, save_path=path)
        
        elif s == 3:
            print("━━━ Scenario 3: ECM Engagement (Noise Jamming) ━━━")
            truth, meas, meta = generate_ecm_engagement()
            history = run_tracker_on_scenario(meas, meta, is_ecm=True)
            
            mean_err = _compute_mean_error(history, truth, len(meas))
            print(f"  Scans: {len(meas)} | Tracks: {len(history)} | Mean Error: {mean_err:.0f}m")
            
            path = os.path.join(output_dir, 'demo_ecm_engagement.png') if save else None
            plot_scenario_3(truth, meas, history, meta, save_path=path)
    
    print("\n━━━ Demo complete. ━━━")
    if save:
        print(f"  PNGs saved to: {output_dir}")
    else:
        try:
            plt.show()
        except Exception:
            print("  (No display available — use --save to export PNGs)")


def _compute_mean_error(track_history, truth, n_scans):
    """Compute mean position error for closest track to single-target truth."""
    errors = []
    for scan_idx in range(n_scans):
        min_err = float('inf')
        for tid, pts in track_history.items():
            for row in pts:
                if int(row[0]) == scan_idx:
                    if truth.ndim == 2:
                        err = np.linalg.norm(row[1:4] - truth[scan_idx])
                    else:
                        # Multi-target: find closest truth
                        err = min(np.linalg.norm(row[1:4] - truth[scan_idx, j])
                                  for j in range(truth.shape[1]))
                    min_err = min(min_err, err)
        if min_err < 1e9:
            errors.append(min_err)
    return float(np.mean(errors)) if errors else float('inf')


def main():
    parser = argparse.ArgumentParser(
        description='NX-MIMOSA Demo — Zero-Code Tracker Demonstration',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Scenarios:
  1  Fighter Intercept — Mach 1.5, 7g break turn, IMM model switching
  2  Multi-Target Clutter — 3 targets + Poisson(10) false alarms
  3  ECM Engagement — noise jamming, adaptive gating recovery

Examples:
  python -m nx_mimosa.demo              # Run all 3 scenarios
  python -m nx_mimosa.demo --scenario 1 # Fighter intercept only
  python -m nx_mimosa.demo --save       # Save PNGs (headless)
""")
    parser.add_argument('--scenario', '-s', type=int, default=None,
                        choices=[1, 2, 3],
                        help='Scenario number (default: all)')
    parser.add_argument('--save', action='store_true',
                        help='Save PNG files instead of displaying')
    parser.add_argument('--output-dir', '-o', type=str, default='.',
                        help='Output directory for PNGs (default: current)')
    
    args = parser.parse_args()
    run_demo(scenario=args.scenario, save=args.save, output_dir=args.output_dir)


if __name__ == '__main__':
    main()
