#!/usr/bin/env python3
"""NX-MIMOSA Quick Start: Track 20 targets with 6-model IMM.

Run:
    python examples/simple_radar_demo.py

Output:
    Scan-by-scan track updates with position, velocity, and track quality.
"""
import numpy as np

# Works both as installed package and from repo root
try:
    from nx_mimosa import MultiTargetTracker, compute_gospa
except ImportError:
    import sys; sys.path.insert(0, "python")
    from nx_mimosa_mtt import MultiTargetTracker, compute_gospa


def generate_crossing_targets(n_targets=20, n_scans=30, dt=5.0, noise_std=150.0):
    """Generate synthetic radar measurements for crossing targets."""
    rng = np.random.default_rng(42)
    ground_truth = []
    measurements = []
    
    for scan_idx in range(n_scans):
        t = scan_idx * dt
        gt_positions = []
        meas = []
        
        for i in range(n_targets):
            # Diverse trajectories: some straight, some turning
            angle = 2 * np.pi * i / n_targets
            speed = 200.0 + 50.0 * (i % 5)  # 200-400 m/s
            
            if i % 4 == 0:  # Coordinated turn
                turn_rate = 0.02  # rad/s
                x = speed / turn_rate * np.sin(turn_rate * t + angle)
                y = speed / turn_rate * (1 - np.cos(turn_rate * t + angle))
            else:  # Constant velocity
                x = speed * t * np.cos(angle) + 1000 * i
                y = speed * t * np.sin(angle) + 1000 * i
            
            z = 8000.0 + 1000.0 * (i % 3)  # FL260-FL330
            gt_positions.append([x, y, z])
            meas.append([x + rng.normal(0, noise_std),
                         y + rng.normal(0, noise_std),
                         z + rng.normal(0, noise_std / 3)])
        
        ground_truth.append(np.array(gt_positions))
        measurements.append(np.array(meas))
    
    return measurements, ground_truth


def main():
    print("NX-MIMOSA v5.9.3 — Quick Start Demo")
    print("=" * 50)
    
    # Generate scenario
    n_targets, n_scans = 20, 30
    measurements, ground_truth = generate_crossing_targets(n_targets, n_scans)
    
    # Create tracker (air surveillance defaults)
    tracker = MultiTargetTracker(dt=5.0, r_std=150.0, domain='air')
    
    print(f"\nScenario: {n_targets} targets, {n_scans} scans, σ=150m")
    print(f"Tracker: 6-model IMM + GNN association")
    print("-" * 50)
    
    # Process scans
    for scan_idx, meas in enumerate(measurements):
        tracker.process_scan(meas)
        
        n_confirmed = len(tracker.confirmed_tracks)
        n_tentative = len(tracker.tentative_tracks) if hasattr(tracker, 'tentative_tracks') else 0
        
        # Compute GOSPA if we have ground truth
        if hasattr(tracker, 'confirmed_tracks') and len(tracker.confirmed_tracks) > 0:
            est_positions = np.array([t.position for t in tracker.confirmed_tracks])
            gospa = compute_gospa(est_positions, ground_truth[scan_idx])
            
            if (scan_idx + 1) % 5 == 0 or scan_idx == 0:
                print(f"  Scan {scan_idx+1:3d}: {n_confirmed:2d} confirmed tracks | "
                      f"GOSPA={gospa['total']:.1f}m "
                      f"(loc={gospa['localization']:.1f}, "
                      f"miss={gospa['missed']:.1f}, "
                      f"false={gospa['false']:.1f})")
    
    # Final summary
    print("-" * 50)
    print(f"\nFinal: {len(tracker.confirmed_tracks)} confirmed tracks "
          f"(expected {n_targets})")
    
    # Show sample track details
    print("\nSample tracks:")
    for track in list(tracker.confirmed_tracks)[:5]:
        pos = track.position
        vel = track.velocity if hasattr(track, 'velocity') else [0, 0, 0]
        speed = np.linalg.norm(vel)
        print(f"  Track {track.track_id:4d}: "
              f"pos=({pos[0]:8.0f}, {pos[1]:8.0f}, {pos[2]:8.0f})m  "
              f"speed={speed:.0f} m/s")
    
    print(f"\n✓ Demo complete. {n_targets} targets tracked over {n_scans} scans.")
    print("  Install: pip install nx-mimosa")
    print("  Docs:    https://nx-mimosa.readthedocs.io")


if __name__ == "__main__":
    main()
