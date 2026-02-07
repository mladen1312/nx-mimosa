"""NX-MIMOSA v6.0 C++ Core — Test Suite.

Tests: correctness, performance, API compatibility.
Run: pytest cpp/tests/test_cpp_core.py -v
"""
import sys
import os
import time
import numpy as np
import pytest

# Add C++ build to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'build'))
import _nx_core as nx


class TestBasicTracking:
    """Verify tracking correctness."""

    def test_single_target(self):
        """Single target with no clutter → 1 confirmed track."""
        tracker = nx.MultiTargetTracker(dt=1.0, r_std=10.0)
        pos = np.array([1000.0, 2000.0, 5000.0])

        for i in range(5):
            meas = (pos + np.array([50*i, 30*i, 0])).reshape(1, 3)
            meas += np.random.normal(0, 10, (1, 3))
            tracker.process_scan(meas, float(i))

        assert tracker.n_confirmed == 1

    def test_ten_targets(self):
        """10 well-separated targets → 10 confirmed tracks."""
        tracker = nx.MultiTargetTracker(dt=1.0, r_std=50.0)
        rng = np.random.default_rng(42)
        bases = rng.uniform(-50000, 50000, (10, 3))

        for i in range(5):
            meas = bases + rng.normal(0, 50, (10, 3))
            bases += rng.normal(20, 5, (10, 3))
            tracker.process_scan(meas, float(i))

        assert tracker.n_confirmed == 10

    def test_missed_detection(self):
        """Target survives missed scans (coasting)."""
        cfg = nx.TrackerConfig()
        cfg.dt = 1.0; cfg.r_std = 10.0; cfg.delete_misses = 5
        tracker = nx.MultiTargetTracker(cfg)

        pos = np.array([[0.0, 0.0, 100.0]])
        for i in range(4):
            tracker.process_scan(pos + i * 10, float(i))

        assert tracker.n_confirmed == 1

        # 3 empty scans (misses)
        for i in range(3):
            tracker.process_scan(np.empty((0, 3)), float(4 + i))

        # Should still not be deleted (miss_count=3 < delete_misses=5)
        active = [t for t in tracker.active_tracks]
        assert len(active) >= 1

    def test_2d_input(self):
        """Nx2 measurements auto-padded to 3D."""
        tracker = nx.MultiTargetTracker(dt=1.0, r_std=10.0)
        for i in range(5):
            meas = np.array([[100.0 + i*10, 200.0 + i*5]])
            tracker.process_scan(meas, float(i))
        assert tracker.n_confirmed == 1


class TestBulkStateExtraction:
    """Test efficient bulk state access."""

    def test_get_confirmed_states(self):
        tracker = nx.MultiTargetTracker(dt=1.0, r_std=50.0)
        rng = np.random.default_rng(42)
        N = 50
        bases = rng.uniform(-10000, 10000, (N, 3))

        for i in range(5):
            meas = bases + rng.normal(0, 50, (N, 3))
            bases += 10
            tracker.process_scan(meas, float(i))

        ids, states = tracker.get_confirmed_states()
        assert ids.shape == (N,)
        assert states.shape == (N, 6)
        # Positions should be reasonable
        assert np.all(np.abs(states[:, :3]) < 50000)

    def test_track_properties(self):
        tracker = nx.MultiTargetTracker(dt=1.0, r_std=10.0)
        for i in range(5):
            meas = np.array([[0.0, 0.0, 100.0]])
            tracker.process_scan(meas, float(i))

        tracks = tracker.confirmed_tracks
        assert len(tracks) == 1
        t = tracks[0]
        assert t.id >= 0
        assert t.hit_count >= 3
        pos = np.array(t.position)
        assert pos.shape == (3,)
        vel = np.array(t.velocity)
        assert vel.shape == (3,)
        cov = np.array(t.covariance)
        assert cov.shape == (6, 6)
        mp = np.array(t.model_probs)
        assert len(mp) == 4
        assert abs(mp.sum() - 1.0) < 1e-6


class TestPerformance:
    """Verify sub-50ms requirement at 1000 targets."""

    @pytest.mark.parametrize("n_targets,max_ms", [
        (100, 10),
        (500, 30),
        (761, 50),
        (1000, 50),
    ])
    def test_scan_time(self, n_targets, max_ms):
        """[REQ-V60-PERF-01] Scan time < {max_ms}ms for {n_targets} targets."""
        tracker = nx.MultiTargetTracker(dt=5.0, r_std=150.0)
        rng = np.random.default_rng(42)
        base = rng.uniform(-200000, 200000, (n_targets, 3))
        base[:, 2] = rng.uniform(5000, 15000, n_targets)

        # Warmup
        for i in range(3):
            meas = base + rng.normal(0, 150, (n_targets, 3))
            base += rng.normal(50, 10, (n_targets, 3))
            tracker.process_scan(meas, float(i * 5))

        # Timed scan
        meas = base + rng.normal(0, 150, (n_targets, 3))
        ms = tracker.process_scan(meas, 15.0)

        assert ms < max_ms, f"{n_targets} targets: {ms:.1f}ms > {max_ms}ms"
        assert tracker.n_confirmed >= n_targets * 0.95


class TestConfig:
    """Test configuration options."""

    def test_custom_config(self):
        cfg = nx.TrackerConfig()
        cfg.dt = 2.0
        cfg.r_std = 200.0
        cfg.gate_threshold = 25.0
        cfg.confirm_hits = 2
        cfg.delete_misses = 10
        tracker = nx.MultiTargetTracker(cfg)

        for i in range(3):
            meas = np.array([[0.0, 0.0, 100.0]])
            tracker.process_scan(meas, float(i * 2))

        assert tracker.n_confirmed == 1

    def test_convenience_constructor(self):
        tracker = nx.MultiTargetTracker(dt=3.0, r_std=100.0, q_base=2.0)
        assert tracker.cfg.dt == 3.0
        assert tracker.cfg.r_std == 100.0


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
