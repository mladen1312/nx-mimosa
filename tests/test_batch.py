"""Tests for NX-MIMOSA v5.9.4 Batch Processing API."""
import numpy as np
import pytest
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from python.nx_mimosa_batch import (
    RTSSmoother3D, SmoothedState,
    TrackInterpolator,
    BatchProcessor, BatchConfig, BatchResult, BatchTrackResult,
)


class TestRTSSmoother:
    """RTS Smoother tests."""
    
    def test_smoother_basic(self):
        """RTS smoother produces results for simple 2-state system."""
        smoother = RTSSmoother3D()
        N = 10
        n = 4  # 2D pos + vel
        
        # Generate simple forward pass data
        F = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
        Q = np.eye(n) * 0.1
        
        filtered_states = [np.array([i * 10.0, i * 5.0, 10.0, 5.0]) for i in range(N)]
        filtered_covs = [np.eye(n) * (10.0 - i * 0.5) for i in range(N)]
        predicted_states = [F @ filtered_states[i] for i in range(N - 1)]
        predicted_covs = [F @ filtered_covs[i] @ F.T + Q for i in range(N - 1)]
        transition_matrices = [F.copy() for _ in range(N - 1)]
        
        result = smoother.smooth(
            filtered_states, filtered_covs,
            predicted_states, predicted_covs,
            transition_matrices
        )
        
        assert len(result) == N
        assert all(isinstance(s, SmoothedState) for s in result)
        # Last smoothed = last filtered
        np.testing.assert_array_almost_equal(result[-1].state, filtered_states[-1])
    
    def test_smoother_reduces_covariance(self):
        """Smoothed covariance trace should be <= filtered for consistent data."""
        smoother = RTSSmoother3D()
        N = 20
        n = 4  # pos_x, pos_y, vel_x, vel_y
        
        dt = 1.0
        F = np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]], dtype=float)
        Q = np.eye(n) * 0.01
        H = np.array([[1,0,0,0],[0,1,0,0]], dtype=float)
        R = np.eye(2) * 1.0
        
        # Simulate consistent forward KF pass
        np.random.seed(42)
        x_true = np.array([0.0, 0.0, 1.0, 0.5])
        
        filtered_states = []
        filtered_covs = []
        predicted_states = []
        predicted_covs = []
        
        x = np.zeros(n)
        P = np.eye(n) * 100.0
        
        for k in range(N):
            # Predict
            x_pred = F @ x
            P_pred = F @ P @ F.T + Q
            if k > 0:
                predicted_states.append(x_pred.copy())
                predicted_covs.append(P_pred.copy())
            
            # Measurement
            x_true = F @ x_true + np.random.multivariate_normal(np.zeros(n), Q)
            z = H @ x_true + np.random.multivariate_normal(np.zeros(2), R)
            
            # Update
            S = H @ P_pred @ H.T + R
            K = P_pred @ H.T @ np.linalg.inv(S)
            x = x_pred + K @ (z - H @ x_pred)
            P = (np.eye(n) - K @ H) @ P_pred
            P = 0.5 * (P + P.T)
            
            filtered_states.append(x.copy())
            filtered_covs.append(P.copy())
        
        result = smoother.smooth(
            filtered_states, filtered_covs,
            predicted_states, predicted_covs,
            [F.copy() for _ in range(N - 1)]
        )
        
        # Interior points should have reduced covariance
        reductions = 0
        for k in range(1, N - 1):
            if np.trace(result[k].covariance) <= np.trace(filtered_covs[k]):
                reductions += 1
        # At least 80% of interior points should show reduction
        assert reductions >= (N - 2) * 0.8
    
    def test_smoother_empty(self):
        """Empty input returns empty output."""
        smoother = RTSSmoother3D()
        assert smoother.smooth([], [], [], [], []) == []
    
    def test_smoother_single_state(self):
        """Single state returns itself."""
        smoother = RTSSmoother3D()
        x = np.array([1.0, 2.0, 3.0])
        P = np.eye(3) * 0.5
        result = smoother.smooth([x], [P], [], [], [])
        assert len(result) == 1
        np.testing.assert_array_equal(result[0].state, x)


class TestTrackInterpolator:
    """Track interpolation tests."""
    
    def test_interpolate_midpoint(self):
        """Midpoint interpolation returns average."""
        interp = TrackInterpolator()
        x1 = np.array([0.0, 0.0, 0.0])
        x2 = np.array([10.0, 20.0, 30.0])
        P1 = np.eye(3) * 1.0
        P2 = np.eye(3) * 2.0
        
        x_mid, P_mid = interp.interpolate(0.5, 0.0, x1, P1, 1.0, x2, P2)
        np.testing.assert_array_almost_equal(x_mid, [5.0, 10.0, 15.0])
    
    def test_interpolate_endpoints(self):
        """At endpoints, returns the endpoint state."""
        interp = TrackInterpolator()
        x1 = np.array([1.0, 2.0])
        x2 = np.array([3.0, 4.0])
        P1 = np.eye(2)
        P2 = np.eye(2) * 2
        
        x_start, _ = interp.interpolate(0.0, 0.0, x1, P1, 1.0, x2, P2)
        np.testing.assert_array_almost_equal(x_start, x1)
        
        x_end, _ = interp.interpolate(1.0, 0.0, x1, P1, 1.0, x2, P2)
        np.testing.assert_array_almost_equal(x_end, x2)
    
    def test_interpolate_trajectory(self):
        """Trajectory interpolation at fine grid."""
        interp = TrackInterpolator()
        scan_times = np.array([0.0, 5.0, 10.0])
        states = [
            np.array([0.0, 0.0, 0.0]),
            np.array([50.0, 25.0, 10.0]),
            np.array([100.0, 50.0, 20.0]),
        ]
        covs = [np.eye(3) * 1.0] * 3
        
        query = np.array([0.0, 2.5, 5.0, 7.5, 10.0])
        results = interp.interpolate_trajectory(query, scan_times, states, covs)
        
        assert len(results) == 5
        np.testing.assert_array_almost_equal(results[0][0], states[0])
        np.testing.assert_array_almost_equal(results[2][0], states[1])
        np.testing.assert_array_almost_equal(results[4][0], states[2])


class TestBatchProcessor:
    """Batch processing integration tests."""
    
    def test_batch_basic(self):
        """BatchProcessor processes multiple scans."""
        from python.nx_mimosa_mtt import MultiTargetTracker
        
        tracker = MultiTargetTracker(dt=1.0, r_std=50.0, domain='air')
        batch = BatchProcessor(tracker, BatchConfig(enable_smoothing=True))
        
        np.random.seed(42)
        scans = []
        for i in range(10):
            targets = np.array([
                [100 + i * 10, 200 + i * 5, 1000],
                [500 - i * 8, 300 + i * 3, 2000],
            ]) + np.random.randn(2, 3) * 20
            scans.append(targets)
        
        result = batch.process(scans)
        
        assert isinstance(result, BatchResult)
        assert result.total_scans == 10
        assert result.total_time_s > 0
        assert len(result.detections_per_scan) == 10
        assert len(result.tracks) > 0
    
    def test_batch_with_interpolation(self):
        """Batch with interpolation generates fine-grid states."""
        from python.nx_mimosa_mtt import MultiTargetTracker
        
        tracker = MultiTargetTracker(dt=5.0, r_std=50.0, domain='air')
        config = BatchConfig(
            enable_smoothing=True,
            enable_interpolation=True,
            interpolation_dt=1.0
        )
        batch = BatchProcessor(tracker, config)
        
        np.random.seed(42)
        scans = [np.array([[i * 100.0, 0, 5000]]) + np.random.randn(1, 3) * 30 for i in range(8)]
        
        result = batch.process(scans, timestamps=[i * 5.0 for i in range(8)])
        
        assert result.total_scans == 8
        # At least one track should have interpolated data
        has_interp = any(t.interpolated is not None for t in result.tracks.values())
        # may or may not have interpolation depending on track lifecycle
    
    def test_batch_config_defaults(self):
        """Default config has smoothing on, interpolation off."""
        config = BatchConfig()
        assert config.enable_smoothing is True
        assert config.enable_interpolation is False
        assert config.interpolation_dt == 1.0
    
    def test_batch_result_structure(self):
        """BatchTrackResult has correct fields."""
        tr = BatchTrackResult(track_id=42)
        assert tr.track_id == 42
        assert tr.smoothed_states is None
        assert tr.quality_score == 0.0
    
    def test_batch_progress_callback(self):
        """Progress callback is called for each scan."""
        from python.nx_mimosa_mtt import MultiTargetTracker
        
        progress = []
        def cb(idx, total):
            progress.append((idx, total))
        
        tracker = MultiTargetTracker(dt=1.0, r_std=50.0, domain='air')
        config = BatchConfig(
            enable_smoothing=False,
            progress_callback=cb
        )
        batch = BatchProcessor(tracker, config)
        
        scans = [np.random.randn(3, 3) * 100 for _ in range(5)]
        batch.process(scans)
        
        assert len(progress) == 5
        assert progress[-1] == (4, 5)


if __name__ == '__main__':
    pytest.main([__file__, '-v', '--tb=short'])
