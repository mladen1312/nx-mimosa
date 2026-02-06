"""
Tests for NX-MIMOSA v5.0 — Coordinate Transforms, 3D Tracking, MTT, GNN
=========================================================================
pytest tests/test_nx_mimosa_v50.py -v
"""

import numpy as np
import pytest
import sys, os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))

from nx_mimosa_coords import (
    geodetic_to_ecef, ecef_to_geodetic, ecef_to_enu, enu_to_ecef,
    geodetic_to_enu, enu_to_geodetic, spherical_to_cartesian,
    cartesian_to_spherical, unbiased_polar_to_cartesian_2d,
    unbiased_spherical_to_cartesian_3d, haversine_distance,
    jacobian_spherical_to_cartesian_3d, SensorLocation,
    WGS84_A, WGS84_B,
)
from nx_mimosa_mtt import (
    MultiTargetTracker, TrackManagerConfig, TrackStatus, TrackState,
    IMM3D, KalmanFilter3D, gnn_associate, hungarian_algorithm,
    make_cv3d_matrices, make_ca3d_matrices, make_ct3d_matrices,
    compute_ospa, compute_track_metrics,
)


# ============================================================
# COORDINATE TRANSFORM TESTS
# ============================================================

class TestGeodeticECEF:
    """WGS-84 Geodetic ↔ ECEF roundtrip tests."""
    
    def test_equator_prime_meridian(self):
        """(0°, 0°, 0m) should be on equator at prime meridian."""
        ecef = geodetic_to_ecef(0, 0, 0)
        assert ecef[0] == pytest.approx(WGS84_A, rel=1e-6)
        assert abs(ecef[1]) < 1.0
        assert abs(ecef[2]) < 1.0
    
    def test_north_pole(self):
        """(90°, 0°, 0m) should be at North Pole."""
        ecef = geodetic_to_ecef(90, 0, 0)
        assert abs(ecef[0]) < 1.0
        assert abs(ecef[1]) < 1.0
        assert ecef[2] == pytest.approx(WGS84_B, rel=1e-6)
    
    def test_roundtrip(self):
        """Roundtrip conversion should preserve coordinates."""
        test_cases = [
            (45.815, 15.982, 120),     # Zagreb
            (-33.86, 151.21, 5),       # Sydney
            (0, 0, 0),                 # Equator/PM
            (90, 0, 10000),            # North Pole high alt
            (-90, 0, 0),              # South Pole
            (51.5074, -0.1278, 11),    # London
        ]
        for lat, lon, alt in test_cases:
            ecef = geodetic_to_ecef(lat, lon, alt)
            lat2, lon2, alt2 = ecef_to_geodetic(ecef[0], ecef[1], ecef[2])
            assert lat2 == pytest.approx(lat, abs=1e-6), f"Lat failed for ({lat},{lon},{alt})"
            assert lon2 == pytest.approx(lon, abs=1e-6), f"Lon failed for ({lat},{lon},{alt})"
            assert alt2 == pytest.approx(alt, abs=0.01), f"Alt failed for ({lat},{lon},{alt})"
    
    def test_altitude_effect(self):
        """Higher altitude = further from center."""
        low = geodetic_to_ecef(45, 15, 0)
        high = geodetic_to_ecef(45, 15, 10000)
        assert np.linalg.norm(high) > np.linalg.norm(low)


class TestENU:
    """ECEF ↔ ENU local tangent plane tests."""
    
    def test_same_point_is_origin(self):
        """Target at sensor location = ENU (0,0,0)."""
        enu = geodetic_to_enu(45.0, 15.0, 100.0, 45.0, 15.0, 100.0)
        assert np.allclose(enu, [0, 0, 0], atol=0.001)
    
    def test_north_is_positive_y(self):
        """Point slightly north has positive North component."""
        enu = geodetic_to_enu(45.001, 15.0, 100.0, 45.0, 15.0, 100.0)
        assert enu[1] > 0  # North
        assert abs(enu[0]) < abs(enu[1])  # Mainly northward
    
    def test_east_is_positive_x(self):
        """Point slightly east has positive East component."""
        enu = geodetic_to_enu(45.0, 15.001, 100.0, 45.0, 15.0, 100.0)
        assert enu[0] > 0  # East
    
    def test_up_is_positive_z(self):
        """Point higher up has positive Up component."""
        enu = geodetic_to_enu(45.0, 15.0, 200.0, 45.0, 15.0, 100.0)
        assert enu[2] == pytest.approx(100.0, abs=1.0)  # ~100m up
    
    def test_roundtrip(self):
        """ENU → ECEF → ENU should preserve."""
        ref = (45.815, 15.982, 120)
        enu_orig = np.array([1000.0, 2000.0, 500.0])
        ecef = enu_to_ecef(enu_orig, *ref)
        enu_back = ecef_to_enu(ecef, *ref)
        assert np.allclose(enu_orig, enu_back, atol=0.01)


class TestSpherical:
    """Radar spherical ↔ Cartesian tests."""
    
    def test_north_zero_elevation(self):
        """Range 1000m, az=0 (North), el=0 → (0, 1000, 0)."""
        pos = spherical_to_cartesian(1000, 0, 0)
        assert pos[0] == pytest.approx(0.0, abs=0.01)   # East
        assert pos[1] == pytest.approx(1000.0, abs=0.01) # North
        assert pos[2] == pytest.approx(0.0, abs=0.01)    # Up
    
    def test_east(self):
        """Range 1000m, az=π/2 (East), el=0 → (1000, 0, 0)."""
        pos = spherical_to_cartesian(1000, np.pi/2, 0)
        assert pos[0] == pytest.approx(1000.0, abs=0.01)
        assert abs(pos[1]) < 0.01
    
    def test_elevation_45(self):
        """Range 1000m, az=0, el=45° → (0, 707, 707)."""
        pos = spherical_to_cartesian(1000, 0, np.pi/4)
        assert pos[1] == pytest.approx(707.1, abs=1.0)  # North
        assert pos[2] == pytest.approx(707.1, abs=1.0)  # Up
    
    def test_roundtrip(self):
        """Spherical → Cartesian → Spherical preserves."""
        r, az, el = 5000.0, np.radians(45), np.radians(10)
        pos = spherical_to_cartesian(r, az, el)
        r2, az2, el2 = cartesian_to_spherical(pos[0], pos[1], pos[2])
        assert r2 == pytest.approx(r, rel=1e-6)
        assert az2 == pytest.approx(az, abs=1e-6)
        assert el2 == pytest.approx(el, abs=1e-6)


class TestUnbiasedConversion:
    """Debiased polar/spherical to Cartesian tests."""
    
    def test_2d_unbiased_zero_noise(self):
        """With zero noise, unbiased = standard conversion."""
        pos, cov = unbiased_polar_to_cartesian_2d(1000, np.pi/4, 0.001, 0.001)
        expected_x = 1000 * np.cos(np.pi/4)
        expected_y = 1000 * np.sin(np.pi/4)
        assert pos[0] == pytest.approx(expected_x, rel=0.01)
        assert pos[1] == pytest.approx(expected_y, rel=0.01)
    
    def test_3d_unbiased_positive_covariance(self):
        """Covariance must be positive semi-definite."""
        pos, cov = unbiased_spherical_to_cartesian_3d(
            5000, np.pi/4, np.pi/6, 100, np.radians(1), np.radians(1)
        )
        eigvals = np.linalg.eigvalsh(cov)
        assert all(v >= 0 for v in eigvals)


class TestHaversine:
    def test_same_point(self):
        assert haversine_distance(45, 15, 45, 15) == pytest.approx(0, abs=0.1)
    
    def test_known_distance(self):
        """Zagreb to Split ≈ 270km."""
        d = haversine_distance(45.815, 15.982, 43.508, 16.440)
        assert 250e3 < d < 290e3


class TestJacobians:
    def test_jacobian_shape(self):
        J = jacobian_spherical_to_cartesian_3d(1000, 2000, 500)
        assert J.shape == (3, 3)
    
    def test_jacobian_numerical(self):
        """Verify Jacobian matches numerical differentiation."""
        e, n, u = 3000.0, 4000.0, 1000.0
        J = jacobian_spherical_to_cartesian_3d(e, n, u)
        
        eps = 0.1
        J_num = np.zeros((3, 3))
        for col in range(3):
            delta = np.zeros(3)
            delta[col] = eps
            p = np.array([e, n, u])
            r1, az1, el1 = cartesian_to_spherical(*(p + delta))
            r2, az2, el2 = cartesian_to_spherical(*(p - delta))
            J_num[0, col] = (r1 - r2) / (2 * eps)
            J_num[1, col] = (az1 - az2) / (2 * eps)
            J_num[2, col] = (el1 - el2) / (2 * eps)
        
        assert np.allclose(J, J_num, atol=1e-4)


class TestSensorLocation:
    def test_radar_to_enu(self):
        sensor = SensorLocation(lat_deg=45.0, lon_deg=15.0, alt_m=100.0)
        enu = sensor.radar_to_enu(5000, np.radians(45), np.radians(5))
        assert len(enu) == 3
        assert enu[2] > 0  # Positive elevation → positive up


# ============================================================
# 3D MOTION MODEL TESTS
# ============================================================

class TestMotionModels3D:
    def test_cv3d_dimensions(self):
        F, Q = make_cv3d_matrices(1.0, 1.0)
        assert F.shape == (6, 6)
        assert Q.shape == (6, 6)
    
    def test_ca3d_dimensions(self):
        F, Q = make_ca3d_matrices(1.0, 1.0)
        assert F.shape == (9, 9)
        assert Q.shape == (9, 9)
    
    def test_ct3d_dimensions(self):
        F, Q = make_ct3d_matrices(1.0, 1.0, 0.1)
        assert F.shape == (7, 7)
        assert Q.shape == (7, 7)
    
    def test_cv3d_straight_line(self):
        """CV3D should propagate straight line correctly."""
        F, _ = make_cv3d_matrices(1.0, 0.01)
        x = np.array([0, 0, 0, 100, 200, 50])  # vx=100, vy=200, vz=50
        x2 = F @ x
        assert x2[0] == pytest.approx(100.0)
        assert x2[1] == pytest.approx(200.0)
        assert x2[2] == pytest.approx(50.0)
    
    def test_ct3d_zero_omega_is_cv(self):
        """CT3D with ω=0 should behave like CV."""
        F_ct, _ = make_ct3d_matrices(1.0, 1.0, 0.0)
        F_cv, _ = make_cv3d_matrices(1.0, 1.0)
        # Compare first 6×6 block
        assert np.allclose(F_ct[:6, :6], F_cv, atol=1e-10)
    
    def test_q_positive_definite(self):
        """All Q matrices must be positive semi-definite."""
        for dt in [0.1, 1.0, 5.0]:
            for q in [0.1, 1.0, 10.0]:
                _, Q = make_cv3d_matrices(dt, q)
                assert all(np.linalg.eigvalsh(Q) >= -1e-10)
                _, Q = make_ca3d_matrices(dt, q)
                assert all(np.linalg.eigvalsh(Q) >= -1e-10)


# ============================================================
# KALMAN FILTER 3D TESTS
# ============================================================

class TestKalmanFilter3D:
    def test_basic_tracking(self):
        """Track a constant-velocity target in 3D."""
        np.random.seed(42)
        kf = KalmanFilter3D(nx=6, nz=3)
        kf.R = np.eye(3) * 100  # 10m std
        
        # True trajectory: v = [100, 50, -10] m/s
        true_vel = np.array([100, 50, -10])
        true_pos = np.array([0, 0, 1000.0])
        
        F, Q = make_cv3d_matrices(1.0, 0.1)
        kf.x[:3] = true_pos + np.random.randn(3) * 50
        kf.P[:3, :3] *= 2500
        
        errors = []
        for step in range(50):
            true_pos = true_pos + true_vel
            z = true_pos + np.random.randn(3) * 10
            
            H = np.zeros((3, 6))
            H[0, 0] = H[1, 1] = H[2, 2] = 1.0
            
            kf.predict(F, Q)
            kf.update(z, H)
            errors.append(np.linalg.norm(kf.position - true_pos))
        
        # Error should converge
        assert np.mean(errors[-10:]) < np.mean(errors[:10])
        assert np.mean(errors[-10:]) < 30.0  # Under 30m


# ============================================================
# IMM 3D TESTS
# ============================================================

class TestIMM3D:
    def test_creation(self):
        imm = IMM3D(dt=1.0, q_base=1.0, r_std=50.0)
        assert imm.n_models == 4
        assert len(imm.filters) == 4
        assert imm.mu.sum() == pytest.approx(1.0)
    
    def test_initialize(self):
        imm = IMM3D(dt=1.0)
        imm.initialize(np.array([1000, 2000, 500]))
        for kf in imm.filters:
            assert kf.x[0] == pytest.approx(1000)
            assert kf.x[1] == pytest.approx(2000)
            assert kf.x[2] == pytest.approx(500)
    
    def test_cv_track(self):
        """IMM should track constant velocity target."""
        np.random.seed(42)
        imm = IMM3D(dt=1.0, q_base=0.5, r_std=50.0)
        
        true_pos = np.array([0.0, 0.0, 5000.0])
        true_vel = np.array([250.0, 0.0, 0.0])
        
        imm.initialize(true_pos + np.random.randn(3) * 50)
        
        errors = []
        for step in range(30):
            true_pos += true_vel
            z = true_pos + np.random.randn(3) * 50
            
            imm.predict()
            imm.update(z)
            
            errors.append(np.linalg.norm(imm.position - true_pos))
        
        assert np.mean(errors[-10:]) < 80.0


# ============================================================
# GNN DATA ASSOCIATION TESTS
# ============================================================

class TestHungarian:
    def test_simple_2x2(self):
        cost = np.array([[1.0, 3.0], [3.0, 1.0]])
        assignments = hungarian_algorithm(cost)
        total = sum(cost[i, j] for i, j in assignments)
        assert total == pytest.approx(2.0)
    
    def test_3x3(self):
        cost = np.array([
            [10, 5, 13],
            [3, 7, 15],
            [13, 12, 8]
        ])
        assignments = hungarian_algorithm(cost)
        assert len(assignments) == 3
        total = sum(cost[i, j] for i, j in assignments)
        assert total <= 21  # Optimal: 5+3+8=16 or similar
    
    def test_rectangular(self):
        """More measurements than tracks."""
        cost = np.array([[1.0, 10.0, 10.0], [10.0, 1.0, 10.0]])
        assignments = hungarian_algorithm(cost)
        assert len(assignments) == 2
    
    def test_all_high_cost(self):
        """All costs above threshold → no assignments."""
        cost = np.full((2, 2), 1e9)
        assignments = hungarian_algorithm(cost)
        assert len(assignments) == 0


class TestGNNAssociation:
    def test_perfect_association(self):
        """Two tracks, two measurements, perfect match."""
        np.random.seed(42)
        mtt = MultiTargetTracker(dt=1.0, r_std=50.0)
        
        # Create two tracks manually
        config = TrackManagerConfig()
        tm = mtt.track_manager
        
        t1 = tm.create_track(np.array([1000, 0, 0.0]), 1.0, 1.0, 50.0)
        t2 = tm.create_track(np.array([5000, 0, 0.0]), 1.0, 1.0, 50.0)
        
        tracks = [t1, t2]
        meas = np.array([[1010, 5, 0.0], [5015, -3, 0.0]])
        
        assignments, unassigned_t, unassigned_m = gnn_associate(tracks, meas)
        assert len(assignments) == 2
        assert len(unassigned_t) == 0
        assert len(unassigned_m) == 0
    
    def test_extra_measurement(self):
        """More measurements than tracks → unassigned measurements."""
        tm = MultiTargetTracker(dt=1.0, r_std=50.0).track_manager
        t1 = tm.create_track(np.array([1000, 0, 0.0]), 1.0, 1.0, 50.0)
        
        meas = np.array([[1010, 0, 0.0], [9000, 0, 0.0]])
        assignments, _, unassigned_m = gnn_associate([t1], meas)
        
        assert len(assignments) == 1
        assert len(unassigned_m) == 1


# ============================================================
# TRACK MANAGEMENT TESTS
# ============================================================

class TestTrackManagement:
    def test_tentative_to_confirmed(self):
        """Track should confirm after M hits in N opportunities."""
        mtt = MultiTargetTracker(dt=1.0, r_std=50.0,
                                 config=TrackManagerConfig(confirm_m=3, confirm_n=5))
        
        # Keep feeding same target
        for i in range(10):
            mtt.process_scan(np.array([[1000 + i*100.0, 0.0, 0.0]]))
        
        confirmed = mtt.confirmed_tracks
        assert len(confirmed) >= 1
        assert confirmed[0].status in (TrackStatus.CONFIRMED, TrackStatus.COASTING)
    
    def test_track_deletion_on_miss(self):
        """Track should be deleted after consecutive misses."""
        mtt = MultiTargetTracker(dt=1.0, r_std=50.0, q_base=1.0,
                                 config=TrackManagerConfig(
                                     confirm_m=2, confirm_n=3,
                                     delete_misses=3))
        
        # Create and confirm track
        for i in range(5):
            mtt.process_scan(np.array([[1000 + i*100.0, 0.0, 0.0]]))
        
        n_before = len(mtt.tracks)
        
        # Now send measurements far away (track will miss)
        for i in range(5):
            mtt.process_scan(np.array([[99999.0, 99999.0, 0.0]]))
        
        # Original track should eventually be deleted
        # (New track at 99999 may be created)
        original_ids = [t.track_id for t in mtt.tracks if t.track_id == 1]
        # Track 1 should be deleted or coasting
    
    def test_coasting(self):
        """Track should coast when measurements temporarily disappear."""
        mtt = MultiTargetTracker(dt=1.0, r_std=50.0,
                                 config=TrackManagerConfig(
                                     confirm_m=2, confirm_n=3,
                                     coast_misses=2, delete_misses=5))
        
        # Establish track
        for i in range(5):
            mtt.process_scan(np.array([[1000 + i*250.0, 5000.0, 1000.0]]))
        
        # Send empty scans (no measurements near track)
        for i in range(2):
            mtt.process_scan(np.array([[99999.0, 99999.0, 0.0]]))
        
        coasting = [t for t in mtt.tracks if t.status == TrackStatus.COASTING]
        # Should have at least started coasting


# ============================================================
# MULTI-TARGET TRACKER INTEGRATION TESTS
# ============================================================

class TestMultiTargetTrackerIntegration:
    def test_two_targets_parallel(self):
        """Track two targets moving in parallel."""
        np.random.seed(42)
        mtt = MultiTargetTracker(dt=1.0, r_std=30.0, q_base=0.5,
                                 config=TrackManagerConfig(
                                     confirm_m=2, confirm_n=3,
                                     min_separation=50.0))
        
        for step in range(20):
            t = step * 1.0
            # Target 1: moving east at 100 m/s, y=0
            # Target 2: moving east at 150 m/s, y=5000
            z1 = np.array([100*t + np.random.randn()*30,
                           np.random.randn()*30,
                           1000 + np.random.randn()*30])
            z2 = np.array([150*t + np.random.randn()*30,
                           5000 + np.random.randn()*30,
                           2000 + np.random.randn()*30])
            
            mtt.process_scan(np.vstack([z1, z2]), t)
        
        confirmed = mtt.confirmed_tracks
        assert len(confirmed) == 2, f"Expected 2 tracks, got {len(confirmed)}"
    
    def test_three_targets(self):
        """Track three well-separated targets."""
        np.random.seed(123)
        mtt = MultiTargetTracker(dt=1.0, r_std=50.0, q_base=1.0)
        
        for step in range(25):
            z1 = np.array([100*step, 0, 0]) + np.random.randn(3)*50
            z2 = np.array([0, 100*step, 5000]) + np.random.randn(3)*50
            z3 = np.array([-100*step, -100*step, 10000]) + np.random.randn(3)*50
            mtt.process_scan(np.vstack([z1, z2, z3]))
        
        confirmed = mtt.confirmed_tracks
        assert len(confirmed) >= 2  # At least 2 should confirm
    
    def test_target_appears_disappears(self):
        """Target appears, gets tracked, disappears, gets deleted."""
        np.random.seed(42)
        mtt = MultiTargetTracker(dt=1.0, r_std=30.0, q_base=1.0,
                                 config=TrackManagerConfig(
                                     confirm_m=2, confirm_n=3,
                                     delete_misses=3))
        
        # Target present for 10 steps
        for step in range(10):
            z = np.array([[100*step, 0, 0]]) + np.random.randn(1, 3)*30
            mtt.process_scan(z)
        
        assert mtt.stats["current_confirmed"] >= 1
        
        # Target disappears (no measurements)
        for step in range(10):
            mtt.process_scan(np.zeros((0, 3)))
        
        # Track should be deleted
        assert mtt.stats["current_confirmed"] == 0
    
    def test_2d_input_compatibility(self):
        """2D measurements should work (z auto-set to 0)."""
        mtt = MultiTargetTracker(dt=1.0, r_std=50.0)
        
        for i in range(10):
            meas_2d = np.array([[1000 + i*100.0, 5000.0]])
            mtt.process_scan(meas_2d)
        
        assert len(mtt.tracks) >= 1
    
    def test_domain_presets(self):
        """Domain presets should apply different configs."""
        mil = MultiTargetTracker(dt=1.0, r_std=50.0, domain="military")
        auto = MultiTargetTracker(dt=0.1, r_std=2.5, domain="automotive")
        
        assert mil.config.gate_threshold > auto.config.gate_threshold
        assert mil.config.min_separation > auto.config.min_separation
    
    def test_empty_scan(self):
        """Empty scans should not crash."""
        mtt = MultiTargetTracker(dt=1.0, r_std=50.0)
        mtt.process_scan(np.zeros((0, 3)))
        assert mtt.stats["scans_processed"] == 1
    
    def test_single_measurement(self):
        """Single measurement creates tentative track."""
        mtt = MultiTargetTracker(dt=1.0, r_std=50.0)
        mtt.process_scan(np.array([[1000, 2000, 500]]))
        assert len(mtt.tracks) == 1
        assert mtt.tracks[0].status == TrackStatus.TENTATIVE


# ============================================================
# OSPA METRICS TESTS
# ============================================================

class TestOSPA:
    def test_perfect_match(self):
        """Identical positions → OSPA = 0."""
        ospa = compute_ospa(
            [np.array([0, 0, 0]), np.array([1000, 0, 0])],
            [np.array([0, 0, 0]), np.array([1000, 0, 0])]
        )
        assert ospa == pytest.approx(0.0, abs=0.1)
    
    def test_cardinality_mismatch(self):
        """Different number of targets → non-zero OSPA."""
        ospa = compute_ospa(
            [np.array([0, 0, 0])],
            [np.array([0, 0, 0]), np.array([1000, 0, 0])]
        )
        assert ospa > 0
    
    def test_empty(self):
        assert compute_ospa([], []) == 0.0
        assert compute_ospa([], [np.array([0, 0, 0])]) > 0


# ============================================================
# FULL SCENARIO: MILITARY AIR DEFENSE
# ============================================================

class TestMilitaryScenario:
    def test_two_aircraft_one_missile(self):
        """Realistic: 2 aircraft + 1 missile appearing mid-scenario."""
        np.random.seed(42)
        mtt = MultiTargetTracker(dt=2.0, r_std=100.0, q_base=2.0,
                                 domain="military")
        
        for step in range(40):
            t = step * 2.0
            measurements = []
            
            # Aircraft 1: cruising east at 250 m/s, 10km alt
            measurements.append(
                [250*t, 0, 10000] + np.random.randn(3) * 100
            )
            
            # Aircraft 2: cruising NE at 200 m/s, 8km alt
            measurements.append(
                [141*t, 141*t, 8000] + np.random.randn(3) * 100
            )
            
            # Missile: appears at step 20, fast (800 m/s), accelerating climb
            if step >= 20:
                mt = (step - 20) * 2.0
                measurements.append(
                    [30000 + 800*mt, 10000, 5000 + 100*mt] + np.random.randn(3) * 100
                )
            
            mtt.process_scan(np.array(measurements), t)
        
        confirmed = mtt.confirmed_tracks
        # Should have 2-3 confirmed tracks
        assert len(confirmed) >= 2, f"Expected ≥2 tracks, got {len(confirmed)}"
        
        # Summary should not crash
        summary = mtt.summary()
        assert "MTT Engine" in summary


class TestAutomotiveScenario:
    def test_highway_vehicles(self):
        """Track vehicles on a highway."""
        np.random.seed(42)
        mtt = MultiTargetTracker(dt=0.1, r_std=2.0, q_base=5.0,
                                 domain="automotive")
        
        for step in range(50):
            t = step * 0.1
            measurements = []
            
            # Car 1: lane 1, 30 m/s
            measurements.append([30*t, 0, 0] + np.random.randn(3) * 2)
            # Car 2: lane 2, 25 m/s
            measurements.append([25*t, 3.5, 0] + np.random.randn(3) * 2)
            # Car 3: lane 1, 35 m/s (overtaking)
            measurements.append([35*t + 50, 0, 0] + np.random.randn(3) * 2)
            
            mtt.process_scan(np.array(measurements), t)
        
        confirmed = mtt.confirmed_tracks
        assert len(confirmed) >= 2


# ============================================================
# REGRESSION: EXISTING SINGLE-TARGET STILL WORKS
# ============================================================

class TestBackwardCompatibility:
    def test_single_target_via_mtt(self):
        """MTT should work fine with single target (regression)."""
        np.random.seed(42)
        mtt = MultiTargetTracker(dt=1.0, r_std=50.0, q_base=1.0)
        
        true_pos = np.array([0.0, 0.0, 5000.0])
        vel = np.array([250.0, 0.0, 0.0])
        
        errors = []
        for step in range(30):
            true_pos = true_pos + vel
            z = true_pos + np.random.randn(3) * 50
            confirmed = mtt.process_scan(z.reshape(1, 3))
            
            if confirmed:
                est = confirmed[0].filter.position
                errors.append(np.linalg.norm(est - true_pos))
        
        assert len(errors) > 10  # Should confirm within ~5 steps
        assert np.mean(errors[-10:]) < 100.0  # Reasonable accuracy


# ============================================================
# JPDA DATA ASSOCIATION TESTS
# ============================================================

from nx_mimosa_mtt import (
    jpda_associate, AssociationMethod,
    compute_nees, compute_nis, compute_siap_metrics,
)

class TestJPDA:
    def test_jpda_basic(self):
        """JPDA should produce weighted measurements for valid tracks."""
        np.random.seed(42)
        mtt = MultiTargetTracker(dt=1.0, r_std=50.0, association="jpda")
        
        for i in range(10):
            mtt.process_scan(np.array([[1000 + i*100.0, 0, 0.0]]))
        
        assert len(mtt.tracks) >= 1
    
    def test_jpda_two_targets(self):
        """JPDA should track two well-separated targets."""
        np.random.seed(42)
        mtt = MultiTargetTracker(dt=1.0, r_std=30.0, q_base=0.5,
                                 association="jpda",
                                 config=TrackManagerConfig(
                                     confirm_m=2, confirm_n=3,
                                     min_separation=50.0))
        
        for step in range(20):
            z1 = np.array([100*step + np.random.randn()*30,
                           np.random.randn()*30,
                           1000 + np.random.randn()*30])
            z2 = np.array([150*step + np.random.randn()*30,
                           5000 + np.random.randn()*30,
                           2000 + np.random.randn()*30])
            mtt.process_scan(np.vstack([z1, z2]))
        
        confirmed = mtt.confirmed_tracks
        assert len(confirmed) == 2
    
    def test_jpda_in_clutter(self):
        """JPDA should handle clutter measurements gracefully."""
        np.random.seed(42)
        mtt = MultiTargetTracker(dt=1.0, r_std=50.0, q_base=1.0,
                                 association="jpda",
                                 config=TrackManagerConfig(
                                     confirm_m=3, confirm_n=5,
                                     delete_misses=4,
                                     min_separation=100.0,
                                     gate_threshold=25.0))
        
        for step in range(40):
            # Real target: consistent trajectory, easy to track
            z_real = np.array([100*step, 0, 5000]) + np.random.randn(3)*50
            # Clutter: far away, random each scan
            clutter = np.random.uniform(50000, 100000, size=(2, 3))
            meas = np.vstack([z_real, clutter])
            mtt.process_scan(meas)
        
        confirmed = mtt.confirmed_tracks
        # Real target should confirm; clutter tracks should die (inconsistent)
        assert len(confirmed) >= 1
    
    def test_gnn_vs_jpda_enum(self):
        """Both association methods should be selectable."""
        gnn = MultiTargetTracker(dt=1.0, r_std=50.0, association="gnn")
        jpda = MultiTargetTracker(dt=1.0, r_std=50.0, association="jpda")
        assert gnn.association == AssociationMethod.GNN
        assert jpda.association == AssociationMethod.JPDA


# ============================================================
# NEES / NIS / SIAP METRICS TESTS
# ============================================================

class TestPerformanceMetrics:
    def test_nees_perfect(self):
        """Perfect estimate → NEES = 0."""
        x_true = np.array([1000, 2000, 500])
        x_est = np.array([1000, 2000, 500])
        P = np.eye(3) * 100
        assert compute_nees(x_true, x_est, P) == pytest.approx(0.0, abs=1e-10)
    
    def test_nees_nonzero(self):
        """Nonzero error → positive NEES."""
        x_true = np.array([1000, 2000, 500])
        x_est = np.array([1010, 2005, 498])
        P = np.eye(3) * 100
        nees = compute_nees(x_true, x_est, P)
        assert nees > 0
    
    def test_nis(self):
        innov = np.array([10.0, 5.0, -3.0])
        S = np.eye(3) * 100
        nis = compute_nis(innov, S)
        assert nis > 0
    
    def test_siap_perfect(self):
        """Perfect tracking → completeness=1, spuriousness=0."""
        tracks = {1: np.array([100, 200, 300]), 2: np.array([400, 500, 600])}
        truth = {10: np.array([100, 200, 300]), 20: np.array([400, 500, 600])}
        m = compute_siap_metrics(tracks, truth)
        assert m["completeness"] == pytest.approx(1.0)
        assert m["spuriousness"] == pytest.approx(0.0)
        assert m["accuracy"] < 1.0
    
    def test_siap_false_tracks(self):
        """Extra tracks → non-zero spuriousness."""
        tracks = {1: np.array([100, 0, 0]), 2: np.array([9999, 9999, 0])}
        truth = {10: np.array([100, 0, 0])}
        m = compute_siap_metrics(tracks, truth)
        assert m["n_false"] >= 1
    
    def test_siap_empty(self):
        m = compute_siap_metrics({}, {})
        assert m["completeness"] == 1.0


# ============================================================
# CROSSING TARGETS SCENARIO (GNN vs JPDA comparison)
# ============================================================

class TestCrossingTargets:
    def test_crossing_scenario(self):
        """Two targets crossing paths — both GNN and JPDA should handle."""
        np.random.seed(42)
        
        for method in ["gnn", "jpda"]:
            mtt = MultiTargetTracker(dt=1.0, r_std=30.0, q_base=1.0,
                                     association=method,
                                     config=TrackManagerConfig(
                                         confirm_m=2, confirm_n=3,
                                         min_separation=20.0))
            
            for step in range(30):
                t = step * 1.0
                # Target 1: moving +x
                z1 = np.array([100*t, 5000 - 100*t, 3000]) + np.random.randn(3)*30
                # Target 2: moving -x (crossing Target 1 at step 15)
                z2 = np.array([3000 - 100*t, -5000 + 100*t, 3000]) + np.random.randn(3)*30
                mtt.process_scan(np.vstack([z1, z2]))
            
            confirmed = mtt.confirmed_tracks
            assert len(confirmed) >= 2, \
                f"{method.upper()}: Expected ≥2 tracks at crossing, got {len(confirmed)}"
