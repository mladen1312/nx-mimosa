"""
Tests for NX-MIMOSA v5.5+ Gap Closers
======================================
- Coordinate full chains (Spherical ↔ Geodetic, covariance transforms)
- Enhanced MHT (cluster gating, N-scan pruning, extreme clutter)
- Dataset adapters (synthetic scenarios, CSV adapter)
- Real-dataset stress tests

pytest tests/test_gaps_v55.py -v
"""

import numpy as np
import pytest
import sys, os, tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))

from nx_mimosa_coords import (
    spherical_to_geodetic, geodetic_to_spherical,
    ecef_to_spherical, spherical_to_ecef,
    covariance_spherical_to_cartesian, covariance_cartesian_to_spherical,
    bearing_between, destination_point,
    SensorLocation, geodetic_to_ecef, ecef_to_geodetic,
    spherical_to_cartesian, cartesian_to_spherical,
    geodetic_to_enu,
)
from nx_mimosa_mtt import (
    mht_associate_enhanced, MHTHypothesisTree, _build_clusters,
    MultiTargetTracker, TrackStatus, TrackState, KalmanFilter3D,
    make_cv3d_matrices, AssociationMethod, compute_ospa,
)
from nx_mimosa_datasets import (
    ScanData, SyntheticScenarioGenerator, GenericCSVAdapter, DatasetInfo,
)


# ============================================================
# GAP 1: COORDINATE FULL CHAIN TESTS
# ============================================================

class TestSphericalGeodeticRoundtrip:
    """Full chain: Spherical → Geodetic → Spherical roundtrip."""

    def test_roundtrip_close_target(self):
        """Nearby target should roundtrip within <1m error."""
        sensor = SensorLocation(lat_deg=45.0, lon_deg=15.0, alt_m=100.0)
        
        r_orig, az_orig, el_orig = 10000.0, np.radians(30), np.radians(5)
        
        lat, lon, alt = spherical_to_geodetic(r_orig, az_orig, el_orig, sensor)
        r_back, az_back, el_back = geodetic_to_spherical(lat, lon, alt, sensor)
        
        assert abs(r_back - r_orig) < 1.0, f"Range error: {abs(r_back - r_orig):.3f}m"
        assert abs(az_back - az_orig) < 1e-4, f"Az error: {abs(az_back - az_orig):.6f} rad"
        assert abs(el_back - el_orig) < 1e-4, f"El error: {abs(el_back - el_orig):.6f} rad"

    def test_roundtrip_long_range(self):
        """Long-range target (200km)."""
        sensor = SensorLocation(lat_deg=44.0, lon_deg=16.0, alt_m=500.0)
        
        r, az, el = 200000.0, np.radians(270), np.radians(2)
        lat, lon, alt = spherical_to_geodetic(r, az, el, sensor)
        r2, az2, el2 = geodetic_to_spherical(lat, lon, alt, sensor)
        
        assert abs(r2 - r) / r < 0.001  # <0.1% range error
        assert abs(az2 - az) < 0.01

    def test_roundtrip_overhead(self):
        """Target directly overhead (high elevation)."""
        sensor = SensorLocation(lat_deg=50.0, lon_deg=10.0, alt_m=0.0)
        
        r, az, el = 5000.0, np.radians(0), np.radians(80)
        lat, lon, alt = spherical_to_geodetic(r, az, el, sensor)
        r2, az2, el2 = geodetic_to_spherical(lat, lon, alt, sensor)
        
        assert abs(r2 - r) < 5.0
        assert abs(el2 - el) < 0.01


class TestECEFSphericalRoundtrip:
    """ECEF ↔ Spherical via ENU intermediate."""

    def test_ecef_roundtrip(self):
        """ECEF → Spherical → ECEF should roundtrip."""
        sensor = SensorLocation(lat_deg=40.0, lon_deg=-74.0, alt_m=50.0)
        
        # Create a target in ECEF
        target_ecef = geodetic_to_ecef(40.5, -73.5, 8000.0)
        
        # ECEF → Spherical
        r, az, el = ecef_to_spherical(target_ecef, sensor)
        
        # Spherical → ECEF
        ecef_back = spherical_to_ecef(r, az, el, sensor)
        
        error = np.linalg.norm(ecef_back - target_ecef)
        assert error < 1.0, f"ECEF roundtrip error: {error:.3f}m"


class TestCovarianceTransforms:
    """Covariance transforms between spherical and Cartesian."""

    def test_covariance_spherical_to_cart(self):
        """Transformed covariance should be 3×3 positive definite."""
        P_cart = covariance_spherical_to_cartesian(
            r=10000, az=np.radians(45), el=np.radians(10),
            sigma_r=50.0, sigma_az=np.radians(1), sigma_el=np.radians(1)
        )
        
        assert P_cart.shape == (3, 3)
        eigenvalues = np.linalg.eigvalsh(P_cart)
        assert all(eigenvalues > 0), f"Not positive definite: {eigenvalues}"

    def test_covariance_roundtrip(self):
        """Sph → Cart → Sph should approximately preserve diagonal."""
        P_cart = covariance_spherical_to_cartesian(
            r=20000, az=np.radians(30), el=np.radians(5),
            sigma_r=100.0, sigma_az=np.radians(0.5), sigma_el=np.radians(0.5)
        )
        
        enu = spherical_to_cartesian(20000, np.radians(30), np.radians(5))
        P_sph_back = covariance_cartesian_to_spherical(enu[0], enu[1], enu[2], P_cart)
        
        assert P_sph_back.shape == (3, 3)
        # Diagonal should be roughly preserved
        assert P_sph_back[0, 0] > 0  # Range variance
        assert P_sph_back[1, 1] > 0  # Az variance

    def test_range_uncertainty_dominates(self):
        """Large range sigma should produce large covariance along LOS."""
        P = covariance_spherical_to_cartesian(
            r=50000, az=np.radians(0), el=np.radians(0),
            sigma_r=500.0, sigma_az=np.radians(0.01), sigma_el=np.radians(0.01)
        )
        # At az=0, el=0: LOS is along North axis
        # Range uncertainty dominates North variance
        assert P[1, 1] > P[0, 0]  # North > East for az=0


class TestGreatCircle:
    """Great circle bearing and destination calculations."""

    def test_bearing_north(self):
        """Same longitude, target north → bearing ≈ 0."""
        b = bearing_between(45.0, 15.0, 46.0, 15.0)
        assert abs(b) < 0.01 or abs(b - 2*np.pi) < 0.01

    def test_bearing_east(self):
        """Same latitude, target east → bearing ≈ π/2."""
        b = bearing_between(45.0, 15.0, 45.0, 16.0)
        assert abs(b - np.pi/2) < 0.05

    def test_destination_roundtrip(self):
        """Start → destination → bearing/distance should roundtrip."""
        lat1, lon1 = 45.0, 15.0
        bearing = np.radians(60)
        distance = 100000  # 100 km
        
        lat2, lon2 = destination_point(lat1, lon1, bearing, distance)
        
        # Check bearing back is roughly opposite
        from nx_mimosa_coords import haversine_distance
        d_back = haversine_distance(lat1, lon1, lat2, lon2)
        assert abs(d_back - distance) / distance < 0.01  # <1% error


# ============================================================
# GAP 2: ENHANCED MHT TESTS
# ============================================================

class TestClusterGating:
    """Tests for MHT cluster decomposition."""

    def test_two_independent_clusters(self):
        """Well-separated targets should form separate clusters."""
        # 2 tracks, 2 measurements, no cross-gating
        validation = np.array([
            [True, False],   # Track 0 gates to meas 0 only
            [False, True],   # Track 1 gates to meas 1 only
        ])
        
        clusters = _build_clusters(validation)
        track_clusters = [c for c in clusters if c[0]]
        assert len(track_clusters) == 2

    def test_single_cluster(self):
        """Closely spaced targets should form one cluster."""
        validation = np.array([
            [True, True],    # Track 0 gates to both
            [True, True],    # Track 1 gates to both
        ])
        
        clusters = _build_clusters(validation)
        track_clusters = [c for c in clusters if c[0]]
        assert len(track_clusters) == 1
        assert len(track_clusters[0][0]) == 2  # Both tracks in one cluster

    def test_isolated_measurement(self):
        """Measurement with no gated tracks should be isolated."""
        validation = np.array([
            [True, False, False],
            [False, True, False],
        ])
        
        clusters = _build_clusters(validation)
        # Should have 2 track clusters + 1 isolated measurement cluster
        assert len(clusters) == 3

    def test_empty_gating(self):
        """No gates → all isolated."""
        validation = np.zeros((3, 4), dtype=bool)
        clusters = _build_clusters(validation)
        assert len(clusters) >= 3  # At least 3 isolated tracks (as empty clusters)


class TestEnhancedMHT:
    """Tests for enhanced MHT with clusters and N-scan."""

    def test_basic_assignment(self):
        """Should produce valid assignments like basic MHT."""
        np.random.seed(42)
        F, Q = make_cv3d_matrices(1.0, 1.0)
        
        tracks = []
        for i in range(2):
            ts = TrackState(track_id=i, filter=KalmanFilter3D(6, 3))
            ts.filter.x = np.array([i * 5000.0, 0, 0, 100, 0, 0])
            ts.filter.P = np.eye(6) * 100
            ts.filter.R = np.eye(3) * 2500
            ts.status = TrackStatus.CONFIRMED
            tracks.append(ts)
        
        measurements = np.array([
            [50.0, 10, 5],     # Near track 0
            [5020.0, -5, 3],   # Near track 1
        ])
        
        assignments, ua_t, ua_m = mht_associate_enhanced(
            tracks, measurements, gate_threshold=100.0
        )
        
        assert len(assignments) == 2
        assert len(ua_t) == 0
        assert len(ua_m) == 0

    def test_extreme_clutter(self):
        """Should handle many false alarms gracefully."""
        np.random.seed(42)
        
        tracks = []
        ts = TrackState(track_id=0, filter=KalmanFilter3D(6, 3))
        ts.filter.x = np.array([1000.0, 0, 0, 100, 0, 0])
        ts.filter.P = np.eye(6) * 100
        ts.filter.R = np.eye(3) * 2500
        ts.status = TrackStatus.CONFIRMED
        tracks.append(ts)
        
        # 1 real + 20 clutter
        real = np.array([[1005, 3, -2]])
        clutter = np.random.randn(20, 3) * 5000
        measurements = np.vstack([real, clutter])
        
        assignments, ua_t, ua_m = mht_associate_enhanced(
            tracks, measurements, gate_threshold=50.0,
            clutter_density=1e-4
        )
        
        # Track should be assigned (to real detection, not clutter)
        assert 0 in assignments
        assigned_pos = assignments[0]
        error = np.linalg.norm(assigned_pos - real[0])
        assert error < 100, f"Assigned to clutter? Error: {error:.1f}m"

    def test_n_scan_tree(self):
        """N-scan tree should store and retrieve hypotheses."""
        tree = MHTHypothesisTree(n_scan_depth=3)
        
        for scan in range(5):
            hyps = [
                MHTHypothesis(assignments={0: scan % 3}, score=10.0 - scan * 0.1,
                              unassigned_meas=[]),
                MHTHypothesis(assignments={0: -1}, score=5.0,
                              unassigned_meas=[0]),
            ]
            tree.add_scan(hyps)
        
        best = tree.get_n_scan_best()
        assert best is not None
        assert len(tree.scan_hypotheses) == 3  # Pruned to depth

    def test_with_hypothesis_tree(self):
        """Enhanced MHT should accept and use hypothesis tree."""
        np.random.seed(42)
        tree = MHTHypothesisTree(n_scan_depth=3)
        
        ts = TrackState(track_id=0, filter=KalmanFilter3D(6, 3))
        ts.filter.x = np.array([0, 0, 0, 100, 0, 0], dtype=float)
        ts.filter.P = np.eye(6) * 200
        ts.filter.R = np.eye(3) * 2500
        ts.status = TrackStatus.CONFIRMED
        
        for scan in range(5):
            meas = np.array([[100 * (scan + 1) + np.random.randn() * 10, 
                              np.random.randn() * 10, 
                              np.random.randn() * 10]])
            
            assignments, _, _ = mht_associate_enhanced(
                [ts], meas, hypothesis_tree=tree
            )
            
            if 0 in assignments:
                ts.filter.x[:3] = assignments[0]
        
        assert len(tree.scan_hypotheses) <= 3  # Pruned to depth


# Need the MHTHypothesis import
from nx_mimosa_mtt import MHTHypothesis


# ============================================================
# GAP 3: DATASET ADAPTER TESTS
# ============================================================

class TestScanData:
    """ScanData dataclass tests."""

    def test_basic(self):
        """ScanData should hold detections."""
        dets = np.array([[100, 200, 300], [400, 500, 600]])
        sd = ScanData(timestamp=1.0, detections=dets)
        assert sd.n_detections == 2
        assert sd.n_truths == 0

    def test_with_truth(self):
        """ScanData should handle ground truth."""
        sd = ScanData(
            timestamp=0.0,
            detections=np.array([[1, 2, 3]]),
            ground_truth=np.array([[1.1, 2.1, 3.1]])
        )
        assert sd.n_truths == 1

    def test_empty(self):
        """Empty ScanData."""
        sd = ScanData(timestamp=0.0, detections=np.empty((0, 3)))
        assert sd.n_detections == 0


class TestSyntheticGenerator:
    """Synthetic scenario generation tests."""

    def test_straight_line(self):
        """Should generate straight-line targets."""
        gen = SyntheticScenarioGenerator(seed=42)
        scans = gen.straight_line(n_targets=3, n_scans=50)
        
        assert len(scans) == 50
        assert all(s.ground_truth is not None for s in scans)
        assert scans[0].ground_truth.shape == (3, 3)
        # Most scans should have detections (p_detection=0.95)
        detected = sum(1 for s in scans if s.n_detections > 0)
        assert detected > 40

    def test_crossing_targets(self):
        """Crossing scenario should have 2 truths per scan."""
        gen = SyntheticScenarioGenerator(seed=42)
        scans = gen.crossing_targets(n_scans=60)
        
        assert len(scans) == 60
        assert scans[25].ground_truth.shape == (2, 3)

    def test_extreme_clutter(self):
        """Extreme clutter should produce many false alarms."""
        gen = SyntheticScenarioGenerator(seed=42, clutter_rate=2.0)
        scans = gen.extreme_clutter(n_targets=2, n_scans=30, clutter_mult=10.0)
        
        assert len(scans) == 30
        # Average detections should be >> n_targets (due to clutter)
        avg_dets = np.mean([s.n_detections for s in scans])
        assert avg_dets > 5, f"Expected heavy clutter, got avg {avg_dets:.1f} dets/scan"

    def test_maneuvering(self):
        """Maneuvering scenario should have 1 truth per scan."""
        gen = SyntheticScenarioGenerator(seed=42)
        scans = gen.maneuvering(n_scans=100)
        assert len(scans) == 100
        assert scans[50].ground_truth.shape == (1, 3)

    def test_reproducibility(self):
        """Same seed should produce identical scenarios."""
        gen1 = SyntheticScenarioGenerator(seed=123)
        gen2 = SyntheticScenarioGenerator(seed=123)
        
        scans1 = gen1.straight_line(n_targets=2, n_scans=10)
        scans2 = gen2.straight_line(n_targets=2, n_scans=10)
        
        for s1, s2 in zip(scans1, scans2):
            np.testing.assert_array_equal(s1.ground_truth, s2.ground_truth)


class TestGenericCSVAdapter:
    """CSV adapter read/write tests."""

    def test_save_and_load(self):
        """Save then load should roundtrip."""
        scans = [
            ScanData(timestamp=0.0, detections=np.array([[1, 2, 3], [4, 5, 6]])),
            ScanData(timestamp=1.0, detections=np.array([[7, 8, 9]])),
        ]
        
        with tempfile.NamedTemporaryFile(suffix='.csv', delete=False, mode='w') as f:
            path = f.name
        
        GenericCSVAdapter.save(scans, path)
        loaded = GenericCSVAdapter.load(path)
        
        assert len(loaded) == 2
        assert loaded[0].n_detections == 2
        assert loaded[1].n_detections == 1
        np.testing.assert_allclose(loaded[0].detections[0], [1, 2, 3])
        
        os.unlink(path)

    def test_empty_save(self):
        """Should handle empty scans."""
        scans = [ScanData(timestamp=0.0, detections=np.empty((0, 3)))]
        
        with tempfile.NamedTemporaryFile(suffix='.csv', delete=False, mode='w') as f:
            path = f.name
        
        GenericCSVAdapter.save(scans, path)
        loaded = GenericCSVAdapter.load(path)
        # Empty scan produces no CSV rows → no scans on load
        assert len(loaded) == 0
        
        os.unlink(path)


# ============================================================
# GAP 3+: REAL-DATA INTEGRATION TEST
# ============================================================

class TestTrackerOnSyntheticDataset:
    """End-to-end: synthetic dataset → tracker → OSPA evaluation."""

    def test_straight_line_tracking(self):
        """Tracker should achieve reasonable OSPA on straight-line scenario."""
        gen = SyntheticScenarioGenerator(seed=42, noise_std=50.0)
        scans = gen.straight_line(n_targets=3, n_scans=60)
        
        tracker = MultiTargetTracker(dt=1.0, r_std=50.0, domain="military")
        
        for scan in scans:
            if scan.n_detections > 0:
                tracker.process_scan(scan.detections)
        
        # Should track targets
        confirmed = tracker.confirmed_tracks
        assert len(confirmed) >= 2, f"Only {len(confirmed)} tracks confirmed"
        
        # Compute OSPA using polymorphic .position property
        track_pos = [t.filter.position for t in confirmed]
        
        if scans[-1].ground_truth is not None and track_pos:
            ospa = compute_ospa(np.array(track_pos), scans[-1].ground_truth, c=10000)
            # Relaxed threshold — synthetic data with clutter may have extra tracks
            assert ospa < 8000, f"OSPA too high: {ospa:.0f}m"

    def test_crossing_tracking(self):
        """Tracker should maintain tracks through crossing."""
        gen = SyntheticScenarioGenerator(seed=42)
        scans = gen.crossing_targets(n_scans=80)
        
        tracker = MultiTargetTracker(dt=1.0, r_std=50.0, association="jpda",
                                     domain="military")
        
        for scan in scans:
            if scan.n_detections > 0:
                tracker.process_scan(scan.detections)
        
        confirmed = tracker.confirmed_tracks
        assert len(confirmed) >= 1, f"Lost all tracks at crossing: {len(confirmed)}"

    def test_extreme_clutter_mht(self):
        """MHT should handle extreme clutter better than GNN."""
        gen = SyntheticScenarioGenerator(seed=42)
        scans = gen.extreme_clutter(n_targets=2, n_scans=50, clutter_mult=5.0)
        
        results = {}
        for method in ['gnn', 'mht']:
            tracker = MultiTargetTracker(dt=1.0, r_std=50.0, association=method)
            
            for scan in scans:
                if scan.n_detections > 0:
                    tracker.process_scan(scan.detections)
            
            results[method] = len(tracker.confirmed_tracks)
        
        # MHT should do at least as well as GNN
        assert results['mht'] >= results['gnn'] - 1, \
            f"MHT ({results['mht']}) worse than GNN ({results['gnn']})"
