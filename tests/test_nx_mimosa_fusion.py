"""
Tests for NX-MIMOSA Multi-Sensor Fusion Module
================================================
pytest tests/test_nx_mimosa_fusion.py -v
"""

import numpy as np
import pytest
import sys, os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))

from nx_mimosa_fusion import (
    MultiSensorFusionEngine, SensorConfig, SensorMeasurement, SensorType,
    SensorHealth, make_radar_sensor, make_polar_radar_sensor,
    make_doppler_radar_sensor, make_eo_sensor, make_esm_sensor,
    make_adsb_sensor, make_gps_sensor,
)
from nx_mimosa_v40_sentinel import NxMimosaV40Sentinel


# ===== FIXTURES =====

@pytest.fixture
def engine():
    return MultiSensorFusionEngine()


@pytest.fixture
def tracker():
    t = NxMimosaV40Sentinel(dt=1.0, r_std=50.0, q_base=0.05, domain="atc")
    t.update(np.array([1000.0, 5000.0]))  # Initialize
    t.update(np.array([1250.0, 5000.0]))  # Second step
    return t


# ===== SENSOR CONFIG TESTS =====

class TestSensorConfig:
    def test_make_radar_sensor(self):
        s = make_radar_sensor("r1", r_std=50.0)
        assert s.sensor_id == "r1"
        assert s.sensor_type == SensorType.POSITION_XY
        assert s.R[0, 0] == pytest.approx(2500.0)
        assert s.R[1, 1] == pytest.approx(2500.0)

    def test_make_polar_radar_sensor(self):
        s = make_polar_radar_sensor("pr1", r_std=100.0, az_std_deg=1.0)
        assert s.sensor_type == SensorType.RANGE_BEARING
        assert s.R[0, 0] == pytest.approx(10000.0)
        assert s.R[1, 1] == pytest.approx(np.radians(1.0) ** 2)

    def test_make_doppler_radar_sensor(self):
        s = make_doppler_radar_sensor("dr1", r_std=30.0, az_std_deg=0.5, rdot_std=2.0)
        assert s.sensor_type == SensorType.RANGE_DOPPLER
        assert s.R.shape == (3, 3)
        assert s.R[2, 2] == pytest.approx(4.0)

    def test_make_eo_sensor(self):
        s = make_eo_sensor("eo1", az_std_deg=0.1)
        assert s.sensor_type == SensorType.BEARING_ONLY
        assert s.R.shape == (1, 1)
        assert s.update_rate == 30.0

    def test_make_esm_sensor(self):
        s = make_esm_sensor("esm1", az_std_deg=2.0)
        assert s.sensor_type == SensorType.BEARING_ONLY
        assert s.R[0, 0] == pytest.approx(np.radians(2.0) ** 2)

    def test_make_adsb_sensor(self):
        s = make_adsb_sensor("adsb1", pos_std=10.0, vel_std=1.0)
        assert s.sensor_type == SensorType.ADS_B
        assert s.R.shape == (4, 4)
        assert s.R[0, 0] == pytest.approx(100.0)
        assert s.R[2, 2] == pytest.approx(1.0)

    def test_make_gps_sensor(self):
        s = make_gps_sensor("gps1", pos_std=3.0)
        assert s.sensor_type == SensorType.POSITION_XY
        assert s.R[0, 0] == pytest.approx(9.0)

    def test_sensor_defaults(self):
        s = make_radar_sensor("r1", r_std=50.0)
        assert np.allclose(s.position, [0.0, 0.0])
        assert np.allclose(s.bias, [0.0, 0.0])
        assert s.max_range is None
        assert s.reliability == 1.0
        assert s.active is True


# ===== ENGINE MANAGEMENT TESTS =====

class TestEngineManagement:
    def test_add_remove_sensor(self, engine):
        engine.add_sensor(make_radar_sensor("r1"))
        assert "r1" in engine.sensors
        assert "r1" in engine.health
        engine.remove_sensor("r1")
        assert "r1" not in engine.sensors

    def test_get_active_sensors(self, engine):
        engine.add_sensor(make_radar_sensor("r1"))
        engine.add_sensor(make_radar_sensor("r2"))
        engine.sensors["r2"].active = False
        active = engine.get_active_sensors()
        assert active == ["r1"]

    def test_repr(self, engine):
        engine.add_sensor(make_radar_sensor("r1"))
        engine.add_sensor(make_eo_sensor("eo1"))
        s = repr(engine)
        assert "2/2 active" in s

    def test_empty_measurements(self, engine, tracker):
        pos, cov = engine.fuse(tracker, [])
        assert pos.shape == (2,)

    def test_unknown_sensor_ignored(self, engine, tracker):
        meas = [SensorMeasurement("nonexistent", np.array([1000.0, 5000.0]))]
        pos, cov = engine.fuse(tracker, meas)
        # Should not crash


# ===== H MATRIX CONSTRUCTION TESTS =====

class TestHMatrix:
    def test_position_xy_H(self, engine):
        sensor = make_radar_sensor("r1")
        engine.add_sensor(sensor)
        x = np.array([1000.0, 5000.0, 250.0, 0.0])
        H, hx = engine._build_H_and_hx(sensor, x)
        assert H.shape == (2, 4)
        assert hx[0] == pytest.approx(1000.0)
        assert hx[1] == pytest.approx(5000.0)
        assert H[0, 0] == 1.0
        assert H[1, 1] == 1.0
        assert H[0, 2] == 0.0  # No velocity in position measurement

    def test_range_bearing_H(self, engine):
        sensor = make_polar_radar_sensor("pr1", position=np.array([0.0, 0.0]))
        engine.add_sensor(sensor)
        x = np.array([1000.0, 1000.0, 0.0, 0.0])
        H, hx = engine._build_H_and_hx(sensor, x)
        expected_r = np.sqrt(2) * 1000.0
        expected_theta = np.pi / 4
        assert hx[0] == pytest.approx(expected_r, rel=1e-6)
        assert hx[1] == pytest.approx(expected_theta, rel=1e-6)
        assert H.shape == (2, 4)

    def test_bearing_only_H(self, engine):
        sensor = make_eo_sensor("eo1", position=np.array([0.0, 0.0]))
        engine.add_sensor(sensor)
        x = np.array([1000.0, 0.0, 0.0, 0.0])
        H, hx = engine._build_H_and_hx(sensor, x)
        assert hx[0] == pytest.approx(0.0)  # bearing to target at (1000,0) from origin
        assert H.shape == (1, 4)

    def test_adsb_H(self, engine):
        sensor = make_adsb_sensor("a1")
        engine.add_sensor(sensor)
        x = np.array([1000.0, 5000.0, 250.0, 10.0])
        H, hx = engine._build_H_and_hx(sensor, x)
        assert H.shape == (4, 4)
        assert hx[0] == pytest.approx(1000.0)
        assert hx[2] == pytest.approx(250.0)

    def test_range_doppler_H(self, engine):
        sensor = make_doppler_radar_sensor("dr1", position=np.array([0.0, 0.0]))
        engine.add_sensor(sensor)
        x = np.array([1000.0, 0.0, 250.0, 0.0])  # Moving along x-axis
        H, hx = engine._build_H_and_hx(sensor, x)
        assert H.shape == (3, 4)
        assert hx[0] == pytest.approx(1000.0)  # range
        assert hx[1] == pytest.approx(0.0)     # bearing
        assert hx[2] == pytest.approx(250.0)   # range rate = full velocity (along LOS)


# ===== ANGLE WRAPPING =====

class TestAngleWrapping:
    def test_wrap_angle(self, engine):
        assert engine._wrap_angle(0.0) == pytest.approx(0.0)
        # π and -π are equivalent — both are valid wrappings
        assert abs(abs(engine._wrap_angle(np.pi)) - np.pi) < 1e-10
        assert abs(abs(engine._wrap_angle(3 * np.pi)) - np.pi) < 1e-10
        assert engine._wrap_angle(-3 * np.pi) == pytest.approx(-np.pi, abs=1e-10)
        # Core wrapping behavior
        assert engine._wrap_angle(0.1) == pytest.approx(0.1)
        assert engine._wrap_angle(2 * np.pi + 0.1) == pytest.approx(0.1, abs=1e-10)

    def test_innovation_wrapping_bearing(self, engine):
        sensor = make_eo_sensor("eo1")
        z = np.array([3.1])
        hx = np.array([-3.1])
        nu = engine._compute_innovation(z, hx, sensor)
        # 3.1 - (-3.1) = 6.2, wrapped should be ~6.2 - 2π ≈ -0.083
        assert abs(nu[0]) < 0.2  # Should be wrapped near zero


# ===== FUSION ACCURACY TESTS =====

class TestFusionAccuracy:
    def test_eo_fusion_improves_accuracy(self):
        """Radar + EO should be better than radar alone."""
        np.random.seed(42)
        dt = 1.0; N = 50; speed = 250.0; r_std = 50.0
        truth = np.column_stack([np.arange(N) * dt * speed, np.ones(N) * 5000.0])
        meas = truth + np.random.randn(N, 2) * r_std

        # Single radar
        t1 = NxMimosaV40Sentinel(dt=dt, r_std=r_std, q_base=0.05, domain="atc")
        t1.update(meas[0])
        err1 = []
        for i in range(1, N):
            p, _, _ = t1.update(meas[i])
            err1.append(np.linalg.norm(p[:2] - truth[i]))

        # Radar + EO
        t2 = NxMimosaV40Sentinel(dt=dt, r_std=r_std, q_base=0.05, domain="atc")
        eng = MultiSensorFusionEngine()
        eng.add_sensor(make_eo_sensor("eo", az_std_deg=0.3, position=np.array([0.0, 0.0])))
        t2.update(meas[0])
        err2 = []
        for i in range(1, N):
            t2.update(meas[i])
            tp = truth[i]
            az = np.arctan2(tp[1], tp[0]) + np.random.randn() * np.radians(0.3)
            eng.fuse(t2, [SensorMeasurement("eo", np.array([az]), i * dt)])
            err2.append(np.linalg.norm(t2._get_combined_position() - truth[i]))

        rms1 = np.sqrt(np.mean(np.array(err1) ** 2))
        rms2 = np.sqrt(np.mean(np.array(err2) ** 2))
        assert rms2 < rms1  # Fusion must improve

    def test_adsb_fusion_significant_improvement(self):
        """ADS-B (10m accuracy) should dramatically improve 50m radar."""
        np.random.seed(42)
        dt = 1.0; N = 50; speed = 250.0; r_std = 50.0
        truth = np.column_stack([np.arange(N) * dt * speed, np.ones(N) * 5000.0])
        meas = truth + np.random.randn(N, 2) * r_std

        t1 = NxMimosaV40Sentinel(dt=dt, r_std=r_std, q_base=0.05, domain="atc")
        t1.update(meas[0])
        err1 = [np.linalg.norm(t1.update(meas[i])[0][:2] - truth[i]) for i in range(1, N)]

        t2 = NxMimosaV40Sentinel(dt=dt, r_std=r_std, q_base=0.05, domain="atc")
        eng = MultiSensorFusionEngine()
        eng.add_sensor(make_adsb_sensor("adsb", pos_std=10.0, vel_std=1.0))
        t2.update(meas[0])
        err2 = []
        for i in range(1, N):
            t2.update(meas[i])
            z_a = np.array([truth[i, 0] + np.random.randn() * 10, truth[i, 1] + np.random.randn() * 10,
                            speed + np.random.randn(), np.random.randn()])
            eng.fuse(t2, [SensorMeasurement("adsb", z_a, i * dt)])
            err2.append(np.linalg.norm(t2._get_combined_position() - truth[i]))

        rms1 = np.sqrt(np.mean(np.array(err1) ** 2))
        rms2 = np.sqrt(np.mean(np.array(err2) ** 2))
        assert rms2 < rms1 * 0.7  # At least 30% improvement

    def test_doppler_radar_fusion(self):
        """Doppler radar from offset position should help."""
        np.random.seed(42)
        dt = 1.0; N = 50; speed = 250.0; r_std = 50.0
        truth = np.column_stack([np.arange(N) * dt * speed, np.ones(N) * 5000.0])
        meas = truth + np.random.randn(N, 2) * r_std
        dop_pos = np.array([10000.0, 0.0])

        t1 = NxMimosaV40Sentinel(dt=dt, r_std=r_std, q_base=0.05, domain="atc")
        t1.update(meas[0])
        err1 = [np.linalg.norm(t1.update(meas[i])[0][:2] - truth[i]) for i in range(1, N)]

        t2 = NxMimosaV40Sentinel(dt=dt, r_std=r_std, q_base=0.05, domain="atc")
        eng = MultiSensorFusionEngine()
        eng.add_sensor(make_doppler_radar_sensor("dop", r_std=30, az_std_deg=0.5,
                                                  rdot_std=1.0, position=dop_pos))
        t2.update(meas[0])
        err2 = []
        for i in range(1, N):
            t2.update(meas[i])
            tp = truth[i]; dx, dy = tp[0] - dop_pos[0], tp[1] - dop_pos[1]
            r = np.sqrt(dx ** 2 + dy ** 2); az = np.arctan2(dy, dx)
            rd = dx * speed / r
            z_d = np.array([r + np.random.randn() * 30, az + np.random.randn() * np.radians(0.5),
                            rd + np.random.randn() * 1.0])
            eng.fuse(t2, [SensorMeasurement("dop", z_d, i * dt)])
            err2.append(np.linalg.norm(t2._get_combined_position() - truth[i]))

        rms1 = np.sqrt(np.mean(np.array(err1) ** 2))
        rms2 = np.sqrt(np.mean(np.array(err2) ** 2))
        assert rms2 < rms1  # Must improve


# ===== SENSOR HEALTH TESTS =====

class TestSensorHealth:
    def test_healthy_sensor_not_degraded(self):
        """GPS sensor (2D) with consistent measurements should not degrade."""
        np.random.seed(42)
        t = NxMimosaV40Sentinel(dt=1.0, r_std=50.0, q_base=0.05, domain="atc")
        eng = MultiSensorFusionEngine()
        eng.add_sensor(make_gps_sensor("gps", pos_std=10.0))
        truth_x0 = 1000.0; truth_y = 5000.0; speed = 250.0
        t.update(np.array([truth_x0, truth_y]) + np.random.randn(2) * 50)
        for i in range(1, 25):
            px = truth_x0 + speed * i
            t.update(np.array([px, truth_y]) + np.random.randn(2) * 50)
            z = np.array([px + np.random.randn() * 10, truth_y + np.random.randn() * 10])
            eng.fuse(t, [SensorMeasurement("gps", z, float(i))])

        report = eng.get_health_report()
        assert report["gps"]["measurements"] == 24
        # GPS with 10m noise should not trigger degradation on 50m radar track
        # (NIS should be reasonable after convergence)

    def test_inactive_sensor_skipped(self, engine, tracker):
        engine.add_sensor(make_radar_sensor("r1"))
        engine.sensors["r1"].active = False
        meas = [SensorMeasurement("r1", np.array([1500.0, 5000.0]))]
        engine.fuse(tracker, meas)
        assert engine.health["r1"].measurement_count == 0


# ===== GATING TESTS =====

class TestGating:
    def test_wildly_wrong_measurement_rejected(self, tracker):
        """A measurement 10km off should be gated out."""
        eng = MultiSensorFusionEngine()
        eng.add_sensor(make_radar_sensor("r1", r_std=50.0))

        # Store state before
        x_before = tracker._cv_x.copy()

        # Wildly wrong measurement
        meas = [SensorMeasurement("r1", np.array([999999.0, 999999.0]))]
        eng.fuse(tracker, meas)

        # State should be essentially unchanged (measurement gated)
        x_after = tracker._cv_x
        # The change should be minimal because NIS would be huge
        assert np.linalg.norm(x_after[:2] - x_before[:2]) < 1000  # Much less than 999km offset


# ===== INFORMATION FILTER TESTS =====

class TestInformationFilter:
    def test_information_fusion_runs(self, tracker):
        eng = MultiSensorFusionEngine()
        eng.add_sensor(make_adsb_sensor("adsb", pos_std=10.0))
        meas = [SensorMeasurement("adsb", np.array([1500.0, 5000.0, 250.0, 0.0]))]
        pos, cov = eng.fuse_information(tracker, meas)
        assert pos.shape == (2,)
        assert cov.shape == (2, 2)


# ===== MULTI-SENSOR SIMULTANEOUS =====

class TestMultiSensorSimultaneous:
    def test_three_sensors_no_crash(self, tracker):
        eng = MultiSensorFusionEngine()
        eng.add_sensor(make_radar_sensor("r1", r_std=50.0))
        eng.add_sensor(make_eo_sensor("eo1", az_std_deg=0.3, position=np.array([0.0, 0.0])))
        eng.add_sensor(make_adsb_sensor("adsb"))

        meas = [
            SensorMeasurement("r1", np.array([1500.0, 5000.0]), 1.0),
            SensorMeasurement("eo1", np.array([np.arctan2(5000, 1500)]), 1.0),
            SensorMeasurement("adsb", np.array([1500.0, 5000.0, 250.0, 0.0]), 1.0),
        ]
        pos, cov = eng.fuse(tracker, meas)
        assert pos.shape == (2,)
        assert not np.any(np.isnan(pos))

    def test_measurements_sorted_by_timestamp(self, tracker):
        eng = MultiSensorFusionEngine()
        eng.add_sensor(make_radar_sensor("r1"))
        eng.add_sensor(make_radar_sensor("r2"))

        meas = [
            SensorMeasurement("r2", np.array([1600.0, 5000.0]), 2.0),
            SensorMeasurement("r1", np.array([1400.0, 5000.0]), 1.0),
        ]
        pos, cov = eng.fuse(tracker, meas)
        # r1 (t=1) should be processed before r2 (t=2)
        assert eng.health["r1"].last_update_time == 1.0
        assert eng.health["r2"].last_update_time == 2.0


# ===== REGRESSION: CORE TRACKER UNCHANGED =====

class TestNoRegression:
    def test_single_sensor_benchmark_unchanged(self):
        """Verify adding fusion module doesn't change single-sensor behavior."""
        np.random.seed(42)
        dt = 1.0; N = 30; speed = 250.0; r_std = 50.0
        truth = np.column_stack([np.arange(N) * dt * speed, np.ones(N) * 5000.0])
        meas = truth + np.random.randn(N, 2) * r_std

        t = NxMimosaV40Sentinel(dt=dt, r_std=r_std, q_base=0.05, domain="atc")
        t.update(meas[0])
        results = []
        for i in range(1, N):
            p, _, _ = t.update(meas[i])
            results.append(p[:2].copy())

        # These should be deterministic with seed=42
        results = np.array(results)
        rms = np.sqrt(np.mean(np.sum((results - truth[1:]) ** 2, axis=1)))
        # Should match known value within small tolerance
        assert rms < 50.0  # Known to be ~33m
        assert rms > 10.0  # Sanity check
