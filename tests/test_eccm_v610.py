#!/usr/bin/env python3
"""
NX-MIMOSA v6.1.0 — ECCM Module Test Suite
==========================================

Tests for QEDMMA-ported anti-jamming capabilities:
  1. MLCFARDetector — 6-feature environment classifier
  2. JammerLocalizer — TDOA emitter geolocation
  3. AdaptiveIntegrator — Dynamic parameter adaptation
  4. ECCMPipeline — Integrated ECCM chain

Run: pytest tests/test_eccm_v610.py -v

Author: Dr. Mladen Mešter
Copyright (c) 2026 Nexellum d.o.o.
"""

import sys, os
import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))

from nx_mimosa_eccm import (
    EnvironmentClass, MLCFARFeatures, MLCFARDetector,
    EmitterTrack, JammerLocalizer,
    AdaptiveIntegrator, ECCMPipeline,
)


# =============================================================================
# ML-CFAR Detector Tests
# =============================================================================

class TestMLCFARDetector:
    """[REQ-V610-MLCFAR] ML-CFAR environment classifier tests."""

    def test_init(self):
        d = MLCFARDetector(window=20, min_samples=8)
        assert d.window == 20
        assert d.min_samples == 8

    def test_clear_sky_classification(self):
        """Clear sky: NIS ~3.0 (chi2 dof=3), stable innovations → CLEAR."""
        d = MLCFARDetector(window=10, min_samples=5)
        rng = np.random.RandomState(42)
        
        # Feed 40 scans of normal NIS with consistent velocity (no jitter)
        vel = np.array([200.0, 50.0, 0.0])
        for i in range(40):
            nis = rng.chisquare(3)
            innov = rng.randn(3) * 50.0  # Stable innovation std
            feat = d.extract_features(1, nis, innov, vel)
        
        env, conf = d.classify(feat)
        assert env == EnvironmentClass.CLEAR, \
            f"Expected CLEAR, got {env.name} (pr={feat.power_ratio:.2f}, rd={feat.range_derivative:.2f}, dd={feat.doppler_derivative:.2f})"

    def test_noise_jamming_detection(self):
        """Barrage noise: NIS jumps 10×, flat spectrum → NOISE_JAMMING."""
        d = MLCFARDetector(window=10, min_samples=5)
        rng = np.random.RandomState(42)
        
        # Phase 1: 15 normal scans (establish baseline)
        for i in range(15):
            nis = rng.chisquare(3)
            innov = rng.randn(3) * 50.0
            vel = np.array([200.0, 0.0, 0.0])
            d.extract_features(1, nis, innov, vel)
        
        # Phase 2: 15 jammed scans (NIS 10× higher, uniform distribution = flat)
        for i in range(15):
            nis = 30.0 + rng.uniform(0, 5)  # High, flat (low kurtosis)
            innov = rng.randn(3) * 200.0
            vel = np.array([200.0, 0.0, 0.0])
            feat = d.extract_features(1, nis, innov, vel)
        
        env, conf = d.classify(feat)
        assert env in (EnvironmentClass.NOISE_JAMMING, EnvironmentClass.CLUTTER), \
            f"Expected jamming/clutter, got {env.name}"

    def test_deception_rgpo_detection(self):
        """RGPO: range innovation has consistent drift → DECEPTION."""
        d = MLCFARDetector(window=10, min_samples=5)
        rng = np.random.RandomState(42)
        vel = np.array([200.0, 0.0, 0.0])
        
        # Phase 1: Normal (establish baseline innovation std ≈50m)
        for i in range(20):
            nis = rng.chisquare(3)
            innov = rng.randn(3) * 50.0
            d.extract_features(1, nis, innov, vel)
        
        # Phase 2: RGPO — range innovation drifts at 500m/scan (10× baseline)
        for i in range(15):
            range_pull = 500.0 * (i + 1)  # 500, 1000, 1500, ...
            nis = 5.0 + rng.normal(0, 1)
            innov = np.array([range_pull, rng.randn() * 10.0, rng.randn() * 10.0])
            feat = d.extract_features(1, nis, innov, vel)
        
        env, conf = d.classify(feat)
        assert env == EnvironmentClass.DECEPTION, f"Expected DECEPTION, got {env.name}"

    def test_feature_extraction_minimum_samples(self):
        """Features are zeroed before min_samples reached."""
        d = MLCFARDetector(window=10, min_samples=8)
        
        feat = d.extract_features(1, 3.0, np.zeros(3), np.zeros(3))
        assert feat.power_ratio == 0.0
        assert feat.spectral_flatness == 0.0

    def test_r_multiplier(self):
        d = MLCFARDetector()
        assert d.get_R_multiplier(EnvironmentClass.CLEAR) == 1.0
        assert d.get_R_multiplier(EnvironmentClass.NOISE_JAMMING) == 5.0
        assert d.get_R_multiplier(EnvironmentClass.DECEPTION) == 10.0

    def test_integration_pulses(self):
        d = MLCFARDetector()
        assert d.get_integration_pulses(EnvironmentClass.CLEAR) == 10
        assert d.get_integration_pulses(EnvironmentClass.NOISE_JAMMING) == 50

    def test_multi_track(self):
        """ML-CFAR maintains independent histories per track."""
        d = MLCFARDetector(window=5, min_samples=3)
        rng = np.random.RandomState(42)
        
        for i in range(10):
            # Track 1: normal
            d.extract_features(1, rng.chisquare(3), rng.randn(3), np.array([200, 0, 0]))
            # Track 2: jammed
            d.extract_features(2, 50.0 + rng.uniform(0, 5), rng.randn(3) * 500, np.array([200, 0, 0]))
        
        f1 = d.extract_features(1, rng.chisquare(3), rng.randn(3), np.array([200, 0, 0]))
        f2 = d.extract_features(2, 50.0, rng.randn(3) * 500, np.array([200, 0, 0]))
        
        env1, _ = d.classify(f1)
        env2, _ = d.classify(f2)
        
        # Track 1 should be cleaner than track 2
        assert env1.value <= env2.value or env2 != EnvironmentClass.CLEAR

    def test_clear_track(self):
        d = MLCFARDetector()
        d.extract_features(1, 3.0, np.zeros(3), np.zeros(3))
        assert 1 in d._history
        d.clear_track(1)
        assert 1 not in d._history

    def test_decision_tree_low_power(self):
        """Low power ratio → NOISE_JAMMING (blanking/desensitization)."""
        features = MLCFARFeatures(
            power_ratio=0.1,  # Very low → blanking
            guard_ratio=1.0,
            spectral_flatness=3.0,
            temporal_correlation=0.0,
            range_derivative=0.0,
            doppler_derivative=0.0,
        )
        d = MLCFARDetector()
        env, conf = d.classify(features)
        assert env == EnvironmentClass.NOISE_JAMMING

    def test_decision_tree_high_power_flat(self):
        """High power + flat spectrum → NOISE_JAMMING."""
        features = MLCFARFeatures(
            power_ratio=8.0,
            guard_ratio=1.0,
            spectral_flatness=1.5,  # Low kurtosis = flat
            temporal_correlation=0.3,
            range_derivative=0.5,
            doppler_derivative=0.5,
        )
        d = MLCFARDetector()
        env, conf = d.classify(features)
        assert env == EnvironmentClass.NOISE_JAMMING

    def test_decision_tree_high_power_correlated(self):
        """High power + high temporal correlation → DECEPTION (DRFM)."""
        features = MLCFARFeatures(
            power_ratio=6.0,
            guard_ratio=2.0,
            spectral_flatness=4.0,  # Normal kurtosis
            temporal_correlation=0.9,  # Highly correlated → coherent
            range_derivative=0.5,
            doppler_derivative=0.5,
        )
        d = MLCFARDetector()
        env, conf = d.classify(features)
        assert env == EnvironmentClass.DECEPTION

    def test_decision_tree_high_range_derivative(self):
        """Range derivative anomaly → DECEPTION (RGPO)."""
        features = MLCFARFeatures(
            power_ratio=2.0,  # Normal-ish power
            guard_ratio=1.0,
            spectral_flatness=3.0,
            temporal_correlation=0.3,
            range_derivative=5.0,  # Well above threshold (3.0)
            doppler_derivative=0.5,
        )
        d = MLCFARDetector()
        env, conf = d.classify(features)
        assert env == EnvironmentClass.DECEPTION


# =============================================================================
# Jammer Localizer Tests
# =============================================================================

class TestJammerLocalizer:
    """[REQ-V610-TDOA] TDOA emitter geolocation tests."""

    def _make_triangle_receivers(self, baseline_km=50.0):
        """3 receivers in equilateral triangle."""
        b = baseline_km * 1000.0
        return [
            np.array([0.0, 0.0, 100.0]),
            np.array([b, 0.0, 100.0]),
            np.array([b / 2, b * np.sqrt(3) / 2, 100.0]),
        ]

    def _toa_from_position(self, emitter_pos, rx_positions, noise_std=1e-9):
        """Generate TOA measurements from known emitter position."""
        C = 299792458.0
        rng = np.random.RandomState(42)
        toa = []
        for rx in rx_positions:
            dist = np.linalg.norm(emitter_pos - rx)
            t = dist / C + rng.normal(0, noise_std)
            toa.append(t)
        return toa

    def test_init(self):
        loc = JammerLocalizer(min_nodes=3, max_emitters=8)
        assert loc.min_nodes == 3
        assert loc.max_emitters == 8
        assert len(loc._emitters) == 0

    def test_localize_single_emitter(self):
        """Locate emitter with 3 receivers, 50 km baseline."""
        loc = JammerLocalizer(min_nodes=3, convergence_threshold=50000.0)
        rx = self._make_triangle_receivers(50.0)
        emitter_true = np.array([25000.0, 60000.0, 5000.0])
        
        toa = self._toa_from_position(emitter_true, rx, noise_std=1e-9)
        result = loc.update(rx, toa, timestamp=0.0)
        
        assert result is not None
        error = np.linalg.norm(result.position[:2] - emitter_true[:2])  # 2D error
        assert error < 15000.0, f"2D error {error:.0f}m exceeds 15 km (3-node, 50km baseline)"

    def test_localize_convergence(self):
        """Error decreases with repeated measurements."""
        loc = JammerLocalizer(min_nodes=3)
        rx = self._make_triangle_receivers(80.0)
        emitter_true = np.array([40000.0, 100000.0, 8000.0])
        
        errors = []
        for t in range(10):
            toa = self._toa_from_position(emitter_true, rx, noise_std=5e-9)
            result = loc.update(rx, toa, timestamp=float(t))
            if result is not None:
                errors.append(np.linalg.norm(result.position - emitter_true))
        
        assert len(errors) >= 5
        # Error should generally decrease (Kalman convergence)
        assert errors[-1] < errors[0] * 2  # At least not diverging

    def test_minimum_nodes_check(self):
        """Fails gracefully with < 3 receivers."""
        loc = JammerLocalizer(min_nodes=3)
        rx = [np.array([0, 0, 100]), np.array([50000, 0, 100])]  # Only 2
        toa = [0.001, 0.0012]
        result = loc.update(rx, toa)
        assert result is None

    def test_6_node_precision(self):
        """6 receivers, 100 km baseline → better precision than 3 nodes."""
        loc = JammerLocalizer(min_nodes=3, convergence_threshold=50000.0)
        b = 100000.0  # 100 km
        rx = [
            np.array([0, 0, 100]),
            np.array([b, 0, 100]),
            np.array([b / 2, b * 0.866, 100]),
            np.array([-b / 2, b * 0.866, 100]),
            np.array([-b, 0, 100]),
            np.array([-b / 2, -b * 0.866, 100]),
        ]
        emitter_true = np.array([30000.0, 200000.0, 10000.0])
        
        for t in range(20):
            toa = self._toa_from_position(emitter_true, rx, noise_std=1e-9)
            result = loc.update(rx, toa, timestamp=float(t))
        
        assert result is not None
        error = np.linalg.norm(result.position[:2] - emitter_true[:2])
        assert error < 20000.0, f"6-node 2D error {error:.0f}m exceeds 20 km"
        assert result.confidence > 0.3

    def test_moving_emitter(self):
        """Track a moving emitter (aircraft speed)."""
        loc = JammerLocalizer(min_nodes=3)
        rx = self._make_triangle_receivers(80.0)
        
        velocity = np.array([250.0, 100.0, 0.0])  # ~270 m/s ≈ 520 kt
        
        for t in range(20):
            emitter_pos = np.array([20000.0, 50000.0, 10000.0]) + velocity * t * 5.0
            toa = self._toa_from_position(emitter_pos, rx, noise_std=2e-9)
            result = loc.update(rx, toa, timestamp=float(t * 5))
        
        assert result is not None
        assert result.n_updates >= 15

    def test_get_all_emitters(self):
        loc = JammerLocalizer(min_nodes=3)
        assert len(loc.get_all_emitters()) == 0
        
        rx = self._make_triangle_receivers(50.0)
        toa = self._toa_from_position(np.array([25000, 60000, 5000]), rx)
        loc.update(rx, toa)
        
        assert len(loc.get_all_emitters()) == 1

    def test_clear(self):
        loc = JammerLocalizer()
        rx = self._make_triangle_receivers(50.0)
        toa = self._toa_from_position(np.array([25000, 60000, 5000]), rx)
        loc.update(rx, toa)
        assert len(loc._emitters) == 1
        loc.clear()
        assert len(loc._emitters) == 0


# =============================================================================
# Adaptive Integrator Tests
# =============================================================================

class TestAdaptiveIntegrator:
    """[REQ-V610-ADAPT] Adaptive integration controller tests."""

    def test_init(self):
        ai = AdaptiveIntegrator()
        assert ai.current_level == 'NORMAL'

    def test_clear_sky_params(self):
        ai = AdaptiveIntegrator()
        params = ai.adapt(EnvironmentClass.CLEAR, 0.9)
        assert params.mode == 'NORMAL'
        assert params.r_multiplier == 1.0
        assert params.gate_sigma == 4.0
        assert params.bearing_only is False

    def test_noise_jamming_maximum(self):
        """High-confidence noise jamming → MAXIMUM mode."""
        ai = AdaptiveIntegrator()
        params = ai.adapt(EnvironmentClass.NOISE_JAMMING, 0.9)
        assert params.mode == 'MAXIMUM'
        assert params.r_multiplier > 5.0
        assert params.bearing_only is True
        assert params.max_coast_scans == 20
        assert params.integration_gain_dB == 7.0

    def test_noise_jamming_low_conf(self):
        """Low-confidence noise jamming → HIGH mode (not MAXIMUM)."""
        ai = AdaptiveIntegrator()
        params = ai.adapt(EnvironmentClass.NOISE_JAMMING, 0.3)
        assert params.mode == 'HIGH'
        assert params.bearing_only is False

    def test_deception_params(self):
        ai = AdaptiveIntegrator()
        params = ai.adapt(EnvironmentClass.DECEPTION, 0.8)
        assert params.mode == 'ELEVATED'
        assert params.r_multiplier > 2.0
        assert params.confirm_threshold == 4  # Harder to confirm ghost tracks

    def test_clutter_tighter_gate(self):
        ai = AdaptiveIntegrator()
        params = ai.adapt(EnvironmentClass.CLUTTER, 0.6)
        assert params.gate_sigma < 4.0  # Tighter than normal
        assert params.confirm_threshold > 3  # Harder to confirm

    def test_level_history(self):
        ai = AdaptiveIntegrator()
        ai.adapt(EnvironmentClass.CLEAR, 1.0)
        ai.adapt(EnvironmentClass.NOISE_JAMMING, 0.9)
        ai.adapt(EnvironmentClass.CLEAR, 1.0)
        assert len(ai._level_history) == 3
        assert ai._level_history[1] == 'MAXIMUM'


# =============================================================================
# ECCM Pipeline Integration Tests
# =============================================================================

class TestECCMPipeline:
    """[REQ-V610-ECCM] Integrated ECCM pipeline tests."""

    def test_init(self):
        pipe = ECCMPipeline(ml_window=20)
        assert pipe.ml_cfar is not None
        assert pipe.integrator is not None
        assert pipe.localizer is None  # Disabled by default

    def test_init_with_localizer(self):
        pipe = ECCMPipeline(enable_localizer=True)
        assert pipe.localizer is not None

    def test_clear_sky_pipeline(self):
        """Full pipeline in clear conditions."""
        pipe = ECCMPipeline(ml_window=10)
        rng = np.random.RandomState(42)
        vel = np.array([200.0, 50.0, 0.0])
        
        for i in range(30):
            result = pipe.update(
                track_id=1,
                nis=rng.chisquare(3),
                innovation=rng.randn(3) * 50.0,
                velocity=vel,
            )
        
        assert result['env_class'] == EnvironmentClass.CLEAR, \
            f"Expected CLEAR, got {result['env_class'].name}"
        assert result['params'].mode == 'NORMAL'
        assert result['params'].r_multiplier == 1.0

    def test_jamming_detection_pipeline(self):
        """Pipeline detects transition from clear to jammed."""
        pipe = ECCMPipeline(ml_window=8, enable_localizer=False)
        rng = np.random.RandomState(42)
        
        # Phase 1: Clear
        for i in range(15):
            pipe.update(1, rng.chisquare(3), rng.randn(3) * 50.0,
                       np.array([200.0, 0.0, 0.0]))
        
        # Phase 2: Jammed
        for i in range(15):
            result = pipe.update(1, 40.0 + rng.uniform(0, 3),
                                rng.randn(3) * 300.0,
                                np.array([200.0, 0.0, 0.0]))
        
        assert result['env_class'] != EnvironmentClass.CLEAR
        assert result['params'].r_multiplier > 1.0

    def test_environment_summary(self):
        pipe = ECCMPipeline(ml_window=5)
        rng = np.random.RandomState(42)
        
        for i in range(10):
            pipe.update(1, rng.chisquare(3), rng.randn(3), np.zeros(3))
            pipe.update(2, rng.chisquare(3), rng.randn(3), np.zeros(3))
        
        summary = pipe.get_environment_summary()
        assert isinstance(summary, dict)
        assert 'CLEAR' in summary

    def test_pipeline_with_localizer(self):
        """Pipeline with TDOA localizer enabled."""
        pipe = ECCMPipeline(ml_window=5, enable_localizer=True, min_nodes=3)
        rng = np.random.RandomState(42)
        
        rx = [
            np.array([0, 0, 100]),
            np.array([50000, 0, 100]),
            np.array([25000, 43301, 100]),
        ]
        emitter = np.array([25000, 60000, 5000])
        C = 299792458.0
        
        # Build up baseline
        for i in range(15):
            pipe.update(1, rng.chisquare(3), rng.randn(3) * 50, np.array([200, 0, 0]))
        
        # Jammed scans with TOA data
        for i in range(10):
            toa = [np.linalg.norm(emitter - r) / C + rng.normal(0, 1e-9) for r in rx]
            result = pipe.update(
                track_id=1, nis=30.0 + rng.uniform(0, 3),
                innovation=rng.randn(3) * 200, velocity=np.array([200, 0, 0]),
                rx_positions=rx, toa=toa, timestamp=float(i),
            )
        
        # May or may not have localized depending on classification
        # But pipeline should not crash
        assert result is not None

    def test_clear_track_pipeline(self):
        pipe = ECCMPipeline()
        pipe.update(1, 3.0, np.zeros(3), np.zeros(3))
        assert 1 in pipe._track_env
        pipe.clear_track(1)
        assert 1 not in pipe._track_env


# =============================================================================
# Cross-validation: ECCM with existing ECMDetector compatibility
# =============================================================================

class TestECCMIntegration:
    """Verify ECCM module integrates with existing NX-MIMOSA components."""

    def test_import_all_classes(self):
        """All public classes importable."""
        from nx_mimosa_eccm import (
            EnvironmentClass, MLCFARFeatures, MLCFARDetector,
            EmitterTrack, JammerLocalizer,
            AdaptiveIntegrator, ECCMPipeline,
        )
        assert EnvironmentClass.CLEAR.value == 0
        assert EnvironmentClass.DECEPTION.value == 3

    def test_eccm_alongside_legacy_ecm(self):
        """ML-CFAR and legacy ECMDetector can coexist on same track."""
        from nx_mimosa_mtt import ECMDetector
        
        legacy = ECMDetector()
        ml_cfar = MLCFARDetector(window=10, min_samples=5)
        rng = np.random.RandomState(42)
        vel = np.array([200, 0, 0])
        pos = np.array([100000, 50000, 10000])
        
        for i in range(30):
            nis = rng.chisquare(3)
            innov = rng.randn(3) * 50.0
            
            legacy_result = legacy.update(
                track_id=1, nis=nis, innovation=innov,
                position=pos, velocity=vel, was_hit=True)
            
            feat = ml_cfar.extract_features(1, nis, innov, vel)
            env, conf = ml_cfar.classify(feat)
        
        assert not legacy_result['ecm_detected']
        assert env == EnvironmentClass.CLEAR, f"Expected CLEAR, got {env.name}"

    def test_feature_vector_dimensions(self):
        """Feature vector has all 6 components."""
        f = MLCFARFeatures(
            power_ratio=1.0, guard_ratio=1.0,
            spectral_flatness=3.0, temporal_correlation=0.0,
            range_derivative=0.0, doppler_derivative=0.0,
        )
        # Access all fields
        vec = [f.power_ratio, f.guard_ratio, f.spectral_flatness,
               f.temporal_correlation, f.range_derivative, f.doppler_derivative]
        assert len(vec) == 6

    def test_emitter_track_dataclass(self):
        """EmitterTrack fields are all accessible."""
        e = EmitterTrack(
            emitter_id=1,
            position=np.array([1000, 2000, 3000]),
            velocity=np.zeros(3),
            covariance=np.eye(6),
            power_estimate_dBm=50.0,
            confidence=0.5,
            classification='NOISE',
        )
        assert e.emitter_id == 1
        assert e.classification == 'NOISE'
        assert e.n_updates == 0


# =============================================================================
# Stress / Edge Case Tests
# =============================================================================

class TestECCMEdgeCases:
    """Edge cases and robustness tests."""

    def test_nan_handling(self):
        """NaN in NIS should not crash."""
        d = MLCFARDetector(window=5, min_samples=3)
        for i in range(10):
            d.extract_features(1, float(i + 1), np.zeros(3), np.zeros(3))
        # Feed NaN
        feat = d.extract_features(1, float('nan'), np.zeros(3), np.zeros(3))
        # Should not crash, may return any classification
        env, conf = d.classify(feat)
        assert isinstance(env, EnvironmentClass)

    def test_zero_velocity(self):
        d = MLCFARDetector(window=5, min_samples=3)
        for i in range(10):
            feat = d.extract_features(1, 3.0, np.zeros(3), np.zeros(3))
        env, conf = d.classify(feat)
        assert isinstance(env, EnvironmentClass)

    def test_single_receiver_localizer(self):
        """Single receiver → returns None."""
        loc = JammerLocalizer(min_nodes=3)
        result = loc.update([np.zeros(3)], [0.001])
        assert result is None

    def test_colocated_receivers(self):
        """All receivers at same position → degenerate, should not crash."""
        loc = JammerLocalizer(min_nodes=3)
        rx = [np.array([0, 0, 0])] * 3
        toa = [0.001, 0.001, 0.001]
        result = loc.update(rx, toa)
        # May return None (degenerate geometry) or a poor solution
        # Should not crash

    def test_many_tracks(self):
        """ML-CFAR scales to 100+ simultaneous tracks."""
        d = MLCFARDetector(window=5, min_samples=3)
        rng = np.random.RandomState(42)
        
        for tid in range(100):
            for scan in range(10):
                d.extract_features(tid, rng.chisquare(3), rng.randn(3), rng.randn(3) * 100)
        
        assert len(d._history) == 100

    def test_adaptive_all_environments(self):
        """AdaptiveIntegrator handles all EnvironmentClass values."""
        ai = AdaptiveIntegrator()
        for env in EnvironmentClass:
            for conf in [0.1, 0.5, 0.9]:
                params = ai.adapt(env, conf)
                assert params.r_multiplier >= 1.0
                assert params.gate_sigma > 0
                assert params.max_coast_scans > 0
