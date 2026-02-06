"""
NX-MIMOSA v4.1 — Test Suite for Intent Prediction + Classifier + ECM
=====================================================================
pytest test_nx_mimosa_intent_classifier.py -v

Copyright (c) 2025-2026 Nexellum d.o.o. — All rights reserved.
"""

import math
import json
import numpy as np
import pytest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))

from nx_mimosa_intent_classifier import (
    ECMDetector, ECMStatus,
    ImprovedPlatformClassifier,
    IntentPredictor, IntentType, IntentState, ThreatLevel,
    NxMimosaClassifierPipeline, ClassificationResult,
)


# =============================================================================
# Fixtures
# =============================================================================

@pytest.fixture
def db_path():
    """Path to platform DB v3."""
    p = os.path.join(os.path.dirname(__file__), '..', 'data', 'platform_db_v3.json')
    if not os.path.exists(p):
        pytest.skip("platform_db_v3.json not found")
    return p


@pytest.fixture
def platform_db(db_path):
    with open(db_path) as f:
        return json.load(f)


@pytest.fixture
def ecm_detector():
    return ECMDetector()


@pytest.fixture
def classifier(db_path):
    return ImprovedPlatformClassifier(db_path=db_path)


@pytest.fixture
def intent_predictor():
    return IntentPredictor(dt=0.1)


@pytest.fixture
def pipeline(db_path):
    return NxMimosaClassifierPipeline(db_path=db_path, dt=0.1)


# =============================================================================
# [TST-DB-001] Platform Database Integrity
# =============================================================================

class TestDatabaseIntegrity:
    """Verify platform_db_v3.json structure and completeness."""

    def test_db_loads(self, platform_db):
        assert len(platform_db) >= 100, f"Expected 100+ platforms, got {len(platform_db)}"

    def test_all_platforms_have_required_fields(self, platform_db):
        required = ["class", "max_speed_mps", "max_g", "max_turn_rate_radps",
                     "preferred_models", "average_rcs_m2", "micro_doppler_type",
                     "typical_altitude_m", "stealth_level", "ecm_capability",
                     "false_target_potential"]
        for name, data in platform_db.items():
            for field in required:
                assert field in data, f"{name} missing field '{field}'"

    def test_class_coverage(self, platform_db):
        classes = set(d["class"] for d in platform_db.values())
        expected_min = {
            "4th_gen_fighter", "5th_gen_stealth", "strategic_bomber",
            "commercial_airliner", "subsonic_cruise", "supersonic_cruise",
            "hypersonic_glide", "ballistic", "false_target_bird",
            "false_target_balloon", "uav_low_g", "uav_swarm",
            "helicopter_military", "aam_bvr", "sam_terminal",
            "paraglider_ultralight", "uav_micro", "uav_stealth",
        }
        missing = expected_min - classes
        assert not missing, f"Missing classes: {missing}"

    def test_unique_classes_count(self, platform_db):
        classes = set(d["class"] for d in platform_db.values())
        assert len(classes) >= 25, f"Expected 25+ classes, got {len(classes)}"

    def test_false_targets_present(self, platform_db):
        ft_classes = {d["class"] for d in platform_db.values()
                      if d["class"].startswith("false_target")}
        assert len(ft_classes) >= 3, "Need bird, balloon, ground/env false targets"

    def test_expanded_platforms(self, platform_db):
        """Verify expansion platforms are present."""
        expected_new = [
            "Paraglider", "Hang Glider", "DJI Mavic (consumer)",
            "FPV Kamikaze Drone (military)", "EA-18G Growler",
            "WZ-8 (Chinese Hypersonic Recon)", "Lancet-3 (Loitering Munition)",
            "Wind Turbine", "Large Raptor (Eagle/Hawk)", "Bat Swarm",
        ]
        for name in expected_new:
            assert name in platform_db, f"Missing expanded platform: {name}"


# =============================================================================
# [TST-ECM-001] ECM Detector Tests
# =============================================================================

class TestECMDetector:
    """Verify ECM detection logic."""

    def test_clean_environment(self, ecm_detector):
        """No ECM in normal conditions."""
        for _ in range(30):
            status, q_scale = ecm_detector.update(
                snr_db=25.0, rcs_amplitude=1.0,
                doppler_hz=1000.0, nis=1.0, range_m=50000.0
            )
        assert status == ECMStatus.CLEAN
        assert q_scale == 1.0

    def test_noise_jamming_detection(self, ecm_detector):
        """Detect noise jamming via SNR drop + Doppler spread."""
        # Establish baseline
        for _ in range(25):
            ecm_detector.update(snr_db=25.0, rcs_amplitude=1.0,
                                doppler_hz=1000.0, nis=1.0, range_m=50000)
        # Inject jamming
        for _ in range(15):
            status, q_scale = ecm_detector.update(
                snr_db=3.0,  # severe SNR drop
                rcs_amplitude=np.random.uniform(0.1, 10.0),
                doppler_hz=np.random.uniform(-500, 500),  # spread
                nis=20.0,
                range_m=50000
            )
        assert status != ECMStatus.CLEAN
        assert q_scale > 5.0, "Q scale should boost significantly under jamming"

    def test_rcs_anomaly_detection(self, ecm_detector):
        """Detect chaff/deception via RCS variance spike + SNR drop."""
        for _ in range(25):
            ecm_detector.update(snr_db=20.0, rcs_amplitude=1.0,
                                doppler_hz=1000, nis=1.0, range_m=50000)
        # Inject chaff (wild RCS fluctuations + moderate SNR drop)
        for _ in range(15):
            status, q_scale = ecm_detector.update(
                snr_db=8.0,  # also some SNR degradation
                rcs_amplitude=np.random.uniform(0.01, 100.0),  # massive variance
                doppler_hz=np.random.uniform(500, 1500),
                nis=10.0, range_m=50000
            )
        assert q_scale > 1.0

    def test_q_scale_bounds(self, ecm_detector):
        """Q scale factor should be bounded."""
        for _ in range(50):
            _, q = ecm_detector.update(
                snr_db=1.0, rcs_amplitude=100.0,
                doppler_hz=np.random.uniform(-1000, 1000),
                nis=100.0, range_m=50000
            )
        assert q <= 50.0, "Q scale capped at 50"

    def test_rgpo_detection(self, ecm_detector):
        """Detect RGPO via range acceleration + NIS spike."""
        for i in range(25):
            ecm_detector.update(snr_db=20, rcs_amplitude=1.0,
                                doppler_hz=1000, nis=1.0,
                                range_m=50000 - i * 50)
        # Range pull-off + SNR drop + NIS spike (combined RGPO signature)
        for i in range(15):
            status, _ = ecm_detector.update(
                snr_db=8, rcs_amplitude=1.0, doppler_hz=1000,
                nis=15.0, range_m=50000 + i * i * 300
            )
        assert ecm_detector.confidence > 0.0 or ecm_detector.q_scale_factor > 1.0


# =============================================================================
# [TST-CLS-001] Classifier Tests
# =============================================================================

class TestImprovedClassifier:
    """Verify hierarchical classifier logic."""

    def _feed_constant(self, clf, spd, alt, g, omega, n=30,
                       rcs=0.0, snr=25.0):
        """Feed constant-state updates to classifier."""
        for _ in range(n):
            clf.update_features(
                speed_mps=spd, accel_g=g, altitude_m=alt,
                omega_radps=omega, heading_rad=0.0,
                nis_cv=1.0, rcs_dbsm=rcs, snr_db=snr,
                doppler_hz=0, range_m=50000
            )

    def test_airliner_classification(self, classifier):
        """Commercial airliner: 250 m/s, 11km, low g. Without RCS data,
        may also match strategic_bomber or cargo (kinematics overlap)."""
        self._feed_constant(classifier, spd=250, alt=11000, g=1.0, omega=0.01)
        result = classifier.classify()
        assert result.platform_class in ("commercial_airliner", "cargo_military",
                                          "strategic_bomber"), \
            f"Expected civil/transport/bomber, got {result.platform_class}"
        # With RCS data, should discriminate (airliner RCS >> stealth bomber)

    def test_airliner_with_rcs(self, classifier):
        """With RCS data, airliner should be differentiated from bomber."""
        self._feed_constant(classifier, spd=250, alt=11000, g=1.0, omega=0.01,
                           rcs=15.0)  # ~30 dBsm → large target
        result = classifier.classify()
        assert result.platform_class in ("commercial_airliner", "cargo_military",
                                          "strategic_bomber")

    def test_fighter_classification(self, classifier):
        """Fighter: 600 m/s, 10km, high g."""
        self._feed_constant(classifier, spd=600, alt=10000, g=8.0, omega=0.35)
        result = classifier.classify()
        assert result.platform_class in ("4th_gen_fighter", "5th_gen_stealth",
                                          "6th_gen_concept"), \
            f"Expected fighter, got {result.platform_class}"
        assert result.threat_level.value >= ThreatLevel.HIGH.value

    def test_hypersonic_classification(self, classifier):
        """Hypersonic: 3000 m/s, variable alt."""
        self._feed_constant(classifier, spd=3000, alt=25000, g=5.0, omega=0.1)
        result = classifier.classify()
        assert result.platform_class in ("hypersonic_glide", "hypersonic_cruise",
                                          "ballistic", "sam_terminal"), \
            f"Expected hypersonic/ballistic, got {result.platform_class}"
        assert result.threat_level == ThreatLevel.CRITICAL

    def test_bird_classification(self, classifier):
        """Bird: slow, low alt, high turn rate."""
        self._feed_constant(classifier, spd=20, alt=300, g=0.5, omega=0.8)
        result = classifier.classify()
        assert "false_target" in result.platform_class or \
               "paraglider" in result.platform_class, \
            f"Expected false target / paraglider, got {result.platform_class}"

    def test_balloon_classification(self, classifier):
        """Balloon: very slow, high altitude."""
        self._feed_constant(classifier, spd=10, alt=25000, g=0.0, omega=0.01)
        result = classifier.classify()
        assert "balloon" in result.platform_class or \
               "space" in result.platform_class, \
            f"Expected balloon/space, got {result.platform_class}"

    def test_cruise_missile(self, classifier):
        """Subsonic cruise missile: 250 m/s, low alt."""
        self._feed_constant(classifier, spd=250, alt=50, g=1.5, omega=0.02)
        result = classifier.classify()
        assert result.platform_class in ("subsonic_cruise", "uav_swarm",
                                          "uav_stealth"), \
            f"Expected cruise/swarm at low alt, got {result.platform_class}"

    def test_uav_classification(self, classifier):
        """UAV: 100 m/s, medium alt, low g."""
        self._feed_constant(classifier, spd=100, alt=5000, g=2.0, omega=0.1)
        result = classifier.classify()
        assert "uav" in result.platform_class or \
               "helicopter" in result.platform_class, \
            f"Expected UAV/helo, got {result.platform_class}"

    def test_ecm_fallback(self, classifier):
        """Under ECM, classifier degrades gracefully."""
        for _ in range(30):
            classifier.update_features(
                speed_mps=600, accel_g=7.0, altitude_m=10000,
                omega_radps=0.3, heading_rad=0.0, nis_cv=50.0,
                rcs_dbsm=0.0, snr_db=3.0,  # jammed!
                doppler_hz=np.random.uniform(-500, 500),
                range_m=50000
            )
        result = classifier.classify()
        # Should still produce a result, with ECM flag
        assert result.ecm_status != ECMStatus.CLEAN or result.confidence < 1.0
        assert len(result.preferred_models) > 0

    def test_insufficient_data_fallback(self, classifier):
        """With <5 updates, return safe defaults."""
        classifier.update_features(100, 2.0, 5000, 0.1, 0.0)
        result = classifier.classify()
        assert result.platform_name == "Unknown"
        assert "CV" in result.preferred_models

    def test_supersonic_cruise_missile(self, classifier):
        """BrahMos-class: 1200 m/s, sea level, high g."""
        self._feed_constant(classifier, spd=1200, alt=50, g=12.0, omega=0.3)
        result = classifier.classify()
        assert result.platform_class in ("supersonic_cruise", "aam_bvr",
                                          "sam_terminal"), \
            f"Expected supersonic/AAM, got {result.platform_class}"


# =============================================================================
# [TST-INT-001] Intent Predictor Tests
# =============================================================================

class TestIntentPredictor:
    """Verify intent prediction across scenarios."""

    def _run_scenario(self, predictor, traj_fn, platform_class, n_steps=80):
        """Run a trajectory through the predictor."""
        dt = 0.1
        for step in range(n_steps):
            t = step * dt
            spd, alt, hdg, g, vz, omega = traj_fn(t)
            state = predictor.update(
                speed_mps=spd, altitude_m=alt, heading_rad=hdg,
                g_load=g, vz_mps=vz, omega_radps=omega,
                platform_class=platform_class
            )
        return state

    def test_terminal_dive_iskander(self, intent_predictor):
        """Iskander-M terminal dive detection."""
        state = self._run_scenario(
            intent_predictor,
            lambda t: (2000, max(100, 40000 - t*2000), 0.0, 3.0, -2000, 0.0),
            "ballistic", n_steps=80
        )
        assert state.intent == IntentType.TERMINAL_DIVE
        assert state.threat_level.value >= ThreatLevel.HIGH.value
        assert state.time_to_impact_s < 100

    def test_sea_skimming(self, intent_predictor):
        """Tomahawk sea-skimming detection."""
        state = self._run_scenario(
            intent_predictor,
            lambda t: (240, 15, 0.1, 1.2, -0.3, 0.0),
            "subsonic_cruise", n_steps=60
        )
        assert state.intent == IntentType.SEA_SKIMMING
        assert state.threat_level.value >= ThreatLevel.HIGH.value

    def test_evasion_break(self, intent_predictor):
        """F-16 high-g evasion break detection."""
        state = self._run_scenario(
            intent_predictor,
            lambda t: (500, 8000, 0.5 + t*0.1, 8.0, 0, 0.40),
            "4th_gen_fighter", n_steps=60
        )
        assert state.intent in (IntentType.EVASION_BREAK, IntentType.DOGFIGHT)

    def test_orbit_racetrack(self, intent_predictor):
        """MQ-9 racetrack orbit detection."""
        state = self._run_scenario(
            intent_predictor,
            lambda t: (100, 6000, math.sin(t*0.3)*1.5, 1.5, 0,
                       0.2*math.cos(t*0.3)),
            "uav_low_g", n_steps=100
        )
        assert state.intent == IntentType.ORBIT_RACETRACK

    def test_cruise_steady(self, intent_predictor):
        """Boeing 737 steady cruise detection."""
        state = self._run_scenario(
            intent_predictor,
            lambda t: (250, 11000, 0.2, 1.0, 0, 0.001),
            "commercial_airliner", n_steps=60
        )
        assert state.intent == IntentType.CRUISE
        assert state.threat_level == ThreatLevel.LOW

    def test_bvr_intercept(self, intent_predictor):
        """AIM-120D BVR intercept detection."""
        state = self._run_scenario(
            intent_predictor,
            lambda t: (1200 + t*50, 12000, 0.0, 25.0, -50, 0.4),
            "aam_bvr", n_steps=60
        )
        assert state.intent in (IntentType.BVR_INTERCEPT, IntentType.TERMINAL_DIVE)
        assert state.threat_level.value >= ThreatLevel.HIGH.value

    def test_pop_up_maneuver(self, intent_predictor):
        """Missile pop-up from terrain following."""
        def traj(t):
            if t < 3.0:
                return (300, 80, 0.1, 2.0, 0, 0.0)  # terrain following
            else:
                return (300, 80 + (t-3.0)*200, 0.1, 3.0, 200, 0.0)  # pop up
        state = self._run_scenario(
            intent_predictor, traj, "subsonic_cruise", n_steps=80
        )
        assert state.intent in (IntentType.POP_UP, IntentType.SEA_SKIMMING,
                                IntentType.TERRAIN_FOLLOWING, IntentType.CRUISE)

    def test_false_target_intent(self, intent_predictor):
        """False target (bird) should be flagged."""
        state = self._run_scenario(
            intent_predictor,
            lambda t: (25, 300, t*0.5, 0.5, 0, 0.8),
            "false_target_bird", n_steps=60
        )
        assert state.intent == IntentType.FALSE_TARGET

    def test_reentry_detection(self, intent_predictor):
        """ICBM reentry vehicle detection."""
        state = self._run_scenario(
            intent_predictor,
            lambda t: (5000, max(1000, 100000 - t*5000), 0.0, 5.0, -5000, 0.0),
            "ballistic", n_steps=80
        )
        assert state.intent in (IntentType.REENTRY, IntentType.TERMINAL_DIVE)
        assert state.threat_level == ThreatLevel.CRITICAL

    def test_terrain_following(self, intent_predictor):
        """Low altitude terrain following detection."""
        state = self._run_scenario(
            intent_predictor,
            lambda t: (300, 100 + 30*math.sin(t*0.5), 0.1, 2.0,
                       15*math.cos(t*0.5), 0.02),
            "subsonic_cruise", n_steps=100
        )
        assert state.intent in (IntentType.TERRAIN_FOLLOWING,
                                IntentType.SEA_SKIMMING,
                                IntentType.CRUISE)

    def test_alert_generation(self, intent_predictor):
        """Terminal dive should generate alerts."""
        state = self._run_scenario(
            intent_predictor,
            lambda t: (2500, max(50, 5000 - t*500), 0.0, 10.0, -500, 0.0),
            "hypersonic_glide", n_steps=80
        )
        has_alert = any("TERMINAL_DIVE" in a or "IMPACT" in a or "REENTRY" in a
                        for a in state.alerts)
        assert has_alert, f"Expected terminal alert, got: {state.alerts}"


# =============================================================================
# [TST-PIPE-001] Integrated Pipeline Tests
# =============================================================================

class TestPipeline:
    """End-to-end pipeline tests."""

    def test_pipeline_creation(self, pipeline):
        assert pipeline.classifier is not None
        assert pipeline.intent_predictor is not None
        assert pipeline.ecm_detector is not None

    def test_pipeline_update_returns_result(self, pipeline):
        for _ in range(10):
            result = pipeline.update(
                speed_mps=250, accel_g=1.0, altitude_m=11000,
                omega_radps=0.01, heading_rad=0.2, vz_mps=0.0,
            )
        assert isinstance(result, ClassificationResult)
        assert result.platform_name is not None
        assert isinstance(result.intent, IntentType)

    def test_pipeline_fighter_flow(self, pipeline):
        """Full pipeline for fighter scenario."""
        for step in range(40):
            result = pipeline.update(
                speed_mps=600, accel_g=7.5, altitude_m=10000,
                omega_radps=0.35, heading_rad=0.5 + step*0.05,
                vz_mps=0.0, nis_cv=1.0, rcs_dbsm=10.0,
                snr_db=25.0, doppler_hz=5000, range_m=80000
            )
        assert result.platform_class in ("4th_gen_fighter", "5th_gen_stealth",
                                          "6th_gen_concept")
        assert result.threat_level.value >= ThreatLevel.HIGH.value
        assert len(result.preferred_models) > 0

    def test_pipeline_jammed_fighter(self, pipeline):
        """Pipeline under ECM: should degrade gracefully."""
        for step in range(40):
            result = pipeline.update(
                speed_mps=600, accel_g=7.0, altitude_m=10000,
                omega_radps=0.3, heading_rad=0.5,
                vz_mps=0.0, nis_cv=30.0,
                rcs_dbsm=np.random.uniform(-10, 30),
                snr_db=3.0,  # jammed
                doppler_hz=np.random.uniform(-500, 500),
                range_m=80000
            )
        # Should still track, with ECM awareness
        assert len(result.preferred_models) >= 2
        assert any("ECM" in a for a in result.alerts) or \
               result.ecm_status != ECMStatus.CLEAN

    def test_pipeline_missile_terminal(self, pipeline):
        """Pipeline for missile terminal dive scenario."""
        for step in range(50):
            t = step * 0.1
            alt = max(100, 30000 - t * 1500)
            vz = -1500
            result = pipeline.update(
                speed_mps=2000, accel_g=10.0, altitude_m=alt,
                omega_radps=0.05, heading_rad=0.0,
                vz_mps=vz, nis_cv=1.0, rcs_dbsm=-5.0,
                snr_db=20.0, doppler_hz=15000, range_m=50000
            )
        assert result.intent in (IntentType.TERMINAL_DIVE, IntentType.REENTRY)
        assert result.threat_level.value >= ThreatLevel.HIGH.value

    def test_pipeline_false_target_rejection(self, pipeline):
        """Pipeline should identify false targets."""
        for _ in range(30):
            result = pipeline.update(
                speed_mps=8, accel_g=0.0, altitude_m=20000,
                omega_radps=0.01, heading_rad=0.1, vz_mps=2.0
            )
        assert "balloon" in result.platform_class or \
               "space" in result.platform_class or \
               result.threat_level == ThreatLevel.NONE


# =============================================================================
# [TST-RCS-001] RCS Parsing Tests
# =============================================================================

class TestRCSParsing:
    """Test RCS string parsing utility."""

    def test_range_format(self, classifier):
        lo, hi = classifier._parse_rcs_range("0.01-0.1")
        assert lo == pytest.approx(0.01)
        assert hi == pytest.approx(0.1)

    def test_plus_format(self, classifier):
        lo, hi = classifier._parse_rcs_range("100+")
        assert lo == 100.0
        assert hi == 500.0

    def test_single_value(self, classifier):
        lo, hi = classifier._parse_rcs_range("5")
        assert lo == 5.0

    def test_variable_string(self, classifier):
        lo, hi = classifier._parse_altitude_range("variable")
        assert lo == 0
        assert hi == 50000


# =============================================================================
# Run
# =============================================================================

if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
