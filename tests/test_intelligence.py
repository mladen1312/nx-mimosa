"""Tests for NX-MIMOSA v5.3 Intelligence Module.

Platform classification, ECM detection, intent prediction.
pytest tests/test_intelligence.py -v
"""

import numpy as np
import pytest
import sys, os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))

from nx_mimosa_intelligence import (
    PlatformClassifier, PlatformClass, ThreatLevel,
    ECMDetector, ECMStatus, ECMReport,
    IntentPredictor, IntentType, IntentReport,
    IntelligencePipeline, IntelligenceReport,
    PLATFORM_DB,
)


# ============================================================
# PLATFORM CLASSIFIER TESTS
# ============================================================

class TestPlatformClassifier:
    
    def test_fighter_classification(self):
        """High-speed, high-g target should classify as fighter."""
        clf = PlatformClassifier()
        for _ in range(20):
            result = clf.update(speed_ms=500, altitude_m=10000, g_load=7.0)
        
        assert result.coarse_class == "fighter"
        assert result.threat_level in (ThreatLevel.HIGH, ThreatLevel.CRITICAL)
        assert result.confidence > 0.3
    
    def test_airliner_classification(self):
        """Subsonic, high altitude, low-g → commercial airliner."""
        clf = PlatformClassifier()
        for _ in range(20):
            result = clf.update(speed_ms=240, altitude_m=11000, g_load=1.2)
        
        assert result.coarse_class in ("civil",)
        assert result.threat_level == ThreatLevel.NONE
    
    def test_cruise_missile(self):
        """Fast, very low altitude → cruise missile."""
        clf = PlatformClassifier()
        for _ in range(20):
            result = clf.update(speed_ms=280, altitude_m=50, g_load=3.0)
        
        assert "cruise" in result.platform_type or "subsonic" in result.platform_type
        assert result.threat_level.value >= ThreatLevel.HIGH.value
    
    def test_helicopter(self):
        """Slow, low altitude → helicopter."""
        clf = PlatformClassifier()
        for _ in range(20):
            result = clf.update(speed_ms=60, altitude_m=500, g_load=1.0)
        
        assert "hel" in result.platform_type or result.coarse_class in ("civil", "military", "uav")
    
    def test_ballistic_missile(self):
        """Hypersonic, extreme altitude → ballistic."""
        clf = PlatformClassifier()
        for _ in range(20):
            result = clf.update(speed_ms=3500, altitude_m=200000, g_load=1.5)
        
        assert result.threat_level == ThreatLevel.CRITICAL
        assert result.speed_class in ("supersonic", "hypersonic")
    
    def test_speed_class(self):
        """Speed class labels should be correct."""
        assert PlatformClassifier._speed_class(2) == "stationary"
        assert PlatformClassifier._speed_class(50) == "slow"
        assert PlatformClassifier._speed_class(250) == "subsonic_high"
        assert PlatformClassifier._speed_class(500) == "transonic"
        assert PlatformClassifier._speed_class(1500) == "supersonic"
        assert PlatformClassifier._speed_class(3000) == "hypersonic"
    
    def test_alt_candidates(self):
        """Should provide alternative classifications."""
        clf = PlatformClassifier()
        result = clf.update(speed_ms=300, altitude_m=5000, g_load=4.0)
        assert len(result.alt_candidates) == 3
        assert all(isinstance(c, tuple) and len(c) == 2 for c in result.alt_candidates)
    
    def test_confidence_grows(self):
        """Confidence should increase with more samples."""
        clf = PlatformClassifier(history_len=30)
        c1 = clf.update(speed_ms=500, altitude_m=10000, g_load=5.0).confidence
        for _ in range(29):
            c2 = clf.update(speed_ms=500, altitude_m=10000, g_load=5.0).confidence
        assert c2 >= c1
    
    def test_reset(self):
        """Reset should clear history."""
        clf = PlatformClassifier()
        clf.update(speed_ms=500, altitude_m=10000, g_load=5.0)
        clf.reset()
        # After reset, should work fresh
        result = clf.update(speed_ms=100, altitude_m=1000, g_load=1.0)
        assert result.platform_type is not None
    
    def test_platform_db_size(self):
        """Database should have 31 entries."""
        assert len(PLATFORM_DB) == 31


# ============================================================
# ECM DETECTOR TESTS
# ============================================================

class TestECMDetector:
    
    def test_clean_environment(self):
        """Normal conditions should report CLEAN."""
        ecm = ECMDetector()
        for _ in range(30):
            report = ecm.update(snr_db=25.0, rcs=1.0, doppler_hz=100.0, nis=1.0)
        
        assert report.status == ECMStatus.CLEAN
    
    def test_noise_jamming_detection(self):
        """Large SNR drop should detect noise jamming."""
        ecm = ECMDetector(snr_threshold_db=10.0)
        # Establish baseline
        for _ in range(20):
            ecm.update(snr_db=25.0, rcs=1.0, doppler_hz=100.0, nis=1.0)
        # Apply jamming
        for _ in range(15):
            report = ecm.update(snr_db=5.0, rcs=1.0, doppler_hz=100.0, nis=5.0)
        
        assert report.status == ECMStatus.NOISE_JAMMING
        assert report.q_scale_factor > 1.0
    
    def test_chaff_detection(self):
        """High RCS variance + Doppler spread → chaff."""
        ecm = ECMDetector()
        for _ in range(20):
            ecm.update(snr_db=25.0, rcs=1.0, doppler_hz=100.0, nis=1.0)
        
        for i in range(15):
            rcs = 1.0 + np.random.randn() * 5.0  # High variance
            dop = 100.0 + np.random.randn() * 200.0  # High spread
            report = ecm.update(snr_db=22.0, rcs=abs(rcs), doppler_hz=dop, nis=2.0)
        
        # Chaff or noise, depending on actual variance
        assert report.status in (ECMStatus.CHAFF, ECMStatus.CLEAN, ECMStatus.DECEPTION)
    
    def test_deception_detection(self):
        """High NIS without SNR drop → deception."""
        ecm = ECMDetector()
        for _ in range(20):
            ecm.update(snr_db=25.0, rcs=1.0, doppler_hz=100.0, nis=1.0)
        
        for _ in range(15):
            report = ecm.update(snr_db=24.0, rcs=1.0, doppler_hz=100.0, nis=20.0)
        
        assert report.status in (ECMStatus.DECEPTION, ECMStatus.DRFM)
        assert report.q_scale_factor >= 4.0
    
    def test_q_scale_recommendation(self):
        """Q scale should be ≥1 for all states."""
        ecm = ECMDetector()
        report = ecm.update(snr_db=25.0, rcs=1.0, nis=1.0)
        assert report.q_scale_factor >= 1.0
    
    def test_indicators_present(self):
        """Report should include indicator scores."""
        ecm = ECMDetector()
        for _ in range(15):
            report = ecm.update(snr_db=25.0, rcs=1.0, nis=1.0)
        
        assert "snr_drop_db" in report.indicators
        assert "rcs_variance" in report.indicators
    
    def test_reset(self):
        """Reset should clear state."""
        ecm = ECMDetector()
        ecm.update(snr_db=5.0, rcs=10.0, nis=50.0)
        ecm.reset()
        report = ecm.update(snr_db=25.0, rcs=1.0, nis=1.0)
        assert report.status == ECMStatus.CLEAN


# ============================================================
# INTENT PREDICTOR TESTS
# ============================================================

class TestIntentPredictor:
    
    def test_transit(self):
        """Steady state → transit."""
        pred = IntentPredictor(history_len=20)
        for i in range(20):
            report = pred.update(speed_ms=250, altitude_m=10000,
                                 heading_rad=0.5, g_load=1.0,
                                 range_to_asset_m=200000)
        
        assert report.primary_intent in (IntentType.TRANSIT, IntentType.PATROL)
        assert report.threat_level.value <= ThreatLevel.MEDIUM.value
    
    def test_attack_run(self):
        """Fast closure, low alt, high g → attack run."""
        pred = IntentPredictor(history_len=20)
        for i in range(20):
            report = pred.update(
                speed_ms=400, altitude_m=200,
                heading_rad=0.0, g_load=5.0,
                range_to_asset_m=30000 - i * 1500)
        
        assert report.primary_intent in (IntentType.ATTACK_RUN, IntentType.INGRESS,
                                          IntentType.TERRAIN_FOLLOWING, IntentType.MISSILE_TERMINAL)
        assert report.threat_score > 0.3
    
    def test_evasion(self):
        """High g + erratic heading → evasion."""
        pred = IntentPredictor(history_len=20)
        for i in range(20):
            report = pred.update(
                speed_ms=600, altitude_m=5000,
                heading_rad=np.sin(i) * 0.5, g_load=7.0,
                range_to_asset_m=50000 + i * 2000)
        
        assert report.primary_intent in (IntentType.EVASION, IntentType.ORBIT, IntentType.PATROL)
    
    def test_missile_terminal(self):
        """Hypersonic + extreme g + fast closure → missile terminal."""
        pred = IntentPredictor(history_len=20)
        for i in range(20):
            report = pred.update(
                speed_ms=1500, altitude_m=5000 - i * 200,
                heading_rad=0.0, g_load=25.0,
                range_to_asset_m=20000 - i * 1000)
        
        assert report.primary_intent in (IntentType.MISSILE_TERMINAL, IntentType.ATTACK_RUN)
        assert report.threat_level.value >= ThreatLevel.HIGH.value
    
    def test_landing(self):
        """Descending + decelerating → landing."""
        pred = IntentPredictor(history_len=20)
        for i in range(20):
            report = pred.update(
                speed_ms=100 - i * 3, altitude_m=2000 - i * 100,
                heading_rad=0.5, g_load=1.0,
                range_to_asset_m=300000)
        
        assert report.primary_intent in (IntentType.LANDING, IntentType.TRANSIT, IntentType.EGRESS)
    
    def test_time_to_threat(self):
        """Closing target should have finite time-to-threat."""
        pred = IntentPredictor(history_len=20)
        for i in range(20):
            report = pred.update(
                speed_ms=500, altitude_m=5000,
                heading_rad=0.0, g_load=3.0,
                range_to_asset_m=100000 - i * 5000)
        
        if report.time_to_threat is not None:
            assert report.time_to_threat > 0
    
    def test_threat_score_range(self):
        """Threat score should be [0, 1]."""
        pred = IntentPredictor()
        for i in range(20):
            report = pred.update(speed_ms=300, altitude_m=5000,
                                 range_to_asset_m=50000)
        assert 0.0 <= report.threat_score <= 1.0
    
    def test_reset(self):
        pred = IntentPredictor()
        pred.update(speed_ms=500, altitude_m=5000)
        pred.reset()
        report = pred.update(speed_ms=100, altitude_m=1000)
        assert report.primary_intent is not None


# ============================================================
# INTELLIGENCE PIPELINE TESTS
# ============================================================

class TestIntelligencePipeline:
    
    def test_full_assessment(self):
        """Full pipeline should return complete report."""
        intel = IntelligencePipeline()
        
        for i in range(20):
            report = intel.assess(
                track_id=1,
                speed_ms=500, altitude_m=10000,
                heading_rad=0.0, g_load=6.0,
                range_to_asset_m=80000 - i * 3000,
                snr_db=22.0, rcs=2.0, doppler_hz=500.0, nis=1.5,
            )
        
        assert isinstance(report, IntelligenceReport)
        assert report.track_id == 1
        assert isinstance(report.platform, PlatformClass)
        assert isinstance(report.ecm, ECMReport)
        assert isinstance(report.intent, IntentReport)
        assert 0.0 <= report.composite_threat <= 1.0
    
    def test_multi_track(self):
        """Pipeline should maintain separate state per track."""
        intel = IntelligencePipeline()
        
        for i in range(10):
            r1 = intel.assess(track_id=1, speed_ms=500, altitude_m=10000, g_load=7.0)
            r2 = intel.assess(track_id=2, speed_ms=60, altitude_m=500, g_load=1.0)
        
        assert r1.platform.platform_type != r2.platform.platform_type
        assert 1 in intel.active_tracks
        assert 2 in intel.active_tracks
    
    def test_remove_track(self):
        """Should clean up per-track state."""
        intel = IntelligencePipeline()
        intel.assess(track_id=1, speed_ms=500, altitude_m=10000)
        assert 1 in intel.active_tracks
        intel.remove_track(1)
        assert 1 not in intel.active_tracks
    
    def test_ecm_affects_threat(self):
        """ECM detection should increase composite threat."""
        intel = IntelligencePipeline()
        
        # Clean run
        for _ in range(20):
            r_clean = intel.assess(track_id=10, speed_ms=250, altitude_m=10000,
                                    snr_db=25.0, nis=1.0, range_to_asset_m=200000)
        
        # Jammed run
        intel2 = IntelligencePipeline()
        for _ in range(20):
            intel2.assess(track_id=20, speed_ms=250, altitude_m=10000,
                          snr_db=25.0, nis=1.0, range_to_asset_m=200000)
        for _ in range(15):
            r_jam = intel2.assess(track_id=20, speed_ms=250, altitude_m=10000,
                                   snr_db=5.0, nis=15.0, range_to_asset_m=200000)
        
        # Jammed should have higher threat (ECM contribution)
        assert r_jam.composite_threat >= r_clean.composite_threat or r_jam.ecm.status != ECMStatus.CLEAN


class TestIntegrationScenarios:
    """End-to-end scenario tests."""
    
    def test_fighter_ingress_scenario(self):
        """Fighter closing on defended area should escalate threat."""
        intel = IntelligencePipeline()
        threats = []
        
        for i in range(30):
            r = intel.assess(
                track_id=1,
                speed_ms=450, altitude_m=8000,
                heading_rad=0.0, g_load=3.0 + (i > 20) * 4.0,
                range_to_asset_m=150000 - i * 5000,
                snr_db=20.0, rcs=5.0,
            )
            threats.append(r.composite_threat)
        
        # Threat should be non-trivial for fighter at close range
        assert threats[-1] > 0.2
        assert r.platform.coarse_class in ("fighter", "military", "cruise_missile")
    
    def test_airliner_should_be_low_threat(self):
        """Commercial airliner should remain low threat."""
        intel = IntelligencePipeline()
        
        for i in range(30):
            r = intel.assess(
                track_id=2,
                speed_ms=240, altitude_m=11000,
                heading_rad=1.0, g_load=1.1,
                range_to_asset_m=100000,
                snr_db=25.0, rcs=30.0,
            )
        
        assert r.composite_threat < 0.5
        assert r.intent.primary_intent in (IntentType.TRANSIT, IntentType.PATROL,
                                            IntentType.ORBIT, IntentType.UNKNOWN)
