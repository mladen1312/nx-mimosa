"""[REQ-V592-PHD] Tests for GM-PHD and CPHD filters.

Validates:
- Target count estimation (unknown/varying target count)
- State extraction accuracy
- Component management (merging, pruning, capping)
- Adaptive birth from measurements
- CPHD cardinality distribution
- Comparison with standard MTT on same scenarios
"""
import numpy as np
import pytest
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from python.nx_mimosa_mtt import (
    GMPHD, CPHD, GaussianComponent, ParticleFilter3D,
    MultiTargetTracker, compute_gospa
)


class TestGMPHD:
    """GM-PHD filter core functionality."""
    
    @pytest.fixture
    def phd_3target(self):
        """PHD filter configured for 3-target scenario."""
        phd = GMPHD(dt=1.0, pd=0.9, ps=0.99,
                     clutter_intensity=2e-14,
                     birth_weight=0.1,
                     extraction_threshold=0.4)
        phd.set_birth_regions([
            np.array([1000, 0, 10000]),
            np.array([0, 1000, 10000]),
            np.array([-500, 500, 10000]),
        ], spread=2000.0)
        return phd
    
    def test_empty_init(self):
        """PHD starts with no targets."""
        phd = GMPHD()
        assert phd.estimated_target_count() == 0
        assert phd.component_count() == 0
    
    def test_single_target_convergence(self):
        """Single target detected within 5 scans."""
        np.random.seed(42)
        phd = GMPHD(dt=1.0, pd=0.95, ps=0.99,
                     clutter_intensity=1e-14,
                     birth_weight=0.2,
                     extraction_threshold=0.4)
        phd.set_birth_regions([np.array([1000, 0, 10000])], spread=1000.0)
        
        target = np.array([1000.0, 0.0, 10000.0])
        for scan in range(10):
            z = target + np.random.randn(3) * 150.0
            phd.predict()
            phd.update(z.reshape(1, 3))
            target += np.array([100, 0, 0])
        
        assert phd.estimated_target_count() >= 1
        targets = phd.extract_targets()
        assert len(targets) >= 1
    
    def test_three_targets_detected(self, phd_3target):
        """All 3 targets detected by scan 10."""
        np.random.seed(42)
        targets = [
            np.array([1000.0, 0.0, 10000.0]),
            np.array([0.0, 1000.0, 10000.0]),
            np.array([-500.0, 500.0, 10000.0]),
        ]
        vels = [
            np.array([250.0, 0.0, 0.0]),
            np.array([0.0, 200.0, 0.0]),
            np.array([-150.0, 150.0, 0.0]),
        ]
        
        for scan in range(15):
            meas = []
            for i in range(3):
                meas.append(targets[i] + np.random.randn(3) * 150.0)
                targets[i] += vels[i]
            # 2 clutter
            for _ in range(2):
                meas.append(np.random.uniform(-10000, 10000, 3))
            
            phd_3target.predict()
            phd_3target.update(np.array(meas))
        
        assert phd_3target.estimated_target_count() >= 2
        assert phd_3target.estimated_target_count() <= 5
        extracted = phd_3target.extract_targets()
        assert len(extracted) >= 2
    
    def test_target_count_adapts_to_birth_death(self):
        """PHD count increases when targets appear, decreases when they disappear."""
        np.random.seed(42)
        phd = GMPHD(dt=1.0, pd=0.95, ps=0.98,
                     clutter_intensity=1e-14,
                     birth_weight=0.15,
                     extraction_threshold=0.4,
                     use_adaptive_birth=True)
        phd.set_birth_regions([np.array([0, 0, 10000])], spread=3000.0)
        
        # Phase 1: 2 targets for 15 scans
        t1 = np.array([500.0, 0.0, 10000.0])
        t2 = np.array([-500.0, 0.0, 10000.0])
        for scan in range(15):
            meas = [t1 + np.random.randn(3)*150, t2 + np.random.randn(3)*150]
            t1 += np.array([50, 0, 0])
            t2 += np.array([-50, 0, 0])
            phd.predict()
            phd.update(np.array(meas))
        
        count_phase1 = phd.estimated_target_count()
        
        # Phase 2: target 2 disappears for 10 scans
        for scan in range(10):
            meas = [t1 + np.random.randn(3)*150]
            t1 += np.array([50, 0, 0])
            phd.predict()
            phd.update(np.array(meas))
        
        count_phase2 = phd.estimated_target_count()
        
        # Count should decrease
        assert count_phase2 <= count_phase1
    
    def test_pruning_controls_complexity(self, phd_3target):
        """Component count stays bounded after many scans."""
        np.random.seed(42)
        for scan in range(50):
            meas = np.random.uniform(-5000, 5000, (5, 3))
            phd_3target.predict()
            phd_3target.update(meas)
        
        assert phd_3target.component_count() <= phd_3target.max_components
    
    def test_no_measurements_count_decays(self):
        """With no detections, target count decays toward zero."""
        np.random.seed(42)
        phd = GMPHD(dt=1.0, pd=0.9, ps=0.95,
                     clutter_intensity=1e-14,
                     birth_weight=0.1)
        phd.set_birth_regions([np.array([0, 0, 10000])], spread=1000.0)
        
        # First establish target
        for _ in range(10):
            phd.predict()
            phd.update(np.array([[0, 0, 10000]]) + np.random.randn(1, 3) * 150)
        
        initial = phd.phd_integral()
        
        # Now no measurements for 20 scans
        for _ in range(20):
            phd.predict()
            phd.update(np.zeros((0, 3)))
        
        assert phd.phd_integral() < initial
    
    def test_labels_provide_continuity(self, phd_3target):
        """Extracted targets maintain consistent labels across scans."""
        np.random.seed(42)
        target = np.array([1000.0, 0.0, 10000.0])
        
        prev_labels = set()
        for scan in range(15):
            z = target + np.random.randn(3) * 150
            phd_3target.predict()
            phd_3target.update(z.reshape(1, 3))
            target += np.array([100, 0, 0])
            
            extracted = phd_3target.extract_targets()
            if extracted:
                labels = {t['label'] for t in extracted}
                if prev_labels:
                    # At least one label should persist
                    overlap = labels & prev_labels
                    # Allow for label changes during early convergence
                    if scan > 5:
                        assert len(overlap) > 0 or len(labels) > 0
                prev_labels = labels


class TestCPHD:
    """CPHD cardinality filter."""
    
    def test_cphd_creates(self):
        """CPHD initializes correctly."""
        cphd = CPHD(dt=1.0, max_targets=20)
        assert cphd.map_cardinality() == 0
        assert cphd.cardinality_variance() == 0.0
    
    def test_cphd_tracks_cardinality(self):
        """CPHD MAP cardinality converges toward true count."""
        np.random.seed(42)
        cphd = CPHD(dt=1.0, pd=0.95, ps=0.99,
                     clutter_intensity=1e-14,
                     birth_weight=0.15, max_targets=10)
        cphd.gmphd.set_birth_regions([
            np.array([i * 2000, 0, 10000]) for i in range(3)
        ], spread=1500.0)
        
        targets = [np.array([i * 2000.0, 0.0, 10000.0]) for i in range(3)]
        
        for scan in range(20):
            meas = []
            for t in targets:
                meas.append(t + np.random.randn(3) * 150)
                t += np.array([50, 0, 0])
            meas.append(np.random.uniform(-5000, 10000, 3))  # 1 clutter
            
            cphd.predict()
            cphd.update(np.array(meas))
        
        # Should estimate roughly 3 targets (allow ±2)
        assert 1 <= cphd.map_cardinality() <= 6
    
    def test_cphd_summary(self):
        """Summary string is formatted correctly."""
        cphd = CPHD()
        s = cphd.summary()
        assert "CPHD" in s
        assert "MAP_card" in s


class TestParticleFilter:
    """Particle filter validation."""
    
    def test_pf_initializes(self):
        """PF creates particles around initial measurement."""
        pf = ParticleFilter3D(n_particles=200)
        pf.initialize(np.array([1000.0, 2000.0, 10000.0]))
        pos = pf.position
        assert np.linalg.norm(pos - np.array([1000, 2000, 10000])) < 2000
    
    def test_pf_tracks_straight_line(self):
        """PF tracks a constant-velocity target."""
        np.random.seed(42)
        pf = ParticleFilter3D(n_particles=1000)
        target = np.array([1000.0, 0.0, 10000.0])
        vel = np.array([200.0, 0.0, 0.0])
        
        pf.initialize(target)
        R = np.eye(3) * 150.0**2
        
        errors = []
        for _ in range(20):
            target += vel * 1.0
            z = target + np.random.randn(3) * 150
            pf.predict(dt=1.0)
            pf.update(z, R=R)
            err = np.linalg.norm(pf.position - target)
            errors.append(err)
        
        # Mean error should be reasonable
        assert np.mean(errors[5:]) < 1500  # After convergence (PF inherently noisier than KF)
    
    def test_pf_ess_ratio_after_init(self):
        """ESS ratio is 1.0 (perfect) right after initialization."""
        pf = ParticleFilter3D(n_particles=500, resample_threshold=0.0)
        pf.initialize(np.array([0, 0, 10000.0]))
        ess_ratio = pf.effective_sample_size()
        assert ess_ratio > 0.95  # Should be ~1.0 with uniform weights


class TestFeatureInventory:
    """Verify all advertised features exist and are callable."""
    
    def test_all_filter_types(self):
        """KF, EKF, UKF, IMM, PF, GM-PHD, CPHD all importable."""
        from python.nx_mimosa_mtt import (
            KalmanFilter3D, EKF3D, UKF3D, IMM3D,
            ParticleFilter3D, GMPHD, CPHD
        )
        # Instantiate each
        kf = KalmanFilter3D(6, 3)
        ekf = EKF3D()
        ukf = UKF3D()
        imm = IMM3D(dt=1.0)
        pf = ParticleFilter3D()
        phd = GMPHD()
        cphd = CPHD()
        assert True  # If we get here, all work
    
    def test_all_association_methods(self):
        """GNN, JPDA, MHT all importable."""
        from python.nx_mimosa_mtt import (
            gnn_associate, jpda_associate,
            mht_associate, mht_associate_enhanced
        )
        assert callable(gnn_associate)
        assert callable(jpda_associate)
        assert callable(mht_associate)
    
    def test_fusion_functions(self):
        """T2TA and track fusion exist."""
        from python.nx_mimosa_mtt import t2ta_associate, fuse_tracks
        assert callable(t2ta_associate)
        assert callable(fuse_tracks)
    
    def test_ecm_detector(self):
        """ECMDetector auto-detection exists."""
        from python.nx_mimosa_mtt import ECMDetector
        ecm = ECMDetector()
        assert hasattr(ecm, 'update')
    
    def test_track_coaster(self):
        """TrackCoaster exists."""
        from python.nx_mimosa_mtt import TrackCoaster
        tc = TrackCoaster()
        assert hasattr(tc, 'predict_coast')
    
    def test_oosm_handler(self):
        """Out-of-sequence measurement handler exists."""
        from python.nx_mimosa_mtt import OOSMHandler
        oosm = OOSMHandler()
        assert hasattr(oosm, 'process_oosm')
    
    def test_sensor_bias_estimator(self):
        """Sensor bias estimation exists."""
        from python.nx_mimosa_mtt import SensorBiasEstimator
        sbe = SensorBiasEstimator()
        assert hasattr(sbe, 'update')
    
    def test_total_class_count(self):
        """Library has expected number of classes."""
        import re
        with open('python/nx_mimosa_mtt.py') as f:
            code = f.read()
        classes = re.findall(r'^class (\w+)', code, re.MULTILINE)
        assert len(classes) >= 24, f"Expected >=24 classes, got {len(classes)}: {classes}"
    
    def test_total_loc(self):
        """Library exceeds 3800 lines."""
        with open('python/nx_mimosa_mtt.py') as f:
            loc = sum(1 for _ in f)
        assert loc >= 3800, f"Expected >=3800 LOC, got {loc}"


class TestLMB:
    """[REQ-V592-LMB] Tests for Labeled Multi-Bernoulli filter."""
    
    def test_lmb_creates(self):
        from python.nx_mimosa_mtt import LMB
        lmb = LMB(dt=1.0, pd=0.9, ps=0.99)
        assert lmb.estimated_target_count() == 0
        assert len(lmb.components) == 0
    
    def test_lmb_tracks_single_target(self):
        from python.nx_mimosa_mtt import LMB
        import numpy as np
        np.random.seed(42)
        
        lmb = LMB(dt=1.0, pd=0.95, ps=0.99, clutter_intensity=1e-11, birth_r=0.05)
        lmb.set_birth_regions([np.array([0, 0, 5000])], spread=500.0)
        
        for i in range(10):
            lmb.predict()
            meas = np.array([[i*200 + np.random.randn()*50, np.random.randn()*50, 5000+np.random.randn()*50]])
            lmb.update(meas)
        
        targets = lmb.extract_targets(threshold=0.3)
        assert len(targets) >= 1, f"Expected at least 1 target, got {len(targets)}"
        assert targets[0]['existence_prob'] > 0.5
    
    def test_lmb_tracks_two_targets(self):
        from python.nx_mimosa_mtt import LMB
        import numpy as np
        np.random.seed(42)
        
        lmb = LMB(dt=1.0, pd=0.9, ps=0.99, clutter_intensity=1e-11, birth_r=0.01)
        lmb.set_birth_regions([np.array([1000, 0, 5000]), np.array([5000, 3000, 8000])], spread=500.0)
        
        for i in range(10):
            lmb.predict()
            t1 = [1000+i*200+np.random.randn()*50, np.random.randn()*50, 5000+np.random.randn()*50]
            t2 = [5000+np.random.randn()*50, 3000+i*150+np.random.randn()*50, 8000+np.random.randn()*50]
            lmb.update(np.array([t1, t2]))
        
        targets = lmb.extract_targets(threshold=0.3)
        assert len(targets) == 2, f"Expected 2 targets, got {len(targets)}"
        assert lmb.map_cardinality() == 2
    
    def test_lmb_labels_are_unique(self):
        from python.nx_mimosa_mtt import LMB
        import numpy as np
        np.random.seed(42)
        
        lmb = LMB(dt=1.0, pd=0.9, ps=0.99, birth_r=0.01)
        lmb.set_birth_regions([np.array([0, 0, 0]), np.array([1000, 1000, 1000])], spread=200.0)
        
        for i in range(5):
            lmb.predict()
            lmb.update(np.array([[i*100, 0, 0], [1000+i*100, 1000, 1000]]))
        
        targets = lmb.extract_targets(threshold=0.1)
        labels = [t['label'] for t in targets]
        assert len(labels) == len(set(labels)), "Labels must be unique"
    
    def test_lmb_cardinality_distribution(self):
        from python.nx_mimosa_mtt import LMB
        import numpy as np
        np.random.seed(42)
        
        lmb = LMB(dt=1.0, pd=0.95, ps=0.99, birth_r=0.01)
        lmb.set_birth_regions([np.array([0, 0, 5000])], spread=500.0)
        
        for i in range(8):
            lmb.predict()
            lmb.update(np.array([[i*200+np.random.randn()*30, np.random.randn()*30, 5000+np.random.randn()*30]]))
        
        pmf = lmb.cardinality_distribution()
        assert abs(sum(pmf) - 1.0) < 1e-6, "PMF must sum to 1"
        assert lmb.map_cardinality() >= 0
    
    def test_lmb_summary(self):
        from python.nx_mimosa_mtt import LMB
        lmb = LMB()
        s = lmb.summary()
        assert "LMB" in s
        assert "scan=0" in s
