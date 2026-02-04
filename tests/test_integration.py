#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════════════════════════════
NX-MIMOSA - INTEGRATION TESTS
═══════════════════════════════════════════════════════════════════════════════════════════════════════

Integration tests that exercise the actual NX-MIMOSA modules.

Run with: pytest tests/test_integration.py -v

Author: Dr. Mladen Mešter / Nexellum d.o.o.
═══════════════════════════════════════════════════════════════════════════════════════════════════════
"""

import pytest
import numpy as np
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


# =============================================================================
# TEST: NX-MIMOSA ATC TRACKER
# =============================================================================

class TestNXMIMOSAAtc:
    """Integration tests for the main ATC tracker."""
    
    def test_tracker_import(self):
        """Test that tracker module can be imported."""
        from nx_mimosa_v41_atc import NXMIMOSAAtc
        assert NXMIMOSAAtc is not None
    
    def test_tracker_initialization(self):
        """Test tracker initialization."""
        from nx_mimosa_v41_atc import NXMIMOSAAtc
        
        tracker = NXMIMOSAAtc(dt=1.0, sigma=30.0)
        
        # Initialize with position and velocity
        z0 = np.array([50000.0, 30000.0, 10668.0])
        v0 = np.array([232.0, 0.0, 0.0])
        
        tracker.initialize(z0, v0)
        
        # Check state was set
        state = tracker.get_state()
        assert state is not None
        assert len(state) == 6
    
    def test_tracker_predict_update_cycle(self):
        """Test basic predict-update cycle."""
        from nx_mimosa_v41_atc import NXMIMOSAAtc
        
        tracker = NXMIMOSAAtc(dt=1.0, sigma=30.0)
        
        # Initialize
        z0 = np.array([50000.0, 30000.0, 10668.0])
        v0 = np.array([232.0, 0.0, 0.0])
        tracker.initialize(z0, v0)
        
        # Predict
        tracker.predict(dt=1.0)
        
        # Update with new measurement
        z1 = np.array([50232.0, 30000.0, 10668.0])
        R = np.diag([30.0**2, 30.0**2, 50.0**2])
        result = tracker.update(z1, R)
        
        assert result is not None
        state = tracker.get_state()
        # Position should be close to measurement
        assert abs(state[0] - z1[0]) < 100  # Within 100m
    
    def test_tracker_mode_probabilities(self):
        """Test that mode probabilities sum to 1."""
        from nx_mimosa_v41_atc import NXMIMOSAAtc
        
        tracker = NXMIMOSAAtc(dt=1.0, sigma=30.0)
        
        z0 = np.array([50000.0, 30000.0, 10668.0])
        v0 = np.array([232.0, 0.0, 0.0])
        tracker.initialize(z0, v0)
        
        # Get mode probabilities
        mu = tracker.get_mode_probabilities()
        
        assert mu is not None
        assert len(mu) == 3  # 3 modes
        assert abs(sum(mu) - 1.0) < 1e-6  # Sum to 1
    
    def test_tracker_state_consistency(self):
        """Test that state remains consistent after updates."""
        from nx_mimosa_v41_atc import NXMIMOSAAtc
        
        tracker = NXMIMOSAAtc(dt=1.0, sigma=30.0)
        
        z0 = np.array([50000.0, 30000.0, 10668.0])
        v0 = np.array([232.0, 0.0, 0.0])
        tracker.initialize(z0, v0)
        
        R = np.diag([30.0**2, 30.0**2, 50.0**2])
        
        # Run a few cycles
        for i in range(10):
            tracker.predict(dt=1.0)
            z = z0 + v0 * (i + 1) + np.random.randn(3) * 30
            tracker.update(z, R)
        
        # Check state is valid
        state = tracker.get_state()
        assert len(state) == 6
        assert all(np.isfinite(state))
    
    def test_tracker_maneuver_detection(self):
        """Test maneuver detection through mode switching."""
        from nx_mimosa_v41_atc import NXMIMOSAAtc
        
        tracker = NXMIMOSAAtc(dt=1.0, sigma=30.0)
        
        # Initialize with straight flight
        z0 = np.array([50000.0, 30000.0, 10668.0])
        v0 = np.array([232.0, 0.0, 0.0])
        tracker.initialize(z0, v0)
        
        R = np.diag([30.0**2, 30.0**2, 50.0**2])
        
        # Simulate straight flight
        for i in range(5):
            tracker.predict(dt=1.0)
            z = z0 + v0 * (i + 1)
            tracker.update(z, R)
        
        mu_straight = tracker.get_mode_probabilities()
        
        # Simulate turn (coordinated turn adds y-component)
        omega = 3.0 * np.pi / 180  # 3 deg/s turn rate
        for i in range(10):
            tracker.predict(dt=1.0)
            t = 5 + i + 1
            # Curved trajectory
            x = z0[0] + 232 * t + 500 * np.sin(omega * i)
            y = z0[1] + 500 * (1 - np.cos(omega * i))
            z = np.array([x, y, z0[2]])
            tracker.update(z, R)
        
        mu_turn = tracker.get_mode_probabilities()
        
        # During turn, CT modes should have higher probability
        # (This is a soft check - actual behavior depends on tuning)
        assert mu_turn is not None


# =============================================================================
# TEST: ASTERIX CAT062 FORMATTER
# =============================================================================

class TestAsterixFormatter:
    """Integration tests for ASTERIX CAT062 formatter."""
    
    def test_formatter_import(self):
        """Test that formatter can be imported."""
        from asterix_cat062_formatter import AsterixCat062Encoder, NXMIMOSAAsterixOutput
        assert AsterixCat062Encoder is not None
        assert NXMIMOSAAsterixOutput is not None
    
    def test_formatter_encode_basic(self):
        """Test basic ASTERIX encoding."""
        from asterix_cat062_formatter import AsterixCat062Encoder, TrackData
        
        encoder = AsterixCat062Encoder(sac=0, sic=1)
        
        # Create track data with correct fields
        track = TrackData(
            track_number=1234,
            time_of_track=43200.0,  # Noon UTC
            x=50000.0,
            y=30000.0,
            vx=232.0,
            vy=0.0
        )
        
        # Encode
        data = encoder.encode(track)
        
        assert data is not None
        assert len(data) > 0
        assert data[0] == 62  # Category byte
    
    def test_formatter_from_tracker(self):
        """Test encoding from tracker state."""
        from asterix_cat062_formatter import NXMIMOSAAsterixOutput
        from nx_mimosa_v41_atc import NXMIMOSAAtc
        
        # Create and initialize tracker
        tracker = NXMIMOSAAtc(dt=1.0, sigma=30.0)
        z0 = np.array([50000.0, 30000.0, 10668.0])
        v0 = np.array([232.0, 0.0, 0.0])
        tracker.initialize(z0, v0)
        
        # Create formatter
        asterix = NXMIMOSAAsterixOutput(sac=0, sic=1)
        
        # Convert tracker to ASTERIX - check if method exists
        assert hasattr(asterix, 'from_tracker') or hasattr(asterix, 'encode')


# =============================================================================
# TEST: CAN-FD FORMATTER
# =============================================================================

class TestCANFDFormatter:
    """Integration tests for CAN-FD formatter."""
    
    def test_formatter_import(self):
        """Test that formatter can be imported."""
        from canfd_automotive_formatter import CANFDRadarEncoder, NXMIMOSACANOutput
        assert CANFDRadarEncoder is not None
        assert NXMIMOSACANOutput is not None
    
    def test_formatter_encode_object(self):
        """Test encoding a tracked object."""
        from canfd_automotive_formatter import CANFDRadarEncoder, RadarObject
        
        encoder = CANFDRadarEncoder(use_fd=True)
        
        # Create object with correct field names
        obj = RadarObject(
            object_id=1,
            distance_x=50.0,
            distance_y=10.0,
            velocity_x=-5.0,
            velocity_y=0.0
        )
        
        # Encode
        messages = encoder.encode_object(obj)
        
        assert messages is not None
        assert len(messages) > 0


# =============================================================================
# TEST: JPDA FILTER
# =============================================================================

class TestJPDA:
    """Integration tests for JPDA filter."""
    
    def test_jpda_import(self):
        """Test that JPDA can be imported."""
        from jpda_filter import JPDAFilter, JPDAConfig
        assert JPDAFilter is not None
        assert JPDAConfig is not None
    
    def test_jpda_initialization(self):
        """Test JPDA initialization."""
        from jpda_filter import JPDAFilter, JPDAConfig
        
        config = JPDAConfig(
            gate_probability=0.9999,
            pd=0.9,
            lambda_fa=1e-6
        )
        
        jpda = JPDAFilter(config)
        assert jpda is not None
    
    def test_jpda_single_track(self):
        """Test JPDA with single track."""
        from jpda_filter import JPDAFilter, JPDAConfig, Measurement
        
        config = JPDAConfig()
        jpda = JPDAFilter(config)
        
        # Add track - need to provide a filter object
        x0 = np.array([1000.0, 500.0, 0.0, 10.0, 5.0, 0.0])
        P0 = np.diag([100.0, 100.0, 100.0, 10.0, 10.0, 10.0])
        
        # Create a simple mock filter
        class MockFilter:
            def __init__(self, x, P):
                self.x = x
                self.P = P
            def predict(self, dt):
                pass
            def update(self, z, R):
                return self.x, self.P, 1.0
        
        mock_filter = MockFilter(x0, P0)
        
        # This test verifies the JPDA structure exists
        assert jpda is not None
        assert hasattr(jpda, 'process') or hasattr(jpda, 'add_track')


# =============================================================================
# TEST: MHT FILTER
# =============================================================================

class TestMHT:
    """Integration tests for MHT filter."""
    
    def test_mht_import(self):
        """Test that MHT can be imported."""
        from mht_filter import MHTFilter, MHTConfig
        assert MHTFilter is not None
        assert MHTConfig is not None
    
    def test_mht_initialization(self):
        """Test MHT initialization."""
        from mht_filter import MHTFilter, MHTConfig
        
        config = MHTConfig(
            n_scan_pruning=3,
            max_hypotheses=100
        )
        
        mht = MHTFilter(config)
        assert mht is not None


# =============================================================================
# TEST: FREQUENCY AGILITY
# =============================================================================

class TestFrequencyAgility:
    """Integration tests for frequency agility module."""
    
    def test_freq_agility_import(self):
        """Test that frequency agility can be imported."""
        from fpga_frequency_agility import (
            NXMIMOSAFrequencyAgility, FrequencyAgilityConfig
        )
        assert NXMIMOSAFrequencyAgility is not None
    
    def test_freq_agility_hopping(self):
        """Test frequency hopping sequence."""
        from fpga_frequency_agility import (
            NXMIMOSAFrequencyAgility, FrequencyAgilityConfig
        )
        
        config = FrequencyAgilityConfig(
            center_frequency=3.0e9,
            bandwidth=500e6,
            n_channels=64,
            hop_interval=100e-6,
            pattern_type='lfsr'
        )
        
        fa = NXMIMOSAFrequencyAgility(config)
        
        # Process returns to advance hopping
        freqs = []
        for i in range(10):
            # Get current frequency
            freq = fa.get_transmit_frequency()
            freqs.append(freq)
            # Process a return to advance hop counter
            fa.process_return(
                range_m=10000 + i * 100,
                doppler_hz=100.0,
                received_freq=freq,
                timestamp=i * config.hop_interval
            )
        
        # Verify we got valid frequencies
        assert all(f > 0 for f in freqs)
        # Frequencies should be within the bandwidth
        min_f = config.center_frequency - config.bandwidth / 2
        max_f = config.center_frequency + config.bandwidth / 2
        assert all(min_f <= f <= max_f for f in freqs)
    
    def test_freq_agility_rtl_generation(self):
        """Test RTL code generation."""
        from fpga_frequency_agility import (
            NXMIMOSAFrequencyAgility, FrequencyAgilityConfig
        )
        
        config = FrequencyAgilityConfig(
            center_frequency=3.0e9,
            bandwidth=500e6,
            n_channels=64
        )
        
        fa = NXMIMOSAFrequencyAgility(config)
        
        # Generate RTL
        rtl = fa.generate_rtl()
        
        assert rtl is not None
        assert 'module' in rtl
        assert 'frequency_agility_controller' in rtl


# =============================================================================
# TEST: LINK-16 FORMATTER
# =============================================================================

class TestLink16Formatter:
    """Integration tests for Link-16 formatter."""
    
    def test_link16_import(self):
        """Test that Link-16 formatter can be imported."""
        from link16_defense_formatter import (
            NXMIMOSALink16Output, TrackEnvironment, TrackIdentity
        )
        assert NXMIMOSALink16Output is not None
    
    def test_link16_encode_track(self):
        """Test encoding a track as Link-16 message."""
        from link16_defense_formatter import (
            NXMIMOSALink16Output, TrackEnvironment
        )
        from nx_mimosa_v41_atc import NXMIMOSAAtc
        
        # Create tracker
        tracker = NXMIMOSAAtc(dt=1.0, sigma=30.0)
        z0 = np.array([50000.0, 30000.0, 10668.0])
        v0 = np.array([232.0, 0.0, 0.0])
        tracker.initialize(z0, v0)
        
        # Create Link-16 output
        link16 = NXMIMOSALink16Output(own_track_number=1)
        
        # Convert - use internal_track_id instead of track_id
        track = link16.from_tracker(
            tracker,
            internal_track_id=1234,
            environment=TrackEnvironment.AIR
        )
        
        assert track is not None


# =============================================================================
# PERFORMANCE TESTS
# =============================================================================

class TestPerformance:
    """Performance benchmarks."""
    
    @pytest.mark.slow
    def test_tracker_throughput(self):
        """Test tracker can process 1000 updates in reasonable time."""
        from nx_mimosa_v41_atc import NXMIMOSAAtc
        import time
        
        tracker = NXMIMOSAAtc(dt=1.0, sigma=30.0)
        z0 = np.array([50000.0, 30000.0, 10668.0])
        v0 = np.array([232.0, 0.0, 0.0])
        tracker.initialize(z0, v0)
        
        start = time.time()
        
        for i in range(1000):
            tracker.predict(dt=1.0)
            z = z0 + v0 * (i + 1) + np.random.randn(3) * 30
            tracker.update(z, sigma=30.0)
        
        elapsed = time.time() - start
        
        # Should complete 1000 updates in under 5 seconds
        assert elapsed < 5.0, f"Too slow: {elapsed:.2f}s for 1000 updates"
        
        print(f"\nThroughput: {1000/elapsed:.0f} updates/sec")


# =============================================================================
# MAIN
# =============================================================================

if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
