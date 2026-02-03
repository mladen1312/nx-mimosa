#!/usr/bin/env python3
"""
QEDMMA PoC - PlutoSDR VHF Radar Application
"Garažni Pobunjenik" v3.4

Author: Dr. Mladen Mešter
Copyright (c) 2026 - All Rights Reserved

Requirements:
    pip install pyadi-iio numpy scipy matplotlib

Usage:
    python3 pluto_radar.py --mode loopback   # Self-test
    python3 pluto_radar.py --mode monostatic # Single antenna
    python3 pluto_radar.py --mode bistatic   # Tx/Rx separated
"""

import numpy as np
from scipy import signal
import time
import argparse
import sys

try:
    import adi
    PLUTO_AVAILABLE = True
except ImportError:
    PLUTO_AVAILABLE = False
    print("Warning: pyadi-iio not installed. Running in simulation mode.")

#=============================================================================
# Configuration
#=============================================================================

class RadarConfig:
    """Radar system configuration"""
    
    # RF parameters
    CENTER_FREQ = 155e6      # 155 MHz (VHF)
    SAMPLE_RATE = 4e6        # 4 MSPS
    BANDWIDTH = 2e6          # 2 MHz
    TX_GAIN = -10            # dB (Pluto internal, before PA)
    RX_GAIN = 40             # dB
    
    # PRBS parameters
    PRBS_ORDER = 15          # PRBS-15 (32767 chips)
    CHIP_RATE = 1e6          # 1 Mchip/s
    
    # Processing parameters
    NUM_RANGE_BINS = 512
    CPI_LENGTH = 32768       # Samples per CPI
    NUM_CPIS = 100           # For averaging
    
    # Derived parameters
    CHIPS_PER_SAMPLE = CHIP_RATE / SAMPLE_RATE
    RANGE_RESOLUTION = 3e8 / (2 * CHIP_RATE)  # meters
    MAX_RANGE = RANGE_RESOLUTION * NUM_RANGE_BINS
    
    @classmethod
    def print_config(cls):
        print("=" * 60)
        print("QEDMMA PoC Radar Configuration")
        print("=" * 60)
        print(f"Center Frequency:   {cls.CENTER_FREQ/1e6:.1f} MHz")
        print(f"Sample Rate:        {cls.SAMPLE_RATE/1e6:.1f} MSPS")
        print(f"PRBS Order:         {cls.PRBS_ORDER}")
        print(f"Chip Rate:          {cls.CHIP_RATE/1e6:.1f} Mchip/s")
        print(f"Range Resolution:   {cls.RANGE_RESOLUTION:.1f} m")
        print(f"Max Range:          {cls.MAX_RANGE/1000:.1f} km")
        print(f"Processing Gain:    {10*np.log10(2**cls.PRBS_ORDER - 1):.1f} dB")
        print("=" * 60)

#=============================================================================
# PRBS Generator
#=============================================================================

class PRBSGenerator:
    """PRBS-N sequence generator using LFSR"""
    
    # LFSR taps for different PRBS orders (maximal length)
    TAPS = {
        7:  [7, 6],
        9:  [9, 5],
        11: [11, 9],
        15: [15, 14],
        20: [20, 17],
        23: [23, 18],
    }
    
    def __init__(self, order=15, seed=None):
        """
        Initialize PRBS generator
        
        Args:
            order: PRBS order (7, 9, 11, 15, 20, 23)
            seed: Initial LFSR state (default: all ones)
        """
        if order not in self.TAPS:
            raise ValueError(f"PRBS order {order} not supported")
        
        self.order = order
        self.taps = self.TAPS[order]
        self.length = 2**order - 1
        
        if seed is None:
            self.state = (1 << order) - 1  # All ones
        else:
            self.state = seed & ((1 << order) - 1)
            if self.state == 0:
                self.state = 1  # Avoid all-zeros lock
    
    def next_bit(self):
        """Generate next PRBS bit"""
        # Calculate feedback
        feedback = 0
        for tap in self.taps:
            feedback ^= (self.state >> (tap - 1)) & 1
        
        # Output bit (LSB)
        output = self.state & 1
        
        # Shift register
        self.state = ((self.state >> 1) | (feedback << (self.order - 1)))
        
        return output
    
    def generate_sequence(self, length=None):
        """Generate PRBS sequence as numpy array"""
        if length is None:
            length = self.length
        
        # Reset state
        self.state = (1 << self.order) - 1
        
        # Generate bits
        bits = np.zeros(length, dtype=np.int8)
        for i in range(length):
            bits[i] = self.next_bit()
        
        return bits
    
    def generate_bpsk_waveform(self, samples_per_chip=4, amplitude=0.8):
        """
        Generate BPSK modulated PRBS waveform
        
        Args:
            samples_per_chip: Oversampling factor
            amplitude: Signal amplitude (0-1)
            
        Returns:
            Complex IQ samples
        """
        # Generate PRBS bits
        bits = self.generate_sequence()
        
        # Convert to BPSK symbols (+1, -1)
        symbols = 2 * bits - 1
        
        # Upsample
        waveform = np.repeat(symbols, samples_per_chip)
        
        # Apply amplitude
        waveform = amplitude * waveform.astype(np.complex64)
        
        return waveform

#=============================================================================
# Zero-DSP Correlator
#=============================================================================

class ZeroDSPCorrelator:
    """
    Zero-DSP PRBS Correlator
    
    Uses conditional sign inversion instead of multiplication:
        prbs_bit = 1 → accumulator += sample
        prbs_bit = 0 → accumulator -= sample
    
    This is mathematically equivalent to:
        accumulator += sample * (2*prbs_bit - 1)
    
    But requires NO DSP multipliers!
    """
    
    def __init__(self, prbs_order=15, num_lanes=512):
        """
        Initialize correlator
        
        Args:
            prbs_order: PRBS sequence order
            num_lanes: Number of parallel correlation lanes (range bins)
        """
        self.prbs_order = prbs_order
        self.num_lanes = num_lanes
        self.prbs_length = 2**prbs_order - 1
        
        # Generate reference PRBS sequence
        gen = PRBSGenerator(order=prbs_order)
        self.prbs_ref = gen.generate_sequence()
        
        # Pre-compute BPSK symbols for reference
        self.prbs_bpsk = 2 * self.prbs_ref - 1  # +1 or -1
        
        print(f"[Correlator] Initialized: PRBS-{prbs_order}, {num_lanes} lanes")
        print(f"[Correlator] Processing gain: {10*np.log10(self.prbs_length):.1f} dB")
    
    def correlate_zero_dsp(self, rx_samples):
        """
        Perform zero-DSP correlation
        
        This simulates the FPGA implementation using conditional
        sign inversion instead of DSP multiplication.
        
        Args:
            rx_samples: Complex received samples
            
        Returns:
            Range profile (correlation magnitude vs delay)
        """
        # Extract real part for correlation (or use magnitude)
        samples = np.real(rx_samples)
        n_samples = len(samples)
        
        # Ensure we have enough samples
        if n_samples < self.prbs_length:
            samples = np.pad(samples, (0, self.prbs_length - n_samples))
            n_samples = self.prbs_length
        
        # Create delay line (shift register simulation)
        # Each lane correlates with a different delay
        accumulators = np.zeros(self.num_lanes, dtype=np.float64)
        
        # Process sample by sample (simulating streaming FPGA)
        prbs_idx = 0
        
        for sample_idx in range(min(n_samples, self.prbs_length)):
            sample = samples[sample_idx]
            prbs_bit = self.prbs_ref[prbs_idx]
            
            # Shift delay line
            for lane in range(self.num_lanes - 1, 0, -1):
                # Each lane sees a different delay of the PRBS
                delay = lane
                if sample_idx >= delay:
                    delayed_prbs_bit = self.prbs_ref[(prbs_idx - delay) % self.prbs_length]
                else:
                    delayed_prbs_bit = 0
                
                # Zero-DSP correlation: conditional sign
                if delayed_prbs_bit:
                    accumulators[lane] += sample
                else:
                    accumulators[lane] -= sample
            
            # Lane 0: no delay
            if prbs_bit:
                accumulators[0] += sample
            else:
                accumulators[0] -= sample
            
            prbs_idx = (prbs_idx + 1) % self.prbs_length
        
        return np.abs(accumulators)
    
    def correlate_fft(self, rx_samples):
        """
        FFT-based correlation (fast, for comparison)
        
        Uses circular cross-correlation via FFT.
        Mathematically equivalent to zero-DSP but much faster in Python.
        """
        # Ensure same length
        n = max(len(rx_samples), self.prbs_length)
        
        # Pad signals
        rx_padded = np.zeros(n, dtype=np.complex64)
        rx_padded[:len(rx_samples)] = rx_samples
        
        ref_padded = np.zeros(n, dtype=np.complex64)
        ref_padded[:self.prbs_length] = self.prbs_bpsk
        
        # FFT correlation
        rx_fft = np.fft.fft(rx_padded)
        ref_fft = np.fft.fft(ref_padded)
        
        correlation = np.fft.ifft(rx_fft * np.conj(ref_fft))
        
        # Return first num_lanes bins (range profile)
        return np.abs(correlation[:self.num_lanes])

#=============================================================================
# PlutoSDR Interface
#=============================================================================

class PlutoRadar:
    """PlutoSDR Radar Interface"""
    
    def __init__(self, uri="ip:192.168.2.1", config=RadarConfig):
        """
        Initialize PlutoSDR radar
        
        Args:
            uri: PlutoSDR URI (default: ip:192.168.2.1)
            config: Radar configuration class
        """
        self.config = config
        self.uri = uri
        self.sdr = None
        self.tx_waveform = None
        
        # Initialize correlator
        self.correlator = ZeroDSPCorrelator(
            prbs_order=config.PRBS_ORDER,
            num_lanes=config.NUM_RANGE_BINS
        )
        
        # Generate PRBS waveform
        gen = PRBSGenerator(order=config.PRBS_ORDER)
        samples_per_chip = int(config.SAMPLE_RATE / config.CHIP_RATE)
        self.tx_waveform = gen.generate_bpsk_waveform(
            samples_per_chip=samples_per_chip,
            amplitude=0.8
        )
        
        print(f"[PlutoRadar] TX waveform: {len(self.tx_waveform)} samples")
    
    def connect(self):
        """Connect to PlutoSDR"""
        if not PLUTO_AVAILABLE:
            print("[PlutoRadar] Simulation mode (no hardware)")
            return True
        
        try:
            print(f"[PlutoRadar] Connecting to {self.uri}...")
            self.sdr = adi.Pluto(uri=self.uri)
            
            # Configure TX
            self.sdr.tx_lo = int(self.config.CENTER_FREQ)
            self.sdr.tx_rf_bandwidth = int(self.config.BANDWIDTH)
            self.sdr.tx_hardwaregain_chan0 = self.config.TX_GAIN
            self.sdr.sample_rate = int(self.config.SAMPLE_RATE)
            
            # Configure RX
            self.sdr.rx_lo = int(self.config.CENTER_FREQ)
            self.sdr.rx_rf_bandwidth = int(self.config.BANDWIDTH)
            self.sdr.gain_control_mode_chan0 = "manual"
            self.sdr.rx_hardwaregain_chan0 = self.config.RX_GAIN
            self.sdr.rx_buffer_size = self.config.CPI_LENGTH
            
            print("[PlutoRadar] Connected successfully!")
            print(f"  TX LO: {self.sdr.tx_lo/1e6:.1f} MHz")
            print(f"  RX LO: {self.sdr.rx_lo/1e6:.1f} MHz")
            print(f"  Sample Rate: {self.sdr.sample_rate/1e6:.1f} MSPS")
            
            return True
            
        except Exception as e:
            print(f"[PlutoRadar] Connection failed: {e}")
            return False
    
    def start_tx(self):
        """Start continuous TX"""
        if self.sdr is None:
            print("[PlutoRadar] No hardware - simulating TX")
            return
        
        # Enable cyclic buffer for continuous TX
        self.sdr.tx_cyclic_buffer = True
        self.sdr.tx(self.tx_waveform)
        print("[PlutoRadar] TX started (cyclic PRBS)")
    
    def stop_tx(self):
        """Stop TX"""
        if self.sdr is not None:
            self.sdr.tx_destroy_buffer()
            print("[PlutoRadar] TX stopped")
    
    def capture_cpi(self):
        """Capture one CPI of RX samples"""
        if self.sdr is None:
            # Simulation mode - generate synthetic data
            return self._simulate_rx()
        
        return self.sdr.rx()
    
    def _simulate_rx(self):
        """Generate simulated RX data for testing"""
        n_samples = self.config.CPI_LENGTH
        
        # Add delayed copies of TX waveform (simulated targets)
        rx = np.zeros(n_samples, dtype=np.complex64)
        
        # Target 1: Delay 100 samples, amplitude 0.5
        delay1 = 100
        amp1 = 0.5
        tx_len = len(self.tx_waveform)
        if delay1 + tx_len <= n_samples:
            rx[delay1:delay1+tx_len] += amp1 * self.tx_waveform
        
        # Target 2: Delay 300 samples, amplitude 0.2
        delay2 = 300
        amp2 = 0.2
        if delay2 + tx_len <= n_samples:
            rx[delay2:delay2+tx_len] += amp2 * self.tx_waveform
        
        # Add noise
        noise_power = 0.01
        noise = np.sqrt(noise_power/2) * (
            np.random.randn(n_samples) + 1j * np.random.randn(n_samples)
        )
        rx += noise.astype(np.complex64)
        
        return rx
    
    def process_cpi(self, rx_samples):
        """
        Process one CPI
        
        Args:
            rx_samples: Complex RX samples
            
        Returns:
            Range profile (magnitude vs range bin)
        """
        # Use FFT correlation (fast)
        range_profile = self.correlator.correlate_fft(rx_samples)
        
        return range_profile
    
    def run_cpi_loop(self, num_cpis=100, callback=None):
        """
        Run continuous CPI processing loop
        
        Args:
            num_cpis: Number of CPIs to process
            callback: Optional callback function(cpi_idx, range_profile)
        """
        print(f"\n[PlutoRadar] Starting CPI loop ({num_cpis} CPIs)...")
        
        range_profiles = []
        
        for cpi_idx in range(num_cpis):
            # Capture
            rx_samples = self.capture_cpi()
            
            # Process
            range_profile = self.process_cpi(rx_samples)
            range_profiles.append(range_profile)
            
            # Callback
            if callback:
                callback(cpi_idx, range_profile)
            
            # Progress
            if (cpi_idx + 1) % 10 == 0:
                print(f"  CPI {cpi_idx + 1}/{num_cpis}")
        
        # Average across CPIs
        avg_profile = np.mean(range_profiles, axis=0)
        
        return avg_profile, range_profiles

#=============================================================================
# Main Application
#=============================================================================

def run_loopback_test():
    """Run loopback self-test"""
    print("\n" + "=" * 60)
    print("LOOPBACK TEST")
    print("=" * 60)
    print("Connect PlutoSDR TX to RX with 30dB attenuator")
    print()
    
    radar = PlutoRadar()
    
    if radar.connect():
        radar.start_tx()
        time.sleep(0.5)  # Let TX stabilize
        
        # Capture and process
        avg_profile, _ = radar.run_cpi_loop(num_cpis=10)
        
        radar.stop_tx()
        
        # Analyze results
        peak_idx = np.argmax(avg_profile)
        peak_val = avg_profile[peak_idx]
        noise_floor = np.median(avg_profile)
        snr = 20 * np.log10(peak_val / noise_floor)
        
        print("\n" + "-" * 40)
        print("RESULTS:")
        print(f"  Peak bin:     {peak_idx}")
        print(f"  Peak value:   {peak_val:.2f}")
        print(f"  Noise floor:  {noise_floor:.4f}")
        print(f"  SNR:          {snr:.1f} dB")
        print("-" * 40)
        
        if snr > 40:
            print("✅ TEST PASSED - System working correctly")
        else:
            print("⚠️ TEST WARNING - SNR lower than expected")
        
        return avg_profile
    
    return None

def run_radar_mode(mode="monostatic"):
    """Run radar in specified mode"""
    print("\n" + "=" * 60)
    print(f"RADAR MODE: {mode.upper()}")
    print("=" * 60)
    
    RadarConfig.print_config()
    
    radar = PlutoRadar()
    
    if radar.connect():
        radar.start_tx()
        time.sleep(0.5)
        
        try:
            print("\nPress Ctrl+C to stop...\n")
            cpi_count = 0
            
            while True:
                rx_samples = radar.capture_cpi()
                range_profile = radar.process_cpi(rx_samples)
                
                # Find peaks
                threshold = np.median(range_profile) * 10
                peaks = np.where(range_profile > threshold)[0]
                
                cpi_count += 1
                
                if len(peaks) > 0:
                    for peak_idx in peaks[:5]:  # Top 5 peaks
                        range_m = peak_idx * RadarConfig.RANGE_RESOLUTION
                        snr = 20 * np.log10(range_profile[peak_idx] / np.median(range_profile))
                        print(f"[CPI {cpi_count}] Detection: bin={peak_idx}, "
                              f"range={range_m:.0f}m, SNR={snr:.1f}dB")
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\nStopping...")
        
        finally:
            radar.stop_tx()

def main():
    parser = argparse.ArgumentParser(description="QEDMMA PoC Radar")
    parser.add_argument("--mode", choices=["loopback", "monostatic", "bistatic", "sim"],
                       default="sim", help="Operating mode")
    parser.add_argument("--uri", default="ip:192.168.2.1",
                       help="PlutoSDR URI")
    args = parser.parse_args()
    
    print("\n" + "=" * 60)
    print("QEDMMA PoC - 'Garažni Pobunjenik' v3.4")
    print("Author: Dr. Mladen Mešter")
    print("=" * 60)
    
    if args.mode == "loopback":
        run_loopback_test()
    elif args.mode == "sim":
        # Simulation mode
        print("\nRunning in SIMULATION mode (no hardware)")
        run_loopback_test()
    else:
        run_radar_mode(args.mode)

if __name__ == "__main__":
    main()
