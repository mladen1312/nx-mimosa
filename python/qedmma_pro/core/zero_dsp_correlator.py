#!/usr/bin/env python3
"""
QEDMMA PoC - Zero-DSP Correlator Implementation
Demonstrates FPGA-equivalent algorithm in Python

Author: Dr. Mladen Mešter
Copyright (c) 2026 - All Rights Reserved

Key Innovation:
    Instead of: product = sample × prbs_chip  (requires DSP multiplier)
    We use:     if prbs_bit: acc += sample else: acc -= sample  (ZERO DSP!)
    
    This is mathematically equivalent because PRBS chips are {+1, -1}
"""

import numpy as np
from numba import jit, prange
import time

#=============================================================================
# PRBS Generator (Optimized)
#=============================================================================

@jit(nopython=True)
def generate_prbs_fast(order, length):
    """
    Fast PRBS generation using Numba JIT
    
    Args:
        order: PRBS order (15 or 20)
        length: Number of bits to generate
        
    Returns:
        numpy array of {0, 1} bits
    """
    # Tap positions for maximal length sequences
    if order == 15:
        tap1, tap2 = 15, 14
    elif order == 20:
        tap1, tap2 = 20, 17
    elif order == 11:
        tap1, tap2 = 11, 9
    else:
        tap1, tap2 = 15, 14  # Default
    
    state = (1 << order) - 1  # All ones
    bits = np.zeros(length, dtype=np.int8)
    
    for i in range(length):
        # Output LSB
        bits[i] = state & 1
        
        # Calculate feedback
        fb = ((state >> (tap1-1)) ^ (state >> (tap2-1))) & 1
        
        # Shift
        state = ((state >> 1) | (fb << (order-1))) & ((1 << order) - 1)
    
    return bits

#=============================================================================
# Zero-DSP Correlator (Streaming Implementation)
#=============================================================================

@jit(nopython=True, parallel=True)
def correlate_zero_dsp_streaming(rx_samples, prbs_bits, num_lanes):
    """
    Zero-DSP correlation - streaming implementation
    
    Simulates FPGA architecture where each clock cycle:
    1. Shift PRBS through delay line
    2. For each lane: if prbs_delayed[lane]: acc[lane] += sample
                      else: acc[lane] -= sample
    
    Args:
        rx_samples: Real part of received samples
        prbs_bits: PRBS reference sequence {0, 1}
        num_lanes: Number of parallel correlation lanes
        
    Returns:
        Correlation magnitudes for each lane (range bin)
    """
    n_samples = len(rx_samples)
    n_prbs = len(prbs_bits)
    
    # Accumulators for each lane
    accumulators = np.zeros(num_lanes, dtype=np.float64)
    
    # Process each sample
    for sample_idx in range(min(n_samples, n_prbs)):
        sample = rx_samples[sample_idx]
        
        # Each lane correlates with different delay
        for lane in prange(num_lanes):
            # Get PRBS bit at this delay
            prbs_idx = (sample_idx - lane) % n_prbs
            if prbs_idx < 0:
                prbs_idx += n_prbs
            
            prbs_bit = prbs_bits[prbs_idx]
            
            # Zero-DSP operation: conditional sign
            if prbs_bit == 1:
                accumulators[lane] += sample
            else:
                accumulators[lane] -= sample
    
    return np.abs(accumulators)

#=============================================================================
# FFT-based Correlator (Fast Reference)
#=============================================================================

def correlate_fft(rx_samples, prbs_bits, num_lanes):
    """
    FFT-based correlation (fast, for comparison/validation)
    
    Uses circular cross-correlation via FFT.
    Result is mathematically identical to zero-DSP.
    """
    n = max(len(rx_samples), len(prbs_bits))
    
    # Convert PRBS to BPSK (+1/-1)
    prbs_bpsk = 2.0 * prbs_bits.astype(np.float64) - 1.0
    
    # Pad to same length
    rx_padded = np.zeros(n, dtype=np.complex128)
    rx_padded[:len(rx_samples)] = rx_samples
    
    ref_padded = np.zeros(n, dtype=np.complex128)
    ref_padded[:len(prbs_bits)] = prbs_bpsk
    
    # FFT correlation
    rx_fft = np.fft.fft(rx_padded)
    ref_fft = np.fft.fft(ref_padded)
    
    corr = np.fft.ifft(rx_fft * np.conj(ref_fft))
    
    return np.abs(corr[:num_lanes])

#=============================================================================
# CFAR Detector
#=============================================================================

def cfar_ca(range_profile, guard_cells=4, ref_cells=16, pfa=1e-4):
    """
    Cell-Averaging CFAR detector
    
    Args:
        range_profile: Input range profile (magnitudes)
        guard_cells: Guard cells on each side
        ref_cells: Reference cells on each side
        pfa: Probability of false alarm
        
    Returns:
        detections: Boolean array of detections
        threshold: Adaptive threshold array
    """
    n = len(range_profile)
    threshold = np.zeros(n)
    
    # CFAR constant (for Swerling I target)
    # alpha = N * (Pfa^(-1/N) - 1) where N = 2*ref_cells
    N = 2 * ref_cells
    alpha = N * (pfa ** (-1/N) - 1)
    
    for i in range(n):
        # Leading cells
        lead_start = max(0, i - guard_cells - ref_cells)
        lead_end = max(0, i - guard_cells)
        leading = range_profile[lead_start:lead_end]
        
        # Lagging cells
        lag_start = min(n, i + guard_cells + 1)
        lag_end = min(n, i + guard_cells + ref_cells + 1)
        lagging = range_profile[lag_start:lag_end]
        
        # Noise estimate
        ref_cells_data = np.concatenate([leading, lagging])
        if len(ref_cells_data) > 0:
            noise_est = np.mean(ref_cells_data)
        else:
            noise_est = np.median(range_profile)
        
        threshold[i] = alpha * noise_est
    
    detections = range_profile > threshold
    
    return detections, threshold

#=============================================================================
# Main Correlator Class
#=============================================================================

class ZeroDSPCorrelator:
    """
    Complete Zero-DSP Correlator System
    
    Features:
    - PRBS-15 or PRBS-20 support
    - Streaming (FPGA-like) or FFT modes
    - Built-in CFAR detector
    - Performance benchmarking
    """
    
    def __init__(self, prbs_order=15, num_lanes=512, mode='fft'):
        """
        Initialize correlator
        
        Args:
            prbs_order: 15 or 20 (PRBS length = 2^order - 1)
            num_lanes: Number of range bins
            mode: 'streaming' (FPGA-like) or 'fft' (fast)
        """
        self.prbs_order = prbs_order
        self.num_lanes = num_lanes
        self.mode = mode
        self.prbs_length = 2**prbs_order - 1
        
        # Generate PRBS reference
        print(f"[Correlator] Generating PRBS-{prbs_order}...")
        self.prbs_bits = generate_prbs_fast(prbs_order, self.prbs_length)
        
        # Processing gain
        self.proc_gain_db = 10 * np.log10(self.prbs_length)
        
        print(f"[Correlator] Initialized:")
        print(f"  PRBS Order:       {prbs_order}")
        print(f"  PRBS Length:      {self.prbs_length:,} chips")
        print(f"  Processing Gain:  {self.proc_gain_db:.1f} dB")
        print(f"  Range Bins:       {num_lanes}")
        print(f"  Mode:             {mode}")
    
    def correlate(self, rx_samples):
        """
        Perform correlation
        
        Args:
            rx_samples: Complex or real received samples
            
        Returns:
            Range profile (magnitude vs range bin)
        """
        # Use real part if complex
        if np.iscomplexobj(rx_samples):
            samples = np.real(rx_samples).astype(np.float64)
        else:
            samples = rx_samples.astype(np.float64)
        
        if self.mode == 'streaming':
            return correlate_zero_dsp_streaming(
                samples, self.prbs_bits, self.num_lanes
            )
        else:
            return correlate_fft(
                samples, self.prbs_bits, self.num_lanes
            )
    
    def detect(self, range_profile, pfa=1e-4):
        """
        CFAR detection on range profile
        
        Args:
            range_profile: Correlation output
            pfa: Probability of false alarm
            
        Returns:
            detections: List of (bin, magnitude, snr) tuples
        """
        det_mask, threshold = cfar_ca(range_profile, pfa=pfa)
        
        detections = []
        noise_floor = np.median(range_profile)
        
        for i in np.where(det_mask)[0]:
            snr_db = 20 * np.log10(range_profile[i] / noise_floor)
            detections.append({
                'bin': i,
                'magnitude': range_profile[i],
                'snr_db': snr_db,
                'threshold': threshold[i]
            })
        
        return detections
    
    def benchmark(self, n_iterations=100):
        """
        Benchmark correlator performance
        """
        print(f"\n[Benchmark] Running {n_iterations} iterations...")
        
        # Generate test signal
        test_signal = np.random.randn(self.prbs_length).astype(np.float64)
        
        # Warm up
        _ = self.correlate(test_signal)
        
        # Benchmark
        start = time.perf_counter()
        for _ in range(n_iterations):
            _ = self.correlate(test_signal)
        elapsed = time.perf_counter() - start
        
        time_per_corr = elapsed / n_iterations * 1000  # ms
        throughput = n_iterations * self.prbs_length / elapsed / 1e6  # Msamples/s
        
        print(f"[Benchmark] Results:")
        print(f"  Time per correlation: {time_per_corr:.2f} ms")
        print(f"  Throughput:           {throughput:.1f} Msamples/s")
        
        return time_per_corr, throughput

#=============================================================================
# Demo / Test
#=============================================================================

def demo():
    """Demonstrate zero-DSP correlator"""
    print("\n" + "=" * 60)
    print("ZERO-DSP CORRELATOR DEMO")
    print("=" * 60)
    
    # Initialize
    correlator = ZeroDSPCorrelator(prbs_order=15, num_lanes=512, mode='fft')
    
    # Generate test signal with targets
    print("\n[Demo] Generating test signal with 3 targets...")
    
    n_samples = correlator.prbs_length
    prbs_bpsk = 2.0 * correlator.prbs_bits - 1.0
    
    # Create received signal
    rx = np.zeros(n_samples, dtype=np.float64)
    
    # Target 1: delay 50, amplitude 1.0
    rx[50:50+len(prbs_bpsk)] += 1.0 * prbs_bpsk[:n_samples-50]
    
    # Target 2: delay 200, amplitude 0.5
    rx[200:200+len(prbs_bpsk)] += 0.5 * prbs_bpsk[:n_samples-200]
    
    # Target 3: delay 400, amplitude 0.2
    rx[400:400+len(prbs_bpsk)] += 0.2 * prbs_bpsk[:n_samples-400]
    
    # Add noise (SNR before processing ~ 0 dB)
    noise_power = 1.0
    rx += np.sqrt(noise_power) * np.random.randn(n_samples)
    
    # Correlate
    print("[Demo] Running correlation...")
    range_profile = correlator.correlate(rx)
    
    # Detect
    detections = correlator.detect(range_profile, pfa=1e-6)
    
    print(f"\n[Demo] Found {len(detections)} detections:")
    for det in detections:
        print(f"  Bin {det['bin']:3d}: SNR = {det['snr_db']:.1f} dB")
    
    # Benchmark
    correlator.benchmark(n_iterations=50)
    
    return range_profile, detections

if __name__ == "__main__":
    demo()
