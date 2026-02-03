#!/usr/bin/env python3
"""
QEDMMA PoC - bladeRF 2.0 xA9 Radar Interface
Alternative SDR driver for native VHF support

Author: Dr. Mladen Mešter
Copyright (c) 2026 - All Rights Reserved

Requires: pip install bladerf

Note: This is a TEMPLATE for bladeRF upgrade path.
      Currently the PoC uses PlutoSDR (pluto_radar.py)
"""

import numpy as np
import time

try:
    import bladerf
    BLADERF_AVAILABLE = True
except ImportError:
    BLADERF_AVAILABLE = False
    print("Warning: bladerf module not installed. Install with: pip install bladerf")

#=============================================================================
# Configuration
#=============================================================================

class BladeRFConfig:
    """bladeRF xA9 configuration for VHF radar"""
    
    # RF parameters - Native VHF support!
    CENTER_FREQ = 155e6      # 155 MHz (native, no hack needed!)
    SAMPLE_RATE = 10e6       # 10 MSPS (bladeRF can do 122 MSPS)
    BANDWIDTH = 5e6          # 5 MHz
    TX_GAIN = 40             # dB (0-66 dB range)
    RX_GAIN = 40             # dB
    
    # PRBS parameters
    PRBS_ORDER = 15
    CHIP_RATE = 1e6
    
    # Processing
    NUM_RANGE_BINS = 512
    CPI_LENGTH = 32768
    
    @classmethod
    def print_config(cls):
        print("=" * 60)
        print("bladeRF xA9 Radar Configuration")
        print("=" * 60)
        print(f"Center Frequency:   {cls.CENTER_FREQ/1e6:.1f} MHz (NATIVE VHF!)")
        print(f"Sample Rate:        {cls.SAMPLE_RATE/1e6:.1f} MSPS")
        print(f"Bandwidth:          {cls.BANDWIDTH/1e6:.1f} MHz")
        print(f"TX Gain:            {cls.TX_GAIN} dB")
        print(f"RX Gain:            {cls.RX_GAIN} dB")
        print("=" * 60)

#=============================================================================
# bladeRF Interface
#=============================================================================

class BladeRFRadar:
    """
    bladeRF 2.0 xA9 Radar Interface
    
    Advantages over PlutoSDR:
    - Native VHF support (47 MHz min vs 70 MHz hacked on Pluto)
    - USB 3.0 SuperSpeed (5 Gbps vs 480 Mbps)
    - Larger FPGA (301K LEs vs 28K)
    - Higher bandwidth (122 MHz vs 56 MHz)
    - Better oscillator (VCTCXO with 10 MHz ref input)
    """
    
    def __init__(self, device_identifier="*:serial=*"):
        self.config = BladeRFConfig
        self.device = None
        self.tx_channel = None
        self.rx_channel = None
        self.tx_waveform = None
        
    def connect(self):
        """Connect to bladeRF device"""
        if not BLADERF_AVAILABLE:
            print("[BladeRF] Module not available - simulation mode")
            return True
            
        try:
            # Find and open device
            devices = bladerf.get_device_list()
            if not devices:
                print("[BladeRF] No devices found")
                return False
            
            print(f"[BladeRF] Found {len(devices)} device(s)")
            self.device = bladerf.BladeRF()
            
            # Print device info
            info = self.device.get_devinfo()
            print(f"[BladeRF] Connected to: {info}")
            
            # Check FPGA loaded
            fpga_size = self.device.fpga_size
            print(f"[BladeRF] FPGA size: {fpga_size}")
            
            # Configure TX channel
            self.tx_channel = self.device.Channel(bladerf.CHANNEL_TX(0))
            self.tx_channel.frequency = int(self.config.CENTER_FREQ)
            self.tx_channel.sample_rate = int(self.config.SAMPLE_RATE)
            self.tx_channel.bandwidth = int(self.config.BANDWIDTH)
            self.tx_channel.gain = self.config.TX_GAIN
            
            # Configure RX channel
            self.rx_channel = self.device.Channel(bladerf.CHANNEL_RX(0))
            self.rx_channel.frequency = int(self.config.CENTER_FREQ)
            self.rx_channel.sample_rate = int(self.config.SAMPLE_RATE)
            self.rx_channel.bandwidth = int(self.config.BANDWIDTH)
            self.rx_channel.gain = self.config.RX_GAIN
            
            print(f"[BladeRF] TX configured: {self.config.CENTER_FREQ/1e6:.1f} MHz")
            print(f"[BladeRF] RX configured: {self.config.CENTER_FREQ/1e6:.1f} MHz")
            
            return True
            
        except Exception as e:
            print(f"[BladeRF] Connection error: {e}")
            return False
    
    def configure_sync(self, num_buffers=16, buffer_size=8192):
        """Configure synchronous interface"""
        if self.device is None:
            return
            
        # Configure TX sync
        self.device.sync_config(
            layout=bladerf.ChannelLayout.TX_X1,
            fmt=bladerf.Format.SC16_Q11,
            num_buffers=num_buffers,
            buffer_size=buffer_size,
            num_transfers=8,
            stream_timeout=3500
        )
        
        # Configure RX sync
        self.device.sync_config(
            layout=bladerf.ChannelLayout.RX_X1,
            fmt=bladerf.Format.SC16_Q11,
            num_buffers=num_buffers,
            buffer_size=buffer_size,
            num_transfers=8,
            stream_timeout=3500
        )
        
        print("[BladeRF] Sync interface configured")
    
    def set_prbs_waveform(self, waveform):
        """Set transmit waveform"""
        self.tx_waveform = waveform
        
    def start_tx(self):
        """Enable TX"""
        if self.device is None:
            print("[BladeRF] Simulation - TX started")
            return
            
        self.tx_channel.enable = True
        print("[BladeRF] TX enabled")
    
    def stop_tx(self):
        """Disable TX"""
        if self.device is None:
            return
            
        self.tx_channel.enable = False
        print("[BladeRF] TX disabled")
    
    def transmit(self, samples):
        """Transmit samples"""
        if self.device is None:
            return
            
        # Convert to int16 format
        samples_int = (samples * 2047).astype(np.int16)
        self.device.sync_tx(samples_int, len(samples_int))
    
    def receive(self, num_samples):
        """Receive samples"""
        if self.device is None:
            # Simulation mode
            return self._simulate_rx(num_samples)
            
        # Allocate buffer
        buf = np.zeros(num_samples, dtype=np.int16)
        
        # Receive
        self.rx_channel.enable = True
        self.device.sync_rx(buf, num_samples)
        
        # Convert to complex float
        samples = buf.astype(np.float32) / 2047.0
        return samples
    
    def _simulate_rx(self, num_samples):
        """Simulate received signal for testing"""
        # Generate noise + simulated target
        noise = 0.01 * (np.random.randn(num_samples) + 1j * np.random.randn(num_samples))
        
        # Add delayed copy of TX waveform if available
        if self.tx_waveform is not None and len(self.tx_waveform) < num_samples:
            delay = 100
            noise[delay:delay+len(self.tx_waveform)] += 0.5 * self.tx_waveform
        
        return noise.astype(np.complex64)
    
    def close(self):
        """Close device"""
        if self.device is not None:
            self.stop_tx()
            self.device.close()
            print("[BladeRF] Device closed")

#=============================================================================
# FPGA Correlator Interface (Future)
#=============================================================================

class BladeRFFPGACorrelator:
    """
    Interface to FPGA-based correlator on bladeRF xA9
    
    The xA9 has 301K LEs which can implement:
    - Full Zero-DSP correlator (512 lanes)
    - CFAR detector
    - Doppler processing
    - All on-FPGA, freeing host CPU
    
    This class would interface with custom FPGA bitstream.
    """
    
    def __init__(self, device):
        self.device = device
        self.fpga_loaded = False
        
    def load_correlator_bitstream(self, bitstream_path):
        """Load custom correlator FPGA image"""
        # Would load custom bitstream with correlator
        print(f"[FPGA] Would load: {bitstream_path}")
        self.fpga_loaded = True
        
    def configure_prbs(self, order=15):
        """Configure PRBS generator in FPGA"""
        # Write to FPGA registers
        pass
        
    def read_range_profile(self):
        """Read processed range profile from FPGA"""
        # Read from FPGA memory
        pass
        
    def read_detections(self):
        """Read CFAR detections from FPGA"""
        # Read detection FIFO
        pass

#=============================================================================
# Demo
#=============================================================================

def demo():
    """Demo bladeRF radar interface"""
    print("\n" + "=" * 60)
    print("bladeRF 2.0 xA9 Radar Interface Demo")
    print("=" * 60)
    
    BladeRFConfig.print_config()
    
    radar = BladeRFRadar()
    
    if radar.connect():
        print("\n[Demo] Running simulated capture...")
        
        # Simulate capture
        samples = radar.receive(32768)
        
        print(f"[Demo] Captured {len(samples)} samples")
        print(f"[Demo] Mean power: {np.mean(np.abs(samples)**2):.6f}")
        
        radar.close()
    
    print("\n" + "=" * 60)
    print("bladeRF Advantages for Radar:")
    print("=" * 60)
    print("  ✅ Native 47 MHz - 6 GHz (no hacks for VHF!)")
    print("  ✅ USB 3.0 SuperSpeed (10× faster than Pluto)")
    print("  ✅ 301K LE FPGA (10× larger than Pluto)")
    print("  ✅ 122 MHz max bandwidth (2× Pluto)")
    print("  ✅ 10 MHz reference input for coherence")
    print("  ✅ Headless operation possible")
    print("=" * 60)

if __name__ == "__main__":
    demo()
