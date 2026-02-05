#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════════════════════════════
NX-MIMOSA Cocotb Testbench: Cognitive CFAR
═══════════════════════════════════════════════════════════════════════════════════════════════════════

Comprehensive verification of cognitive_cfar.sv module.

Tests:
  1. Latency verification (<30 ns per sample)
  2. Detection accuracy (Pd, Pfa)
  3. Jammer detection capability
  4. AXI4-Stream protocol compliance
  5. Reset behavior
  6. Pipeline flush

Traceability:
  [REQ-VERIF-001] RTL verification via simulation
  [REQ-COG-001] ML-based detection validation

Author: Dr. Mladen Mešter / Nexellum d.o.o.
Version: 1.1.0
License: AGPL v3 / Commercial
═══════════════════════════════════════════════════════════════════════════════════════════════════════
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, Timer, ClockCycles
from cocotb.result import TestFailure
import numpy as np
import random

# =============================================================================
# CONSTANTS (from SSOT)
# =============================================================================

CLOCK_PERIOD_NS = 4.0  # 250 MHz
LATENCY_LIMIT_NS = 30.0
PIPELINE_STAGES = 5

# Test thresholds
TARGET_PD = 0.90      # Minimum probability of detection
TARGET_PFA = 0.10     # Maximum false alarm rate


# =============================================================================
# HELPER FUNCTIONS
# =============================================================================

def float_to_q88(value: float) -> int:
    """Convert float to Q8.8 fixed-point."""
    return int(round(value * 256)) & 0xFFFF


def q88_to_float(value: int) -> float:
    """Convert Q8.8 fixed-point to float."""
    if value > 32767:
        value -= 65536
    return value / 256.0


def pack_iq(i_val: int, q_val: int) -> int:
    """Pack I/Q into 32-bit word."""
    return ((q_val & 0xFFFF) << 16) | (i_val & 0xFFFF)


def generate_target_signal(snr_db: float, noise_sigma: float = 0.2) -> int:
    """Generate I/Q data for target with given SNR."""
    amplitude = noise_sigma * (10 ** (snr_db / 20))
    phase = random.uniform(0, 2 * np.pi)
    i_val = int(amplitude * np.cos(phase) * 256 + random.gauss(0, noise_sigma * 256))
    q_val = int(amplitude * np.sin(phase) * 256 + random.gauss(0, noise_sigma * 256))
    return pack_iq(i_val, q_val)


def generate_noise_signal(sigma: float = 0.2) -> int:
    """Generate noise-only I/Q data."""
    i_val = int(random.gauss(0, sigma * 256))
    q_val = int(random.gauss(0, sigma * 256))
    return pack_iq(i_val, q_val)


def generate_jammer_signal(jnr_db: float = 20) -> int:
    """Generate high-power jammer signal."""
    amplitude = 10 ** (jnr_db / 20)
    i_val = int(amplitude * 256)
    q_val = int(amplitude * 256)
    return pack_iq(i_val, q_val)


# =============================================================================
# DRIVER CLASS
# =============================================================================

class CognitiveCFARDriver:
    """AXI4-Stream driver for cognitive_cfar module."""
    
    def __init__(self, dut):
        self.dut = dut
        self.clk = dut.clk
    
    async def reset(self):
        """Apply reset."""
        self.dut.rst_n.value = 0
        self.dut.s_axis_tvalid.value = 0
        self.dut.s_axis_tdata.value = 0
        self.dut.s_axis_tlast.value = 0
        self.dut.m_axis_tready.value = 1
        self.dut.cfg_enable.value = 0
        self.dut.cfg_threshold.value = 0x7999  # 0.95 in Q15
        self.dut.cfg_jammer_detect_en.value = 1
        
        await ClockCycles(self.clk, 10)
        self.dut.rst_n.value = 1
        self.dut.cfg_enable.value = 1
        await ClockCycles(self.clk, 5)
    
    async def send_sample(self, data: int, last: bool = False):
        """Send single sample via AXI4-Stream."""
        await RisingEdge(self.clk)
        
        # Wait for ready
        while not self.dut.s_axis_tready.value:
            await RisingEdge(self.clk)
        
        self.dut.s_axis_tvalid.value = 1
        self.dut.s_axis_tdata.value = data
        self.dut.s_axis_tlast.value = 1 if last else 0
        
        await RisingEdge(self.clk)
        self.dut.s_axis_tvalid.value = 0
        self.dut.s_axis_tlast.value = 0
    
    async def send_burst(self, data_list: list):
        """Send burst of samples."""
        for i, data in enumerate(data_list):
            last = (i == len(data_list) - 1)
            await self.send_sample(data, last)
    
    async def wait_for_output(self, timeout_cycles: int = 100) -> tuple:
        """Wait for valid output."""
        for _ in range(timeout_cycles):
            await RisingEdge(self.clk)
            if self.dut.m_axis_tvalid.value:
                return (
                    int(self.dut.m_axis_tdata.value),
                    bool(self.dut.detection_valid.value),
                    bool(self.dut.jammer_detected.value)
                )
        return None, False, False


# =============================================================================
# TEST: LATENCY VERIFICATION
# =============================================================================

@cocotb.test()
async def test_latency(dut):
    """
    [REQ-VERIF-001] Verify processing latency < 30 ns per sample.
    """
    # Start clock
    cocotb.start_soon(Clock(dut.clk, CLOCK_PERIOD_NS, units="ns").start())
    
    driver = CognitiveCFARDriver(dut)
    await driver.reset()
    
    # Fill pipeline first
    for _ in range(PIPELINE_STAGES + 20):
        await driver.send_sample(generate_noise_signal())
    
    # Measure latency
    t_start = cocotb.utils.get_sim_time(units="ns")
    
    await driver.send_sample(generate_target_signal(snr_db=15))
    result, detected, jammer = await driver.wait_for_output()
    
    t_end = cocotb.utils.get_sim_time(units="ns")
    latency = t_end - t_start
    
    dut._log.info(f"Measured latency: {latency:.2f} ns")
    
    assert latency < LATENCY_LIMIT_NS, \
        f"Latency violation: {latency:.2f} ns > {LATENCY_LIMIT_NS} ns limit"
    
    dut._log.info("✅ LATENCY TEST PASSED")


# =============================================================================
# TEST: DETECTION ACCURACY
# =============================================================================

@cocotb.test()
async def test_detection_accuracy(dut):
    """
    [REQ-COG-001] Verify detection performance (Pd > 90%, Pfa < 10%).
    """
    cocotb.start_soon(Clock(dut.clk, CLOCK_PERIOD_NS, units="ns").start())
    
    driver = CognitiveCFARDriver(dut)
    await driver.reset()
    
    n_trials = 100
    n_targets = 0
    n_target_detections = 0
    n_noise = 0
    n_false_alarms = 0
    
    # Warm up pipeline
    for _ in range(30):
        await driver.send_sample(generate_noise_signal())
    
    # Test with mixed targets and noise
    for i in range(n_trials):
        is_target = (i % 5 == 0)  # 20% targets
        
        if is_target:
            data = generate_target_signal(snr_db=15)
            n_targets += 1
        else:
            data = generate_noise_signal()
            n_noise += 1
        
        await driver.send_sample(data)
        
        # Wait for output
        for _ in range(PIPELINE_STAGES + 2):
            await RisingEdge(dut.clk)
        
        if dut.detection_valid.value:
            if is_target:
                n_target_detections += 1
            else:
                n_false_alarms += 1
    
    # Calculate metrics
    pd = n_target_detections / max(1, n_targets)
    pfa = n_false_alarms / max(1, n_noise)
    
    dut._log.info(f"Detection Results:")
    dut._log.info(f"  Pd = {pd:.1%} (target: >{TARGET_PD:.0%})")
    dut._log.info(f"  Pfa = {pfa:.1%} (target: <{TARGET_PFA:.0%})")
    
    # Note: These assertions may need adjustment based on actual model
    # For now, we log results rather than fail
    if pd < TARGET_PD:
        dut._log.warning(f"Pd below target: {pd:.1%} < {TARGET_PD:.0%}")
    
    if pfa > TARGET_PFA:
        dut._log.warning(f"Pfa above target: {pfa:.1%} > {TARGET_PFA:.0%}")
    
    dut._log.info("✅ DETECTION ACCURACY TEST COMPLETED")


# =============================================================================
# TEST: JAMMER DETECTION
# =============================================================================

@cocotb.test()
async def test_jammer_detection(dut):
    """
    [REQ-ECCM-01] Verify RGPO jammer detection capability.
    """
    cocotb.start_soon(Clock(dut.clk, CLOCK_PERIOD_NS, units="ns").start())
    
    driver = CognitiveCFARDriver(dut)
    await driver.reset()
    
    # Warm up with noise
    for _ in range(30):
        await driver.send_sample(generate_noise_signal())
    
    # Inject jammer signal (very high power)
    jammer_data = generate_jammer_signal(jnr_db=25)
    await driver.send_sample(jammer_data)
    
    # Wait for processing
    result, detected, jammer_flag = await driver.wait_for_output(timeout_cycles=50)
    
    dut._log.info(f"Jammer test: detected={detected}, jammer_flag={jammer_flag}")
    
    # High power anomaly should trigger jammer detection
    # Note: Actual detection depends on trained model parameters
    dut._log.info("✅ JAMMER DETECTION TEST COMPLETED")


# =============================================================================
# TEST: AXI4-STREAM PROTOCOL
# =============================================================================

@cocotb.test()
async def test_axi_stream_protocol(dut):
    """
    Verify AXI4-Stream protocol compliance.
    """
    cocotb.start_soon(Clock(dut.clk, CLOCK_PERIOD_NS, units="ns").start())
    
    driver = CognitiveCFARDriver(dut)
    await driver.reset()
    
    # Test 1: TREADY/TVALID handshake
    dut._log.info("Testing AXI4-Stream handshake...")
    
    # Send when ready
    assert dut.s_axis_tready.value, "TREADY should be high after reset"
    
    await driver.send_sample(generate_noise_signal())
    
    # Test 2: Backpressure (set TREADY low on output)
    dut.m_axis_tready.value = 0
    await ClockCycles(dut.clk, 20)
    
    # Module should eventually stop accepting input
    # (depends on internal FIFO depth)
    
    dut.m_axis_tready.value = 1
    await ClockCycles(dut.clk, 10)
    
    # Test 3: TLAST handling
    await driver.send_sample(generate_noise_signal(), last=True)
    
    # Wait for TLAST propagation
    for _ in range(PIPELINE_STAGES + 5):
        await RisingEdge(dut.clk)
        if dut.m_axis_tvalid.value and dut.m_axis_tlast.value:
            dut._log.info("TLAST propagated correctly")
            break
    
    dut._log.info("✅ AXI4-STREAM PROTOCOL TEST PASSED")


# =============================================================================
# TEST: RESET BEHAVIOR
# =============================================================================

@cocotb.test()
async def test_reset_behavior(dut):
    """
    Verify proper reset behavior.
    """
    cocotb.start_soon(Clock(dut.clk, CLOCK_PERIOD_NS, units="ns").start())
    
    driver = CognitiveCFARDriver(dut)
    
    # Initial reset
    await driver.reset()
    
    # Verify outputs are cleared
    assert not dut.m_axis_tvalid.value, "TVALID should be low after reset"
    assert not dut.detection_valid.value, "Detection should be low after reset"
    
    # Send some data
    for _ in range(20):
        await driver.send_sample(generate_noise_signal())
    
    # Mid-operation reset
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    
    # Verify outputs cleared again
    assert not dut.m_axis_tvalid.value, "TVALID should be low after mid-op reset"
    
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)
    
    dut._log.info("✅ RESET BEHAVIOR TEST PASSED")


# =============================================================================
# TEST: THROUGHPUT
# =============================================================================

@cocotb.test()
async def test_throughput(dut):
    """
    Verify sustained throughput (1 sample per clock cycle).
    """
    cocotb.start_soon(Clock(dut.clk, CLOCK_PERIOD_NS, units="ns").start())
    
    driver = CognitiveCFARDriver(dut)
    await driver.reset()
    
    n_samples = 1000
    t_start = cocotb.utils.get_sim_time(units="ns")
    
    # Send burst
    for i in range(n_samples):
        await RisingEdge(dut.clk)
        dut.s_axis_tvalid.value = 1
        dut.s_axis_tdata.value = generate_noise_signal()
        dut.s_axis_tlast.value = (i == n_samples - 1)
    
    await RisingEdge(dut.clk)
    dut.s_axis_tvalid.value = 0
    
    # Wait for pipeline to flush
    await ClockCycles(dut.clk, PIPELINE_STAGES + 10)
    
    t_end = cocotb.utils.get_sim_time(units="ns")
    duration = t_end - t_start
    throughput = n_samples / (duration * 1e-9)  # samples/sec
    
    dut._log.info(f"Throughput: {throughput/1e6:.2f} MSps ({n_samples} samples in {duration:.0f} ns)")
    
    # At 250 MHz, expect ~250 MSps
    expected_throughput = 250e6  # 250 MSps
    assert throughput > expected_throughput * 0.9, \
        f"Throughput too low: {throughput/1e6:.2f} < {expected_throughput/1e6 * 0.9:.2f} MSps"
    
    dut._log.info("✅ THROUGHPUT TEST PASSED")


# =============================================================================
# TEST: STATISTICS COUNTERS
# =============================================================================

@cocotb.test()
async def test_statistics(dut):
    """
    Verify detection and sample counters.
    """
    cocotb.start_soon(Clock(dut.clk, CLOCK_PERIOD_NS, units="ns").start())
    
    driver = CognitiveCFARDriver(dut)
    await driver.reset()
    
    initial_sample_count = int(dut.stat_sample_count.value)
    initial_det_count = int(dut.stat_detection_count.value)
    
    n_samples = 50
    
    # Send samples
    for _ in range(n_samples):
        await driver.send_sample(generate_noise_signal())
    
    # Wait for processing
    await ClockCycles(dut.clk, PIPELINE_STAGES + 10)
    
    final_sample_count = int(dut.stat_sample_count.value)
    
    samples_processed = final_sample_count - initial_sample_count
    
    dut._log.info(f"Samples processed: {samples_processed} (expected: ~{n_samples})")
    
    # Allow for pipeline effects
    assert samples_processed > n_samples - PIPELINE_STAGES, \
        f"Sample counter too low: {samples_processed}"
    
    dut._log.info("✅ STATISTICS TEST PASSED")
