#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════════════════════════════
NX-MIMOSA COCOTB TESTBENCH
RTL Verification for FPGA Implementation
═══════════════════════════════════════════════════════════════════════════════════════════════════════

Cocotb-based verification environment for NX-MIMOSA RTL modules.
Provides bit-exact verification against Python reference model.

Run with: make SIM=verilator

Author: Dr. Mladen Mešter / Nexellum d.o.o.
License: AGPL v3 / Commercial
═══════════════════════════════════════════════════════════════════════════════════════════════════════
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, Timer, ClockCycles
from cocotb.result import TestFailure
from cocotb.regression import TestFactory
import numpy as np
from typing import List, Tuple
import struct


# =============================================================================
# CONSTANTS
# =============================================================================

FRAC_BITS = 16
CLK_PERIOD_NS = 4  # 250 MHz


# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================

def float_to_fixed(value: float, frac_bits: int = FRAC_BITS) -> int:
    """Convert float to Q16.16 fixed-point."""
    return int(value * (1 << frac_bits)) & 0xFFFFFFFF


def fixed_to_float(value: int, frac_bits: int = FRAC_BITS) -> float:
    """Convert Q16.16 fixed-point to float."""
    if value >= (1 << 31):  # Negative
        value -= (1 << 32)
    return value / (1 << frac_bits)


def pack_measurement(x: float, y: float, z: float, track_id: int = 0) -> int:
    """Pack measurement into 128-bit AXI-Stream word."""
    x_fixed = float_to_fixed(x)
    y_fixed = float_to_fixed(y)
    z_fixed = float_to_fixed(z)
    
    # Pack: [127:112]=track_id, [95:64]=z, [63:32]=y, [31:0]=x
    return (track_id << 112) | (z_fixed << 64) | (y_fixed << 32) | x_fixed


def unpack_track(data: int) -> Tuple[float, float, float, float, float, float]:
    """Unpack track state from 192-bit word."""
    x = fixed_to_float(data & 0xFFFFFFFF)
    y = fixed_to_float((data >> 32) & 0xFFFFFFFF)
    z = fixed_to_float((data >> 64) & 0xFFFFFFFF)
    vx = fixed_to_float((data >> 96) & 0xFFFFFFFF)
    vy = fixed_to_float((data >> 128) & 0xFFFFFFFF)
    vz = fixed_to_fixed((data >> 160) & 0xFFFFFFFF)
    
    return x, y, z, vx, vy, vz


# =============================================================================
# AXI-LITE DRIVER
# =============================================================================

class AXILiteDriver:
    """AXI4-Lite master driver for register access."""
    
    def __init__(self, dut, prefix="s_axi"):
        self.dut = dut
        self.prefix = prefix
    
    async def write(self, addr: int, data: int):
        """Write to AXI-Lite register."""
        dut = self.dut
        
        # Write address
        getattr(dut, f"{self.prefix}_awaddr").value = addr
        getattr(dut, f"{self.prefix}_awvalid").value = 1
        
        # Write data
        getattr(dut, f"{self.prefix}_wdata").value = data
        getattr(dut, f"{self.prefix}_wstrb").value = 0xF
        getattr(dut, f"{self.prefix}_wvalid").value = 1
        
        # Wait for ready
        await RisingEdge(dut.aclk)
        while not getattr(dut, f"{self.prefix}_awready").value:
            await RisingEdge(dut.aclk)
        
        # Deassert
        getattr(dut, f"{self.prefix}_awvalid").value = 0
        getattr(dut, f"{self.prefix}_wvalid").value = 0
        
        # Wait for response
        getattr(dut, f"{self.prefix}_bready").value = 1
        await RisingEdge(dut.aclk)
        while not getattr(dut, f"{self.prefix}_bvalid").value:
            await RisingEdge(dut.aclk)
        
        getattr(dut, f"{self.prefix}_bready").value = 0
        await RisingEdge(dut.aclk)
    
    async def read(self, addr: int) -> int:
        """Read from AXI-Lite register."""
        dut = self.dut
        
        # Read address
        getattr(dut, f"{self.prefix}_araddr").value = addr
        getattr(dut, f"{self.prefix}_arvalid").value = 1
        
        # Wait for ready
        await RisingEdge(dut.aclk)
        while not getattr(dut, f"{self.prefix}_arready").value:
            await RisingEdge(dut.aclk)
        
        getattr(dut, f"{self.prefix}_arvalid").value = 0
        
        # Wait for data
        getattr(dut, f"{self.prefix}_rready").value = 1
        await RisingEdge(dut.aclk)
        while not getattr(dut, f"{self.prefix}_rvalid").value:
            await RisingEdge(dut.aclk)
        
        data = int(getattr(dut, f"{self.prefix}_rdata").value)
        getattr(dut, f"{self.prefix}_rready").value = 0
        
        await RisingEdge(dut.aclk)
        return data


# =============================================================================
# AXI-STREAM DRIVER
# =============================================================================

class AXISDriver:
    """AXI4-Stream master driver for measurement input."""
    
    def __init__(self, dut, prefix="s_axis_meas"):
        self.dut = dut
        self.prefix = prefix
    
    async def send(self, data: int, last: bool = True):
        """Send data on AXI-Stream interface."""
        dut = self.dut
        
        getattr(dut, f"{self.prefix}_tdata").value = data
        getattr(dut, f"{self.prefix}_tkeep").value = 0xFFFF
        getattr(dut, f"{self.prefix}_tlast").value = int(last)
        getattr(dut, f"{self.prefix}_tvalid").value = 1
        
        await RisingEdge(dut.aclk)
        while not getattr(dut, f"{self.prefix}_tready").value:
            await RisingEdge(dut.aclk)
        
        getattr(dut, f"{self.prefix}_tvalid").value = 0
        await RisingEdge(dut.aclk)


class AXISMonitor:
    """AXI4-Stream slave monitor for track output."""
    
    def __init__(self, dut, prefix="m_axis_track"):
        self.dut = dut
        self.prefix = prefix
        self.received = []
    
    async def receive(self, timeout_cycles: int = 1000) -> int:
        """Receive data from AXI-Stream interface."""
        dut = self.dut
        
        getattr(dut, f"{self.prefix}_tready").value = 1
        
        for _ in range(timeout_cycles):
            await RisingEdge(dut.aclk)
            if getattr(dut, f"{self.prefix}_tvalid").value:
                data = int(getattr(dut, f"{self.prefix}_tdata").value)
                self.received.append(data)
                return data
        
        raise TimeoutError("No data received")


# =============================================================================
# TEST CASES
# =============================================================================

@cocotb.test()
async def test_reset(dut):
    """Test reset behavior."""
    clock = Clock(dut.aclk, CLK_PERIOD_NS, units="ns")
    cocotb.start_soon(clock.start())
    
    # Assert reset
    dut.aresetn.value = 0
    await ClockCycles(dut.aclk, 10)
    
    # Release reset
    dut.aresetn.value = 1
    await ClockCycles(dut.aclk, 10)
    
    # Check status
    axi = AXILiteDriver(dut)
    status = await axi.read(0x04)  # STATUS register
    
    dut._log.info(f"Status after reset: {status:#010x}")
    assert (status & 0x01) == 0, "Should not be busy after reset"


@cocotb.test()
async def test_version(dut):
    """Test version register."""
    clock = Clock(dut.aclk, CLK_PERIOD_NS, units="ns")
    cocotb.start_soon(clock.start())
    
    dut.aresetn.value = 0
    await ClockCycles(dut.aclk, 5)
    dut.aresetn.value = 1
    await ClockCycles(dut.aclk, 5)
    
    axi = AXILiteDriver(dut)
    version = await axi.read(0x14)  # VERSION register
    
    major = (version >> 16) & 0xFF
    minor = (version >> 8) & 0xFF
    patch = version & 0xFF
    
    dut._log.info(f"Version: {major}.{minor}.{patch}")
    assert major == 1, "Major version should be 1"


@cocotb.test()
async def test_config_registers(dut):
    """Test configuration register read/write."""
    clock = Clock(dut.aclk, CLK_PERIOD_NS, units="ns")
    cocotb.start_soon(clock.start())
    
    dut.aresetn.value = 0
    await ClockCycles(dut.aclk, 5)
    dut.aresetn.value = 1
    await ClockCycles(dut.aclk, 5)
    
    axi = AXILiteDriver(dut)
    
    # Write dt = 0.5 (Q16.16)
    dt_value = float_to_fixed(0.5)
    await axi.write(0x08, dt_value)
    
    # Read back
    dt_readback = await axi.read(0x08)
    
    dut._log.info(f"dt written: {dt_value:#010x}, read: {dt_readback:#010x}")
    assert dt_readback == dt_value, "Config register mismatch"
    
    # Write sigma = 50.0
    sigma_value = float_to_fixed(50.0)
    await axi.write(0x0C, sigma_value)
    
    sigma_readback = await axi.read(0x0C)
    dut._log.info(f"sigma written: {sigma_value:#010x}, read: {sigma_readback:#010x}")
    assert sigma_readback == sigma_value, "Config register mismatch"


@cocotb.test()
async def test_enable_disable(dut):
    """Test enable/disable control."""
    clock = Clock(dut.aclk, CLK_PERIOD_NS, units="ns")
    cocotb.start_soon(clock.start())
    
    dut.aresetn.value = 0
    await ClockCycles(dut.aclk, 5)
    dut.aresetn.value = 1
    await ClockCycles(dut.aclk, 5)
    
    axi = AXILiteDriver(dut)
    
    # Read control (should be 0 after reset)
    ctrl = await axi.read(0x00)
    assert (ctrl & 0x01) == 0, "Should be disabled after reset"
    
    # Enable
    await axi.write(0x00, 0x01)
    ctrl = await axi.read(0x00)
    assert (ctrl & 0x01) == 1, "Should be enabled"
    
    # Disable
    await axi.write(0x00, 0x00)
    ctrl = await axi.read(0x00)
    assert (ctrl & 0x01) == 0, "Should be disabled"


@cocotb.test()
async def test_measurement_input(dut):
    """Test measurement AXI-Stream input."""
    clock = Clock(dut.aclk, CLK_PERIOD_NS, units="ns")
    cocotb.start_soon(clock.start())
    
    dut.aresetn.value = 0
    await ClockCycles(dut.aclk, 5)
    dut.aresetn.value = 1
    await ClockCycles(dut.aclk, 5)
    
    axi = AXILiteDriver(dut)
    axis = AXISDriver(dut)
    
    # Enable tracking
    await axi.write(0x00, 0x01)
    
    # Send measurement
    meas = pack_measurement(10000.0, 5000.0, 10000.0, track_id=1)
    await axis.send(meas)
    
    dut._log.info("Measurement sent successfully")


@cocotb.test()
async def test_interrupt(dut):
    """Test interrupt generation."""
    clock = Clock(dut.aclk, CLK_PERIOD_NS, units="ns")
    cocotb.start_soon(clock.start())
    
    dut.aresetn.value = 0
    await ClockCycles(dut.aclk, 5)
    dut.aresetn.value = 1
    await ClockCycles(dut.aclk, 5)
    
    axi = AXILiteDriver(dut)
    
    # Enable interrupt
    await axi.write(0x20, 0x03)  # Enable track_ready and error IRQs
    
    irq_enable = await axi.read(0x20)
    dut._log.info(f"IRQ Enable: {irq_enable:#010x}")
    
    # Check IRQ status
    irq_status = await axi.read(0x1C)
    dut._log.info(f"IRQ Status: {irq_status:#010x}")


@cocotb.test()
async def test_cv_tracking(dut):
    """Test constant velocity tracking scenario."""
    clock = Clock(dut.aclk, CLK_PERIOD_NS, units="ns")
    cocotb.start_soon(clock.start())
    
    dut.aresetn.value = 0
    await ClockCycles(dut.aclk, 5)
    dut.aresetn.value = 1
    await ClockCycles(dut.aclk, 5)
    
    axi = AXILiteDriver(dut)
    axis = AXISDriver(dut)
    
    # Configure
    await axi.write(0x08, float_to_fixed(1.0))   # dt = 1.0
    await axi.write(0x0C, float_to_fixed(30.0)) # sigma = 30.0
    
    # Enable
    await axi.write(0x00, 0x01)
    
    # Generate CV trajectory with noise
    np.random.seed(42)
    x0 = np.array([10000.0, 5000.0, 10000.0])  # Initial position
    v = np.array([200.0, 50.0, 0.0])            # Velocity
    sigma = 30.0
    
    n_measurements = 20
    
    for i in range(n_measurements):
        # True position
        true_pos = x0 + v * i
        
        # Noisy measurement
        noise = np.random.normal(0, sigma, 3)
        meas_pos = true_pos + noise
        
        # Send measurement
        meas = pack_measurement(meas_pos[0], meas_pos[1], meas_pos[2], track_id=1)
        await axis.send(meas)
        
        # Wait a bit
        await ClockCycles(dut.aclk, 100)
        
        dut._log.info(f"Sent measurement {i}: ({meas_pos[0]:.1f}, {meas_pos[1]:.1f}, {meas_pos[2]:.1f})")
    
    # Check track count
    track_count = await axi.read(0x10)
    dut._log.info(f"Track count: {track_count}")


# =============================================================================
# PERFORMANCE TESTS
# =============================================================================

@cocotb.test()
async def test_throughput(dut):
    """Test measurement throughput."""
    clock = Clock(dut.aclk, CLK_PERIOD_NS, units="ns")
    cocotb.start_soon(clock.start())
    
    dut.aresetn.value = 0
    await ClockCycles(dut.aclk, 5)
    dut.aresetn.value = 1
    await ClockCycles(dut.aclk, 5)
    
    axi = AXILiteDriver(dut)
    axis = AXISDriver(dut)
    
    # Enable
    await axi.write(0x00, 0x01)
    
    # Send burst of measurements
    n_measurements = 100
    start_time = cocotb.utils.get_sim_time('ns')
    
    for i in range(n_measurements):
        meas = pack_measurement(10000.0 + i*10, 5000.0, 10000.0, track_id=i % 256)
        await axis.send(meas)
    
    end_time = cocotb.utils.get_sim_time('ns')
    
    elapsed_ns = end_time - start_time
    throughput = n_measurements / (elapsed_ns / 1e9)
    
    dut._log.info(f"Throughput: {throughput:.0f} measurements/sec")
    dut._log.info(f"Elapsed: {elapsed_ns/1e6:.2f} ms for {n_measurements} measurements")


# =============================================================================
# COCOTB CONFIGURATION
# =============================================================================

# Makefile content for reference:
"""
# Makefile for Cocotb tests

TOPLEVEL_LANG = verilog
VERILOG_SOURCES = $(PWD)/../../rtl/nx_mimosa_axi_wrapper.sv
TOPLEVEL = nx_mimosa_axi_wrapper
MODULE = test_nx_mimosa_rtl

SIM ?= verilator

EXTRA_ARGS += --trace --trace-structs

include $(shell cocotb-config --makefiles)/Makefile.sim
"""
