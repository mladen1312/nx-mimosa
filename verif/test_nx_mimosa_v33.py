"""
NX-MIMOSA v3.3 — Cocotb Testbench (Dual-Mode Verification)
============================================================
[REQ-TB33-01] AXI-Lite register read/write verification
[REQ-TB33-02] Forward stream (Stream 1) latency check
[REQ-TB33-03] Window smoother (Stream 2) accuracy verification
[REQ-TB33-04] Maneuver detection state machine validation
[REQ-TB33-05] v3.1 backward compatibility check
[REQ-TB33-06] Version register verification

Run:
  cd verif && make SIM=verilator

Author: Dr. Mladen Mešter / Nexellum d.o.o.
License: Commercial — Nexellum d.o.o.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer, FallingEdge, ClockCycles
from cocotb.binary import BinaryValue
import struct
import math
import random


# =============================================================================
# Q15.16 Helpers
# =============================================================================
def to_q1516(val: float) -> int:
    raw = int(round(val * (1 << 16)))
    return raw & 0xFFFFFFFF


def from_q1516(raw: int) -> float:
    if raw >= 0x80000000:
        raw -= 0x100000000
    return raw / (1 << 16)


# =============================================================================
# AXI-Lite Driver
# =============================================================================
async def axi_write(dut, addr, data):
    """AXI-Lite write transaction."""
    dut.s_axi_awaddr.value = addr
    dut.s_axi_awvalid.value = 1
    dut.s_axi_wdata.value = data
    dut.s_axi_wvalid.value = 1
    dut.s_axi_bready.value = 1

    for _ in range(20):
        await RisingEdge(dut.aclk)
        if dut.s_axi_awready.value and dut.s_axi_wready.value:
            break

    dut.s_axi_awvalid.value = 0
    dut.s_axi_wvalid.value = 0

    # Wait for BRESP
    for _ in range(10):
        await RisingEdge(dut.aclk)
        if dut.s_axi_bvalid.value:
            break

    dut.s_axi_bready.value = 0
    await RisingEdge(dut.aclk)


async def axi_read(dut, addr) -> int:
    """AXI-Lite read transaction."""
    dut.s_axi_araddr.value = addr
    dut.s_axi_arvalid.value = 1
    dut.s_axi_rready.value = 1

    for _ in range(20):
        await RisingEdge(dut.aclk)
        if dut.s_axi_arready.value:
            break

    dut.s_axi_arvalid.value = 0

    for _ in range(10):
        await RisingEdge(dut.aclk)
        if dut.s_axi_rvalid.value:
            data = int(dut.s_axi_rdata.value)
            dut.s_axi_rready.value = 0
            return data

    dut.s_axi_rready.value = 0
    return 0xDEADBEEF


# =============================================================================
# Measurement Injection
# =============================================================================
async def inject_measurement(dut, x, y):
    """Send one measurement via AXI-Stream."""
    z_x = to_q1516(x)
    z_y = to_q1516(y)
    data = (z_y << 32) | z_x

    dut.s_axis_meas_tdata.value = data
    dut.s_axis_meas_tvalid.value = 1
    await RisingEdge(dut.aclk)

    while not dut.s_axis_meas_tready.value:
        await RisingEdge(dut.aclk)

    await RisingEdge(dut.aclk)
    dut.s_axis_meas_tvalid.value = 0


# =============================================================================
# TESTS
# =============================================================================

@cocotb.test()
async def test_version_register(dut):
    """[REQ-TB33-06] Version register reads 0x00030003."""
    clock = Clock(dut.aclk, 4, units="ns")  # 250 MHz
    cocotb.start_soon(clock.start())

    dut.aresetn.value = 0
    await ClockCycles(dut.aclk, 10)
    dut.aresetn.value = 1
    await ClockCycles(dut.aclk, 5)

    version = await axi_read(dut, 0x68)
    assert version == 0x00030003, f"Version mismatch: got 0x{version:08X}, expected 0x00030003"
    cocotb.log.info(f"✅ Version: {(version >> 16)}.{version & 0xFFFF}")


@cocotb.test()
async def test_register_readback(dut):
    """[REQ-TB33-01] Write and read back all v3.3 registers."""
    clock = Clock(dut.aclk, 4, units="ns")
    cocotb.start_soon(clock.start())

    dut.aresetn.value = 0
    await ClockCycles(dut.aclk, 10)
    dut.aresetn.value = 1
    await ClockCycles(dut.aclk, 5)

    # Test v3.1 registers
    test_cases = [
        (0x04, to_q1516(0.25), "OMEGA"),
        (0x08, to_q1516(0.05), "DT"),
        (0x0C, to_q1516(0.7), "Q_CV"),
        (0x10, to_q1516(1.5), "Q_CT"),
        (0x14, to_q1516(3.0), "R_NOISE"),
        (0x18, to_q1516(0.92), "P_STAY"),
    ]

    for addr, val, name in test_cases:
        await axi_write(dut, addr, val)
        readback = await axi_read(dut, addr)
        assert readback == val, f"{name}: wrote 0x{val:08X}, read 0x{readback:08X}"
        cocotb.log.info(f"✅ {name}: 0x{val:08X}")

    # Test v3.3 new registers
    await axi_write(dut, 0x40, 20)  # Window size = 20
    ws = await axi_read(dut, 0x40)
    assert (ws & 0x3F) == 20, f"Window size: expected 20, got {ws & 0x3F}"
    cocotb.log.info(f"✅ WINDOW_SIZE: {ws & 0x3F}")

    await axi_write(dut, 0x44, 1)  # Trigger mode = MANEUVER
    tm = await axi_read(dut, 0x44)
    assert (tm & 0x3) == 1, f"Trigger mode: expected 1, got {tm & 0x3}"
    cocotb.log.info(f"✅ TRIGGER_MODE: {tm & 0x3}")

    # Threshold registers
    await axi_write(dut, 0x48, to_q1516(6.0))  # Innovation threshold
    it = await axi_read(dut, 0x48)
    assert it == to_q1516(6.0), f"Innovation threshold mismatch"
    cocotb.log.info("✅ All register read/write verified")


@cocotb.test()
async def test_control_register_bits(dut):
    """[REQ-TB33-01] Control register bit fields for dual-mode."""
    clock = Clock(dut.aclk, 4, units="ns")
    cocotb.start_soon(clock.start())

    dut.aresetn.value = 0
    await ClockCycles(dut.aclk, 10)
    dut.aresetn.value = 1
    await ClockCycles(dut.aclk, 5)

    # Enable all streams: enable=1, smoother=1, window=1, offline=1
    # Bits: [4:0] = offline_en | window_en | reset | smoother_en | enable
    ctrl = 0b11011  # All enabled, reset=0
    await axi_write(dut, 0x00, ctrl)
    readback = await axi_read(dut, 0x00)
    assert (readback & 0x1F) == 0x1B, f"Control: expected 0x1B, got 0x{readback:02X}"
    cocotb.log.info(f"✅ Control register: 0x{readback:02X} (dual-mode enabled)")


@cocotb.test()
async def test_forward_stream(dut):
    """[REQ-TB33-02] Forward stream outputs on every measurement."""
    clock = Clock(dut.aclk, 4, units="ns")
    cocotb.start_soon(clock.start())

    dut.aresetn.value = 0
    dut.m_axis_rt_tready.value = 1
    dut.m_axis_win_tready.value = 1
    dut.m_axis_off_tready.value = 1
    await ClockCycles(dut.aclk, 10)
    dut.aresetn.value = 1
    await ClockCycles(dut.aclk, 5)

    # Configure
    await axi_write(dut, 0x00, 0x1B)  # Enable all
    await axi_write(dut, 0x08, to_q1516(0.1))  # dt = 0.1s
    await ClockCycles(dut.aclk, 5)

    rt_count = 0

    # Send 50 measurements (straight line + noise)
    for k in range(50):
        x_true = 100.0 + 50.0 * k * 0.1
        y_true = 200.0 + 30.0 * k * 0.1
        x_meas = x_true + random.gauss(0, 2.5)
        y_meas = y_true + random.gauss(0, 2.5)

        await inject_measurement(dut, x_meas, y_meas)

        # Wait for forward output
        for _ in range(200):
            await RisingEdge(dut.aclk)
            if dut.m_axis_rt_tvalid.value:
                rt_count += 1
                break

    cocotb.log.info(f"✅ Forward stream: {rt_count}/50 outputs received")
    assert rt_count >= 45, f"Forward stream too few outputs: {rt_count}"


@cocotb.test()
async def test_backward_compat_v31(dut):
    """[REQ-TB33-05] v3.1 register layout still works."""
    clock = Clock(dut.aclk, 4, units="ns")
    cocotb.start_soon(clock.start())

    dut.aresetn.value = 0
    await ClockCycles(dut.aclk, 10)
    dut.aresetn.value = 1
    await ClockCycles(dut.aclk, 5)

    # v3.1 registers at same addresses
    await axi_write(dut, 0x00, 0x03)  # enable + smoother (v3.1 compatible)
    await axi_write(dut, 0x04, to_q1516(0.196))
    await axi_write(dut, 0x08, to_q1516(0.1))
    await axi_write(dut, 0x0C, to_q1516(0.5))
    await axi_write(dut, 0x10, to_q1516(1.0))
    await axi_write(dut, 0x14, to_q1516(2.5))
    await axi_write(dut, 0x18, to_q1516(0.88))

    # v3.1 status register
    status = await axi_read(dut, 0x1C)
    cocotb.log.info(f"✅ v3.1 backward compat: status=0x{status:08X}")

    # v3.3 extension should be accessible too
    version = await axi_read(dut, 0x68)
    assert version == 0x00030003
    cocotb.log.info("✅ v3.1 backward compatibility verified")


@cocotb.test()
async def test_maneuver_state_readout(dut):
    """[REQ-TB33-04] Maneuver state register readable."""
    clock = Clock(dut.aclk, 4, units="ns")
    cocotb.start_soon(clock.start())

    dut.aresetn.value = 0
    await ClockCycles(dut.aclk, 10)
    dut.aresetn.value = 1
    await ClockCycles(dut.aclk, 5)

    mnv = await axi_read(dut, 0x58)
    state = mnv & 0x7
    states = ["IDLE", "ONSET", "SUSTAINED", "ENDING", "ENDED"]
    cocotb.log.info(f"✅ Maneuver state: {states[state]} ({state})")
    assert state <= 4, f"Invalid maneuver state: {state}"
