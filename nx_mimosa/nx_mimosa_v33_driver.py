#!/usr/bin/env python3
"""
NX-MIMOSA v3.3 — Register Map Auto-Gen & Python Driver
=======================================================
Generated from: config/nx_mimosa_v33_regmap.yaml (SSOT)
Target: ZU48DR RFSoC via /dev/mem or UIO

[REQ-DRIVER-01] Python driver for AXI-Lite register access
[REQ-DRIVER-02] Q15.16 fixed-point conversion helpers
[REQ-DRIVER-03] Dual-mode configuration API
[REQ-DRIVER-04] C header auto-generation

Author: Dr. Mladen Mešter / Nexellum d.o.o.
License: Commercial — Nexellum d.o.o.
"""

import struct
import mmap
import os
import yaml
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional


# =============================================================================
# Register Addresses (auto-generated from YAML)
# =============================================================================
class Reg:
    """v3.3 Register Map — matches nx_mimosa_v33_regmap.yaml"""
    CONTROL         = 0x00
    OMEGA           = 0x04
    DT              = 0x08
    Q_CV            = 0x0C
    Q_CT            = 0x10
    R_NOISE         = 0x14
    P_STAY          = 0x18
    STATUS          = 0x1C
    X_INIT_0        = 0x20
    X_INIT_1        = 0x24
    X_INIT_2        = 0x28
    X_INIT_3        = 0x2C
    P_INIT_0        = 0x30
    P_INIT_1        = 0x34
    P_INIT_2        = 0x38
    P_INIT_3        = 0x3C
    # v3.3 NEW
    WINDOW_SIZE     = 0x40
    TRIGGER_MODE    = 0x44
    INNOV_THRESHOLD = 0x48
    COV_THRESHOLD   = 0x4C
    MANUAL_TRIGGER  = 0x50
    WINDOW_COUNT    = 0x54
    MANEUVER_STATE  = 0x58
    FWD_CYCLES      = 0x5C
    SMOOTH_CYCLES   = 0x60
    TRACK_FILL      = 0x64
    VERSION         = 0x68


class TriggerMode(IntEnum):
    SLIDING   = 0  # Continuous: smooth last N every step
    MANEUVER  = 1  # Trigger on maneuver end
    COV_SPIKE = 2  # Trigger on covariance spike
    MANUAL    = 3  # Software trigger via MANUAL_TRIGGER register


class ManeuverState(IntEnum):
    IDLE      = 0
    ONSET     = 1
    SUSTAINED = 2
    ENDING    = 3
    ENDED     = 4


# =============================================================================
# Q15.16 Fixed-Point Helpers
# =============================================================================
def float_to_q1516(val: float) -> int:
    """Convert float to Q15.16 32-bit signed."""
    raw = int(round(val * (1 << 16)))
    return raw & 0xFFFFFFFF


def q1516_to_float(raw: int) -> float:
    """Convert Q15.16 32-bit signed to float."""
    if raw >= 0x80000000:
        raw -= 0x100000000
    return raw / (1 << 16)


# =============================================================================
# NX-MIMOSA v3.3 Driver
# =============================================================================
class NxMimosaV33:
    """
    Python driver for NX-MIMOSA v3.3 dual-mode tracker.
    
    Three output streams:
      1. Real-time (forward IMM) — 0 latency, ~8.66m RMSE
      2. Window-smoothed (RTS) — Ndt latency, ~4.22m RMSE (Window-30)
      3. Offline (full-track RTS) — full latency, ~4.18m RMSE
    
    Usage:
        tracker = NxMimosaV33(base_addr=0x40000000)
        tracker.configure(dt=0.1, omega=0.196, window_size=30)
        tracker.set_trigger_mode(TriggerMode.SLIDING)
        tracker.enable()
        
        # Read status
        print(tracker.get_version())
        print(tracker.get_maneuver_state())
        print(tracker.get_performance_counters())
    """

    def __init__(self, base_addr: int = 0x40000000, uio_device: Optional[str] = None):
        """
        Initialize driver.
        
        Args:
            base_addr: AXI-Lite base address (default 0x40000000)
            uio_device: UIO device path (e.g., '/dev/uio0') — if None, uses /dev/mem
        """
        self.base_addr = base_addr
        self._mm = None
        
        if uio_device:
            fd = os.open(uio_device, os.O_RDWR | os.O_SYNC)
            self._mm = mmap.mmap(fd, 0x1000, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
            os.close(fd)
        else:
            fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
            self._mm = mmap.mmap(fd, 0x1000, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE,
                                 offset=base_addr)
            os.close(fd)

    def _write32(self, offset: int, value: int):
        struct.pack_into('<I', self._mm, offset, value & 0xFFFFFFFF)

    def _read32(self, offset: int) -> int:
        return struct.unpack_from('<I', self._mm, offset)[0]

    # =========================================================================
    # Configuration
    # =========================================================================
    def configure(self, dt: float = 0.1, omega: float = 0.196,
                  q_cv: float = 0.5, q_ct: float = 1.0,
                  r: float = 2.5, p_stay: float = 0.88,
                  window_size: int = 30):
        """Configure all tracker parameters in one call."""
        self._write32(Reg.DT, float_to_q1516(dt))
        self._write32(Reg.OMEGA, float_to_q1516(omega))
        self._write32(Reg.Q_CV, float_to_q1516(q_cv))
        self._write32(Reg.Q_CT, float_to_q1516(q_ct))
        self._write32(Reg.R_NOISE, float_to_q1516(r))
        self._write32(Reg.P_STAY, float_to_q1516(p_stay))
        self._write32(Reg.WINDOW_SIZE, min(window_size, 64))

    def set_trigger_mode(self, mode: TriggerMode):
        """Set window smoother trigger mode."""
        self._write32(Reg.TRIGGER_MODE, int(mode))

    def set_thresholds(self, innov_threshold: float = 4.0, cov_threshold: float = 100.0):
        """Set maneuver detection thresholds."""
        self._write32(Reg.INNOV_THRESHOLD, float_to_q1516(innov_threshold))
        self._write32(Reg.COV_THRESHOLD, float_to_q1516(cov_threshold))

    def enable(self, forward: bool = True, window: bool = True, offline: bool = True):
        """Enable/disable output streams."""
        ctrl = (int(forward)) | (int(window) << 1) | (int(offline) << 4) | (int(window) << 3)
        self._write32(Reg.CONTROL, ctrl)

    def reset(self):
        """Soft reset — auto-clears."""
        ctrl = self._read32(Reg.CONTROL)
        self._write32(Reg.CONTROL, ctrl | 0x04)

    def trigger_smooth(self):
        """Manual trigger for window smoother (when mode=MANUAL)."""
        self._write32(Reg.MANUAL_TRIGGER, 1)

    # =========================================================================
    # Status & Diagnostics
    # =========================================================================
    def get_version(self) -> str:
        """Read version register. Returns e.g., '3.3'."""
        raw = self._read32(Reg.VERSION)
        major = (raw >> 16) & 0xFFFF
        minor = raw & 0xFFFF
        return f"{major}.{minor}"

    def get_status(self) -> dict:
        """Read status register."""
        raw = self._read32(Reg.STATUS)
        return {
            'dominant_mode': ['CV', 'CT+', 'CT-'][raw & 0x7],
            'track_initialized': bool(raw >> 31),
        }

    def get_maneuver_state(self) -> ManeuverState:
        """Read maneuver detector state."""
        raw = self._read32(Reg.MANEUVER_STATE) & 0x7
        return ManeuverState(raw)

    def get_performance_counters(self) -> dict:
        """Read all performance counters."""
        return {
            'fwd_cycles': self._read32(Reg.FWD_CYCLES),
            'smooth_cycles': self._read32(Reg.SMOOTH_CYCLES),
            'window_count': self._read32(Reg.WINDOW_COUNT),
            'track_fill': self._read32(Reg.TRACK_FILL),
        }

    def get_window_size(self) -> int:
        return self._read32(Reg.WINDOW_SIZE) & 0x3F

    def close(self):
        if self._mm:
            self._mm.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


# =============================================================================
# C Header Auto-Generator
# =============================================================================
def generate_c_header(yaml_path: str, output_path: str):
    """Generate C header from YAML register map (SSOT)."""
    with open(yaml_path, 'r') as f:
        regmap = yaml.safe_load(f)

    mod = regmap['module']
    lines = [
        f"/* Auto-generated from {yaml_path} — DO NOT EDIT */",
        f"/* {mod['description']} v{mod['version']} */",
        f"/* (c) Nexellum d.o.o. — Commercial License */",
        "",
        f"#ifndef __{mod['name'].upper()}_REGS_H__",
        f"#define __{mod['name'].upper()}_REGS_H__",
        "",
        "#include <stdint.h>",
        "",
        f"#define {mod['name'].upper()}_BASE_ADDR  0x{mod['base_addr']:08X}U",
        f"#define {mod['name'].upper()}_VERSION     0x{regmap['registers'][-1].get('reset', 0):08X}U",
        "",
    ]

    for reg in regmap['registers']:
        name = reg['name']
        offset = reg['offset']
        access = reg.get('access', 'RW')
        desc = reg.get('description', '')
        lines.append(f"/* {desc} ({access}) */")
        lines.append(f"#define REG_{name:<24s} 0x{offset:02X}U")

        if 'fields' in reg:
            for field in reg['fields']:
                fname = field['name']
                hi, lo = field['bits']
                mask = ((1 << (hi - lo + 1)) - 1) << lo
                lines.append(f"#define REG_{name}_{fname.upper()}_MASK  0x{mask:08X}U")
                lines.append(f"#define REG_{name}_{fname.upper()}_SHIFT {lo}")
        lines.append("")

    # Helper macros
    lines += [
        "/* Q15.16 conversion */",
        "#define FP_TO_Q1516(f)   ((int32_t)((f) * 65536.0f))",
        "#define Q1516_TO_FP(q)   ((float)(q) / 65536.0f)",
        "",
        "/* Register access (assumes volatile pointer) */",
        f"#define MIMOSA_WR(off, val) \\",
        f"    (*(volatile uint32_t *)({mod['name'].upper()}_BASE_ADDR + (off)) = (val))",
        f"#define MIMOSA_RD(off) \\",
        f"    (*(volatile uint32_t *)({mod['name'].upper()}_BASE_ADDR + (off)))",
        "",
        f"#endif /* __{mod['name'].upper()}_REGS_H__ */",
    ]

    with open(output_path, 'w') as f:
        f.write('\n'.join(lines))
    print(f"Generated: {output_path} ({len(lines)} lines)")


# =============================================================================
# Device Tree Overlay Auto-Generator
# =============================================================================
def generate_dts_overlay(yaml_path: str, output_path: str):
    """Generate Device Tree overlay from YAML register map."""
    with open(yaml_path, 'r') as f:
        regmap = yaml.safe_load(f)

    mod = regmap['module']
    base = mod['base_addr']

    dts = f"""\
/* Auto-generated from {yaml_path} — DO NOT EDIT */
/* NX-MIMOSA v3.3 Device Tree Overlay */
/* (c) Nexellum d.o.o. */

/dts-v1/;
/plugin/;

/ {{
    compatible = "xlnx,zynqmp";

    fragment@0 {{
        target = <&amba>;
        __overlay__ {{
            nx_mimosa_v33: nx-mimosa@{base:08x} {{
                compatible = "nexellum,nx-mimosa-3.3";
                reg = <0x0 0x{base:08x} 0x0 0x1000>;
                interrupt-parent = <&gic>;
                interrupts = <0 89 4>;  /* SPI 89, level high */
                clocks = <&zynqmp_clk 71>;
                clock-names = "aclk";

                /* Stream DMA channels */
                dma-names = "meas_in", "rt_out", "win_out", "off_out";

                /* NX-MIMOSA v3.3 specific */
                nexellum,version = <0x00030003>;
                nexellum,window-size-default = <30>;
                nexellum,max-targets = <8>;
                nexellum,state-dim = <4>;
                nexellum,n-models = <3>;
            }};
        }};
    }};
}};
"""

    with open(output_path, 'w') as f:
        f.write(dts)
    print(f"Generated: {output_path}")


# =============================================================================
# CLI
# =============================================================================
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="NX-MIMOSA v3.3 Register Tools")
    parser.add_argument("--gen-c", metavar="OUTPUT", help="Generate C header")
    parser.add_argument("--gen-dts", metavar="OUTPUT", help="Generate DTS overlay")
    parser.add_argument("--yaml", default="config/nx_mimosa_v33_regmap.yaml",
                        help="Input YAML register map")
    args = parser.parse_args()

    if args.gen_c:
        generate_c_header(args.yaml, args.gen_c)
    if args.gen_dts:
        generate_dts_overlay(args.yaml, args.gen_dts)
    if not args.gen_c and not args.gen_dts:
        print("NX-MIMOSA v3.3 Driver — use --gen-c or --gen-dts for code generation")
        print(f"Registers: {len(Reg.__dict__) - 2} defined")
        print(f"Trigger modes: {[m.name for m in TriggerMode]}")
