# ═══════════════════════════════════════════════════════════════════════════════
# NX-MIMOSA INTEGRATION GUIDE
# Complete Integration Manual for All Supported Domains
# ═══════════════════════════════════════════════════════════════════════════════

## Table of Contents

1. [Overview](#1-overview)
2. [Quick Start](#2-quick-start)
3. [Aviation (ATC/ATM) Integration](#3-aviation-atcatm-integration)
4. [Automotive (ADAS) Integration](#4-automotive-adas-integration)
5. [Defense Integration](#5-defense-integration)
6. [Space (SSA) Integration](#6-space-ssa-integration)
7. [Maritime (VTS) Integration](#7-maritime-vts-integration)
8. [FPGA Deployment](#8-fpga-deployment)
9. [Performance Tuning](#9-performance-tuning)
10. [Troubleshooting](#10-troubleshooting)

---

## 1. Overview

### 1.1 System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         NX-MIMOSA SYSTEM ARCHITECTURE                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│    SENSORS              PROCESSING                    OUTPUTS               │
│                                                                             │
│  ┌─────────┐          ┌─────────────────┐          ┌─────────────────┐     │
│  │ Radar   │─────────>│                 │─────────>│ ASTERIX CAT062  │     │
│  │ PSR/SSR │          │   NX-MIMOSA     │          │ (Aviation)      │     │
│  └─────────┘          │                 │          └─────────────────┘     │
│                       │  ┌───────────┐  │                                   │
│  ┌─────────┐          │  │  VS-IMM   │  │          ┌─────────────────┐     │
│  │ ADS-B   │─────────>│  │  UKF/CKF  │  │─────────>│ CAN-FD          │     │
│  │         │          │  │  JPDA/MHT │  │          │ (Automotive)    │     │
│  └─────────┘          │  └───────────┘  │          └─────────────────┘     │
│                       │                 │                                   │
│  ┌─────────┐          │  ┌───────────┐  │          ┌─────────────────┐     │
│  │ Camera  │─────────>│  │   ECCM    │  │─────────>│ Link-16         │     │
│  │ Lidar   │          │  │   Suite   │  │          │ (Defense)       │     │
│  └─────────┘          │  └───────────┘  │          └─────────────────┘     │
│                       │                 │                                   │
│  ┌─────────┐          └─────────────────┘          ┌─────────────────┐     │
│  │ Optical │                   │                   │ CCSDS           │     │
│  │ Sensors │───────────────────┼──────────────────>│ (Space)         │     │
│  └─────────┘                   │                   └─────────────────┘     │
│                                │                                            │
│                                │                   ┌─────────────────┐     │
│                                └──────────────────>│ NMEA 2000       │     │
│                                                    │ (Maritime)      │     │
│                                                    └─────────────────┘     │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 Supported Platforms

| Platform | Software | FPGA |
|----------|----------|------|
| Linux (x86_64) | ✅ | - |
| Linux (ARM64) | ✅ | - |
| Windows | ✅ | - |
| RFSoC 4x2 | ✅ | ✅ |
| ZCU208 | ✅ | ✅ |
| Zynq UltraScale+ | ✅ | ✅ |

---

## 2. Quick Start

### 2.1 Installation

```bash
# Clone repository
git clone https://github.com/mladen1312/nx-mimosa.git
cd nx-mimosa

# Install Python package
pip install -e .

# Or install from PyPI (when available)
pip install nx-mimosa
```

### 2.2 Basic Usage

```python
import numpy as np
from python.nx_mimosa_v41_atc import NXMIMOSAAtc

# Create tracker
tracker = NXMIMOSAAtc(dt=1.0, sigma=30.0)

# Initialize with first measurement
z0 = np.array([50000, 30000, 10668])  # Position [x, y, z] meters
v0 = np.array([232, 0, 0])             # Velocity [vx, vy, vz] m/s
tracker.initialize(z0, v0)

# Tracking loop
for measurement in sensor_data:
    # Predict
    tracker.predict(dt=1.0)
    
    # Update with measurement
    state = tracker.update(measurement, sigma=30.0)
    
    # Get results
    position = tracker.get_position()
    velocity = tracker.get_velocity()
    mode_probs = tracker.get_mode_probabilities()
    
    print(f"Position: {position}")
    print(f"Velocity: {velocity}")
    print(f"Mode: CV={mode_probs[0]:.2f}, CT1={mode_probs[1]:.2f}, CT2={mode_probs[2]:.2f}")
```

---

## 3. Aviation (ATC/ATM) Integration

### 3.1 EUROCONTROL ARTAS Integration

```python
from python.output.asterix_cat062_formatter import NXMIMOSAAsterixOutput
from python.nx_mimosa_v41_atc import NXMIMOSAAtc
import socket

# Create tracker and formatter
tracker = NXMIMOSAAtc(dt=4.0, sigma=50.0)  # 4 second update rate
asterix = NXMIMOSAAsterixOutput(sac=0, sic=1)  # Your SAC/SIC

# ARTAS connection
artas_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
artas_address = ("artas.eurocontrol.int", 8600)

# Processing loop
for radar_plot in radar_input:
    # Update tracker
    tracker.predict(dt=4.0)
    state = tracker.update(radar_plot, sigma=50.0)
    
    # Format as ASTERIX CAT062
    track_data = asterix.from_tracker(
        tracker,
        track_id=radar_plot.track_id,
        callsign=radar_plot.callsign,
        mode_3a=radar_plot.mode_3a
    )
    
    # Encode and send
    asterix_bytes = asterix.encode(track_data)
    artas_socket.sendto(asterix_bytes, artas_address)
```

### 3.2 Multi-Sensor Fusion Configuration

```python
# Configure for PSR + SSR + ADS-B fusion
tracker_config = {
    'sensors': {
        'psr': {'sigma': 150.0, 'update_rate': 4.0},   # Primary radar
        'ssr': {'sigma': 50.0, 'update_rate': 4.0},    # Secondary radar  
        'adsb': {'sigma': 10.0, 'update_rate': 1.0}    # ADS-B
    },
    'fusion': {
        'method': 'adaptive_weighted',
        'gate_probability': 0.9999
    }
}
```

### 3.3 Performance Requirements Verification

```python
# EUROCONTROL EASSP compliance check
def verify_eassp_compliance(tracker, test_data):
    results = {
        'en_route_rms': calculate_rms(tracker, test_data['en_route']),
        'tma_rms': calculate_rms(tracker, test_data['tma']),
        'continuity': calculate_continuity(tracker, test_data),
    }
    
    # Requirements
    assert results['en_route_rms'] <= 500, f"En-route RMS {results['en_route_rms']} > 500m"
    assert results['tma_rms'] <= 150, f"TMA RMS {results['tma_rms']} > 150m"
    assert results['continuity'] >= 0.995, f"Continuity {results['continuity']} < 99.5%"
    
    return results
```

---

## 4. Automotive (ADAS) Integration

### 4.1 CAN-FD Bus Configuration

```python
from python.output.canfd_automotive_formatter import NXMIMOSACANOutput
import can

# Initialize CAN-FD interface
bus = can.interface.Bus(channel='can0', bustype='socketcan', fd=True)

# Create formatter
canfd = NXMIMOSACANOutput(
    use_fd=True,
    base_object_id=0x300,
    base_quality_id=0x400,
    status_id=0x500
)

# Processing loop (20 Hz typical for automotive radar)
while True:
    # Get tracked objects from tracker
    objects = []
    for track_id, tracker in trackers.items():
        obj = canfd.from_tracker(tracker, track_id)
        objects.append(obj)
    
    # Encode to CAN messages
    messages = canfd.encode_all(objects)
    
    # Send on CAN bus
    for msg in messages:
        can_msg = can.Message(
            arbitration_id=msg.can_id,
            data=msg.data,
            is_fd=True
        )
        bus.send(can_msg)
```

### 4.2 AUTOSAR Integration

```c
// AUTOSAR SWC integration example
#include "Rte_NxMimosa.h"

void NxMimosa_MainFunction(void)
{
    Std_ReturnType ret;
    NxMimosa_RadarObjectList objectList;
    
    // Read radar measurements from RTE
    ret = Rte_Read_RadarMeasurements(&measurements);
    
    // Process through NX-MIMOSA
    NxMimosa_Process(&measurements, &objectList);
    
    // Write object list to RTE
    ret = Rte_Write_RadarObjectList(&objectList);
    
    // Write diagnostics
    NxMimosa_Diagnostics diag;
    NxMimosa_GetDiagnostics(&diag);
    Rte_Write_DiagnosticInfo(&diag);
}
```

### 4.3 Object Classification

```python
# Classification thresholds for automotive
CLASSIFICATION_RULES = {
    'pedestrian': {
        'rcs_range': (-15, 0),      # dBsm
        'speed_max': 2.0,            # m/s
        'height_range': (0.5, 2.0)   # m
    },
    'bicycle': {
        'rcs_range': (-10, 5),
        'speed_max': 10.0,
        'height_range': (0.8, 1.8)
    },
    'car': {
        'rcs_range': (5, 20),
        'speed_range': (0, 50),
        'length_range': (3.5, 5.5)
    },
    'truck': {
        'rcs_range': (15, 30),
        'length_min': 6.0
    }
}
```

---

## 5. Defense Integration

### 5.1 Link-16 TADIL-J Configuration

```python
from python.output.link16_defense_formatter import (
    NXMIMOSALink16Output, TrackEnvironment, TrackIdentity
)

# Initialize Link-16 output
link16 = NXMIMOSALink16Output(
    own_track_number=1,  # Platform track number
    own_lat=45.0,        # Platform position
    own_lon=16.0
)

# Process tracks
for track_id, tracker in air_tracks.items():
    # Convert to Link-16 format
    l16_track = link16.from_tracker(
        tracker,
        track_id,
        environment=TrackEnvironment.AIR,
        identity=TrackIdentity.UNKNOWN  # IFF will update
    )
    
    # Encode J2.2 message
    messages = link16.encode([l16_track])
    
    # Send via tactical data link
    for msg in messages:
        tdl_interface.send(msg)

# Send PPLI (own position)
ppli_msg = link16.encode_ppli(
    lat=own_lat, lon=own_lon, alt=own_alt,
    speed=own_speed, heading=own_heading
)
tdl_interface.send(ppli_msg)
```

### 5.2 ECCM Configuration

```python
from python.eccm.fpga_frequency_agility import (
    NXMIMOSAFrequencyAgility, FrequencyAgilityConfig
)

# Configure frequency agility for DRFM countermeasure
eccm_config = FrequencyAgilityConfig(
    center_frequency=9.5e9,      # X-band
    bandwidth=1.0e9,             # 1 GHz hopping
    n_channels=128,
    hop_interval=50e-6,          # 50 µs
    pattern_type='aes',          # Cryptographic pattern
    seed=classified_seed,        # From key management
    min_hop_distance=16
)

freq_agility = NXMIMOSAFrequencyAgility(eccm_config)

# In radar processing loop
tx_freq = freq_agility.get_transmit_frequency()
# ... transmit pulse at tx_freq ...

# On receive
valid, corrected_range, confidence = freq_agility.process_return(
    measured_range, doppler, received_freq, timestamp
)

if not valid:
    print(f"RGPO jammer detected! Confidence: {confidence}")
```

---

## 6. Space (SSA) Integration

### 6.1 CCSDS Telemetry Output

```python
from python.output.ccsds_space_formatter import NXMIMOSACCSDSOutput

# Configure CCSDS formatter
ccsds = NXMIMOSACCSDSOutput(
    spacecraft_id=0x1234,
    apid=0x100
)

# Process space objects
for obj_id, tracker in space_objects.items():
    # Get orbital elements
    state = tracker.get_state()
    
    # Format as CCSDS packet
    packet = ccsds.format_orbit_message(
        object_id=obj_id,
        epoch=current_epoch,
        state_vector=state,
        covariance=tracker.get_covariance()
    )
    
    # Send via ground station link
    ground_link.send(packet)
```

### 6.2 TLE Generation

```python
# Generate Two-Line Elements from tracked state
def state_to_tle(tracker, norad_id, name):
    state = tracker.get_state()  # [x, y, z, vx, vy, vz] ECI
    
    # Convert to Keplerian elements
    elements = cartesian_to_keplerian(state, mu_earth)
    
    # Format TLE
    tle_line1 = format_tle_line1(norad_id, elements, epoch)
    tle_line2 = format_tle_line2(elements)
    
    return f"{name}\n{tle_line1}\n{tle_line2}"
```

---

## 7. Maritime (VTS) Integration

### 7.1 NMEA 2000 Configuration

```python
from python.output.nmea2000_maritime_formatter import NXMIMOSANMEA2000Output

# Initialize NMEA 2000 output
nmea = NXMIMOSANMEA2000Output(source_address=10)

# Process vessel tracks
for vessel_id, tracker in vessel_tracks.items():
    # Format as NMEA 2000 PGN
    messages = nmea.format_target_data(
        tracker,
        target_id=vessel_id,
        mmsi=vessel_mmsi.get(vessel_id)
    )
    
    # Send on NMEA 2000 bus
    for msg in messages:
        nmea_bus.send(msg)
```

---

## 8. FPGA Deployment

### 8.1 RFSoC Integration

```tcl
# Vivado TCL script for NX-MIMOSA integration

# Create block design
create_bd_design "nx_mimosa_system"

# Add Zynq UltraScale+ MPSoC
create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e:3.4 zynq_ultra_ps_e_0

# Add NX-MIMOSA core
add_files -norecurse {
    rtl/nx_mimosa_axi_wrapper.sv
    rtl/nx_mimosa_top.sv
    rtl/ukf_core.sv
    rtl/imm_core.sv
    rtl/frequency_agility_controller.sv
}

# Create hierarchy
create_bd_cell -type module -reference nx_mimosa_axi_wrapper nx_mimosa_0

# Connect AXI interfaces
connect_bd_intf_net [get_bd_intf_pins zynq_ultra_ps_e_0/M_AXI_HPM0_FPD] \
                    [get_bd_intf_pins nx_mimosa_0/s_axi]

# Connect AXI-Stream for ADC data
connect_bd_intf_net [get_bd_intf_pins rf_data_converter_0/m_axis] \
                    [get_bd_intf_pins nx_mimosa_0/s_axis_meas]
```

### 8.2 Device Tree Overlay

```dts
/dts-v1/;
/plugin/;

/ {
    fragment@0 {
        target = <&fpga_full>;
        __overlay__ {
            #address-cells = <2>;
            #size-cells = <2>;
            
            nx_mimosa_0: nx_mimosa@a0000000 {
                compatible = "nexellum,nx-mimosa-1.0";
                reg = <0x0 0xa0000000 0x0 0x10000>;
                interrupts = <0 89 4>;
                interrupt-parent = <&gic>;
                
                clocks = <&zynqmp_clk 71>;
                clock-names = "aclk";
                
                nexellum,max-tracks = <256>;
                nexellum,update-rate-hz = <100>;
            };
        };
    };
};
```

### 8.3 Linux Driver Usage

```c
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>

// Register access
#define NX_MIMOSA_CTRL      0x00
#define NX_MIMOSA_STATUS    0x04
#define NX_MIMOSA_TRACK_CNT 0x08

static void nx_mimosa_enable(void __iomem *base)
{
    writel(0x01, base + NX_MIMOSA_CTRL);
}

static u32 nx_mimosa_get_track_count(void __iomem *base)
{
    return readl(base + NX_MIMOSA_TRACK_CNT);
}
```

---

## 9. Performance Tuning

### 9.1 Filter Parameters

```python
# Optimal parameters for different scenarios

# En-route ATC (stable, high-altitude)
ENROUTE_PARAMS = {
    'process_noise': [0.01, 0.1, 0.5],  # CV, CT1, CT2
    'transition_matrix': [
        [0.98, 0.015, 0.005],
        [0.02, 0.96, 0.02],
        [0.01, 0.04, 0.95]
    ],
    'gate_probability': 0.9999
}

# Terminal area (maneuvering)
TMA_PARAMS = {
    'process_noise': [0.1, 1.0, 5.0],
    'transition_matrix': [
        [0.90, 0.08, 0.02],
        [0.05, 0.85, 0.10],
        [0.02, 0.08, 0.90]
    ],
    'gate_probability': 0.999
}

# Automotive (high dynamics)
AUTOMOTIVE_PARAMS = {
    'process_noise': [0.5, 3.0, 10.0],
    'transition_matrix': [
        [0.80, 0.15, 0.05],
        [0.10, 0.75, 0.15],
        [0.05, 0.15, 0.80]
    ],
    'gate_probability': 0.99
}
```

### 9.2 Computational Optimization

```python
# Use CKF for lower computational cost
# (similar accuracy to UKF but ~30% faster)
from python.qedmma_pro.core import CKFFilter

ckf = CKFFilter(n_states=6, n_meas=3)

# Or use vectorized processing for multiple tracks
import numpy as np

# Process all tracks in parallel
all_states = np.stack([t.state for t in trackers])
all_measurements = np.stack(measurements)

# Vectorized predict/update
predicted_states = vectorized_predict(all_states, dt)
updated_states = vectorized_update(predicted_states, all_measurements)
```

---

## 10. Troubleshooting

### 10.1 Common Issues

| Issue | Cause | Solution |
|-------|-------|----------|
| Track divergence | Process noise too low | Increase Q matrix values |
| Excessive covariance | Process noise too high | Decrease Q matrix values |
| Missed associations | Gate too small | Increase gate_probability |
| False associations | Gate too large | Decrease gate_probability |
| Mode switching instability | TPM poorly tuned | Increase diagonal values |

### 10.2 Diagnostic Tools

```python
# Enable diagnostic output
tracker.set_debug(True)

# Get filter diagnostics
diagnostics = tracker.get_diagnostics()
print(f"NIS: {diagnostics['nis']}")           # Should be ~chi2(n_meas)
print(f"NEES: {diagnostics['nees']}")         # Should be ~chi2(n_states)
print(f"Innovation: {diagnostics['innovation']}")
print(f"Covariance trace: {diagnostics['cov_trace']}")

# Plot mode probabilities over time
import matplotlib.pyplot as plt
plt.plot(diagnostics['mode_history'])
plt.legend(['CV', 'CT-light', 'CT-heavy'])
plt.xlabel('Time step')
plt.ylabel('Mode probability')
```

### 10.3 Support

- **Documentation**: https://nx-mimosa.readthedocs.io/
- **Issues**: https://github.com/mladen1312/nx-mimosa/issues
- **Email**: mladen@nexellum.com
- **Phone**: +385 99 737 5100

---

*© 2024-2026 Nexellum d.o.o. All Rights Reserved.*
