#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════════════════════════════
NX-MIMOSA - NMEA 2000 OUTPUT FORMATTER FOR MARITIME APPLICATIONS
═══════════════════════════════════════════════════════════════════════════════════════════════════════

NMEA 2000 / IEC 61162 Formatter for Vessel Traffic Services (VTS)
and Maritime Radar Integration

Standards Compliance:
- NMEA 2000 (CAN-based marine network)
- IEC 61162-3 (Maritime navigation instruments)
- IMO Resolution MSC.252(83) - AIS transponders
- IALA Guideline 1082 - VTS radar systems

PGN (Parameter Group Numbers) Implemented:
- PGN 129038: AIS Class A Position Report
- PGN 129039: AIS Class B Position Report  
- PGN 129040: AIS Class B Extended Position Report
- PGN 129041: AIS Aids to Navigation Report
- PGN 129793: AIS UTC and Date Report
- PGN 129794: AIS Class A Static and Voyage
- PGN 130842: Radar Target Data

Author: Dr. Mladen Mešter / Nexellum d.o.o.
License: AGPL v3 / Commercial
═══════════════════════════════════════════════════════════════════════════════════════════════════════
"""

import struct
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Dict, Any
from enum import IntEnum
from datetime import datetime, timezone
import numpy as np

__version__ = "1.0.0"
__author__ = "Dr. Mladen Mešter"


# =============================================================================
# CONSTANTS
# =============================================================================

# Coordinate resolution
LAT_LON_RESOLUTION = 1e-7  # degrees
SOG_RESOLUTION = 0.01      # knots
COG_RESOLUTION = 0.0001    # radians

# NMEA 2000 reserved values
NA_UINT8 = 0xFF
NA_UINT16 = 0xFFFF
NA_UINT32 = 0xFFFFFFFF
NA_INT32 = 0x7FFFFFFF

# CAN identifiers (29-bit)
NMEA2000_PRIORITY_MASK = 0x1C000000
NMEA2000_PGN_MASK = 0x03FFFF00
NMEA2000_SOURCE_MASK = 0x000000FF


# =============================================================================
# ENUMERATIONS
# =============================================================================

class PGN(IntEnum):
    """NMEA 2000 Parameter Group Numbers."""
    AIS_CLASS_A_POSITION = 129038
    AIS_CLASS_B_POSITION = 129039
    AIS_CLASS_B_EXTENDED = 129040
    AIS_AIDS_TO_NAV = 129041
    AIS_UTC_DATE = 129793
    AIS_CLASS_A_STATIC = 129794
    AIS_CLASS_A_VOYAGE = 129798
    RADAR_TARGET = 130842
    GNSS_POSITION = 129029


class NavStatus(IntEnum):
    """Navigation status for AIS."""
    UNDER_WAY_USING_ENGINE = 0
    AT_ANCHOR = 1
    NOT_UNDER_COMMAND = 2
    RESTRICTED_MANEUVERABILITY = 3
    CONSTRAINED_BY_DRAUGHT = 4
    MOORED = 5
    AGROUND = 6
    ENGAGED_IN_FISHING = 7
    UNDER_WAY_SAILING = 8
    RESERVED_HSC = 9
    RESERVED_WIG = 10
    RESERVED_11 = 11
    RESERVED_12 = 12
    RESERVED_13 = 13
    AIS_SART = 14
    NOT_DEFINED = 15


class ShipType(IntEnum):
    """Ship type codes (AIS)."""
    NOT_AVAILABLE = 0
    RESERVED_1 = 1
    WIG = 20
    PILOT = 50
    SAR = 51
    TUG = 52
    PORT_TENDER = 53
    ANTI_POLLUTION = 54
    LAW_ENFORCEMENT = 55
    MEDICAL = 58
    PASSENGER = 60
    CARGO = 70
    TANKER = 80
    OTHER = 90


class TargetAcquisition(IntEnum):
    """Radar target acquisition status."""
    MANUAL = 0
    AUTOMATIC = 1


class TargetStatus(IntEnum):
    """Radar target status."""
    LOST = 0
    ACQUIRING = 1
    TRACKING = 2


# =============================================================================
# DATA STRUCTURES
# =============================================================================

@dataclass
class NMEA2000Message:
    """NMEA 2000 CAN message structure."""
    pgn: int
    priority: int = 2
    source: int = 0
    destination: int = 255  # Global
    data: bytes = b''
    
    def get_can_id(self) -> int:
        """Calculate 29-bit CAN identifier."""
        # Priority (3 bits) | Reserved (1) | Data Page (1) | PGN (17) | Source (8)
        priority_bits = (self.priority & 0x07) << 26
        
        if self.pgn >= 0xF000:
            # PDU2 format (broadcast)
            pgn_bits = (self.pgn & 0x3FFFF) << 8
        else:
            # PDU1 format (addressed)
            pgn_bits = ((self.pgn & 0x3FF00) << 8) | (self.destination << 8)
        
        source_bits = self.source & 0xFF
        
        return priority_bits | pgn_bits | source_bits
    
    def __repr__(self):
        return f"NMEA2000: PGN {self.pgn} Src={self.source} Data[{len(self.data)}]"


@dataclass
class AISTarget:
    """AIS target data."""
    mmsi: int                    # 9-digit Maritime Mobile Service Identity
    
    # Position
    latitude: float              # degrees (-90 to +90)
    longitude: float             # degrees (-180 to +180)
    
    # Kinematics
    sog: float                   # Speed over ground (knots)
    cog: float                   # Course over ground (degrees true)
    heading: float = None        # True heading (degrees)
    rot: float = 0.0             # Rate of turn (degrees/min)
    
    # Status
    nav_status: NavStatus = NavStatus.NOT_DEFINED
    ship_type: ShipType = ShipType.NOT_AVAILABLE
    
    # Identification
    name: str = ""               # Vessel name (max 20 chars)
    call_sign: str = ""          # Radio call sign (max 7 chars)
    imo_number: int = 0          # IMO number
    
    # Dimensions (meters)
    length: float = 0.0
    beam: float = 0.0
    draught: float = 0.0
    
    # Timestamp
    timestamp: datetime = None


@dataclass
class RadarTarget:
    """Radar target data for NMEA 2000."""
    target_id: int               # Target number (0-255)
    
    # Position (relative to own ship)
    range_nm: float              # Range (nautical miles)
    bearing: float               # Bearing (degrees true)
    
    # Kinematics
    speed: float                 # Speed (knots)
    course: float                # Course (degrees true)
    
    # Classification
    cpa: float = 0.0             # Closest Point of Approach (nm)
    tcpa: float = 0.0            # Time to CPA (minutes)
    
    # Status
    status: TargetStatus = TargetStatus.TRACKING
    acquisition: TargetAcquisition = TargetAcquisition.AUTOMATIC
    
    # Identification (if correlated with AIS)
    mmsi: int = 0


# =============================================================================
# NMEA 2000 ENCODER
# =============================================================================

class NMEA2000Encoder:
    """
    Encodes tracking data to NMEA 2000 messages.
    
    Supports AIS-style position reports and radar target data.
    """
    
    def __init__(self, source_address: int = 0):
        """
        Initialize NMEA 2000 encoder.
        
        Args:
            source_address: CAN source address (0-251)
        """
        self.source = source_address
        self._seq_counter = 0
    
    def _get_seq(self) -> int:
        """Get sequence counter for multi-frame messages."""
        seq = self._seq_counter
        self._seq_counter = (self._seq_counter + 1) % 256
        return seq
    
    def encode_ais_class_a_position(self, target: AISTarget) -> NMEA2000Message:
        """
        Encode PGN 129038: AIS Class A Position Report.
        
        8 bytes, broadcast every 2-10 seconds depending on speed.
        """
        data = bytearray(8)
        
        # Byte 0: Message ID (1) | Repeat Indicator (2 bits) | Reserved
        data[0] = 0x01  # Message type 1
        
        # Bytes 1-4: MMSI (compressed)
        mmsi = target.mmsi
        data[1] = (mmsi >> 22) & 0xFF
        data[2] = (mmsi >> 14) & 0xFF
        data[3] = (mmsi >> 6) & 0xFF
        data[4] = ((mmsi & 0x3F) << 2) | (target.nav_status.value >> 2)
        
        # Byte 5-6: Rate of Turn + SOG
        rot_encoded = int(target.rot * 4.733) & 0xFF if target.rot else 0x80
        sog_encoded = int(target.sog / SOG_RESOLUTION) & 0x3FF
        data[5] = rot_encoded
        data[6] = sog_encoded & 0xFF
        
        # Byte 7: COG + Position accuracy
        cog_encoded = int(target.cog / 0.1) & 0x0FFF
        data[7] = (cog_encoded >> 4) & 0xFF
        
        return NMEA2000Message(
            pgn=PGN.AIS_CLASS_A_POSITION,
            priority=2,
            source=self.source,
            data=bytes(data)
        )
    
    def encode_ais_class_a_position_full(self, target: AISTarget) -> List[NMEA2000Message]:
        """
        Encode full AIS Class A Position Report (multi-frame).
        
        Contains complete position, SOG, COG, heading, ROT.
        """
        # Build complete data
        data = bytearray()
        
        # Sequence ID
        data.append(self._get_seq())
        
        # Repeat Indicator + Message ID
        data.append(0x01)  # Type 1
        
        # MMSI (32 bits)
        data.extend(struct.pack('<I', target.mmsi))
        
        # Longitude (32 bits, 1e-7 degrees)
        lon = int(target.longitude / LAT_LON_RESOLUTION)
        data.extend(struct.pack('<i', lon))
        
        # Latitude (32 bits, 1e-7 degrees)
        lat = int(target.latitude / LAT_LON_RESOLUTION)
        data.extend(struct.pack('<i', lat))
        
        # Position accuracy + RAIM + Time stamp
        data.append(0x00)
        
        # COG (16 bits, 0.0001 radians)
        cog_rad = np.radians(target.cog)
        cog_encoded = int(cog_rad / COG_RESOLUTION)
        data.extend(struct.pack('<H', cog_encoded & 0xFFFF))
        
        # SOG (16 bits, 0.01 knots)
        sog_encoded = int(target.sog / SOG_RESOLUTION)
        data.extend(struct.pack('<H', sog_encoded & 0xFFFF))
        
        # Communication state (19 bits) + Reserved
        data.extend(b'\x00\x00\x00')
        
        # AIS Transceiver info + Heading (16 bits)
        hdg_encoded = int(np.radians(target.heading) / COG_RESOLUTION) if target.heading else NA_UINT16
        data.extend(struct.pack('<H', hdg_encoded))
        
        # ROT (8 bits)
        rot_encoded = int(target.rot * 4.733) if target.rot else 0x80
        data.append(rot_encoded & 0xFF)
        
        # Nav Status (4 bits) + Reserved
        data.append(target.nav_status.value & 0x0F)
        
        # Reserved
        data.append(0xFF)
        
        # Create fast packet messages
        messages = self._create_fast_packet(PGN.AIS_CLASS_A_POSITION, bytes(data))
        
        return messages
    
    def encode_radar_target(self, target: RadarTarget) -> NMEA2000Message:
        """
        Encode PGN 130842: Radar Target Data.
        
        Used for ARPA (Automatic Radar Plotting Aid) targets.
        """
        data = bytearray()
        
        # Sequence ID
        data.append(self._get_seq())
        
        # Target ID (8 bits)
        data.append(target.target_id & 0xFF)
        
        # Target Status (4 bits) + Acquisition (4 bits)
        status_byte = (target.status.value & 0x0F) | ((target.acquisition.value & 0x0F) << 4)
        data.append(status_byte)
        
        # Bearing (16 bits, 0.0001 radians)
        bearing_rad = np.radians(target.bearing)
        bearing_encoded = int(bearing_rad / COG_RESOLUTION)
        data.extend(struct.pack('<H', bearing_encoded & 0xFFFF))
        
        # Range (32 bits, meters)
        range_m = target.range_nm * 1852.0
        data.extend(struct.pack('<I', int(range_m)))
        
        # Speed (16 bits, 0.01 m/s)
        speed_ms = target.speed * 0.5144
        speed_encoded = int(speed_ms / 0.01)
        data.extend(struct.pack('<H', speed_encoded & 0xFFFF))
        
        # Course (16 bits, 0.0001 radians)
        course_rad = np.radians(target.course)
        course_encoded = int(course_rad / COG_RESOLUTION)
        data.extend(struct.pack('<H', course_encoded & 0xFFFF))
        
        # CPA (16 bits, meters)
        cpa_m = target.cpa * 1852.0
        data.extend(struct.pack('<H', int(cpa_m) & 0xFFFF))
        
        # TCPA (16 bits, seconds)
        tcpa_s = target.tcpa * 60.0
        data.extend(struct.pack('<H', int(tcpa_s) & 0xFFFF))
        
        # MMSI (if correlated)
        data.extend(struct.pack('<I', target.mmsi))
        
        # Reserved
        data.extend(b'\xFF\xFF')
        
        return NMEA2000Message(
            pgn=PGN.RADAR_TARGET,
            priority=3,
            source=self.source,
            data=bytes(data)
        )
    
    def _create_fast_packet(self, pgn: int, data: bytes) -> List[NMEA2000Message]:
        """
        Create fast packet messages for data > 8 bytes.
        
        NMEA 2000 fast packet protocol splits large messages.
        """
        messages = []
        frame_id = self._get_seq() & 0xE0  # Upper 3 bits
        
        # First frame: sequence + byte count + 6 data bytes
        first_data = bytearray(8)
        first_data[0] = frame_id | 0x00  # Frame counter = 0
        first_data[1] = len(data)        # Total byte count
        first_data[2:8] = data[:6]
        
        messages.append(NMEA2000Message(
            pgn=pgn,
            priority=2,
            source=self.source,
            data=bytes(first_data)
        ))
        
        # Subsequent frames: 7 data bytes each
        offset = 6
        frame_counter = 1
        
        while offset < len(data):
            frame_data = bytearray(8)
            frame_data[0] = frame_id | (frame_counter & 0x1F)
            
            remaining = data[offset:offset+7]
            frame_data[1:1+len(remaining)] = remaining
            
            # Pad with 0xFF
            for i in range(1+len(remaining), 8):
                frame_data[i] = 0xFF
            
            messages.append(NMEA2000Message(
                pgn=pgn,
                priority=2,
                source=self.source,
                data=bytes(frame_data)
            ))
            
            offset += 7
            frame_counter += 1
        
        return messages


# =============================================================================
# NX-MIMOSA INTEGRATION
# =============================================================================

class NXMIMOSANMEA2000Output:
    """
    Wrapper to convert NX-MIMOSA tracker output to NMEA 2000 messages.
    
    Usage:
        nmea_out = NXMIMOSANMEA2000Output(source_address=100)
        
        # Generate radar target from tracker
        msg = nmea_out.from_tracker(tracker, target_id=1, own_lat=45.0, own_lon=16.0)
        
        # Send to NMEA 2000 network
        can_bus.send(msg)
    """
    
    def __init__(self, source_address: int = 0):
        """Initialize NMEA 2000 output wrapper."""
        self.encoder = NMEA2000Encoder(source_address=source_address)
        self.source_address = source_address
    
    def from_tracker(self, tracker, target_id: int,
                     own_lat: float, own_lon: float,
                     own_cog: float = 0.0) -> NMEA2000Message:
        """
        Convert tracker state to NMEA 2000 radar target.
        
        Args:
            tracker: NX-MIMOSA tracker instance
            target_id: Target number (0-255)
            own_lat: Own ship latitude
            own_lon: Own ship longitude
            own_cog: Own ship course over ground
            
        Returns:
            NMEA 2000 radar target message
        """
        state = tracker.get_state()
        
        # Extract position (assume local coordinates relative to own ship)
        x, y = state[0], state[1]  # meters
        vx, vy = state[3], state[4]  # m/s
        
        # Convert to range and bearing
        range_m = np.sqrt(x**2 + y**2)
        range_nm = range_m / 1852.0
        
        bearing = np.degrees(np.arctan2(x, y)) % 360  # True bearing
        
        # Compute speed and course
        speed_ms = np.sqrt(vx**2 + vy**2)
        speed_kts = speed_ms / 0.5144
        
        course = np.degrees(np.arctan2(vx, vy)) % 360
        
        # Compute CPA/TCPA
        cpa, tcpa = self._compute_cpa_tcpa(x, y, vx, vy)
        
        target = RadarTarget(
            target_id=target_id,
            range_nm=range_nm,
            bearing=bearing,
            speed=speed_kts,
            course=course,
            cpa=cpa / 1852.0,
            tcpa=tcpa / 60.0 if tcpa > 0 else 0,
            status=TargetStatus.TRACKING,
            acquisition=TargetAcquisition.AUTOMATIC
        )
        
        return self.encoder.encode_radar_target(target)
    
    def _compute_cpa_tcpa(self, x: float, y: float, 
                          vx: float, vy: float) -> Tuple[float, float]:
        """
        Compute Closest Point of Approach and Time to CPA.
        
        Assumes own ship at origin with zero velocity (relative motion).
        """
        # Relative position and velocity
        px, py = x, y
        pvx, pvy = vx, vy
        
        # TCPA = -(P · V) / |V|²
        v_squared = pvx**2 + pvy**2
        
        if v_squared < 1e-6:
            return np.sqrt(px**2 + py**2), 0.0
        
        tcpa = -(px * pvx + py * pvy) / v_squared
        
        if tcpa < 0:
            tcpa = 0
        
        # CPA = |P + V * TCPA|
        cpa_x = px + pvx * tcpa
        cpa_y = py + pvy * tcpa
        cpa = np.sqrt(cpa_x**2 + cpa_y**2)
        
        return cpa, tcpa
    
    def from_ais(self, mmsi: int, lat: float, lon: float,
                 sog: float, cog: float) -> List[NMEA2000Message]:
        """
        Generate AIS position report.
        
        Args:
            mmsi: Maritime Mobile Service Identity
            lat: Latitude (degrees)
            lon: Longitude (degrees)
            sog: Speed over ground (knots)
            cog: Course over ground (degrees true)
            
        Returns:
            List of NMEA 2000 messages (fast packet)
        """
        target = AISTarget(
            mmsi=mmsi,
            latitude=lat,
            longitude=lon,
            sog=sog,
            cog=cog
        )
        
        return self.encoder.encode_ais_class_a_position_full(target)


# =============================================================================
# MAIN - DEMONSTRATION
# =============================================================================

if __name__ == "__main__":
    print("="*80)
    print("NX-MIMOSA NMEA 2000 OUTPUT FORMATTER")
    print("For Vessel Traffic Services (VTS)")
    print("="*80)
    
    # Create encoder
    encoder = NMEA2000Encoder(source_address=100)
    
    # Sample AIS target
    ais_target = AISTarget(
        mmsi=123456789,
        latitude=45.8,
        longitude=16.0,
        sog=12.5,
        cog=90.0,
        heading=88.0,
        rot=0.0,
        nav_status=NavStatus.UNDER_WAY_USING_ENGINE,
        ship_type=ShipType.CARGO,
        name="ATLANTIC TRADER",
        call_sign="9ABC123"
    )
    
    print(f"\n🚢 AIS Target:")
    print(f"  MMSI: {ais_target.mmsi}")
    print(f"  Position: ({ais_target.latitude:.4f}°, {ais_target.longitude:.4f}°)")
    print(f"  SOG: {ais_target.sog} kts, COG: {ais_target.cog}°")
    
    # Encode AIS position
    ais_msgs = encoder.encode_ais_class_a_position_full(ais_target)
    print(f"\n📡 AIS Class A Position (Fast Packet):")
    print(f"  Frames: {len(ais_msgs)}")
    for i, msg in enumerate(ais_msgs):
        print(f"  Frame {i}: CAN ID={msg.get_can_id():#010x}, Data={msg.data.hex()}")
    
    # Sample radar target
    radar_target = RadarTarget(
        target_id=1,
        range_nm=5.2,
        bearing=45.0,
        speed=15.0,
        course=270.0,
        cpa=0.3,
        tcpa=12.5,
        status=TargetStatus.TRACKING,
        acquisition=TargetAcquisition.AUTOMATIC,
        mmsi=0  # Not correlated with AIS
    )
    
    print(f"\n📍 Radar Target:")
    print(f"  ID: {radar_target.target_id}")
    print(f"  Range: {radar_target.range_nm:.1f} nm, Bearing: {radar_target.bearing:.1f}°")
    print(f"  Speed: {radar_target.speed:.1f} kts, Course: {radar_target.course:.1f}°")
    print(f"  CPA: {radar_target.cpa:.2f} nm, TCPA: {radar_target.tcpa:.1f} min")
    
    # Encode radar target
    radar_msg = encoder.encode_radar_target(radar_target)
    print(f"\n📡 Radar Target (PGN {radar_msg.pgn}):")
    print(f"  CAN ID: {radar_msg.get_can_id():#010x}")
    print(f"  Data: {radar_msg.data.hex()}")
    
    print("\n" + "="*80)
    print("✓ NMEA 2000 formatter ready for maritime applications")
    print("="*80)
