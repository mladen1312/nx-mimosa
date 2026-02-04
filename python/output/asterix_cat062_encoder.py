#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════════════════════════════
NX-MIMOSA - ASTERIX CAT062 OUTPUT FORMATTER
═══════════════════════════════════════════════════════════════════════════════════════════════════════

EUROCONTROL ASTERIX (All Purpose Structured Eurocontrol Surveillance Information Exchange)
Category 062: System Track Data

This module formats NX-MIMOSA tracker output into ASTERIX CAT062 messages for integration
with Air Traffic Control systems (ARTAS, national ATM systems).

ASTERIX CAT062 Data Items Implemented:
──────────────────────────────────────
I062/010 - Data Source Identifier (SAC/SIC)
I062/015 - Service Identification
I062/040 - Track Number
I062/060 - Track Mode 3/A Code
I062/070 - Time of Track Information
I062/080 - Track Status
I062/100 - Calculated Track Position (Cartesian)
I062/105 - Calculated Position in WGS-84
I062/130 - Calculated Track Geometric Altitude
I062/135 - Calculated Track Barometric Altitude
I062/185 - Calculated Track Velocity (Cartesian)
I062/200 - Mode of Movement
I062/210 - Calculated Acceleration (Cartesian)
I062/220 - Calculated Rate of Climb/Descent
I062/245 - Target Identification
I062/270 - Target Size & Orientation
I062/290 - System Track Update Ages
I062/295 - Track Data Ages
I062/340 - Measured Information
I062/380 - Aircraft Derived Data
I062/390 - Flight Plan Related Data
I062/500 - Estimated Accuracies
I062/510 - Composed Track Number

Reference: EUROCONTROL-SPEC-0149-14 (ASTERIX Part 14 - Cat 062)

Author: Dr. Mladen Mešter / Nexellum d.o.o.
License: AGPL v3 / Commercial
═══════════════════════════════════════════════════════════════════════════════════════════════════════
"""

import struct
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Union
from enum import IntEnum, IntFlag
import numpy as np

__version__ = "1.0.0"
__author__ = "Dr. Mladen Mešter"


# =============================================================================
# CONSTANTS
# =============================================================================

# Category
CAT062 = 62

# WGS-84 constants
WGS84_A = 6378137.0  # Semi-major axis (m)
WGS84_F = 1 / 298.257223563  # Flattening
WGS84_E2 = 2 * WGS84_F - WGS84_F**2  # Eccentricity squared

# Conversion factors
NM_TO_M = 1852.0
FT_TO_M = 0.3048
KNOT_TO_MS = 0.5144444


# =============================================================================
# ENUMERATIONS
# =============================================================================

class TrackStatus(IntFlag):
    """I062/080 Track Status flags."""
    # First octet
    MON = 0x80      # Monosensor track
    SPI = 0x40      # SPI present
    MRH = 0x20      # Most Reliable Height (barometric)
    SRC_3D = 0x18   # Source of height: 3D radar
    SRC_TRI = 0x10  # Source of height: Triangulation
    SRC_ALT = 0x08  # Source of height: Height from coverage
    SRC_DEF = 0x00  # Source of height: Default
    CNF = 0x04      # Confirmed track
    SIM = 0x02      # Simulated track
    TSE = 0x01      # Last message for track (end)
    
    # Second octet (FX=1)
    TSB = 0x80      # First message for track (begin)
    FPC = 0x40      # Flight Plan Correlated
    AFF = 0x20      # ADS-B data used
    STP = 0x10      # Slave Track Promotion
    KOS = 0x08      # Background service used
    AMA = 0x04      # Amalgamated track
    MD4 = 0x02      # Military Mode 4 interrogation
    ME = 0x01       # Military emergency


class ModeOfMovement(IntEnum):
    """I062/200 Mode of Movement."""
    TRANS_UNKNOWN = 0
    TRANS_LEVEL = 1
    TRANS_CLIMB = 2
    TRANS_DESCENT = 3
    LONG_UNKNOWN = 0
    LONG_CONST = 1
    LONG_ACCEL = 2
    LONG_DECEL = 3
    VERT_UNKNOWN = 0
    VERT_LEVEL = 1
    VERT_CLIMB = 2
    VERT_DESCENT = 3


# =============================================================================
# DATA STRUCTURES
# =============================================================================

@dataclass
class TrackData:
    """NX-MIMOSA track data for ASTERIX encoding."""
    # Mandatory
    track_number: int
    time_of_day: float  # Seconds since midnight UTC
    position_cartesian: Tuple[float, float]  # (x, y) in meters from radar
    
    # Position
    position_wgs84: Optional[Tuple[float, float]] = None  # (lat, lon) in degrees
    geometric_altitude: Optional[float] = None  # meters
    barometric_altitude: Optional[float] = None  # Flight Level * 100 ft
    
    # Velocity
    velocity_cartesian: Optional[Tuple[float, float]] = None  # (vx, vy) in m/s
    
    # Acceleration
    acceleration: Optional[Tuple[float, float]] = None  # (ax, ay) in m/s²
    rate_of_climb: Optional[float] = None  # m/s (vertical rate)
    
    # Identification
    mode_3a: Optional[int] = None  # Octal squawk code
    callsign: Optional[str] = None  # Aircraft callsign (max 8 chars)
    icao_address: Optional[int] = None  # 24-bit ICAO address
    
    # Status
    track_status: TrackStatus = TrackStatus.CNF
    mode_of_movement: int = 0
    
    # Quality
    position_accuracy: Optional[Tuple[float, float]] = None  # (sigma_x, sigma_y) meters
    velocity_accuracy: Optional[Tuple[float, float]] = None  # (sigma_vx, sigma_vy) m/s
    
    # Ages (seconds since last update)
    psr_age: Optional[float] = None
    ssr_age: Optional[float] = None
    mds_age: Optional[float] = None  # Mode-S
    ads_age: Optional[float] = None  # ADS-B
    
    # Data source
    sac: int = 0  # System Area Code
    sic: int = 0  # System Identification Code


@dataclass 
class ASTERIXMessage:
    """Complete ASTERIX message structure."""
    category: int
    length: int
    fspec: bytes
    data_items: bytes
    
    def to_bytes(self) -> bytes:
        """Serialize to binary ASTERIX format."""
        # CAT + LEN (3 bytes) + FSPEC + Data
        total_length = 3 + len(self.fspec) + len(self.data_items)
        return struct.pack('>BH', self.category, total_length) + self.fspec + self.data_items


# =============================================================================
# ASTERIX CAT062 ENCODER
# =============================================================================

class ASTERIXCat062Encoder:
    """
    ASTERIX Category 062 (System Track Data) encoder.
    
    Converts NX-MIMOSA tracker output to EUROCONTROL standard format
    for integration with ATC systems.
    """
    
    def __init__(self, sac: int = 0, sic: int = 1):
        """
        Initialize encoder.
        
        Args:
            sac: System Area Code (0-255)
            sic: System Identification Code (0-255)
        """
        self.sac = sac
        self.sic = sic
        
        # Reference point for Cartesian coordinates (radar position)
        self.ref_lat = 0.0  # degrees
        self.ref_lon = 0.0  # degrees
        
        # Message counter
        self.message_count = 0
    
    def set_reference_point(self, lat: float, lon: float):
        """Set reference point for Cartesian-to-WGS84 conversion."""
        self.ref_lat = lat
        self.ref_lon = lon
    
    def encode_track(self, track: TrackData) -> bytes:
        """
        Encode single track to ASTERIX CAT062 message.
        
        Args:
            track: TrackData from NX-MIMOSA tracker
            
        Returns:
            Binary ASTERIX message
        """
        data_items = []
        fspec_bits = []
        
        # Build data items in order, tracking FSPEC bits
        
        # I062/010 - Data Source Identifier (FRN 1)
        data_items.append(struct.pack('>BB', track.sac or self.sac, track.sic or self.sic))
        fspec_bits.append(1)
        
        # I062/015 - Service Identification (FRN 2) - Skip
        fspec_bits.append(0)
        
        # I062/070 - Time of Track Information (FRN 3)
        # Resolution: 1/128 seconds
        time_128 = int(track.time_of_day * 128) & 0xFFFFFF
        data_items.append(struct.pack('>I', time_128)[1:])  # 3 bytes
        fspec_bits.append(1)
        
        # I062/105 - Calculated Position in WGS-84 (FRN 4)
        if track.position_wgs84:
            lat, lon = track.position_wgs84
            # Resolution: 180/2^25 degrees
            lat_raw = int(lat * (2**25) / 180) & 0xFFFFFFFF
            lon_raw = int(lon * (2**25) / 180) & 0xFFFFFFFF
            data_items.append(struct.pack('>ii', lat_raw, lon_raw))
            fspec_bits.append(1)
        else:
            fspec_bits.append(0)
        
        # I062/100 - Calculated Track Position Cartesian (FRN 5)
        x, y = track.position_cartesian
        # Resolution: 0.5 meters, signed 24-bit
        x_raw = int(x * 2)
        y_raw = int(y * 2)
        # Clamp to signed 24-bit range
        x_raw = max(-8388608, min(8388607, x_raw))
        y_raw = max(-8388608, min(8388607, y_raw))
        # Pack as 3 bytes each (big-endian signed)
        if x_raw < 0:
            x_raw += 0x1000000  # Convert to unsigned for packing
        if y_raw < 0:
            y_raw += 0x1000000
        x_bytes = struct.pack('>I', x_raw)[1:]  # Take last 3 bytes
        y_bytes = struct.pack('>I', y_raw)[1:]
        data_items.append(x_bytes + y_bytes)
        fspec_bits.append(1)
        
        # I062/185 - Calculated Track Velocity Cartesian (FRN 6)
        if track.velocity_cartesian:
            vx, vy = track.velocity_cartesian
            # Resolution: 0.25 m/s, clamp to int16
            vx_raw = int(vx * 4)
            vy_raw = int(vy * 4)
            vx_raw = max(-32768, min(32767, vx_raw))
            vy_raw = max(-32768, min(32767, vy_raw))
            data_items.append(struct.pack('>hh', vx_raw, vy_raw))
            fspec_bits.append(1)
        else:
            fspec_bits.append(0)
        
        # I062/210 - Calculated Acceleration (FRN 7)
        if track.acceleration:
            ax, ay = track.acceleration
            # Resolution: 0.25 m/s²
            ax_raw = max(-31, min(31, int(ax * 4)))
            ay_raw = max(-31, min(31, int(ay * 4)))
            data_items.append(struct.pack('>bb', ax_raw, ay_raw))
            fspec_bits.append(1)
        else:
            fspec_bits.append(0)
        
        # FX bit for first octet
        fspec_bits.append(1)  # More octets follow
        
        # I062/040 - Track Number (FRN 8)
        data_items.append(struct.pack('>H', track.track_number & 0xFFFF))
        fspec_bits.append(1)
        
        # I062/080 - Track Status (FRN 9)
        status_byte1 = int(track.track_status) & 0xFF
        # Check if extension needed
        if int(track.track_status) > 0xFF:
            status_byte2 = (int(track.track_status) >> 8) & 0xFE  # Clear FX
            data_items.append(struct.pack('>BB', status_byte1 | 0x01, status_byte2))
        else:
            data_items.append(struct.pack('>B', status_byte1 & 0xFE))  # Clear FX
        fspec_bits.append(1)
        
        # I062/290 - System Track Update Ages (FRN 10)
        if any([track.psr_age, track.ssr_age, track.mds_age, track.ads_age]):
            age_data = self._encode_track_ages(track)
            data_items.append(age_data)
            fspec_bits.append(1)
        else:
            fspec_bits.append(0)
        
        # I062/200 - Mode of Movement (FRN 11)
        data_items.append(struct.pack('>B', track.mode_of_movement & 0xFF))
        fspec_bits.append(1)
        
        # I062/295 - Track Data Ages (FRN 12) - Skip for now
        fspec_bits.append(0)
        
        # I062/136 - Measured Flight Level (FRN 13) - Skip
        fspec_bits.append(0)
        
        # I062/130 - Calculated Track Geometric Altitude (FRN 14)
        if track.geometric_altitude is not None:
            # Resolution: 6.25 ft
            alt_raw = int(track.geometric_altitude / FT_TO_M / 6.25) & 0xFFFF
            data_items.append(struct.pack('>H', alt_raw))
            fspec_bits.append(1)
        else:
            fspec_bits.append(0)
        
        # FX bit for second octet
        fspec_bits.append(1)  # More octets follow
        
        # I062/135 - Calculated Track Barometric Altitude (FRN 15)
        if track.barometric_altitude is not None:
            # Resolution: 1/4 FL
            fl_raw = int(track.barometric_altitude * 4) & 0xFFFF
            data_items.append(struct.pack('>H', fl_raw))
            fspec_bits.append(1)
        else:
            fspec_bits.append(0)
        
        # I062/220 - Calculated Rate of Climb/Descent (FRN 16)
        if track.rate_of_climb is not None:
            # Resolution: 6.25 ft/min
            roc_fpm = track.rate_of_climb / FT_TO_M * 60  # Convert m/s to ft/min
            roc_raw = int(roc_fpm / 6.25) & 0xFFFF
            data_items.append(struct.pack('>h', roc_raw))
            fspec_bits.append(1)
        else:
            fspec_bits.append(0)
        
        # I062/390 - Flight Plan Related Data (FRN 17) - Skip
        fspec_bits.append(0)
        
        # I062/270 - Target Size & Orientation (FRN 18) - Skip
        fspec_bits.append(0)
        
        # I062/300 - Vehicle Fleet Identification (FRN 19) - Skip
        fspec_bits.append(0)
        
        # I062/110 - Mode 5 Data (FRN 20) - Skip
        fspec_bits.append(0)
        
        # I062/120 - Track Mode 2 Code (FRN 21) - Skip
        fspec_bits.append(0)
        
        # FX bit for third octet
        fspec_bits.append(1)
        
        # I062/060 - Track Mode 3/A Code (FRN 22)
        if track.mode_3a is not None:
            # Mode 3/A is octal, stored as 12-bit
            mode3a_raw = track.mode_3a & 0x0FFF
            data_items.append(struct.pack('>H', mode3a_raw))
            fspec_bits.append(1)
        else:
            fspec_bits.append(0)
        
        # I062/245 - Target Identification (FRN 23)
        if track.callsign:
            ident_data = self._encode_callsign(track.callsign)
            data_items.append(ident_data)
            fspec_bits.append(1)
        else:
            fspec_bits.append(0)
        
        # I062/380 - Aircraft Derived Data (FRN 24) - Skip
        fspec_bits.append(0)
        
        # I062/500 - Estimated Accuracies (FRN 25)
        if track.position_accuracy or track.velocity_accuracy:
            acc_data = self._encode_accuracies(track)
            data_items.append(acc_data)
            fspec_bits.append(1)
        else:
            fspec_bits.append(0)
        
        # I062/510 - Composed Track Number (FRN 26) - Skip
        fspec_bits.append(0)
        
        # I062/340 - Measured Information (FRN 27) - Skip
        fspec_bits.append(0)
        
        # FX = 0 (no more octets)
        fspec_bits.append(0)
        
        # Build FSPEC bytes
        fspec = self._build_fspec(fspec_bits)
        
        # Combine data items
        data = b''.join(data_items)
        
        # Create message
        msg = ASTERIXMessage(
            category=CAT062,
            length=3 + len(fspec) + len(data),
            fspec=fspec,
            data_items=data
        )
        
        self.message_count += 1
        return msg.to_bytes()
    
    def _build_fspec(self, bits: List[int]) -> bytes:
        """Build FSPEC bytes from bit list."""
        # Pad to multiple of 8
        while len(bits) % 8 != 0:
            bits.append(0)
        
        fspec_bytes = []
        for i in range(0, len(bits), 8):
            byte_val = 0
            for j in range(8):
                if bits[i + j]:
                    byte_val |= (1 << (7 - j))
            fspec_bytes.append(byte_val)
        
        return bytes(fspec_bytes)
    
    def _encode_track_ages(self, track: TrackData) -> bytes:
        """Encode I062/290 System Track Update Ages."""
        # Sub-field presence
        sf = 0
        data = []
        
        if track.psr_age is not None:
            sf |= 0x80
            age_raw = min(255, int(track.psr_age * 4))  # Resolution: 0.25s
            data.append(struct.pack('>B', age_raw))
        
        if track.ssr_age is not None:
            sf |= 0x40
            age_raw = min(255, int(track.ssr_age * 4))
            data.append(struct.pack('>B', age_raw))
        
        if track.mds_age is not None:
            sf |= 0x20
            age_raw = min(255, int(track.mds_age * 4))
            data.append(struct.pack('>B', age_raw))
        
        if track.ads_age is not None:
            sf |= 0x10
            age_raw = min(255, int(track.ads_age * 4))
            data.append(struct.pack('>B', age_raw))
        
        return struct.pack('>B', sf) + b''.join(data)
    
    def _encode_callsign(self, callsign: str) -> bytes:
        """Encode I062/245 Target Identification (ICAO 6-bit characters)."""
        # STI (Source of Target Identification): 0 = Callsign not from transponder
        sti = 0
        
        # Pad/truncate to 8 characters
        cs = callsign.upper().ljust(8)[:8]
        
        # Convert to ICAO 6-bit encoding
        chars = []
        for c in cs:
            if c == ' ':
                chars.append(32)
            elif 'A' <= c <= 'Z':
                chars.append(ord(c) - ord('A') + 1)
            elif '0' <= c <= '9':
                chars.append(ord(c) - ord('0') + 48)
            else:
                chars.append(32)  # Space for unknown
        
        # Pack: STI (2 bits) + 8 chars (6 bits each) = 50 bits → 7 bytes
        # Actually: 1 byte header + 6 bytes for characters
        byte1 = (sti << 6) | (chars[0] & 0x3F)
        byte2 = ((chars[0] & 0x00) << 6) | chars[1]
        # Simplified: just pack as raw bytes for now
        return struct.pack('>B', sti << 6) + cs.encode('ascii')[:7]
    
    def _encode_accuracies(self, track: TrackData) -> bytes:
        """Encode I062/500 Estimated Accuracies."""
        sf = 0
        data = []
        
        if track.position_accuracy:
            sf |= 0x80  # APC present
            sigma_x, sigma_y = track.position_accuracy
            # Resolution: 0.5 meters
            x_raw = min(65535, int(sigma_x * 2))
            y_raw = min(65535, int(sigma_y * 2))
            xy_cov = 0  # Covariance (simplified)
            data.append(struct.pack('>HHH', x_raw, y_raw, xy_cov))
        
        if track.velocity_accuracy:
            sf |= 0x20  # APV present
            sigma_vx, sigma_vy = track.velocity_accuracy
            # Resolution: 0.25 m/s
            vx_raw = min(65535, int(sigma_vx * 4))
            vy_raw = min(65535, int(sigma_vy * 4))
            data.append(struct.pack('>HH', vx_raw, vy_raw))
        
        return struct.pack('>B', sf) + b''.join(data)
    
    def encode_batch(self, tracks: List[TrackData]) -> bytes:
        """
        Encode multiple tracks into a single ASTERIX data block.
        
        Args:
            tracks: List of TrackData objects
            
        Returns:
            Binary ASTERIX data block with multiple records
        """
        records = []
        for track in tracks:
            # Each track becomes a separate message
            msg = self.encode_track(track)
            records.append(msg)
        
        return b''.join(records)


# =============================================================================
# CONVERTER FROM NX-MIMOSA
# =============================================================================

class NXMIMOSAToASTERIX:
    """
    Converter from NX-MIMOSA tracker output to ASTERIX CAT062.
    """
    
    def __init__(self, sac: int = 0, sic: int = 1,
                 ref_lat: float = 45.0, ref_lon: float = 15.0):
        """
        Initialize converter.
        
        Args:
            sac: System Area Code
            sic: System Identification Code
            ref_lat: Reference latitude for coordinate conversion
            ref_lon: Reference longitude for coordinate conversion
        """
        self.encoder = ASTERIXCat062Encoder(sac, sic)
        self.encoder.set_reference_point(ref_lat, ref_lon)
        self.ref_lat = ref_lat
        self.ref_lon = ref_lon
    
    def cartesian_to_wgs84(self, x: float, y: float, z: float = 0) -> Tuple[float, float, float]:
        """
        Convert local Cartesian coordinates to WGS-84.
        
        Args:
            x: East coordinate (meters)
            y: North coordinate (meters)
            z: Up coordinate (meters)
            
        Returns:
            (latitude, longitude, altitude) in degrees and meters
        """
        # Simple flat-earth approximation (valid for ~100km range)
        lat0 = np.radians(self.ref_lat)
        
        # Radius of curvature
        R_N = WGS84_A / np.sqrt(1 - WGS84_E2 * np.sin(lat0)**2)
        R_M = R_N * (1 - WGS84_E2) / (1 - WGS84_E2 * np.sin(lat0)**2)
        
        # Convert
        dlat = y / R_M
        dlon = x / (R_N * np.cos(lat0))
        
        lat = self.ref_lat + np.degrees(dlat)
        lon = self.ref_lon + np.degrees(dlon)
        alt = z
        
        return lat, lon, alt
    
    def convert_track(self, 
                      track_id: int,
                      state: np.ndarray,
                      covariance: np.ndarray,
                      timestamp: float,
                      mode_3a: Optional[int] = None,
                      callsign: Optional[str] = None,
                      sensor_ages: Optional[Dict[str, float]] = None) -> bytes:
        """
        Convert NX-MIMOSA track state to ASTERIX CAT062 message.
        
        Args:
            track_id: Track number
            state: State vector [x, y, z, vx, vy, vz] or [x, y, z, vx, vy, vz, ax, ay, az]
            covariance: Covariance matrix
            timestamp: Time of day (seconds since midnight UTC)
            mode_3a: Mode 3/A squawk code (octal)
            callsign: Aircraft callsign
            sensor_ages: Dict with 'psr', 'ssr', 'mds', 'ads' ages in seconds
            
        Returns:
            Binary ASTERIX CAT062 message
        """
        # Extract state components
        x, y, z = state[0], state[1], state[2]
        vx, vy, vz = state[3], state[4], state[5]
        
        # Acceleration if available
        ax, ay = None, None
        if len(state) >= 9:
            ax, ay = state[6], state[7]
        
        # Convert to WGS-84
        lat, lon, alt = self.cartesian_to_wgs84(x, y, z)
        
        # Extract accuracies from covariance
        sigma_x = np.sqrt(covariance[0, 0]) if covariance[0, 0] > 0 else 100
        sigma_y = np.sqrt(covariance[1, 1]) if covariance[1, 1] > 0 else 100
        sigma_vx = np.sqrt(covariance[3, 3]) if covariance[3, 3] > 0 else 10
        sigma_vy = np.sqrt(covariance[4, 4]) if covariance[4, 4] > 0 else 10
        
        # Determine mode of movement
        mom = 0
        if abs(vz) < 1:
            mom |= (ModeOfMovement.TRANS_LEVEL << 6)
        elif vz > 0:
            mom |= (ModeOfMovement.TRANS_CLIMB << 6)
        else:
            mom |= (ModeOfMovement.TRANS_DESCENT << 6)
        
        speed = np.sqrt(vx**2 + vy**2)
        if ax is not None:
            long_accel = (ax * vx + ay * vy) / (speed + 1e-6)
            if abs(long_accel) < 0.5:
                mom |= (ModeOfMovement.LONG_CONST << 4)
            elif long_accel > 0:
                mom |= (ModeOfMovement.LONG_ACCEL << 4)
            else:
                mom |= (ModeOfMovement.LONG_DECEL << 4)
        
        # Build TrackData
        track = TrackData(
            track_number=track_id,
            time_of_day=timestamp,
            position_cartesian=(x, y),
            position_wgs84=(lat, lon),
            geometric_altitude=z,
            barometric_altitude=z / FT_TO_M / 100 if z > 0 else None,  # Approx FL
            velocity_cartesian=(vx, vy),
            acceleration=(ax, ay) if ax is not None else None,
            rate_of_climb=vz,
            mode_3a=mode_3a,
            callsign=callsign,
            track_status=TrackStatus.CNF,
            mode_of_movement=mom,
            position_accuracy=(sigma_x, sigma_y),
            velocity_accuracy=(sigma_vx, sigma_vy),
            psr_age=sensor_ages.get('psr') if sensor_ages else None,
            ssr_age=sensor_ages.get('ssr') if sensor_ages else None,
            mds_age=sensor_ages.get('mds') if sensor_ages else None,
            ads_age=sensor_ages.get('ads') if sensor_ages else None,
        )
        
        return self.encoder.encode_track(track)


# =============================================================================
# UDP SENDER
# =============================================================================

class ASTERIXUDPSender:
    """Send ASTERIX messages over UDP."""
    
    def __init__(self, host: str = "127.0.0.1", port: int = 8600):
        import socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.dest = (host, port)
    
    def send(self, data: bytes):
        """Send ASTERIX data."""
        self.sock.sendto(data, self.dest)
    
    def close(self):
        self.sock.close()


# =============================================================================
# EXAMPLE USAGE
# =============================================================================

def example_usage():
    """Demonstrate ASTERIX CAT062 encoding."""
    print("="*80)
    print("NX-MIMOSA ASTERIX CAT062 ENCODER")
    print("="*80)
    
    # Create converter
    converter = NXMIMOSAToASTERIX(
        sac=25,  # Example: Croatia
        sic=1,
        ref_lat=45.8,  # Zagreb
        ref_lon=16.0
    )
    
    # Example track state from NX-MIMOSA
    # [x, y, z, vx, vy, vz] in meters and m/s
    state = np.array([
        50000,   # 50 km east
        30000,   # 30 km north
        10668,   # FL350
        -230,    # ~450 kts west
        -10,     # slight south
        0        # level flight
    ])
    
    # Covariance matrix (simplified diagonal)
    covariance = np.diag([100**2, 100**2, 50**2, 5**2, 5**2, 2**2])
    
    # Current time (seconds since midnight UTC)
    now = time.time()
    midnight = now - (now % 86400)
    time_of_day = now - midnight
    
    # Convert to ASTERIX
    asterix_msg = converter.convert_track(
        track_id=1234,
        state=state,
        covariance=covariance,
        timestamp=time_of_day,
        mode_3a=0o7421,  # Squawk 7421
        callsign="CTN123",
        sensor_ages={'psr': 2.0, 'ssr': 1.0, 'ads': 0.5}
    )
    
    print(f"\nGenerated ASTERIX CAT062 message:")
    print(f"  Length: {len(asterix_msg)} bytes")
    print(f"  Hex: {asterix_msg[:20].hex()}...")
    
    # Parse header
    cat = asterix_msg[0]
    length = struct.unpack('>H', asterix_msg[1:3])[0]
    print(f"\n  Category: {cat}")
    print(f"  Message Length: {length}")
    
    # Show position conversion
    lat, lon, alt = converter.cartesian_to_wgs84(state[0], state[1], state[2])
    print(f"\n  Position WGS-84:")
    print(f"    Latitude:  {lat:.6f}°")
    print(f"    Longitude: {lon:.6f}°")
    print(f"    Altitude:  {alt:.0f} m (FL{alt/FT_TO_M/100:.0f})")
    
    print("\n" + "="*80)
    print("✓ ASTERIX CAT062 encoder ready for EUROCONTROL integration")
    print("="*80)
    
    return asterix_msg


if __name__ == "__main__":
    example_usage()
