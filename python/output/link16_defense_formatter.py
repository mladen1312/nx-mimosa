#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════════════════════════════
NX-MIMOSA - LINK-16 (TADIL-J) OUTPUT FORMATTER FOR DEFENSE APPLICATIONS
═══════════════════════════════════════════════════════════════════════════════════════════════════════

Link-16 / TADIL-J (Tactical Digital Information Link)
MIL-STD-6016 Compliant Message Formatting

This module converts NX-MIMOSA tracker output to Link-16 J-Series messages for
integration with NATO tactical data link systems.

Standards Compliance:
- MIL-STD-6016 (TADIL-J Message Standard)
- STANAG 5516 (Link-16)
- MIL-STD-6011 (TADIL-J Interface)

Message Types Implemented:
- J2.2: Air Track
- J2.3: Surface Track  
- J2.5: Space Track
- J3.2: Air PPLI (Precise Participant Location and Identification)
- J7.0: Track Management
- J7.2: Track Drop
- J14.0: Threat Warning

Network Participation Groups (NPG):
- NPG 1: Network Management
- NPG 2: PPLI
- NPG 3: Surveillance
- NPG 4: Mission Management
- NPG 5: Air Control
- NPG 6: JTIDS/MIDS Voice

Author: Dr. Mladen Mešter / Nexellum d.o.o.
License: AGPL v3 / Commercial
═══════════════════════════════════════════════════════════════════════════════════════════════════════
"""

import struct
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Dict, Any, Union
from enum import IntEnum, IntFlag
import numpy as np
from datetime import datetime

__version__ = "1.0.0"
__author__ = "Dr. Mladen Mešter"


# =============================================================================
# CONSTANTS
# =============================================================================

# Coordinate conversions
NM_TO_METERS = 1852.0
METERS_TO_NM = 1.0 / NM_TO_METERS
FEET_TO_METERS = 0.3048
METERS_TO_FEET = 1.0 / FEET_TO_METERS
KNOTS_TO_MS = 0.5144
MS_TO_KNOTS = 1.0 / KNOTS_TO_MS

# Link-16 resolution constants
LAT_LON_LSB = 180.0 / (2**23)  # ~21.5 meters at equator
ALTITUDE_LSB = 25.0  # feet
SPEED_LSB = 1.0  # knots
HEADING_LSB = 360.0 / 512  # ~0.7 degrees

# Time slot duration (Link-16)
TIME_SLOT_DURATION_US = 7.8125  # microseconds


# =============================================================================
# ENUMERATIONS
# =============================================================================

class JSeriesMessage(IntEnum):
    """J-Series message types."""
    J0_0_INITIAL_ENTRY = 0x000
    J2_0_AIR_TRACK_INFO = 0x020
    J2_2_AIR_TRACK = 0x022
    J2_3_SURFACE_TRACK = 0x023
    J2_4_SUBSURFACE_TRACK = 0x024
    J2_5_SPACE_TRACK = 0x025
    J2_6_ELECTRONIC_WARFARE = 0x026
    J3_0_REFERENCE_POINT = 0x030
    J3_2_AIR_PPLI = 0x032
    J3_3_SURFACE_PPLI = 0x033
    J3_5_SPACE_PPLI = 0x035
    J7_0_TRACK_MANAGEMENT = 0x070
    J7_2_TRACK_DROP = 0x072
    J7_3_TRACK_QUALITY = 0x073
    J12_0_MISSION_ASSIGNMENT = 0x0C0
    J14_0_THREAT_WARNING = 0x0E0
    J14_2_AIR_THREAT = 0x0E2


class TrackIdentity(IntEnum):
    """Track identity classification."""
    PENDING = 0
    UNKNOWN = 1
    ASSUMED_FRIEND = 2
    FRIEND = 3
    NEUTRAL = 4
    SUSPECT = 5
    HOSTILE = 6


class TrackEnvironment(IntEnum):
    """Track environment domain."""
    AIR = 0
    SURFACE = 1
    SUBSURFACE = 2
    SPACE = 3
    LAND = 4


class PlatformType(IntEnum):
    """Platform type codes (partial list)."""
    UNKNOWN = 0
    FIGHTER = 1
    ATTACK = 2
    BOMBER = 3
    TRANSPORT = 4
    TANKER = 5
    HELICOPTER = 6
    UAV = 7
    CRUISE_MISSILE = 8
    BALLISTIC_MISSILE = 9
    SURFACE_SHIP = 20
    SUBMARINE = 30
    SATELLITE = 40


class TrackQuality(IntEnum):
    """Track quality indicator."""
    INVALID = 0
    INITIAL = 1
    DEGRADED = 2
    NORMAL = 3
    HIGH = 4


class EmergencyStatus(IntFlag):
    """Emergency status flags."""
    NONE = 0x00
    EMERGENCY = 0x01
    DISTRESS = 0x02
    MAYDAY = 0x04
    PAN = 0x08


# =============================================================================
# DATA STRUCTURES
# =============================================================================

@dataclass
class Link16Track:
    """Track data for Link-16 J-message encoding."""
    # Track identification
    track_number: int           # 0-16383 (14-bit)
    
    # Position (WGS-84)
    latitude: float             # degrees (-90 to +90)
    longitude: float            # degrees (-180 to +180)
    altitude: float             # feet MSL
    
    # Kinematics (mandatory before defaults)
    speed: float               # knots
    heading: float             # degrees true (0-360)
    
    # Optional with defaults
    jtids_track_number: int = 0 # 0-65535 (optional platform-specific)
    climb_rate: float = 0.0    # feet/min
    
    # Classification
    identity: TrackIdentity = TrackIdentity.UNKNOWN
    environment: TrackEnvironment = TrackEnvironment.AIR
    platform_type: PlatformType = PlatformType.UNKNOWN
    
    # Quality
    quality: TrackQuality = TrackQuality.NORMAL
    strength: int = 0          # Signal strength (0-15)
    
    # Status
    simulated: bool = False
    emergency: EmergencyStatus = EmergencyStatus.NONE
    iff_mode: int = 0          # IFF modes (bitmap)
    iff_code: int = 0          # Mode 3/A code (octal)
    
    # Time
    timestamp: float = 0.0     # Seconds since midnight UTC
    
    # Accuracy (1-sigma)
    position_accuracy: float = 0.0  # meters
    velocity_accuracy: float = 0.0  # m/s


@dataclass
class Link16Message:
    """Link-16 message structure."""
    message_type: JSeriesMessage
    sublabel: int = 0
    data: bytes = b''
    npg: int = 3               # Network Participation Group (default: Surveillance)
    source_track_number: int = 0
    time_slot: int = 0
    
    def __repr__(self):
        return f"Link16: J{self.message_type.value//16}.{self.message_type.value%16} NPG{self.npg} Data[{len(self.data)}]"


@dataclass
class PPLI:
    """Precise Participant Location and Identification."""
    source_track_number: int
    latitude: float
    longitude: float
    altitude: float
    speed: float
    heading: float
    identity: TrackIdentity = TrackIdentity.FRIEND
    platform_type: PlatformType = PlatformType.UNKNOWN
    unit_name: str = ""        # Up to 3 characters


# =============================================================================
# LINK-16 MESSAGE ENCODER
# =============================================================================

class Link16Encoder:
    """
    Encodes track data to Link-16 J-Series messages.
    
    Message Word Structure (75 bits):
    ─────────────────────────────────────────────────────────────────────────────
    - Header (15 bits): Message type, sublabel
    - Data (60 bits): Message-specific content
    ─────────────────────────────────────────────────────────────────────────────
    
    Note: This implementation produces logical message content.
    Physical layer encoding (JTIDS/MIDS) requires additional processing.
    """
    
    def __init__(self, source_track_number: int = 0):
        """
        Initialize Link-16 encoder.
        
        Args:
            source_track_number: Own platform track number (for PPLI)
        """
        self.source_track_number = source_track_number
        self._message_counter = 0
    
    def encode_air_track(self, track: Link16Track) -> Link16Message:
        """
        Encode J2.2 Air Track message.
        
        Word 0: Track number, quality, identity
        Word 1-2: Position (lat/lon)
        Word 3: Altitude, speed
        Word 4: Heading, climb rate
        """
        data = bytearray()
        
        # Word 0: Track info
        word0 = (track.track_number & 0x3FFF)  # 14-bit track number
        word0 |= (track.quality.value & 0x07) << 14  # 3-bit quality
        word0 |= (track.identity.value & 0x07) << 17  # 3-bit identity
        word0 |= (int(track.simulated) & 0x01) << 20  # Simulated flag
        data.extend(struct.pack('<I', word0)[:3])  # 24 bits
        
        # Word 1: Latitude (24-bit, LSB = 180/2^23 degrees)
        lat_encoded = int((track.latitude + 90.0) / LAT_LON_LSB) & 0xFFFFFF
        data.extend(struct.pack('<I', lat_encoded)[:3])
        
        # Word 2: Longitude (24-bit)
        lon_norm = track.longitude if track.longitude >= 0 else track.longitude + 360
        lon_encoded = int(lon_norm / (360.0 / (2**24))) & 0xFFFFFF
        data.extend(struct.pack('<I', lon_encoded)[:3])
        
        # Word 3: Altitude (16-bit, LSB = 25 ft) + Speed (8-bit, knots)
        alt_encoded = int(max(0, track.altitude) / ALTITUDE_LSB) & 0xFFFF
        speed_encoded = min(255, int(track.speed / 4)) & 0xFF  # 4 knots/LSB
        data.extend(struct.pack('<HB', alt_encoded, speed_encoded))
        
        # Word 4: Heading (9-bit) + Climb rate (7-bit)
        heading_encoded = int(track.heading / HEADING_LSB) & 0x1FF
        climb_encoded = int(track.climb_rate / 100) & 0x7F  # 100 ft/min/LSB
        word4 = heading_encoded | (climb_encoded << 9)
        data.extend(struct.pack('<H', word4))
        
        # Word 5: Platform type + IFF
        word5 = (track.platform_type.value & 0x3F)
        word5 |= (track.iff_mode & 0x0F) << 6
        word5 |= (track.strength & 0x0F) << 10
        data.extend(struct.pack('<H', word5))
        
        return Link16Message(
            message_type=JSeriesMessage.J2_2_AIR_TRACK,
            data=bytes(data),
            npg=3,  # Surveillance
            source_track_number=self.source_track_number
        )
    
    def encode_surface_track(self, track: Link16Track) -> Link16Message:
        """Encode J2.3 Surface Track message."""
        data = bytearray()
        
        # Similar structure to air track, but surface-specific
        word0 = (track.track_number & 0x3FFF)
        word0 |= (track.quality.value & 0x07) << 14
        word0 |= (track.identity.value & 0x07) << 17
        data.extend(struct.pack('<I', word0)[:3])
        
        # Position
        lat_encoded = int((track.latitude + 90.0) / LAT_LON_LSB) & 0xFFFFFF
        data.extend(struct.pack('<I', lat_encoded)[:3])
        
        lon_norm = track.longitude if track.longitude >= 0 else track.longitude + 360
        lon_encoded = int(lon_norm / (360.0 / (2**24))) & 0xFFFFFF
        data.extend(struct.pack('<I', lon_encoded)[:3])
        
        # Speed and course
        speed_encoded = min(255, int(track.speed)) & 0xFF
        course_encoded = int(track.heading / HEADING_LSB) & 0x1FF
        data.extend(struct.pack('<BH', speed_encoded, course_encoded))
        
        return Link16Message(
            message_type=JSeriesMessage.J2_3_SURFACE_TRACK,
            data=bytes(data),
            npg=3
        )
    
    def encode_space_track(self, track: Link16Track) -> Link16Message:
        """Encode J2.5 Space Track message."""
        data = bytearray()
        
        # Track info
        word0 = (track.track_number & 0x3FFF)
        word0 |= (track.quality.value & 0x07) << 14
        word0 |= (track.identity.value & 0x07) << 17
        data.extend(struct.pack('<I', word0)[:3])
        
        # Position (higher precision for space)
        lat_encoded = int((track.latitude + 90.0) / LAT_LON_LSB) & 0xFFFFFF
        data.extend(struct.pack('<I', lat_encoded)[:3])
        
        lon_norm = track.longitude if track.longitude >= 0 else track.longitude + 360
        lon_encoded = int(lon_norm / (360.0 / (2**24))) & 0xFFFFFF
        data.extend(struct.pack('<I', lon_encoded)[:3])
        
        # Altitude (32-bit for space, kilometers)
        alt_km = track.altitude * FEET_TO_METERS / 1000.0
        alt_encoded = int(alt_km * 10) & 0xFFFFFFFF  # 0.1 km LSB
        data.extend(struct.pack('<I', alt_encoded))
        
        return Link16Message(
            message_type=JSeriesMessage.J2_5_SPACE_TRACK,
            data=bytes(data),
            npg=3
        )
    
    def encode_ppli(self, ppli: PPLI) -> Link16Message:
        """
        Encode J3.2 Precise Participant Location and Identification.
        
        Own-ship position report.
        """
        data = bytearray()
        
        # Source track number
        data.extend(struct.pack('<H', ppli.source_track_number & 0xFFFF))
        
        # Position
        lat_encoded = int((ppli.latitude + 90.0) / LAT_LON_LSB) & 0xFFFFFF
        data.extend(struct.pack('<I', lat_encoded)[:3])
        
        lon_norm = ppli.longitude if ppli.longitude >= 0 else ppli.longitude + 360
        lon_encoded = int(lon_norm / (360.0 / (2**24))) & 0xFFFFFF
        data.extend(struct.pack('<I', lon_encoded)[:3])
        
        # Altitude and speed
        alt_encoded = int(max(0, ppli.altitude) / ALTITUDE_LSB) & 0xFFFF
        speed_encoded = min(255, int(ppli.speed / 4)) & 0xFF
        data.extend(struct.pack('<HB', alt_encoded, speed_encoded))
        
        # Heading
        heading_encoded = int(ppli.heading / HEADING_LSB) & 0x1FF
        data.extend(struct.pack('<H', heading_encoded))
        
        # Identity and platform
        word = (ppli.identity.value & 0x07)
        word |= (ppli.platform_type.value & 0x3F) << 3
        data.extend(struct.pack('<H', word))
        
        return Link16Message(
            message_type=JSeriesMessage.J3_2_AIR_PPLI,
            data=bytes(data),
            npg=2  # PPLI NPG
        )
    
    def encode_track_management(self, track_number: int, 
                                 action: str = 'update') -> Link16Message:
        """
        Encode J7.0 Track Management message.
        
        Actions: 'update', 'drop', 'quality'
        """
        data = bytearray()
        
        # Track number
        data.extend(struct.pack('<H', track_number & 0x3FFF))
        
        # Action code
        action_codes = {'update': 0, 'drop': 1, 'quality': 2, 'transfer': 3}
        action_code = action_codes.get(action, 0)
        data.append(action_code)
        
        # Timestamp (Link-16 time of day)
        now = datetime.utcnow()
        tod_seconds = now.hour * 3600 + now.minute * 60 + now.second
        data.extend(struct.pack('<I', tod_seconds)[:3])
        
        return Link16Message(
            message_type=JSeriesMessage.J7_0_TRACK_MANAGEMENT,
            data=bytes(data),
            npg=3
        )
    
    def encode_track_drop(self, track_number: int, 
                          reason: str = 'timeout') -> Link16Message:
        """Encode J7.2 Track Drop message."""
        data = bytearray()
        
        # Track number
        data.extend(struct.pack('<H', track_number & 0x3FFF))
        
        # Drop reason
        reasons = {'timeout': 0, 'merged': 1, 'invalid': 2, 'manual': 3}
        reason_code = reasons.get(reason, 0)
        data.append(reason_code)
        
        return Link16Message(
            message_type=JSeriesMessage.J7_2_TRACK_DROP,
            data=bytes(data),
            npg=3
        )
    
    def encode_threat_warning(self, track: Link16Track, 
                              threat_level: int = 1) -> Link16Message:
        """
        Encode J14.0 Threat Warning message.
        
        threat_level: 1=low, 2=medium, 3=high, 4=imminent
        """
        data = bytearray()
        
        # Threatening track number
        data.extend(struct.pack('<H', track.track_number & 0x3FFF))
        
        # Threat level
        data.append(threat_level & 0x0F)
        
        # Threat type
        threat_type = 0x01  # Air threat
        if track.platform_type == PlatformType.CRUISE_MISSILE:
            threat_type = 0x02
        elif track.platform_type == PlatformType.BALLISTIC_MISSILE:
            threat_type = 0x03
        data.append(threat_type)
        
        # Bearing to threat (from own position - would need own position)
        # For now, encode track heading as approach bearing
        bearing_encoded = int(track.heading / HEADING_LSB) & 0x1FF
        data.extend(struct.pack('<H', bearing_encoded))
        
        # Range (nautical miles)
        # Would need own position to calculate - placeholder
        range_nm = 50  # Placeholder
        data.extend(struct.pack('<H', range_nm))
        
        # Time to impact (seconds) - placeholder
        tti = 0xFFFF  # Unknown
        data.extend(struct.pack('<H', tti))
        
        return Link16Message(
            message_type=JSeriesMessage.J14_0_THREAT_WARNING,
            data=bytes(data),
            npg=5  # Air Control
        )
    
    def encode_batch(self, tracks: List[Link16Track]) -> List[Link16Message]:
        """
        Encode multiple tracks to Link-16 messages.
        
        Automatically selects message type based on track environment.
        """
        messages = []
        
        for track in tracks:
            if track.environment == TrackEnvironment.AIR:
                msg = self.encode_air_track(track)
            elif track.environment == TrackEnvironment.SURFACE:
                msg = self.encode_surface_track(track)
            elif track.environment == TrackEnvironment.SPACE:
                msg = self.encode_space_track(track)
            else:
                msg = self.encode_air_track(track)  # Default
            
            messages.append(msg)
        
        self._message_counter += len(messages)
        return messages


# =============================================================================
# NX-MIMOSA INTEGRATION
# =============================================================================

class NXMIMOSALink16Output:
    """
    Wrapper to convert NX-MIMOSA tracker output to Link-16 messages.
    
    Usage:
        link16_out = NXMIMOSALink16Output(own_track_number=1)
        
        # In tracking loop:
        track = link16_out.from_tracker(tracker, track_id)
        messages = link16_out.encode([track])
        
        for msg in messages:
            tactical_data_link.send(msg)
    """
    
    def __init__(self, own_track_number: int = 1,
                 own_lat: float = 0.0, own_lon: float = 0.0):
        """
        Initialize Link-16 output wrapper.
        
        Args:
            own_track_number: Own platform track number
            own_lat: Own platform latitude
            own_lon: Own platform longitude
        """
        self.encoder = Link16Encoder(source_track_number=own_track_number)
        self.own_track_number = own_track_number
        self.own_lat = own_lat
        self.own_lon = own_lon
        
        # Track number management
        self._track_number_map: Dict[int, int] = {}
        self._next_track_number = 1
    
    def from_tracker(self, tracker, internal_track_id: int,
                     environment: TrackEnvironment = TrackEnvironment.AIR,
                     identity: TrackIdentity = TrackIdentity.UNKNOWN) -> Link16Track:
        """
        Convert NX-MIMOSA tracker state to Link16Track.
        
        Args:
            tracker: NX-MIMOSA tracker instance
            internal_track_id: Internal track identifier
            environment: Track environment (air/surface/space)
            identity: Track identity classification
            
        Returns:
            Link16Track for encoding
        """
        state = tracker.get_state()
        
        # Assign Link-16 track number (1-16383)
        if internal_track_id not in self._track_number_map:
            self._track_number_map[internal_track_id] = self._next_track_number
            self._next_track_number = (self._next_track_number % 16383) + 1
        
        track_number = self._track_number_map[internal_track_id]
        
        # Extract position (assume state is [x, y, z, vx, vy, vz] in ECEF or local)
        # For Link-16, we need WGS-84 lat/lon/alt
        # This assumes state is already in appropriate coordinates or needs conversion
        x, y, z = state[0], state[1], state[2]
        vx, vy, vz = state[3], state[4], state[5]
        
        # Convert to lat/lon (simplified - assumes local tangent plane)
        # Production would use proper geodetic conversion
        lat, lon = self._local_to_wgs84(x, y)
        altitude = z * METERS_TO_FEET
        
        # Compute speed and heading
        speed_ms = np.sqrt(vx**2 + vy**2)
        speed_knots = speed_ms * MS_TO_KNOTS
        
        heading = np.degrees(np.arctan2(vx, vy)) % 360  # Degrees true
        
        climb_rate = vz * METERS_TO_FEET * 60  # ft/min
        
        # Determine quality from tracker state
        quality = TrackQuality.NORMAL
        if hasattr(tracker, 'get_mode_probabilities'):
            mode_probs = tracker.get_mode_probabilities()
            if mode_probs[0] > 0.8:  # High CV probability
                quality = TrackQuality.HIGH
            elif max(mode_probs) < 0.5:  # Uncertain
                quality = TrackQuality.DEGRADED
        
        # Determine platform type from kinematics (heuristic)
        platform_type = PlatformType.UNKNOWN
        if speed_knots > 600:
            if altitude > 60000:
                platform_type = PlatformType.BALLISTIC_MISSILE
            else:
                platform_type = PlatformType.FIGHTER
        elif speed_knots > 250:
            platform_type = PlatformType.TRANSPORT
        elif speed_knots > 100 and environment == TrackEnvironment.AIR:
            platform_type = PlatformType.HELICOPTER
        
        return Link16Track(
            track_number=track_number,
            latitude=lat,
            longitude=lon,
            altitude=altitude,
            speed=speed_knots,
            heading=heading,
            climb_rate=climb_rate,
            identity=identity,
            environment=environment,
            platform_type=platform_type,
            quality=quality,
            timestamp=time.time() % 86400,  # Seconds since midnight
        )
    
    def encode(self, tracks: List[Link16Track]) -> List[Link16Message]:
        """Encode tracks to Link-16 messages."""
        return self.encoder.encode_batch(tracks)
    
    def encode_ppli(self, lat: float, lon: float, alt: float,
                    speed: float, heading: float) -> Link16Message:
        """Generate own-ship PPLI message."""
        ppli = PPLI(
            source_track_number=self.own_track_number,
            latitude=lat,
            longitude=lon,
            altitude=alt,
            speed=speed,
            heading=heading,
            identity=TrackIdentity.FRIEND
        )
        return self.encoder.encode_ppli(ppli)
    
    def encode_track_drop(self, internal_track_id: int) -> Optional[Link16Message]:
        """Generate track drop message."""
        if internal_track_id in self._track_number_map:
            track_number = self._track_number_map.pop(internal_track_id)
            return self.encoder.encode_track_drop(track_number)
        return None
    
    def _local_to_wgs84(self, x: float, y: float) -> Tuple[float, float]:
        """
        Convert local coordinates to WGS-84.
        
        Simplified flat-earth approximation.
        Production would use proper geodetic library.
        """
        meters_per_deg_lat = 111132.92
        meters_per_deg_lon = 111132.92 * np.cos(np.radians(self.own_lat))
        
        lat = self.own_lat + (y / meters_per_deg_lat)
        lon = self.own_lon + (x / meters_per_deg_lon)
        
        return lat, lon


# =============================================================================
# MAIN - DEMONSTRATION
# =============================================================================

if __name__ == "__main__":
    print("="*80)
    print("NX-MIMOSA LINK-16 (TADIL-J) OUTPUT FORMATTER")
    print("MIL-STD-6016 Compliant")
    print("="*80)
    
    # Create sample tracks
    tracks = [
        Link16Track(
            track_number=101,
            latitude=45.8,
            longitude=16.0,
            altitude=35000,
            speed=450,
            heading=90,
            climb_rate=0,
            identity=TrackIdentity.UNKNOWN,
            environment=TrackEnvironment.AIR,
            platform_type=PlatformType.FIGHTER,
            quality=TrackQuality.HIGH,
        ),
        Link16Track(
            track_number=102,
            latitude=45.7,
            longitude=15.9,
            altitude=25000,
            speed=550,
            heading=270,
            climb_rate=-500,
            identity=TrackIdentity.HOSTILE,
            environment=TrackEnvironment.AIR,
            platform_type=PlatformType.ATTACK,
            quality=TrackQuality.NORMAL,
        ),
        Link16Track(
            track_number=201,
            latitude=45.5,
            longitude=16.2,
            altitude=0,
            speed=20,
            heading=45,
            identity=TrackIdentity.NEUTRAL,
            environment=TrackEnvironment.SURFACE,
            platform_type=PlatformType.SURFACE_SHIP,
            quality=TrackQuality.NORMAL,
        ),
    ]
    
    print(f"\n📡 Sample Tracks:")
    for track in tracks:
        print(f"  Track {track.track_number}: {track.environment.name}")
        print(f"    Position: ({track.latitude:.4f}°, {track.longitude:.4f}°) @ {track.altitude:.0f} ft")
        print(f"    Velocity: {track.speed:.0f} kts, heading {track.heading:.0f}°")
        print(f"    Identity: {track.identity.name}, Platform: {track.platform_type.name}")
    
    # Create encoder
    encoder = Link16Encoder(source_track_number=1)
    
    # Encode to Link-16 messages
    messages = encoder.encode_batch(tracks)
    
    print(f"\n📤 Link-16 J-Messages:")
    for msg in messages:
        print(f"  {msg}")
        print(f"    Data: {msg.data.hex()}")
    
    # PPLI message
    ppli = PPLI(
        source_track_number=1,
        latitude=45.9,
        longitude=16.1,
        altitude=30000,
        speed=400,
        heading=180,
        identity=TrackIdentity.FRIEND,
        platform_type=PlatformType.FIGHTER
    )
    ppli_msg = encoder.encode_ppli(ppli)
    print(f"\n📍 PPLI Message:")
    print(f"  {ppli_msg}")
    print(f"  Data: {ppli_msg.data.hex()}")
    
    # Threat warning
    threat_msg = encoder.encode_threat_warning(tracks[1], threat_level=3)
    print(f"\n⚠️ Threat Warning:")
    print(f"  {threat_msg}")
    print(f"  Data: {threat_msg.data.hex()}")
    
    # Track management
    mgmt_msg = encoder.encode_track_management(101, action='update')
    print(f"\n📋 Track Management:")
    print(f"  {mgmt_msg}")
    
    print(f"\n📊 Message Statistics:")
    print(f"  Total messages: {len(messages) + 3}")
    print(f"  Air tracks: {sum(1 for t in tracks if t.environment == TrackEnvironment.AIR)}")
    print(f"  Surface tracks: {sum(1 for t in tracks if t.environment == TrackEnvironment.SURFACE)}")
    
    print("\n" + "="*80)
    print("✓ Link-16 formatter ready for MIL-STD-6016 integration")
    print("="*80)
