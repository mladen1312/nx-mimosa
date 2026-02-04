#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════════════════════════════
NX-MIMOSA - CCSDS OUTPUT FORMATTER FOR SPACE APPLICATIONS
═══════════════════════════════════════════════════════════════════════════════════════════════════════

CCSDS (Consultative Committee for Space Data Systems) Telemetry Formatter
For Space Situational Awareness (SSA) and Satellite Tracking

Standards Compliance:
- CCSDS 133.0-B-2 (Space Packet Protocol)
- CCSDS 301.0-B-4 (Time Code Formats)
- CCSDS 502.0-B-2 (Orbit Data Messages)
- CCSDS 503.0-B-2 (Tracking Data Message)

Data Products:
- TDM: Tracking Data Message (range, Doppler, angles)
- OEM: Orbit Ephemeris Message (state vectors)
- CDM: Conjunction Data Message (collision assessment)

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

# CCSDS Primary Header
CCSDS_VERSION = 0
CCSDS_TYPE_TM = 0  # Telemetry
CCSDS_TYPE_TC = 1  # Telecommand

# Packet types
PACKET_TYPE_TDM = 0x01  # Tracking Data
PACKET_TYPE_OEM = 0x02  # Orbit Ephemeris
PACKET_TYPE_CDM = 0x03  # Conjunction Data

# Time code formats
CDS_TIME_CODE = 0x44  # CCSDS Day Segmented
CUC_TIME_CODE = 0x1E  # CCSDS Unsegmented


# =============================================================================
# ENUMERATIONS
# =============================================================================

class DataType(IntEnum):
    """Tracking data types."""
    RANGE = 1
    DOPPLER_INSTANTANEOUS = 2
    DOPPLER_INTEGRATED = 3
    ANGLE_1 = 4  # Azimuth
    ANGLE_2 = 5  # Elevation
    RECEIVE_FREQ = 6
    TRANSMIT_FREQ = 7


class ReferenceFrame(IntEnum):
    """Reference frame identifiers."""
    EME2000 = 1       # J2000 Earth Mean Equator
    GCRF = 2          # Geocentric Celestial Reference Frame
    ITRF = 3          # International Terrestrial Reference Frame
    TEME = 4          # True Equator Mean Equinox
    TOD = 5           # True of Date


class TrackType(IntEnum):
    """Track type identifiers."""
    DEBRIS = 1
    PAYLOAD = 2
    ROCKET_BODY = 3
    UNKNOWN = 4


# =============================================================================
# DATA STRUCTURES
# =============================================================================

@dataclass
class CCSDSPacketHeader:
    """CCSDS Space Packet Primary Header (6 bytes)."""
    version: int = CCSDS_VERSION
    type: int = CCSDS_TYPE_TM
    sec_hdr_flag: int = 1
    apid: int = 0           # Application Process ID (0-2047)
    seq_flags: int = 3      # 3 = standalone packet
    seq_count: int = 0      # Packet sequence count (0-16383)
    data_length: int = 0    # Data field length - 1
    
    def encode(self) -> bytes:
        """Encode primary header to 6 bytes."""
        # Word 1: Version (3) | Type (1) | Sec Hdr (1) | APID (11)
        word1 = ((self.version & 0x07) << 13) | \
                ((self.type & 0x01) << 12) | \
                ((self.sec_hdr_flag & 0x01) << 11) | \
                (self.apid & 0x7FF)
        
        # Word 2: Seq Flags (2) | Seq Count (14)
        word2 = ((self.seq_flags & 0x03) << 14) | (self.seq_count & 0x3FFF)
        
        # Word 3: Data Length (16)
        word3 = self.data_length & 0xFFFF
        
        return struct.pack('>HHH', word1, word2, word3)


@dataclass
class CCSDSTimeCode:
    """CCSDS Time Code (CDS format)."""
    days: int = 0           # Days since epoch (J2000 or custom)
    ms_of_day: int = 0      # Milliseconds of day
    us_of_ms: int = 0       # Microseconds of millisecond
    
    @classmethod
    def from_datetime(cls, dt: datetime) -> 'CCSDSTimeCode':
        """Create time code from datetime."""
        # J2000 epoch: 2000-01-01T12:00:00
        j2000 = datetime(2000, 1, 1, 12, 0, 0, tzinfo=timezone.utc)
        
        if dt.tzinfo is None:
            dt = dt.replace(tzinfo=timezone.utc)
        
        delta = dt - j2000
        
        days = delta.days
        seconds = delta.seconds
        microseconds = delta.microseconds
        
        ms_of_day = seconds * 1000 + microseconds // 1000
        us_of_ms = microseconds % 1000
        
        return cls(days=days, ms_of_day=ms_of_day, us_of_ms=us_of_ms)
    
    def encode(self) -> bytes:
        """Encode to 8 bytes (CDS-A format)."""
        return struct.pack('>HIH', self.days, self.ms_of_day, self.us_of_ms)


@dataclass
class TrackingDataPoint:
    """Single tracking data observation."""
    timestamp: datetime
    data_type: DataType
    value: float
    uncertainty: float = 0.0
    
    station_id: str = ""
    target_id: str = ""


@dataclass 
class OrbitStateVector:
    """Orbit state vector in CCSDS format."""
    epoch: datetime
    
    # Position (km)
    x: float
    y: float
    z: float
    
    # Velocity (km/s)
    vx: float
    vy: float
    vz: float
    
    # Covariance (optional, 6x6 in km and km/s)
    covariance: Optional[np.ndarray] = None
    
    # Reference frame
    ref_frame: ReferenceFrame = ReferenceFrame.EME2000
    
    # Object identification
    object_id: str = ""
    object_name: str = ""
    norad_cat_id: int = 0


@dataclass
class ConjunctionData:
    """Conjunction (collision) assessment data."""
    tca: datetime               # Time of Closest Approach
    miss_distance: float        # km
    
    object1_id: str
    object2_id: str
    
    collision_probability: float = 0.0
    
    # Relative state at TCA
    relative_position: Tuple[float, float, float] = (0, 0, 0)  # km
    relative_velocity: Tuple[float, float, float] = (0, 0, 0)  # km/s


# =============================================================================
# CCSDS PACKET ENCODER
# =============================================================================

class CCSDSEncoder:
    """
    Encodes tracking data to CCSDS Space Packets.
    
    Supports:
    - Tracking Data Message (TDM)
    - Orbit Ephemeris Message (OEM)
    - Conjunction Data Message (CDM)
    """
    
    def __init__(self, apid: int = 100, originator: str = "NEXELLUM"):
        """
        Initialize CCSDS encoder.
        
        Args:
            apid: Application Process ID
            originator: Message originator identifier
        """
        self.apid = apid
        self.originator = originator
        self._seq_count = 0
    
    def _get_seq_count(self) -> int:
        """Get and increment sequence counter."""
        count = self._seq_count
        self._seq_count = (self._seq_count + 1) % 16384
        return count
    
    def encode_tdm(self, observations: List[TrackingDataPoint]) -> bytes:
        """
        Encode Tracking Data Message.
        
        TDM contains tracking observations (range, Doppler, angles).
        """
        # Build data field
        data = bytearray()
        
        # Secondary header: time code + message type
        now = datetime.now(timezone.utc)
        time_code = CCSDSTimeCode.from_datetime(now)
        data.extend(time_code.encode())
        data.append(PACKET_TYPE_TDM)
        
        # Message metadata
        data.extend(self.originator.encode('ascii')[:8].ljust(8, b'\x00'))
        
        # Number of observations
        data.extend(struct.pack('>H', len(observations)))
        
        # Encode each observation
        for obs in observations:
            # Timestamp (relative to secondary header time)
            obs_time = CCSDSTimeCode.from_datetime(obs.timestamp)
            data.extend(struct.pack('>I', obs_time.ms_of_day))
            
            # Data type
            data.append(obs.data_type.value)
            
            # Value (64-bit double)
            data.extend(struct.pack('>d', obs.value))
            
            # Uncertainty (32-bit float)
            data.extend(struct.pack('>f', obs.uncertainty))
            
            # Station ID (8 bytes, null-padded)
            data.extend(obs.station_id.encode('ascii')[:8].ljust(8, b'\x00'))
            
            # Target ID (8 bytes, null-padded)
            data.extend(obs.target_id.encode('ascii')[:8].ljust(8, b'\x00'))
        
        # Build primary header
        header = CCSDSPacketHeader(
            apid=self.apid,
            seq_count=self._get_seq_count(),
            data_length=len(data) - 1
        )
        
        return header.encode() + bytes(data)
    
    def encode_oem(self, state_vectors: List[OrbitStateVector]) -> bytes:
        """
        Encode Orbit Ephemeris Message.
        
        OEM contains predicted state vectors for an object.
        """
        data = bytearray()
        
        # Secondary header
        now = datetime.now(timezone.utc)
        time_code = CCSDSTimeCode.from_datetime(now)
        data.extend(time_code.encode())
        data.append(PACKET_TYPE_OEM)
        
        # Originator
        data.extend(self.originator.encode('ascii')[:8].ljust(8, b'\x00'))
        
        # Object ID (from first state vector)
        if state_vectors:
            obj_id = state_vectors[0].object_id
            data.extend(obj_id.encode('ascii')[:12].ljust(12, b'\x00'))
            
            # Reference frame
            data.append(state_vectors[0].ref_frame.value)
        else:
            data.extend(b'\x00' * 13)
        
        # Number of state vectors
        data.extend(struct.pack('>H', len(state_vectors)))
        
        # Encode each state vector
        for sv in state_vectors:
            # Epoch time
            epoch_time = CCSDSTimeCode.from_datetime(sv.epoch)
            data.extend(epoch_time.encode())
            
            # Position (km, 64-bit doubles)
            data.extend(struct.pack('>ddd', sv.x, sv.y, sv.z))
            
            # Velocity (km/s, 64-bit doubles)
            data.extend(struct.pack('>ddd', sv.vx, sv.vy, sv.vz))
            
            # Covariance flag
            has_cov = sv.covariance is not None
            data.append(int(has_cov))
            
            if has_cov:
                # Lower-triangular covariance (21 elements)
                for i in range(6):
                    for j in range(i + 1):
                        data.extend(struct.pack('>d', sv.covariance[i, j]))
        
        # Build primary header
        header = CCSDSPacketHeader(
            apid=self.apid,
            seq_count=self._get_seq_count(),
            data_length=len(data) - 1
        )
        
        return header.encode() + bytes(data)
    
    def encode_cdm(self, conjunction: ConjunctionData) -> bytes:
        """
        Encode Conjunction Data Message.
        
        CDM contains collision assessment between two objects.
        """
        data = bytearray()
        
        # Secondary header
        now = datetime.now(timezone.utc)
        time_code = CCSDSTimeCode.from_datetime(now)
        data.extend(time_code.encode())
        data.append(PACKET_TYPE_CDM)
        
        # Originator
        data.extend(self.originator.encode('ascii')[:8].ljust(8, b'\x00'))
        
        # Object 1 ID
        data.extend(conjunction.object1_id.encode('ascii')[:12].ljust(12, b'\x00'))
        
        # Object 2 ID
        data.extend(conjunction.object2_id.encode('ascii')[:12].ljust(12, b'\x00'))
        
        # TCA (Time of Closest Approach)
        tca_time = CCSDSTimeCode.from_datetime(conjunction.tca)
        data.extend(tca_time.encode())
        
        # Miss distance (km, 64-bit double)
        data.extend(struct.pack('>d', conjunction.miss_distance))
        
        # Collision probability (64-bit double)
        data.extend(struct.pack('>d', conjunction.collision_probability))
        
        # Relative position at TCA (km)
        data.extend(struct.pack('>ddd', *conjunction.relative_position))
        
        # Relative velocity at TCA (km/s)
        data.extend(struct.pack('>ddd', *conjunction.relative_velocity))
        
        # Build primary header
        header = CCSDSPacketHeader(
            apid=self.apid,
            seq_count=self._get_seq_count(),
            data_length=len(data) - 1
        )
        
        return header.encode() + bytes(data)


# =============================================================================
# NX-MIMOSA INTEGRATION
# =============================================================================

class NXMIMOSACCSDSOutput:
    """
    Wrapper to convert NX-MIMOSA tracker output to CCSDS packets.
    
    Usage:
        ccsds_out = NXMIMOSACCSDSOutput(apid=100)
        
        # Generate OEM from tracker
        oem = ccsds_out.from_tracker(tracker, object_id="SAT12345")
        
        # Send to ground station
        ground_station.send(oem)
    """
    
    def __init__(self, apid: int = 100, originator: str = "NEXELLUM"):
        """Initialize CCSDS output wrapper."""
        self.encoder = CCSDSEncoder(apid=apid, originator=originator)
    
    def from_tracker(self, tracker, object_id: str,
                     ref_frame: ReferenceFrame = ReferenceFrame.EME2000) -> bytes:
        """
        Convert tracker state to CCSDS OEM.
        
        Args:
            tracker: NX-MIMOSA tracker instance
            object_id: NORAD catalog ID or designator
            ref_frame: Reference frame
            
        Returns:
            CCSDS OEM packet bytes
        """
        state = tracker.get_state()
        cov = tracker.get_covariance() if hasattr(tracker, 'get_covariance') else None
        
        # Convert from meters to kilometers
        sv = OrbitStateVector(
            epoch=datetime.now(timezone.utc),
            x=state[0] / 1000.0,
            y=state[1] / 1000.0,
            z=state[2] / 1000.0,
            vx=state[3] / 1000.0,
            vy=state[4] / 1000.0,
            vz=state[5] / 1000.0,
            covariance=cov / 1e6 if cov is not None else None,  # m² to km²
            ref_frame=ref_frame,
            object_id=object_id
        )
        
        return self.encoder.encode_oem([sv])
    
    def from_measurement(self, meas_type: DataType, value: float,
                        uncertainty: float, station_id: str,
                        target_id: str) -> bytes:
        """
        Convert single measurement to CCSDS TDM.
        
        Args:
            meas_type: Type of measurement
            value: Measurement value
            uncertainty: 1-sigma uncertainty
            station_id: Ground station identifier
            target_id: Target object identifier
            
        Returns:
            CCSDS TDM packet bytes
        """
        obs = TrackingDataPoint(
            timestamp=datetime.now(timezone.utc),
            data_type=meas_type,
            value=value,
            uncertainty=uncertainty,
            station_id=station_id,
            target_id=target_id
        )
        
        return self.encoder.encode_tdm([obs])
    
    def encode_conjunction(self, obj1_id: str, obj2_id: str,
                          tca: datetime, miss_distance_km: float,
                          collision_prob: float = 0.0) -> bytes:
        """
        Encode conjunction assessment.
        
        Args:
            obj1_id: Primary object ID
            obj2_id: Secondary object ID
            tca: Time of closest approach
            miss_distance_km: Miss distance in km
            collision_prob: Collision probability
            
        Returns:
            CCSDS CDM packet bytes
        """
        cdm = ConjunctionData(
            tca=tca,
            miss_distance=miss_distance_km,
            object1_id=obj1_id,
            object2_id=obj2_id,
            collision_probability=collision_prob
        )
        
        return self.encoder.encode_cdm(cdm)


# =============================================================================
# MAIN - DEMONSTRATION
# =============================================================================

if __name__ == "__main__":
    print("="*80)
    print("NX-MIMOSA CCSDS OUTPUT FORMATTER")
    print("For Space Situational Awareness (SSA)")
    print("="*80)
    
    # Create encoder
    encoder = CCSDSEncoder(apid=100, originator="NEXELLUM")
    
    # Create sample tracking observations
    observations = [
        TrackingDataPoint(
            timestamp=datetime.now(timezone.utc),
            data_type=DataType.RANGE,
            value=35786.0,  # km (GEO altitude)
            uncertainty=0.001,
            station_id="DIEGO",
            target_id="25544"  # ISS
        ),
        TrackingDataPoint(
            timestamp=datetime.now(timezone.utc),
            data_type=DataType.DOPPLER_INSTANTANEOUS,
            value=-0.5,  # km/s
            uncertainty=0.0001,
            station_id="DIEGO",
            target_id="25544"
        ),
    ]
    
    print(f"\n📡 Tracking Data Message (TDM):")
    tdm = encoder.encode_tdm(observations)
    print(f"  Length: {len(tdm)} bytes")
    print(f"  Header: {tdm[:6].hex()}")
    print(f"  Data: {tdm[6:30].hex()}...")
    
    # Create sample state vector
    state_vectors = [
        OrbitStateVector(
            epoch=datetime.now(timezone.utc),
            x=-6045.0, y=-3490.0, z=2500.0,      # km (LEO)
            vx=-3.457, vy=6.618, vz=2.533,       # km/s
            ref_frame=ReferenceFrame.EME2000,
            object_id="25544",
            object_name="ISS (ZARYA)"
        ),
    ]
    
    print(f"\n🛰️ Orbit Ephemeris Message (OEM):")
    oem = encoder.encode_oem(state_vectors)
    print(f"  Length: {len(oem)} bytes")
    print(f"  Header: {oem[:6].hex()}")
    
    # Create sample conjunction
    conjunction = ConjunctionData(
        tca=datetime.now(timezone.utc),
        miss_distance=0.5,  # km
        object1_id="25544",
        object2_id="37820",
        collision_probability=1e-5,
        relative_position=(0.3, 0.2, 0.1),
        relative_velocity=(0.01, -0.02, 0.005)
    )
    
    print(f"\n⚠️ Conjunction Data Message (CDM):")
    cdm = encoder.encode_cdm(conjunction)
    print(f"  Length: {len(cdm)} bytes")
    print(f"  Miss Distance: {conjunction.miss_distance} km")
    print(f"  Collision Probability: {conjunction.collision_probability:.2e}")
    
    print("\n" + "="*80)
    print("✓ CCSDS formatter ready for space applications")
    print("="*80)
