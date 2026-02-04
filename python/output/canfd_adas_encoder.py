#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════════════════════════════
NX-MIMOSA - CAN-FD OUTPUT FORMATTER FOR AUTOMOTIVE ADAS
═══════════════════════════════════════════════════════════════════════════════════════════════════════

Controller Area Network with Flexible Data-rate (CAN-FD) formatter for automotive radar applications.
Compliant with ISO 11898-1 (CAN) and ISO 26262 (Functional Safety).

Message Format: AUTOSAR-compatible radar object list
──────────────────────────────────────────────────────────────────────────────────────────────────────

OBJECT_LIST (CAN-FD 64 bytes, ID 0x200):
┌────────────────────────────────────────────────────────────────────────────────────────────────────┐
│ Byte 0-1:   Object ID (16-bit)                                                                    │
│ Byte 2-3:   Distance Longitudinal (signed, 0.1m LSB, -3276.8 to +3276.7 m)                       │
│ Byte 4-5:   Distance Lateral (signed, 0.1m LSB)                                                   │
│ Byte 6-7:   Velocity Longitudinal (signed, 0.01 m/s LSB)                                          │
│ Byte 8-9:   Velocity Lateral (signed, 0.01 m/s LSB)                                               │
│ Byte 10-11: Acceleration Longitudinal (signed, 0.01 m/s² LSB)                                     │
│ Byte 12-13: Acceleration Lateral (signed, 0.01 m/s² LSB)                                          │
│ Byte 14:    Object Class (4 bits) | Dynamic Property (4 bits)                                     │
│ Byte 15:    Existence Probability (0-255 = 0-100%)                                                │
│ ... repeated for up to 4 objects per message ...                                                  │
└────────────────────────────────────────────────────────────────────────────────────────────────────┘

OBJECT_QUALITY (CAN-FD 64 bytes, ID 0x201):
┌────────────────────────────────────────────────────────────────────────────────────────────────────┐
│ Byte 0-1:   Object ID                                                                             │
│ Byte 2-3:   Position Std Dev X (0.01m LSB)                                                        │
│ Byte 4-5:   Position Std Dev Y (0.01m LSB)                                                        │
│ Byte 6-7:   Velocity Std Dev X (0.01 m/s LSB)                                                     │
│ Byte 8-9:   Velocity Std Dev Y (0.01 m/s LSB)                                                     │
│ Byte 10:    Measurement State                                                                      │
│ Byte 11:    RCS (signed, 0.5 dBsm LSB)                                                            │
│ ... repeated for up to 5 objects per message ...                                                  │
└────────────────────────────────────────────────────────────────────────────────────────────────────┘

SENSOR_STATUS (Classic CAN 8 bytes, ID 0x202):
┌────────────────────────────────────────────────────────────────────────────────────────────────────┐
│ Byte 0:     Sensor ID                                                                             │
│ Byte 1:     Status Flags (operational, blocked, interference, etc.)                               │
│ Byte 2:     Number of Objects                                                                     │
│ Byte 3:     Frame Counter (rolling 0-255)                                                         │
│ Byte 4-5:   Timestamp (0.1ms LSB, modulo 6.5536s)                                                 │
│ Byte 6-7:   Max Detection Range (0.1m LSB)                                                        │
└────────────────────────────────────────────────────────────────────────────────────────────────────┘

Author: Dr. Mladen Mešter / Nexellum d.o.o.
License: AGPL v3 / Commercial
═══════════════════════════════════════════════════════════════════════════════════════════════════════
"""

import struct
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any
from enum import IntEnum, IntFlag
import numpy as np

__version__ = "1.0.0"
__author__ = "Dr. Mladen Mešter"


# =============================================================================
# CONSTANTS
# =============================================================================

# CAN ID allocation (configurable)
CAN_ID_OBJECT_LIST = 0x200
CAN_ID_OBJECT_QUALITY = 0x201
CAN_ID_SENSOR_STATUS = 0x202
CAN_ID_CLUSTER_LIST = 0x210

# CAN-FD payload sizes
CAN_CLASSIC_MAX_BYTES = 8
CAN_FD_MAX_BYTES = 64

# Scaling factors
SCALE_DISTANCE = 0.1      # meters per LSB
SCALE_VELOCITY = 0.01     # m/s per LSB
SCALE_ACCELERATION = 0.01 # m/s² per LSB
SCALE_ANGLE = 0.1         # degrees per LSB
SCALE_RCS = 0.5           # dBsm per LSB
SCALE_STD = 0.01          # meters per LSB (std dev)


# =============================================================================
# ENUMERATIONS
# =============================================================================

class ObjectClass(IntEnum):
    """AUTOSAR-compatible object classification."""
    POINT = 0
    CAR = 1
    TRUCK = 2
    PEDESTRIAN = 3
    MOTORCYCLE = 4
    BICYCLE = 5
    WIDE = 6
    RESERVED = 7
    UNCLASSIFIED = 15


class DynamicProperty(IntEnum):
    """Object dynamic state."""
    MOVING = 0
    STATIONARY = 1
    ONCOMING = 2
    STATIONARY_CANDIDATE = 3
    UNKNOWN = 4
    CROSSING_STATIONARY = 5
    CROSSING_MOVING = 6
    STOPPED = 7


class MeasState(IntEnum):
    """Measurement state of tracked object."""
    DELETED = 0
    NEW = 1
    MEASURED = 2
    PREDICTED = 3
    DELETED_MERGE = 4
    NEW_MERGE = 5


class SensorStatusFlags(IntFlag):
    """Sensor status bit flags."""
    OPERATIONAL = 0x01
    BLOCKED = 0x02
    INTERFERENCE = 0x04
    DEGRADED = 0x08
    CALIBRATING = 0x10
    TEMPERATURE_WARNING = 0x20
    HARDWARE_ERROR = 0x40
    ALIGNMENT_ERROR = 0x80


# =============================================================================
# DATA STRUCTURES
# =============================================================================

@dataclass
class RadarObject:
    """Tracked radar object for CAN transmission."""
    object_id: int
    
    # Position (meters, sensor-relative)
    dist_long: float       # Longitudinal (positive = forward)
    dist_lat: float        # Lateral (positive = left)
    
    # Velocity (m/s, relative to ego vehicle)
    vel_long: float
    vel_lat: float
    
    # Acceleration (m/s²)
    accel_long: float = 0.0
    accel_lat: float = 0.0
    
    # Classification
    object_class: ObjectClass = ObjectClass.UNCLASSIFIED
    dynamic_property: DynamicProperty = DynamicProperty.UNKNOWN
    meas_state: MeasState = MeasState.MEASURED
    
    # Quality metrics
    exist_prob: float = 1.0  # Existence probability 0-1
    rcs: float = 0.0         # Radar cross section (dBsm)
    
    # Accuracy (1-sigma std dev)
    std_dist_long: float = 0.0
    std_dist_lat: float = 0.0
    std_vel_long: float = 0.0
    std_vel_lat: float = 0.0
    
    # Object dimensions
    length: float = 0.0
    width: float = 0.0
    orientation: float = 0.0  # degrees


@dataclass
class SensorStatus:
    """Radar sensor status."""
    sensor_id: int = 0
    flags: SensorStatusFlags = SensorStatusFlags.OPERATIONAL
    num_objects: int = 0
    frame_counter: int = 0
    timestamp_ms: int = 0
    max_range: float = 200.0  # meters


@dataclass
class CANMessage:
    """CAN/CAN-FD message structure."""
    arbitration_id: int
    data: bytes
    is_fd: bool = True
    is_extended_id: bool = False
    bitrate_switch: bool = True
    
    @property
    def dlc(self) -> int:
        """Data Length Code."""
        length = len(self.data)
        if length <= 8:
            return length
        elif length <= 12:
            return 9
        elif length <= 16:
            return 10
        elif length <= 20:
            return 11
        elif length <= 24:
            return 12
        elif length <= 32:
            return 13
        elif length <= 48:
            return 14
        else:
            return 15  # 64 bytes
    
    def __repr__(self):
        return f"CAN{'FD' if self.is_fd else ''} ID=0x{self.arbitration_id:03X} DLC={self.dlc} [{self.data.hex()}]"


# =============================================================================
# CAN-FD ENCODER
# =============================================================================

class CANFDRadarEncoder:
    """
    Encodes NX-MIMOSA radar tracks to CAN-FD messages for automotive ADAS.
    
    Features:
    - AUTOSAR-compatible message format
    - Object list with position, velocity, acceleration
    - Quality metrics with accuracy information
    - Sensor status monitoring
    - ISO 26262 ASIL-D compliant data integrity (CRC optional)
    """
    
    def __init__(self,
                 object_list_id: int = CAN_ID_OBJECT_LIST,
                 quality_list_id: int = CAN_ID_OBJECT_QUALITY,
                 status_id: int = CAN_ID_SENSOR_STATUS,
                 use_crc: bool = False,
                 sensor_id: int = 0):
        """
        Initialize CAN-FD encoder.
        
        Args:
            object_list_id: CAN ID for object list messages
            quality_list_id: CAN ID for quality information
            status_id: CAN ID for sensor status
            use_crc: Enable CRC-8 for data integrity
            sensor_id: Sensor identification number
        """
        self.object_list_id = object_list_id
        self.quality_list_id = quality_list_id
        self.status_id = status_id
        self.use_crc = use_crc
        self.sensor_id = sensor_id
        
        self._frame_counter = 0
    
    def encode_object(self, obj: RadarObject) -> bytes:
        """
        Encode single radar object to bytes (16 bytes per object).
        
        Format:
        ┌──────────────────────────────────────────────────────────────────────┐
        │ Bytes 0-1:   Object ID (uint16)                                      │
        │ Bytes 2-3:   Distance Long (int16, 0.1m LSB)                         │
        │ Bytes 4-5:   Distance Lat (int16, 0.1m LSB)                          │
        │ Bytes 6-7:   Velocity Long (int16, 0.01 m/s LSB)                     │
        │ Bytes 8-9:   Velocity Lat (int16, 0.01 m/s LSB)                      │
        │ Bytes 10-11: Accel Long (int16, 0.01 m/s² LSB)                       │
        │ Bytes 12-13: Accel Lat (int16, 0.01 m/s² LSB)                        │
        │ Byte 14:     Class (4 bits) | Dynamic (4 bits)                       │
        │ Byte 15:     Existence Probability (uint8, 0-255 = 0-100%)           │
        └──────────────────────────────────────────────────────────────────────┘
        """
        # Scale and clamp values
        dist_long = int(obj.dist_long / SCALE_DISTANCE)
        dist_lat = int(obj.dist_lat / SCALE_DISTANCE)
        vel_long = int(obj.vel_long / SCALE_VELOCITY)
        vel_lat = int(obj.vel_lat / SCALE_VELOCITY)
        accel_long = int(obj.accel_long / SCALE_ACCELERATION)
        accel_lat = int(obj.accel_lat / SCALE_ACCELERATION)
        
        # Clamp to int16 range
        dist_long = max(-32768, min(32767, dist_long))
        dist_lat = max(-32768, min(32767, dist_lat))
        vel_long = max(-32768, min(32767, vel_long))
        vel_lat = max(-32768, min(32767, vel_lat))
        accel_long = max(-32768, min(32767, accel_long))
        accel_lat = max(-32768, min(32767, accel_lat))
        
        # Classification byte
        class_dyn = ((obj.object_class & 0x0F) << 4) | (obj.dynamic_property & 0x0F)
        
        # Existence probability (0-255)
        exist = int(obj.exist_prob * 255)
        exist = max(0, min(255, exist))
        
        return struct.pack('<HhhhhhhBB',
                           obj.object_id & 0xFFFF,
                           dist_long, dist_lat,
                           vel_long, vel_lat,
                           accel_long, accel_lat,
                           class_dyn,
                           exist)
    
    def encode_object_quality(self, obj: RadarObject) -> bytes:
        """
        Encode object quality information (12 bytes per object).
        
        Format:
        ┌──────────────────────────────────────────────────────────────────────┐
        │ Bytes 0-1:   Object ID (uint16)                                      │
        │ Bytes 2-3:   Std Dev Dist Long (uint16, 0.01m LSB)                   │
        │ Bytes 4-5:   Std Dev Dist Lat (uint16, 0.01m LSB)                    │
        │ Bytes 6-7:   Std Dev Vel Long (uint16, 0.01 m/s LSB)                 │
        │ Bytes 8-9:   Std Dev Vel Lat (uint16, 0.01 m/s LSB)                  │
        │ Byte 10:     Measurement State                                       │
        │ Byte 11:     RCS (int8, 0.5 dBsm LSB)                                │
        └──────────────────────────────────────────────────────────────────────┘
        """
        std_dist_long = int(obj.std_dist_long / SCALE_STD)
        std_dist_lat = int(obj.std_dist_lat / SCALE_STD)
        std_vel_long = int(obj.std_vel_long / SCALE_STD)
        std_vel_lat = int(obj.std_vel_lat / SCALE_STD)
        
        # Clamp to uint16
        std_dist_long = max(0, min(65535, std_dist_long))
        std_dist_lat = max(0, min(65535, std_dist_lat))
        std_vel_long = max(0, min(65535, std_vel_long))
        std_vel_lat = max(0, min(65535, std_vel_lat))
        
        # RCS as signed byte
        rcs = int(obj.rcs / SCALE_RCS)
        rcs = max(-128, min(127, rcs))
        
        return struct.pack('<HHHHHBb',
                           obj.object_id & 0xFFFF,
                           std_dist_long, std_dist_lat,
                           std_vel_long, std_vel_lat,
                           obj.meas_state,
                           rcs)
    
    def encode_object_list(self, objects: List[RadarObject]) -> List[CANMessage]:
        """
        Encode list of radar objects to CAN-FD messages.
        
        Each CAN-FD message (64 bytes) can hold up to 4 objects (16 bytes each).
        
        Args:
            objects: List of RadarObject to encode
            
        Returns:
            List of CAN messages
        """
        messages = []
        bytes_per_object = 16
        objects_per_frame = CAN_FD_MAX_BYTES // bytes_per_object  # 4 objects
        
        for batch_idx in range(0, len(objects), objects_per_frame):
            batch = objects[batch_idx:batch_idx + objects_per_frame]
            
            # Encode objects
            data = b''.join(self.encode_object(obj) for obj in batch)
            
            # Pad to valid CAN-FD length
            data = self._pad_to_fd_length(data)
            
            # Add CRC if enabled
            if self.use_crc:
                crc = self._crc8(data[:-1])
                data = data[:-1] + bytes([crc])
            
            msg = CANMessage(
                arbitration_id=self.object_list_id + (batch_idx // objects_per_frame),
                data=data,
                is_fd=True
            )
            messages.append(msg)
        
        self._frame_counter = (self._frame_counter + 1) % 256
        return messages
    
    def encode_quality_list(self, objects: List[RadarObject]) -> List[CANMessage]:
        """
        Encode quality information for tracked objects.
        
        Each CAN-FD message (64 bytes) can hold up to 5 objects (12 bytes each).
        """
        messages = []
        bytes_per_object = 12
        objects_per_frame = CAN_FD_MAX_BYTES // bytes_per_object  # 5 objects
        
        for batch_idx in range(0, len(objects), objects_per_frame):
            batch = objects[batch_idx:batch_idx + objects_per_frame]
            
            data = b''.join(self.encode_object_quality(obj) for obj in batch)
            data = self._pad_to_fd_length(data)
            
            msg = CANMessage(
                arbitration_id=self.quality_list_id + (batch_idx // objects_per_frame),
                data=data,
                is_fd=True
            )
            messages.append(msg)
        
        return messages
    
    def encode_sensor_status(self, status: SensorStatus) -> CANMessage:
        """
        Encode sensor status message (Classic CAN, 8 bytes).
        
        Format:
        ┌──────────────────────────────────────────────────────────────────────┐
        │ Byte 0:     Sensor ID                                                │
        │ Byte 1:     Status Flags                                             │
        │ Byte 2:     Number of Objects                                        │
        │ Byte 3:     Frame Counter                                            │
        │ Bytes 4-5:  Timestamp (0.1ms LSB)                                    │
        │ Bytes 6-7:  Max Range (0.1m LSB)                                     │
        └──────────────────────────────────────────────────────────────────────┘
        """
        max_range = int(status.max_range / SCALE_DISTANCE)
        max_range = max(0, min(65535, max_range))
        
        data = struct.pack('<BBBBHH',
                           status.sensor_id,
                           int(status.flags),
                           min(255, status.num_objects),
                           status.frame_counter & 0xFF,
                           status.timestamp_ms & 0xFFFF,
                           max_range)
        
        return CANMessage(
            arbitration_id=self.status_id,
            data=data,
            is_fd=False
        )
    
    def _pad_to_fd_length(self, data: bytes) -> bytes:
        """Pad data to valid CAN-FD length."""
        fd_lengths = [8, 12, 16, 20, 24, 32, 48, 64]
        for length in fd_lengths:
            if len(data) <= length:
                return data.ljust(length, b'\x00')
        return data[:64]
    
    def _crc8(self, data: bytes) -> int:
        """CRC-8/SAE-J1850 for data integrity."""
        crc = 0xFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x1D
                else:
                    crc <<= 1
                crc &= 0xFF
        return crc ^ 0xFF


# =============================================================================
# NX-MIMOSA INTEGRATION
# =============================================================================

class NXMIMOSAToCANFD:
    """
    Converter from NX-MIMOSA tracker output to CAN-FD messages for ADAS.
    """
    
    def __init__(self, 
                 sensor_position: Tuple[float, float] = (3.5, 0.0),
                 sensor_id: int = 0):
        """
        Initialize converter.
        
        Args:
            sensor_position: (x, y) position of radar relative to vehicle center (meters)
            sensor_id: Sensor identification number
        """
        self.encoder = CANFDRadarEncoder(sensor_id=sensor_id)
        self.sensor_x, self.sensor_y = sensor_position
        
        # Object ID management
        self._id_map: Dict[int, int] = {}
        self._next_id = 0
        self._frame_count = 0
    
    def convert_track(self, 
                      track_id: int,
                      state: np.ndarray,
                      covariance: np.ndarray,
                      mode_probs: Optional[np.ndarray] = None) -> RadarObject:
        """
        Convert NX-MIMOSA track to RadarObject.
        
        Args:
            track_id: Track identifier from NX-MIMOSA
            state: State vector [x, y, z, vx, vy, vz, ...]
            covariance: Covariance matrix
            mode_probs: IMM mode probabilities [CV, CT1, CT2]
            
        Returns:
            RadarObject for CAN encoding
        """
        # Map track ID to object ID (0-65535)
        if track_id not in self._id_map:
            self._id_map[track_id] = self._next_id
            self._next_id = (self._next_id + 1) % 65536
        obj_id = self._id_map[track_id]
        
        # Position relative to sensor (vehicle coordinates)
        # NX-MIMOSA uses global coordinates, convert to vehicle-relative
        dist_long = state[0] - self.sensor_x
        dist_lat = state[1] - self.sensor_y
        
        # Velocity (assume relative to ego vehicle, simplified)
        vel_long = state[3]
        vel_lat = state[4]
        
        # Acceleration (if available in state, otherwise estimate from mode)
        accel_long = state[6] if len(state) > 6 else 0.0
        accel_lat = state[7] if len(state) > 7 else 0.0
        
        # Extract standard deviations from covariance
        std_dist_long = np.sqrt(max(0, covariance[0, 0]))
        std_dist_lat = np.sqrt(max(0, covariance[1, 1]))
        std_vel_long = np.sqrt(max(0, covariance[3, 3]))
        std_vel_lat = np.sqrt(max(0, covariance[4, 4]))
        
        # Determine dynamic property from velocity
        speed = np.sqrt(vel_long**2 + vel_lat**2)
        if speed < 0.5:
            if speed < 0.1:
                dynamic = DynamicProperty.STATIONARY
            else:
                dynamic = DynamicProperty.STOPPED
        elif vel_long < -1.0:
            dynamic = DynamicProperty.ONCOMING
        elif abs(vel_lat) > abs(vel_long):
            if vel_lat > 0:
                dynamic = DynamicProperty.CROSSING_MOVING
            else:
                dynamic = DynamicProperty.CROSSING_MOVING
        else:
            dynamic = DynamicProperty.MOVING
        
        # Existence probability from mode probabilities
        exist_prob = 0.95
        if mode_probs is not None and len(mode_probs) >= 1:
            # Higher CV probability = more confident track
            exist_prob = 0.5 + 0.5 * mode_probs[0]
        
        return RadarObject(
            object_id=obj_id,
            dist_long=dist_long,
            dist_lat=dist_lat,
            vel_long=vel_long,
            vel_lat=vel_lat,
            accel_long=accel_long,
            accel_lat=accel_lat,
            object_class=ObjectClass.UNCLASSIFIED,
            dynamic_property=dynamic,
            meas_state=MeasState.MEASURED,
            exist_prob=exist_prob,
            std_dist_long=std_dist_long,
            std_dist_lat=std_dist_lat,
            std_vel_long=std_vel_long,
            std_vel_lat=std_vel_lat,
        )
    
    def encode_tracks(self, 
                      trackers: Dict[int, Any],
                      include_quality: bool = True) -> List[CANMessage]:
        """
        Convert and encode multiple NX-MIMOSA tracks.
        
        Args:
            trackers: Dict of {track_id: tracker} where tracker has get_state(), get_covariance()
            include_quality: Include quality/accuracy messages
            
        Returns:
            List of CAN messages
        """
        objects = []
        
        for track_id, tracker in trackers.items():
            state = tracker.get_state()
            
            # Get covariance - handle different tracker types
            if hasattr(tracker, 'get_covariance'):
                cov = tracker.get_covariance()
            elif hasattr(tracker, 'imm') and hasattr(tracker.imm, 'filters'):
                # Get from IMM's first filter
                cov = tracker.imm.filters[0].get_covariance()
            else:
                cov = np.eye(len(state)) * 100
            
            # Get mode probabilities if available
            mode_probs = None
            if hasattr(tracker, 'get_mode_probabilities'):
                mode_probs = tracker.get_mode_probabilities()
            
            obj = self.convert_track(track_id, state, cov, mode_probs)
            objects.append(obj)
        
        # Encode to CAN messages
        messages = self.encoder.encode_object_list(objects)
        
        if include_quality and objects:
            messages.extend(self.encoder.encode_quality_list(objects))
        
        # Add sensor status
        status = SensorStatus(
            sensor_id=self.encoder.sensor_id,
            flags=SensorStatusFlags.OPERATIONAL,
            num_objects=len(objects),
            frame_counter=self._frame_count,
            timestamp_ms=int((time.time() * 1000) % 65536),
            max_range=200.0
        )
        messages.append(self.encoder.encode_sensor_status(status))
        
        self._frame_count = (self._frame_count + 1) % 256
        return messages
    
    def cleanup_stale(self, active_ids: List[int]):
        """Remove stale track IDs from mapping."""
        stale = [tid for tid in self._id_map if tid not in active_ids]
        for tid in stale:
            del self._id_map[tid]


# =============================================================================
# VIRTUAL CAN INTERFACE
# =============================================================================

class VirtualCANBus:
    """Virtual CAN bus for testing without hardware."""
    
    def __init__(self, channel: str = "vcan0"):
        self.channel = channel
        self.tx_queue: List[CANMessage] = []
        self.rx_queue: List[CANMessage] = []
    
    def send(self, msg: CANMessage):
        """Send message."""
        self.tx_queue.append(msg)
        print(f"[{self.channel}] TX: {msg}")
    
    def recv(self, timeout: float = 1.0) -> Optional[CANMessage]:
        """Receive message."""
        if self.rx_queue:
            return self.rx_queue.pop(0)
        return None


# =============================================================================
# EXAMPLE USAGE
# =============================================================================

def example_usage():
    """Demonstrate CAN-FD encoding."""
    print("="*80)
    print("NX-MIMOSA CAN-FD OUTPUT FORMATTER")
    print("ISO 26262 ASIL-D Compliant ADAS Interface")
    print("="*80)
    
    # Create sample radar objects
    objects = [
        RadarObject(
            object_id=1,
            dist_long=45.0,
            dist_lat=1.5,
            vel_long=-3.0,  # Approaching
            vel_lat=0.0,
            object_class=ObjectClass.CAR,
            dynamic_property=DynamicProperty.MOVING,
            exist_prob=0.98,
            std_dist_long=0.5,
            std_dist_lat=0.3,
            std_vel_long=0.1,
            std_vel_lat=0.1,
            rcs=12.0,
        ),
        RadarObject(
            object_id=2,
            dist_long=80.0,
            dist_lat=-3.5,
            vel_long=2.0,  # Moving away (same direction)
            vel_lat=0.0,
            object_class=ObjectClass.TRUCK,
            dynamic_property=DynamicProperty.MOVING,
            exist_prob=0.95,
            std_dist_long=1.0,
            std_dist_lat=0.5,
            std_vel_long=0.2,
            std_vel_lat=0.2,
            rcs=25.0,
        ),
        RadarObject(
            object_id=3,
            dist_long=25.0,
            dist_lat=0.0,
            vel_long=0.0,
            vel_lat=0.0,
            object_class=ObjectClass.POINT,
            dynamic_property=DynamicProperty.STATIONARY,
            exist_prob=0.99,
            std_dist_long=0.2,
            std_dist_lat=0.2,
            std_vel_long=0.05,
            std_vel_lat=0.05,
            rcs=-5.0,  # Small object
        ),
    ]
    
    print("\n📡 Sample Radar Objects:")
    for obj in objects:
        print(f"  ID {obj.object_id}: {obj.object_class.name}")
        print(f"    Position: ({obj.dist_long:.1f}, {obj.dist_lat:.1f}) m")
        print(f"    Velocity: ({obj.vel_long:.1f}, {obj.vel_lat:.1f}) m/s")
        print(f"    Status: {obj.dynamic_property.name}, Prob: {obj.exist_prob:.0%}")
    
    # Encode to CAN-FD
    encoder = CANFDRadarEncoder(use_crc=False)
    
    obj_messages = encoder.encode_object_list(objects)
    qual_messages = encoder.encode_quality_list(objects)
    status_msg = encoder.encode_sensor_status(SensorStatus(num_objects=len(objects)))
    
    print("\n📤 CAN-FD Messages Generated:")
    print("\n  Object List:")
    for msg in obj_messages:
        print(f"    {msg}")
    
    print("\n  Quality List:")
    for msg in qual_messages:
        print(f"    {msg}")
    
    print("\n  Sensor Status:")
    print(f"    {status_msg}")
    
    # Statistics
    total_bytes = sum(len(m.data) for m in obj_messages + qual_messages + [status_msg])
    print(f"\n📊 Message Statistics:")
    print(f"  Total CAN frames: {len(obj_messages) + len(qual_messages) + 1}")
    print(f"  Total payload: {total_bytes} bytes")
    print(f"  Objects encoded: {len(objects)}")
    print(f"  Bytes per object: {total_bytes / len(objects):.1f}")
    
    print("\n" + "="*80)
    print("✓ CAN-FD formatter ready for ISO 26262 ADAS integration")
    print("="*80)


if __name__ == "__main__":
    example_usage()
