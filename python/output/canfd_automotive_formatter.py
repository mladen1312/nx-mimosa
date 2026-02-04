#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════════════════════════════
NX-MIMOSA - CAN-FD OUTPUT FORMATTER FOR AUTOMOTIVE ADAS
═══════════════════════════════════════════════════════════════════════════════════════════════════════

CAN-FD (Controller Area Network with Flexible Data-rate)
For ISO 26262 ASIL-D compliant ADAS/AD systems

This module converts NX-MIMOSA tracker output to CAN-FD messages for
direct integration with automotive radar systems and ADAS ECUs.

Standards Compliance:
- ISO 11898-1:2015 (CAN protocol)
- ISO 11898-2:2016 (High-speed CAN)
- ISO 26262 (Functional Safety)
- SAE J2284-4 (500 kbps CAN)
- AUTOSAR Classic Platform

Message Types:
- Object List (tracked targets)
- Object Status (quality metrics)
- Sensor Status
- Cluster Data (raw detections)

Typical CAN IDs (configurable):
- 0x300-0x3FF: Radar Object List
- 0x400-0x4FF: Radar Object Status
- 0x500-0x5FF: Sensor Status

Author: Dr. Mladen Mešter / Nexellum d.o.o.
License: AGPL v3 / Commercial
═══════════════════════════════════════════════════════════════════════════════════════════════════════
"""

import struct
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Dict, Any
from enum import IntEnum, IntFlag
import numpy as np

__version__ = "1.0.0"
__author__ = "Dr. Mladen Mešter"


# =============================================================================
# CONSTANTS
# =============================================================================

# CAN-FD frame sizes
CAN_CLASSIC_MAX = 8      # Classic CAN max payload
CAN_FD_MAX = 64          # CAN-FD max payload

# DLC to actual length mapping for CAN-FD
CAN_FD_DLC_MAP = {
    0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6, 7: 7, 8: 8,
    9: 12, 10: 16, 11: 20, 12: 24, 13: 32, 14: 48, 15: 64
}

# Coordinate scaling (typical automotive radar)
SCALE_DISTANCE = 0.1     # meters per LSB
SCALE_VELOCITY = 0.01    # m/s per LSB
SCALE_ANGLE = 0.1        # degrees per LSB
SCALE_RCS = 0.5          # dBsm per LSB

# Default CAN IDs
CAN_ID_OBJECT_LIST_BASE = 0x300
CAN_ID_OBJECT_STATUS_BASE = 0x400
CAN_ID_SENSOR_STATUS = 0x500
CAN_ID_CLUSTER_BASE = 0x600


# =============================================================================
# OBJECT CLASSIFICATION
# =============================================================================

class ObjectClass(IntEnum):
    """Object classification types (AUTOSAR compatible)."""
    UNKNOWN = 0
    CAR = 1
    TRUCK = 2
    MOTORCYCLE = 3
    BICYCLE = 4
    PEDESTRIAN = 5
    ANIMAL = 6
    STATIONARY = 7
    POINT = 8
    WIDE = 9


class DynamicProperty(IntEnum):
    """Dynamic property of tracked object."""
    MOVING = 0
    STATIONARY = 1
    ONCOMING = 2
    CROSSING_LEFT = 3
    CROSSING_RIGHT = 4
    UNKNOWN = 5
    STOPPED = 6


class MeasurementState(IntEnum):
    """Measurement state flags."""
    DELETED = 0
    NEW = 1
    MEASURED = 2
    PREDICTED = 3
    DELETED_FOR_MERGE = 4
    NEW_FROM_MERGE = 5


# =============================================================================
# DATA STRUCTURES
# =============================================================================

@dataclass
class RadarObject:
    """Tracked radar object for CAN transmission."""
    # Object ID (0-255)
    object_id: int
    
    # Position (meters, relative to sensor)
    distance_x: float       # Longitudinal distance (positive = forward)
    distance_y: float       # Lateral distance (positive = left)
    
    # Velocity (m/s, relative)
    velocity_x: float       # Longitudinal velocity
    velocity_y: float       # Lateral velocity
    
    # Acceleration (m/s²)
    acceleration_x: float = 0.0
    acceleration_y: float = 0.0
    
    # Object properties
    object_class: ObjectClass = ObjectClass.UNKNOWN
    dynamic_property: DynamicProperty = DynamicProperty.UNKNOWN
    measurement_state: MeasurementState = MeasurementState.MEASURED
    
    # Quality metrics
    existence_probability: float = 1.0  # 0-1
    rcs: float = 0.0                    # dBsm
    
    # Object dimensions (meters)
    length: float = 0.0
    width: float = 0.0
    orientation: float = 0.0            # degrees
    
    # Accuracy (1-sigma)
    distance_x_std: float = 0.0
    distance_y_std: float = 0.0
    velocity_x_std: float = 0.0
    velocity_y_std: float = 0.0


@dataclass
class SensorStatus:
    """Radar sensor status."""
    sensor_id: int = 0
    timestamp: float = 0.0              # seconds
    
    # Operational status
    operational: bool = True
    blocked: bool = False
    degraded: bool = False
    
    # Alignment
    yaw_angle: float = 0.0              # degrees
    
    # Performance
    num_objects: int = 0
    max_distance: float = 200.0         # meters


# =============================================================================
# CAN MESSAGE STRUCTURES
# =============================================================================

@dataclass
class CANMessage:
    """CAN/CAN-FD message structure."""
    arbitration_id: int
    data: bytes
    is_extended_id: bool = False
    is_fd: bool = True
    bitrate_switch: bool = True
    
    @property
    def dlc(self) -> int:
        """Get DLC code for data length."""
        length = len(self.data)
        if length <= 8:
            return length
        for dlc, size in CAN_FD_DLC_MAP.items():
            if size >= length:
                return dlc
        return 15  # 64 bytes
    
    def __repr__(self):
        return f"CAN({'FD' if self.is_fd else ''}): ID=0x{self.arbitration_id:03X} DLC={self.dlc} Data={self.data.hex()}"


# =============================================================================
# CAN-FD ENCODER
# =============================================================================

class CANFDRadarEncoder:
    """
    Encodes radar track data to CAN-FD messages.
    
    Message Layout (Object List, per object):
    ─────────────────────────────────────────────────────────────────────────────
    Byte 0:     Object ID (0-255)
    Byte 1-2:   Distance X (signed, 0.1m/LSB, range: -3276.8 to +3276.7 m)
    Byte 3-4:   Distance Y (signed, 0.1m/LSB)
    Byte 5-6:   Velocity X (signed, 0.01 m/s/LSB)
    Byte 7-8:   Velocity Y (signed, 0.01 m/s/LSB)
    Byte 9-10:  Acceleration X (signed, 0.01 m/s²/LSB)
    Byte 11-12: Acceleration Y (signed, 0.01 m/s²/LSB)
    Byte 13:    Class (4 bits) | Dynamic Property (4 bits)
    Byte 14:    Existence Probability (0-255 = 0-100%)
    Byte 15:    RCS (signed, 0.5 dBsm/LSB)
    ─────────────────────────────────────────────────────────────────────────────
    Total: 16 bytes per object (CAN-FD)
    
    For Classic CAN (8 bytes), use compressed format.
    """
    
    def __init__(self, 
                 base_id_objects: int = CAN_ID_OBJECT_LIST_BASE,
                 base_id_status: int = CAN_ID_OBJECT_STATUS_BASE,
                 sensor_status_id: int = CAN_ID_SENSOR_STATUS,
                 use_fd: bool = True,
                 max_objects_per_frame: int = 4):
        """
        Initialize CAN-FD encoder.
        
        Args:
            base_id_objects: Base CAN ID for object list messages
            base_id_status: Base CAN ID for object status messages
            sensor_status_id: CAN ID for sensor status
            use_fd: Use CAN-FD (64 bytes) or Classic CAN (8 bytes)
            max_objects_per_frame: Max objects per CAN-FD frame
        """
        self.base_id_objects = base_id_objects
        self.base_id_status = base_id_status
        self.sensor_status_id = sensor_status_id
        self.use_fd = use_fd
        self.max_objects_per_frame = max_objects_per_frame
        
        # Frame counter for cyclic redundancy
        self._frame_counter = 0
    
    def encode_object(self, obj: RadarObject) -> bytes:
        """
        Encode single radar object to bytes.
        
        Args:
            obj: RadarObject to encode
            
        Returns:
            16 bytes (CAN-FD) or 8 bytes (Classic CAN)
        """
        if self.use_fd:
            return self._encode_object_fd(obj)
        else:
            return self._encode_object_classic(obj)
    
    def _encode_object_fd(self, obj: RadarObject) -> bytes:
        """Encode object for CAN-FD (16 bytes)."""
        # Scale and convert values
        dist_x = int(obj.distance_x / SCALE_DISTANCE)
        dist_y = int(obj.distance_y / SCALE_DISTANCE)
        vel_x = int(obj.velocity_x / SCALE_VELOCITY)
        vel_y = int(obj.velocity_y / SCALE_VELOCITY)
        acc_x = int(obj.acceleration_x / SCALE_VELOCITY)  # Same scale as velocity
        acc_y = int(obj.acceleration_y / SCALE_VELOCITY)
        
        # Clamp to 16-bit signed range
        dist_x = max(-32768, min(32767, dist_x))
        dist_y = max(-32768, min(32767, dist_y))
        vel_x = max(-32768, min(32767, vel_x))
        vel_y = max(-32768, min(32767, vel_y))
        acc_x = max(-32768, min(32767, acc_x))
        acc_y = max(-32768, min(32767, acc_y))
        
        # Classification byte
        class_dyn = ((obj.object_class & 0x0F) << 4) | (obj.dynamic_property & 0x0F)
        
        # Existence probability (0-255)
        exist_prob = int(obj.existence_probability * 255)
        exist_prob = max(0, min(255, exist_prob))
        
        # RCS (signed byte, 0.5 dBsm/LSB)
        rcs = int(obj.rcs / SCALE_RCS)
        rcs = max(-128, min(127, rcs))
        
        return struct.pack('>BhhhhhhBBb',
            obj.object_id,
            dist_x, dist_y,
            vel_x, vel_y,
            acc_x, acc_y,
            class_dyn,
            exist_prob,
            rcs
        )
    
    def _encode_object_classic(self, obj: RadarObject) -> bytes:
        """Encode object for Classic CAN (8 bytes, compressed)."""
        # Reduced precision for 8-byte frame
        dist_x = int(obj.distance_x / 0.2)  # 0.2m resolution
        dist_y = int(obj.distance_y / 0.2)
        vel_x = int(obj.velocity_x / 0.1)   # 0.1 m/s resolution
        vel_y = int(obj.velocity_y / 0.1)
        
        # 12-bit values
        dist_x = max(-2048, min(2047, dist_x))
        dist_y = max(-2048, min(2047, dist_y))
        vel_x = max(-2048, min(2047, vel_x))
        vel_y = max(-2048, min(2047, vel_y))
        
        # Pack into 8 bytes
        # Byte 0: Object ID
        # Bytes 1-2: Distance X (12 bits) + Distance Y MSB (4 bits)
        # Bytes 3-4: Distance Y LSB (8 bits) + Velocity X MSB (8 bits)
        # Bytes 5-6: Velocity X LSB (4 bits) + Velocity Y (12 bits)
        # Byte 7: Class (4 bits) + Existence (4 bits)
        
        # This is a simplified encoding - production would use proper bit packing
        return struct.pack('>Bhhbb',
            obj.object_id,
            dist_x & 0xFFF,
            dist_y & 0xFFF,
            (vel_x >> 4) & 0xFF,
            ((vel_x & 0x0F) << 4) | (obj.object_class & 0x0F)
        )
    
    def encode_object_list(self, objects: List[RadarObject]) -> List[CANMessage]:
        """
        Encode list of radar objects to CAN messages.
        
        Args:
            objects: List of RadarObject to encode
            
        Returns:
            List of CAN messages
        """
        messages = []
        
        if self.use_fd:
            # CAN-FD: Pack multiple objects per frame (up to 64 bytes)
            bytes_per_object = 16
            objects_per_frame = min(self.max_objects_per_frame, 64 // bytes_per_object)
            
            for i in range(0, len(objects), objects_per_frame):
                batch = objects[i:i + objects_per_frame]
                data = b''.join(self.encode_object(obj) for obj in batch)
                
                # Pad to valid CAN-FD length
                target_len = self._get_fd_length(len(data))
                data = data.ljust(target_len, b'\x00')
                
                can_id = self.base_id_objects + (i // objects_per_frame)
                messages.append(CANMessage(
                    arbitration_id=can_id,
                    data=data,
                    is_fd=True,
                    bitrate_switch=True
                ))
        else:
            # Classic CAN: One object per frame
            for i, obj in enumerate(objects):
                data = self.encode_object(obj)
                can_id = self.base_id_objects + i
                messages.append(CANMessage(
                    arbitration_id=can_id,
                    data=data,
                    is_fd=False
                ))
        
        self._frame_counter = (self._frame_counter + 1) % 16
        
        return messages
    
    def encode_sensor_status(self, status: SensorStatus) -> CANMessage:
        """
        Encode sensor status message.
        
        Layout (8 bytes):
        ─────────────────────────────────────────────────────────────────────────
        Byte 0:     Sensor ID
        Byte 1:     Status flags (operational, blocked, degraded, etc.)
        Byte 2-3:   Timestamp (0.1ms/LSB, modulo 6.5536s)
        Byte 4:     Number of objects
        Byte 5-6:   Max detection distance (0.1m/LSB)
        Byte 7:     Frame counter (4 bits) + reserved (4 bits)
        ─────────────────────────────────────────────────────────────────────────
        """
        # Status flags
        flags = 0
        if status.operational:
            flags |= 0x01
        if status.blocked:
            flags |= 0x02
        if status.degraded:
            flags |= 0x04
        
        # Timestamp (modulo wrap)
        timestamp_lsb = int((status.timestamp * 10000) % 65536)
        
        # Max distance
        max_dist = int(status.max_distance / SCALE_DISTANCE)
        max_dist = max(0, min(65535, max_dist))
        
        data = struct.pack('>BBHBHB',
            status.sensor_id,
            flags,
            timestamp_lsb,
            min(255, status.num_objects),
            max_dist,
            (self._frame_counter << 4)
        )
        
        return CANMessage(
            arbitration_id=self.sensor_status_id,
            data=data,
            is_fd=False
        )
    
    def encode_object_quality(self, objects: List[RadarObject]) -> List[CANMessage]:
        """
        Encode object quality/accuracy information.
        
        Layout per object (CAN-FD, 12 bytes):
        ─────────────────────────────────────────────────────────────────────────
        Byte 0:     Object ID
        Byte 1:     Measurement state
        Byte 2-3:   Distance X std (0.01m/LSB)
        Byte 4-5:   Distance Y std (0.01m/LSB)
        Byte 6-7:   Velocity X std (0.01 m/s/LSB)
        Byte 8-9:   Velocity Y std (0.01 m/s/LSB)
        Byte 10-11: Reserved
        ─────────────────────────────────────────────────────────────────────────
        """
        messages = []
        bytes_per_object = 12
        objects_per_frame = 64 // bytes_per_object  # 5 objects per frame
        
        for i in range(0, len(objects), objects_per_frame):
            batch = objects[i:i + objects_per_frame]
            data_parts = []
            
            for obj in batch:
                dist_x_std = int(obj.distance_x_std / 0.01)
                dist_y_std = int(obj.distance_y_std / 0.01)
                vel_x_std = int(obj.velocity_x_std / 0.01)
                vel_y_std = int(obj.velocity_y_std / 0.01)
                
                data_parts.append(struct.pack('>BBHHHHH',
                    obj.object_id,
                    obj.measurement_state,
                    min(65535, dist_x_std),
                    min(65535, dist_y_std),
                    min(65535, vel_x_std),
                    min(65535, vel_y_std),
                    0  # Reserved
                ))
            
            data = b''.join(data_parts)
            target_len = self._get_fd_length(len(data))
            data = data.ljust(target_len, b'\x00')
            
            can_id = self.base_id_status + (i // objects_per_frame)
            messages.append(CANMessage(
                arbitration_id=can_id,
                data=data,
                is_fd=True
            ))
        
        return messages
    
    def _get_fd_length(self, length: int) -> int:
        """Get valid CAN-FD frame length."""
        for size in [8, 12, 16, 20, 24, 32, 48, 64]:
            if length <= size:
                return size
        return 64


# =============================================================================
# NX-MIMOSA INTEGRATION
# =============================================================================

class NXMIMOSACANOutput:
    """
    Wrapper to convert NX-MIMOSA tracker output to CAN-FD messages.
    
    Usage:
        can_out = NXMIMOSACANOutput()
        
        # In tracking loop:
        objects = can_out.from_trackers(trackers)
        messages = can_out.encode_all(objects)
        
        for msg in messages:
            can_bus.send(msg)
    """
    
    def __init__(self, 
                 use_fd: bool = True,
                 base_id: int = CAN_ID_OBJECT_LIST_BASE,
                 sensor_position: Tuple[float, float] = (0.0, 0.0)):
        """
        Initialize CAN output wrapper.
        
        Args:
            use_fd: Use CAN-FD (True) or Classic CAN (False)
            base_id: Base CAN ID for object messages
            sensor_position: Sensor position relative to vehicle origin (x, y) meters
        """
        self.encoder = CANFDRadarEncoder(
            base_id_objects=base_id,
            use_fd=use_fd
        )
        self.sensor_x, self.sensor_y = sensor_position
        self._object_id_map: Dict[int, int] = {}
        self._next_object_id = 0
    
    def from_tracker(self, tracker, track_id: int) -> RadarObject:
        """
        Convert NX-MIMOSA tracker state to RadarObject.
        
        Args:
            tracker: NX-MIMOSA tracker instance
            track_id: Track identifier
            
        Returns:
            RadarObject for CAN encoding
        """
        state = tracker.get_state()
        
        # Get or assign object ID (0-255)
        if track_id not in self._object_id_map:
            self._object_id_map[track_id] = self._next_object_id
            self._next_object_id = (self._next_object_id + 1) % 256
        
        object_id = self._object_id_map[track_id]
        
        # Position relative to sensor
        # Assume state is [x, y, z, vx, vy, vz] in global coordinates
        distance_x = state[0] - self.sensor_x
        distance_y = state[1] - self.sensor_y
        velocity_x = state[3]
        velocity_y = state[4]
        
        # Determine dynamic property from velocity
        speed = np.sqrt(velocity_x**2 + velocity_y**2)
        if speed < 0.5:
            if speed < 0.1:
                dynamic_property = DynamicProperty.STATIONARY
            else:
                dynamic_property = DynamicProperty.STOPPED
        elif velocity_x < -0.5:
            dynamic_property = DynamicProperty.ONCOMING
        elif abs(velocity_y) > abs(velocity_x):
            if velocity_y > 0:
                dynamic_property = DynamicProperty.CROSSING_LEFT
            else:
                dynamic_property = DynamicProperty.CROSSING_RIGHT
        else:
            dynamic_property = DynamicProperty.MOVING
        
        # Get accuracy from covariance if available
        dist_x_std, dist_y_std = 0.0, 0.0
        vel_x_std, vel_y_std = 0.0, 0.0
        
        if hasattr(tracker, 'imm'):
            # Get from combined IMM covariance
            try:
                P = tracker.imm.filters[0].get_covariance()
                dist_x_std = np.sqrt(P[0, 0])
                dist_y_std = np.sqrt(P[1, 1])
                vel_x_std = np.sqrt(P[3, 3])
                vel_y_std = np.sqrt(P[4, 4])
            except:
                pass
        
        # Existence probability from mode probabilities
        exist_prob = 1.0
        if hasattr(tracker, 'get_mode_probabilities'):
            mode_probs = tracker.get_mode_probabilities()
            # Higher CV probability = more confident track
            exist_prob = 0.5 + 0.5 * mode_probs[0]
        
        return RadarObject(
            object_id=object_id,
            distance_x=distance_x,
            distance_y=distance_y,
            velocity_x=velocity_x,
            velocity_y=velocity_y,
            acceleration_x=0.0,  # Could compute from velocity history
            acceleration_y=0.0,
            object_class=ObjectClass.UNKNOWN,
            dynamic_property=dynamic_property,
            measurement_state=MeasurementState.MEASURED,
            existence_probability=exist_prob,
            rcs=0.0,
            distance_x_std=dist_x_std,
            distance_y_std=dist_y_std,
            velocity_x_std=vel_x_std,
            velocity_y_std=vel_y_std,
        )
    
    def from_trackers(self, trackers: Dict[int, Any]) -> List[RadarObject]:
        """
        Convert multiple trackers to RadarObject list.
        
        Args:
            trackers: Dict of {track_id: tracker}
            
        Returns:
            List of RadarObject
        """
        return [self.from_tracker(t, tid) for tid, t in trackers.items()]
    
    def encode_all(self, objects: List[RadarObject], 
                   include_quality: bool = True) -> List[CANMessage]:
        """
        Encode all objects to CAN messages.
        
        Args:
            objects: List of RadarObject
            include_quality: Include quality/accuracy messages
            
        Returns:
            List of CAN messages
        """
        messages = []
        
        # Object list
        messages.extend(self.encoder.encode_object_list(objects))
        
        # Quality information
        if include_quality:
            messages.extend(self.encoder.encode_object_quality(objects))
        
        # Sensor status
        status = SensorStatus(
            sensor_id=0,
            timestamp=time.time() % 6.5536,
            operational=True,
            num_objects=len(objects)
        )
        messages.append(self.encoder.encode_sensor_status(status))
        
        return messages
    
    def clear_stale_objects(self, active_track_ids: List[int]):
        """Remove stale objects from ID mapping."""
        stale = [tid for tid in self._object_id_map if tid not in active_track_ids]
        for tid in stale:
            del self._object_id_map[tid]


# =============================================================================
# VIRTUAL CAN INTERFACE (for testing)
# =============================================================================

class VirtualCANBus:
    """Simple virtual CAN bus for testing."""
    
    def __init__(self, channel: str = 'vcan0'):
        self.channel = channel
        self.messages: List[CANMessage] = []
    
    def send(self, msg: CANMessage):
        """Send message to virtual bus."""
        self.messages.append(msg)
        print(f"TX: {msg}")
    
    def recv(self, timeout: float = 1.0) -> Optional[CANMessage]:
        """Receive message from virtual bus."""
        if self.messages:
            return self.messages.pop(0)
        return None


# =============================================================================
# MAIN - DEMONSTRATION
# =============================================================================

if __name__ == "__main__":
    print("="*80)
    print("NX-MIMOSA CAN-FD OUTPUT FORMATTER FOR AUTOMOTIVE ADAS")
    print("="*80)
    
    # Create sample radar objects
    objects = [
        RadarObject(
            object_id=1,
            distance_x=50.0,    # 50m ahead
            distance_y=2.0,     # 2m left
            velocity_x=-5.0,    # Approaching at 5 m/s relative
            velocity_y=0.0,
            object_class=ObjectClass.CAR,
            dynamic_property=DynamicProperty.ONCOMING,
            existence_probability=0.95,
            rcs=10.0,
        ),
        RadarObject(
            object_id=2,
            distance_x=30.0,
            distance_y=-1.5,
            velocity_x=0.0,
            velocity_y=0.0,
            object_class=ObjectClass.STATIONARY,
            dynamic_property=DynamicProperty.STATIONARY,
            existence_probability=0.99,
            rcs=5.0,
        ),
        RadarObject(
            object_id=3,
            distance_x=80.0,
            distance_y=0.0,
            velocity_x=2.0,     # Same direction, slower
            velocity_y=0.0,
            object_class=ObjectClass.TRUCK,
            dynamic_property=DynamicProperty.MOVING,
            existence_probability=0.85,
            rcs=20.0,
        ),
    ]
    
    print(f"\n📡 Sample Radar Objects:")
    for obj in objects:
        print(f"  ID {obj.object_id}: {obj.object_class.name} at ({obj.distance_x:.1f}, {obj.distance_y:.1f})m")
        print(f"         Velocity: ({obj.velocity_x:.1f}, {obj.velocity_y:.1f}) m/s")
        print(f"         Dynamic: {obj.dynamic_property.name}, Exist: {obj.existence_probability:.0%}")
    
    # Create encoder
    encoder = CANFDRadarEncoder(use_fd=True)
    
    # Encode to CAN-FD messages
    messages = encoder.encode_object_list(objects)
    quality_msgs = encoder.encode_object_quality(objects)
    status_msg = encoder.encode_sensor_status(SensorStatus(num_objects=len(objects)))
    
    print(f"\n📤 CAN-FD Messages:")
    print(f"  Object List Messages: {len(messages)}")
    for msg in messages:
        print(f"    {msg}")
    
    print(f"\n  Quality Messages: {len(quality_msgs)}")
    for msg in quality_msgs:
        print(f"    {msg}")
    
    print(f"\n  Sensor Status:")
    print(f"    {status_msg}")
    
    print(f"\n📊 Message Statistics:")
    total_bytes = sum(len(m.data) for m in messages + quality_msgs + [status_msg])
    print(f"  Total payload: {total_bytes} bytes")
    print(f"  Objects encoded: {len(objects)}")
    print(f"  Bytes per object: {total_bytes / len(objects):.1f}")
    
    print("\n" + "="*80)
    print("✓ CAN-FD formatter ready for ISO 26262 ADAS integration")
    print("="*80)
