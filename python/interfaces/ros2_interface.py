#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════════════════════════════
NX-MIMOSA ROS2 Interface
═══════════════════════════════════════════════════════════════════════════════════════════════════════

ROS2 wrapper for NX-MIMOSA tracking system:
- Subscribes to sensor_msgs/PointCloud2 or custom radar messages
- Publishes visualization_msgs/MarkerArray for RViz
- Supports tf2 coordinate transforms
- Service interface for track queries

Compatible with:
- ROS2 Humble/Iron/Jazzy
- Autoware.Universe
- PX4/MAVROS

Author: Dr. Mladen Mešter / Nexellum d.o.o.
Version: 1.1.0
License: AGPL v3 / Commercial

NOTE: This module requires ROS2 installation. Import will fail gracefully if ROS2 is not available.
═══════════════════════════════════════════════════════════════════════════════════════════════════════
"""

import sys
import numpy as np
from typing import List, Dict, Optional, Callable, Any
from dataclasses import dataclass
from enum import IntEnum
import time
import threading

# Try to import ROS2 - graceful fallback if not available
ROS2_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from std_msgs.msg import Header
    from geometry_msgs.msg import Point, Pose, PoseWithCovariance, Vector3
    from visualization_msgs.msg import Marker, MarkerArray
    from sensor_msgs.msg import PointCloud2, PointField
    import struct
    ROS2_AVAILABLE = True
except ImportError:
    pass

# Import NX-MIMOSA tracker
NXMIMOSAAtc = None
try:
    from nx_mimosa_v41_atc import NXMIMOSAAtc
except ImportError:
    try:
        import sys
        import os
        # Try parent directory
        sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
        from nx_mimosa_v41_atc import NXMIMOSAAtc
    except ImportError:
        pass


# =============================================================================
# DATA STRUCTURES
# =============================================================================

@dataclass
class RadarDetection:
    """Single radar detection."""
    x: float          # meters
    y: float          # meters
    z: float          # meters
    vx: float = 0.0   # m/s (Doppler)
    vy: float = 0.0   # m/s
    vz: float = 0.0   # m/s
    rcs: float = 0.0  # dBsm
    snr: float = 0.0  # dB
    timestamp: float = 0.0


@dataclass 
class TrackedObject:
    """Tracked object for publishing."""
    track_id: int
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    covariance: np.ndarray
    classification: int = 0
    mode_probabilities: Optional[np.ndarray] = None


class ObjectClass(IntEnum):
    """Object classification."""
    UNKNOWN = 0
    VEHICLE = 1
    PEDESTRIAN = 2
    CYCLIST = 3
    AIRCRAFT = 4
    DRONE = 5


# =============================================================================
# ROS2 MESSAGE BUILDERS (Standalone functions for non-ROS2 testing)
# =============================================================================

def build_marker(track: TrackedObject, frame_id: str = "base_link", 
                 namespace: str = "nx_mimosa") -> Dict[str, Any]:
    """
    Build marker dictionary for visualization.
    
    Returns dict that can be converted to ROS2 Marker message.
    """
    # Color based on classification
    colors = {
        ObjectClass.UNKNOWN: (0.5, 0.5, 0.5, 0.8),
        ObjectClass.VEHICLE: (0.0, 1.0, 0.0, 0.8),
        ObjectClass.PEDESTRIAN: (1.0, 1.0, 0.0, 0.8),
        ObjectClass.CYCLIST: (0.0, 1.0, 1.0, 0.8),
        ObjectClass.AIRCRAFT: (0.0, 0.0, 1.0, 0.8),
        ObjectClass.DRONE: (1.0, 0.0, 1.0, 0.8),
    }
    
    color = colors.get(track.classification, colors[ObjectClass.UNKNOWN])
    
    # Size based on classification
    sizes = {
        ObjectClass.UNKNOWN: (2.0, 2.0, 2.0),
        ObjectClass.VEHICLE: (4.5, 2.0, 1.5),
        ObjectClass.PEDESTRIAN: (0.5, 0.5, 1.8),
        ObjectClass.CYCLIST: (2.0, 0.8, 1.5),
        ObjectClass.AIRCRAFT: (30.0, 25.0, 5.0),
        ObjectClass.DRONE: (1.0, 1.0, 0.3),
    }
    
    size = sizes.get(track.classification, sizes[ObjectClass.UNKNOWN])
    
    return {
        'header': {'frame_id': frame_id},
        'ns': namespace,
        'id': track.track_id,
        'type': 1,  # CUBE
        'action': 0,  # ADD
        'pose': {
            'position': {'x': track.x, 'y': track.y, 'z': track.z},
            'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}
        },
        'scale': {'x': size[0], 'y': size[1], 'z': size[2]},
        'color': {'r': color[0], 'g': color[1], 'b': color[2], 'a': color[3]},
        'lifetime': 0.5
    }


def build_velocity_marker(track: TrackedObject, frame_id: str = "base_link",
                          namespace: str = "nx_mimosa_vel") -> Dict[str, Any]:
    """Build velocity arrow marker."""
    speed = np.sqrt(track.vx**2 + track.vy**2 + track.vz**2)
    
    return {
        'header': {'frame_id': frame_id},
        'ns': namespace,
        'id': track.track_id,
        'type': 0,  # ARROW
        'action': 0,
        'points': [
            {'x': track.x, 'y': track.y, 'z': track.z},
            {'x': track.x + track.vx, 'y': track.y + track.vy, 'z': track.z + track.vz}
        ],
        'scale': {'x': 0.3, 'y': 0.6, 'z': 0.0},
        'color': {'r': 1.0, 'g': 0.5, 'b': 0.0, 'a': 0.9},
        'lifetime': 0.5
    }


# =============================================================================
# NX-MIMOSA ROS2 NODE (Only available when ROS2 is installed)
# =============================================================================

if ROS2_AVAILABLE:
    
    class NXMIMOSARos2Node(Node):
        """
        ROS2 Node wrapping NX-MIMOSA tracker.
        
        Subscriptions:
            /radar/detections (PointCloud2): Raw radar detections
            
        Publishers:
            /nx_mimosa/tracks (MarkerArray): Track visualization
            /nx_mimosa/tracks_array (custom): Full track data
        
        Services:
            /nx_mimosa/get_tracks: Query current tracks
        """
        
        def __init__(self, node_name: str = "nx_mimosa_tracker"):
            super().__init__(node_name)
            
            # Parameters
            self.declare_parameter('dt', 0.1)
            self.declare_parameter('sigma', 30.0)
            self.declare_parameter('frame_id', 'base_link')
            self.declare_parameter('max_tracks', 100)
            
            self.dt = self.get_parameter('dt').value
            self.sigma = self.get_parameter('sigma').value
            self.frame_id = self.get_parameter('frame_id').value
            self.max_tracks = self.get_parameter('max_tracks').value
            
            # Tracker instances (one per track)
            self.trackers: Dict[int, NXMIMOSAAtc] = {}
            self.next_track_id = 1
            self.last_update_time = time.time()
            
            # QoS profile for sensor data
            sensor_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
            
            # Subscribers
            self.detection_sub = self.create_subscription(
                PointCloud2,
                '/radar/detections',
                self.detection_callback,
                sensor_qos
            )
            
            # Publishers
            self.marker_pub = self.create_publisher(
                MarkerArray,
                '/nx_mimosa/tracks',
                10
            )
            
            # Timer for track maintenance
            self.maintenance_timer = self.create_timer(1.0, self.maintenance_callback)
            
            self.get_logger().info(f'NX-MIMOSA ROS2 Node initialized (dt={self.dt}, sigma={self.sigma})')
        
        def detection_callback(self, msg: PointCloud2):
            """Process incoming radar detections."""
            # Parse PointCloud2
            detections = self._parse_pointcloud2(msg)
            
            if not detections:
                return
            
            current_time = time.time()
            dt = current_time - self.last_update_time
            self.last_update_time = current_time
            
            # Predict all existing tracks
            for tracker in self.trackers.values():
                tracker.predict(dt=dt)
            
            # Associate detections with tracks (simple nearest neighbor)
            unassociated = list(range(len(detections)))
            
            for track_id, tracker in list(self.trackers.items()):
                state = tracker.get_state()
                best_idx = None
                best_dist = float('inf')
                
                for i in unassociated:
                    d = detections[i]
                    dist = np.sqrt((state[0] - d.x)**2 + (state[1] - d.y)**2 + (state[2] - d.z)**2)
                    
                    if dist < best_dist and dist < 50.0:  # Gate
                        best_dist = dist
                        best_idx = i
                
                if best_idx is not None:
                    # Update track
                    d = detections[best_idx]
                    z = np.array([d.x, d.y, d.z])
                    R = np.diag([self.sigma**2, self.sigma**2, (self.sigma * 2)**2])
                    tracker.update(z, R)
                    unassociated.remove(best_idx)
            
            # Create new tracks for unassociated detections
            for i in unassociated:
                if len(self.trackers) >= self.max_tracks:
                    break
                
                d = detections[i]
                tracker = NXMIMOSAAtc(dt=self.dt, sigma=self.sigma)
                z0 = np.array([d.x, d.y, d.z])
                v0 = np.array([d.vx, d.vy, d.vz])
                tracker.initialize(z0, v0)
                
                self.trackers[self.next_track_id] = tracker
                self.next_track_id += 1
            
            # Publish tracks
            self._publish_tracks()
        
        def _parse_pointcloud2(self, msg: PointCloud2) -> List[RadarDetection]:
            """Parse PointCloud2 message to list of detections."""
            detections = []
            
            # Find field offsets
            x_offset = y_offset = z_offset = None
            for field in msg.fields:
                if field.name == 'x':
                    x_offset = field.offset
                elif field.name == 'y':
                    y_offset = field.offset
                elif field.name == 'z':
                    z_offset = field.offset
            
            if x_offset is None:
                return detections
            
            # Parse points
            point_step = msg.point_step
            for i in range(0, len(msg.data), point_step):
                try:
                    x = struct.unpack_from('f', msg.data, i + x_offset)[0]
                    y = struct.unpack_from('f', msg.data, i + y_offset)[0]
                    z = struct.unpack_from('f', msg.data, i + z_offset)[0] if z_offset else 0.0
                    
                    detections.append(RadarDetection(x=x, y=y, z=z))
                except:
                    pass
            
            return detections
        
        def _publish_tracks(self):
            """Publish track visualization."""
            marker_array = MarkerArray()
            
            for track_id, tracker in self.trackers.items():
                state = tracker.get_state()
                
                # Position marker
                marker = Marker()
                marker.header.frame_id = self.frame_id
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "nx_mimosa"
                marker.id = track_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = state[0]
                marker.pose.position.y = state[1]
                marker.pose.position.z = state[2]
                marker.pose.orientation.w = 1.0
                marker.scale.x = 4.0
                marker.scale.y = 2.0
                marker.scale.z = 1.5
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.8
                marker.lifetime.sec = 0
                marker.lifetime.nanosec = 500000000
                
                marker_array.markers.append(marker)
                
                # Velocity arrow
                vel_marker = Marker()
                vel_marker.header = marker.header
                vel_marker.ns = "nx_mimosa_vel"
                vel_marker.id = track_id
                vel_marker.type = Marker.ARROW
                vel_marker.action = Marker.ADD
                
                start = Point()
                start.x = state[0]
                start.y = state[1]
                start.z = state[2]
                
                end = Point()
                end.x = state[0] + state[3]
                end.y = state[1] + state[4]
                end.z = state[2] + state[5]
                
                vel_marker.points = [start, end]
                vel_marker.scale.x = 0.3
                vel_marker.scale.y = 0.6
                vel_marker.color.r = 1.0
                vel_marker.color.g = 0.5
                vel_marker.color.b = 0.0
                vel_marker.color.a = 0.9
                vel_marker.lifetime = marker.lifetime
                
                marker_array.markers.append(vel_marker)
            
            self.marker_pub.publish(marker_array)
        
        def maintenance_callback(self):
            """Remove stale tracks."""
            # In production, implement track coasting/deletion logic
            pass


# =============================================================================
# STANDALONE TRACKER INTERFACE (Works without ROS2)
# =============================================================================

class NXMIMOSAStandaloneInterface:
    """
    Standalone interface for NX-MIMOSA without ROS2.
    
    Useful for:
    - Unit testing
    - Integration with other frameworks
    - Batch processing
    """
    
    def __init__(self, dt: float = 0.1, sigma: float = 30.0):
        self.dt = dt
        self.sigma = sigma
        self.trackers: Dict[int, Any] = {}
        self.next_track_id = 1
        self.last_update_time = time.time()
    
    def process_detections(self, detections: List[RadarDetection]) -> List[TrackedObject]:
        """
        Process detections and return tracked objects.
        
        Args:
            detections: List of radar detections
        
        Returns:
            List of tracked objects
        """
        if NXMIMOSAAtc is None:
            raise RuntimeError("NXMIMOSAAtc not available")
        
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # Predict
        for tracker in self.trackers.values():
            tracker.predict(dt=dt)
        
        # Associate and update (simple nearest neighbor)
        unassociated = list(range(len(detections)))
        R = np.diag([self.sigma**2, self.sigma**2, (self.sigma * 2)**2])
        
        for track_id, tracker in list(self.trackers.items()):
            state = tracker.get_state()
            best_idx = None
            best_dist = float('inf')
            
            for i in unassociated:
                d = detections[i]
                dist = np.sqrt((state[0] - d.x)**2 + (state[1] - d.y)**2 + (state[2] - d.z)**2)
                
                if dist < best_dist and dist < 100.0:
                    best_dist = dist
                    best_idx = i
            
            if best_idx is not None:
                d = detections[best_idx]
                z = np.array([d.x, d.y, d.z])
                tracker.update(z, R)
                unassociated.remove(best_idx)
        
        # Create new tracks
        for i in unassociated:
            d = detections[i]
            tracker = NXMIMOSAAtc(dt=self.dt, sigma=self.sigma)
            z0 = np.array([d.x, d.y, d.z])
            v0 = np.array([d.vx, d.vy, d.vz])
            tracker.initialize(z0, v0)
            
            self.trackers[self.next_track_id] = tracker
            self.next_track_id += 1
        
        # Build output
        tracked_objects = []
        for track_id, tracker in self.trackers.items():
            state = tracker.get_state()
            mu = tracker.get_mode_probabilities()
            
            tracked_objects.append(TrackedObject(
                track_id=track_id,
                x=state[0], y=state[1], z=state[2],
                vx=state[3], vy=state[4], vz=state[5],
                covariance=np.eye(6),  # Simplified
                mode_probabilities=mu
            ))
        
        return tracked_objects
    
    def get_tracks(self) -> List[TrackedObject]:
        """Get current tracked objects."""
        tracked_objects = []
        for track_id, tracker in self.trackers.items():
            state = tracker.get_state()
            tracked_objects.append(TrackedObject(
                track_id=track_id,
                x=state[0], y=state[1], z=state[2],
                vx=state[3], vy=state[4], vz=state[5],
                covariance=np.eye(6),
                mode_probabilities=tracker.get_mode_probabilities()
            ))
        return tracked_objects


# =============================================================================
# MAIN / DEMO
# =============================================================================

def main():
    """Main entry point."""
    print("NX-MIMOSA ROS2 Interface v1.1.0")
    print("=" * 60)
    
    if ROS2_AVAILABLE:
        print("✅ ROS2 available - full functionality enabled")
        print("   Launch with: ros2 run nx_mimosa nx_mimosa_ros2_node")
        
        # Can launch node here if called as ROS2 executable
        # rclpy.init()
        # node = NXMIMOSARos2Node()
        # rclpy.spin(node)
    else:
        print("⚠️  ROS2 not available - using standalone interface")
        print("   Install ROS2 for full functionality")
    
    # Demo standalone interface
    print("\n📡 Testing standalone interface...")
    
    interface = NXMIMOSAStandaloneInterface(dt=0.1, sigma=30.0)
    
    # Simulate some detections
    detections = [
        RadarDetection(x=100.0, y=50.0, z=2.0, vx=-10.0, vy=0.0, vz=0.0),
        RadarDetection(x=200.0, y=-30.0, z=1.5, vx=-15.0, vy=2.0, vz=0.0),
    ]
    
    for i in range(5):
        # Update detections (simulate movement)
        detections = [
            RadarDetection(x=d.x + d.vx * 0.1, y=d.y + d.vy * 0.1, z=d.z, 
                          vx=d.vx, vy=d.vy, vz=d.vz)
            for d in detections
        ]
        
        # Add noise
        detections = [
            RadarDetection(x=d.x + np.random.randn() * 5, 
                          y=d.y + np.random.randn() * 5,
                          z=d.z + np.random.randn() * 2,
                          vx=d.vx, vy=d.vy, vz=d.vz)
            for d in detections
        ]
        
        tracks = interface.process_detections(detections)
        print(f"   Step {i+1}: {len(tracks)} tracks")
    
    print("\n✅ ROS2 Interface Demo Complete")


if __name__ == "__main__":
    main()
