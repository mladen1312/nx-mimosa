#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════════════════════════════
NX-MIMOSA ROS2 Radar Node
═══════════════════════════════════════════════════════════════════════════════════════════════════════

ROS2 node for publishing radar tracks and Range-Doppler maps.
Compatible with ROS2 Humble/Iron/Jazzy.

Topics:
  - /mimosa/tracks      : RadarTrackArray (custom msg)
  - /mimosa/rd_map      : sensor_msgs/PointCloud2
  - /mimosa/detections  : visualization_msgs/MarkerArray

Traceability:
  [REQ-DEPLOY-001] ROS2 integration
  [REQ-DEPLOY-002] Real-time data streaming

Author: Dr. Mladen Mešter / Nexellum d.o.o.
Version: 1.1.0
License: AGPL v3 / Commercial
═══════════════════════════════════════════════════════════════════════════════════════════════════════
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Optional, Tuple
import struct
import time

# ROS2 imports (with fallback for non-ROS environments)
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from std_msgs.msg import Header
    from sensor_msgs.msg import PointCloud2, PointField
    from visualization_msgs.msg import Marker, MarkerArray
    from geometry_msgs.msg import Point
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("Warning: ROS2 not available. Running in simulation mode.")


# =============================================================================
# DATA STRUCTURES
# =============================================================================

@dataclass
class RadarTrack:
    """Single radar track."""
    track_id: int
    range_m: float
    azimuth_deg: float
    elevation_deg: float
    velocity_ms: float
    rcs_m2: float
    confidence: int  # 0-100
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    
    def to_cartesian(self):
        """Convert spherical to Cartesian coordinates."""
        az_rad = np.radians(self.azimuth_deg)
        el_rad = np.radians(self.elevation_deg)
        
        self.x = self.range_m * np.cos(el_rad) * np.sin(az_rad)
        self.y = self.range_m * np.cos(el_rad) * np.cos(az_rad)
        self.z = self.range_m * np.sin(el_rad)
        
        # Velocity components (assume radial)
        self.vx = self.velocity_ms * np.cos(el_rad) * np.sin(az_rad)
        self.vy = self.velocity_ms * np.cos(el_rad) * np.cos(az_rad)
        self.vz = self.velocity_ms * np.sin(el_rad)


@dataclass 
class RangeDopplerMap:
    """Range-Doppler map data."""
    data: np.ndarray  # (n_doppler, n_range) in dB
    range_bins: np.ndarray  # Range values in meters
    doppler_bins: np.ndarray  # Doppler values in m/s
    timestamp: float


# =============================================================================
# ROS2 NODE
# =============================================================================

if ROS2_AVAILABLE:
    class MimosaRadarNode(Node):
        """
        NX-MIMOSA ROS2 radar node.
        
        Publishes tracks and Range-Doppler maps from the tracking system.
        """
        
        def __init__(self):
            super().__init__('mimosa_radar_node')
            
            # Parameters
            self.declare_parameter('update_rate_hz', 20.0)
            self.declare_parameter('frame_id', 'radar_link')
            self.declare_parameter('max_range_m', 150000.0)
            self.declare_parameter('n_range_bins', 1024)
            self.declare_parameter('n_doppler_bins', 64)
            
            self.update_rate = self.get_parameter('update_rate_hz').value
            self.frame_id = self.get_parameter('frame_id').value
            self.max_range = self.get_parameter('max_range_m').value
            self.n_range_bins = self.get_parameter('n_range_bins').value
            self.n_doppler_bins = self.get_parameter('n_doppler_bins').value
            
            # QoS for real-time data
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
            
            # Publishers
            self.pub_rd_map = self.create_publisher(
                PointCloud2, '/mimosa/rd_map', qos
            )
            self.pub_markers = self.create_publisher(
                MarkerArray, '/mimosa/detections', 10
            )
            
            # Timer for periodic publishing
            period = 1.0 / self.update_rate
            self.timer = self.create_timer(period, self.publish_callback)
            
            # Data buffers (populated by tracker interface)
            self.current_tracks: List[RadarTrack] = []
            self.rd_map: Optional[RangeDopplerMap] = None
            
            # Statistics
            self.publish_count = 0
            
            self.get_logger().info(
                f'NX-MIMOSA Radar Node initialized @ {self.update_rate} Hz'
            )
        
        def set_tracks(self, tracks: List[RadarTrack]):
            """Update current track list."""
            self.current_tracks = tracks
        
        def set_rd_map(self, rd_map: RangeDopplerMap):
            """Update Range-Doppler map."""
            self.rd_map = rd_map
        
        def publish_callback(self):
            """Periodic publish callback."""
            now = self.get_clock().now()
            
            # Publish RD map as PointCloud2
            if self.rd_map is not None:
                pc_msg = self._create_rd_pointcloud(now)
                self.pub_rd_map.publish(pc_msg)
            
            # Publish track markers
            if self.current_tracks:
                marker_msg = self._create_track_markers(now)
                self.pub_markers.publish(marker_msg)
            
            self.publish_count += 1
        
        def _create_rd_pointcloud(self, stamp) -> PointCloud2:
            """Create PointCloud2 from Range-Doppler map."""
            header = Header()
            header.stamp = stamp.to_msg()
            header.frame_id = self.frame_id
            
            # Fields: x (range), y (doppler), z (0), intensity (power dB)
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            
            # Generate point data
            points = []
            rd_data = self.rd_map.data
            n_doppler, n_range = rd_data.shape
            
            # Subsample for performance if needed
            step_r = max(1, n_range // 256)
            step_d = max(1, n_doppler // 32)
            
            for d_idx in range(0, n_doppler, step_d):
                for r_idx in range(0, n_range, step_r):
                    r = self.rd_map.range_bins[r_idx] if r_idx < len(self.rd_map.range_bins) else r_idx * 3.0
                    doppler = self.rd_map.doppler_bins[d_idx] if d_idx < len(self.rd_map.doppler_bins) else (d_idx - n_doppler/2) * 10
                    intensity = rd_data[d_idx, r_idx]
                    
                    # Pack as binary
                    points.append(struct.pack('ffff', r, doppler, 0.0, intensity))
            
            # Create message
            msg = PointCloud2()
            msg.header = header
            msg.height = 1
            msg.width = len(points)
            msg.fields = fields
            msg.is_bigendian = False
            msg.point_step = 16  # 4 floats * 4 bytes
            msg.row_step = msg.point_step * msg.width
            msg.data = b''.join(points)
            msg.is_dense = True
            
            return msg
        
        def _create_track_markers(self, stamp) -> MarkerArray:
            """Create visualization markers for tracks."""
            marker_array = MarkerArray()
            
            for i, track in enumerate(self.current_tracks):
                track.to_cartesian()
                
                # Track position marker (sphere)
                marker = Marker()
                marker.header.stamp = stamp.to_msg()
                marker.header.frame_id = self.frame_id
                marker.ns = 'tracks'
                marker.id = track.track_id
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                
                marker.pose.position.x = track.x
                marker.pose.position.y = track.y
                marker.pose.position.z = track.z
                marker.pose.orientation.w = 1.0
                
                # Size based on RCS
                size = max(50.0, min(500.0, track.rcs_m2 * 100))
                marker.scale.x = size
                marker.scale.y = size
                marker.scale.z = size
                
                # Color based on confidence (green=high, red=low)
                marker.color.r = 1.0 - track.confidence / 100.0
                marker.color.g = track.confidence / 100.0
                marker.color.b = 0.2
                marker.color.a = 0.8
                
                marker.lifetime.sec = 0
                marker.lifetime.nanosec = int(0.2e9)  # 200ms
                
                marker_array.markers.append(marker)
                
                # Velocity arrow
                if abs(track.velocity_ms) > 1.0:
                    arrow = Marker()
                    arrow.header = marker.header
                    arrow.ns = 'velocities'
                    arrow.id = track.track_id + 10000
                    arrow.type = Marker.ARROW
                    arrow.action = Marker.ADD
                    
                    start = Point()
                    start.x = track.x
                    start.y = track.y
                    start.z = track.z
                    
                    end = Point()
                    scale = 10.0  # Arrow length scaling
                    end.x = track.x + track.vx * scale
                    end.y = track.y + track.vy * scale
                    end.z = track.z + track.vz * scale
                    
                    arrow.points = [start, end]
                    arrow.scale.x = 20.0  # Shaft diameter
                    arrow.scale.y = 40.0  # Head diameter
                    arrow.scale.z = 0.0
                    
                    arrow.color.r = 0.0
                    arrow.color.g = 0.5
                    arrow.color.b = 1.0
                    arrow.color.a = 0.7
                    
                    arrow.lifetime = marker.lifetime
                    
                    marker_array.markers.append(arrow)
            
            return marker_array


# =============================================================================
# SIMULATION MODE (for testing without ROS2)
# =============================================================================

class SimulatedRadarNode:
    """Simulated radar node for testing without ROS2."""
    
    def __init__(self, update_rate_hz: float = 20.0):
        self.update_rate = update_rate_hz
        self.frame_id = 'radar_link'
        self.current_tracks: List[RadarTrack] = []
        self.rd_map: Optional[RangeDopplerMap] = None
        self.publish_count = 0
        
        print(f"SimulatedRadarNode initialized @ {update_rate_hz} Hz")
    
    def set_tracks(self, tracks: List[RadarTrack]):
        self.current_tracks = tracks
    
    def set_rd_map(self, rd_map: RangeDopplerMap):
        self.rd_map = rd_map
    
    def spin_once(self):
        """Process one cycle."""
        if self.rd_map is not None:
            print(f"[{self.publish_count}] RD Map: {self.rd_map.data.shape}, "
                  f"Tracks: {len(self.current_tracks)}")
        self.publish_count += 1
        time.sleep(1.0 / self.update_rate)


# =============================================================================
# TRACKER INTERFACE
# =============================================================================

class TrackerInterface:
    """
    Interface between NX-MIMOSA tracker and ROS2 node.
    
    Can connect to:
    - Shared memory (FPGA driver)
    - UDP socket
    - Direct Python API
    """
    
    def __init__(self, node):
        self.node = node
        self.n_range_bins = 1024
        self.n_doppler_bins = 64
        
        # Generate dummy data for testing
        self.range_bins = np.linspace(0, 150000, self.n_range_bins)
        self.doppler_bins = np.linspace(-500, 500, self.n_doppler_bins)
    
    def update_from_tracker(self, tracker_state: dict):
        """
        Update node from tracker state dictionary.
        
        Expected keys:
            'tracks': List of track dictionaries
            'rd_map': 2D numpy array (dB)
        """
        # Convert tracks
        tracks = []
        for t in tracker_state.get('tracks', []):
            track = RadarTrack(
                track_id=t.get('id', 0),
                range_m=t.get('range', 0),
                azimuth_deg=t.get('azimuth', 0),
                elevation_deg=t.get('elevation', 0),
                velocity_ms=t.get('velocity', 0),
                rcs_m2=t.get('rcs', 1.0),
                confidence=t.get('confidence', 50)
            )
            tracks.append(track)
        
        self.node.set_tracks(tracks)
        
        # Convert RD map
        rd_data = tracker_state.get('rd_map')
        if rd_data is not None:
            rd_map = RangeDopplerMap(
                data=rd_data,
                range_bins=self.range_bins[:rd_data.shape[1]],
                doppler_bins=self.doppler_bins[:rd_data.shape[0]],
                timestamp=time.time()
            )
            self.node.set_rd_map(rd_map)
    
    def generate_test_data(self):
        """Generate test data for demonstration."""
        # Simulated tracks
        tracks = [
            {'id': 1, 'range': 5000, 'azimuth': 30, 'elevation': 5, 
             'velocity': 250, 'rcs': 10, 'confidence': 95},
            {'id': 2, 'range': 12000, 'azimuth': -15, 'elevation': 2,
             'velocity': -150, 'rcs': 5, 'confidence': 80},
            {'id': 3, 'range': 8000, 'azimuth': 45, 'elevation': -3,
             'velocity': 50, 'rcs': 2, 'confidence': 60},
        ]
        
        # Simulated RD map with targets
        rd_map = np.random.randn(self.n_doppler_bins, self.n_range_bins) * 5 + 10
        
        # Add target peaks
        for t in tracks:
            r_idx = int(t['range'] / 150000 * self.n_range_bins)
            d_idx = int((t['velocity'] + 500) / 1000 * self.n_doppler_bins)
            if 0 <= r_idx < self.n_range_bins and 0 <= d_idx < self.n_doppler_bins:
                rd_map[d_idx-2:d_idx+3, r_idx-2:r_idx+3] += 30
        
        return {'tracks': tracks, 'rd_map': rd_map}


# =============================================================================
# MAIN
# =============================================================================

def main():
    """Main entry point."""
    if ROS2_AVAILABLE:
        rclpy.init()
        node = MimosaRadarNode()
        interface = TrackerInterface(node)
        
        # Demo: Generate test data periodically
        def test_callback():
            test_data = interface.generate_test_data()
            interface.update_from_tracker(test_data)
        
        test_timer = node.create_timer(0.1, test_callback)
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        # Simulation mode
        node = SimulatedRadarNode(update_rate_hz=10.0)
        interface = TrackerInterface(node)
        
        print("Running in simulation mode (no ROS2)")
        print("Press Ctrl+C to exit")
        
        try:
            while True:
                test_data = interface.generate_test_data()
                interface.update_from_tracker(test_data)
                node.spin_once()
        except KeyboardInterrupt:
            print("\nShutdown")


if __name__ == '__main__':
    main()
