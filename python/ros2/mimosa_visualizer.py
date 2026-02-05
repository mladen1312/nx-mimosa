#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════════════════════════════
NX-MIMOSA Real-Time Radar Visualizer
═══════════════════════════════════════════════════════════════════════════════════════════════════════

High-performance radar display using PyQtGraph.
Achieves >60 FPS for real-time Range-Doppler map visualization.

Features:
  - Range-Doppler heatmap display
  - Track overlay with velocity vectors
  - Detection markers
  - CFAR threshold visualization
  - Performance statistics

Traceability:
  [REQ-VIS-001] Real-time visualization (>30 FPS)
  [REQ-VIS-002] Track display with metadata

Author: Dr. Mladen Mešter / Nexellum d.o.o.
Version: 1.1.0
License: AGPL v3 / Commercial
═══════════════════════════════════════════════════════════════════════════════════════════════════════
"""

import numpy as np
import time
from dataclasses import dataclass
from typing import List, Optional, Callable
from collections import deque

# PyQtGraph imports
try:
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtCore, QtWidgets
    PYQTGRAPH_AVAILABLE = True
except ImportError:
    PYQTGRAPH_AVAILABLE = False
    print("Warning: PyQtGraph not available. Install with: pip install pyqtgraph")

# Optional ROS2 subscriber
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


# =============================================================================
# DATA STRUCTURES
# =============================================================================

@dataclass
class DisplayTrack:
    """Track for display."""
    track_id: int
    x: float  # Range or X position
    y: float  # Doppler or Y position
    vx: float = 0.0
    vy: float = 0.0
    rcs: float = 1.0
    confidence: int = 50
    is_jammer: bool = False


@dataclass
class DetectionMarker:
    """Single detection marker."""
    x: float
    y: float
    amplitude: float
    is_cfar: bool = True


# =============================================================================
# MAIN VISUALIZER
# =============================================================================

if PYQTGRAPH_AVAILABLE:
    class MimosaVisualizer:
        """
        Real-time radar visualizer using PyQtGraph.
        
        Optimized for >60 FPS display of Range-Doppler maps and tracks.
        """
        
        def __init__(self, 
                     n_range_bins: int = 1024,
                     n_doppler_bins: int = 64,
                     max_range_m: float = 150000,
                     max_doppler_ms: float = 500,
                     update_rate_hz: float = 30):
            """
            Initialize visualizer.
            
            Args:
                n_range_bins: Number of range bins
                n_doppler_bins: Number of Doppler bins
                max_range_m: Maximum range in meters
                max_doppler_ms: Maximum Doppler in m/s
                update_rate_hz: Target display update rate
            """
            self.n_range_bins = n_range_bins
            self.n_doppler_bins = n_doppler_bins
            self.max_range = max_range_m
            self.max_doppler = max_doppler_ms
            self.update_rate = update_rate_hz
            
            # Data buffers
            self.rd_map = np.zeros((n_doppler_bins, n_range_bins), dtype=np.float32)
            self.tracks: List[DisplayTrack] = []
            self.detections: List[DetectionMarker] = []
            
            # Performance tracking
            self.frame_times = deque(maxlen=60)
            self.last_update = time.time()
            
            # Initialize Qt application
            self.app = pg.mkQApp("NX-MIMOSA Radar Visualizer")
            
            # Configure PyQtGraph for performance
            pg.setConfigOptions(
                antialias=False,  # Faster rendering
                useOpenGL=True,   # GPU acceleration
                enableExperimental=True
            )
            
            # Create main window
            self._setup_ui()
            
            # Update timer
            self.timer = QtCore.QTimer()
            self.timer.timeout.connect(self._update_display)
            self.timer.start(int(1000 / self.update_rate))
            
            # Data callback (set externally)
            self.data_callback: Optional[Callable] = None
        
        def _setup_ui(self):
            """Setup the user interface."""
            # Main window
            self.win = pg.GraphicsLayoutWidget(
                show=True,
                title='NX-MIMOSA Radar Visualizer v1.1'
            )
            self.win.resize(1400, 900)
            self.win.setWindowTitle('NX-MIMOSA Real-Time Radar Display')
            
            # ─────────────────────────────────────────────────────────────────
            # Range-Doppler Map (main display)
            # ─────────────────────────────────────────────────────────────────
            self.rd_plot = self.win.addPlot(
                row=0, col=0, rowspan=2,
                title='Range-Doppler Map'
            )
            self.rd_plot.setLabel('bottom', 'Range', units='km')
            self.rd_plot.setLabel('left', 'Doppler', units='m/s')
            
            # Image item for RD map
            self.rd_image = pg.ImageItem()
            self.rd_plot.addItem(self.rd_image)
            
            # Colormap
            colormap = pg.colormap.get('viridis')
            self.rd_image.setLookupTable(colormap.getLookupTable())
            
            # Scale axes
            range_scale = self.max_range / 1000 / self.n_range_bins  # km per bin
            doppler_scale = 2 * self.max_doppler / self.n_doppler_bins
            self.rd_image.setTransform(
                pg.QtGui.QTransform().scale(range_scale, doppler_scale).translate(0, -self.n_doppler_bins/2)
            )
            
            # Color bar
            self.colorbar = pg.ColorBarItem(
                values=(0, 50),
                colorMap=colormap,
                label='Power (dB)'
            )
            self.colorbar.setImageItem(self.rd_image)
            
            # Track scatter plot (overlay)
            self.track_scatter = pg.ScatterPlotItem(
                size=15,
                pen=pg.mkPen('w', width=2),
                brush=pg.mkBrush(0, 255, 0, 200)
            )
            self.rd_plot.addItem(self.track_scatter)
            
            # Detection markers
            self.detection_scatter = pg.ScatterPlotItem(
                size=8,
                pen=pg.mkPen(None),
                brush=pg.mkBrush(255, 255, 0, 150),
                symbol='x'
            )
            self.rd_plot.addItem(self.detection_scatter)
            
            # ─────────────────────────────────────────────────────────────────
            # Range Profile (right side)
            # ─────────────────────────────────────────────────────────────────
            self.range_plot = self.win.addPlot(row=0, col=1, title='Range Profile')
            self.range_plot.setLabel('bottom', 'Range', units='km')
            self.range_plot.setLabel('left', 'Power', units='dB')
            self.range_curve = self.range_plot.plot(pen='c')
            self.cfar_curve = self.range_plot.plot(pen=pg.mkPen('r', style=QtCore.Qt.DashLine))
            
            # ─────────────────────────────────────────────────────────────────
            # Doppler Profile
            # ─────────────────────────────────────────────────────────────────
            self.doppler_plot = self.win.addPlot(row=1, col=1, title='Doppler Profile')
            self.doppler_plot.setLabel('bottom', 'Doppler', units='m/s')
            self.doppler_plot.setLabel('left', 'Power', units='dB')
            self.doppler_curve = self.doppler_plot.plot(pen='m')
            
            # ─────────────────────────────────────────────────────────────────
            # Status bar
            # ─────────────────────────────────────────────────────────────────
            self.status_label = self.win.addLabel(
                text='Initializing...',
                row=2, col=0, colspan=2
            )
        
        def set_rd_map(self, data: np.ndarray):
            """
            Update Range-Doppler map data.
            
            Args:
                data: 2D array (n_doppler, n_range) in dB
            """
            self.rd_map = data.astype(np.float32)
        
        def set_tracks(self, tracks: List[DisplayTrack]):
            """Update track list."""
            self.tracks = tracks
        
        def set_detections(self, detections: List[DetectionMarker]):
            """Update detection markers."""
            self.detections = detections
        
        def _update_display(self):
            """Update all display elements (called by timer)."""
            t_start = time.time()
            
            # Call data callback if set
            if self.data_callback is not None:
                self.data_callback()
            
            # Update RD map image
            self.rd_image.setImage(
                self.rd_map.T,  # Transpose for correct orientation
                levels=(0, 50),
                autoLevels=False
            )
            
            # Update track markers
            if self.tracks:
                track_data = []
                for t in self.tracks:
                    # Convert to display coordinates
                    x = t.x / 1000  # km
                    y = t.y  # m/s (already Doppler)
                    
                    # Color based on type
                    if t.is_jammer:
                        brush = pg.mkBrush(255, 0, 0, 200)  # Red for jammer
                    elif t.confidence > 80:
                        brush = pg.mkBrush(0, 255, 0, 200)  # Green for high confidence
                    else:
                        brush = pg.mkBrush(255, 255, 0, 200)  # Yellow otherwise
                    
                    track_data.append({
                        'pos': (x, y),
                        'brush': brush,
                        'size': 10 + t.rcs * 2
                    })
                
                self.track_scatter.setData(track_data)
            else:
                self.track_scatter.clear()
            
            # Update detection markers
            if self.detections:
                det_x = [d.x / 1000 for d in self.detections]
                det_y = [d.y for d in self.detections]
                self.detection_scatter.setData(det_x, det_y)
            else:
                self.detection_scatter.clear()
            
            # Update range profile (sum over Doppler)
            if self.rd_map.size > 0:
                range_profile = np.max(self.rd_map, axis=0)
                range_axis = np.linspace(0, self.max_range/1000, len(range_profile))
                self.range_curve.setData(range_axis, range_profile)
                
                # Simple CFAR threshold visualization
                cfar_thresh = np.convolve(range_profile, np.ones(20)/20, mode='same') + 6
                self.cfar_curve.setData(range_axis, cfar_thresh)
            
            # Update Doppler profile (sum over range)
            if self.rd_map.size > 0:
                doppler_profile = np.max(self.rd_map, axis=1)
                doppler_axis = np.linspace(-self.max_doppler, self.max_doppler, len(doppler_profile))
                self.doppler_curve.setData(doppler_axis, doppler_profile)
            
            # Update performance stats
            frame_time = time.time() - t_start
            self.frame_times.append(frame_time)
            
            if len(self.frame_times) > 0:
                avg_fps = 1.0 / (sum(self.frame_times) / len(self.frame_times))
                n_tracks = len(self.tracks)
                n_dets = len(self.detections)
                
                self.status_label.setText(
                    f'FPS: {avg_fps:.1f} | Tracks: {n_tracks} | '
                    f'Detections: {n_dets} | RD Shape: {self.rd_map.shape}'
                )
        
        def run(self):
            """Start the visualizer main loop."""
            if not PYQTGRAPH_AVAILABLE:
                print("PyQtGraph not available!")
                return
            
            import sys
            sys.exit(self.app.exec_())
        
        def close(self):
            """Close the visualizer."""
            self.timer.stop()
            self.win.close()


# =============================================================================
# ROS2 SUBSCRIBER NODE
# =============================================================================

if ROS2_AVAILABLE and PYQTGRAPH_AVAILABLE:
    class VisualizerROSNode(Node):
        """ROS2 node that subscribes to radar data and updates visualizer."""
        
        def __init__(self, visualizer: MimosaVisualizer):
            super().__init__('mimosa_visualizer')
            self.viz = visualizer
            
            self.sub_rd = self.create_subscription(
                PointCloud2,
                '/mimosa/rd_map',
                self.rd_callback,
                10
            )
            
            self.get_logger().info('Visualizer ROS node initialized')
        
        def rd_callback(self, msg):
            """Process incoming RD map PointCloud2."""
            # Extract intensity values and reshape
            # (This is simplified - real implementation would properly parse PointCloud2)
            pass


# =============================================================================
# DEMO / SIMULATION
# =============================================================================

def generate_demo_data(viz: 'MimosaVisualizer', frame: int):
    """Generate simulated radar data for demonstration."""
    # Simulated RD map with noise and targets
    rd_map = np.random.randn(viz.n_doppler_bins, viz.n_range_bins) * 3 + 10
    
    # Add moving targets
    targets = [
        (0.2 + 0.001 * frame, 0.3, 35),   # Range fraction, Doppler fraction, amplitude
        (0.5 - 0.002 * frame % 0.5, 0.6, 30),
        (0.7, 0.5 + 0.003 * np.sin(frame * 0.1), 25),
    ]
    
    tracks = []
    detections = []
    
    for i, (r_frac, d_frac, amp) in enumerate(targets):
        r_idx = int(r_frac * viz.n_range_bins) % viz.n_range_bins
        d_idx = int(d_frac * viz.n_doppler_bins)
        
        # Add target peak
        rd_map[max(0, d_idx-2):min(viz.n_doppler_bins, d_idx+3),
               max(0, r_idx-2):min(viz.n_range_bins, r_idx+3)] += amp
        
        # Create track
        range_m = r_frac * viz.max_range
        doppler_ms = (d_frac - 0.5) * 2 * viz.max_doppler
        
        tracks.append(DisplayTrack(
            track_id=i+1,
            x=range_m,
            y=doppler_ms,
            rcs=amp / 10,
            confidence=90 - i * 10
        ))
        
        detections.append(DetectionMarker(
            x=range_m,
            y=doppler_ms,
            amplitude=amp
        ))
    
    # Add jammer (if frame permits)
    if frame % 100 < 50:
        j_r = 0.3
        j_d = 0.4 + 0.01 * (frame % 50)  # Moving in Doppler
        j_r_idx = int(j_r * viz.n_range_bins)
        j_d_idx = int(j_d * viz.n_doppler_bins)
        rd_map[j_d_idx-1:j_d_idx+2, j_r_idx-3:j_r_idx+4] += 45
        
        tracks.append(DisplayTrack(
            track_id=100,
            x=j_r * viz.max_range,
            y=(j_d - 0.5) * 2 * viz.max_doppler,
            rcs=20,
            confidence=30,
            is_jammer=True
        ))
    
    viz.set_rd_map(rd_map)
    viz.set_tracks(tracks)
    viz.set_detections(detections)


def main():
    """Main entry point."""
    if not PYQTGRAPH_AVAILABLE:
        print("PyQtGraph required! Install with: pip install pyqtgraph PyQt5")
        return
    
    print("═" * 70)
    print("NX-MIMOSA Real-Time Radar Visualizer")
    print("═" * 70)
    
    # Create visualizer
    viz = MimosaVisualizer(
        n_range_bins=512,
        n_doppler_bins=64,
        max_range_m=100000,
        max_doppler_ms=300,
        update_rate_hz=30
    )
    
    # Set up demo data generator
    frame_counter = [0]
    
    def demo_callback():
        generate_demo_data(viz, frame_counter[0])
        frame_counter[0] += 1
    
    viz.data_callback = demo_callback
    
    print("\nStarting visualizer...")
    print("Press Ctrl+C or close window to exit")
    
    viz.run()


if __name__ == '__main__':
    main()
