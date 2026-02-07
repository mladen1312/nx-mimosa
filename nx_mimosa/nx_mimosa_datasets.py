"""NX-MIMOSA Dataset Adapters — Bridge Real Data to Tracker
=====================================================================

Adapters for common radar/lidar datasets, converting their native formats
into NX-MIMOSA's scan-based input format.

Supported formats:
    - nuScenes (radar/lidar, Motional)
    - CARLA simulator (synthetic radar)
    - RADIATE (real-world radar, Heriot-Watt)
    - Generic CSV/JSON detections

Each adapter produces:
    - detections: Nx3 numpy arrays [x, y, z] per scan (ENU or Cartesian)
    - ground_truth: Nx3 arrays for OSPA/SIAP evaluation
    - metadata: timing, sensor info, scenario description

Author: Dr. Mladen Mešter — Nexellum d.o.o.
"""

from __future__ import annotations
import numpy as np
import json
import csv
from pathlib import Path
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Iterator


@dataclass
class ScanData:
    """[REQ-V55-DATA-01] A single radar scan with detections and optional truth."""
    timestamp: float                      # Seconds since scenario start
    detections: np.ndarray                # Nx3 [x, y, z] in meters
    ground_truth: Optional[np.ndarray] = None  # Mx3 truth positions
    metadata: Dict = field(default_factory=dict)
    
    @property
    def n_detections(self) -> int:
        return len(self.detections) if self.detections.ndim > 1 else 0
    
    @property
    def n_truths(self) -> int:
        if self.ground_truth is None:
            return 0
        return len(self.ground_truth) if self.ground_truth.ndim > 1 else 0


@dataclass
class DatasetInfo:
    """[REQ-V55-DATA-02] Dataset metadata."""
    name: str
    source: str               # "nuscenes", "carla", "radiate", "csv", "synthetic"
    n_scans: int
    dt_nominal: float         # Nominal scan interval
    coordinate_frame: str     # "enu", "ego", "sensor"
    description: str = ""
    sensor_type: str = "radar"


class NuScenesAdapter:
    """[REQ-V55-DATA-03] Adapter for nuScenes radar point cloud.
    
    Reads nuScenes-format radar point cloud data and converts to
    NX-MIMOSA scan format.
    
    nuScenes radar provides 5 radar channels around the vehicle, each
    producing point clouds with [x, y, z, dyn_prop, rcs, vx_comp, vy_comp].
    
    Usage::
    
        adapter = NuScenesAdapter("/data/nuscenes/v1.0-mini")
        for scan in adapter.iterate_scene("scene-0061"):
            tracker.process_scan(scan.detections)
    
    Note: Requires nuscenes-devkit: ``pip install nuscenes-devkit``
    """
    
    def __init__(self, dataroot: str, version: str = "v1.0-mini"):
        self.dataroot = Path(dataroot)
        self.version = version
        self._nusc = None
    
    def _load(self):
        """Lazy load nuscenes-devkit."""
        if self._nusc is not None:
            return
        try:
            from nuscenes.nuscenes import NuScenes
            self._nusc = NuScenes(version=self.version, dataroot=str(self.dataroot))
        except ImportError:
            raise ImportError(
                "nuscenes-devkit required: pip install nuscenes-devkit\n"
                "Dataset: https://www.nuscenes.org/nuscenes"
            )
    
    def iterate_scene(self, scene_name: str,
                      radar_channels: Optional[List[str]] = None
                      ) -> Iterator[ScanData]:
        """[REQ-V55-DATA-04] Iterate radar scans from a nuScenes scene.
        
        Args:
            scene_name: e.g. "scene-0061"
            radar_channels: Which radars to use. Default: all 5.
                Options: RADAR_FRONT, RADAR_FRONT_LEFT, RADAR_FRONT_RIGHT,
                         RADAR_BACK_LEFT, RADAR_BACK_RIGHT
        
        Yields:
            ScanData with detections in ego vehicle frame [x, y, z].
        """
        self._load()
        
        if radar_channels is None:
            radar_channels = [
                'RADAR_FRONT', 'RADAR_FRONT_LEFT', 'RADAR_FRONT_RIGHT',
                'RADAR_BACK_LEFT', 'RADAR_BACK_RIGHT'
            ]
        
        # Find scene
        scene = None
        for s in self._nusc.scene:
            if s['name'] == scene_name:
                scene = s
                break
        if scene is None:
            raise ValueError(f"Scene '{scene_name}' not found")
        
        # Iterate samples
        sample_token = scene['first_sample_token']
        t0 = None
        
        while sample_token:
            sample = self._nusc.get('sample', sample_token)
            timestamp = sample['timestamp'] / 1e6  # µs → s
            if t0 is None:
                t0 = timestamp
            
            all_points = []
            for channel in radar_channels:
                if channel in sample['data']:
                    sd_token = sample['data'][channel]
                    # Get radar points in ego frame
                    from nuscenes.utils.data_classes import RadarPointCloud
                    pc_path = self._nusc.get_sample_data_path(sd_token)
                    pc = RadarPointCloud.from_file(pc_path)
                    
                    # Points: [x, y, z, dyn_prop, id, rcs, vx, vy, ...]
                    # Take x, y, z (in sensor frame)
                    pts = pc.points[:3, :].T  # Nx3
                    
                    # Transform sensor → ego using calibration
                    sd = self._nusc.get('sample_data', sd_token)
                    cs = self._nusc.get('calibrated_sensor', sd['calibrated_sensor_token'])
                    # Simplified: add sensor translation offset
                    offset = np.array(cs['translation'])
                    pts = pts + offset
                    
                    all_points.append(pts)
            
            if all_points:
                detections = np.vstack(all_points)
            else:
                detections = np.empty((0, 3))
            
            # Get ground truth annotations
            gt_positions = []
            for ann_token in sample['anns']:
                ann = self._nusc.get('sample_annotation', ann_token)
                pos = np.array(ann['translation'][:3])
                gt_positions.append(pos)
            
            gt = np.array(gt_positions) if gt_positions else None
            
            yield ScanData(
                timestamp=timestamp - t0,
                detections=detections,
                ground_truth=gt,
                metadata={'scene': scene_name, 'token': sample_token}
            )
            
            sample_token = sample.get('next', '')
    
    def get_info(self) -> DatasetInfo:
        return DatasetInfo(
            name="nuScenes", source="nuscenes",
            n_scans=0, dt_nominal=0.5,
            coordinate_frame="ego",
            description="Automotive radar point clouds from 5 sensors",
            sensor_type="radar"
        )


class CARLAAdapter:
    """[REQ-V55-DATA-05] Adapter for CARLA simulator radar output.
    
    Reads radar data from CARLA's radar sensor output format.
    Each detection has [altitude, azimuth, depth, velocity].
    
    Usage::
    
        adapter = CARLAAdapter()
        scans = adapter.load_from_csv("carla_radar_log.csv")
        for scan in scans:
            tracker.process_scan(scan.detections)
    """
    
    @staticmethod
    def load_from_csv(filepath: str, dt: float = 0.05) -> List[ScanData]:
        """[REQ-V55-DATA-06] Load CARLA radar CSV.
        
        Expected CSV format (per-detection rows):
            frame, altitude, azimuth, depth, velocity
        
        Converts polar (depth, azimuth, altitude) to Cartesian (x, y, z).
        
        Args:
            filepath: Path to CSV file
            dt: Frame interval in seconds
        
        Returns:
            List of ScanData per frame
        """
        frames = {}
        
        with open(filepath) as f:
            reader = csv.DictReader(f)
            for row in reader:
                frame = int(row['frame'])
                alt = float(row['altitude'])   # radians
                az = float(row['azimuth'])     # radians
                depth = float(row['depth'])    # meters
                
                # Polar → Cartesian (CARLA frame: x=forward, y=right, z=up)
                x = depth * np.cos(alt) * np.cos(az)
                y = depth * np.cos(alt) * np.sin(az)
                z = depth * np.sin(alt)
                
                if frame not in frames:
                    frames[frame] = []
                frames[frame].append([x, y, z])
        
        scans = []
        for frame_idx in sorted(frames.keys()):
            pts = np.array(frames[frame_idx])
            scans.append(ScanData(
                timestamp=frame_idx * dt,
                detections=pts,
                metadata={'source': 'carla', 'frame': frame_idx}
            ))
        
        return scans
    
    @staticmethod
    def from_carla_sensor(raw_data) -> np.ndarray:
        """[REQ-V55-DATA-07] Direct conversion from CARLA SensorData.
        
        For use inside CARLA Python API callback:
        
            def radar_callback(data):
                dets = CARLAAdapter.from_carla_sensor(data)
                tracker.process_scan(dets)
        
        Args:
            raw_data: carla.RadarMeasurement object
        
        Returns:
            Nx3 array [x, y, z] in vehicle frame
        """
        points = np.frombuffer(raw_data.raw_data, dtype=np.float32).reshape(-1, 4)
        # [velocity, azimuth, altitude, depth]
        vel, az, alt, depth = points.T
        
        x = depth * np.cos(alt) * np.cos(az)
        y = depth * np.cos(alt) * np.sin(az)
        z = depth * np.sin(alt)
        
        return np.column_stack([x, y, z])


class RADIATEAdapter:
    """[REQ-V55-DATA-08] Adapter for RADIATE dataset (Heriot-Watt).
    
    RADIATE provides real-world 360° scanning radar data captured in
    diverse weather (rain, fog, snow, night). Published in RA-L 2021.
    
    Dataset: https://pro.hw.ac.uk/radiate/
    
    The radar provides polar images (range-azimuth). This adapter
    processes the accompanying object annotations as Cartesian detections.
    """
    
    @staticmethod
    def load_annotations(annotation_path: str,
                         radar_resolution: float = 0.175
                         ) -> List[ScanData]:
        """[REQ-V55-DATA-09] Load RADIATE annotation JSON.
        
        Args:
            annotation_path: Path to annotations JSON/folder
            radar_resolution: Range resolution in meters/pixel
        
        Returns:
            List of ScanData from annotated radar frames
        """
        path = Path(annotation_path)
        
        if path.suffix == '.json':
            with open(path) as f:
                annotations = json.load(f)
        else:
            raise ValueError("Expected .json annotation file")
        
        scans = []
        for frame_ann in annotations.get('frames', []):
            frame_id = frame_ann.get('frame_id', 0)
            timestamp = frame_ann.get('timestamp', frame_id * 0.25)
            
            detections = []
            truths = []
            for obj in frame_ann.get('objects', []):
                x = obj.get('x', 0) * radar_resolution
                y = obj.get('y', 0) * radar_resolution
                z = 0.0  # 2D radar
                
                detections.append([x, y, z])
                if 'class' in obj:
                    truths.append([x, y, z])
            
            dets = np.array(detections) if detections else np.empty((0, 3))
            gt = np.array(truths) if truths else None
            
            scans.append(ScanData(
                timestamp=timestamp,
                detections=dets,
                ground_truth=gt,
                metadata={'source': 'radiate', 'frame': frame_id}
            ))
        
        return scans


class GenericCSVAdapter:
    """[REQ-V55-DATA-10] Generic CSV adapter for any detection source.
    
    Reads CSV files with columns: timestamp, x, y, z [, truth_x, truth_y, truth_z]
    
    This is the simplest way to connect any detection source to NX-MIMOSA.
    
    Usage::
    
        adapter = GenericCSVAdapter()
        scans = adapter.load("my_radar_data.csv")
        for scan in scans:
            tracker.process_scan(scan.detections)
    """
    
    @staticmethod
    def load(filepath: str, delimiter: str = ',',
             time_col: str = 'timestamp',
             x_col: str = 'x', y_col: str = 'y', z_col: str = 'z',
             has_truth: bool = False) -> List[ScanData]:
        """[REQ-V55-DATA-11] Load generic CSV detections.
        
        Groups rows by timestamp into scans.
        
        Args:
            filepath: CSV file path
            delimiter: Column separator
            time_col, x_col, y_col, z_col: Column names
            has_truth: If True, also reads truth_x, truth_y, truth_z columns
        
        Returns:
            List of ScanData sorted by timestamp
        """
        frames = {}
        
        with open(filepath) as f:
            reader = csv.DictReader(f, delimiter=delimiter)
            for row in reader:
                t = float(row[time_col])
                x = float(row[x_col])
                y = float(row[y_col])
                z = float(row.get(z_col, 0))
                
                if t not in frames:
                    frames[t] = {'dets': [], 'truth': []}
                frames[t]['dets'].append([x, y, z])
                
                if has_truth and 'truth_x' in row:
                    frames[t]['truth'].append([
                        float(row['truth_x']),
                        float(row['truth_y']),
                        float(row.get('truth_z', 0))
                    ])
        
        scans = []
        for t in sorted(frames.keys()):
            dets = np.array(frames[t]['dets'])
            gt = np.array(frames[t]['truth']) if frames[t]['truth'] else None
            scans.append(ScanData(
                timestamp=t, detections=dets, ground_truth=gt,
                metadata={'source': 'csv'}
            ))
        
        return scans
    
    @staticmethod
    def save(scans: List[ScanData], filepath: str) -> None:
        """[REQ-V55-DATA-12] Save scan data to CSV.
        
        Args:
            scans: List of ScanData
            filepath: Output CSV path
        """
        with open(filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            header = ['timestamp', 'x', 'y', 'z']
            writer.writerow(header)
            
            for scan in scans:
                if scan.n_detections == 0:
                    continue
                for det in scan.detections:
                    writer.writerow([scan.timestamp, det[0], det[1], det[2]])


class SyntheticScenarioGenerator:
    """[REQ-V55-DATA-13] Generate synthetic test scenarios for benchmarking.
    
    Creates reproducible radar scenarios with ground truth for validation.
    Simulates clutter, missed detections, and measurement noise.
    
    Usage::
    
        gen = SyntheticScenarioGenerator(seed=42)
        scans = gen.crossing_targets(n_scans=100)
        for scan in scans:
            tracker.process_scan(scan.detections)
    """
    
    def __init__(self, seed: int = 42, noise_std: float = 50.0,
                 p_detection: float = 0.95, clutter_rate: float = 2.0):
        self.rng = np.random.RandomState(seed)
        self.noise_std = noise_std
        self.p_detection = p_detection
        self.clutter_rate = clutter_rate
        self.volume = np.array([100000, 100000, 20000])  # Surveillance volume
    
    def _add_clutter(self, detections: List[np.ndarray], center: np.ndarray) -> List[np.ndarray]:
        """Add Poisson-distributed clutter."""
        n_clutter = self.rng.poisson(self.clutter_rate)
        for _ in range(n_clutter):
            clutter_pos = center + (self.rng.rand(3) - 0.5) * self.volume * 0.1
            detections.append(clutter_pos)
        return detections
    
    def straight_line(self, n_targets: int = 3, n_scans: int = 100,
                      dt: float = 1.0) -> List[ScanData]:
        """[REQ-V55-DATA-14] Straight-line constant-velocity targets."""
        scans = []
        trajectories = []
        
        for i in range(n_targets):
            x0 = self.rng.randn(3) * 10000
            v = self.rng.randn(3) * 100 + np.array([200, 0, 0])
            trajectories.append((x0, v))
        
        for t_idx in range(n_scans):
            t = t_idx * dt
            truths = []
            dets = []
            
            for x0, v in trajectories:
                pos = x0 + v * t
                truths.append(pos)
                
                if self.rng.rand() < self.p_detection:
                    noisy = pos + self.rng.randn(3) * self.noise_std
                    dets.append(noisy)
            
            dets = self._add_clutter(dets, np.mean(truths, axis=0))
            
            scans.append(ScanData(
                timestamp=t,
                detections=np.array(dets) if dets else np.empty((0, 3)),
                ground_truth=np.array(truths),
                metadata={'scenario': 'straight_line', 'scan': t_idx}
            ))
        
        return scans
    
    def crossing_targets(self, n_scans: int = 100, dt: float = 1.0) -> List[ScanData]:
        """[REQ-V55-DATA-15] Two targets crossing paths — association stress test."""
        scans = []
        
        for t_idx in range(n_scans):
            t = t_idx * dt
            
            # Target A: moving east
            pos_a = np.array([200 * t, 5000, 8000])
            # Target B: moving north, crossing A at t≈25
            pos_b = np.array([5000, 200 * t, 8000])
            
            truths = [pos_a, pos_b]
            dets = []
            
            for pos in truths:
                if self.rng.rand() < self.p_detection:
                    dets.append(pos + self.rng.randn(3) * self.noise_std)
            
            dets = self._add_clutter(dets, np.mean(truths, axis=0))
            
            scans.append(ScanData(
                timestamp=t,
                detections=np.array(dets) if dets else np.empty((0, 3)),
                ground_truth=np.array(truths),
                metadata={'scenario': 'crossing', 'scan': t_idx}
            ))
        
        return scans
    
    def extreme_clutter(self, n_targets: int = 2, n_scans: int = 80,
                        clutter_mult: float = 10.0, dt: float = 1.0
                        ) -> List[ScanData]:
        """[REQ-V55-DATA-16] Dense clutter scenario — MHT stress test.
        
        10× normal clutter rate. Tests association algorithm robustness.
        """
        saved_rate = self.clutter_rate
        self.clutter_rate = saved_rate * clutter_mult
        
        scans = []
        trajectories = []
        for i in range(n_targets):
            x0 = np.array([i * 3000, 0, 5000], dtype=float)
            v = np.array([150, 50 * (i - n_targets//2), 0], dtype=float)
            trajectories.append((x0, v))
        
        for t_idx in range(n_scans):
            t = t_idx * dt
            truths = []
            dets = []
            
            for x0, v in trajectories:
                pos = x0 + v * t
                truths.append(pos)
                if self.rng.rand() < self.p_detection:
                    dets.append(pos + self.rng.randn(3) * self.noise_std)
            
            center = np.mean(truths, axis=0)
            dets = self._add_clutter(dets, center)
            
            scans.append(ScanData(
                timestamp=t,
                detections=np.array(dets) if dets else np.empty((0, 3)),
                ground_truth=np.array(truths),
                metadata={'scenario': 'extreme_clutter', 'clutter_mult': clutter_mult}
            ))
        
        self.clutter_rate = saved_rate
        return scans
    
    def maneuvering(self, n_scans: int = 120, dt: float = 1.0) -> List[ScanData]:
        """[REQ-V55-DATA-17] High-g maneuvering target (fighter dogfight)."""
        scans = []
        omega = 0.05  # Turn rate rad/s
        speed = 280.0  # m/s
        
        for t_idx in range(n_scans):
            t = t_idx * dt
            
            # Fighter executing S-turn
            phase = omega * t
            if t_idx > 60:
                phase = -omega * (t - 60 * dt)  # Reverse turn
            
            x = speed * t
            y = 5000 + (speed / omega) * (1 - np.cos(phase))
            z = 8000 + 500 * np.sin(0.02 * t)
            pos = np.array([x, y, z])
            
            dets = []
            if self.rng.rand() < self.p_detection:
                dets.append(pos + self.rng.randn(3) * self.noise_std)
            
            dets = self._add_clutter(dets, pos)
            
            scans.append(ScanData(
                timestamp=t,
                detections=np.array(dets) if dets else np.empty((0, 3)),
                ground_truth=pos.reshape(1, 3),
                metadata={'scenario': 'maneuvering', 'scan': t_idx}
            ))
        
        return scans
