"""NX-MIMOSA: Adaptive Multi-Sensor Multi-Target Radar Tracker.

One line of Python. Any target. Any sensor. Any domain.
8.6× more accurate than the nearest open-source alternative.

Quick Start::

    from nx_mimosa import MultiTargetTracker
    tracker = MultiTargetTracker(dt=1.0, r_std=50.0, domain="military")
    for scan in radar_scans:
        tracks = tracker.process_scan(detections_3d)

Nexellum d.o.o. — Dr. Mladen Mešter — mladen@nexellum.com
"""

__version__ = "5.6.0"
__author__ = "Dr. Mladen Mešter"
__email__ = "mladen@nexellum.com"
__license__ = "AGPL-3.0-or-later"

# Core MTT
from python.nx_mimosa_mtt import (
    MultiTargetTracker,
    TrackManagerConfig,
    TrackState,
    TrackStatus,
    TrackManager,
    KalmanFilter3D,
    IMM3D,
    AssociationMethod,
    # Motion models
    make_cv3d_matrices,
    make_ca3d_matrices,
    make_ct3d_matrices,
    # Association
    gnn_associate,
    jpda_associate,
    mht_associate,
    # Metrics
    compute_nees,
    compute_nis,
    compute_ospa,
    compute_siap_metrics,
    # Sensor Bias
    SensorBiasEstimator,
    SensorBias,
    # OOSM
    OOSMHandler,
    # Doppler
    make_cv3d_doppler_matrices,
    doppler_measurement_matrix,
    compute_radial_velocity,
    # Track-to-Track Association
    t2ta_associate,
    fuse_tracks,
    T2TAPair,
    # Track Quality
    assess_track_quality,
    TrackQualityReport,
)

# Intelligence (Platform ID, ECM, Intent)
from python.nx_mimosa_intelligence import (
    IntelligencePipeline,
    PlatformClassifier,
    ECMDetector,
    IntentPredictor,
    ThreatLevel,
    ECMStatus,
    IntentType,
    PlatformClass,
    ECMReport,
    IntentReport,
    IntelligenceReport,
)

# Multi-Sensor Fusion
from python.nx_mimosa_fusion import (
    MultiSensorFusionEngine,
    SensorType,
    SensorConfig,
    SensorMeasurement,
    SensorHealth,
    make_radar_sensor,
    make_doppler_radar_sensor,
    make_eo_sensor,
    make_adsb_sensor,
    make_esm_sensor,
)

# Extended Coordinate Chains
from python.nx_mimosa_coords import (
    spherical_to_geodetic,
    geodetic_to_spherical,
    ecef_to_spherical,
    spherical_to_ecef,
    covariance_spherical_to_cartesian,
    covariance_cartesian_to_spherical,
    bearing_between,
    destination_point,
)

# Enhanced MHT
from python.nx_mimosa_mtt import (
    mht_associate_enhanced,
    MHTHypothesisTree,
)

# Dataset Adapters
from python.nx_mimosa_datasets import (
    ScanData,
    DatasetInfo,
    NuScenesAdapter,
    CARLAAdapter,
    RADIATEAdapter,
    GenericCSVAdapter,
    SyntheticScenarioGenerator,
)

# Coordinate transforms
from python.nx_mimosa_coords import (
    geodetic_to_ecef,
    ecef_to_geodetic,
    ecef_to_enu,
    enu_to_ecef,
    geodetic_to_enu,
    enu_to_geodetic,
    spherical_to_cartesian,
    cartesian_to_spherical,
    haversine_distance,
    SensorLocation,
    unbiased_polar_to_cartesian_2d,
    unbiased_spherical_to_cartesian_3d,
    jacobian_spherical_to_cartesian_3d,
    jacobian_cartesian_to_spherical_3d,
)

__all__ = [
    # Version
    "__version__",
    # MTT
    "MultiTargetTracker", "TrackManagerConfig", "TrackState", "TrackStatus",
    "TrackManager", "KalmanFilter3D", "IMM3D", "AssociationMethod",
    "make_cv3d_matrices", "make_ca3d_matrices", "make_ct3d_matrices",
    "gnn_associate", "jpda_associate", "mht_associate",
    "compute_nees", "compute_nis", "compute_ospa", "compute_siap_metrics",
    "SensorBiasEstimator", "SensorBias", "OOSMHandler",
    "make_cv3d_doppler_matrices", "doppler_measurement_matrix", "compute_radial_velocity",
    "t2ta_associate", "fuse_tracks", "T2TAPair",
    "assess_track_quality", "TrackQualityReport",
    # Intelligence
    "IntelligencePipeline", "PlatformClassifier", "ECMDetector", "IntentPredictor",
    "ThreatLevel", "ECMStatus", "IntentType", "PlatformClass",
    "ECMReport", "IntentReport", "IntelligenceReport",
    # Fusion
    "MultiSensorFusionEngine", "SensorType", "SensorConfig", "SensorMeasurement",
    "SensorHealth", "make_radar_sensor", "make_doppler_radar_sensor",
    "make_eo_sensor", "make_adsb_sensor", "make_esm_sensor",
    # Coordinate chains
    "spherical_to_geodetic", "geodetic_to_spherical",
    "ecef_to_spherical", "spherical_to_ecef",
    "covariance_spherical_to_cartesian", "covariance_cartesian_to_spherical",
    "bearing_between", "destination_point",
    # Enhanced MHT
    "mht_associate_enhanced", "MHTHypothesisTree",
    # Datasets
    "ScanData", "DatasetInfo", "NuScenesAdapter", "CARLAAdapter",
    "RADIATEAdapter", "GenericCSVAdapter", "SyntheticScenarioGenerator",
    # Coords
    "geodetic_to_ecef", "ecef_to_geodetic", "ecef_to_enu", "enu_to_ecef",
    "geodetic_to_enu", "enu_to_geodetic",
    "spherical_to_cartesian", "cartesian_to_spherical", "haversine_distance",
    "SensorLocation", "unbiased_polar_to_cartesian_2d",
    "unbiased_spherical_to_cartesian_3d",
    "jacobian_spherical_to_cartesian_3d", "jacobian_cartesian_to_spherical_3d",
]
