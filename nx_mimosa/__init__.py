"""NX-MIMOSA v6.0.1: Radar multi-target tracker — 5,000 targets in 40 ms.

C++ core + Python intelligence. ECM detection, military ID, NATO output.
11,493 lines across 9 modules. 62 unique classes. 340 tests.

Quick Start::

    from nx_mimosa import MultiTargetTracker
    tracker = MultiTargetTracker(dt=5.0, r_std=150.0, domain="air")
    for scan in radar_scans:
        tracks = tracker.process_scan(measurements)

Nexellum d.o.o. — Dr. Mladen Mešter — mladen@nexellum.com
"""

__version__ = "6.0.1"
__author__ = "Dr. Mladen Mešter"
__email__ = "mladen@nexellum.com"
__license__ = "AGPL-3.0-or-later"

# ---------------------------------------------------------------------------
# Core MTT — filters, association, metrics
# ---------------------------------------------------------------------------
from .nx_mimosa_mtt import (
    # Primary API
    MultiTargetTracker,
    TrackManagerConfig,
    TrackState,
    TrackStatus,
    TrackManager,
    # Kalman filters
    KalmanFilter3D,
    EKF3D,
    UKF3D,
    IMM3D,
    # Particle filter
    ParticleFilter3D,
    # RFS filters
    GaussianComponent,
    GMPHD,
    CPHD,
    LMBComponent,
    LMB,
    # Association
    AssociationMethod,
    gnn_associate,
    jpda_associate,
    mht_associate,
    mht_associate_enhanced,
    MHTHypothesis,
    MHTHypothesisTree,
    # Motion models
    MotionModel3D,
    make_cv3d_matrices,
    make_ca3d_matrices,
    make_ct3d_matrices,
    # Metrics
    compute_nees,
    compute_nis,
    compute_ospa,
    compute_gospa,
    compute_siap_metrics,
    # Sensor bias
    SensorBiasEstimator,
    SensorBias,
    # OOSM
    OOSMHandler,
    # Doppler
    make_cv3d_doppler_matrices,
    doppler_measurement_matrix,
    compute_radial_velocity,
    # Track-to-track association
    t2ta_associate,
    fuse_tracks,
    T2TAPair,
    # Track quality & coasting
    assess_track_quality,
    TrackQualityReport,
    TrackCoaster,
    # ECM detection (core)
    ECMDetector,
)

# ---------------------------------------------------------------------------
# Intelligence — platform ID, ECM pipeline, intent prediction
# ---------------------------------------------------------------------------
from .nx_mimosa_intelligence import (
    IntelligencePipeline,
    PlatformClassifier,
    IntentPredictor,
    ThreatLevel,
    ECMStatus,
    IntentType,
    PlatformClass,
    ECMReport,
    IntentReport,
    IntelligenceReport,
)

# ---------------------------------------------------------------------------
# Multi-sensor fusion
# ---------------------------------------------------------------------------
from .nx_mimosa_fusion import (
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

# ---------------------------------------------------------------------------
# Coordinate transforms — WGS-84 / ECEF / ENU / polar / spherical
# ---------------------------------------------------------------------------
from .nx_mimosa_coords import (
    geodetic_to_ecef,
    ecef_to_geodetic,
    ecef_to_enu,
    enu_to_ecef,
    geodetic_to_enu,
    enu_to_geodetic,
    spherical_to_cartesian,
    cartesian_to_spherical,
    spherical_to_geodetic,
    geodetic_to_spherical,
    ecef_to_spherical,
    spherical_to_ecef,
    covariance_spherical_to_cartesian,
    covariance_cartesian_to_spherical,
    bearing_between,
    destination_point,
    haversine_distance,
    SensorLocation,
    unbiased_polar_to_cartesian_2d,
    unbiased_spherical_to_cartesian_3d,
    jacobian_spherical_to_cartesian_3d,
    jacobian_cartesian_to_spherical_3d,
)

# ---------------------------------------------------------------------------
# Dataset adapters — nuScenes / CARLA / RADIATE / CSV / synthetic
# ---------------------------------------------------------------------------
from .nx_mimosa_datasets import (
    ScanData,
    DatasetInfo,
    NuScenesAdapter,
    CARLAAdapter,
    RADIATEAdapter,
    GenericCSVAdapter,
    SyntheticScenarioGenerator,
)

# ---------------------------------------------------------------------------
# __all__
# ---------------------------------------------------------------------------
__all__ = [
    "__version__",
    # Core tracker
    "MultiTargetTracker", "TrackManagerConfig", "TrackState", "TrackStatus", "TrackManager",
    # Filters
    "KalmanFilter3D", "EKF3D", "UKF3D", "IMM3D", "ParticleFilter3D",
    # RFS
    "GaussianComponent", "GMPHD", "CPHD", "LMBComponent", "LMB",
    # Association
    "AssociationMethod", "gnn_associate", "jpda_associate", "mht_associate",
    "mht_associate_enhanced", "MHTHypothesis", "MHTHypothesisTree",
    # Motion models
    "MotionModel3D", "make_cv3d_matrices", "make_ca3d_matrices", "make_ct3d_matrices",
    # Metrics
    "compute_nees", "compute_nis", "compute_ospa", "compute_gospa", "compute_siap_metrics",
    # Sensor bias / OOSM
    "SensorBiasEstimator", "SensorBias", "OOSMHandler",
    # Doppler
    "make_cv3d_doppler_matrices", "doppler_measurement_matrix", "compute_radial_velocity",
    # T2TA
    "t2ta_associate", "fuse_tracks", "T2TAPair",
    # Track management
    "assess_track_quality", "TrackQualityReport", "TrackCoaster", "ECMDetector",
    # Intelligence
    "IntelligencePipeline", "PlatformClassifier", "IntentPredictor",
    "ThreatLevel", "ECMStatus", "IntentType", "PlatformClass",
    "ECMReport", "IntentReport", "IntelligenceReport",
    # Fusion
    "MultiSensorFusionEngine", "SensorType", "SensorConfig", "SensorMeasurement",
    "SensorHealth", "make_radar_sensor", "make_doppler_radar_sensor",
    "make_eo_sensor", "make_adsb_sensor", "make_esm_sensor",
    # Coords
    "geodetic_to_ecef", "ecef_to_geodetic", "ecef_to_enu", "enu_to_ecef",
    "geodetic_to_enu", "enu_to_geodetic",
    "spherical_to_cartesian", "cartesian_to_spherical",
    "spherical_to_geodetic", "geodetic_to_spherical",
    "ecef_to_spherical", "spherical_to_ecef",
    "covariance_spherical_to_cartesian", "covariance_cartesian_to_spherical",
    "bearing_between", "destination_point", "haversine_distance",
    "SensorLocation", "unbiased_polar_to_cartesian_2d",
    "unbiased_spherical_to_cartesian_3d",
    "jacobian_spherical_to_cartesian_3d", "jacobian_cartesian_to_spherical_3d",
    # Datasets
    "ScanData", "DatasetInfo", "NuScenesAdapter", "CARLAAdapter",
    "RADIATEAdapter", "GenericCSVAdapter", "SyntheticScenarioGenerator",
]
