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

__version__ = "5.0.0"
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
    # Coords
    "geodetic_to_ecef", "ecef_to_geodetic", "ecef_to_enu", "enu_to_ecef",
    "geodetic_to_enu", "enu_to_geodetic",
    "spherical_to_cartesian", "cartesian_to_spherical", "haversine_distance",
    "SensorLocation", "unbiased_polar_to_cartesian_2d",
    "unbiased_spherical_to_cartesian_3d",
    "jacobian_spherical_to_cartesian_3d", "jacobian_cartesian_to_spherical_3d",
]
