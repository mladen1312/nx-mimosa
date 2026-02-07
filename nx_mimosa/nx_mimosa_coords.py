"""
NX-MIMOSA Coordinate Transform Library
========================================
[REQ-V50-CT] Full coordinate system support for real-world radar integration.

Supported frames:
  - WGS-84 Geodetic (lat, lon, alt)
  - ECEF (Earth-Centered Earth-Fixed)
  - ENU (East-North-Up) local tangent plane
  - Spherical/Radar (range, azimuth, elevation)
  - Cartesian 2D/3D

References:
  - Bowring (1976) — Geodetic to ECEF
  - Bar-Shalom, Li, Kirubarajan (2001) — Unbiased polar-to-Cartesian
  - Farrell (2008) — Aided Navigation: GPS with High Rate Sensors

Author: Dr. Mladen Mešter, Nexellum d.o.o.
"""

import numpy as np
from typing import Tuple, Optional
from dataclasses import dataclass

# ===== WGS-84 CONSTANTS =====
WGS84_A = 6378137.0              # Semi-major axis [m]
WGS84_F = 1.0 / 298.257223563   # Flattening
WGS84_B = WGS84_A * (1 - WGS84_F)  # Semi-minor axis
WGS84_E2 = 2 * WGS84_F - WGS84_F**2  # First eccentricity squared
WGS84_EP2 = WGS84_E2 / (1 - WGS84_E2)  # Second eccentricity squared


# ===== GEODETIC ↔ ECEF =====

def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> np.ndarray:
    """[REQ-V50-CT-01] WGS-84 geodetic to ECEF.
    
    Args:
        lat_deg: Latitude in degrees [-90, 90]
        lon_deg: Longitude in degrees [-180, 180]
        alt_m: Altitude above ellipsoid in meters
    
    Returns:
        np.ndarray: [x, y, z] ECEF in meters
    """
    lat = np.radians(lat_deg)
    lon = np.radians(lon_deg)
    
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    sin_lon = np.sin(lon)
    cos_lon = np.cos(lon)
    
    N = WGS84_A / np.sqrt(1 - WGS84_E2 * sin_lat**2)
    
    x = (N + alt_m) * cos_lat * cos_lon
    y = (N + alt_m) * cos_lat * sin_lon
    z = (N * (1 - WGS84_E2) + alt_m) * sin_lat
    
    return np.array([x, y, z])


def ecef_to_geodetic(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """[REQ-V50-CT-02] ECEF to WGS-84 geodetic (Bowring iterative).
    
    Returns:
        Tuple: (lat_deg, lon_deg, alt_m)
    """
    lon = np.arctan2(y, x)
    
    p = np.sqrt(x**2 + y**2)
    
    # Bowring's method (converges in 2-3 iterations)
    theta = np.arctan2(z * WGS84_A, p * WGS84_B)
    lat = np.arctan2(
        z + WGS84_EP2 * WGS84_B * np.sin(theta)**3,
        p - WGS84_E2 * WGS84_A * np.cos(theta)**3
    )
    
    # Iterate once for sub-mm accuracy
    for _ in range(2):
        sin_lat = np.sin(lat)
        N = WGS84_A / np.sqrt(1 - WGS84_E2 * sin_lat**2)
        lat = np.arctan2(z + WGS84_E2 * N * sin_lat, p)
    
    sin_lat = np.sin(lat)
    N = WGS84_A / np.sqrt(1 - WGS84_E2 * sin_lat**2)
    
    if np.abs(np.cos(lat)) > 1e-10:
        alt = p / np.cos(lat) - N
    else:
        alt = np.abs(z) - WGS84_B
    
    return np.degrees(lat), np.degrees(lon), alt


# ===== ECEF ↔ ENU =====

def ecef_to_enu(target_ecef: np.ndarray, ref_lat_deg: float, ref_lon_deg: float,
                ref_alt_m: float) -> np.ndarray:
    """[REQ-V50-CT-03] ECEF to ENU (local tangent plane).
    
    Args:
        target_ecef: [x, y, z] ECEF position of target
        ref_lat_deg, ref_lon_deg, ref_alt_m: Reference point (sensor location)
    
    Returns:
        np.ndarray: [east, north, up] in meters
    """
    ref_ecef = geodetic_to_ecef(ref_lat_deg, ref_lon_deg, ref_alt_m)
    diff = target_ecef - ref_ecef
    
    lat = np.radians(ref_lat_deg)
    lon = np.radians(ref_lon_deg)
    
    R = _ecef_to_enu_rotation(lat, lon)
    return R @ diff


def enu_to_ecef(enu: np.ndarray, ref_lat_deg: float, ref_lon_deg: float,
                ref_alt_m: float) -> np.ndarray:
    """[REQ-V50-CT-04] ENU to ECEF."""
    ref_ecef = geodetic_to_ecef(ref_lat_deg, ref_lon_deg, ref_alt_m)
    
    lat = np.radians(ref_lat_deg)
    lon = np.radians(ref_lon_deg)
    
    R = _ecef_to_enu_rotation(lat, lon)
    return R.T @ enu + ref_ecef


def _ecef_to_enu_rotation(lat_rad: float, lon_rad: float) -> np.ndarray:
    """Rotation matrix from ECEF to ENU frame."""
    sin_lat = np.sin(lat_rad)
    cos_lat = np.cos(lat_rad)
    sin_lon = np.sin(lon_rad)
    cos_lon = np.cos(lon_rad)
    
    return np.array([
        [-sin_lon,             cos_lon,             0       ],
        [-sin_lat * cos_lon,  -sin_lat * sin_lon,   cos_lat ],
        [ cos_lat * cos_lon,   cos_lat * sin_lon,   sin_lat ]
    ])


# ===== GEODETIC ↔ ENU (convenience) =====

def geodetic_to_enu(target_lat: float, target_lon: float, target_alt: float,
                    ref_lat: float, ref_lon: float, ref_alt: float) -> np.ndarray:
    """[REQ-V50-CT-05] Geodetic to ENU in one call."""
    target_ecef = geodetic_to_ecef(target_lat, target_lon, target_alt)
    return ecef_to_enu(target_ecef, ref_lat, ref_lon, ref_alt)


def enu_to_geodetic(enu: np.ndarray, ref_lat: float, ref_lon: float,
                    ref_alt: float) -> Tuple[float, float, float]:
    """[REQ-V50-CT-06] ENU to Geodetic in one call."""
    ecef = enu_to_ecef(enu, ref_lat, ref_lon, ref_alt)
    return ecef_to_geodetic(ecef[0], ecef[1], ecef[2])


# ===== SPHERICAL (RADAR) ↔ CARTESIAN =====

def spherical_to_cartesian(r: float, az_rad: float, el_rad: float) -> np.ndarray:
    """[REQ-V50-CT-07] Radar spherical to local Cartesian (ENU convention).
    
    Convention:
        az = 0 → North (positive y), clockwise positive
        el = 0 → horizontal, positive up
    
    Args:
        r: Range [m]
        az_rad: Azimuth [rad] from North, clockwise
        el_rad: Elevation [rad] from horizontal
    
    Returns:
        np.ndarray: [east, north, up]
    """
    cos_el = np.cos(el_rad)
    east = r * cos_el * np.sin(az_rad)
    north = r * cos_el * np.cos(az_rad)
    up = r * np.sin(el_rad)
    return np.array([east, north, up])


def cartesian_to_spherical(east: float, north: float, up: float) -> Tuple[float, float, float]:
    """[REQ-V50-CT-08] Local Cartesian to radar spherical.
    
    Returns:
        Tuple: (range_m, azimuth_rad, elevation_rad)
    """
    r = np.sqrt(east**2 + north**2 + up**2)
    az = np.arctan2(east, north)  # Note: atan2(E,N) for North-referenced
    if az < 0:
        az += 2 * np.pi
    el = np.arctan2(up, np.sqrt(east**2 + north**2))
    return r, az, el


# ===== UNBIASED POLAR-TO-CARTESIAN =====

def unbiased_polar_to_cartesian_2d(r: float, theta: float,
                                    sigma_r: float, sigma_theta: float) -> Tuple[np.ndarray, np.ndarray]:
    """[REQ-V50-CT-09] Unbiased polar-to-Cartesian conversion (Bar-Shalom 2001).
    
    Standard polar-to-Cartesian is biased because E[cos(θ+ε)] ≠ cos(θ).
    This uses the debiased conversion from:
        Bar-Shalom, Li, Kirubarajan, "Estimation with Applications to 
        Tracking and Navigation", eq. 4.7.2-3

    Args:
        r: Measured range
        theta: Measured angle [rad]
        sigma_r: Range std dev
        sigma_theta: Angle std dev [rad]
    
    Returns:
        Tuple: (position [x,y], covariance [2×2])
    """
    # Debiasing factor
    e_cos = np.exp(-sigma_theta**2 / 2)
    e_sin = np.exp(-sigma_theta**2 / 2)
    e_cos2 = np.exp(-2 * sigma_theta**2)
    
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    
    # Debiased position
    r_eff = r  # Range is unbiased
    x = r_eff * cos_t * e_cos
    y = r_eff * sin_t * e_sin
    
    # Covariance (Bar-Shalom eq. 4.7.2-9)
    r2 = r**2 + sigma_r**2
    
    Pxx = r2 * (1 + e_cos2 * np.cos(2*theta)) / 2 - (r * e_cos * cos_t)**2
    Pyy = r2 * (1 - e_cos2 * np.cos(2*theta)) / 2 - (r * e_cos * sin_t)**2
    Pxy = r2 * e_cos2 * np.sin(2*theta) / 2 - r**2 * e_cos**2 * cos_t * sin_t
    
    pos = np.array([x, y])
    cov = np.array([[Pxx, Pxy], [Pxy, Pyy]])
    
    return pos, cov


def unbiased_spherical_to_cartesian_3d(r: float, az: float, el: float,
                                        sigma_r: float, sigma_az: float,
                                        sigma_el: float) -> Tuple[np.ndarray, np.ndarray]:
    """[REQ-V50-CT-10] Unbiased spherical-to-Cartesian 3D conversion.
    
    Extension of Bar-Shalom debiased conversion to 3D.
    
    Returns:
        Tuple: (position [e,n,u], covariance [3×3])
    """
    e_az = np.exp(-sigma_az**2 / 2)
    e_el = np.exp(-sigma_el**2 / 2)
    e_2az = np.exp(-2 * sigma_az**2)
    e_2el = np.exp(-2 * sigma_el**2)
    
    cos_az = np.cos(az)
    sin_az = np.sin(az)
    cos_el = np.cos(el)
    sin_el = np.sin(el)
    
    # Debiased position
    east = r * sin_az * e_az * cos_el * e_el
    north = r * cos_az * e_az * cos_el * e_el
    up = r * sin_el * e_el
    
    pos = np.array([east, north, up])
    
    # Covariance (diagonal approximation for speed, adequate for σ < 10°)
    r2 = r**2 + sigma_r**2
    
    ce2 = (1 + e_2el * np.cos(2*el)) / 2  # E[cos²(el)]
    se2 = (1 - e_2el * np.cos(2*el)) / 2  # E[sin²(el)]
    ca2 = (1 + e_2az * np.cos(2*az)) / 2  # E[cos²(az)]
    sa2 = (1 - e_2az * np.cos(2*az)) / 2  # E[sin²(az)]
    
    Pee = r2 * sa2 * ce2 - east**2
    Pnn = r2 * ca2 * ce2 - north**2
    Puu = r2 * se2 - up**2
    
    # Ensure positive diagonal
    Pee = max(Pee, sigma_r**2 * 0.01)
    Pnn = max(Pnn, sigma_r**2 * 0.01)
    Puu = max(Puu, sigma_r**2 * 0.01)
    
    cov = np.diag([Pee, Pnn, Puu])
    
    return pos, cov


# ===== RANGE-RATE (DOPPLER) =====

def range_rate_to_velocity_component(r_dot: float, az: float, el: float,
                                      r: float) -> np.ndarray:
    """[REQ-V50-CT-11] Convert range-rate to velocity LOS unit vector.
    
    v_radial = v · r_hat where r_hat is the unit vector from sensor to target.
    
    Returns:
        np.ndarray: LOS unit vector [e, n, u] (multiply by r_dot for velocity component)
    """
    cos_el = np.cos(el)
    los = np.array([
        cos_el * np.sin(az),
        cos_el * np.cos(az),
        np.sin(el)
    ])
    return los


# ===== JACOBIANS FOR EKF =====

def jacobian_spherical_to_cartesian_3d(east: float, north: float, up: float) -> np.ndarray:
    """[REQ-V50-CT-12] Jacobian ∂(r,az,el)/∂(e,n,u) for EKF measurement update.
    
    For state x=[e,n,u,...] and measurement z=[r,az,el]:
    H = ∂h/∂x where h(x) = [r(x), az(x), el(x)]
    
    Returns:
        np.ndarray: 3×3 Jacobian
    """
    r_horiz = np.sqrt(east**2 + north**2)
    r = np.sqrt(east**2 + north**2 + up**2)
    
    if r < 1e-10:
        return np.eye(3)  # Degenerate
    
    r2 = r**2
    rh2 = r_horiz**2
    
    # ∂r/∂(e,n,u)
    dr = np.array([east/r, north/r, up/r])
    
    # ∂az/∂(e,n,u)  — az = atan2(e, n)
    if rh2 < 1e-10:
        daz = np.array([0.0, 0.0, 0.0])
    else:
        daz = np.array([north/rh2, -east/rh2, 0.0])
    
    # ∂el/∂(e,n,u)  — el = atan2(u, r_horiz)
    del_ = np.array([
        -east * up / (r2 * r_horiz) if r_horiz > 1e-10 else 0.0,
        -north * up / (r2 * r_horiz) if r_horiz > 1e-10 else 0.0,
        r_horiz / r2
    ])
    
    return np.array([dr, daz, del_])


def jacobian_cartesian_to_spherical_3d(r: float, az: float, el: float) -> np.ndarray:
    """[REQ-V50-CT-13] Jacobian ∂(e,n,u)/∂(r,az,el) — inverse direction.
    
    Returns:
        np.ndarray: 3×3 Jacobian
    """
    cos_el = np.cos(el)
    sin_el = np.sin(el)
    cos_az = np.cos(az)
    sin_az = np.sin(az)
    
    return np.array([
        [cos_el * sin_az,   r * cos_el * cos_az,  -r * sin_el * sin_az],
        [cos_el * cos_az,  -r * cos_el * sin_az,  -r * sin_el * cos_az],
        [sin_el,            0.0,                    r * cos_el          ]
    ])


# ===== GREAT CIRCLE DISTANCE =====

def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """[REQ-V50-CT-14] Great circle distance between two geodetic points.
    
    Args:
        lat1, lon1, lat2, lon2: Degrees
    
    Returns:
        Distance in meters
    """
    lat1, lon1, lat2, lon2 = map(np.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = np.sin(dlat/2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon/2)**2
    return 2 * WGS84_A * np.arcsin(np.sqrt(a))


# ===== SENSOR FRAME TRANSFORMS =====

@dataclass
class SensorLocation:
    """Full sensor location specification for coordinate transforms."""
    lat_deg: float = 0.0
    lon_deg: float = 0.0
    alt_m: float = 0.0
    # Sensor mounting offsets (body frame → ENU rotation)
    heading_deg: float = 0.0   # Sensor boresight heading from North
    pitch_deg: float = 0.0     # Sensor pitch (elevation tilt)
    roll_deg: float = 0.0      # Sensor roll
    
    def get_ecef(self) -> np.ndarray:
        return geodetic_to_ecef(self.lat_deg, self.lon_deg, self.alt_m)
    
    def radar_to_enu(self, r: float, az_rad: float, el_rad: float) -> np.ndarray:
        """Convert radar measurement (sensor frame) to ENU (local frame).
        
        Applies sensor mounting rotation if heading/pitch/roll != 0.
        """
        # Radar spherical → sensor-frame Cartesian
        local = spherical_to_cartesian(r, az_rad, el_rad)
        
        # Apply sensor mounting rotation (heading only for now — most common case)
        if abs(self.heading_deg) > 1e-6:
            h = np.radians(self.heading_deg)
            cos_h = np.cos(h)
            sin_h = np.sin(h)
            # Rotate around Up axis by heading
            e, n, u = local
            local = np.array([
                e * cos_h + n * sin_h,
               -e * sin_h + n * cos_h,
                u
            ])
        
        return local
    
    def radar_to_geodetic(self, r: float, az_rad: float, el_rad: float
                          ) -> Tuple[float, float, float]:
        """Convert radar measurement directly to geodetic coordinates."""
        enu = self.radar_to_enu(r, az_rad, el_rad)
        return enu_to_geodetic(enu, self.lat_deg, self.lon_deg, self.alt_m)


# ===== BATCH CONVERSIONS =====

def measurements_spherical_to_enu(measurements: np.ndarray,
                                   sensor: SensorLocation) -> np.ndarray:
    """[REQ-V50-CT-15] Batch convert radar measurements to ENU.
    
    Args:
        measurements: Nx3 array of [range, az_rad, el_rad] per row
        sensor: Sensor location
    
    Returns:
        Nx3 array of [east, north, up]
    """
    result = np.zeros_like(measurements)
    for i in range(len(measurements)):
        result[i] = sensor.radar_to_enu(
            measurements[i, 0], measurements[i, 1], measurements[i, 2]
        )
    return result


# ===== FULL COORDINATE CHAINS =====

def spherical_to_geodetic(r: float, az_rad: float, el_rad: float,
                          sensor: 'SensorLocation') -> Tuple[float, float, float]:
    """[REQ-V55-CT-20] Full chain: Spherical (radar) → Geodetic (lat/lon/alt).
    
    Converts a radar measurement (range, azimuth, elevation) taken from
    a known sensor location into WGS-84 geodetic coordinates.
    
    Chain: Spherical → ENU → ECEF → Geodetic
    
    Args:
        r: Range in meters
        az_rad: Azimuth in radians (CW from north)
        el_rad: Elevation in radians (above horizon)
        sensor: SensorLocation with known geodetic position
    
    Returns:
        Tuple of (latitude_deg, longitude_deg, altitude_m)
    """
    enu = sensor.radar_to_enu(r, az_rad, el_rad)
    ecef = enu_to_ecef(enu, sensor.lat_deg, sensor.lon_deg, sensor.alt_m)
    return ecef_to_geodetic(ecef[0], ecef[1], ecef[2])


def geodetic_to_spherical(target_lat: float, target_lon: float, target_alt: float,
                          sensor: 'SensorLocation') -> Tuple[float, float, float]:
    """[REQ-V55-CT-21] Full chain: Geodetic → Spherical (radar coordinates).
    
    Computes what a radar at `sensor` would measure for a target at the given
    geodetic position: (range, azimuth, elevation).
    
    Chain: Geodetic → ENU → Spherical
    
    Args:
        target_lat: Target latitude in degrees
        target_lon: Target longitude in degrees
        target_alt: Target altitude in meters
        sensor: SensorLocation
    
    Returns:
        Tuple of (range_m, azimuth_rad, elevation_rad)
    """
    enu = geodetic_to_enu(target_lat, target_lon, target_alt,
                          sensor.lat_deg, sensor.lon_deg, sensor.alt_m)
    return cartesian_to_spherical(enu[0], enu[1], enu[2])


def ecef_to_spherical(target_ecef: np.ndarray,
                      sensor: 'SensorLocation') -> Tuple[float, float, float]:
    """[REQ-V55-CT-22] ECEF → Spherical via ENU intermediate.
    
    Args:
        target_ecef: [X, Y, Z] in ECEF meters
        sensor: SensorLocation
    
    Returns:
        Tuple of (range_m, azimuth_rad, elevation_rad)
    """
    enu = ecef_to_enu(target_ecef, sensor.lat_deg, sensor.lon_deg, sensor.alt_m)
    return cartesian_to_spherical(enu[0], enu[1], enu[2])


def spherical_to_ecef(r: float, az_rad: float, el_rad: float,
                      sensor: 'SensorLocation') -> np.ndarray:
    """[REQ-V55-CT-23] Spherical → ECEF via ENU intermediate.
    
    Args:
        r: Range in meters
        az_rad: Azimuth in radians
        el_rad: Elevation in radians
        sensor: SensorLocation
    
    Returns:
        np.ndarray [X, Y, Z] in ECEF meters
    """
    enu = sensor.radar_to_enu(r, az_rad, el_rad)
    return enu_to_ecef(enu, sensor.lat_deg, sensor.lon_deg, sensor.alt_m)


# ===== COVARIANCE TRANSFORMS =====

def covariance_spherical_to_cartesian(r: float, az: float, el: float,
                                       sigma_r: float, sigma_az: float,
                                       sigma_el: float) -> np.ndarray:
    """[REQ-V55-CT-24] Transform measurement covariance from spherical to Cartesian.
    
    Uses the Jacobian of the spherical→Cartesian transform:
        P_cart = J @ P_sph @ J.T
    
    where P_sph = diag(σ_r², σ_az², σ_el²)
    
    This is the standard first-order (EKF-style) covariance conversion.
    For unbiased conversion, use `unbiased_spherical_to_cartesian_3d`.
    
    Args:
        r: Range (meters)
        az: Azimuth (radians, CW from north)
        el: Elevation (radians)
        sigma_r: Range standard deviation (meters)
        sigma_az: Azimuth standard deviation (radians)
        sigma_el: Elevation standard deviation (radians)
    
    Returns:
        3×3 Cartesian covariance matrix [east, north, up]
    """
    # Compute ENU position for Jacobian
    enu = spherical_to_cartesian(r, az, el)
    J = jacobian_spherical_to_cartesian_3d(enu[0], enu[1], enu[2])
    
    R_sph = np.diag([sigma_r**2, sigma_az**2, sigma_el**2])
    return J @ R_sph @ J.T


def covariance_cartesian_to_spherical(east: float, north: float, up: float,
                                       P_cart: np.ndarray) -> np.ndarray:
    """[REQ-V55-CT-25] Transform covariance from Cartesian (ENU) to spherical.
    
    P_sph = J_inv @ P_cart @ J_inv.T
    
    Args:
        east, north, up: Position in ENU meters
        P_cart: 3×3 Cartesian covariance
    
    Returns:
        3×3 Spherical covariance [σ_r², σ_az², σ_el²] (diagonal-dominant)
    """
    r, az, el = cartesian_to_spherical(east, north, up)
    J_inv = jacobian_cartesian_to_spherical_3d(r, az, el)
    return J_inv @ P_cart @ J_inv.T


# ===== GREAT CIRCLE UTILITIES =====

def bearing_between(lat1: float, lon1: float,
                    lat2: float, lon2: float) -> float:
    """[REQ-V55-CT-26] Initial bearing (azimuth) from point 1 to point 2.
    
    Uses the forward azimuth formula on the WGS-84 ellipsoid (spherical approx).
    
    Args:
        lat1, lon1: Start point (degrees)
        lat2, lon2: End point (degrees)
    
    Returns:
        Bearing in radians [0, 2π) clockwise from north
    """
    lat1_r = np.radians(lat1)
    lat2_r = np.radians(lat2)
    dlon = np.radians(lon2 - lon1)
    
    x = np.sin(dlon) * np.cos(lat2_r)
    y = np.cos(lat1_r) * np.sin(lat2_r) - np.sin(lat1_r) * np.cos(lat2_r) * np.cos(dlon)
    
    return np.arctan2(x, y) % (2 * np.pi)


def destination_point(lat: float, lon: float, bearing_rad: float,
                      distance_m: float) -> Tuple[float, float]:
    """[REQ-V55-CT-27] Destination point given start, bearing, and distance.
    
    Vincenty-style direct problem (spherical approximation).
    
    Args:
        lat, lon: Start point (degrees)
        bearing_rad: Bearing in radians CW from north
        distance_m: Distance in meters
    
    Returns:
        Tuple of (dest_lat_deg, dest_lon_deg)
    """
    R = 6371000.0  # Earth radius (meters)
    
    lat_r = np.radians(lat)
    lon_r = np.radians(lon)
    d_over_R = distance_m / R
    
    lat2 = np.arcsin(
        np.sin(lat_r) * np.cos(d_over_R) +
        np.cos(lat_r) * np.sin(d_over_R) * np.cos(bearing_rad)
    )
    lon2 = lon_r + np.arctan2(
        np.sin(bearing_rad) * np.sin(d_over_R) * np.cos(lat_r),
        np.cos(d_over_R) - np.sin(lat_r) * np.sin(lat2)
    )
    
    return np.degrees(lat2), np.degrees(lon2)
