Coordinate Transforms
=====================

NX-MIMOSA provides complete geodetic coordinate conversion.

Supported Systems
-----------------

- **WGS-84 (geodetic):** latitude, longitude, altitude
- **ECEF:** Earth-Centered Earth-Fixed Cartesian
- **ENU:** East-North-Up local tangent plane
- **Spherical:** range, azimuth, elevation

All conversions are bidirectional with sub-millimeter accuracy.

Common Conversions
------------------

.. code-block:: python

   from nx_mimosa import geodetic_to_ecef, ecef_to_enu, SensorLocation

   # GPS → ECEF
   ecef = geodetic_to_ecef(lat_deg=45.815, lon_deg=15.982, alt_m=150.0)

   # ECEF → ENU relative to a reference point
   enu = ecef_to_enu(target_ecef, ref_lat_deg=45.815, ref_lon_deg=15.982, ref_alt_m=150.0)

SensorLocation Class
---------------------

For radar applications, the :class:`~python.nx_mimosa_coords.SensorLocation` class
encapsulates sensor position and provides direct conversions:

.. code-block:: python

   sensor = SensorLocation(lat_deg=45.815, lon_deg=15.982, alt_m=150.0)
   enu = sensor.radar_to_enu(range_m=50000, az_rad=0.3, el_rad=0.05)

Unbiased Conversion
--------------------

Naive polar-to-Cartesian conversion introduces systematic bias at long range
with large angular uncertainty. NX-MIMOSA implements the Bar-Shalom (2001)
unbiased conversion:

.. code-block:: python

   from nx_mimosa import unbiased_polar_to_cartesian_2d

   xy, cov = unbiased_polar_to_cartesian_2d(
       r=50000, theta=0.3, sigma_r=100, sigma_theta=0.02
   )
