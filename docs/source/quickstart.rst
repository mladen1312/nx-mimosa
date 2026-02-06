Quick Start
===========

5 Minutes to Your First Track
------------------------------

Install NX-MIMOSA:

.. code-block:: bash

   pip install nx-mimosa

Track Multiple Targets
~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   import numpy as np
   from nx_mimosa import MultiTargetTracker

   # Create tracker — domain preset auto-configures everything
   tracker = MultiTargetTracker(dt=1.0, r_std=50.0, domain="military")

   # Simulate 3D radar detections
   for t in range(100):
       # Each scan: Nx3 array of [x, y, z] detections
       detections = np.array([
           [100*t + np.random.randn()*30, 200 + np.random.randn()*30, 5000 + np.random.randn()*30],
           [50*t + np.random.randn()*30, -100*t + np.random.randn()*30, 8000 + np.random.randn()*30],
       ])
       
       confirmed = tracker.process_scan(detections)
       
       for track in confirmed:
           pos = track.filter.position
           vel = track.filter.velocity
           print(f"Track {track.track_id}: pos={pos.astype(int)} vel={vel.astype(int)}")

Choose Your Association Method
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   # GNN — fastest, optimal for well-separated targets
   tracker = MultiTargetTracker(dt=1.0, r_std=50.0, association="gnn")
   
   # JPDA — robust in clutter and crossing targets
   tracker = MultiTargetTracker(dt=1.0, r_std=50.0, association="jpda")
   
   # MHT — best for dense, ambiguous environments
   tracker = MultiTargetTracker(dt=1.0, r_std=50.0, association="mht")

Domain Presets
~~~~~~~~~~~~~~

Each preset auto-configures motion models, process noise, track management,
gating thresholds, and all other parameters:

.. code-block:: python

   MultiTargetTracker(dt=1.0,  r_std=50.0,  domain="military")   # 9g fighters, SAMs
   MultiTargetTracker(dt=4.0,  r_std=100.0, domain="atc")        # Air traffic
   MultiTargetTracker(dt=0.05, r_std=0.5,   domain="automotive")  # Highway ADAS
   MultiTargetTracker(dt=10.0, r_std=200.0, domain="space")      # LEO/GEO/reentry
   MultiTargetTracker(dt=2.0,  r_std=50.0,  domain="maritime")   # Surface vessels

Coordinate Transforms
~~~~~~~~~~~~~~~~~~~~~

Convert between any coordinate system:

.. code-block:: python

   from nx_mimosa import geodetic_to_ecef, ecef_to_enu, SensorLocation

   # Radar at known GPS position
   sensor = SensorLocation(lat_deg=45.815, lon_deg=15.982, alt_m=150.0)
   
   # Convert radar spherical (r, az, el) → ENU local tangent plane
   enu_pos = sensor.radar_to_enu(range_m=50000, az_rad=0.3, el_rad=0.05)

Quality Metrics
~~~~~~~~~~~~~~~

Verify your tracker is working correctly:

.. code-block:: python

   from nx_mimosa import compute_nees, compute_ospa, compute_siap_metrics

   # Filter consistency
   nees = compute_nees(truth, estimate, covariance)
   
   # Multi-target quality (single number)
   ospa = compute_ospa(track_set, truth_set, c=1000, p=2)
   
   # NATO SIAP metrics
   siap = compute_siap_metrics(track_positions, truth_positions)
