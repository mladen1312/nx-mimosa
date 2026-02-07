NX-MIMOSA v5.9.3 Documentation
================================

**Multi-target tracker with platform intelligence for defence radar systems.**

11,493 lines · 62 classes · 340 tests · 8 filters · 3 associators · 6-sensor fusion

.. code-block:: python

   from nx_mimosa import MultiTargetTracker
   tracker = MultiTargetTracker(dt=5.0, r_std=150.0, domain="air")
   tracks = tracker.process_scan(measurements)

761 aircraft tracked simultaneously from live ADS-B data. 99.8% detection rate.
519 ms mean scan processing. Five military jets autonomously identified.

Key Capabilities
-----------------

**Filtering**: 6-model IMM (CV/CA/CT/Jerk/Ballistic/Orbital), EKF, UKF, Particle Filter,
GM-PHD, CPHD, LMB — covering both explicit association and random finite set approaches.

**Association**: GNN (speed), JPDA (dense clutter), MHT with N-scan pruning (crossing targets).

**Intelligence**: Platform identification (111 types), military ICAO ID (30+ forces),
ECM detection (5 types), intent prediction (16 behaviours), threat assessment.

**Fusion**: 6 sensor types (radar, Doppler, EO/IR, ESM, ADS-B, secondary), track-to-track
association, covariance intersection, OOSM handling, online sensor bias estimation.

**Outputs**: Dual-mode (real-time display + fire control), ASTERIX Cat048, Link-16 J3.2,
GOSPA/OSPA/NEES/NIS/SIAP metrics, track quality scoring.

**Datasets**: nuScenes, CARLA, RADIATE, GenericCSV adapters plus synthetic scenario generator.

**Coordinates**: WGS-84, ECEF, ENU, polar, spherical — 27 conversion functions.

.. toctree::
   :maxdepth: 2
   :caption: Getting Started

   installation
   quickstart

.. toctree::
   :maxdepth: 2
   :caption: User Guide

   guide/multi_target
   guide/filters
   guide/association
   guide/intelligence
   guide/sensors
   guide/coordinates
   guide/metrics
   guide/rfs
   guide/datasets

.. toctree::
   :maxdepth: 2
   :caption: API Reference

   api/mtt
   api/intelligence
   api/fusion
   api/coords
   api/datasets
   api/metrics

.. toctree::
   :maxdepth: 1
   :caption: Project

   changelog
   license


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
