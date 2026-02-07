NX-MIMOSA v6.0.1 Documentation
================================

**The radar tracker that sees through jamming.**

5,000 targets in 40 ms · ECM detection · Military ID · NATO output · C++ core + Python intelligence

.. code-block:: bash

   pip install nx-mimosa

.. code-block:: python

   from nx_mimosa import MultiTargetTracker
   tracker = MultiTargetTracker(dt=5.0, r_std=150.0, domain="air")
   tracks = tracker.process_scan(measurements)

   # For C++ speed (50× faster):
   from nx_mimosa.accel import MultiTargetTracker

761 aircraft tracked simultaneously from live ADS-B data. 99.8% detection rate.
C++ core processes 5,000 targets in 40 ms. Five military jets autonomously identified.

Key Capabilities
-----------------

**Filtering**: 6-model IMM (CV/CA/CT/Jerk/Ballistic/Orbital), EKF, UKF, Particle Filter,
GM-PHD, CPHD, LMB — covering both explicit association and random finite set approaches.

**Association**: GNN (speed), JPDA (dense clutter), MHT with N-scan pruning (crossing targets),
Sparse Auction (Bertsekas, C++ core for 2000+ targets).

**Intelligence**: Platform identification (111 types), military ICAO ID (30+ forces),
ECM detection (5 types), intent prediction (16 behaviours), threat assessment.

**Fusion**: 6 sensor types (radar, Doppler, EO/IR, ESM, ADS-B, secondary), track-to-track
association, covariance intersection, OOSM handling, online sensor bias estimation.

**C++ Core**: Eigen batch KF/IMM, header-only KDTree, sparse auction assignment, OpenMP.
50× faster than Python. Same API via ``nx_mimosa.accel``.

**Outputs**: Dual-mode (real-time display + fire control), ASTERIX Cat048, Link-16 J3.2,
GOSPA/OSPA/NEES/NIS/SIAP metrics, track quality scoring.

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
   api/accel
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
