NX-MIMOSA Documentation
========================

**Adaptive Multi-Sensor Multi-Target Radar Tracker**

*One line of Python. Any target. Any sensor. Any domain.*

.. code-block:: python

   from nx_mimosa import MultiTargetTracker

   tracker = MultiTargetTracker(dt=1.0, r_std=50.0, domain="military")
   tracks = tracker.process_scan(detections_3d)

NX-MIMOSA is a production-ready radar tracking library that replaces months of
filter engineering with a single line of code. It achieves **8.6× lower error**
than the nearest open-source alternative across 19 benchmark scenarios in 5
operational domains.

Key Features
------------

- **6-model IMM** — CV, CA, CT±, Jerk, Ballistic running in parallel
- **3D multi-target** — GNN, JPDA, and MHT data association
- **Multi-sensor fusion** — radar, EO/IR, ESM, Doppler, ADS-B
- **Platform identification** — 111 types from kinematics alone
- **ECM detection** — noise, deception, DRFM, chaff
- **Intent prediction** — 16 threat behaviors
- **Domain presets** — military, atc, automotive, space, maritime
- **Only dependency:** NumPy

.. toctree::
   :maxdepth: 2
   :caption: Getting Started

   quickstart
   installation

.. toctree::
   :maxdepth: 2
   :caption: User Guide

   guide/multi_target
   guide/sensors
   guide/coordinates
   guide/metrics

.. toctree::
   :maxdepth: 2
   :caption: API Reference

   api/mtt
   api/coords
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
