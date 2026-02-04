═══════════════════════════════════════════════════════════════════════════════
NX-MIMOSA Documentation
═══════════════════════════════════════════════════════════════════════════════

.. image:: https://img.shields.io/badge/version-1.0.0-blue.svg
   :target: https://github.com/mladen1312/nx-mimosa/releases/tag/v1.0.0
.. image:: https://img.shields.io/badge/License-AGPL%20v3-blue.svg
   :target: https://www.gnu.org/licenses/agpl-3.0
.. image:: https://img.shields.io/badge/EUROCONTROL-COMPLIANT-brightgreen.svg
.. image:: https://img.shields.io/badge/DO--178C-DAL--C-brightgreen.svg

**Production-Ready Multi-Domain Radar Tracking System**

NX-MIMOSA (Nexellum Multi-model IMM Optimal Smoothing Algorithm) is a certified-ready
tracking system delivering exceptional accuracy across six industry verticals.

Key Features
------------

- **EUROCONTROL EASSP Compliant** — 122m RMS en-route (req: ≤500m)
- **DO-178C DAL-C Ready** — Aviation software certification
- **ISO 26262 ASIL-D Ready** — Automotive functional safety
- **Multi-Domain Support** — Aviation, Automotive, Defense, Space, Maritime
- **FPGA Implementation** — RFSoC ZU48DR @ 250 MHz

Performance Summary
-------------------

.. list-table::
   :header-rows: 1
   :widths: 30 20 20 30

   * - Metric
     - Requirement
     - Achieved
     - Margin
   * - ATC En-route
     - ≤ 500 m
     - **122 m**
     - +309%
   * - ATC Terminal
     - ≤ 150 m
     - **47 m**
     - +219%
   * - Track Continuity
     - ≥ 99.5%
     - **100%**
     - ✓
   * - Latency
     - ≤ 100 ms
     - **45 ms**
     - +122%

Quick Start
-----------

Installation
^^^^^^^^^^^^

.. code-block:: bash

   pip install nx-mimosa

Basic Usage
^^^^^^^^^^^

.. code-block:: python

   from python.nx_mimosa_v41_atc import NXMIMOSAAtc

   # Create tracker
   tracker = NXMIMOSAAtc(dt=1.0, sigma=30.0)
   
   # Initialize
   tracker.initialize(z0=[50000, 30000, 10668], v0=[232, 0, 0])
   
   # Track
   for measurement in sensor_data:
       tracker.predict(dt=1.0)
       state = tracker.update(measurement)

Contents
--------

.. toctree::
   :maxdepth: 2
   :caption: User Guide

   INTEGRATION_GUIDE

.. toctree::
   :maxdepth: 2
   :caption: Certification

   DO178C_CERT_PACKAGE
   DO254_FPGA_CERTIFICATION
   ISO26262_ASIL_D_CERTIFICATION

.. toctree::
   :maxdepth: 2
   :caption: API Reference

   api/modules

Indices and Tables
------------------

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

Support
-------

- **Documentation**: https://nx-mimosa.readthedocs.io/
- **Issues**: https://github.com/mladen1312/nx-mimosa/issues
- **Email**: mladen@nexellum.com
- **Phone**: +385 99 737 5100

License
-------

Dual License:

- **Open Source**: `AGPL v3 <https://www.gnu.org/licenses/agpl-3.0>`_
- **Commercial**: Contact licensing@nexellum.com

---

*© 2024-2026 Nexellum d.o.o. All Rights Reserved.*
