Metrics Reference
=================

NX-MIMOSA provides five metric families:

NEES — Normalised Estimation Error Squared
-------------------------------------------
Filter consistency check. Values near χ² expected value indicate well-tuned filters.

.. autofunction:: python.nx_mimosa_mtt.compute_nees

NIS — Normalised Innovation Squared
------------------------------------
Measurement consistency. Drives automatic ECM detection.

.. autofunction:: python.nx_mimosa_mtt.compute_nis

OSPA — Optimal Sub-Pattern Assignment
--------------------------------------
Combined localisation and cardinality error (Schuhmacher et al., 2008).

.. autofunction:: python.nx_mimosa_mtt.compute_ospa

GOSPA — Generalised OSPA
--------------------------
Decomposed into localisation, missed targets, and false tracks (García-Fernández et al., 2020).

.. autofunction:: python.nx_mimosa_mtt.compute_gospa

SIAP — Single Integrated Air Picture
--------------------------------------
Completeness, ambiguity, spuriousness, timeliness.

.. autofunction:: python.nx_mimosa_mtt.compute_siap_metrics
