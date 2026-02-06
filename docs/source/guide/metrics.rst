Performance Metrics
===================

NX-MIMOSA provides standard tracking quality metrics.

NEES (Normalized Estimation Error Squared)
------------------------------------------

Tests filter consistency. For a consistent filter, E[NEES] equals the state dimension.

.. code-block:: python

   from nx_mimosa import compute_nees
   nees = compute_nees(truth_state, estimated_state, covariance)

NIS (Normalized Innovation Squared)
------------------------------------

Tests measurement consistency. E[NIS] should equal measurement dimension.

.. code-block:: python

   from nx_mimosa import compute_nis
   nis = compute_nis(innovation, innovation_covariance)

OSPA (Optimal Sub-Pattern Assignment)
--------------------------------------

Single-number multi-target quality combining position error and cardinality penalty.

.. code-block:: python

   from nx_mimosa import compute_ospa
   ospa = compute_ospa(tracks, truths, c=1000, p=2)

NATO SIAP Metrics
-----------------

Standard metrics from STANAG 4162:

.. code-block:: python

   from nx_mimosa import compute_siap_metrics
   siap = compute_siap_metrics(track_positions, truth_positions)
   # Returns: completeness, spuriousness, ambiguity, accuracy
