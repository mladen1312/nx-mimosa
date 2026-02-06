Multi-Target Tracking
=====================

NX-MIMOSA provides three data association algorithms for multi-target tracking.

GNN (Global Nearest Neighbor)
-----------------------------

Optimal one-to-one assignment via the Hungarian algorithm. Best for
well-separated targets with low clutter.

.. code-block:: python

   tracker = MultiTargetTracker(dt=1.0, r_std=50.0, association="gnn")

**Complexity:** O(N³) where N = max(tracks, measurements)

JPDA (Joint Probabilistic Data Association)
-------------------------------------------

Computes probabilistic weights for all measurement-to-track pairings.
Each track update uses a weighted average of gated measurements. Robust
in clutter and when targets cross.

.. code-block:: python

   tracker = MultiTargetTracker(dt=1.0, r_std=50.0, association="jpda")

NX-MIMOSA uses Mahalanobis-based relative likelihood (not absolute Gaussian),
making it robust across all covariance scales — from newly initiated tracks
with large uncertainty to well-established tracks with tight covariance.

MHT (Multi-Hypothesis Tracking)
--------------------------------

Maintains multiple global hypotheses for measurement-to-track assignment,
scores each by log-likelihood, and selects the best. Defers ambiguous
decisions until more data arrives.

.. code-block:: python

   tracker = MultiTargetTracker(dt=1.0, r_std=50.0, association="mht")

MHT is the most computationally expensive but provides the best performance
in dense, ambiguous scenarios.

**Reference:** Reid, "An algorithm for tracking multiple targets," IEEE TAC, 1979

Track Management
----------------

All association methods use M-of-N track lifecycle management:

- **TENTATIVE:** New detection, waiting for confirmation
- **CONFIRMED:** M hits in last N scans → reported to user
- **COASTING:** Missed recent scans, predicting forward
- **DELETED:** Too many misses, removed

Configure via :class:`~python.nx_mimosa_mtt.TrackManagerConfig`:

.. code-block:: python

   from nx_mimosa import TrackManagerConfig, MultiTargetTracker
   
   config = TrackManagerConfig(
       confirm_m=3,          # Need 3 hits...
       confirm_n=5,          # ...in 5 scans to confirm
       delete_misses=5,      # Delete after 5 consecutive misses
       coast_max=3,          # Coast for 3 scans before starting deletion
       min_separation=100.0, # Don't init tracks within 100m of each other
       gate_threshold=16.0,  # Chi-squared gate (99.7% for 3D)
   )
   tracker = MultiTargetTracker(dt=1.0, r_std=50.0, config=config)
