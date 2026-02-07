Core Tracker (``nx_mimosa_mtt``)
================================

.. automodule:: python.nx_mimosa_mtt
   :members:
   :show-inheritance:
   :member-order: bysource

Filters
-------

.. autoclass:: python.nx_mimosa_mtt.KalmanFilter3D
   :members:

.. autoclass:: python.nx_mimosa_mtt.EKF3D
   :members:

.. autoclass:: python.nx_mimosa_mtt.UKF3D
   :members:

.. autoclass:: python.nx_mimosa_mtt.IMM3D
   :members:

.. autoclass:: python.nx_mimosa_mtt.ParticleFilter3D
   :members:

RFS Filters
-----------

.. autoclass:: python.nx_mimosa_mtt.GMPHD
   :members:

.. autoclass:: python.nx_mimosa_mtt.CPHD
   :members:

.. autoclass:: python.nx_mimosa_mtt.LMB
   :members:

Track Management
----------------

.. autoclass:: python.nx_mimosa_mtt.MultiTargetTracker
   :members:

.. autoclass:: python.nx_mimosa_mtt.TrackManager
   :members:

.. autoclass:: python.nx_mimosa_mtt.TrackState
   :members:

.. autoclass:: python.nx_mimosa_mtt.TrackManagerConfig
   :members:

Association
-----------

.. autofunction:: python.nx_mimosa_mtt.gnn_associate
.. autofunction:: python.nx_mimosa_mtt.jpda_associate
.. autofunction:: python.nx_mimosa_mtt.mht_associate

Metrics
-------

.. autofunction:: python.nx_mimosa_mtt.compute_nees
.. autofunction:: python.nx_mimosa_mtt.compute_nis
.. autofunction:: python.nx_mimosa_mtt.compute_ospa
.. autofunction:: python.nx_mimosa_mtt.compute_gospa
.. autofunction:: python.nx_mimosa_mtt.compute_siap_metrics
