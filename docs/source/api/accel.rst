C++ Accelerated Tracker
========================

.. automodule:: nx_mimosa.accel
   :members:
   :undoc-members:
   :show-inheritance:

Performance
-----------

The C++ core achieves sub-50ms scan processing at 5,000+ simultaneous targets:

.. list-table::
   :header-rows: 1

   * - Targets
     - Python
     - C++
     - Speedup
   * - 100
     - 44 ms
     - 0.9 ms
     - 49×
   * - 761
     - 469 ms
     - 7 ms
     - 67×
   * - 1,000
     - 673 ms
     - 8 ms
     - 84×
   * - 2,000
     - —
     - 17 ms
     - —
   * - 5,000
     - —
     - 40 ms
     - —

Usage
-----

Drop-in replacement for the Python tracker:

.. code-block:: python

   from nx_mimosa.accel import MultiTargetTracker
   
   tracker = MultiTargetTracker(dt=5.0, r_std=150.0)
   tracks = tracker.process_scan(measurements)
   print(f"Scan time: {tracker.scan_time_ms:.1f} ms")
   
   # Bulk state extraction (most efficient)
   ids, states = tracker.get_states()  # (N,), (N, 6)

Build Requirements
------------------

- C++17 compiler (GCC 11+, Clang 14+)
- Eigen 3.3+ (``apt install libeigen3-dev``)
- pybind11 (``pip install pybind11``)
- OpenMP (optional, ~20% speedup)

.. code-block:: bash

   cd cpp && pip install .
