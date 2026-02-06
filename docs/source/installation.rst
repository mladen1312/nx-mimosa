Installation
============

Requirements
------------

- Python 3.9+
- NumPy >= 1.21

That's it. NX-MIMOSA has no other dependencies.

From PyPI (Recommended)
-----------------------

.. code-block:: bash

   pip install nx-mimosa

From Source
-----------

.. code-block:: bash

   git clone https://github.com/mladen1312/nx-mimosa.git
   cd nx-mimosa
   pip install -e .

With Development Dependencies
-----------------------------

.. code-block:: bash

   pip install nx-mimosa[dev]

This installs pytest, Sphinx, and type checking tools.

With Benchmark Dependencies
----------------------------

.. code-block:: bash

   pip install nx-mimosa[benchmark]

This installs Stone Soup, FilterPy, and PyKalman for running comparative benchmarks.

Verify Installation
-------------------

.. code-block:: python

   from nx_mimosa import __version__, MultiTargetTracker
   print(f"NX-MIMOSA v{__version__}")
   tracker = MultiTargetTracker(dt=1.0, r_std=50.0)
   print("Installation OK")
