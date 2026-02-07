Filters
=======

NX-MIMOSA provides 8 filter types to cover the full spectrum of tracking scenarios.

Kalman Filters
--------------

**KalmanFilter3D** — Standard linear Kalman filter for Cartesian measurements.

**EKF3D** — Extended Kalman Filter for polar radar measurements (range, azimuth, elevation).
Linearises the nonlinear measurement function h(x) via Jacobian.

**UKF3D** — Unscented Kalman Filter for polar measurements. Uses sigma point propagation
instead of Jacobian linearisation — more accurate for highly nonlinear systems.

IMM (Interacting Multiple Model)
---------------------------------

The 6-model IMM runs CV, CA, CT (both turn directions), Jerk, Ballistic, and Orbital
models in parallel. Mode probabilities update each scan via Markov transition matrix.

Random Finite Set Filters
--------------------------

**GM-PHD** (Vo & Ma 2006) — tracks unknown number of targets as a Poisson point process.

**CPHD** (Vo, Vo & Cantoni 2007) — extends PHD with cardinality distribution for more
accurate target count estimation.

**LMB** (Reuter, Vo, Vo & Dietmayer 2014) — provides explicit track identity through
labeled Bernoulli components with existence probabilities.

Particle Filter
---------------

**ParticleFilter3D** — SIR with systematic resampling for non-Gaussian scenarios.
