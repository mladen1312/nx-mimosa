Random Finite Set Filters
=========================

For scenarios where the number of targets is unknown and variable.

GM-PHD
------
Gaussian Mixture Probability Hypothesis Density filter. Models the multi-target
state as a Poisson point process. Target birth modeled as Gaussian mixtures.

CPHD
----
Cardinalized PHD adds a cardinality distribution to the PHD filter for more
accurate target count estimation.

LMB
---
Labeled Multi-Bernoulli provides explicit track identity through labeled
Bernoulli components. Each component has an existence probability and a
spatial distribution. The cardinality follows a Poisson binomial distribution.

For most radar applications with thresholded detections, the IMM+GNN/JPDA/MHT
pipeline remains recommended — it provides better single-target accuracy with
lower computational cost.
