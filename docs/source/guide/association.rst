Data Association
================

Three association engines handle the measurement-to-track assignment problem.

GNN (Global Nearest Neighbour)
-------------------------------
Fastest option. Solves the assignment as a linear sum assignment problem using the
Hungarian algorithm. Best for low-clutter environments.

JPDA (Joint Probabilistic Data Association)
--------------------------------------------
Computes association probabilities for all measurement-track pairs within the gate.
Best for dense clutter where multiple measurements may originate from the same target.

MHT (Multiple Hypothesis Tracking)
------------------------------------
Maintains a tree of association hypotheses over multiple scans with N-scan pruning.
Best for crossing targets where single-scan methods may swap tracks.
