Intelligence Pipeline
=====================

NX-MIMOSA provides four intelligence layers that operate on confirmed tracks.

Platform Identification
------------------------
Classifies 111 aircraft types from kinematics alone (speed, altitude, turn rate).
66.5% fine-grain accuracy, 99.8% top-3. No IFF transponder needed.

Military Identification
------------------------
ICAO24 hex range lookup (military blocks per ICAO Annex 10) plus 35 callsign
regex patterns covering 30+ air forces.

ECM Detection
--------------
NIS-based anomaly detection for 5 ECM types: DRFM/repeater jamming, RGPO,
noise jamming, chaff discrimination, ghost track identification.

Intent Prediction
------------------
16 tactical behaviours: holding, intercept, terrain following, SAM evasion,
aerial refuelling, and more.
