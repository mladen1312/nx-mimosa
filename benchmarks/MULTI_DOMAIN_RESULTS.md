## Multi-Domain Open Benchmark: NX-MIMOSA v4.2

**19 scenarios across 5 domains** — fully reproducible.

```bash
pip install stonesoup filterpy pykalman numpy
python benchmarks/multi_domain_benchmark.py
```

### Results — Best-of-class RMS (meters)

| ID | Scenario | Domain | Stone Soup | FilterPy | PyKalman | NX-MIMOSA | Winner |
|---|---|---|---:|---:|---:|---:|---|
| S01 | Enroute Cruise | ATC | 69.98 | 96.71 | 109.75 | **32.20** ★ | **NX-MIMOSA** |
| S02 | Holding Pattern | ATC | 72.23 | 56.50 | 77.18 | **55.56** ★ | **NX-MIMOSA** |
| S03 | ILS Approach | ATC | 39.77 | 42.33 | 48.17 | **35.53** ★ | **NX-MIMOSA** |
| S04 | Missed Approach | ATC | 59.27 | 73.55 | 69.55 | **52.50** ★ | **NX-MIMOSA** |
| S05 | Cruise + Wind Shear | AVIATION | 16.06 | 19.64 | 21.04 | **11.58** ★ | **NX-MIMOSA** |
| S06 | Turbulence | AVIATION | 17.72 | 20.43 | 23.04 | **12.94** ★ | **NX-MIMOSA** |
| S07 | TCAS RA Climb | AVIATION | 24.79 | 31.22 | 33.50 | **11.48** ★ | **NX-MIMOSA** |
| S08 | Fighter Intercept | MILITARY | 23.60 | 13.86 | 23.78 | **7.79** ★ | **NX-MIMOSA** |
| S09 | SAM Engagement | MILITARY | 99.91 | 45.86 | 100.04 | **11.37** ★ | **NX-MIMOSA** |
| S10 | Cruise Missile | MILITARY | 7.39 | 6.89 | 7.58 | **3.88** ★ | **NX-MIMOSA** |
| S11 | Helicopter NOE | MILITARY | 4.20 | 5.60 | 4.21 | **3.34** ★ | **NX-MIMOSA** |
| S12 | Highway Cruise | AUTOMOTIVE | 0.16 | 0.16 | 0.16 | **0.16** ★ | **NX-MIMOSA** |
| S13 | Urban Intersection | AUTOMOTIVE | 0.21 | 0.24 | 0.22 | **0.20** ★ | **NX-MIMOSA** |
| S14 | Emergency Brake | AUTOMOTIVE | 0.22 | 0.30 | 0.22 | **0.22** ★ | **NX-MIMOSA** |
| S15 | Lane Change | AUTOMOTIVE | 0.18 | 0.18 | **0.18** ★ | 0.19 | PyKalman |
| S16 | LEO Satellite | SPACE | 4194.55 | 6450.92 | 6916.74 | **1011.93** ★ | **NX-MIMOSA** |
| S17 | GEO Stationkeeping | SPACE | 68.94 | 70.03 | 71.39 | **65.42** ★ | **NX-MIMOSA** |
| S18 | Orbital Maneuver | SPACE | 5670.49 | 8655.95 | 9863.84 | **105.14** ★ | **NX-MIMOSA** |
| S19 | Reentry Vehicle | SPACE | 6077.77 | 14724.27 | 9297.91 | **483.48** ★ | **NX-MIMOSA** |

**Overall: Stone Soup 0/19 | FilterPy 0/19 | PyKalman 1/19 | NX-MIMOSA 18/19**
