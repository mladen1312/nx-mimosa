## Open Benchmark: NX-MIMOSA vs Stone Soup vs FilterPy vs PyKalman

**Fully reproducible** — anyone can verify:

```bash
pip install stonesoup filterpy pykalman numpy
python benchmarks/open_benchmark.py
```

### Fairness Principles

| Principle | Implementation |
|-----------|---------------|
| Fixed seed | `seed=42` — identical truth + noise for ALL |
| Same noise | R_std = 5.0m for all trackers |
| Fair Q tuning | CV=0.5, CA=2.0, CT=1.0 — not per-tracker optimized |
| Best-of-class | Stone Soup picks BEST of 3 models per scenario |
| No oracle info | No tracker receives true turn rate or acceleration |
| Single config | NX-MIMOSA uses ONE fixed 5-model config for ALL scenarios |

### Results — RMS Position Error (meters)

| Library | 1. Constant V | 2. Gentle Tur | 3. Hard Turn | 4. Accelerati | 5. Fighter Do | 6. Jinking Ev | 7. Ballistic  | **AVG** | **Wins** |
|---------|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Stone Soup | 7.2 | 9.0 | 11.9 | **7.7** ★ | 13.3 | 15.1 | 13.1 | **11.0** | **1/7** |
| FilterPy | 7.7 | 9.7 | 15.1 | 42.5 | 12.3 | 8.3 | 17.8 | **16.2** | **0/7** |
| PyKalman | 7.6 | 9.3 | 12.7 | 9.0 | 13.4 | 15.3 | 13.5 | **11.5** | **0/7** |
| NX-MIMOSA v4.2 | **6.5** ★ | **8.2** ★ | **9.7** ★ | 9.3 | **7.7** ★ | **7.3** ★ | **11.3** ★ | **8.6** | **6/7** |

### Scenarios

1. **1. Constant Velocity — straight line ~291 m/s** (30s, 300 steps)
2. **2. Gentle Turn — 2 deg/s coordinated turn** (20s, 200 steps)
3. **3. Hard Turn — 8 deg/s (~4g at 300 m/s)** (15s, 150 steps)
4. **4. Acceleration — 5g burst (100 to 500 m/s)** (20s, 200 steps)
5. **5. Fighter Dogfight — multi-segment profile** (40s, 400 steps)
6. **6. Jinking Evasion — +/-5 deg/s alternating** (30s, 300 steps)
7. **7. Ballistic Arc — parabolic with gravity** (25s, 250 steps)

### Honest Disclosure

- **S4 4. Accelerati**: Stone Soup wins (7.7m vs NX 9.3m, +19.9%). Pure acceleration/ballistic scenarios favor dedicated CA model; IMM splits probability across 5 models.
- **S4 4. Accelerati**: PyKalman wins (9.0m vs NX 9.3m, +3.2%). Pure acceleration/ballistic scenarios favor dedicated CA model; IMM splits probability across 5 models.

### Head-to-Head

- **vs Stone Soup**: avg +17.8% (range -19.9% to +51.7%)
- **vs FilterPy**: avg +32.8% (range +11.3% to +78.2%)
- **vs PyKalman**: avg +22.5% (range -3.2% to +52.1%)
