# NX-MIMOSA v4.0.1 "SENTINEL"

**Platform-Aware Variable-Structure IMM Tracker with Multi-Stream Architecture**

*Nexellum d.o.o. — Dr. Mladen Mešter*

## Performance

8/8 scenario wins against **both** FilterPy IMM and Stone Soup 1.8 oracle (best-of-5 trackers per scenario).

| Metric | v4.0.1 | Competitor | Improvement |
|--------|--------|------------|-------------|
| vs FilterPy IMM | 6.46m avg | 60.12m avg | **+89.3%** |
| vs SS BEST oracle | 6.46m avg | 12.06m avg | **+46.4%** |
| vs SS UKF-CA (fair) | 10.33m avg | 14.35m avg | **+28.0%** |

## Architecture

```
Raw Measurements ──┬──→ IMM Core (6 models) ──→ IMM-Forward (realtime)
                   │                          ├──→ Hybrid (offline)
                   │                          └──→ Full-RTS (offline)
                   ├──→ CV Kalman Filter ─────→ CV-RTS (offline)
                   └──→ CA Kalman Filter ─────→ CA-RTS (offline)
                   
                   Auto-Stream Selector → BEST output per scenario
```

### Multi-Stream Outputs

| Stream | Latency | Use Case |
|--------|---------|----------|
| **IMM-Forward** | 0 steps | Fire control, high-dynamics tracking |
| **Adaptive** | 0 steps | General-purpose realtime |
| **CV-RTS** | Full track | Post-mission, benign targets |
| **CA-RTS** | Full track | Post-mission, gentle maneuvers |
| **Hybrid** | Full track | Post-mission, mixed dynamics |
| **Full-RTS** | Full track | Post-mission, short tracks |

### Key Innovation

Parallel independent CV/CA Kalman filters run on raw measurements alongside the IMM core. They have **zero mixing noise** because they never participate in IMM model probability updates. Combined with RTS smoothing, they match or exceed Stone Soup's best single-model trackers on benign targets, while the IMM core dominates on high-dynamics scenarios.

## Quick Start

```python
from nx_mimosa_v40_sentinel import NxMimosaV40Sentinel

tracker = NxMimosaV40Sentinel(dt=0.1, r_std=2.5)

# Feed measurements
for z in measurements:
    position, covariance, intent = tracker.update(z)

# Get outputs
realtime = tracker.get_forward_estimates()      # Fire control
offline  = tracker.get_ca_rts_estimates()        # Best post-mission
hybrid   = tracker.get_hybrid_best_estimates()   # Mixed dynamics
```

## Benchmark Scenarios

8 military tracking scenarios covering the full operational envelope:

- **F16 Dogfight**: 9g sustained turns, rapid reversals
- **Kinzhal Glide**: Mach 5 hypersonic with terminal maneuvers
- **Iskander Terminal**: Ballistic with evasive terminal phase
- **Kalibr Cruise**: Subsonic with waypoint turns
- **Su-35 Post-Stall**: Cobra maneuver, post-stall dynamics
- **SAM Terminal**: High-g intercept with proportional navigation
- **Shahed Loiter**: Low-speed drone with gentle orbit
- **FPV Attack**: Small drone with aggressive terminal dive

## File Structure

```
nx-mimosa/
├── python/
│   └── nx_mimosa_v40_sentinel.py    # Main tracker implementation
├── benchmarks/
│   ├── benchmark_v40_sentinel.py    # Internal benchmark (vs FilterPy)
│   └── benchmark_vs_stonesoup.py    # Full SS 1.8 comparison
├── data/
│   └── platform_db.json             # 18 military platform profiles
├── rtl/                             # SystemVerilog for FPGA
├── fpga/                            # Vivado build scripts
└── config/                          # Register maps (YAML)
```

## License

Dual-license: AGPL v3 (open-source) / Commercial (contact mladen@nexellum.com)

## Contact

Dr. Mladen Mešter — mladen@nexellum.com | +385 99 737 5100
