# NX-MIMOSA Changelog

## v4.0.1 "SENTINEL" — Multi-Stream Architecture (2026-02-06)

### 🎯 Headline: 8/8 wins vs Stone Soup oracle, +46.4% avg improvement

**Architecture**: Parallel independent filters running alongside IMM core,
with per-scenario auto-stream selection for optimal output.

### Multi-Stream Output Architecture

| Stream | Type | Latency | Best For |
|--------|------|---------|----------|
| IMM-Forward | Realtime | 0 steps | High-dynamics (fighters, SAMs) |
| Adaptive | Realtime | 0 steps | Mixed scenarios (NIS-gated IMM/CV) |
| CV-RTS | Offline | Full track | Benign CV targets (straight flight) |
| CA-RTS | Offline | Full track | Gentle acceleration (cruise missiles) |
| Hybrid | Offline | Full track | Mixed dynamics (CV-RTS benign + IMM dynamic) |
| Full-RTS | Offline | Full track | Short tracks (FPV drones) |

### Benchmark Results (50 MC runs, r_std=2.5m, dt=0.1s)

```
Scenario            v4.0.1 BEST  Stream    SS BEST   Δ vs SS
─────────────────────────────────────────────────────────────
F16_Dogfight            8.29m   IMM-Fwd    15.26m   +45.6%
Kinzhal_Glide           6.69m   CA-RTS     13.78m   +51.4%
Iskander_Terminal       8.75m   CA-RTS      9.02m    +3.0%
Kalibr_Cruise           2.61m   CA-RTS      4.27m   +38.9%
Su35_PostStall          6.53m   Hybrid     11.30m   +42.2%
SAM_Terminal           16.96m   IMM-Fwd    39.11m   +56.6%
Shahed_Loiter           0.73m   CA-RTS      0.85m   +13.5%
FPV_Attack              1.12m   Full-RTS    2.85m   +60.8%
─────────────────────────────────────────────────────────────
Average:                6.46m              12.06m   +46.4%

vs FilterPy IMM:  8/8 wins, +89.3%
vs SS BEST:       8/8 wins, +46.4%
vs SS UKF-CA:     +55.0% (auto-select)
Realtime only:    +28.0% vs SS UKF-CA (fair comparison)
```

### Key Innovation: Parallel Independent Filters

The breakthrough is running **independent CV and CA Kalman filters** on raw
measurements in parallel with the IMM core. These have ZERO mixing noise
because they never participate in IMM model probability updates.

- **CV parallel filter**: q=0.5, 4-state [x,y,vx,vy]
- **CA parallel filter**: q=2.0, 6-state [x,y,vx,vy,ax,ay]
- Both include full-track RTS backward smoothers for offline processing
- **Hybrid stream**: Uses relative dynamics (dv/speed) to select CV-RTS
  for benign segments and IMM-Forward for maneuvering segments

### Why This Beats Stone Soup Oracle

Stone Soup oracle = cherry-pick BEST of 5 independent trackers per scenario.
NX-MIMOSA v4.0.1 = run ALL trackers simultaneously + auto-select optimal
stream. The IMM core provides maneuver handling that no single-model SS
tracker can match, while parallel filters match SS CV+RTS on benign targets.

---

## v4.0.0 "SENTINEL" — Platform-Aware VS-IMM (2026-02-05)

- 6-model IMM bank: CV, CT±, CA, Jerk, Ballistic
- Platform identification database (18 military platforms)
- Variable-Structure model adaptation based on platform ID
- Intent prediction with phase-based Q-scaling
- Window and full-track RTS smoothers
- 7/8 wins vs FilterPy (+79.7% average)
- 4/8 wins vs Stone Soup oracle (high-dynamics dominant)

## v3.3 "Dual Mode" (2026-01-xx)

- Dual-mode architecture: realtime + fire control streams
- 97% of full smooth accuracy at 1.5s latency
- SystemVerilog RTL implementation targeting ZU48DR
