<p align="center">
  <img src="reports/nx-mimosa-hero.jpg" alt="NX-MIMOSA: Physics-First Radar Architecture" width="100%">
</p>

<h1 align="center">NX-MIMOSA</h1>
<p align="center"><strong>The radar tracker that sees through jamming.</strong></p>

<p align="center">
  <a href="https://pypi.org/project/nx-mimosa/"><img src="https://img.shields.io/pypi/v/nx-mimosa?color=%2300B4D8&label=PyPI" alt="PyPI"></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/License-AGPL_v3-blue.svg" alt="License"></a>
  <a href="https://www.python.org/downloads/"><img src="https://img.shields.io/badge/python-3.9+-blue.svg" alt="Python"></a>
  <a href="https://nx-mimosa.readthedocs.io"><img src="https://img.shields.io/badge/docs-readthedocs-success" alt="Docs"></a>
  <a href="docs/BENCHMARK_v591_500plus.md"><img src="https://img.shields.io/badge/Live_Test-761_Aircraft-success" alt="Live Test"></a>
</p>

<p align="center">
  <code>pip install nx-mimosa</code>
</p>

---

**5,000 targets in 40 ms.** Detects jamming. Identifies military jets. Outputs NATO standard.
One `pip install` — zero vendor lock-in.

<p align="center">
  <img src="reports/fighter_intercept_demo.gif" alt="Fighter Intercept — 7g turn, 30 clutter/scan" width="720">
  <br>
  <em>7g fighter turn with 30 false alarms per scan. 12 m tracking error. Seed 42, fully reproducible.</em>
</p>

---

## What it does

NX-MIMOSA is a **complete radar tracking system** — not a toolkit, not building blocks.
Drop it into your radar pipeline and it handles everything from raw detections to
NATO-formatted track output.

```python
from nx_mimosa import MultiTargetTracker

tracker = MultiTargetTracker(dt=5.0, r_std=150.0, domain="air")
for scan in radar_scans:
    tracks = tracker.process_scan(measurements)
    for t in tracks:
        print(f"Track {t.track_id}: pos={t.position}, vel={t.velocity}")
```

**That's it.** No assembly. No configuration files. No 50-page integration guide.

For C++ speed (50× faster):
```python
from nx_mimosa.accel import MultiTargetTracker  # Same API, C++ backend
```

---

## The numbers

| Metric | Value |
|--------|-------|
| Simultaneous targets | **5,000** @ 40 ms scan time |
| Live aircraft tracked | **761** (OpenSky ADS-B, Central Europe) |
| Benchmark wins | **18/19** vs Stone Soup, FilterPy, PyKalman |
| Average RMS improvement | **8.6×** over Stone Soup |
| Detection rate | **99.8%** |
| Military jets auto-ID'd | **5** across 4 NATO air forces |
| ECM types detected | **5** (DRFM, RGPO, noise, chaff, ghost) |

The one benchmark loss (S15 lane change): 0.19 m vs 0.18 m. One centimetre. We show it because we show everything.

---

## What makes it different

**Most trackers give you filtering. This one gives you intelligence.**

| | NX-MIMOSA | Stone Soup | MATLAB SFT |
|---|:---:|:---:|:---:|
| Track 5,000 targets | ✅ 40 ms | — | — |
| IMM (6 motion models) | ✅ | Composable | ✅ |
| Auto ECM detection | ✅ 5 types | — | — |
| Military jet identification | ✅ 30+ forces | — | — |
| Platform classification | ✅ 111 types | — | — |
| Intent prediction | ✅ 16 behaviours | — | — |
| ASTERIX + Link-16 output | ✅ | — | — |
| Sensor fusion (6 types) | ✅ | ✅ | ✅ |
| Deploy with one command | `pip install` | `pip install` | MATLAB runtime |
| Price | Free / $50K+ | Free | ~$5K/yr |

Stone Soup is excellent for *research* — modular, extensible, government-backed.
NX-MIMOSA is built for *deployment* — integrated, tested at scale, intelligence included.

---

## Speed

C++ core with Eigen, sparse auction assignment, OpenMP parallelism.

| Targets | Python | C++ | Speedup |
|---------|--------|-----|---------|
| 100 | 44 ms | **0.9 ms** | 49× |
| 761 | 469 ms | **7 ms** | 67× |
| 1,000 | 673 ms | **8 ms** | 84× |
| 2,000 | — | **17 ms** | — |
| 5,000 | — | **40 ms** | — |

Build from source:
```bash
cd cpp && pip install .  # requires Eigen3 + pybind11
```

---

## Capabilities at a glance

**Filtering:** 6-model IMM, EKF, UKF, Particle Filter, GM-PHD, CPHD, LMB

**Association:** GNN, JPDA, MHT (N-scan pruning), Sparse Auction (Bertsekas)

**Intelligence:** 111 platforms, 30+ military forces, 5 ECM types, 16 intent behaviours

**Fusion:** 6 sensor types, track-to-track, covariance intersection, OOSM, bias estimation

**Output:** ASTERIX Cat048, Link-16 J3.2, dual-mode (display + fire control), GOSPA/OSPA/NEES/SIAP

**Deployment:** `pip install`, single-file copy, or C++ extension — zero deps beyond NumPy/SciPy

**9 modules · 11,493 lines · 62 classes · 340 tests**

---

## Live proof: 761 aircraft

OpenSky Network ADS-B, Central Europe, February 2026. Every measurement is a real aircraft.

| | |
|---|---|
| Peak simultaneous targets | **761** |
| Detection rate | **99.8%** |
| Mean scan time | **519 ms** (Python) / **7 ms** (C++) |
| Military jets identified | NSZ21U (NATO), RCH4539 (USAF), CTM2004 (French AF), PLF105 (Polish AF), NSZ3YT (NATO) |

That test ran on a laptop. The data and scripts are in this repo. Run it yourself:
```bash
python benchmarks/multi_domain_benchmark.py
```

Full report: [`docs/BENCHMARK_v591_500plus.md`](docs/BENCHMARK_v591_500plus.md)

---

## 19-scenario benchmark

All at seed = 42. RMS position error (metres). Lower is better.

| Scenario | Stone Soup | NX-MIMOSA | Winner |
|----------|-----------|-----------|--------|
| ATC Enroute | 69.98 | **32.20** | ★ |
| Fighter Intercept | 23.60 | **7.79** | ★ |
| SAM Engagement | 99.91 | **11.37** | ★ |
| Cruise Missile | 7.39 | **3.88** | ★ |
| Ballistic Missile | 86.33 | **51.04** | ★ |
| LEO Satellite | 4,194 | **1,012** | ★ |
| Atmospheric Reentry | 6,078 | **483** | ★ |
| Lane Change | **0.18** | 0.19 | Stone Soup |
| **Average (19 scenarios)** | **865.66** | **100.26** | **8.6×** |

**Honesty note:** Stone Soup uses single-model KF in these benchmarks; NX-MIMOSA uses 6-model IMM. Any IMM would beat single-model on high-maneuver scenarios. The fair comparison is cruise-flight (1.1–2.2× improvement). Full table in the [benchmark report](docs/BENCHMARK_v591_500plus.md).

---

## Quick start

```bash
pip install nx-mimosa
```

```python
from nx_mimosa import MultiTargetTracker
import numpy as np

tracker = MultiTargetTracker(dt=5.0, r_std=150.0, domain='air')

for scan in radar_scans:
    tracks = tracker.process_scan(np.array(scan))
    for t in tracks:
        print(f"Track {t.track_id}: {t.position}")
```

**JPDA for dense clutter:**
```python
tracker = MultiTargetTracker(dt=5.0, r_std=150.0, association='jpda')
```

**ECM detection:**
```python
from nx_mimosa import ECMDetector
ecm = ECMDetector()
result = ecm.update(track_id=1, nis=15.2, position=pos, velocity=vel, was_hit=True)
if result['ecm_detected']:
    print(f"ECM: {result['ecm_types']} → {result['recommended_action']}")
```

**C++ accelerated (same API, 50× faster):**
```python
from nx_mimosa.accel import MultiTargetTracker
tracker = MultiTargetTracker(dt=5.0, r_std=150.0)  # auto C++ backend
print(f"Scan time: {tracker.scan_time_ms:.1f} ms")
```

---

## Documentation

📖 **API Reference:** [nx-mimosa.readthedocs.io](https://nx-mimosa.readthedocs.io) — 1,000+ documented symbols

📊 **761-Aircraft Benchmark:** [`docs/BENCHMARK_v591_500plus.md`](docs/BENCHMARK_v591_500plus.md)

⚡ **C++ Performance Report:** [`docs/BENCHMARK_v600_cpp_core.md`](docs/BENCHMARK_v600_cpp_core.md)

---

## Licensing

| | Open Source | Lite | Pro | Enterprise |
|---|---|---|---|---|
| **Price** | Free | $50,000 | $150,000 | $350,000 |
| Core tracker + IMM | ✅ | ✅ | ✅ | ✅ |
| Intelligence layer | ✅ | — | ✅ | ✅ |
| C++ core | ✅ | ✅ | ✅ | ✅ |
| Private modifications | AGPL | ✅ | ✅ | ✅ |
| FPGA SystemVerilog | — | — | — | ✅ |
| DO-254 cert support | — | — | — | ✅ |
| Support | Community | 12 months | 12 months | 24 months |

**Build vs buy:** 3–5 engineers × 12–18 months = $1.5M–$2.5M for basic tracking only.

---

## Roadmap

| Version | Milestone | Status |
|---------|-----------|--------|
| v5.9.3 | LMB filter, 62 classes, 11,493 LOC | ✅ Done |
| v6.0 | C++ core — 40 ms @ 5,000 targets | ✅ Done |
| v6.0.1 | PyPI + ReadTheDocs + marketing update | ✅ Done |
| v6.1 | FPGA proof-of-concept (Xilinx Versal) | Q2 2026 |
| v6.2 | Multi-radar fusion service | Q3 2026 |
| v7.0 | DO-254 certification path | Q4 2026 |

---

<p align="center">
  <strong>Dr. Mladen Mešter</strong> · Nexellum d.o.o. · Zagreb, Croatia<br>
  <a href="mailto:mladen@nexellum.com">mladen@nexellum.com</a> · +385 99 737 5100<br><br>
  Every number in this document is reproducible.<br>
  Run the benchmarks or ask us to demonstrate.
</p>
