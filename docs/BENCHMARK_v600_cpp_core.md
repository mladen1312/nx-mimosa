# NX-MIMOSA v6.0 — C++ Core Benchmark Report

**Date:** 2026-02-07  
**Objective:** Sub-50ms scan processing at 1000+ simultaneous targets  
**Status:** ✅ ACHIEVED — 13.5 ms mean at 1000 targets (50× vs Python)

## Architecture

C++ core (`_nx_core`) replaces the Python hot path:
- **Batch IMM predict** with Eigen + OpenMP
- **KDTree spatial pre-gating** (header-only, O(n log n))
- **Batch Mahalanobis distance** (Eigen LLT Cholesky)
- **LAPJV assignment** (Jonker-Volgenant, O(n²) avg)
- **pybind11 bindings** for zero-copy NumPy interop

Python intelligence/fusion/coordinates layers unchanged.

## Profiling — Where Time Was Spent (Python v5.9.3)

| Component | % of scan | Calls/scan (1000 trk) | Issue |
|---|---|---|---|
| `gnn_associate` → `mahalanobis_distance` | 82% | 25,265 | Python loop, repeated `combined_state` |
| `combined_state` (IMM mixing) | 51% | 26,265 | Per-track 4-model matrix ops |
| `predict` | 8% | 1,000 | Sequential Python loop |
| `update` | 9% | 1,000 | Sequential Python loop |

## Head-to-Head Results (same data, seed=42)

| Targets | Python v5.9.3 | C++ v6.0 | Speedup | <50ms |
|---|---|---|---|---|
| 100 | 44.4 ms | 0.9 ms | **49×** | ✅ |
| 500 | 265.4 ms | 5.1 ms | **52×** | ✅ |
| 761 | 468.7 ms | 10.3 ms | **46×** | ✅ |
| 1,000 | 672.6 ms | 13.5 ms | **50×** | ✅ |
| 2,000 | est ~2,500 ms | 51.1 ms | **~49×** | ❌ (marginal) |

## C++ Standalone Benchmark (no Python overhead)

| Targets | Mean (ms) | Max (ms) | Confirmed |
|---|---|---|---|
| 100 | 1.1 | 1.3 | 100 |
| 500 | 5.6 | 6.9 | 500 |
| 761 | 9.9 | 13.1 | 761 |
| 1,000 | 15.4 | 21.8 | 1,002 |
| 2,000 | 51.1 | 63.7 | 2,002 |

## Correctness Verification

12/12 pytest tests pass:
- Single target tracking ✅
- 10-target separation ✅  
- Missed detection / coasting ✅
- 2D input auto-padding ✅
- Bulk state extraction ✅
- Track property access ✅
- Performance @ 100/500/761/1000 targets ✅
- Custom config ✅
- Convenience constructor ✅

## Build Requirements

- C++17 compiler (GCC 11+, Clang 14+)
- Eigen 3.3+ (`apt install libeigen3-dev`)
- pybind11 (`pip install pybind11`)
- OpenMP (optional, ~20% speedup on multi-core)

## Usage

```python
# C++ accelerated (drop-in replacement)
from nx_mimosa.accel import MultiTargetTracker
tracker = MultiTargetTracker(dt=5.0, r_std=150.0)

for scan in radar_scans:
    tracks = tracker.process_scan(measurements)
    print(f"{tracker.n_confirmed} tracks in {tracker.scan_time_ms:.1f} ms")

# Bulk state extraction (most efficient)
ids, states = tracker.get_states()  # (N,), (N, 6)
```

## File Structure

```
cpp/
├── include/nx_core.hpp    # 580 LOC — full tracker (KF, IMM, KDTree, LAPJV)
├── src/bindings.cpp       # 160 LOC — pybind11 Python bindings
├── tests/
│   ├── bench.cpp          # C++ standalone benchmark
│   └── test_cpp_core.py   # 12 pytest tests (correctness + performance)
├── CMakeLists.txt         # CMake build (Eigen + pybind11 + OpenMP)
└── setup.py               # pip install ./cpp
nx_mimosa/accel.py         # Python wrapper (drop-in API compatibility)
```

## Roadmap

| Version | Target | Status |
|---|---|---|
| v6.0 | C++ core, <50ms @ 1000 tracks | ✅ Done |
| v6.0.1 | Sparse LAPJV for >2000 tracks | Planned |
| v6.1 | FPGA proof-of-concept (Versal) | Q2 2026 |
| v6.2 | Multi-radar fusion service | Q3 2026 |
