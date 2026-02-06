# NX-MIMOSA v4.3.0 — Commercial Readiness Audit
## "Do we have everything a commercial system needs?"

**Date:** 2026-02-06  
**Auditor:** Radar Systems Architect v9.0  
**Verdict:** ⚠️ **ALPHA-GRADE — NOT YET COMMERCIALLY VIABLE**

---

## Executive Summary

NX-MIMOSA has **world-class single-target state estimation** (8.6× better than Stone Soup) but is missing **4 critical subsystems** that every commercial radar tracker must have. Without them, no serious integrator will deploy it.

Current state: **~35% of a complete commercial tracking system.**

---

## TIER 1 — CRITICAL BLOCKERS (Cannot sell without these)

### ❌ 1. Multi-Target Tracking (MTT)
**Status: NOT IMPLEMENTED**  
**Impact: SHOW-STOPPER for every market segment**

Every commercial radar sees **multiple targets simultaneously**. A fighter squadron, highway traffic, airport approach — all have N>1 targets.

| Required Component | Status | Complexity |
|---|---|---|
| Track table (managed list of active tracks) | ❌ Missing | 2 weeks |
| GNN data association (Munkres/Hungarian) | ❌ Missing | 3 weeks |
| JPDA data association | ❌ Missing | 4 weeks |
| MHT (deferred decision) | ❌ Missing | 8 weeks |
| Track-to-track correlation | ❌ Missing | 3 weeks |

**What MATLAB Sensor Fusion Toolbox offers:**
- trackerGNN, trackerJPDA, trackerTOMHT, trackerPHD — all four
- Munkres (Hungarian), Auction, JV assignment algorithms
- Configurable gating, confirmation, deletion logic

**What Stone Soup offers:**
- GNN, JPDA, MHT — all three  
- PHD/CPHD/GM-PHD random finite set trackers
- Full track management lifecycle

**What we offer:** Nothing. Single target only.

**Priority:** 🔴 P0 — Implement GNN first (simplest, covers 80% of use cases)

---

### ❌ 2. 3D Tracking
**Status: NOT IMPLEMENTED**  
**Impact: SHOW-STOPPER for air defense, space, UAV, ATC**

Current system tracks in 2D Cartesian [x, y] only. Every real radar operates in 3D.

| Required Component | Status | Complexity |
|---|---|---|
| 3D state vector [x, y, z, vx, vy, vz] | ❌ Missing | 1 week |
| 3D motion models (CV3D, CA3D, CT3D) | ❌ Missing | 2 weeks |
| Elevation measurement model | ❌ Missing | 1 week |
| 3D IMM bank adaptation | ❌ Missing | 2 weeks |
| 3D platform DB kinematics | ❌ Missing | 1 week |

**Reality check:** Our 2D tracker is useful ONLY for:
- Ground-based maritime surveillance (flat earth, targets at sea level)
- 2D simulation/research

**Everything else requires 3D:** ATC, air defense, space, UAV, automotive (vehicles have height differences on hills)

**Priority:** 🔴 P0 — Extend state vector to 6D minimum

---

### ❌ 3. Track Management (Initiation / Confirmation / Deletion)
**Status: NOT IMPLEMENTED**  
**Impact: SHOW-STOPPER — assumes perfect continuous track**

Current system assumes: "here is measurement #1 for a target, here is #2, #3..." forever. Real systems must handle:

| Required Component | Status | Complexity |
|---|---|---|
| Track initiation (2-of-3, 3-of-5 logic) | ❌ Missing | 2 weeks |
| Tentative → Confirmed promotion | ❌ Missing | 1 week |
| Track deletion (M-of-N miss logic) | ❌ Missing | 1 week |
| Track scoring (log-likelihood ratio) | ❌ Missing | 2 weeks |
| Track ID management | ❌ Missing | 1 week |
| Track coasting (predict without update) | ❌ Missing | 3 days |
| Track merging/splitting | ❌ Missing | 2 weeks |

**Every competing system has this:**
- MATLAB: Built into every tracker (GNN/JPDA/TOMHT)
- Stone Soup: Full lifecycle management
- Any deployed radar: Mandatory

**Priority:** 🔴 P0 — Implement M-of-N logic first

---

### ❌ 4. Coordinate Transforms
**Status: NOT IMPLEMENTED**  
**Impact: Cannot interface with any real sensor**

Real radars output [range, azimuth, elevation] or [range, azimuth, elevation, range_rate]. Real systems use WGS-84 geodetic coordinates. We have none of this.

| Required Component | Status | Complexity |
|---|---|---|
| Spherical → Cartesian (and inverse) | ❌ Missing | 3 days |
| WGS-84 geodetic ↔ ECEF | ❌ Missing | 1 week |
| ENU (East-North-Up) local frame | ❌ Missing | 1 week |
| Sensor frame → track frame rotation | ❌ Missing | 1 week |
| Unbiased polar-to-Cartesian conversion | ❌ Missing | 3 days |
| EKF measurement Jacobians (spherical) | 🟡 Partial (in fusion) | 1 week |

**Note:** Our fusion module has EKF Jacobians for range-bearing, but no geodetic/ECEF transforms.

**Priority:** 🔴 P0 — Without this, cannot connect to a single real radar

---

## TIER 2 — IMPORTANT GAPS (Required for specific markets)

### ❌ 5. Out-of-Sequence Measurement (OOSM) Handling
**Status: NOT IMPLEMENTED**

Multi-sensor systems regularly deliver measurements out of timestamp order (network delays, different scan rates). Current system assumes perfectly ordered measurements.

| Required | Status | Market |
|---|---|---|
| Retrodiction (Bar-Shalom algorithm) | ❌ Missing | All multi-sensor |
| Measurement buffering + reordering | ❌ Missing | All networked |
| Maximum acceptable lag policy | ❌ Missing | Real-time |

**Complexity:** 3 weeks  
**Priority:** 🟠 P1

---

### ❌ 6. Performance Metrics Suite
**Status: PARTIAL (RMS only)**

Commercial systems need standardized metrics for acceptance testing.

| Metric | Status | Standard |
|---|---|---|
| RMS position error | ✅ Implemented | — |
| NEES (Normalized Estimation Error Squared) | ❌ Missing | Bar-Shalom |
| NIS (Normalized Innovation Squared) | 🟡 Internal only | Bar-Shalom |
| Track completeness (% of truth covered) | ❌ Missing | SIAP |
| Track purity (% correct association) | ❌ Missing | SIAP |
| CPEP (Circular Position Error Probable) | ❌ Missing | Military |
| OSPA / GOSPA (multi-target metric) | ❌ Missing | Schuhmacher |
| Track latency (init delay) | ❌ Missing | SIAP |
| False track rate | ❌ Missing | SIAP |
| Track fragmentation | ❌ Missing | SIAP |

**Note:** SIAP (Single Integrated Air Picture) metrics are NATO standard for track quality assessment.

**Complexity:** 4 weeks  
**Priority:** 🟠 P1 — Required for any acceptance testing

---

### ❌ 7. Sensor Registration / Bias Estimation
**Status: NOT IMPLEMENTED**

Real multi-sensor systems have systematic sensor biases (range bias, azimuth bias, timing offset). Without bias estimation, fusion degrades significantly.

| Required | Status |
|---|---|
| Static bias estimation (range, azimuth, elevation) | ❌ Missing |
| Dynamic bias tracking (temperature drift, etc.) | ❌ Missing |
| Multi-sensor registration (least squares) | ❌ Missing |

**Complexity:** 4 weeks  
**Priority:** 🟠 P1

---

### ❌ 8. Doppler / Range-Rate Processing
**Status: PARTIAL (in fusion only)**

Fusion module accepts range-rate measurements but core tracker has no native Doppler processing:

| Required | Status |
|---|---|
| Range-rate measurement model | 🟡 Fusion only |
| Doppler ambiguity resolution | ❌ Missing |
| MTI/MTD integration | ❌ Missing |
| Clutter map + CFAR | ❌ Missing |

**Complexity:** 6 weeks  
**Priority:** 🟠 P1 for military radar

---

## TIER 3 — NICE-TO-HAVE (Differentiators for premium product)

### ✅ / ❌ 9. Existing Strengths vs Missing Polish

| Feature | Status | Notes |
|---|---|---|
| IMM (6 models) | ✅ **Excellent** | Best-in-class |
| Adaptive smoother (AOS) | ✅ **Excellent** | Unique capability |
| Platform identification | ✅ **Excellent** | 18 military platforms |
| Intent prediction | ✅ **Excellent** | Unique — no competitor has this |
| ECM detection | ✅ **Excellent** | Unique — no competitor has this |
| Multi-sensor fusion | ✅ **Good** | 6 sensor types |
| GUARDIAN gating | ✅ **Good** | NIS-based outlier rejection |
| Dual-mode smoother | ✅ **Good** | Real-time + offline |
| API documentation | 🟡 Partial | Docstrings only, no Sphinx/RTD |
| pip installable package | ❌ Missing | No setup.py/pyproject.toml |
| Type hints throughout | 🟡 Partial | Some functions |
| Logging (Python logging) | ❌ Missing | Print statements only |
| Configuration file support | ❌ Missing | Hardcoded params |
| Network I/O (ZMQ/DDS/STANAG) | ❌ Missing | No external interface |
| Recording/playback | ❌ Missing | No track recording |
| Thread safety | ❌ Missing | Not thread-safe |
| Docker container | ❌ Missing | No Dockerfile |
| CI/CD pipeline | ❌ Missing | No GitHub Actions |

---

## COMPETITIVE FEATURE MATRIX

```
Feature                    NX-MIMOSA   MATLAB SFT   Stone Soup   Custom
─────────────────────────────────────────────────────────────────────────
Single-target estimation   ★★★★★       ★★★★         ★★★          ★★★
IMM adaptive tracking      ★★★★★       ★★★★         ★★★★         ★★
Multi-target (GNN)         ☆            ★★★★★       ★★★★         ★★★
Multi-target (JPDA)        ☆            ★★★★★       ★★★★         ★★
Multi-target (MHT)         ☆            ★★★★        ★★★★         ★
3D tracking                ☆            ★★★★★       ★★★★★        ★★★
Track management           ☆            ★★★★★       ★★★★         ★★★
Coordinate transforms      ☆            ★★★★★       ★★★★         ★★★
Multi-sensor fusion        ★★★★         ★★★★★       ★★★★         ★★
Platform identification    ★★★★★       ☆            ☆             ☆
Intent prediction          ★★★★★       ☆            ☆             ☆
ECM detection              ★★★★★       ☆            ☆             ☆
Bias estimation            ☆            ★★★★        ★★★          ★★
OSPA metrics               ☆            ★★★★★       ★★★★         ☆
OOSM handling              ☆            ★★★         ★★★          ★
FPGA deployment path       ★★           ★★★★ (C gen) ☆            ★★★
Documentation              ★★           ★★★★★       ★★★★         ★
Licensing cost             Free/AGPL    ~$5K/yr      Free/MIT     $$$
─────────────────────────────────────────────────────────────────────────
OVERALL READINESS          35%          95%          80%          varies
```

**Legend:** ★★★★★ = Excellent, ★★★ = Good, ★ = Basic, ☆ = Missing

---

## DEVELOPMENT ROADMAP TO COMMERCIAL VIABILITY

### Phase 1: "Minimum Viable Tracker" (3 months, ~$150K effort)

**Goal: 70% feature parity → first pilot customer**

| Week | Deliverable | Effort |
|---|---|---|
| 1-2 | 3D state vector + 3D motion models (CV3D, CA3D, CT3D) | 80h |
| 3-4 | Coordinate transform library (spherical, WGS-84, ENU) | 80h |
| 5-6 | Track table + track management (M-of-N init/delete) | 80h |
| 7-9 | GNN data association (Munkres algorithm) | 120h |
| 10 | SIAP metrics suite (completeness, purity, OSPA) | 40h |
| 11 | pip packaging + GitHub Actions CI + Sphinx docs | 40h |
| 12 | Integration test: simulated 3D multi-target scenario | 40h |

**Result:** NX-MIMOSA v5.0 — 3D multi-target tracker with GNN, coordinate transforms, track management.

### Phase 2: "Professional Edition" (3 months, ~$150K effort)

| Week | Deliverable | Effort |
|---|---|---|
| 13-16 | JPDA data association | 160h |
| 17-18 | OOSM handling (retrodiction) | 80h |
| 19-20 | Sensor bias estimation | 80h |
| 21-22 | Network I/O (ZMQ publisher + STANAG 4586 messages) | 80h |
| 23-24 | Docker container + Yocto recipe + recording/playback | 80h |

**Result:** NX-MIMOSA v5.5 — Production-grade multi-sensor MTT system.

### Phase 3: "Defense Certified" (6 months, ~$300K effort)

| Month | Deliverable |
|---|---|
| 7-8 | MHT tracker (deferred decision) |
| 9-10 | FPGA feasibility + RTL prototype (IMM core on ZU48DR) |
| 11 | DO-254 / MIL-STD-882E compliance documentation |
| 12 | Acceptance test suite with NATO SIAP metrics |

**Result:** NX-MIMOSA v6.0 — Defense-grade, certifiable, FPGA-ready.

---

## HONEST ASSESSMENT

### What we ARE world-class at:
1. **Single-target state estimation** — 8.6× better than Stone Soup, proven
2. **Adaptive intelligence** — IMM + AOS + platform ID + intent prediction
3. **ECM resilience** — No competitor has this at the algorithm level
4. **Multi-sensor fusion** — 6 sensor types, +64% improvement with Doppler+ADS-B

### What we are MISSING that prevents any sale:
1. **Can't track multiple targets** — every real scenario has N>1
2. **Can't track in 3D** — every real radar operates in 3D
3. **Can't manage track lifecycle** — assumes perfect continuous data
4. **Can't interface with real sensors** — no coordinate transforms

### The brutal truth:
We have built **the world's best engine** but put it in **a car with no wheels, no steering, and no doors**. The IMM+AOS+intent core is genuinely superior. But without MTT, 3D, track management, and coordinate transforms, it's a research prototype — not a product.

The good news: Phase 1 (3 months) gets us to **minimum viable product**. The architecture is sound — these are additive features, not redesigns.

---

## RECOMMENDED NEXT ACTION

**Build Phase 1 in this order (dependencies):**

```
Week 1-2:  3D state vector          ← Foundation for everything
    ↓
Week 3-4:  Coordinate transforms    ← Required for real sensor input
    ↓
Week 5-6:  Track management         ← Required for MTT
    ↓
Week 7-9:  GNN data association     ← The "multi" in multi-target
    ↓
Week 10:   Metrics suite            ← Prove it works
    ↓
Week 11:   Packaging + CI           ← Make it installable
    ↓
Week 12:   Integration test         ← End-to-end validation
```

**Dr. Mešter, želiš li da krenem s 3D tracking implementacijom odmah? To je prvi blok u kritičnom putu.**
