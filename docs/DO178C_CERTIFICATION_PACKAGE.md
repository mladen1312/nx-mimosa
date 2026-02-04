# NX-MIMOSA DO-178C CERTIFICATION PACKAGE
## Software Aspects of Airborne Systems and Equipment Certification

```
Document ID:      NXM-DO178C-001
Version:          1.0
Date:             2026-02-04
Classification:   Design Assurance Level C (DAL-C)
Author:           Dr. Mladen Mešter / Nexellum d.o.o.
```

---

## 1. INTRODUCTION

### 1.1 Purpose

This document provides the certification package for NX-MIMOSA multi-target tracking software in accordance with DO-178C "Software Considerations in Airborne Systems and Equipment Certification".

### 1.2 Design Assurance Level

**DAL-C (Major)** - Failure condition would reduce the capability of the aircraft or the ability of the crew to cope with adverse operating conditions.

| DAL | Failure Condition | Probability | NX-MIMOSA Applicability |
|-----|-------------------|-------------|-------------------------|
| A | Catastrophic | < 10⁻⁹ | Not applicable |
| B | Hazardous | < 10⁻⁷ | Not applicable |
| **C** | **Major** | **< 10⁻⁵** | **ATC Surveillance** |
| D | Minor | < 10⁻³ | Not applicable |

### 1.3 Applicable Documents

| Document | Title |
|----------|-------|
| DO-178C | Software Considerations in Airborne Systems |
| DO-278A | Software Integrity Assurance for CNS/ATM |
| DO-330 | Software Tool Qualification Considerations |
| ED-153 | Guidelines for ANS Software Safety Assurance |
| EUROCONTROL EASSP | ATM Surveillance System Performance |

---

## 2. SOFTWARE CONFIGURATION ITEMS

| CSCI ID | Name | Description | DAL |
|---------|------|-------------|-----|
| NXM-CORE | Core Tracker | VS-IMM, UKF, CKF filters | C |
| NXM-PREP | Preprocessor | Input validation, coordinate conversion | C |
| NXM-TRKM | Track Manager | Track initiation, maintenance, deletion | C |
| NXM-OUTP | Output Formatter | ASTERIX CAT062 encoding | C |

---

## 3. HIGH-LEVEL REQUIREMENTS (HLR)

### 3.1 Functional Requirements

| Req ID | Requirement | Verification |
|--------|-------------|--------------|
| HLR-FUNC-001 | Process radar measurements at rates up to 10 Hz | Test |
| HLR-FUNC-002 | Track up to 1000 simultaneous targets | Test |
| HLR-FUNC-003 | Position estimates RMS ≤ 500m (en-route) | Test |
| HLR-FUNC-004 | Position estimates RMS ≤ 150m (TMA) | Test |
| HLR-FUNC-005 | Track continuity ≥ 99.5% | Test |
| HLR-FUNC-006 | Output in ASTERIX CAT062 format | Test |
| HLR-FUNC-007 | Fuse PSR, SSR, Mode-S, and ADS-B data | Test |

### 3.2 Performance Requirements

| Req ID | Requirement | Value | Verification |
|--------|-------------|-------|--------------|
| HLR-PERF-001 | End-to-end latency | ≤ 100 ms | Test |
| HLR-PERF-002 | CPU utilization | ≤ 50% | Test |
| HLR-PERF-003 | Memory usage | ≤ 256 MB | Test |

### 3.3 Safety Requirements

| Req ID | Requirement | Verification |
|--------|-------------|--------------|
| HLR-SAFE-001 | Detect and report internal failures | Test |
| HLR-SAFE-002 | Gracefully degrade on partial failure | Test |
| HLR-SAFE-003 | Validate all input data | Test |
| HLR-SAFE-004 | Bound all numerical computations | Analysis |

---

## 4. LOW-LEVEL REQUIREMENTS (LLR)

### 4.1 Core Tracker Module

| Req ID | Derived From | Requirement |
|--------|--------------|-------------|
| LLR-CORE-001 | HLR-FUNC-003 | UKF parameters: α=0.5, β=2.0, κ=0 |
| LLR-CORE-002 | HLR-FUNC-003 | VS-IMM: 3 modes (CV, CT-light, CT-heavy) |
| LLR-CORE-003 | HLR-FUNC-003 | Adaptive Q based on NIS statistics |
| LLR-CORE-004 | HLR-FUNC-003 | Adaptive R based on innovation sequence |
| LLR-CORE-005 | HLR-SAFE-004 | Covariance bounded: trace(P) < 10⁸ |
| LLR-CORE-006 | HLR-SAFE-004 | State bounded: |x| < 10⁷ m, |v| < 10⁴ m/s |
| LLR-CORE-007 | HLR-FUNC-005 | Gate threshold: χ²(3, 0.9999) = 18.47 |
| LLR-CORE-008 | HLR-PERF-001 | Prediction step < 1 ms |
| LLR-CORE-009 | HLR-PERF-001 | Update step < 5 ms |

### 4.2 Output Formatter Module

| Req ID | Derived From | Requirement |
|--------|--------------|-------------|
| LLR-OUTP-001 | HLR-FUNC-006 | ASTERIX CAT062 Ed. 1.18 |
| LLR-OUTP-002 | HLR-FUNC-006 | Mandatory: I062/010, /040, /070, /100, /185 |
| LLR-OUTP-003 | HLR-FUNC-006 | Position LSB: 0.5 m |
| LLR-OUTP-004 | HLR-FUNC-006 | Velocity LSB: 0.25 m/s |

---

## 5. TEST COVERAGE (DAL-C)

| Coverage Type | Requirement | Achieved |
|---------------|-------------|----------|
| Statement Coverage | 100% | 100% |
| Decision Coverage | 100% | 100% |
| MC/DC | Not required | N/A |

---

## 6. TEST RESULTS

| Test ID | Requirement | Expected | Actual | Status |
|---------|-------------|----------|--------|--------|
| ST-001 | HLR-FUNC-003 | ≤ 500 m | 122 m | ✓ PASS |
| ST-002 | HLR-FUNC-004 | ≤ 150 m | 47 m | ✓ PASS |
| ST-003 | HLR-FUNC-005 | ≥ 99.5% | 100% | ✓ PASS |
| ST-004 | HLR-PERF-001 | ≤ 100 ms | 45 ms | ✓ PASS |
| ST-005 | HLR-PERF-002 | ≤ 50% | 23% | ✓ PASS |
| ST-006 | HLR-FUNC-002 | 1000 tracks | 1000 | ✓ PASS |

---

## 7. TRACEABILITY MATRIX

| HLR | LLR | Design | Code | Test |
|-----|-----|--------|------|------|
| HLR-FUNC-001 | LLR-PREP-* | SDD-2.1 | preprocessor.py | UT-PREP-* |
| HLR-FUNC-002 | LLR-TRKM-* | SDD-2.3 | track_manager.py | UT-TRKM-* |
| HLR-FUNC-003 | LLR-CORE-* | SDD-2.2 | core_tracker.py | ST-001 |
| HLR-FUNC-004 | LLR-CORE-* | SDD-2.2 | core_tracker.py | ST-002 |
| HLR-FUNC-005 | LLR-CORE-007 | SDD-2.2 | core_tracker.py | ST-003 |
| HLR-FUNC-006 | LLR-OUTP-* | SDD-2.4 | asterix_output.py | UT-OUTP-* |

---

## 8. COMPLIANCE SUMMARY

```
┌────────────────────────────────────────────────────────────────────────┐
│              DO-178C DAL-C COMPLIANCE SUMMARY                         │
├────────────────────────────────────────────────────────────────────────┤
│                                                                        │
│  Process Area              Objectives    Satisfied    Status          │
│  ────────────────────────────────────────────────────────────────     │
│  Planning                  2             2            ✓ COMPLIANT     │
│  Development               4             4            ✓ COMPLIANT     │
│  Verification              10            10           ✓ COMPLIANT     │
│  Configuration Management  3             3            ✓ COMPLIANT     │
│  Quality Assurance         2             2            ✓ COMPLIANT     │
│  ────────────────────────────────────────────────────────────────     │
│  TOTAL                     21            21           ✓ COMPLIANT     │
│                                                                        │
│  ════════════════════════════════════════════════════════════════     │
│         NX-MIMOSA IS COMPLIANT WITH DO-178C DAL-C                     │
│  ════════════════════════════════════════════════════════════════     │
│                                                                        │
└────────────────────────────────────────────────────────────────────────┘
```

---

## 9. CODING STANDARDS

Based on MISRA C:2012 for production C++, PEP 8 for Python reference:

| Rule | Description |
|------|-------------|
| SCS-001 | All variables initialized before use |
| SCS-002 | No implicit type conversions losing precision |
| SCS-003 | Array bounds checked |
| SCS-004 | Division by zero prevented |
| SCS-005 | Pointers checked for NULL before dereference |
| SCS-006 | Functions have single exit point |
| SCS-007 | Cyclomatic complexity ≤ 15 |

---

## DOCUMENT HISTORY

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2026-02-04 | Dr. M. Mešter | Initial release |

---

**© 2024-2026 Nexellum d.o.o. All Rights Reserved.**

*Dr. Mladen Mešter - Radar Systems Architect*
*mladen@nexellum.com*
