# NX-MIMOSA v2.0 — Improvement Plan

## 📊 Current State Analysis

### RTL Structure (2,147 LOC)
| File | LOC | Function | Status |
|------|-----|----------|--------|
| nx_mimosa_pkg.sv | 166 | Package + Q15.16 fixed-point | ✅ Solid |
| imm_core.sv | 458 | 3-model IMM mixing | ⚠️ Needs upgrade |
| kalman_filter_core.sv | 400 | Linear KF + Joseph form | ⚠️ Only linear |
| fixed_lag_smoother.sv | 427 | Per-model RTS smoother | ✅ Innovative |
| nx_mimosa_top.sv | 390 | Top-level integration | ✅ OK |

### Current Limitations
1. **Linear-only KF** — fails with polar/spherical measurements
2. **Fixed Q** — no adaptation to target dynamics
3. **Fixed TPM** — slow model switching during maneuvers
4. **3 models only** — missing CA and Jerk models

---

## 🎯 Proposed Improvements

### TIER 1: High-Impact (Implement Now)

#### 1.1 Adaptive Process Noise (NIS-based)
**Impact: +15-20% RMSE improvement**

```python
# Normalized Innovation Squared
NIS = y.T @ S_inv @ y

if NIS > chi2_threshold:
    Q *= alpha_increase  # 1.5-2.0x
else:
    Q *= alpha_decrease  # 0.95x (slow return)
```

**RTL Change**: Add NIS computation in kalman_filter_core.sv

#### 1.2 Dynamic Mode Persistence
**Impact: +10-15% RMSE improvement**

```python
# VS-IMM: Increase persistence when confident
if max(mu) > 0.9:
    p_stay = 0.99  # High confidence → stay
else:
    p_stay = 0.90  # Uncertain → allow switching
```

**RTL Change**: Add dynamic PI matrix in imm_core.sv

#### 1.3 4-Model Set (CV + CA + CT+ + CT-)
**Impact: +8-12% RMSE improvement on accelerating targets**

Add Constant Acceleration model:
```
F_CA = [1, 0, dt, 0,  dt²/2, 0    ]
       [0, 1, 0,  dt, 0,     dt²/2]
       [0, 0, 1,  0,  dt,    0    ]
       [0, 0, 0,  1,  0,     dt   ]
       [0, 0, 0,  0,  1,     0    ]
       [0, 0, 0,  0,  0,     1    ]
```

**RTL Change**: Expand STATE_DIM to 6, add F[3] matrix

---

### TIER 2: Medium-Impact (Next Version)

#### 2.1 Unscented Kalman Filter (UKF)
**Impact: +5-10% for nonlinear measurements**

```python
# Sigma points
n = STATE_DIM
kappa = 3 - n
lambda_ = alpha**2 * (n + kappa) - n

# Generate 2n+1 sigma points
chi[0] = x
for i in range(n):
    chi[i+1] = x + sqrt((n + lambda_) * P)[:, i]
    chi[n+i+1] = x - sqrt((n + lambda_) * P)[:, i]

# Transform through nonlinear function
gamma = [h(chi_i) for chi_i in chi]

# Weighted recombination
z_pred = sum(Wm[i] * gamma[i])
```

**RTL Change**: New ukf_core.sv with sigma point generation

#### 2.2 Cubature Kalman Filter (CKF)
**Impact: Similar to UKF, better numerical stability**

Uses spherical-radial cubature rule:
- 2n points instead of 2n+1
- No tuning parameters (α, β, κ)
- Better for high-dimensional systems

---

### TIER 3: Future (Research)

#### 3.1 PMBM (Poisson Multi-Bernoulli Mixture)
Full multi-target tracking with birth/death

#### 3.2 ML-based Parameter Tuning
Neural network for Q, R, TPM optimization

#### 3.3 Adaptive Turn Rate Estimation
Real-time ω estimation from measurements

---

## 📐 Performance Projections

| Configuration | RMSE (6g turn) | Improvement |
|---------------|----------------|-------------|
| v1.0 Baseline | 2.39m | — |
| + Adaptive Q | 2.03m | +15% |
| + Dynamic TPM | 1.79m | +25% |
| + 4-Model CA | 1.65m | +31% |
| + UKF (polar) | 1.57m | +34% |
| **v2.0 Full** | **~1.5m** | **+37%** |

---

## 📁 Implementation Files

```
nx-mimosa-v2/
├── rtl/
│   ├── nx_mimosa_pkg_v2.sv      # Extended package (6D state)
│   ├── adaptive_q_module.sv     # NIS-based Q adaptation
│   ├── dynamic_tpm_module.sv    # VS-IMM dynamic switching
│   ├── ukf_core.sv              # Unscented Kalman Filter
│   ├── ckf_core.sv              # Cubature Kalman Filter
│   ├── imm_core_v2.sv           # 4-model IMM
│   └── model_ca.sv              # Constant Acceleration model
├── python/
│   └── nx_mimosa_v2_reference.py
├── sim/
│   └── tb_nx_mimosa_v2.sv
└── docs/
    └── IMPROVEMENT_PLAN.md
```
