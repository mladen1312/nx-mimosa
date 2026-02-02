# QEDMMA Product Comparison

## Open Source vs Commercial Editions

| Feature | QEDMMA-Lite (Free) | QEDMMA-Pro™ (Commercial) |
|---------|-------------------|--------------------------|
| **License** | MIT (Open Source) | Commercial License |
| **Price** | FREE | Starting $50,000 |

### Core Algorithm

| Feature | Lite | Pro |
|---------|------|-----|
| IMM Filter (2 models) | ✅ | ✅ |
| IMM Filter (4 models) | ✅ | ✅ |
| Constant Velocity Model | ✅ | ✅ |
| Constant Acceleration Model | ✅ | ✅ |
| Coordinated Turn Model | ✅ | ✅ |
| Constant Jerk Model | ✅ | ✅ |
| Basic Kalman Filter | ✅ | ✅ |
| Extended Kalman Filter | ✅ | ✅ |
| **Deep Physics Layer (Layer 2A)** | ❌ | ✅ |
| **Anomaly Hunter™ (Layer 2B)** | ❌ | ✅ |
| **Anomaly Divergence Monitor** | ❌ | ✅ |

### TDOA Fusion

| Feature | Lite | Pro |
|---------|------|-----|
| Basic TDOA Solver | ✅ | ✅ |
| Gauss-Newton Optimization | ✅ | ✅ |
| Doppler Fusion | ❌ | ✅ |
| **Clock-Bias Estimation** | ❌ | ✅ |
| **Async Network Support** | ❌ | ✅ |
| Max Nodes | 6 | 16 |

### FPGA/Hardware

| Feature | Lite | Pro |
|---------|------|-----|
| Python Reference | ✅ | ✅ |
| C++ Reference | ✅ | ✅ |
| **Synthesizable RTL** | ❌ | ✅ |
| **Fixed-Point Optimized** | ❌ | ✅ |
| **AXI4-Stream Interface** | ❌ | ✅ |
| **Timing Constraints** | ❌ | ✅ |
| AMD RFSoC Support | ❌ | ✅ |
| Intel FPGA Support | ❌ | ✅ |

### Performance

| Metric | Lite | Pro |
|--------|------|-----|
| Position RMSE (Mach 8, 60g) | ~200m | **< 50m** |
| Max Trackable G-Load | 60g | **100g+** |
| Anomaly Detection | ❌ | ✅ |
| Real-Time FPGA | ❌ | **< 1 μs** |
| Unconventional Target Tracking | ❌ | ✅ |

### Support & Services

| Feature | Lite | Pro |
|---------|------|-----|
| Community Support (GitHub) | ✅ | ✅ |
| Documentation | Basic | Comprehensive |
| **Email Support** | ❌ | ✅ (12 months) |
| **Phone Support** | ❌ | ✅ (Enterprise) |
| **Integration Assistance** | ❌ | ✅ |
| **Custom Development** | ❌ | Available |
| **On-Site Training** | ❌ | Available |

### Compliance & Certification

| Feature | Lite | Pro |
|---------|------|-----|
| DO-254 Traceability | ❌ | ✅ |
| MISRA Compliance | ❌ | ✅ |
| Export Control Docs | ❌ | ✅ |
| Safety Analysis | ❌ | Available |

---

## When to Choose Each Edition

### Choose QEDMMA-Lite (Free) If You:

- Are a **researcher or student** exploring tracking algorithms
- Need a **benchmark tool** to compare against your own algorithms
- Want to **evaluate** QEDMMA before committing to commercial license
- Are building a **prototype** or proof-of-concept
- Have **limited budget** and software-only requirements
- Don't need real-time FPGA performance
- Are tracking **conventional targets** only

### Choose QEDMMA-Pro™ If You:

- Need **production-ready FPGA IP** for deployment
- Require **real-time performance** (< 1 μs latency)
- Must track **hypersonic threats** with high accuracy
- Need **anomaly detection** for unconventional targets
- Operating **distributed radar networks** with timing challenges
- Require **DO-254 certification** support
- Need **commercial support** and SLA guarantees
- Are a **defense contractor** or government agency

---

## Upgrade Path

```
┌──────────────────┐
│  QEDMMA-Lite     │     FREE
│  (Evaluation)    │     ─────────────────────────────────────────
└────────┬─────────┘
         │
         │  Like what you see?
         │
         ▼
┌──────────────────┐
│  QEDMMA-Pro      │     $50,000+ per project
│  (Single Project)│     ─────────────────────────────────────────
└────────┬─────────┘
         │
         │  Need more flexibility?
         │
         ▼
┌──────────────────┐
│  QEDMMA-Pro      │     $200,000 (unlimited products)
│  (Enterprise)    │     ─────────────────────────────────────────
└────────┬─────────┘
         │
         │  Full suite?
         │
         ▼
┌──────────────────┐
│  Full Suite      │     $350,000
│  + Anomaly Hunter│     ─────────────────────────────────────────
│  + Async Fusion  │
└──────────────────┘
```

---

## ROI Calculator

### Scenario: Defense Radar System

**Without QEDMMA-Pro:**
- Track loss rate: 20%
- Re-acquisition cost per loss: $10,000 (operator time, missed intercepts)
- Losses per year: 500
- Annual cost of track loss: **$5,000,000**

**With QEDMMA-Pro:**
- Track loss rate: < 1%
- Losses per year: 25
- Annual cost of track loss: **$250,000**
- **Annual Savings: $4,750,000**

**License Cost: $200,000 (one-time)**
**ROI: 23.75x in Year 1**

---

### Scenario: Automotive Radar (ADAS)

**Without QEDMMA-Pro:**
- External tracking IP license: $5/unit
- Units per year: 1,000,000
- Annual cost: **$5,000,000**

**With QEDMMA-Pro Enterprise:**
- License: $200,000 (one-time)
- Per-unit cost: **$0**
- **Break-even: 40,000 units**
- **Year 1 Savings: $4,800,000**

---

## Contact Sales

Ready to upgrade to QEDMMA-Pro? Contact our sales team:

- **Email:** mladen@nexellum.com
- **Phone:** +385 XX XXX XXXX
- **Web:** https://www.nexellum.com/contact

**Request a Demo:** https://www.nexellum.com/demo

---

*© 2026 Nexellum. QEDMMA-Pro™ and Anomaly Hunter™ are trademarks of Nexellum.*
