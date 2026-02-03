# QEDMMA-Pro v3.1.0

**Commercial License — Nexellum d.o.o.**

Production-grade IMM tracker with True IMM Smoother and FPGA RTL for defense applications.

## Features (Pro-only)

- ✅ True IMM Smoother (+48% RMSE improvement)
- ✅ Fixed-Lag Smoother for real-time guidance
- ✅ FPGA RTL (SystemVerilog) for Xilinx RFSoC
- ✅ DO-254 DAL C certification package
- ✅ UKF support for polar measurements
- ✅ Multi-target tracking (1024 simultaneous)

## Performance

| Scenario | Filter RMSE | Smoother RMSE | Improvement |
|----------|-------------|---------------|-------------|
| 3g Turn | 1.81m | 0.88m | +51.1% |
| 6g Turn | 1.91m | 0.98m | +48.5% |
| 9g Turn | 1.95m | 1.02m | +47.7% |

## FPGA Resources (ZU28DR)

| Resource | Used | Available | Utilization |
|----------|------|-----------|-------------|
| LUT | 14,700 | 425,280 | 3.5% |
| DSP48E2 | 48 | 4,272 | 1.1% |
| BRAM36 | 40 | 1,080 | 3.7% |

## License

Commercial license required. Contact for pricing:

- **Email**: mladen@nexellum.com
- **Phone**: +385 99 737 5100

## Pricing

| Tier | Price | Includes |
|------|-------|----------|
| Development | $15,000 | RTL + Python + 1 year support |
| Production | $50,000 | Unlimited deployment + DO-254 pkg |
| Enterprise | $150,000 | Source + customization + training |

---

*Nexellum d.o.o. — Advanced Defense Technology Systems*
*ITAR/EAR Controlled*
