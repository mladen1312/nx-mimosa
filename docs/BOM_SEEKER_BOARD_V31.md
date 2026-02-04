# ═══════════════════════════════════════════════════════════════════════════════
# NX-MIMOSA v3.1 SEEKER BOARD — BILL OF MATERIALS (BOM)
# ═══════════════════════════════════════════════════════════════════════════════
# [REQ-V31-COST-01] Production BOM for missile seeker application
# Target: AIM-120 class active radar seeker upgrade
# Author: Dr. Mladen Mešter / Nexellum d.o.o.
# ═══════════════════════════════════════════════════════════════════════════════

## 📋 SYSTEM OVERVIEW

| Parameter | Value |
|-----------|-------|
| Application | Active Radar Seeker (Missile Guidance) |
| Update Rate | 50 Hz |
| Tracker | NX-MIMOSA v3.1 (IMM + True Smoother) |
| Target Platform | XCZU48DR RFSoC |
| Operating Temp | -40°C to +85°C (Industrial) |
| Power Budget | < 25W total |

---

## 💰 BILL OF MATERIALS — PRODUCTION VOLUME PRICING

### Option A: RFSoC 4x2 Development Platform (Low Volume / Prototyping)

| # | Part Number | Description | Qty | Unit Cost | Ext Cost | Supplier | Lead Time | EOL Risk |
|---|-------------|-------------|-----|-----------|----------|----------|-----------|----------|
| 1 | RFSOC4X2 | AMD RFSoC 4x2 Board (ZU48DR) | 1 | $2,499 | $2,499 | AMD/Xilinx | 8 weeks | LOW |
| 2 | SMA-KIT-8 | SMA Cables & Adapters Kit | 1 | $150 | $150 | Mini-Circuits | 2 weeks | LOW |
| 3 | PS-12V-5A | 12V 5A Power Supply | 1 | $45 | $45 | Mean Well | 1 week | LOW |
| 4 | JTAG-HS3 | Digilent JTAG Programmer | 1 | $65 | $65 | Digilent | 1 week | LOW |
| 5 | SD-32GB-IND | Industrial SD Card 32GB | 1 | $28 | $28 | Delkin | 2 weeks | LOW |
| 6 | ENCL-ALU-200 | Aluminum Enclosure 200x150x50 | 1 | $85 | $85 | Hammond | 3 weeks | LOW |
| 7 | THERM-PAD | Thermal Interface Pad | 2 | $12 | $24 | 3M | 1 week | LOW |
| 8 | FAN-40MM | 40mm Cooling Fan | 2 | $8 | $16 | Sunon | 1 week | LOW |

**SUBTOTAL OPTION A: $2,912**

---

### Option B: Custom Seeker Board (Production Volume 100+ units)

| # | Part Number | Description | Qty | Unit Cost (@100) | Ext Cost | Supplier | Lead Time | EOL Risk | Alt Part |
|---|-------------|-------------|-----|------------------|----------|----------|-----------|----------|----------|
| **FPGA & Processing** |
| 1 | XCZU48DR-2FFVG1517I | Zynq UltraScale+ RFSoC | 1 | $4,850 | $4,850 | AMD/Xilinx | 16 weeks | LOW | XCZU47DR |
| 2 | MT40A1G16TB-062E | DDR4 16Gb SDRAM | 4 | $18 | $72 | Micron | 12 weeks | LOW | IS46DR16640D |
| 3 | MT25QU01GBBB | 1Gb QSPI Flash | 1 | $12 | $12 | Micron | 8 weeks | LOW | S25FL01GS |
| **Power Management** |
| 4 | LTM4700 | Dual 50A μModule DC/DC | 2 | $145 | $290 | Analog Devices | 14 weeks | MED | TPSM84824 |
| 5 | LT3045 | Ultra-Low Noise LDO | 8 | $6 | $48 | Analog Devices | 6 weeks | LOW | ADP7118 |
| 6 | LTC3310S | 10A Silent Switcher | 4 | $8 | $32 | Analog Devices | 6 weeks | LOW | TPS62913 |
| **RF Frontend** |
| 7 | HMC1040LP3CE | LNA 2-20GHz | 2 | $28 | $56 | Analog Devices | 10 weeks | LOW | MGA-633P8 |
| 8 | ADL5380 | Broadband I/Q Demod | 1 | $22 | $22 | Analog Devices | 8 weeks | LOW | LTC5588 |
| 9 | HMC407MS8G | PA Driver 5-18GHz | 1 | $45 | $45 | Analog Devices | 12 weeks | MED | TGA2595-SM |
| **Clocking** |
| 10 | LMK04828 | Dual PLL Clock Gen | 1 | $85 | $85 | Texas Instruments | 10 weeks | LOW | HMC7044 |
| 11 | SIT5356 | MEMS TCXO 100MHz | 1 | $35 | $35 | SiTime | 6 weeks | LOW | ASTMUPCD |
| **Connectors & Misc** |
| 12 | SMP-JACK | SMP RF Connectors | 8 | $8 | $64 | Amphenol | 4 weeks | LOW | - |
| 13 | SAMTEC-QTH | High-Speed Mezzanine | 2 | $45 | $90 | Samtec | 6 weeks | LOW | - |
| 14 | 10K-0402-1% | Precision Resistors Kit | 1 | $25 | $25 | Vishay | 2 weeks | LOW | - |
| 15 | MLCC-KIT | Capacitor Kit (0402-1206) | 1 | $80 | $80 | Murata | 2 weeks | LOW | - |
| **PCB** |
| 16 | PCB-12L-HYBRID | 12-layer Rogers/FR4 Hybrid | 1 | $450 | $450 | Advanced Circuits | 4 weeks | LOW | - |
| **Assembly** |
| 17 | ASSY-SMT | SMT Assembly | 1 | $300 | $300 | Tempo Automation | 3 weeks | - | - |

**SUBTOTAL OPTION B (per unit @100): $6,556**

---

### Option C: Volume Production (1000+ units)

| Category | Cost @100 | Cost @1000 | Cost @5000 | Notes |
|----------|-----------|------------|------------|-------|
| FPGA | $4,850 | $3,880 | $3,492 | -20% / -28% volume discount |
| DDR4 (4x) | $72 | $58 | $52 | -19% / -28% |
| Power ICs | $370 | $296 | $267 | -20% / -28% |
| RF Frontend | $123 | $98 | $89 | -20% / -28% |
| Clocking | $120 | $96 | $86 | -20% / -28% |
| Connectors | $179 | $143 | $129 | -20% / -28% |
| PCB | $450 | $180 | $90 | -60% / -80% (volume) |
| Assembly | $300 | $150 | $80 | -50% / -73% (volume) |
| **TOTAL** | **$6,556** | **$4,901** | **$4,285** | |

---

## 📊 COST BREAKDOWN ANALYSIS

```
                    COST BREAKDOWN (Option B @100 units)
    ┌────────────────────────────────────────────────────────────┐
    │  FPGA (74%)        ████████████████████████████████░░░░░░  │
    │  Power (6%)        ████░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  │
    │  PCB+Assy (11%)    ██████░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  │
    │  RF (2%)           █░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  │
    │  Other (7%)        ████░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  │
    └────────────────────────────────────────────────────────────┘
    
    FPGA dominates cost → Volume discount critical!
```

---

## ⚠️ SUPPLY CHAIN RISK ASSESSMENT

| Part | Risk Level | Mitigation | Alternative |
|------|------------|------------|-------------|
| XCZU48DR | 🟡 MEDIUM | Multi-source AMD, 6-month buffer stock | XCZU47DR (-1 speed grade) |
| LTM4700 | 🟡 MEDIUM | ADI allocation program | TPSM84824 (TI) |
| HMC407MS8G | 🟡 MEDIUM | 12-week lead time, pre-order | TGA2595-SM (Qorvo) |
| DDR4 | 🟢 LOW | Multi-vendor (Micron, Samsung, SK) | - |
| PCB | 🟢 LOW | Multiple qualified fabs | - |

### EOL Watch List
- **HMC407MS8G**: Older Hittite part, monitor for EOL notice
- **LMK04828**: Mature product, stable supply

---

## 💵 PRICING SUMMARY

| Volume | Unit Cost | NRE | Total Program Cost | Per-Unit Delivered |
|--------|-----------|-----|--------------------|--------------------|
| 1 (Dev) | $2,912 | $0 | $2,912 | $2,912 |
| 10 | $6,556 | $25,000 | $90,560 | $9,056 |
| 100 | $6,556 | $50,000 | $705,600 | $7,056 |
| 1,000 | $4,901 | $75,000 | $4,976,000 | $4,976 |
| 5,000 | $4,285 | $100,000 | $21,525,000 | $4,305 |

**NRE includes:** Schematic, Layout, Firmware, Testing, Qualification

---

## 🎯 ROI ANALYSIS (Defense Contract Scenario)

| Parameter | Value |
|-----------|-------|
| Contract Volume | 1,000 units |
| Hardware Cost | $4,901/unit |
| Software License | $5,000/unit (NX-MIMOSA v3.1) |
| Integration & Test | $2,000/unit |
| **Total Unit Cost** | **$11,901** |
| Sell Price | $25,000/unit |
| **Gross Margin** | **52%** |
| **Total Revenue** | **$25,000,000** |
| **Gross Profit** | **$13,099,000** |

---

## 📞 RECOMMENDED SUPPLIERS

| Category | Primary | Secondary | Notes |
|----------|---------|-----------|-------|
| FPGA | AMD Direct | Avnet | AMD for mil-spec, Avnet for commercial |
| RF | Analog Devices | Qorvo | ADI single-source for most RF |
| Power | Analog Devices | Texas Instruments | LTM4700 sole-source concern |
| Memory | Micron Direct | Mouser | Micron for mil-grade |
| PCB | Advanced Circuits | TTM Technologies | US-based ITAR compliant |
| Assembly | Tempo Automation | Sanmina | Tempo for prototype, Sanmina for volume |

---

## ✅ BOM VALIDATION CHECKLIST

- [x] All parts available (no discontinued)
- [x] Lead times < 16 weeks for critical path
- [x] Alternative parts identified for single-source
- [x] Temperature range -40°C to +85°C verified
- [x] ITAR/Export control reviewed (N/A for civilian version)
- [x] Conflict minerals compliance (3TG)

---

*Generated by Radar Systems Architect v9.0 — Cost & Supply Lead*
*Date: February 4, 2026*

**Contact:** mladen@nexellum.com | +385 99 737 5100
