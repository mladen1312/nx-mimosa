"""
QEDMMA-Pro v3.0 - Enterprise Radar Tracking Suite
==================================================
Copyright (C) 2026 Dr. Mladen Mešter / Nexellum
License: Commercial - See LICENSE_COMMERCIAL.md

QEDMMA-Pro is the commercial extension of QEDMMA-Lite, providing
enterprise-grade tracking capabilities:

┌─────────────────────────────────────────────────────────────────┐
│                    QEDMMA-Pro Features                          │
├─────────────────────────────────────────────────────────────────┤
│ ✅ All QEDMMA-Lite features (IMM, UKF, CKF, Zero-DSP)          │
│ ✅ PRO: UKF-Pro (SR-UKF, Adaptive, IUKF, Constraints)          │
│ ✅ PRO: CKF-Pro (5th Order, Chi² Gating, Adaptive Q/R)         │
│ ✅ PRO: Anomaly Hunter™ (Physics-Agnostic Layer 2B)            │
│ ✅ PRO: Multi-Sensor Fusion (JDL Model, Async Compensation)    │
│ ✅ PRO: FPGA IP Cores (Vivado/Vitis ready)                     │
│ ✅ PRO: DO-254/DO-178C Certification Artifacts                  │
│ ✅ PRO: Priority Support + SLA                                  │
└─────────────────────────────────────────────────────────────────┘

Quick Start:
    >>> from qedmma_pro.core import UKFPro, create_hypersonic_ukf
    >>> from qedmma_pro.exclusive import AnomalyHunter, MultiSensorFusion
    >>> 
    >>> # Create hypersonic tracker
    >>> ukf, state = create_hypersonic_ukf()
    >>> 
    >>> # Track with Anomaly Hunter
    >>> hunter = AnomalyHunter()
    >>> hunter_state = hunter.process(target_id=1, ...)

For licensing: mladen@nexellum.com | www.nexellum.com
Phone: +385 99 737 5100
"""

__version__ = "3.0.0"
__author__ = "Dr. Mladen Mešter"
__email__ = "mladen@nexellum.com"
__license__ = "Commercial"

# Core tracking (enhanced versions)
from .core import (
    UKFPro, UKFProParams, UKFProState, UKFVariant,
    CKFPro, CKFProParams, CKFProState, CKFOrder,
    create_hypersonic_ukf_pro,
    create_high_dim_ckf,
)

# PRO Exclusive
from .exclusive import (
    AnomalyHunter, AnomalyHunterConfig, AnomalyHunterState, AnomalyLevel,
    MultiSensorFusion, SensorConfig, SensorType, Measurement, FusedTrack, JDLLevel,
)

__all__ = [
    # Version info
    '__version__', '__author__', '__email__', '__license__',
    
    # Core - UKF-Pro
    'UKFPro', 'UKFProParams', 'UKFProState', 'UKFVariant',
    'create_hypersonic_ukf_pro',
    
    # Core - CKF-Pro
    'CKFPro', 'CKFProParams', 'CKFProState', 'CKFOrder',
    'create_high_dim_ckf',
    
    # Exclusive - Anomaly Hunter
    'AnomalyHunter', 'AnomalyHunterConfig', 'AnomalyHunterState', 'AnomalyLevel',
    
    # Exclusive - Multi-Sensor Fusion
    'MultiSensorFusion', 'SensorConfig', 'SensorType', 
    'Measurement', 'FusedTrack', 'JDLLevel',
]


def show_features():
    """Display QEDMMA-Pro feature comparison"""
    print("""
╔══════════════════════════════════════════════════════════════════════════════╗
║                         QEDMMA-Pro v3.0 Features                             ║
╠══════════════════════════════════════════════════════════════════════════════╣
║                                                                              ║
║  Feature                              │ Lite      │ Pro                      ║
║  ─────────────────────────────────────┼───────────┼──────────────────────────║
║  IMM/UKF/CKF Tracking                 │ ✅        │ ✅ (Enhanced)            ║
║  Zero-DSP Correlator                  │ ✅        │ ✅ (FPGA IP)             ║
║  Adaptive Noise Estimation            │ ✅        │ ✅ (ML-based)            ║
║  Square-Root Formulation              │ ❌        │ ✅                       ║
║  Adaptive Sigma Scaling               │ ❌        │ ✅                       ║
║  State Constraints                    │ ❌        │ ✅                       ║
║  Iterated Updates (IUKF)              │ ❌        │ ✅                       ║
║  Anomaly Hunter™ (Layer 2B)           │ ❌        │ ✅                       ║
║  Multi-Sensor Fusion (JDL)            │ ❌        │ ✅                       ║
║  FPGA Bitstreams                      │ ❌        │ ✅                       ║
║  DO-254 Certification                 │ ❌        │ ✅                       ║
║  Priority Support                     │ ❌        │ ✅                       ║
║                                                                              ║
║  Hypersonic RMSE (Mach 5+)            │ ~95m      │ <50m                     ║
║                                                                              ║
╚══════════════════════════════════════════════════════════════════════════════╝

📧 Contact: mladen@nexellum.com
🌐 Web: www.nexellum.com
""")
