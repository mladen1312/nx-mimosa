"""
QEDMMA-Pro Exclusive Modules
=============================
Copyright (C) 2026 Dr. Mladen Mešter / Nexellum
License: Commercial - See LICENSE_COMMERCIAL.md

🔒 PRO EXCLUSIVE - These modules are NOT available in QEDMMA-Lite

Exclusive modules:
- Anomaly Hunter™: Physics-agnostic tracking (Layer 2B)
- Multi-Sensor Fusion: JDL-compliant distributed tracking
- Neural CFAR: AI-adaptive detection (coming soon)

For licensing: mladen@nexellum.com
"""

from .anomaly_hunter import (
    AnomalyHunter,
    AnomalyHunterConfig,
    AnomalyHunterState,
    AnomalyLevel,
    TargetHistory
)

from .multi_fusion import (
    MultiSensorFusion,
    SensorConfig,
    SensorType,
    Measurement,
    FusedTrack,
    JDLLevel
)

__all__ = [
    # Anomaly Hunter
    'AnomalyHunter', 'AnomalyHunterConfig', 'AnomalyHunterState',
    'AnomalyLevel', 'TargetHistory',
    
    # Multi-Sensor Fusion
    'MultiSensorFusion', 'SensorConfig', 'SensorType',
    'Measurement', 'FusedTrack', 'JDLLevel',
]
