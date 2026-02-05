"""
nx_mimosa_config.py - Auto-generated from system_config.yaml

System: NX-MIMOSA v1.1.0
Generated: 2026-02-05 01:55:15

DO NOT EDIT MANUALLY - Regenerate from YAML
"""

from dataclasses import dataclass
from typing import List, Dict, Any


# ═══════════════════════════════════════════════════════════════════════════════
# RADAR PARAMETERS
# ═══════════════════════════════════════════════════════════════════════════════
RADAR_FS = 100000000
RADAR_FC = 10000000000
RADAR_BANDWIDTH = 10000000
RADAR_PRF = 1000
RADAR_MAX_RANGE = 150000

# ═══════════════════════════════════════════════════════════════════════════════
# CFAR PARAMETERS
# ═══════════════════════════════════════════════════════════════════════════════
CFAR_WINDOW_SIZE = 20
CFAR_GUARD_CELLS = 4
CFAR_THRESHOLD_FACTOR = 5.0

# ═══════════════════════════════════════════════════════════════════════════════
# COGNITIVE CFAR
# ═══════════════════════════════════════════════════════════════════════════════
COGNITIVE_ENABLED = True
SVM_NUM_SUPPORT_VECTORS = 64
SVM_GAMMA = 0.5
SVM_PROB_THRESHOLD = 0.95

# ═══════════════════════════════════════════════════════════════════════════════
# REGISTER MAP
# ═══════════════════════════════════════════════════════════════════════════════
BASE_ADDRESS = 0x80000000

REGISTERS = {
    'REG_CONTROL': {'offset': 0x00},
    'REG_STATUS': {'offset': 0x04},
    'REG_CFAR_CONFIG': {'offset': 0x08},
    'REG_TRACK_COUNT': {'offset': 0x0C},
    'REG_DETECTION_COUNT': {'offset': 0x10},
    'REG_VERSION': {'offset': 0xFC},
}
