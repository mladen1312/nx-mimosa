"""
QEDMMA-Pro v3.0 - Commercial Radar Tracking Suite
Copyright (C) 2026 Dr. Mladen Mešter / Nexellum
License: Commercial

For licensing: mladen@nexellum.com | www.nexellum.com
"""

__version__ = "3.0.0"
__author__ = "Dr. Mladen Mešter"
__email__ = "mladen@nexellum.com"

from .core.ukf_pro import UKFPro, UKFProParams, UKFProState, create_hypersonic_ukf
from .exclusive.anomaly_hunter import AnomalyHunter, AnomalyConfig, AnomalyLevel

__all__ = [
    "__version__", "UKFPro", "UKFProParams", "UKFProState", 
    "create_hypersonic_ukf", "AnomalyHunter", "AnomalyConfig", "AnomalyLevel"
]
