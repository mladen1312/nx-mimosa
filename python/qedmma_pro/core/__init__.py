"""
QEDMMA-Pro Core Tracking Algorithms
====================================
Copyright (C) 2026 Dr. Mladen Mešter / Nexellum
License: Commercial - See LICENSE_COMMERCIAL.md

Core modules (enhanced versions of Lite algorithms):
- UKF-Pro: Square-root, adaptive, iterated, constrained
- CKF-Pro: Higher-order cubature, chi² gating, adaptive noise
- Adaptive Noise Pro: ML-based, multi-hypothesis

For licensing: mladen@nexellum.com
"""

from .ukf_pro import UKFPro, UKFProParams, UKFProState, UKFVariant
from .ukf_pro import create_hypersonic_ukf_pro

from .ckf_pro import CKFPro, CKFProParams, CKFProState, CKFOrder
from .ckf_pro import create_high_dim_ckf

__all__ = [
    # UKF-Pro
    'UKFPro', 'UKFProParams', 'UKFProState', 'UKFVariant',
    'create_hypersonic_ukf_pro',
    
    # CKF-Pro  
    'CKFPro', 'CKFProParams', 'CKFProState', 'CKFOrder',
    'create_high_dim_ckf',
]
