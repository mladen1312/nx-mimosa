#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════════════════════════════
NX-MIMOSA Electronic Warfare (EW) Module
═══════════════════════════════════════════════════════════════════════════════════════════════════════

Cognitive ECCM capabilities:
  - Jammer Classification (Random Forest / CNN)
  - RL-based Waveform Agility (PPO / Q-Learning)
  - Adaptive Frequency Hopping
  - DRFM Mitigation

Modules:
  - jammer_classifier: ML-based jammer type classification
  - ppo_agent: PPO reinforcement learning agent
  - eccm_pipeline: Integrated ECCM processing pipeline

Author: Dr. Mladen Mešter / Nexellum d.o.o.
License: AGPL v3 / Commercial
═══════════════════════════════════════════════════════════════════════════════════════════════════════
"""

__version__ = "1.1.0"
__author__ = "Dr. Mladen Mešter"

# Module exports
__all__ = [
    'JammerType',
    'SignalFeatures',
    'FeatureExtractor',
    'JammerClassifier',
    'JammerSignalGenerator',
    'PPOConfig',
    'PPOAgent',
    'ReactiveJammerEnv',
    'ECCMPipeline'
]

# Conditional imports
try:
    from .jammer_classifier import (
        JammerType,
        SignalFeatures,
        FeatureExtractor,
        JammerClassifier,
        JammerSignalGenerator
    )
except ImportError:
    pass

try:
    from .ppo_agent import (
        PPOConfig,
        PPOAgent,
        ReactiveJammerEnv
    )
except ImportError:
    pass
