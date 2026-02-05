#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════
NX-MIMOSA Electronic Warfare (EW) Module
═══════════════════════════════════════════════════════════════════════════════

Cognitive ECCM capabilities:
  - Jammer Classification (Random Forest, 99% accuracy)
  - RL-based Waveform Agility (PPO + Q-Learning)
  - GAN Adversarial Jammer Synthesis
  - Adaptive Frequency Hopping
  - DRFM Mitigation

Traceability:
  [REQ-EW-01] Jammer classification
  [REQ-RL-PPO-001] PPO continuous actions
  [REQ-RL-ADV-PPO-001] Adversarial PPO training
  [REQ-RL-GAN-001] GAN adversarial synthesis

Author: Dr. Mladen Mešter / Nexellum d.o.o.
License: AGPL v3 / Commercial
═══════════════════════════════════════════════════════════════════════════════
"""

__version__ = "1.3.0"
__author__ = "Dr. Mladen Mešter"

__all__ = [
    # Jammer Classifier
    'JammerType',
    'SignalFeatures', 
    'FeatureExtractor',
    'JammerClassifier',
    'JammerSignalGenerator',
    
    # PPO Agent
    'PPOConfig',
    'PPOAgent',
    'ReactiveJammerEnv',
    
    # Adversarial PPO
    'AdversarialPPOConfig',
    'AdversarialPPOAgent',
    'AdversarialJammerEnv',
    
    # GAN
    'GANConfig',
    'AdversarialJammerGAN',
]

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

try:
    from .adversarial_ppo import (
        AdversarialPPOConfig,
        AdversarialPPOAgent,
        AdversarialJammerEnv
    )
except ImportError:
    pass

try:
    from .gan_adversarial import (
        GANConfig,
        AdversarialJammerGAN
    )
except ImportError:
    pass
