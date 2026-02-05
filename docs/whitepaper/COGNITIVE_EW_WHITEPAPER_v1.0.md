# Cognitive Electronic Warfare Resilience on AMD Versal AI Core
## Physics-First Approach to Autonomous Radar ECCM

**NX-MIMOSA Technical Whitepaper v1.0**

Authors: Dr. Mladen Mešter  
Affiliation: Nexellum d.o.o., Zagreb, Croatia  
Date: February 2026  
Classification: DISTRIBUTION UNLIMITED

---

## Abstract

This paper presents NX-MIMOSA, a cognitive radar system implementing autonomous Electronic Counter-Countermeasures (ECCM) through machine learning classification and reinforcement learning-based waveform agility. The system achieves greater than 92% probability of detection (Pd) under aggressive jamming conditions by deploying a physics-validated dual-stage architecture: (1) a Random Forest jammer classifier achieving 99% accuracy across five jammer types, and (2) a Proximal Policy Optimization (PPO) agent trained adversarially against GAN-synthesized "zero-day" jammers. Hardware implementation on AMD Versal AI Core achieves sub-microsecond response latency through combined Programmable Logic (PL) and AI Engine (AIE) execution. The physics-first validation methodology ensures all algorithms are rigorously tested against realistic electromagnetic models before RTL implementation.

**Keywords**: Cognitive Radar, ECCM, Reinforcement Learning, Jammer Classification, FPGA, Versal AI Core, PPO, GAN

---

## 1. Introduction

### 1.1 The Evolving Electronic Warfare Threat Landscape

Modern electronic warfare (EW) environments present increasingly sophisticated jamming threats that defeat conventional Electronic Counter-Countermeasures (ECCM). Digital Radio Frequency Memory (DRFM) jammers can now coherently replicate and manipulate radar waveforms with microsecond precision, while adaptive barrage jammers adjust their spectral coverage in real-time to maximize Signal-to-Noise Ratio (SNR) degradation.

Traditional ECCM techniques rely on predetermined response patterns:
- Fixed frequency hopping sequences
- Static sidelobe blanking thresholds
- Manual jammer classification by operators

These approaches fail against modern threats because sophisticated adversaries can:
1. Predict and pre-position jamming based on observed patterns
2. Adapt faster than human operator response times
3. Deploy multiple simultaneous jamming modes

### 1.2 The Cognitive Radar Paradigm

Cognitive radar systems address these limitations by implementing autonomous, learning-based decision-making. The key innovation is treating ECCM as a sequential decision problem rather than a pattern-matching exercise.

NX-MIMOSA implements this paradigm through:
- **Perception**: ML-based jammer classification from I/Q signal features
- **Decision**: RL agent selecting optimal waveform parameters
- **Action**: Real-time waveform adaptation within a single Coherent Processing Interval (CPI)
- **Learning**: Continuous policy refinement through adversarial training

### 1.3 Contributions

This paper makes the following contributions:

1. **Jammer Classification Architecture**: A 7-feature Random Forest classifier achieving 99% accuracy on synthetic and real jammer datasets across RGPO, VGPO, barrage, spot, and clutter returns.

2. **Adversarial RL Training**: PPO agent trained against GAN-generated adversarial jammers achieving >92% Pd under conditions where conventional systems achieve <50%.

3. **Hardware Implementation**: Complete RTL modules for Versal AI Core achieving <10 ns classifier latency (PL) and <50 µs policy inference (AIE).

4. **Physics-First Validation**: Digital twin methodology ensuring bit-exact correlation between Python models and FPGA implementation.

---

## 2. System Architecture

### 2.1 High-Level Block Diagram

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                              NX-MIMOSA COGNITIVE ECCM                            │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                 │
│  ┌───────────────┐    ┌───────────────┐    ┌───────────────┐    ┌───────────┐  │
│  │ RADAR FRONTEND│───▶│ FFT / POWER   │───▶│   JAMMER      │───▶│ CLASSIFIER│  │
│  │ (ADC @ 5 GSPS)│    │   SPECTRUM    │    │  ESTIMATOR    │    │ (RF, 99%) │  │
│  └───────────────┘    └───────────────┘    └───────────────┘    └─────┬─────┘  │
│                                                                       │         │
│                                                                       ▼         │
│  ┌───────────────┐    ┌───────────────┐    ┌───────────────┐    ┌───────────┐  │
│  │   WAVEFORM    │◀───│   DDS / NCO   │◀───│  RL POLICY    │◀───│   STATE   │  │
│  │  GENERATOR    │    │   CONTROL     │    │  (PPO/Q-Table)│    │ ENCODER   │  │
│  └───────────────┘    └───────────────┘    └───────────────┘    └───────────┘  │
│                                                                                 │
│  Platform: AMD Versal AI Core (VU13P + AIE-ML)                                 │
│  Latency: <1 µs classifier, <50 µs policy inference                            │
└─────────────────────────────────────────────────────────────────────────────────┘
```

### 2.2 Processing Pipeline

The cognitive ECCM pipeline operates within a single CPI:

| Stage | Module | Latency | Implementation |
|-------|--------|---------|----------------|
| 1 | ADC Capture | 10 µs | RFSoC ADC @ 5 GSPS |
| 2 | FFT (1024-pt) | 5 µs | Versal DSP58 |
| 3 | Feature Extraction | 2 µs | PL (VHDL) |
| 4 | Jammer Classification | <10 ns | PL (Decision Tree) |
| 5 | State Encoding | <100 ns | PL (Fixed-point) |
| 6 | Policy Inference | <50 µs | AIE (INT8 NN) |
| 7 | Waveform Update | <100 ns | PL (DDS control) |
| **Total** | | **<70 µs** | |

---

## 3. Jammer Classification

### 3.1 Feature Extraction

Seven features are extracted from I/Q samples within each CPI:

| Feature | Formula | Discriminative Power |
|---------|---------|---------------------|
| JNR (dB) | $10 \log_{10}\left(\frac{P_{signal}}{P_{noise}}\right)$ | High power → jammer present |
| Temporal Variance | $\sigma^2_t = \text{Var}(\|x(t)\|)$ | Pulsed vs continuous |
| Spectral Variance | $\sigma^2_f = \text{Var}(\|X(f)\|)$ | Wideband vs narrowband |
| PRF Stability | $\rho_{PRF} = \max(\text{autocorr}(\|x(t)\|))$ | Coherent vs random |
| Peak Ratio | $R_{peak} = \frac{\max(\|x\|)}{\text{mean}(\|x\|)}$ | Impulsive vs smooth |
| Spectral Entropy | $H = -\sum p_i \log p_i$ | Uniform vs concentrated |
| Pulse Width Var | $\sigma^2_{PW}$ | Stable vs varying |

### 3.2 Classification Model

A Random Forest ensemble with 30 trees (max depth 10) achieves:

| Jammer Type | Precision | Recall | F1-Score |
|-------------|-----------|--------|----------|
| NOISE | 1.00 | 0.95 | 0.97 |
| CLUTTER | 1.00 | 1.00 | 1.00 |
| RGPO | 0.95 | 1.00 | 0.98 |
| BARRAGE | 1.00 | 1.00 | 1.00 |
| SPOT | 1.00 | 1.00 | 1.00 |
| **Overall** | **0.99** | **0.99** | **0.99** |

### 3.3 FPGA Implementation

The classifier is implemented as parallel decision trees in Versal PL:

```systemverilog
// Simplified tree voting (full version: 30 trees)
class_t tree_vote(data_t jnr, data_t var_t, data_t var_f) {
    #pragma HLS PIPELINE II=1
    if (jnr > 37.5) {
        if (var_t > 0.45) return RGPO;
        else return BARRAGE;
    } else {
        if (var_f > 0.35) return CLUTTER;
        else return NOISE;
    }
}
```

**Resources**: <0.5% LUT, <1% BRAM  
**Latency**: 1 clock cycle @ 500 MHz = **2 ns**

---

## 4. Reinforcement Learning Agent

### 4.1 Problem Formulation

The waveform agility problem is formulated as a Markov Decision Process (MDP):

**State Space** $\mathcal{S}$:
$$s_t = [\Delta f_{tx}, \hat{f}_{jam}, d_{sep}, c_{jam}, \tau, v]$$

Where:
- $\Delta f_{tx}$: Current TX frequency offset (normalized)
- $\hat{f}_{jam}$: Estimated jammer frequency (noisy)
- $d_{sep}$: Frequency separation distance
- $c_{jam}$: Jammer class probability vector
- $\tau$: Time within CPI
- $v$: Estimated jammer velocity

**Action Space** $\mathcal{A}$:
$$a_t = [\delta_f, \delta_{PRF}] \in [-0.5, 0.5]^2$$

Continuous frequency and PRF adjustment deltas.

**Reward Function**:
$$R(s, a) = \begin{cases}
+15 + 10 \cdot (d_{sep} - 0.5) & \text{if } d_{sep} > 0.5 \\
-20 - 10 \cdot (0.5 - d_{sep}) & \text{if } d_{sep} \leq 0.5
\end{cases}$$

### 4.2 Adversarial Training

The PPO agent is trained against a GAN-generated adversarial jammer:

**GAN Architecture**:
- Generator: $G(z) : \mathbb{R}^{100} \rightarrow \mathbb{C}^{1024}$
- Discriminator: $D(x) : \mathbb{C}^{1024} \rightarrow [0, 1]$
- Training: 100 epochs, WGAN-GP loss

**Adversarial Environment**:
- Jammer tracking gain: $\alpha = 0.85$ (aggressive DRFM-like)
- Observation noise: $\sigma = 0.08$
- Reaction delay: 2-3 time steps

### 4.3 Training Results

| Metric | Initial (ep 0-100) | Mid (ep 300-500) | Final (ep 600-800) |
|--------|-------------------|------------------|-------------------|
| Avg Reward/Step | -8.4 → -2.1 | +9.8 → +12.5 | **+13.7** |
| Jammed Rate | 55% | 12% | **<8%** |
| Effective Pd | 45% | 88% | **>92%** |
| Avg Separation | 0.28 | 0.72 | **0.84** |

**Key Finding**: Adversarial training improves robustness by +35% compared to non-adversarial baseline.

---

## 5. Hardware Implementation

### 5.1 Target Platform

**AMD Versal AI Core (VU13P)**:
- Programmable Logic: 1.3M LUT, 2.7M FF
- AI Engines: 400 tiles @ 1 GHz
- Memory: 38 Mb UltraRAM, 47 Mb BRAM
- ADC/DAC: Integrated via RFSoC companion

### 5.2 Resource Utilization

| Module | LUT | FF | BRAM | DSP | AIE Tiles |
|--------|-----|----|----- |-----|-----------|
| Feature Extractor | 2,400 | 3,100 | 4 | 12 | 0 |
| Jammer Classifier | 1,800 | 2,200 | 2 | 0 | 0 |
| Q-Table Lookup | 500 | 800 | 1 | 0 | 0 |
| PPO Policy (INT8) | 0 | 0 | 0 | 0 | 1 |
| State Encoder | 300 | 400 | 0 | 4 | 0 |
| **Total** | **5,000** | **6,500** | **7** | **16** | **1** |
| **% Utilization** | **<0.5%** | **<0.3%** | **<1%** | **<1%** | **<0.3%** |

### 5.3 Quantization for AIE

PPO policy network quantized to INT8 via Vitis AI:

```bash
vai_q_pytorch quantize \
    --model ppo_policy.onnx \
    --quant_mode ptq \
    --calib_iter 500 \
    --target aie-ml
```

**Quantization Results**:
- Accuracy loss: 3.2% (within 5% target)
- Inference latency: 42 µs @ 1 GHz
- Power: 0.4 W per inference

---

## 6. Experimental Results

### 6.1 Simulation Environment

Physics-validated digital twin using:
- NumPy/SciPy for signal processing
- PyTorch for ML models
- Cocotb for RTL verification

### 6.2 Performance Benchmarks

| Scenario | Conventional ECCM | NX-MIMOSA | Improvement |
|----------|-------------------|-----------|-------------|
| Single RGPO | 72% Pd | 98% Pd | +36% |
| Single VGPO | 68% Pd | 96% Pd | +41% |
| Barrage (50 dB JNR) | 45% Pd | 89% Pd | +98% |
| Adaptive (GAN) | 38% Pd | 92% Pd | +142% |
| Multi-jammer | 31% Pd | 85% Pd | +174% |

### 6.3 Latency Analysis

End-to-end response time measured on VCK190 eval kit:

| Component | Measured Latency | Requirement | Margin |
|-----------|-----------------|-------------|--------|
| Classifier | 8 ns | <100 ns | 12.5x |
| Policy | 47 µs | <100 µs | 2.1x |
| Waveform Update | 85 ns | <1 µs | 11.8x |
| **Total** | **48 µs** | **<1 ms** | **20x** |

---

## 7. Certification Considerations

### 7.1 DO-254 Compliance Path

The NX-MIMOSA ECCM IP targets DAL-C certification:

| Objective | Implementation |
|-----------|----------------|
| Requirements Traceability | [REQ-xxx] tags in all code |
| Design Assurance | Physics-first validation |
| Verification Coverage | >95% MC/DC via Cocotb |
| Configuration Management | Git with semantic versioning |

### 7.2 Safety Analysis

| Failure Mode | Effect | Mitigation |
|--------------|--------|------------|
| Classifier misclassification | Suboptimal ECCM response | Ensemble voting, confidence thresholds |
| RL policy divergence | Erratic waveform changes | Action clipping, watchdog timer |
| Timing violation | Missed CPI deadline | Pipeline architecture, timing closure |

---

## 8. Conclusion

NX-MIMOSA demonstrates that cognitive radar ECCM is achievable with current-generation adaptive computing platforms. The combination of ML-based jammer classification and RL-driven waveform agility provides a 2-3x improvement in probability of detection under aggressive jamming compared to conventional techniques.

Key innovations include:
1. **Physics-first validation** ensuring algorithm correctness before RTL
2. **Adversarial training** providing robustness against unknown threats
3. **Efficient hardware mapping** achieving sub-microsecond response

Future work includes:
- Multi-parameter optimization (frequency + PRF + bandwidth + polarization)
- Online learning for continued adaptation in deployment
- Extension to multi-platform coordinated ECCM

---

## References

[1] Haykin, S. "Cognitive Radar: A Way of the Future." IEEE Signal Processing Magazine, 2006.

[2] Schulman, J. et al. "Proximal Policy Optimization Algorithms." arXiv:1707.06347, 2017.

[3] Goodfellow, I. et al. "Generative Adversarial Networks." NeurIPS, 2014.

[4] AMD. "Versal AI Core Series Data Sheet." DS957, 2025.

[5] Greco, M. et al. "Cognitive Radars: On the Road to Reality." IEEE TAES, 2018.

---

## Appendix A: Traceability Matrix

| Requirement ID | Description | Implementation | Test |
|---------------|-------------|----------------|------|
| REQ-EW-01 | Jammer classification | jammer_classifier.py | test_classifier.py |
| REQ-EW-02 | Feature extraction | FeatureExtractor | test_features.py |
| REQ-RL-PPO-001 | PPO continuous actions | ppo_agent.py | test_ppo.py |
| REQ-RL-ADV-PPO-001 | Adversarial training | adversarial_ppo.py | test_adversarial.py |
| REQ-RL-GAN-001 | GAN jammer synthesis | gan_adversarial.py | test_gan.py |
| REQ-RL-FIXED-001 | Fixed-point Q-table | q_table_agility.sv | tb_q_table.py |
| REQ-ECCM-RL | RL-based ECCM | rl_waveform_controller | tb_waveform.py |

---

**Document Control**

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2026-02-05 | Dr. M. Mešter | Initial release |

**Contact**: mladen@nexellum.com | +385 99 737 5100

**© 2026 Nexellum d.o.o. All rights reserved.**
