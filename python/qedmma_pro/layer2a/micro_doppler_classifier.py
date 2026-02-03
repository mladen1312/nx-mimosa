#!/usr/bin/env python3
"""
QEDMMA v3.0 - AI-Native ECCM: Micro-Doppler Target Classifier
[REQ-AI-001] LSTM-based micro-Doppler signature classification
[REQ-AI-002] Distinguish F-35 turbine from birds/decoys

Author: Dr. Mladen Mešter
Copyright (c) 2026 - All Rights Reserved

Purpose:
  - Classify targets based on micro-Doppler signatures
  - Distinguish stealth aircraft from birds, decoys, chaff
  - Reject DRFM/false targets with AI-based pattern recognition
  - Enable autonomous track prioritization

Architecture:
  - LSTM encoder for temporal micro-Doppler sequences
  - Attention mechanism for salient feature extraction
  - Multi-class classification head
  - Confidence estimation for track fusion

References:
  - Chen, V.C. "Micro-Doppler Effect in Radar" (Artech House)
  - Kim, Y. et al. "Human Activity Classification with micro-Doppler"
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict
from enum import IntEnum
import json

# =============================================================================
# TARGET CLASSES
# =============================================================================

class TargetClass(IntEnum):
    """Micro-Doppler target classification categories."""
    UNKNOWN = 0
    FIXED_WING_JET = 1       # Jet aircraft (F-35, F-16, etc.)
    FIXED_WING_PROP = 2      # Propeller aircraft
    ROTARY_WING = 3          # Helicopter
    UAV_SMALL = 4            # Small drone
    UAV_LARGE = 5            # Large UAV (Predator class)
    BIRD_SINGLE = 6          # Single bird
    BIRD_FLOCK = 7           # Bird flock
    DECOY_CHAFF = 8          # Chaff cloud
    DECOY_TOWED = 9          # Towed decoy
    DRFM_JAMMER = 10         # DRFM false target
    GROUND_VEHICLE = 11      # Ground vehicle
    SHIP = 12                # Maritime target
    CLUTTER = 13             # Ground/sea clutter
    
    @classmethod
    def is_threat(cls, target_class: int) -> bool:
        """Check if target class is a potential threat."""
        return target_class in [
            cls.FIXED_WING_JET, cls.FIXED_WING_PROP, 
            cls.ROTARY_WING, cls.UAV_LARGE
        ]
    
    @classmethod
    def is_decoy(cls, target_class: int) -> bool:
        """Check if target class is a decoy/false target."""
        return target_class in [
            cls.DECOY_CHAFF, cls.DECOY_TOWED, cls.DRFM_JAMMER
        ]


# =============================================================================
# MICRO-DOPPLER SIGNATURE DATABASE
# =============================================================================

@dataclass
class MicroDopplerSignature:
    """Characteristic micro-Doppler signature parameters."""
    target_class: TargetClass
    
    # Primary modulation (turbine/rotor/wing)
    primary_freq_hz: Tuple[float, float]      # (min, max) frequency range
    primary_amplitude_db: float               # Relative amplitude
    
    # Secondary modulation (blade flash, etc.)
    secondary_freq_hz: Optional[Tuple[float, float]] = None
    secondary_amplitude_db: float = -20.0
    
    # Temporal characteristics
    modulation_period_ms: Optional[float] = None  # Rotation period
    duty_cycle: float = 0.5                       # Blade visibility
    
    # Statistical properties
    bandwidth_hz: float = 10.0                    # Doppler spread
    coherence_time_ms: float = 100.0              # Signature stability
    
    # Distinguishing features
    unique_features: List[str] = field(default_factory=list)


# Reference signature database
SIGNATURE_DATABASE: Dict[TargetClass, MicroDopplerSignature] = {
    TargetClass.FIXED_WING_JET: MicroDopplerSignature(
        target_class=TargetClass.FIXED_WING_JET,
        primary_freq_hz=(100, 500),      # JEM (Jet Engine Modulation)
        primary_amplitude_db=-15,
        secondary_freq_hz=(2000, 8000),  # Compressor blade flash
        secondary_amplitude_db=-25,
        modulation_period_ms=None,       # Continuous
        bandwidth_hz=50,
        coherence_time_ms=500,
        unique_features=["jem_harmonics", "high_freq_blade_flash", "stable_doppler"]
    ),
    
    TargetClass.ROTARY_WING: MicroDopplerSignature(
        target_class=TargetClass.ROTARY_WING,
        primary_freq_hz=(10, 30),         # Main rotor
        primary_amplitude_db=-10,
        secondary_freq_hz=(50, 150),      # Tail rotor
        secondary_amplitude_db=-15,
        modulation_period_ms=200,         # ~5 Hz rotor
        duty_cycle=0.3,                   # Blade visibility
        bandwidth_hz=100,
        coherence_time_ms=200,
        unique_features=["rotor_harmonics", "blade_flash", "periodic_modulation"]
    ),
    
    TargetClass.BIRD_SINGLE: MicroDopplerSignature(
        target_class=TargetClass.BIRD_SINGLE,
        primary_freq_hz=(5, 50),          # Wing beat
        primary_amplitude_db=-20,
        modulation_period_ms=100,         # ~10 Hz wing beat
        bandwidth_hz=30,
        coherence_time_ms=50,
        unique_features=["irregular_wingbeat", "low_rcs_variation", "erratic_motion"]
    ),
    
    TargetClass.DECOY_CHAFF: MicroDopplerSignature(
        target_class=TargetClass.DECOY_CHAFF,
        primary_freq_hz=(0, 20),          # Slow tumbling
        primary_amplitude_db=-5,
        bandwidth_hz=200,                 # Very wide spread
        coherence_time_ms=10,             # Very short coherence
        unique_features=["wide_bandwidth", "rapid_decorrelation", "no_modulation"]
    ),
    
    TargetClass.DRFM_JAMMER: MicroDopplerSignature(
        target_class=TargetClass.DRFM_JAMMER,
        primary_freq_hz=(0, 0),           # Perfect replay = no modulation
        primary_amplitude_db=0,
        bandwidth_hz=1,                   # Suspiciously narrow
        coherence_time_ms=10000,          # Too stable
        unique_features=["no_micro_doppler", "perfect_coherence", "exact_waveform_match"]
    ),
}


# =============================================================================
# LSTM MICRO-DOPPLER CLASSIFIER
# =============================================================================

class LSTMClassifier:
    """
    LSTM-based micro-Doppler classifier.
    
    Lightweight implementation suitable for FPGA deployment via HLS.
    Uses fixed-point compatible operations.
    """
    
    def __init__(self, 
                 input_size: int = 64,      # Doppler bins
                 hidden_size: int = 128,    # LSTM hidden state
                 num_layers: int = 2,       # LSTM layers
                 num_classes: int = 14,     # Target classes
                 sequence_length: int = 32  # Time steps
                ):
        self.input_size = input_size
        self.hidden_size = hidden_size
        self.num_layers = num_layers
        self.num_classes = num_classes
        self.sequence_length = sequence_length
        
        # Initialize weights (would be loaded from trained model)
        np.random.seed(42)
        self._init_weights()
        
    def _init_weights(self):
        """Initialize LSTM weights (placeholder for trained weights)."""
        # LSTM weights for each layer
        self.W_ih = []  # Input-hidden weights
        self.W_hh = []  # Hidden-hidden weights
        self.b_ih = []  # Input-hidden bias
        self.b_hh = []  # Hidden-hidden bias
        
        for layer in range(self.num_layers):
            input_dim = self.input_size if layer == 0 else self.hidden_size
            
            # 4 gates: input, forget, cell, output
            self.W_ih.append(np.random.randn(4 * self.hidden_size, input_dim) * 0.1)
            self.W_hh.append(np.random.randn(4 * self.hidden_size, self.hidden_size) * 0.1)
            self.b_ih.append(np.zeros(4 * self.hidden_size))
            self.b_hh.append(np.zeros(4 * self.hidden_size))
        
        # Classification head
        self.W_fc = np.random.randn(self.num_classes, self.hidden_size) * 0.1
        self.b_fc = np.zeros(self.num_classes)
        
        # Attention weights
        self.W_attn = np.random.randn(self.hidden_size, self.hidden_size) * 0.1
        self.v_attn = np.random.randn(self.hidden_size) * 0.1
    
    @staticmethod
    def sigmoid(x: np.ndarray) -> np.ndarray:
        """Sigmoid activation (fixed-point friendly)."""
        return 1.0 / (1.0 + np.exp(-np.clip(x, -20, 20)))
    
    @staticmethod
    def tanh(x: np.ndarray) -> np.ndarray:
        """Tanh activation."""
        return np.tanh(np.clip(x, -20, 20))
    
    def lstm_cell(self, x: np.ndarray, h: np.ndarray, c: np.ndarray, 
                  layer: int) -> Tuple[np.ndarray, np.ndarray]:
        """
        Single LSTM cell forward pass.
        
        Args:
            x: Input [input_size]
            h: Hidden state [hidden_size]
            c: Cell state [hidden_size]
            layer: Layer index
            
        Returns:
            (new_h, new_c): Updated states
        """
        # Combined gates computation
        gates = (self.W_ih[layer] @ x + self.b_ih[layer] + 
                 self.W_hh[layer] @ h + self.b_hh[layer])
        
        # Split into 4 gates
        H = self.hidden_size
        i = self.sigmoid(gates[0:H])        # Input gate
        f = self.sigmoid(gates[H:2*H])      # Forget gate
        g = self.tanh(gates[2*H:3*H])       # Cell gate
        o = self.sigmoid(gates[3*H:4*H])    # Output gate
        
        # Update cell and hidden state
        c_new = f * c + i * g
        h_new = o * self.tanh(c_new)
        
        return h_new, c_new
    
    def attention(self, hidden_states: np.ndarray) -> np.ndarray:
        """
        Attention mechanism over sequence of hidden states.
        
        Args:
            hidden_states: [sequence_length, hidden_size]
            
        Returns:
            context: Weighted sum [hidden_size]
        """
        # Compute attention scores
        scores = np.tanh(hidden_states @ self.W_attn.T) @ self.v_attn
        weights = np.exp(scores - np.max(scores))
        weights = weights / np.sum(weights)
        
        # Weighted sum
        context = weights @ hidden_states
        return context
    
    def forward(self, x: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Forward pass through LSTM classifier.
        
        Args:
            x: Input sequence [sequence_length, input_size]
            
        Returns:
            (logits, confidence): Class logits and confidence scores
        """
        seq_len = x.shape[0]
        
        # Initialize hidden states
        h = [np.zeros(self.hidden_size) for _ in range(self.num_layers)]
        c = [np.zeros(self.hidden_size) for _ in range(self.num_layers)]
        
        # Store hidden states for attention
        hidden_sequence = []
        
        # Process sequence
        for t in range(seq_len):
            layer_input = x[t]
            
            for layer in range(self.num_layers):
                h[layer], c[layer] = self.lstm_cell(
                    layer_input, h[layer], c[layer], layer
                )
                layer_input = h[layer]
            
            hidden_sequence.append(h[-1].copy())
        
        # Apply attention
        hidden_sequence = np.array(hidden_sequence)
        context = self.attention(hidden_sequence)
        
        # Classification
        logits = self.W_fc @ context + self.b_fc
        
        # Softmax for confidence
        exp_logits = np.exp(logits - np.max(logits))
        confidence = exp_logits / np.sum(exp_logits)
        
        return logits, confidence
    
    def predict(self, x: np.ndarray) -> Tuple[TargetClass, float]:
        """
        Predict target class from micro-Doppler sequence.
        
        Args:
            x: Input sequence [sequence_length, input_size]
            
        Returns:
            (predicted_class, confidence): Classification result
        """
        logits, confidence = self.forward(x)
        predicted_class = TargetClass(np.argmax(confidence))
        max_confidence = np.max(confidence)
        
        return predicted_class, max_confidence


# =============================================================================
# MICRO-DOPPLER FEATURE EXTRACTOR
# =============================================================================

class MicroDopplerFeatureExtractor:
    """
    Extract micro-Doppler features from radar returns.
    
    Converts raw I/Q data to spectrogram features suitable for LSTM.
    """
    
    def __init__(self,
                 sample_rate_hz: float = 1e6,
                 fft_size: int = 256,
                 hop_size: int = 64,
                 num_bins: int = 64,
                 max_doppler_hz: float = 5000
                ):
        self.sample_rate = sample_rate_hz
        self.fft_size = fft_size
        self.hop_size = hop_size
        self.num_bins = num_bins
        self.max_doppler = max_doppler_hz
        
        # Compute Doppler bin edges
        self.doppler_bins = np.linspace(-max_doppler_hz, max_doppler_hz, num_bins + 1)
        
        # Window function
        self.window = np.hanning(fft_size)
    
    def extract_spectrogram(self, iq_data: np.ndarray) -> np.ndarray:
        """
        Extract micro-Doppler spectrogram from I/Q data.
        
        Args:
            iq_data: Complex I/Q samples
            
        Returns:
            spectrogram: [num_frames, num_bins] power spectrogram
        """
        num_samples = len(iq_data)
        num_frames = (num_samples - self.fft_size) // self.hop_size + 1
        
        spectrogram = np.zeros((num_frames, self.num_bins))
        
        for frame in range(num_frames):
            start = frame * self.hop_size
            segment = iq_data[start:start + self.fft_size] * self.window
            
            # FFT
            spectrum = np.fft.fftshift(np.fft.fft(segment))
            power = np.abs(spectrum) ** 2
            
            # Map to Doppler bins
            freq_axis = np.fft.fftshift(np.fft.fftfreq(self.fft_size, 1/self.sample_rate))
            
            # Resample to output bins
            for b in range(self.num_bins):
                mask = (freq_axis >= self.doppler_bins[b]) & (freq_axis < self.doppler_bins[b+1])
                if np.any(mask):
                    spectrogram[frame, b] = np.mean(power[mask])
        
        # Log scale and normalize
        spectrogram = 10 * np.log10(spectrogram + 1e-10)
        spectrogram = (spectrogram - np.mean(spectrogram)) / (np.std(spectrogram) + 1e-6)
        
        return spectrogram
    
    def extract_features(self, iq_data: np.ndarray) -> Dict:
        """
        Extract comprehensive micro-Doppler features.
        
        Returns dict with spectrogram and derived features.
        """
        spectrogram = self.extract_spectrogram(iq_data)
        
        # Derived features
        features = {
            'spectrogram': spectrogram,
            'bandwidth': np.std(spectrogram, axis=1).mean(),
            'centroid': np.sum(spectrogram * np.arange(self.num_bins), axis=1).mean() / self.num_bins,
            'peak_doppler': self.doppler_bins[np.argmax(np.mean(spectrogram, axis=0))],
            'modulation_index': np.std(np.max(spectrogram, axis=1)),
            'coherence': np.corrcoef(spectrogram[:-1].flatten(), spectrogram[1:].flatten())[0,1],
        }
        
        return features


# =============================================================================
# ECCM DECISION ENGINE
# =============================================================================

class ECCMDecisionEngine:
    """
    AI-enhanced ECCM decision engine.
    
    Combines LSTM classification with rule-based filtering
    for robust target/decoy discrimination.
    """
    
    def __init__(self, confidence_threshold: float = 0.7):
        self.classifier = LSTMClassifier()
        self.feature_extractor = MicroDopplerFeatureExtractor()
        self.confidence_threshold = confidence_threshold
        
        # Track classification history for consistency check
        self.track_history: Dict[int, List[TargetClass]] = {}
    
    def classify_target(self, track_id: int, iq_data: np.ndarray) -> Dict:
        """
        Classify target and make ECCM decision.
        
        Args:
            track_id: Unique track identifier
            iq_data: Raw I/Q samples from target
            
        Returns:
            Decision dict with classification and ECCM action
        """
        # Extract features
        features = self.feature_extractor.extract_features(iq_data)
        spectrogram = features['spectrogram']
        
        # Ensure correct sequence length
        if spectrogram.shape[0] < self.classifier.sequence_length:
            # Pad with zeros
            pad = np.zeros((self.classifier.sequence_length - spectrogram.shape[0], 
                           spectrogram.shape[1]))
            spectrogram = np.vstack([pad, spectrogram])
        elif spectrogram.shape[0] > self.classifier.sequence_length:
            # Take last N frames
            spectrogram = spectrogram[-self.classifier.sequence_length:]
        
        # Classify
        predicted_class, confidence = self.classifier.predict(spectrogram)
        
        # Update track history
        if track_id not in self.track_history:
            self.track_history[track_id] = []
        self.track_history[track_id].append(predicted_class)
        
        # Keep only last 10 classifications
        if len(self.track_history[track_id]) > 10:
            self.track_history[track_id].pop(0)
        
        # Consistency check
        history = self.track_history[track_id]
        if len(history) >= 3:
            mode_class = max(set(history), key=history.count)
            consistency = history.count(mode_class) / len(history)
        else:
            mode_class = predicted_class
            consistency = confidence
        
        # ECCM Decision
        decision = {
            'track_id': track_id,
            'predicted_class': predicted_class,
            'class_name': predicted_class.name,
            'confidence': float(confidence),
            'consistency': float(consistency),
            'is_threat': TargetClass.is_threat(predicted_class),
            'is_decoy': TargetClass.is_decoy(predicted_class),
            'features': {k: float(v) if np.isscalar(v) else None 
                        for k, v in features.items() if k != 'spectrogram'},
        }
        
        # ECCM action
        if TargetClass.is_decoy(predicted_class) and confidence > self.confidence_threshold:
            decision['eccm_action'] = 'REJECT'
            decision['eccm_reason'] = f"Classified as {predicted_class.name} with {confidence:.1%} confidence"
        elif predicted_class == TargetClass.CLUTTER:
            decision['eccm_action'] = 'SUPPRESS'
            decision['eccm_reason'] = "Clutter detected"
        elif TargetClass.is_threat(predicted_class):
            decision['eccm_action'] = 'TRACK_PRIORITY'
            decision['eccm_reason'] = f"Threat: {predicted_class.name}"
        else:
            decision['eccm_action'] = 'TRACK_NORMAL'
            decision['eccm_reason'] = "Non-threat target"
        
        return decision


# =============================================================================
# DRFM DETECTION MODULE
# =============================================================================

class DRFMDetector:
    """
    Specialized DRFM (Digital RF Memory) jammer detection.
    
    DRFM jammers replay radar waveforms, creating false targets.
    Detection based on:
    - Lack of micro-Doppler modulation
    - Suspiciously perfect coherence
    - Exact waveform match
    """
    
    def __init__(self):
        self.coherence_threshold = 0.99    # DRFM has too-perfect coherence
        self.modulation_threshold = 0.01   # DRFM lacks micro-Doppler
        
    def detect(self, iq_data: np.ndarray, reference_waveform: np.ndarray) -> Dict:
        """
        Detect DRFM false target.
        
        Args:
            iq_data: Received I/Q data
            reference_waveform: Transmitted waveform
            
        Returns:
            Detection result
        """
        # Cross-correlation for waveform match
        correlation = np.abs(np.correlate(iq_data, reference_waveform, mode='valid'))
        max_corr = np.max(correlation) / (np.std(iq_data) * np.std(reference_waveform) * len(reference_waveform))
        
        # Micro-Doppler modulation check
        # Real targets have micro-Doppler, DRFM doesn't
        envelope = np.abs(iq_data)
        modulation_index = np.std(envelope) / np.mean(envelope)
        
        # Coherence check (DRFM is too stable)
        # Split into segments and check correlation
        n_segments = 4
        segment_len = len(iq_data) // n_segments
        coherence_values = []
        for i in range(n_segments - 1):
            seg1 = iq_data[i*segment_len:(i+1)*segment_len]
            seg2 = iq_data[(i+1)*segment_len:(i+2)*segment_len]
            coh = np.abs(np.corrcoef(np.abs(seg1), np.abs(seg2))[0, 1])
            coherence_values.append(coh)
        avg_coherence = np.mean(coherence_values)
        
        # DRFM indicators
        is_drfm = (
            (max_corr > self.coherence_threshold) or
            (modulation_index < self.modulation_threshold) or
            (avg_coherence > self.coherence_threshold)
        )
        
        confidence = 0.0
        if max_corr > self.coherence_threshold:
            confidence += 0.4
        if modulation_index < self.modulation_threshold:
            confidence += 0.3
        if avg_coherence > self.coherence_threshold:
            confidence += 0.3
        
        return {
            'is_drfm': is_drfm,
            'confidence': confidence,
            'waveform_match': float(max_corr),
            'modulation_index': float(modulation_index),
            'coherence': float(avg_coherence),
            'indicators': {
                'perfect_replay': max_corr > self.coherence_threshold,
                'no_micro_doppler': modulation_index < self.modulation_threshold,
                'too_coherent': avg_coherence > self.coherence_threshold,
            }
        }


# =============================================================================
# MAIN TEST
# =============================================================================

if __name__ == "__main__":
    print("\n" + "=" * 70)
    print("QEDMMA v3.0 - AI-Native ECCM Micro-Doppler Classifier")
    print("=" * 70)
    
    # Initialize components
    classifier = LSTMClassifier()
    feature_extractor = MicroDopplerFeatureExtractor()
    eccm_engine = ECCMDecisionEngine()
    drfm_detector = DRFMDetector()
    
    print(f"\n📊 Model Configuration:")
    print(f"   LSTM hidden size:    {classifier.hidden_size}")
    print(f"   LSTM layers:         {classifier.num_layers}")
    print(f"   Sequence length:     {classifier.sequence_length}")
    print(f"   Target classes:      {classifier.num_classes}")
    print(f"   Doppler bins:        {feature_extractor.num_bins}")
    
    # Simulate test data
    print(f"\n🎯 Running Classification Tests...")
    print("-" * 70)
    
    np.random.seed(123)
    
    test_cases = [
        ("F-35 Jet", lambda: np.exp(1j * 2 * np.pi * 300 * np.arange(10000) / 1e6) * 
                           (1 + 0.1 * np.sin(2 * np.pi * 5000 * np.arange(10000) / 1e6)) +
                           0.1 * np.random.randn(10000)),
        ("Helicopter", lambda: np.exp(1j * 2 * np.pi * 50 * np.arange(10000) / 1e6) *
                              (1 + 0.3 * np.sin(2 * np.pi * 20 * np.arange(10000) / 1e6)) +
                              0.1 * np.random.randn(10000)),
        ("Bird", lambda: np.exp(1j * 2 * np.pi * 10 * np.arange(10000) / 1e6) *
                        (1 + 0.5 * np.sin(2 * np.pi * 8 * np.arange(10000) / 1e6)) +
                        0.2 * np.random.randn(10000)),
        ("Chaff", lambda: 0.5 * np.random.randn(10000) + 0.5j * np.random.randn(10000)),
    ]
    
    for name, data_gen in test_cases:
        iq_data = data_gen()
        
        # Classify
        decision = eccm_engine.classify_target(track_id=hash(name) % 1000, iq_data=iq_data)
        
        print(f"\n   {name}:")
        print(f"      Class:      {decision['class_name']}")
        print(f"      Confidence: {decision['confidence']:.1%}")
        print(f"      ECCM Action: {decision['eccm_action']}")
        print(f"      Is Threat:  {decision['is_threat']}")
        print(f"      Is Decoy:   {decision['is_decoy']}")
    
    # DRFM detection test
    print(f"\n\n🛡️ DRFM Detection Test...")
    print("-" * 70)
    
    # Simulate transmitted waveform
    tx_waveform = np.exp(1j * 2 * np.pi * np.linspace(0, 100, 1000))
    
    # Normal target (has micro-Doppler)
    normal_rx = tx_waveform * (1 + 0.2 * np.sin(2 * np.pi * 10 * np.arange(1000) / 1000))
    normal_rx += 0.1 * np.random.randn(1000)
    
    # DRFM replay (perfect copy)
    drfm_rx = tx_waveform * 1.0 + 0.01 * np.random.randn(1000)
    
    print(f"\n   Normal Target:")
    result = drfm_detector.detect(normal_rx, tx_waveform)
    print(f"      DRFM Detected: {result['is_drfm']}")
    print(f"      Confidence:    {result['confidence']:.1%}")
    print(f"      Modulation:    {result['modulation_index']:.3f}")
    
    print(f"\n   DRFM Jammer:")
    result = drfm_detector.detect(drfm_rx, tx_waveform)
    print(f"      DRFM Detected: {result['is_drfm']}")
    print(f"      Confidence:    {result['confidence']:.1%}")
    print(f"      Waveform Match: {result['waveform_match']:.3f}")
    
    print("\n" + "=" * 70)
    print("✅ AI-Native ECCM Module Test Complete")
    print("=" * 70)
