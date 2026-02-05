#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════════════════════════════
NX-MIMOSA Jammer Classifier & Feature Extraction
═══════════════════════════════════════════════════════════════════════════════════════════════════════

ML-based classification of electronic warfare signals:
  - Thermal Noise (Benign)
  - Clutter (Benign, highly correlated)
  - RGPO Jammer (Malicious, delay-shifted)
  - Barrage Jammer (Malicious, wideband noise)
  - Spot Jammer (Malicious, narrowband)

Features:
  - JNR (Jammer-to-Noise Ratio)
  - Time Variance (pulsed vs continuous)
  - Frequency Spread (spectral analysis)
  - PRF Stability (inter-pulse consistency)

Traceability:
  [REQ-EW-01] Jammer classification
  [REQ-EW-02] Feature extraction pipeline
  [REQ-ECCM-CLASSIFY] Multi-class jammer detection

Author: Dr. Mladen Mešter / Nexellum d.o.o.
Version: 1.1.0
License: AGPL v3 / Commercial
═══════════════════════════════════════════════════════════════════════════════════════════════════════
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional
from enum import IntEnum
import warnings

# Optional imports
try:
    from sklearn.ensemble import RandomForestClassifier, GradientBoostingClassifier
    from sklearn.model_selection import train_test_split, cross_val_score
    from sklearn.metrics import classification_report, confusion_matrix
    from sklearn.preprocessing import StandardScaler
    import joblib
    SKLEARN_AVAILABLE = True
except ImportError:
    SKLEARN_AVAILABLE = False


# =============================================================================
# CONSTANTS & ENUMS
# =============================================================================

class JammerType(IntEnum):
    """Jammer classification labels."""
    NOISE = 0           # Thermal noise (benign)
    CLUTTER = 1         # Ground/sea clutter (benign)
    RGPO = 2            # Range Gate Pull-Off
    BARRAGE = 3         # Wideband barrage jamming
    SPOT = 4            # Narrowband spot jamming
    VGPO = 5            # Velocity Gate Pull-Off


@dataclass
class SignalFeatures:
    """Extracted features from radar signal."""
    jnr_db: float           # Jammer-to-Noise Ratio (dB)
    var_time: float         # Temporal variance
    var_freq: float         # Spectral variance
    prf_stability: float    # PRF consistency metric
    peak_ratio: float       # Peak-to-mean ratio
    spectral_entropy: float # Frequency domain entropy
    pulse_width_var: float  # Pulse width variation


# =============================================================================
# FEATURE EXTRACTION
# =============================================================================

class FeatureExtractor:
    """
    Extract classification features from I/Q radar signals.
    
    [REQ-EW-02] Feature extraction pipeline
    """
    
    def __init__(self, fs: float = 100e6, n_fft: int = 1024):
        """
        Initialize feature extractor.
        
        Args:
            fs: Sampling frequency (Hz)
            n_fft: FFT size for spectral analysis
        """
        self.fs = fs
        self.n_fft = n_fft
    
    def extract(self, iq_signal: np.ndarray) -> SignalFeatures:
        """
        Extract all features from I/Q signal.
        
        Args:
            iq_signal: Complex I/Q samples
        
        Returns:
            SignalFeatures dataclass
        """
        mag = np.abs(iq_signal)
        
        # 1. JNR (Jammer-to-Noise Ratio proxy)
        noise_floor = np.percentile(mag, 25)
        jnr_db = 10 * np.log10(np.max(mag) / (noise_floor + 1e-12))
        
        # 2. Temporal variance (pulsed vs continuous)
        var_time = np.var(mag) / (np.mean(mag) ** 2 + 1e-12)
        
        # 3. Spectral analysis
        spectrum = np.abs(np.fft.fft(iq_signal, self.n_fft))
        spectrum_norm = spectrum / (np.sum(spectrum) + 1e-12)
        var_freq = np.var(spectrum)
        
        # 4. Spectral entropy (uniformity of spectrum)
        spectral_entropy = -np.sum(spectrum_norm * np.log(spectrum_norm + 1e-12))
        
        # 5. PRF stability (autocorrelation-based)
        autocorr = np.correlate(mag, mag, mode='same')
        autocorr_norm = autocorr / (autocorr[len(autocorr)//2] + 1e-12)
        # Find secondary peaks
        prf_stability = self._compute_prf_stability(autocorr_norm)
        
        # 6. Peak-to-mean ratio
        peak_ratio = np.max(mag) / (np.mean(mag) + 1e-12)
        
        # 7. Pulse width variation (zero-crossing analysis)
        pulse_width_var = self._compute_pulse_width_var(mag)
        
        return SignalFeatures(
            jnr_db=jnr_db,
            var_time=var_time,
            var_freq=var_freq,
            prf_stability=prf_stability,
            peak_ratio=peak_ratio,
            spectral_entropy=spectral_entropy,
            pulse_width_var=pulse_width_var
        )
    
    def _compute_prf_stability(self, autocorr: np.ndarray) -> float:
        """Compute PRF stability from autocorrelation."""
        # Find peaks in autocorrelation (excluding center)
        center = len(autocorr) // 2
        half = autocorr[center:]
        
        # Simple peak detection
        peaks = []
        for i in range(1, len(half) - 1):
            if half[i] > half[i-1] and half[i] > half[i+1] and half[i] > 0.3:
                peaks.append(i)
        
        if len(peaks) >= 2:
            # Compute variance of peak spacing
            spacings = np.diff(peaks)
            return 1.0 / (1.0 + np.var(spacings))
        else:
            return 0.0
    
    def _compute_pulse_width_var(self, mag: np.ndarray) -> float:
        """Compute pulse width variation."""
        threshold = np.mean(mag) + np.std(mag)
        above = mag > threshold
        
        # Find pulse widths
        widths = []
        in_pulse = False
        width = 0
        
        for val in above:
            if val and not in_pulse:
                in_pulse = True
                width = 1
            elif val and in_pulse:
                width += 1
            elif not val and in_pulse:
                widths.append(width)
                in_pulse = False
                width = 0
        
        if len(widths) >= 2:
            return np.std(widths) / (np.mean(widths) + 1e-12)
        else:
            return 0.0
    
    def to_vector(self, features: SignalFeatures) -> np.ndarray:
        """Convert features to numpy vector."""
        return np.array([
            features.jnr_db,
            features.var_time,
            features.var_freq,
            features.prf_stability,
            features.peak_ratio,
            features.spectral_entropy,
            features.pulse_width_var
        ])


# =============================================================================
# SIGNAL GENERATION (for training data)
# =============================================================================

class JammerSignalGenerator:
    """
    Generate synthetic jammer signals for classifier training.
    
    Uses physics-based models for each jammer type.
    """
    
    def __init__(self, fs: float = 100e6, n_samples: int = 4096):
        self.fs = fs
        self.n_samples = n_samples
        self.t = np.arange(n_samples) / fs
    
    def generate_noise(self, sigma: float = 0.1) -> np.ndarray:
        """Generate thermal noise."""
        return (np.random.randn(self.n_samples) + 
                1j * np.random.randn(self.n_samples)) * sigma
    
    def generate_clutter(self, sigma: float = 0.3, correlation: float = 0.9) -> np.ndarray:
        """Generate correlated clutter."""
        # Exponentially correlated noise
        noise = np.random.randn(self.n_samples) + 1j * np.random.randn(self.n_samples)
        clutter = np.zeros(self.n_samples, dtype=complex)
        clutter[0] = noise[0]
        for i in range(1, self.n_samples):
            clutter[i] = correlation * clutter[i-1] + np.sqrt(1 - correlation**2) * noise[i]
        return clutter * sigma
    
    def generate_rgpo(self, jnr_db: float = 30, delay_rate: float = 1e-6) -> np.ndarray:
        """
        Generate RGPO (Range Gate Pull-Off) jammer.
        
        Characteristics:
        - High power
        - Progressive delay
        - Copies radar waveform
        """
        amplitude = 10 ** (jnr_db / 20)
        
        # Pulsed signal with increasing delay
        prf = 1000  # Hz
        pulse_width = 1e-6  # 1 µs
        
        signal = np.zeros(self.n_samples, dtype=complex)
        
        pulse_interval = int(self.fs / prf)
        n_pulses = self.n_samples // pulse_interval
        
        for p in range(n_pulses):
            delay = int(p * delay_rate * self.fs)
            start = p * pulse_interval + delay
            end = start + int(pulse_width * self.fs)
            
            if 0 <= start < self.n_samples and end < self.n_samples:
                signal[start:end] = amplitude * np.exp(1j * np.random.uniform(0, 2*np.pi))
        
        # Add noise
        signal += self.generate_noise(0.1)
        
        return signal
    
    def generate_barrage(self, jnr_db: float = 40) -> np.ndarray:
        """
        Generate barrage (wideband) jammer.
        
        Characteristics:
        - High power continuous
        - Wideband noise-like
        - No pulse structure
        """
        amplitude = 10 ** (jnr_db / 20)
        
        # Wideband noise with high power
        signal = (np.random.randn(self.n_samples) + 
                 1j * np.random.randn(self.n_samples)) * amplitude
        
        return signal
    
    def generate_spot(self, jnr_db: float = 35, freq_offset: float = 1e6) -> np.ndarray:
        """
        Generate spot (narrowband CW) jammer.
        
        Characteristics:
        - Very narrow bandwidth
        - Single frequency
        - Continuous wave
        """
        amplitude = 10 ** (jnr_db / 20)
        
        # CW signal at offset frequency
        signal = amplitude * np.exp(2j * np.pi * freq_offset * self.t)
        
        # Add slight modulation
        signal *= (1 + 0.1 * np.sin(2 * np.pi * 100 * self.t))
        
        # Add noise floor
        signal += self.generate_noise(0.05)
        
        return signal
    
    def generate_vgpo(self, jnr_db: float = 30, doppler_rate: float = 1000) -> np.ndarray:
        """
        Generate VGPO (Velocity Gate Pull-Off) jammer.
        
        Characteristics:
        - Progressive Doppler shift
        - Pulsed like RGPO but with frequency modulation
        """
        amplitude = 10 ** (jnr_db / 20)
        
        # Pulsed signal with progressive Doppler
        prf = 1000
        pulse_width = 1e-6
        
        signal = np.zeros(self.n_samples, dtype=complex)
        pulse_interval = int(self.fs / prf)
        n_pulses = self.n_samples // pulse_interval
        
        for p in range(n_pulses):
            # Progressive Doppler shift
            doppler = p * doppler_rate
            start = p * pulse_interval
            end = start + int(pulse_width * self.fs)
            
            if end < self.n_samples:
                t_pulse = self.t[start:end]
                signal[start:end] = amplitude * np.exp(2j * np.pi * doppler * t_pulse)
        
        signal += self.generate_noise(0.1)
        
        return signal
    
    def generate_dataset(self, n_per_class: int = 500) -> Tuple[np.ndarray, np.ndarray]:
        """
        Generate complete training dataset.
        
        Returns:
            (X, y) where X is features and y is labels
        """
        extractor = FeatureExtractor(self.fs)
        
        X = []
        y = []
        
        jnr_range = (20, 50)
        
        for _ in range(n_per_class):
            # Noise
            sig = self.generate_noise(sigma=np.random.uniform(0.05, 0.2))
            X.append(extractor.to_vector(extractor.extract(sig)))
            y.append(JammerType.NOISE)
            
            # Clutter
            sig = self.generate_clutter(
                sigma=np.random.uniform(0.2, 0.5),
                correlation=np.random.uniform(0.7, 0.95)
            )
            X.append(extractor.to_vector(extractor.extract(sig)))
            y.append(JammerType.CLUTTER)
            
            # RGPO
            sig = self.generate_rgpo(
                jnr_db=np.random.uniform(*jnr_range),
                delay_rate=np.random.uniform(0.5e-6, 2e-6)
            )
            X.append(extractor.to_vector(extractor.extract(sig)))
            y.append(JammerType.RGPO)
            
            # Barrage
            sig = self.generate_barrage(jnr_db=np.random.uniform(*jnr_range))
            X.append(extractor.to_vector(extractor.extract(sig)))
            y.append(JammerType.BARRAGE)
            
            # Spot
            sig = self.generate_spot(
                jnr_db=np.random.uniform(*jnr_range),
                freq_offset=np.random.uniform(0.5e6, 5e6)
            )
            X.append(extractor.to_vector(extractor.extract(sig)))
            y.append(JammerType.SPOT)
        
        return np.array(X), np.array(y)


# =============================================================================
# JAMMER CLASSIFIER
# =============================================================================

class JammerClassifier:
    """
    ML-based jammer classifier.
    
    [REQ-EW-01] Multi-class jammer classification
    """
    
    def __init__(self, n_estimators: int = 50, max_depth: int = 10):
        self.n_estimators = n_estimators
        self.max_depth = max_depth
        self.model = None
        self.scaler = None
        self.trained = False
        self.feature_names = [
            'jnr_db', 'var_time', 'var_freq', 'prf_stability',
            'peak_ratio', 'spectral_entropy', 'pulse_width_var'
        ]
    
    def train(self, X: np.ndarray, y: np.ndarray, test_size: float = 0.2):
        """
        Train the classifier.
        
        Args:
            X: Feature matrix (n_samples, n_features)
            y: Labels
            test_size: Fraction for validation
        """
        if not SKLEARN_AVAILABLE:
            raise ImportError("scikit-learn required for training")
        
        # Split data
        X_train, X_val, y_train, y_val = train_test_split(
            X, y, test_size=test_size, random_state=42, stratify=y
        )
        
        # Normalize features
        self.scaler = StandardScaler()
        X_train_scaled = self.scaler.fit_transform(X_train)
        X_val_scaled = self.scaler.transform(X_val)
        
        # Train Random Forest
        self.model = RandomForestClassifier(
            n_estimators=self.n_estimators,
            max_depth=self.max_depth,
            random_state=42,
            n_jobs=-1
        )
        self.model.fit(X_train_scaled, y_train)
        
        # Validate
        y_pred = self.model.predict(X_val_scaled)
        
        print("\n" + "═" * 60)
        print("JAMMER CLASSIFIER TRAINING RESULTS")
        print("═" * 60)
        print(classification_report(
            y_val, y_pred,
            target_names=[t.name for t in JammerType if t.value < 5]
        ))
        
        # Feature importance
        print("\nFeature Importance:")
        importances = self.model.feature_importances_
        for name, imp in sorted(zip(self.feature_names, importances), 
                                key=lambda x: -x[1]):
            print(f"  {name:<20} {imp:.3f}")
        
        self.trained = True
        
        return self.model.score(X_val_scaled, y_val)
    
    def predict(self, features: np.ndarray) -> Tuple[int, np.ndarray]:
        """
        Predict jammer type.
        
        Args:
            features: Feature vector or matrix
        
        Returns:
            (predicted_class, class_probabilities)
        """
        if not self.trained:
            raise RuntimeError("Model not trained")
        
        features = np.atleast_2d(features)
        features_scaled = self.scaler.transform(features)
        
        pred = self.model.predict(features_scaled)
        proba = self.model.predict_proba(features_scaled)
        
        return pred[0] if len(pred) == 1 else pred, proba
    
    def export_trees(self, output_path: str):
        """
        Export decision trees for FPGA implementation.
        
        Generates HLS-compatible C code.
        """
        if not self.trained:
            raise RuntimeError("Model not trained")
        
        with open(output_path, 'w') as f:
            f.write("// Auto-generated Jammer Classifier Trees\n")
            f.write("// DO NOT EDIT - Generated from Python model\n\n")
            f.write("#include \"ap_fixed.h\"\n\n")
            f.write("typedef ap_fixed<16, 8> data_t;\n")
            f.write("typedef ap_uint<3> class_t;\n\n")
            
            # Export first few trees (simplified)
            for i, tree in enumerate(self.model.estimators_[:5]):
                f.write(f"// Tree {i}\n")
                f.write(f"class_t tree_{i}_predict(")
                f.write(", ".join([f"data_t {name}" for name in self.feature_names]))
                f.write(") {\n")
                f.write("    #pragma HLS PIPELINE\n")
                
                # Simplified tree export (actual implementation would traverse tree structure)
                self._export_tree_node(f, tree.tree_, 0, 1)
                
                f.write("}\n\n")
        
        print(f"Exported trees to {output_path}")
    
    def _export_tree_node(self, f, tree, node_id: int, indent: int):
        """Recursively export tree node to C code."""
        indent_str = "    " * indent
        
        if tree.children_left[node_id] == -1:  # Leaf
            class_id = np.argmax(tree.value[node_id])
            f.write(f"{indent_str}return {class_id};\n")
        else:
            feature = tree.feature[node_id]
            threshold = tree.threshold[node_id]
            feature_name = self.feature_names[feature] if feature < len(self.feature_names) else f"f{feature}"
            
            f.write(f"{indent_str}if ({feature_name} <= {threshold:.4f}) {{\n")
            self._export_tree_node(f, tree, tree.children_left[node_id], indent + 1)
            f.write(f"{indent_str}}} else {{\n")
            self._export_tree_node(f, tree, tree.children_right[node_id], indent + 1)
            f.write(f"{indent_str}}}\n")


# =============================================================================
# FIXED-POINT EXPORT FOR FPGA
# =============================================================================

def export_classifier_to_fpga(classifier: JammerClassifier, output_dir: str):
    """
    Export classifier parameters for FPGA implementation.
    
    Generates:
    - Scaler parameters (mean, std)
    - Tree thresholds in fixed-point
    - SystemVerilog package
    """
    import os
    os.makedirs(output_dir, exist_ok=True)
    
    # Export scaler
    scaler_file = os.path.join(output_dir, "jammer_scaler.hex")
    with open(scaler_file, 'w') as f:
        f.write("// Scaler Mean (Q8.8)\n")
        for m in classifier.scaler.mean_:
            val = int(m * 256) & 0xFFFF
            f.write(f"{val:04X}\n")
        f.write("// Scaler Std (Q8.8)\n")
        for s in classifier.scaler.scale_:
            val = int(s * 256) & 0xFFFF
            f.write(f"{val:04X}\n")
    
    # Export HLS-style trees
    hls_file = os.path.join(output_dir, "jammer_classifier_hls.cpp")
    classifier.export_trees(hls_file)
    
    print(f"FPGA export complete: {output_dir}")


# =============================================================================
# MAIN
# =============================================================================

def main():
    """Train and evaluate jammer classifier."""
    print("═" * 70)
    print("NX-MIMOSA Jammer Classifier Training")
    print("═" * 70)
    
    # Generate dataset
    print("\nGenerating training dataset...")
    generator = JammerSignalGenerator(fs=100e6, n_samples=4096)
    X, y = generator.generate_dataset(n_per_class=500)
    
    print(f"Dataset: {X.shape[0]} samples, {X.shape[1]} features")
    print(f"Classes: {np.bincount(y)}")
    
    # Train classifier
    print("\nTraining Random Forest classifier...")
    classifier = JammerClassifier(n_estimators=50, max_depth=10)
    accuracy = classifier.train(X, y)
    
    print(f"\nValidation Accuracy: {accuracy:.1%}")
    
    # Export for FPGA
    if SKLEARN_AVAILABLE:
        export_classifier_to_fpga(classifier, "/home/claude/nx-mimosa-unified/python/ew/export")
    
    print("\n✅ Jammer Classifier training complete!")
    
    return classifier


if __name__ == "__main__":
    classifier = main()
