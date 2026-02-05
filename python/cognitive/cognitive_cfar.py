#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════════════════════════════
NX-MIMOSA Cognitive CFAR Simulation
═══════════════════════════════════════════════════════════════════════════════════════════════════════

Physics-first simulation and validation of ML-based CFAR detection.
Includes multi-target scenarios, clutter, and RGPO jammer simulation.

Traceability:
  [REQ-SIM-001] Multi-target scenario validation
  [REQ-ECCM-01] RGPO jammer detection
  [REQ-COG-01] ML-based adaptive threshold

Author: Dr. Mladen Mešter / Nexellum d.o.o.
Version: 1.1.0
License: AGPL v3 / Commercial
═══════════════════════════════════════════════════════════════════════════════════════════════════════
"""

import numpy as np
from typing import List, Tuple, Dict, Any, Optional
from dataclasses import dataclass, field
from scipy.signal import find_peaks
import warnings

# Optional imports
try:
    from sklearn.svm import SVC
    from sklearn.preprocessing import StandardScaler
    from sklearn.model_selection import train_test_split
    from sklearn.metrics import classification_report
    SKLEARN_AVAILABLE = True
except ImportError:
    SKLEARN_AVAILABLE = False


# =============================================================================
# CONSTANTS
# =============================================================================

C = 3e8  # Speed of light (m/s)


# =============================================================================
# CONFIGURATION
# =============================================================================

@dataclass
class RadarConfig:
    """Radar system configuration."""
    fs: float = 100e6           # Sampling frequency (Hz)
    fc: float = 10e9            # Carrier frequency (Hz)
    bandwidth: float = 10e6     # Bandwidth (Hz)
    prf: float = 1000           # PRF (Hz)
    n_pulses: int = 64          # Pulses per CPI
    max_range: float = 6000     # Maximum range (m)
    
    @property
    def range_resolution(self) -> float:
        return C / (2 * self.bandwidth)
    
    @property
    def max_unambiguous_range(self) -> float:
        return C / (2 * self.prf)


@dataclass
class TargetConfig:
    """Target configuration."""
    range_m: float              # Range (meters)
    rcs: float = 1.0            # RCS (m²)
    velocity: float = 0.0       # Radial velocity (m/s)
    snr_db: float = 15.0        # SNR at detector (dB)


@dataclass
class JammerConfig:
    """RGPO jammer configuration."""
    enabled: bool = False
    target_range: float = 0.0   # Range of target being spoofed
    pull_off: float = 500.0     # Pull-off distance (m)
    jnr_db: float = 20.0        # Jammer-to-noise ratio (dB)


@dataclass 
class ClutterConfig:
    """Clutter configuration."""
    sigma: float = 0.2          # Clutter RMS amplitude
    distribution: str = 'rayleigh'  # 'rayleigh', 'weibull', 'k'


@dataclass
class CFARConfig:
    """CFAR detector configuration."""
    window_size: int = 20       # Reference window size
    guard_cells: int = 4        # Guard cells around CUT
    pfa: float = 1e-6           # Desired false alarm rate
    threshold_factor: float = 5.0  # Multiplier for adaptive threshold


# =============================================================================
# SIGNAL GENERATION
# =============================================================================

class RadarSignalGenerator:
    """
    Generate simulated radar returns for CFAR testing.
    
    [REQ-SIM-001] Multi-target scenario generation
    """
    
    def __init__(self, config: RadarConfig):
        self.config = config
        self.t_max = 2 * config.max_range / C
        self.n_samples = int(self.t_max * config.fs)
        self.t = np.linspace(0, self.t_max, self.n_samples)
        self.range_bins = self.t * C / 2
    
    def generate_target_return(self, target: TargetConfig) -> np.ndarray:
        """Generate return from a single target."""
        signal = np.zeros(self.n_samples, dtype=complex)
        
        delay = 2 * target.range_m / C
        idx = int(delay * self.config.fs)
        
        if 0 <= idx < self.n_samples:
            # Amplitude from SNR and RCS
            amplitude = np.sqrt(target.rcs) * 10 ** (target.snr_db / 20)
            
            # Add Doppler phase (for coherent processing)
            doppler_freq = 2 * target.velocity * self.config.fc / C
            phase = 2 * np.pi * doppler_freq * delay
            
            signal[idx] = amplitude * np.exp(1j * phase)
        
        return signal
    
    def generate_clutter(self, config: ClutterConfig) -> np.ndarray:
        """Generate clutter returns."""
        if config.distribution == 'rayleigh':
            # Rayleigh clutter (complex Gaussian)
            real = np.random.normal(0, config.sigma, self.n_samples)
            imag = np.random.normal(0, config.sigma, self.n_samples)
        elif config.distribution == 'weibull':
            # Weibull clutter (heavy-tailed)
            magnitude = np.random.weibull(2.0, self.n_samples) * config.sigma
            phase = np.random.uniform(0, 2 * np.pi, self.n_samples)
            return magnitude * np.exp(1j * phase)
        else:
            real = np.random.normal(0, config.sigma, self.n_samples)
            imag = np.random.normal(0, config.sigma, self.n_samples)
        
        return real + 1j * imag
    
    def generate_jammer(self, config: JammerConfig) -> np.ndarray:
        """
        Generate RGPO jammer return.
        
        [REQ-ECCM-01] RGPO simulation
        """
        signal = np.zeros(self.n_samples, dtype=complex)
        
        if not config.enabled:
            return signal
        
        # Jammer creates false target at pulled-off range
        jammer_range = config.target_range + config.pull_off
        delay = 2 * jammer_range / C
        idx = int(delay * self.config.fs)
        
        if 0 <= idx < self.n_samples:
            # High power jamming signal
            amplitude = 10 ** (config.jnr_db / 20)
            # Jammer has characteristic signature (broadband noise-like)
            signal[max(0, idx-2):min(self.n_samples, idx+3)] = amplitude * (1 + 1j)
        
        return signal
    
    def generate_scenario(self, targets: List[TargetConfig],
                         clutter: ClutterConfig,
                         jammer: Optional[JammerConfig] = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        Generate complete radar scenario.
        
        Returns:
            (range_bins, complex_signal)
        """
        signal = np.zeros(self.n_samples, dtype=complex)
        
        # Add targets
        for target in targets:
            signal += self.generate_target_return(target)
        
        # Add clutter
        signal += self.generate_clutter(clutter)
        
        # Add jammer
        if jammer is not None:
            signal += self.generate_jammer(jammer)
        
        return self.range_bins, signal


# =============================================================================
# STANDARD CFAR DETECTOR
# =============================================================================

class StandardCFAR:
    """
    Standard Cell-Averaging CFAR detector.
    
    Used as baseline comparison for cognitive CFAR.
    """
    
    def __init__(self, config: CFARConfig):
        self.config = config
    
    def detect(self, signal: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Apply CA-CFAR detection.
        
        Args:
            signal: Complex radar signal
        
        Returns:
            (magnitude, threshold, detections)
        """
        magnitude = np.abs(signal)
        n = len(magnitude)
        
        threshold = np.zeros(n)
        half_window = self.config.window_size // 2
        guard = self.config.guard_cells
        
        for i in range(n):
            # Leading window
            lead_start = max(0, i - half_window - guard)
            lead_end = max(0, i - guard)
            
            # Lagging window
            lag_start = min(n, i + guard + 1)
            lag_end = min(n, i + half_window + guard + 1)
            
            # Combine reference cells
            ref_cells = np.concatenate([
                magnitude[lead_start:lead_end],
                magnitude[lag_start:lag_end]
            ])
            
            if len(ref_cells) > 0:
                threshold[i] = np.mean(ref_cells) * self.config.threshold_factor
            else:
                threshold[i] = self.config.threshold_factor
        
        detections = magnitude > threshold
        
        return magnitude, threshold, detections


# =============================================================================
# COGNITIVE CFAR (ML-BASED)
# =============================================================================

class CognitiveCFAR:
    """
    ML-based Cognitive CFAR detector using SVM.
    
    [REQ-COG-01] Adaptive threshold via machine learning
    
    Features:
    - Magnitude
    - Local mean
    - Local variance
    """
    
    def __init__(self, config: CFARConfig):
        self.config = config
        self.model = None
        self.scaler = None
        self.trained = False
    
    def extract_features(self, magnitude: np.ndarray) -> np.ndarray:
        """
        Extract feature vector for each range bin.
        
        Features: [magnitude, local_mean, local_variance]
        """
        n = len(magnitude)
        features = np.zeros((n, 3))
        
        # Feature 1: Magnitude
        features[:, 0] = magnitude
        
        # Feature 2: Local mean (moving average)
        kernel = np.ones(self.config.window_size) / self.config.window_size
        features[:, 1] = np.convolve(magnitude, kernel, mode='same')
        
        # Feature 3: Local variance
        mag_sq = magnitude ** 2
        mean_sq = features[:, 1] ** 2
        e_x_sq = np.convolve(mag_sq, kernel, mode='same')
        features[:, 2] = np.maximum(0, e_x_sq - mean_sq)
        
        return features
    
    def train(self, signals: List[np.ndarray], labels: List[np.ndarray]):
        """
        Train SVM classifier on labeled data.
        
        Args:
            signals: List of complex radar signals
            labels: List of binary labels (1=target, 0=noise)
        """
        if not SKLEARN_AVAILABLE:
            raise ImportError("scikit-learn required for training")
        
        # Extract features from all signals
        X_all = []
        y_all = []
        
        for signal, label in zip(signals, labels):
            magnitude = np.abs(signal)
            features = self.extract_features(magnitude)
            X_all.append(features)
            y_all.append(label)
        
        X = np.vstack(X_all)
        y = np.concatenate(y_all)
        
        # Balance classes (subsample majority class)
        n_pos = np.sum(y == 1)
        n_neg = np.sum(y == 0)
        
        if n_neg > 10 * n_pos:
            # Subsample negatives
            neg_idx = np.where(y == 0)[0]
            keep_neg = np.random.choice(neg_idx, size=10 * n_pos, replace=False)
            pos_idx = np.where(y == 1)[0]
            keep_idx = np.concatenate([pos_idx, keep_neg])
            X = X[keep_idx]
            y = y[keep_idx]
        
        # Split for validation
        X_train, X_val, y_train, y_val = train_test_split(
            X, y, test_size=0.2, random_state=42, stratify=y
        )
        
        # Normalize features
        self.scaler = StandardScaler()
        X_train_scaled = self.scaler.fit_transform(X_train)
        X_val_scaled = self.scaler.transform(X_val)
        
        # Train SVM
        self.model = SVC(
            kernel='rbf',
            gamma='scale',
            C=10.0,
            probability=True,
            class_weight='balanced'
        )
        self.model.fit(X_train_scaled, y_train)
        
        # Validate
        y_pred = self.model.predict(X_val_scaled)
        print("\nValidation Results:")
        print(classification_report(y_val, y_pred, target_names=['Noise', 'Target']))
        
        self.trained = True
        
        print(f"\nTrained SVM with {len(self.model.support_vectors_)} support vectors")
    
    def detect(self, signal: np.ndarray, 
               prob_threshold: float = 0.95) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Apply ML-based detection.
        
        Args:
            signal: Complex radar signal
            prob_threshold: Probability threshold for detection
        
        Returns:
            (magnitude, probability, detections)
        """
        magnitude = np.abs(signal)
        
        if not self.trained:
            # Fallback to simple threshold if not trained
            threshold = np.mean(magnitude) * self.config.threshold_factor
            detections = magnitude > threshold
            probability = (magnitude / threshold).clip(0, 1)
            return magnitude, probability, detections
        
        # Extract and normalize features
        features = self.extract_features(magnitude)
        features_scaled = self.scaler.transform(features)
        
        # Get probability predictions
        probability = self.model.predict_proba(features_scaled)[:, 1]
        
        # Apply threshold
        detections = probability >= prob_threshold
        
        return magnitude, probability, detections
    
    def get_model(self) -> Tuple['SVC', 'StandardScaler']:
        """Get trained model and scaler for export."""
        if not self.trained:
            raise RuntimeError("Model not trained")
        return self.model, self.scaler


# =============================================================================
# JAMMER DETECTOR
# =============================================================================

class RGPODetector:
    """
    RGPO (Range Gate Pull-Off) jammer detector.
    
    [REQ-ECCM-01] Jammer detection using power ratio analysis
    """
    
    def __init__(self, power_ratio_threshold: float = 5.0):
        self.threshold = power_ratio_threshold
    
    def detect(self, magnitude: np.ndarray, local_mean: np.ndarray,
               detections: np.ndarray) -> np.ndarray:
        """
        Identify potential jammer returns among detections.
        
        Jammers have:
        - Very high power relative to local background
        - Broader range extent
        - Characteristic temporal signature
        """
        jammer_flags = np.zeros_like(detections, dtype=bool)
        
        # Power ratio analysis
        power_ratio = np.zeros_like(magnitude)
        mask = local_mean > 1e-10
        power_ratio[mask] = magnitude[mask] / local_mean[mask]
        
        # Flag high power ratio detections as potential jammers
        jammer_flags = detections & (power_ratio > self.threshold)
        
        return jammer_flags


# =============================================================================
# PERFORMANCE EVALUATION
# =============================================================================

@dataclass
class DetectionMetrics:
    """Detection performance metrics."""
    n_targets: int = 0
    n_detections: int = 0
    n_true_positives: int = 0
    n_false_positives: int = 0
    n_false_negatives: int = 0
    n_jammers_detected: int = 0
    
    @property
    def pd(self) -> float:
        """Probability of detection."""
        if self.n_targets == 0:
            return 0.0
        return self.n_true_positives / self.n_targets
    
    @property
    def pfa(self) -> float:
        """Probability of false alarm."""
        total_noise = self.n_false_positives + (self.n_detections - self.n_true_positives)
        if total_noise == 0:
            return 0.0
        return self.n_false_positives / max(1, total_noise)


def evaluate_detector(magnitude: np.ndarray, detections: np.ndarray,
                     true_target_indices: List[int], 
                     tolerance: int = 5) -> DetectionMetrics:
    """
    Evaluate detector performance against ground truth.
    
    Args:
        magnitude: Signal magnitude
        detections: Boolean detection array
        true_target_indices: Indices of actual targets
        tolerance: Index tolerance for matching
    """
    metrics = DetectionMetrics()
    metrics.n_targets = len(true_target_indices)
    
    detection_indices = np.where(detections)[0]
    metrics.n_detections = len(detection_indices)
    
    # Match detections to targets
    matched_targets = set()
    
    for det_idx in detection_indices:
        matched = False
        for i, target_idx in enumerate(true_target_indices):
            if abs(det_idx - target_idx) <= tolerance:
                matched = True
                matched_targets.add(i)
                break
        
        if not matched:
            metrics.n_false_positives += 1
    
    metrics.n_true_positives = len(matched_targets)
    metrics.n_false_negatives = metrics.n_targets - metrics.n_true_positives
    
    return metrics


# =============================================================================
# MAIN SIMULATION
# =============================================================================

def run_simulation(verbose: bool = True) -> Dict[str, Any]:
    """
    Run complete CFAR simulation with physics validation.
    
    [REQ-SIM-001] Multi-target + clutter + RGPO scenario
    """
    results = {}
    
    # Configuration
    radar_cfg = RadarConfig(fs=100e6, max_range=6000)
    
    targets = [
        TargetConfig(range_m=500, rcs=1.0, snr_db=15),
        TargetConfig(range_m=1200, rcs=0.5, snr_db=12),
        TargetConfig(range_m=1500, rcs=2.0, snr_db=18),
        TargetConfig(range_m=3000, rcs=1.2, snr_db=14),
        TargetConfig(range_m=4500, rcs=0.8, snr_db=13),
    ]
    
    clutter_cfg = ClutterConfig(sigma=0.2, distribution='rayleigh')
    
    jammer_cfg = JammerConfig(
        enabled=True,
        target_range=500,  # Spoofs first target
        pull_off=500,      # Creates false target at 1000m
        jnr_db=20
    )
    
    cfar_cfg = CFARConfig(window_size=20, guard_cells=4, threshold_factor=5.0)
    
    # Generate signal
    generator = RadarSignalGenerator(radar_cfg)
    range_bins, signal = generator.generate_scenario(targets, clutter_cfg, jammer_cfg)
    
    # True target indices
    true_indices = [int(2 * t.range_m / C * radar_cfg.fs) for t in targets]
    jammer_idx = int(2 * (jammer_cfg.target_range + jammer_cfg.pull_off) / C * radar_cfg.fs)
    
    if verbose:
        print("═" * 70)
        print("NX-MIMOSA Cognitive CFAR Simulation")
        print("═" * 70)
        print(f"\nTargets: {[t.range_m for t in targets]} m")
        print(f"RGPO Jammer: {jammer_cfg.target_range + jammer_cfg.pull_off} m")
        print(f"Clutter σ: {clutter_cfg.sigma}")
    
    # Standard CFAR
    std_cfar = StandardCFAR(cfar_cfg)
    std_mag, std_thresh, std_det = std_cfar.detect(signal)
    std_metrics = evaluate_detector(std_mag, std_det, true_indices)
    
    if verbose:
        print(f"\n--- Standard CA-CFAR ---")
        print(f"Detections: {np.sum(std_det)}")
        print(f"Pd: {std_metrics.pd:.1%}")
        print(f"False Alarms: {std_metrics.n_false_positives}")
    
    results['standard'] = {
        'detections': int(np.sum(std_det)),
        'pd': std_metrics.pd,
        'fa': std_metrics.n_false_positives,
        'metrics': std_metrics
    }
    
    # Cognitive CFAR (with training data)
    cog_cfar = CognitiveCFAR(cfar_cfg)
    
    if SKLEARN_AVAILABLE:
        # Generate training data
        print("\n--- Training Cognitive CFAR ---")
        train_signals = []
        train_labels = []
        
        for _ in range(50):
            _, sig = generator.generate_scenario(targets, clutter_cfg, None)
            label = np.zeros(len(sig))
            for idx in true_indices:
                if 0 <= idx < len(label):
                    label[max(0, idx-2):min(len(label), idx+3)] = 1
            train_signals.append(sig)
            train_labels.append(label)
        
        cog_cfar.train(train_signals, train_labels)
        
        # Test
        cog_mag, cog_prob, cog_det = cog_cfar.detect(signal, prob_threshold=0.95)
        cog_metrics = evaluate_detector(cog_mag, cog_det, true_indices)
        
        if verbose:
            print(f"\n--- Cognitive ML-CFAR ---")
            print(f"Detections: {np.sum(cog_det)}")
            print(f"Pd: {cog_metrics.pd:.1%}")
            print(f"False Alarms: {cog_metrics.n_false_positives}")
        
        results['cognitive'] = {
            'detections': int(np.sum(cog_det)),
            'pd': cog_metrics.pd,
            'fa': cog_metrics.n_false_positives,
            'metrics': cog_metrics
        }
        
        # RGPO Detection
        features = cog_cfar.extract_features(np.abs(signal))
        rgpo_detector = RGPODetector(power_ratio_threshold=5.0)
        jammer_flags = rgpo_detector.detect(np.abs(signal), features[:, 1], cog_det)
        
        results['jammer_detected'] = bool(np.any(jammer_flags))
        
        if verbose:
            print(f"\n--- RGPO Jammer Detection ---")
            print(f"Jammer flagged: {np.sum(jammer_flags)} bins")
            print(f"Near true jammer location: {np.any(jammer_flags[max(0,jammer_idx-10):jammer_idx+10])}")
    
    # Summary
    if verbose and SKLEARN_AVAILABLE:
        print("\n" + "═" * 70)
        print("SUMMARY")
        print("═" * 70)
        print(f"\n{'Metric':<25} {'Standard':<15} {'Cognitive':<15} {'Improvement'}")
        print("-" * 70)
        print(f"{'Detections':<25} {results['standard']['detections']:<15} {results['cognitive']['detections']:<15}")
        pd_std = f"{results['standard']['pd']:.1%}"
        pd_cog = f"{results['cognitive']['pd']:.1%}"
        print(f"{'Probability of Detection':<25} {pd_std:<15} {pd_cog:<15}")
        fa_reduction = results['standard']['fa'] - results['cognitive']['fa']
        print(f"{'False Alarms':<25} {results['standard']['fa']:<15} {results['cognitive']['fa']:<15} {fa_reduction:+d}")
        print(f"{'Jammer Detected':<25} {'-':<15} {'Yes' if results.get('jammer_detected') else 'No':<15}")
    
    return results


# =============================================================================
# MAIN
# =============================================================================

if __name__ == "__main__":
    results = run_simulation(verbose=True)
    
    print("\n✅ Simulation complete!")
    
    if SKLEARN_AVAILABLE:
        print("\nPhysics Validation:")
        print(f"  Pd (Cognitive): {results['cognitive']['pd']:.1%} (target: 100%)")
        print(f"  Pfa (Cognitive): {results['cognitive']['fa']} (target: 0)")
        print(f"  RGPO Detection: {'PASSED' if results.get('jammer_detected') else 'FAILED'}")
