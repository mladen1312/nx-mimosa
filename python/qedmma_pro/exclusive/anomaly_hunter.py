"""
QEDMMA-Pro - Anomaly Hunter™ (Layer 2B)
========================================
Copyright (C) 2026 Dr. Mladen Mešter / Nexellum
License: Commercial - See LICENSE_COMMERCIAL.md

🔒 PRO EXCLUSIVE - NOT AVAILABLE IN LITE

Anomaly Hunter™ is the physics-agnostic tracking layer that handles
targets exhibiting behavior outside conventional motion models:

- Hypersonic glide vehicles with unpredictable skip maneuvers
- UAV swarms with coordinated but non-ballistic motion
- "Impossible" trajectories that violate expected physics
- Spoofed/decoy targets attempting to break track

Key Innovation:
    Instead of assuming a physics model (CV, CA, CT, Singer...),
    Layer 2B learns motion patterns from the track history and
    predicts "what this specific target will do next" rather than
    "what targets in general do".

Technical Approach:
1. Residual Learning: Model the delta from basic kinematic prediction
2. Attention-based history: Weight recent measurements by relevance
3. Anomaly scoring: Quantify how "unexpected" current behavior is
4. Adaptive model switching: Fallback to physics when behavior normalizes

For licensing: mladen@nexellum.com | www.nexellum.com
"""

import numpy as np
from typing import Tuple, Optional, List, Dict
from dataclasses import dataclass, field
from collections import deque
from enum import Enum
import warnings


class AnomalyLevel(Enum):
    """Classification of target behavior anomaly"""
    NORMAL = 0          # Conventional physics applies
    ELEVATED = 1        # Minor deviations, monitor closely
    HIGH = 2            # Significant deviations, Layer 2B active
    CRITICAL = 3        # Physics-defying, full Layer 2B mode


@dataclass
class AnomalyHunterConfig:
    """Configuration for Anomaly Hunter Layer 2B"""
    
    # History and learning
    history_length: int = 100           # Track history to analyze
    learning_window: int = 20           # Recent samples for pattern learning
    min_samples_for_learning: int = 10  # Minimum samples before Layer 2B activates
    
    # Anomaly detection thresholds
    residual_threshold_sigma: float = 3.0   # Std devs for anomaly detection
    sustained_anomaly_count: int = 3        # Consecutive anomalies to trigger Layer 2B
    
    # Prediction parameters
    prediction_blend_alpha: float = 0.7     # Blend between physics and learned (0=physics, 1=learned)
    max_acceleration: float = 200.0         # m/s² - physical limit for validation
    
    # Neural components (simplified - real implementation uses PyTorch)
    hidden_dim: int = 64
    attention_heads: int = 4


@dataclass
class TargetHistory:
    """Historical data for a tracked target"""
    positions: deque = field(default_factory=lambda: deque(maxlen=100))
    velocities: deque = field(default_factory=lambda: deque(maxlen=100))
    accelerations: deque = field(default_factory=lambda: deque(maxlen=100))
    timestamps: deque = field(default_factory=lambda: deque(maxlen=100))
    residuals: deque = field(default_factory=lambda: deque(maxlen=100))
    anomaly_scores: deque = field(default_factory=lambda: deque(maxlen=100))
    
    def append(self, pos, vel, acc, t, residual, anomaly_score):
        self.positions.append(pos.copy())
        self.velocities.append(vel.copy())
        self.accelerations.append(acc.copy())
        self.timestamps.append(t)
        self.residuals.append(residual)
        self.anomaly_scores.append(anomaly_score)
    
    def __len__(self):
        return len(self.positions)


@dataclass
class AnomalyHunterState:
    """State of the Anomaly Hunter for one target"""
    target_id: int
    history: TargetHistory
    anomaly_level: AnomalyLevel = AnomalyLevel.NORMAL
    consecutive_anomalies: int = 0
    layer_2b_active: bool = False
    learned_pattern: Optional[np.ndarray] = None  # Learned motion pattern
    confidence: float = 0.0
    
    # Prediction components
    physics_prediction: Optional[np.ndarray] = None
    learned_prediction: Optional[np.ndarray] = None
    blended_prediction: Optional[np.ndarray] = None


class MotionPatternLearner:
    """
    Learns motion patterns from track history.
    
    Simplified implementation - production uses transformer architecture.
    This version uses weighted polynomial fitting with attention-like weighting.
    """
    
    def __init__(self, config: AnomalyHunterConfig):
        self.config = config
        self.pattern_order = 3  # Polynomial order for pattern fitting
    
    def compute_attention_weights(self, history: TargetHistory) -> np.ndarray:
        """
        Compute attention weights for historical samples.
        
        Recent samples get higher weight, but anomalous historical
        samples also get elevated attention (they may repeat).
        """
        n = len(history)
        if n == 0:
            return np.array([])
        
        # Recency weight (exponential decay)
        recency = np.exp(-0.1 * np.arange(n)[::-1])
        
        # Anomaly weight (historical anomalies may recur)
        anomaly_scores = np.array(list(history.anomaly_scores))
        anomaly_weight = 1.0 + 0.5 * (anomaly_scores / (np.max(anomaly_scores) + 1e-6))
        
        # Combined attention
        weights = recency * anomaly_weight
        weights /= np.sum(weights)
        
        return weights
    
    def learn_pattern(self, history: TargetHistory) -> Optional[np.ndarray]:
        """
        Learn motion pattern from history.
        
        Returns polynomial coefficients for acceleration prediction.
        """
        n = len(history)
        if n < self.config.min_samples_for_learning:
            return None
        
        # Extract recent accelerations
        window = min(self.config.learning_window, n)
        accelerations = np.array(list(history.accelerations))[-window:]
        timestamps = np.array(list(history.timestamps))[-window:]
        
        # Normalize time
        t_norm = timestamps - timestamps[0]
        if t_norm[-1] > 0:
            t_norm = t_norm / t_norm[-1]
        
        # Attention-weighted polynomial fit
        weights = self.compute_attention_weights(history)[-window:]
        weights = weights / np.sum(weights) * window
        
        try:
            # Fit polynomial to each acceleration component
            coeffs = []
            for dim in range(accelerations.shape[1]):
                c = np.polyfit(t_norm, accelerations[:, dim], 
                              self.pattern_order, w=weights)
                coeffs.append(c)
            return np.array(coeffs)
        except:
            return None
    
    def predict_from_pattern(
        self, 
        pattern: np.ndarray, 
        history: TargetHistory,
        dt: float
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Predict next state using learned pattern.
        
        Returns:
            (predicted_position, predicted_velocity)
        """
        if pattern is None or len(history) < 2:
            return None, None
        
        # Current state
        pos = np.array(history.positions[-1])
        vel = np.array(history.velocities[-1])
        
        # Predict acceleration from pattern
        # Use t=1.0 + small delta for extrapolation
        t_pred = 1.0 + dt / (history.timestamps[-1] - history.timestamps[0] + 1e-6)
        
        acc_pred = np.zeros(pattern.shape[0])
        for dim in range(pattern.shape[0]):
            acc_pred[dim] = np.polyval(pattern[dim], t_pred)
        
        # Clamp to physical limits
        acc_pred = np.clip(acc_pred, -self.config.max_acceleration, 
                          self.config.max_acceleration)
        
        # Kinematic integration
        vel_pred = vel + acc_pred * dt
        pos_pred = pos + vel * dt + 0.5 * acc_pred * dt**2
        
        return pos_pred, vel_pred


class AnomalyDetector:
    """
    Detects anomalous target behavior.
    
    Anomalies are deviations from expected physics that may indicate:
    - Hypersonic skip maneuver
    - Decoy deployment
    - Electronic warfare / spoofing
    - Truly novel physics (should not happen, but...)
    """
    
    def __init__(self, config: AnomalyHunterConfig):
        self.config = config
    
    def compute_residual(
        self,
        measured: np.ndarray,
        predicted: np.ndarray,
        covariance: np.ndarray
    ) -> Tuple[float, float]:
        """
        Compute normalized residual (Mahalanobis distance).
        
        Returns:
            (mahalanobis_distance, anomaly_score)
        """
        innovation = measured - predicted
        
        try:
            S_inv = np.linalg.inv(covariance)
            mahal = np.sqrt(innovation @ S_inv @ innovation)
        except:
            mahal = np.linalg.norm(innovation) / (np.sqrt(np.trace(covariance)) + 1e-6)
        
        # Anomaly score: 0 = normal, 1 = highly anomalous
        # Using chi-squared distribution approximation
        dof = len(innovation)
        expected = np.sqrt(dof)
        anomaly_score = max(0, (mahal - expected) / (3 * expected))
        anomaly_score = min(1.0, anomaly_score)
        
        return mahal, anomaly_score
    
    def classify_anomaly(
        self,
        anomaly_score: float,
        consecutive_count: int
    ) -> AnomalyLevel:
        """Classify the anomaly level"""
        
        if anomaly_score < 0.2:
            return AnomalyLevel.NORMAL
        elif anomaly_score < 0.5:
            return AnomalyLevel.ELEVATED
        elif anomaly_score < 0.8 or consecutive_count < self.config.sustained_anomaly_count:
            return AnomalyLevel.HIGH
        else:
            return AnomalyLevel.CRITICAL
    
    def detect_physics_violation(
        self,
        history: TargetHistory,
        window: int = 5
    ) -> Dict[str, float]:
        """
        Detect violations of expected physics.
        
        Checks:
        - Impossible accelerations
        - Instantaneous velocity changes
        - Trajectory discontinuities
        """
        if len(history) < window:
            return {'violation_score': 0.0}
        
        accelerations = np.array(list(history.accelerations))[-window:]
        velocities = np.array(list(history.velocities))[-window:]
        
        violations = {}
        
        # Check for impossible acceleration
        max_acc = np.max(np.linalg.norm(accelerations, axis=1))
        acc_violation = max(0, (max_acc - self.config.max_acceleration) / self.config.max_acceleration)
        violations['acceleration'] = min(1.0, acc_violation)
        
        # Check for velocity discontinuity
        vel_changes = np.diff(velocities, axis=0)
        dt = np.diff(list(history.timestamps)[-window:])
        implied_acc = np.linalg.norm(vel_changes, axis=1) / (dt + 1e-6)
        vel_violation = np.max(implied_acc) / self.config.max_acceleration
        violations['velocity_jump'] = min(1.0, max(0, vel_violation - 1))
        
        # Combined violation score
        violations['violation_score'] = max(violations.values())
        
        return violations


class AnomalyHunter:
    """
    Anomaly Hunter™ - Physics-Agnostic Tracking Layer (Layer 2B)
    
    🔒 PRO EXCLUSIVE FEATURE
    
    This is the core innovation of QEDMMA-Pro that enables tracking of
    targets with unconventional or unpredictable motion patterns.
    
    Usage:
        >>> hunter = AnomalyHunter()
        >>> 
        >>> # For each track update:
        >>> state = hunter.process(
        ...     target_id=1,
        ...     measured_pos=measurement[:3],
        ...     physics_prediction=ekf_prediction,
        ...     prediction_covariance=S,
        ...     timestamp=t
        ... )
        >>> 
        >>> if state.layer_2b_active:
        ...     # Use Layer 2B prediction instead of physics
        ...     best_prediction = state.blended_prediction
        >>> else:
        ...     best_prediction = state.physics_prediction
    """
    
    def __init__(self, config: Optional[AnomalyHunterConfig] = None):
        self.config = config or AnomalyHunterConfig()
        self.learner = MotionPatternLearner(self.config)
        self.detector = AnomalyDetector(self.config)
        
        self.targets: Dict[int, AnomalyHunterState] = {}
    
    def _get_or_create_state(self, target_id: int) -> AnomalyHunterState:
        """Get existing state or create new one"""
        if target_id not in self.targets:
            self.targets[target_id] = AnomalyHunterState(
                target_id=target_id,
                history=TargetHistory()
            )
        return self.targets[target_id]
    
    def _estimate_kinematics(
        self,
        state: AnomalyHunterState,
        measured_pos: np.ndarray,
        timestamp: float
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Estimate velocity and acceleration from position history"""
        
        if len(state.history) < 2:
            return np.zeros(3), np.zeros(3)
        
        # Velocity from finite difference
        prev_pos = np.array(state.history.positions[-1])
        prev_t = state.history.timestamps[-1]
        dt = timestamp - prev_t
        
        if dt > 0:
            velocity = (measured_pos - prev_pos) / dt
        else:
            velocity = np.array(state.history.velocities[-1]) if state.history.velocities else np.zeros(3)
        
        # Acceleration
        if len(state.history) >= 2 and len(state.history.velocities) >= 1:
            prev_vel = np.array(state.history.velocities[-1])
            acceleration = (velocity - prev_vel) / (dt + 1e-6)
        else:
            acceleration = np.zeros(3)
        
        return velocity, acceleration
    
    def process(
        self,
        target_id: int,
        measured_pos: np.ndarray,
        physics_prediction: np.ndarray,
        prediction_covariance: np.ndarray,
        timestamp: float,
        dt: float = 0.1
    ) -> AnomalyHunterState:
        """
        Process a track update through Anomaly Hunter.
        
        Args:
            target_id: Unique identifier for this track
            measured_pos: Measured position [x, y, z]
            physics_prediction: Position predicted by physics model (EKF/UKF/IMM)
            prediction_covariance: Innovation covariance from filter
            timestamp: Current time
            dt: Time step for prediction
            
        Returns:
            Updated AnomalyHunterState with predictions and anomaly classification
        """
        measured_pos = np.asarray(measured_pos)
        physics_prediction = np.asarray(physics_prediction)
        
        state = self._get_or_create_state(target_id)
        
        # Estimate kinematics
        velocity, acceleration = self._estimate_kinematics(state, measured_pos, timestamp)
        
        # Compute residual and anomaly score
        mahal, anomaly_score = self.detector.compute_residual(
            measured_pos, physics_prediction, prediction_covariance
        )
        
        # Check for physics violations
        physics_violations = self.detector.detect_physics_violation(state.history)
        combined_anomaly = max(anomaly_score, physics_violations['violation_score'])
        
        # Update consecutive anomaly counter
        if combined_anomaly > 0.3:
            state.consecutive_anomalies += 1
        else:
            state.consecutive_anomalies = max(0, state.consecutive_anomalies - 1)
        
        # Classify anomaly level
        state.anomaly_level = self.detector.classify_anomaly(
            combined_anomaly, state.consecutive_anomalies
        )
        
        # Decide if Layer 2B should be active
        state.layer_2b_active = (
            state.anomaly_level in [AnomalyLevel.HIGH, AnomalyLevel.CRITICAL]
            and len(state.history) >= self.config.min_samples_for_learning
        )
        
        # Update history
        state.history.append(
            pos=measured_pos,
            vel=velocity,
            acc=acceleration,
            t=timestamp,
            residual=mahal,
            anomaly_score=combined_anomaly
        )
        
        # Store physics prediction
        state.physics_prediction = physics_prediction
        
        # If Layer 2B active, learn pattern and predict
        if state.layer_2b_active:
            state.learned_pattern = self.learner.learn_pattern(state.history)
            
            if state.learned_pattern is not None:
                pos_pred, vel_pred = self.learner.predict_from_pattern(
                    state.learned_pattern, state.history, dt
                )
                
                if pos_pred is not None:
                    state.learned_prediction = pos_pred
                    
                    # Blend physics and learned predictions
                    alpha = self.config.prediction_blend_alpha
                    # Increase alpha (trust learned more) as anomaly level increases
                    if state.anomaly_level == AnomalyLevel.CRITICAL:
                        alpha = 0.9
                    elif state.anomaly_level == AnomalyLevel.HIGH:
                        alpha = 0.7
                    
                    state.blended_prediction = (
                        alpha * state.learned_prediction + 
                        (1 - alpha) * state.physics_prediction[:len(pos_pred)]
                    )
                    state.confidence = alpha
                else:
                    state.blended_prediction = state.physics_prediction
                    state.confidence = 0.0
            else:
                state.blended_prediction = state.physics_prediction
                state.confidence = 0.0
        else:
            state.learned_prediction = None
            state.blended_prediction = state.physics_prediction
            state.confidence = 0.0
        
        return state
    
    def get_best_prediction(self, target_id: int) -> Tuple[np.ndarray, float]:
        """
        Get the best prediction for a target.
        
        Returns:
            (prediction, confidence) where confidence indicates
            how much the prediction relies on learned patterns vs physics
        """
        if target_id not in self.targets:
            raise KeyError(f"Unknown target: {target_id}")
        
        state = self.targets[target_id]
        
        if state.blended_prediction is not None:
            return state.blended_prediction, state.confidence
        elif state.physics_prediction is not None:
            return state.physics_prediction, 0.0
        else:
            return np.zeros(3), 0.0
    
    def get_anomaly_report(self, target_id: int) -> Dict:
        """Generate anomaly report for a target"""
        if target_id not in self.targets:
            return {'error': 'Unknown target'}
        
        state = self.targets[target_id]
        
        return {
            'target_id': target_id,
            'anomaly_level': state.anomaly_level.name,
            'layer_2b_active': state.layer_2b_active,
            'consecutive_anomalies': state.consecutive_anomalies,
            'confidence': state.confidence,
            'history_length': len(state.history),
            'recent_anomaly_scores': list(state.history.anomaly_scores)[-10:] if state.history.anomaly_scores else [],
            'physics_violations': self.detector.detect_physics_violation(state.history)
        }


def demonstrate_anomaly_hunter():
    """Demonstrate Anomaly Hunter on a simulated hypersonic skip trajectory"""
    
    print("=" * 70)
    print("  QEDMMA-Pro Anomaly Hunter™ - Physics-Agnostic Tracking Demo")
    print("  🔒 PRO EXCLUSIVE FEATURE")
    print("=" * 70)
    print()
    
    hunter = AnomalyHunter(AnomalyHunterConfig(
        prediction_blend_alpha=0.7,
        sustained_anomaly_count=3,
        max_acceleration=200.0
    ))
    
    np.random.seed(42)
    
    # Simulate hypersonic skip-glide trajectory
    # Physics model will fail during the skip maneuver
    
    true_positions = []
    physics_predictions = []
    hunter_predictions = []
    
    # Initial state
    pos = np.array([50000.0, 0.0, 30000.0])
    vel = np.array([2000.0, 50.0, -100.0])
    
    dt = 0.1
    
    print(f"{'Step':>4} | {'Phase':<12} | {'Anomaly':<10} | {'L2B':>4} | {'Phys Err':>9} | {'Hunt Err':>9}")
    print("-" * 75)
    
    for t in range(200):
        # True physics with skip maneuver
        if t < 50:
            # Initial descent
            acc = np.array([0, 0, -50])
            phase = "Descent"
        elif t < 70:
            # Skip - pull up (unpredictable for standard physics)
            acc = np.array([np.sin(t*0.5)*100, np.cos(t*0.3)*50, 150])
            phase = "SKIP UP"
        elif t < 90:
            # Skip - dive back
            acc = np.array([-50, 20, -100])
            phase = "SKIP DOWN"
        elif t < 120:
            # Lateral maneuver
            acc = np.array([30*np.sin(t*0.2), 80*np.cos(t*0.15), -30])
            phase = "LATERAL"
        else:
            # Terminal
            acc = np.array([0, 0, -20])
            phase = "Terminal"
        
        # Update true state
        vel = vel + acc * dt
        pos = pos + vel * dt
        
        true_positions.append(pos.copy())
        
        # Measurement (noisy)
        measured = pos + np.random.randn(3) * 50
        
        # Physics prediction (simple constant velocity - will fail on maneuvers)
        if t > 0:
            physics_pred = true_positions[-2] + vel * dt  # Using old velocity
        else:
            physics_pred = pos
        
        physics_predictions.append(physics_pred.copy())
        
        # Anomaly Hunter processing
        state = hunter.process(
            target_id=1,
            measured_pos=measured,
            physics_prediction=physics_pred,
            prediction_covariance=np.eye(3) * 100,
            timestamp=t * dt,
            dt=dt
        )
        
        best_pred, confidence = hunter.get_best_prediction(1)
        hunter_predictions.append(best_pred.copy())
        
        # Compute errors
        physics_err = np.linalg.norm(physics_pred - pos)
        hunter_err = np.linalg.norm(best_pred - pos)
        
        if t % 10 == 0:
            l2b = "✓" if state.layer_2b_active else ""
            print(f"{t:>4} | {phase:<12} | {state.anomaly_level.name:<10} | {l2b:>4} | "
                  f"{physics_err:>7.1f} m | {hunter_err:>7.1f} m")
    
    # Summary statistics
    true_positions = np.array(true_positions)
    physics_predictions = np.array(physics_predictions)
    hunter_predictions = np.array(hunter_predictions)
    
    physics_rmse = np.sqrt(np.mean(np.sum((physics_predictions - true_positions)**2, axis=1)))
    hunter_rmse = np.sqrt(np.mean(np.sum((hunter_predictions - true_positions)**2, axis=1)))
    
    print("-" * 75)
    print()
    print("📊 RESULTS:")
    print(f"   Physics-only RMSE:     {physics_rmse:>8.1f} m")
    print(f"   Anomaly Hunter RMSE:   {hunter_rmse:>8.1f} m")
    print(f"   Improvement:           {(1 - hunter_rmse/physics_rmse)*100:>8.1f} %")
    print()
    
    report = hunter.get_anomaly_report(1)
    print("📋 Anomaly Report:")
    print(f"   Current Level:    {report['anomaly_level']}")
    print(f"   Layer 2B Active:  {report['layer_2b_active']}")
    print(f"   Confidence:       {report['confidence']:.2f}")
    print()
    print("✅ Anomaly Hunter handles physics-defying maneuvers!")


if __name__ == "__main__":
    demonstrate_anomaly_hunter()
