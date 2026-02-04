#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════════════════════════════
NX-MIMOSA - JPDA (JOINT PROBABILISTIC DATA ASSOCIATION) IMPLEMENTATION
═══════════════════════════════════════════════════════════════════════════════════════════════════════

Joint Probabilistic Data Association Filter for Multi-Target Tracking
Handles measurement-to-track association in dense environments.

Features:
- Full JPDA with joint association probabilities
- Efficient implementation using cluster decomposition
- Gating for computational tractability
- Integration with UKF/CKF base filters
- Support for VS-IMM mode switching

Mathematical Foundation:
- Bar-Shalom & Fortmann, "Tracking and Data Fusion", 1988
- Bar-Shalom et al., "Estimation with Applications to Tracking", 2001

Author: Dr. Mladen Mešter / Nexellum d.o.o.
License: AGPL v3 / Commercial
═══════════════════════════════════════════════════════════════════════════════════════════════════════
"""

import numpy as np
from scipy.stats import chi2
from scipy.special import comb
from typing import List, Tuple, Dict, Optional, Any
from dataclasses import dataclass, field
from enum import IntEnum
import itertools
from collections import defaultdict

__version__ = "1.0.0"
__author__ = "Dr. Mladen Mešter"


# =============================================================================
# CONFIGURATION
# =============================================================================

@dataclass
class JPDAConfig:
    """JPDA configuration parameters."""
    # Gating
    gate_probability: float = 0.9999    # Probability that true measurement is in gate
    gate_threshold: float = None        # Chi-squared threshold (computed from gate_probability)
    
    # Association
    pd: float = 0.9                      # Detection probability
    lambda_fa: float = 1e-6             # False alarm density (per unit volume)
    
    # Clustering
    use_clustering: bool = True         # Use cluster decomposition
    max_cluster_size: int = 10          # Maximum tracks per cluster
    max_hypotheses: int = 1000          # Maximum hypotheses to evaluate
    
    # Numerical
    min_probability: float = 1e-10      # Minimum association probability
    
    def __post_init__(self):
        if self.gate_threshold is None:
            # Chi-squared threshold for given probability
            # Assuming 3D measurements (x, y, z)
            self.gate_threshold = chi2.ppf(self.gate_probability, df=3)


# =============================================================================
# DATA STRUCTURES
# =============================================================================

@dataclass
class Track:
    """Track state for JPDA."""
    track_id: int
    state: np.ndarray              # State vector
    covariance: np.ndarray         # State covariance
    filter: Any                    # Base filter (UKF/CKF)
    
    # Predicted values (after predict, before update)
    predicted_state: np.ndarray = None
    predicted_covariance: np.ndarray = None
    predicted_measurement: np.ndarray = None
    innovation_covariance: np.ndarray = None
    
    # Gated measurements
    gated_measurements: List[int] = field(default_factory=list)
    
    # Association probabilities
    association_probs: Dict[int, float] = field(default_factory=dict)
    
    def __post_init__(self):
        if self.predicted_state is None:
            self.predicted_state = self.state.copy()
        if self.predicted_covariance is None:
            self.predicted_covariance = self.covariance.copy()


@dataclass  
class Measurement:
    """Measurement for JPDA."""
    measurement_id: int
    z: np.ndarray                  # Measurement vector
    R: np.ndarray                  # Measurement noise covariance
    
    # Gated tracks
    gated_tracks: List[int] = field(default_factory=list)


@dataclass
class Cluster:
    """Cluster of tracks and measurements for joint processing."""
    cluster_id: int
    track_ids: List[int]
    measurement_ids: List[int]
    
    def size(self) -> int:
        return len(self.track_ids) + len(self.measurement_ids)


# =============================================================================
# JPDA FILTER
# =============================================================================

class JPDAFilter:
    """
    Joint Probabilistic Data Association Filter.
    
    Handles measurement-to-track association for multiple targets
    in a probabilistically optimal way.
    
    Algorithm:
    1. Predict all tracks
    2. Gate measurements to tracks
    3. Form clusters of coupled tracks/measurements
    4. Compute joint association probabilities per cluster
    5. Update tracks with weighted innovations
    
    Usage:
        jpda = JPDAFilter(config)
        
        # Add tracks
        jpda.add_track(track_id=1, state=x0, covariance=P0, filter=ukf)
        
        # Process measurements
        measurements = [Measurement(i, z, R) for i, z in enumerate(sensor_data)]
        jpda.process(measurements, dt=1.0)
        
        # Get updated states
        states = jpda.get_states()
    """
    
    def __init__(self, config: JPDAConfig = None):
        """
        Initialize JPDA filter.
        
        Args:
            config: JPDA configuration
        """
        self.config = config or JPDAConfig()
        self.tracks: Dict[int, Track] = {}
        self._next_track_id = 0
    
    def add_track(self, track_id: int, state: np.ndarray, 
                  covariance: np.ndarray, filter: Any) -> int:
        """
        Add a track to the filter.
        
        Args:
            track_id: Track identifier
            state: Initial state vector
            covariance: Initial state covariance
            filter: Base filter instance (UKF/CKF)
            
        Returns:
            Track ID
        """
        if track_id is None:
            track_id = self._next_track_id
            self._next_track_id += 1
        
        self.tracks[track_id] = Track(
            track_id=track_id,
            state=state.copy(),
            covariance=covariance.copy(),
            filter=filter
        )
        
        return track_id
    
    def remove_track(self, track_id: int):
        """Remove a track from the filter."""
        if track_id in self.tracks:
            del self.tracks[track_id]
    
    def process(self, measurements: List[Measurement], dt: float) -> Dict[int, np.ndarray]:
        """
        Process measurements with JPDA.
        
        Args:
            measurements: List of measurements
            dt: Time step
            
        Returns:
            Dict of {track_id: updated_state}
        """
        if not self.tracks or not measurements:
            return {tid: t.state for tid, t in self.tracks.items()}
        
        # Step 1: Predict all tracks
        self._predict_tracks(dt)
        
        # Step 2: Gate measurements to tracks
        self._gate_measurements(measurements)
        
        # Step 3: Form clusters
        clusters = self._form_clusters(measurements)
        
        # Step 4: Compute association probabilities per cluster
        for cluster in clusters:
            self._compute_association_probabilities(cluster, measurements)
        
        # Step 5: Update tracks with weighted innovations
        self._update_tracks(measurements)
        
        return {tid: t.state for tid, t in self.tracks.items()}
    
    def _predict_tracks(self, dt: float):
        """Predict all tracks forward in time."""
        for track in self.tracks.values():
            # Use the base filter's predict
            if hasattr(track.filter, 'predict'):
                track.filter.predict(dt)
                track.predicted_state = track.filter.get_state()
                track.predicted_covariance = track.filter.get_covariance()
            else:
                # Simple constant velocity prediction
                F = np.eye(len(track.state))
                n = len(track.state) // 2
                F[:n, n:] = np.eye(n) * dt
                track.predicted_state = F @ track.state
                Q = np.eye(len(track.state)) * 0.1
                track.predicted_covariance = F @ track.covariance @ F.T + Q
            
            # Predicted measurement (assuming H = [I, 0])
            n_meas = 3  # Assuming 3D measurements
            H = np.zeros((n_meas, len(track.predicted_state)))
            H[:n_meas, :n_meas] = np.eye(n_meas)
            
            track.predicted_measurement = H @ track.predicted_state
            
            # Clear previous gating
            track.gated_measurements = []
            track.association_probs = {}
    
    def _gate_measurements(self, measurements: List[Measurement]):
        """Gate measurements to tracks using Mahalanobis distance."""
        for meas in measurements:
            meas.gated_tracks = []
        
        for track in self.tracks.values():
            # Measurement model
            n_meas = len(measurements[0].z) if measurements else 3
            n_state = len(track.predicted_state)
            H = np.zeros((n_meas, n_state))
            H[:n_meas, :n_meas] = np.eye(n_meas)
            
            # Innovation covariance
            S = H @ track.predicted_covariance @ H.T + measurements[0].R
            track.innovation_covariance = S
            
            try:
                S_inv = np.linalg.inv(S)
            except np.linalg.LinAlgError:
                S_inv = np.linalg.pinv(S)
            
            for meas in measurements:
                # Innovation
                y = meas.z - track.predicted_measurement
                
                # Mahalanobis distance
                d2 = y.T @ S_inv @ y
                
                if d2 <= self.config.gate_threshold:
                    track.gated_measurements.append(meas.measurement_id)
                    meas.gated_tracks.append(track.track_id)
    
    def _form_clusters(self, measurements: List[Measurement]) -> List[Cluster]:
        """Form clusters of coupled tracks and measurements."""
        if not self.config.use_clustering:
            # Single cluster with everything
            return [Cluster(
                cluster_id=0,
                track_ids=list(self.tracks.keys()),
                measurement_ids=[m.measurement_id for m in measurements]
            )]
        
        # Build adjacency
        track_to_meas = defaultdict(set)
        meas_to_track = defaultdict(set)
        
        for track in self.tracks.values():
            for mid in track.gated_measurements:
                track_to_meas[track.track_id].add(mid)
                meas_to_track[mid].add(track.track_id)
        
        # Find connected components using union-find
        visited_tracks = set()
        visited_meas = set()
        clusters = []
        
        def bfs(start_track):
            """BFS to find connected component."""
            cluster_tracks = set()
            cluster_meas = set()
            
            track_queue = [start_track]
            while track_queue:
                tid = track_queue.pop(0)
                if tid in visited_tracks:
                    continue
                visited_tracks.add(tid)
                cluster_tracks.add(tid)
                
                for mid in track_to_meas[tid]:
                    if mid not in visited_meas:
                        visited_meas.add(mid)
                        cluster_meas.add(mid)
                        
                        for other_tid in meas_to_track[mid]:
                            if other_tid not in visited_tracks:
                                track_queue.append(other_tid)
            
            return cluster_tracks, cluster_meas
        
        for track_id in self.tracks.keys():
            if track_id not in visited_tracks:
                c_tracks, c_meas = bfs(track_id)
                if c_tracks:  # Non-empty cluster
                    clusters.append(Cluster(
                        cluster_id=len(clusters),
                        track_ids=list(c_tracks),
                        measurement_ids=list(c_meas)
                    ))
        
        return clusters
    
    def _compute_association_probabilities(self, cluster: Cluster, 
                                            measurements: List[Measurement]):
        """
        Compute joint association probabilities for a cluster.
        
        Uses enumeration of feasible joint events.
        """
        n_tracks = len(cluster.track_ids)
        n_meas = len(cluster.measurement_ids)
        
        if n_tracks == 0:
            return
        
        # Get measurement objects
        meas_dict = {m.measurement_id: m for m in measurements}
        cluster_meas = [meas_dict[mid] for mid in cluster.measurement_ids]
        
        # Compute likelihoods for all track-measurement pairs
        likelihoods = np.zeros((n_tracks, n_meas + 1))  # +1 for missed detection
        
        for i, tid in enumerate(cluster.track_ids):
            track = self.tracks[tid]
            
            # Missed detection likelihood
            likelihoods[i, n_meas] = 1.0 - self.config.pd
            
            # Measurement likelihoods
            S = track.innovation_covariance
            try:
                S_inv = np.linalg.inv(S)
                S_det = np.linalg.det(S)
            except np.linalg.LinAlgError:
                S_inv = np.linalg.pinv(S)
                S_det = max(np.linalg.det(S), 1e-10)
            
            for j, mid in enumerate(cluster.measurement_ids):
                if mid not in track.gated_measurements:
                    likelihoods[i, j] = 0.0
                    continue
                
                meas = meas_dict[mid]
                y = meas.z - track.predicted_measurement
                
                # Gaussian likelihood
                exponent = -0.5 * (y.T @ S_inv @ y)
                norm = 1.0 / np.sqrt((2 * np.pi) ** len(y) * S_det)
                
                likelihoods[i, j] = self.config.pd * norm * np.exp(exponent)
        
        # Enumerate feasible joint events
        # A joint event assigns each track to at most one measurement
        # and each measurement to at most one track
        
        # Initialize association probabilities
        for tid in cluster.track_ids:
            self.tracks[tid].association_probs = {mid: 0.0 for mid in cluster.measurement_ids}
            self.tracks[tid].association_probs[None] = 0.0  # Missed detection
        
        # Generate and evaluate joint events
        event_probs = []
        events = []
        
        # Use efficient enumeration
        assignments = self._enumerate_assignments(n_tracks, n_meas, likelihoods)
        
        total_prob = 0.0
        for assignment, prob in assignments:
            if prob < self.config.min_probability:
                continue
            
            event_probs.append(prob)
            events.append(assignment)
            total_prob += prob
        
        # Normalize and accumulate marginal probabilities
        if total_prob > 0:
            for event, prob in zip(events, event_probs):
                normalized_prob = prob / total_prob
                
                for i, mid_idx in enumerate(event):
                    tid = cluster.track_ids[i]
                    
                    if mid_idx == n_meas:  # Missed detection
                        self.tracks[tid].association_probs[None] += normalized_prob
                    else:
                        mid = cluster.measurement_ids[mid_idx]
                        self.tracks[tid].association_probs[mid] += normalized_prob
    
    def _enumerate_assignments(self, n_tracks: int, n_meas: int,
                               likelihoods: np.ndarray) -> List[Tuple[List[int], float]]:
        """
        Enumerate feasible track-to-measurement assignments.
        
        Returns list of (assignment, probability) tuples.
        Assignment is list where assignment[i] is measurement index for track i,
        or n_meas for missed detection.
        """
        assignments = []
        
        # For small problems, full enumeration
        if n_tracks <= self.config.max_cluster_size:
            # Generate all possible assignments
            # Each track can be assigned to any gated measurement or missed detection
            options = []
            for i in range(n_tracks):
                track_options = [n_meas]  # Missed detection always possible
                for j in range(n_meas):
                    if likelihoods[i, j] > 0:
                        track_options.append(j)
                options.append(track_options)
            
            # Enumerate valid combinations
            count = 0
            for combo in itertools.product(*options):
                if count >= self.config.max_hypotheses:
                    break
                
                # Check validity (no measurement assigned to multiple tracks)
                assigned_meas = [m for m in combo if m != n_meas]
                if len(assigned_meas) != len(set(assigned_meas)):
                    continue  # Invalid: duplicate assignment
                
                # Compute probability
                prob = 1.0
                for i, mid_idx in enumerate(combo):
                    prob *= likelihoods[i, mid_idx]
                
                # False alarm contribution
                n_false_alarms = n_meas - len(assigned_meas)
                prob *= (self.config.lambda_fa ** n_false_alarms)
                
                if prob > self.config.min_probability:
                    assignments.append((list(combo), prob))
                    count += 1
        else:
            # For large clusters, use greedy approximation
            # This is a simplification - full JPDA would use more sophisticated methods
            assignment = [n_meas] * n_tracks
            prob = 1.0
            
            used_meas = set()
            for i in range(n_tracks):
                best_j = n_meas
                best_like = likelihoods[i, n_meas]
                
                for j in range(n_meas):
                    if j not in used_meas and likelihoods[i, j] > best_like:
                        best_j = j
                        best_like = likelihoods[i, j]
                
                assignment[i] = best_j
                prob *= best_like
                
                if best_j != n_meas:
                    used_meas.add(best_j)
            
            assignments.append((assignment, prob))
        
        return assignments
    
    def _update_tracks(self, measurements: List[Measurement]):
        """Update tracks using weighted innovations."""
        meas_dict = {m.measurement_id: m for m in measurements}
        
        for track in self.tracks.values():
            if not track.association_probs:
                # No associations - keep predicted state
                track.state = track.predicted_state
                track.covariance = track.predicted_covariance
                continue
            
            # Compute weighted innovation
            n_meas = len(track.predicted_measurement)
            n_state = len(track.predicted_state)
            
            H = np.zeros((n_meas, n_state))
            H[:n_meas, :n_meas] = np.eye(n_meas)
            
            # Combined innovation
            y_combined = np.zeros(n_meas)
            beta_0 = track.association_probs.get(None, 0.0)  # Missed detection prob
            
            for mid, beta in track.association_probs.items():
                if mid is None:
                    continue
                if mid not in meas_dict:
                    continue
                
                meas = meas_dict[mid]
                y = meas.z - track.predicted_measurement
                y_combined += beta * y
            
            # Kalman gain
            S = track.innovation_covariance
            try:
                S_inv = np.linalg.inv(S)
            except np.linalg.LinAlgError:
                S_inv = np.linalg.pinv(S)
            
            K = track.predicted_covariance @ H.T @ S_inv
            
            # State update
            track.state = track.predicted_state + K @ y_combined
            
            # Covariance update (Joseph form with spread of innovations)
            P_standard = (np.eye(n_state) - K @ H) @ track.predicted_covariance
            
            # Spread of innovations term
            P_spread = np.zeros((n_state, n_state))
            for mid, beta in track.association_probs.items():
                if mid is None:
                    continue
                if mid not in meas_dict:
                    continue
                
                meas = meas_dict[mid]
                y = meas.z - track.predicted_measurement
                y_tilde = y - y_combined
                
                P_spread += beta * (K @ np.outer(y_tilde, y_tilde) @ K.T)
            
            track.covariance = beta_0 * track.predicted_covariance + \
                               (1 - beta_0) * P_standard + P_spread
            
            # Ensure symmetry and positive definiteness
            track.covariance = 0.5 * (track.covariance + track.covariance.T)
            min_eig = np.min(np.linalg.eigvalsh(track.covariance))
            if min_eig < 0:
                track.covariance += np.eye(n_state) * (abs(min_eig) + 1e-6)
            
            # Update base filter
            if hasattr(track.filter, 'set_state'):
                track.filter.set_state(track.state)
                track.filter.set_covariance(track.covariance)
    
    def get_states(self) -> Dict[int, np.ndarray]:
        """Get all track states."""
        return {tid: t.state.copy() for tid, t in self.tracks.items()}
    
    def get_covariances(self) -> Dict[int, np.ndarray]:
        """Get all track covariances."""
        return {tid: t.covariance.copy() for tid, t in self.tracks.items()}
    
    def get_association_probabilities(self) -> Dict[int, Dict[int, float]]:
        """Get association probabilities for all tracks."""
        return {tid: t.association_probs.copy() for tid, t in self.tracks.items()}


# =============================================================================
# JPDA-IMM: JPDA WITH INTERACTING MULTIPLE MODEL
# =============================================================================

class JPDAIMM:
    """
    JPDA combined with Interacting Multiple Model.
    
    Handles both data association uncertainty and motion model uncertainty.
    """
    
    def __init__(self, config: JPDAConfig = None,
                 mode_models: List[Any] = None,
                 transition_matrix: np.ndarray = None):
        """
        Initialize JPDA-IMM.
        
        Args:
            config: JPDA configuration
            mode_models: List of motion models
            transition_matrix: Mode transition probability matrix
        """
        self.config = config or JPDAConfig()
        self.mode_models = mode_models or []
        self.n_modes = len(self.mode_models)
        
        if transition_matrix is None and self.n_modes > 0:
            # Default: slight preference to stay in current mode
            self.tpm = 0.95 * np.eye(self.n_modes) + 0.05 / self.n_modes
        else:
            self.tpm = transition_matrix
        
        # Separate JPDA filters for each mode
        self.mode_filters: List[JPDAFilter] = [
            JPDAFilter(config) for _ in range(max(1, self.n_modes))
        ]
        
        # Mode probabilities per track
        self.mode_probs: Dict[int, np.ndarray] = {}
    
    def add_track(self, track_id: int, state: np.ndarray,
                  covariance: np.ndarray, filters: List[Any]) -> int:
        """Add track with multiple mode filters."""
        for i, (jpda, filt) in enumerate(zip(self.mode_filters, filters)):
            jpda.add_track(track_id, state, covariance, filt)
        
        # Initialize mode probabilities (uniform)
        self.mode_probs[track_id] = np.ones(self.n_modes) / self.n_modes
        
        return track_id
    
    def process(self, measurements: List[Measurement], dt: float) -> Dict[int, np.ndarray]:
        """Process measurements with JPDA-IMM."""
        if not self.mode_probs:
            return {}
        
        # IMM mixing step
        self._imm_mixing()
        
        # Process each mode with JPDA
        mode_states = []
        mode_likelihoods = []
        
        for i, jpda in enumerate(self.mode_filters):
            states = jpda.process(measurements, dt)
            mode_states.append(states)
            
            # Compute mode likelihood from innovation
            likelihoods = self._compute_mode_likelihoods(jpda, measurements)
            mode_likelihoods.append(likelihoods)
        
        # IMM mode probability update
        self._imm_mode_update(mode_likelihoods)
        
        # IMM output combination
        combined_states = self._imm_combine(mode_states)
        
        return combined_states
    
    def _imm_mixing(self):
        """IMM mixing step."""
        for track_id, mu in self.mode_probs.items():
            # Mixing probabilities
            c_bar = self.tpm.T @ mu
            mu_ij = np.zeros((self.n_modes, self.n_modes))
            
            for i in range(self.n_modes):
                for j in range(self.n_modes):
                    if c_bar[j] > 1e-10:
                        mu_ij[i, j] = self.tpm[i, j] * mu[i] / c_bar[j]
            
            # Mix states and covariances
            for j in range(self.n_modes):
                x_mixed = np.zeros_like(self.mode_filters[0].tracks[track_id].state)
                P_mixed = np.zeros_like(self.mode_filters[0].tracks[track_id].covariance)
                
                for i in range(self.n_modes):
                    x_i = self.mode_filters[i].tracks[track_id].state
                    x_mixed += mu_ij[i, j] * x_i
                
                for i in range(self.n_modes):
                    x_i = self.mode_filters[i].tracks[track_id].state
                    P_i = self.mode_filters[i].tracks[track_id].covariance
                    dx = x_i - x_mixed
                    P_mixed += mu_ij[i, j] * (P_i + np.outer(dx, dx))
                
                self.mode_filters[j].tracks[track_id].state = x_mixed
                self.mode_filters[j].tracks[track_id].covariance = P_mixed
    
    def _compute_mode_likelihoods(self, jpda: JPDAFilter,
                                   measurements: List[Measurement]) -> Dict[int, float]:
        """Compute mode likelihoods for each track."""
        likelihoods = {}
        
        for track_id, track in jpda.tracks.items():
            # Use innovation likelihood
            likelihood = 0.0
            
            for mid, beta in track.association_probs.items():
                if mid is None:
                    likelihood += beta  # Missed detection contribution
                else:
                    # Find measurement
                    for m in measurements:
                        if m.measurement_id == mid:
                            y = m.z - track.predicted_measurement
                            S = track.innovation_covariance
                            try:
                                S_inv = np.linalg.inv(S)
                                S_det = np.linalg.det(S)
                            except:
                                S_inv = np.linalg.pinv(S)
                                S_det = 1e-10
                            
                            exponent = -0.5 * (y.T @ S_inv @ y)
                            norm = 1.0 / np.sqrt((2*np.pi)**len(y) * max(S_det, 1e-10))
                            likelihood += beta * norm * np.exp(exponent)
                            break
            
            likelihoods[track_id] = max(likelihood, 1e-10)
        
        return likelihoods
    
    def _imm_mode_update(self, mode_likelihoods: List[Dict[int, float]]):
        """Update mode probabilities."""
        for track_id in self.mode_probs.keys():
            c_bar = self.tpm.T @ self.mode_probs[track_id]
            
            # Update
            for j in range(self.n_modes):
                if track_id in mode_likelihoods[j]:
                    self.mode_probs[track_id][j] = mode_likelihoods[j][track_id] * c_bar[j]
            
            # Normalize
            total = np.sum(self.mode_probs[track_id])
            if total > 1e-10:
                self.mode_probs[track_id] /= total
            else:
                self.mode_probs[track_id] = np.ones(self.n_modes) / self.n_modes
    
    def _imm_combine(self, mode_states: List[Dict[int, np.ndarray]]) -> Dict[int, np.ndarray]:
        """Combine mode estimates."""
        combined = {}
        
        for track_id, mu in self.mode_probs.items():
            x_combined = np.zeros_like(mode_states[0][track_id])
            
            for j in range(self.n_modes):
                x_combined += mu[j] * mode_states[j][track_id]
            
            combined[track_id] = x_combined
        
        return combined
    
    def get_mode_probabilities(self, track_id: int) -> np.ndarray:
        """Get mode probabilities for a track."""
        return self.mode_probs.get(track_id, np.ones(self.n_modes) / self.n_modes)


# =============================================================================
# MAIN - DEMONSTRATION
# =============================================================================

if __name__ == "__main__":
    print("="*80)
    print("NX-MIMOSA JPDA (JOINT PROBABILISTIC DATA ASSOCIATION)")
    print("="*80)
    
    # Configuration
    config = JPDAConfig(
        gate_probability=0.9999,
        pd=0.9,
        lambda_fa=1e-6,
        use_clustering=True,
    )
    
    print(f"\n📋 Configuration:")
    print(f"  Gate probability: {config.gate_probability}")
    print(f"  Gate threshold: {config.gate_threshold:.2f}")
    print(f"  Detection probability: {config.pd}")
    print(f"  False alarm density: {config.lambda_fa}")
    
    # Create JPDA filter
    jpda = JPDAFilter(config)
    
    # Add tracks
    print(f"\n🎯 Initializing tracks...")
    
    # Track 1: Moving east
    jpda.add_track(
        track_id=1,
        state=np.array([0, 0, 10000, 200, 0, 0]),  # [x, y, z, vx, vy, vz]
        covariance=np.diag([100, 100, 100, 10, 10, 10]),
        filter=None  # Placeholder
    )
    
    # Track 2: Moving north
    jpda.add_track(
        track_id=2,
        state=np.array([1000, 0, 10000, 0, 200, 0]),
        covariance=np.diag([100, 100, 100, 10, 10, 10]),
        filter=None
    )
    
    # Track 3: Close to track 1 (challenging association)
    jpda.add_track(
        track_id=3,
        state=np.array([100, 50, 10000, 195, 10, 0]),
        covariance=np.diag([100, 100, 100, 10, 10, 10]),
        filter=None
    )
    
    print(f"  Added {len(jpda.tracks)} tracks")
    
    # Generate measurements
    print(f"\n📡 Generating measurements...")
    
    np.random.seed(42)
    R = np.diag([50**2, 50**2, 100**2])  # Measurement noise
    
    measurements = [
        # Measurement from track 1
        Measurement(
            measurement_id=0,
            z=np.array([210, 5, 10010]) + np.random.multivariate_normal([0,0,0], R),
            R=R
        ),
        # Measurement from track 2
        Measurement(
            measurement_id=1,
            z=np.array([1005, 210, 9990]) + np.random.multivariate_normal([0,0,0], R),
            R=R
        ),
        # Measurement from track 3 (close to track 1)
        Measurement(
            measurement_id=2,
            z=np.array([305, 65, 10005]) + np.random.multivariate_normal([0,0,0], R),
            R=R
        ),
        # False alarm
        Measurement(
            measurement_id=3,
            z=np.array([5000, 5000, 8000]),
            R=R
        ),
    ]
    
    print(f"  Generated {len(measurements)} measurements (including 1 false alarm)")
    
    # Process
    print(f"\n⚙️ Processing with JPDA...")
    dt = 1.0
    states = jpda.process(measurements, dt)
    
    print(f"\n📊 Results:")
    print(f"\n  Association Probabilities:")
    for tid, probs in jpda.get_association_probabilities().items():
        print(f"    Track {tid}:")
        for mid, prob in probs.items():
            if prob > 0.01:
                mid_str = f"M{mid}" if mid is not None else "Miss"
                print(f"      {mid_str}: {prob:.3f}")
    
    print(f"\n  Updated States:")
    for tid, state in states.items():
        print(f"    Track {tid}: pos=({state[0]:.1f}, {state[1]:.1f}, {state[2]:.1f}), "
              f"vel=({state[3]:.1f}, {state[4]:.1f}, {state[5]:.1f})")
    
    print("\n" + "="*80)
    print("✓ JPDA implementation ready for multi-target tracking")
    print("="*80)
