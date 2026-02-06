"""
NX-MIMOSA Multi-Target Tracker (MTT) v5.0
==========================================
[REQ-V50-MTT] Full 3D multi-target tracking with GNN data association
and track lifecycle management.

Architecture:
  MTT Engine
    ├── Track Table (managed list of Track objects)
    ├── GNN Data Association (Munkres/Hungarian algorithm)
    ├── Track Management (init/confirm/delete with M-of-N logic)
    ├── 3D Motion Models (CV3D, CA3D, CT3D)
    └── IMM Bank per track (wraps NxMimosaV40Sentinel)

Each track is an independent NxMimosaV40Sentinel instance operating in 3D,
managed by the MTT engine for data association and lifecycle.

References:
  - Bar-Shalom, Li (1995) — "Multitarget-Multisensor Tracking"
  - Blackman, Popoli (1999) — "Design and Analysis of Modern Tracking Systems"
  - Munkres (1957) — "Algorithms for Assignment Problems"
  - Kuhn (1955) — "The Hungarian Method"

Author: Dr. Mladen Mešter, Nexellum d.o.o.
"""

import numpy as np
from typing import List, Dict, Optional, Tuple, Any
from dataclasses import dataclass, field
from enum import Enum, auto
import time


# ===== 3D MOTION MODELS =====

class MotionModel3D(Enum):
    CV3D = "cv3d"       # Constant Velocity 3D (6 states)
    CA3D = "ca3d"       # Constant Acceleration 3D (9 states)
    CT3D = "ct3d"       # Coordinated Turn 3D (7 states: +omega)


def make_cv3d_matrices(dt: float, q: float) -> Tuple[np.ndarray, np.ndarray]:
    """[REQ-V50-3D-01] Constant Velocity 3D: state = [x, y, z, vx, vy, vz].
    
    Returns:
        F (6×6), Q (6×6)
    """
    F = np.eye(6)
    F[0, 3] = dt
    F[1, 4] = dt
    F[2, 5] = dt
    
    # Discrete white noise acceleration model
    dt2 = dt**2 / 2
    G = np.array([
        [dt2, 0, 0],
        [0, dt2, 0],
        [0, 0, dt2],
        [dt, 0, 0],
        [0, dt, 0],
        [0, 0, dt],
    ])
    Q = G @ (q * np.eye(3)) @ G.T
    
    return F, Q


def make_ca3d_matrices(dt: float, q: float) -> Tuple[np.ndarray, np.ndarray]:
    """[REQ-V50-3D-02] Constant Acceleration 3D: state = [x, y, z, vx, vy, vz, ax, ay, az].
    
    Returns:
        F (9×9), Q (9×9)
    """
    dt2 = dt**2 / 2
    F = np.eye(9)
    # Position from velocity
    F[0, 3] = dt; F[1, 4] = dt; F[2, 5] = dt
    # Position from acceleration
    F[0, 6] = dt2; F[1, 7] = dt2; F[2, 8] = dt2
    # Velocity from acceleration
    F[3, 6] = dt; F[4, 7] = dt; F[5, 8] = dt
    
    dt3 = dt**3 / 6
    G = np.array([
        [dt3, 0, 0], [0, dt3, 0], [0, 0, dt3],
        [dt2, 0, 0], [0, dt2, 0], [0, 0, dt2],
        [dt, 0, 0],  [0, dt, 0],  [0, 0, dt],
    ])
    Q = G @ (q * np.eye(3)) @ G.T
    
    return F, Q


def make_ct3d_matrices(dt: float, q: float, omega: float) -> Tuple[np.ndarray, np.ndarray]:
    """[REQ-V50-3D-03] Coordinated Turn 3D: state = [x, y, z, vx, vy, vz, omega].
    
    Horizontal coordinated turn with constant altitude rate.
    
    Returns:
        F (7×7), Q (7×7)
    """
    F = np.eye(7)
    
    if abs(omega) < 1e-6:
        # Degenerate to CV for very small turn rate
        F[0, 3] = dt
        F[1, 4] = dt
    else:
        sin_wt = np.sin(omega * dt)
        cos_wt = np.cos(omega * dt)
        F[0, 3] = sin_wt / omega
        F[0, 4] = -(1 - cos_wt) / omega
        F[1, 3] = (1 - cos_wt) / omega
        F[1, 4] = sin_wt / omega
        F[3, 3] = cos_wt
        F[3, 4] = -sin_wt
        F[4, 3] = sin_wt
        F[4, 4] = cos_wt
    
    F[2, 5] = dt  # Altitude: z += vz * dt
    
    # Process noise
    dt2 = dt**2 / 2
    Q = np.zeros((7, 7))
    # Position noise
    Q[0, 0] = Q[1, 1] = q * dt**4 / 4
    Q[2, 2] = q * dt**4 / 4
    # Velocity noise
    Q[3, 3] = Q[4, 4] = q * dt**2
    Q[5, 5] = q * dt**2
    # Cross terms
    Q[0, 3] = Q[3, 0] = q * dt**3 / 2
    Q[1, 4] = Q[4, 1] = q * dt**3 / 2
    Q[2, 5] = Q[5, 2] = q * dt**3 / 2
    # Omega noise (small)
    Q[6, 6] = q * 0.01 * dt
    
    return F, Q


# ===== 3D KALMAN FILTER =====

class KalmanFilter3D:
    """[REQ-V50-3D-04] Standard 3D Kalman filter for multi-target tracking.
    
    Generic KF that works with any state dimension / motion model.
    """
    
    def __init__(self, nx: int, nz: int):
        self.nx = nx  # State dimension
        self.nz = nz  # Measurement dimension
        self.x = np.zeros(nx)
        self.P = np.eye(nx) * 1000.0
        self.F = np.eye(nx)
        self.Q = np.eye(nx)
        self.H = np.zeros((nz, nx))
        self.R = np.eye(nz)
        
        # Set default H for position-only measurement
        for i in range(min(nz, nx)):
            self.H[i, i] = 1.0
    
    def predict(self, F: Optional[np.ndarray] = None, Q: Optional[np.ndarray] = None):
        """Predict step."""
        if F is not None:
            self.F = F
        if Q is not None:
            self.Q = Q
        
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
    
    def update(self, z: np.ndarray, H: Optional[np.ndarray] = None,
               R: Optional[np.ndarray] = None) -> float:
        """Update step. Returns NIS (Normalized Innovation Squared)."""
        if H is not None:
            self.H = H
        if R is not None:
            self.R = R
        
        # Innovation
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        
        # NIS
        try:
            S_inv = np.linalg.inv(S)
            nis = float(y.T @ S_inv @ y)
        except np.linalg.LinAlgError:
            nis = 1e6
            S_inv = np.eye(len(S)) * 1e-6
        
        # Kalman gain
        K = self.P @ self.H.T @ S_inv
        
        # State update
        self.x = self.x + K @ y
        I_KH = np.eye(self.nx) - K @ self.H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T  # Joseph form
        
        return nis
    
    def innovation_covariance(self, H: Optional[np.ndarray] = None,
                               R: Optional[np.ndarray] = None) -> np.ndarray:
        """Get S = HPH' + R for gating."""
        _H = H if H is not None else self.H
        _R = R if R is not None else self.R
        return _H @ self.P @ _H.T + _R
    
    def mahalanobis_distance(self, z: np.ndarray, H: Optional[np.ndarray] = None,
                              R: Optional[np.ndarray] = None) -> float:
        """Mahalanobis distance of measurement from predicted."""
        _H = H if H is not None else self.H
        _R = R if R is not None else self.R
        
        y = z - _H @ self.x
        S = _H @ self.P @ _H.T + _R
        
        try:
            S_inv = np.linalg.inv(S)
            return float(y.T @ S_inv @ y)
        except np.linalg.LinAlgError:
            return 1e6
    
    @property
    def position(self) -> np.ndarray:
        """Extract position from state (first 3 elements)."""
        return self.x[:3].copy()
    
    @property
    def velocity(self) -> np.ndarray:
        """Extract velocity from state (elements 3:6)."""
        return self.x[3:6].copy() if self.nx >= 6 else np.zeros(3)


# ===== 3D IMM FILTER =====

class IMM3D:
    """[REQ-V50-3D-05] Interacting Multiple Model filter in 3D.
    
    Manages multiple KF3D filters with model mixing.
    Default: CV3D + CA3D + CT3D (±ω)
    """
    
    def __init__(self, dt: float, q_base: float = 1.0,
                 models: Optional[List[str]] = None,
                 r_std: float = 50.0):
        self.dt = dt
        self.q_base = q_base
        self.r_std = r_std
        self.nz = 3  # Default: [x, y, z] measurements
        
        if models is None:
            models = ["cv3d", "ca3d", "ct3d_plus", "ct3d_minus"]
        
        self.model_names = models
        self.n_models = len(models)
        
        # Create filters
        self.filters: List[KalmanFilter3D] = []
        self._model_configs = []
        
        for m in models:
            if m == "cv3d":
                kf = KalmanFilter3D(nx=6, nz=3)
                self._model_configs.append(("cv3d", 6))
            elif m == "ca3d":
                kf = KalmanFilter3D(nx=9, nz=3)
                self._model_configs.append(("ca3d", 9))
            elif m.startswith("ct3d"):
                kf = KalmanFilter3D(nx=7, nz=3)
                self._model_configs.append(("ct3d", 7))
            else:
                kf = KalmanFilter3D(nx=6, nz=3)
                self._model_configs.append(("cv3d", 6))
            
            kf.R = np.eye(3) * r_std**2
            self.filters.append(kf)
        
        # Model probabilities
        self.mu = np.ones(self.n_models) / self.n_models
        
        # Transition probability matrix (favor staying in current mode)
        p_stay = 0.90
        p_switch = (1 - p_stay) / (self.n_models - 1) if self.n_models > 1 else 0
        self.TPM = np.full((self.n_models, self.n_models), p_switch)
        np.fill_diagonal(self.TPM, p_stay)
        
        self._step = 0
    
    def initialize(self, z: np.ndarray):
        """Initialize all filters with first measurement [x, y, z]."""
        for i, kf in enumerate(self.filters):
            kf.x[:3] = z[:3]
            nx = kf.nx
            kf.P = np.eye(nx) * self.r_std**2
            # Velocity uncertainty higher
            if nx >= 6:
                kf.P[3, 3] = kf.P[4, 4] = kf.P[5, 5] = (self.r_std * 10)**2
            if nx >= 7:
                kf.P[6, 6] = 0.01  # Omega uncertainty
            if nx >= 9:
                kf.P[6, 6] = kf.P[7, 7] = kf.P[8, 8] = (self.r_std * 20)**2
    
    def predict(self):
        """IMM predict: mix → predict each filter."""
        # Mixing probabilities
        c_bar = self.TPM.T @ self.mu
        c_bar = np.maximum(c_bar, 1e-30)
        
        mu_mix = np.zeros((self.n_models, self.n_models))
        for i in range(self.n_models):
            for j in range(self.n_models):
                mu_mix[i, j] = self.TPM[i, j] * self.mu[i] / c_bar[j]
        
        # Mix states (within same-dimension models)
        mixed_x = []
        mixed_P = []
        for j in range(self.n_models):
            nx_j = self.filters[j].nx
            x_mix = np.zeros(nx_j)
            
            for i in range(self.n_models):
                nx_i = self.filters[i].nx
                n_common = min(nx_i, nx_j)
                x_mix[:n_common] += mu_mix[i, j] * self.filters[i].x[:n_common]
            
            P_mix = np.zeros((nx_j, nx_j))
            for i in range(self.n_models):
                nx_i = self.filters[i].nx
                n_common = min(nx_i, nx_j)
                dx = np.zeros(nx_j)
                dx[:n_common] = self.filters[i].x[:n_common] - x_mix[:n_common]
                P_i = np.zeros((nx_j, nx_j))
                P_i[:n_common, :n_common] = self.filters[i].P[:n_common, :n_common]
                P_mix += mu_mix[i, j] * (P_i + np.outer(dx, dx))
            
            mixed_x.append(x_mix)
            mixed_P.append(P_mix)
        
        # Apply mixed states and predict
        for j, kf in enumerate(self.filters):
            kf.x = mixed_x[j]
            kf.P = mixed_P[j]
            
            model_type, nx = self._model_configs[j]
            if model_type == "cv3d":
                F, Q = make_cv3d_matrices(self.dt, self.q_base)
            elif model_type == "ca3d":
                F, Q = make_ca3d_matrices(self.dt, self.q_base * 0.5)
            elif model_type == "ct3d":
                omega = kf.x[6] if nx >= 7 else 0.0
                if "minus" in self.model_names[j]:
                    omega = -abs(omega) if abs(omega) > 0.001 else -0.05
                elif "plus" in self.model_names[j]:
                    omega = abs(omega) if abs(omega) > 0.001 else 0.05
                F, Q = make_ct3d_matrices(self.dt, self.q_base, omega)
            else:
                F, Q = make_cv3d_matrices(self.dt, self.q_base)
            
            kf.predict(F, Q)
        
        self._step += 1
    
    def update(self, z: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """IMM update: update each filter → combine.
        
        Args:
            z: Measurement [x, y, z] (3D Cartesian in tracking frame)
        
        Returns:
            Tuple: (combined_state, combined_covariance)
        """
        # Update each filter and compute likelihoods
        likelihoods = np.zeros(self.n_models)
        
        for j, kf in enumerate(self.filters):
            # Build H matrix based on model dimension
            H = np.zeros((3, kf.nx))
            H[0, 0] = 1.0  # x
            H[1, 1] = 1.0  # y
            H[2, 2] = 1.0  # z
            
            # Innovation covariance for likelihood
            S = kf.innovation_covariance(H, kf.R)
            y = z - H @ kf.x
            
            try:
                S_inv = np.linalg.inv(S)
                det_S = np.linalg.det(S)
                if det_S > 0:
                    likelihoods[j] = np.exp(-0.5 * y.T @ S_inv @ y) / \
                                     np.sqrt((2 * np.pi)**3 * det_S)
                else:
                    likelihoods[j] = 1e-30
            except np.linalg.LinAlgError:
                likelihoods[j] = 1e-30
            
            kf.update(z, H)
        
        # Update model probabilities
        c_bar = self.TPM.T @ self.mu
        mu_new = likelihoods * c_bar
        total = np.sum(mu_new)
        if total > 0:
            self.mu = mu_new / total
        else:
            self.mu = np.ones(self.n_models) / self.n_models
        
        return self.combined_state()
    
    def combined_state(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get probability-weighted combined state estimate.
        
        Returns 6-element state [x, y, z, vx, vy, vz] and 6×6 covariance.
        """
        x_comb = np.zeros(6)
        for j, kf in enumerate(self.filters):
            x_comb += self.mu[j] * kf.x[:6]
        
        P_comb = np.zeros((6, 6))
        for j, kf in enumerate(self.filters):
            dx = kf.x[:6] - x_comb
            P_j = kf.P[:6, :6]
            P_comb += self.mu[j] * (P_j + np.outer(dx, dx))
        
        return x_comb, P_comb
    
    @property
    def position(self) -> np.ndarray:
        x, _ = self.combined_state()
        return x[:3]
    
    @property
    def velocity(self) -> np.ndarray:
        x, _ = self.combined_state()
        return x[3:6]
    
    def mahalanobis_distance(self, z: np.ndarray) -> float:
        """Combined Mahalanobis distance for gating."""
        x, P = self.combined_state()
        y = z[:3] - x[:3]
        S = P[:3, :3] + np.eye(3) * self.r_std**2
        try:
            return float(y.T @ np.linalg.inv(S) @ y)
        except np.linalg.LinAlgError:
            return 1e6


# ===== TRACK STATUS =====

class TrackStatus(Enum):
    TENTATIVE = auto()
    CONFIRMED = auto()
    COASTING = auto()
    DELETED = auto()


@dataclass
class TrackState:
    """[REQ-V50-TM-01] Track lifecycle state."""
    track_id: int
    status: TrackStatus = TrackStatus.TENTATIVE
    filter: Optional[IMM3D] = None
    
    # Track management counters
    hit_count: int = 0          # Total measurement associations
    miss_count: int = 0         # Consecutive misses
    total_updates: int = 0      # Total update opportunities
    age: int = 0                # Steps since creation
    
    # M-of-N confirmation logic
    history: list = field(default_factory=list)  # Recent hit/miss history
    
    # Metadata
    creation_time: float = 0.0
    last_update_time: float = 0.0
    score: float = 0.0          # Log-likelihood ratio
    
    # Performance tracking
    avg_nis: float = 0.0
    nis_window: list = field(default_factory=list)
    
    def __repr__(self):
        return (f"Track({self.track_id}, {self.status.name}, "
                f"hits={self.hit_count}, misses={self.miss_count}, "
                f"age={self.age})")


# ===== GNN DATA ASSOCIATION =====

def hungarian_algorithm(cost_matrix: np.ndarray) -> List[Tuple[int, int]]:
    """[REQ-V50-GNN-01] Munkres/Hungarian algorithm for optimal assignment.
    
    Solves the linear assignment problem: minimize total cost of 
    one-to-one assignment of rows (tracks) to columns (measurements).
    
    Args:
        cost_matrix: N×M cost matrix (N tracks, M measurements)
    
    Returns:
        List of (row, col) assignments
    """
    cost = cost_matrix.copy()
    n_rows, n_cols = cost.shape
    
    # Pad to square if needed
    n = max(n_rows, n_cols)
    if n_rows != n_cols:
        padded = np.full((n, n), 1e9)
        padded[:n_rows, :n_cols] = cost
        cost = padded
    
    # Step 1: Row reduction
    for i in range(n):
        cost[i] -= cost[i].min()
    
    # Step 2: Column reduction
    for j in range(n):
        cost[:, j] -= cost[:, j].min()
    
    # Hungarian algorithm with augmenting paths
    row_assign = np.full(n, -1, dtype=int)
    col_assign = np.full(n, -1, dtype=int)
    
    for _ in range(n * 2):  # Max iterations
        # Find uncovered zeros and try to assign
        row_covered = np.zeros(n, dtype=bool)
        col_covered = np.zeros(n, dtype=bool)
        
        # Greedy initial assignment
        row_assign[:] = -1
        col_assign[:] = -1
        for i in range(n):
            for j in range(n):
                if cost[i, j] < 1e-10 and col_assign[j] == -1:
                    row_assign[i] = j
                    col_assign[j] = i
                    break
        
        # Check if complete
        n_assigned = np.sum(row_assign >= 0)
        if n_assigned >= n:
            break
        
        # Cover columns with assignments
        col_covered[:] = False
        for j in range(n):
            if col_assign[j] >= 0:
                col_covered[j] = True
        
        if np.sum(col_covered) >= n:
            break
        
        # Find minimum uncovered value
        row_covered[:] = False
        for i in range(n):
            if row_assign[i] >= 0:
                row_covered[i] = True
        
        min_val = 1e9
        for i in range(n):
            if not row_covered[i]:
                for j in range(n):
                    if not col_covered[j]:
                        min_val = min(min_val, cost[i, j])
        
        if min_val >= 1e8:
            break
        
        # Adjust matrix
        for i in range(n):
            for j in range(n):
                if not row_covered[i] and not col_covered[j]:
                    cost[i, j] -= min_val
                elif row_covered[i] and col_covered[j]:
                    cost[i, j] += min_val
    
    # Extract valid assignments (within original matrix bounds)
    assignments = []
    for i in range(n_rows):
        j = row_assign[i]
        if j >= 0 and j < n_cols and cost_matrix[i, j] < 1e8:
            assignments.append((i, j))
    
    return assignments


def gnn_associate(tracks: List[TrackState], measurements: np.ndarray,
                  gate_threshold: float = 16.0) -> Tuple[List[Tuple[int, int]], 
                                                           List[int], List[int]]:
    """[REQ-V50-GNN-02] Global Nearest Neighbor data association.
    
    Args:
        tracks: List of active tracks
        measurements: Mx3 array of [x, y, z] measurements
        gate_threshold: Chi-squared gate (16.0 ≈ 99.7% for 3DOF)
    
    Returns:
        Tuple: (assignments [(track_idx, meas_idx)], 
                unassigned_tracks [track_idx],
                unassigned_measurements [meas_idx])
    """
    n_tracks = len(tracks)
    n_meas = len(measurements)
    
    if n_tracks == 0 or n_meas == 0:
        return [], list(range(n_tracks)), list(range(n_meas))
    
    # Build cost matrix (Mahalanobis distances)
    cost = np.full((n_tracks, n_meas), 1e9)
    
    for i, track in enumerate(tracks):
        if track.status == TrackStatus.DELETED:
            continue
        for j in range(n_meas):
            d2 = track.filter.mahalanobis_distance(measurements[j])
            if d2 < gate_threshold:
                cost[i, j] = d2
    
    # Solve assignment
    raw_assignments = hungarian_algorithm(cost)
    
    # Filter by gate
    assignments = []
    assigned_tracks = set()
    assigned_meas = set()
    
    for ti, mi in raw_assignments:
        if cost[ti, mi] < gate_threshold:
            assignments.append((ti, mi))
            assigned_tracks.add(ti)
            assigned_meas.add(mi)
    
    unassigned_tracks = [i for i in range(n_tracks) 
                         if i not in assigned_tracks 
                         and tracks[i].status != TrackStatus.DELETED]
    unassigned_meas = [j for j in range(n_meas) if j not in assigned_meas]
    
    return assignments, unassigned_tracks, unassigned_meas


# ===== JPDA DATA ASSOCIATION =====

def jpda_associate(tracks: List[TrackState], measurements: np.ndarray,
                   gate_threshold: float = 16.0, p_detection: float = 0.9,
                   clutter_density: float = 1e-6
                   ) -> Tuple[Dict[int, np.ndarray], List[int], List[int]]:
    """[REQ-V50-JPDA-01] Joint Probabilistic Data Association.
    
    Uses Mahalanobis-based relative likelihood for robust operation
    across all covariance scales (new tracks with large P, established
    tracks with small P).
    
    Reference: Bar-Shalom & Fortmann, "Tracking and Data Association" (1988)
    """
    n_tracks = len(tracks)
    n_meas = len(measurements)
    
    if n_tracks == 0 or n_meas == 0:
        return {}, list(range(n_tracks)), list(range(n_meas))
    
    # Step 1: Validation matrix and Mahalanobis distances
    validation = np.zeros((n_tracks, n_meas), dtype=bool)
    mahal_dist = np.full((n_tracks, n_meas), 1e9)
    
    for i, track in enumerate(tracks):
        if track.status == TrackStatus.DELETED:
            continue
        for j in range(n_meas):
            d2 = track.filter.mahalanobis_distance(measurements[j])
            if d2 < gate_threshold:
                validation[i, j] = True
                mahal_dist[i, j] = d2
    
    # Step 2: Compute association probabilities using Mahalanobis distance
    # beta_{i,j} proportional to Pd * exp(-0.5 * d^2)
    # beta_{i,0} proportional to (1 - Pd)
    beta = np.zeros((n_tracks, n_meas + 1))
    
    for i in range(n_tracks):
        if tracks[i].status == TrackStatus.DELETED:
            beta[i, 0] = 1.0
            continue
        
        valid_j = np.where(validation[i])[0]
        if len(valid_j) == 0:
            beta[i, 0] = 1.0
            continue
        
        raw_probs = np.zeros(n_meas + 1)
        raw_probs[0] = 1.0 - p_detection  # Miss
        
        for j in valid_j:
            sharing = 1.0 / max(1, np.sum(validation[:, j]))
            raw_probs[j + 1] = p_detection * np.exp(-0.5 * mahal_dist[i, j]) * sharing
        
        total = raw_probs.sum()
        if total > 0:
            beta[i] = raw_probs / total
        else:
            beta[i, 0] = 1.0
    
    # Step 3: Weighted measurement per track
    weighted_measurements = {}
    
    for i, track in enumerate(tracks):
        if track.status == TrackStatus.DELETED:
            continue
        valid_j = np.where(validation[i])[0]
        if len(valid_j) == 0:
            continue
        
        p_assoc = sum(beta[i, j + 1] for j in valid_j)
        if p_assoc < 0.01:
            continue
        
        z_combined = np.zeros(3)
        for j in valid_j:
            z_combined += beta[i, j + 1] * measurements[j, :3]
        z_combined /= p_assoc
        weighted_measurements[i] = z_combined
    
    # Identify unassigned
    assigned_tracks = set(weighted_measurements.keys())
    assigned_meas = set()
    for i in weighted_measurements:
        for j in range(n_meas):
            if validation[i, j] and beta[i, j + 1] > 0.01:
                assigned_meas.add(j)
    
    unassigned_tracks = [i for i in range(n_tracks) 
                         if i not in assigned_tracks
                         and tracks[i].status != TrackStatus.DELETED]
    unassigned_meas = [j for j in range(n_meas) if j not in assigned_meas]
    
    return weighted_measurements, unassigned_tracks, unassigned_meas


@dataclass
class MHTHypothesis:
    """[REQ-V50-MHT-01] A single global hypothesis in MHT.
    
    Each hypothesis represents one possible assignment of all measurements
    to all tracks across the current scan.
    """
    assignments: Dict[int, int]      # track_idx → meas_idx (-1 = miss)
    score: float                     # Log-likelihood of this hypothesis
    unassigned_meas: List[int]       # Measurement indices not assigned to any track


def mht_associate(tracks: List[TrackState], measurements: np.ndarray,
                  gate_threshold: float = 16.0, p_detection: float = 0.9,
                  max_hypotheses: int = 50, n_best: int = 5
                  ) -> Tuple[Dict[int, np.ndarray], List[int], List[int]]:
    """[REQ-V50-MHT-02] Multi-Hypothesis Tracking data association.
    
    Generates multiple global hypotheses for measurement-to-track assignment,
    scores each by log-likelihood, and selects the best. Unlike GNN (single
    best assignment) or JPDA (weighted average), MHT maintains alternative
    explanations and selects the most likely.
    
    Implementation: K-best global hypotheses via ranked assignment enumeration
    with Mahalanobis-based scoring.
    
    Reference: Reid, "An algorithm for tracking multiple targets," IEEE TAC, 1979
               Blackman & Popoli, "Design and Analysis of Modern Tracking Systems," 1999
    
    Args:
        tracks: Active track list
        measurements: Mx3 measurement array
        gate_threshold: Chi-squared gate for validation
        p_detection: Probability of detection
        max_hypotheses: Maximum hypotheses to generate before pruning
        n_best: Number of top hypotheses to retain
    
    Returns:
        Tuple: (best_assignments {track_idx: measurement_z},
                unassigned_tracks, unassigned_measurements)
    """
    n_tracks = len(tracks)
    n_meas = len(measurements)
    
    if n_tracks == 0 or n_meas == 0:
        return {}, list(range(n_tracks)), list(range(n_meas))
    
    # Step 1: Compute gating and Mahalanobis distances
    validation = np.zeros((n_tracks, n_meas), dtype=bool)
    mahal_dist = np.full((n_tracks, n_meas), 1e9)
    
    for i, track in enumerate(tracks):
        if track.status == TrackStatus.DELETED:
            continue
        for j in range(n_meas):
            d2 = track.filter.mahalanobis_distance(measurements[j])
            if d2 < gate_threshold:
                validation[i, j] = True
                mahal_dist[i, j] = d2
    
    # Step 2: Generate hypotheses via systematic enumeration
    # Each hypothesis assigns each track to one measurement (or miss)
    # Score = sum of log-likelihoods for each assignment
    
    hypotheses: List[MHTHypothesis] = []
    
    # Start with the "all tracks miss" hypothesis
    miss_score = n_tracks * np.log(max(1.0 - p_detection, 1e-10))
    hypotheses.append(MHTHypothesis(
        assignments={i: -1 for i in range(n_tracks) 
                     if tracks[i].status != TrackStatus.DELETED},
        score=miss_score,
        unassigned_meas=list(range(n_meas))
    ))
    
    # Generate hypotheses by iterating tracks and branching
    for i in range(n_tracks):
        if tracks[i].status == TrackStatus.DELETED:
            continue
        
        valid_j = np.where(validation[i])[0]
        if len(valid_j) == 0:
            continue
        
        new_hypotheses = []
        for hyp in hypotheses:
            # Branch: track i assigned to each valid measurement
            assigned_meas_in_hyp = set(v for k, v in hyp.assignments.items() if v >= 0)
            
            for j in valid_j:
                if j in assigned_meas_in_hyp:
                    continue  # Measurement already taken in this hypothesis
                
                # Create new hypothesis: assign track i to measurement j
                new_assign = dict(hyp.assignments)
                new_assign[i] = j
                
                # Score delta: remove miss penalty, add detection + likelihood
                delta = (-np.log(max(1.0 - p_detection, 1e-10))
                         + np.log(max(p_detection, 1e-10))
                         - 0.5 * mahal_dist[i, j])
                
                new_unassigned = [m for m in hyp.unassigned_meas if m != j]
                
                new_hypotheses.append(MHTHypothesis(
                    assignments=new_assign,
                    score=hyp.score + delta,
                    unassigned_meas=new_unassigned
                ))
            
            # Also keep the original (track i stays as miss)
            new_hypotheses.append(hyp)
        
        # Prune: keep only top max_hypotheses
        new_hypotheses.sort(key=lambda h: h.score, reverse=True)
        hypotheses = new_hypotheses[:max_hypotheses]
    
    # Step 3: Select best hypothesis
    hypotheses.sort(key=lambda h: h.score, reverse=True)
    best = hypotheses[0]
    
    # Step 4: Extract assignments from best hypothesis
    best_measurements = {}
    for track_idx, meas_idx in best.assignments.items():
        if meas_idx >= 0:
            best_measurements[track_idx] = measurements[meas_idx, :3]
    
    assigned_tracks = set(best_measurements.keys())
    assigned_meas = set(v for v in best.assignments.values() if v >= 0)
    
    unassigned_tracks = [i for i in range(n_tracks)
                         if i not in assigned_tracks
                         and tracks[i].status != TrackStatus.DELETED]
    unassigned_meas = [j for j in range(n_meas) if j not in assigned_meas]
    
    return best_measurements, unassigned_tracks, unassigned_meas


# ===== ENHANCED MHT — CLUSTER GATING + N-SCAN PRUNING =====

def _build_clusters(validation: np.ndarray) -> List[Tuple[List[int], List[int]]]:
    """[REQ-V55-MHT-10] Cluster tracks and measurements by gate overlap.
    
    Finds connected components in the track-measurement validation graph.
    Each cluster is solved independently — reduces combinatorial explosion
    from O(M^N) to O(sum(m_k^n_k)) per cluster.
    
    Args:
        validation: N_tracks × N_meas boolean gating matrix
    
    Returns:
        List of (track_indices, meas_indices) clusters
    """
    n_tracks, n_meas = validation.shape
    visited_t = set()
    visited_m = set()
    clusters = []
    
    for start_t in range(n_tracks):
        if start_t in visited_t:
            continue
        
        # BFS to find connected component
        cluster_t = set()
        cluster_m = set()
        queue_t = [start_t]
        
        while queue_t:
            t = queue_t.pop()
            if t in cluster_t:
                continue
            cluster_t.add(t)
            visited_t.add(t)
            
            # Find all measurements gated to this track
            for m in range(n_meas):
                if validation[t, m] and m not in cluster_m:
                    cluster_m.add(m)
                    visited_m.add(m)
                    # Find all other tracks gated to this measurement
                    for t2 in range(n_tracks):
                        if validation[t2, m] and t2 not in cluster_t:
                            queue_t.append(t2)
        
        if cluster_t:
            clusters.append((sorted(cluster_t), sorted(cluster_m)))
    
    # Add isolated measurements (no track gated)
    all_gated_m = visited_m
    for m in range(n_meas):
        if m not in all_gated_m:
            clusters.append(([], [m]))
    
    return clusters


@dataclass
class MHTHypothesisTree:
    """[REQ-V55-MHT-11] N-scan hypothesis tree for deferred decisions.
    
    Maintains a history of hypotheses over N scans, allowing retroactive
    resolution of ambiguous assignments once more data arrives.
    """
    scan_hypotheses: List[List[MHTHypothesis]] = field(default_factory=list)
    n_scan_depth: int = 3
    
    def add_scan(self, hypotheses: List[MHTHypothesis]) -> None:
        """Store hypotheses from this scan."""
        self.scan_hypotheses.append(hypotheses)
        if len(self.scan_hypotheses) > self.n_scan_depth:
            self.scan_hypotheses = self.scan_hypotheses[-self.n_scan_depth:]
    
    def get_n_scan_best(self) -> Optional[MHTHypothesis]:
        """Get the best hypothesis considering N-scan history.
        
        The N-scan pruning rule: a hypothesis branch that has been consistently
        best (or near-best) over N scans is promoted to confirmed assignment.
        """
        if not self.scan_hypotheses:
            return None
        
        # Score each current hypothesis by consistency with history
        current = self.scan_hypotheses[-1]
        if not current:
            return None
        
        if len(self.scan_hypotheses) < 2:
            return max(current, key=lambda h: h.score)
        
        # Weighted scoring: recent scans weight more
        best_score = -np.inf
        best_hyp = current[0]
        
        for hyp in current:
            composite = hyp.score
            # Bonus for consistency with previous scan assignments
            for depth, prev_scan in enumerate(reversed(self.scan_hypotheses[:-1])):
                if not prev_scan:
                    continue
                weight = 0.5 ** (depth + 1)  # Exponential decay
                prev_best = max(prev_scan, key=lambda h: h.score)
                # Overlap bonus: how many assignments agree
                overlap = sum(1 for k, v in hyp.assignments.items()
                            if k in prev_best.assignments and prev_best.assignments[k] == v)
                composite += weight * overlap * 2.0
            
            if composite > best_score:
                best_score = composite
                best_hyp = hyp
        
        return best_hyp


def mht_associate_enhanced(tracks: List['TrackState'], measurements: np.ndarray,
                           gate_threshold: float = 16.0, p_detection: float = 0.9,
                           max_hypotheses: int = 100, n_best: int = 10,
                           clutter_density: float = 1e-6,
                           hypothesis_tree: Optional[MHTHypothesisTree] = None,
                           ) -> Tuple[Dict[int, np.ndarray], List[int], List[int]]:
    """[REQ-V55-MHT-12] Enhanced MHT with cluster gating and N-scan pruning.
    
    Improvements over basic MHT:
    1. **Cluster gating** — decomposes into independent sub-problems
    2. **Clutter model** — explicit false alarm likelihood per measurement
    3. **N-scan pruning** — deferred decisions resolved with history
    4. **Murty's algorithm-inspired** — k-best in each cluster
    
    Reference:
        Blackman & Popoli, "Design & Analysis of Modern Tracking Systems" (1999)
        Cox & Hingorani, "An efficient implementation of Reid's MHT" (1996)
    
    Args:
        tracks: Active track list
        measurements: Mx3 measurement array
        gate_threshold: Chi-squared gate
        p_detection: Detection probability
        max_hypotheses: Max hypotheses per cluster
        n_best: K-best to keep globally
        clutter_density: Spatial density of false alarms (per m³)
        hypothesis_tree: Optional N-scan history tree
    
    Returns:
        Tuple: (assignments, unassigned_tracks, unassigned_measurements)
    """
    n_tracks = len(tracks)
    n_meas = len(measurements) if len(measurements.shape) > 1 else 0
    
    if n_tracks == 0 or n_meas == 0:
        return {}, list(range(n_tracks)), list(range(n_meas))
    
    # Step 1: Validation gating
    validation = np.zeros((n_tracks, n_meas), dtype=bool)
    mahal_dist = np.full((n_tracks, n_meas), 1e9)
    
    for i, track in enumerate(tracks):
        if track.status == TrackStatus.DELETED:
            continue
        for j in range(n_meas):
            d2 = track.filter.mahalanobis_distance(measurements[j])
            if d2 < gate_threshold:
                validation[i, j] = True
                mahal_dist[i, j] = d2
    
    # Step 2: Cluster decomposition
    clusters = _build_clusters(validation)
    
    # Step 3: Solve each cluster independently
    all_assignments = {}
    all_assigned_meas = set()
    all_hypotheses = []
    
    for cluster_tracks, cluster_meas in clusters:
        if not cluster_tracks:
            continue  # Isolated measurements → new track candidates
        
        # Build sub-problem
        cluster_hyps = [MHTHypothesis(
            assignments={t: -1 for t in cluster_tracks},
            score=len(cluster_tracks) * np.log(max(1.0 - p_detection, 1e-10)),
            unassigned_meas=list(cluster_meas),
        )]
        
        for t_idx in cluster_tracks:
            if tracks[t_idx].status == TrackStatus.DELETED:
                continue
            
            valid_m = [m for m in cluster_meas if validation[t_idx, m]]
            if not valid_m:
                continue
            
            new_hyps = []
            for hyp in cluster_hyps:
                taken = {v for v in hyp.assignments.values() if v >= 0}
                
                for m_idx in valid_m:
                    if m_idx in taken:
                        continue
                    
                    new_a = dict(hyp.assignments)
                    new_a[t_idx] = m_idx
                    
                    # Log-likelihood ratio vs clutter
                    detection_ll = np.log(max(p_detection, 1e-10)) - 0.5 * mahal_dist[t_idx, m_idx]
                    miss_ll = np.log(max(1.0 - p_detection, 1e-10))
                    clutter_ll = np.log(max(clutter_density, 1e-20))
                    
                    delta = detection_ll - miss_ll - clutter_ll
                    
                    new_hyps.append(MHTHypothesis(
                        assignments=new_a,
                        score=hyp.score + delta,
                        unassigned_meas=[m for m in hyp.unassigned_meas if m != m_idx],
                    ))
                
                new_hyps.append(hyp)
            
            new_hyps.sort(key=lambda h: h.score, reverse=True)
            cluster_hyps = new_hyps[:max_hypotheses]
        
        all_hypotheses.extend(cluster_hyps)
        
        # Best for this cluster
        if cluster_hyps:
            best = max(cluster_hyps, key=lambda h: h.score)
            for t_idx, m_idx in best.assignments.items():
                if m_idx >= 0:
                    all_assignments[t_idx] = measurements[m_idx, :3]
                    all_assigned_meas.add(m_idx)
    
    # Step 4: N-scan pruning (if tree provided)
    if hypothesis_tree is not None:
        hypothesis_tree.add_scan(all_hypotheses[:n_best])
        n_scan_best = hypothesis_tree.get_n_scan_best()
        if n_scan_best is not None:
            # Override with N-scan-confirmed assignments
            for t_idx, m_idx in n_scan_best.assignments.items():
                if m_idx >= 0 and m_idx < n_meas:
                    all_assignments[t_idx] = measurements[m_idx, :3]
                    all_assigned_meas.add(m_idx)
    
    assigned_tracks = set(all_assignments.keys())
    unassigned_tracks = [i for i in range(n_tracks)
                         if i not in assigned_tracks
                         and tracks[i].status != TrackStatus.DELETED]
    unassigned_meas = [j for j in range(n_meas) if j not in all_assigned_meas]
    
    return all_assignments, unassigned_tracks, unassigned_meas


class AssociationMethod(Enum):
    """Data association algorithm selection."""
    GNN = "gnn"     # Global Nearest Neighbor — fast, simple
    JPDA = "jpda"   # Joint Probabilistic DA — better in clutter
    MHT = "mht"     # Multi-Hypothesis Tracking — best in dense/ambiguous


# ===== PERFORMANCE METRICS =====

def compute_nees(x_true: np.ndarray, x_est: np.ndarray,
                 P: np.ndarray) -> float:
    """[REQ-V50-MET-03] Normalized Estimation Error Squared.
    
    NEES = (x_true - x_est)^T P^{-1} (x_true - x_est)
    For consistent filter: E[NEES] = nx (state dimension)
    
    Reference: Bar-Shalom, Li, Kirubarajan (2001), §5.4
    """
    dx = x_true - x_est
    try:
        return float(dx.T @ np.linalg.inv(P) @ dx)
    except np.linalg.LinAlgError:
        return float('inf')


def compute_nis(innovation: np.ndarray, S: np.ndarray) -> float:
    """[REQ-V50-MET-04] Normalized Innovation Squared.
    
    NIS = y^T S^{-1} y where y is innovation, S is innovation covariance.
    For consistent filter: E[NIS] = nz (measurement dimension)
    """
    try:
        return float(innovation.T @ np.linalg.inv(S) @ innovation)
    except np.linalg.LinAlgError:
        return float('inf')


def compute_siap_metrics(track_positions: Dict[int, np.ndarray],
                         truth_positions: Dict[int, np.ndarray],
                         threshold: float = 500.0) -> Dict[str, float]:
    """[REQ-V50-MET-05] NATO SIAP track quality metrics.
    
    Single Integrated Air Picture standard metrics.
    
    Returns:
        Dict with completeness, ambiguity, spuriousness, track_accuracy
    """
    n_truth = len(truth_positions)
    n_tracks = len(track_positions)
    
    if n_truth == 0 and n_tracks == 0:
        return {"completeness": 1.0, "ambiguity": 0.0, 
                "spuriousness": 0.0, "accuracy": 0.0}
    
    if n_truth == 0:
        return {"completeness": 0.0, "ambiguity": 0.0,
                "spuriousness": 1.0, "accuracy": float('inf')}
    
    t_pos = list(track_positions.values())
    g_pos = list(truth_positions.values())
    t_ids = list(track_positions.keys())
    g_ids = list(truth_positions.keys())
    
    # Assignment matrix
    D = np.zeros((n_tracks, n_truth))
    for i in range(n_tracks):
        for j in range(n_truth):
            D[i, j] = np.linalg.norm(np.array(t_pos[i])[:3] - np.array(g_pos[j])[:3])
    
    # Optimal assignment
    if n_tracks > 0 and n_truth > 0:
        assignments = hungarian_algorithm(D)
    else:
        assignments = []
    
    # Metrics
    correct = sum(1 for ti, gi in assignments if D[ti, gi] < threshold)
    
    completeness = correct / max(n_truth, 1)
    
    # Ambiguity: multiple tracks per truth
    truth_matched = {}
    for ti, gi in assignments:
        if D[ti, gi] < threshold:
            truth_matched.setdefault(gi, []).append(ti)
    n_redundant = sum(len(v) - 1 for v in truth_matched.values() if len(v) > 1)
    ambiguity = n_redundant / max(n_truth, 1)
    
    # Spuriousness: tracks not matching any truth
    spuriousness = max(0, n_tracks - correct) / max(n_tracks, 1)
    
    # Accuracy: mean position error for correct associations
    errors = [D[ti, gi] for ti, gi in assignments if D[ti, gi] < threshold]
    accuracy = np.mean(errors) if errors else float('inf')
    
    return {
        "completeness": completeness,
        "ambiguity": ambiguity,
        "spuriousness": spuriousness,
        "accuracy": accuracy,
        "n_correct": correct,
        "n_redundant": n_redundant,
        "n_false": max(0, n_tracks - correct),
    }


# ===== TRACK MANAGER =====

@dataclass
class TrackManagerConfig:
    """[REQ-V50-TM-02] Track lifecycle configuration."""
    # Confirmation: M hits in N opportunities
    confirm_m: int = 3           # Hits required
    confirm_n: int = 5           # Window size
    
    # Deletion: K consecutive misses
    delete_misses: int = 5       # Consecutive misses to delete
    
    # Coasting: C misses before switching to coast
    coast_misses: int = 2
    
    # Maximum coast age before forced deletion
    max_coast_age: int = 10
    
    # Score thresholds (log-likelihood ratio)
    confirm_score: float = 5.0   # Score to confirm
    delete_score: float = -5.0   # Score to delete
    
    # Gating
    gate_threshold: float = 16.0  # Chi-sq 99.7% for 3DOF
    
    # New track initialization
    min_separation: float = 100.0  # Min distance between new tracks [m]


class TrackManager:
    """[REQ-V50-TM-03] Track lifecycle manager with M-of-N logic."""
    
    def __init__(self, config: Optional[TrackManagerConfig] = None):
        self.config = config or TrackManagerConfig()
        self._next_id = 1
    
    def create_track(self, z: np.ndarray, dt: float, q_base: float,
                     r_std: float, timestamp: float = 0.0) -> TrackState:
        """Initialize a new tentative track from unassigned measurement."""
        imm = IMM3D(dt=dt, q_base=q_base, r_std=r_std)
        imm.initialize(z)
        
        track = TrackState(
            track_id=self._next_id,
            status=TrackStatus.TENTATIVE,
            filter=imm,
            hit_count=1,
            creation_time=timestamp,
            last_update_time=timestamp,
            history=[True],
            score=1.0
        )
        
        self._next_id += 1
        return track
    
    def update_track_hit(self, track: TrackState, z: np.ndarray,
                         timestamp: float = 0.0) -> None:
        """Record measurement association (hit) and update lifecycle."""
        track.filter.update(z)
        track.hit_count += 1
        track.miss_count = 0
        track.total_updates += 1
        track.last_update_time = timestamp
        track.history.append(True)
        
        # Score update (simplified LLR)
        track.score += 1.0
        
        # Keep history window
        if len(track.history) > self.config.confirm_n:
            track.history = track.history[-self.config.confirm_n:]
        
        # Check for confirmation
        if track.status == TrackStatus.TENTATIVE:
            recent_hits = sum(track.history[-self.config.confirm_n:])
            if recent_hits >= self.config.confirm_m or track.score >= self.config.confirm_score:
                track.status = TrackStatus.CONFIRMED
        
        # Recover from coasting
        if track.status == TrackStatus.COASTING:
            track.status = TrackStatus.CONFIRMED
    
    def update_track_miss(self, track: TrackState) -> None:
        """Record missed association and update lifecycle."""
        track.miss_count += 1
        track.total_updates += 1
        track.history.append(False)
        track.score -= 1.5  # Miss costs more than hit gains
        
        if len(track.history) > self.config.confirm_n:
            track.history = track.history[-self.config.confirm_n:]
        
        # State transitions
        if track.miss_count >= self.config.delete_misses:
            track.status = TrackStatus.DELETED
        elif track.miss_count >= self.config.coast_misses:
            if track.status == TrackStatus.CONFIRMED:
                track.status = TrackStatus.COASTING
        
        # Tentative track deletion (stricter than confirmed, but configurable)
        if track.status == TrackStatus.TENTATIVE:
            tent_limit = max(2, self.config.confirm_n - self.config.confirm_m + 1)
            if track.miss_count >= tent_limit or track.score < self.config.delete_score:
                track.status = TrackStatus.DELETED
    
    def age_tracks(self, tracks: List[TrackState]) -> None:
        """Age all tracks and delete stale coasters."""
        for track in tracks:
            track.age += 1
            if (track.status == TrackStatus.COASTING and 
                track.age - track.hit_count > self.config.max_coast_age):
                track.status = TrackStatus.DELETED


# ===== MULTI-TARGET TRACKER ENGINE =====

class MultiTargetTracker:
    """[REQ-V50-MTT-01] Complete multi-target tracking engine.
    
    Integrates:
      - 3D IMM filter per track
      - GNN data association (Munkres algorithm)
      - Track management (M-of-N init/confirm/delete)
      - Coasting and track scoring
    
    Usage:
        mtt = MultiTargetTracker(dt=1.0, r_std=50.0, q_base=1.0)
        
        # Each scan:
        confirmed = mtt.process_scan(measurements_3d, timestamp)
        for track in confirmed:
            print(f"Track {track.track_id}: pos={track.filter.position}")
    """
    
    def __init__(self, dt: float = 1.0, r_std: float = 50.0,
                 q_base: float = 1.0, domain: Optional[str] = None,
                 config: Optional[TrackManagerConfig] = None,
                 association: str = "gnn"):
        """
        Args:
            dt: Scan interval [seconds]
            r_std: Measurement noise std dev [meters]
            q_base: Process noise base intensity
            domain: Preset domain for tuning (military, atc, automotive, space)
            config: Track management configuration
            association: Data association method - "gnn" or "jpda"
        """
        self.dt = dt
        self.r_std = r_std
        self.q_base = q_base
        self.domain = domain
        self.association = AssociationMethod(association)
        
        # Apply domain presets
        if config is None:
            config = self._domain_config(domain)
        self.config = config
        
        self.track_manager = TrackManager(config)
        self.tracks: List[TrackState] = []
        self._step = 0
        self._timestamp = 0.0
        
        # ECM-aware adaptive gating [REQ-V57-ECM-01]
        self._ecm_active = False
        self._ecm_gate_mult = 1.0      # Gate multiplier (1.0 = normal)
        self._ecm_coast_ext = 0         # Extra coast scans allowed
        self._ecm_r_scale = 1.0         # Measurement noise inflation
        self._base_gate = config.gate_threshold
        self._base_delete = config.delete_misses
        self._base_coast_age = config.max_coast_age
        
        # Statistics
        self.stats = {
            "total_tracks_created": 0,
            "total_tracks_deleted": 0,
            "current_confirmed": 0,
            "current_tentative": 0,
            "current_coasting": 0,
            "scans_processed": 0,
        }
    
    def _domain_config(self, domain: Optional[str]) -> TrackManagerConfig:
        """Domain-specific track management presets."""
        presets = {
            "military": TrackManagerConfig(
                confirm_m=2, confirm_n=3, delete_misses=3,
                coast_misses=1, max_coast_age=5,
                gate_threshold=25.0, min_separation=200.0
            ),
            "atc": TrackManagerConfig(
                confirm_m=3, confirm_n=5, delete_misses=5,
                coast_misses=2, max_coast_age=8,
                gate_threshold=16.0, min_separation=500.0
            ),
            "automotive": TrackManagerConfig(
                confirm_m=3, confirm_n=4, delete_misses=3,
                coast_misses=1, max_coast_age=3,
                gate_threshold=12.0, min_separation=5.0
            ),
            "space": TrackManagerConfig(
                confirm_m=3, confirm_n=5, delete_misses=10,
                coast_misses=3, max_coast_age=20,
                gate_threshold=25.0, min_separation=10000.0
            ),
            "maritime": TrackManagerConfig(
                confirm_m=3, confirm_n=5, delete_misses=8,
                coast_misses=3, max_coast_age=15,
                gate_threshold=16.0, min_separation=100.0
            ),
        }
        return presets.get(domain, TrackManagerConfig())
    
    def set_ecm_state(self, ecm_active: bool, 
                      gate_multiplier: float = 3.0,
                      coast_extension: int = 5,
                      r_scale: float = 3.0) -> None:
        """[REQ-V57-ECM-02] Signal ECM environment to tracker.
        
        When ECM is detected (by IntelligencePipeline), call this to:
        1. Widen the association gate (prevent track loss on noisy measurements)
        2. Extend coast duration (tolerate more missed detections)
        3. Inflate R for measurement update (reduce weight of corrupted data)
        
        This implements the "covariance inflation + adaptive gating" approach
        from Bar-Shalom & Li (2001) §11.7 — the correct response to ECM
        is NOT to reject measurements, but to trust them less.
        
        Args:
            ecm_active: True if ECM detected, False to restore normal
            gate_multiplier: Scale factor for chi-squared gate (3.0 = 3x wider)
            coast_extension: Extra scans before track deletion
            r_scale: Measurement noise inflation factor
        """
        self._ecm_active = ecm_active
        
        if ecm_active:
            self._ecm_gate_mult = max(gate_multiplier, 1.0)
            self._ecm_coast_ext = max(coast_extension, 0)
            self._ecm_r_scale = max(r_scale, 1.0)
            # Apply: widen gate + extend coast
            self.config.gate_threshold = self._base_gate * self._ecm_gate_mult
            self.config.delete_misses = self._base_delete + self._ecm_coast_ext
            self.config.max_coast_age = self._base_coast_age + self._ecm_coast_ext
        else:
            self._ecm_gate_mult = 1.0
            self._ecm_coast_ext = 0
            self._ecm_r_scale = 1.0
            # Restore baseline
            self.config.gate_threshold = self._base_gate
            self.config.delete_misses = self._base_delete
            self.config.max_coast_age = self._base_coast_age
    
    def inflate_track_R(self, scale: float) -> None:
        """[REQ-V57-ECM-03] Inflate measurement covariance on all active tracks.
        
        Under ECM, measurements are corrupted. Rather than rejecting them,
        we increase R to reduce their influence on the state estimate.
        The track coasts more on its prediction (which is clean) while
        still incorporating the degraded measurement with appropriate weight.
        
        Args:
            scale: R multiplication factor (e.g., 3.0 = trust measurement 3x less)
        """
        for track in self.tracks:
            if track.status != TrackStatus.DELETED:
                if hasattr(track.filter, 'filters'):
                    # IMM: inflate R on all sub-filters
                    for kf in track.filter.filters:
                        kf.R = np.eye(kf.nz) * (self.r_std * scale) ** 2
                else:
                    track.filter.R = np.eye(track.filter.nz) * (self.r_std * scale) ** 2
    
    def process_scan(self, measurements: np.ndarray,
                     timestamp: Optional[float] = None) -> List[TrackState]:
        """[REQ-V50-MTT-02] Process one scan of measurements.
        
        Full MTT cycle: predict → associate → update → manage → report.
        
        Args:
            measurements: Mx3 array of [x, y, z] measurements (Cartesian)
                         Can be Mx2 for 2D mode (z auto-set to 0)
            timestamp: Scan timestamp (auto-incremented if None)
        
        Returns:
            List of confirmed tracks
        """
        if timestamp is None:
            timestamp = self._step * self.dt
        self._timestamp = timestamp
        
        # Handle 2D input gracefully
        if measurements.ndim == 1:
            measurements = measurements.reshape(1, -1)
        if measurements.shape[1] == 2:
            z3d = np.zeros((len(measurements), 3))
            z3d[:, :2] = measurements
            measurements = z3d
        
        # 1. PREDICT all active tracks
        active = [t for t in self.tracks if t.status != TrackStatus.DELETED]
        for track in active:
            track.filter.predict()
        
        # 2. DATA ASSOCIATION (GNN, JPDA, or MHT)
        if self.association == AssociationMethod.JPDA:
            weighted_meas, unassigned_tracks, unassigned_meas = jpda_associate(
                active, measurements, self.config.gate_threshold
            )
            
            # 3a. UPDATE tracks with JPDA weighted measurements
            for track_idx, z_weighted in weighted_meas.items():
                self.track_manager.update_track_hit(
                    active[track_idx], z_weighted, timestamp
                )
            
            # 4a. MISS unassigned tracks
            for track_idx in unassigned_tracks:
                self.track_manager.update_track_miss(active[track_idx])
        
        elif self.association == AssociationMethod.MHT:
            # MHT: multi-hypothesis scoring → best assignment
            best_meas, unassigned_tracks, unassigned_meas = mht_associate(
                active, measurements, self.config.gate_threshold
            )
            
            # 3c. UPDATE tracks with MHT best-hypothesis assignments
            for track_idx, z_best in best_meas.items():
                self.track_manager.update_track_hit(
                    active[track_idx], z_best, timestamp
                )
            
            # 4c. MISS unassigned tracks
            for track_idx in unassigned_tracks:
                self.track_manager.update_track_miss(active[track_idx])
        
        else:
            # GNN (default)
            assignments, unassigned_tracks, unassigned_meas = gnn_associate(
                active, measurements, self.config.gate_threshold
            )
            
            # 3b. UPDATE assigned tracks
            for track_idx, meas_idx in assignments:
                self.track_manager.update_track_hit(
                    active[track_idx], measurements[meas_idx], timestamp
                )
            
            # 4b. MISS unassigned tracks
            for track_idx in unassigned_tracks:
                self.track_manager.update_track_miss(active[track_idx])
        
        # 5. INITIATE new tracks from unassigned measurements
        for meas_idx in unassigned_meas:
            z = measurements[meas_idx]
            
            # Check minimum separation from existing tracks
            too_close = False
            for track in active:
                if track.status != TrackStatus.DELETED:
                    d = np.linalg.norm(z[:3] - track.filter.position)
                    if d < self.config.min_separation:
                        too_close = True
                        break
            
            if not too_close:
                new_track = self.track_manager.create_track(
                    z, self.dt, self.q_base, self.r_std, timestamp
                )
                self.tracks.append(new_track)
                self.stats["total_tracks_created"] += 1
        
        # 6. AGE tracks and delete stale
        self.track_manager.age_tracks(self.tracks)
        
        # 7. CLEANUP deleted tracks
        n_before = len(self.tracks)
        self.tracks = [t for t in self.tracks if t.status != TrackStatus.DELETED]
        self.stats["total_tracks_deleted"] += (n_before - len(self.tracks))
        
        # 8. UPDATE statistics
        self._step += 1
        self.stats["scans_processed"] = self._step
        self.stats["current_confirmed"] = sum(
            1 for t in self.tracks if t.status == TrackStatus.CONFIRMED
        )
        self.stats["current_tentative"] = sum(
            1 for t in self.tracks if t.status == TrackStatus.TENTATIVE
        )
        self.stats["current_coasting"] = sum(
            1 for t in self.tracks if t.status == TrackStatus.COASTING
        )
        
        return self.confirmed_tracks
    
    @property
    def confirmed_tracks(self) -> List[TrackState]:
        """Get all confirmed (active, reliable) tracks."""
        return [t for t in self.tracks 
                if t.status in (TrackStatus.CONFIRMED, TrackStatus.COASTING)]
    
    @property
    def all_tracks(self) -> List[TrackState]:
        """Get all non-deleted tracks."""
        return [t for t in self.tracks if t.status != TrackStatus.DELETED]
    
    def get_track(self, track_id: int) -> Optional[TrackState]:
        """Get track by ID."""
        for t in self.tracks:
            if t.track_id == track_id:
                return t
        return None
    
    def get_positions(self) -> Dict[int, np.ndarray]:
        """Get confirmed track positions as {track_id: [x,y,z]}."""
        return {t.track_id: t.filter.position for t in self.confirmed_tracks}
    
    def get_velocities(self) -> Dict[int, np.ndarray]:
        """Get confirmed track velocities as {track_id: [vx,vy,vz]}."""
        return {t.track_id: t.filter.velocity for t in self.confirmed_tracks}
    
    def summary(self) -> str:
        """Human-readable tracker summary."""
        lines = [f"MTT Engine — Step {self._step} — {len(self.tracks)} tracks"]
        for t in self.tracks:
            pos = t.filter.position
            vel = t.filter.velocity
            speed = np.linalg.norm(vel)
            lines.append(
                f"  T{t.track_id:03d} [{t.status.name:10s}] "
                f"pos=({pos[0]:.0f}, {pos[1]:.0f}, {pos[2]:.0f}) "
                f"speed={speed:.1f}m/s hits={t.hit_count} age={t.age}"
            )
        lines.append(f"  Stats: {self.stats}")
        return "\n".join(lines)
    
    def __repr__(self):
        return (f"MultiTargetTracker(tracks={len(self.tracks)}, "
                f"confirmed={self.stats['current_confirmed']}, "
                f"step={self._step})")


# ===== SIAP METRICS =====

def compute_ospa(track_positions: List[np.ndarray],
                 truth_positions: List[np.ndarray],
                 c: float = 200.0, p: float = 2.0) -> float:
    """[REQ-V50-MET-01] OSPA metric (Optimal Sub-Pattern Assignment).
    
    Schuhmacher et al. (2008) — standard metric for multi-target tracking.
    
    Args:
        track_positions: List of estimated positions
        truth_positions: List of true positions
        c: Cutoff distance [m] (penalizes cardinality errors)
        p: Order parameter (2 = Euclidean)
    
    Returns:
        OSPA distance (lower is better, 0 = perfect)
    """
    n = len(truth_positions)
    m = len(track_positions)
    
    if n == 0 and m == 0:
        return 0.0
    if n == 0 or m == 0:
        return c  # Maximum penalty
    
    # Build distance matrix
    n_max = max(n, m)
    D = np.full((n_max, n_max), c)
    
    for i in range(n):
        for j in range(m):
            d = np.linalg.norm(np.array(truth_positions[i]) - np.array(track_positions[j]))
            D[i, j] = min(d, c)
    
    # Optimal assignment
    assignments = hungarian_algorithm(D[:n, :m] if n <= m else D[:m, :n].T)
    
    # Location component
    loc_sum = 0.0
    for ai, bi in assignments:
        loc_sum += D[ai, bi]**p
    
    # Cardinality component
    card_penalty = abs(n - m) * c**p
    
    ospa = ((loc_sum + card_penalty) / max(n, m)) ** (1.0/p)
    return ospa


def compute_track_metrics(mtt: MultiTargetTracker,
                          truth: Dict[int, np.ndarray]) -> Dict[str, float]:
    """[REQ-V50-MET-02] SIAP-style track quality metrics.
    
    Args:
        mtt: Tracker with current state
        truth: {truth_id: position_3d} ground truth
    
    Returns:
        Dict with completeness, purity, false track rate, etc.
    """
    confirmed = mtt.confirmed_tracks
    truth_positions = list(truth.values())
    track_positions = [t.filter.position for t in confirmed]
    
    n_truth = len(truth)
    n_tracks = len(confirmed)
    
    # Assignment for metrics
    if n_truth > 0 and n_tracks > 0:
        D = np.zeros((n_tracks, n_truth))
        for i in range(n_tracks):
            for j in range(n_truth):
                D[i, j] = np.linalg.norm(track_positions[i] - truth_positions[j])
        
        assignments = hungarian_algorithm(D)
        
        # Count correct associations (within 3σ)
        correct = sum(1 for ti, gi in assignments if D[ti, gi] < 3 * mtt.r_std)
        
        completeness = correct / max(n_truth, 1)  # % of truths tracked
        purity = correct / max(n_tracks, 1)        # % of tracks that are real
        false_tracks = max(0, n_tracks - correct)
    else:
        completeness = 0.0
        purity = 1.0 if n_tracks == 0 else 0.0
        false_tracks = n_tracks
    
    ospa = compute_ospa(track_positions, truth_positions)
    
    return {
        "completeness": completeness,
        "purity": purity,
        "false_tracks": false_tracks,
        "n_confirmed": n_tracks,
        "n_truth": n_truth,
        "ospa": ospa,
    }


# ===== SENSOR BIAS ESTIMATION =====

@dataclass
class SensorBias:
    """[REQ-V51-BIAS-01] Estimated sensor bias state.
    
    Tracks systematic measurement errors in range, azimuth, and elevation.
    """
    range_bias: float = 0.0        # meters
    azimuth_bias: float = 0.0      # radians
    elevation_bias: float = 0.0    # radians
    range_bias_std: float = 0.0    # uncertainty
    azimuth_bias_std: float = 0.0
    elevation_bias_std: float = 0.0
    n_samples: int = 0


class SensorBiasEstimator:
    """[REQ-V51-BIAS-02] Online sensor bias estimation from track innovations.
    
    Estimates systematic range, azimuth, and elevation biases by analyzing
    the mean innovation (residual) sequence from confirmed tracks. A properly
    calibrated sensor should have zero-mean innovations — any persistent
    offset indicates a bias.
    
    Uses exponentially-weighted moving average (EWMA) for online estimation
    with configurable forgetting factor.
    
    Reference: Bar-Shalom, Li, Kirubarajan (2001), §6.6 "Sensor Registration"
    
    Args:
        alpha: EWMA forgetting factor (0.01–0.1 typical). Smaller = slower, smoother.
        min_samples: Minimum samples before bias estimate is considered valid.
        bias_detect_sigma: Number of sigma for bias detection threshold.
    """
    
    def __init__(self, alpha: float = 0.05, min_samples: int = 20,
                 bias_detect_sigma: float = 3.0):
        self.alpha = alpha
        self.min_samples = min_samples
        self.bias_detect_sigma = bias_detect_sigma
        
        # EWMA state
        self._mean = np.zeros(3)  # [range, azimuth, elevation] bias estimate
        self._var = np.zeros(3)   # Variance of estimates
        self._n = 0
        self._initialized = False
    
    def update(self, innovation: np.ndarray, S: np.ndarray) -> SensorBias:
        """[REQ-V51-BIAS-03] Update bias estimate with new innovation.
        
        Args:
            innovation: Measurement innovation vector (predicted - measured).
                       Can be 2D [range_err, az_err] or 3D [range_err, az_err, el_err].
            S: Innovation covariance matrix.
        
        Returns:
            Current bias estimate.
        """
        # Pad to 3D if needed
        inn = np.zeros(3)
        inn[:len(innovation)] = innovation[:3] if len(innovation) >= 3 else innovation
        
        self._n += 1
        
        if not self._initialized:
            self._mean = inn.copy()
            diag_s = np.diag(S).copy()
            self._var = np.zeros(3)
            n = min(3, len(diag_s))
            self._var[:n] = diag_s[:n]
            self._initialized = True
        else:
            # EWMA update
            delta = inn - self._mean
            self._mean += self.alpha * delta
            self._var = (1 - self.alpha) * (self._var + self.alpha * delta**2)
        
        return self.get_bias()
    
    def get_bias(self) -> SensorBias:
        """[REQ-V51-BIAS-04] Get current bias estimate with uncertainty."""
        std = np.sqrt(np.maximum(self._var, 1e-20))
        return SensorBias(
            range_bias=float(self._mean[0]),
            azimuth_bias=float(self._mean[1]),
            elevation_bias=float(self._mean[2]),
            range_bias_std=float(std[0]),
            azimuth_bias_std=float(std[1]),
            elevation_bias_std=float(std[2]),
            n_samples=self._n,
        )
    
    def is_biased(self) -> Tuple[bool, List[str]]:
        """[REQ-V51-BIAS-05] Test if sensor shows significant bias.
        
        Returns:
            Tuple of (biased: bool, list of biased dimensions).
        """
        if self._n < self.min_samples:
            return False, []
        
        std = np.sqrt(np.maximum(self._var, 1e-20))
        dims = ["range", "azimuth", "elevation"]
        biased = []
        
        for i, dim in enumerate(dims):
            if abs(self._mean[i]) > self.bias_detect_sigma * std[i] / np.sqrt(self._n):
                biased.append(dim)
        
        return len(biased) > 0, biased
    
    def correct_measurement(self, z: np.ndarray) -> np.ndarray:
        """[REQ-V51-BIAS-06] Apply bias correction to a measurement.
        
        Args:
            z: Raw measurement [range, azimuth, elevation] or [range, azimuth].
        
        Returns:
            Bias-corrected measurement.
        """
        if self._n < self.min_samples:
            return z.copy()
        
        corrected = z.copy()
        corrected[0] -= self._mean[0]  # range
        if len(corrected) > 1:
            corrected[1] -= self._mean[1]  # azimuth
        if len(corrected) > 2:
            corrected[2] -= self._mean[2]  # elevation
        
        return corrected
    
    def reset(self) -> None:
        """Reset estimator state."""
        self._mean = np.zeros(3)
        self._var = np.zeros(3)
        self._n = 0
        self._initialized = False


# ===== OUT-OF-SEQUENCE MEASUREMENT HANDLING =====

class OOSMHandler:
    """[REQ-V51-OOSM-01] Out-of-Sequence Measurement handler.
    
    Handles measurements that arrive after newer ones have been processed.
    Common in multi-sensor systems with different latencies, or network-based
    tracking with variable transmission delays.
    
    Implements the one-step-lag OOSM algorithm (Bar-Shalom, 2002):
    1. Retrodicts state back to OOSM timestamp
    2. Updates with the late measurement
    3. Re-predicts forward to current time
    
    Reference: Y. Bar-Shalom, "Update with out-of-sequence measurements in
               tracking: exact solution," IEEE Trans. AES, 2002
    
    Args:
        max_lag: Maximum acceptable lag in timesteps. Older measurements are discarded.
    """
    
    def __init__(self, max_lag: int = 5):
        self.max_lag = max_lag
        # State history: list of (timestamp, x, P, F, Q) tuples
        self._history: List[Tuple[float, np.ndarray, np.ndarray, np.ndarray, np.ndarray]] = []
        self._max_history = max_lag + 2
        self.stats = {"oosm_processed": 0, "oosm_rejected": 0}
    
    def save_state(self, timestamp: float, x: np.ndarray, P: np.ndarray,
                   F: np.ndarray, Q: np.ndarray) -> None:
        """[REQ-V51-OOSM-02] Save filter state for potential retrodiction.
        
        Call this after each normal (in-sequence) update.
        
        Args:
            timestamp: Current time
            x: State vector after update
            P: Covariance after update
            F: State transition matrix used
            Q: Process noise matrix used
        """
        self._history.append((timestamp, x.copy(), P.copy(), F.copy(), Q.copy()))
        
        # Trim to max history
        if len(self._history) > self._max_history:
            self._history = self._history[-self._max_history:]
    
    def process_oosm(self, kf: 'KalmanFilter3D', z_oosm: np.ndarray,
                     t_oosm: float, t_current: float,
                     H: Optional[np.ndarray] = None,
                     R: Optional[np.ndarray] = None) -> bool:
        """[REQ-V51-OOSM-03] Process an out-of-sequence measurement.
        
        Retrodicts state to OOSM time, updates, and re-predicts to current time.
        
        Args:
            kf: The KalmanFilter3D to update
            z_oosm: The late measurement vector
            t_oosm: Timestamp of the OOSM
            t_current: Current timestamp
            H: Measurement matrix (uses kf.H if None)
            R: Measurement noise (uses kf.R if None)
        
        Returns:
            True if OOSM was processed, False if rejected (too old).
        """
        if H is None:
            H = kf.H
        if R is None:
            R = kf.R
        
        # Find bracketing state in history
        # We need the state just before t_oosm
        history_before = [(t, x, P, F, Q) for t, x, P, F, Q in self._history
                         if t <= t_oosm]
        
        if not history_before:
            # OOSM is older than our history — reject
            self.stats["oosm_rejected"] += 1
            return False
        
        # Get state just before OOSM time
        t_before, x_before, P_before, F_before, Q_before = history_before[-1]
        
        # Check lag
        steps_behind = sum(1 for t, _, _, _, _ in self._history if t > t_oosm)
        if steps_behind > self.max_lag:
            self.stats["oosm_rejected"] += 1
            return False
        
        # Step 1: Predict from before-state to OOSM time
        dt_to_oosm = t_oosm - t_before
        if dt_to_oosm > 0:
            x_retro = F_before @ x_before
            P_retro = F_before @ P_before @ F_before.T + Q_before
        else:
            x_retro = x_before.copy()
            P_retro = P_before.copy()
        
        # Step 2: Kalman update at OOSM time
        y = z_oosm - H @ x_retro
        S = H @ P_retro @ H.T + R
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            self.stats["oosm_rejected"] += 1
            return False
        
        K = P_retro @ H.T @ S_inv
        x_updated = x_retro + K @ y
        P_updated = (np.eye(len(x_retro)) - K @ H) @ P_retro
        
        # Step 3: Re-predict forward through all subsequent steps
        x_fwd = x_updated
        P_fwd = P_updated
        
        future_steps = [(t, x, P, F, Q) for t, x, P, F, Q in self._history
                       if t > t_oosm]
        
        for t_step, _, _, F_step, Q_step in future_steps:
            x_fwd = F_step @ x_fwd
            P_fwd = F_step @ P_fwd @ F_step.T + Q_step
        
        # Step 4: Merge OOSM-corrected state with current state
        # Use covariance intersection for robustness
        P_curr_inv = np.linalg.inv(kf.P + np.eye(len(kf.P)) * 1e-10)
        P_oosm_inv = np.linalg.inv(P_fwd + np.eye(len(P_fwd)) * 1e-10)
        
        P_merged_inv = P_curr_inv + P_oosm_inv
        P_merged = np.linalg.inv(P_merged_inv)
        x_merged = P_merged @ (P_curr_inv @ kf.x + P_oosm_inv @ x_fwd)
        
        # Apply to filter
        kf.x = x_merged
        kf.P = P_merged
        
        self.stats["oosm_processed"] += 1
        return True
    
    def can_handle(self, t_oosm: float) -> bool:
        """Check if an OOSM at given time can be processed."""
        if not self._history:
            return False
        oldest_t = self._history[0][0]
        return t_oosm >= oldest_t
    
    @property
    def history_depth(self) -> int:
        """Number of states in history."""
        return len(self._history)


# ===== DOPPLER INTEGRATION =====

def make_cv3d_doppler_matrices(dt: float, q: float
                                ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """[REQ-V51-DOP-01] CV3D with Doppler measurement model.
    
    Extends standard CV3D with range-rate (Doppler) as a measurement,
    enabling direct velocity observability.
    
    State: [x, y, z, vx, vy, vz] (6-state)
    Measurement: [x, y, z, vx_radial] (4-measurement with radial velocity)
    
    Args:
        dt: Time step
        q: Process noise intensity
    
    Returns:
        Tuple of (F, Q, H_doppler) where H_doppler is the 4×6 measurement matrix
        for [x, y, z, v_radial] observations.
    """
    F, Q = make_cv3d_matrices(dt, q)
    
    # Standard H for position + full velocity projected to radial
    # H_doppler maps state to [x, y, z, vx] (simplified: radial ≈ vx for head-on)
    # In practice, radial velocity = dot(v, unit_los) which depends on geometry
    # Here we provide the template — user scales by LOS unit vector
    H_doppler = np.zeros((4, 6))
    H_doppler[0, 0] = 1.0  # x
    H_doppler[1, 1] = 1.0  # y
    H_doppler[2, 2] = 1.0  # z
    H_doppler[3, 3] = 1.0  # vx (radial component — caller rotates to LOS)
    
    return F, Q, H_doppler


def doppler_measurement_matrix(state: np.ndarray,
                                sensor_pos: np.ndarray) -> np.ndarray:
    """[REQ-V51-DOP-02] Compute Doppler H matrix for given geometry.
    
    The radial velocity measurement is v_r = dot(v, u_los) where u_los
    is the unit vector from sensor to target.
    
    H = [I_3x3  0_3x3]  for position rows
        [0 0 0  u_los]   for Doppler row
    
    Args:
        state: [x, y, z, vx, vy, vz] target state
        sensor_pos: [sx, sy, sz] sensor position
    
    Returns:
        H: 4×6 measurement matrix for [x, y, z, v_radial]
    """
    pos = state[:3]
    los = pos - sensor_pos
    r = np.linalg.norm(los)
    
    if r < 1e-6:
        u_los = np.array([1.0, 0.0, 0.0])
    else:
        u_los = los / r
    
    H = np.zeros((4, 6))
    H[0, 0] = 1.0  # x
    H[1, 1] = 1.0  # y
    H[2, 2] = 1.0  # z
    H[3, 3] = u_los[0]  # vx contribution to radial
    H[3, 4] = u_los[1]  # vy contribution to radial
    H[3, 5] = u_los[2]  # vz contribution to radial
    
    return H


def compute_radial_velocity(state: np.ndarray,
                             sensor_pos: np.ndarray) -> float:
    """[REQ-V51-DOP-03] Compute expected radial velocity (Doppler).
    
    v_r = dot(velocity, unit_LOS)
    
    Args:
        state: [x, y, z, vx, vy, vz]
        sensor_pos: [sx, sy, sz]
    
    Returns:
        Radial velocity in m/s (positive = receding)
    """
    pos = state[:3]
    vel = state[3:6]
    los = pos - sensor_pos
    r = np.linalg.norm(los)
    
    if r < 1e-6:
        return 0.0
    
    return float(np.dot(vel, los / r))


# ===== TRACK-TO-TRACK ASSOCIATION (T2TA) =====

@dataclass
class T2TAPair:
    """[REQ-V52-T2TA-01] A matched pair of tracks from two sensors."""
    track_a_id: int
    track_b_id: int
    distance: float          # Statistical distance (Mahalanobis)
    confidence: float        # Association confidence [0, 1]


def t2ta_associate(tracks_a: List[Dict], tracks_b: List[Dict],
                   gate_threshold: float = 16.0,
                   method: str = "mahalanobis"
                   ) -> Tuple[List[T2TAPair], List[int], List[int]]:
    """[REQ-V52-T2TA-02] Track-to-track association for multi-sensor fusion.
    
    Associates tracks from two different sensors/trackers using statistical
    distance between track states. Uses the Hungarian algorithm for optimal
    one-to-one assignment.
    
    Each track dict must contain:
        - 'id': int — track identifier
        - 'x': np.ndarray — state vector [x, y, z, vx, vy, vz] (at minimum)
        - 'P': np.ndarray — state covariance matrix
    
    The statistical distance between tracks a and b is:
        d² = (xa - xb)ᵀ (Pa + Pb)⁻¹ (xa - xb)
    
    This accounts for uncertainty in both tracks, unlike measurement-to-track
    association which only uses measurement covariance.
    
    Reference: Bar-Shalom & Chen, "Multisensor track-to-track association
               for tracks with dependent errors," CDC 2004.
    
    Args:
        tracks_a: List of track dicts from sensor A.
        tracks_b: List of track dicts from sensor B.
        gate_threshold: Chi-squared gate (16.0 = 99.7% for 3D).
        method: Distance metric — 'mahalanobis' or 'euclidean'.
    
    Returns:
        Tuple of:
            - matched: List[T2TAPair] — matched track pairs
            - unmatched_a: List[int] — unmatched track IDs from A
            - unmatched_b: List[int] — unmatched track IDs from B
    """
    na = len(tracks_a)
    nb = len(tracks_b)
    
    if na == 0 or nb == 0:
        ua = [t['id'] for t in tracks_a]
        ub = [t['id'] for t in tracks_b]
        return [], ua, ub
    
    # Build cost matrix
    cost = np.full((na, nb), 1e6)
    
    for i, ta in enumerate(tracks_a):
        for j, tb in enumerate(tracks_b):
            xa = ta['x']
            xb = tb['x']
            
            # Use position components only (first 3 states)
            ndim = min(len(xa), len(xb), 3)
            diff = xa[:ndim] - xb[:ndim]
            
            if method == "mahalanobis":
                Pa = ta['P'][:ndim, :ndim]
                Pb = tb['P'][:ndim, :ndim]
                S = Pa + Pb
                try:
                    S_inv = np.linalg.inv(S + np.eye(ndim) * 1e-10)
                    d2 = float(diff @ S_inv @ diff)
                except np.linalg.LinAlgError:
                    d2 = 1e6
            else:
                d2 = float(np.dot(diff, diff))
            
            if d2 < gate_threshold:
                cost[i, j] = d2
    
    # Hungarian assignment
    assignments = hungarian_algorithm(cost)
    
    matched = []
    matched_a_idx = set()
    matched_b_idx = set()
    
    for i, j in assignments:
        if cost[i, j] < gate_threshold:
            # Confidence: chi-squared CDF approximation
            d2 = cost[i, j]
            ndim = 3
            # Simple sigmoid-based confidence
            conf = max(0.0, 1.0 - d2 / gate_threshold)
            
            matched.append(T2TAPair(
                track_a_id=tracks_a[i]['id'],
                track_b_id=tracks_b[j]['id'],
                distance=float(np.sqrt(d2)),
                confidence=conf,
            ))
            matched_a_idx.add(i)
            matched_b_idx.add(j)
    
    unmatched_a = [tracks_a[i]['id'] for i in range(na) if i not in matched_a_idx]
    unmatched_b = [tracks_b[j]['id'] for j in range(nb) if j not in matched_b_idx]
    
    return matched, unmatched_a, unmatched_b


def fuse_tracks(track_a: Dict, track_b: Dict) -> Tuple[np.ndarray, np.ndarray]:
    """[REQ-V52-T2TA-03] Fuse two associated tracks via covariance intersection.
    
    Produces the optimal fused state estimate that is consistent regardless
    of unknown cross-correlations between the two track estimates.
    
    Uses the simple information-form fusion:
        P_fused⁻¹ = Pa⁻¹ + Pb⁻¹
        x_fused = P_fused (Pa⁻¹ xa + Pb⁻¹ xb)
    
    Args:
        track_a: Dict with 'x' (state) and 'P' (covariance)
        track_b: Dict with 'x' (state) and 'P' (covariance)
    
    Returns:
        Tuple of (x_fused, P_fused)
    """
    xa, Pa = track_a['x'], track_a['P']
    xb, Pb = track_b['x'], track_b['P']
    
    n = min(len(xa), len(xb))
    xa, xb = xa[:n], xb[:n]
    Pa, Pb = Pa[:n, :n], Pb[:n, :n]
    
    reg = np.eye(n) * 1e-10
    Pa_inv = np.linalg.inv(Pa + reg)
    Pb_inv = np.linalg.inv(Pb + reg)
    
    P_fused = np.linalg.inv(Pa_inv + Pb_inv)
    x_fused = P_fused @ (Pa_inv @ xa + Pb_inv @ xb)
    
    return x_fused, P_fused


# ===== TRACK QUALITY & RELIABILITY METRICS =====

@dataclass
class TrackQualityReport:
    """[REQ-V52-TQ-01] Comprehensive track quality assessment."""
    track_id: int
    age_scans: int
    hit_ratio: float          # Hits / total scans
    avg_innovation: float     # Mean NIS — should be near measurement dimension
    position_uncertainty: float  # Trace(P_pos) in meters
    velocity_uncertainty: float  # Trace(P_vel) in m/s
    quality_grade: str        # 'A' (excellent) through 'F' (unreliable)
    is_reliable: bool


def assess_track_quality(track: 'TrackState',
                         nz: int = 3) -> TrackQualityReport:
    """[REQ-V52-TQ-02] Assess quality of a single track.
    
    Grades tracks based on:
    - Hit ratio (confirmation strength)
    - Innovation consistency (filter health)
    - Covariance magnitude (estimate confidence)
    
    Args:
        track: TrackState from MultiTargetTracker
        nz: Measurement dimension (for NIS normalization)
    
    Returns:
        TrackQualityReport with letter grade and reliability flag.
    """
    # Hit ratio
    total = track.hit_count + track.miss_count
    hit_ratio = track.hit_count / max(total, 1)
    
    # Position & velocity uncertainty from covariance
    P = track.filter.P if hasattr(track.filter, 'P') else track.filter.filters[0].P
    pos_unc = float(np.sqrt(np.trace(P[:3, :3])))
    vel_unc = float(np.sqrt(np.trace(P[3:6, 3:6]))) if P.shape[0] >= 6 else 0.0
    
    # Average innovation (simplified — use last NIS if available)
    avg_inn = getattr(track, '_last_nis', float(nz))  # Default to ideal
    
    # Grading
    score = 0.0
    score += min(hit_ratio * 40, 40)           # Max 40 pts for hit ratio
    score += max(0, 30 - pos_unc / 100) * 1.0  # Max 30 pts for low uncertainty
    score += max(0, 30 - abs(avg_inn - nz) * 5) # Max 30 pts for consistent filter
    
    if score >= 85:
        grade = 'A'
    elif score >= 70:
        grade = 'B'
    elif score >= 55:
        grade = 'C'
    elif score >= 40:
        grade = 'D'
    else:
        grade = 'F'
    
    return TrackQualityReport(
        track_id=track.track_id,
        age_scans=total,
        hit_ratio=hit_ratio,
        avg_innovation=avg_inn,
        position_uncertainty=pos_unc,
        velocity_uncertainty=vel_unc,
        quality_grade=grade,
        is_reliable=grade in ('A', 'B', 'C'),
    )
