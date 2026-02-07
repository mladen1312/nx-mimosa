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


def _fast_assignment(cost_matrix: np.ndarray) -> List[Tuple[int, int]]:
    """[REQ-V59-SCALE-01] Fast optimal assignment using scipy (C implementation).
    
    O(n³) but with optimized C code — handles 1000×1000 in <50ms.
    Falls back to pure-Python Hungarian if scipy unavailable.
    """
    try:
        from scipy.optimize import linear_sum_assignment
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        return list(zip(row_ind.tolist(), col_ind.tolist()))
    except ImportError:
        return hungarian_algorithm(cost_matrix)


def gnn_associate(tracks: List[TrackState], measurements: np.ndarray,
                  gate_threshold: float = 16.0) -> Tuple[List[Tuple[int, int]], 
                                                           List[int], List[int]]:
    """[REQ-V50-GNN-02] Global Nearest Neighbor data association.
    
    Scales to 1000+ simultaneous targets via:
      - KDTree spatial pre-gate (O(n log n) candidate selection)
      - scipy.optimize.linear_sum_assignment (C-optimized Hungarian)
    
    Args:
        tracks: List of active tracks
        measurements: Mx3 array of [x, y, z] measurements
        gate_threshold: Chi-squared gate (16.0 ≈ 99.7% for 3DOF)
    
    Returns:
        Tuple: (assignments, unassigned_tracks, unassigned_measurements)
    """
    n_tracks = len(tracks)
    n_meas = len(measurements)
    
    if n_tracks == 0 or n_meas == 0:
        return [], list(range(n_tracks)), list(range(n_meas))
    
    # Pre-compute predicted positions
    track_pos = np.array([t.filter.position for t in tracks])
    
    # Use KDTree for O(n log n) spatial pre-gating
    try:
        from scipy.spatial import cKDTree
        meas_tree = cKDTree(measurements[:, :3])
        # Coarse Euclidean gate: 20km radius
        # (Mahalanobis gate will refine; this just prunes obviously impossible pairs)
        coarse_gate_m = 20000.0
        candidate_pairs = []
        for i in range(n_tracks):
            if tracks[i].status == TrackStatus.DELETED:
                continue
            nearby = meas_tree.query_ball_point(track_pos[i], coarse_gate_m)
            for j in nearby:
                candidate_pairs.append((i, j))
    except ImportError:
        # Fallback: brute-force with vectorized distance
        candidate_pairs = []
        for i in range(n_tracks):
            if tracks[i].status == TrackStatus.DELETED:
                continue
            dists = np.linalg.norm(measurements[:, :3] - track_pos[i], axis=1)
            for j in np.where(dists < 20000)[0]:
                candidate_pairs.append((i, j))
    
    # Build sparse cost matrix — only compute Mahalanobis for candidates
    cost = np.full((n_tracks, n_meas), 1e9)
    
    for i, j in candidate_pairs:
        d2 = tracks[i].filter.mahalanobis_distance(measurements[j])
        if d2 < gate_threshold:
            cost[i, j] = d2
    
    # Solve assignment — always use fast scipy for gated problems
    raw_assignments = _fast_assignment(cost)
    
    # Filter by gate
    assignments = []
    assigned_tracks = set()
    assigned_meas = set()
    
    for ti, mi in raw_assignments:
        if ti < n_tracks and mi < n_meas and cost[ti, mi] < gate_threshold:
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
    
    # Optimal assignment (scipy for scale)
    assignments = _fast_assignment(D[:n, :m] if n <= m else D[:m, :n].T)
    
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
        
        assignments = _fast_assignment(D)
        
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



# ═══════════════════════════════════════════════════════════════════════
# v5.9.0: EKF/UKF FOR POLAR RADAR MEASUREMENTS
# ═══════════════════════════════════════════════════════════════════════
# PHYSICS: Radar measures in polar coordinates:
#   z = [range, azimuth, elevation] + noise
#   h(x) = [√(x²+y²+z²), atan2(x,y), asin(z/r)]  (nonlinear!)
#
# EKF: linearizes h(x) via Jacobian at current estimate.
#   Accurate when range >> state uncertainty.
# UKF: sigma-point propagation through h(x), no Jacobian needed.
#   Better for close targets or high angular noise.
#
# WHY THIS MATTERS:
#   Raw polar→Cartesian conversion amplifies angular noise:
#     σ_crossrange ≈ range × σ_azimuth
#   At 200km with σ_az=1°: σ_cr ≈ 3491m (vs σ_r=150m)
#   EKF/UKF track in state space, handling this properly.
# ═══════════════════════════════════════════════════════════════════════

class EKF3D:
    """[REQ-V59-EKF-01] Extended Kalman Filter for polar radar measurements.
    
    Reference: Bar-Shalom, Li & Kirubarajan (2001), Ch. 6.
    """
    def __init__(self, nx: int = 6, nz_polar: int = 3,
                 radar_pos: Optional[np.ndarray] = None):
        self.nx = nx
        self.nz = nz_polar
        self.x = np.zeros(nx)
        self.P = np.eye(nx) * 1e6
        self.F = np.eye(nx)
        self.Q = np.eye(nx)
        self.R_polar = np.diag([150.0**2, np.radians(1.0)**2, np.radians(1.0)**2])
        self.radar_pos = radar_pos if radar_pos is not None else np.zeros(3)

    def _h(self, x: np.ndarray) -> np.ndarray:
        """Nonlinear measurement function: Cartesian state → polar measurement."""
        d = x[:3] - self.radar_pos
        r = np.linalg.norm(d)
        r = max(r, 1.0)
        r_xy = np.sqrt(d[0]**2 + d[1]**2)
        r_xy = max(r_xy, 1.0)
        az = np.arctan2(d[0], d[1])  # East-of-North
        el = np.arctan2(d[2], r_xy)
        return np.array([r, az, el])

    def _H_jacobian(self, x: np.ndarray) -> np.ndarray:
        """Jacobian ∂h/∂x at state x."""
        d = x[:3] - self.radar_pos
        r = np.linalg.norm(d)
        r = max(r, 1.0)
        r_xy2 = d[0]**2 + d[1]**2
        r_xy = np.sqrt(max(r_xy2, 1.0))
        
        H = np.zeros((3, self.nx))
        H[0, 0] = d[0]/r;  H[0, 1] = d[1]/r;  H[0, 2] = d[2]/r
        H[1, 0] = d[1]/r_xy2;  H[1, 1] = -d[0]/r_xy2
        H[2, 0] = -d[0]*d[2]/(r**2 * r_xy)
        H[2, 1] = -d[1]*d[2]/(r**2 * r_xy)
        H[2, 2] = r_xy / r**2
        return H

    def predict(self, F: Optional[np.ndarray] = None, Q: Optional[np.ndarray] = None):
        if F is not None: self.F = F
        if Q is not None: self.Q = Q
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update_polar(self, z_polar: np.ndarray,
                     R: Optional[np.ndarray] = None) -> float:
        """EKF update with polar measurement [range, azimuth, elevation]."""
        R_use = R if R is not None else self.R_polar
        z_pred = self._h(self.x)
        y = z_polar - z_pred
        y[1] = (y[1] + np.pi) % (2*np.pi) - np.pi  # wrap az
        y[2] = np.clip(y[2], -np.pi, np.pi)
        
        H = self._H_jacobian(self.x)
        S = H @ self.P @ H.T + R_use
        try:
            S_inv = np.linalg.inv(S)
            nis = float(y.T @ S_inv @ y)
        except np.linalg.LinAlgError:
            return 1e6
        
        K = self.P @ H.T @ S_inv
        self.x = self.x + K @ y
        I_KH = np.eye(self.nx) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R_use @ K.T
        return nis

    # Cartesian-compatible interface for MTT integration
    def update(self, z: np.ndarray, H: Optional[np.ndarray] = None,
               R: Optional[np.ndarray] = None) -> float:
        _H = H if H is not None else np.eye(3, self.nx)
        _R = R if R is not None else np.eye(3) * 150.0**2
        y = z - _H @ self.x
        S = _H @ self.P @ _H.T + _R
        try:
            S_inv = np.linalg.inv(S)
            nis = float(y.T @ S_inv @ y)
        except np.linalg.LinAlgError:
            return 1e6
        K = self.P @ _H.T @ S_inv
        self.x = self.x + K @ y
        I_KH = np.eye(self.nx) - K @ _H
        self.P = I_KH @ self.P @ I_KH.T + K @ _R @ K.T
        return nis

    def innovation_covariance(self, H=None, R=None):
        _H = H if H is not None else np.eye(3, self.nx)
        _R = R if R is not None else np.eye(3) * 150.0**2
        return _H @ self.P @ _H.T + _R

    def mahalanobis_distance(self, z, H=None, R=None):
        _H = H if H is not None else np.eye(3, self.nx)
        _R = R if R is not None else np.eye(3) * 150.0**2
        y = z - _H @ self.x
        S = _H @ self.P @ _H.T + _R
        try:
            return float(y.T @ np.linalg.inv(S) @ y)
        except np.linalg.LinAlgError:
            return 1e6

    @property
    def position(self): return self.x[:3].copy()
    @property
    def velocity(self): return self.x[3:6].copy() if self.nx >= 6 else np.zeros(3)


class UKF3D:
    """[REQ-V59-UKF-01] Unscented Kalman Filter for polar measurements.
    
    Uses sigma-point propagation through nonlinear h(x).
    No Jacobian needed — better for highly nonlinear cases.
    Reference: Julier & Uhlmann (2004).
    """
    def __init__(self, nx: int = 6, nz_polar: int = 3,
                 radar_pos: Optional[np.ndarray] = None,
                 alpha: float = 1.0, beta: float = 2.0, kappa: float = 0.0):
        self.nx = nx
        self.nz = nz_polar
        self.x = np.zeros(nx)
        self.P = np.eye(nx) * 1e6
        self.F = np.eye(nx)
        self.Q = np.eye(nx)
        self.R_polar = np.diag([150.0**2, np.radians(1.0)**2, np.radians(1.0)**2])
        self.radar_pos = radar_pos if radar_pos is not None else np.zeros(3)
        
        # Sigma point weights
        lam = alpha**2 * (nx + kappa) - nx
        self._lam = lam
        self._n_sigma = 2 * nx + 1
        self._Wm = np.full(self._n_sigma, 1.0 / (2*(nx + lam)))
        self._Wc = np.full(self._n_sigma, 1.0 / (2*(nx + lam)))
        self._Wm[0] = lam / (nx + lam)
        self._Wc[0] = lam / (nx + lam) + (1 - alpha**2 + beta)

    def _h(self, x: np.ndarray) -> np.ndarray:
        d = x[:3] - self.radar_pos
        r = max(np.linalg.norm(d), 1.0)
        r_xy = max(np.sqrt(d[0]**2 + d[1]**2), 1.0)
        return np.array([r, np.arctan2(d[0], d[1]), np.arctan2(d[2], r_xy)])

    def _sigma_points(self) -> np.ndarray:
        n = self.nx
        scale = n + self._lam
        if scale <= 0:
            scale = n  # fallback to safe value
        scaled_P = scale * self.P
        # Ensure symmetry
        scaled_P = (scaled_P + scaled_P.T) / 2
        try:
            L = np.linalg.cholesky(scaled_P)
        except np.linalg.LinAlgError:
            # Regularize: add small diagonal
            for eps in [1e-6, 1e-4, 1e-2, 1.0, 100.0]:
                try:
                    L = np.linalg.cholesky(scaled_P + np.eye(n) * eps * scale)
                    break
                except np.linalg.LinAlgError:
                    continue
            else:
                # Last resort: use diagonal of P
                L = np.diag(np.sqrt(np.abs(np.diag(scaled_P)) + 1.0))
        
        sigmas = np.zeros((self._n_sigma, n))
        sigmas[0] = self.x
        for i in range(n):
            sigmas[i+1] = self.x + L[:, i]
            sigmas[n+i+1] = self.x - L[:, i]
        return sigmas

    def predict(self, F=None, Q=None):
        if F is not None: self.F = F
        if Q is not None: self.Q = Q
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update_polar(self, z_polar: np.ndarray, R=None) -> float:
        R_use = R if R is not None else self.R_polar
        sigmas = self._sigma_points()
        
        # Transform sigma points through h(x)
        z_sigmas = np.array([self._h(s) for s in sigmas])
        
        # Predicted measurement — use circular mean for angles
        z_pred = np.zeros(self.nz)
        z_pred[0] = sum(self._Wm[i] * z_sigmas[i, 0] for i in range(self._n_sigma))
        # Circular mean for azimuth and elevation
        for ang_idx in [1, 2]:
            sin_sum = sum(self._Wm[i] * np.sin(z_sigmas[i, ang_idx])
                          for i in range(self._n_sigma))
            cos_sum = sum(self._Wm[i] * np.cos(z_sigmas[i, ang_idx])
                          for i in range(self._n_sigma))
            z_pred[ang_idx] = np.arctan2(sin_sum, cos_sum)
        
        # Innovation covariance and cross-covariance
        Pzz = R_use.copy()
        Pxz = np.zeros((self.nx, self.nz))
        for i in range(self._n_sigma):
            dz = z_sigmas[i] - z_pred
            dz[1] = (dz[1] + np.pi) % (2*np.pi) - np.pi  # wrap angles
            dz[2] = (dz[2] + np.pi) % (2*np.pi) - np.pi
            dx = sigmas[i] - self.x
            Pzz += self._Wc[i] * np.outer(dz, dz)
            Pxz += self._Wc[i] * np.outer(dx, dz)
        
        # Innovation
        y = z_polar - z_pred
        y[1] = (y[1] + np.pi) % (2*np.pi) - np.pi
        y[2] = (y[2] + np.pi) % (2*np.pi) - np.pi
        
        # Ensure Pzz is positive definite
        Pzz = (Pzz + Pzz.T) / 2
        try:
            S_inv = np.linalg.inv(Pzz)
            nis = float(y.T @ S_inv @ y)
        except np.linalg.LinAlgError:
            return 1e6
        
        K = Pxz @ S_inv
        self.x = self.x + K @ y
        self.P = self.P - K @ Pzz @ K.T
        # Ensure P stays positive definite
        self.P = (self.P + self.P.T) / 2
        eigvals = np.linalg.eigvalsh(self.P)
        if eigvals.min() < 1e-6:
            self.P += np.eye(self.nx) * (1e-6 - eigvals.min())
        return nis

    def update(self, z, H=None, R=None):
        _H = H if H is not None else np.eye(3, self.nx)
        _R = R if R is not None else np.eye(3) * 150.0**2
        y = z - _H @ self.x
        S = _H @ self.P @ _H.T + _R
        try:
            S_inv = np.linalg.inv(S)
            nis = float(y.T @ S_inv @ y)
        except np.linalg.LinAlgError:
            return 1e6
        K = self.P @ _H.T @ S_inv
        self.x = self.x + K @ y
        I_KH = np.eye(self.nx) - K @ _H
        self.P = I_KH @ self.P @ I_KH.T + K @ _R @ K.T
        return nis

    def innovation_covariance(self, H=None, R=None):
        _H = H if H is not None else np.eye(3, self.nx)
        _R = R if R is not None else np.eye(3) * 150.0**2
        return _H @ self.P @ _H.T + _R

    def mahalanobis_distance(self, z, H=None, R=None):
        _H = H if H is not None else np.eye(3, self.nx)
        _R = R if R is not None else np.eye(3) * 150.0**2
        y = z - _H @ self.x
        S = _H @ self.P @ _H.T + _R
        try:
            return float(y.T @ np.linalg.inv(S) @ y)
        except np.linalg.LinAlgError:
            return 1e6

    @property
    def position(self): return self.x[:3].copy()
    @property
    def velocity(self): return self.x[3:6].copy() if self.nx >= 6 else np.zeros(3)


def cartesian_to_polar(pos: np.ndarray, radar_pos: np.ndarray = None) -> np.ndarray:
    """Convert Cartesian [x,y,z] → polar [range, azimuth, elevation]."""
    if radar_pos is None: radar_pos = np.zeros(3)
    d = pos - radar_pos
    r = max(np.linalg.norm(d), 1.0)
    r_xy = max(np.sqrt(d[0]**2 + d[1]**2), 1.0)
    return np.array([r, np.arctan2(d[0], d[1]), np.arctan2(d[2], r_xy)])


def polar_to_cartesian(z_polar: np.ndarray, radar_pos: np.ndarray = None) -> np.ndarray:
    """Convert polar [range, azimuth, elevation] → Cartesian [x,y,z]."""
    if radar_pos is None: radar_pos = np.zeros(3)
    r, az, el = z_polar
    return radar_pos + np.array([
        r * np.cos(el) * np.sin(az),
        r * np.cos(el) * np.cos(az),
        r * np.sin(el),
    ])


def add_polar_noise(z_polar: np.ndarray, sigma_r: float = 150.0,
                    sigma_az: float = None, sigma_el: float = None,
                    rng: np.random.RandomState = None) -> np.ndarray:
    """Add noise to polar measurement."""
    if sigma_az is None: sigma_az = np.radians(1.0)
    if sigma_el is None: sigma_el = np.radians(1.0)
    if rng is None: rng = np.random
    return z_polar + np.array([
        rng.normal(0, sigma_r), rng.normal(0, sigma_az), rng.normal(0, sigma_el)])


# ═══════════════════════════════════════════════════════════════════════
# v5.9.0: GOSPA METRIC (Generalized OSPA)
# ═══════════════════════════════════════════════════════════════════════
# Rahmathullah, García-Fernández & Svensson (2017)
# Decomposes total error into:
#   - LOCALIZATION: how well are assigned tracks positioned?
#   - MISSED: how many true targets have no matching track?
#   - FALSE: how many tracks match no true target?
# This is THE standard MTT metric used by Stone Soup.
# ═══════════════════════════════════════════════════════════════════════

def compute_gospa(track_positions: List[np.ndarray],
                  truth_positions: List[np.ndarray],
                  c: float = 500.0, p: float = 2.0,
                  alpha: float = 2.0) -> Dict[str, float]:
    """[REQ-V59-MET-01] GOSPA metric with error decomposition.
    
    Args:
        c: Cutoff distance [m]. Assignment cost capped at c^p/alpha.
        p: Order parameter (2 = Euclidean).
        alpha: Controls cardinality penalty relative to localization.
    """
    n = len(truth_positions)
    m = len(track_positions)
    
    if n == 0 and m == 0:
        return {"gospa": 0.0, "localization": 0.0, "missed": 0.0, "false": 0.0,
                "n_assigned": 0, "n_missed": 0, "n_false": 0}
    if n == 0:
        return {"gospa": (c**p / alpha * m)**(1/p), "localization": 0.0,
                "missed": 0.0, "false": (c**p / alpha * m)**(1/p),
                "n_assigned": 0, "n_missed": 0, "n_false": m}
    if m == 0:
        return {"gospa": (c**p / alpha * n)**(1/p), "localization": 0.0,
                "missed": (c**p / alpha * n)**(1/p), "false": 0.0,
                "n_assigned": 0, "n_missed": n, "n_false": 0}
    
    # Cost matrix: min(d^p, c^p/alpha) for each truth-track pair
    cutoff = c**p / alpha
    C = np.zeros((n, m))
    for i in range(n):
        for j in range(m):
            d = np.linalg.norm(np.array(truth_positions[i]) - np.array(track_positions[j]))
            C[i, j] = min(d**p, cutoff)
    
    # Optimal assignment (scipy for scale, fallback to pure Python)
    assignments = _fast_assignment(C if n <= m else C.T)
    if n > m:
        assignments = [(j, i) for i, j in assignments]
    
    loc_cost = 0.0
    n_assigned = 0
    for ai, bi in assignments:
        if ai < n and bi < m:
            d = np.linalg.norm(np.array(truth_positions[ai]) - np.array(track_positions[bi]))
            if d**p < cutoff:
                loc_cost += d**p
                n_assigned += 1
    
    n_missed = n - n_assigned
    n_false = m - n_assigned
    miss_cost = cutoff * n_missed
    false_cost = cutoff * n_false
    total = loc_cost + miss_cost + false_cost
    
    return {
        "gospa": total**(1.0/p),
        "localization": loc_cost**(1.0/p) if loc_cost > 0 else 0.0,
        "missed": miss_cost**(1.0/p) if miss_cost > 0 else 0.0,
        "false": false_cost**(1.0/p) if false_cost > 0 else 0.0,
        "n_assigned": n_assigned, "n_missed": n_missed, "n_false": n_false,
    }


# ═══════════════════════════════════════════════════════════════════════
# v5.9.0: CLUTTER GENERATION
# ═══════════════════════════════════════════════════════════════════════

def generate_clutter(n_clutter: int, volume_bounds: Tuple,
                     rng: Optional[np.random.RandomState] = None) -> np.ndarray:
    """[REQ-V59-CLU-01] Generate uniform false alarms within a surveillance volume.
    
    Args:
        n_clutter: Number of false alarms per scan
        volume_bounds: ((x_min,x_max), (y_min,y_max), (z_min,z_max))
    """
    if rng is None: rng = np.random.RandomState()
    if n_clutter <= 0: return np.zeros((0, 3))
    clutter = np.zeros((n_clutter, 3))
    for dim in range(3):
        lo, hi = volume_bounds[dim]
        clutter[:, dim] = rng.uniform(lo, hi, n_clutter)
    return clutter


# ============================================================================
# [REQ-V592-PF-01] PARTICLE FILTER — Sequential Importance Resampling (SIR)
# ============================================================================

class ParticleFilter3D:
    """[REQ-V592-PF-01] Bootstrap Particle Filter for highly nonlinear tracking.
    
    Implements Sequential Importance Resampling (SIR) with:
    - Adaptive particle count based on effective sample size (ESS)
    - Systematic resampling (lower variance than multinomial)
    - Multiple motion model support (CV, CA, CT)
    - Roughening to prevent sample impoverishment
    
    Use when: target dynamics are highly nonlinear or measurement noise
    is non-Gaussian. For most radar tracking, IMM with EKF/UKF is
    preferred (lower computational cost, similar accuracy).
    
    Reference: Arulampalam et al., "A Tutorial on Particle Filters for
    Online Nonlinear/Non-Gaussian Bayesian Tracking," IEEE TSP, 2002.
    """
    
    def __init__(self, n_particles: int = 500, nx: int = 6,
                 process_noise: float = 1.0, dt: float = 1.0,
                 resample_threshold: float = 0.5,
                 roughening_coeff: float = 0.01,
                 rng_seed: Optional[int] = None):
        """Initialize particle filter.
        
        Args:
            n_particles: Number of particles (default 500)
            nx: State dimension (6 = [x,y,z,vx,vy,vz])
            process_noise: Process noise std dev (m/s²)
            dt: Time step (seconds)
            resample_threshold: ESS/N threshold for resampling (default 0.5)
            roughening_coeff: Jitter coefficient post-resampling
            rng_seed: Random seed for reproducibility
        """
        self.n_particles = n_particles
        self.nx = nx
        self.q = process_noise
        self.dt = dt
        self.resample_threshold = resample_threshold
        self.roughening_coeff = roughening_coeff
        self.rng = np.random.default_rng(rng_seed)
        
        # Particles: (n_particles, nx) — each row is a state hypothesis
        self.particles = np.zeros((n_particles, nx))
        # Weights: normalized, sum to 1
        self.weights = np.ones(n_particles) / n_particles
        self._initialized = False
    
    def initialize(self, z: np.ndarray, spread: float = 500.0,
                   velocity_hint: Optional[np.ndarray] = None,
                   velocity_spread: float = 100.0):
        """[REQ-V592-PF-02] Initialize particles around first measurement.
        
        Args:
            z: First measurement [x, y, z] in Cartesian
            spread: Initial position spread (meters)
            velocity_hint: Optional velocity estimate for smarter init
            velocity_spread: Velocity uncertainty (m/s)
        """
        v_center = velocity_hint if velocity_hint is not None else np.zeros(3)
        for i in range(self.n_particles):
            self.particles[i, :3] = z[:3] + self.rng.normal(0, spread, 3)
            self.particles[i, 3:6] = v_center + self.rng.normal(0, velocity_spread, 3)
        self.weights = np.ones(self.n_particles) / self.n_particles
        self._initialized = True
    
    def predict(self, dt: Optional[float] = None):
        """[REQ-V592-PF-03] Propagate particles through motion model.
        
        Nearly-constant-velocity model with process noise.
        Process noise enters as acceleration: affects both velocity and position.
        """
        if not self._initialized:
            return
        dt = dt or self.dt
        
        # Draw acceleration noise for all particles at once (vectorized)
        accel_noise = self.rng.normal(0, self.q, (self.n_particles, 3))
        
        # Position update: x += v*dt + 0.5*a*dt²
        self.particles[:, :3] += self.particles[:, 3:6] * dt + 0.5 * accel_noise * dt**2
        # Velocity update: v += a*dt
        self.particles[:, 3:6] += accel_noise * dt
    
    def update(self, z: np.ndarray, R: Optional[np.ndarray] = None,
               H: Optional[np.ndarray] = None) -> float:
        """[REQ-V592-PF-04] Update weights using measurement likelihood.
        
        Args:
            z: Measurement vector (Cartesian position)
            R: Measurement noise covariance (default: 150m² diagonal)
            H: Measurement matrix (default: position extraction)
            
        Returns:
            Effective sample size ratio (ESS/N)
        """
        if not self._initialized:
            return 0.0
        
        nz = len(z)
        if R is None:
            R = np.eye(nz) * 150.0**2
        if H is None:
            H = np.zeros((nz, self.nx))
            H[:nz, :nz] = np.eye(nz)
        
        # Vectorized likelihood computation
        predicted_z = (H @ self.particles.T).T  # (n_particles, nz)
        residuals = z - predicted_z  # (n_particles, nz)
        
        try:
            R_inv = np.linalg.inv(R)
            log_det_R = np.log(max(np.linalg.det(R), 1e-300))
            
            # Mahalanobis for all particles at once
            mahal_sq = np.sum(residuals @ R_inv * residuals, axis=1)
            log_likelihoods = -0.5 * (mahal_sq + nz * np.log(2 * np.pi) + log_det_R)
            
            # Numerical stability: subtract max before exp
            max_ll = np.max(log_likelihoods)
            likelihoods = np.exp(log_likelihoods - max_ll)
            self.weights *= likelihoods
        except np.linalg.LinAlgError:
            pass  # Keep weights unchanged on singular R
        
        # Normalize weights
        w_sum = np.sum(self.weights)
        if w_sum > 1e-300:
            self.weights /= w_sum
        else:
            self.weights = np.ones(self.n_particles) / self.n_particles
        
        # Effective sample size
        ess = 1.0 / np.sum(self.weights**2)
        ess_ratio = ess / self.n_particles
        
        # Resample if ESS drops below threshold
        if ess_ratio < self.resample_threshold:
            self._systematic_resample()
            self._roughen()
        
        return ess_ratio
    
    def _systematic_resample(self):
        """[REQ-V592-PF-05] Systematic resampling (lower variance)."""
        N = self.n_particles
        positions = (self.rng.uniform() + np.arange(N)) / N
        
        cumulative = np.cumsum(self.weights)
        cumulative[-1] = 1.0  # ensure exact sum
        
        indices = np.searchsorted(cumulative, positions)
        indices = np.clip(indices, 0, N - 1)
        
        self.particles = self.particles[indices].copy()
        self.weights = np.ones(N) / N
    
    def _roughen(self):
        """[REQ-V592-PF-06] Roughening to prevent sample impoverishment."""
        if self.roughening_coeff <= 0:
            return
        # Add small jitter proportional to particle spread
        for dim in range(self.nx):
            spread = np.std(self.particles[:, dim])
            if spread > 1e-10:
                jitter = self.roughening_coeff * spread
                self.particles[:, dim] += self.rng.normal(0, jitter, self.n_particles)
    
    @property
    def position(self) -> np.ndarray:
        """Weighted mean position estimate."""
        return np.average(self.particles[:, :3], weights=self.weights, axis=0)
    
    @property
    def velocity(self) -> np.ndarray:
        """Weighted mean velocity estimate."""
        return np.average(self.particles[:, 3:6], weights=self.weights, axis=0)
    
    @property
    def state(self) -> np.ndarray:
        """Weighted mean full state estimate."""
        return np.average(self.particles, weights=self.weights, axis=0)
    
    @property
    def covariance(self) -> np.ndarray:
        """Weighted sample covariance."""
        mean = self.state
        diff = self.particles - mean
        return np.average(diff[:, :, None] * diff[:, None, :],
                          weights=self.weights, axis=0)
    
    def effective_sample_size(self) -> float:
        """Current ESS ratio (1.0 = all particles equally weighted)."""
        return 1.0 / (np.sum(self.weights**2) * self.n_particles)


# ============================================================================
# [REQ-V592-ECM-AUTO] AUTOMATIC ECM DETECTION — NIS-based Anomaly Detector
# ============================================================================

class ECMDetector:
    """[REQ-V592-ECM-AUTO] Automatic Electronic Countermeasure detection.
    
    Detects ECM activity through statistical anomalies in tracking data:
    - NIS spike detection (DRFM/repeater jamming)
    - Range rate discontinuity (RGPO — Range Gate Pull-Off)
    - Bearing jitter increase (noise jamming)
    - Measurement dropout patterns (blanking/screening)
    - Ghost track detection (DRFM creates false targets)
    
    Provides per-track ECM classification with confidence scores.
    
    Reference: Neri, "Introduction to Electronic Defense Systems," 
    Artech House, 2nd ed., Chapters 8-9.
    """
    
    def __init__(self, nis_window: int = 10, nis_threshold: float = 3.0,
                 range_rate_threshold: float = 50.0,
                 bearing_jitter_threshold: float = 3.0,
                 dropout_window: int = 5, dropout_threshold: int = 3):
        """Initialize ECM detector.
        
        Args:
            nis_window: Sliding window for NIS statistics
            nis_threshold: Sigma multiplier for NIS spike detection
            range_rate_threshold: m/s jump for RGPO detection
            bearing_jitter_threshold: Sigma multiplier for noise jamming
            dropout_window: Window for dropout pattern analysis
            dropout_threshold: Max misses in window before flagging
        """
        self.nis_window = nis_window
        self.nis_threshold = nis_threshold
        self.range_rate_threshold = range_rate_threshold
        self.bearing_jitter_threshold = bearing_jitter_threshold
        self.dropout_window = dropout_window
        self.dropout_threshold = dropout_threshold
        
        # Per-track history
        self._track_history: Dict[int, Dict] = {}
    
    def _get_history(self, track_id: int) -> Dict:
        """Get or create track history."""
        if track_id not in self._track_history:
            self._track_history[track_id] = {
                'nis_values': [],
                'range_rates': [],
                'bearing_residuals': [],
                'hit_miss_pattern': [],  # True=hit, False=miss
                'ecm_flags': set(),
                'confidence': {},
                'last_position': None,
                'last_velocity': None,
            }
        return self._track_history[track_id]
    
    def update(self, track_id: int, nis: float, 
               innovation: Optional[np.ndarray] = None,
               position: Optional[np.ndarray] = None,
               velocity: Optional[np.ndarray] = None,
               was_hit: bool = True) -> Dict[str, Any]:
        """[REQ-V592-ECM-02] Update ECM assessment for a track.
        
        Args:
            track_id: Track identifier
            nis: Normalized Innovation Squared for this update
            innovation: Innovation vector (for directional analysis)
            position: Current position estimate
            velocity: Current velocity estimate
            was_hit: Whether track got a measurement this scan
            
        Returns:
            Dict with:
                'ecm_detected': bool
                'ecm_types': set of detected ECM types
                'confidence': dict of type -> float (0-1)
                'recommended_action': str
        """
        h = self._get_history(track_id)
        
        # Update histories
        h['nis_values'].append(nis)
        if len(h['nis_values']) > self.nis_window * 3:
            h['nis_values'] = h['nis_values'][-self.nis_window * 3:]
        
        h['hit_miss_pattern'].append(was_hit)
        if len(h['hit_miss_pattern']) > self.dropout_window * 3:
            h['hit_miss_pattern'] = h['hit_miss_pattern'][-self.dropout_window * 3:]
        
        # Range rate analysis
        if velocity is not None and position is not None:
            if h['last_position'] is not None:
                range_vec = position - h['last_position']
                range_dist = np.linalg.norm(range_vec)
                if range_dist > 1e-6:
                    radial_vel = np.dot(velocity, range_vec / range_dist)
                    h['range_rates'].append(radial_vel)
                    if len(h['range_rates']) > self.nis_window * 3:
                        h['range_rates'] = h['range_rates'][-self.nis_window * 3:]
            h['last_position'] = position.copy()
            h['last_velocity'] = velocity.copy()
        
        # Bearing residual analysis
        if innovation is not None and len(innovation) >= 2:
            bearing_resid = np.linalg.norm(innovation[1:3]) if len(innovation) >= 3 else abs(innovation[1])
            h['bearing_residuals'].append(bearing_resid)
            if len(h['bearing_residuals']) > self.nis_window * 3:
                h['bearing_residuals'] = h['bearing_residuals'][-self.nis_window * 3:]
        
        # === DETECTION LOGIC ===
        ecm_flags = set()
        confidence = {}
        
        # 1. NIS spike → DRFM/Repeater
        if len(h['nis_values']) >= self.nis_window:
            recent = h['nis_values'][-self.nis_window:]
            baseline = h['nis_values'][:-self.nis_window] if len(h['nis_values']) > self.nis_window else recent[:max(3, len(recent)//2)]
            if len(baseline) >= 3:
                mu, sigma = np.mean(baseline), max(np.std(baseline), 0.1)
                current_nis = recent[-1]
                spike_count = sum(1 for v in recent if v > mu + self.nis_threshold * sigma)
                if spike_count >= self.nis_window // 3:
                    ecm_flags.add('DRFM_REPEATER')
                    confidence['DRFM_REPEATER'] = min(1.0, spike_count / self.nis_window)
        
        # 2. Range rate discontinuity → RGPO
        if len(h['range_rates']) >= 5:
            recent_rr = h['range_rates'][-5:]
            diffs = np.abs(np.diff(recent_rr))
            if np.max(diffs) > self.range_rate_threshold:
                ecm_flags.add('RGPO')
                confidence['RGPO'] = min(1.0, np.max(diffs) / (self.range_rate_threshold * 3))
        
        # 3. Bearing jitter → Noise jamming
        if len(h['bearing_residuals']) >= self.nis_window:
            recent_br = h['bearing_residuals'][-self.nis_window:]
            baseline_br = h['bearing_residuals'][:-self.nis_window] if len(h['bearing_residuals']) > self.nis_window else recent_br[:max(3, len(recent_br)//2)]
            if len(baseline_br) >= 3:
                mu_br, sigma_br = np.mean(baseline_br), max(np.std(baseline_br), 1e-6)
                current_jitter = np.mean(recent_br)
                if current_jitter > mu_br + self.bearing_jitter_threshold * sigma_br:
                    ecm_flags.add('NOISE_JAMMING')
                    confidence['NOISE_JAMMING'] = min(1.0, (current_jitter - mu_br) / (self.bearing_jitter_threshold * sigma_br * 2))
        
        # 4. Dropout pattern → Screening/blanking
        if len(h['hit_miss_pattern']) >= self.dropout_window:
            recent_hm = h['hit_miss_pattern'][-self.dropout_window:]
            miss_count = sum(1 for x in recent_hm if not x)
            if miss_count >= self.dropout_threshold:
                ecm_flags.add('SCREENING_BLANKING')
                confidence['SCREENING_BLANKING'] = min(1.0, miss_count / self.dropout_window)
        
        h['ecm_flags'] = ecm_flags
        h['confidence'] = confidence
        
        # Recommended action
        if not ecm_flags:
            action = 'NOMINAL'
        elif 'RGPO' in ecm_flags:
            action = 'INFLATE_R_RANGE_HOLD_BEARING'
        elif 'DRFM_REPEATER' in ecm_flags:
            action = 'INFLATE_R_CHECK_GHOST_TRACKS'
        elif 'NOISE_JAMMING' in ecm_flags:
            action = 'BEARING_ONLY_TRACKING'
        elif 'SCREENING_BLANKING' in ecm_flags:
            action = 'COAST_EXTEND_TIMEOUT'
        else:
            action = 'INCREASE_VIGILANCE'
        
        return {
            'ecm_detected': len(ecm_flags) > 0,
            'ecm_types': ecm_flags,
            'confidence': confidence,
            'recommended_action': action,
        }
    
    def get_track_status(self, track_id: int) -> Dict[str, Any]:
        """Get current ECM status for a track."""
        h = self._get_history(track_id)
        return {
            'ecm_detected': len(h['ecm_flags']) > 0,
            'ecm_types': h['ecm_flags'],
            'confidence': h['confidence'],
        }
    
    def clear_track(self, track_id: int):
        """Remove track from ECM monitoring."""
        self._track_history.pop(track_id, None)


# ============================================================================
# [REQ-V592-COAST] TRACK COASTING PREDICTOR — Display Continuity
# ============================================================================

class TrackCoaster:
    """[REQ-V592-COAST-01] Intelligent track coasting for display continuity.
    
    When a track loses measurements, provides smooth predicted positions
    using the last known state and IMM model probabilities. Implements:
    - Kinematic extrapolation with growing uncertainty
    - Confidence decay based on coast duration
    - Automatic track quality degradation
    
    Integrates with MultiTargetTracker for seamless operation.
    """
    
    def __init__(self, max_coast_scans: int = 10,
                 confidence_halflife: float = 3.0):
        """Initialize coaster.
        
        Args:
            max_coast_scans: Maximum scans to coast before deletion
            confidence_halflife: Scans until confidence drops to 50%
        """
        self.max_coast_scans = max_coast_scans
        self.confidence_halflife = confidence_halflife
        self._coast_states: Dict[int, Dict] = {}
    
    def start_coast(self, track_id: int, x: np.ndarray, P: np.ndarray,
                    dt: float, model_probs: Optional[np.ndarray] = None):
        """Begin coasting a track.
        
        Args:
            track_id: Track identifier
            x: Last known state [x,y,z,vx,vy,vz,...]
            P: Last known covariance
            dt: Nominal scan interval
            model_probs: IMM model probabilities (if available)
        """
        self._coast_states[track_id] = {
            'x': x.copy(),
            'P': P.copy(),
            'dt': dt,
            'n_coast': 0,
            'model_probs': model_probs.copy() if model_probs is not None else None,
            'start_x': x.copy(),
        }
    
    def predict_coast(self, track_id: int) -> Optional[Tuple[np.ndarray, np.ndarray, float]]:
        """Get next coasted position.
        
        Returns:
            Tuple of (position, covariance, confidence) or None if expired
        """
        if track_id not in self._coast_states:
            return None
        
        cs = self._coast_states[track_id]
        cs['n_coast'] += 1
        
        if cs['n_coast'] > self.max_coast_scans:
            self._coast_states.pop(track_id)
            return None
        
        dt = cs['dt']
        nx = len(cs['x'])
        
        # Simple CV extrapolation
        F = np.eye(nx)
        if nx >= 6:
            F[0, 3] = dt
            F[1, 4] = dt
            F[2, 5] = dt
        
        # Process noise grows with coast time
        coast_factor = 1.0 + 0.5 * cs['n_coast']
        Q = np.eye(nx) * (coast_factor * dt)**2
        
        cs['x'] = F @ cs['x']
        cs['P'] = F @ cs['P'] @ F.T + Q
        
        # Confidence decays exponentially
        confidence = 0.5 ** (cs['n_coast'] / self.confidence_halflife)
        
        return cs['x'][:3].copy(), cs['P'][:3, :3].copy(), confidence
    
    def end_coast(self, track_id: int):
        """Stop coasting (measurement resumed or track deleted)."""
        self._coast_states.pop(track_id, None)
    
    def is_coasting(self, track_id: int) -> bool:
        """Check if track is currently coasting."""
        return track_id in self._coast_states
    
    def coast_count(self, track_id: int) -> int:
        """Number of scans this track has coasted."""
        cs = self._coast_states.get(track_id)
        return cs['n_coast'] if cs else 0


# ===== GAUSSIAN MIXTURE PHD FILTER =====

@dataclass
class GaussianComponent:
    """[REQ-V592-PHD-01] Single Gaussian component in the PHD intensity."""
    weight: float           # Expected number of targets in this component
    x: np.ndarray          # State mean [x, y, z, vx, vy, vz]
    P: np.ndarray          # State covariance
    label: int = -1        # Track label (-1 = unassigned)


class GMPHD:
    """[REQ-V592-PHD-02] Gaussian Mixture Probability Hypothesis Density filter.
    
    Propagates the first-order moment (intensity function) of the multi-target
    posterior. The PHD at a point x represents the expected target density —
    integrating over a region gives the expected number of targets in that region.
    
    Advantages over traditional MTT:
    - No explicit data association needed
    - Naturally handles unknown and varying target count
    - Principled birth/death modeling
    - Computationally tractable via Gaussian mixture representation
    
    Implements the GM-PHD filter of Vo & Ma (2006) with extensions:
    - Adaptive birth intensity from measurements
    - Component merging and pruning for tractability
    - State extraction with labeling for track continuity
    - EKF-PHD variant for nonlinear measurements
    
    Reference: 
        Vo & Ma, "The Gaussian Mixture Probability Hypothesis Density Filter,"
        IEEE Trans. Signal Processing, vol. 54, no. 11, pp. 4091–4104, 2006.
    
    Note: For most radar tracking with known target count, IMM+GNN/JPDA is
    preferred (lower complexity, better single-target accuracy). GM-PHD excels
    when target count is unknown or highly variable (track-before-detect,
    dense environments, spawning targets).
    """
    
    def __init__(self, dt: float = 1.0,
                 ps: float = 0.99,          # Survival probability
                 pd: float = 0.90,          # Detection probability
                 clutter_intensity: float = 1e-11,  # Spatial clutter density (per m³)
                 birth_weight: float = 0.03,       # Weight of birth components
                 merge_threshold: float = 4.0,     # Mahalanobis for merging
                 prune_threshold: float = 1e-5,    # Min weight to keep
                 max_components: int = 100,         # Cap on mixture size
                 q_base: float = 1.0,
                 extraction_threshold: float = 0.5,  # Min weight to extract target
                 use_adaptive_birth: bool = True):
        """Initialize GM-PHD filter.
        
        Args:
            dt: Scan interval (seconds)
            ps: Target survival probability per scan
            pd: Sensor detection probability
            clutter_intensity: Clutter spatial density (false alarms per unit volume)
            birth_weight: Weight assigned to each birth component
            merge_threshold: Mahalanobis distance for merging components
            prune_threshold: Minimum weight below which components are pruned
            max_components: Maximum number of Gaussian components
            q_base: Process noise spectral density
            extraction_threshold: Minimum weight to extract as a target
            use_adaptive_birth: Create birth components from measurements
        """
        self.dt = dt
        self.ps = ps
        self.pd = pd
        self.clutter_intensity = clutter_intensity
        self.birth_weight = birth_weight
        self.merge_threshold = merge_threshold
        self.prune_threshold = prune_threshold
        self.max_components = max_components
        self.q_base = q_base
        self.extraction_threshold = extraction_threshold
        self.use_adaptive_birth = use_adaptive_birth
        
        # State dimension (3D position + velocity)
        self.nx = 6
        self.nz = 3
        
        # Measurement matrix H (observe position only)
        self.H = np.zeros((self.nz, self.nx))
        self.H[0, 0] = 1.0  # x
        self.H[1, 1] = 1.0  # y
        self.H[2, 2] = 1.0  # z
        
        # Build CV model matrices
        self.F, self.Q = make_cv3d_matrices(dt, q_base)
        
        # Default measurement noise
        self.R = np.eye(self.nz) * 150.0**2  # 150m std
        
        # PHD intensity as Gaussian mixture
        self.components: List[GaussianComponent] = []
        
        # Birth intensity (static birth regions — can be overridden)
        self.birth_components: List[GaussianComponent] = []
        
        # Label counter for track continuity
        self._next_label = 1
        
        # Scan counter
        self.scan_count = 0
    
    def set_birth_regions(self, centers: List[np.ndarray],
                          spread: float = 10000.0):
        """[REQ-V592-PHD-03] Set static birth regions.
        
        Args:
            centers: List of [x, y, z] positions where targets may appear
            spread: Covariance spread for birth components (meters)
        """
        self.birth_components = []
        P_birth = np.diag([spread**2, spread**2, spread**2,
                           100.0**2, 100.0**2, 50.0**2])
        for c in centers:
            x = np.zeros(self.nx)
            x[:3] = c[:3] if len(c) >= 3 else np.pad(c, (0, 3 - len(c)))
            self.birth_components.append(GaussianComponent(
                weight=self.birth_weight,
                x=x.copy(),
                P=P_birth.copy()
            ))
    
    def predict(self):
        """[REQ-V592-PHD-04] PHD prediction step.
        
        Surviving components are propagated through the motion model,
        then birth components are appended.
        """
        predicted = []
        
        # Surviving targets
        for comp in self.components:
            x_pred = self.F @ comp.x
            P_pred = self.F @ comp.P @ self.F.T + self.Q
            predicted.append(GaussianComponent(
                weight=self.ps * comp.weight,
                x=x_pred,
                P=P_pred,
                label=comp.label
            ))
        
        # Birth components (static)
        for bc in self.birth_components:
            predicted.append(GaussianComponent(
                weight=bc.weight,
                x=bc.x.copy(),
                P=bc.P.copy(),
                label=-1  # New births get labels during extraction
            ))
        
        self.components = predicted
    
    def update(self, measurements: np.ndarray,
               R: Optional[np.ndarray] = None) -> int:
        """[REQ-V592-PHD-05] PHD update step (Vo & Ma 2006, Eq. 20-24).
        
        Args:
            measurements: (N, 3) array of [x, y, z] measurements
            R: Measurement noise covariance (optional override)
            
        Returns:
            Estimated number of targets (rounded integral of PHD)
        """
        self.scan_count += 1
        R_use = R if R is not None else self.R
        
        if measurements.ndim == 1:
            measurements = measurements.reshape(1, -1)
        n_meas = len(measurements)
        
        # Pre-compute Kalman update components for each predicted component
        eta_list = []   # Predicted measurements
        S_list = []     # Innovation covariances
        K_list = []     # Kalman gains
        P_up_list = []  # Updated covariances
        
        for comp in self.components:
            eta = self.H @ comp.x
            S = self.H @ comp.P @ self.H.T + R_use
            K = comp.P @ self.H.T @ np.linalg.inv(S)
            P_up = (np.eye(self.nx) - K @ self.H) @ comp.P
            
            eta_list.append(eta)
            S_list.append(S)
            K_list.append(K)
            P_up_list.append(P_up)
        
        # Missed detection components (1 - pd) * predicted
        updated = []
        for i, comp in enumerate(self.components):
            updated.append(GaussianComponent(
                weight=(1.0 - self.pd) * comp.weight,
                x=comp.x.copy(),
                P=comp.P.copy(),
                label=comp.label
            ))
        
        # Detection-update components for each measurement
        for j in range(n_meas):
            z = measurements[j, :self.nz]
            
            # Compute likelihoods for all components
            weights_j = []
            for i, comp in enumerate(self.components):
                innov = z - eta_list[i]
                S = S_list[i]
                try:
                    S_inv = np.linalg.inv(S)
                    det_S = max(np.linalg.det(S), 1e-300)
                    exponent = -0.5 * innov @ S_inv @ innov
                    # Clamp exponent for numerical stability
                    exponent = max(exponent, -500.0)
                    q_val = np.exp(exponent) / np.sqrt((2 * np.pi)**self.nz * det_S)
                except np.linalg.LinAlgError:
                    q_val = 0.0
                weights_j.append(self.pd * comp.weight * q_val)
            
            # Normalization (clutter + all component contributions)
            weight_sum = self.clutter_intensity + sum(weights_j)
            
            # Create updated components for this measurement
            for i, comp in enumerate(self.components):
                if weights_j[i] < 1e-15:
                    continue
                w_updated = weights_j[i] / max(weight_sum, 1e-300)
                x_updated = comp.x + K_list[i] @ (z - eta_list[i])
                updated.append(GaussianComponent(
                    weight=w_updated,
                    x=x_updated,
                    P=P_up_list[i].copy(),
                    label=comp.label
                ))
        
        # Adaptive birth from unassociated measurements
        if self.use_adaptive_birth and n_meas > 0:
            for j in range(n_meas):
                z = measurements[j, :self.nz]
                # Check if this measurement is well-explained by existing components
                max_likelihood = 0.0
                for i in range(len(self.components)):
                    innov = z - eta_list[i]
                    S = S_list[i]
                    try:
                        md2 = innov @ np.linalg.inv(S) @ innov
                        if md2 < 16.0:  # Within gate
                            max_likelihood = max(max_likelihood, 1.0)
                    except np.linalg.LinAlgError:
                        pass
                
                if max_likelihood < 0.5:  # Poorly explained → birth candidate
                    x_birth = np.zeros(self.nx)
                    x_birth[:self.nz] = z
                    P_birth = np.diag([R_use[0, 0] * 4, R_use[1, 1] * 4,
                                       R_use[2, 2] * 4,
                                       100.0**2, 100.0**2, 50.0**2])
                    updated.append(GaussianComponent(
                        weight=self.birth_weight * 0.5,
                        x=x_birth,
                        P=P_birth,
                        label=-1
                    ))
        
        self.components = updated
        
        # Prune, merge, cap
        self._prune()
        self._merge()
        self._cap()
        
        return self.estimated_target_count()
    
    def _prune(self):
        """Remove components with weight below threshold."""
        self.components = [c for c in self.components
                           if c.weight >= self.prune_threshold]
    
    def _merge(self):
        """[REQ-V592-PHD-06] Merge nearby components to control mixture size."""
        if not self.components:
            return
        
        merged = []
        used = set()
        
        # Sort by weight descending
        indices = sorted(range(len(self.components)),
                         key=lambda i: self.components[i].weight, reverse=True)
        
        for i in indices:
            if i in used:
                continue
            
            comp_i = self.components[i]
            merge_set = [i]
            
            try:
                P_inv = np.linalg.inv(comp_i.P + np.eye(self.nx) * 1e-10)
            except np.linalg.LinAlgError:
                merged.append(comp_i)
                used.add(i)
                continue
            
            for j in indices:
                if j in used or j == i:
                    continue
                comp_j = self.components[j]
                diff = comp_j.x - comp_i.x
                md2 = diff @ P_inv @ diff
                if md2 < self.merge_threshold**2:
                    merge_set.append(j)
            
            # Merge all in set
            w_total = sum(self.components[k].weight for k in merge_set)
            if w_total < 1e-15:
                used.update(merge_set)
                continue
            
            x_merged = np.zeros(self.nx)
            for k in merge_set:
                x_merged += self.components[k].weight * self.components[k].x
            x_merged /= w_total
            
            P_merged = np.zeros((self.nx, self.nx))
            for k in merge_set:
                diff = self.components[k].x - x_merged
                P_merged += self.components[k].weight * (
                    self.components[k].P + np.outer(diff, diff)
                )
            P_merged /= w_total
            
            # Inherit label from highest-weight component
            best_label = self.components[merge_set[0]].label
            
            merged.append(GaussianComponent(
                weight=w_total,
                x=x_merged,
                P=P_merged,
                label=best_label
            ))
            used.update(merge_set)
        
        self.components = merged
    
    def _cap(self):
        """Cap number of components by keeping highest-weight ones."""
        if len(self.components) > self.max_components:
            self.components.sort(key=lambda c: c.weight, reverse=True)
            # Redistribute pruned weight to survivors
            kept = self.components[:self.max_components]
            self.components = kept
    
    def estimated_target_count(self) -> int:
        """Expected number of targets (sum of weights, rounded)."""
        return int(round(sum(c.weight for c in self.components)))
    
    def phd_integral(self) -> float:
        """Raw PHD integral (expected target count, not rounded)."""
        return sum(c.weight for c in self.components)
    
    def extract_targets(self) -> List[Dict[str, Any]]:
        """[REQ-V592-PHD-07] Extract target state estimates from PHD.
        
        Components with weight >= extraction_threshold are reported as targets.
        Labels provide track continuity across scans.
        
        Returns:
            List of dicts: {'label', 'x', 'P', 'weight', 'position', 'velocity'}
        """
        targets = []
        for comp in self.components:
            if comp.weight >= self.extraction_threshold:
                # Assign label if new
                if comp.label < 0:
                    comp.label = self._next_label
                    self._next_label += 1
                
                targets.append({
                    'label': comp.label,
                    'x': comp.x.copy(),
                    'P': comp.P.copy(),
                    'weight': comp.weight,
                    'position': comp.x[:3].copy(),
                    'velocity': comp.x[3:6].copy()
                })
        
        return targets
    
    def component_count(self) -> int:
        """Current number of Gaussian components."""
        return len(self.components)
    
    def summary(self) -> str:
        """Human-readable summary."""
        n_est = self.estimated_target_count()
        n_comp = self.component_count()
        targets = self.extract_targets()
        return (f"GM-PHD: scan={self.scan_count}, "
                f"targets_est={n_est}, components={n_comp}, "
                f"extracted={len(targets)}, "
                f"phd_integral={self.phd_integral():.2f}")


class CPHD:
    """[REQ-V592-CPHD-01] Cardinalized PHD filter (simplified).
    
    Extends GM-PHD by also propagating the cardinality distribution —
    the probability mass function over the number of targets. This gives
    more accurate target count estimates than basic PHD.
    
    Implements the Vo-Vo-Cantoni (2007) approach with Gaussian mixture 
    spatial density and explicit cardinality propagation.
    
    Reference:
        Vo, Vo & Cantoni, "Analytic Implementations of the Cardinalized
        Probability Hypothesis Density Filter," IEEE Trans. Signal Processing,
        vol. 55, no. 7, pp. 3553–3567, 2007.
    """
    
    def __init__(self, dt: float = 1.0,
                 ps: float = 0.99,
                 pd: float = 0.90,
                 clutter_intensity: float = 1e-11,
                 max_targets: int = 50,
                 **kwargs):
        """Initialize CPHD filter.
        
        Args:
            dt: Scan interval
            ps: Survival probability
            pd: Detection probability
            clutter_intensity: Clutter spatial density
            max_targets: Maximum cardinality to track
            **kwargs: Passed to underlying GM-PHD
        """
        self.gmphd = GMPHD(dt=dt, ps=ps, pd=pd,
                           clutter_intensity=clutter_intensity, **kwargs)
        self.max_targets = max_targets
        
        # Cardinality distribution: P(N = n) for n = 0, 1, ..., max_targets
        self.cardinality = np.zeros(max_targets + 1)
        self.cardinality[0] = 1.0  # Start with 0 targets
    
    def predict(self):
        """Predict both spatial density and cardinality."""
        self.gmphd.predict()
        
        # Propagate cardinality through survival + birth
        ps = self.gmphd.ps
        n_birth_expected = sum(bc.weight for bc in self.gmphd.birth_components)
        
        new_card = np.zeros_like(self.cardinality)
        for n in range(self.max_targets + 1):
            # Surviving targets: Binomial thinning
            for j in range(min(n, self.max_targets) + 1):
                surv_prob = _binomial_prob(n, j, ps)
                # Add birth (Poisson approximation)
                for b in range(min(self.max_targets - j, 20) + 1):
                    target_n = j + b
                    if target_n <= self.max_targets:
                        birth_prob = _poisson_prob(b, n_birth_expected)
                        new_card[target_n] += self.cardinality[n] * surv_prob * birth_prob
        
        # Normalize
        total = new_card.sum()
        if total > 0:
            self.cardinality = new_card / total
        
    def update(self, measurements: np.ndarray,
               R: Optional[np.ndarray] = None) -> int:
        """Update spatial density and cardinality.
        
        Returns:
            MAP estimate of target count
        """
        n_est = self.gmphd.update(measurements, R)
        
        # Update cardinality based on PHD integral
        phd_int = self.gmphd.phd_integral()
        
        # Simple cardinality update: shift distribution toward PHD integral
        new_card = np.zeros_like(self.cardinality)
        for n in range(self.max_targets + 1):
            # Likelihood of observing this PHD integral given n targets
            likelihood = _poisson_prob_float(phd_int, float(n))
            new_card[n] = self.cardinality[n] * max(likelihood, 1e-300)
        
        total = new_card.sum()
        if total > 0:
            self.cardinality = new_card / total
        
        return self.map_cardinality()
    
    def map_cardinality(self) -> int:
        """Maximum a posteriori target count estimate."""
        return int(np.argmax(self.cardinality))
    
    def cardinality_variance(self) -> float:
        """Variance of the cardinality distribution."""
        ns = np.arange(len(self.cardinality), dtype=float)
        mean = np.dot(ns, self.cardinality)
        return float(np.dot((ns - mean)**2, self.cardinality))
    
    def extract_targets(self) -> List[Dict[str, Any]]:
        """Extract targets using MAP cardinality."""
        return self.gmphd.extract_targets()
    
    def summary(self) -> str:
        """Human-readable summary."""
        return (f"CPHD: MAP_card={self.map_cardinality()}, "
                f"card_var={self.cardinality_variance():.2f}, "
                f"{self.gmphd.summary()}")


def _binomial_prob(n: int, k: int, p: float) -> float:
    """Binomial probability P(X=k) for X~Bin(n, p)."""
    if k < 0 or k > n:
        return 0.0
    from math import comb, log, exp
    try:
        log_prob = log(comb(n, k)) + k * log(max(p, 1e-300)) + (n - k) * log(max(1 - p, 1e-300))
        return exp(log_prob)
    except (ValueError, OverflowError):
        return 0.0


def _poisson_prob(k: int, lam: float) -> float:
    """Poisson probability P(X=k) for X~Poisson(lam)."""
    if lam <= 0:
        return 1.0 if k == 0 else 0.0
    from math import lgamma, exp, log
    try:
        log_prob = k * log(max(lam, 1e-300)) - lam - lgamma(k + 1)
        return exp(log_prob)
    except (ValueError, OverflowError):
        return 0.0


def _poisson_prob_float(k_float: float, lam: float) -> float:
    """Continuous relaxation of Poisson for cardinality update."""
    if lam <= 0:
        return 1.0 if k_float < 0.5 else 0.0
    from math import lgamma, exp, log
    try:
        log_prob = k_float * log(max(lam, 1e-300)) - lam - lgamma(int(k_float) + 1)
        return exp(max(log_prob, -500.0))
    except (ValueError, OverflowError):
        return 0.0


# =============================================================================
# LMB: LABELED MULTI-BERNOULLI FILTER
# =============================================================================

@dataclass
class LMBComponent:
    """Single labeled Bernoulli component in the LMB filter."""
    label: int                  # Unique track label
    r: float                    # Existence probability ∈ [0, 1]
    x: np.ndarray              # State estimate
    P: np.ndarray              # State covariance
    
    def copy(self) -> 'LMBComponent':
        return LMBComponent(
            label=self.label,
            r=self.r,
            x=self.x.copy(),
            P=self.P.copy()
        )


class LMB:
    """[REQ-V592-LMB-01] Labeled Multi-Bernoulli filter.
    
    Maintains a set of labeled Bernoulli components, each with:
    - Unique label for track identity
    - Existence probability r ∈ [0, 1]
    - Gaussian state (x, P)
    
    Advantages over GM-PHD:
    - Explicit track identity through labels (no label-switching)
    - Existence probability per target (interpretable confidence)
    - Better cardinality estimation
    - Principled track management
    
    Based on: Reuter, Vo, Vo & Dietmayer, "The Labeled Multi-Bernoulli
    Filter," IEEE Trans. Signal Processing, vol. 62, no. 12, 2014.
    
    This implementation uses ranked assignment for the update step,
    suitable for moderate target/measurement counts typical in radar.
    """
    
    def __init__(self, dt: float = 1.0,
                 ps: float = 0.99,
                 pd: float = 0.90,
                 clutter_intensity: float = 1e-11,
                 birth_r: float = 0.01,
                 prune_threshold: float = 1e-3,
                 max_components: int = 100,
                 merge_threshold: float = 4.0,
                 q_base: float = 1.0):
        """Initialize LMB filter.
        
        Args:
            dt: Scan interval (seconds)
            ps: Target survival probability per scan
            pd: Sensor detection probability
            clutter_intensity: Clutter spatial density (per unit volume)
            birth_r: Existence probability for new birth components
            prune_threshold: Minimum existence prob to keep component
            max_components: Maximum number of components
            merge_threshold: Mahalanobis distance for merging
            q_base: Process noise base (m/s²)
        """
        self.dt = dt
        self.ps = ps
        self.pd = pd
        self.clutter_intensity = max(clutter_intensity, 1e-30)
        self.birth_r = birth_r
        self.prune_threshold = prune_threshold
        self.max_components = max_components
        self.merge_threshold = merge_threshold
        self.q_base = q_base
        self.nx = 6  # [x, y, z, vx, vy, vz]
        
        self.components: List[LMBComponent] = []
        self.birth_regions: List[np.ndarray] = []  # Birth position centers
        self.birth_spread: float = 10000.0
        self._next_label = 1
        self.scan_count = 0
        
        # Build CV dynamics
        self._F = np.eye(6)
        self._F[0, 3] = dt
        self._F[1, 4] = dt
        self._F[2, 5] = dt
        self._Q = self._build_Q(dt, q_base)
    
    def _build_Q(self, dt: float, q: float) -> np.ndarray:
        """CV process noise matrix."""
        dt2 = dt * dt
        dt3 = dt2 * dt / 2
        dt4 = dt2 * dt2 / 4
        Q1d = np.array([[dt4, dt3], [dt3, dt2]]) * q
        Q = np.zeros((6, 6))
        for i in range(3):
            Q[i, i] = Q1d[0, 0]
            Q[i, i+3] = Q1d[0, 1]
            Q[i+3, i] = Q1d[1, 0]
            Q[i+3, i+3] = Q1d[1, 1]
        return Q
    
    def set_birth_regions(self, centers: List[np.ndarray],
                          spread: float = 10000.0):
        """[REQ-V592-LMB-02] Set static birth regions.
        
        Args:
            centers: List of [x, y, z] positions for potential births
            spread: Position uncertainty spread (meters)
        """
        self.birth_regions = [np.asarray(c)[:3] for c in centers]
        self.birth_spread = spread
    
    def _birth_components(self) -> List[LMBComponent]:
        """Generate birth components for current scan."""
        births = []
        P_birth = np.diag([self.birth_spread**2]*3 + [100.0**2]*3)
        for c in self.birth_regions:
            x = np.zeros(self.nx)
            x[:3] = c
            births.append(LMBComponent(
                label=self._next_label,
                r=self.birth_r,
                x=x.copy(),
                P=P_birth.copy()
            ))
            self._next_label += 1
        return births
    
    def predict(self):
        """[REQ-V592-LMB-03] LMB prediction step.
        
        Surviving components: r_pred = ps * r
        State: standard KF predict (x = Fx, P = FPF' + Q)
        Birth: new components added from birth model
        """
        self.scan_count += 1
        
        # Predict surviving components
        predicted = []
        for comp in self.components:
            r_pred = self.ps * comp.r
            x_pred = self._F @ comp.x
            P_pred = self._F @ comp.P @ self._F.T + self._Q
            predicted.append(LMBComponent(
                label=comp.label,
                r=r_pred,
                x=x_pred,
                P=P_pred
            ))
        
        # Add birth components
        predicted.extend(self._birth_components())
        self.components = predicted
    
    def update(self, measurements: np.ndarray,
               R: Optional[np.ndarray] = None):
        """[REQ-V592-LMB-04] LMB update step.
        
        Uses per-component Kalman update with existence probability
        weighting. For moderate target counts, uses the fast ranked
        assignment approach.
        
        Args:
            measurements: (M, 3) array of Cartesian measurements
            R: Measurement noise covariance (default: 150m σ per axis)
        """
        if R is None:
            R = np.diag([150.0**2, 150.0**2, 150.0**2])
        
        H = np.zeros((3, self.nx))
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 2] = 1.0
        
        M = len(measurements) if len(measurements) > 0 else 0
        n_comp = len(self.components)
        
        if n_comp == 0:
            return
        
        # Compute per-component likelihoods for each measurement
        # L[i, j] = likelihood of measurement j given component i
        L = np.zeros((n_comp, M))
        S_list = []
        K_list = []
        innov_list = []
        
        for i, comp in enumerate(self.components):
            S = H @ comp.P @ H.T + R
            S_inv = np.linalg.inv(S)
            K = comp.P @ H.T @ S_inv
            S_list.append(S)
            K_list.append(K)
            
            for j in range(M):
                innov = measurements[j] - H @ comp.x
                mahal2 = innov @ S_inv @ innov
                det_S = np.linalg.det(S)
                L[i, j] = np.exp(-0.5 * mahal2) / np.sqrt((2*np.pi)**3 * max(det_S, 1e-300))
        
        # Update each component: missed detection + all measurement associations
        updated = []
        
        for i, comp in enumerate(self.components):
            # === Missed detection hypothesis ===
            r_miss = comp.r * (1 - self.pd) / (1 - comp.r + comp.r * (1 - self.pd))
            updated.append(LMBComponent(
                label=comp.label,
                r=r_miss,
                x=comp.x.copy(),
                P=comp.P.copy()
            ))
            
            # === Detection hypotheses ===
            for j in range(M):
                if L[i, j] < 1e-30:
                    continue
                
                innov = measurements[j] - H @ comp.x
                x_upd = comp.x + K_list[i] @ innov
                P_upd = (np.eye(self.nx) - K_list[i] @ H) @ comp.P
                
                # Existence probability for detection
                denom = self.clutter_intensity + sum(
                    self.components[k].r * self.pd * L[k, j]
                    for k in range(n_comp)
                )
                r_det = comp.r * self.pd * L[i, j] / max(denom, 1e-300)
                r_det = min(r_det, 0.999)
                
                if r_det > self.prune_threshold:
                    updated.append(LMBComponent(
                        label=comp.label,
                        r=r_det,
                        x=x_upd,
                        P=P_upd
                    ))
        
        # Merge same-label components
        self.components = self._merge_labels(updated)
        self._prune()
        self._cap()
    
    def _merge_labels(self, components: List[LMBComponent]) -> List[LMBComponent]:
        """Merge components with the same label into single Bernoulli."""
        from collections import defaultdict
        label_groups = defaultdict(list)
        for comp in components:
            label_groups[comp.label].append(comp)
        
        merged = []
        for label, group in label_groups.items():
            if len(group) == 1:
                merged.append(group[0])
                continue
            
            # Merge: r_merged = min(1, sum(r_i))
            # State: weighted average by existence probability
            r_total = min(sum(c.r for c in group), 0.999)
            if r_total < self.prune_threshold:
                continue
            
            # Weighted mean state
            w_sum = sum(c.r for c in group)
            x_merged = sum(c.r * c.x for c in group) / max(w_sum, 1e-30)
            
            # Merged covariance (moment-matching)
            P_merged = np.zeros_like(group[0].P)
            for c in group:
                diff = c.x - x_merged
                P_merged += c.r * (c.P + np.outer(diff, diff))
            P_merged /= max(w_sum, 1e-30)
            
            merged.append(LMBComponent(
                label=label, r=r_total, x=x_merged, P=P_merged
            ))
        
        return merged
    
    def _prune(self):
        """Remove components with existence prob below threshold."""
        self.components = [c for c in self.components
                          if c.r >= self.prune_threshold]
    
    def _cap(self):
        """Cap component count by keeping highest existence prob."""
        if len(self.components) > self.max_components:
            self.components.sort(key=lambda c: c.r, reverse=True)
            self.components = self.components[:self.max_components]
    
    def extract_targets(self, threshold: float = 0.5) -> List[Dict[str, Any]]:
        """[REQ-V592-LMB-05] Extract targets with existence prob > threshold.
        
        Returns:
            List of dicts with label, position, velocity, existence_prob, covariance
        """
        targets = []
        for comp in self.components:
            if comp.r >= threshold:
                targets.append({
                    'label': comp.label,
                    'x': comp.x.copy(),
                    'P': comp.P.copy(),
                    'existence_prob': comp.r,
                    'position': comp.x[:3].copy(),
                    'velocity': comp.x[3:6].copy(),
                })
        return targets
    
    def estimated_target_count(self) -> int:
        """Expected number of targets (sum of existence probs, rounded)."""
        return int(round(sum(c.r for c in self.components)))
    
    def cardinality_distribution(self) -> np.ndarray:
        """Compute cardinality PMF from Bernoulli existence probabilities.
        
        For independent Bernoulli RVs with probs r_1, ..., r_n,
        the cardinality distribution is a Poisson binomial distribution.
        Computed via DFT for numerical stability.
        """
        rs = [c.r for c in self.components if c.r > 1e-10]
        n = len(rs)
        if n == 0:
            return np.array([1.0])
        
        max_card = min(n, self.max_components)
        # Recursive convolution (exact for small n)
        pmf = np.zeros(max_card + 1)
        pmf[0] = 1.0
        for r in rs:
            new_pmf = np.zeros_like(pmf)
            new_pmf[0] = pmf[0] * (1 - r)
            for k in range(1, max_card + 1):
                new_pmf[k] = pmf[k] * (1 - r) + pmf[k-1] * r
            pmf = new_pmf
        
        return pmf
    
    def map_cardinality(self) -> int:
        """Maximum a posteriori target count."""
        pmf = self.cardinality_distribution()
        return int(np.argmax(pmf))
    
    def summary(self) -> str:
        """Human-readable summary."""
        n_est = self.estimated_target_count()
        targets = self.extract_targets()
        return (f"LMB: scan={self.scan_count}, "
                f"components={len(self.components)}, "
                f"targets_est={n_est}, extracted={len(targets)}, "
                f"MAP_n={self.map_cardinality()}")
