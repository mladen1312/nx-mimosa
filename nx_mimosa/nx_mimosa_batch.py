"""NX-MIMOSA v5.9.4: Batch Processing API + RTS Smoother + Track Interpolation.

New capabilities for offline / post-mission analysis:
- BatchProcessor: Feed all scans at once, get smoothed results
- RTSSmoother3D: Rauch-Tung-Striebel fixed-interval smoother
- RetrodictionSmoother: Multi-pass retrodiction for fire control
- TrackInterpolator: Interpolate states to arbitrary timestamps

Author: Dr. Mladen Mešter — Nexellum d.o.o.
License: AGPL-3.0-or-later
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Tuple, Any
from enum import Enum
import time
import copy


# ---------------------------------------------------------------------------
# [REQ-BATCH-001] RTS Smoother — Rauch-Tung-Striebel fixed-interval smoother
# ---------------------------------------------------------------------------
@dataclass
class SmoothedState:
    """Result of RTS smoothing at one time step.
    
    Attributes:
        time_index: Scan index
        state: Smoothed state vector (9,)
        covariance: Smoothed covariance matrix (9,9)
        gain: Smoother gain matrix
    """
    time_index: int
    state: np.ndarray
    covariance: np.ndarray
    gain: Optional[np.ndarray] = None


class RTSSmoother3D:
    """Rauch-Tung-Striebel fixed-interval smoother for 3D tracking.
    
    Given a sequence of forward Kalman filter estimates, produces
    optimal smoothed estimates that use ALL measurements (past and future).
    
    This is the gold standard for offline / post-mission track analysis.
    The smoothed estimates have lower covariance than filtered estimates
    at every time step except the last.
    
    Mathematical basis:
        x_s(k) = x_f(k) + G(k) * [x_s(k+1) - x_p(k+1)]
        P_s(k) = P_f(k) + G(k) * [P_s(k+1) - P_p(k+1)] * G(k)^T
        G(k) = P_f(k) * F^T * P_p(k+1)^{-1}
    
    where _f = filtered, _p = predicted, _s = smoothed.
    
    Reference:
        Rauch, Tung, Striebel (1965). Maximum likelihood estimates of
        linear dynamic systems. AIAA Journal, 3(8), 1445-1450.
    
    Example::
    
        smoother = RTSSmoother3D()
        smoothed = smoother.smooth(
            filtered_states, filtered_covs,
            predicted_states, predicted_covs,
            F_matrices
        )
    """
    # [REQ-BATCH-002]
    
    def smooth(
        self,
        filtered_states: List[np.ndarray],
        filtered_covs: List[np.ndarray],
        predicted_states: List[np.ndarray],
        predicted_covs: List[np.ndarray],
        transition_matrices: List[np.ndarray],
    ) -> List[SmoothedState]:
        """Run RTS backward pass.
        
        Args:
            filtered_states: Forward KF state estimates [x_f(0), ..., x_f(N)]
            filtered_covs: Forward KF covariances [P_f(0), ..., P_f(N)]
            predicted_states: Forward KF predictions [x_p(1), ..., x_p(N)]
            predicted_covs: Forward KF predicted covariances [P_p(1), ..., P_p(N)]
            transition_matrices: State transition matrices [F(0), ..., F(N-1)]
            
        Returns:
            List of SmoothedState from time 0 to N.
        """
        N = len(filtered_states)
        if N == 0:
            return []
        
        # Initialize with last filtered estimate
        xs = [None] * N
        Ps = [None] * N
        Gs = [None] * N
        
        xs[N-1] = filtered_states[N-1].copy()
        Ps[N-1] = filtered_covs[N-1].copy()
        
        # Backward pass
        for k in range(N - 2, -1, -1):
            # Smoother gain: G(k) = P_f(k) * F^T * P_p(k+1)^{-1}
            Pp_inv = np.linalg.inv(predicted_covs[k])
            F = transition_matrices[k]
            G = filtered_covs[k] @ F.T @ Pp_inv
            Gs[k] = G
            
            # Smoothed state
            xs[k] = filtered_states[k] + G @ (xs[k+1] - predicted_states[k])
            
            # Smoothed covariance
            Ps[k] = filtered_covs[k] + G @ (Ps[k+1] - predicted_covs[k]) @ G.T
            
            # Ensure symmetry
            Ps[k] = 0.5 * (Ps[k] + Ps[k].T)
        
        return [
            SmoothedState(
                time_index=k,
                state=xs[k],
                covariance=Ps[k],
                gain=Gs[k]
            )
            for k in range(N)
        ]
    
    def smooth_imm(
        self,
        imm_states: List[List[np.ndarray]],
        imm_covs: List[List[np.ndarray]],
        imm_weights: List[np.ndarray],
        predicted_states: List[List[np.ndarray]],
        predicted_covs: List[List[np.ndarray]],
        transition_matrices: List[List[np.ndarray]],
    ) -> List[SmoothedState]:
        """Smooth IMM estimates using model-conditioned RTS.
        
        Applies RTS smoother to each IMM model independently,
        then combines using smoothed mode probabilities.
        
        Args:
            imm_states: Per-model filtered states [time][model]
            imm_covs: Per-model filtered covariances [time][model]
            imm_weights: Mode probabilities [time] -> array of weights
            predicted_states: Per-model predictions [time][model]
            predicted_covs: Per-model predicted covs [time][model]
            transition_matrices: Per-model F matrices [time][model]
            
        Returns:
            Combined smoothed states.
        """
        N = len(imm_states)
        if N == 0:
            return []
        
        n_models = len(imm_states[0])
        
        # Smooth each model independently
        model_smoothed = []
        for m in range(n_models):
            ms = self.smooth(
                [imm_states[t][m] for t in range(N)],
                [imm_covs[t][m] for t in range(N)],
                [predicted_states[t][m] for t in range(N)],
                [predicted_covs[t][m] for t in range(N)],
                [transition_matrices[t][m] for t in range(N)],
            )
            model_smoothed.append(ms)
        
        # Combine using mode probabilities
        result = []
        for k in range(N):
            w = imm_weights[k]
            x_combined = sum(w[m] * model_smoothed[m][k].state for m in range(n_models))
            P_combined = np.zeros_like(model_smoothed[0][k].covariance)
            for m in range(n_models):
                dx = model_smoothed[m][k].state - x_combined
                P_combined += w[m] * (model_smoothed[m][k].covariance + np.outer(dx, dx))
            
            result.append(SmoothedState(
                time_index=k,
                state=x_combined,
                covariance=P_combined,
            ))
        
        return result


# ---------------------------------------------------------------------------
# [REQ-BATCH-003] Track Interpolator — states at arbitrary timestamps
# ---------------------------------------------------------------------------
class TrackInterpolator:
    """Interpolate track states to arbitrary timestamps.
    
    Uses linear interpolation of state vectors with covariance
    propagation for timestamps between scans.
    
    Useful for:
    - Synchronising tracks from different sensors
    - Generating smooth display trajectories
    - Computing intercept points at exact times
    
    Example::
    
        interp = TrackInterpolator()
        state, cov = interp.interpolate(
            t_query=12.5,
            t_before=12.0, x_before=state1, P_before=cov1,
            t_after=13.0, x_after=state2, P_after=cov2
        )
    """
    # [REQ-BATCH-004]
    
    def interpolate(
        self,
        t_query: float,
        t_before: float,
        x_before: np.ndarray,
        P_before: np.ndarray,
        t_after: float,
        x_after: np.ndarray,
        P_after: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Interpolate state and covariance at t_query.
        
        Args:
            t_query: Desired timestamp
            t_before: Timestamp of earlier state
            x_before: State at t_before
            P_before: Covariance at t_before
            t_after: Timestamp of later state
            x_after: State at t_after
            P_after: Covariance at t_after
            
        Returns:
            Tuple of (interpolated_state, interpolated_covariance)
        """
        if t_after <= t_before:
            return x_before.copy(), P_before.copy()
        
        alpha = (t_query - t_before) / (t_after - t_before)
        alpha = np.clip(alpha, 0.0, 1.0)
        
        x_interp = (1.0 - alpha) * x_before + alpha * x_after
        
        # Covariance: blend with additional uncertainty for interpolation
        P_interp = (1.0 - alpha) * P_before + alpha * P_after
        
        # Add interpolation uncertainty proportional to alpha*(1-alpha)
        dx = x_after - x_before
        interp_uncertainty = alpha * (1.0 - alpha) * np.outer(dx, dx) * 0.1
        P_interp += interp_uncertainty
        
        P_interp = 0.5 * (P_interp + P_interp.T)
        
        return x_interp, P_interp
    
    def interpolate_trajectory(
        self,
        query_times: np.ndarray,
        scan_times: np.ndarray,
        states: List[np.ndarray],
        covariances: List[np.ndarray],
    ) -> List[Tuple[np.ndarray, np.ndarray]]:
        """Interpolate full trajectory at multiple query times.
        
        Args:
            query_times: Array of desired timestamps
            scan_times: Array of scan timestamps (sorted)
            states: States at each scan time
            covariances: Covariances at each scan time
            
        Returns:
            List of (state, covariance) at each query time.
        """
        results = []
        j = 0
        
        for t_q in query_times:
            # Find bracketing scans
            while j < len(scan_times) - 1 and scan_times[j + 1] < t_q:
                j += 1
            
            if j >= len(scan_times) - 1:
                results.append((states[-1].copy(), covariances[-1].copy()))
            elif t_q <= scan_times[0]:
                results.append((states[0].copy(), covariances[0].copy()))
            else:
                x_i, P_i = self.interpolate(
                    t_q, scan_times[j], states[j], covariances[j],
                    scan_times[j + 1], states[j + 1], covariances[j + 1]
                )
                results.append((x_i, P_i))
        
        return results


# ---------------------------------------------------------------------------
# [REQ-BATCH-005] Batch Processor — process all scans at once
# ---------------------------------------------------------------------------
@dataclass
class BatchConfig:
    """Configuration for batch processing.
    
    Attributes:
        enable_smoothing: Run RTS smoother after forward pass (default True)
        enable_interpolation: Generate interpolated states (default False)
        interpolation_dt: Time step for interpolated output (seconds)
        parallel_hint: Number of parallel workers (0 = auto)
        progress_callback: Optional callback(scan_index, total_scans)
    """
    enable_smoothing: bool = True
    enable_interpolation: bool = False
    interpolation_dt: float = 1.0
    parallel_hint: int = 0
    progress_callback: Optional[Any] = None


@dataclass
class BatchTrackResult:
    """Complete result for one track from batch processing.
    
    Attributes:
        track_id: Unique track identifier
        scan_indices: Which scans this track was present
        filtered_states: Forward KF estimates
        filtered_covs: Forward KF covariances
        smoothed_states: RTS smoothed estimates (if enabled)
        smoothed_covs: RTS smoothed covariances (if enabled)
        interpolated: Interpolated states at fine time grid (if enabled)
        quality_score: Track quality assessment (0-1)
        classification: Platform classification result (if available)
    """
    track_id: int
    scan_indices: List[int] = field(default_factory=list)
    filtered_states: List[np.ndarray] = field(default_factory=list)
    filtered_covs: List[np.ndarray] = field(default_factory=list)
    smoothed_states: Optional[List[np.ndarray]] = None
    smoothed_covs: Optional[List[np.ndarray]] = None
    interpolated: Optional[List[Tuple[float, np.ndarray, np.ndarray]]] = None
    quality_score: float = 0.0
    classification: Optional[str] = None


@dataclass
class BatchResult:
    """Complete batch processing result.
    
    Attributes:
        tracks: Dict of track_id -> BatchTrackResult
        total_scans: Number of scans processed
        total_time_s: Wall-clock processing time
        detections_per_scan: Number of measurements per scan
        confirmed_tracks: Number of confirmed tracks
        gospa_history: GOSPA at each scan (if ground truth provided)
    """
    tracks: Dict[int, BatchTrackResult] = field(default_factory=dict)
    total_scans: int = 0
    total_time_s: float = 0.0
    detections_per_scan: List[int] = field(default_factory=list)
    confirmed_tracks: int = 0
    gospa_history: Optional[List[float]] = None


class BatchProcessor:
    """Process multiple radar scans at once with optional smoothing.
    
    The BatchProcessor wraps MultiTargetTracker for offline / post-mission
    analysis. It runs the forward tracking pass, then optionally applies
    RTS smoothing and track interpolation.
    
    Example::
    
        from nx_mimosa import MultiTargetTracker
        from nx_mimosa_batch import BatchProcessor, BatchConfig
        
        tracker = MultiTargetTracker(dt=5.0, r_std=150.0, domain='air')
        batch = BatchProcessor(tracker, BatchConfig(enable_smoothing=True))
        
        # scans: list of np.ndarray, each (N_k, 3)
        result = batch.process(scans)
        
        for tid, track in result.tracks.items():
            print(f"Track {tid}: {len(track.smoothed_states)} smoothed states")
    
    Args:
        tracker: Configured MultiTargetTracker instance
        config: BatchConfig with processing options
    """
    # [REQ-BATCH-006]
    
    def __init__(self, tracker, config: Optional[BatchConfig] = None):
        self.tracker = tracker
        self.config = config or BatchConfig()
        self._smoother = RTSSmoother3D()
        self._interpolator = TrackInterpolator()
    
    def process(
        self,
        scans: List[np.ndarray],
        timestamps: Optional[List[float]] = None,
        ground_truth: Optional[List[np.ndarray]] = None,
    ) -> BatchResult:
        """Process all scans through forward tracking + optional smoothing.
        
        Args:
            scans: List of measurement arrays, each shape (N_k, 3)
            timestamps: Optional scan timestamps (defaults to dt increments)
            ground_truth: Optional ground truth positions for GOSPA computation
            
        Returns:
            BatchResult with all track data.
        """
        t_start = time.perf_counter()
        result = BatchResult()
        result.total_scans = len(scans)
        
        # Default timestamps
        if timestamps is None:
            dt = self.tracker.config.dt if hasattr(self.tracker.config, 'dt') else 5.0
            timestamps = [i * dt for i in range(len(scans))]
        
        # ── Forward pass ──
        track_histories: Dict[int, Dict] = {}  # track_id -> {states, covs, scans}
        
        for scan_idx, scan in enumerate(scans):
            self.tracker.process_scan(scan)
            result.detections_per_scan.append(len(scan))
            
            # Record states for all tracks
            for track in self.tracker.all_tracks:
                tid = track.track_id
                if tid not in track_histories:
                    track_histories[tid] = {
                        'states': [], 'covs': [], 'scans': [],
                        'predicted_states': [], 'predicted_covs': [],
                        'F_matrices': [],
                    }
                
                # Get state and covariance
                if hasattr(track, 'filter') and track.filter is not None:
                    if hasattr(track.filter, 'combined_state'):
                        x, P = track.filter.combined_state()
                    elif hasattr(track.filter, 'x'):
                        x, P = track.filter.x.copy(), track.filter.P.copy()
                    else:
                        continue
                elif hasattr(track, 'imm') and track.imm is not None:
                    x, P = track.imm.combined_state()
                elif hasattr(track, 'kf'):
                    x, P = track.kf.x.copy(), track.kf.P.copy()
                else:
                    continue
                
                h = track_histories[tid]
                h['states'].append(x.copy())
                h['covs'].append(P.copy())
                h['scans'].append(scan_idx)
                
                # Store prediction for RTS smoother
                h['predicted_states'].append(x.copy())
                h['predicted_covs'].append(P.copy())
                
                # Transition matrix
                n = x.shape[0]
                dt_scan = timestamps[scan_idx] - timestamps[scan_idx - 1] if scan_idx > 0 else timestamps[0]
                F = np.eye(n)
                if n >= 6:
                    for i in range(min(3, n)):
                        if i + 3 < n:
                            F[i, i + 3] = dt_scan
                        if i + 6 < n:
                            F[i, i + 6] = 0.5 * dt_scan**2
                            F[i + 3, i + 6] = dt_scan
                h['F_matrices'].append(F)
            
            if self.config.progress_callback:
                self.config.progress_callback(scan_idx, len(scans))
        
        # ── Build results ──
        for tid, h in track_histories.items():
            tr = BatchTrackResult(track_id=tid)
            tr.scan_indices = h['scans']
            tr.filtered_states = h['states']
            tr.filtered_covs = h['covs']
            
            # RTS smoothing
            if self.config.enable_smoothing and len(h['states']) >= 3:
                try:
                    smoothed = self._smoother.smooth(
                        h['states'], h['covs'],
                        h['predicted_states'][1:], h['predicted_covs'][1:],
                        h['F_matrices'][:-1]
                    )
                    tr.smoothed_states = [s.state for s in smoothed]
                    tr.smoothed_covs = [s.covariance for s in smoothed]
                except Exception:
                    tr.smoothed_states = h['states']
                    tr.smoothed_covs = h['covs']
            
            # Interpolation
            if self.config.enable_interpolation and len(h['states']) >= 2:
                scan_ts = [timestamps[s] for s in h['scans']]
                t_start_interp = scan_ts[0]
                t_end_interp = scan_ts[-1]
                query_ts = np.arange(t_start_interp, t_end_interp, self.config.interpolation_dt)
                
                src_states = tr.smoothed_states or h['states']
                src_covs = tr.smoothed_covs or h['covs']
                
                interp_results = self._interpolator.interpolate_trajectory(
                    query_ts, np.array(scan_ts), src_states, src_covs
                )
                tr.interpolated = [
                    (t, x, P) for t, (x, P) in zip(query_ts, interp_results)
                ]
            
            result.tracks[tid] = tr
        
        result.confirmed_tracks = sum(
            1 for t in self.tracker.confirmed_tracks
        ) if hasattr(self.tracker, 'confirmed_tracks') else len(track_histories)
        
        result.total_time_s = time.perf_counter() - t_start
        
        return result


# ---------------------------------------------------------------------------
# [REQ-BATCH-007] Convenience: process_file for CSV/scenario input
# ---------------------------------------------------------------------------
def process_file(
    filepath: str,
    tracker_config: Optional[Dict] = None,
    batch_config: Optional[BatchConfig] = None,
) -> BatchResult:
    """Convenience function: load CSV file and process all scans.
    
    Args:
        filepath: Path to CSV file with columns: time, x, y, z
        tracker_config: Dict of MultiTargetTracker kwargs
        batch_config: Optional BatchConfig
        
    Returns:
        BatchResult
    """
    # Lazy import to avoid circular dependency
    from python.nx_mimosa_mtt import MultiTargetTracker
    
    config = tracker_config or {'dt': 5.0, 'r_std': 150.0, 'domain': 'air'}
    tracker = MultiTargetTracker(**config)
    batch = BatchProcessor(tracker, batch_config)
    
    # Load CSV
    data = np.loadtxt(filepath, delimiter=',', skiprows=1)
    times = np.unique(data[:, 0])
    
    scans = []
    for t in times:
        mask = data[:, 0] == t
        scans.append(data[mask, 1:4])  # x, y, z columns
    
    return batch.process(scans, timestamps=times.tolist())
