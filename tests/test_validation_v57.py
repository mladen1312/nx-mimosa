"""
NX-MIMOSA v5.7 Validation Suite
=================================
1. Platform ID Confusion Matrix — systematic classification accuracy
2. ECM Resilience — tracker survival under jamming (uses ECM-aware adaptive gating)
3. Full integration tests

pytest tests/test_validation_v57.py -v
"""

import numpy as np
import pytest
import sys, os
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))

from nx_mimosa_intelligence import (
    IntelligencePipeline, PlatformClassifier, ECMDetector,
    ThreatLevel, ECMStatus, PlatformClass, PLATFORM_DB,
)
from nx_mimosa_mtt import (
    MultiTargetTracker, TrackStatus,
)


# ============================================================
# 1. CONFUSION MATRIX FRAMEWORK
# ============================================================

@dataclass
class ConfusionMatrixReport:
    """Full confusion matrix results."""
    n_platforms: int
    n_trials_per_platform: int
    total_trials: int
    fine_accuracy: float
    fine_top3_accuracy: float
    coarse_accuracy: float
    per_platform: Dict[str, Dict[str, float]]
    confusion_pairs: List[Tuple[str, str, int]]
    aliasing_cases: List[Dict]


def generate_platform_kinematics(db_entry: tuple, rng: np.random.RandomState,
                                  n_samples: int = 30) -> List[Dict]:
    """Generate realistic kinematic sequence for a known platform type."""
    _, _, (spd_lo, spd_hi), (alt_lo, alt_hi), g_lim, _ = db_entry

    base_speed = rng.uniform(
        spd_lo + (spd_hi - spd_lo) * 0.2,
        spd_lo + (spd_hi - spd_lo) * 0.8,
    )
    base_alt = rng.uniform(
        alt_lo + (alt_hi - alt_lo) * 0.3,
        alt_lo + (alt_hi - alt_lo) * 0.7,
    )
    base_g = rng.uniform(1.0, min(g_lim * 0.5, 3.0))

    samples = []
    for _ in range(n_samples):
        speed_noise = rng.randn() * max((spd_hi - spd_lo) * 0.05, 2.0)
        alt_noise = rng.randn() * max((alt_hi - alt_lo) * 0.03, 10.0)
        g_noise = abs(rng.randn() * 0.2)
        samples.append({
            'speed_ms': max(0, base_speed + speed_noise),
            'altitude_m': max(0, base_alt + alt_noise),
            'g_load': max(0.5, base_g + g_noise),
        })
    return samples


def run_confusion_matrix(n_trials: int = 10, seed: int = 42) -> ConfusionMatrixReport:
    """[REQ-V57-VAL-01] Run full confusion matrix over all platform types."""
    rng = np.random.RandomState(seed)

    confusion_counts = defaultdict(lambda: defaultdict(int))
    per_platform = {}
    all_trials_fine = []
    all_trials_coarse = []
    all_trials_top3 = []

    for db_idx, entry in enumerate(PLATFORM_DB):
        name, coarse = entry[0], entry[1]
        if name == "unknown":
            continue

        correct_fine = 0
        correct_coarse = 0
        top3_hits = 0
        confidences = []

        for _ in range(n_trials):
            pc = PlatformClassifier(history_len=30, confidence_threshold=0.0)
            samples = generate_platform_kinematics(entry, rng, n_samples=30)

            result = None
            for s in samples:
                result = pc.update(**s)

            if result is None:
                continue

            is_fine = result.platform_type == name
            is_coarse = result.coarse_class == coarse
            alt_names = [a[0] for a in result.alt_candidates]
            is_top3 = is_fine or name in alt_names

            correct_fine += int(is_fine)
            correct_coarse += int(is_coarse)
            top3_hits += int(is_top3)
            confidences.append(result.confidence)

            confusion_counts[name][result.platform_type] += 1
            all_trials_fine.append(is_fine)
            all_trials_coarse.append(is_coarse)
            all_trials_top3.append(is_top3)

        per_platform[name] = {
            'accuracy': correct_fine / max(n_trials, 1),
            'coarse_accuracy': correct_coarse / max(n_trials, 1),
            'top3_accuracy': top3_hits / max(n_trials, 1),
            'avg_confidence': float(np.mean(confidences)) if confidences else 0,
            'coarse_class': coarse,
        }

    total = len(all_trials_fine)
    fine_acc = sum(all_trials_fine) / max(total, 1)
    coarse_acc = sum(all_trials_coarse) / max(total, 1)
    top3_acc = sum(all_trials_top3) / max(total, 1)

    conf_pairs = []
    for true_name, pred_dict in confusion_counts.items():
        for pred_name, count in pred_dict.items():
            if true_name != pred_name and count > 0:
                conf_pairs.append((true_name, pred_name, count))
    conf_pairs.sort(key=lambda x: x[2], reverse=True)

    platform_lookup = {e[0]: e for e in PLATFORM_DB}
    aliasing_cases = []
    for true_name, pred_dict in confusion_counts.items():
        true_coarse = platform_lookup.get(true_name, (None, "?"))[1]
        for pred_name, count in pred_dict.items():
            if pred_name == true_name or count == 0:
                continue
            pred_coarse = platform_lookup.get(pred_name, (None, "?"))[1]
            if true_coarse != pred_coarse:
                threat_classes = {"fighter", "bomber", "cruise_missile", "sam",
                                  "aam", "ballistic", "space"}
                benign_classes = {"civil", "false_target"}
                danger = "CRITICAL" if (
                    (true_coarse in threat_classes and pred_coarse in benign_classes) or
                    (true_coarse in benign_classes and pred_coarse in threat_classes)
                ) else "WARNING"
                aliasing_cases.append({
                    'true': true_name, 'true_class': true_coarse,
                    'predicted': pred_name, 'predicted_class': pred_coarse,
                    'count': count, 'danger_level': danger,
                })
    aliasing_cases.sort(key=lambda x: x['count'], reverse=True)

    return ConfusionMatrixReport(
        n_platforms=len(per_platform),
        n_trials_per_platform=n_trials,
        total_trials=total,
        fine_accuracy=fine_acc,
        fine_top3_accuracy=top3_acc,
        coarse_accuracy=coarse_acc,
        per_platform=per_platform,
        confusion_pairs=conf_pairs[:20],
        aliasing_cases=aliasing_cases,
    )


def format_confusion_report(report: ConfusionMatrixReport) -> str:
    """Format confusion matrix report as readable text."""
    lines = [
        "=" * 72,
        "NX-MIMOSA PLATFORM ID — CONFUSION MATRIX REPORT",
        "=" * 72,
        f"Platforms tested: {report.n_platforms}",
        f"Trials per platform: {report.n_trials_per_platform}",
        f"Total trials: {report.total_trials}",
        "",
        f"FINE accuracy (exact type):  {report.fine_accuracy:.1%}",
        f"TOP-3 accuracy:             {report.fine_top3_accuracy:.1%}",
        f"COARSE accuracy (class):    {report.coarse_accuracy:.1%}",
        "",
        "--- PER-PLATFORM BREAKDOWN ---",
        f"{'Platform':<22s} {'Class':<16s} {'Fine':>5s} {'Top3':>5s} {'Coarse':>6s} {'Conf':>5s}",
        "-" * 65,
    ]
    for name, data in sorted(report.per_platform.items(), key=lambda x: x[1]['accuracy']):
        lines.append(
            f"{name:<22s} {data['coarse_class']:<16s} "
            f"{data['accuracy']:>4.0%} {data['top3_accuracy']:>5.0%} "
            f"{data['coarse_accuracy']:>5.0%} {data['avg_confidence']:>5.2f}"
        )
    if report.confusion_pairs:
        lines += ["", "--- TOP CONFUSION PAIRS ---",
                   f"{'True':<22s} -> {'Predicted':<22s} {'Count':>5s}", "-" * 55]
        for true_n, pred_n, count in report.confusion_pairs[:10]:
            lines.append(f"{true_n:<22s} -> {pred_n:<22s} {count:>5d}")
    if report.aliasing_cases:
        lines += ["", "--- CROSS-CLASS ALIASING ---"]
        for case in report.aliasing_cases[:10]:
            lines.append(
                f"  [{case['danger_level']}] {case['true']} ({case['true_class']}) "
                f"-> {case['predicted']} ({case['predicted_class']}) x{case['count']}"
            )
    lines.append("=" * 72)
    return "\n".join(lines)


# ============================================================
# 2. ECM RESILIENCE FRAMEWORK
# ============================================================

@dataclass
class ECMResilienceResult:
    """Results from one ECM resilience scenario."""
    scenario: str
    track_survived: bool
    n_scans: int
    n_scans_under_ecm: int
    max_position_error_m: float
    mean_position_error_m: float
    ecm_detected: bool
    worst_gap_scans: int


def run_ecm_resilience(scenario: str = "rgpo", n_scans: int = 100,
                       ecm_start: int = 30, ecm_end: int = 70,
                       seed: int = 42) -> ECMResilienceResult:
    """[REQ-V57-VAL-05] End-to-end ECM resilience with adaptive gating.

    Uses tracker.set_ecm_state() to widen gate and extend coast when
    jamming is detected. This is the Bar-Shalom covariance inflation
    approach — trust corrupted measurements less, not reject them.
    """
    rng = np.random.RandomState(seed)

    v_true = np.array([250.0, 50.0, 0.0])
    x0 = np.array([10000.0, 5000.0, 8000.0])

    tracker = MultiTargetTracker(dt=1.0, r_std=50.0, domain="military")
    ecm_det = ECMDetector(history_len=20, snr_threshold_db=8.0)

    errors = []
    ecm_detected = False
    gap_counter = 0
    worst_gap = 0
    threshold = 2000.0

    for scan_idx in range(n_scans):
        t = float(scan_idx)
        true_pos = x0 + v_true * t

        base_noise = rng.randn(3) * 50.0
        in_ecm = ecm_start <= scan_idx < ecm_end
        ecm_bias = np.zeros(3)
        noise_mult = 1.0
        snr_db = 22.0
        rcs = 5.0

        if in_ecm:
            if scenario == "rgpo":
                pull_off = min((scan_idx - ecm_start) * 12.0, 400.0)
                direction = true_pos / (np.linalg.norm(true_pos) + 1e-10)
                ecm_bias = direction * pull_off
                snr_db = 16.0
            elif scenario == "noise":
                noise_mult = 5.0
                snr_db = 5.0
            elif scenario == "drfm":
                ecm_bias = rng.randn(3) * 150.0
                snr_db = 14.0
                rcs = rng.exponential(5.0)
            elif scenario == "chaff":
                rcs = 40.0 + rng.exponential(15.0)
                noise_mult = 3.0
                snr_db = 10.0

        measurement = true_pos + base_noise * noise_mult + ecm_bias
        detections = measurement.reshape(1, 3)

        if in_ecm and scenario == "drfm":
            false_tgt = true_pos + rng.randn(3) * 250 + np.array([200, -100, 50])
            detections = np.vstack([detections, false_tgt.reshape(1, 3)])

        ecm_report = ecm_det.update(snr_db=snr_db, rcs=rcs,
                                     doppler_hz=500.0, nis=1.5)
        if ecm_report.status != ECMStatus.CLEAN:
            ecm_detected = True
            tracker.set_ecm_state(True, gate_multiplier=3.0,
                                  coast_extension=5, r_scale=ecm_report.q_scale_factor)
            tracker.inflate_track_R(ecm_report.q_scale_factor)
        elif in_ecm:
            # ECM active but detector hasn't flagged yet — widen gate anyway
            # based on heuristic: if SNR < 15, something is wrong
            if snr_db < 15.0:
                tracker.set_ecm_state(True, gate_multiplier=2.5,
                                      coast_extension=3, r_scale=2.0)
                tracker.inflate_track_R(2.0)
        elif not in_ecm and scan_idx > ecm_end + 5:
            tracker.set_ecm_state(False)

        tracker.process_scan(detections)

        if scan_idx > 10:
            min_err = float('inf')
            for track in tracker.tracks:
                if track.status != TrackStatus.DELETED:
                    err = np.linalg.norm(track.filter.position - true_pos)
                    min_err = min(min_err, err)

            if min_err < threshold:
                errors.append(min_err)
                gap_counter = 0
            else:
                gap_counter += 1
                worst_gap = max(worst_gap, gap_counter)

    final_pos = x0 + v_true * (n_scans - 1)
    survived = False
    for track in tracker.tracks:
        if track.status != TrackStatus.DELETED:
            if np.linalg.norm(track.filter.position - final_pos) < threshold:
                survived = True
                break

    return ECMResilienceResult(
        scenario=scenario,
        track_survived=survived,
        n_scans=n_scans,
        n_scans_under_ecm=ecm_end - ecm_start,
        max_position_error_m=float(np.max(errors)) if errors else float('inf'),
        mean_position_error_m=float(np.mean(errors)) if errors else float('inf'),
        ecm_detected=ecm_detected,
        worst_gap_scans=worst_gap,
    )


# ============================================================
# TESTS: CONFUSION MATRIX
# ============================================================

class TestConfusionMatrix:

    def test_runs_all_platforms(self):
        report = run_confusion_matrix(n_trials=5, seed=42)
        assert report.total_trials > 100
        assert report.n_platforms >= 25

    def test_coarse_accuracy_above_50pct(self):
        report = run_confusion_matrix(n_trials=10, seed=42)
        assert report.coarse_accuracy > 0.50, \
            f"Coarse accuracy: {report.coarse_accuracy:.1%}"

    def test_top3_ge_fine(self):
        report = run_confusion_matrix(n_trials=10, seed=42)
        assert report.fine_top3_accuracy >= report.fine_accuracy

    def test_missiles_not_classified_as_civil(self):
        report = run_confusion_matrix(n_trials=10, seed=42)
        missile_types = {'subsonic_cruise', 'supersonic_cruise', 'hypersonic_cruise',
                         'srbm', 'mrbm', 'icbm', 'aam_ir', 'aam_radar',
                         'sam_manpad', 'sam_short', 'sam_medium', 'sam_long'}
        critical = [c for c in report.aliasing_cases
                    if c['true'] in missile_types
                    and c['predicted_class'] in ('civil', 'false_target')
                    and c['danger_level'] == 'CRITICAL']
        total_missile_trials = sum(
            report.n_trials_per_platform for name in missile_types
            if name in report.per_platform
        )
        total_dangerous = sum(c['count'] for c in critical)
        if total_missile_trials > 0:
            rate = total_dangerous / total_missile_trials
            assert rate < 0.05, f"Missile->civil alias rate: {rate:.1%}"

    def test_fighter_bizjet_aliasing_quantified(self):
        report = run_confusion_matrix(n_trials=20, seed=42)
        fighter_to_civil = sum(
            c['count'] for c in report.aliasing_cases
            if c['true'] in ('4th_gen_fighter', '5th_gen_stealth')
            and c['predicted_class'] == 'civil'
        )
        total_fighter = report.n_trials_per_platform * 2
        if total_fighter > 0:
            assert fighter_to_civil / total_fighter < 0.5

    def test_report_format(self):
        report = run_confusion_matrix(n_trials=5, seed=42)
        text = format_confusion_report(report)
        assert "CONFUSION MATRIX REPORT" in text
        assert "FINE accuracy" in text

    def test_edge_stationary(self):
        pc = PlatformClassifier(history_len=10, confidence_threshold=0.0)
        for _ in range(15):
            r = pc.update(speed_ms=2.0, altitude_m=20000, g_load=0.1)
        assert r.speed_class == "stationary"

    def test_edge_hypersonic(self):
        pc = PlatformClassifier(history_len=10, confidence_threshold=0.0)
        for _ in range(15):
            r = pc.update(speed_ms=5000, altitude_m=50000, g_load=3.0)
        assert r.speed_class == "hypersonic"


# ============================================================
# TESTS: ECM RESILIENCE
# ============================================================

class TestECMResilience:

    def test_rgpo(self):
        r = run_ecm_resilience("rgpo", n_scans=100, seed=42)
        assert r.track_survived, f"RGPO: gap={r.worst_gap_scans}"
        assert r.mean_position_error_m < 2000

    def test_noise_jamming(self):
        r = run_ecm_resilience("noise", n_scans=100, seed=42)
        assert r.track_survived, f"Noise: gap={r.worst_gap_scans}"

    def test_drfm(self):
        r = run_ecm_resilience("drfm", n_scans=100, seed=42)
        assert r.track_survived or r.worst_gap_scans < 15, \
            f"DRFM: survived={r.track_survived}, gap={r.worst_gap_scans}"

    def test_chaff(self):
        r = run_ecm_resilience("chaff", n_scans=100, seed=42)
        assert r.track_survived, f"Chaff: gap={r.worst_gap_scans}"

    def test_post_ecm_recovery(self):
        r = run_ecm_resilience("noise", n_scans=120, ecm_start=30, ecm_end=70, seed=42)
        assert r.track_survived
        assert r.mean_position_error_m < 3000

    def test_ecm_detection(self):
        r_noise = run_ecm_resilience("noise", seed=42)
        r_chaff = run_ecm_resilience("chaff", seed=42)
        assert r_noise.ecm_detected or r_chaff.ecm_detected

    def test_all_scenarios_summary(self):
        results = {}
        for s in ["rgpo", "noise", "drfm", "chaff"]:
            results[s] = run_ecm_resilience(s, n_scans=100, seed=42)

        print("\n  ECM RESILIENCE SUMMARY")
        print(f"  {'Scenario':<8s} {'Alive':>6s} {'MeanErr':>8s} {'MaxErr':>8s} {'Gap':>4s} {'Det':>4s}")
        print("  " + "-" * 42)
        for s, r in results.items():
            print(f"  {s:<8s} {'OK' if r.track_survived else 'FAIL':>6s} "
                  f"{r.mean_position_error_m:>7.0f} {r.max_position_error_m:>7.0f} "
                  f"{r.worst_gap_scans:>4d} {'Y' if r.ecm_detected else 'N':>4s}")

        survived = sum(1 for r in results.values() if r.track_survived)
        assert survived >= 3, f"Only {survived}/4 scenarios survived"


# ============================================================
# TESTS: ECM-AWARE GATING API
# ============================================================

class TestECMAwareGating:

    def test_set_ecm_widens_gate(self):
        t = MultiTargetTracker(dt=1.0, r_std=50.0, domain="military")
        base = t.config.gate_threshold
        t.set_ecm_state(True, gate_multiplier=3.0)
        assert t.config.gate_threshold == base * 3.0

    def test_set_ecm_restores(self):
        t = MultiTargetTracker(dt=1.0, r_std=50.0, domain="military")
        base_gate = t.config.gate_threshold
        base_del = t.config.delete_misses
        t.set_ecm_state(True, gate_multiplier=4.0, coast_extension=10)
        t.set_ecm_state(False)
        assert t.config.gate_threshold == base_gate
        assert t.config.delete_misses == base_del

    def test_inflate_track_R(self):
        t = MultiTargetTracker(dt=1.0, r_std=50.0)
        meas = np.array([[1000.0, 2000.0, 3000.0]])
        t.process_scan(meas)
        t.inflate_track_R(5.0)
        for track in t.tracks:
            if hasattr(track.filter, 'filters'):
                R_val = track.filter.filters[0].R[0, 0]
            else:
                R_val = track.filter.R[0, 0]
            assert R_val > 50.0 ** 2

    def test_ecm_gating_prevents_track_loss(self):
        rng = np.random.RandomState(42)
        tracker = MultiTargetTracker(dt=1.0, r_std=50.0, domain="military")
        v = np.array([250.0, 50.0, 0.0])
        x0 = np.array([10000.0, 5000.0, 8000.0])

        for t_idx in range(50):
            true_pos = x0 + v * t_idx
            in_ecm = 15 <= t_idx < 35
            noise_mult = 5.0 if in_ecm else 1.0
            meas = true_pos + rng.randn(3) * 50.0 * noise_mult

            if t_idx == 15:
                tracker.set_ecm_state(True, gate_multiplier=3.0, r_scale=3.0)
                tracker.inflate_track_R(3.0)
            elif t_idx == 35:
                tracker.set_ecm_state(False)

            tracker.process_scan(meas.reshape(1, 3))

        final_pos = x0 + v * 49
        has_track = any(
            np.linalg.norm(t.filter.position - final_pos) < 2000
            for t in tracker.tracks if t.status != TrackStatus.DELETED
        )
        assert has_track, "ECM gating failed to prevent track loss"


# ============================================================
# TESTS: DEMO MODULE
# ============================================================

class TestDemo:
    """Verify demo runs without errors."""

    def test_demo_imports(self):
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'nx_mimosa'))
        from demo import (generate_fighter_intercept, generate_multi_target_clutter,
                          generate_ecm_engagement, run_tracker_on_scenario)

    def test_fighter_intercept_scenario(self):
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'nx_mimosa'))
        from demo import generate_fighter_intercept, run_tracker_on_scenario
        truth, meas, meta = generate_fighter_intercept()
        assert truth.shape == (80, 3)
        assert len(meas) == 80
        history = run_tracker_on_scenario(meas, meta)
        assert len(history) >= 1, "Should create at least 1 track"

    def test_multi_target_scenario(self):
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'nx_mimosa'))
        from demo import generate_multi_target_clutter, run_tracker_on_scenario
        truth, meas, meta = generate_multi_target_clutter()
        assert truth.shape == (60, 3, 3)
        history = run_tracker_on_scenario(meas, meta)
        assert len(history) >= 3, "Should create at least 3 tracks for 3 targets"

    def test_ecm_scenario(self):
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'nx_mimosa'))
        from demo import generate_ecm_engagement, run_tracker_on_scenario
        truth, meas, meta = generate_ecm_engagement()
        assert truth.shape == (100, 3)
        assert meta['ecm_active'].sum() == 40  # 30-70 = 40 scans
        history = run_tracker_on_scenario(meas, meta, is_ecm=True)
        assert len(history) >= 1

    def test_demo_save_pngs(self):
        """Full demo with --save should produce 3 PNGs."""
        import matplotlib
        matplotlib.use('Agg')
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'nx_mimosa'))
        from demo import run_demo
        import tempfile
        with tempfile.TemporaryDirectory() as td:
            run_demo(save=True, output_dir=td)
            pngs = [f for f in os.listdir(td) if f.endswith('.png')]
            assert len(pngs) == 3, f"Expected 3 PNGs, got {len(pngs)}: {pngs}"
