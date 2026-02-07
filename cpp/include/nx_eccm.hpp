// [REQ-V620-ECCM-01] NX-MIMOSA v6.2.0 — C++ ECCM Core Module
// Ported from Python nx_mimosa_eccm.py → zero-copy inline with tracking pipeline
// Copyright (c) 2026 Nexellum d.o.o. — AGPL-3.0-or-later
//
// Key v6.2.0 changes vs v6.1.0 Python overlay:
//   1. ECCM runs INSIDE process_scan() → no Python→C++ boundary crossing
//   2. Closed-loop: classify() output directly inflates R in IMM update()
//   3. ML window extended to 20 (was 10 in Python)
//   4. Cross-track sector correlation: coherent threat detection
//
// References:
//   - QEDMMA v2/rtl/eccm/ml_cfar_engine.sv (553 lines SV → C++)
//   - Neri, "Introduction to Electronic Defense Systems," Artech House
//   - Richards et al., "Principles of Modern Radar," SciTech, Ch. 16
#pragma once

#include <cmath>
#include <cstring>
#include <array>
#include <vector>
#include <algorithm>
#include <numeric>
#include <cassert>

namespace nx {
namespace eccm {

// ============================================================
// ENVIRONMENT CLASSIFICATION
// ============================================================
enum class EnvironmentClass : uint8_t {
    CLEAR        = 0,   // Normal — R multiplier 1.0×
    CLUTTER      = 1,   // Heavy clutter — R multiplier 2.0×
    NOISE_JAM    = 2,   // Barrage/spot noise — R multiplier 5.0×
    DECEPTION    = 3    // RGPO/VGPO/DRFM — R multiplier 10.0×
};

inline const char* env_class_str(EnvironmentClass c) {
    switch (c) {
        case EnvironmentClass::CLEAR:     return "CLEAR";
        case EnvironmentClass::CLUTTER:   return "CLUTTER";
        case EnvironmentClass::NOISE_JAM: return "NOISE_JAM";
        case EnvironmentClass::DECEPTION: return "DECEPTION";
    }
    return "UNKNOWN";
}

// [REQ-V620-ECCM-02] R-matrix inflation factors per environment class
// Matches QEDMMA integration_controller.sv adaptive thresholds
inline double r_multiplier(EnvironmentClass c) {
    switch (c) {
        case EnvironmentClass::CLEAR:     return 1.0;
        case EnvironmentClass::CLUTTER:   return 2.0;
        case EnvironmentClass::NOISE_JAM: return 5.0;
        case EnvironmentClass::DECEPTION: return 10.0;
    }
    return 1.0;
}


// ============================================================
// 6-FEATURE VECTOR (stack-allocated, matches FPGA layout)
// ============================================================
struct Features {
    double power_ratio       = 0.0;  // F1: NIS / baseline NIS
    double guard_ratio       = 0.0;  // F2: local NIS var / global var
    double spectral_flatness = 0.0;  // F3: kurtosis of NIS sequence
    double temporal_corr     = 0.0;  // F4: lag-1 autocorrelation
    double range_deriv       = 0.0;  // F5: range residual gradient (σ)
    double doppler_deriv     = 0.0;  // F6: velocity residual gradient (σ)
};

struct ClassifyResult {
    EnvironmentClass env_class = EnvironmentClass::CLEAR;
    double confidence = 1.0;
    double r_mult     = 1.0;   // Precomputed R multiplier
};


// ============================================================
// CIRCULAR BUFFER (fixed-size, stack-friendly, no heap per track)
// ============================================================
template<int CAPACITY>
struct CircBuf {
    double data[CAPACITY];
    int head = 0;     // Next write position
    int count = 0;    // Current number of elements

    void push(double v) {
        data[head] = v;
        head = (head + 1) % CAPACITY;
        if (count < CAPACITY) count++;
    }

    void clear() { head = 0; count = 0; }

    int size() const { return count; }
    bool full() const { return count == CAPACITY; }

    // Get element by age (0 = most recent)
    double recent(int age) const {
        assert(age < count);
        int idx = (head - 1 - age + CAPACITY) % CAPACITY;
        return data[idx];
    }

    // Get N most recent into output array (output[0] = oldest of the N)
    int recent_n(double* out, int n) const {
        int actual = std::min(n, count);
        for (int i = 0; i < actual; i++)
            out[i] = recent(actual - 1 - i);
        return actual;
    }

    // Statistics over last N samples
    double mean_last(int n) const {
        int actual = std::min(n, count);
        if (actual == 0) return 0;
        double s = 0;
        for (int i = 0; i < actual; i++) s += recent(i);
        return s / actual;
    }

    double var_last(int n) const {
        int actual = std::min(n, count);
        if (actual < 2) return 0;
        double m = mean_last(n);
        double s = 0;
        for (int i = 0; i < actual; i++) {
            double d = recent(i) - m;
            s += d * d;
        }
        return s / actual;
    }

    double std_last(int n) const { return std::sqrt(var_last(n)); }
};


// ============================================================
// [REQ-V620-ECCM-03] PER-TRACK ECCM STATE
// Stack-allocated, ~2 KB per track — embedded in Track struct
// ML window = 20 (extended from 10 in v6.1.0)
// History buffer = 60 (3× window for baseline)
// ============================================================
static constexpr int ML_WINDOW     = 20;    // v6.2.0: extended from 10
static constexpr int HISTORY_DEPTH = 60;    // 3× window for baseline building

struct TrackECCM {
    // Circular buffers for feature extraction
    CircBuf<HISTORY_DEPTH> nis_history;
    CircBuf<HISTORY_DEPTH> range_innov;
    CircBuf<HISTORY_DEPTH> bearing_innov;
    CircBuf<HISTORY_DEPTH> speed_history;

    // Baseline statistics (established from first ML_WINDOW clear samples)
    double baseline_nis_mean = -1.0;   // -1 = not established
    double baseline_nis_std  = 1.0;
    double baseline_ri_std   = 1.0;    // Range innovation baseline
    double baseline_rr_std   = 1.0;    // Range-rate baseline

    // Current classification state
    EnvironmentClass current_class = EnvironmentClass::CLEAR;
    double current_confidence = 1.0;
    double current_r_mult = 1.0;

    // Track metadata for ECCM
    int n_updates = 0;
    int n_jammed_scans = 0;          // Consecutive jammed scans
    double sector_azimuth_rad = 0.0; // For cross-track correlation

    void reset() {
        nis_history.clear();
        range_innov.clear();
        bearing_innov.clear();
        speed_history.clear();
        baseline_nis_mean = -1.0;
        baseline_nis_std = 1.0;
        baseline_ri_std = 1.0;
        baseline_rr_std = 1.0;
        current_class = EnvironmentClass::CLEAR;
        current_confidence = 1.0;
        current_r_mult = 1.0;
        n_updates = 0;
        n_jammed_scans = 0;
        sector_azimuth_rad = 0.0;
    }
};


// ============================================================
// [REQ-V620-ECCM-04] ML-CFAR DETECTOR — DECISION TREE ENGINE
// Ported from QEDMMA ml_cfar_engine.sv (Q16.16 fixed-point → double)
// ============================================================
class MLCFARDetector {
public:
    // Decision tree thresholds (from QEDMMA FPGA ml_weights[0:63])
    static constexpr double THRESH_POWER_HIGH    = 4.0;   // NIS ratio → jamming
    static constexpr double THRESH_POWER_LOW     = 0.3;   // NIS ratio → blanking
    static constexpr double THRESH_KURTOSIS_FLAT = 2.0;   // < 2.0 → barrage
    static constexpr double THRESH_KURTOSIS_SPIKY= 6.0;   // > 6.0 → deception
    static constexpr double THRESH_TEMPORAL_CORR = 0.7;   // Autocorr → coherent
    static constexpr double THRESH_RANGE_DERIV   = 3.0;   // σ multiplier for RGPO
    static constexpr double THRESH_DOPPLER_DERIV = 3.0;   // σ multiplier for VGPO

    static constexpr int MIN_SAMPLES = 8;  // Minimum before classify activates

    // ── Feature Extraction ──
    // [REQ-V620-ECCM-05] Extract 6-feature vector from track statistics
    // Runs O(1) per call — just circular buffer arithmetic
    static Features extract_features(TrackECCM& st, double nis,
                                     double innov_range, double innov_bearing,
                                     double speed) {
        st.nis_history.push(nis);
        st.range_innov.push(innov_range);
        st.bearing_innov.push(innov_bearing);
        st.speed_history.push(speed);
        st.n_updates++;

        Features f;
        if (st.n_updates < MIN_SAMPLES) return f;

        // Establish baseline from first window (clear-sky assumption)
        if (st.baseline_nis_mean < 0 && st.nis_history.size() >= ML_WINDOW) {
            // Use oldest ML_WINDOW samples as baseline
            double buf[ML_WINDOW];
            // Get the oldest samples for baseline
            int n = st.nis_history.size();
            for (int i = 0; i < ML_WINDOW && i < n; i++) {
                buf[i] = st.nis_history.recent(n - 1 - i);
            }
            double sum = 0;
            for (int i = 0; i < ML_WINDOW; i++) sum += buf[i];
            st.baseline_nis_mean = sum / ML_WINDOW;

            double var = 0;
            for (int i = 0; i < ML_WINDOW; i++) {
                double d = buf[i] - st.baseline_nis_mean;
                var += d * d;
            }
            st.baseline_nis_std = std::max(std::sqrt(var / ML_WINDOW), 0.1);

            // Range innovation baseline
            if (st.range_innov.size() >= ML_WINDOW) {
                double ri_buf[ML_WINDOW];
                for (int i = 0; i < ML_WINDOW; i++)
                    ri_buf[i] = st.range_innov.recent(st.range_innov.size() - 1 - i);
                double ri_m = 0;
                for (int i = 0; i < ML_WINDOW; i++) ri_m += ri_buf[i];
                ri_m /= ML_WINDOW;
                double ri_v = 0;
                for (int i = 0; i < ML_WINDOW; i++) {
                    double d = ri_buf[i] - ri_m;
                    ri_v += d * d;
                }
                st.baseline_ri_std = std::max(std::sqrt(ri_v / ML_WINDOW), 1.0);
            }

            // Range-rate baseline
            if (st.speed_history.size() >= ML_WINDOW) {
                double rr_buf[ML_WINDOW];
                for (int i = 0; i < ML_WINDOW; i++)
                    rr_buf[i] = st.speed_history.recent(st.speed_history.size() - 1 - i);
                double rr_m = 0;
                for (int i = 0; i < ML_WINDOW; i++) rr_m += rr_buf[i];
                rr_m /= ML_WINDOW;
                double rr_v = 0;
                for (int i = 0; i < ML_WINDOW; i++) {
                    double d = rr_buf[i] - rr_m;
                    rr_v += d * d;
                }
                st.baseline_rr_std = std::max(std::sqrt(rr_v / ML_WINDOW), 1.0);
            }
        }

        double bl_mean = (st.baseline_nis_mean > 0)
                         ? st.baseline_nis_mean
                         : st.nis_history.mean_last(ML_WINDOW);
        double bl_std  = (st.baseline_nis_mean > 0)
                         ? st.baseline_nis_std
                         : std::max(st.nis_history.std_last(ML_WINDOW), 0.1);

        // ── F1: Power ratio ──
        f.power_ratio = st.nis_history.mean_last(ML_WINDOW) / std::max(bl_mean, 0.01);

        // ── F2: Guard ratio (local var / global var) ──
        if (st.nis_history.size() >= 4) {
            double local_var  = st.nis_history.var_last(4);
            double global_var = st.nis_history.var_last(st.nis_history.size());
            f.guard_ratio = local_var / std::max(global_var, 0.01);
        }

        // ── F3: Spectral flatness (kurtosis of NIS window) ──
        if (st.nis_history.size() >= ML_WINDOW) {
            double mu   = st.nis_history.mean_last(ML_WINDOW);
            double sigma = std::max(st.nis_history.std_last(ML_WINDOW), 1e-6);
            double k4 = 0;
            for (int i = 0; i < ML_WINDOW; i++) {
                double z = (st.nis_history.recent(i) - mu) / sigma;
                k4 += z * z * z * z;
            }
            f.spectral_flatness = k4 / ML_WINDOW;
        }

        // ── F4: Temporal correlation (lag-1 autocorrelation) ──
        if (st.nis_history.size() >= 4) {
            int n = std::min(ML_WINDOW, st.nis_history.size());
            double mu = st.nis_history.mean_last(n);
            double var = st.nis_history.var_last(n);
            if (var > 1e-10) {
                double cov = 0;
                for (int i = 0; i < n - 1; i++) {
                    cov += (st.nis_history.recent(i) - mu) *
                           (st.nis_history.recent(i + 1) - mu);
                }
                cov /= (n - 1);
                f.temporal_corr = std::clamp(cov / var, -1.0, 1.0);
            }
        }

        // ── F5: Range derivative (RGPO detection) ──
        if (st.range_innov.size() >= 6) {
            // Mean absolute diff of last 4 range innovations
            double deriv = 0;
            for (int i = 0; i < 3; i++)
                deriv += std::abs(st.range_innov.recent(i) - st.range_innov.recent(i + 1));
            deriv /= 3.0;
            f.range_deriv = deriv / st.baseline_ri_std;
        }

        // ── F6: Doppler derivative (VGPO detection) ──
        if (st.speed_history.size() >= 6) {
            double max_diff = 0;
            for (int i = 0; i < 3; i++) {
                double d = std::abs(st.speed_history.recent(i) -
                                    st.speed_history.recent(i + 1));
                max_diff = std::max(max_diff, d);
            }
            f.doppler_deriv = max_diff / st.baseline_rr_std;
        }

        return f;
    }

    // ── Classification ──
    // [REQ-V620-ECCM-06] Decision tree matching QEDMMA FPGA inference
    // (ml_cfar_engine.sv lines 380-420)
    //
    //   power_ratio > 4.0?
    //   ├─ YES → kurtosis < 2.0?
    //   │        ├─ YES → NOISE_JAM (flat spectrum barrage)
    //   │        └─ NO  → temporal_corr > 0.7?
    //   │                 ├─ YES → DECEPTION (coherent DRFM)
    //   │                 └─ NO  → CLUTTER (high but random)
    //   └─ NO  → range_deriv > 3σ OR doppler_deriv > 3σ?
    //            ├─ YES → DECEPTION (pull-off detected)
    //            └─ NO  → power_ratio < 0.3?
    //                     ├─ YES → NOISE_JAM (blanking/desensitization)
    //                     └─ NO  → CLEAR
    //
    static ClassifyResult classify(const Features& f) {
        ClassifyResult r;

        // Branch 1: High power ratio
        if (f.power_ratio > THRESH_POWER_HIGH) {
            if (f.spectral_flatness < THRESH_KURTOSIS_FLAT) {
                r.env_class  = EnvironmentClass::NOISE_JAM;
                r.confidence = std::min(1.0, f.power_ratio / (THRESH_POWER_HIGH * 2));
            } else if (f.temporal_corr > THRESH_TEMPORAL_CORR) {
                r.env_class  = EnvironmentClass::DECEPTION;
                r.confidence = std::min(1.0, (double)f.temporal_corr);
            } else {
                r.env_class  = EnvironmentClass::CLUTTER;
                r.confidence = std::min(1.0,
                    (f.power_ratio - THRESH_POWER_HIGH) / THRESH_POWER_HIGH);
            }
        }
        // Branch 2: Pull-off detection
        else if (f.range_deriv > THRESH_RANGE_DERIV ||
                 f.doppler_deriv > THRESH_DOPPLER_DERIV) {
            r.env_class  = EnvironmentClass::DECEPTION;
            r.confidence = std::min(1.0,
                std::max(f.range_deriv, f.doppler_deriv) /
                std::max(THRESH_RANGE_DERIV, THRESH_DOPPLER_DERIV) / 2.0);
        }
        // Branch 3: Low power → blanking
        else if (f.power_ratio < THRESH_POWER_LOW) {
            r.env_class  = EnvironmentClass::NOISE_JAM;
            r.confidence = std::min(1.0,
                (THRESH_POWER_LOW - f.power_ratio) / THRESH_POWER_LOW);
        }
        // Default: clear
        else {
            r.env_class  = EnvironmentClass::CLEAR;
            r.confidence = 1.0 - std::min(0.5, std::abs(f.power_ratio - 1.0));
        }

        r.r_mult = r_multiplier(r.env_class);
        return r;
    }

    // ── Closed-loop update ──
    // [REQ-V620-ECCM-07] Updates TrackECCM state and returns R multiplier
    // Called from inside process_scan → IMM update chain
    static double update_track_eccm(TrackECCM& st, double nis,
                                    double innov_range, double innov_bearing,
                                    double speed) {
        Features f = extract_features(st, nis, innov_range, innov_bearing, speed);

        if (st.n_updates < MIN_SAMPLES) {
            st.current_class = EnvironmentClass::CLEAR;
            st.current_confidence = 0.5;
            st.current_r_mult = 1.0;
            return 1.0;
        }

        ClassifyResult cr = classify(f);
        st.current_class = cr.env_class;
        st.current_confidence = cr.confidence;
        st.current_r_mult = cr.r_mult;

        // Track consecutive jammed scans
        if (cr.env_class != EnvironmentClass::CLEAR) {
            st.n_jammed_scans++;
        } else {
            st.n_jammed_scans = 0;
        }

        return cr.r_mult;
    }
};


// ============================================================
// [REQ-V620-XCORR-01] CROSS-TRACK SECTOR CORRELATION
// Detects coherent threats: multiple jammed tracks in same sector
// → evidence of a single jammer affecting an angular sector
// ============================================================
struct SectorThreat {
    double az_center_rad;    // Sector center azimuth
    double az_width_rad;     // Sector angular width
    int n_jammed_tracks;     // Number of jammed tracks in sector
    EnvironmentClass dominant_class;
    double mean_confidence;
    bool coherent;           // True if ≥ coherence_threshold tracks affected
};

class CrossTrackCorrelator {
public:
    static constexpr double SECTOR_WIDTH_RAD  = 0.1745;   // 10° sector bins
    static constexpr int    COHERENCE_THRESH  = 3;         // ≥3 tracks → coherent
    static constexpr int    MAX_SECTORS       = 36;        // 360° / 10°

    struct SectorBin {
        int n_jammed       = 0;
        int n_noise_jam    = 0;
        int n_deception    = 0;
        int n_clutter      = 0;
        double sum_conf    = 0;
        std::vector<int> track_ids;  // IDs of jammed tracks in this sector

        void reset() {
            n_jammed = n_noise_jam = n_deception = n_clutter = 0;
            sum_conf = 0;
            track_ids.clear();
        }
    };

    // [REQ-V620-XCORR-02] Analyze all active tracks for sector correlation
    // Called once per scan after all per-track ECCM updates
    // Returns list of detected sector threats
    template<typename TrackArray>
    std::vector<SectorThreat> analyze(const TrackArray& tracks,
                                      const std::vector<int>& active_idx) {
        // Reset bins
        for (int i = 0; i < MAX_SECTORS; i++) bins_[i].reset();

        // Bin jammed tracks by azimuth sector
        for (int ai : active_idx) {
            const auto& trk = tracks[ai];
            if (trk.eccm.current_class == EnvironmentClass::CLEAR) continue;

            // Compute azimuth from position
            double az = std::atan2(trk.imm.position()[1],
                                   trk.imm.position()[0]);
            if (az < 0) az += 2.0 * M_PI;

            int sector = static_cast<int>(az / SECTOR_WIDTH_RAD) % MAX_SECTORS;
            auto& bin = bins_[sector];
            bin.n_jammed++;
            bin.sum_conf += trk.eccm.current_confidence;
            bin.track_ids.push_back(trk.id);

            switch (trk.eccm.current_class) {
                case EnvironmentClass::NOISE_JAM: bin.n_noise_jam++; break;
                case EnvironmentClass::DECEPTION: bin.n_deception++; break;
                case EnvironmentClass::CLUTTER:   bin.n_clutter++;   break;
                default: break;
            }
        }

        // Build threat list
        std::vector<SectorThreat> threats;
        for (int s = 0; s < MAX_SECTORS; s++) {
            auto& bin = bins_[s];
            if (bin.n_jammed < 2) continue;  // Need at least 2 for correlation

            SectorThreat st;
            st.az_center_rad = (s + 0.5) * SECTOR_WIDTH_RAD;
            st.az_width_rad  = SECTOR_WIDTH_RAD;
            st.n_jammed_tracks = bin.n_jammed;
            st.mean_confidence = bin.sum_conf / bin.n_jammed;
            st.coherent = (bin.n_jammed >= COHERENCE_THRESH);

            // Dominant class
            if (bin.n_noise_jam >= bin.n_deception && bin.n_noise_jam >= bin.n_clutter)
                st.dominant_class = EnvironmentClass::NOISE_JAM;
            else if (bin.n_deception >= bin.n_clutter)
                st.dominant_class = EnvironmentClass::DECEPTION;
            else
                st.dominant_class = EnvironmentClass::CLUTTER;

            threats.push_back(st);
        }

        // [REQ-V620-XCORR-03] Merge adjacent sectors if both have threats
        // Handles jammer straddling sector boundary
        std::vector<SectorThreat> merged;
        for (size_t i = 0; i < threats.size(); i++) {
            if (i + 1 < threats.size()) {
                double gap = std::abs(threats[i+1].az_center_rad -
                                      threats[i].az_center_rad);
                if (gap <= SECTOR_WIDTH_RAD * 1.5) {
                    // Merge
                    SectorThreat m;
                    m.az_center_rad = (threats[i].az_center_rad +
                                       threats[i+1].az_center_rad) / 2.0;
                    m.az_width_rad  = threats[i].az_width_rad +
                                      threats[i+1].az_width_rad;
                    m.n_jammed_tracks = threats[i].n_jammed_tracks +
                                        threats[i+1].n_jammed_tracks;
                    m.mean_confidence = (threats[i].mean_confidence *
                                         threats[i].n_jammed_tracks +
                                         threats[i+1].mean_confidence *
                                         threats[i+1].n_jammed_tracks) /
                                        m.n_jammed_tracks;
                    m.coherent = (m.n_jammed_tracks >= COHERENCE_THRESH);
                    m.dominant_class = threats[i].n_jammed_tracks >=
                                      threats[i+1].n_jammed_tracks
                                      ? threats[i].dominant_class
                                      : threats[i+1].dominant_class;
                    merged.push_back(m);
                    i++;  // skip next
                    continue;
                }
            }
            merged.push_back(threats[i]);
        }

        return merged;
    }

    // [REQ-V620-XCORR-04] Boost R multiplier for tracks in coherent threat sectors
    // Rationale: if 3+ tracks in a 10° sector are all classified as jammed,
    //            confidence is high → apply more aggressive R inflation
    template<typename TrackArray>
    int apply_coherent_boost(TrackArray& tracks,
                             const std::vector<int>& active_idx,
                             const std::vector<SectorThreat>& threats) {
        if (threats.empty()) return 0;

        int boosted = 0;
        for (const auto& threat : threats) {
            if (!threat.coherent) continue;

            double az_lo = threat.az_center_rad - threat.az_width_rad / 2.0;
            double az_hi = threat.az_center_rad + threat.az_width_rad / 2.0;

            for (int ai : active_idx) {
                auto& trk = tracks[ai];
                double az = std::atan2(trk.imm.position()[1],
                                       trk.imm.position()[0]);
                if (az < 0) az += 2.0 * M_PI;

                if (az >= az_lo && az <= az_hi) {
                    // Even CLEAR tracks in a coherent jammed sector get a bump
                    if (trk.eccm.current_class == EnvironmentClass::CLEAR) {
                        trk.eccm.current_r_mult = std::max(trk.eccm.current_r_mult, 1.5);
                    } else {
                        // Already jammed → extra 50% boost for coherent confirmation
                        trk.eccm.current_r_mult *= 1.5;
                        trk.eccm.current_r_mult = std::min(trk.eccm.current_r_mult, 20.0);
                    }
                    boosted++;
                }
            }
        }
        return boosted;
    }

    // Access sector data for reporting
    const SectorBin& sector(int idx) const { return bins_[idx]; }

private:
    SectorBin bins_[MAX_SECTORS];
};


// ============================================================
// ECCM SCAN STATISTICS (aggregate per-scan output)
// ============================================================
struct ScanECCMStats {
    int n_clear      = 0;
    int n_clutter    = 0;
    int n_noise_jam  = 0;
    int n_deception  = 0;
    int n_coherent_sectors = 0;
    int n_boosted_tracks   = 0;
    double mean_r_mult     = 1.0;
};

}  // namespace eccm
}  // namespace nx
