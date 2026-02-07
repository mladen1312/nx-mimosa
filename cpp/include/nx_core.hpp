// [REQ-V60-CPP-01] NX-MIMOSA v6.0 C++ Core — Sub-50ms at 1000+ tracks
// Copyright (c) 2026 Nexellum d.o.o. — AGPL-3.0-or-later
#pragma once

#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <limits>
#include <cassert>
#include <cstring>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace nx {

// ============================================================
// TYPE ALIASES
// ============================================================
using Vec3  = Eigen::Vector3d;
using Vec6  = Eigen::Matrix<double, 6, 1>;
using Vec7  = Eigen::Matrix<double, 7, 1>;
using Vec9  = Eigen::Matrix<double, 9, 1>;
using Mat3  = Eigen::Matrix3d;
using Mat6  = Eigen::Matrix<double, 6, 6>;
using Mat7  = Eigen::Matrix<double, 7, 7>;
using Mat9  = Eigen::Matrix<double, 9, 9>;
using Mat63 = Eigen::Matrix<double, 6, 3>;
using Mat73 = Eigen::Matrix<double, 7, 3>;
using Mat93 = Eigen::Matrix<double, 9, 3>;
using MatX  = Eigen::MatrixXd;
using VecX  = Eigen::VectorXd;

constexpr double INF_COST = 1e9;
constexpr int MAX_MODELS = 6;  // IMM: max sub-models

// ============================================================
// MOTION MODELS
// ============================================================
enum class MotionModel : uint8_t { CV3D = 0, CA3D = 1, CT3D_PLUS = 2, CT3D_MINUS = 3 };

inline int state_dim(MotionModel m) {
    switch (m) {
        case MotionModel::CV3D:       return 6;
        case MotionModel::CA3D:       return 9;
        case MotionModel::CT3D_PLUS:
        case MotionModel::CT3D_MINUS: return 7;
    }
    return 6;
}

// [REQ-V50-CV-01] Constant Velocity 3D: F,Q for dt
inline void make_cv3d(double dt, double q, Mat6& F, Mat6& Q) {
    F.setIdentity();
    F(0,3) = dt; F(1,4) = dt; F(2,5) = dt;

    double dt2 = dt*dt, dt3 = dt2*dt, dt4 = dt3*dt;
    Q.setZero();
    Q(0,0) = Q(1,1) = Q(2,2) = q * dt4 / 4.0;
    Q(3,3) = Q(4,4) = Q(5,5) = q * dt2;
    Q(0,3) = Q(3,0) = q * dt3 / 2.0;
    Q(1,4) = Q(4,1) = q * dt3 / 2.0;
    Q(2,5) = Q(5,2) = q * dt3 / 2.0;
}

// [REQ-V50-CA-01] Constant Acceleration 3D
inline void make_ca3d(double dt, double q, Mat9& F, Mat9& Q) {
    F.setIdentity();
    double dt2 = dt*dt / 2.0;
    F(0,3) = dt; F(1,4) = dt; F(2,5) = dt;
    F(0,6) = dt2; F(1,7) = dt2; F(2,8) = dt2;
    F(3,6) = dt;  F(4,7) = dt;  F(5,8) = dt;

    Q.setZero();
    double dt3 = dt*dt*dt, dt4 = dt3*dt, dt5 = dt4*dt;
    for (int i = 0; i < 3; ++i) {
        Q(i,   i)   = q * dt5 / 20.0;
        Q(i+3, i+3) = q * dt3 / 3.0;
        Q(i+6, i+6) = q * dt;
        Q(i, i+3) = Q(i+3, i) = q * dt4 / 8.0;
        Q(i, i+6) = Q(i+6, i) = q * dt3 / 6.0;
        Q(i+3, i+6) = Q(i+6, i+3) = q * dt2;
    }
}

// [REQ-V50-CT-01] Coordinated Turn 3D
inline void make_ct3d(double dt, double q, double omega, Mat7& F, Mat7& Q) {
    F.setIdentity();
    if (std::abs(omega) > 1e-6) {
        double sw = std::sin(omega * dt), cw = std::cos(omega * dt);
        F(0,3) = sw / omega;       F(0,4) = -(1.0 - cw) / omega;
        F(1,3) = (1.0 - cw) / omega; F(1,4) = sw / omega;
        F(3,3) = cw;  F(3,4) = -sw;
        F(4,3) = sw;  F(4,4) = cw;
    } else {
        F(0,3) = dt; F(1,4) = dt;
    }
    F(2,5) = dt;

    Q.setZero();
    double dt4 = dt*dt*dt*dt / 4.0, dt2 = dt*dt, dt3h = dt*dt*dt / 2.0;
    Q(0,0) = Q(1,1) = Q(2,2) = q * dt4;
    Q(3,3) = Q(4,4) = Q(5,5) = q * dt2;
    Q(0,3) = Q(3,0) = Q(1,4) = Q(4,1) = Q(2,5) = Q(5,2) = q * dt3h;
    Q(6,6) = q * 0.01 * dt;
}


// ============================================================
// GENERIC KF STATE (fixed max dimension, no heap per track)
// ============================================================
struct KFState {
    static constexpr int MAXN = 9;  // CA3D = 9 states
    static constexpr int NZ   = 3;  // measurement dim

    int nx;                         // actual state dim (6,7,9)
    MotionModel model;

    // State (stored in fixed arrays — avoid heap)
    double x[MAXN];
    double P[MAXN * MAXN];          // row-major
    double r_std;
    double q_base;
    double dt;

    // Eigen mapped accessors
    Eigen::Map<VecX> x_vec()             { return {x, nx}; }
    Eigen::Map<const VecX> x_vec() const { return {x, nx}; }
    Eigen::Map<MatX> P_mat()             { return {P, nx, nx}; }
    Eigen::Map<const MatX> P_mat() const { return {P, nx, nx}; }
    Vec3 position() const   { return {x[0], x[1], x[2]}; }
    Vec3 velocity() const   { return {x[3], x[4], x[5]}; }

    void init(int nx_, MotionModel m, double r, double q, double dt_) {
        nx = nx_; model = m; r_std = r; q_base = q; dt = dt_;
        std::memset(x, 0, sizeof(x));
        std::memset(P, 0, sizeof(P));
        auto Pm = P_mat();
        Pm.setIdentity();
        Pm *= r * r;
        if (nx >= 6) { Pm(3,3) = Pm(4,4) = Pm(5,5) = (r*10)*(r*10); }
        if (nx >= 7) { Pm(6,6) = 0.01; }
        if (nx >= 9) { Pm(6,6) = Pm(7,7) = Pm(8,8) = (r*20)*(r*20); }
    }

    void set_measurement(const Vec3& z) {
        x[0] = z[0]; x[1] = z[1]; x[2] = z[2];
    }

    // [REQ-V60-KF-PREDICT] In-place predict
    void predict() {
        auto xv = x_vec();
        auto Pv = P_mat();

        if (model == MotionModel::CV3D) {
            Mat6 F, Q; make_cv3d(dt, q_base, F, Q);
            Vec6 xs = F * xv.head<6>();
            Mat6 Ps = F * Pv.topLeftCorner<6,6>() * F.transpose() + Q;
            xv.head<6>() = xs;
            Pv.topLeftCorner<6,6>() = Ps;
        } else if (model == MotionModel::CA3D) {
            Mat9 F, Q; make_ca3d(dt, q_base * 0.5, F, Q);
            Vec9 xs = F * xv.head<9>();
            Mat9 Ps = F * Pv.topLeftCorner<9,9>() * F.transpose() + Q;
            xv.head<9>() = xs;
            Pv.topLeftCorner<9,9>() = Ps;
        } else {
            double omega = (nx >= 7) ? x[6] : 0.0;
            if (model == MotionModel::CT3D_MINUS)
                omega = (std::abs(omega) > 0.001) ? -std::abs(omega) : -0.05;
            else
                omega = (std::abs(omega) > 0.001) ? std::abs(omega) : 0.05;
            Mat7 F, Q; make_ct3d(dt, q_base, omega, F, Q);
            Vec7 xs = F * xv.head<7>();
            Mat7 Ps = F * Pv.topLeftCorner<7,7>() * F.transpose() + Q;
            xv.head<7>() = xs;
            Pv.topLeftCorner<7,7>() = Ps;
        }
    }

    // [REQ-V60-KF-UPDATE] Returns NIS
    double update(const Vec3& z) {
        auto xv = x_vec();
        auto Pv = P_mat();

        // H: 3 x nx (position-only)
        MatX H = MatX::Zero(3, nx);
        H(0,0) = 1.0; H(1,1) = 1.0; H(2,2) = 1.0;

        Mat3 R = Mat3::Identity() * r_std * r_std;
        Vec3 y = z - H * xv;
        Mat3 S = H * Pv * H.transpose() + R;

        Eigen::LLT<Mat3> llt(S);
        if (llt.info() != Eigen::Success) return 1e6;

        Mat3 S_inv = llt.solve(Mat3::Identity());
        double nis = y.transpose() * S_inv * y;

        MatX K = Pv * H.transpose() * S_inv;  // nx x 3
        xv += K * y;

        // Joseph form
        MatX I_KH = MatX::Identity(nx, nx) - K * H;
        Pv = I_KH * Pv * I_KH.transpose() + K * R * K.transpose();

        return nis;
    }

    // Mahalanobis distance (no combined_state overhead for single-model)
    double mahalanobis(const Vec3& z) const {
        Vec3 y = z - position();
        auto Pv = P_mat();
        Mat3 S = Pv.topLeftCorner<3,3>() + Mat3::Identity() * r_std * r_std;
        Eigen::LLT<Mat3> llt(S);
        if (llt.info() != Eigen::Success) return 1e6;
        Vec3 v = llt.solve(y);
        return y.dot(v);
    }
};


// ============================================================
// IMM FILTER (4 models, stack-allocated)
// ============================================================
struct IMMState {
    int n_models;
    KFState filters[MAX_MODELS];
    double mu[MAX_MODELS];
    double TPM[MAX_MODELS * MAX_MODELS];  // row-major transition matrix
    double r_std, q_base, dt;

    void init(double r, double q, double dt_, int n_mdl = 4) {
        r_std = r; q_base = q; dt = dt_; n_models = n_mdl;

        const MotionModel models[4] = {
            MotionModel::CV3D, MotionModel::CA3D,
            MotionModel::CT3D_PLUS, MotionModel::CT3D_MINUS
        };
        for (int i = 0; i < n_models; ++i) {
            filters[i].init(state_dim(models[i]), models[i], r, q, dt);
            mu[i] = 1.0 / n_models;
        }

        double p_stay = 0.90;
        double p_sw = (1.0 - p_stay) / (n_models - 1);
        for (int i = 0; i < n_models; ++i)
            for (int j = 0; j < n_models; ++j)
                TPM[i * n_models + j] = (i == j) ? p_stay : p_sw;
    }

    void set_measurement(const Vec3& z) {
        for (int i = 0; i < n_models; ++i)
            filters[i].set_measurement(z);
    }

    // [REQ-V60-IMM-PREDICT] Mixing + predict all sub-filters
    void predict() {
        // Mixing probabilities
        double c_bar[MAX_MODELS];
        for (int j = 0; j < n_models; ++j) {
            c_bar[j] = 0;
            for (int i = 0; i < n_models; ++i)
                c_bar[j] += TPM[i * n_models + j] * mu[i];
            if (c_bar[j] < 1e-30) c_bar[j] = 1e-30;
        }

        double mu_mix[MAX_MODELS * MAX_MODELS];
        for (int i = 0; i < n_models; ++i)
            for (int j = 0; j < n_models; ++j)
                mu_mix[i * n_models + j] = TPM[i * n_models + j] * mu[i] / c_bar[j];

        // Mix states for each target filter
        for (int j = 0; j < n_models; ++j) {
            int nx_j = filters[j].nx;
            VecX x_mix = VecX::Zero(nx_j);
            for (int i = 0; i < n_models; ++i) {
                int nc = std::min(filters[i].nx, nx_j);
                x_mix.head(nc) += mu_mix[i * n_models + j] * filters[i].x_vec().head(nc);
            }

            MatX P_mix = MatX::Zero(nx_j, nx_j);
            for (int i = 0; i < n_models; ++i) {
                int nc = std::min(filters[i].nx, nx_j);
                VecX dx = VecX::Zero(nx_j);
                dx.head(nc) = filters[i].x_vec().head(nc) - x_mix.head(nc);
                MatX P_i = MatX::Zero(nx_j, nx_j);
                P_i.topLeftCorner(nc, nc) = filters[i].P_mat().topLeftCorner(nc, nc);
                P_mix += mu_mix[i * n_models + j] * (P_i + dx * dx.transpose());
            }

            filters[j].x_vec() = x_mix;
            filters[j].P_mat() = P_mix;
            filters[j].predict();
        }
    }

    // [REQ-V60-IMM-UPDATE] Update all + reweight
    double update(const Vec3& z) {
        double likelihoods[MAX_MODELS];
        for (int j = 0; j < n_models; ++j) {
            auto& kf = filters[j];
            MatX H = MatX::Zero(3, kf.nx);
            H(0,0) = 1; H(1,1) = 1; H(2,2) = 1;
            Mat3 R = Mat3::Identity() * r_std * r_std;
            Vec3 y = z - H * kf.x_vec();
            Mat3 S = H * kf.P_mat() * H.transpose() + R;

            double det = S.determinant();
            if (det < 1e-30) { likelihoods[j] = 1e-30; }
            else {
                Eigen::LLT<Mat3> llt(S);
                Vec3 v = llt.solve(y);
                double exponent = -0.5 * y.dot(v);
                likelihoods[j] = std::exp(exponent) / std::sqrt(std::pow(2*M_PI, 3) * det);
            }
            kf.update(z);
        }

        // Model probability update
        double c_bar[MAX_MODELS];
        for (int j = 0; j < n_models; ++j) {
            c_bar[j] = 0;
            for (int i = 0; i < n_models; ++i)
                c_bar[j] += TPM[i * n_models + j] * mu[i];
        }
        double total = 0;
        for (int j = 0; j < n_models; ++j) {
            mu[j] = likelihoods[j] * c_bar[j];
            total += mu[j];
        }
        if (total > 0) {
            for (int j = 0; j < n_models; ++j) mu[j] /= total;
        } else {
            for (int j = 0; j < n_models; ++j) mu[j] = 1.0 / n_models;
        }
        return 0;
    }

    // [REQ-V60-IMM-COMBINE] Combined state (6D) — cached-friendly
    void combined_state(Vec6& x_out, Mat6& P_out) const {
        x_out.setZero();
        for (int j = 0; j < n_models; ++j)
            x_out += mu[j] * filters[j].x_vec().head<6>();

        P_out.setZero();
        for (int j = 0; j < n_models; ++j) {
            Vec6 dx = filters[j].x_vec().head<6>() - x_out;
            P_out += mu[j] * (filters[j].P_mat().topLeftCorner<6,6>() + dx * dx.transpose());
        }
    }

    Vec3 position() const {
        Vec3 p = Vec3::Zero();
        for (int j = 0; j < n_models; ++j)
            p += mu[j] * filters[j].position();
        return p;
    }

    // Mahalanobis using combined state
    double mahalanobis(const Vec3& z) const {
        Vec6 xc; Mat6 Pc;
        combined_state(xc, Pc);
        Vec3 y = z - xc.head<3>();
        Mat3 S = Pc.topLeftCorner<3,3>() + Mat3::Identity() * r_std * r_std;
        Eigen::LLT<Mat3> llt(S);
        if (llt.info() != Eigen::Success) return 1e6;
        Vec3 v = llt.solve(y);
        return y.dot(v);
    }
};


// ============================================================
// TRACK STATUS & STATE
// ============================================================
enum class TrackStatus : uint8_t { TENTATIVE=0, CONFIRMED=1, COASTING=2, DELETED=3 };

struct Track {
    int id;
    TrackStatus status;
    IMMState imm;

    int hit_count;
    int miss_count;
    int total_updates;
    int age;
    double last_timestamp;

    void init(int id_, double r_std, double q_base, double dt, const Vec3& z, double ts) {
        id = id_; status = TrackStatus::TENTATIVE;
        hit_count = 1; miss_count = 0; total_updates = 1; age = 0;
        last_timestamp = ts;
        imm.init(r_std, q_base, dt);
        imm.set_measurement(z);
    }
};


// ============================================================
// TRACK MANAGER CONFIG
// ============================================================
struct TrackerConfig {
    double dt = 5.0;
    double r_std = 150.0;
    double q_base = 1.0;
    double gate_threshold = 16.0;    // chi2 gate
    double coarse_gate_m = 20000.0;  // Euclidean pre-gate
    double min_separation = 500.0;

    int confirm_hits = 3;
    int confirm_window = 5;
    int delete_misses = 7;
    int coast_max = 5;
};


// ============================================================
// KD-TREE (lightweight, header-only, 3D)
// ============================================================
struct KDNode {
    int idx;
    int split_dim;
    double split_val;
    int left, right;  // indices into node pool
};

class KDTree3 {
public:
    std::vector<KDNode> nodes;
    const double* pts;  // Nx3 row-major
    int n;

    void build(const double* points, int count) {
        pts = points; n = count;
        nodes.clear();
        if (n == 0) return;
        nodes.reserve(2 * n);
        std::vector<int> indices(n);
        std::iota(indices.begin(), indices.end(), 0);
        build_recursive(indices.data(), n, 0);
    }

    void query_ball(const double* q, double radius, std::vector<int>& result) const {
        if (nodes.empty()) return;
        double r2 = radius * radius;
        query_recursive(0, q, r2, result);
    }

private:
    int build_recursive(int* idx, int count, int depth) {
        if (count <= 0) return -1;
        int dim = depth % 3;
        int mid = count / 2;
        std::nth_element(idx, idx + mid, idx + count, [&](int a, int b) {
            return pts[a * 3 + dim] < pts[b * 3 + dim];
        });

        int node_idx = (int)nodes.size();
        nodes.push_back({idx[mid], dim, pts[idx[mid] * 3 + dim], -1, -1});

        if (mid > 0)
            nodes[node_idx].left = build_recursive(idx, mid, depth + 1);
        if (mid + 1 < count)
            nodes[node_idx].right = build_recursive(idx + mid + 1, count - mid - 1, depth + 1);

        return node_idx;
    }

    void query_recursive(int ni, const double* q, double r2, std::vector<int>& result) const {
        if (ni < 0) return;
        const auto& node = nodes[ni];
        const double* p = pts + node.idx * 3;

        double d2 = 0;
        for (int i = 0; i < 3; ++i) { double d = q[i] - p[i]; d2 += d*d; }
        if (d2 <= r2) result.push_back(node.idx);

        double diff = q[node.split_dim] - node.split_val;
        int near = (diff <= 0) ? node.left : node.right;
        int far  = (diff <= 0) ? node.right : node.left;

        query_recursive(near, q, r2, result);
        if (diff * diff <= r2) query_recursive(far, q, r2, result);
    }
};


// ============================================================
// LAPJV — Jonker-Volgenant Linear Assignment (O(n²) avg)
// Adapted from Crouse 2016 / scipy C implementation
// ============================================================
namespace lapjv {

inline void solve(const double* cost, int n_rows, int n_cols,
                  int* row_assign, int* col_assign) {
    // Simplified auction / shortest-augmenting-path for sparse problems
    // For production: use full Jonker-Volgenant.
    // Here: scipy-compatible via row reduction + augmentation.

    std::vector<double> u(n_rows, 0), v(n_cols, 0);  // dual variables
    std::fill(row_assign, row_assign + n_rows, -1);
    std::fill(col_assign, col_assign + n_cols, -1);

    // Column reduction
    for (int j = 0; j < n_cols; ++j) {
        double min_val = INF_COST;
        int min_row = -1;
        for (int i = 0; i < n_rows; ++i) {
            double c = cost[i * n_cols + j];
            if (c < min_val) { min_val = c; min_row = i; }
        }
        v[j] = min_val;
        if (min_row >= 0 && row_assign[min_row] < 0) {
            row_assign[min_row] = j;
            col_assign[j] = min_row;
        }
    }

    // Augmentation for unassigned rows
    for (int i = 0; i < n_rows; ++i) {
        if (row_assign[i] >= 0) continue;

        // Find shortest augmenting path (Dijkstra-like)
        std::vector<double> d(n_cols, INF_COST);
        std::vector<int> pred(n_cols, -1);
        std::vector<bool> scanned(n_cols, false);

        for (int j = 0; j < n_cols; ++j) {
            d[j] = cost[i * n_cols + j] - u[i] - v[j];
            pred[j] = i;
        }

        int j_sink = -1;
        while (j_sink < 0) {
            // Find unscanned col with min d
            double min_d = INF_COST;
            int j_min = -1;
            for (int j = 0; j < n_cols; ++j) {
                if (!scanned[j] && d[j] < min_d) { min_d = d[j]; j_min = j; }
            }
            if (j_min < 0 || min_d >= INF_COST - 1) break;

            scanned[j_min] = true;
            if (col_assign[j_min] < 0) {
                j_sink = j_min;
            } else {
                int i2 = col_assign[j_min];
                for (int j = 0; j < n_cols; ++j) {
                    if (scanned[j]) continue;
                    double new_d = min_d + cost[i2 * n_cols + j] - u[i2] - v[j];
                    if (new_d < d[j]) {
                        d[j] = new_d;
                        pred[j] = i2;
                    }
                }
            }
        }

        // Update dual variables
        for (int j = 0; j < n_cols; ++j) {
            if (scanned[j]) {
                double delta = d[j] - d[j_sink >= 0 ? j_sink : 0];
                u[col_assign[j] >= 0 ? col_assign[j] : i] += delta;
                v[j] -= delta;
            }
        }

        // Augment along path
        if (j_sink >= 0) {
            int j = j_sink;
            while (true) {
                int i2 = pred[j];
                col_assign[j] = i2;
                int prev_j = row_assign[i2];
                row_assign[i2] = j;
                if (i2 == i) break;
                j = prev_j;
            }
        }
    }
}

}  // namespace lapjv


// ============================================================
// MULTI-TARGET TRACKER
// ============================================================
class MultiTargetTracker {
public:
    TrackerConfig cfg;
    std::vector<Track> tracks;
    int next_id = 0;
    int step = 0;
    double timestamp = 0;

    // Stats
    int total_created = 0;
    int total_deleted = 0;

    MultiTargetTracker() = default;
    explicit MultiTargetTracker(const TrackerConfig& c) : cfg(c) {}

    // [REQ-V60-SCAN-01] Main processing pipeline — target <50ms for 1000 tracks
    void process_scan(const double* measurements, int n_meas, double ts = -1) {
        if (ts < 0) ts = step * cfg.dt;
        timestamp = ts;

        // Collect active tracks
        std::vector<int> active_idx;
        active_idx.reserve(tracks.size());
        for (int i = 0; i < (int)tracks.size(); ++i)
            if (tracks[i].status != TrackStatus::DELETED)
                active_idx.push_back(i);

        int n_tracks = (int)active_idx.size();

        // === 1. BATCH PREDICT (parallelized) ===
        #pragma omp parallel for schedule(static) if(n_tracks > 100)
        for (int i = 0; i < n_tracks; ++i)
            tracks[active_idx[i]].imm.predict();

        // === 2. ASSOCIATION (KDTree + sparse Mahalanobis + LAPJV) ===
        std::vector<std::pair<int,int>> assignments;
        std::vector<int> unassigned_tracks, unassigned_meas;

        if (n_tracks > 0 && n_meas > 0) {
            gnn_associate(active_idx, measurements, n_meas,
                          assignments, unassigned_tracks, unassigned_meas);
        } else {
            for (int i = 0; i < n_tracks; ++i) unassigned_tracks.push_back(i);
            for (int j = 0; j < n_meas; ++j) unassigned_meas.push_back(j);
        }

        // === 3. BATCH UPDATE assigned tracks ===
        for (auto& [ti, mi] : assignments) {
            auto& trk = tracks[active_idx[ti]];
            Vec3 z(measurements[mi*3], measurements[mi*3+1], measurements[mi*3+2]);
            trk.imm.update(z);
            update_hit(trk, ts);
        }

        // === 4. MISS unassigned tracks ===
        for (int ti : unassigned_tracks)
            update_miss(tracks[active_idx[ti]]);

        // === 5. INITIATE new tracks ===
        for (int mi : unassigned_meas) {
            Vec3 z(measurements[mi*3], measurements[mi*3+1], measurements[mi*3+2]);
            // Check min separation
            bool too_close = false;
            for (int ai : active_idx) {
                if (tracks[ai].status == TrackStatus::DELETED) continue;
                double d = (z - tracks[ai].imm.position()).norm();
                if (d < cfg.min_separation) { too_close = true; break; }
            }
            if (!too_close) {
                Track t;
                t.init(next_id++, cfg.r_std, cfg.q_base, cfg.dt, z, ts);
                tracks.push_back(std::move(t));
                total_created++;
            }
        }

        // === 6. AGE & DELETE ===
        for (int ai : active_idx) {
            auto& trk = tracks[ai];
            trk.age++;
            if (trk.status == TrackStatus::DELETED) continue;

            if (trk.miss_count >= cfg.delete_misses) {
                trk.status = TrackStatus::DELETED;
                total_deleted++;
            } else if (trk.miss_count >= cfg.coast_max && trk.status == TrackStatus::CONFIRMED) {
                trk.status = TrackStatus::COASTING;
            }
        }

        step++;
    }

    // Get confirmed tracks
    std::vector<int> confirmed_indices() const {
        std::vector<int> result;
        for (int i = 0; i < (int)tracks.size(); ++i)
            if (tracks[i].status == TrackStatus::CONFIRMED)
                result.push_back(i);
        return result;
    }

private:
    void gnn_associate(const std::vector<int>& active_idx,
                       const double* meas, int n_meas,
                       std::vector<std::pair<int,int>>& assignments,
                       std::vector<int>& unassigned_trk,
                       std::vector<int>& unassigned_meas) {
        int n_trk = (int)active_idx.size();

        // Build KDTree on measurements
        KDTree3 tree;
        tree.build(meas, n_meas);

        // Precompute track positions
        std::vector<Vec3> track_pos(n_trk);
        for (int i = 0; i < n_trk; ++i)
            track_pos[i] = tracks[active_idx[i]].imm.position();

        // Sparse candidate pairs via KDTree
        struct Pair { int ti, mi; };
        std::vector<Pair> candidates;
        candidates.reserve(n_trk * 4);  // expect ~4 candidates per track

        for (int i = 0; i < n_trk; ++i) {
            double q[3] = {track_pos[i][0], track_pos[i][1], track_pos[i][2]};
            std::vector<int> nearby;
            tree.query_ball(q, cfg.coarse_gate_m, nearby);
            for (int j : nearby)
                candidates.push_back({i, j});
        }

        // Build cost matrix (dense, but only compute Mahalanobis for candidates)
        std::vector<double> cost(n_trk * n_meas, INF_COST);

        #pragma omp parallel for schedule(dynamic) if(candidates.size() > 1000)
        for (int c = 0; c < (int)candidates.size(); ++c) {
            int ti = candidates[c].ti;
            int mi = candidates[c].mi;
            Vec3 z(meas[mi*3], meas[mi*3+1], meas[mi*3+2]);
            double d2 = tracks[active_idx[ti]].imm.mahalanobis(z);
            if (d2 < cfg.gate_threshold)
                cost[ti * n_meas + mi] = d2;
        }

        // Solve assignment
        std::vector<int> row_assign(n_trk, -1), col_assign(n_meas, -1);
        lapjv::solve(cost.data(), n_trk, n_meas, row_assign.data(), col_assign.data());

        // Filter gated assignments
        std::vector<bool> trk_assigned(n_trk, false), meas_assigned(n_meas, false);
        for (int i = 0; i < n_trk; ++i) {
            int j = row_assign[i];
            if (j >= 0 && j < n_meas && cost[i * n_meas + j] < cfg.gate_threshold) {
                assignments.push_back({i, j});
                trk_assigned[i] = true;
                meas_assigned[j] = true;
            }
        }
        for (int i = 0; i < n_trk; ++i)
            if (!trk_assigned[i]) unassigned_trk.push_back(i);
        for (int j = 0; j < n_meas; ++j)
            if (!meas_assigned[j]) unassigned_meas.push_back(j);
    }

    void update_hit(Track& t, double ts) {
        t.hit_count++;
        t.miss_count = 0;
        t.total_updates++;
        t.last_timestamp = ts;

        if (t.status == TrackStatus::TENTATIVE && t.hit_count >= cfg.confirm_hits)
            t.status = TrackStatus::CONFIRMED;
        else if (t.status == TrackStatus::COASTING)
            t.status = TrackStatus::CONFIRMED;
    }

    void update_miss(Track& t) {
        t.miss_count++;
        t.total_updates++;
    }
};

}  // namespace nx
