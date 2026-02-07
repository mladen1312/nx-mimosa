// [REQ-V620-CORE-01] NX-MIMOSA v6.2.0 C++ Core — ECCM-Integrated Tracking
// Copyright (c) 2026 Nexellum d.o.o. — AGPL-3.0-or-later
//
// v6.2.0 DELTA vs v6.1.0:
//   1. TrackECCM embedded in Track struct (zero extra allocation)
//   2. ECCM classify runs INSIDE process_scan after IMM update
//   3. Closed-loop: R inflation applied BEFORE next IMM update
//   4. Cross-track sector correlation after all per-track updates
//   5. ML window = 20 (was 10)
//
// Performance target: <60ms @ 5K targets (ECCM adds ~10% overhead)
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
#include "nx_eccm.hpp"

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
constexpr int MAX_MODELS = 6;

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
// KF STATE (stack-allocated, max 9D)
// ============================================================
struct KFState {
    static constexpr int MAXN = 9;
    static constexpr int NZ   = 3;

    int nx;
    MotionModel model;
    double x[MAXN];
    double P[MAXN * MAXN];
    double r_std;
    double q_base;
    double dt;

    // [REQ-V620-CLOSED-01] Effective R std — inflated by ECCM
    // This is the key closed-loop mechanism:
    // r_effective = r_std * eccm_r_mult
    double r_effective() const { return r_std * eccm_r_mult_; }
    double eccm_r_mult_ = 1.0;  // Set by ECCM before update

    Eigen::Map<VecX> x_vec()             { return {x, nx}; }
    Eigen::Map<const VecX> x_vec() const { return {x, nx}; }
    Eigen::Map<MatX> P_mat()             { return {P, nx, nx}; }
    Eigen::Map<const MatX> P_mat() const { return {P, nx, nx}; }
    Vec3 position() const   { return {x[0], x[1], x[2]}; }
    Vec3 velocity() const   { return {x[3], x[4], x[5]}; }

    void init(int nx_, MotionModel m, double r, double q, double dt_) {
        nx = nx_; model = m; r_std = r; q_base = q; dt = dt_;
        eccm_r_mult_ = 1.0;
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

    // [REQ-V620-CLOSED-02] Update uses r_effective() for R matrix
    // This is where closed-loop ECCM takes effect
    double update(const Vec3& z) {
        auto xv = x_vec();
        auto Pv = P_mat();
        MatX H = MatX::Zero(3, nx);
        H(0,0) = 1.0; H(1,1) = 1.0; H(2,2) = 1.0;

        double r_eff = r_effective();
        Mat3 R = Mat3::Identity() * r_eff * r_eff;  // <-- ECCM-inflated R
        Vec3 y = z - H * xv;
        Mat3 S = H * Pv * H.transpose() + R;

        Eigen::LLT<Mat3> llt(S);
        if (llt.info() != Eigen::Success) return 1e6;

        Mat3 S_inv = llt.solve(Mat3::Identity());
        double nis = y.transpose() * S_inv * y;

        MatX K = Pv * H.transpose() * S_inv;
        xv += K * y;

        // Joseph form
        MatX I_KH = MatX::Identity(nx, nx) - K * H;
        Pv = I_KH * Pv * I_KH.transpose() + K * R * K.transpose();

        return nis;
    }

    double mahalanobis(const Vec3& z) const {
        Vec3 y = z - position();
        auto Pv = P_mat();
        double r_eff = r_std * eccm_r_mult_;
        Mat3 S = Pv.topLeftCorner<3,3>() + Mat3::Identity() * r_eff * r_eff;
        Eigen::LLT<Mat3> llt(S);
        if (llt.info() != Eigen::Success) return 1e6;
        Vec3 v = llt.solve(y);
        return y.dot(v);
    }
};


// ============================================================
// IMM FILTER
// ============================================================
struct IMMState {
    int n_models;
    KFState filters[MAX_MODELS];
    double mu[MAX_MODELS];
    double TPM[MAX_MODELS * MAX_MODELS];
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

    // [REQ-V620-CLOSED-03] Set ECCM R multiplier on ALL sub-filters
    void set_eccm_r_mult(double mult) {
        for (int i = 0; i < n_models; ++i)
            filters[i].eccm_r_mult_ = mult;
    }

    void predict() {
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

    double update(const Vec3& z) {
        double likelihoods[MAX_MODELS];
        for (int j = 0; j < n_models; ++j) {
            auto& kf = filters[j];
            MatX H = MatX::Zero(3, kf.nx);
            H(0,0) = 1; H(1,1) = 1; H(2,2) = 1;
            double r_eff = kf.r_effective();
            Mat3 R = Mat3::Identity() * r_eff * r_eff;
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

    double mahalanobis(const Vec3& z) const {
        Vec6 xc; Mat6 Pc;
        combined_state(xc, Pc);
        Vec3 y = z - xc.head<3>();
        double r_eff = r_std * filters[0].eccm_r_mult_;
        Mat3 S = Pc.topLeftCorner<3,3>() + Mat3::Identity() * r_eff * r_eff;
        Eigen::LLT<Mat3> llt(S);
        if (llt.info() != Eigen::Success) return 1e6;
        Vec3 v = llt.solve(y);
        return y.dot(v);
    }
};


// ============================================================
// TRACK (now with embedded ECCM state)
// ============================================================
enum class TrackStatus : uint8_t { TENTATIVE=0, CONFIRMED=1, COASTING=2, DELETED=3 };

struct Track {
    int id;
    TrackStatus status;
    IMMState imm;
    eccm::TrackECCM eccm;  // [REQ-V620-TRACK-01] Embedded ECCM state

    int hit_count;
    int miss_count;
    int total_updates;
    int age;
    double last_timestamp;

    // Last scan innovation (for ECCM feature extraction)
    double last_nis = 0;
    Vec3 last_innovation = Vec3::Zero();

    void init(int id_, double r_std, double q_base, double dt, const Vec3& z, double ts) {
        id = id_; status = TrackStatus::TENTATIVE;
        hit_count = 1; miss_count = 0; total_updates = 1; age = 0;
        last_timestamp = ts;
        imm.init(r_std, q_base, dt);
        imm.set_measurement(z);
        eccm.reset();  // Fresh ECCM state
        last_nis = 0;
        last_innovation.setZero();
    }
};


// ============================================================
// TRACKER CONFIG
// ============================================================
struct TrackerConfig {
    double dt = 5.0;
    double r_std = 150.0;
    double q_base = 1.0;
    double gate_threshold = 16.0;
    double coarse_gate_m = 20000.0;
    double min_separation = 500.0;
    int confirm_hits = 3;
    int confirm_window = 5;
    int delete_misses = 7;
    int coast_max = 5;

    // [REQ-V620-CFG-01] ECCM configuration
    bool eccm_enabled = true;            // Master enable
    bool eccm_cross_track = true;        // Cross-track correlation
};


// ============================================================
// KD-TREE (3D, header-only)
// ============================================================
struct KDNode {
    int idx; int split_dim; double split_val; int left, right;
};

class KDTree3 {
public:
    std::vector<KDNode> nodes;
    const double* pts;
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
// SPARSE AUCTION ASSIGNMENT
// ============================================================
namespace assignment {

inline void sparse_auction(const std::vector<std::vector<std::pair<int,double>>>& row_candidates,
                           int n_rows, int n_cols,
                           int* row_assign, int* col_assign,
                           double epsilon = 0.01, int max_iter = 200) {
    std::fill(row_assign, row_assign + n_rows, -1);
    std::fill(col_assign, col_assign + n_cols, -1);
    std::vector<double> prices(n_cols, 0.0);
    std::vector<int> unassigned;
    unassigned.reserve(n_rows);
    for (int i = 0; i < n_rows; ++i)
        if (!row_candidates[i].empty()) unassigned.push_back(i);

    for (int iter = 0; iter < max_iter && !unassigned.empty(); ++iter) {
        std::vector<int> still_unassigned;
        still_unassigned.reserve(unassigned.size());
        for (int i : unassigned) {
            const auto& cands = row_candidates[i];
            if (cands.empty()) continue;
            double best_val = -INF_COST, second_val = -INF_COST;
            int best_j = -1;
            for (auto& [j, c] : cands) {
                double val = -c - prices[j];
                if (val > best_val) { second_val = best_val; best_val = val; best_j = j; }
                else if (val > second_val) { second_val = val; }
            }
            if (best_j < 0) continue;
            double bid = best_val - second_val + epsilon;
            prices[best_j] += bid;
            int prev_owner = col_assign[best_j];
            if (prev_owner >= 0) { row_assign[prev_owner] = -1; still_unassigned.push_back(prev_owner); }
            row_assign[i] = best_j;
            col_assign[best_j] = i;
        }
        unassigned = std::move(still_unassigned);
        if (iter > 0 && iter % 50 == 0) epsilon *= 0.5;
    }
}

inline void dense_solve(const double* cost, int n_rows, int n_cols,
                        int* row_assign, int* col_assign) {
    std::vector<double> u(n_rows, 0), v(n_cols, 0);
    std::fill(row_assign, row_assign + n_rows, -1);
    std::fill(col_assign, col_assign + n_cols, -1);
    for (int j = 0; j < n_cols; ++j) {
        double min_val = INF_COST; int min_row = -1;
        for (int i = 0; i < n_rows; ++i) {
            double c = cost[i * n_cols + j];
            if (c < min_val) { min_val = c; min_row = i; }
        }
        v[j] = min_val;
        if (min_row >= 0 && row_assign[min_row] < 0) {
            row_assign[min_row] = j; col_assign[j] = min_row;
        }
    }
    for (int i = 0; i < n_rows; ++i) {
        if (row_assign[i] >= 0) continue;
        std::vector<double> d(n_cols, INF_COST);
        std::vector<int> pred(n_cols, -1);
        std::vector<bool> scanned(n_cols, false);
        for (int j = 0; j < n_cols; ++j) { d[j] = cost[i*n_cols+j]-u[i]-v[j]; pred[j]=i; }
        int j_sink = -1;
        while (j_sink < 0) {
            double min_d = INF_COST; int j_min = -1;
            for (int j = 0; j < n_cols; ++j)
                if (!scanned[j] && d[j] < min_d) { min_d = d[j]; j_min = j; }
            if (j_min < 0 || min_d >= INF_COST - 1) break;
            scanned[j_min] = true;
            if (col_assign[j_min] < 0) { j_sink = j_min; }
            else {
                int i2 = col_assign[j_min];
                for (int j = 0; j < n_cols; ++j) {
                    if (scanned[j]) continue;
                    double nd = min_d + cost[i2*n_cols+j] - u[i2] - v[j];
                    if (nd < d[j]) { d[j] = nd; pred[j] = i2; }
                }
            }
        }
        if (j_sink >= 0) {
            for (int j = 0; j < n_cols; ++j)
                if (scanned[j]) { double delta = d[j]-d[j_sink]; v[j]-=delta; }
            int j = j_sink;
            while (true) {
                int i2 = pred[j]; col_assign[j]=i2; int pj=row_assign[i2]; row_assign[i2]=j;
                if (i2==i) break; j=pj;
            }
        }
    }
}

}  // namespace assignment


// ============================================================
// [REQ-V620-MTT-01] MULTI-TARGET TRACKER — ECCM-Integrated
// ============================================================
class MultiTargetTracker {
public:
    TrackerConfig cfg;
    std::vector<Track> tracks;
    int next_id = 0;
    int step = 0;
    double timestamp = 0;
    int total_created = 0;
    int total_deleted = 0;

    // [REQ-V620-MTT-02] Per-scan ECCM statistics
    eccm::ScanECCMStats last_eccm_stats;

    // Cross-track correlator (persistent across scans)
    eccm::CrossTrackCorrelator cross_correlator;

    MultiTargetTracker() = default;
    explicit MultiTargetTracker(const TrackerConfig& c) : cfg(c) {}

    // ═══════════════════════════════════════════════════════════
    // [REQ-V620-SCAN-01] Main processing pipeline
    // v6.2.0: ECCM integrated in-line
    //
    // Pipeline order:
    //   1. Predict (IMM mixing + predict, with ECCM R from PREVIOUS scan)
    //   2. Associate (KDTree + sparse Mahalanobis + auction)
    //   3. Update + ECCM classify (per-track, captures innovation/NIS)
    //   4. Cross-track correlation (sector-wide coherent threat)
    //   5. Set R multipliers for NEXT scan predict/update cycle
    //   6. Track management (init/confirm/coast/delete)
    // ═══════════════════════════════════════════════════════════
    void process_scan(const double* measurements, int n_meas, double ts = -1) {
        if (ts < 0) ts = step * cfg.dt;
        timestamp = ts;

        std::vector<int> active_idx;
        active_idx.reserve(tracks.size());
        for (int i = 0; i < (int)tracks.size(); ++i)
            if (tracks[i].status != TrackStatus::DELETED)
                active_idx.push_back(i);

        int n_tracks = (int)active_idx.size();

        // ═══ 1. BATCH PREDICT ═══
        // R multiplier from previous scan's ECCM is already set on each sub-filter
        #pragma omp parallel for schedule(static) if(n_tracks > 100)
        for (int i = 0; i < n_tracks; ++i)
            tracks[active_idx[i]].imm.predict();

        // ═══ 2. ASSOCIATION ═══
        std::vector<std::pair<int,int>> assignments;
        std::vector<int> unassigned_tracks, unassigned_meas;

        if (n_tracks > 0 && n_meas > 0) {
            gnn_associate(active_idx, measurements, n_meas,
                          assignments, unassigned_tracks, unassigned_meas);
        } else {
            for (int i = 0; i < n_tracks; ++i) unassigned_tracks.push_back(i);
            for (int j = 0; j < n_meas; ++j) unassigned_meas.push_back(j);
        }

        // ═══ 3. UPDATE + ECCM CLASSIFY (per-track) ═══
        // Reset ECCM stats
        last_eccm_stats = {};

        for (auto& [ti, mi] : assignments) {
            auto& trk = tracks[active_idx[ti]];
            Vec3 z(measurements[mi*3], measurements[mi*3+1], measurements[mi*3+2]);

            // Compute innovation BEFORE update (for ECCM feature extraction)
            Vec3 predicted_pos = trk.imm.position();
            Vec3 innovation = z - predicted_pos;
            // Get velocity from IMM combined state for speed estimate
            Vec6 xc; Mat6 Pc;
            trk.imm.combined_state(xc, Pc);
            double speed = std::sqrt(xc[3]*xc[3] + xc[4]*xc[4]);

            // IMM update (uses current R multiplier)
            trk.imm.update(z);

            // Compute NIS post-update from innovation and S
            double nis = innovation.squaredNorm() /
                         std::max(trk.imm.r_std * trk.imm.r_std * trk.eccm.current_r_mult, 1.0);

            // Store for ECCM
            trk.last_nis = nis;
            trk.last_innovation = innovation;

            // [REQ-V620-CLOSED-04] Run ECCM classification
            if (cfg.eccm_enabled && trk.status == TrackStatus::CONFIRMED) {
                double r_mult = eccm::MLCFARDetector::update_track_eccm(
                    trk.eccm, nis,
                    innovation[0],  // range innovation
                    innovation[1],  // bearing innovation
                    speed
                );

                // [REQ-V620-CLOSED-05] Apply R multiplier to all IMM sub-filters
                // Takes effect on NEXT scan's predict + update
                trk.imm.set_eccm_r_mult(r_mult);

                // Accumulate stats
                switch (trk.eccm.current_class) {
                    case eccm::EnvironmentClass::CLEAR:     last_eccm_stats.n_clear++;     break;
                    case eccm::EnvironmentClass::CLUTTER:   last_eccm_stats.n_clutter++;   break;
                    case eccm::EnvironmentClass::NOISE_JAM: last_eccm_stats.n_noise_jam++; break;
                    case eccm::EnvironmentClass::DECEPTION: last_eccm_stats.n_deception++;  break;
                }
            }

            update_hit(trk, ts);
        }

        // ═══ 4. CROSS-TRACK CORRELATION ═══
        if (cfg.eccm_enabled && cfg.eccm_cross_track) {
            auto threats = cross_correlator.analyze(tracks, active_idx);
            if (!threats.empty()) {
                last_eccm_stats.n_coherent_sectors = 0;
                for (const auto& t : threats)
                    if (t.coherent) last_eccm_stats.n_coherent_sectors++;

                // [REQ-V620-XCORR-05] Apply coherent boost
                last_eccm_stats.n_boosted_tracks =
                    cross_correlator.apply_coherent_boost(tracks, active_idx, threats);
            }
        }

        // Compute mean R multiplier
        {
            double sum_r = 0; int cnt = 0;
            for (int ai : active_idx) {
                if (tracks[ai].status == TrackStatus::CONFIRMED) {
                    sum_r += tracks[ai].eccm.current_r_mult;
                    cnt++;
                }
            }
            last_eccm_stats.mean_r_mult = (cnt > 0) ? sum_r / cnt : 1.0;
        }

        // ═══ 5. MISS unassigned tracks ═══
        for (int ti : unassigned_tracks)
            update_miss(tracks[active_idx[ti]]);

        // ═══ 6. INITIATE new tracks ═══
        for (int mi : unassigned_meas) {
            Vec3 z(measurements[mi*3], measurements[mi*3+1], measurements[mi*3+2]);
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

        // ═══ 7. AGE & DELETE ═══
        for (int ai : active_idx) {
            auto& trk = tracks[ai];
            trk.age++;
            if (trk.status == TrackStatus::DELETED) continue;
            if (trk.miss_count >= cfg.delete_misses) {
                trk.status = TrackStatus::DELETED;
                total_deleted++;
                trk.eccm.reset();  // Free ECCM buffers
            } else if (trk.miss_count >= cfg.coast_max &&
                       trk.status == TrackStatus::CONFIRMED) {
                trk.status = TrackStatus::COASTING;
            }
        }

        step++;
    }

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
        KDTree3 tree;
        tree.build(meas, n_meas);

        std::vector<Vec3> track_pos(n_trk);
        for (int i = 0; i < n_trk; ++i)
            track_pos[i] = tracks[active_idx[i]].imm.position();

        std::vector<std::vector<std::pair<int,double>>> row_candidates(n_trk);

        #pragma omp parallel for schedule(dynamic) if(n_trk > 200)
        for (int i = 0; i < n_trk; ++i) {
            double q[3] = {track_pos[i][0], track_pos[i][1], track_pos[i][2]};
            std::vector<int> nearby;
            tree.query_ball(q, cfg.coarse_gate_m, nearby);
            for (int j : nearby) {
                Vec3 z(meas[j*3], meas[j*3+1], meas[j*3+2]);
                double d2 = tracks[active_idx[i]].imm.mahalanobis(z);
                if (d2 < cfg.gate_threshold)
                    row_candidates[i].emplace_back(j, d2);
            }
        }

        std::vector<int> row_assign(n_trk, -1), col_assign(n_meas, -1);
        size_t total_cands = 0;
        for (auto& rc : row_candidates) total_cands += rc.size();
        size_t dense_size = (size_t)n_trk * n_meas;

        if (total_cands < dense_size / 4 || n_trk > 500) {
            assignment::sparse_auction(row_candidates, n_trk, n_meas,
                                       row_assign.data(), col_assign.data());
        } else {
            std::vector<double> cost(dense_size, INF_COST);
            for (int i = 0; i < n_trk; ++i)
                for (auto& [j, c] : row_candidates[i])
                    cost[i * n_meas + j] = c;
            assignment::dense_solve(cost.data(), n_trk, n_meas,
                                    row_assign.data(), col_assign.data());
        }

        std::vector<bool> trk_assigned(n_trk, false), meas_assigned(n_meas, false);
        for (int i = 0; i < n_trk; ++i) {
            int j = row_assign[i];
            if (j >= 0 && j < n_meas) {
                bool valid = false;
                for (auto& [mj, mc] : row_candidates[i]) {
                    if (mj == j && mc < cfg.gate_threshold) { valid = true; break; }
                }
                if (valid) {
                    assignments.push_back({i, j});
                    trk_assigned[i] = true;
                    meas_assigned[j] = true;
                }
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
