// [REQ-V60-BENCH-01] NX-MIMOSA C++ Core Benchmark
// Target: <50ms for 1000 tracks
#include "nx_core.hpp"
#include <iostream>
#include <chrono>
#include <random>
#include <iomanip>

using namespace nx;
using Clock = std::chrono::high_resolution_clock;

int main() {
    std::cout << "=== NX-MIMOSA v6.0 C++ Core Benchmark ===" << std::endl;

    const int N_TARGETS[] = {100, 500, 761, 1000, 2000};
    const int N_SCANS = 10;

    for (int n_tgt : N_TARGETS) {
        TrackerConfig cfg;
        cfg.dt = 5.0;
        cfg.r_std = 150.0;
        cfg.q_base = 1.0;
        MultiTargetTracker tracker(cfg);

        std::mt19937 rng(42);
        std::uniform_real_distribution<> pos_dist(-200000, 200000);
        std::uniform_real_distribution<> alt_dist(5000, 15000);
        std::normal_distribution<> noise(0, cfg.r_std);

        // Generate base positions
        std::vector<double> base(n_tgt * 3);
        for (int i = 0; i < n_tgt; ++i) {
            base[i*3]   = pos_dist(rng);
            base[i*3+1] = pos_dist(rng);
            base[i*3+2] = alt_dist(rng);
        }

        std::vector<double> timings;

        for (int scan = 0; scan < N_SCANS; ++scan) {
            // Generate measurements with noise
            std::vector<double> meas(n_tgt * 3);
            for (int i = 0; i < n_tgt * 3; ++i)
                meas[i] = base[i] + noise(rng);

            // Slowly move targets
            for (int i = 0; i < n_tgt; ++i) {
                base[i*3]   += 50.0 + noise(rng) * 0.1;
                base[i*3+1] += 30.0 + noise(rng) * 0.1;
            }

            auto t0 = Clock::now();
            tracker.process_scan(meas.data(), n_tgt, scan * cfg.dt);
            auto t1 = Clock::now();
            double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
            timings.push_back(ms);
        }

        // Stats
        int n_confirmed = 0;
        for (auto& t : tracker.tracks)
            if (t.status == TrackStatus::CONFIRMED) n_confirmed++;

        double mean_ms = 0, max_ms = 0;
        // Skip first 2 scans (warmup)
        int start = std::min(2, (int)timings.size());
        for (int i = start; i < (int)timings.size(); ++i) {
            mean_ms += timings[i];
            max_ms = std::max(max_ms, timings[i]);
        }
        mean_ms /= (timings.size() - start);

        std::cout << std::fixed << std::setprecision(1)
                  << "  " << std::setw(5) << n_tgt << " targets: "
                  << std::setw(8) << mean_ms << " ms mean, "
                  << std::setw(8) << max_ms  << " ms max | "
                  << n_confirmed << " confirmed"
                  << (mean_ms < 50.0 ? "  ✓" : "  ✗")
                  << std::endl;
    }

    return 0;
}
