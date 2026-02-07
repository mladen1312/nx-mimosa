// [REQ-V60-BIND-01] NX-MIMOSA C++ Core — Python Bindings
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include "nx_core.hpp"
#include <chrono>

namespace py = pybind11;

PYBIND11_MODULE(_nx_core, m) {
    m.doc() = "NX-MIMOSA v6.0 C++ Core — sub-50ms multi-target tracking";

    // TrackerConfig
    py::class_<nx::TrackerConfig>(m, "TrackerConfig")
        .def(py::init<>())
        .def_readwrite("dt", &nx::TrackerConfig::dt)
        .def_readwrite("r_std", &nx::TrackerConfig::r_std)
        .def_readwrite("q_base", &nx::TrackerConfig::q_base)
        .def_readwrite("gate_threshold", &nx::TrackerConfig::gate_threshold)
        .def_readwrite("coarse_gate_m", &nx::TrackerConfig::coarse_gate_m)
        .def_readwrite("min_separation", &nx::TrackerConfig::min_separation)
        .def_readwrite("confirm_hits", &nx::TrackerConfig::confirm_hits)
        .def_readwrite("confirm_window", &nx::TrackerConfig::confirm_window)
        .def_readwrite("delete_misses", &nx::TrackerConfig::delete_misses)
        .def_readwrite("coast_max", &nx::TrackerConfig::coast_max);

    // TrackStatus
    py::enum_<nx::TrackStatus>(m, "TrackStatus")
        .value("TENTATIVE", nx::TrackStatus::TENTATIVE)
        .value("CONFIRMED", nx::TrackStatus::CONFIRMED)
        .value("COASTING", nx::TrackStatus::COASTING)
        .value("DELETED", nx::TrackStatus::DELETED);

    // Track (read-only view)
    py::class_<nx::Track>(m, "Track")
        .def_readonly("id", &nx::Track::id)
        .def_readonly("status", &nx::Track::status)
        .def_readonly("hit_count", &nx::Track::hit_count)
        .def_readonly("miss_count", &nx::Track::miss_count)
        .def_readonly("age", &nx::Track::age)
        .def_property_readonly("position", [](const nx::Track& t) {
            return t.imm.position();
        })
        .def_property_readonly("velocity", [](const nx::Track& t) {
            nx::Vec6 xc; nx::Mat6 Pc;
            t.imm.combined_state(xc, Pc);
            return nx::Vec3(xc[3], xc[4], xc[5]);
        })
        .def_property_readonly("covariance", [](const nx::Track& t) {
            nx::Vec6 xc; nx::Mat6 Pc;
            t.imm.combined_state(xc, Pc);
            return Pc;
        })
        .def_property_readonly("state", [](const nx::Track& t) {
            nx::Vec6 xc; nx::Mat6 Pc;
            t.imm.combined_state(xc, Pc);
            return xc;
        })
        .def_property_readonly("model_probs", [](const nx::Track& t) -> py::array_t<double> {
            py::array_t<double> arr(t.imm.n_models);
            auto buf = arr.mutable_unchecked<1>();
            for (int i = 0; i < t.imm.n_models; ++i) buf(i) = t.imm.mu[i];
            return arr;
        });

    // MultiTargetTracker
    py::class_<nx::MultiTargetTracker>(m, "MultiTargetTracker")
        .def(py::init<>())
        .def(py::init<const nx::TrackerConfig&>())
        .def(py::init([](double dt, double r_std, double q_base) {
            nx::TrackerConfig cfg;
            cfg.dt = dt; cfg.r_std = r_std; cfg.q_base = q_base;
            return std::make_unique<nx::MultiTargetTracker>(cfg);
        }), py::arg("dt") = 5.0, py::arg("r_std") = 150.0, py::arg("q_base") = 1.0)

        .def_readwrite("cfg", &nx::MultiTargetTracker::cfg)

        // process_scan: accepts Nx3 numpy array
        .def("process_scan", [](nx::MultiTargetTracker& self,
                                py::array_t<double, py::array::c_style | py::array::forcecast> meas,
                                double timestamp) {
            auto buf = meas.request();
            if (buf.ndim != 2 || buf.shape[1] < 2)
                throw std::runtime_error("measurements must be Nx3 (or Nx2)");

            int n_meas = (int)buf.shape[0];
            int n_cols = (int)buf.shape[1];
            const double* ptr = (const double*)buf.ptr;

            // Handle 2D input (pad z=0)
            std::vector<double> meas3;
            if (n_cols == 2) {
                meas3.resize(n_meas * 3, 0.0);
                for (int i = 0; i < n_meas; ++i) {
                    meas3[i*3]   = ptr[i*2];
                    meas3[i*3+1] = ptr[i*2+1];
                }
                ptr = meas3.data();
            }

            auto t0 = std::chrono::high_resolution_clock::now();
            self.process_scan(ptr, n_meas, timestamp);
            auto t1 = std::chrono::high_resolution_clock::now();
            double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
            return ms;
        }, py::arg("measurements"), py::arg("timestamp") = -1.0,
           "Process one scan. Returns elapsed time in ms.")

        // Get confirmed tracks as list
        .def_property_readonly("confirmed_tracks", [](const nx::MultiTargetTracker& self) {
            std::vector<const nx::Track*> result;
            for (auto& t : self.tracks)
                if (t.status == nx::TrackStatus::CONFIRMED)
                    result.push_back(&t);
            return result;
        }, py::return_value_policy::reference_internal)

        // Get all non-deleted tracks
        .def_property_readonly("active_tracks", [](const nx::MultiTargetTracker& self) {
            std::vector<const nx::Track*> result;
            for (auto& t : self.tracks)
                if (t.status != nx::TrackStatus::DELETED)
                    result.push_back(&t);
            return result;
        }, py::return_value_policy::reference_internal)

        // Bulk state extraction (Nx6 numpy array) — zero-copy efficient
        .def("get_confirmed_states", [](const nx::MultiTargetTracker& self) {
            auto idx = self.confirmed_indices();
            int n = (int)idx.size();
            py::array_t<double> states({n, 6});
            py::array_t<int> ids(n);
            auto s = states.mutable_unchecked<2>();
            auto id = ids.mutable_unchecked<1>();
            for (int i = 0; i < n; ++i) {
                nx::Vec6 xc; nx::Mat6 Pc;
                self.tracks[idx[i]].imm.combined_state(xc, Pc);
                for (int j = 0; j < 6; ++j) s(i, j) = xc[j];
                id(i) = self.tracks[idx[i]].id;
            }
            return py::make_tuple(ids, states);
        }, "Returns (ids[N], states[N,6]) for confirmed tracks")

        // Stats
        .def_property_readonly("n_confirmed", [](const nx::MultiTargetTracker& self) {
            int c = 0;
            for (auto& t : self.tracks)
                if (t.status == nx::TrackStatus::CONFIRMED) c++;
            return c;
        })
        .def_readonly("total_created", &nx::MultiTargetTracker::total_created)
        .def_readonly("step", &nx::MultiTargetTracker::step);

    // Version
    m.attr("__version__") = "6.0.0";
}
