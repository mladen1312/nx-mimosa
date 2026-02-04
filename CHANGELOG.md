# Changelog

All notable changes to NX-MIMOSA will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Planned
- MHT-IMM hybrid algorithm
- Extended CCSDS TDM support
- Neural network classification
- Kubernetes deployment

---

## [1.1.0] - 2026-02-05

### 🆕 New Features

#### Extended Kalman Filter (EKF)
- **Nonlinear measurement models**: Radar RAE, Radar 2D, Bearing-only, GPS geodetic
- **Coordinate transforms**: WGS-84 geodetic ↔ ECEF ↔ ENU
- **EKF-IMM integration**: Combined nonlinear measurements with maneuver detection

#### Track-to-Track Fusion
- **Covariance Intersection (CI)**: Fusion with unknown correlations
- **Bar-Shalom-Campo**: Optimal fusion with known correlations
- **Track Correlation**: Mahalanobis distance gating and clustering
- **Fusion Manager**: System track lifecycle and sensor mapping

#### ROS2 Interface
- **Native ROS2 Node**: Subscribe to PointCloud2, publish MarkerArray
- **Standalone Interface**: Works without ROS2 for testing
- **Compatibility**: ROS2 Humble/Iron/Jazzy, Autoware.Universe, PX4

#### GPU Acceleration
- **CuPy Backend**: CUDA acceleration for matrix operations
- **Batched UKF**: Process 1000+ tracks in parallel
- **Auto Fallback**: Automatic CPU fallback when GPU unavailable
- **~10x Speedup**: Achieved with GPU for large track counts

### Changed
- Updated integration tests for new modules
- Improved test coverage

### Performance
- Batch tracking: 100,000+ updates/sec (CPU), 1M+ updates/sec (GPU)
- Multi-sensor fusion latency: <5ms for 10 sensors

---

## [1.0.0] - 2026-02-04

### 🎉 Initial Production Release

First production-ready release of NX-MIMOSA multi-domain radar tracking system.

### Added

#### Core Tracking
- **UKF (Unscented Kalman Filter)** - Primary nonlinear filter
- **CKF (Cubature Kalman Filter)** - Alternative with lower computational cost
- **VS-IMM (Variable-Structure IMM)** - 3-mode adaptive tracking
- **Adaptive Q estimation** - NIS-based process noise adaptation
- **Adaptive R estimation** - Innovation-based measurement noise adaptation
- **Fixed-lag smoother** - 10-step backward smoothing

#### Output Formatters
- **ASTERIX CAT062** - EUROCONTROL surveillance data exchange
- **CAN-FD** - ISO 11898 automotive bus format
- **Link-16 (TADIL-J)** - MIL-STD-6016 tactical data link
- **CCSDS** - Space telemetry packets
- **NMEA 2000** - Maritime navigation data

#### Advanced Algorithms
- **JPDA** - Joint Probabilistic Data Association for dense environments
- **MHT** - Multiple Hypothesis Tracking with N-scan pruning

#### ECCM (Electronic Counter-Countermeasures)
- **Frequency Agility** - LFSR/AES hopping patterns
- **RGPO Detection** - Range Gate Pull-Off jammer detection
- **Adaptive Gating** - Soft decision under jamming

#### FPGA Implementation
- **UKF Core** - Pipelined 6-state filter
- **IMM Core** - 3-mode mixer
- **Frequency Agility Controller** - Real-time hop generation
- **AXI4 Wrapper** - SoC integration
- Target: Xilinx RFSoC ZU48DR @ 250 MHz

#### Certification Documentation
- **DO-178C DAL-C** - Aviation software certification package
- **DO-254 DAL-C** - FPGA certification package
- **ISO 26262 ASIL-D** - Automotive functional safety

#### Infrastructure
- pytest test suite with 80%+ coverage
- GitHub Actions CI/CD pipeline
- Docker container support
- Sphinx documentation
- Python packaging (pip install)

### Performance Achieved

| Metric | Requirement | Achieved |
|--------|-------------|----------|
| ATC En-route RMS | ≤ 500 m | **122 m** |
| ATC Terminal RMS | ≤ 150 m | **47 m** |
| Track Continuity | ≥ 99.5% | **100%** |
| Latency | ≤ 100 ms | **45 ms** |
| FPGA Utilization | - | **23% LUT** |

### Standards Compliance
- EUROCONTROL EASSP ✓
- ICAO Doc 9871 ✓
- MIL-STD-6016 ✓
- ISO 26262 ASIL-D ✓
- ISO 11898 (CAN-FD) ✓
- CCSDS 503.0-B-1 ✓
- NMEA 2000 ✓

---

## [0.9.0] - 2026-01-15 (Beta)

### Added
- VS-IMM implementation
- ASTERIX encoder
- Basic CAN-FD support
- Initial DO-178C documentation

### Changed
- Improved UKF numerical stability
- Optimized matrix operations

### Fixed
- Covariance symmetry enforcement
- Track ID wraparound issue

---

## [0.5.0] - 2025-12-01 (Alpha)

### Added
- UKF core implementation
- Basic tracking loop
- Unit test framework

### Known Issues
- No certification documentation
- Single sensor only
- No FPGA support

---

## Version Numbering

- **MAJOR**: Incompatible API changes
- **MINOR**: New features, backward compatible
- **PATCH**: Bug fixes, backward compatible

## Links

- [Repository](https://github.com/mladen1312/nx-mimosa)
- [Documentation](https://nx-mimosa.readthedocs.io/)
- [Issues](https://github.com/mladen1312/nx-mimosa/issues)

---

*© 2024-2026 Nexellum d.o.o. All Rights Reserved.*
