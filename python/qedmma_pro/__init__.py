"""
╔══════════════════════════════════════════════════════════════════════════════╗
║                         QEDMMA-PRO v3.0                                      ║
║              Commercial Radar Tracking & Signal Processing Suite             ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  Copyright (C) 2026 Dr. Mladen Mešter / Nexellum d.o.o.                     ║
║  License: Commercial - All Rights Reserved                                   ║
║                                                                              ║
║  📧 mladen@nexellum.com | 🌐 www.nexellum.com | 📱 +385 99 737 5100         ║
╚══════════════════════════════════════════════════════════════════════════════╝

QEDMMA-PRO extends the open-source QEDMMA-Lite with production-ready features:

LAYER 1 - SIGNAL PROCESSING:
  • Zero-DSP Correlator (streaming, parallel lanes)
  • SDR drivers (BladeRF, PlutoSDR, USRP)
  • Coherent integration

LAYER 2A - DETECTION & CLASSIFICATION:
  • ML-CFAR Engine (ML-assisted CFAR)
  • Micro-Doppler AI Classifier (F-35 vs birds/decoys)
  • Jammer Localizer (HOJ capability)
  • DRFM/Decoy rejection

LAYER 2B - ANOMALY TRACKING (EXCLUSIVE):
  • Anomaly Hunter™ - Physics-agnostic tracking
  • Skip-glide maneuver tracking
  • Auto physics↔learned handoff

LAYER 3 - TRACKING (ENHANCED):
  • UKF-Pro (SR-UKF, IUKF, Constraints)
  • CKF-Pro (higher-order cubature)
  • GPU Acceleration (CUDA/CuPy)
  • Health Monitoring

LAYER 4 - MULTI-SENSOR FUSION:
  • Track Fusion Engine (1024 simultaneous tracks)
  • Covariance Intersection
  • JDL Levels 0-4

LAYER 5 - C2 INTEGRATION:
  • Link-16 Interface (NATO)
  • ASTERIX Parser (EUROCONTROL)
  • AIS Integration (Maritime)

FPGA IP CORES (22+ modules):
  • Correlator bank
  • ML-CFAR hardware
  • White Rabbit PTP sync (<1ns)
  • Digital frontend (AGC, decimation)

TARGET INDUSTRIES:
  🚗 Automotive (ADAS, autonomous vehicles)
  ✈️ Aerospace/Defense (air defense, missile tracking)
  🚢 Maritime (VTS, collision avoidance)
  🤖 Robotics (warehouse, drones)
  🛰️ Space (debris tracking)

For licensing inquiries, contact: mladen@nexellum.com
Open-source alternative: https://github.com/mladen1312/qedmma-lite
"""

__version__ = "3.0.0"
__author__ = "Dr. Mladen Mešter"
__email__ = "mladen@nexellum.com"
__company__ = "Nexellum d.o.o."
__license__ = "Commercial"

# Expose main classes
try:
    from .core.ukf_pro import UKFPro, UKFProParams, UKFProState, create_hypersonic_ukf
except ImportError:
    pass

try:
    from .layer2b.anomaly_hunter import AnomalyHunter, AnomalyConfig, AnomalyLevel
except ImportError:
    pass

try:
    from .layer2a.micro_doppler_classifier import MicroDopplerClassifier, TargetClass
except ImportError:
    pass

__all__ = [
    "__version__",
    "__author__",
    # Layer 3 - Tracking
    "UKFPro", "UKFProParams", "UKFProState", "create_hypersonic_ukf",
    # Layer 2B - Anomaly
    "AnomalyHunter", "AnomalyConfig", "AnomalyLevel",
    # Layer 2A - Detection
    "MicroDopplerClassifier", "TargetClass",
]


def get_license_info():
    """Display license and contact information."""
    return __doc__


def list_fpga_ip():
    """List available FPGA IP cores."""
    return {
        "correlator": [
            "parallel_correlator_engine.sv",
            "coherent_integrator.sv",
            "prbs20_segmented_correlator.sv",
            "qedmma_correlator_bank_v32.sv",
        ],
        "eccm": [
            "ml_cfar_engine.sv",
            "jammer_localizer.sv",
            "eccm_controller.sv",
            "ml_cfar_hoj_controller.sv",
        ],
        "fusion": [
            "track_fusion_engine.sv",
            "track_database.sv",
            "asterix_parser.sv",
            "link16_interface.sv",
            "external_track_adapter.sv",
        ],
        "sync": [
            "white_rabbit_ptp_core.sv",
            "toa_capture_unit.sv",
            "dmtd_phase_detector.sv",
        ],
        "frontend": [
            "digital_agc.sv",
            "polyphase_decimator.sv",
        ],
        "comm": [
            "comm_controller_top.sv",
            "link16_asterix_adapter.sv",
            "failover_fsm.sv",
            "link_monitor.sv",
        ],
    }
