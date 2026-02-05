/* Auto-generated from config/nx_mimosa_v33_regmap.yaml — DO NOT EDIT */
/* NX-MIMOSA v3.3 Dual-Mode IMM Tracker v3.3.0 */
/* (c) Nexellum d.o.o. — Commercial License */

#ifndef __NX_MIMOSA_V33_REGS_H__
#define __NX_MIMOSA_V33_REGS_H__

#include <stdint.h>

#define NX_MIMOSA_V33_BASE_ADDR  0x40000000U
#define NX_MIMOSA_V33_VERSION     0x00030003U

/* Master control register (RW) */
#define REG_CONTROL                  0x00U
#define REG_CONTROL_ENABLE_MASK  0x00000001U
#define REG_CONTROL_ENABLE_SHIFT 0
#define REG_CONTROL_SMOOTHER_ENABLE_MASK  0x00000002U
#define REG_CONTROL_SMOOTHER_ENABLE_SHIFT 1
#define REG_CONTROL_RESET_MASK  0x00000004U
#define REG_CONTROL_RESET_SHIFT 2
#define REG_CONTROL_WINDOW_ENABLE_MASK  0x00000008U
#define REG_CONTROL_WINDOW_ENABLE_SHIFT 3
#define REG_CONTROL_OFFLINE_ENABLE_MASK  0x00000010U
#define REG_CONTROL_OFFLINE_ENABLE_SHIFT 4

/* Turn rate omega [rad/s] (default 0.196 = 6g@300m/s) (RW) */
#define REG_OMEGA                    0x04U

/* Time step dt [s] (default 0.1) (RW) */
#define REG_DT                       0x08U

/* CV process noise q_cv (default 0.5) (RW) */
#define REG_Q_CV                     0x0CU

/* CT process noise q_ct (default 1.0) (RW) */
#define REG_Q_CT                     0x10U

/* Measurement noise std r [m] (default 2.5) (RW) */
#define REG_R_NOISE                  0x14U

/* Mode stay probability (default 0.88) (RW) */
#define REG_P_STAY                   0x18U

/* Status register (RO) */
#define REG_STATUS                   0x1CU
#define REG_STATUS_DOMINANT_MODE_MASK  0x00000007U
#define REG_STATUS_DOMINANT_MODE_SHIFT 0
#define REG_STATUS_TRACK_INITIALIZED_MASK  0x80000000U
#define REG_STATUS_TRACK_INITIALIZED_SHIFT 31

/* Initial state x (RW) */
#define REG_X_INIT_0                 0x20U

/* Initial state y (RW) */
#define REG_X_INIT_1                 0x24U

/* Initial state vx (RW) */
#define REG_X_INIT_2                 0x28U

/* Initial state vy (RW) */
#define REG_X_INIT_3                 0x2CU

/* P_init[0][0] diagonal (default 100.0) (RW) */
#define REG_P_INIT_0                 0x30U

/* P_init[1][1] diagonal (RW) */
#define REG_P_INIT_1                 0x34U

/* P_init[2][2] diagonal (RW) */
#define REG_P_INIT_2                 0x38U

/* P_init[3][3] diagonal (RW) */
#define REG_P_INIT_3                 0x3CU

/* Window smoother depth (1..64, default 30) (RW) */
#define REG_WINDOW_SIZE              0x40U
#define REG_WINDOW_SIZE_SIZE_MASK  0x0000003FU
#define REG_WINDOW_SIZE_SIZE_SHIFT 0

/* Smoother trigger mode (RW) */
#define REG_TRIGGER_MODE             0x44U
#define REG_TRIGGER_MODE_MODE_MASK  0x00000003U
#define REG_TRIGGER_MODE_MODE_SHIFT 0

/* Innovation NIS threshold (default 4.0) (RW) */
#define REG_INNOV_THRESHOLD          0x48U

/* Covariance trace spike threshold (default 100.0) (RW) */
#define REG_COV_THRESHOLD            0x4CU

/* Write 1 to trigger window smooth (auto-clear) (RW) */
#define REG_MANUAL_TRIGGER           0x50U
#define REG_MANUAL_TRIGGER_TRIGGER_MASK  0x00000001U
#define REG_MANUAL_TRIGGER_TRIGGER_SHIFT 0

/* Number of window-smoothed outputs produced (RO) */
#define REG_WINDOW_COUNT             0x54U

/* Current maneuver detector state (RO) */
#define REG_MANEUVER_STATE           0x58U
#define REG_MANEUVER_STATE_STATE_MASK  0x00000007U
#define REG_MANEUVER_STATE_STATE_SHIFT 0

/* Forward filter cycle count (RO) */
#define REG_FWD_CYCLES               0x5CU

/* Window smoother backward pass cycle count (RO) */
#define REG_SMOOTH_CYCLES            0x60U

/* Track buffer fill level (RO) */
#define REG_TRACK_FILL               0x64U

/* Version register (major.minor = 3.3) (RO) */
#define REG_VERSION                  0x68U

/* Q15.16 conversion */
#define FP_TO_Q1516(f)   ((int32_t)((f) * 65536.0f))
#define Q1516_TO_FP(q)   ((float)(q) / 65536.0f)

/* Register access (assumes volatile pointer) */
#define MIMOSA_WR(off, val) \
    (*(volatile uint32_t *)(NX_MIMOSA_V33_BASE_ADDR + (off)) = (val))
#define MIMOSA_RD(off) \
    (*(volatile uint32_t *)(NX_MIMOSA_V33_BASE_ADDR + (off)))

#endif /* __NX_MIMOSA_V33_REGS_H__ */