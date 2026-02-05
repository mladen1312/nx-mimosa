/**
 * @file nx_mimosa_config.h
 * @brief Auto-generated from system_config.yaml
 * 
 * System: NX-MIMOSA v1.1.0
 * Generated: 2026-02-05 01:55:15
 * 
 * DO NOT EDIT MANUALLY - Regenerate from YAML
 */

#ifndef NX_MIMOSA_CONFIG_H
#define NX_MIMOSA_CONFIG_H

#include <stdint.h>

// ═══════════════════════════════════════════════════════════════════════════════
// RADAR PARAMETERS
// ═══════════════════════════════════════════════════════════════════════════════
#define RADAR_FS           100000000UL
#define RADAR_FC           10000000000ULL
#define RADAR_BANDWIDTH    10000000UL
#define RADAR_PRF          1000U
#define RADAR_MAX_RANGE    150000U

// ═══════════════════════════════════════════════════════════════════════════════
// FPGA PARAMETERS
// ═══════════════════════════════════════════════════════════════════════════════
#define AXI_DATA_WIDTH     32U
#define FIFO_DEPTH         1024U

// ═══════════════════════════════════════════════════════════════════════════════
// CFAR PARAMETERS
// ═══════════════════════════════════════════════════════════════════════════════
#define CFAR_WINDOW_SIZE   20U
#define CFAR_GUARD_CELLS   4U

// ═══════════════════════════════════════════════════════════════════════════════
// COGNITIVE CFAR
// ═══════════════════════════════════════════════════════════════════════════════
#define COGNITIVE_ENABLED  1
#define SVM_NUM_SV         64U

// ═══════════════════════════════════════════════════════════════════════════════
// REGISTER MAP
// ═══════════════════════════════════════════════════════════════════════════════
#define NX_MIMOSA_BASE_ADDR  0x80000000UL

#define REG_CONTROL_OFFSET   0x00U
#define REG_STATUS_OFFSET   0x04U
#define REG_CFAR_CONFIG_OFFSET   0x08U
#define REG_TRACK_COUNT_OFFSET   0x0CU
#define REG_DETECTION_COUNT_OFFSET   0x10U
#define REG_VERSION_OFFSET   0xFCU

// Register access macros
#define REG_WRITE(reg, val)  (*(volatile uint32_t*)(NX_MIMOSA_BASE_ADDR + reg##_OFFSET) = (val))
#define REG_READ(reg)        (*(volatile uint32_t*)(NX_MIMOSA_BASE_ADDR + reg##_OFFSET))

#endif // NX_MIMOSA_CONFIG_H
