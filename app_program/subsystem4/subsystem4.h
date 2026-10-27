/* SPDX-License-Identifier: Apache-2.0 */
/* Buddy 4 -- GY-511 / LSM303DLHC sensor acquisition. */
#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"
#include "sensor_data.h"

#define S4_PRI_SENSOR     18 /* Existing TEAM.md priority band 18-19. */
#define S4_PRI_MONITOR    19
#define S4_POLL_MS        20
#define S4_REPORT_MS      200
#define S4_RETRY_US       UINT64_C(1000000)
#define S4_LOCK_WAIT_MS   5

/* Compatibility with the sample S3 -> S4 text interface (IF-03). This is
 * not the future telemetry/event protocol. New consumers use the snapshot. */
#define S4_SUMMARY_MAX    48
#define FLG_BATCH         0x0002
#define FLG_ALARM         0x0001 /* Reserved legacy bit; no S4 alarm task. */
IMPORT ID mbf_summary;
IMPORT ID flg_events;

/* Task-context API; bounded mutex wait. E_OBJ before init, E_PAR for NULL,
 * or a kernel lock error. E_OK copies a coherent snapshot; it does not mean
 * the sensor is connected. Check configured and each *_usable flag. */
/* Compact cross-task snapshot; decode after taking the consumer clock. */
IMPORT ER s4_sensor_get_record(s4_sensor_cache_t *p_record);
/* Compatibility/calculation view for local diagnostics. */
IMPORT ER s4_sensor_get_status(s4_sensor_status_t *p_status);

/* Internal task/lifecycle helpers. No other task may call the I2C driver. */
IMPORT ID tid_s4_sensor;
IMPORT ID tid_s4_monitor;
IMPORT void s4_sensor_task(INT start_code, void *p_exinf);
IMPORT void monitor_task(INT start_code, void *p_exinf);
IMPORT uint64_t s4_kernel_now_us(void *p_context);
IMPORT ER s4_status_publish(const s4_sensor_cache_t *p_status);
IMPORT ER s4_note_legacy_line(void);

#endif
