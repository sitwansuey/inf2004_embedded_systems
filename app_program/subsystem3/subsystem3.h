/* Buddy 3 public controller contract. Student ID: 2500383. */
#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"
#include "buddy3_board.h"
#include "buddy3_core.h"
#include "drv_ir.h"

#define S3_PRI_CONSUMER 16
#define S3_PERIOD_MS 10U
#define S3_STATE_LOCK_WAIT_MS 5 /* Requested bound; kernel tick/dispatch adds latency. */
#define S3_STATUS_MAX_AGE_MS 30U
#define S3_EVENT_MAX_AGE_MS 100U
#define S3_EVENT_CAPACITY 4U
#define S3_CONTROLLER_MAX_LEASE_MS 1000U
#define S3_BATCH_SIZE 5 /* Retained example interface constant. */

/* Legacy symbols remain link-compatible with the old S4 monitor. The sample
 * producer protocol in S2 no longer exists; these are not IR telemetry. */
IMPORT ID mbx_samples;
IMPORT ID mtx_stats;
typedef struct
{
    UW received;
    INT min;
    INT max;
    W sum;
} APP_STATS;
IMPORT APP_STATS app_stats;

typedef struct
{
    s3_config_t algorithms;
    s3_ir_config_t ir;
} s3_adapter_config_t;

#if defined(BOARD_IR_MOTOR_SCAN_BENCH) && BOARD_IR_MOTOR_SCAN_BENCH
typedef struct {
    uint32_t generation,time_ms,samples;
    ER error;
    s3_symbol_t symbol;
    uint8_t result; /* s3_barcode_status_t, bounded 0..7. */
} s3_barcode_bench_status_t;
/* Bound controller may reset/cancel this diagnostic decoder. The sole IR
 * sampler feeds it, under the existing status lock, on every acquisition.
 * No line steering, navigation events, extra task or new buffer. */
IMPORT ER s3_barcode_bench_control(bool observe,uint32_t *p_generation);
#endif

typedef struct
{
    s3_output_t output;
    s3_sample_t sample; /* Latest complete raw scan, valid only on E_OK acquisition. */
    uint32_t sample_time_ms;
    uint32_t generation;
    ER acquisition_error;
    bool configured;
#if defined(BOARD_IR_MOTOR_SCAN_BENCH) && BOARD_IR_MOTOR_SCAN_BENCH
    s3_barcode_bench_status_t bench;
#endif
} s3_status_t;

typedef struct
{
    s3_event_t event;
    uint32_t generation;
} s3_event_record_t;

IMPORT ER subsystem3_init(void);
IMPORT ER subsystem3_start(void);
/* Configure after init and before start. Defaults perform no GPIO/ADC access.
 * Calibrate off-line using s3_calibration_* before supplying this config. */
IMPORT ER s3_configure(const s3_adapter_config_t *p_config);
/* Motor-locked pre-start raw acquisition. No calibrated/steering/barcode
 * authority; accepts either digital inputs or kernel ADC channels 0..2. */
IMPORT ER s3_configure_raw(const s3_ir_config_t *p_config);
/* Single task binding; controller alone may direct modes, renew or consume
 * discrete events. TRACK is an explicit resume/ack, not a lease renewal. */
IMPORT ER s3_controller_bind(void);
IMPORT ER s3_controller_control(s3_mode_t mode, bool marker_clear, UW lease_ms);
IMPORT ER s3_controller_renew(UW lease_ms);
IMPORT ER s3_event_receive(s3_event_record_t *p_record);
#if defined(BOARD_IR_MOTOR_SCAN_BENCH) && BOARD_IR_MOTOR_SCAN_BENCH
/* Bound scan controller only: priority-inheritance state handoff, at most the
 * existing 5 ms requested mutex wait. No device/ADC wait; caller re-ages data. */
IMPORT ER s3_barcode_bench_status_get(s3_status_t *p_status);
#endif

/* Snapshots are copied with a nonblocking priority-inheritance mutex. A stale
 * or preempted snapshot always reports steering_allowed=false. */
IMPORT ER s3_status_get(s3_status_t *p_status);

#if defined(BOARD_IR_BARCODE_CAPTURE) && BOARD_IR_BARCODE_CAPTURE
#define S3_CAPTURE_CAPACITY 8U
typedef struct { s3_sample_t sample; uint32_t dropped; } s3_capture_record_t;
/* Passive diagnostic copies; never ADC/GPIO access or steering authority.
 * Nonblocking. Overflow is counted, never allowed to delay the sampler. */
IMPORT ER s3_capture_receive(s3_sample_t *p_sample, uint32_t *p_dropped);
#endif

/* Private sampling task entry and one bounded service iteration. */
IMPORT void consumer_task(INT stacd, void *exinf);
IMPORT ER s3_sample_wait(void);
IMPORT ER s3_service_once(void);

#endif
