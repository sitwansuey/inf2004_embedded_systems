/* Buddy 3 public controller contract. Student ID: 2500383. */
#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"
#include "buddy3_core.h"
#include "drv_ir.h"

#define S3_PRI_CONSUMER 16
#define S3_PERIOD_MS 10U
#define S3_STATE_LOCK_WAIT_MS 5 /* Requested bound; kernel tick/dispatch adds latency. */
#define S3_STATUS_MAX_AGE_MS 30U
#define S3_EVENT_MAX_AGE_MS 100U
#define S3_EVENT_CAPACITY 4U
#define S3_CONTROLLER_MAX_LEASE_MS 1000U

/* S3 status mutex. */
IMPORT ID mtx_stats;

typedef struct
{
    s3_config_t algorithms;
    s3_ir_config_t ir;
} s3_adapter_config_t;

typedef struct
{
    s3_output_t output;
    s3_sample_t sample; /* Latest complete raw scan, valid only on E_OK acquisition. */
    uint32_t sample_time_ms;
    uint32_t generation;
    ER acquisition_error;
    bool configured;
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
 * discrete events. TRACK is an explicit resume/ack, not a lease renewal.
 * control and renew wait up to S3_STATE_LOCK_WAIT_MS (plus kernel tick
 * rounding) for the status lock. Their codes for the controller:
 *   E_OK    applied.
 *   E_TMOUT status lock still busy: nothing changed (the old mode and lease
 *           stand). Retry next period; the lease still bounds a lost link.
 *   E_OBJ   renew only: the lease has expired (or mode is not TRACK).
 *           Never renewed; send TRACK again (a resume) or stop.
 *   E_PAR   invalid mode/lease.  E_OACV  not the bound controller (or
 *           TRACK in raw mode).  Other negative: kernel clock/lock error. */
IMPORT ER s3_controller_bind(void);
IMPORT ER s3_controller_control(s3_mode_t mode, bool marker_clear, UW lease_ms);
IMPORT ER s3_controller_renew(UW lease_ms);
/* Controller only: the vehicle measured the end of the marker (no GP28 edge
 * over BOARD_BARCODE_END_GAP_MM of travel); finish the running capture now.
 * Applied by the S3 task on its next sample; ignored if nothing is reading. */
IMPORT ER s3_controller_barcode_end(void);
/* Controller only: operator codebook override, see s3_core_set_map (letter
 * 1..26 = A..Z, nav 0..4; 0/0 restores the codebook). E_PAR if invalid.
 * Applied by the S3 task on its next sample; affects future decodes only. */
IMPORT ER s3_controller_set_map(UINT letter, UINT nav);
/* Controller only, nonblocking (E_TMOUT: nothing to deliver, or lock busy).
 * Every event needs the current generation and an age under
 * S3_EVENT_MAX_AGE_MS. NAVIGATION, STOP_REQUEST and FAULT need nothing else:
 * a turn decoded from a fresh sample is delivered even if a later sample
 * latched STALE/SENSOR. RECOVERY_REQUEST also needs a healthy status no older
 * than the configured max_sample_gap_ms. */
IMPORT ER s3_event_receive(s3_event_record_t *p_record);

/* Snapshots are copied with a nonblocking priority-inheritance mutex. A stale
 * or preempted snapshot always reports steering_allowed=false.
 * output.awaiting_controller: S3 holds a decoded turn or a latched fault and
 * ignores further markers until the next TRACK. With output.error ==
 * S3_ERROR_NONE and line_straddle it is a decoded turn waiting for the
 * controller (edge-follow configs also wait after a recovery/stop request). */
IMPORT ER s3_status_get(s3_status_t *p_status);

/* Private sampling task entry and one bounded service iteration. */
IMPORT void consumer_task(INT stacd, void *exinf);
IMPORT ER s3_sample_wait(void);
IMPORT ER s3_service_once(void);

#endif
