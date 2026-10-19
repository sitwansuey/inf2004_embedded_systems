#ifndef APP_STATE_H
#define APP_STATE_H
#include "app.h"
#include "subsystem2/subsystem2.h"
#include "subsystem5/subsystem5.h"

#define APP_STATE_PERIOD_MS 10U
#define VEHICLE_FORWARD_POWER 30U
#define VEHICLE_TURN_POWER 30U
#define VEHICLE_START_DELAY_MS 2000U

typedef enum {
    VEHICLE_CRUISE, VEHICLE_ACQUISITION,
    VEHICLE_APPROACH_ROTATE, VEHICLE_APPROACH_CRUISE, VEHICLE_APPROACH_ROTATE_BACK,
    VEHICLE_OVERCOME_CRUISE, VEHICLE_OVERCOME_CLEARANCE,
    VEHICLE_REORG_ROTATE90, VEHICLE_REORG_CRUISE, VEHICLE_REORG_ROTATE_MINUS90,
    VEHICLE_RESTORE, VEHICLE_ABORT
} vehicle_phase_t;
typedef struct {
    vehicle_phase_t phase;
    UW boot_ms, deadline_ms, turn_ms, scan_id;
    UW approach_mm, return_mm, remaining_ms, last_ms;
    INT side;
    BOOL motion_started;
} vehicle_fsm_t;
typedef s5_stage_t app_avoidance_stage_t;
static inline app_avoidance_stage_t vehicle_avoidance_stage(vehicle_phase_t phase)
{
    switch (phase) {
    case VEHICLE_ACQUISITION: return S5_STAGE_ACQUISITION;
    case VEHICLE_APPROACH_ROTATE: case VEHICLE_APPROACH_CRUISE:
    case VEHICLE_APPROACH_ROTATE_BACK: return S5_STAGE_APPROACH;
    case VEHICLE_OVERCOME_CRUISE: case VEHICLE_OVERCOME_CLEARANCE:
        return S5_STAGE_OVERCOME;
    case VEHICLE_REORG_ROTATE90: case VEHICLE_REORG_CRUISE:
    case VEHICLE_REORG_ROTATE_MINUS90: case VEHICLE_RESTORE:
        return S5_STAGE_REORG;
    default: return S5_STAGE_NONE;
    }
}
/* S3/S4 payloads remain undefined until their sensors are implemented. */
typedef struct { BOOL available; UW time_ms; } app_sensor_placeholder_t;
typedef struct {
    UW sequence, time_ms;
    app_sensor_placeholder_t s3, s4;
    s5_status_t s5;
    BOOL s5_available;
    vehicle_phase_t phase;
    app_avoidance_stage_t avoidance_stage;
    s2_move_command_t movement; /* last command successfully sent to S2 */
    UINT power_percent;
} app_shared_state_t;

IMPORT ER app_state_init(void);
/* Main alone collects/publishes each 10 ms tick. Readers receive copies. */
IMPORT void app_state_collect(app_shared_state_t *next);
IMPORT ER app_state_publish(const app_shared_state_t *next);
IMPORT ER app_get_state(app_shared_state_t *copy);
#endif
