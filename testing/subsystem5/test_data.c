#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "subsystem5.c"
#include "../shared/edr.h"

static ER lock_result;
static unsigned locks;
ID tk_get_tid(void) {return 100;}
ER tk_loc_mtx(ID id, INT timeout) { (void)id; (void)timeout; if (lock_result == E_OK) ++locks; return lock_result; }
ER tk_unl_mtx(ID id) { (void)id; assert(locks == 1); --locks; return E_OK; }
ID tk_cre_mtx(const T_CMTX *cfg) { (void)cfg; return 1; }
ER tk_dly_tsk(INT delay) { (void)delay; return E_OK; }
ER tk_get_otm(SYSTIM *now) { *now = (SYSTIM){0}; return E_OK; }
void tk_ext_tsk(void) {}
void tm_printf(const UB *format, ...) { (void)format; }
ID app_start_task(FP entry, PRI pri, UW core, const char *name) { (void)entry; (void)pri; (void)core; (void)name; return 1; }
ER s5_hw_timer_init(void) { return E_OK; }
void s5_hw_io_init(void) {}
void s5_hw_servo(INT angle) { (void)angle; }
void s5_hw_sample(s5_status_t *sample) { (void)sample; }

int main(void)
{
    s5_status_t v = {0}, decoded;
    uint8_t data[S5_DATA_SIZE], again[S5_DATA_SIZE];
    assert(s5_get_status(&v)==E_OBJ);
    assert(subsystem5_init()==E_OK && s5_controller_bind()==E_OK);
    assert(s5_get_status(&v) == E_OK && v.state == S5_CENTERING && v.angle_deg == 0);
    /* Exhaust all control-byte values, including reserved bits/codes. */
    assert(s5_data_encode(&v, data, sizeof(data)));
    for (unsigned control = 0; control < 256; ++control) {
        data[0] = (uint8_t)control;
        assert(s5_data_decode(data, sizeof(data), &decoded) ==
               (!(control & 128) && (control & 15) < 15 && ((control >> 4) & 7) < 5));
    }
    data[0] = 0;
    /* Bit 7 is width_one_sided: every flag combination round-trips. */
    for (unsigned flags = 0; flags < 256; ++flags) {
        data[1] = (uint8_t)flags;
        assert(s5_data_decode(data, sizeof(data), &decoded));
        assert(decoded.width_one_sided == (BOOL)(flags >> 7) && decoded.object_width_valid == (BOOL)((flags >> 6) & 1U));
        assert(s5_data_encode(&decoded, again, sizeof(again)));
        assert(memcmp(data, again, sizeof(data)) == 0);
    }
    data[1] = 0;
    for (unsigned field = 2; field < 8; ++field) {
        for (unsigned angle = 0; angle < 256; ++angle) {
            data[field] = (uint8_t)angle;
            assert(s5_data_decode(data, sizeof(data), &decoded) == (angle <= 180));
            if (angle <= 180) {
                assert(s5_data_encode(&decoded, again, sizeof(again)));
                assert(memcmp(data, again, sizeof(data)) == 0);
            }
        }
        data[field] = 90;
    }
    /* Exercise all bit positions of all 32-bit fields, including sign bits. */
    for (unsigned field = 8; field < S5_DATA_SIZE; field += 4) {
        for (unsigned bit = 0; bit < 32; ++bit) {
            memset(data + 8, 0, S5_DATA_SIZE - 8);
            data[field + bit / 8] = (uint8_t)(1U << (bit % 8));
            assert(s5_data_decode(data, sizeof(data), &decoded));
            assert(s5_data_encode(&decoded, again, sizeof(again)));
            assert(memcmp(data, again, sizeof(data)) == 0);
        }
    }
    v = (s5_status_t){.state=S5_READY, .valid=TRUE, .scan_id=0x12345678,
        .angle_deg=-90, .sample_angle_deg=90, .motor1_counts_start=INT32_MIN,
        .motor2_counts_end=INT32_MAX, .time_ms=UINT32_MAX,
        .object_left_mm=86, .object_right_mm=0xA1B2C3D4};
    assert(s5_data_encode(&v, data, sizeof(data)));
    assert(data[0] == 6 && data[1] == 8 && data[2] == 0 && data[3] == 180);
    assert(data[8] == 0x78 && data[9] == 0x56 && data[10] == 0x34 && data[11] == 0x12);
    /* Centre-to-edge halves append at 84 and 88, little-endian. */
    assert(data[84] == 86 && !data[85] && !data[86] && !data[87]);
    assert(data[88] == 0xD4 && data[89] == 0xC3 && data[90] == 0xB2 && data[91] == 0xA1);
    assert(s5_data_decode(data, sizeof(data), &decoded));
    assert(decoded.motor1_counts_start == INT32_MIN && decoded.motor2_counts_end == INT32_MAX);
    assert(decoded.time_ms == UINT32_MAX);
    assert(decoded.object_left_mm == 86 && decoded.object_right_mm == 0xA1B2C3D4U);
    for (size_t i=0; i<sizeof(data); ++i) printf("%02x", data[i]);
    puts("");
    memcpy(again, data, sizeof(data));
    v.angle_deg = 91;
    assert(!s5_data_encode(&v, data, sizeof(data)) && !memcmp(data, again, sizeof(data)));
    v.angle_deg = -90;
    v.valid = 2;
    assert(!s5_data_encode(&v, data, sizeof(data)));
    v.valid = TRUE;
    v.width_one_sided = 2;
    assert(!s5_data_encode(&v, data, sizeof(data)));
    v.width_one_sided = FALSE;
    assert(!s5_data_encode(&v, data, sizeof(data)-1));
    assert(!s5_data_encode(NULL, data, sizeof(data)));
    assert(!s5_data_decode(NULL, sizeof(data), &decoded));
    decoded.scan_id = 42;
    assert(!s5_data_decode(data, sizeof(data)-1, &decoded) && decoded.scan_id == 42);
    data[0] = 0xff;
    assert(!s5_data_decode(data, sizeof(data), &decoded) && decoded.scan_id == 42);
    status_store(&v);
    assert(s5_avoidance_begin(v.scan_id + 1) == E_OBJ);
    assert(s5_avoidance_begin(v.scan_id) == E_OK);
    assert(s5_get_status(&decoded) == E_OK && decoded.state == S5_ROTATING && !decoded.valid);
    assert(s5_navigation_set(v.scan_id, S5_OVERCOME, -90) == E_OK);
    assert(s5_get_status(&decoded) == E_OK && decoded.state == S5_OVERCOME && !decoded.valid);
    memset(data, 0xa5, sizeof(data));
    memcpy(again, data, sizeof(data));
    lock_result = E_TMOUT;
    assert(s5_status_encode(data, sizeof(data)) == E_TMOUT && !memcmp(data, again, sizeof(data)));
    assert(s5_navigation_set(v.scan_id, S5_MONITOR, 0) == E_TMOUT);
    lock_result = E_OK;
    assert(s5_status_encode(data, sizeof(data)) == E_OK);
    assert(s5_data_decode(data, sizeof(data), &decoded) && decoded.state == S5_OVERCOME);
    /* The one-sided width flag survives the shared status storage. */
    v.state = S5_READY; v.width_one_sided = TRUE;
    status_store(&v);
    assert(s5_get_status(&decoded) == E_OK && decoded.width_one_sided && decoded.object_left_mm == 86);
    v.width_one_sided = FALSE;
    v.state = (s5_state_t)15;
    status_store(&v);
    assert(s5_get_status(&decoded) == E_OK && decoded.state == S5_FAULT && !decoded.valid);
    /* Policy result survives packing, including clock rollover freshness. */
    v = (s5_status_t){.state=S5_MONITOR, .valid=TRUE, .distance_mm=201, .time_ms=UINT32_MAX-10};
    assert(s5_data_encode(&v, data, sizeof(data)) && s5_data_decode(data, sizeof(data), &decoded));
    assert(s5_assess_path(&decoded, TRUE, 10).state == S5_PATH_FORWARD);
    assert(s5_assess_path(&decoded, TRUE, 1000).state == S5_PATH_UNAVAILABLE);
    /* O3: the VC latched READY's plan and asked for APPROACH (the reverse
     * starts on its next tick). The S5 task's next sample is still taken in
     * READY with the wheels already turning; the request applies before the
     * step, so READY's motion guard keeps the plan, bounds, widths and side
     * the status goes on publishing, and no motion restart is used. */
    {
        s5_status_t ready = {.state=S5_READY, .scan_id=9, .valid=TRUE, .distance_mm=400,
            .angle_deg=30, .sample_angle_deg=30, .obstacle_distance_mm=150,
            .approach_plan_valid=TRUE, .planned_rotate_deg=30, .avoidance_deg=21,
            .planned_approach_mm=120, .planned_return_mm=120,
            .left_bound_valid=TRUE, .right_bound_valid=TRUE, .left_bound_deg=20, .right_bound_deg=-20,
            .left_bound_distance_mm=160, .right_bound_distance_mm=160,
            .object_width_valid=TRUE, .object_width_mm=110, .object_width_max_mm=110,
            .object_left_mm=55, .object_right_mm=55, .encoder_valid=TRUE,
            .motor1_counts_start=0, .motor1_counts_end=12, .motor2_counts_start=0, .motor2_counts_end=12};
        status_store(&ready);
        assert(s5_navigation_set(9, S5_APPROACH, 0) == E_OK && navigation_requested);
        s5_scan_context_t scan = {0};
        s5_status_t next = ready;
        assert(navigation_pending(&next) == E_OK && next.state == S5_APPROACH);
        assert(s5_scan_step(&next, &scan) == 30 && next.state == S5_APPROACH && !scan.motion_restarts);
        assert(next.approach_plan_valid && next.planned_rotate_deg == 30 && next.avoidance_deg == 21);
        assert(next.left_bound_valid && next.right_bound_valid && next.object_width_valid);
        assert(next.object_left_mm == 55 && next.object_right_mm == 55 && next.object_width_mm == 110);
        /* With no request, READY's guard still restarts on real motion. */
        navigation_requested = FALSE; next = ready;
        assert(navigation_pending(&next) == E_OK && next.state == S5_READY);
        assert(s5_scan_step(&next, &scan) == 0 && next.state == S5_CENTERING && scan.motion_restarts == 1U);
        assert(!next.approach_plan_valid && !next.object_width_valid && !next.left_bound_valid);
        /* A MONITOR request is left to the task's own full reset. */
        status_store(&ready);
        assert(s5_navigation_set(9, S5_MONITOR, 0) == E_OK && navigation_requested);
        next = ready;
        assert(navigation_pending(&next) == E_OK && next.state == S5_READY);
        navigation_requested = FALSE;
    }
    assert(locks == 0);
    printf("S5 PASS: native=%zu packed=%u bytes\n", sizeof(v), S5_DATA_SIZE);
    return 0;
}
