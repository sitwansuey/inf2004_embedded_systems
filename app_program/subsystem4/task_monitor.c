/* SPDX-License-Identifier: Apache-2.0 */
#include "subsystem4.h"
#include "terrain.h"
#include "mounting.h"
#include "accel_calibration.h"

static INT display_age(uint32_t age)
{
    return (age == S4_NO_SAMPLE_AGE) ? -1 :
           (age > INT32_MAX) ? INT32_MAX : (INT)age;
}

void monitor_task(INT start_code, void *p_exinf)
{
    UB text[S4_SUMMARY_MAX + 1];
    UINT events;
    INT length;
    UW reports = 0;
    terrain_t tilt = {0};
    const s4_accel_calibration_t accel_calibration=S4_BOARD_ACCEL_CALIBRATION;

    (void)start_code;
    (void)p_exinf;
    for (;;)
    {
        /* Keep IF-03 without controlling S1 or reading S2/S3 internals. */
        (void)tk_wai_flg(flg_events, FLG_BATCH, TWF_ORW | TWF_CLR,
                          &events, TMO_POL);
        while ((length = tk_rcv_mbf(mbf_summary, text, TMO_POL)) > 0)
        {
            text[length] = '\0';
            (void)s4_note_legacy_line();
            tm_printf((UB *)"[S4 legacy] %s\n", text);
        }
        /* Repeat the heading for terminals opened after boot. */
        if ((reports++ % 25u) == 0u)
        {
            tm_printf((UB *)"[S4] t_ms,init,a_ok,m_ok,ax_mg,ay_mg,az_mg,"
                              "mx_mG,my_mG,mz_mG,a_age_ms,m_age_ms,"
                              "a_seen,m_seen,a_errors,m_errors,heartbeat\n");
        }
        {
            s4_sensor_status_t status;

            if (s4_sensor_get_status(&status) >= E_OK)
            {
                int32_t body[3] = {0}, pitch_uncorrected = 0, roll_uncorrected = 0;
                bool mapped = s4_calibrated_body(status.latest.accel.milli_units,
                    BOARD_S4_MOUNTING,&accel_calibration,body);
                bool available = status.accel_usable && !status.latest.accel.overrun && mapped;
                bool gravity_ok = available && s4_gravity_tilt(body,
                    &pitch_uncorrected, &roll_uncorrected);
                terrain_step(&tilt,
                    (uint32_t)(status.latest.accel.read_time_us / 1000U),
                    body, available,
                    0, false, BOARD_TERRAIN_CALIBRATED,
                    BOARD_PITCH_ZERO_CDEG, BOARD_ROLL_ZERO_CDEG);
                tm_printf((UB *)"[S4] %u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
                                  "%u,%u,%u,%u,%u\n",
                          (UW)(status.observed_time_us / 1000u),
                          (INT)status.init_result,
                          (INT)status.accel_usable, (INT)status.mag_usable,
                          (INT)status.latest.accel.milli_units[0],
                          (INT)status.latest.accel.milli_units[1],
                          (INT)status.latest.accel.milli_units[2],
                          (INT)status.latest.mag.milli_units[0],
                          (INT)status.latest.mag.milli_units[1],
                          (INT)status.latest.mag.milli_units[2],
                          display_age(status.accel_age_ms),
                          display_age(status.mag_age_ms),
                          (UW)status.accel_samples, (UW)status.mag_samples,
                          (UW)status.accel_io_errors, (UW)status.mag_io_errors,
                          (UW)status.task_heartbeat);
                /* Existing copied quality fields; no extra acquisition or I/O. */
                if ((reports % 5u) == 0u) {
                    tm_printf((UB *)"[S4Q] t=%u init_attempts=%u poll=%u a_status=%u m_status=%u a_overrun=%u a_overruns=%u publish_errors=%u a_sample_ms=%u m_sample_ms=%u\n",
                        (UW)(status.observed_time_us / 1000u),
                        (UW)status.init_attempts, (UW)status.poll_result,
                        (UW)status.latest.accel.status, (UW)status.latest.mag.status,
                        (UW)status.latest.accel.overrun, (UW)status.accel_overruns,
                        (UW)status.publish_errors,
                        (UW)(status.latest.accel.read_time_us / 1000u),
                        (UW)(status.latest.mag.read_time_us / 1000u));
                    tm_printf((UB *)"[S4BODY] t=%u mounting=%u available=%u gravity_ok=%u forward_mg=%d left_mg=%d up_mg=%d pitch_uncorrected_cdeg=%d roll_uncorrected_cdeg=%d mounting_zero_enabled=%u\n",
                        (UW)(status.observed_time_us / 1000u), (UW)BOARD_S4_MOUNTING,
                        (UW)available, (UW)gravity_ok,
                        (INT)body[0], (INT)body[1], (INT)body[2],
                        (INT)pitch_uncorrected, (INT)roll_uncorrected,
                        (UW)BOARD_TERRAIN_CALIBRATED);
                }
                tm_printf((UB *)"[S4TILT] calibrated=%u valid=%u pitch_cdeg=%d roll_cdeg=%d\n",
                    (unsigned)BOARD_TERRAIN_CALIBRATED, (unsigned)tilt.tilt_valid,
                    tilt.tilt_valid ? (INT)terrain_pitch_cdeg(&tilt) : 0,
                    tilt.tilt_valid ? (INT)terrain_roll_cdeg(&tilt) : 0);
            }
        }
        (void)tk_dly_tsk(S4_REPORT_MS);
    }
}
