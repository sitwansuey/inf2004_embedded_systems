/* SPDX-License-Identifier: Apache-2.0 */
#include "subsystem4.h"

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
            }
        }
        (void)tk_dly_tsk(S4_REPORT_MS);
    }
}
