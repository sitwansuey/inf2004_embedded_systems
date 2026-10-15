/* Buddy 3 sampling task: sole caller of its GPIO/kernel ADC adapter. */
#include "subsystem3.h"

EXPORT void consumer_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;
    for (;;)
    {
        (void)s3_service_once();
        /* Kernel tick is 10 ms. Actual period includes execution/jitter;
         * freshness checks revoke steering if deadlines are missed. */
        if (tk_dly_tsk(S3_PERIOD_MS) < E_OK)
        {
            /* A broken task context cannot safely continue sampling. */
            tk_ext_tsk();
            return;
        }
    }
}
