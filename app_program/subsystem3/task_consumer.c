/* Buddy 3 sampling task: sole caller of its GPIO/kernel ADC adapter. */
#include "subsystem3.h"

EXPORT void consumer_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;
    for (;;)
    {
        (void)s3_service_once();
        /* Kernel cyclic releases avoid adding a full guard tick after
         * every scan. The cyclic handler only wakes this sole sampler. */
        if (s3_sample_wait() < E_OK)
        {
            /* A broken task context cannot safely continue sampling. */
            tk_ext_tsk();
            return;
        }
    }
}
