/* S5 path assessment module; its API is exported through subsystem5.h. */
#ifndef S5_POLICY_H
#define S5_POLICY_H
#include "subsystem5.h"
/* Acquisition computes the plan before publishing S5_READY. */
IMPORT BOOL s5_acquisition_plan(s5_status_t *sensor);
/* side: 0 chooses (smaller half-width), +1 LEFT only, -1 RIGHT only. */
IMPORT BOOL s5_acquisition_plan_side(s5_status_t *sensor, INT side);
#endif
