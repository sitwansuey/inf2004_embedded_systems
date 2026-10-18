/* SPDX-License-Identifier: Apache-2.0 */
#include "hardware/clocks.h"

/* The WiFi port already provides this hook when its driver is built.
 * Otherwise only SDK I2C needs clk_sys. Kernel startup still owns clocks. */
#if !TM_WIFI_CYW43
uint32_t clock_get_hz(clock_handle_t clock)
{
    return (clock == clk_sys) ? 125000000u : 0u;
}
#endif
