/* Steering mapping retained from main 58b89c2, with acquisition/decoder
 * ownership left in Buddy 3 and all movement authorization in the controller. */
#include "vehicle_line.h"
uint8_t vehicle_line_command(uint8_t black_mask)
{
    if (black_mask > 3U) return 0U;
    return black_mask == 2U ? 6U : black_mask == 1U ? 5U : 1U;
}
