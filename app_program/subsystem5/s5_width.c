/* Width from one-degree angular bounds and the saved forward target depth. */
#include "s5_width.h"
_Static_assert(S5_MAX_ANGLE <= 90 && S5_REFINE_STEP_DEG == 1,
               "One-degree bound geometry supports angles through 80 degrees");
EXPORT const UW s5_cosine_q10000[91] = {
    10000U, 9998U, 9994U, 9986U, 9976U, 9962U, 9945U, 9925U, 9903U, 9877U,
    9848U, 9816U, 9781U, 9744U, 9703U, 9659U, 9613U, 9563U, 9511U, 9455U,
    9397U, 9336U, 9272U, 9205U, 9135U, 9063U, 8988U, 8910U, 8829U, 8746U,
    8660U, 8572U, 8480U, 8387U, 8290U, 8192U, 8090U, 7986U, 7880U, 7771U,
    7660U, 7547U, 7431U, 7314U, 7193U, 7071U, 6947U, 6820U, 6691U, 6561U,
    6428U, 6293U, 6157U, 6018U, 5878U, 5736U, 5592U, 5446U, 5299U, 5150U,
    5000U, 4848U, 4695U, 4540U, 4384U, 4226U, 4067U, 3907U, 3746U, 3584U,
    3420U, 3256U, 3090U, 2924U, 2756U, 2588U, 2419U, 2250U, 2079U, 1908U,
    1736U, 1564U, 1392U, 1219U, 1045U, 872U, 698U, 523U, 349U, 175U, 0U
};
static const INT tangent_q[] = {
    0, 175, 349, 524, 699, 875, 1051, 1228, 1405, 1584,
    1763, 1944, 2126, 2309, 2493, 2679, 2867, 3057, 3249, 3443,
    3640, 3839, 4040, 4245, 4452, 4663, 4877, 5095, 5317, 5543,
    5774, 6009, 6249, 6494, 6745, 7002, 7265, 7536, 7813, 8098,
    8391, 8693, 9004, 9325, 9657, 10000, 10355, 10724, 11106, 11504,
    11918, 12349, 12799, 13270, 13764, 14281, 14826, 15399, 16003, 16643,
    17321, 18040, 18807, 19626, 20503, 21445, 22460, 23559, 24751, 26051,
    27475, 29042, 30777, 32709, 34874, 37321, 40108, 43315, 47046, 51446,
    56713
};

EXPORT BOOL s5_target_detected(const s5_status_t *sample)
{
    if (!sample || !sample->valid || sample->distance_mm == 0U ||
        sample->distance_mm > 5000U || sample->obstacle_distance_mm == 0U)
        return FALSE;
    INT magnitude = sample->angle_deg < 0 ? -sample->angle_deg : sample->angle_deg;
    if (magnitude > 90) return FALSE;
    /* At a perpendicular side look, forward projection is always zero. */
    if (magnitude==90) return sample->distance_mm<=S5_OVERCOME_CLEARANCE_MM;
    /* A closer foreground surface also obstructs this scan direction. */
    UW forward = sample->distance_mm * s5_cosine_q10000[magnitude] / 10000U;
    return forward <= sample->obstacle_distance_mm + S5_TARGET_DEPTH_TOLERANCE_MM;
}

static UW isqrt64(unsigned long long n)
{
    unsigned long long root = 0, bit = 1ULL << 62;
    while (bit > n) bit >>= 2;
    while (bit) {
        if (n >= root + bit) {n -= root + bit; root = (root >> 1) + bit;}
        else root >>= 1;
        bit >>= 2;
    }
    return (UW)root;
}
/* Pythagoras from the averaged slant edge range; tangent when range <= depth.
 * The edge is at least at the measured bound angle, so the half-width never
 * drops below floor(depth*tan(bound)): without that floor an edge range 1 mm
 * above depth gives sqrt(2*depth+1) ~ 17 mm at any angle, and sonar noise
 * would decide the bypass side. */
static UW half_width(UW depth, UW edge_mm, UINT degree)
{
    unsigned long long tangent = (unsigned long long)depth * (UW)tangent_q[degree];
    if (edge_mm > depth) {
        UW pythagoras = isqrt64((unsigned long long)edge_mm * edge_mm -
                                (unsigned long long)depth * depth);
        UW minimum = (UW)(tangent / 10000U);
        return pythagoras > minimum ? pythagoras : minimum;
    }
    return (UW)((tangent + 5000U) / 10000U);
}

EXPORT void s5_width_from_bounds(s5_status_t *result)
{
    if (!result) return;
    result->object_width_valid = FALSE;
    result->object_width_mm = result->object_width_min_mm = result->object_width_max_mm = 0U;
    result->object_left_mm = result->object_right_mm = 0U;
    if (!result->left_bound_valid || !result->right_bound_valid ||
        result->left_bound_deg < 0 || result->left_bound_deg >= 80 ||
        result->right_bound_deg > 0 || result->right_bound_deg <= -80 ||
        result->obstacle_distance_mm == 0U || result->obstacle_distance_mm > S5_OBSTACLE_MM)
        return;
    UINT left = (UINT)result->left_bound_deg;
    UINT right = (UINT)-result->right_bound_deg;
    UW inner = (UW)(tangent_q[left] + tangent_q[right]);
    UW outer = (UW)(tangent_q[left+1U] + tangent_q[right+1U]);
    UW depth = result->obstacle_distance_mm;
    result->object_width_min_mm = depth * inner / 10000U;
    result->object_width_max_mm = (depth * outer + 9999U) / 10000U;
    if(result->object_width_max_mm>AVOID_MAX_WIDTH_MM) {
        result->object_width_min_mm=result->object_width_max_mm=0U;
        return;
    }
    /* The tangent envelope above stays the validity/clearance bound. */
    result->object_left_mm = half_width(depth, result->left_bound_distance_mm, left);
    result->object_right_mm = half_width(depth, result->right_bound_distance_mm, right);
    result->object_width_mm = result->object_left_mm + result->object_right_mm;
    result->object_width_valid = TRUE;
}
