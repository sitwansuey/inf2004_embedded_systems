/* In-place spins (S2 commands LEFT=3, RIGHT=4: one wheel back, one forward,
 * used by navigation turns and the spot scan): tyre scrub needs more torque
 * than straight driving, so a spin gets its own PWM ceiling, a breakaway kick
 * per wheel and a longer no-progress timeout. Straight commands and the line
 * follower's pivots (5/6, one wheel stopped) keep their tuned values. */
#ifndef S2_TURN_DRIVE_H
#define S2_TURN_DRIVE_H
#include <stdint.h>
#include <stdbool.h>
static inline bool s2_turn_command(unsigned command)
{
    return command==3U || command==4U;
}
/* PWM ceiling in percent. */
static inline unsigned s2_turn_limit(unsigned command,unsigned straight,unsigned turn)
{
    return s2_turn_command(command)?turn:straight;
}
/* Milliseconds a driven wheel may go without progress before S2 faults. */
static inline uint32_t s2_stall_ms(unsigned command,uint32_t straight,uint32_t turn)
{
    return s2_turn_command(command)?turn:straight;
}
/* PWM floor (milli-percent) for a spinning wheel at the start of a spin,
 * capped at the ceiling; 0 for other commands or kick_ms == 0. */
static inline unsigned s2_turn_kick_milli(unsigned command,uint32_t elapsed_ms,
    uint32_t kick_ms,unsigned kick,unsigned limit)
{
    if(!s2_turn_command(command) || elapsed_ms>=kick_ms)return 0;
    return (kick<limit?kick:limit)*1000U;
}
/* Per-wheel breakaway kick during a spin. The kick holds until the wheel has
 * really moved (moved_counts, not one backlash count) or its window ends; a
 * wheel that then goes rekick_ms without progress is kicked again, so a wheel
 * that re-stalls (e.g. when the turn slows to its seek power) recovers at once
 * instead of waiting for the PID integrator. */
typedef struct { uint32_t until_ms; int32_t moved; bool armed; } s2_kick_t;
static inline void s2_kick_start(s2_kick_t *k,uint32_t now,uint32_t kick_ms)
{
    k->until_ms=now+kick_ms;k->moved=0;k->armed=kick_ms!=0U;
}
/* progress: counts moved this step in the commanded direction (>0 forward).
 * since_progress_ms: time since the wheel last made progress. Returns true
 * while the kick floor applies. */
static inline bool s2_kick_step(s2_kick_t *k,uint32_t now,int32_t progress,
    uint32_t since_progress_ms,uint32_t kick_ms,int32_t moved_counts,uint32_t rekick_ms)
{
    if(progress>0)k->moved+=progress;
    bool window=k->armed && (int32_t)(k->until_ms-now)>0;
    if(!window && kick_ms!=0U && since_progress_ms>=rekick_ms) {
        s2_kick_start(k,now,kick_ms);window=true; /* stalled again */
    }
    if(window && k->moved>=moved_counts) {k->armed=false;window=false;}
    return window;
}
#endif
