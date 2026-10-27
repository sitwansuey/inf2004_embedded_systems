#include "app_logging.h"
void app_log_state(const app_state_t *s) {
    static struct __attribute__((packed)) { uint32_t phase:4,fault:3,command:3,power:7,seen:1; } previous;
    if(!s)return;
    if(!previous.seen || s->vehicle.phase!=previous.phase || s->vehicle.fault!=previous.fault ||
       s->vehicle.command!=previous.command || s->vehicle.power!=previous.power) {
        tm_printf((UB *)"[STATE] t=%u seq=%u phase=%u fault=%u command=%u power=%u armed=%u\n",
            s->time_ms,s->sequence,(unsigned)s->vehicle.phase,(unsigned)s->vehicle.fault,
            (unsigned)s->vehicle.command,(unsigned)s->vehicle.power,(unsigned)s->vehicle.armed);
        previous.phase=s->vehicle.phase;previous.fault=s->vehicle.fault&7U;
        previous.command=s->vehicle.command;previous.power=s->vehicle.power;previous.seen=1;
    }
}
