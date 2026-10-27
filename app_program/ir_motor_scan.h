#ifndef IR_MOTOR_SCAN_H
#define IR_MOTOR_SCAN_H
#include <stdbool.h>
#include <stdint.h>
#ifndef IR_SCAN_MAX_POWER
#define IR_SCAN_MAX_POWER 20U
#endif
#if IR_SCAN_MAX_POWER != 20U && IR_SCAN_MAX_POWER != 30U && IR_SCAN_MAX_POWER != 50U
#error Unsupported bounded bench power cap
#endif
#if IR_SCAN_MAX_POWER == 50U
#define IR_SCAN_POWER_BITS 6
#define IR_SCAN_POWER_MASK 63U
#else
#define IR_SCAN_POWER_BITS 5
#define IR_SCAN_POWER_MASK 31U
#endif
#define IR_SCAN_LEASE_MS 250U
#ifndef IR_SCAN_MAX_MS
#define IR_SCAN_MAX_MS 6000U
#endif
#if IR_SCAN_MAX_MS != 6000U && IR_SCAN_MAX_MS != 20000U
#error Unsupported bounded bench duration
#endif
#define IR_SCAN_PULSE_MAX_MS 250U
enum { IR_SCAN_PULSE=1,IR_SCAN_RUN,IR_SCAN_RENEW,IR_SCAN_PREPARE,IR_SCAN_QUERY };
typedef enum {
    IR_SCAN_IDLE,IR_SCAN_COMPLETE,IR_SCAN_USB_LOST,IR_SCAN_ADC_LOST,
    IR_SCAN_MOTOR_FAULT,IR_SCAN_OBSTACLE,IR_SCAN_CLOCK,IR_SCAN_BAD_COMMAND,
    IR_SCAN_REQUESTED_STOP,IR_SCAN_DECODED,IR_SCAN_BAD_PATTERN,IR_SCAN_LINE_AMBIGUOUS,IR_SCAN_NO_PROGRESS,IR_SCAN_COURSE_TIMEOUT
} ir_scan_reason_t;
typedef struct {
    uint32_t prepared_ms,started_ms,renewed_ms,duration_ms,last_tick_ms;
    uint8_t active:1,prepared:1,scan:1,have_tick:1,sequence_valid:1;
    uint8_t power:IR_SCAN_POWER_BITS;
    uint8_t sequence:6;
    uint8_t reason:4;
} ir_scan_state_t;
typedef struct {
    uint32_t now_ms;
    bool motor_ok,adc_ok,white,front_clear;
} ir_scan_input_t;
bool ir_scan_frame_valid(const uint8_t wire[6]);
void ir_scan_stop(ir_scan_state_t *state,ir_scan_reason_t reason);
void ir_scan_step(ir_scan_state_t *state,const ir_scan_input_t *input);
bool ir_scan_command(ir_scan_state_t *state,const ir_scan_input_t *input,
                     const uint8_t wire[6],uint32_t issued_ms);
#endif
