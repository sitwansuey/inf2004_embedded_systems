/* Build: gcc -std=c99 -Wall -Wextra -Werror telemetry_host_test.c -o s1-test */
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* Isolate real telemetry.c from board headers; simulate copied RTOS messages. */
#define SUBSYSTEM1_H
#define TM_MQTT 1
#define TM_NET_LWIP 1
#define TA_TFIFO 0
#define TMO_POL 0
#define E_OK 0
#define E_PAR (-17)
#define E_OBJ (-41)
#define E_TMOUT (-50)
typedef int INT, ER, ID, BOOL;
typedef unsigned int UINT, UW;
typedef unsigned char UB;
typedef struct { UW hi, lo; } SYSTIM;
typedef struct { UINT mbfatr; INT maxmsz, bufsz; } T_CMBF;
#include "../../telemetry_topics.h"

static unsigned char queue[S1_TELEMETRY_CAPACITY][256];
static INT sizes[S1_TELEMETRY_CAPACITY];
static unsigned int head, count;
static ID tk_cre_mbf(T_CMBF *c) {
    assert(c->maxmsz <= 256);
    assert(c->bufsz >= S1_TELEMETRY_CAPACITY * (c->maxmsz + 4));
    return 7;
}
static ER tk_snd_mbf(ID id, const void *p, INT size, INT timeout) {
    unsigned int tail = (head + count) % S1_TELEMETRY_CAPACITY;
    assert(id == 7 && timeout == TMO_POL);
    if (count == S1_TELEMETRY_CAPACITY) return E_TMOUT;
    memcpy(queue[tail], p, (unsigned int)size); sizes[tail] = size; ++count;
    return E_OK;
}
static INT tk_rcv_mbf(ID id, void *p, INT timeout) {
    INT size;
    assert(id == 7 && timeout == TMO_POL);
    if (!count) return E_TMOUT;
    size = sizes[head]; memcpy(p, queue[head], (unsigned int)size);
    head = (head + 1) % S1_TELEMETRY_CAPACITY; --count;
    return size;
}
static ER tk_get_otm(SYSTIM *p) { p->hi = 0; p->lo = 100; return E_OK; }
static INT tm_sprintf(UB *p, const UB *format, ...) {
    INT result; va_list args; va_start(args, format);
    result = vsprintf((char*)p, (const char*)format, args);
    va_end(args); return result;
}
#include "../../telemetry.c"
static int take_text(char *topic, char *payload) {
    unsigned int length;
    return s1_telemetry_take_bytes(topic, (unsigned char *)payload, &length);
}

int main(void) {
    char topic[S1_TOPIC_MAX], payload[S1_PAYLOAD_MAX], input[] = "650";
    char oversized[S1_PAYLOAD_MAX + 1];
    unsigned int i;
    assert(s1_telemetry_send(S1_LEFT_IR_IR_LEFT, input) == E_OBJ);
    assert(s1_telemetry_init() == E_OK);
    assert(s1_telemetry_init() == E_OBJ);
    assert(s1_telemetry_send(S1_TOPIC_COUNT, "1") == E_PAR);
    assert(s1_telemetry_send(S1_LEFT_IR_IR_LEFT, "") == E_PAR);
    assert(s1_telemetry_send(S1_LEFT_IR_IR_LEFT, "{}") == E_PAR);
    memset(oversized, 'x', sizeof(oversized)-1); oversized[sizeof(oversized)-1] = 0;
    assert(s1_telemetry_send(S1_LEFT_IR_IR_LEFT, oversized) == E_PAR);
    assert(s1_telemetry_send(S1_LEFT_IR_IR_LEFT, input) == E_OK);
    input[0] = '9';
    assert(take_text(topic, payload) == 1);
    assert(strcmp(topic, "Robopico/sensors/left_ir/ir_left") == 0);
    assert(strcmp(payload, "650") == 0); /* copied, not dangling caller memory */
    assert(s1_telemetry_text(S1_BARCODE_IR_BARCODE, "001\"\\") == E_OK);
    assert(take_text(topic, payload) == 1);
    assert(strcmp(payload, "\"001\\\"\\\\\"") == 0);
    assert(s1_telemetry_boolean(S1_TERRAIN_VALID, 0) == E_OK);
    assert(take_text(topic, payload) == 1 && strcmp(payload, "false") == 0);
    for (i = 0; i < S1_TELEMETRY_CAPACITY; ++i)
        assert(s1_telemetry_integer(S1_MOTORS_POWER_PERCENT, (INT)i) == E_OK);
    assert(s1_telemetry_integer(S1_MOTORS_POWER_PERCENT, 99) == E_TMOUT);
    for (i = 0; i < S1_TELEMETRY_CAPACITY; ++i) {
        char expected[32]; sprintf(expected, "%u", i);
        assert(take_text(topic, payload) == 1);
        assert(strcmp(topic, "Robopico/Actuators/motors/power_percent") == 0);
        assert(strcmp(payload, expected) == 0);
    }
    assert(take_text(topic, payload) == 0);
    {
        unsigned char binary[S1_PAYLOAD_MAX]; unsigned int length;
        assert(s1_telemetry_u12(0, 1) == E_PAR);
        assert(s1_telemetry_u12(1, 4096) == E_PAR);
        assert(s1_telemetry_u12(3, 181) == E_PAR);
        assert(s1_telemetry_magnetic(8, -2049) == E_PAR);
        assert(s1_telemetry_u12(1, 0) == E_OK);
        assert(s1_telemetry_take_bytes(topic, binary, &length) == 1);
        assert(length == 2 && binary[0] == 1 && binary[1] == 0);
        assert(strcmp(topic, S1_PACKED16_TOPIC) == 0);
        assert(s1_telemetry_magnetic(8, -2048) == E_OK);
        assert(s1_telemetry_take_bytes(topic, binary, &length) == 1);
        assert(length == 2 && binary[0] == 8 && binary[1] == 0);
    }
    {
        uint8_t bytes[2] = {0x1bU,0x4fU};
        unsigned char result[S1_PAYLOAD_MAX]; unsigned int length;
        assert(s1_telemetry_record(bytes,1U) == E_PAR);
        assert(s1_telemetry_record(NULL,2U) == E_PAR);
        assert(s1_telemetry_record(bytes,2U) == E_OK);
        bytes[0]=0; bytes[1]=0;
        assert(s1_telemetry_take_bytes(topic,result,&length)==1);
        assert(length==2U && result[0]==0x1bU && result[1]==0x4fU);
        assert(s1_telemetry_record(bytes,2U)==E_PAR);
        assert(s1_telemetry_acceleration(EDR_ACCEL_X,-2049)==E_PAR);
        assert(s1_telemetry_acceleration(EDR_ACCEL_Z,2048)==E_PAR);
        assert(s1_telemetry_acceleration(EDR_MAG_X,0)==E_PAR);
        assert(s1_telemetry_acceleration(EDR_ACCEL_X,0)==E_OK);
        assert(s1_telemetry_take_bytes(topic,result,&length)==1);
        assert(length==2U && result[0]==0x0dU && result[1]==0x80U);
    }
    puts("S1 telemetry: validation, copying, escaping, FIFO and backpressure passed");
    return 0;
}
