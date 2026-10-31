/* Buddy 1: copied telemetry inbox consumed by the single network owner.
 * No additional forwarding task: the service polls this inbox without waiting.
 */
#include "subsystem1.h"

typedef struct {
    S1_READING_ID reading;
    UW timestamp_ms;
    UINT length; /* Explicit binary/text payload size. */
    char payload[S1_PAYLOAD_MAX];
} S1_TELEMETRY_MSG;

static volatile ID telemetry_mbf;
#define S1_TOPIC_STRING(name, topic) topic,
static const char *const topic_names[] = { S1_TOPIC_LIST(S1_TOPIC_STRING) };
#undef S1_TOPIC_STRING

ER s1_telemetry_init(void)
{
    T_CMBF config = {0};
    ID id;
    if (telemetry_mbf > 0) return E_OBJ;
    config.mbfatr = TA_TFIFO;
    config.maxmsz = sizeof(S1_TELEMETRY_MSG);
    /* Include the kernel's four-byte per-message length header/alignment. */
    config.bufsz = S1_TELEMETRY_CAPACITY *
                   (((sizeof(S1_TELEMETRY_MSG) + 3) & ~3) + 4);
    id = tk_cre_mbf(&config);
    if (id < E_OK) return id;
    telemetry_mbf = id;
    return E_OK;
}

ER s1_telemetry_send(S1_READING_ID reading, const char *payload)
{
    S1_TELEMETRY_MSG message = {0};
    SYSTIM now;
    UINT i;
    if ((UINT)reading >= S1_TOPIC_COUNT || payload == 0) return E_PAR;
#if !defined(TM_MQTT) || !TM_MQTT || !defined(TM_NET_LWIP) || !TM_NET_LWIP
    return E_NOSPT;
#endif
    if (telemetry_mbf <= 0) return E_OBJ;
    for (i = 0; i < S1_PAYLOAD_MAX; ++i) {
        unsigned char c = (unsigned char)payload[i];
        if (c == 0) break;
        if (c < 0x20 || c > 0x7e) return E_PAR;
        message.payload[i] = (char)c;
    }
    if (i == 0 || i == S1_PAYLOAD_MAX) return E_PAR;
    if (reading != S1_VEHICLE_CONTROLLER_EVENTS &&
        (payload[0] == '{' || payload[0] == '[')) return E_PAR;
    message.length = i;
    message.reading = reading;
    tk_get_otm(&now);
    message.timestamp_ms = now.lo;
    return tk_snd_mbf(telemetry_mbf, &message, sizeof(message), TMO_POL);
}

ER s1_telemetry_integer(S1_READING_ID reading, INT value)
{
    UB payload[32];
    tm_sprintf(payload, (const UB*)"%d", value);
    return s1_telemetry_send(reading, (const char*)payload);
}

ER s1_telemetry_boolean(S1_READING_ID reading, BOOL value)
{
    return s1_telemetry_send(reading, value ? "true" : "false");
}

ER s1_telemetry_text(S1_READING_ID reading, const char *value)
{
    char payload[S1_PAYLOAD_MAX];
    UINT used = 0, i;
    if (value == 0 || reading == S1_VEHICLE_CONTROLLER_EVENTS) return E_PAR;
    payload[used++] = '"';
    for (i = 0; value[i] != 0; ++i) {
        unsigned char c = (unsigned char)value[i];
        if (c < 0x20 || c > 0x7e) return E_PAR;
        if (used + ((c == '"' || c == '\\') ? 2 : 1) + 2 > sizeof(payload))
            return E_PAR;
        if (c == '"' || c == '\\') payload[used++] = '\\';
        payload[used++] = (char)c;
    }
    payload[used++] = '"';
    payload[used] = 0;
    return s1_telemetry_send(reading, payload);
}

/* S1_TOPIC_COUNT is the internal binary-message tag, not a text reading ID. */
ER s1_telemetry_u12(UINT sensor_id, UINT value)
{
    S1_TELEMETRY_MSG message = {0};
    uint16_t word;
    SYSTIM now;
    if (!s1_pack16(sensor_id, value, &word)) return E_PAR;
#if !defined(TM_MQTT) || !TM_MQTT || !defined(TM_NET_LWIP) || !TM_NET_LWIP
    return E_NOSPT;
#endif
    if (telemetry_mbf <= 0) return E_OBJ;
    message.reading = S1_TOPIC_COUNT;
    message.length = 2;
    (void)word; /* Checked before network/configuration state above. */
    (void)edr_encode(sensor_id, value, (uint8_t *)message.payload, 2U);
    tk_get_otm(&now);
    message.timestamp_ms = now.lo;
    return tk_snd_mbf(telemetry_mbf, &message, sizeof(message), TMO_POL);
}

ER s1_telemetry_magnetic(UINT sensor_id, INT milligauss)
{
    if (sensor_id < S1_SENSOR_MAG_X || sensor_id > S1_SENSOR_MAG_Z ||
        milligauss < -2048 || milligauss > 2047) return E_PAR;
    return s1_telemetry_u12(sensor_id, (UINT)(milligauss + 2048));
}

/* All buddies hand off exactly one standard record, copied by the queue. */
ER s1_telemetry_record(const uint8_t *bytes, UINT length)
{
    unsigned int id, data;
    if (!edr_decode(bytes, length, &id, &data)) return E_PAR;
    return s1_telemetry_u12(id, data);
}
ER s1_telemetry_acceleration(UINT sensor_id, INT millig)
{
    if (sensor_id < EDR_ACCEL_X || sensor_id > EDR_ACCEL_Z ||
        millig < -2048 || millig > 2047) return E_PAR;
    return s1_telemetry_u12(sensor_id, (UINT)(millig + 2048));
}

int s1_telemetry_take_bytes(char *topic, unsigned char *payload, unsigned int *length)
{
    S1_TELEMETRY_MSG message;
    const char *name;
    INT result;
    UINT i;
    if (!topic || !payload || !length) return E_PAR;
    *length = 0;
    if (telemetry_mbf <= 0) return 0;
    result = tk_rcv_mbf(telemetry_mbf, &message, TMO_POL);
    if (result == E_TMOUT) return 0;
    if (result < 0) return result;
    if (result != (INT)sizeof(message) ||
        (UINT)message.reading > S1_TOPIC_COUNT ||
        message.length == 0 || message.length >= S1_PAYLOAD_MAX) return E_PAR;
    if (message.reading == S1_TOPIC_COUNT && message.length != 2) return E_PAR;
    name = message.reading == S1_TOPIC_COUNT ? S1_PACKED16_TOPIC : topic_names[message.reading];
    for (i = 0; i < S1_TOPIC_MAX - 1 && name[i]; ++i) topic[i] = name[i];
    topic[i] = 0;
    if (name[i] != 0) return E_PAR;
    for (i = 0; i < message.length; ++i) payload[i] = (unsigned char)message.payload[i];
    payload[i] = 0; /* Convenience for text consumers; never used for wire length. */
    *length = message.length;
    return 1;
}
