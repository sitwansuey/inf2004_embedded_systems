/* Copied nonblocking handoff, derived from Buddy 1's telemetry inbox design. */
#include "subsystem1.h"
#include "control.h"
#include "network_bridge.h"
#include "../app_state.h"
#include <string.h>
static ID telemetry_buffer, command_buffer, ack_buffer, event_buffer;
#define S1_EVENT_CAPACITY                                                      \
    16U /* queued events; kept through a short link drop */
/* STOP: (STOPs offered << 8) | the last STOP's sequence, in one word, so the
 * VC never pairs one STOP's count with another STOP's sequence. Commands
 * carry the count (epoch) they were offered under; seen_stop is the VC's. */
static UW stop_word, seen_stop;
static UW drops;
#if TK_SUPPORT_SMP
/* tk/smp.h: a hardware spinlock across both cores (M0+ has no LDREX). */
IMPORT UW atomic_inc (UW * addr);
IMPORT UW atomic_cmpxchg (UW * addr, UW val, UW cmp);
#else
static UW
atomic_inc (UW * addr)
{
    UINT s;
    DI(s);
    UW n = ++*addr;
    EI(s);
    return n;
}
static UW
atomic_cmpxchg (UW * addr, UW val, UW cmp)
{
    UINT s;
    DI(s);
    UW old = *addr;
    if (old == cmp)
    {
        *addr = val;
    }
    EI(s);
    return old;
}
#endif
/* Wraps, never saturates: telemetry sends it modulo 256 (status subtype 12)
 * and receivers count drops as differences, so it must keep counting. Both
 * cores drop (telemetry on core 1, VC and network on core 0): one atomic
 * increment, so two drops at once are never counted as one. */
static void
dropped (void)
{
    (void) atomic_inc(&drops);
}
uint16_t
s1_drop_count (void)
{
    return (uint16_t) __atomic_load_n(&drops, __ATOMIC_RELAXED);
}
/* The network owner abandoned a snapshot part-way (older than 500 ms). */
void
s1_telemetry_dropped (void)
{
    dropped();
}
ER
s1_binary_init (void)
{
    if (telemetry_buffer > 0 || command_buffer > 0)
    {
        return E_OBJ;
    }
    T_CMBF c         = {0};
    c.mbfatr         = TA_TFIFO;
    c.maxmsz         = TELEMETRY_BYTES;
    c.bufsz          = 2 * ((TELEMETRY_BYTES + 3U) / 4U * 4U + 4U);
    telemetry_buffer = tk_cre_mbf(&c);
    if (telemetry_buffer < 0)
    {
        return telemetry_buffer;
    }
    c.maxmsz       = sizeof(control_message_t);
    c.bufsz        = 4 * (sizeof(control_message_t) + 4);
    command_buffer = tk_cre_mbf(&c);
    if (command_buffer < 0)
    {
        ER er = command_buffer;
        tk_del_mbf(telemetry_buffer);
        telemetry_buffer = 0;
        command_buffer   = 0;
        return er;
    }
    c.maxmsz   = CSC02_ACK_BYTES;
    c.bufsz    = 8 * (CSC02_ACK_BYTES + 8U);
    ack_buffer = tk_cre_mbf(&c);
    if (ack_buffer < 0)
    {
        ER er = ack_buffer;
        tk_del_mbf(command_buffer);
        tk_del_mbf(telemetry_buffer);
        command_buffer = telemetry_buffer = ack_buffer = 0;
        return er;
    }
    c.maxmsz     = EDR_RECORD_SIZE;
    c.bufsz      = S1_EVENT_CAPACITY * (EDR_RECORD_SIZE + 8U);
    event_buffer = tk_cre_mbf(&c);
    if (event_buffer < 0)
    {
        ER er = event_buffer;
        tk_del_mbf(ack_buffer);
        tk_del_mbf(command_buffer);
        tk_del_mbf(telemetry_buffer);
        command_buffer = telemetry_buffer = ack_buffer = event_buffer = 0;
        return er;
    }
    return E_OK;
}
/* Robot events (shared/events.h): one two-byte record each, never blocking. */
void
s1_event_post (unsigned id, unsigned value)
{
    if (event_buffer <= 0 || id == 0U || id > 15U || value > EDR_DATA_MAX)
    {
        return;
    }
    unsigned      word                    = (value << 4U) | id;
    unsigned char record[EDR_RECORD_SIZE] = {(unsigned char) (word & 255U),
                                             (unsigned char) (word >> 8U)};
    if (tk_snd_mbf(event_buffer, record, sizeof(record), TMO_POL) < 0)
    {
        dropped();
    }
}
int
s1_event_take (unsigned char * out)
{
    if (!out || event_buffer <= 0)
    {
        return E_OBJ;
    }
    return tk_rcv_mbf(event_buffer, out, TMO_POL);
}
void
s1_command_ack (unsigned seq, unsigned opcode, unsigned result, unsigned phase,
                unsigned fault)
{
    unsigned char ack[CSC02_ACK_BYTES] = {0xC2,
                                          (unsigned char) seq,
                                          (unsigned char) opcode,
                                          (unsigned char) result,
                                          (unsigned char) phase,
                                          (unsigned char) fault};
    if (ack_buffer <= 0)
    {
        return;
    }
    if (tk_snd_mbf(ack_buffer, ack, sizeof(ack), TMO_POL) == E_OK)
    {
        return;
    }
    /* Full (no broker for a while): the oldest reply goes, counted. The
     * newest is the one a waiting dashboard can still use. */
    unsigned char oldest[CSC02_ACK_BYTES];
    if (tk_rcv_mbf(ack_buffer, oldest, TMO_POL) >= 0)
    {
        dropped();
    }
    if (tk_snd_mbf(ack_buffer, ack, sizeof(ack), TMO_POL) != E_OK)
    {
        dropped();
    }
}
/* phase and fault for a reply the VC did not make: its last published state. */
static void
command_reply (unsigned seq, unsigned opcode, unsigned result,
               app_state_t * view)
{
    if (app_get_state(view) != E_OK)
    {
        memset(view, 0, sizeof(*view));
    }
    s1_command_ack(seq, opcode, result, view->vehicle.phase,
                   view->vehicle.fault);
}
void
s1_command_reply (unsigned seq, unsigned opcode, unsigned result)
{
    app_state_t view;
    command_reply(seq, opcode, result, &view);
}
int
s1_command_ack_take (unsigned char * out)
{
    if (!out || ack_buffer <= 0)
    {
        return E_OBJ;
    }
    return tk_rcv_mbf(ack_buffer, out, TMO_POL);
}
ER
s1_binary_send (const uint8_t data[TELEMETRY_BYTES])
{
    if (!data)
    {
        return E_PAR;
    }
    if (telemetry_buffer <= 0)
    {
        return E_OBJ;
    }
    ER er = tk_snd_mbf(telemetry_buffer, data, TELEMETRY_BYTES, TMO_POL);
    if (er != E_TMOUT)
    {
        if (er < 0)
        {
            dropped();
        }
        return er;
    }
    /* Full: the oldest snapshot goes (counted) and hands its one-snapshot
     * flags (impact, IMU fault) to this newer one, which is queued. */
    uint8_t newer[TELEMETRY_BYTES], older[TELEMETRY_BYTES];
    memcpy(newer, data, sizeof(newer));
    if (tk_rcv_mbf(telemetry_buffer, older, TMO_POL) == (INT) TELEMETRY_BYTES)
    {
        telemetry_keep_flags(newer, older);
        dropped();
    }
    er = tk_snd_mbf(telemetry_buffer, newer, TELEMETRY_BYTES, TMO_POL);
    if (er < 0)
    {
        dropped();
    }
    return er;
}
int
s1_telemetry_take (unsigned char * out)
{
    if (!out || telemetry_buffer <= 0)
    {
        return E_OBJ;
    }
    /* Newest first: an older or stale snapshot still queued is dropped
     * (counted); its one-snapshot flags go on in the newest current one,
     * or in the next one taken if none is current (network owner only). */
    static unsigned char flags[TELEMETRY_BYTES];
    unsigned char        next[TELEMETRY_BYTES];
    INT                  result = E_TMOUT;
    for (unsigned n = 0; n < 4; ++n)
    {
        INT size = tk_rcv_mbf(telemetry_buffer, next, TMO_POL);
        if (size < 0)
        {
            if (result != (INT) TELEMETRY_BYTES)
            {
                result = size;
            }
            break;
        }
        if (size == (INT) TELEMETRY_BYTES)
        {
            telemetry_keep_flags(flags, next);
        }
        if (size != (INT) TELEMETRY_BYTES || !s1_telemetry_current(next))
        {
            dropped();
            continue;
        }
        if (result == (INT) TELEMETRY_BYTES)
        {
            dropped();
        }
        memcpy(out, next, TELEMETRY_BYTES);
        result = (INT) TELEMETRY_BYTES;
    }
    if (result == (INT) TELEMETRY_BYTES)
    {
        telemetry_keep_flags(out, flags);
        memset(flags, 0, sizeof(flags));
    }
    return result;
}
int
s1_telemetry_current (const unsigned char * data)
{
    if (!data)
    {
        return 0;
    }
    /* Version(4) + sequence(16), followed by the full 32-bit kernel time. */
    uint32_t stamp = 0;
    for (unsigned n = 0; n < 32; ++n)
    {
        stamp |= (uint32_t) ((data[(20 + n) / 8] >> ((20 + n) % 8)) & 1U) << n;
    }
    SYSTIM now;
    return tk_get_otm(&now) == E_OK && now.lo - stamp < 500U;
}
int
s1_command_offer (const unsigned char * data, unsigned length)
{
    if (!data || length != CSC02_COMMAND_BYTES || data[0] != 0xC1 ||
        data[1] > 4)
    {
        return E_PAR;
    }
    if (data[1] == 0)
    { /* STOP survives a full queue and is handled before ARM. */
        UW old, word;
        do
        {
            old  = __atomic_load_n(&stop_word, __ATOMIC_ACQUIRE);
            word = (((old >> 8) + 1U) << 8) | data[5];
        } while (atomic_cmpxchg(&stop_word, word, old) != old);
        /* A STOP the VC has not taken yet runs once for both: the earlier
         * one is answered here (accepted), the later one by the VC. */
        if ((old >> 8) != __atomic_load_n(&seen_stop, __ATOMIC_ACQUIRE))
        {
            s1_command_reply(old & 255U, 0, 0);
        }
        return E_OK;
    }
    unsigned lease = (unsigned) data[3] | ((unsigned) data[4] << 8);
    if (data[1] == 1 &&
        (data[2] == 0 || data[2] > 50 || lease == 0 || lease > 1000))
    {
        return E_PAR;
    }
    if (data[1] == 2 && (data[2] || lease))
    {
        return E_PAR;
    }
    if (data[1] == 3 && (data[2] == 0 || data[2] > 50 || lease))
    {
        return E_PAR;
    }
    /* MAP: letter 1..26 (A..Z), action 0..4 (s3_navigation_t); 0/0 = codebook.
     */
    if (data[1] == 4 &&
        (data[2] > 26 || data[3] > 4 || data[4] || (data[2] == 0 && data[3])))
    {
        return E_PAR;
    }
    if (command_buffer <= 0)
    {
        return E_OBJ;
    }
    SYSTIM now;
    ER     er = tk_get_otm(&now);
    if (er < 0)
    {
        return er;
    }
    control_message_t m = {
        .time_ms = now.lo,
        .epoch   = __atomic_load_n(&stop_word, __ATOMIC_ACQUIRE) >> 8};
    for (unsigned i = 0; i < CSC02_COMMAND_BYTES; ++i)
    {
        m.wire[i] = data[i];
    }
    er = tk_snd_mbf(command_buffer, &m, sizeof(m), TMO_POL);
    if (er < 0)
    {
        dropped();
    }
    return er;
}
ER
s1_command_take (control_message_t * p)
{
    if (!p)
    {
        return E_PAR;
    }
    *p      = (control_message_t){0};
    UW word = __atomic_load_n(&stop_word, __ATOMIC_ACQUIRE), epoch = word >> 8;
    if (epoch != seen_stop)
    {
        __atomic_store_n(&seen_stop, epoch, __ATOMIC_RELEASE);
        p->wire[0] = 0xC1;
        p->wire[5] = (uint8_t) (word & 255U);
        return E_OK;
    }
    if (command_buffer <= 0)
    {
        return E_OBJ;
    }
    static app_state_t view; /* VC only: kept off its stack */
    for (unsigned n = 0; n < 4; ++n)
    {
        INT size = tk_rcv_mbf(command_buffer, p, TMO_POL);
        if (size < 0)
        {
            return size;
        }
        if (size == sizeof(*p) && p->epoch == epoch)
        {
            return E_OK;
        }
        /* Queued before a STOP the VC has handled: superseded, rejected. */
        if (size == sizeof(*p))
        {
            command_reply(p->wire[5], p->wire[1], 1, &view);
        }
    }
    *p = (control_message_t){0};
    return E_TMOUT;
}

/*** end of file ***/
