/*
 *	robot_health.h
 *	Task heartbeats, the RP2040 hardware watchdog and the crash record.
 *
 *	Plain C (stdint/stdbool only), so the pico-sdk namespace files (USB,
 *	CYW43, lwIP ports), the kernel's fault handler, the application and the
 *	host tests can all include it.
 *
 *	Heartbeats: each monitored loop adds one to its own counter once per
 *	loop (one writer each, 32-bit aligned, so a reader on the other core
 *	never sees a torn value). app_program/app_watchdog.c checks them from a
 *	100 ms cyclic handler and feeds the watchdog only while all advance.
 *
 *	Crash record: four words in watchdog SCRATCH0..3, which survive a
 *	watchdog reboot (SCRATCH4..7 belong to the bootrom and are never
 *	touched). Written just before a forced reboot (panic, hard assert, lwIP
 *	assert, HardFault) or when the supervisor stops feeding the watchdog.
 *	The next boot reads it, clears it and prints one "[RESET] ..." line.
 *	The register access lives in lib/libtm/sysdepend/pico_rp2040/
 *	robot_health.c; everything inline below is pure and host-tested.
 */
#ifndef ROBOT_HEALTH_H
#define ROBOT_HEALTH_H
#include <stdbool.h>
#include <stdint.h>

/* Monitored loops. NET is the CYW43/lwIP/MQTT poll task (core 0). */
enum {
	HEALTH_HB_VC,		/* vehicle controller, core 0 */
	HEALTH_HB_MOTOR,	/* S2 motor/PID task, core 0 */
	HEALTH_HB_NET,		/* network task, core 0 */
	HEALTH_HB_TELEMETRY,	/* S1 telemetry task, core 1 */
	HEALTH_HB_IMU,		/* S4 IMU sensor task, core 1 */
	HEALTH_HB_COUNT
};

/* Why the previous run ended (crash record reason code). */
enum {
	HEALTH_REASON_NONE,
	HEALTH_REASON_PANIC,		/* pico-sdk panic(); value = caller, value2 = message */
	HEALTH_REASON_HARD_ASSERT,	/* hard_assertion_failure(); value = caller */
	HEALTH_REASON_LWIP_ASSERT,	/* lwip_utk_assert(); value = caller, value2 = message */
	HEALTH_REASON_HARD_FAULT,	/* HardFault; value = stacked PC, value2 = stacked LR */
	HEALTH_REASON_HEARTBEAT,	/* supervisor; value = id | stale mask << 8, value2 = age ms */
	HEALTH_REASON_ARMED,		/* not a crash: written when the watchdog arms, its
					 * uptime refreshed on every feed. Found after a
					 * watchdog timeout: the supervisor itself stopped. */
	HEALTH_REASON_COUNT
};

/* A crash (panic, assert, HardFault) within this time of boot is "early".
 * HEALTH_HOLD_AFTER early crashes in a row: the next boot holds (no tasks,
 * no radio) and keeps printing the record instead of rebooting in a loop. */
#define HEALTH_EARLY_CRASH_MS	20000u
#define HEALTH_HOLD_AFTER	3u
/* HEALTH_REASON_HEARTBEAT value: named loop in bits 0..7, the mask of every
 * monitored loop that was stale too in bits 8..15. */
#define HEALTH_HB_VALUE(id, mask)	((uint32_t)(id) | ((uint32_t)(mask) << 8))
#define HEALTH_HB_VALUE_ID(value)	((value) & 0xFFu)
#define HEALTH_HB_VALUE_MASK(value)	(((value) >> 8) & 0xFFu)

/* SCRATCH0 = magic (upper 16 bits) | ~reason (8 bits) | reason (8 bits). */
#define HEALTH_RECORD_MAGIC	0xC5A4u

/* SCRATCH3 = early crash count (8 bits) << 24 | uptime ms (24 bits, held at
 * 0xFFFFFF after about 4.6 hours). */
#define HEALTH_UPTIME_MAX	0x00FFFFFFu

typedef struct {
	uint32_t reason;	/* HEALTH_REASON_* */
	uint32_t value;		/* caller / PC / heartbeat id and mask */
	uint32_t value2;	/* message pointer / LR / stale age ms */
	uint32_t uptime_ms;	/* time since boot when it was written */
	uint32_t crash_count;	/* early crashes in a row, this one included */
} health_record_t;

static inline void health_record_encode(const health_record_t *r, uint32_t word[4])
{
	uint32_t reason = r->reason & 0xFFu;
	uint32_t uptime = r->uptime_ms > HEALTH_UPTIME_MAX ? HEALTH_UPTIME_MAX : r->uptime_ms;
	uint32_t count = r->crash_count > 0xFFu ? 0xFFu : r->crash_count;
	word[0] = (HEALTH_RECORD_MAGIC << 16) | ((~reason & 0xFFu) << 8) | reason;
	word[1] = r->value;
	word[2] = r->value2;
	word[3] = (count << 24) | uptime;
}

static inline bool health_reason_is_crash(uint32_t reason)
{
	return reason >= HEALTH_REASON_PANIC && reason <= HEALTH_REASON_HARD_FAULT;
}

/* The early-crash count a record carries forward: its own count for a
 * crash record (non-zero only for an early crash), else 0. */
static inline uint32_t health_record_prior_crashes(const uint32_t word[4]);

/* Early crashes in a row after this record: prior + 1 for a crash within
 * HEALTH_EARLY_CRASH_MS of boot, else 0 (a late crash, a heartbeat stop). */
static inline uint32_t health_next_crash_count(uint32_t reason, uint32_t uptime_ms, uint32_t prior)
{
	if (!health_reason_is_crash(reason) || uptime_ms >= HEALTH_EARLY_CRASH_MS) return 0;
	return prior >= 0xFFu ? 0xFFu : prior + 1u;
}

/* true only for an intact record with a known reason. */
static inline bool health_record_decode(const uint32_t word[4], health_record_t *r)
{
	uint32_t reason = word[0] & 0xFFu;
	if ((word[0] >> 16) != HEALTH_RECORD_MAGIC) return false;
	if (((word[0] >> 8) & 0xFFu) != (~reason & 0xFFu)) return false;
	if (reason == HEALTH_REASON_NONE || reason >= HEALTH_REASON_COUNT) return false;
	r->reason = reason;
	r->value = word[1];
	r->value2 = word[2];
	r->uptime_ms = word[3] & HEALTH_UPTIME_MAX;
	r->crash_count = word[3] >> 24;
	return true;
}

static inline uint32_t health_record_prior_crashes(const uint32_t word[4])
{
	health_record_t r;
	if (!health_record_decode(word, &r) || !health_reason_is_crash(r.reason)) return 0;
	return r.crash_count;
}

/* Watchdog REASON register bits (RP2040 datasheet, WATCHDOG block). */
#define HEALTH_RESET_WATCHDOG_TIMER	0x1u
#define HEALTH_RESET_WATCHDOG_FORCE	0x2u

/* ---- Firmware only (lib/libtm/sysdepend/pico_rp2040/robot_health.c) ---- */
extern volatile uint32_t health_heartbeat[HEALTH_HB_COUNT];
/* Set by the network task once cyw43 init and its first join attempt are
 * done (or the radio bus failed and it is idling): its boot has finished. */
extern volatile uint32_t health_net_ready;

/* Set by health_record_write() for a crash reason: a crash path owns the
 * reboot from then on, so the supervisor neither feeds the watchdog nor
 * overwrites the record. */
extern volatile uint32_t health_crash_pending;
/* Boot: set by health_boot_read() when the previous record shows
 * HEALTH_HOLD_AFTER early crashes in a row: the board holds (app_watchdog.c). */
extern volatile uint32_t health_boot_hold;
/* Non-zero while the board holds, also before usermain has read the record
 * (it reads SCRATCH directly): the network task waits on it. */
int health_boot_hold_pending(void);

/* One loop done. Only the loop that owns the counter calls it. */
#define HEALTH_BEAT(id)	((void)(health_heartbeat[(id)]++))

/* Microseconds since boot (1 MHz hardware timer, reset at boot). */
uint64_t health_uptime_us(void);
/* Watchdog tick generator at 1 MHz (clk_ref 12 MHz / 12). It also clocks
 * the hardware timer. Registers only: safe before .bss is set up. */
void health_tick_start(void);
/* Write SCRATCH0..3. Safe from any context, interrupts included. A crash
 * reason also sets health_crash_pending. */
void health_record_write(uint32_t reason, uint32_t value, uint32_t value2);
/* Write the record, then reboot at once through the watchdog. */
void health_crash(uint32_t reason, uint32_t value, uint32_t value2) __attribute__((noreturn));
/* Reboot at once through the watchdog (no record change). */
void health_reboot_now(void) __attribute__((noreturn));
/* Boot: copy the REASON bits and SCRATCH0..3, then clear SCRATCH0..3
 * (kept when holding). Later crash records count on from this record. */
void health_boot_read(uint32_t *reset_reason, uint32_t word[4]);
/* Start the watchdog: reboot if not fed within timeout_ms (max 8000). */
void health_watchdog_start(uint32_t timeout_ms);
/* Reload the watchdog (nothing once a crash is pending) and refresh the
 * HEALTH_REASON_ARMED record's uptime, if that is the record held. */
void health_watchdog_feed(void);
void health_watchdog_stop(void);

#endif /* ROBOT_HEALTH_H */
