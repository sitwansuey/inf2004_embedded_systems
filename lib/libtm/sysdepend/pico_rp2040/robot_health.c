/*
 *	robot_health.c
 *	Thin RP2040 register layer for include/tm/robot_health.h: the
 *	heartbeat counters, the hardware watchdog and the crash record in
 *	watchdog SCRATCH0..3.
 *
 *	Plain register access with no pico-sdk or kernel headers, so every
 *	build profile links it (UART or USB console, with or without Wi-Fi) and
 *	it is safe from a fault handler. Register offsets are the RP2040
 *	datasheet's (and pico-sdk hardware/regs/watchdog.h, psm.h, timer.h).
 *	SCRATCH4..7 are left alone: the bootrom reads them on a watchdog reboot.
 */
#include <stdint.h>
#include <tm/robot_health.h>

#define REG(address)		(*(volatile uint32_t *)(address))
#define ALIAS_SET		0x2000u		/* atomic bit set alias */
#define ALIAS_CLR		0x3000u		/* atomic bit clear alias */

#define WATCHDOG_BASE		0x40058000u
#define WATCHDOG_CTRL		(WATCHDOG_BASE + 0x00u)
#define WATCHDOG_LOAD		(WATCHDOG_BASE + 0x04u)
#define WATCHDOG_REASON		(WATCHDOG_BASE + 0x08u)
#define WATCHDOG_SCRATCH0	(WATCHDOG_BASE + 0x0Cu)	/* SCRATCH0..3: 0x0C..0x18 */
#define WATCHDOG_TICK		(WATCHDOG_BASE + 0x2Cu)
#define WATCHDOG_CTRL_TRIGGER	0x80000000u
#define WATCHDOG_CTRL_ENABLE	0x40000000u
#define WATCHDOG_TICK_ENABLE	0x00000200u
#define WATCHDOG_TICK_RW	0x000003FFu	/* ENABLE and CYCLES */
#define WATCHDOG_LOAD_MAX	0x00FFFFFFu
/* The tick runs at 1 MHz from the 12 MHz crystal (clk_ref / 12), the same
 * setting S4's I2C timing and S5's echo timing already write. */
#define WATCHDOG_TICK_CYCLES	12u
/* RP2040-E1: the counter drops twice per tick, so 2 counts per us. */
#define WATCHDOG_COUNTS_PER_MS	2000u

#define PSM_WDSEL		(0x40010000u + 0x08u)
/* Reset everything except the two oscillators, as pico-sdk does. */
#define PSM_WDSEL_RESET		(0x0001FFFFu & ~0x3u)

#define TIMER_TIMERAWH		(0x40054000u + 0x24u)
#define TIMER_TIMERAWL		(0x40054000u + 0x28u)

volatile uint32_t health_heartbeat[HEALTH_HB_COUNT];
volatile uint32_t health_net_ready;
volatile uint32_t health_crash_pending;
volatile uint32_t health_boot_hold;

static uint32_t watchdog_load;
/* Early crashes in a row before this run; valid once boot_captured. */
static uint32_t prior_crashes;
static volatile uint32_t boot_captured;

static void scratch_read(uint32_t word[4])
{
	for (unsigned i = 0; i < 4u; i++) word[i] = REG(WATCHDOG_SCRATCH0 + 4u * i);
}

void health_tick_start(void)
{
	/* The reset value is ENABLE with CYCLES 0, so check both fields. */
	if ((REG(WATCHDOG_TICK) & WATCHDOG_TICK_RW) != (WATCHDOG_TICK_ENABLE | WATCHDOG_TICK_CYCLES))
		REG(WATCHDOG_TICK) = WATCHDOG_TICK_ENABLE | WATCHDOG_TICK_CYCLES;
}

uint64_t health_uptime_us(void)
{
	uint32_t hi0, lo, hi1;
	do {
		hi0 = REG(TIMER_TIMERAWH);
		lo = REG(TIMER_TIMERAWL);
		hi1 = REG(TIMER_TIMERAWH);
	} while (hi0 != hi1);
	return ((uint64_t)hi0 << 32) | lo;
}

void health_record_write(uint32_t reason, uint32_t value, uint32_t value2)
{
	health_record_t r;
	uint32_t word[4];

	r.reason = reason;
	r.value = value;
	r.value2 = value2;
	r.uptime_ms = (uint32_t)(health_uptime_us() / 1000u);
	if (health_reason_is_crash(reason)) {
		uint32_t prior = prior_crashes;
		health_crash_pending = 1;
		if (!boot_captured) {
			/* Before usermain read the record (a crash in kernel or
			 * driver start-up): the previous record is still there. */
			scratch_read(word);
			prior = health_record_prior_crashes(word);
		}
		r.crash_count = health_next_crash_count(reason, r.uptime_ms, prior);
	} else {
		r.crash_count = 0;
	}
	health_record_encode(&r, word);
	/* The magic word last: a record cut short is never valid. */
	REG(WATCHDOG_SCRATCH0 + 4u) = word[1];
	REG(WATCHDOG_SCRATCH0 + 8u) = word[2];
	REG(WATCHDOG_SCRATCH0 + 12u) = word[3];
	REG(WATCHDOG_SCRATCH0) = word[0];
}

void health_reboot_now(void)
{
	REG(WATCHDOG_CTRL + ALIAS_CLR) = WATCHDOG_CTRL_ENABLE;
	REG(PSM_WDSEL + ALIAS_SET) = PSM_WDSEL_RESET;
	REG(WATCHDOG_CTRL + ALIAS_SET) = WATCHDOG_CTRL_TRIGGER;
	for (;;) __asm__ volatile ("nop");
}

void health_crash(uint32_t reason, uint32_t value, uint32_t value2)
{
	/* Nothing else on this core runs between the record and the reset. */
	__asm__ volatile ("cpsid i" ::: "memory");
	health_record_write(reason, value, value2);
	health_reboot_now();
}

void health_boot_read(uint32_t *reset_reason, uint32_t word[4])
{
	unsigned i;

	*reset_reason = REG(WATCHDOG_REASON) & (HEALTH_RESET_WATCHDOG_TIMER |
						HEALTH_RESET_WATCHDOG_FORCE);
	scratch_read(word);
	prior_crashes = health_record_prior_crashes(word);
	boot_captured = 1;
	if (prior_crashes >= HEALTH_HOLD_AFTER) {
		/* Holding: keep the record (nothing reboots while holding; a
		 * power-cycle clears it), so health_boot_hold_pending() stays
		 * true for a task that checks it at any time. */
		health_boot_hold = 1;
		return;
	}
	for (i = 0; i < 4u; i++) REG(WATCHDOG_SCRATCH0 + 4u * i) = 0;
}

int health_boot_hold_pending(void)
{
	uint32_t word[4];

	if (health_boot_hold) return 1;
	scratch_read(word);
	return health_record_prior_crashes(word) >= HEALTH_HOLD_AFTER;
}

void health_watchdog_start(uint32_t timeout_ms)
{
	uint32_t load;

	REG(WATCHDOG_CTRL + ALIAS_CLR) = WATCHDOG_CTRL_ENABLE;
	health_tick_start();
	REG(PSM_WDSEL + ALIAS_SET) = PSM_WDSEL_RESET;
	load = (timeout_ms > WATCHDOG_LOAD_MAX / WATCHDOG_COUNTS_PER_MS) ?
		WATCHDOG_LOAD_MAX : timeout_ms * WATCHDOG_COUNTS_PER_MS;
	watchdog_load = load;
	REG(WATCHDOG_LOAD) = load;
	/* The reset value keeps PAUSE_DBG0/DBG1/JTAG set: a debugger halt does
	 * not reboot the board. */
	REG(WATCHDOG_CTRL + ALIAS_SET) = WATCHDOG_CTRL_ENABLE;
}

void health_watchdog_feed(void)
{
	static const health_record_t armed = {HEALTH_REASON_ARMED, 0, 0, 0, 0};
	uint32_t word[4];

	/* A crash path started its own short watchdog: leave it alone. */
	if (health_crash_pending) return;
	REG(WATCHDOG_LOAD) = watchdog_load;
	health_record_encode(&armed, word);
	if (REG(WATCHDOG_SCRATCH0) == word[0]) {
		uint32_t ms = (uint32_t)(health_uptime_us() / 1000u);
		REG(WATCHDOG_SCRATCH0 + 12u) = ms > HEALTH_UPTIME_MAX ? HEALTH_UPTIME_MAX : ms;
	}
}

void health_watchdog_stop(void)
{
	REG(WATCHDOG_CTRL + ALIAS_CLR) = WATCHDOG_CTRL_ENABLE;
}
