/*
 *	app_watchdog.c
 *	Hardware watchdog supervisor and the boot reset report.
 *
 *	Supervisor: a 100 ms kernel cyclic handler on core 1 reads the five
 *	heartbeat counters (include/tm/robot_health.h) and feeds the RP2040
 *	watchdog only while every monitored loop advanced within
 *	BOARD_WATCHDOG_WINDOW_MS (shared/watchdog_policy.c decides). When one
 *	stops, it writes the crash record naming that loop (and every loop
 *	stale with it) and reboots at once; the watchdog timeout
 *	(BOARD_WATCHDOG_TIMEOUT_MS after the last feed) is the backup.
 *	Running on core 1 keeps a task-level stall on core 0 (where the
 *	2026-11-19 freezes happened) named. The cyclic is still dispatched by
 *	the core-0 kernel tick, so a core-0 hang at interrupt level (or a
 *	stopped core 1) stops the supervisor itself: the watchdog then reboots
 *	the board and the "watchdog armed" record written at arming makes the
 *	next boot say so, with the time of the last feed.
 *
 *	Report: usermain reads the record and the watchdog REASON bits before
 *	any task starts, then prints one "[RESET] ..." line, repeated every
 *	10 s for the first 60 s (USB may enumerate late). After
 *	HEALTH_HOLD_AFTER crashes in a row, each within HEALTH_EARLY_CRASH_MS
 *	of boot, the board holds instead of starting (no tasks, no radio) and
 *	repeats the line every 2 s, so a reboot loop cannot hide it. USB
 *	console only.
 */
#include "app_watchdog.h"
#include "shared/watchdog_policy.h"

#define WDT_PERIOD_MS		100U	/* supervisor cyclic period */
#define WDT_REPORT_EVERY_MS	10000U
#define WDT_REPORT_FOR_MS	60000U
#define WDT_LOOP_MS		1000U
#define WDT_HOLD_REPORT_MS	2000U
#if defined(TM_WIFI_CYW43) && TM_WIFI_CYW43
#define WDT_WIFI		1
#else
#define WDT_WIFI		0	/* no network task to monitor */
#endif

static uint32_t reset_reason;		/* watchdog REASON bits at boot */
static health_record_t reset_record;
static BOOL reset_record_ok;

#if BOARD_WATCHDOG
static wdt_policy_t policy;		/* cyclic handler only */
static wdt_policy_config_t policy_config;
#endif
static ID cyc_watchdog;
/* Written by the handler, read by the report loop. */
static volatile uint32_t wdt_missing;	/* heartbeats holding up arming */
static volatile uint8_t wdt_armed;

/* Print a crash message string if it is still readable in flash (the same
 * image is running again): printable ASCII up to a NUL, else nothing. */
static void print_flash_text(uint32_t address)
{
	char text[61];
	unsigned n = 0;

	if (address < 0x10000000U || address >= 0x10200000U - sizeof(text)) return;
	for (const char *s = (const char *)address; n < sizeof(text) - 1U; ++s, ++n) {
		if (*s == '\0') break;
		if (*s < 0x20 || *s > 0x7E) return;
		text[n] = *s;
	}
	text[n] = '\0';
	if (n > 0U) tm_printf((UB *)", message \"%s\"", text);
}

static void print_names(uint32_t mask)
{
	const char *sep = "";

	for (unsigned i = 0; i < HEALTH_HB_COUNT; ++i) {
		if (!(mask & (1U << i))) continue;
		tm_printf((UB *)"%s%s", sep, wdt_heartbeat_name(i));
		sep = ", ";
	}
}

static void print_crash_count(const health_record_t *r)
{
	if (r->crash_count >= 2U)
		tm_printf((UB *)", early crash %u in a row", (unsigned)r->crash_count);
}

static void print_reset_line(BOOL first)
{
	if (reset_record_ok) {
		const health_record_t *r = &reset_record;
		switch (r->reason) {
		case HEALTH_REASON_HEARTBEAT:
			tm_printf((UB *)"[RESET] watchdog: %s heartbeat stale (no progress for %u ms; stale: ",
				  wdt_heartbeat_name(HEALTH_HB_VALUE_ID(r->value)), (unsigned)r->value2);
			print_names(HEALTH_HB_VALUE_MASK(r->value));
			tm_printf((UB *)"), uptime %u ms\n", (unsigned)r->uptime_ms);
			return;
		case HEALTH_REASON_HARD_FAULT:
			tm_printf((UB *)"[RESET] previous run ended: hard fault, pc 0x%08x, lr 0x%08x, uptime %u ms",
				  (unsigned)r->value, (unsigned)r->value2, (unsigned)r->uptime_ms);
			print_crash_count(r);
			tm_printf((UB *)"\n");
			return;
		case HEALTH_REASON_ARMED:
			/* Armed and fed, then a timeout with no other record. */
			if (reset_reason & HEALTH_RESET_WATCHDOG_TIMER) {
				tm_printf((UB *)"[RESET] watchdog: the supervisor stopped feeding with no stale loop recorded "
					  "(core-0 tick or interrupts stuck, or core 1 stopped); last feed at uptime %u ms\n",
					  (unsigned)r->uptime_ms);
				return;
			}
			break;	/* not a watchdog timeout: the REASON bits decide */
		default:
			tm_printf((UB *)"[RESET] previous run ended: %s, caller 0x%08x, uptime %u ms",
				  wdt_reason_name(r->reason), (unsigned)r->value,
				  (unsigned)r->uptime_ms);
			print_flash_text(r->value2);
			print_crash_count(r);
			tm_printf((UB *)"\n");
			return;
		}
	}
	if (reset_reason & HEALTH_RESET_WATCHDOG_FORCE) {
		tm_printf((UB *)"[RESET] forced watchdog reboot, no crash record\n");
	} else if (reset_reason & HEALTH_RESET_WATCHDOG_TIMER) {
		tm_printf((UB *)"[RESET] watchdog reboot, no crash record (normal after a UF2 flash)\n");
	} else if (first) {
		tm_printf((UB *)"[RESET] power-on\n");
	}
}

EXPORT BOOL app_reset_capture(void)
{
	uint32_t word[4];

	/* Also decides the hold (health_boot_hold): the record shows
	 * HEALTH_HOLD_AFTER early crashes in a row. */
	health_boot_read(&reset_reason, word);
	reset_record_ok = health_record_decode(word, &reset_record) ? TRUE : FALSE;
	print_reset_line(TRUE);
	return health_boot_hold ? TRUE : FALSE;
}

EXPORT void app_boot_hold_loop(void)
{
	for (;;) {
		tm_printf((UB *)"[RESET] %u crashes in a row, each within %u ms of boot: holding here "
			  "(motors, sensors and Wi-Fi not started) so this can be read. Power-cycle to try again.\n",
			  (unsigned)reset_record.crash_count, (unsigned)HEALTH_EARLY_CRASH_MS);
		print_reset_line(FALSE);
		(void)tk_dly_tsk(WDT_HOLD_REPORT_MS);
	}
}

#if BOARD_WATCHDOG
/* Cyclic handler: interrupt context, never waits. */
static void supervisor_handler(void *exinf)
{
	uint32_t count[HEALTH_HB_COUNT];
	uint32_t now = (uint32_t)(health_uptime_us() / 1000U);
	BOOL boot_done = !WDT_WIFI || health_net_ready != 0U;

	(void)exinf;
	/* A crash path (panic, assert, HardFault) owns the reboot and the
	 * record from now on: no feeding, no overwrite. */
	if (health_crash_pending) return;
	for (unsigned i = 0; i < HEALTH_HB_COUNT; ++i) count[i] = health_heartbeat[i];
	switch (wdt_policy_step(&policy, &policy_config, count, now, boot_done)) {
	case WDT_ARM:
		health_watchdog_start(BOARD_WATCHDOG_TIMEOUT_MS);
		/* Not a crash: kept fresh by every feed, it is what a timeout
		 * with no other record leaves behind (the supervisor stopped). */
		if (!health_crash_pending) health_record_write(HEALTH_REASON_ARMED, 0U, 0U);
		wdt_armed = 1;
		break;
	case WDT_FEED:
		health_watchdog_feed();
		break;
	case WDT_STALE:
		/* Reboot now rather than after the watchdog timeout: the motors
		 * stop about 1.1 s after the stale loop's last beat, not 2.5 s.
		 * The timeout stays as the backup. */
		if (health_crash_pending) break;
		health_record_write(HEALTH_REASON_HEARTBEAT,
				    HEALTH_HB_VALUE(policy.stale_id, policy.stale_mask),
				    policy.stale_age_ms);
		health_reboot_now();
		break;
	case WDT_WAIT:
		wdt_missing = (boot_done ? 0U : 1U << HEALTH_HB_NET) |
			wdt_policy_missing(&policy, &policy_config, now);
		break;
	default:
		break;
	}
}
#endif

EXPORT ER app_watchdog_start(void)
{
#if BOARD_WATCHDOG
	T_CCYC c = {0};

	wdt_policy_init(&policy);
	policy_config.window_ms = BOARD_WATCHDOG_WINDOW_MS;
	policy_config.grace_ms = BOARD_WATCHDOG_GRACE_MS;
	policy_config.required_mask = (1U << HEALTH_HB_COUNT) - 1U;
	if (!WDT_WIFI) policy_config.required_mask &= ~(1U << HEALTH_HB_NET);
	wdt_missing = policy_config.required_mask;
	c.cycatr = TA_HLNG | TA_STA;
	c.cychdr = (FP)supervisor_handler;
	c.cyctim = WDT_PERIOD_MS;
	c.cycphs = WDT_PERIOD_MS;
#if TK_SUPPORT_SMP
	c.cycatr |= TA_ASSPRC;
	c.assprc = TP_PRC2;
#endif
	cyc_watchdog = tk_cre_cyc(&c);
	if (cyc_watchdog < E_OK) return cyc_watchdog;
	tm_printf((UB *)"[WDT] supervisor started: arms after %u ms once every loop runs; timeout %u ms, window %u ms\n",
		  (unsigned)BOARD_WATCHDOG_GRACE_MS, (unsigned)BOARD_WATCHDOG_TIMEOUT_MS,
		  (unsigned)BOARD_WATCHDOG_WINDOW_MS);
#else
	tm_printf((UB *)"[WDT] off (BOARD_WATCHDOG 0)\n");
#endif
	return E_OK;
}

static void print_missing(const char *prefix, uint32_t missing)
{
	tm_printf((UB *)"%s", prefix);
	for (unsigned i = 0; i < HEALTH_HB_COUNT; ++i)
		if (missing & (1U << i)) tm_printf((UB *)" %s", wdt_heartbeat_name(i));
	tm_printf((UB *)"\n");
}

EXPORT void app_watchdog_report_loop(void)
{
	BOOL armed_shown = FALSE;

	/* Wakes once a second (a few instructions) for the life of the run,
	 * so a late arming is still reported. */
	for (uint32_t t = WDT_LOOP_MS; ; t += WDT_LOOP_MS) {
		(void)tk_dly_tsk(WDT_LOOP_MS);
		if (t <= WDT_REPORT_FOR_MS && t % WDT_REPORT_EVERY_MS == 0U) print_reset_line(FALSE);
		if (cyc_watchdog > 0 && wdt_armed && !armed_shown) {
			tm_printf((UB *)"[WDT] armed: every monitored loop is running\n");
			armed_shown = TRUE;
		}
		/* Still not armed after the grace: name the loops holding it up
		 * (never started, or stalled), every 10 s for 40 s. */
		if (cyc_watchdog > 0 && !wdt_armed && t > BOARD_WATCHDOG_GRACE_MS &&
		    t <= BOARD_WATCHDOG_GRACE_MS + 4U * WDT_REPORT_EVERY_MS &&
		    t % WDT_REPORT_EVERY_MS == 0U)
			print_missing("[WDT] not armed yet (watchdog off), waiting for:", wdt_missing);
	}
}
