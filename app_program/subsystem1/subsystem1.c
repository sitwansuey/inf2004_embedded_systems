/*
 *	subsystem1.c
 *	Buddy 1 -- telemetry inbox and retained example LED heartbeat.
 *
 *	APIs shown: tk_cre_cyc, tk_sta_cyc
 */

#include "subsystem1.h"

#if defined(TM_NET_LWIP) && TM_NET_LWIP && defined(TM_MQTT) && TM_MQTT
#include "wifi_secrets.h"
#endif

EXPORT ID	tid_led;	/* IF-05, see subsystem1.h */
EXPORT ID	cyc_led;

EXPORT ER subsystem1_init(void)
{
	T_CCYC	ccyc = {0};
	ER er = s1_telemetry_init();
	if (er < E_OK) return er;

	/* Cyclic handler: once started, the kernel calls led_cyclic_handler
	   every S1_TICK_MS.  Without TA_STA it is created stopped; it is
	   started in subsystem1_start(), once the task it wakes exists. */
	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)led_cyclic_handler;
	ccyc.cyctim = S1_TICK_MS;	/* period, ms */
	ccyc.cycphs = S1_TICK_MS;	/* delay before the first call, ms */
	cyc_led = tk_cre_cyc(&ccyc);
	return (cyc_led < E_OK) ? cyc_led : E_OK;
}

EXPORT ER subsystem1_start(void)
{
#if defined(TM_NET_LWIP) && TM_NET_LWIP && defined(TM_MQTT) && TM_MQTT
	/* The built-in service connects after Wi-Fi gets an IP. Do not call
	 * mqtt_utk_start() here: lwIP belongs to cyw43_utk_task, on its core.
	 * Log configuration only, never SSID passwords or broker credentials.
	 */
	tm_printf((UB*)"S1: MQTT via Wi-Fi service: " MQTT_BROKER_IP
	          ":%d, topic=" MQTT_TOPIC "\n", MQTT_BROKER_PORT);
	tm_printf((UB*)"S1: awaiting service connection; watch mqtt: connected / TX / RX\n");
#else
	tm_printf((UB*)"S1: MQTT disabled; build with NET=lwip MQTT=1\n");
#endif
	tid_led = app_start_task(led_task, S1_PRI_LED, 0, "led_task");
	if (tid_led < E_OK) {
		return tid_led;
	}
	return tk_sta_cyc(cyc_led);	/* start the cyclic handler now */
}
