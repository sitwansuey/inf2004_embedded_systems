/*
 *	wifi_secrets.example.h
 *
 *	Copy this file to wifi_secrets.h (same directory) and fill in your own
 *	network's credentials. wifi_secrets.h is listed in .gitignore -- it
 *	will never be committed, so it's safe to put your real password there.
 *
 *	Only used when built with NET=lwip (see build_make/pico_rp2040.mk and
 *	lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c).
 */

#ifndef WIFI_SECRETS_H
#define WIFI_SECRETS_H

#define WIFI_SSID     "your-network-name"
#define WIFI_PASSWORD "your-network-password"

/* Only used when built with MQTT=1 (also requires NET=lwip). */
#define MQTT_BROKER_IP   "192.168.1.10"
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID   "mtk3smp-pico"
#define MQTT_TOPIC       "mtk3smp-pico/heartbeat"

/* Optional broker login. NULL (or leaving both out) connects anonymously,
 * as before. With a broker that requires a login and an ACL that lets only
 * the dashboard publish csc02/command, other devices on the network cannot
 * command the robot. Give the robot its own account, for example:
 *   #define MQTT_USERNAME "robopico"
 *   #define MQTT_PASSWORD "robot-account-password"
 * The mosquitto password file and ACL are in
 * app_program/external_systems/Dashboard/README.md ("Broker access"). */
#define MQTT_USERNAME    NULL
#define MQTT_PASSWORD    NULL

#endif /* WIFI_SECRETS_H */
