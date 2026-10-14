# Buddy 1 MQTT connection setup

The built-in `cyw43_utk_task` owns Wi-Fi and lwIP. After Wi-Fi obtains an IP,
it calls `mqtt_utk_start()` in its own context. Subsystem 1 logs the configured
broker/topic at startup; it must not call lwIP or MQTT functions itself.
The existing LED example remains available for the monitor and on-target tests.

## Configure locally

Edit `config/wifi_secrets.h` (ignored by Git):

- `WIFI_SSID`, `WIFI_PASSWORD`: your 2.4 GHz network.
- `MQTT_BROKER_IP`: broker IPv4 address on that network, not `localhost`.
- `MQTT_BROKER_PORT`: normally 1883 for plain MQTT.
- `MQTT_CLIENT_ID`: unique to this robot.
- `MQTT_TOPIC`: currently `robot/heartbeat`.
- `MQTT_USERNAME`, `MQTT_PASSWORD`: quoted strings if required, otherwise `NULL`.

The newly prepared local file contains placeholders, not working credentials.
It does not enable TLS. Do not use the heartbeat topic as sensor telemetry:
the current payload is still `hb <count>`, every five seconds.

## Build and test

From a WSL terminal with the Pico SDK and Arm toolchain configured:

```sh
cd build_make
make -j8 NET=lwip MQTT=1
```

Flash the resulting UF2 and watch the USB serial console. Expected progression:
Wi-Fi join/IP, `mqtt: connected to ...`, subscription success, then heartbeat
TX and matching RX. A broker subscriber listening on the configured topic should
receive the same heartbeat. Ensure the broker listens on a LAN interface and its
firewall permits the configured port. The Pico cannot reach a laptop-only
loopback listener.

Connection has not been tested against hardware or a real broker. The repository
also has unrelated integration gaps: the old producer/consumer/monitor refer to
removed Subsystem 2 sample interfaces, and the motor module requires a shared
`app_program/board_pins.h` absent from this checkout. Resolve those with the team
before expecting a complete firmware build.

## Sending readings from other buddies

Include `../subsystem1/subsystem1.h` from your subsystem and call the task-context
API after the team has initialized all subsystems:

```c
ER er;
er = s1_telemetry_integer(S1_LEFT_IR_IR_LEFT, 650);
er = s1_telemetry_integer(S1_ULTRASONIC_RANGER_DISTANCE_CM, 32);
er = s1_telemetry_boolean(S1_TERRAIN_VALID, TRUE);
er = s1_telemetry_text(S1_BARCODE_IR_BARCODE, "RIGHT");
/* Decimal values are formatted by the sender; no float printf is required. */
er = s1_telemetry_send(S1_ACCELEROMETER_ACCEL_X_MPS2, "0.12");
/* Check each result immediately in real code, not just the final result. */
```

The complete list of reading IDs and exact topic strings is in
`telemetry_topics.h`, matching the current dashboard. Scalar payloads are at
most 127 printable ASCII bytes. Use the text helper for quoted JSON strings
such as a barcode `001`, so the dashboard does not parse them as numbers.
Event JSON goes to `S1_VEHICLE_CONTROLLER_EVENTS`, for example:

```c
er = s1_telemetry_send(S1_VEHICLE_CONTROLLER_EVENTS,
    "{\"time\":\"00:12\",\"source\":\"Controller\",\"level\":\"INFO\","
    "\"message\":\"Turn started\"}");
```

This owns one RTOS message buffer with 16 copied messages, approximately 2.2 KB
of payload storage. Calls never wait: `E_TMOUT` means full, `E_OBJ` means not
initialized, `E_PAR` means invalid ID/size/characters, and `E_NOSPT` means the
MQTT/network build is disabled. Decide a bounded retry/drop policy in each
producer; do not block motor or safety tasks on telemetry. Safety requests go
directly to the controller as well.

The existing network task polls Buddy 1's inbox and publishes at most four
messages per service visit. No extra task or network owner is created. A staged
message is retried if lwIP cannot accept it. MQTT uses QoS 0 and non-retained
messages: local acceptance is not broker delivery confirmation. While offline,
the inbox fills; queued older samples are sent after reconnect. The internal
timestamp is captured at enqueue but is not included in scalar MQTT payloads.
Do not claim freshness/latency accuracy from these payloads.

Host checks (requires GCC or compatible compiler):

```sh
gcc -std=c99 -Wall -Wextra -Werror tests/host/telemetry_host_test.c -o s1-test
./s1-test
```

Run from this subsystem directory. The harness checks copied payload lifetime,
escaping, topic mapping, bounded queue full behavior and FIFO order. It does not
validate lwIP or hardware. Full Pico build/connection tests remain required.
Other buddies must call these APIs with real measurements; no fake sensor task
is started. Remote command routing and audio playback remain separate work.
