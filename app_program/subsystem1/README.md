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

Robot sensor JSON publication, command routing and the dashboard MQTT receiver
remain separate integration work. This setup enables the existing connection
path and optional authentication; it does not implement those data interfaces.
