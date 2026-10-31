# Buddy 1 — shared EDR16 telemetry

The common format is [DATA_REPRESENTATION.md](../../docs/DATA_REPRESENTATION.md).
All buddies use `app_program/shared/edr.h`. Each record is exactly two bytes:
12-bit data above a 4-bit reading ID, little-endian. MQTT topic:
`Robopico/sensors/edr16/v1`. The packed16 header provides old-name aliases only.

With `NET=lwip MQTT=1`, priority-13 telemetry runs every 250 ms and collects B2
encoder/motor records, fresh B3 IR, and usable B4 magnetic/acceleration samples.
B5 must call the send API for servo and ultrasonic readings; its public getters
are absent. Invalid readings are skipped, queue-full drops the observation.

Call `s1_telemetry_record(bytes, 2)` for an encoded record, or
`s1_telemetry_u12(id, value)` for unsigned data. Magnetic/acceleration helpers
accept signed mG/mg and add 2048. Every call copies data and returns immediately:
E_PAR invalid input, E_OBJ uninitialized, E_TMOUT full, E_NOSPT network disabled.
Only the network service calls MQTT/lwIP; it retries a staged publish with the
same explicit length and sends up to four messages per visit. QoS is zero.

Configure local `config/wifi_secrets.h` and build from build_make with
`make -j8 NET=lwip MQTT=1`. Do not commit credentials. Other repository build
gaps may need resolving before linking. No real Pico/broker test is claimed.

The legacy text APIs remain for optional status/events outside EDR16. The
GP19 LED example remains at priority 12. Neither telemetry nor the dashboard
sends movement commands. Local controller snapshots retain their full precision.
Run the shared `testing/data_representation/run.py` gate before integration.
