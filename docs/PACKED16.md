# Packed16 is now the shared EDR16 standard

The full repository contract is [DATA_REPRESENTATION.md](DATA_REPRESENTATION.md).
B1's `packed16.h` aliases `app_program/shared/edr.h`; it no longer defines an
independent format. Use `Robopico/sensors/edr16/v1` and two-byte little-endian
records. The old B1-only topic is retired. IDs 1–10 retain their assignments;
11 is motor status, 12 motor faults, and 13–15 accelerometer X/Y/Z.
