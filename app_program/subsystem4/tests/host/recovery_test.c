/* SPDX-License-Identifier: Apache-2.0 */
/* s4_sensor_task's cycle (sensor_cycle.c) against a modelled GY-511, and the
 * I2C bus clear (i2c_bus_clear.h) against a modelled stuck slave. */
#include "sensor_cycle.h"
#include "i2c_bus_clear.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define REQUIRE(c) do { if (!(c)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #c); \
    exit(EXIT_FAILURE); } } while (0)

/* The accelerometer converts at 50 Hz only once CTRL_REG1_A is 0x47 (its
 * power-on default 0x07 is powered down): a new sample every second 10 ms
 * poll. The magnetometer has an identity and never new data. */
typedef struct
{
    uint8_t accel[256];
    uint8_t mag[256];
    bool accel_fail;
    bool mag_fail;
    unsigned status_reads;
    uint64_t time_us;
} model_t;

static model_t model;
static unsigned recovers;

static bool model_read(void *p_context, uint8_t address, uint8_t subaddress,
                       uint8_t *p_data, uint32_t length)
{
    model_t *p = p_context;

    if (address == LSM303_ACCEL_ADDRESS)
    {
        if (p->accel_fail) return false;
        if (subaddress == 0x27)
        {
            p->status_reads++;
            p_data[0] = (p->accel[0x20] == 0x47 && (p->status_reads % 2U) == 0U) ? 0x08 : 0;
            return true;
        }
        if (subaddress == 0xa8)
        {
            static const uint8_t level[6] = { 0, 0, 0, 0, 0x80, 0x3e }; /* z 1000 mg */
            REQUIRE(length == 6);
            memcpy(p_data, level, sizeof(level));
            return true;
        }
        memcpy(p_data, &p->accel[subaddress], length);
        return true;
    }
    REQUIRE(address == LSM303_MAG_ADDRESS);
    if (p->mag_fail) return false;
    if (subaddress == 0x09)
    {
        p_data[0] = 0;
        return true;
    }
    memcpy(p_data, &p->mag[subaddress], length);
    return true;
}

static bool model_write(void *p_context, uint8_t address, uint8_t reg, uint8_t value)
{
    model_t *p = p_context;

    if (address == LSM303_ACCEL_ADDRESS)
    {
        if (p->accel_fail) return false;
        p->accel[reg] = value;
        return true;
    }
    if (p->mag_fail) return false;
    p->mag[reg] = value;
    return true;
}

static uint64_t model_now(void *p_context)
{
    return ((model_t *)p_context)->time_us;
}

static void recover(void)
{
    ++recovers;
}

static s4_sensor_cycle_t cycle;
static s4_sensor_cache_t state;
static uint64_t now_ms;

static void start(void)
{
    lsm303_bus_t bus = { &model, model_read, model_write, model_now };

    memset(&model, 0, sizeof(model));
    model.accel[0x20] = 0x07;
    model.mag[0x0a] = 0x48;
    model.mag[0x0b] = 0x34;
    model.mag[0x0c] = 0x33;
    recovers = 0;
    now_ms = 1000;
    s4_data_reset(&state);
    s4_cycle_reset(&cycle, &bus, recover);
}

/* One 10 ms cycle; returns whether the accelerometer is usable after it. */
static bool run(void)
{
    s4_sensor_status_t view;

    now_ms += 10U;
    model.time_us = now_ms * 1000U;
    s4_cycle_run(&cycle, &state, now_ms);
    s4_data_snapshot(&state, now_ms * 1000U, &view);
    return view.accel_usable;
}

static s4_sensor_status_t view_now(void)
{
    s4_sensor_status_t view;

    s4_data_snapshot(&state, now_ms * 1000U, &view);
    return view;
}

/* I7: a magnetometer fault never takes the accelerometer down. */
static void test_magnetometer_only(void)
{
    start();
    for (unsigned n = 0; n < 10U; ++n) (void)run();
    REQUIRE(cycle.device.initialized && cycle.device.mag_ready && view_now().accel_usable);
    model.mag_fail = true;
    for (unsigned n = 0; n < 300U; ++n) REQUIRE(run());
    REQUIRE(view_now().init_attempts == 1 && view_now().mag_io_errors >= 290);
    REQUIRE(view_now().accel_faults == 0);
    /* Dead at power-up: the accelerometer still comes up; the magnetometer
     * set-up is retried every S4_RETRY_MS without touching it. */
    start();
    model.mag_fail = true;
    (void)run();
    REQUIRE(cycle.device.initialized && !cycle.device.mag_ready);
    for (unsigned n = 0; n < 150U; ++n) (void)run();
    REQUIRE(view_now().accel_usable && view_now().init_attempts == 1);
    model.mag_fail = false;
    for (unsigned n = 0; n < 100U && !cycle.device.mag_ready; ++n) REQUIRE(run());
    REQUIRE(cycle.device.mag_ready && view_now().init_attempts == 1);
    puts("[TEST] S4-U41 PASS  a dead or failing magnetometer never takes the accelerometer down");
}

/* I7: the first re-init runs on the next cycle; 1 s back-off only after a
 * failed attempt. A 100 ms bus glitch is over well inside the VC's 800 ms
 * tilt grace. I8: the bus is cleared before every init attempt. */
static void test_reinit_timing(void)
{
    unsigned cycles = 0;

    start();
    for (unsigned n = 0; n < 10U; ++n) (void)run();
    REQUIRE(view_now().init_attempts == 1 && recovers == 1);
    model.accel_fail = true;
    for (unsigned n = 0; n < S4_FAILED_POLLS; ++n) (void)run();
    REQUIRE(!cycle.device.initialized && view_now().init_attempts == 1);
    model.accel_fail = false;
    while (!run()) REQUIRE(++cycles < 80U);
    REQUIRE(cycles <= 3U && view_now().init_attempts == 2 && recovers == 2);
    /* A longer outage: the first retry at once, the next after S4_RETRY_MS. */
    model.accel_fail = true;
    for (unsigned n = 0; n < S4_FAILED_POLLS + 1U; ++n) (void)run();
    REQUIRE(view_now().init_attempts == 3 && recovers == 3);
    for (unsigned n = 0; n < S4_RETRY_MS / 10U - 1U; ++n) (void)run();
    REQUIRE(view_now().init_attempts == 3);
    (void)run();
    REQUIRE(view_now().init_attempts == 4 && recovers == 4);
    model.accel_fail = false;
    for (unsigned n = 0; n < S4_RETRY_MS / 10U + 2U; ++n) (void)run();
    REQUIRE(view_now().init_attempts == 5 && run());
    puts("[TEST] S4-U42 PASS  re-init on the next cycle, back-off only after a failed attempt, bus cleared first");
}

/* I10: a device that answers but sends no data (a supply dip restored its
 * power-down default) is re-initialised, and counted as a fault. */
static void test_answers_without_data(void)
{
    unsigned cycles = 0;
    uint32_t faults;

    start();
    for (unsigned n = 0; n < 10U; ++n) (void)run();
    REQUIRE(view_now().accel_usable && model.accel[0x20] == 0x47);
    faults = view_now().accel_faults;
    model.accel[0x20] = 0x07;
    while (run()) REQUIRE(++cycles < 80U); /* the last sample goes stale */
    while (!run()) REQUIRE(++cycles < 80U);
    REQUIRE(model.accel[0x20] == 0x47 && view_now().init_attempts == 2);
    /* Stale after 100 ms, re-init after 20 polls without data: well inside
     * the VC's 800 ms tilt grace. */
    REQUIRE(cycles * 10U <= 300U);
    REQUIRE(view_now().accel_faults >= faults + 2U);
    puts("[TEST] S4-U43 PASS  a sensor that answers without data is re-initialised");
}

/* Two open-drain lines and a slave that holds SDA low until it has been
 * clocked `hold` times (it changes SDA after SCL falls, like a transmitter). */
typedef struct
{
    bool scl_low;
    bool sda_low;
    unsigned hold;
    unsigned falls;
    unsigned rises;
    bool sda_was_high;
    bool stop;
} wire_t;

static bool wire_sda(const wire_t *p)
{
    return !p->sda_low && p->falls >= p->hold;
}

static void wire_update(wire_t *p)
{
    bool high = wire_sda(p);

    if (!p->scl_low && high && !p->sda_was_high) p->stop = true;
    p->sda_was_high = high;
}

static bool wire_read_sda(void *p_context)
{
    return wire_sda(p_context);
}

static void wire_scl(void *p_context, bool low)
{
    wire_t *p = p_context;

    if (low && !p->scl_low) p->falls++;
    if (!low && p->scl_low) p->rises++;
    p->scl_low = low;
    wire_update(p);
}

static void wire_drive_sda(void *p_context, bool low)
{
    wire_t *p = p_context;

    p->sda_low = low;
    wire_update(p);
}

static void wire_wait(void *p_context)
{
    (void)p_context;
}

static bool clear_bus(wire_t *p_wire, unsigned hold)
{
    const s4_i2c_lines_t lines = { p_wire, wire_read_sda, wire_scl, wire_drive_sda, wire_wait };

    *p_wire = (wire_t){ 0 };
    p_wire->hold = hold;
    p_wire->sda_was_high = wire_sda(p_wire);
    return s4_i2c_bus_clear(&lines);
}

static void test_bus_clear(void)
{
    wire_t wire;

    REQUIRE(clear_bus(&wire, 0) && wire.rises == 0 && wire.falls == 0);
    for (unsigned hold = 1; hold <= S4_I2C_CLEAR_CLOCKS; ++hold)
    {
        REQUIRE(clear_bus(&wire, hold));
        REQUIRE(wire.stop && wire.rises == hold && !wire.scl_low && !wire.sda_low);
    }
    REQUIRE(!clear_bus(&wire, S4_I2C_CLEAR_CLOCKS + 1U));
    REQUIRE(wire.rises == S4_I2C_CLEAR_CLOCKS && !wire.scl_low && !wire.sda_low);
    puts("[TEST] S4-U44 PASS  a held SDA is clocked free (up to 9 clocks) and ends with a STOP");
}

int main(void)
{
    test_magnetometer_only();
    test_reinit_timing();
    test_answers_without_data();
    test_bus_clear();
    return EXIT_SUCCESS;
}
