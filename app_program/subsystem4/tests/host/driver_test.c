/* SPDX-License-Identifier: Apache-2.0 */
#include "drv_lsm303dlhc.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition); \
    exit(EXIT_FAILURE); } } while (0)

typedef struct
{
    uint8_t accel[256];
    uint8_t mag[256];
    uint8_t fail_address;
    uint8_t fail_register;
    bool fail_reads;
    bool fail_writes;
    bool corrupt_readback;
    unsigned int reads;
    unsigned int writes;
    unsigned int accel_bursts;
    unsigned int mag_bursts;
    uint64_t time_us;
} mock_bus_t;

static bool mock_read(void *p_context, uint8_t address, uint8_t subaddress,
                      uint8_t *p_data, uint32_t length)
{
    mock_bus_t *p_mock = p_context;
    uint8_t *p_registers;
    size_t reg = subaddress;

    p_mock->reads++;
    if (p_mock->fail_reads && (address == p_mock->fail_address) &&
        (subaddress == p_mock->fail_register))
    {
        /* A failed burst may have written a partial buffer. The driver
         * must never publish or decode it as a complete sample. */
        p_data[0] = 0xee;
        return false;
    }
    if (address == LSM303_ACCEL_ADDRESS)
    {
        CHECK(subaddress >= 0x20); /* No undocumented WHO_AM_I. */
        p_registers = p_mock->accel;
        if (length > 1)
        {
            CHECK(subaddress == 0xa8);
            CHECK(length == 6);
            p_mock->accel_bursts++;
            reg &= 0x7f;
            p_mock->accel[0x27] = 0;
        }
    }
    else
    {
        CHECK(address == LSM303_MAG_ADDRESS);
        CHECK((subaddress & 0x80) == 0); /* Mag has no accel-style bit. */
        p_registers = p_mock->mag;
        if (subaddress == 0x03)
        {
            CHECK(length == 6);
            p_mock->mag_bursts++;
            p_mock->mag[0x09] = 0;
        }
    }
    CHECK((reg + length) <= 256);
    memcpy(p_data, &p_registers[reg], length);
    if (p_mock->corrupt_readback && (address == LSM303_ACCEL_ADDRESS) &&
        (subaddress == 0x23))
    {
        p_data[0] ^= 0x08;
    }
    return true;
}

static bool mock_write(void *p_context, uint8_t address, uint8_t reg,
                       uint8_t value)
{
    mock_bus_t *p_mock = p_context;
    uint8_t *p_registers;

    p_mock->writes++;
    if (p_mock->fail_writes && (address == p_mock->fail_address) &&
        (reg == p_mock->fail_register))
    {
        return false;
    }
    if (address == LSM303_ACCEL_ADDRESS)
    {
        CHECK((reg >= 0x20) && (reg <= 0x2e));
        p_registers = p_mock->accel;
    }
    else
    {
        CHECK(address == LSM303_MAG_ADDRESS);
        CHECK(reg <= 0x02);
        p_registers = p_mock->mag;
    }
    p_registers[reg] = value;
    return true;
}

static uint64_t mock_now(void *p_context)
{
    mock_bus_t *p_mock = p_context;

    p_mock->time_us += 1000;
    return p_mock->time_us;
}

static lsm303_bus_t reset_mock(mock_bus_t *p_mock)
{
    *p_mock = (mock_bus_t){ 0 };
    p_mock->mag[0x0a] = 0x48;
    p_mock->mag[0x0b] = 0x34;
    p_mock->mag[0x0c] = 0x33;
    return (lsm303_bus_t)
    {
        .p_context = p_mock,
        .read = mock_read,
        .write = mock_write,
        .now_us = mock_now
    };
}

static void set_accel(mock_bus_t *p_mock, int16_t x, int16_t y, int16_t z)
{
    int16_t raw[3] = { x, y, z };
    size_t axis;

    for (axis = 0; axis < 3; axis++)
    {
        uint16_t word = (uint16_t)(((uint16_t)raw[axis] & 0x0fff) << 4);

        p_mock->accel[0x28 + axis * 2] = (uint8_t)word;
        p_mock->accel[0x29 + axis * 2] = (uint8_t)(word >> 8);
    }
    p_mock->accel[0x27] = 0x08;
}

static void set_mag(mock_bus_t *p_mock, int16_t x, int16_t y, int16_t z)
{
    /* Test input is XYZ; device registers are XZY. */
    int16_t native[3] = { x, z, y };
    size_t axis;

    for (axis = 0; axis < 3; axis++)
    {
        uint16_t word = (uint16_t)native[axis];

        p_mock->mag[0x03 + axis * 2] = (uint8_t)(word >> 8);
        p_mock->mag[0x04 + axis * 2] = (uint8_t)word;
    }
    p_mock->mag[0x09] = 0x01;
}

static void test_configuration(void)
{
    mock_bus_t mock;
    lsm303_bus_t bus = reset_mock(&mock);
    lsm303_device_t device = { 0 };

    CHECK(lsm303dlhc_init(&device, &bus) == LSM303_OK);
    CHECK(device.initialized);
    CHECK(mock.accel[0x20] == 0x47);
    CHECK(mock.accel[0x21] == 0);
    CHECK(mock.accel[0x23] == 0x88);
    CHECK(mock.accel[0x24] == 0);
    CHECK(mock.accel[0x2e] == 0);
    CHECK(mock.mag[0] == 0x14);
    CHECK(mock.mag[1] == 0x20);
    CHECK(mock.mag[2] == 0);

    mock.mag[0x0a] = 0;
    CHECK(lsm303dlhc_init(&device, &bus) == LSM303_WRONG_ID);
    CHECK(!device.initialized);
    bus = reset_mock(&mock);
    mock.corrupt_readback = true;
    CHECK(lsm303dlhc_init(&device, &bus) == LSM303_CONFIG_ERROR);
    CHECK(!device.initialized);
    bus = reset_mock(&mock);
    mock.fail_reads = true;
    mock.fail_address = LSM303_MAG_ADDRESS;
    mock.fail_register = 0x0a;
    CHECK(lsm303dlhc_init(&device, &bus) == LSM303_IO_ERROR);
    CHECK(!device.initialized);
    bus = reset_mock(&mock);
    mock.fail_writes = true;
    mock.fail_address = LSM303_ACCEL_ADDRESS;
    mock.fail_register = 0x23;
    CHECK(lsm303dlhc_init(&device, &bus) == LSM303_IO_ERROR);
    CHECK(!device.initialized);
}

static void test_decode_and_freshness(void)
{
    mock_bus_t mock;
    lsm303_bus_t bus = reset_mock(&mock);
    lsm303_device_t device = { 0 };
    lsm303_frame_t frame;

    CHECK(lsm303dlhc_init(&device, &bus) == LSM303_OK);
    set_accel(&mock, 1000, -1000, -1);
    set_mag(&mock, 1100, -550, 980);
    CHECK(lsm303dlhc_poll(&device, &frame) == LSM303_OK);
    CHECK(frame.accel.fresh && frame.accel.valid);
    CHECK(frame.mag.fresh && frame.mag.valid);
    CHECK(frame.accel.milli_units[0] == 1000);
    CHECK(frame.accel.milli_units[1] == -1000);
    CHECK(frame.accel.raw[2] == -1);
    CHECK(frame.mag.raw[0] == 1100);
    CHECK(frame.mag.raw[1] == -550);
    CHECK(frame.mag.raw[2] == 980);
    CHECK(frame.mag.milli_units[0] == 1000);
    CHECK(frame.mag.milli_units[1] == -500);
    CHECK(frame.mag.milli_units[2] == 1000);
    CHECK(frame.accel.read_time_us == 1000);
    CHECK(frame.mag.read_time_us == 2000);
    CHECK(mock.accel_bursts == 1 && mock.mag_bursts == 1);

    CHECK(lsm303dlhc_poll(&device, &frame) == LSM303_OK);
    CHECK(!frame.accel.fresh && !frame.mag.fresh);
    CHECK(frame.accel.status == LSM303_NO_DATA);
    CHECK(frame.mag.status == LSM303_NO_DATA);
    CHECK(frame.accel.read_time_us == 0 && frame.mag.read_time_us == 0);
    CHECK(mock.accel_bursts == 1 && mock.mag_bursts == 1);

    set_accel(&mock, 0, 1, -2);
    mock.accel[0x27] |= 0x80;
    CHECK(lsm303dlhc_poll(&device, &frame) == LSM303_OK);
    CHECK(frame.accel.fresh && !frame.mag.fresh);
    CHECK(frame.accel.overrun && frame.accel.valid);
    CHECK(frame.accel.read_time_us == 3000);
    set_mag(&mock, -1, 1, -980);
    CHECK(lsm303dlhc_poll(&device, &frame) == LSM303_OK);
    CHECK(!frame.accel.fresh && frame.mag.fresh);
    CHECK(frame.mag.raw[0] == -1 && frame.mag.milli_units[2] == -1000);
}

static void test_clipping(void)
{
    mock_bus_t mock;
    lsm303_bus_t bus = reset_mock(&mock);
    lsm303_device_t device = { 0 };
    lsm303_frame_t frame;

    CHECK(lsm303dlhc_init(&device, &bus) == LSM303_OK);
    set_accel(&mock, -2048, 2047, 0);
    set_mag(&mock, -4096, 0, 0);
    CHECK(lsm303dlhc_poll(&device, &frame) == LSM303_OK);
    CHECK(frame.accel.raw[0] == -2048 && frame.accel.raw[1] == 2047);
    CHECK(frame.accel.fresh && !frame.accel.valid);
    CHECK(frame.mag.raw[0] == -4096);
    CHECK(frame.mag.fresh && !frame.mag.valid);
    CHECK(frame.mag.status == LSM303_DATA_CLIPPED);
    set_mag(&mock, 0, 2047, 0);
    CHECK(lsm303dlhc_poll(&device, &frame) == LSM303_OK);
    CHECK(!frame.mag.valid);
}

static void test_independent_failures(void)
{
    mock_bus_t mock;
    lsm303_bus_t bus = reset_mock(&mock);
    lsm303_device_t device = { 0 };
    lsm303_frame_t frame;

    CHECK(lsm303dlhc_init(&device, &bus) == LSM303_OK);
    set_accel(&mock, 0, 0, 1000);
    set_mag(&mock, 110, 220, 98);
    mock.fail_reads = true;
    mock.fail_address = LSM303_MAG_ADDRESS;
    mock.fail_register = 0x03;
    CHECK(lsm303dlhc_poll(&device, &frame) == LSM303_IO_ERROR);
    CHECK(frame.accel.fresh && frame.accel.valid);
    CHECK(frame.accel.milli_units[2] == 1000);
    CHECK(!frame.mag.fresh && !frame.mag.valid);
    CHECK(frame.mag.status == LSM303_DATA_IO_ERROR && frame.mag.read_time_us == 0);

    set_accel(&mock, 1, 2, 3);
    mock.fail_address = LSM303_ACCEL_ADDRESS;
    mock.fail_register = 0xa8;
    CHECK(lsm303dlhc_poll(&device, &frame) == LSM303_IO_ERROR);
    CHECK(!frame.accel.fresh && !frame.accel.valid);
    CHECK(frame.mag.fresh && frame.mag.valid);
    CHECK(frame.mag.milli_units[2] == 100);

    set_mag(&mock, 0, 0, 98);
    mock.fail_register = 0x27;
    CHECK(lsm303dlhc_poll(&device, &frame) == LSM303_IO_ERROR);
    CHECK(frame.accel.status == LSM303_DATA_IO_ERROR);
    CHECK(frame.mag.fresh && frame.mag.valid);
    mock.fail_address = LSM303_MAG_ADDRESS;
    mock.fail_register = 0x09;
    set_accel(&mock, 0, 0, 1000);
    CHECK(lsm303dlhc_poll(&device, &frame) == LSM303_IO_ERROR);
    CHECK(frame.accel.fresh && frame.mag.status == LSM303_DATA_IO_ERROR);
}

static void test_lock_recovery_and_arguments(void)
{
    mock_bus_t mock;
    lsm303_bus_t bus = reset_mock(&mock);
    lsm303_device_t device = { 0 };
    lsm303_frame_t frame;

    CHECK(lsm303dlhc_poll(&device, &frame) == LSM303_NOT_INITIALIZED);
    CHECK(lsm303dlhc_init(NULL, &bus) == LSM303_BAD_ARGUMENT);
    CHECK(lsm303dlhc_poll(NULL, &frame) == LSM303_BAD_ARGUMENT);
    CHECK(lsm303dlhc_poll(&device, NULL) == LSM303_BAD_ARGUMENT);
    CHECK(lsm303dlhc_init(&device, &bus) == LSM303_OK);
    set_mag(&mock, 110, 220, 98);
    mock.mag[0x09] |= 0x02;
    CHECK(lsm303dlhc_poll(&device, &frame) == LSM303_OK);
    CHECK(frame.mag.status == LSM303_LOCK_DRAINED);
    CHECK(!frame.mag.fresh && frame.mag.read_time_us == 0);
    CHECK(mock.mag[0x09] == 0 && mock.mag_bursts == 1);
    set_mag(&mock, 110, 220, 98);
    CHECK(lsm303dlhc_poll(&device, &frame) == LSM303_OK);
    CHECK(frame.mag.fresh && frame.mag.valid);
    bus.now_us = NULL;
    CHECK(lsm303dlhc_init(&device, &bus) == LSM303_BAD_ARGUMENT);
    CHECK(!device.initialized);
}

int main(void)
{
    test_configuration();
    puts("[TEST] S4-U10 PASS  sensor identity/configuration and initialization faults");
    test_decode_and_freshness();
    puts("[TEST] S4-U11 PASS  signed decoding, XYZ ordering, units and freshness");
    test_clipping();
    puts("[TEST] S4-U12 PASS  clipped/out-of-range samples are invalid");
    test_independent_failures();
    puts("[TEST] S4-U13 PASS  partial reads and independent channel failures");
    test_lock_recovery_and_arguments();
    puts("[TEST] S4-U14 PASS  magnetic lock recovery and invalid API arguments");
    puts("PASS: configuration/identity, signed decoding/axis order/units,");
    puts("      freshness/timestamps/overrun, clipping, independent I/O failures,");
    puts("      partial-read rejection, lock recovery and invalid arguments.");
    return EXIT_SUCCESS;
}
