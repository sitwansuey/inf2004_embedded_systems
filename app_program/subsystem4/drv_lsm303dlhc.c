/* SPDX-License-Identifier: Apache-2.0 */
#include "drv_lsm303dlhc.h"

#include <stddef.h>

#define ACCEL_CTRL1          UINT8_C(0x20)
#define ACCEL_CTRL2          UINT8_C(0x21)
#define ACCEL_CTRL3          UINT8_C(0x22)
#define ACCEL_CTRL4          UINT8_C(0x23)
#define ACCEL_CTRL5          UINT8_C(0x24)
#define ACCEL_CTRL6          UINT8_C(0x25)
#define ACCEL_STATUS         UINT8_C(0x27)
#define ACCEL_OUTPUT_BURST   UINT8_C(0xa8) /* 0x28 | auto-increment bit. */
#define ACCEL_FIFO_CTRL      UINT8_C(0x2e)
#define ACCEL_READY          UINT8_C(0x08)
#define ACCEL_OVERRUN        UINT8_C(0x80)
#define MAG_CRA              UINT8_C(0x00)
#define MAG_CRB              UINT8_C(0x01)
#define MAG_MODE             UINT8_C(0x02)
#define MAG_OUTPUT          UINT8_C(0x03) /* Native X, Z, Y order. */
#define MAG_STATUS          UINT8_C(0x09)
#define MAG_ID              UINT8_C(0x0a)
#define MAG_READY           UINT8_C(0x01)
#define MAG_LOCK            UINT8_C(0x02)

typedef struct
{
    uint8_t address;
    uint8_t reg;
    uint8_t value;
} config_register_t;

static int16_t decode_accel(const uint8_t *p_bytes)
{
    uint16_t word = (uint16_t)((uint16_t)p_bytes[0] |
                               ((uint16_t)p_bytes[1] << 8));
    int32_t value = (int32_t)(word >> 4);

    /* Do not right-shift a signed negative value: that is implementation
     * defined in C11. Sign-extend the left-aligned 12-bit sample explicitly. */
    if (value >= 2048)
    {
        value -= 4096;
    }
    return (int16_t)value;
}

static int16_t decode_mag(const uint8_t *p_bytes)
{
    uint16_t word = (uint16_t)(((uint16_t)p_bytes[0] << 8) |
                               (uint16_t)p_bytes[1]);
    int32_t value = (int32_t)word;

    if (value >= 32768)
    {
        value -= 65536;
    }
    return (int16_t)value;
}

lsm303_result_t lsm303dlhc_init(lsm303_device_t *p_device,
                              const lsm303_bus_t *p_bus)
{
    /* Configure while both converters are stopped, then start them last. */
    static const config_register_t config[] =
    {
        { LSM303_ACCEL_ADDRESS, ACCEL_CTRL1, 0x07 },
        { LSM303_MAG_ADDRESS, MAG_MODE, 0x03 },
        { LSM303_ACCEL_ADDRESS, ACCEL_CTRL2, 0x00 },
        { LSM303_ACCEL_ADDRESS, ACCEL_CTRL3, 0x00 },
        { LSM303_ACCEL_ADDRESS, ACCEL_CTRL4, 0x88 },
        { LSM303_ACCEL_ADDRESS, ACCEL_CTRL5, 0x00 },
        { LSM303_ACCEL_ADDRESS, ACCEL_CTRL6, 0x00 },
        { LSM303_ACCEL_ADDRESS, ACCEL_FIFO_CTRL, 0x00 },
        { LSM303_MAG_ADDRESS, MAG_CRA, 0x14 },
        { LSM303_MAG_ADDRESS, MAG_CRB, 0x20 },
        { LSM303_ACCEL_ADDRESS, ACCEL_CTRL1, 0x47 },
        { LSM303_MAG_ADDRESS, MAG_MODE, 0x00 }
    };
    uint8_t identity[3];
    uint8_t value;
    size_t index;

    if (p_device == NULL)
    {
        return LSM303_BAD_ARGUMENT;
    }
    p_device->initialized = false;
    if ((p_bus == NULL) || (p_bus->read == NULL) || (p_bus->write == NULL) ||
        (p_bus->now_us == NULL))
    {
        return LSM303_BAD_ARGUMENT;
    }
    p_device->bus = *p_bus;
    if (!p_bus->read(p_bus->p_context, LSM303_MAG_ADDRESS, MAG_ID,
                     identity, sizeof(identity)))
    {
        return LSM303_IO_ERROR;
    }
    if ((identity[0] != 0x48) || (identity[1] != 0x34) ||
        (identity[2] != 0x33))
    {
        return LSM303_WRONG_ID;
    }
    /* The DLHC datasheet reserves accel addresses 0x00..0x1f. Do not
     * require an undocumented WHO_AM_I at 0x0f. Verify configuration. */
    for (index = 0; index < (sizeof(config) / sizeof(config[0])); index++)
    {
        if (!p_bus->write(p_bus->p_context, config[index].address,
                          config[index].reg, config[index].value) ||
            !p_bus->read(p_bus->p_context, config[index].address,
                         config[index].reg, &value, 1))
        {
            return LSM303_IO_ERROR;
        }
        if (value != config[index].value)
        {
            return LSM303_CONFIG_ERROR;
        }
    }
    p_device->initialized = true;
    return LSM303_OK;
}

static bool poll_accel(const lsm303_bus_t *p_bus, lsm303_sample_t *p_sample)
{
    uint8_t status;
    uint8_t data[6];
    size_t axis;

    if (!p_bus->read(p_bus->p_context, LSM303_ACCEL_ADDRESS,
                     ACCEL_STATUS, &status, 1))
    {
        p_sample->status = LSM303_DATA_IO_ERROR;
        return false;
    }
    p_sample->overrun = (status & ACCEL_OVERRUN) != 0;
    if ((status & ACCEL_READY) == 0)
    {
        return true;
    }
    if (!p_bus->read(p_bus->p_context, LSM303_ACCEL_ADDRESS,
                     ACCEL_OUTPUT_BURST, data, sizeof(data)))
    {
        p_sample->status = LSM303_DATA_IO_ERROR;
        return false;
    }
    p_sample->read_time_us = p_bus->now_us(p_bus->p_context);
    p_sample->fresh = true;
    p_sample->valid = true;
    for (axis = 0; axis < 3; axis++)
    {
        p_sample->raw[axis] = decode_accel(&data[axis * 2]);
        /* +/-2 g high-resolution mode: 1 mg/LSB. */
        p_sample->milli_units[axis] = p_sample->raw[axis];
        if ((p_sample->raw[axis] == -2048) ||
            (p_sample->raw[axis] == 2047))
        {
            p_sample->valid = false;
        }
    }
    p_sample->status = p_sample->valid ? LSM303_DATA_OK : LSM303_DATA_CLIPPED;
    return true;
}

static bool poll_mag(const lsm303_bus_t *p_bus, lsm303_sample_t *p_sample)
{
    uint8_t status;
    uint8_t data[6];
    size_t axis;
    /* Convert native X,Z,Y into the same X,Y,Z order as acceleration. */
    static const size_t offset[3] = { 0, 4, 2 };
    static const int32_t gain[3] = { 1100, 1100, 980 };

    if (!p_bus->read(p_bus->p_context, LSM303_MAG_ADDRESS,
                     MAG_STATUS, &status, 1))
    {
        p_sample->status = LSM303_DATA_IO_ERROR;
        return false;
    }
    if ((status & (MAG_READY | MAG_LOCK)) == 0)
    {
        return true;
    }
    if (!p_bus->read(p_bus->p_context, LSM303_MAG_ADDRESS,
                     MAG_OUTPUT, data, sizeof(data)))
    {
        p_sample->status = LSM303_DATA_IO_ERROR;
        return false;
    }
    if ((status & MAG_LOCK) != 0)
    {
        /* Reading all six bytes clears a lock left by a partial read.
         * The drained bank is not a newly acquired sample. */
        p_sample->status = LSM303_LOCK_DRAINED;
        return true;
    }
    p_sample->read_time_us = p_bus->now_us(p_bus->p_context);
    p_sample->fresh = true;
    p_sample->valid = true;
    for (axis = 0; axis < 3; axis++)
    {
        p_sample->raw[axis] = decode_mag(&data[offset[axis]]);
        p_sample->milli_units[axis] =
            ((int32_t)p_sample->raw[axis] * 1000) / gain[axis];
        /* Datasheet output range is -2048..2047. This also rejects the
         * commonly observed -4096 overflow value without depending on it. */
        if ((p_sample->raw[axis] <= -2048) ||
            (p_sample->raw[axis] >= 2047))
        {
            p_sample->valid = false;
        }
    }
    p_sample->status = p_sample->valid ? LSM303_DATA_OK : LSM303_DATA_CLIPPED;
    return true;
}

lsm303_result_t lsm303dlhc_poll(lsm303_device_t *p_device,
                              lsm303_frame_t *p_frame)
{
    bool accel_ok;
    bool mag_ok;

    if (p_frame == NULL)
    {
        return LSM303_BAD_ARGUMENT;
    }
    *p_frame = (lsm303_frame_t){ 0 };
    if (p_device == NULL)
    {
        return LSM303_BAD_ARGUMENT;
    }
    if (!p_device->initialized)
    {
        return LSM303_NOT_INITIALIZED;
    }
    accel_ok = poll_accel(&p_device->bus, &p_frame->accel);
    mag_ok = poll_mag(&p_device->bus, &p_frame->mag);
    return (accel_ok && mag_ok) ? LSM303_OK : LSM303_IO_ERROR;
}
