/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BUDDY4_LSM303DLHC_H
#define BUDDY4_LSM303DLHC_H

#include <stdbool.h>
#include <stdint.h>

#define LSM303_ACCEL_ADDRESS    UINT8_C(0x19)
#define LSM303_MAG_ADDRESS      UINT8_C(0x1e)

/* Callbacks return true only for a complete transfer. Enforce a timeout in
 * the adapter. One task owns this driver and its I2C controller. */
typedef struct
{
    void *p_context;
    bool (*read)(void *p_context, uint8_t address, uint8_t subaddress,
                 uint8_t *p_data, uint32_t length);
    bool (*write)(void *p_context, uint8_t address, uint8_t reg, uint8_t value);
    uint64_t (*now_us)(void *p_context);
} lsm303_bus_t;

typedef enum
{
    LSM303_OK = 0,
    LSM303_BAD_ARGUMENT,
    LSM303_NOT_INITIALIZED,
    LSM303_IO_ERROR,
    LSM303_WRONG_ID,
    LSM303_CONFIG_ERROR
} lsm303_result_t;

typedef enum
{
    LSM303_NO_DATA = 0,
    LSM303_DATA_OK,
    LSM303_DATA_CLIPPED,
    LSM303_DATA_IO_ERROR,
    LSM303_LOCK_DRAINED
} lsm303_data_status_t;

typedef struct
{
    bool fresh;
    bool valid;
    bool overrun;                /* Accelerometer only. */
    lsm303_data_status_t status;
    int16_t raw[3];               /* X, Y, Z in sensor axes. */
    int32_t milli_units[3];       /* Acceleration: mg; magnetic field: mG. */
    uint64_t read_time_us;        /* Read completion, NOT conversion time. */
} lsm303_sample_t;

typedef struct
{
    lsm303_sample_t accel;
    lsm303_sample_t mag;
} lsm303_frame_t;

typedef struct
{
    lsm303_bus_t bus;
    bool initialized;
} lsm303_device_t;

/* Fixed bring-up configuration: accel 50 Hz, +/-2 g, high resolution, BDU,
 * no high-pass filter/FIFO; magnetometer 30 Hz, +/-1.3 G, continuous mode.
 * Reinitialization invalidates the previous initialized state on failure. */
lsm303_result_t lsm303dlhc_init(lsm303_device_t *p_device,
                              const lsm303_bus_t *p_bus);

/* Poll each channel independently. Only fresh samples carry a timestamp.
 * NO_DATA is normal at different ODRs. If one channel fails, a successful
 * sample from the other is preserved, even when the return value is IO_ERROR.
 * A locked magnetic register bank is drained and discarded to recover from
 * an interrupted read. Clipped/out-of-range samples must not feed fusion. */
lsm303_result_t lsm303dlhc_poll(lsm303_device_t *p_device,
                              lsm303_frame_t *p_frame);

#endif
