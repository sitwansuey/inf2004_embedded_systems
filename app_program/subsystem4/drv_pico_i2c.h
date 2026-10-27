/* SPDX-License-Identifier: Apache-2.0 */
#ifndef S4_DRV_PICO_I2C_H
#define S4_DRV_PICO_I2C_H

#include "drv_lsm303dlhc.h"

#define S4_SDA_GPIO       2u
#define S4_SCL_GPIO       3u
#define S4_I2C_HZ         100000u
#define S4_IO_TIMEOUT_US  5000u

/* Call only from s4_sensor_task: owns I2C1, GP2/GP3. SDK types remain
 * private to this translation unit, separate from T-Kernel's size_t. */
void s4_pico_i2c_init(lsm303_bus_t *p_bus);

#endif
