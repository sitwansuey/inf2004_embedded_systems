/* SPDX-License-Identifier: Apache-2.0 */
#include "drv_pico_i2c.h"

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/resets.h"
#include "hardware/ticks.h"
#include "hardware/timer.h"

static uint64_t bus_hardware_time(void *p_context)
{
    (void)p_context;
    return time_us_64();
}

static bool bus_read(void *p_context, uint8_t address, uint8_t subaddress,
                     uint8_t *p_data, uint32_t length)
{
    absolute_time_t deadline = make_timeout_time_us(S4_IO_TIMEOUT_US);
    int count;

    (void)p_context;
    count = i2c_write_blocking_until(i2c1, address, &subaddress, 1,
                                     true, deadline);
    if (count == 1)
    {
        count = i2c_read_blocking_until(i2c1, address, p_data, length,
                                        false, deadline);
        if (count == (int)length)
        {
            return true;
        }
    }
    /* Sole owner resets controller FIFO/restart state after an I/O error. */
    (void)i2c_init(i2c1, S4_I2C_HZ);
    return false;
}

static bool bus_write(void *p_context, uint8_t address, uint8_t reg,
                      uint8_t value)
{
    uint8_t data[2] = { reg, value };
    absolute_time_t deadline = make_timeout_time_us(S4_IO_TIMEOUT_US);

    (void)p_context;
    if (i2c_write_blocking_until(i2c1, address, data, sizeof(data),
                                 false, deadline) == (int)sizeof(data))
    {
        return true;
    }
    (void)i2c_init(i2c1, S4_I2C_HZ);
    return false;
}

void s4_pico_i2c_init(lsm303_bus_t *p_bus)
{
    /* 1-us hardware timer tick from the port's 12 MHz clk_ref. This does
     * not enable watchdog resets, reset the timer, or allocate an IRQ. */
    tick_start(TICK_WATCHDOG, 12u);
    unreset_block_num_wait_blocking(RESET_TIMER);
    gpio_set_function(S4_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(S4_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(S4_SDA_GPIO);
    gpio_pull_up(S4_SCL_GPIO);
    (void)i2c_init(i2c1, S4_I2C_HZ);
    *p_bus = (lsm303_bus_t)
    {
        .p_context = NULL,
        .read = bus_read,
        .write = bus_write,
        .now_us = bus_hardware_time
    };
}
