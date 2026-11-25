/* SPDX-License-Identifier: Apache-2.0 */
#include "drv_pico_i2c.h"
#include "i2c_bus_clear.h"

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/resets.h"
#include "hardware/ticks.h"
#include "hardware/timer.h"

static uint64_t
bus_hardware_time (void * p_context)
{
    (void) p_context;
    return time_us_64();
}

static bool
bus_read (void * p_context, uint8_t address, uint8_t subaddress,
          uint8_t * p_data, uint32_t length)
{
    absolute_time_t deadline = make_timeout_time_us(S4_IO_TIMEOUT_US);
    int             count;

    (void) p_context;
    count =
        i2c_write_blocking_until(i2c1, address, &subaddress, 1, true, deadline);
    if (count == 1)
    {
        count = i2c_read_blocking_until(i2c1, address, p_data, length, false,
                                        deadline);
        if (count == (int) length)
        {
            return true;
        }
    }
    /* Sole owner resets controller FIFO/restart state after an I/O error. */
    (void) i2c_init(i2c1, S4_I2C_HZ);
    return false;
}

static bool
bus_write (void * p_context, uint8_t address, uint8_t reg, uint8_t value)
{
    uint8_t         data[2]  = {reg, value};
    absolute_time_t deadline = make_timeout_time_us(S4_IO_TIMEOUT_US);

    (void) p_context;
    if (i2c_write_blocking_until(i2c1, address, data, sizeof(data), false,
                                 deadline) == (int) sizeof(data))
    {
        return true;
    }
    (void) i2c_init(i2c1, S4_I2C_HZ);
    return false;
}

static bool
line_read_sda (void * p_context)
{
    (void) p_context;
    return gpio_get(S4_SDA_GPIO);
}

/* Open drain: the output value stays 0, so output pulls the line low and
 * input releases it to the pull-up. */
static void
line_drive_scl (void * p_context, bool low)
{
    (void) p_context;
    gpio_set_dir(S4_SCL_GPIO, low ? GPIO_OUT : GPIO_IN);
}

static void
line_drive_sda (void * p_context, bool low)
{
    (void) p_context;
    gpio_set_dir(S4_SDA_GPIO, low ? GPIO_OUT : GPIO_IN);
}

static void
line_half_period (void * p_context)
{
    uint64_t end = time_us_64() + 5u; /* half of a 100 kHz clock */

    (void) p_context;
    while (time_us_64() < end)
    {
    }
}

void
s4_pico_i2c_recover (void)
{
    static const s4_i2c_lines_t lines = {.p_context   = NULL,
                                         .read_sda    = line_read_sda,
                                         .drive_scl   = line_drive_scl,
                                         .drive_sda   = line_drive_sda,
                                         .half_period = line_half_period};

    /* Hand both pins from I2C1 to SIO, released (input) with output 0. */
    gpio_set_dir(S4_SDA_GPIO, GPIO_IN);
    gpio_set_dir(S4_SCL_GPIO, GPIO_IN);
    gpio_put(S4_SDA_GPIO, false);
    gpio_put(S4_SCL_GPIO, false);
    gpio_set_function(S4_SDA_GPIO, GPIO_FUNC_SIO);
    gpio_set_function(S4_SCL_GPIO, GPIO_FUNC_SIO);
    (void) s4_i2c_bus_clear(&lines);
    gpio_set_function(S4_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(S4_SCL_GPIO, GPIO_FUNC_I2C);
    (void) i2c_init(i2c1, S4_I2C_HZ);
}

void
s4_pico_i2c_init (lsm303_bus_t * p_bus)
{
    /* 1-us hardware timer tick from the port's 12 MHz clk_ref. This does
     * not enable watchdog resets, reset the timer, or allocate an IRQ. */
    tick_start(TICK_WATCHDOG, 12u);
    unreset_block_num_wait_blocking(RESET_TIMER);
    gpio_set_function(S4_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(S4_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(S4_SDA_GPIO);
    gpio_pull_up(S4_SCL_GPIO);
    (void) i2c_init(i2c1, S4_I2C_HZ);
    *p_bus = (lsm303_bus_t){.p_context = NULL,
                            .read      = bus_read,
                            .write     = bus_write,
                            .now_us    = bus_hardware_time};
}

/*** end of file ***/
