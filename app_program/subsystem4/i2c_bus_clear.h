/* SPDX-License-Identifier: Apache-2.0 */
#ifndef S4_I2C_BUS_CLEAR_H
#define S4_I2C_BUS_CLEAR_H

#include <stdbool.h>

/* Open-drain line access for an I2C bus clear. drive(true) pulls the line
 * low, drive(false) releases it to its pull-up; read returns its level. */
typedef struct
{
    void * p_context;
    bool (*read_sda)(void * p_context);
    void (*drive_scl)(void * p_context, bool low);
    void (*drive_sda)(void * p_context, bool low);
    void (*half_period)(void * p_context);
} s4_i2c_lines_t;

#define S4_I2C_CLEAR_CLOCKS 9U

/* A slave cut off mid-byte (Pico reset or a timeout while the GY-511 stays
 * powered) holds SDA low until it is clocked out. If SDA is low, clock SCL
 * up to nine times; SDA follows SCL half a period later, so every clock
 * whose SDA is free ends in a STOP. Returns true when SDA is released. */
static inline bool
s4_i2c_bus_clear (const s4_i2c_lines_t * p_lines)
{
    void *   p_context = p_lines->p_context;
    unsigned edge;
    bool     high = true;

    p_lines->drive_scl(p_context, false);
    p_lines->drive_sda(p_context, false);
    p_lines->half_period(p_context);
    if (p_lines->read_sda(p_context))
    {
        return true;
    }
    for (edge = 0; edge < 2U * S4_I2C_CLEAR_CLOCKS; edge++)
    {
        if (high && p_lines->read_sda(p_context))
        {
            break; /* SDA rose while SCL was high: a STOP; bus free */
        }
        high = !high;
        p_lines->drive_scl(p_context, !high);
        p_lines->half_period(p_context);
        p_lines->drive_sda(p_context, !high);
        p_lines->half_period(p_context);
    }
    p_lines->drive_scl(p_context, false);
    p_lines->drive_sda(p_context, false);
    p_lines->half_period(p_context);
    return p_lines->read_sda(p_context);
}

#endif

/*** end of file ***/
