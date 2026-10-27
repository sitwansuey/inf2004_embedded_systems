/* Test the production board configurator against the user's physical layout. */
#include <stdio.h>
#include "../../app_program/subsystem3/buddy3_robot_ir.h"

static unsigned int checks;
static unsigned int failures;
#define CHECK(c) do { ++checks; if (!(c)) { ++failures; printf("FAIL %s:%d: %s\n",__FILE__,__LINE__,#c); } } while (0)

int main(void)
{
    s3_config_t cfg;
    s3_line_t line;
    s3_line_result_t result;
    uint16_t raw[3] = {0U,0U,0U};
    s3_config_defaults(&cfg);
    robot_ir_apply_layout(&cfg);
    CHECK(cfg.line.line_mask == 3U && cfg.barcode_sensor == 2U);
    CHECK(cfg.line.position[0] == 1000 && cfg.line.position[1] == -1000);
    CHECK((cfg.line.line_mask & (1U << cfg.barcode_sensor)) == 0U);
    CHECK(cfg.map_a == S3_NAV_NONE && cfg.map_z == S3_NAV_NONE);
    CHECK(s3_config_valid(&cfg));

    s3_line_reset(&line); raw[0]=4095U;
    s3_line_update(&line,&cfg.line,raw,7U,10U,&result);
    CHECK(result.state == S3_LINE_TRACKING && result.error == 1000);
    CHECK(result.black_mask == 1U); /* GP26 is physically right. */

    s3_line_reset(&line); raw[0]=0U;raw[1]=4095U;
    s3_line_update(&line,&cfg.line,raw,7U,20U,&result);
    CHECK(result.state == S3_LINE_TRACKING && result.error == -1000);
    CHECK(result.black_mask == 2U); /* GP27 is physically left. */

    s3_line_reset(&line);raw[1]=0U;raw[2]=4095U;
    s3_line_update(&line,&cfg.line,raw,7U,30U,&result);
    CHECK(result.black_mask == 0U && result.state == S3_LINE_SEARCHING);
    CHECK(result.darkness[2] == 0U); /* Barcode contrast cannot steer the line. */

    s3_line_reset(&line);raw[1]=4095U;
    s3_line_update(&line,&cfg.line,raw,7U,40U,&result);
    CHECK(result.state == S3_LINE_TRACKING && result.error == -1000);
    CHECK(result.black_mask == 2U); /* A dark barcode does not create a junction. */

    s3_line_reset(&line);raw[0]=4095U;
    s3_line_update(&line,&cfg.line,raw,7U,50U,&result);
    CHECK(result.state == S3_LINE_JUNCTION && result.black_mask == 3U);
    printf("S3-U42 confirmed layout: %u checks, %u failures\n",checks,failures);
    return failures == 0U ? 0 : 1;
}
