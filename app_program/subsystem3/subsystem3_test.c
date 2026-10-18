/* On-device algorithm smoke checks, not sensor/hardware validation. */
#include "subsystem3.h"
#include "../test_report.h"

#if APP_TEST
EXPORT void subsystem3_test(void)
{
    uint16_t value = 0U;
    s3_config_t config;
    TEST_INIT();
    s3_config_defaults(&config);
    TEST_CHECK("S3-U02", s3_normalize(4095U, 0U, 4095U, 4095U, &value) &&
               (value == 1000U), "pure reflectance endpoint");
    TEST_CHECK("S3-U03", s3_config_valid(&config) &&
               (config.map_a == S3_NAV_NONE) && (config.map_z == S3_NAV_NONE),
               "no unverified navigation mapping");
    TEST_SUMMARY("subsystem3");
}
#endif
