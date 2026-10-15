#include "buddy3_line.h"

#include <stdio.h>
#include <string.h>

static unsigned int checks;
static unsigned int failures;
#define CHECK(condition) do { ++checks; if (!(condition)) { \
    ++failures; (void)printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
} } while (0)

int main(int argc, char **argv)
{
    uint16_t darkness = 999U;

    CHECK(s3_normalize(100U, 100U, 900U, 1023U, &darkness));
    CHECK(darkness == 0U);
    CHECK(s3_normalize(900U, 100U, 900U, 1023U, &darkness));
    CHECK(darkness == 1000U);
    CHECK(s3_normalize(100U, 900U, 100U, 1023U, &darkness));
    CHECK(darkness == 1000U);
    CHECK(!s3_normalize(500U, 500U, 500U, 1023U, &darkness));
    CHECK(darkness == 0U);
    if ((argc == 2) && (strcmp(argv[1], "--force-failure") == 0))
    {
        CHECK(2 + 2 == 5);
    }
    (void)printf("Buddy 3: %u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
