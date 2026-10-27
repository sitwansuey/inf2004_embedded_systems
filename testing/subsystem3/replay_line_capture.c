/* Passive line-input replay. Uses production normalization and line logic;
 * neither barcode decisions nor motor commands are evaluated here. */
#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../../app_program/subsystem3/buddy3_robot_ir.h"

static int parse_row(char *text, uint32_t values[6])
{
    char *cursor = text;
    for (unsigned i = 0U; i < 6U; ++i) {
        char *end;
        unsigned long long value;
        if (*cursor < '0' || *cursor > '9') return 0;
        errno = 0;
        value = strtoull(cursor, &end, 10);
        if (errno != 0 || value > UINT32_MAX) return 0;
        values[i] = (uint32_t)value;
        if (i < 5U) {
            if (*end != ',') return 0;
            cursor = end + 1;
        } else {
            while (*end == '\r' || *end == '\n') ++end;
            if (*end != '\0') return 0;
        }
    }
    return 1;
}

int main(int argc, char **argv)
{
    FILE *input;
    s3_config_t cfg;
    s3_line_t line;
    uint32_t previous_time = 0U, previous_drop = 0U;
    unsigned rows = 0U, mismatches = 0U, gaps = 0U, invalid = 0U;
    unsigned masks[4] = {0U}, states[5] = {0U};
    uint16_t raw_min[2] = {4095U,4095U}, raw_max[2] = {0U};
    uint16_t dark_min[2] = {1000U,1000U}, dark_max[2] = {0U};
    char buffer[256];
    unsigned expected;
    if (argc != 3 || strlen(argv[2]) != 1U || argv[2][0] < '0' ||
        argv[2][0] > '4') {
        fputs("usage: LINE_REPLAY capture.csv expected_mask(0..3,4=observe)\n",stderr);
        return 3;
    }
    expected = (unsigned)(argv[2][0] - '0');
    s3_config_defaults(&cfg);
    robot_ir_apply_layout(&cfg);
    if (!robot_ir_apply_adc_calibration(&cfg) || cfg.line.line_mask != 3U ||
        cfg.line.position[0] != 1000 || cfg.line.position[1] != -1000) return 3;
    s3_line_reset(&line);
    input = fopen(argv[1],"r");
    if (input == NULL) return 3;
    while (fgets(buffer,sizeof(buffer),input) != NULL) {
        uint32_t v[6];
        uint16_t raw[3];
        s3_line_result_t result;
        int matches = 1;
        if (!parse_row(buffer,v) || v[1] > 4095U || v[2] > 4095U ||
            v[3] > 4095U || v[4] > 7U) { fclose(input);return 3; }
        if (rows != 0U && (v[0]-previous_time == 0U ||
            v[0]-previous_time > 30U || v[5] != previous_drop)) {
            ++gaps;s3_line_reset(&line);
        }
        previous_time=v[0];previous_drop=v[5];++rows;
        for (unsigned i=0U;i<3U;++i) raw[i]=(uint16_t)v[i+1U];
        s3_line_update(&line,&cfg.line,raw,(uint8_t)v[4],v[0],&result);
        if (result.state == S3_LINE_INVALID) { ++invalid;continue; }
        ++masks[result.black_mask & 3U];++states[result.state];
        for (unsigned i=0U;i<2U;++i) {
            if (raw[i]<raw_min[i]) raw_min[i]=raw[i];
            if (raw[i]>raw_max[i]) raw_max[i]=raw[i];
            if (result.darkness[i]<dark_min[i]) dark_min[i]=result.darkness[i];
            if (result.darkness[i]>dark_max[i]) dark_max[i]=result.darkness[i];
            if (expected < 4U && (((expected & (1U<<i)) != 0U) ?
                result.darkness[i] < cfg.line.threshold_high :
                result.darkness[i] > cfg.line.threshold_low)) matches=0;
        }
        if (expected < 4U && result.black_mask != expected) matches=0;
        if (expected==1U && (result.state!=S3_LINE_TRACKING || result.error!=1000)) matches=0;
        if (expected==2U && (result.state!=S3_LINE_TRACKING || result.error!=-1000)) matches=0;
        if (expected==3U && result.state!=S3_LINE_JUNCTION) matches=0;
        if (!matches) ++mismatches;
    }
    if (ferror(input)) { fclose(input);return 3; }
    fclose(input);
    printf("{\"rows\":%u,\"expected_mask\":%u,\"mismatches\":%u,"
           "\"discontinuities\":%u,\"invalid\":%u,"
           "\"raw_min\":[%u,%u],\"raw_max\":[%u,%u],"
           "\"darkness_min\":[%u,%u],\"darkness_max\":[%u,%u],"
           "\"mask_counts\":[%u,%u,%u,%u],"
           "\"state_counts\":[%u,%u,%u,%u,%u]}\n",
           rows,expected,mismatches,gaps,invalid,raw_min[0],raw_min[1],raw_max[0],raw_max[1],
           dark_min[0],dark_min[1],dark_max[0],dark_max[1],
           masks[0],masks[1],masks[2],masks[3],states[0],states[1],states[2],states[3],states[4]);
    return rows < 100U || gaps != 0U || invalid != 0U ? 3 : mismatches != 0U ? 2 : 0;
}
