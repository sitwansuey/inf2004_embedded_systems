/* Same workload built against checkpoint and current production data code.
 * Host timing only; not a robot scheduling or sensor-latency measurement. */
#include "sensor_data.h"
#include <stdio.h>
#include <time.h>
#ifdef S4_CHECKPOINT_BENCHMARK
typedef s4_sensor_status_t cache_t;
#else
typedef s4_sensor_cache_t cache_t;
#endif
int main(void)
{
    cache_t cache;s4_sensor_status_t output;
    lsm303_frame_t frame={0};
    frame.accel.fresh=frame.accel.valid=frame.mag.fresh=frame.mag.valid=1;
    frame.accel.status=frame.mag.status=LSM303_DATA_OK;
    frame.accel.raw[2]=1000;frame.accel.milli_units[2]=1000;
    frame.mag.raw[2]=392;frame.mag.milli_units[2]=400;
    s4_data_reset(&cache);s4_data_init_result(&cache,LSM303_OK);
    volatile uint32_t checksum=0;
    const unsigned iterations=2000000;
    clock_t start=clock();
    for(unsigned n=0;n<iterations;++n) {
        uint64_t stamp=((uint64_t)n+1U)*20000U;
        frame.accel.read_time_us=frame.mag.read_time_us=stamp;
        s4_data_update(&cache,&frame,LSM303_OK);
        s4_data_snapshot(&cache,stamp+1000U,&output);
        checksum+=output.accel_samples+(uint32_t)output.latest.accel.milli_units[2];
    }
    double seconds=(double)(clock()-start)/CLOCKS_PER_SEC;
    printf("{\"cache_bytes\":%u,\"iterations\":%u,\"seconds\":%.6f,\"ns_per_update_snapshot\":%.1f,\"checksum\":%u}\n",
        (unsigned)sizeof(cache),iterations,seconds,seconds*1e9/iterations,checksum);
    return 0;
}
