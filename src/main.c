#include <stdio.h>
#include "pico/stdlib.h"
#include "imu.h"

int main(void) {
    stdio_init_all();
    sleep_ms(1000);   // USB serial stabilization delay

    bool ok = imu_init();
    printf("imu_init() = %d\r\n", ok);

    if (!ok) {
        printf("IMU init failed, aborting.\r\n");
        while (1) {
            sleep_ms(1000);
        }
    }

    while (1) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        imu_update(now);

        int16_t ax, ay, az;
        imu_get_accel_raw(&ax, &ay, &az);
        printf("raw: %6d %6d %6d  steps=%lu\r\n",
               ax, ay, az, (unsigned long)imu_get_total_steps());


        //  - 100 ms → 10 Hz
        //  -  20 ms → 50 Hz 
        sleep_ms(20);  // 50 Hz
    }
}