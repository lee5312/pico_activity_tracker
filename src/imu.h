#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>

bool imu_init(void);
void imu_update(uint32_t now_ms);

void imu_get_accel_raw(int16_t *ax, int16_t *ay, int16_t *az);
void imu_get_accel_filtered(float *ax, float *ay, float *az);

uint32_t imu_get_total_steps(void);
uint16_t imu_get_steps_last_hour(void);
bool     imu_step_goal_reached(void);
uint8_t  imu_get_activity_level(void);

#endif