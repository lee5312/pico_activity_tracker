#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

// 3D vector
typedef struct {
    float x;
    float y;
    float z;
} imu_vec3f_t;

//Initialize the IMU module.
//Returns true on success, false if the sensor is not responding.
bool imu_init(void);

//Update IMU state. Call this at a fixed rate.
void imu_update(uint32_t now_ms);

//Get the last raw accelerometer reading (LSB).
//ax, ay, az: pointers that will receive the raw 16-bit values.
void imu_get_accel_raw(int16_t *ax, int16_t *ay, int16_t *az);

//Get the last filtered accelerometer reading (may or may not be used).
void imu_get_accel_filtered(float *ax, float *ay, float *az);

//Get the total number of steps since boot.
uint32_t imu_get_total_steps(void);

//Get the number of steps in the last 60 minutes.
uint16_t imu_get_steps_last_hour(void);

//Check if the step goal for the last hour is reached.
//Returns true if the goal is reached, false otherwise.
bool imu_step_goal_reached(void);

//Get a simple activity level..
uint8_t imu_get_activity_level(void);

#endif 
