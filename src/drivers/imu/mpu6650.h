#ifndef IMU_H
#define IMU_H

/**
 * IMU data structure to hold accelerometer and gyroscope readings.
 */
typedef struct
{
    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;

    float dt; // Delta time in milliseconds
} imu_raw_t;

extern imu_raw_t imu_calib;

int imu_init(void);

int imu_read(imu_raw_t *imu);

#endif // IMU_H