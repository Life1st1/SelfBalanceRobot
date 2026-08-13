#ifndef DEFINES_H
#define DEFINES_H

#include "../drivers/imu/mpu6650.h"
#include "../sensor_fusion/state_estimator.h"

#define MOTOR_COUNT 2
#define IMU_COUNT 1

typedef struct pid_data {
    float error;
    float kp;
    float ki;
    float kd;
    float output;
} pid_data_t;

typedef struct motor_data {
    uint8_t id;
    float speed;
    float direction;
} motor_data_t;

typedef struct RobotData {
    float pitch_setpoint;
    imu_raw_t imu_data;
    pid_data_t pid_gains;
    motor_data_t motors[MOTOR_COUNT];
    attitude_t attitude;

} RobotData_t;

#endif // DEFINES_H