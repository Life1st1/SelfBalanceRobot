#ifndef TYPES_H
#define TYPES_H

#include <time.h>

/**
 * file: types.h
 * Common types for the SelfBalanceRobot application.
 */


/**
 * Command structure for the robot's motors.
 */
typedef struct
{
    float left_velocity;
    float right_velocity;
    int enable;
} motor_command_t;


/**
 * Robot state structure to hold the current pitch angle and rate.
 */
typedef struct
{
    float pitch_angle;
    float pitch_rate;
} robot_state_t;

/**
 * Motor state structure to hold the enable status and fault status of the motors.
 */
typedef struct
{
    int enable;
    int fault;
} motor_state_t;

#endif // TYPES_H