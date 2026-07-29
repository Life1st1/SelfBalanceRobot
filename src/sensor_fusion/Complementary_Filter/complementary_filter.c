#include "complementary_filter.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float clamp_angle(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

void complementary_filter_init(complementary_filter_state_t *state)
{
    if (state == NULL) {
        return;
    }

    state->roll = 0.0f;
    state->pitch = 0.0f;
    state->yaw = 0.0f;
    state->alpha = COMPLEMENTARY_FILTER_ALPHA;
}

void complementary_filter_update(complementary_filter_state_t *state,
                                 const imu_raw_t *imu,
                                 attitude_t *attitude)
{
    float accel_roll;
    float accel_pitch;
    float accel_yaw;
    float gyro_roll;
    float gyro_pitch;
    float gyro_yaw;
    float dt;

    if ((state == NULL) || (imu == NULL) || (attitude == NULL)) {
        return;
    }

    dt = (imu->dt > 0.0f) ? imu->dt : 0.01f;

    accel_roll = atan2f(imu->ay, imu->az) * 180.0f / (float)M_PI;
    accel_pitch = atan2f(-imu->ax, sqrtf(imu->ay * imu->ay + imu->az * imu->az)) * 180.0f / (float)M_PI;
    accel_yaw = 0.0f;

    gyro_roll = state->roll + imu->gx * dt;
    gyro_pitch = state->pitch + imu->gy * dt;
    gyro_yaw = state->yaw + imu->gz * dt;

    state->roll = state->alpha * gyro_roll + (1.0f - state->alpha) * accel_roll;
    state->pitch = state->alpha * gyro_pitch + (1.0f - state->alpha) * accel_pitch;
    state->yaw = state->alpha * gyro_yaw + (1.0f - state->alpha) * accel_yaw;

    state->roll = clamp_angle(state->roll);
    state->pitch = clamp_angle(state->pitch);
    state->yaw = clamp_angle(state->yaw);

    attitude->roll = state->roll;
    attitude->pitch = state->pitch;
    attitude->yaw = state->yaw;
}
