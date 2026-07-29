#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#include "../state_estimator.h"
#include "../../drivers/imu/mpu6650.h"

#ifndef COMPLEMENTARY_FILTER_ALPHA
#define COMPLEMENTARY_FILTER_ALPHA 0.90f
#endif

typedef struct
{
    float roll;
    float pitch;
    float yaw;
    float alpha;
} complementary_filter_state_t;

void complementary_filter_init(complementary_filter_state_t *state);
void complementary_filter_update(complementary_filter_state_t *state,
                                 const imu_raw_t *imu,
                                 attitude_t *attitude);

#endif /* COMPLEMENTARY_FILTER_H */
