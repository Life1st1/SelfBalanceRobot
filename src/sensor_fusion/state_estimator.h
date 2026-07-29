#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <stddef.h>

#include "../drivers/imu/mpu6650.h"

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} attitude_t;

typedef enum
{
    FILTER_TYPE_MADGWICK = 0,
    FILTER_TYPE_COMPLEMENTARY = 1,
} estimator_filter_type_t;

typedef void (*estimator_init_fn)(void *state);
typedef void (*estimator_update_fn)(void *state, const imu_raw_t *imu, attitude_t *attitude);
typedef void (*estimator_deinit_fn)(void *state);

typedef struct
{
    estimator_filter_type_t type;
    const char *name;
    size_t state_size;
    estimator_init_fn init;
    estimator_update_fn update;
    estimator_deinit_fn deinit;
} estimator_backend_t;

typedef struct
{
    estimator_filter_type_t type;
    void *state;
    const estimator_backend_t *backend;
} estimator_handle_t;

void estimator_init(estimator_handle_t *handle, estimator_filter_type_t type);
int estimator_init_by_name(estimator_handle_t *handle, const char *name);
void estimator_update(estimator_handle_t *handle, const imu_raw_t *imu, attitude_t *attitude);
void estimator_deinit(estimator_handle_t *handle);
const estimator_backend_t *estimator_get_backend(estimator_filter_type_t type);

#endif // STATE_ESTIMATOR_H