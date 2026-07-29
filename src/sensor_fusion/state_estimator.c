#include "state_estimator.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Madgwick_Filter/madgwickFilter.h"
#include "Complementary_Filter/complementary_filter.h"

#define ESTIMATOR_BACKEND_ENTRY(type_, name_, state_size_, init_fn_, update_fn_, deinit_fn_) \
    { \
        type_, \
        name_, \
        state_size_, \
        init_fn_, \
        update_fn_, \
        deinit_fn_ \
    }

static void estimator_init_madgwick_state(void *state)
{
    (void)state;

    q_est.q1 = 1.0f;
    q_est.q2 = 0.0f;
    q_est.q3 = 0.0f;
    q_est.q4 = 0.0f;
}

static void estimator_update_madgwick_state(void *state, const imu_raw_t *imu, attitude_t *attitude)
{
    float roll;
    float pitch;
    float yaw;

    (void)state;

    if ((imu == NULL) || (attitude == NULL)) {
        return;
    }

    imu_filter_dt(imu->ax, imu->ay, imu->az, imu->gx, imu->gy, imu->gz, imu->dt);
    eulerAngles(q_est, &roll, &pitch, &yaw);

    attitude->roll = roll;
    attitude->pitch = pitch;
    attitude->yaw = yaw;
}

static void estimator_deinit_madgwick_state(void *state)
{
    (void)state;
}

static void estimator_init_complementary_state(void *state)
{
    if (state != NULL) {
        complementary_filter_init((complementary_filter_state_t *)state);
    }
}

static void estimator_update_complementary_state(void *state, const imu_raw_t *imu, attitude_t *attitude)
{
    if ((state != NULL) && (imu != NULL) && (attitude != NULL)) {
        complementary_filter_update((complementary_filter_state_t *)state, imu, attitude);
    }
}

static void estimator_deinit_complementary_state(void *state)
{
    (void)state;
}

static const estimator_backend_t backends[] = {
    ESTIMATOR_BACKEND_ENTRY(
        FILTER_TYPE_MADGWICK,
        "madgwick",
        0,
        estimator_init_madgwick_state,
        estimator_update_madgwick_state,
        estimator_deinit_madgwick_state),
    ESTIMATOR_BACKEND_ENTRY(
        FILTER_TYPE_COMPLEMENTARY,
        "complementary",
        sizeof(complementary_filter_state_t),
        estimator_init_complementary_state,
        estimator_update_complementary_state,
        estimator_deinit_complementary_state),
};

const estimator_backend_t *estimator_get_backend(estimator_filter_type_t type)
{
    size_t i;

    for (i = 0; i < (sizeof(backends) / sizeof(backends[0])); ++i) {
        if (backends[i].type == type) {
            return &backends[i];
        }
    }

    return NULL;
}

static const estimator_backend_t *estimator_get_backend_by_name(const char *name)
{
    size_t i;

    if (name == NULL) {
        return NULL;
    }

    for (i = 0; i < (sizeof(backends) / sizeof(backends[0])); ++i) {
        if (strcmp(backends[i].name, name) == 0) {
            return &backends[i];
        }
    }

    return NULL;
}

void estimator_init(estimator_handle_t *handle, estimator_filter_type_t type)
{
    const estimator_backend_t *backend;

    if (handle == NULL) {
        return;
    }

    backend = estimator_get_backend(type);
    handle->type = type;
    handle->state = NULL;
    handle->backend = backend;

    if (backend == NULL) {
        return;
    }

    if (backend->state_size > 0) {
        handle->state = calloc(1, backend->state_size);
    }

    if (backend->init != NULL) {
        backend->init(handle->state);
    }
}

int estimator_init_by_name(estimator_handle_t *handle, const char *name)
{
    const estimator_backend_t *backend = estimator_get_backend_by_name(name);

    if (backend == NULL) {
        return -1;
    }

    estimator_init(handle, backend->type);
    return 0;
}

void estimator_update(estimator_handle_t *handle, const imu_raw_t *imu, attitude_t *attitude)
{
    if ((handle == NULL) || (handle->backend == NULL) || (imu == NULL) || (attitude == NULL)) {
        return;
    }

    if (handle->backend->update != NULL) {
        handle->backend->update(handle->state, imu, attitude);
    }
}

void estimator_deinit(estimator_handle_t *handle)
{
    if (handle == NULL) {
        return;
    }

    if (handle->backend != NULL && handle->backend->deinit != NULL) {
        handle->backend->deinit(handle->state);
    }

    if (handle->state != NULL) {
        free(handle->state);
        handle->state = NULL;
    }

    handle->backend = NULL;
}
