#include <stdio.h>

#include <zephyr/kernel.h>

#include "../drivers/imu/mpu6650.h"
#include "../sensor_fusion/state_estimator.h"

#ifndef ESTIMATOR_FILTER_NAME
#define ESTIMATOR_FILTER_NAME "complementary"

#endif

int controler_update(void) {
	imu_raw_t imu;
	attitude_t attitude;
	estimator_handle_t estimator;

	if (imu_init() != 0) {
		printk("IMU init failed\n");
		return 0;
	}

	if (estimator_init_by_name(&estimator, ESTIMATOR_FILTER_NAME) != 0) {
		printk("Unknown estimator filter: %s\n", ESTIMATOR_FILTER_NAME);
		return 0;
	}

	while (1) {
		if (imu_read(&imu) == 0) {
			estimator_update(&estimator, &imu, &attitude);
			printf("roll=%f pitch=%f yaw=%f\n",
			       attitude.roll, attitude.pitch, attitude.yaw);
		} else {
			printf("IMU read failed\n");
		}

		k_sleep(K_MSEC(10));
	}

	estimator_deinit(&estimator);
	return 0;
}