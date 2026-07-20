/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr/kernel.h>

#include "drivers/imu/mpu6650.h"

int main(void)
{
	imu_raw_t imu;

	if (imu_init() != 0) {
		printk("IMU init failed\n");
		return 0;
	}

	while (1) {
		if (imu_read(&imu) == 0) {
			printf("ax=%f ay=%f az=%f gx=%f gy=%f gz=%f dt=%f\n",
			       imu.ax, imu.ay, imu.az,
			       imu.gx, imu.gy, imu.gz,
			       imu.dt);
		} else {
			printf("IMU read failed\n");
		}

		k_sleep(K_MSEC(1000));
	}

	return 0;
}
