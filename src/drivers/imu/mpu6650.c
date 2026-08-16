#include <errno.h>
#include <stdio.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/counter.h>
#include <stdbool.h>

#include "mpu6650.h"

static const struct device *imu_dev;
static uint32_t last_sample_ms;

int imu_init(void)
{
	imu_dev = DEVICE_DT_GET_ONE(invensense_mpu6050);

	if (!device_is_ready(imu_dev)) {
		printk("IMU device %s is not ready\n", imu_dev->name);
		return -ENODEV;
	}

	last_sample_ms = k_uptime_get_32();
	return 0;
}

#define SAMPLE_TIMER DT_INST(0, espressif_esp32_counter)

int imu_read(imu_raw_t *imu)
{
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	uint32_t now;
	int rc;
	const struct device *const counter_dev = DEVICE_DT_GET(SAMPLE_TIMER);
	uint32_t now_ticks;
	uint64_t now_usec;
	int now_sec;
	int err;

	if (imu == NULL) {
		return -EINVAL;
	}

	if (imu_dev == NULL || !device_is_ready(imu_dev)) {
		return -ENODEV;
	}

	rc = sensor_sample_fetch(imu_dev);
	if (rc != 0) {
		return rc;
	}

	rc = sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	if (rc != 0) {
		return rc;
	}

	rc = sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ, gyro);
	if (rc != 0) {
		return rc;
	}

	imu->ax = (float)sensor_value_to_double(&accel[0]);
	imu->ay = (float)sensor_value_to_double(&accel[1]);
	imu->az = (float)sensor_value_to_double(&accel[2]);

	imu->gx = (float)sensor_value_to_double(&gyro[0]) - imu_calib.gx;
	imu->gy = (float)sensor_value_to_double(&gyro[1]) - imu_calib.gy;
	imu->gz = (float)sensor_value_to_double(&gyro[2]) - imu_calib.gz;

	// Use the hardware counter to compute high-resolution dt between samples.
	// Keep previous tick value across calls so we can compute the delta.
	// static uint32_t last_ticks = 0;
	// static bool has_last = false;

	// err = counter_get_value(counter_dev, &now_ticks);
	// if (err) {
	// 	printk("Failed to read counter value (err %d)\n", err);
	// 	return -EIO;
	// }

	// if (!counter_is_counting_up(counter_dev)) {
	// 	now_ticks = counter_get_top_value(counter_dev) - now_ticks;
	// }

	// if (has_last) {
	// 	uint32_t delta_ticks;
	// 	uint32_t top = counter_get_top_value(counter_dev);

	// 	if (now_ticks >= last_ticks) {
	// 		delta_ticks = now_ticks - last_ticks;
	// 	} else {
	// 		delta_ticks = (top - last_ticks) + now_ticks + 1U;
	// 	}

	// 	now_usec = counter_ticks_to_us(counter_dev, delta_ticks);
	// 	imu->dt = (float)now_usec / (float)USEC_PER_SEC;
	// } else {
	// 	imu->dt = 0.0f;
	// 	has_last = true;
	// }

	// last_ticks = now_ticks;

	return 0;
}
