#include <errno.h>
#include <stdio.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

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

int imu_read(imu_raw_t *imu)
{
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	uint32_t now;
	int rc;

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

	imu->gx = (float)sensor_value_to_double(&gyro[0]);
	imu->gy = (float)sensor_value_to_double(&gyro[1]);
	imu->gz = (float)sensor_value_to_double(&gyro[2]);

	now = k_uptime_get_32();
	if (last_sample_ms != 0U) {
		imu->dt = (float)(now - last_sample_ms) / 1000.0f;
	} else {
		imu->dt = 0.0f;
	}
	last_sample_ms = now;

	return 0;
}
