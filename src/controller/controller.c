#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <math.h>

#include "../drivers/imu/mpu6650.h"
#include "../sensor_fusion/state_estimator.h"
#include "../common/defines.h"

#include "controller.h"

#ifndef ESTIMATOR_FILTER_NAME
#define ESTIMATOR_FILTER_NAME "complementary"
#endif

#define PWM_CHANNEL       1U
#define MOTOR_MIN_PERIOD  00U
#define MOTOR_MAX_PERIOD  300U
#define CONTROL_PERIOD_MS 1U
#define MAX_SPEED         100.0f

#define PID_KP 50.0f
#define PID_KI 1.0f
#define PID_KD 0.0f

struct k_mutex pid_mutex;

static const struct pwm_dt_spec motor1 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor1));
static const struct pwm_dt_spec motor2 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor2));

static const struct gpio_dt_spec motor_dir =
    GPIO_DT_SPEC_GET(DT_ALIAS(motor_dir), gpios);
static const struct gpio_dt_spec motor_dir2 =
    GPIO_DT_SPEC_GET(DT_ALIAS(motor_dir2), gpios);

void set_motor_speed(uint8_t id, uint32_t hz);

static float pid_integral;
static float pid_prev_error;

RobotData_t robot_data = {
    .pid_gains = {0.0f, PID_KP, PID_KI, PID_KD, 0.0f},
};

void update_pid_gains(float kp, float ki, float kd) {
    k_mutex_lock(&pid_mutex, K_FOREVER);
    robot_data.pid_gains.kp = kp;
    robot_data.pid_gains.ki = ki;
    robot_data.pid_gains.kd = kd;
    k_mutex_unlock(&pid_mutex);

    printk("Updated PID gains: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", robot_data.pid_gains.kp, robot_data.pid_gains.ki, robot_data.pid_gains.kd);
}

static float pid_compute(RobotData_t *robot_data)
{
    float error = robot_data->pitch_setpoint - robot_data->attitude.pitch;
    float dt = robot_data->imu_data.dt;

    pid_integral += error * dt;
    float derivative = (error - pid_prev_error) / dt;
    pid_prev_error = error;
    k_mutex_lock(&pid_mutex, K_FOREVER);
    float output = robot_data->pid_gains.kp * error + robot_data->pid_gains.ki * pid_integral + robot_data->pid_gains.kd * derivative;
    k_mutex_unlock(&pid_mutex);
    return output;
}

static void set_motor_direction(uint8_t id, bool forward)
{
    if (id == 1) {
        gpio_pin_set_dt(&motor_dir, forward ? 1 : 0);
    } else if (id == 2) {
        gpio_pin_set_dt(&motor_dir2, forward ? 1 : 0);
    }
}

void set_motor_speed(uint8_t id, uint32_t hz) {
    int ret;
	uint32_t period = 0;
	if (hz == 0) {
		if (id == 1) {
			pwm_set_dt(&motor1, 0, 0);
		} else if (id == 2) {
			pwm_set_dt(&motor2, 0, 0);
		}
		return;
	}
    hz = hz + 1;    // because hz = 1 the prescaler of timer is out of range
    if (hz > MOTOR_MAX_PERIOD) {
        hz = MOTOR_MAX_PERIOD;
    }
    if (hz < MOTOR_MIN_PERIOD) {
        hz = MOTOR_MIN_PERIOD;
    }
	period = 1000000000U / hz;
	if (id == 1) {
        //printk(" Motor1 speed: %d Hz, ", hz);
		ret = pwm_set_dt(&motor1, period, period / 2U);
        if (ret != 0) {
            printk("Failed to set PWM for motor1 hz=%d, period=%d, ret=%d\n", hz, period, ret);
        }
	} else if (id == 2) {
        //printk(" Motor2 speed: %d Hz\n", hz);
		ret = pwm_set_dt(&motor2, period, period / 2U);
		if (ret != 0) {
            printk("Failed to set PWM for motor2 hz=%d, period=%d, ret=%d\n", hz, period, ret);
        }
	}
}

void motor_disable(void) {
    set_motor_speed(1, 0);
    set_motor_speed(2, 0);
}

static float calibrate_mpu(imu_raw_t *imu_calibrated, attitude_t *attitude_calibrated) {
    imu_raw_t imu;
    attitude_t attitude;
    estimator_handle_t estimator;

    const int num_samples = 200;
    float pitch_samples[num_samples];
    const float stddev_threshold = 0.5f; /* degrees */
    float pitch_setpoint = 0.0f;

    float gx_sum = 0.0f;
    float gy_sum = 0.0f;
    float gz_sum = 0.0f;
    float roll_sum = 0.0f;
    float pitch_sum = 0.0f;
    float yaw_sum = 0.0f;

    if (estimator_init_by_name(&estimator, ESTIMATOR_FILTER_NAME) != 0) {
        printk("Estimator init failed during calibration\n");
        return 0.0f;
    }

    /* Give sensor a moment to settle */
    k_msleep(100);

    bool stable = false;
    while (!stable) {
        float sum = 0.0f;
        for (int i = 0; i < num_samples; i++) {
            if (imu_read(&imu) == 0) {
                estimator_update(&estimator, &imu, &attitude);
                pitch_samples[i] = attitude.pitch;
                sum += pitch_samples[i];
                gx_sum += imu.gx;
                gy_sum += imu.gy;
                gz_sum += imu.gz;
                roll_sum += attitude.roll;
                pitch_sum += attitude.pitch;
                yaw_sum += attitude.yaw;
            } else {
                printk("IMU read failed during calibration\n");
                pitch_samples[i] = 0.0f;
            }
            k_msleep(25);
        }

        float mean = sum / (float)num_samples;
        float var = 0.0f;
        for (int i = 0; i < num_samples; i++) {
            float d = pitch_samples[i] - mean;
            var += d * d;
        }
        float stddev = sqrtf(var / (float)num_samples);

        printk("Calibration: mean=%.2f deg, stddev=%.2f deg\n", mean, stddev);

        if (stddev < stddev_threshold) {
            stable = true;
            pitch_setpoint = mean;
        } else {
            printk("Repeat calibration, DONT MOVE!\n");
            k_msleep(200);
        }
    }

    if (imu_calibrated != NULL) {
        imu_calibrated->gx = gx_sum / (float)num_samples;
        imu_calibrated->gy = gy_sum / (float)num_samples;
        imu_calibrated->gz = gz_sum / (float)num_samples;
    }

    if (attitude_calibrated != NULL) {
        attitude_calibrated->roll = roll_sum / (float)num_samples;
        attitude_calibrated->pitch = pitch_sum / (float)num_samples;
        attitude_calibrated->yaw = yaw_sum / (float)num_samples;
    }

    estimator_deinit(&estimator);
    printk("Calibrated MPU: roll=%.2f, pitch=%.2f, yaw=%.2f, gx=%.3f, gy=%.3f, gz=%.3f\n",
           attitude_calibrated ? attitude_calibrated->roll : 0.0f,
           attitude_calibrated ? attitude_calibrated->pitch : 0.0f,
           attitude_calibrated ? attitude_calibrated->yaw : 0.0f,
           imu_calibrated ? imu_calibrated->gx : 0.0f,
           imu_calibrated ? imu_calibrated->gy : 0.0f,
           imu_calibrated ? imu_calibrated->gz : 0.0f);

    return -pitch_setpoint;
}

int controller_init(void) {
    robot_data.pitch_setpoint = 0.0f; // Desired pitch angle (upright position)

    //k_msleep(4000);
    printk("Starting Self Balance Robot\n");

    if (!pwm_is_ready_dt(&motor1)) {
        printk("PWM device not ready\n");
        return -1;
    }
    if (!pwm_is_ready_dt(&motor2)) {
        printk("PWM device not ready\n");
        return -1;
    }
    if (motor_dir.port != NULL && !gpio_is_ready_dt(&motor_dir)) {
        printk("Motor direction GPIO not ready\n");
        return -1;
    }
    if (motor_dir2.port != NULL && !gpio_is_ready_dt(&motor_dir2)) {
        printk("Motor direction 2 GPIO not ready\n");
        return -1;
    }
    if (motor_dir.port != NULL) {
        if (gpio_pin_configure_dt(&motor_dir, GPIO_OUTPUT_INACTIVE) != 0) {
            printk("Motor direction GPIO configure failed\n");
            return -1;
        }
    }
    if (motor_dir2.port != NULL) {
        if (gpio_pin_configure_dt(&motor_dir2, GPIO_OUTPUT_INACTIVE) != 0) {
            printk("Motor direction 2 GPIO configure failed\n");
            return -1;
        }
    }

    if (imu_init() != 0) {
        printk("IMU init failed\n");
        return -1;
    }
}

void print_robot_data(void) {
    RobotData_t robot_data_local;

    while (1) {
        k_msleep(10);
        k_msgq_get(&log_msgq, &robot_data_local, K_FOREVER);

        printk("{\"time_ms\":%lld,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
               "\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f,"
               "\"roll\":%.3f,\"pitch\":%.3f,\"yaw\":%.3f}\n",
               (long long)k_uptime_get(),
               robot_data_local.imu_data.ax,
               robot_data_local.imu_data.ay,
               robot_data_local.imu_data.az,
               robot_data_local.imu_data.gx,
               robot_data_local.imu_data.gy,
               robot_data_local.imu_data.gz,
               robot_data_local.attitude.roll,
               robot_data_local.attitude.pitch,
               robot_data_local.attitude.yaw);
    }
}

#define DELTA_T 0.01f //control loop running at 500 Hz

imu_raw_t imu_calib = {
    .gx = 0.0f,
    .gy = 0.0f,
    .gz = 0.0f
};

int controller_update(void) {
    attitude_t attitude_calib;
    estimator_handle_t estimator;

    if (controller_init() != 0) {
        printk("Controller init failed\n");
        return -1;
    }
    if (estimator_init_by_name(&estimator, "madgwick") != 0) {
        printk("Madgwick estimator init failed\n");
        return -1;
    }
    printk("Madgwick estimator initialized\n");

    robot_data.pitch_setpoint = calibrate_mpu(&imu_calib, &attitude_calib);
    printk("Pitch setpoint: %f\n", robot_data.pitch_setpoint);
    imu_calib.gx = imu_calib.gx / 2.0f;
    imu_calib.gy = imu_calib.gy / 2.0f;
    imu_calib.gz = imu_calib.gz / 2.0f;

    printk("Calibrated IMU: gx=%.3f, gy=%.3f, gz=%.3f\n",
           imu_calib.gx, imu_calib.gy, imu_calib.gz);
    

    while (1) {
        if (k_sem_take(&alarm_sem, K_FOREVER) != 0) {
            printk("Failed to take alarm semaphore\n");
            continue;
        }
        if (imu_read(&robot_data.imu_data) == 0) {
            robot_data.imu_data.dt = DELTA_T; // Assuming a fixed delta time for simplicity
            estimator_update(&estimator, &robot_data.imu_data, &robot_data.attitude);

            // if (robot_data.attitude.pitch > 45.0f || robot_data.attitude.pitch < -45.0f) {
            //     printk("Pitch angle out of range: %.2f\n", robot_data.attitude.pitch);
            //     motor_disable();
            //     k_msleep(1000);
            //     continue;
            // }

            robot_data.pid_gains.output = pid_compute(&robot_data);
            robot_data.motors[0].speed = (uint32_t)(robot_data.pid_gains.output < 0.0f ? -robot_data.pid_gains.output : robot_data.pid_gains.output);
            robot_data.motors[1].speed = (uint32_t)(robot_data.pid_gains.output < 0.0f ? -robot_data.pid_gains.output : robot_data.pid_gains.output);
            robot_data.motors[0].direction = (robot_data.pid_gains.output >= 0.0f);
            robot_data.motors[1].direction = (robot_data.pid_gains.output >= 0.0f);
            //printk("Pitch: %.2f, Direction: %s, output: %.2f, ", attitude.pitch, direction ? "forward" : "backward", pid_output);
            set_motor_direction(1, robot_data.motors[0].direction);
            set_motor_direction(2, robot_data.motors[1].direction);
            set_motor_speed(1, robot_data.motors[0].speed);
            set_motor_speed(2, robot_data.motors[1].speed);
            //print_robot_data();
            if(k_msgq_put(&log_msgq, &robot_data, K_NO_WAIT) != 0) {
                k_msgq_purge(&log_msgq); // Clear the queue if it's full
            }

        } else {
            printk("IMU read failed\n");
        }
        //k_msleep(CONTROL_PERIOD_MS);
    }

    estimator_deinit(&estimator);
    return 0;
}