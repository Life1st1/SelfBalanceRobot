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

#define PID_KP 110.0f
#define PID_KI 2.0f
#define PID_KD 0.0f

struct k_mutex pid_mutex;

static const struct pwm_dt_spec motor1 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor1));
static const struct pwm_dt_spec motor2 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor2));

static const struct gpio_dt_spec motor_dir =
    GPIO_DT_SPEC_GET(DT_ALIAS(motor_dir), gpios);
static const struct gpio_dt_spec motor_dir2 =
    GPIO_DT_SPEC_GET(DT_ALIAS(motor_dir2), gpios);

void set_motor_speed(uint8_t id, uint32_t hz);

#define PID_KP 110.0f
#define PID_KI 2.0f
#define PID_KD 0.0f
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

float calibrate_pitch_setpoint(void) {
    imu_raw_t imu;
    attitude_t attitude;
    estimator_handle_t estimator;

    const int num_samples = 100;
    float samples[num_samples];
    const float stddev_threshold = 0.5f; /* degrees */
    float pitch_setpoint = 0.0f;

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
                samples[i] = attitude.pitch;
                sum += samples[i];
            } else {
                printk("IMU read failed during calibration\n");
                samples[i] = 0.0f;
            }
            k_msleep(25);
        }

        float mean = sum / (float)num_samples;
        float var = 0.0f;
        for (int i = 0; i < num_samples; i++) {
            float d = samples[i] - mean;
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

    estimator_deinit(&estimator);
    printk("Calibrated pitch setpoint: %.2f\n", pitch_setpoint);
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
    while(1) {
        k_msleep(100);
        k_msgq_get(&log_msgq, &robot_data_local, K_FOREVER);
        printk("Robot Data:\n");
        printk("Pitch Setpoint: %.2f\n", robot_data_local.pitch_setpoint);
        printk("IMU Data: ax=%.2f, ay=%.2f, az=%.2f, gx=%.2f, gy=%.2f, gz=%.2f, dt=%.6f\n",
               robot_data_local.imu_data.ax, robot_data_local.imu_data.ay, robot_data_local.imu_data.az,
               robot_data_local.imu_data.gx, robot_data_local.imu_data.gy, robot_data_local.imu_data.gz,
               robot_data_local.imu_data.dt);
        printk("PID Gains: Kp=%.2f, Ki=%.2f, Kd=%.2f, Output=%.2f\n",
               robot_data_local.pid_gains.kp, robot_data_local.pid_gains.ki,
               robot_data_local.pid_gains.kd, robot_data_local.pid_gains.output);
        for (int i = 0; i < MOTOR_COUNT; i++) {
            printk("Motor %d: ID=%d, Speed=%.2f, Direction=%s\n", i + 1,
                   robot_data_local.motors[i].id, robot_data_local.motors[i].speed,
                   robot_data_local.motors[i].direction ? "Forward" : "Backward");
        }
        printk("Attitude: Roll=%.2f, Pitch=%.2f, Yaw=%.2f\n",
               robot_data_local.attitude.roll, robot_data_local.attitude.pitch,
               robot_data_local.attitude.yaw);
    }
}

int controller_update(void) {
    attitude_t attitude;
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

    robot_data.pitch_setpoint = calibrate_pitch_setpoint();
    printk("Pitch setpoint: %f\n", robot_data.pitch_setpoint);

    while (1) {
        if (imu_read(&robot_data.imu_data) == 0) {
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
            // if(k_msgq_put(&log_msgq, &robot_data, K_NO_WAIT) != 0) {
            //     k_msgq_purge(&log_msgq); // Clear the queue if it's full
            // }

        } else {
            printk("IMU read failed\n");
        }
        k_msleep(CONTROL_PERIOD_MS);
    }

    estimator_deinit(&estimator);
    return 0;
}