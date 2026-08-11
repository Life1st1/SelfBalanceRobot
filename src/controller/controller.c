#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <math.h>

#include "../drivers/imu/mpu6650.h"
#include "../sensor_fusion/state_estimator.h"

#include "controller.h"

#ifndef ESTIMATOR_FILTER_NAME
#define ESTIMATOR_FILTER_NAME "complementary"
#endif

#define PWM_CHANNEL       1U
#define MOTOR_MIN_PERIOD  00U
#define MOTOR_MAX_PERIOD  300U
#define CONTROL_PERIOD_MS 1U
#define MAX_SPEED         100.0f

static const struct pwm_dt_spec motor1 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor1));
static const struct pwm_dt_spec motor2 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor2));

static const struct gpio_dt_spec motor_dir =
    GPIO_DT_SPEC_GET(DT_ALIAS(motor_dir), gpios);
static const struct gpio_dt_spec motor_dir2 =
    GPIO_DT_SPEC_GET(DT_ALIAS(motor_dir2), gpios);

void set_motor_speed(uint8_t id, uint32_t hz);

static float pid_kp = 110.0f;
static float pid_kd = 0.0f;
static float pid_ki = 2.0f;
static float pid_integral;
static float pid_prev_error;

void update_pid_gains(float kp, float ki, float kd) {
    pid_kp = kp;
    pid_ki = ki;
    pid_kd = kd;
    printk("Updated PID gains: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", pid_kp, pid_ki, pid_kd);
}

static float pid_compute(float setpoint, float measurement, float dt_ms)
{
    float error = setpoint - measurement;
    float dt = dt_ms / 1000.0f;

    pid_integral += error * dt;
    float derivative = (error - pid_prev_error) / dt;
    pid_prev_error = error;

    float output = pid_kp * error + pid_ki * pid_integral + pid_kd * derivative;
    // //printk("PID compute: setpoint=%.2f, measurement=%.2f, error=%.2f, integral=%.2f, derivative=%.2f, output=%.2f, dt=%.2f\n",
    //        //setpoint, measurement, error, pid_integral, derivative, output, dt);
    // if (output > MAX_SPEED) {
    //     //printk("PID output exceeds MAX_SPEED, limiting to %d\n", MAX_SPEED);
    //     output = MAX_SPEED;
    // } else if (output < -MAX_SPEED) {
    //     //printk("PID output below -MAX_SPEED, limiting to %d\n", -MAX_SPEED);
    //     output = -MAX_SPEED;
    // }
    // //printk("PID compute: setpoint=%.2f, measurement=%.2f, error=%.2f, integral=%.2f, derivative=%.2f, output=%.2f, dt=%.2f\n",
    //        //setpoint, measurement, error, pid_integral, derivative, output, dt);
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

int controller_update(void) {
    imu_raw_t imu;
    attitude_t attitude;
    estimator_handle_t estimator;
    float setpoint = 0.0f; // Desired pitch angle (upright position)

    //k_msleep(4000);
    printk("Starting Self Balance Robot\n");

    if (!pwm_is_ready_dt(&motor1)) {
        printk("PWM device not ready\n");
        return 0;
    }
    if (!pwm_is_ready_dt(&motor2)) {
        printk("PWM device not ready\n");
        return 0;
    }
    printk("PWM device is ready\n");

    printk("Motor dir.port: %p, pin: %d\n", motor_dir.port, motor_dir.pin);
    printk("Motor dir2.port: %p, pin: %d\n", motor_dir2.port, motor_dir2.pin);
    if (motor_dir.port != NULL && !gpio_is_ready_dt(&motor_dir)) {
        printk("Motor direction GPIO not ready\n");
        return 0;
    }
    if (motor_dir2.port != NULL && !gpio_is_ready_dt(&motor_dir2)) {
        printk("Motor direction 2 GPIO not ready\n");
        return 0;
    }
    printk("Motor direction GPIO is ready\n");
    printk("Motor direction GPIO port: %p, pin: %d\n", motor_dir.port, motor_dir.pin);
    if (motor_dir.port != NULL) {
        if (gpio_pin_configure_dt(&motor_dir, GPIO_OUTPUT_INACTIVE) != 0) {
            printk("Motor direction GPIO configure failed\n");
            return 0;
        }
        k_msleep(100);
        printk("Motor direction GPIO configured1\n");
    }
    if (motor_dir2.port != NULL) {
        if (gpio_pin_configure_dt(&motor_dir2, GPIO_OUTPUT_INACTIVE) != 0) {
            printk("Motor direction 2 GPIO configure failed\n");
            return 0;
        }
        k_msleep(100);
        printk("Motor direction 2 GPIO configured1\n");
    }
    printk("Motor direction GPIO configured2\n");

    if (imu_init() != 0) {
        printk("IMU init failed\n");
        return 0;
    }
    printk("IMU initialized\n");

    if (estimator_init_by_name(&estimator, "madgwick") != 0) {
        printk("Madgwick estimator init failed\n");
        return 0;
    }
    printk("Madgwick estimator initialized\n");
    //test get pitch value
    imu_read(&imu);
    estimator_update(&estimator, &imu, &attitude);
    printk("Initial pitch: %f\n", attitude.pitch);

    float pitch_setpoint = calibrate_pitch_setpoint();
    printk("Pitch setpoint: %f\n", pitch_setpoint);

    while (1) {
        if (imu_read(&imu) == 0) {
            estimator_update(&estimator, &imu, &attitude);

            if (attitude.pitch > 45.0f || attitude.pitch < -45.0f) {
                printk("Pitch angle out of range: %.2f\n", attitude.pitch);
                motor_disable();
                k_msleep(1000);
                continue;
            }

            float pid_output = pid_compute(pitch_setpoint, attitude.pitch, CONTROL_PERIOD_MS);
            bool direction = (pid_output >= 0.0f);
            //printk("Pitch: %.2f, Direction: %s, output: %.2f, ", attitude.pitch, direction ? "forward" : "backward", pid_output);
            set_motor_direction(1, direction);
            set_motor_direction(2, direction);
            set_motor_speed(1, (uint32_t)(pid_output < 0.0f ? -pid_output : pid_output));
            set_motor_speed(2, (uint32_t)(pid_output < 0.0f ? -pid_output : pid_output));

        } else {
            printk("IMU read failed\n");
        }

        //k_msleep(CONTROL_PERIOD_MS);
        k_usleep(500);
    }

    estimator_deinit(&estimator);
    return 0;
}