#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <math.h>

#include "drivers/imu/mpu6650.h"
#include "sensor_fusion/state_estimator.h"

#define PWM_CHANNEL       1U
#define MOTOR_MIN_PERIOD  500U
#define MOTOR_MAX_PERIOD  4000U
#define CONTROL_PERIOD_MS 10U
#define MAX_SPEED         100.0f

static const struct device *motor1 = DEVICE_DT_GET(DT_ALIAS(motor_1));
static const struct gpio_dt_spec motor_dir =
    GPIO_DT_SPEC_GET(DT_ALIAS(motor_dir), gpios);

static float pid_kp = 12.0f;
static float pid_ki = 0.35f;
static float pid_kd = 0.08f;
static float pid_integral;
static float pid_prev_error;

static float pid_compute(float setpoint, float measurement, float dt_ms)
{
    float error = setpoint - measurement;
    float dt = dt_ms / 1000.0f;

    pid_integral += error * dt;
    float derivative = (error - pid_prev_error) / dt;
    pid_prev_error = error;

    float output = pid_kp * error + pid_ki * pid_integral + pid_kd * derivative;
    //printk("PID compute: setpoint=%.2f, measurement=%.2f, error=%.2f, integral=%.2f, derivative=%.2f, output=%.2f, dt=%.2f\n",
           //setpoint, measurement, error, pid_integral, derivative, output, dt);
    if (output > MAX_SPEED) {
        //printk("PID output exceeds MAX_SPEED, limiting to %d\n", MAX_SPEED);
        output = MAX_SPEED;
    } else if (output < -MAX_SPEED) {
        //printk("PID output below -MAX_SPEED, limiting to %d\n", -MAX_SPEED);
        output = -MAX_SPEED;
    }
    //printk("PID compute: setpoint=%.2f, measurement=%.2f, error=%.2f, integral=%.2f, derivative=%.2f, output=%.2f, dt=%.2f\n",
           //setpoint, measurement, error, pid_integral, derivative, output, dt);
    return output;
}

static void set_motor_direction(bool forward)
{
    if (!gpio_is_ready_dt(&motor_dir)) {
        return;
    }

    gpio_pin_set_dt(&motor_dir, forward ? 1 : 0);
}

void set_motor_speed(uint32_t speed)
{
    uint32_t period;
    uint32_t pulse;
    int ret;

    if (speed > MAX_SPEED) {
        speed = MAX_SPEED;
    }

    period = MOTOR_MIN_PERIOD + ((MOTOR_MAX_PERIOD - MOTOR_MIN_PERIOD) * speed) / MAX_SPEED;
    pulse = period / 2U;

    ret = pwm_set_cycles(motor1,
                         PWM_CHANNEL,
                         period,
                         pulse,
                         PWM_POLARITY_NORMAL);
    if (ret) {
        printk("set_motor_speed: pwm_set_cycles failed %d\n", ret);
    }
}

int main(void)
{
    imu_raw_t imu;
    attitude_t attitude;
    estimator_handle_t estimator;

    if (!device_is_ready(motor1)) {
        printk("PWM device not ready\n");
        return 0;
    }
    printk("PWM device is ready\n");
    printk("Motor dir.port: %p, pin: %d\n", motor_dir.port, motor_dir.pin);
    if (motor_dir.port != NULL && !gpio_is_ready_dt(&motor_dir)) {
        printk("Motor direction GPIO not ready\n");
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
    printk("Motor direction GPIO configured2\n");
    //test direction
    set_motor_direction(true);
    k_msleep(1000);
    set_motor_direction(false);
    k_msleep(1000);

    if (imu_init() != 0) {
        printk("IMU init failed\n");
        return 0;
    }
    printk("IMU initialized\n");

    if (estimator_init_by_name(&estimator, "complementary") != 0) {
        printk("Complementary estimator init failed\n");
        return 0;
    }
    printk("Complementary estimator initialized\n");
    //test get pitch value
    imu_read(&imu);
    estimator_update(&estimator, &imu, &attitude);
    printk("Initial pitch: %f\n", attitude.pitch);

    while (1) {
        if (imu_read(&imu) == 0) {
            estimator_update(&estimator, &imu, &attitude);

            float pid_output = pid_compute(0.0f, attitude.pitch, CONTROL_PERIOD_MS);
            bool forward = (pid_output >= 0.0f);
            printk("Pitch: %.2f, PID output: %.2f, Direction: %s\n", attitude.pitch, pid_output, forward ? "forward" : "backward");
            set_motor_direction(forward);
            set_motor_speed(100-((uint32_t)(pid_output < 0.0f ? -pid_output : pid_output)));
        } else {
            printk("IMU read failed\n");
        }

        k_msleep(CONTROL_PERIOD_MS);
    }

    estimator_deinit(&estimator);
    return 0;
}