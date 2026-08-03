#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define PWM_CHANNEL 1u
#define PWM_PERIOD_NS 200000u   /* 5 kHz */
#define PWM_DUTY_PERCENT 50u

static const struct device *motor1 = DEVICE_DT_GET(DT_ALIAS(motor_1));

static void motor_set_duty(uint8_t duty_percent)
{
    uint32_t pulse_ns;

    if (!device_is_ready(motor1)) {
        printk("PWM device not ready\n");
        return;
    }

    pulse_ns = (PWM_PERIOD_NS * duty_percent) / 100u;

    int ret = pwm_set(motor1, PWM_CHANNEL, PWM_PERIOD_NS,
                      pulse_ns, PWM_POLARITY_NORMAL);
    if (ret < 0) {
        printk("pwm_set failed: %d\n", ret);
    }
}

int main(void)
{
	while (1) {
		for (uint8_t duty = 0; duty <= 100; duty++) {
			motor_set_duty(duty);
			k_msleep(20); // Delay để đèn sáng dần
		}
		for (uint8_t duty = 100; duty > 0; duty--) {
			motor_set_duty(duty);
			k_msleep(20); // Delay để đèn tắt dần
		}
	}
    return 0;
}