#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define PWM_CHANNEL     1U

/* Counter = 1 MHz (PSC = 71) */
#define PWM_PERIOD      1000U      /* 1000 cycles -> 1000 Hz */
#define PWM_PULSE       (PWM_PERIOD / 2)       /* 50% duty */

static const struct device *motor1 = DEVICE_DT_GET(DT_ALIAS(motor_1));

int main(void)
{
    int ret;

    if (!device_is_ready(motor1)) {
        printk("PWM device not ready\n");
        return 0;
    }

    ret = pwm_set_cycles(motor1,
                         PWM_CHANNEL,
                         PWM_PERIOD,
                         PWM_PULSE,
                         PWM_POLARITY_NORMAL);

    if (ret) {
        printk("pwm_set_cycles failed %d\n", ret);
    }

    while (1) {
        k_sleep(K_FOREVER);
    }
}