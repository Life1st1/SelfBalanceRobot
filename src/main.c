#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define PWM_CHANNEL     1U

/* Counter = 1 MHz (PSC = 71) */
#define PWM_PERIOD      1000U      /* 1000 cycles -> 1000 Hz */
#define PWM_PULSE       (PWM_PERIOD / 2)       /* 50% duty */

static const struct device *motor1 = DEVICE_DT_GET(DT_ALIAS(motor_1));

void set_motor_speed(uint32_t speed)
{
    const uint32_t min_period = 1500U;
    const uint32_t max_period = 5000U;
    uint32_t period;
    uint32_t pulse;
    int ret;

    if (speed > 100U) {
        speed = 100U;
    }

    period = min_period + ((max_period - min_period) * speed) / 100U;
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
    int ret;

    if (!device_is_ready(motor1)) {
        printk("PWM device not ready\n");
        return 0;
    }

    while (1) {
        for (uint32_t speed = 0U; speed <= 100U; speed++) {
            set_motor_speed(speed);
            k_msleep(50);
        }
        k_msleep(1000);
        for (int speed = 100; speed >= 0; speed--) {
            set_motor_speed((uint32_t)speed);
            k_msleep(50);
        }
        k_msleep(1000);
    }
}