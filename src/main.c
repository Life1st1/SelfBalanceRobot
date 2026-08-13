/*
 * Copyright (c) 2016 Intel Corporation
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Sample app to demonstrate PWM.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include "controller/controller.h"
#include "wifi_connect/wifi_setup.h"
#include "wifi_connect/net_sample_common.h"
#include "tcp_server/tcp_server.h"
#include "drivers/motor/motor.h"
#include "common/defines.h"
#include "controller/controller.h"

/* size of stack area used by each thread */
#define STACKSIZE 2048

/* scheduling priority used by each thread */
#define PRIORITY_WIFI 7
#define PRIORITY_CONTROLLER -20
#define PRIORITY_LOG 8
#define SAMPLE_TIMER DT_INST(0, espressif_esp32_counter)

K_MSGQ_DEFINE(log_msgq, sizeof(RobotData_t), 1, 1);

K_THREAD_STACK_DEFINE(thread_init_wifi_stack_area, STACKSIZE);
K_THREAD_STACK_DEFINE(robot_controller_stack_area, STACKSIZE);
K_THREAD_STACK_DEFINE(log_thread_stack_area, STACKSIZE);

static struct k_thread thread_init_wifi_data;
static struct k_thread robot_controller_thread_data;
static struct k_thread log_thread_data;


void thread_init_wifi(void *dummy1, void *dummy2, void *dummy3)
{
    int retry_count = 0;
    const int max_retries = 5;
    k_sleep(K_SECONDS(5));

    init_wifi();
	connect_to_wifi();

    wait_for_network();
    k_sleep(K_SECONDS(3));
    tcp_server_start();
}

void thread_robot_controller(void *dummy1, void *dummy2, void *dummy3)
{
    controller_update();
}

void thread_print_robot_data(void *dummy1, void *dummy2, void *dummy3)
{
    print_robot_data();
}

#define DELAY 2000000
#define ALARM_CHANNEL_ID 0
#define ALARM_FLAGS 0

struct counter_alarm_cfg alarm_cfg;

static void test_counter_interrupt_fn(const struct device *counter_dev,
				      uint8_t chan_id, uint32_t ticks,
				      void *user_data)
{
	struct counter_alarm_cfg *config = user_data;
	uint32_t now_ticks;
	uint64_t now_usec;
	int now_sec;
	int err;

	err = counter_get_value(counter_dev, &now_ticks);
	if (!counter_is_counting_up(counter_dev)) {
		now_ticks = counter_get_top_value(counter_dev) - now_ticks;
	}

	if (err) {
		printk("Failed to read counter value (err %d)", err);
		return;
	}

	now_usec = counter_ticks_to_us(counter_dev, now_ticks);
	now_sec = (int)(now_usec / USEC_PER_SEC);

	printk("!!! Alarm !!!\n");
	printk("Now: %u\n", now_sec);

	/* Set a new alarm with a double length duration */
	config->ticks = config->ticks * 2U;

	printk("Set alarm in %u sec (%u ticks)\n",
	       (uint32_t)(counter_ticks_to_us(counter_dev,
					   config->ticks) / USEC_PER_SEC),
	       config->ticks);

	err = counter_set_channel_alarm(counter_dev, ALARM_CHANNEL_ID,
					user_data);
	if (err != 0) {
		printk("Alarm could not be set\n");
	}
}

int timer_init(void)
{
	const struct device *const counter_dev = DEVICE_DT_GET(SAMPLE_TIMER);
	int err;

	printk("Counter alarm sample\n\n");

	if (!device_is_ready(counter_dev)) {
		printk("device not ready.\n");
		return 0;
	}

	counter_start(counter_dev);

	alarm_cfg.flags = ALARM_FLAGS;
	alarm_cfg.ticks = counter_us_to_ticks(counter_dev, DELAY);
	alarm_cfg.callback = test_counter_interrupt_fn;
	alarm_cfg.user_data = &alarm_cfg;

	err = counter_set_channel_alarm(counter_dev, ALARM_CHANNEL_ID,
					&alarm_cfg);
	printk("Set alarm in %u sec (%u ticks)\n",
	       (uint32_t)(counter_ticks_to_us(counter_dev,
					   alarm_cfg.ticks) / USEC_PER_SEC),
	       alarm_cfg.ticks);

	if (-EINVAL == err) {
		printk("Alarm settings invalid\n");
	} else if (-ENOTSUP == err) {
		printk("Alarm setting request not supported\n");
	} else if (err != 0) {
		printk("Error\n");
	}
}

int main(void)
{
    printk("Self Balance Robot\n");
    timer_init();

    k_mutex_init(&pid_mutex);

    k_thread_create(&thread_init_wifi_data, thread_init_wifi_stack_area,
                    K_THREAD_STACK_SIZEOF(thread_init_wifi_stack_area),
                    thread_init_wifi, NULL, NULL, NULL,
                    PRIORITY_WIFI, 0, K_NO_WAIT);

    k_thread_create(&robot_controller_thread_data, robot_controller_stack_area,
                    K_THREAD_STACK_SIZEOF(robot_controller_stack_area),
                    thread_robot_controller, NULL, NULL, NULL,
                    PRIORITY_CONTROLLER, 0, K_NO_WAIT);
    
    k_thread_create(&log_thread_data, log_thread_stack_area,
                    K_THREAD_STACK_SIZEOF(log_thread_stack_area),
                    thread_print_robot_data, NULL, NULL, NULL,
                    PRIORITY_LOG, 0, K_NO_WAIT);
    return 0;
}
