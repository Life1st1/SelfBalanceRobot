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
#include "controller/controller.h"
#include "wifi_connect/wifi_setup.h"
#include "wifi_connect/net_sample_common.h"
#include "tcp_server/tcp_server.h"
#include "drivers/motor/motor.h"

/* size of stack area used by each thread */
#define STACKSIZE 2048

/* scheduling priority used by each thread */
#define PRIORITY_WIFI 7
#define PRIORITY_CONTROLLER 5
K_THREAD_STACK_DEFINE(thread_init_wifi_stack_area, STACKSIZE);
K_THREAD_STACK_DEFINE(robot_controller_stack_area, STACKSIZE);
static struct k_thread thread_init_wifi_data;
static struct k_thread robot_controller_thread_data;

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

int main(void)
{
    printk("Self Balance Robot\n");
    k_thread_create(&thread_init_wifi_data, thread_init_wifi_stack_area,
                    K_THREAD_STACK_SIZEOF(thread_init_wifi_stack_area),
                    thread_init_wifi, NULL, NULL, NULL,
                    PRIORITY_WIFI, 0, K_NO_WAIT);

    k_thread_create(&robot_controller_thread_data, robot_controller_stack_area,
                    K_THREAD_STACK_SIZEOF(robot_controller_stack_area),
                    thread_robot_controller, NULL, NULL, NULL,
                    PRIORITY_CONTROLLER, 0, K_NO_WAIT);
    return 0;
}
