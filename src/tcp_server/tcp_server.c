#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>

#include <zephyr/posix/netinet/in.h>
#include <zephyr/posix/sys/socket.h>
#include <zephyr/posix/arpa/inet.h>
#include <zephyr/posix/unistd.h>

#include <zephyr/sys/printk.h>
#include "../controller/controller.h"

#include <errno.h>
#include <string.h>
#include <stdio.h>

#define TCP_PORT 5000
#define RX_BUFFER_SIZE 512

#define STACK_SIZE 4096
#define PRIORITY 5

K_THREAD_STACK_DEFINE(
    tcp_server_stack,
    STACK_SIZE
);

static struct k_thread tcp_server_thread_data;

void parse_json_and_update_pid(const char *json_str);


static void tcp_server_thread(
    void *p1,
    void *p2,
    void *p3
)
{
    int server;
    int client;
    int ret;

    struct sockaddr_in server_addr;

    char rx_buffer[RX_BUFFER_SIZE];

    /*
     * Create TCP socket
     */
    server = socket(
        AF_INET,
        SOCK_STREAM,
        IPPROTO_TCP
    );

    if (server < 0) {
        printk(
            "TCP: socket failed: %d\n",
            errno
        );

        return;
    }

    printk("TCP: socket created\n");


    /*
     * Server address
     */
    memset(
        &server_addr,
        0,
        sizeof(server_addr)
    );

    server_addr.sin_family = AF_INET;

    server_addr.sin_port =
        htons(TCP_PORT);

    server_addr.sin_addr.s_addr =
        htonl(INADDR_ANY);


    /*
     * Bind
     */
    ret = bind(
        server,
        (struct sockaddr *)&server_addr,
        sizeof(server_addr)
    );

    if (ret < 0) {

        printk(
            "TCP: bind failed: %d\n",
            errno
        );

        close(server);

        return;
    }

    printk(
        "TCP: bind OK, port %d\n",
        TCP_PORT
    );


    /*
     * Listen
     */
    ret = listen(
        server,
        1
    );

    if (ret < 0) {

        printk(
            "TCP: listen failed: %d\n",
            errno
        );

        close(server);

        return;
    }

    printk(
        "TCP: listening on port %d\n",
        TCP_PORT
    );


    /*
     * Accept client
     */
    while (1) {

        struct sockaddr_in client_addr;

        socklen_t client_addr_len =
            sizeof(client_addr);

        printk(
            "TCP: waiting for client...\n"
        );

        client = accept(
            server,
            (struct sockaddr *)&client_addr,
            &client_addr_len
        );

        if (client < 0) {

            printk(
                "TCP: accept failed: %d\n",
                errno
            );

            continue;
        }

        printk(
            "TCP: client connected\n"
        );


        /*
         * Receive data
         */
        while (1) {

            int len;

            len = recv(
                client,
                rx_buffer,
                sizeof(rx_buffer) - 1,
                0
            );

            if (len <= 0) {

                if (len == 0) {

                    printk(
                        "TCP: client disconnected\n"
                    );

                } else {

                    printk(
                        "TCP: recv failed: %d\n",
                        errno
                    );
                }

                break;
            }


            /*
             * Null terminate
             */
            rx_buffer[len] = '\0';


            /*
             * Print received data
             */
            printk(
                "TCP RX (%d bytes): %s",
                len,
                rx_buffer
            );
            // example output: TCP RX (45 bytes): {"type":"pid","kp":50.0,"ki":0.35,"kd":0.08}
            parse_json_and_update_pid(rx_buffer);
        }


        close(client);

        printk(
            "TCP: connection closed\n"
        );
    }
}

void parse_json_and_update_pid(const char *json_str) {
    if (json_str == NULL) {
        return;
    }

    /* Basic string-based parser for messages like:
     * {"type":"pid","kp":50.0,"ki":0.35,"kd":0.08}
     * This avoids depending on cJSON at build time.
     */
    if (strstr(json_str, "\"type\":\"pid\"") == NULL &&
        strstr(json_str, "\"type\": \"pid\"") == NULL) {
        printk("Unknown or unsupported JSON command\n");
        return;
    }

    const char *p;
    float kp = 0.0f, ki = 0.0f, kd = 0.0f;
    bool ok = true;

    p = strstr(json_str, "\"kp\"");
    if (p) {
        p = strchr(p, ':');
        if (!p || sscanf(p + 1, " %f", &kp) != 1) ok = false;
    } else ok = false;

    p = strstr(json_str, "\"ki\"");
    if (p) {
        p = strchr(p, ':');
        if (!p || sscanf(p + 1, " %f", &ki) != 1) ok = false;
    } else ok = false;

    p = strstr(json_str, "\"kd\"");
    if (p) {
        p = strchr(p, ':');
        if (!p || sscanf(p + 1, " %f", &kd) != 1) ok = false;
    } else ok = false;

    if (ok) {
        update_pid_gains(kp, ki, kd);
        printk("Updated PID gains: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", kp, ki, kd);
    } else {
        printk("Invalid or missing PID parameters in JSON\n");
    }
}

void tcp_server_start(void)
{
    k_thread_create(
        &tcp_server_thread_data,
        tcp_server_stack,
        K_THREAD_STACK_SIZEOF(
            tcp_server_stack
        ),
        tcp_server_thread,
        NULL,
        NULL,
        NULL,
        PRIORITY,
        0,
        K_NO_WAIT
    );

    k_thread_name_set(
        &tcp_server_thread_data,
        "tcp_server"
    );
}