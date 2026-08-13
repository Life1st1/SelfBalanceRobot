#ifndef CONTROLLER_H
#define CONTROLLER_H

extern struct k_msgq log_msgq;
extern struct k_mutex pid_mutex;

int controller_update(void);
void update_pid_gains(float kp, float ki, float kd);
void print_robot_data(void);

#endif // CONTROLLER_H