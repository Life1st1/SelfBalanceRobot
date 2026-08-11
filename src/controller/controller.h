#ifndef CONTROLLER_H
#define CONTROLLER_H

int controller_update(void);
void update_pid_gains(float kp, float ki, float kd);

#endif // CONTROLLER_H