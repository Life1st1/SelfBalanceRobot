#ifndef MOTOR_H
#define MOTOR_H

void motor_set_command(uint8_t id,
                       const motor_command_t *cmd);
void motor_set_velocity(uint8_t id,
                        float vel);
float motor_get_velocity(uint8_t id);
motor_enable()
motor_disable()

#endif // MOTOR_H