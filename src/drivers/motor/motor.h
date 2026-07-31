#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>
#include <stdint.h>
#include "../../common/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MOTOR_DIRECTION_FORWARD = 1,
    MOTOR_DIRECTION_REVERSE = -1,
    MOTOR_DIRECTION_BRAKE = 0
} motor_direction_t;

void motor_init(uint8_t id);
void motor_deinit(uint8_t id);
void motor_enable(uint8_t id);
void motor_disable(uint8_t id);
void motor_set_command(uint8_t id,
                       const motor_command_t *cmd);
void motor_set_velocity(uint8_t id,
                        float vel);
void motor_set_direction(uint8_t id,
                         motor_direction_t dir);
void motor_set_pwm(uint8_t id,
                   float duty_cycle);
void motor_update(uint8_t id,
                  float measured_velocity,
                  float dt);
float motor_get_velocity(uint8_t id);
bool motor_is_enabled(uint8_t id);
void motor_get_state(uint8_t id,
                     motor_state_t *state);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_H