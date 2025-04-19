// motor_driver.h
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "gpio_driver.h"
#include <stdint.h>

typedef struct {
    uint32_t in1_port;
    uint8_t in1_pin;
    uint32_t in2_port;
    uint8_t in2_pin;
    uint32_t ena_port;
    uint8_t ena_pin;
    uint8_t pwm_channel;
} Motor;

typedef enum {
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
    MOTOR_STOP
} MotorDirection;

void Motor_Init(Motor* motor);
void Motor_SetDirection(Motor* motor, MotorDirection direction);
void Motor_SetSpeed(Motor* motor, uint8_t speed); // 0-100%

#endif // MOTOR_DRIVER_H