// inc/motor_driver.h
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>
#include "gpio_driver.h"
#include "pwm_driver.h"

// Motor direction
typedef enum {
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
    MOTOR_STOP
} MotorDirection;

// Motor structure
typedef struct {
    uint8_t in1_pin;   // Arduino pin number for IN1
    uint8_t in2_pin;   // Arduino pin number for IN2
    uint8_t ena_pin;   // Arduino pin number for ENA (PWM pin)
} Motor;

// Function prototypes
void Motor_Init(Motor* motor);
void Motor_SetDirection(Motor* motor, MotorDirection direction);
void Motor_SetSpeed(Motor* motor, uint8_t speed); // 0-100%

#endif // MOTOR_DRIVER_H