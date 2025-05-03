// src/motor_driver.c
#include "../inc/motor_driver.h"

void Motor_Init(Motor* motor) {
    // Initialize pins
    GPIO_Init(motor->in1_pin, GPIO_OUTPUT);
    GPIO_Init(motor->in2_pin, GPIO_OUTPUT);
    
    // Set up PWM for speed control
    PWM_Channel pwm_channel;
    pwm_channel.pin = motor->ena_pin;
    PWM_Init(&pwm_channel);
    
    // Initially stop the motor
    Motor_SetDirection(motor, MOTOR_STOP);
    Motor_SetSpeed(motor, 0);
}

void Motor_SetDirection(Motor* motor, MotorDirection direction) {
    switch (direction) {
        case MOTOR_FORWARD:
            GPIO_Write(motor->in1_pin, GPIO_HIGH);
            GPIO_Write(motor->in2_pin, GPIO_LOW);
            break;
            
        case MOTOR_BACKWARD:
            GPIO_Write(motor->in1_pin, GPIO_LOW);
            GPIO_Write(motor->in2_pin, GPIO_HIGH);
            break;
            
        case MOTOR_STOP:
        default:
            GPIO_Write(motor->in1_pin, GPIO_LOW);
            GPIO_Write(motor->in2_pin, GPIO_LOW);
            break;
    }
}

void Motor_SetSpeed(Motor* motor, uint8_t speed) {
    // Limit speed to 0-100%
    if (speed > 100) speed = 100;
    
    // Create a temporary PWM channel for the enable pin
    PWM_Channel pwm_channel;
    pwm_channel.pin = motor->ena_pin;
    
    // Set PWM duty cycle to control speed
    PWM_SetDutyCycle(&pwm_channel, speed);
}