// inc/pwm_driver.h
#ifndef PWM_DRIVER_H
#define PWM_DRIVER_H

#include <stdint.h>

// Simple PWM channel definition for Arduino
typedef struct {
    uint8_t pin;         // Arduino pin number
    uint8_t channel;     // Timer channel (if needed)
} PWM_Channel;

// Function prototypes
void PWM_Init(PWM_Channel* pwm);
void PWM_SetDutyCycle(PWM_Channel* pwm, uint8_t duty_cycle);
void PWM_Start(PWM_Channel* pwm);
void PWM_Stop(PWM_Channel* pwm);

#endif // PWM_DRIVER_H