// inc/pwm_driver.h
#ifndef PWM_DRIVER_H
#define PWM_DRIVER_H

#include <stdint.h>

// ATmega328P PWM Capable Pins:
// Pin 3:  OC2B
// Pin 5:  OC0B
// Pin 6:  OC0A
// Pin 9:  OC1A
// Pin 10: OC1B
// Pin 11: OC2A

// Simple PWM channel definition
typedef struct {
    uint8_t pin;       // Arduino pin number (3, 5, 6, 9, 10, or 11)
} PWM_Channel;

// Function prototypes
void PWM_Init(PWM_Channel* pwm);
void PWM_SetDutyCycle(PWM_Channel* pwm, uint8_t duty_cycle);
void PWM_Start(PWM_Channel* pwm);
void PWM_Stop(PWM_Channel* pwm);

#endif // PWM_DRIVER_H