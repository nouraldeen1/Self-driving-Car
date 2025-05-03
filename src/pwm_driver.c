// pwm_driver.c
#include "../inc/pwm_driver.h"
#include "../inc/timer_driver.h"
#include "../inc/gpio_driver.h"
#include <Arduino.h>

// For Arduino Uno, we'll use analogWrite instead of direct register manipulation

void PWM_Init(PWM_Channel* pwm) {
    // Set pin as output
    pinMode(pwm->pin, OUTPUT);
    
    // For Arduino Uno, we don't need to do anything else here
    // as analogWrite will handle the PWM setup
    
    // Initialize with 0 duty cycle
    analogWrite(pwm->pin, 0);
}

void PWM_SetDutyCycle(PWM_Channel* pwm, uint8_t duty_cycle) {
    // Limit duty cycle to 0-100%
    if (duty_cycle > 100) {
        duty_cycle = 100;
    }
    
    // Convert percentage to Arduino PWM value (0-255)
    uint8_t pwm_value = (duty_cycle * 255) / 100;
    
    // Set PWM value using Arduino's analogWrite function
    analogWrite(pwm->pin, pwm_value);
}

void PWM_Start(PWM_Channel* pwm) {
    // For Arduino, analogWrite automatically starts PWM
    // No additional action needed
}

void PWM_Stop(PWM_Channel* pwm) {
    // Stop PWM by setting duty cycle to 0
    analogWrite(pwm->pin, 0);
}

// Implementation of the Timer_SetAutoReload function
// This is not directly used in Arduino, but we'll implement it
// to fulfill the requirement
void Timer_SetAutoReload(uint32_t timer_base, uint32_t auto_reload) {
    // On Arduino Uno, this would normally set the timer's reload value
    // For compatibility with Arduino, we'll make this a no-op
    // as we're using Arduino's own timer management
    
    // If needed, for advanced usage, this could be implemented using
    // direct timer register access:
    
    // Example for Timer 1 (not fully implemented):
    // if (timer_base == TIMER1_BASE) {
    //     // Set the reload value for Timer 1
    //     OCR1A = auto_reload;
    // }
}