// pwm_driver.c
// pwm_driver.c
#include "../inc/pwm_driver.h"
#include "../inc/gpio_driver.h"
#include "../inc/avr_registers.h"

// These register definitions should match those in timer_driver.h
// ATmega328P Timer Registers
#define TCCR0A  (*(volatile uint8_t*)0x44)
#define TCCR0B  (*(volatile uint8_t*)0x45)
#define OCR0A   (*(volatile uint8_t*)0x47)
#define OCR0B   (*(volatile uint8_t*)0x48)

#define TCCR1A  (*(volatile uint8_t*)0x80)
#define TCCR1B  (*(volatile uint8_t*)0x81)
#define OCR1A   (*(volatile uint16_t*)0x88)
#define OCR1B   (*(volatile uint16_t*)0x8A)

#define TCCR2A  (*(volatile uint8_t*)0xB0)
#define TCCR2B  (*(volatile uint8_t*)0xB1)
#define OCR2A   (*(volatile uint8_t*)0xB3)
#define OCR2B   (*(volatile uint8_t*)0xB4)

// Timer/Counter Control Register bits - these should be defined in timer_driver.h
// but we'll include them here for reference
// These should match the definitions in timer_driver.h
#define CS00    0
#define CS01    1
#define CS02    2
#define WGM00   0
#define WGM01   1
#define WGM02   3
#define COM0A1  7
#define COM0B1  5

#define CS10    0
#define CS11    1
#define CS12    2
#define WGM10   0
#define WGM11   1
#define WGM12   3
#define COM1A1  7
#define COM1B1  5

#define CS20    0
#define CS21    1
#define CS22    2
#define WGM20   0
#define WGM21   1
#define WGM22   3
#define COM2A1  7
#define COM2B1  5

void PWM_Init(PWM_Channel* pwm) {
    // Set pin as output using GPIO driver
    GPIO_Init(pwm->pin, GPIO_OUTPUT);
    
    switch(pwm->pin) {
        case 3: // OC2B
            // Set Timer2 to Fast PWM mode
            TCCR2A |= (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
            TCCR2B |= (1 << CS21); // Prescaler = 8
            OCR2B = 0; // Initial duty cycle = 0
            break;
            
        case 5: // OC0B
            // Set Timer0 to Fast PWM mode
            TCCR0A |= (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
            TCCR0B |= (1 << CS01); // Prescaler = 8
            OCR0B = 0; // Initial duty cycle = 0
            break;
            
        case 6: // OC0A
            // Set Timer0 to Fast PWM mode
            TCCR0A |= (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
            TCCR0B |= (1 << CS01); // Prescaler = 8
            OCR0A = 0; // Initial duty cycle = 0
            break;
            
        case 9: // OC1A
            // Set Timer1 to Fast PWM mode, 8-bit
            TCCR1A |= (1 << COM1A1) | (1 << WGM10);
            TCCR1B |= (1 << WGM12) | (1 << CS11); // Prescaler = 8
            OCR1A = 0; // Initial duty cycle = 0
            break;
            
        case 10: // OC1B
            // Set Timer1 to Fast PWM mode, 8-bit
            TCCR1A |= (1 << COM1B1) | (1 << WGM10);
            TCCR1B |= (1 << WGM12) | (1 << CS11); // Prescaler = 8
            OCR1B = 0; // Initial duty cycle = 0
            break;
            
        case 11: // OC2A
            // Set Timer2 to Fast PWM mode
            TCCR2A |= (1 << COM2A1) | (1 << WGM21) | (1 << WGM20);
            TCCR2B |= (1 << CS21); // Prescaler = 8
            OCR2A = 0; // Initial duty cycle = 0
            break;
    }
}

void PWM_SetDutyCycle(PWM_Channel* pwm, uint8_t duty_cycle) {
    // Limit duty cycle to 0-100%
    if (duty_cycle > 100) {
        duty_cycle = 100;
    }
    
    // Convert percentage to PWM value (0-255)
    uint8_t pwm_value = (duty_cycle * 255)/100;
    
    // Set PWM value based on pin
    switch(pwm->pin) {
        case 3:  // OC2B
            OCR2B = pwm_value;
            break;
        case 5:  // OC0B
            OCR0B = pwm_value;
            break;
        case 6:  // OC0A
            OCR0A = pwm_value;
            break;
        case 9:  // OC1A
            OCR1A = pwm_value;
            break;
        case 10: // OC1B
            OCR1B = pwm_value;
            break;
        case 11: // OC2A
            OCR2A = pwm_value;
            break;
    }
}

void PWM_Start(PWM_Channel* pwm) {
    // PWM is automatically running after initialization
    // This function is for API compatibility
}

void PWM_Stop(PWM_Channel* pwm) {
    PWM_SetDutyCycle(pwm, 0);
}