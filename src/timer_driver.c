// timer_driver.c
#include "../inc/timer_driver.h"
#include <Arduino.h>

// Arduino Uno timer base addresses (not used directly in Arduino framework but keeping for API compatibility)
#define TIMER0_BASE 0x00
#define TIMER1_BASE 0x01
#define TIMER2_BASE 0x02

// Arduino timer implementation - simplified for compatibility

void Timer_Init(uint32_t timer_base, uint32_t prescaler, uint32_t auto_reload) {
    // In Arduino, we don't manipulate timer registers directly in typical applications
    // This function serves as a compatibility layer

    // For more advanced timer control, we would need direct register access:
    if (timer_base == TIMER1_BASE) {
        // Timer1 is a 16-bit timer we could use for custom timing
        noInterrupts(); // Disable interrupts during timer setup
        
        // Reset Timer1 control registers
        TCCR1A = 0;
        TCCR1B = 0;
        
        // Set prescaler
        // Arduino 16MHz clock / prescaler = timer clock
        // Prescaler options: 1, 8, 64, 256, 1024
        if (prescaler <= 1) TCCR1B |= (1 << CS10); // No prescaling
        else if (prescaler <= 8) TCCR1B |= (1 << CS11); // /8 prescaler
        else if (prescaler <= 64) TCCR1B |= (1 << CS11) | (1 << CS10); // /64 prescaler
        else if (prescaler <= 256) TCCR1B |= (1 << CS12); // /256 prescaler
        else TCCR1B |= (1 << CS12) | (1 << CS10); // /1024 prescaler
        
        // Set compare match value (auto reload)
        OCR1A = auto_reload;
        
        // Enable CTC mode
        TCCR1B |= (1 << WGM12);
        
        interrupts(); // Re-enable interrupts
    }
}

void Timer_Start(uint32_t timer_base) {
    if (timer_base == TIMER1_BASE) {
        // Reset counter
        TCNT1 = 0;
        
        // Enable compare match interrupt if needed
        // TIMSK1 |= (1 << OCIE1A);
    }
}

void Timer_Stop(uint32_t timer_base) {
    if (timer_base == TIMER1_BASE) {
        // Disable all clock sources to stop the timer
        TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
    }
}

uint32_t Timer_GetCounter(uint32_t timer_base) {
    if (timer_base == TIMER1_BASE) {
        return TCNT1;
    }
    return 0;
}

void Timer_EnableInterrupt(uint32_t timer_base) {
    if (timer_base == TIMER1_BASE) {
        // Enable Timer1 compare match interrupt
        TIMSK1 |= (1 << OCIE1A);
    }
}

void Timer_DisableInterrupt(uint32_t timer_base) {
    if (timer_base == TIMER1_BASE) {
        // Disable Timer1 compare match interrupt
        TIMSK1 &= ~(1 << OCIE1A);
    }
}

void Timer_ClearInterruptFlag(uint32_t timer_base) {
    if (timer_base == TIMER1_BASE) {
        // Clear Timer1 compare match interrupt flag
        TIFR1 |= (1 << OCF1A);
    }
}

void Timer_SetAutoReload(uint32_t timer_base, uint32_t auto_reload) {
    if (timer_base == TIMER1_BASE) {
        // Set Timer1 compare match value
        OCR1A = auto_reload;
    }
}

// Helper function for microsecond delays - implemented using Arduino's delayMicroseconds
void delay_us(uint32_t us) {
    delayMicroseconds(us);
}

// Helper function for millisecond delays - implemented using Arduino's delay
void delay_ms(uint32_t ms) {
    delay(ms);
}