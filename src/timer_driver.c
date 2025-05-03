// timer_driver.c
// timer_driver.c
#include "../inc/timer_driver.h"
#include "../inc/avr_registers.h"
// ATmega328P Timer Registers
#define TCCR0A  (*(volatile uint8_t*)0x44)
#define TCCR0B  (*(volatile uint8_t*)0x45)
#define TCNT0   (*(volatile uint8_t*)0x46)
#define OCR0A   (*(volatile uint8_t*)0x47)
#define OCR0B   (*(volatile uint8_t*)0x48)
#define TIMSK0  (*(volatile uint8_t*)0x6E)
#define TIFR0   (*(volatile uint8_t*)0x35)

#define TCCR1A  (*(volatile uint8_t*)0x80)
#define TCCR1B  (*(volatile uint8_t*)0x81)
#define TCCR1C  (*(volatile uint8_t*)0x82)
#define TCNT1   (*(volatile uint16_t*)0x84)
#define OCR1A   (*(volatile uint16_t*)0x88)
#define OCR1B   (*(volatile uint16_t*)0x8A)
#define ICR1    (*(volatile uint16_t*)0x86)
#define TIMSK1  (*(volatile uint8_t*)0x6F)
#define TIFR1   (*(volatile uint8_t*)0x36)

#define TCCR2A  (*(volatile uint8_t*)0xB0)
#define TCCR2B  (*(volatile uint8_t*)0xB1)
#define TCNT2   (*(volatile uint8_t*)0xB2)
#define OCR2A   (*(volatile uint8_t*)0xB3)
#define OCR2B   (*(volatile uint8_t*)0xB4)
#define TIMSK2  (*(volatile uint8_t*)0x70)
#define TIFR2   (*(volatile uint8_t*)0x37)

// System clock is 16 MHz for Arduino Uno
#define F_CPU 16000000UL

void Timer_Init(uint32_t timer_base, uint32_t prescaler, uint32_t auto_reload) {
    if (timer_base == TIMER1_BASE) {
        // Timer1 is 16-bit, suitable for precision timing
        // Reset Timer1 control registers
        TCCR1A = 0;
        TCCR1B = 0;
        TCNT1 = 0;
        
        // Set CTC mode (Clear Timer on Compare Match)
        TCCR1B |= (1 << WGM12);
        
        // Set prescaler based on input parameter
        if (prescaler <= 1) {
            TCCR1B |= (1 << CS10); // No prescaling
        } else if (prescaler <= 8) {
            TCCR1B |= (1 << CS11); // /8 prescaler
        } else if (prescaler <= 64) {
            TCCR1B |= (1 << CS11) | (1 << CS10); // /64 prescaler
        } else if (prescaler <= 256) {
            TCCR1B |= (1 << CS12); // /256 prescaler
        } else {
            TCCR1B |= (1 << CS12) | (1 << CS10); // /1024 prescaler
        }
        
        // Set compare match value
        OCR1A = auto_reload;
    }
    // Add similar code for Timer0 and Timer2 if needed
}

void Timer_Start(uint32_t timer_base) {
    if (timer_base == TIMER1_BASE) {
        // Reset counter
        TCNT1 = 0;
    }
}

void Timer_Stop(uint32_t timer_base) {
    if (timer_base == TIMER1_BASE) {
        // Stop timer by setting clock select bits to 0
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
        // Clear Timer1 compare match interrupt flag by writing 1 to it
        TIFR1 |= (1 << OCF1A);
    }
}

// Hardware timer-based precise delay functions
void delay_us(uint32_t us) {
    // Calculate how many clock cycles needed
    uint32_t cycles = (F_CPU / 1000000UL) * us;
    
    // For very short delays, use NOPs
    if (us < 10) {
        for (uint32_t i = 0; i < cycles / 3; i++) {
            asm volatile("nop");
        }
        return;
    }
    
    // For longer delays, use Timer1
    // Reset Timer1
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    
    // Set up timer with no prescaler
    TCCR1B |= (1 << CS10);
    
    // Wait until required time has passed
    while (TCNT1 < cycles / 4); // division by 4 because each instruction takes multiple cycles
    
    // Stop timer
    TCCR1B = 0;
}

void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        delay_us(1000);
    }
}