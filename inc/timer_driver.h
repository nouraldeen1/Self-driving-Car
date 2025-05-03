// inc/timer_driver.h
#ifndef TIMER_DRIVER_H
#define TIMER_DRIVER_H

#include <stdint.h>

// Timer base addresses (symbolic values for API compatibility)
#define TIMER0_BASE 0x00
#define TIMER1_BASE 0x01
#define TIMER2_BASE 0x02
// These definitions should be added to timer_driver.h

// Timer/Counter Control Register bits
// For Timer0
#define CS00    0   // Clock Select bit 0 for Timer 0
#define CS01    1   // Clock Select bit 1 for Timer 0
#define CS02    2   // Clock Select bit 2 for Timer 0
#define WGM00   0   // Waveform Generation Mode bit 0 for Timer 0
#define WGM01   1   // Waveform Generation Mode bit 1 for Timer 0
#define WGM02   3   // Waveform Generation Mode bit 2 for Timer 0
#define COM0A1  7   // Compare Output Mode bit 1 for Timer 0 Channel A
#define COM0A0  6   // Compare Output Mode bit 0 for Timer 0 Channel A
#define COM0B1  5   // Compare Output Mode bit 1 for Timer 0 Channel B
#define COM0B0  4   // Compare Output Mode bit 0 for Timer 0 Channel B

// For Timer1
#define CS10    0   // Clock Select bit 0 for Timer 1
#define CS11    1   // Clock Select bit 1 for Timer 1
#define CS12    2   // Clock Select bit 2 for Timer 1
#define WGM10   0   // Waveform Generation Mode bit 0 for Timer 1
#define WGM11   1   // Waveform Generation Mode bit 1 for Timer 1
#define WGM12   3   // Waveform Generation Mode bit 2 for Timer 1
#define WGM13   4   // Waveform Generation Mode bit 3 for Timer 1
#define COM1A1  7   // Compare Output Mode bit 1 for Timer 1 Channel A
#define COM1A0  6   // Compare Output Mode bit 0 for Timer 1 Channel A
#define COM1B1  5   // Compare Output Mode bit 1 for Timer 1 Channel B
#define COM1B0  4   // Compare Output Mode bit 0 for Timer 1 Channel B
#define ICNC1   7   // Input Capture Noise Canceler for Timer 1
#define ICES1   6   // Input Capture Edge Select for Timer 1
#define OCIE1A  1   // Output Compare A Match Interrupt Enable for Timer 1
#define OCIE1B  2   // Output Compare B Match Interrupt Enable for Timer 1
#define TOIE1   0   // Timer Overflow Interrupt Enable for Timer 1
#define OCF1A   1   // Output Compare A Match Flag for Timer 1
#define OCF1B   2   // Output Compare B Match Flag for Timer 1
#define TOV1    0   // Timer Overflow Flag for Timer 1

// For Timer2
#define CS20    0   // Clock Select bit 0 for Timer 2
#define CS21    1   // Clock Select bit 1 for Timer 2
#define CS22    2   // Clock Select bit 2 for Timer 2
#define WGM20   0   // Waveform Generation Mode bit 0 for Timer 2
#define WGM21   1   // Waveform Generation Mode bit 1 for Timer 2
#define WGM22   3   // Waveform Generation Mode bit 2 for Timer 2
#define COM2A1  7   // Compare Output Mode bit 1 for Timer 2 Channel A
#define COM2A0  6   // Compare Output Mode bit 0 for Timer 2 Channel A
#define COM2B1  5   // Compare Output Mode bit 1 for Timer 2 Channel B
#define COM2B0  4   // Compare Output Mode bit 0 for Timer 2 Channel B
#define OCIE2A  1   // Output Compare A Match Interrupt Enable for Timer 2
#define OCIE2B  2   // Output Compare B Match Interrupt Enable for Timer 2
#define TOIE2   0   // Timer Overflow Interrupt Enable for Timer 2
#define OCF2A   1   // Output Compare A Match Flag for Timer 2
#define OCF2B   2   // Output Compare B Match Flag for Timer 2
#define TOV2    0   // Timer Overflow Flag for Timer 2
// Function prototypes
void Timer_Init(uint32_t timer_base, uint32_t prescaler, uint32_t auto_reload);
void Timer_Start(uint32_t timer_base);
void Timer_Stop(uint32_t timer_base);
uint32_t Timer_GetCounter(uint32_t timer_base);
void Timer_EnableInterrupt(uint32_t timer_base);
void Timer_DisableInterrupt(uint32_t timer_base);
void Timer_ClearInterruptFlag(uint32_t timer_base);
void Timer_SetAutoReload(uint32_t timer_base, uint32_t auto_reload);

// Helper functions for delays
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

#endif // TIMER_DRIVER_H