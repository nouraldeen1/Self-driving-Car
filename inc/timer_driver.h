// inc/timer_driver.h
#ifndef TIMER_DRIVER_H
#define TIMER_DRIVER_H

#include <stdint.h>

// Timer base addresses (arbitrary values for API compatibility)
#define TIMER0_BASE 0x00
#define TIMER1_BASE 0x01
#define TIMER2_BASE 0x02

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