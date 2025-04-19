// timer_driver.h
#ifndef TIMER_DRIVER_H
#define TIMER_DRIVER_H

#include <stdint.h>

// Timer base addresses
#define TIM2_BASE 0x40000000
#define TIM3_BASE 0x40000400
#define TIM4_BASE 0x40000800
#define TIM5_BASE 0x40000C00

// Timer register offsets
#define TIM_CR1_OFFSET   0x00
#define TIM_CR2_OFFSET   0x04
#define TIM_DIER_OFFSET  0x0C
#define TIM_SR_OFFSET    0x10
#define TIM_EGR_OFFSET   0x14
#define TIM_CNT_OFFSET   0x24
#define TIM_PSC_OFFSET   0x28
#define TIM_ARR_OFFSET   0x2C

// RCC offset for APB1 peripheral clock enable register
#define RCC_APB1ENR_OFFSET 0x40

// Function prototypes
void Timer_Init(uint32_t timer_base, uint32_t prescaler, uint32_t auto_reload);
void Timer_Start(uint32_t timer_base);
void Timer_Stop(uint32_t timer_base);
uint32_t Timer_GetCounter(uint32_t timer_base);
void Timer_EnableInterrupt(uint32_t timer_base);
void Timer_DisableInterrupt(uint32_t timer_base);
void Timer_ClearInterruptFlag(uint32_t timer_base);
void Timer_SetAutoReload(uint32_t timer_base, uint32_t auto_reload);

#endif // TIMER_DRIVER_H