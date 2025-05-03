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

// RCC register base address
#define RCC_BASE           0x40023800
#define RCC_APB1ENR_OFFSET 0x40

void Timer_Init(uint32_t timer_base, uint32_t prescaler, uint32_t auto_reload) {
    // Enable timer clock
    uint32_t rcc_apb1enr = *(volatile uint32_t*)(RCC_BASE + RCC_APB1ENR_OFFSET);
    
    if (timer_base == TIM2_BASE) {
        rcc_apb1enr |= (1 << 0); // TIM2 enable
    } else if (timer_base == TIM3_BASE) {
        rcc_apb1enr |= (1 << 1); // TIM3 enable
    } else if (timer_base == TIM4_BASE) {
        rcc_apb1enr |= (1 << 2); // TIM4 enable
    } else if (timer_base == TIM5_BASE) {
        rcc_apb1enr |= (1 << 3); // TIM5 enable
    }
    
    *(volatile uint32_t*)(RCC_BASE + RCC_APB1ENR_OFFSET) = rcc_apb1enr;
    
    // Configure timer
    *(volatile uint32_t*)(timer_base + TIM_PSC_OFFSET) = prescaler;
    *(volatile uint32_t*)(timer_base + TIM_ARR_OFFSET) = auto_reload;
    
    // Generate an update event to load the registers
    *(volatile uint32_t*)(timer_base + TIM_EGR_OFFSET) = 0x01;
}

void Timer_Start(uint32_t timer_base) {
    // Reset counter
    *(volatile uint32_t*)(timer_base + TIM_CNT_OFFSET) = 0;
    
    // Enable timer
    *(volatile uint32_t*)(timer_base + TIM_CR1_OFFSET) |= 0x01;
}

void Timer_Stop(uint32_t timer_base) {
    // Disable timer
    *(volatile uint32_t*)(timer_base + TIM_CR1_OFFSET) &= ~0x01;
}

uint32_t Timer_GetCounter(uint32_t timer_base) {
    return *(volatile uint32_t*)(timer_base + TIM_CNT_OFFSET);
}

void Timer_EnableInterrupt(uint32_t timer_base) {
    // Enable update interrupt
    *(volatile uint32_t*)(timer_base + TIM_DIER_OFFSET) |= 0x01;
}

void Timer_DisableInterrupt(uint32_t timer_base) {
    // Disable update interrupt
    *(volatile uint32_t*)(timer_base + TIM_DIER_OFFSET) &= ~0x01;
}

void Timer_ClearInterruptFlag(uint32_t timer_base) {
    // Clear update interrupt flag
    *(volatile uint32_t*)(timer_base + TIM_SR_OFFSET) &= ~0x01;
}