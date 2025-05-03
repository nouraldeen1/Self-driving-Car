// gpio_driver.c
#include "../inc/gpio_driver.h"

void GPIO_ClockEnable(uint32_t gpio_base) {
    uint32_t rcc_ahb1enr = *(volatile uint32_t*)(RCC_BASE + RCC_AHB1ENR_OFFSET);
    
    // Determine which GPIO port to enable
    if (gpio_base == GPIOA_BASE) {
        rcc_ahb1enr |= (1 << 0); // GPIOA enable
    } else if (gpio_base == GPIOB_BASE) {
        rcc_ahb1enr |= (1 << 1); // GPIOB enable
    } else if (gpio_base == GPIOC_BASE) {
        rcc_ahb1enr |= (1 << 2); // GPIOC enable
    } else if (gpio_base == GPIOD_BASE) {
        rcc_ahb1enr |= (1 << 3); // GPIOD enable
    } else if (gpio_base == GPIOE_BASE) {
        rcc_ahb1enr |= (1 << 4); // GPIOE enable
    }
    
    // Write back to enable the clock
    *(volatile uint32_t*)(RCC_BASE + RCC_AHB1ENR_OFFSET) = rcc_ahb1enr;
}

void GPIO_PinInit(uint32_t gpio_base, uint8_t pin, GPIO_PinMode mode) {
    // Configure pin mode (input, output, etc)
    volatile uint32_t* moder = (volatile uint32_t*)(gpio_base + GPIO_MODER_OFFSET);
    *moder &= ~(3UL << (pin * 2)); // Clear the 2 bits for this pin
    *moder |= (mode << (pin * 2)); // Set mode
}

void GPIO_PinWrite(uint32_t gpio_base, uint8_t pin, GPIO_PinState state) {
    if (state == GPIO_HIGH) {
        // Set pin using BSRR (Bit Set/Reset Register)
        *(volatile uint32_t*)(gpio_base + GPIO_BSRR_OFFSET) = (1UL << pin);
    } else {
        // Reset pin using BSRR
        *(volatile uint32_t*)(gpio_base + GPIO_BSRR_OFFSET) = (1UL << (pin + 16));
    }
}

GPIO_PinState GPIO_PinRead(uint32_t gpio_base, uint8_t pin) {
    uint32_t idr = *(volatile uint32_t*)(gpio_base + GPIO_IDR_OFFSET);
    return (idr & (1UL << pin)) ? GPIO_HIGH : GPIO_LOW;
}

void GPIO_PinSetAlternateFunction(uint32_t gpio_base, uint8_t pin, uint8_t alt_func) {
    volatile uint32_t* afr;
    
    if (pin < 8) {
        afr = (volatile uint32_t*)(gpio_base + GPIO_AFRL_OFFSET);
    } else {
        afr = (volatile uint32_t*)(gpio_base + GPIO_AFRH_OFFSET);
        pin -= 8;
    }
    
    // Clear the 4 bits for this pin
    *afr &= ~(0xFUL << (pin * 4));
    
    // Set the alternate function
    *afr |= (alt_func << (pin * 4));
}