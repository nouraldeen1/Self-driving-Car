#include <stdint.h>

// Pin modes
typedef enum {
    GPIO_INPUT = 0,
    GPIO_OUTPUT = 1,
    GPIO_ALT_FUNCTION = 2,
    GPIO_ANALOG = 3
} GPIO_PinMode;

// Pin states
typedef enum {
    GPIO_LOW = 0,
    GPIO_HIGH = 1
} GPIO_PinState;

// GPIO port definitions
#define GPIOA_BASE 0x40020000
#define GPIOB_BASE 0x40020400
#define GPIOC_BASE 0x40020800
#define GPIOD_BASE 0x40020C00
#define GPIOE_BASE 0x40021000

// GPIO register offsets
#define GPIO_MODER_OFFSET   0x00
#define GPIO_OTYPER_OFFSET  0x04
#define GPIO_OSPEEDR_OFFSET 0x08
#define GPIO_PUPDR_OFFSET   0x0C
#define GPIO_IDR_OFFSET     0x10
#define GPIO_ODR_OFFSET     0x14
#define GPIO_BSRR_OFFSET    0x18
#define GPIO_AFRL_OFFSET    0x20
#define GPIO_AFRH_OFFSET    0x24

// RCC register for enabling GPIO clock
#define RCC_BASE           0x40023800
#define RCC_AHB1ENR_OFFSET 0x30

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