// gpio_driver.h
#ifndef GPIO_DRIVER_H
#define GPIO_DRIVER_H

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

// Function prototypes
void GPIO_ClockEnable(uint32_t gpio_base);
void GPIO_PinInit(uint32_t gpio_base, uint8_t pin, GPIO_PinMode mode);
void GPIO_PinWrite(uint32_t gpio_base, uint8_t pin, GPIO_PinState state);
GPIO_PinState GPIO_PinRead(uint32_t gpio_base, uint8_t pin);
void GPIO_PinSetAlternateFunction(uint32_t gpio_base, uint8_t pin, uint8_t alt_func);

#endif // GPIO_DRIVER_H