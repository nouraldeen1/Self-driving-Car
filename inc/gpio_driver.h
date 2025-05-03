// inc/gpio_driver.h
#ifndef GPIO_DRIVER_H
#define GPIO_DRIVER_H

#include <stdint.h>

// Pin modes
typedef enum {
    GPIO_INPUT = 0,
    GPIO_OUTPUT = 1,
    GPIO_INPUT_PULLUP = 2
} GPIO_PinMode;

// Pin states
typedef enum {
    GPIO_LOW = 0,
    GPIO_HIGH = 1
} GPIO_PinState;

// Pin mapping constants - use Arduino pin numbers
// Arduino pin numbers are used as identifiers
// The driver internally maps these to AVR ports and pins

// Function prototypes
void GPIO_Init(uint8_t pin, GPIO_PinMode mode);
void GPIO_Write(uint8_t pin, GPIO_PinState state);
GPIO_PinState GPIO_Read(uint8_t pin);

#endif // GPIO_DRIVER_H