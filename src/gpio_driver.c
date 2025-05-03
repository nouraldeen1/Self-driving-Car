// src/gpio_driver.c
#include "../inc/gpio_driver.h"

// ATmega328P Port Registers
#define PORTB (*(volatile uint8_t*)(0x25))
#define DDRB  (*(volatile uint8_t*)(0x24))
#define PINB  (*(volatile uint8_t*)(0x23))

#define PORTC (*(volatile uint8_t*)(0x28))
#define DDRC  (*(volatile uint8_t*)(0x27))
#define PINC  (*(volatile uint8_t*)(0x26))

#define PORTD (*(volatile uint8_t*)(0x2B))
#define DDRD  (*(volatile uint8_t*)(0x2A))
#define PIND  (*(volatile uint8_t*)(0x29))

// Convert Arduino pin to port and bit
void PinToPortBit(uint8_t pin, volatile uint8_t** port, volatile uint8_t** ddr, volatile uint8_t** pin_reg, uint8_t* bit) {
    if (pin <= 7) {
        // Pins 0-7 map to PORTD
        *port = &PORTD;
        *ddr = &DDRD;
        *pin_reg = &PIND;
        *bit = pin;
    } 
    else if (pin >= 8 && pin <= 13) {
        // Pins 8-13 map to PORTB
        *port = &PORTB;
        *ddr = &DDRB;
        *pin_reg = &PINB;
        *bit = pin - 8;
    }
    else if (pin >= 14 && pin <= 19) {
        // Pins 14-19 (A0-A5) map to PORTC
        *port = &PORTC;
        *ddr = &DDRC;
        *pin_reg = &PINC;
        *bit = pin - 14;
    }
}

void GPIO_Init(uint8_t pin, GPIO_PinMode mode) {
    volatile uint8_t* port;
    volatile uint8_t* ddr;
    volatile uint8_t* pin_reg;
    uint8_t bit;
    
    PinToPortBit(pin, &port, &ddr, &pin_reg, &bit);
    
    switch(mode) {
        case GPIO_INPUT:
            // Clear bit in Data Direction Register (input)
            *ddr &= ~(1 << bit);
            // Clear bit in PORT (disable pull-up)
            *port &= ~(1 << bit);
            break;
        case GPIO_OUTPUT:
            // Set bit in Data Direction Register (output)
            *ddr |= (1 << bit);
            break;
        case GPIO_INPUT_PULLUP:
            // Clear bit in Data Direction Register (input)
            *ddr &= ~(1 << bit);
            // Set bit in PORT (enable pull-up)
            *port |= (1 << bit);
            break;
    }
}

void GPIO_Write(uint8_t pin, GPIO_PinState state) {
    volatile uint8_t* port;
    volatile uint8_t* ddr;
    volatile uint8_t* pin_reg;
    uint8_t bit;
    
    PinToPortBit(pin, &port, &ddr, &pin_reg, &bit);
    
    if (state == GPIO_HIGH) {
        *port |= (1 << bit);  // Set bit to HIGH
    } else {
        *port &= ~(1 << bit); // Set bit to LOW
    }
}

GPIO_PinState GPIO_Read(uint8_t pin) {
    volatile uint8_t* port;
    volatile uint8_t* ddr;
    volatile uint8_t* pin_reg;
    uint8_t bit;
    
    PinToPortBit(pin, &port, &ddr, &pin_reg, &bit);
    
    return (*pin_reg & (1 << bit)) ? GPIO_HIGH : GPIO_LOW;
}