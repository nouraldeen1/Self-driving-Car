// avr_registers.h
#ifndef AVR_REGISTERS_H
#define AVR_REGISTERS_H

#include <stdint.h>

// Define all AVR registers used in project

// Timer Registers
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

// UART Registers
#define UDR0    (*(volatile uint8_t*)0xC6)
#define UBRR0H  (*(volatile uint8_t*)0xC5)
#define UBRR0L  (*(volatile uint8_t*)0xC4)
#define UCSR0A  (*(volatile uint8_t*)0xC0)
#define UCSR0B  (*(volatile uint8_t*)0xC1)
#define UCSR0C  (*(volatile uint8_t*)0xC2)

// Register bit definitions - Timer bits
#define CS00    0
#define CS01    1
#define CS02    2
#define WGM00   0
#define WGM01   1
#define WGM02   3
#define WGM10   0
#define WGM11   1
#define WGM12   3
#define WGM13   4
#define COM0A1  7
#define COM0B1  5
#define COM1A1  7
#define COM1B1  5
#define COM2A1  7
#define COM2B1  5
#define OCIE1A  1
#define OCF1A   1
#define CS10    0
#define CS11    1
#define CS12    2
#define CS20    0
#define CS21    1
#define CS22    2

// UART bits
#define RXEN0   4
#define TXEN0   3
#define RXCIE0  7
#define RXC0    7
#define UDRE0   5
#define UCSZ00  1
#define UCSZ01  2

// System clock frequency - standard for Arduino Uno
//#define F_CPU 16000000UL

#endif // AVR_REGISTERS_H