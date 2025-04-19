// uart_driver.h
#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "gpio_driver.h"

// USART register definitions
#define USART1_BASE 0x40011000
#define USART2_BASE 0x40004400
#define USART6_BASE 0x40011400

// USART register offsets
#define USART_SR_OFFSET   0x00
#define USART_DR_OFFSET   0x04
#define USART_BRR_OFFSET  0x08
#define USART_CR1_OFFSET  0x0C
#define USART_CR2_OFFSET  0x10
#define USART_CR3_OFFSET  0x14
#define USART_GTPR_OFFSET 0x18

// USART status register bits
#define USART_SR_TXE  (1 << 7)
#define USART_SR_RXNE (1 << 5)

// USART control register 1 bits
#define USART_CR1_UE    (1 << 13) // USART enable
#define USART_CR1_TE    (1 << 3)  // Transmitter enable
#define USART_CR1_RE    (1 << 2)  // Receiver enable
#define USART_CR1_RXNEIE (1 << 5) // RXNE interrupt enable

// RCC register for enabling USART clocks
#define RCC_APB2ENR_OFFSET 0x44
#define RCC_APB1ENR_OFFSET 0x40

// Baud rates
typedef enum {
    UART_BAUD_9600 = 9600,
    UART_BAUD_19200 = 19200,
    UART_BAUD_38400 = 38400,
    UART_BAUD_57600 = 57600,
    UART_BAUD_115200 = 115200
} UART_BaudRate;

// UART configuration structure
typedef struct {
    uint32_t uart_base;
    uint32_t tx_port;
    uint8_t tx_pin;
    uint32_t rx_port;
    uint8_t rx_pin;
    uint8_t alt_func;
    UART_BaudRate baud_rate;
} UART_Config;

// Function prototypes
void UART_Init(UART_Config* config);
void UART_Transmit(uint32_t uart_base, uint8_t data);
void UART_TransmitString(uint32_t uart_base, const char* str);
bool UART_IsDataAvailable(uint32_t uart_base);
uint8_t UART_Receive(uint32_t uart_base);
void UART_EnableRxInterrupt(uint32_t uart_base);
void UART_DisableRxInterrupt(uint32_t uart_base);

#endif // UART_DRIVER_H