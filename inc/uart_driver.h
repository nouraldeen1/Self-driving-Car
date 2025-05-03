// inc/uart_driver.h
#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

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
    uint8_t tx_pin;      // Arduino pin number for TX (should be 1 for Arduino Uno)
    uint8_t rx_pin;      // Arduino pin number for RX (should be 0 for Arduino Uno)
    UART_BaudRate baud_rate;
} UART_Config;

// Function prototypes
void UART_Init(UART_Config* config);
void UART_Transmit(uint8_t data);
void UART_TransmitString(const char* str);
bool UART_IsDataAvailable(void);
uint8_t UART_Receive(void);
void UART_EnableRxInterrupt(void);
void UART_DisableRxInterrupt(void);

#endif // UART_DRIVER_H