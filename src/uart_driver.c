// uart_driver.c
#include "../inc/uart_driver.h"
#include "../inc/avr_registers.h"

void UART_Init(UART_Config* config) {
    // Calculate baud rate register value
    uint16_t ubrr = F_CPU / 16 / config->baud_rate - 1;
    
    // Set baud rate
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)(ubrr & 0xFF);
    
    // Enable receiver and transmitter
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    
    // Set frame format: 8 data bits, 1 stop bit, no parity
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void UART_Transmit(uint8_t data) {
    // Wait for empty transmit buffer
    while (!(UCSR0A & (1 << UDRE0)));
    
    // Put data into buffer, sends the data
    UDR0 = data;
}

void UART_TransmitString(const char* str) {
    while (*str) {
        UART_Transmit(*str++);
    }
}

bool UART_IsDataAvailable(void) {
    return (UCSR0A & (1 << RXC0)) != 0;
}

uint8_t UART_Receive(void) {
    // Wait for data to be received
    while (!UART_IsDataAvailable());
    
    // Get and return received data from buffer
    return UDR0;
}

void UART_EnableRxInterrupt(void) {
    // Enable UART RX Complete Interrupt
    UCSR0B |= (1 << RXCIE0);
}

void UART_DisableRxInterrupt(void) {
    // Disable UART RX Complete Interrupt
    UCSR0B &= ~(1 << RXCIE0);
}