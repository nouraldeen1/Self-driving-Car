// uart_driver.c
#include "uart_driver.h"

void UART_Init(UART_Config* config) {
    // Enable GPIO clocks
    GPIO_ClockEnable(config->tx_port);
    GPIO_ClockEnable(config->rx_port);
    
    // Configure TX and RX pins as alternate function
    GPIO_PinInit(config->tx_port, config->tx_pin, GPIO_ALT_FUNCTION);
    GPIO_PinInit(config->rx_port, config->rx_pin, GPIO_ALT_FUNCTION);
    
    // Set alternate function for UART pins
    GPIO_PinSetAlternateFunction(config->tx_port, config->tx_pin, config->alt_func);
    GPIO_PinSetAlternateFunction(config->rx_port, config->rx_pin, config->alt_func);
    
    // Enable UART clock
    if (config->uart_base == USART1_BASE || config->uart_base == USART6_BASE) {
        // USART1 and USART6 are on APB2
        uint32_t rcc_apb2enr = *(volatile uint32_t*)(RCC_BASE + RCC_APB2ENR_OFFSET);
        if (config->uart_base == USART1_BASE) {
            rcc_apb2enr |= (1 << 4); // USART1 enable
        } else {
            rcc_apb2enr |= (1 << 5); // USART6 enable
        }
        *(volatile uint32_t*)(RCC_BASE + RCC_APB2ENR_OFFSET) = rcc_apb2enr;
    } else if (config->uart_base == USART2_BASE) {
        // USART2 is on APB1
        uint32_t rcc_apb1enr = *(volatile uint32_t*)(RCC_BASE + RCC_APB1ENR_OFFSET);
        rcc_apb1enr |= (1 << 17); // USART2 enable
        *(volatile uint32_t*)(RCC_BASE + RCC_APB1ENR_OFFSET) = rcc_apb1enr;
    }
    
    // Calculate BRR value (assuming 16MHz system clock)
    uint32_t system_clock = 16000000; // 16MHz
    uint32_t usart_div = system_clock / config->baud_rate;
    
    // Set baud rate
    *(volatile uint32_t*)(config->uart_base + USART_BRR_OFFSET) = usart_div;
    
    // Enable UART, transmitter, and receiver
    *(volatile uint32_t*)(config->uart_base + USART_CR1_OFFSET) = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

void UART_Transmit(uint32_t uart_base, uint8_t data) {
    // Wait until transmit buffer is empty
    while (!(*(volatile uint32_t*)(uart_base + USART_SR_OFFSET) & USART_SR_TXE));
    
    // Write data to data register
    *(volatile uint32_t*)(uart_base + USART_DR_OFFSET) = data;
}

void UART_TransmitString(uint32_t uart_base, const char* str) {
    while (*str) {
        UART_Transmit(uart_base, *str++);
    }
}

bool UART_IsDataAvailable(uint32_t uart_base) {
    return (*(volatile uint32_t*)(uart_base + USART_SR_OFFSET) & USART_SR_RXNE) != 0;
}

uint8_t UART_Receive(uint32_t uart_base) {
    // Wait until data is received
    while (!UART_IsDataAvailable(uart_base));
    
    // Return received data
    return (uint8_t)(*(volatile uint32_t*)(uart_base + USART_DR_OFFSET) & 0xFF);
}

void UART_EnableRxInterrupt(uint32_t uart_base) {
    // Enable RXNE interrupt
    *(volatile uint32_t*)(uart_base + USART_CR1_OFFSET) |= USART_CR1_RXNEIE;
}

void UART_DisableRxInterrupt(uint32_t uart_base) {
    // Disable RXNE interrupt
    *(volatile uint32_t*)(uart_base + USART_CR1_OFFSET) &= ~USART_CR1_RXNEIE;
}