#include <stdint.h>
#include <stdbool.h>

// External GPIO functions (would be linked at compile time)
extern void GPIO_ClockEnable(uint32_t gpio_base);
extern void GPIO_PinInit(uint32_t gpio_base, uint8_t pin, int mode);
extern void GPIO_PinSetAlternateFunction(uint32_t gpio_base, uint8_t pin, uint8_t alt_func);

// GPIO pin modes
#define GPIO_ALT_FUNCTION 2

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
#define RCC_BASE 0x40023800
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