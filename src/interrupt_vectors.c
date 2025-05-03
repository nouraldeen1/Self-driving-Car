// interrupt_vectors.c
#include "../inc/bluetooth_driver.h"
#include "../inc/avr_registers.h"

// External variables
extern BluetoothModule bluetooth;

// USART RX interrupt vector
void __vector_18(void) __attribute__((signal, used, externally_visible));
void __vector_18(void) {
    if (UART_IsDataAvailable()) {
        uint8_t data = UART_Receive();
        Bluetooth_ProcessReceivedData(&bluetooth, data);
    }
}