// inc/bluetooth_driver.h
#ifndef BLUETOOTH_DRIVER_H
#define BLUETOOTH_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "uart_driver.h"

// Maximum command length
#define BT_MAX_CMD_LENGTH 20

// Command definitions
typedef enum {
    COMMAND_NONE,
    COMMAND_START_PARKING,
    COMMAND_STOP,
    COMMAND_MOVE_FORWARD,
    COMMAND_MOVE_BACKWARD,
    COMMAND_TURN_LEFT,
    COMMAND_TURN_RIGHT
} Command;

// Bluetooth module structure
typedef struct {
    UART_Config uart;
    char command_buffer[BT_MAX_CMD_LENGTH];
    uint8_t buffer_index;
    bool command_ready;
    Command last_command;
} BluetoothModule;

// Function prototypes
void Bluetooth_Init(BluetoothModule* bt);
bool Bluetooth_IsCommandReceived(BluetoothModule* bt);
Command Bluetooth_GetCommand(BluetoothModule* bt);
void Bluetooth_SendMessage(BluetoothModule* bt, const char* message);
void Bluetooth_ProcessReceivedData(BluetoothModule* bt, uint8_t data);

// Define USART RX ISR handler
// This doesn't implement the ISR, just declares it will be implemented elsewhere
void USART_RX_vect_handler(void);

#endif // BLUETOOTH_DRIVER_H