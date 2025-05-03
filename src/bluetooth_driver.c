#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// External UART functions (would be linked at compile time)
extern void UART_Init(void* config);
extern void UART_TransmitString(uint32_t uart_base, const char* str);
extern bool UART_IsDataAvailable(uint32_t uart_base);
extern uint8_t UART_Receive(uint32_t uart_base);
extern void UART_EnableRxInterrupt(uint32_t uart_base);

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

// Maximum command length
#define BT_MAX_CMD_LENGTH 20

// UART configuration structure (must match the one in uart_driver.c)
typedef struct {
    uint32_t uart_base;
    uint32_t tx_port;
    uint8_t tx_pin;
    uint32_t rx_port;
    uint8_t rx_pin;
    uint8_t alt_func;
    int baud_rate;
} UART_Config;

// Bluetooth module structure
typedef struct {
    UART_Config uart;
    char command_buffer[BT_MAX_CMD_LENGTH];
    uint8_t buffer_index;
    bool command_ready;
    Command last_command;
} BluetoothModule;

// Command strings
const char* COMMAND_START_PARKING_STR = "START";
const char* COMMAND_STOP_STR = "STOP";
const char* COMMAND_MOVE_FORWARD_STR = "FWD";
const char* COMMAND_MOVE_BACKWARD_STR = "BWD";
const char* COMMAND_TURN_LEFT_STR = "LEFT";
const char* COMMAND_TURN_RIGHT_STR = "RIGHT";

void Bluetooth_Init(BluetoothModule* bt) {
    // Initialize UART for Bluetooth module
    UART_Init(&bt->uart);
    
    // Initialize command buffer
    memset(bt->command_buffer, 0, BT_MAX_CMD_LENGTH);
    bt->buffer_index = 0;
    bt->command_ready = false;
    bt->last_command = COMMAND_NONE;
    
    // Enable UART receive interrupt
    UART_EnableRxInterrupt(bt->uart.uart_base);
}

bool Bluetooth_IsCommandReceived(BluetoothModule* bt) {
    // Check if a complete command is available
    return bt->command_ready;
}

Command Bluetooth_GetCommand(BluetoothModule* bt) {
    if (bt->command_ready) {
        Command cmd = bt->last_command;
        bt->command_ready = false;
        return cmd;
    }
    return COMMAND_NONE;
}

void Bluetooth_SendMessage(BluetoothModule* bt, const char* message) {
    // Send message through UART
    UART_TransmitString(bt->uart.uart_base, message);
    UART_TransmitString(bt->uart.uart_base, "\r\n"); // Add line ending
}

void Bluetooth_ProcessReceivedData(BluetoothModule* bt, uint8_t data) {
    // Process received byte
    if (data == '\n' || data == '\r') {
        // End of command
        bt->command_buffer[bt->buffer_index] = '\0'; // Null terminate
        
        // Parse the command
        if (strncmp(bt->command_buffer, COMMAND_START_PARKING_STR, strlen(COMMAND_START_PARKING_STR)) == 0) {
            bt->last_command = COMMAND_START_PARKING;
        } else if (strncmp(bt->command_buffer, COMMAND_STOP_STR, strlen(COMMAND_STOP_STR)) == 0) {
            bt->last_command = COMMAND_STOP;
        } else if (strncmp(bt->command_buffer, COMMAND_MOVE_FORWARD_STR, strlen(COMMAND_MOVE_FORWARD_STR)) == 0) {
            bt->last_command = COMMAND_MOVE_FORWARD;
        } else if (strncmp(bt->command_buffer, COMMAND_MOVE_BACKWARD_STR, strlen(COMMAND_MOVE_BACKWARD_STR)) == 0) {
            bt->last_command = COMMAND_MOVE_BACKWARD;
        } else if (strncmp(bt->command_buffer, COMMAND_TURN_LEFT_STR, strlen(COMMAND_TURN_LEFT_STR)) == 0) {
            bt->last_command = COMMAND_TURN_LEFT;
        } else if (strncmp(bt->command_buffer, COMMAND_TURN_RIGHT_STR, strlen(COMMAND_TURN_RIGHT_STR)) == 0) {
            bt->last_command = COMMAND_TURN_RIGHT;
        } else {
            bt->last_command = COMMAND_NONE;
        }
        
        // Reset buffer
        memset(bt->command_buffer, 0, BT_MAX_CMD_LENGTH);
        bt->buffer_index = 0;
        
        // Set flag to indicate command is ready
        if (bt->last_command != COMMAND_NONE) {
            bt->command_ready = true;
        }
    } else if (bt->buffer_index < BT_MAX_CMD_LENGTH - 1) {
        // Add character to buffer
        bt->command_buffer[bt->buffer_index++] = data;
    }
}

// Function to be called from the USART IRQ handler
void USART_IRQHandler(BluetoothModule* bt) {
    if (UART_IsDataAvailable(bt->uart.uart_base)) {
        uint8_t data = UART_Receive(bt->uart.uart_base);
        Bluetooth_ProcessReceivedData(bt, data);
    }
}