// main.c
#include "../inc/main.h"
#include "../inc/gpio_driver.h"
#include "../inc/ultrasonic_driver.h"
#include "../inc/motor_driver.h"
#include "../inc/bluetooth_driver.h"
#include "../inc/parking_logic.h"
#include <../inc/pwm_driver.h>

// Global variables for car hardware
Motor leftMotor, rightMotor;
UltrasonicSensor frontSensor, leftSensor, rightSensor;
BluetoothModule bluetooth;

// Timer for general purpose delays
#define DELAY_TIMER_BASE TIM5_BASE

void SystemInit(void) {
    // Enable HSI (High-Speed Internal) oscillator - 16 MHz
    // RCC_CR register: enable HSI
    *(volatile uint32_t*)(RCC_BASE + 0x00) |= (1 << 0);
    
    // Wait for HSI to be ready
    while (!(*(volatile uint32_t*)(RCC_BASE + 0x00) & (1 << 1)));
    
    // Set HSI as system clock source
    // RCC_CFGR register: set system clock source
    *(volatile uint32_t*)(RCC_BASE + 0x08) &= ~0x03; // Clear bits
    *(volatile uint32_t*)(RCC_BASE + 0x08) |= 0x00; // HSI as system clock
    
    // Wait for HSI to be used as system clock
    while ((*(volatile uint32_t*)(RCC_BASE + 0x08) & 0x0C) != 0x00);
    
    // Initialize delay timer (TIM5)
    Timer_Init(DELAY_TIMER_BASE, 16-1, 0xFFFFFFFF);
}

void delay_us(uint32_t us) {
    // Reset counter
    *(volatile uint32_t*)(DELAY_TIMER_BASE + TIM_CNT_OFFSET) = 0;
    
    // Start timer
    *(volatile uint32_t*)(DELAY_TIMER_BASE + TIM_CR1_OFFSET) |= 0x01;
    
    // Wait until counter reaches desired value
    // Each timer tick is 1us (with 16MHz clock and prescaler=16)
    while (*(volatile uint32_t*)(DELAY_TIMER_BASE + TIM_CNT_OFFSET) < us);
    
    // Stop timer
    *(volatile uint32_t*)(DELAY_TIMER_BASE + TIM_CR1_OFFSET) &= ~0x01;
}

void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        delay_us(1000);
    }
}

void SetStatusLED(GPIO_PinState state) {
    GPIO_PinWrite(LED_PORT, LED_PIN, state);
}

void BlinkStatusLED(uint8_t times) {
    for (uint8_t i = 0; i < times; i++) {
        SetStatusLED(GPIO_HIGH);
        delay_ms(200);
        SetStatusLED(GPIO_LOW);
        delay_ms(200);
    }
}

void HardwareInit(void) {
    // Initialize GPIO for status LED
    GPIO_ClockEnable(LED_PORT);
    GPIO_PinInit(LED_PORT, LED_PIN, GPIO_OUTPUT);
    SetStatusLED(GPIO_LOW);
    
    // Configure left motor
    leftMotor.in1_port = GPIOB_BASE;
    leftMotor.in1_pin = 0;
    leftMotor.in2_port = GPIOB_BASE;
    leftMotor.in2_pin = 1;
    leftMotor.ena_port = GPIOB_BASE;
    leftMotor.ena_pin = 4;
    leftMotor.pwm_channel = 1;
    Motor_Init(&leftMotor);
    
    // Configure right motor
    rightMotor.in1_port = GPIOB_BASE;
    rightMotor.in1_pin = 2;
    rightMotor.in2_port = GPIOB_BASE;
    rightMotor.in2_pin = 3;
    rightMotor.ena_port = GPIOB_BASE;
    rightMotor.ena_pin = 5;
    rightMotor.pwm_channel = 2;
    Motor_Init(&rightMotor);
    
    // Configure sensors
    frontSensor.trig_port = GPIOA_BASE;
    frontSensor.trig_pin = 0;
    frontSensor.echo_port = GPIOA_BASE;
    frontSensor.echo_pin = 1;
    Ultrasonic_Init(&frontSensor);
    
    leftSensor.trig_port = GPIOA_BASE;
    leftSensor.trig_pin = 2;
    leftSensor.echo_port = GPIOA_BASE;
    leftSensor.echo_pin = 3;
    Ultrasonic_Init(&leftSensor);
    
    rightSensor.trig_port = GPIOA_BASE;
    rightSensor.trig_pin = 4;
    rightSensor.echo_port = GPIOA_BASE;
    rightSensor.echo_pin = 5;
    Ultrasonic_Init(&rightSensor);
    
    // Initialize Bluetooth
    bluetooth.uart.uart_base = USART2_BASE;
    bluetooth.uart.tx_port = GPIOA_BASE;
    bluetooth.uart.tx_pin = 2;
    bluetooth.uart.rx_port = GPIOA_BASE;
    bluetooth.uart.rx_pin = 3;
    bluetooth.uart.alt_func = 7; // AF7 for USART2 on GPIOA
    bluetooth.uart.baud_rate = UART_BAUD_9600;
    Bluetooth_Init(&bluetooth);
    
    // Blink LED to indicate initialization complete
    BlinkStatusLED(3);
}

// USART2 IRQ Handler
void USART2_IRQHandler(void) {
    if (UART_IsDataAvailable(bluetooth.uart.uart_base)) {
        uint8_t data = UART_Receive(bluetooth.uart.uart_base);
        Bluetooth_ProcessReceivedData(&bluetooth, data);
    }
}

int main(void) {
    // Initialize system and hardware
    SystemInit();
    HardwareInit();
    
    // Send startup message
    Bluetooth_SendMessage(&bluetooth, "Self-Parking Car Ready");
    
    // Main loop
    while (1) {
        // Check for Bluetooth commands
        if (Bluetooth_IsCommandReceived(&bluetooth)) {
            Command cmd = Bluetooth_GetCommand(&bluetooth);
            
            switch (cmd) {
                case COMMAND_START_PARKING:
                    // Blink LED to indicate parking process starts
                    BlinkStatusLED(2);
                    
                    // Start the parking sequence
                    ParkingSlot slot;
                    
                    // Move forward and detect parking space
                    Bluetooth_SendMessage(&bluetooth, "Scanning for parking space...");
                    if (DetectParkingSpace(&frontSensor, &leftSensor, &rightSensor, &slot)) {
                        // Execute parking maneuver based on detected slot
                        if (slot.type == PARALLEL_PARKING) {
                            Bluetooth_SendMessage(&bluetooth, "Parallel parking slot detected");
                            ExecuteParallelParking(&leftMotor, &rightMotor, 
                                                &frontSensor, &leftSensor, &rightSensor);
                        } else {
                            Bluetooth_SendMessage(&bluetooth, "Perpendicular parking slot detected");
                            ExecutePerpendicularParking(&leftMotor, &rightMotor,
                                                    &frontSensor, &leftSensor, &rightSensor);
                        }
                        
                        // Send completion status via Bluetooth
                        Bluetooth_SendMessage(&bluetooth, "Parking completed");
                        BlinkStatusLED(4); // Signal successful parking
                    } else {
                        // No suitable parking space found
                        Bluetooth_SendMessage(&bluetooth, "No parking space found");
                        BlinkStatusLED(1); // Signal failure
                    }
                    break;
                    
                case COMMAND_STOP:
                    // Emergency stop
                    Motor_SetDirection(&leftMotor, MOTOR_STOP);
                    Motor_SetDirection(&rightMotor, MOTOR_STOP);
                    Bluetooth_SendMessage(&bluetooth, "Emergency stop");
                    break;
                    
                case COMMAND_MOVE_FORWARD:
                    // Manual forward control
                    Motor_SetDirection(&leftMotor, MOTOR_FORWARD);
                    Motor_SetDirection(&rightMotor, MOTOR_FORWARD);
                    Motor_SetSpeed(&leftMotor, 50);
                    Motor_SetSpeed(&rightMotor, 50);
                    Bluetooth_SendMessage(&bluetooth, "Moving forward");
                    break;
                    
                case COMMAND_MOVE_BACKWARD:
                    // Manual backward control
                    Motor_SetDirection(&leftMotor, MOTOR_BACKWARD);
                    Motor_SetDirection(&rightMotor, MOTOR_BACKWARD);
                    Motor_SetSpeed(&leftMotor, 50);
                    Motor_SetSpeed(&rightMotor, 50);
                    Bluetooth_SendMessage(&bluetooth, "Moving backward");
                    break;
                    
                case COMMAND_TURN_LEFT:
                    // Manual left turn
                    Motor_SetDirection(&leftMotor, MOTOR_BACKWARD);
                    Motor_SetDirection(&rightMotor, MOTOR_FORWARD);
                    Motor_SetSpeed(&leftMotor, 40);
                    Motor_SetSpeed(&rightMotor, 40);
                    Bluetooth_SendMessage(&bluetooth, "Turning left");
                    break;
                    
                case COMMAND_TURN_RIGHT:
                    // Manual right turn
                    Motor_SetDirection(&leftMotor, MOTOR_FORWARD);
                    Motor_SetDirection(&rightMotor, MOTOR_BACKWARD);
                    Motor_SetSpeed(&leftMotor, 40);
                    Motor_SetSpeed(&rightMotor, 40);
                    Bluetooth_SendMessage(&bluetooth, "Turning right");
                    break;
                    
                default:
                    break;
            }
        }
        
        // Obstacle detection - safety feature
        if (Ultrasonic_MeasureDistance(&frontSensor) < 10) {
            // Emergency stop if obstacle is too close
            Motor_SetDirection(&leftMotor, MOTOR_STOP);
            Motor_SetDirection(&rightMotor, MOTOR_STOP);
            // Add notification about emergency stop
            Bluetooth_SendMessage(&bluetooth, "Emergency stop: Obstacle detected");
            // Visual indicator
            SetStatusLED(GPIO_HIGH);
            delay_ms(500);
            SetStatusLED(GPIO_LOW);
        }
    }
}