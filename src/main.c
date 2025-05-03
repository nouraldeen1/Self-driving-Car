// main.c
#include <avr/io.h>
#include <avr/interrupt.h>
#include "../inc/gpio_driver.h"
#include "../inc/timer_driver.h"
#include "../inc/uart_driver.h"
#include "../inc/bluetooth_driver.h"
#include "../inc/motor_driver.h"
#include "../inc/ultrasonic_driver.h"
#include "../inc/ir_sensor.h"
#include "../inc/pwm_driver.h"
#include "../inc/parking_logic.h"

// Define LED pin
#define LED_PIN 13 // Arduino Uno onboard LED

// Global variables for car hardware
Motor leftMotor, rightMotor;
UltrasonicSensor ultrasonicSensor;
IRSensor leftIRSensor, rightIRSensor;
BluetoothModule bluetooth;

// Forward declarations
void HardwareInit(void);
void SetStatusLED(GPIO_PinState state);
void BlinkStatusLED(uint8_t times);

// UART RX interrupt handler


void SetStatusLED(GPIO_PinState state) {
    GPIO_Write(LED_PIN, state);
}

void BlinkStatusLED(uint8_t times) {
    for (uint8_t i = 0; i < times; i++) {
        GPIO_Write(LED_PIN, GPIO_HIGH);
        delay_ms(200);
        GPIO_Write(LED_PIN, GPIO_LOW);
        delay_ms(200);
    }
}

void HardwareInit(void) {
    // Initialize LED pin
    GPIO_Init(LED_PIN, GPIO_OUTPUT);
    GPIO_Write(LED_PIN, GPIO_LOW);
    
    // Configure left motor
    leftMotor.in1_pin = 2;
    leftMotor.in2_pin = 3;
    leftMotor.ena_pin = 5;  // PWM pin
    Motor_Init(&leftMotor);
    
    // Configure right motor
    rightMotor.in1_pin = 4;
    rightMotor.in2_pin = 7;
    rightMotor.ena_pin = 6;  // PWM pin
    Motor_Init(&rightMotor);
    
    // Configure ultrasonic sensor
    ultrasonicSensor.trig_pin = 8;
    ultrasonicSensor.echo_pin = 9;
    Ultrasonic_Init(&ultrasonicSensor);
    
    // Configure IR sensors
    leftIRSensor.pin = 14;  // A0
    leftIRSensor.isAnalog = true;
    leftIRSensor.threshold = 500;
    IR_Init(&leftIRSensor);
    
    rightIRSensor.pin = 15;  // A1
    rightIRSensor.isAnalog = true;
    rightIRSensor.threshold = 500;
    IR_Init(&rightIRSensor);
    
    // Initialize Bluetooth (HC-05)
    bluetooth.uart.tx_pin = 1;
    bluetooth.uart.rx_pin = 0;
    bluetooth.uart.baud_rate = UART_BAUD_9600;
    Bluetooth_Init(&bluetooth);
    
    // Enable global interrupts
    sei();
    
    // Blink LED to indicate initialization complete
    BlinkStatusLED(3);
}

int main(void) {
    // Initialize hardware
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
                    if (DetectParkingSpace(&ultrasonicSensor, &leftIRSensor, &rightIRSensor, &slot)) {
                        // Execute parking maneuver based on detected slot
                        if (slot.type == PARALLEL_PARKING) {
                            Bluetooth_SendMessage(&bluetooth, "Parallel parking slot detected");
                            ExecuteParallelParking(&leftMotor, &rightMotor, 
                                                &ultrasonicSensor, &leftIRSensor, &rightIRSensor);
                        } else {
                            Bluetooth_SendMessage(&bluetooth, "Perpendicular parking slot detected");
                            ExecutePerpendicularParking(&leftMotor, &rightMotor,
                                                    &ultrasonicSensor, &leftIRSensor, &rightIRSensor);
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
        if (Ultrasonic_MeasureDistance(&ultrasonicSensor) < 10) {
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
    
    return 0;
}