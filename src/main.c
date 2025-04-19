// main.c
#include "main.h"
#include "gpio_driver.h"
#include "ultrasonic_driver.h"
#include "motor_driver.h"
#include "bluetooth_driver.h"
#include "parking_logic.h"

// Define car hardware configuration
Motor leftMotor, rightMotor;
UltrasonicSensor frontSensor, leftSensor, rightSensor;
BluetoothModule bluetooth;

void SystemInit(void) {
    // Initialize system clock and basic hardware
}

void HardwareInit(void) {
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
    Bluetooth_Init(&bluetooth);
}

int main(void) {
    SystemInit();
    HardwareInit();
    
    while(1) {
        // Check for Bluetooth commands
        if (Bluetooth_IsCommandReceived(&bluetooth)) {
            Command cmd = Bluetooth_GetCommand(&bluetooth);
            
            if (cmd == COMMAND_START_PARKING) {
                // Start the parking sequence
                ParkingSlot slot;
                
                // Move forward and detect parking space
                if (DetectParkingSpace(&leftSensor, &rightSensor, &slot)) {
                    // Execute parking maneuver based on detected slot
                    if (slot.type == PARALLEL_PARKING) {
                        ExecuteParallelParking(&leftMotor, &rightMotor, 
                                              &frontSensor, &leftSensor, &rightSensor);
                    } else {
                        ExecutePerpendicularParking(&leftMotor, &rightMotor,
                                                  &frontSensor, &leftSensor, &rightSensor);
                    }
                    
                    // Send completion status via Bluetooth
                    Bluetooth_SendMessage(&bluetooth, "Parking completed");
                } else {
                    // No suitable parking space found
                    Bluetooth_SendMessage(&bluetooth, "No parking space found");
                }
            }
        }
    }
}