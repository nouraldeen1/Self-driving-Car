// main.h
#ifndef MAIN_H
#define MAIN_H

#include "gpio_driver.h"
#include "ultrasonic_driver.h"
#include "motor_driver.h"
#include "bluetooth_driver.h"
#include "parking_logic.h"
#include <stdbool.h>

// Function prototypes
void SystemInit(void);
void HardwareInit(void);

// Car hardware configuration
extern Motor leftMotor, rightMotor;
extern UltrasonicSensor frontSensor, leftSensor, rightSensor;
extern BluetoothModule bluetooth;

// LED indicators
#define LED_PORT GPIOC_BASE
#define LED_PIN  13

// Function for indicating status using LED
void SetStatusLED(GPIO_PinState state);
void BlinkStatusLED(uint8_t times);

// Timer utilities
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

#endif // MAIN_H