// inc/ultrasonic_driver.h
#ifndef ULTRASONIC_DRIVER_H
#define ULTRASONIC_DRIVER_H

#include <stdint.h>
#include "gpio_driver.h"

// HC-SR04 Ultrasonic Sensor structure
typedef struct {
    uint8_t trig_pin;  // Arduino pin number for trigger
    uint8_t echo_pin;  // Arduino pin number for echo
} UltrasonicSensor;

// Function prototypes
void Ultrasonic_Init(UltrasonicSensor* sensor);
uint32_t Ultrasonic_MeasureDistance(UltrasonicSensor* sensor);

#endif // ULTRASONIC_DRIVER_H