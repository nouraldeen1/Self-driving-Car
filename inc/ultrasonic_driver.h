// ultrasonic_driver.h
#ifndef ULTRASONIC_DRIVER_H
#define ULTRASONIC_DRIVER_H

#include "gpio_driver.h"
#include <stdint.h>

typedef struct {
    uint32_t trig_port;
    uint8_t trig_pin;
    uint32_t echo_port;
    uint8_t echo_pin;
} UltrasonicSensor;

void Ultrasonic_Init(UltrasonicSensor* sensor);
uint32_t Ultrasonic_MeasureDistance(UltrasonicSensor* sensor);

#endif // ULTRASONIC_DRIVER_H