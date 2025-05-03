// src/ir_sensor.c
#include "../inc/ir_sensor.h"
#include <Arduino.h>

void IR_Init(IRSensor* sensor) {
    if (sensor->isAnalog) {
        // For analog IR sensors, just set pin mode to INPUT
        pinMode(sensor->pin, INPUT);
    } else {
        // For digital IR sensors, set pin mode to INPUT
        pinMode(sensor->pin, INPUT);
    }
}

bool IR_DetectObstacle(IRSensor* sensor) {
    if (sensor->isAnalog) {
        // For analog sensors, compare reading to threshold
        uint16_t value = analogRead(sensor->pin);
        return (value < sensor->threshold); // Most IR sensors give lower value when obstacle is detected
    } else {
        // For digital sensors, just read the pin (LOW usually means obstacle detected)
        return (digitalRead(sensor->pin) == LOW);
    }
}

uint16_t IR_GetRawValue(IRSensor* sensor) {
    if (sensor->isAnalog) {
        return analogRead(sensor->pin);
    } else {
        return digitalRead(sensor->pin);
    }
}