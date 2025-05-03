// inc/ir_sensor.h
#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t pin;         // Analog or digital pin connected to IR sensor
    bool isAnalog;       // Whether the sensor outputs analog (true) or digital (false) values
    uint16_t threshold;  // Threshold value for analog sensors
} IRSensor;

// Function prototypes
void IR_Init(IRSensor* sensor);
bool IR_DetectObstacle(IRSensor* sensor);
uint16_t IR_GetRawValue(IRSensor* sensor);

#endif // IR_SENSOR_H