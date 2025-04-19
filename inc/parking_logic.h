// parking_logic.h
#ifndef PARKING_LOGIC_H
#define PARKING_LOGIC_H

#include "motor_driver.h"
#include "ultrasonic_driver.h"

typedef enum {
    PARALLEL_PARKING,
    PERPENDICULAR_PARKING
} ParkingType;

typedef struct {
    ParkingType type;
    uint32_t size;
    uint32_t position; // Distance from current position
} ParkingSlot;

// Detect available parking spaces using side sensors
bool DetectParkingSpace(UltrasonicSensor* leftSensor, UltrasonicSensor* rightSensor, ParkingSlot* slot);

// Execute parking maneuvers
void ExecuteParallelParking(Motor* leftMotor, Motor* rightMotor, 
                          UltrasonicSensor* frontSensor, 
                          UltrasonicSensor* leftSensor, 
                          UltrasonicSensor* rightSensor);

void ExecutePerpendicularParking(Motor* leftMotor, Motor* rightMotor, 
                               UltrasonicSensor* frontSensor, 
                               UltrasonicSensor* leftSensor, 
                               UltrasonicSensor* rightSensor);

#endif // PARKING_LOGIC_H