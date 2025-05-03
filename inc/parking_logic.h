// inc/parking_logic.h
#ifndef PARKING_LOGIC_H
#define PARKING_LOGIC_H

#include <stdint.h>
#include <stdbool.h>
#include "motor_driver.h"
#include "ultrasonic_driver.h"
#include "ir_sensor.h"

// Parking types
typedef enum {
    PARALLEL_PARKING,
    PERPENDICULAR_PARKING
} ParkingType;

// Parking slot structure
typedef struct {
    ParkingType type;
    uint32_t size;
    uint32_t position;
} ParkingSlot;

// Function prototypes - for 1 ultrasonic sensor and 2 IR sensors
bool DetectParkingSpace(UltrasonicSensor* ultrasonicSensor, IRSensor* leftIRSensor, IRSensor* rightIRSensor, ParkingSlot* slot);

void ExecuteParallelParking(Motor* leftMotor, Motor* rightMotor, 
                          UltrasonicSensor* ultrasonicSensor, 
                          IRSensor* leftIRSensor, 
                          IRSensor* rightIRSensor);

void ExecutePerpendicularParking(Motor* leftMotor, Motor* rightMotor, 
                               UltrasonicSensor* ultrasonicSensor, 
                               IRSensor* leftIRSensor, 
                               IRSensor* rightIRSensor);

#endif // PARKING_LOGIC_H