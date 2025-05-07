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

typedef enum {
    LEFT,
    RIGHT
} Direction;

typedef struct {
    Direction direction; // Direction of the parking slot (left or right)
    bool detected;
}DetectParkingSpaceReturn;

// Function prototypes - for 1 ultrasonic sensor and 2 IR sensors
DetectParkingSpaceReturn DetectParkingSpace(Motor *leftMotor, Motor *rightMotor,UltrasonicSensor* , UltrasonicSensor*, UltrasonicSensor*,UltrasonicSensor*,  IRSensor* rightIRSensor, ParkingSlot* slot);

void ExecuteParallelParking(Motor* leftMotor, Motor* rightMotor, 
                          UltrasonicSensor* ultrasonicSensor, 
                          IRSensor* leftIRSensor, 
                          IRSensor* rightIRSensor);

void ExecutePerpendicularParking(Motor* leftMotor, Motor* rightMotor, 
                               UltrasonicSensor* ultrasonicSensor, 
                               IRSensor* leftIRSensor, 
                               IRSensor* rightIRSensor);
void MoveVerySlowly(Motor* leftMotor, Motor* rightMotor, MotorDirection direction);
#endif // PARKING_LOGIC_H