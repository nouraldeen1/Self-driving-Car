// parking_logic.c
#include "../inc/parking_logic.h"
#include "../inc/avr_registers.h"
#include <stdint.h>
#include <stdbool.h>

#define LED_PIN 13 // Arduino Uno onboard LED
// External function for delay
extern void delay_ms(uint32_t ms);
// External motor references
extern Motor leftMotor, rightMotor;
void MoveVerySlowly(Motor *leftMotor, Motor *rightMotor, MotorDirection direction)
{
    // Constants for very slow movement
    const uint8_t VERY_SLOW_SPEED = 13; // Using a low PWM value for slow movement
    const uint16_t RAMP_DELAY_MS = 300;  // Time between speed increments for smooth start

    // Set motor directions
    Motor_SetDirection(leftMotor, direction);
    Motor_SetDirection(rightMotor, direction);

    // Start with motors stopped
    Motor_SetSpeed(leftMotor, 0);
    Motor_SetSpeed(rightMotor, 0);

    // Gradually ramp up speed for smooth start
    for (uint8_t speed = 0; speed <= VERY_SLOW_SPEED; speed += 2)
    {
        Motor_SetSpeed(leftMotor, speed);
        Motor_SetSpeed(rightMotor, speed);
        delay_ms(1000);
    }
}
bool DetectParkingSpace(Motor *leftMotor, Motor *rightMotor,UltrasonicSensor *ultrasonicSensor, IRSensor *leftIRSensor, IRSensor *rightIRSensor, ParkingSlot *slot)
{
    //Move a little bit forward to get a reading for other potential slot
    // Set motor directions
    Motor_SetDirection(leftMotor, MOTOR_FORWARD);
    Motor_SetDirection(rightMotor, MOTOR_FORWARD);

    // Start with motors stopped
    Motor_SetSpeed(leftMotor, 80);
    Motor_SetSpeed(rightMotor, 80);

    // Constants for parking space detection
    const uint32_t MIN_SLOT_WIDTH_CM = 25;       // Minimum width for parking
    const uint32_t MIN_CONSECUTIVE_READINGS = 3; // Number of consistent readings needed
    const uint32_t SAMPLING_INTERVAL_MS = 50;    // Take measurements every 50ms

    // Local variables
    uint32_t consecutiveValidReadings = 0;
    bool spaceDetected = false;

    // Take several readings to ensure consistency and filter noise
    for (uint8_t i = 0; i < MIN_CONSECUTIVE_READINGS; i++)
    {
        // Get sensor readings
        bool leftClear = !IR_DetectObstacle(leftIRSensor);   // Rear sensor
        bool rightClear = !IR_DetectObstacle(rightIRSensor); // Front sensor
        uint32_t middleDistance = Ultrasonic_MeasureDistance(ultrasonicSensor);
        bool middleClear = (middleDistance >= MIN_SLOT_WIDTH_CM);

        // Condition 1: All three sensors detect no obstacle
        // Condition 2: Middle and rear sensors detect no obstacle
        if ((leftClear && middleClear && rightClear) ||
            (leftClear && middleClear))
        {
            consecutiveValidReadings++;
        }
        else
        {
            // Reset the counter if any reading is invalid
            consecutiveValidReadings = 0;

            // Early exit - no need to continue checking
            break;
        }

        // Wait before next sampling
        delay_ms(SAMPLING_INTERVAL_MS);
    }

    // If we have enough consecutive valid readings, we've found a space
    if (consecutiveValidReadings >= MIN_CONSECUTIVE_READINGS)
    {
            // Start with motors stopped
        Motor_SetSpeed(leftMotor, 0);
        Motor_SetSpeed(rightMotor, 0);
        Motor_SetDirection(leftMotor, MOTOR_STOP);
        Motor_SetDirection(rightMotor, MOTOR_STOP);
        spaceDetected = true;
    }


    return spaceDetected;
}

void ExecutePerpendicularParking(Motor *leftMotor, Motor *rightMotor,
                                 UltrasonicSensor *ultrasonicSensor,
                                 IRSensor *leftIRSensor,
                                 IRSensor *rightIRSensor);