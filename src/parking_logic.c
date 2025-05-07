// parking_logic.c
#include "../inc/parking_logic.h"
#include "../inc/avr_registers.h"
#include <stdint.h>
#include <stdbool.h>


#define LED_PIN 19 // Arduino Uno onboard LED
// External function for delay
extern void delay_ms(uint32_t ms);
// External motor references
extern Motor leftMotor, rightMotor;
void MoveVerySlowly(Motor *leftMotor, Motor *rightMotor, MotorDirection direction)
{
    // Constants for very slow movement
    const uint8_t VERY_SLOW_SPEED = 13; // Using a low PWM value for slow movement
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
  DetectParkingSpaceReturn DetectParkingSpace(Motor *leftMotor, Motor *rightMotor,UltrasonicSensor *rearLeftUtrasonicSensor, UltrasonicSensor* frontLeftUltrasonicSensor,UltrasonicSensor* frontRightUltrasonicSensor, UltrasonicSensor* rearRightUltrasonicSensor,IRSensor *IRSensor, ParkingSlot *slot)
{
    //Move a little bit forward to get a reading for other potential slot
    // Set motor directions
    Motor_SetDirection(leftMotor, MOTOR_FORWARD);
    Motor_SetDirection(rightMotor, MOTOR_FORWARD);

    // Start with motors stopped
    Motor_SetSpeed(leftMotor, 78);
    Motor_SetSpeed(rightMotor, 78);

    // Constants for parking space detection
    const uint32_t MIN_SLOT_WIDTH_CM = 25;       // Minimum width for parking
    const uint32_t MIN_SLOT_WIDTH_REAR = 30;      // Minimum length for parking
    const uint32_t MIN_CONSECUTIVE_READINGS = 3; // Number of consistent readings needed
    const uint32_t SAMPLING_INTERVAL_MS = 50;    // Take measurements every 50ms

    // Local variables
    uint32_t consecutiveValidReadingsRight = 0;
    uint32_t consecutiveValidReadingsLeft = 0;

    bool spaceDetected = false;
    Direction parkingDirection = LEFT; // Default direction to left

    // Take several readings to ensure consistency and filter noise
    for (uint8_t i = 0; i < MIN_CONSECUTIVE_READINGS; i++)
    {
        // Get sensor readings
        
        bool frontClear = IR_DetectObstacle(IRSensor);
        bool rearLeftClear = Ultrasonic_MeasureDistance(rearLeftUtrasonicSensor) >= MIN_SLOT_WIDTH_REAR;
        bool rearRightClear = Ultrasonic_MeasureDistance(rearRightUltrasonicSensor) >= MIN_SLOT_WIDTH_REAR;
        bool frontLeftClear = Ultrasonic_MeasureDistance(frontLeftUltrasonicSensor) >= MIN_SLOT_WIDTH_CM;
        bool frontRightClear = Ultrasonic_MeasureDistance(frontRightUltrasonicSensor) >= MIN_SLOT_WIDTH_CM;


    
        

        if (!frontClear){


            Motor_SetDirection(leftMotor, MOTOR_STOP);
            Motor_SetDirection(rightMotor, MOTOR_STOP);

            DetectParkingSpaceReturn result = {LEFT, false};
            return result;
        }


        if (rearRightClear && frontRightClear )
        {
            consecutiveValidReadingsRight++;
        }else{
            // Reset the counter if any reading is invalid
            consecutiveValidReadingsRight = 0;
            break;
        }

        if (rearLeftClear && frontLeftClear )
        {
            consecutiveValidReadingsLeft++;
        }else{
            // Reset the counter if any reading is invalid
            consecutiveValidReadingsLeft = 0;
            break;
        }
        

        // Wait before next sampling
        delay_ms(SAMPLING_INTERVAL_MS);
    }

    // If we have enough consecutive valid readings, we've found a space
    if (consecutiveValidReadingsRight >= MIN_CONSECUTIVE_READINGS)
    {
            // Start with motors stopped
        Motor_SetSpeed(leftMotor, 0);
        Motor_SetSpeed(rightMotor, 0);
        Motor_SetDirection(leftMotor, MOTOR_STOP);
        Motor_SetDirection(rightMotor, MOTOR_STOP);
        spaceDetected = true;
        parkingDirection = RIGHT;
    } else if (consecutiveValidReadingsLeft >= MIN_CONSECUTIVE_READINGS)
    {
            // Start with motors stopped
        Motor_SetSpeed(leftMotor, 0);
        Motor_SetSpeed(rightMotor, 0);
        Motor_SetDirection(leftMotor, MOTOR_STOP);
        Motor_SetDirection(rightMotor, MOTOR_STOP);
        spaceDetected = true;
        parkingDirection = LEFT;
    }


    DetectParkingSpaceReturn result = {parkingDirection, spaceDetected};
}

void ExecutePerpendicularParking(Motor *leftMotor, Motor *rightMotor,
                                 UltrasonicSensor *ultrasonicSensor,
                                 IRSensor *leftIRSensor,
                                 IRSensor *rightIRSensor);