// parking_logic.c
#include "../inc/parking_logic.h"
#include "../inc/avr_registers.h"
#include <stdint.h>
#include <stdbool.h>
#include "../inc/bluetooth_driver.h"
#define LED_PIN 19              // Arduino Uno onboard LED
#define NUMBER_oF_ITERATIONS 10 // Number of iterations for the loop
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
DetectParkingSpaceReturn DetectParkingSpace(Motor *leftMotor, Motor *rightMotor,
                                            UltrasonicSensor *rearLeftUtrasonicSensor,
                                            UltrasonicSensor *frontLeftUltrasonicSensor,
                                            UltrasonicSensor *frontRightUltrasonicSensor,
                                            UltrasonicSensor *rearRightUltrasonicSensor,
                                            IRSensor *IRSensor, ParkingSlot *slot,BluetoothModule *bluetooth)
{
    // Set motors to move forward
    Motor_SetSpeed(leftMotor, 65);
    Motor_SetSpeed(rightMotor, 65);
    Motor_SetDirection(leftMotor, MOTOR_FORWARD);
    Motor_SetDirection(rightMotor, MOTOR_FORWARD);

    // Constants
    const uint32_t MIN_SLOT_WIDTH_CM = 15;
    const uint8_t REQUIRED_CONFIRMATIONS = 1;
    const uint32_t DELAY_BETWEEN_READINGS = 100; // ms

    // Check for parking spaces
    for (int i = 0; i < NUMBER_oF_ITERATIONS; i++)
    {
        // Get fresh sensor readings each time
        bool rearLeftClear = Ultrasonic_MeasureDistance(rearLeftUtrasonicSensor) >= MIN_SLOT_WIDTH_CM;
        bool frontLeftClear = Ultrasonic_MeasureDistance(frontLeftUltrasonicSensor) >= MIN_SLOT_WIDTH_CM;
        bool rearRightClear = Ultrasonic_MeasureDistance(rearRightUltrasonicSensor) >= MIN_SLOT_WIDTH_CM;
        bool frontRightClear = Ultrasonic_MeasureDistance(frontRightUltrasonicSensor) >= MIN_SLOT_WIDTH_CM;
        char debug_msg[50];
        sprintf(debug_msg, "Lb: R=%d, F=%d",
                (int)Ultrasonic_MeasureDistance(rearLeftUtrasonicSensor),
                (int)Ultrasonic_MeasureDistance(frontLeftUltrasonicSensor));
        Bluetooth_SendMessage(&bluetooth, debug_msg);
        // Check for left side parking space
           // Check for right side parking space
           if (rearLeftClear && frontLeftClear)
           {
               // Confirm with additional readings
               uint8_t confirmations = 1;
               for (int j = 0; j < REQUIRED_CONFIRMATIONS; j++)
               {
                   delay_ms(DELAY_BETWEEN_READINGS);
   
                   // Get new readings
                   rearLeftClear = Ultrasonic_MeasureDistance(rearLeftUtrasonicSensor) >= MIN_SLOT_WIDTH_CM;
                   frontLeftClear = Ultrasonic_MeasureDistance(frontLeftUltrasonicSensor) >= MIN_SLOT_WIDTH_CM;

                   if (rearLeftClear && frontLeftClear)
                   {
                    char debug_msg[50];
                    sprintf(debug_msg, "CL: R=%d, F=%d",
                            (int)Ultrasonic_MeasureDistance(rearLeftUtrasonicSensor),
                            (int)Ultrasonic_MeasureDistance(frontLeftUltrasonicSensor));
                    Bluetooth_SendMessage(&bluetooth, debug_msg);
                       confirmations++;
                       if (confirmations >= REQUIRED_CONFIRMATIONS)
                       {
                           // Stop motors
                           Motor_SetSpeed(leftMotor, 0);
                           Motor_SetSpeed(rightMotor, 0);
   
                           // Return result
                           DetectParkingSpaceReturn space = {LEFT, true};
                           return space;
                       }
                   }
                   else
                   {
                       break; // Space not confirmed
                   }
               }
           }
        // Check for right side parking space
        if (rearRightClear && frontRightClear)
        {
            // Confirm with additional readings
            uint8_t confirmations = 1;
            for (int j = 0; j < REQUIRED_CONFIRMATIONS; j++)
            {
                delay_ms(DELAY_BETWEEN_READINGS);

                // Get new readings
                rearRightClear = Ultrasonic_MeasureDistance(rearRightUltrasonicSensor) >= MIN_SLOT_WIDTH_CM;
                frontRightClear = Ultrasonic_MeasureDistance(frontRightUltrasonicSensor) >= MIN_SLOT_WIDTH_CM;

                if (rearRightClear && frontRightClear)
                {
                    confirmations++;
                    if (confirmations >= REQUIRED_CONFIRMATIONS)
                    {
                        // Stop motors
                        Motor_SetSpeed(leftMotor, 0);
                        Motor_SetSpeed(rightMotor, 0);

                        // Return result
                        DetectParkingSpaceReturn space = {RIGHT, true};
                        return space;
                    }
                }
                else
                {
                    break; // Space not confirmed
                }
            }
        }

        delay_ms(DELAY_BETWEEN_READINGS);
    }

    // No space found - stop motors and return
    Motor_SetSpeed(leftMotor, 0);
    Motor_SetSpeed(rightMotor, 0);
    DetectParkingSpaceReturn falseSpace = {LEFT, false};
    return falseSpace;
}

void ExecutePerpendicularParking(Motor *leftMotor, Motor *rightMotor,
                                 UltrasonicSensor *ultrasonicSensor,
                                 IRSensor *leftIRSensor,
                                 IRSensor *rightIRSensor);