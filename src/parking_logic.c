// parking_logic.c
#include "../inc/parking_logic.h"
#include "../inc/avr_registers.h"
#include <stdint.h>
#include <stdbool.h>



// External function for delay
extern void delay_ms(uint32_t ms);

// External motor references
extern Motor leftMotor, rightMotor;

bool DetectParkingSpace(UltrasonicSensor* ultrasonicSensor, IRSensor* leftIRSensor, IRSensor* rightIRSensor, ParkingSlot* slot) {
    // Constants for parking space detection
    const uint32_t MIN_PARALLEL_SIZE = 40;      // Minimum length for parallel parking in cm
    const uint32_t MIN_PERPENDICULAR_SIZE = 30; // Minimum width for perpendicular parking in cm
    const uint32_t DETECTION_THRESHOLD = 500;   // IR threshold value
    const uint32_t SAMPLING_INTERVAL_MS = 100;  // Time between measurements
    const uint32_t MAX_SAMPLES = 100;           // Maximum number of samples to take
    
    // Variables to track space detection
    uint32_t ultrasonicDistance;
    bool leftIRDetected, rightIRDetected;
    uint32_t spaceStart = 0;
    uint32_t currentPosition = 0;
    uint32_t spaceSize = 0;
    bool spaceDetected = false;
    bool slotFound = false;
    
    // Move forward slowly while measuring distances
    Motor_SetDirection(&leftMotor, MOTOR_FORWARD);
    Motor_SetDirection(&rightMotor, MOTOR_FORWARD);
    Motor_SetSpeed(&leftMotor, 30); // Slow speed for accurate detection
    Motor_SetSpeed(&rightMotor, 30);
    
    // Detection loop - sample distances while moving forward
    for (uint32_t i = 0; i < MAX_SAMPLES && !slotFound; i++) {
        // Measure distance from ultrasonic sensor (front-facing)
        ultrasonicDistance = Ultrasonic_MeasureDistance(ultrasonicSensor);
        
        // Read IR sensors (side-facing)
        // IR returns true if obstacle detected
        leftIRDetected = IR_DetectObstacle(leftIRSensor);
        rightIRDetected = IR_DetectObstacle(rightIRSensor);
        
        // Check for open space on right side (assuming right-side parking)
        // Space detection: right IR sensor doesn't detect obstacle
        if (!spaceDetected && !rightIRDetected) {
            // Space start detected
            spaceDetected = true;
            spaceStart = currentPosition;
        } else if (spaceDetected && rightIRDetected) {
            // Space end detected (IR now sees obstacle again)
            spaceSize = currentPosition - spaceStart;
            
            // Determine parking type based on size
            if (spaceSize >= MIN_PARALLEL_SIZE) {
                slot->type = PARALLEL_PARKING;
                slot->size = spaceSize;
                slot->position = spaceStart;
                slotFound = true;
            } else if (spaceSize >= MIN_PERPENDICULAR_SIZE) {
                slot->type = PERPENDICULAR_PARKING;
                slot->size = spaceSize;
                slot->position = spaceStart;
                slotFound = true;
            } else {
                // Space too small, continue searching
                spaceDetected = false;
            }
        }
        
        // Increment position counter (estimate distance moved)
        currentPosition += 2; // Approximately 2cm per iteration at this speed
        delay_ms(SAMPLING_INTERVAL_MS);
    }
    
    // Stop motors regardless of whether space was found
    Motor_SetDirection(&leftMotor, MOTOR_STOP);
    Motor_SetDirection(&rightMotor, MOTOR_STOP);
    
    return slotFound;
}

void ExecuteParallelParking(Motor* leftMotor, Motor* rightMotor, 
                          UltrasonicSensor* ultrasonicSensor, 
                          IRSensor* leftIRSensor, 
                          IRSensor* rightIRSensor) {
    // Safety thresholds
    const uint32_t SIDE_CLEARANCE_CM = 15;
    const uint32_t FRONT_CLEARANCE_CM = 10;
    const uint32_t BACK_CLEARANCE_CM = 15;
    const uint32_t ALIGNED_THRESHOLD_CM = 10;
    
    // Step 1: Position the car (move forward a bit past the space)
    Motor_SetDirection(leftMotor, MOTOR_FORWARD);
    Motor_SetDirection(rightMotor, MOTOR_FORWARD);
    Motor_SetSpeed(leftMotor, 40);
    Motor_SetSpeed(rightMotor, 40);
    delay_ms(500);
    
    // Step 2: Stop and prepare for reverse
    Motor_SetDirection(leftMotor, MOTOR_STOP);
    Motor_SetDirection(rightMotor, MOTOR_STOP);
    delay_ms(500);
    
    // Step 3: Reverse and turn right (first part of S-maneuver)
    Motor_SetDirection(leftMotor, MOTOR_BACKWARD);
    Motor_SetDirection(rightMotor, MOTOR_BACKWARD);
    Motor_SetSpeed(leftMotor, 40);
    Motor_SetSpeed(rightMotor, 20); // Lower speed makes right turn
    
    // Continue until car begins to enter the space
    // Using the IR sensor to detect distance from side wall/car
    while (!IR_DetectObstacle(rightIRSensor)) {
        // Safety check - don't hit anything behind us
        if (Ultrasonic_MeasureDistance(ultrasonicSensor) < BACK_CLEARANCE_CM) {
            break;
        }
        delay_ms(50);
    }
    
    // Step 4: Straighten wheels while continuing backward
    Motor_SetSpeed(leftMotor, 30);
    Motor_SetSpeed(rightMotor, 30);
    delay_ms(800);
    
    // Step 5: Reverse and turn left to align with curb (second part of S-maneuver)
    Motor_SetSpeed(leftMotor, 20); // Lower speed makes left turn
    Motor_SetSpeed(rightMotor, 40);
    
    // Continue until close to back obstacle or fully parked
    while (Ultrasonic_MeasureDistance(ultrasonicSensor) > BACK_CLEARANCE_CM) {
        // Done if aligned with side (IR sensor detects close to side)
        if (IR_DetectObstacle(rightIRSensor)) {
            break;
        }
        delay_ms(50);
    }
    
    // Step 6: Final adjustments if needed
    if (!IR_DetectObstacle(rightIRSensor)) {
        // Adjust alignment with curb
        Motor_SetDirection(leftMotor, MOTOR_FORWARD);
        Motor_SetDirection(rightMotor, MOTOR_FORWARD);
        Motor_SetSpeed(leftMotor, 20);
        Motor_SetSpeed(rightMotor, 30);
        delay_ms(300);
    }
    
    // Step 7: Stop and complete
    Motor_SetDirection(leftMotor, MOTOR_STOP);
    Motor_SetDirection(rightMotor, MOTOR_STOP);
}

void ExecutePerpendicularParking(Motor* leftMotor, Motor* rightMotor, 
                               UltrasonicSensor* ultrasonicSensor, 
                               IRSensor* leftIRSensor, 
                               IRSensor* rightIRSensor) {
    // Safety thresholds
    const uint32_t BACK_CLEARANCE_CM = 15;
    
    // Step 1: Position the car next to the parking space
    Motor_SetDirection(leftMotor, MOTOR_FORWARD);
    Motor_SetDirection(rightMotor, MOTOR_FORWARD);
    Motor_SetSpeed(leftMotor, 40);
    Motor_SetSpeed(rightMotor, 40);
    delay_ms(300);
    
    // Step 2: Stop and prepare for the turn
    Motor_SetDirection(leftMotor, MOTOR_STOP);
    Motor_SetDirection(rightMotor, MOTOR_STOP);
    delay_ms(300);
    
    // Step 3: Make a sharp right turn while moving backward
    Motor_SetDirection(leftMotor, MOTOR_BACKWARD);
    Motor_SetDirection(rightMotor, MOTOR_BACKWARD);
    Motor_SetSpeed(leftMotor, 50);
    Motor_SetSpeed(rightMotor, 15); // Very slow on right for sharp turn
    
    // Continue until car is perpendicular to the parking space
    // Save current timer configuration
    uint8_t saved_tccr1b = TCCR1B;
    
    // Reset Timer1 for timing
    TCCR1B = 0;
    TCNT1 = 0;
    
    // Start timer with /64 prescaler
    TCCR1B = (1 << CS11) | (1 << CS10);
    
    uint32_t turnTimeout = 3000; // 3 seconds timeout in ms
    
    // Using timer for delay (TCNT1/250 is approx milliseconds when prescaler is 64)
    while ((TCNT1 / 250) < turnTimeout) {
        // Safety check - don't hit anything behind us
        if (Ultrasonic_MeasureDistance(ultrasonicSensor) < BACK_CLEARANCE_CM) {
            break;
        }
        
        // Check if we've turned enough to see the open space
        if (!IR_DetectObstacle(rightIRSensor)) {
            break;
        }
        
        delay_ms(50);
    }
    
    // Restore timer configuration
    TCCR1B = saved_tccr1b;
    
    // Step 4: Straighten wheels and back into the space
    Motor_SetSpeed(leftMotor, 30);
    Motor_SetSpeed(rightMotor, 30);
    
    // Continue until close to back obstacle
    while (Ultrasonic_MeasureDistance(ultrasonicSensor) > BACK_CLEARANCE_CM) {
        delay_ms(50);
    }
    
    // Step 5: Final adjustments if needed
    if (!IR_DetectObstacle(leftIRSensor) || !IR_DetectObstacle(rightIRSensor)) {
        // Not centered in the spot, make adjustments
        Motor_SetDirection(leftMotor, MOTOR_FORWARD);
        Motor_SetDirection(rightMotor, MOTOR_FORWARD);
        Motor_SetSpeed(leftMotor, 20);
        Motor_SetSpeed(rightMotor, 20);
        delay_ms(200);
    }
    
    // Step 6: Stop and complete
    Motor_SetDirection(leftMotor, MOTOR_STOP);
    Motor_SetDirection(rightMotor, MOTOR_STOP);
}