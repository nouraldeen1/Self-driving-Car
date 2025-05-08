// main.c
#include <avr/io.h>
#include <avr/interrupt.h>
#include "../inc/gpio_driver.h"
#include "../inc/timer_driver.h"
#include "../inc/uart_driver.h"
#include "../inc/bluetooth_driver.h"
#include "../inc/motor_driver.h"
#include "../inc/ultrasonic_driver.h"
#include "../inc/ir_sensor.h"
#include "../inc/pwm_driver.h"
#include "../inc/parking_logic.h"

// Define LED pin
#define LED_PIN 19 // Arduino Uno onboard LED
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19
typedef enum
{
    START,
    SEARCHING,  // Moving forward looking for parking space
    ROTATING,   // Positioning for entry when space found
    BACKING_IN, // Reversing into the parking space
    PARKED      // Parking complete
} ParkingState;
// Global variables for car hardware
Motor leftMotor, rightMotor;
UltrasonicSensor rearLeftUltrasonicSensor, rearRightUltrasonicSensor, frontLeftUltrasonicSensor, frontRightUltrasonicSensor;
IRSensor leftIRSensor, rightIRSensor;
BluetoothModule bluetooth;

// Forward declarations
void HardwareInit(void);
void SetStatusLED(GPIO_PinState state);
void BlinkStatusLED(uint8_t times);
bool CheckForObstacle(void);
// UART RX interrupt handler
// Helper function to check for obstacles using the ultrasonic sensors
bool CheckForObstacle(void)
{
    const uint32_t OBSTACLE_THRESHOLD_CM = 10; // Adjust threshold as needed

    // Check all sensors for obstacles
    // if (Ultrasonic_MeasureDistance(&frontLeftUltrasonicSensor) < OBSTACLE_THRESHOLD_CM ||
    //     Ultrasonic_MeasureDistance(&frontRightUltrasonicSensor) < OBSTACLE_THRESHOLD_CM ||
    //     Ultrasonic_MeasureDistance(&rearLeftUltrasonicSensor) < OBSTACLE_THRESHOLD_CM ||
    //     Ultrasonic_MeasureDistance(&rearRightUltrasonicSensor) < OBSTACLE_THRESHOLD_CM) {
    //     return true; // Obstacle detected
    // }

    // Also check IR sensor if available
    if (IR_DetectObstacle(&rightIRSensor))
    {
        return true;
    }

    return false; // No obstacle
}
void SetStatusLED(GPIO_PinState state)
{
    GPIO_Write(LED_PIN, state);
}

void BlinkStatusLED(uint8_t times)
{
    for (uint8_t i = 0; i < times; i++)
    {
        GPIO_Write(LED_PIN, GPIO_HIGH);
        delay_ms(200);
        GPIO_Write(LED_PIN, GPIO_LOW);
        delay_ms(200);
    }
}

void HardwareInit(void)
{
    // Initialize LED pin
    GPIO_Init(LED_PIN, GPIO_OUTPUT);
    GPIO_Write(LED_PIN, GPIO_LOW);

    // Configure left motor
    leftMotor.in1_pin = 2;
    leftMotor.in2_pin = 3;
    leftMotor.ena_pin = 5; // PWM pin
    Motor_Init(&leftMotor);

    // Configure right motor
    rightMotor.in1_pin = 4;
    rightMotor.in2_pin = 7;
    rightMotor.ena_pin = 6; // PWM pin
    Motor_Init(&rightMotor);

    // Configure ultrasonic sensor
    frontRightUltrasonicSensor.trig_pin = 8;
    frontRightUltrasonicSensor.echo_pin = 9;
    Ultrasonic_Init(&frontRightUltrasonicSensor);
    rearRightUltrasonicSensor.trig_pin = 10;
    rearRightUltrasonicSensor.echo_pin = 11;
    Ultrasonic_Init(&rearRightUltrasonicSensor);

    frontLeftUltrasonicSensor.trig_pin = 12;
    frontLeftUltrasonicSensor.echo_pin = 13;
    Ultrasonic_Init(&frontLeftUltrasonicSensor);
    rearLeftUltrasonicSensor.trig_pin = 18;
    rearLeftUltrasonicSensor.echo_pin = 17;
    Ultrasonic_Init(&rearLeftUltrasonicSensor);
    // Configure IR sensors

    rightIRSensor.pin = 15; // A1
    rightIRSensor.isAnalog = true;
    rightIRSensor.threshold = 500;
    IR_Init(&rightIRSensor);

    // Initialize Bluetooth (HC-05)
    bluetooth.uart.tx_pin = 1;
    bluetooth.uart.rx_pin = 0;
    bluetooth.uart.baud_rate = UART_BAUD_9600;
    Bluetooth_Init(&bluetooth);

    // Enable global interrupts
    sei();

    // Blink LED to indicate initialization complete
    // BlinkStatusLED(3);
}
// Add this to main.c after your existing functions
void ParkBaby(void)
{
    Motor_SetDirection(&leftMotor, MOTOR_STOP);
    Motor_SetDirection(&rightMotor, MOTOR_STOP);
    // Visual indication that parking is starting

    Bluetooth_SendMessage(&bluetooth, "Starting parking maneuver...");

    // Step 1: Move backward to position the car
    Bluetooth_SendMessage(&bluetooth, "Step 1: Moving backward");

    // Set motor directions for backward movement
    Motor_SetDirection(&leftMotor, MOTOR_BACKWARD);
    Motor_SetDirection(&rightMotor, MOTOR_BACKWARD);

    // Set speed (medium speed for backward movement)
    Motor_SetSpeed(&leftMotor, 80);
    Motor_SetSpeed(&rightMotor, 80);

    // Move backward for 2 seconds
    SetStatusLED(GPIO_HIGH);

    delay_ms(600);
    // Stop briefly
    Motor_SetDirection(&leftMotor, MOTOR_STOP);
    Motor_SetDirection(&rightMotor, MOTOR_STOP);
    SetStatusLED(GPIO_LOW);
    delay_ms(5000);

    // Step 2: Rotate half cycle (180 degrees)
    Bluetooth_SendMessage(&bluetooth, "Step 2: Rotating half cycle");

    // Set directions for rotation (one motor forward, one backward)
    Motor_SetDirection(&leftMotor, MOTOR_FORWARD);
    Motor_SetDirection(&rightMotor, MOTOR_BACKWARD);

    // Use medium speed for rotation
    Motor_SetSpeed(&leftMotor, 150);
    Motor_SetSpeed(&rightMotor, 150);

    // Rotate for approximately half a cycle (timing depends on your car)
    // Typical 180-degree rotation takes 1.5-2.5 seconds
    // SetStatusLED(GPIO_HIGH);
    delay_ms(2500); // Adjust this value based on testing

    // Stop briefly
    Motor_SetDirection(&leftMotor, MOTOR_STOP);
    Motor_SetDirection(&rightMotor, MOTOR_STOP);
    SetStatusLED(GPIO_LOW);
    delay_ms(5000);

    // Step 3: Move a little forward to finish parking
    Bluetooth_SendMessage(&bluetooth, "Step 3: Final adjustment forward");

    // Set directions for forward movement
    Motor_SetDirection(&leftMotor, MOTOR_FORWARD);
    Motor_SetDirection(&rightMotor, MOTOR_FORWARD);

    // Use slow speed for final positioning
    Motor_SetSpeed(&leftMotor, 70);
    Motor_SetSpeed(&rightMotor, 70);

    // Move forward slightly (short duration)
    // SetStatusLED(GPIO_HIGH);
    delay_ms(1200); // Adjust based on how far forward you want to move

    // Stop when parking is complete
    Motor_SetDirection(&leftMotor, MOTOR_STOP);
    Motor_SetDirection(&rightMotor, MOTOR_STOP);
    SetStatusLED(GPIO_LOW);

    // // Visual indication that parking is complete
    // BlinkStatusLED(5);
    Bluetooth_SendMessage(&bluetooth, "Parking complete!");
    SetStatusLED(GPIO_HIGH);
    BlinkStatusLED(3);
}
// Right-side parking maneuver with obstacle detection
void ParkBabyRight(void)
{
    Motor_SetDirection(&leftMotor, MOTOR_STOP);
    Motor_SetDirection(&rightMotor, MOTOR_STOP);

    Bluetooth_SendMessage(&bluetooth, "Starting right-side parking maneuver...");

    // Check for obstacles while moving forward
    uint16_t elapsed_time = 0;
    uint16_t target_time = 500;
    uint16_t check_interval = 50; // Check every 50ms


    // Stop briefly
    Motor_SetDirection(&leftMotor, MOTOR_STOP);
    Motor_SetDirection(&rightMotor, MOTOR_STOP);
    delay_ms(1000);

    // Step 2: Turn right to begin parking
    Bluetooth_SendMessage(&bluetooth, "Step 2: Turning right");
    Motor_SetDirection(&leftMotor, MOTOR_FORWARD);
    Motor_SetDirection(&rightMotor,MOTOR_BACKWARD  );
    Motor_SetSpeed(&leftMotor, 80);
    Motor_SetSpeed(&rightMotor, 80);

    // Check for obstacles while turning
    elapsed_time = 0;
    target_time = 1800;

    while (elapsed_time < target_time)
    {
        // if (CheckForObstacle())
        // {
        //     Motor_SetDirection(&leftMotor, MOTOR_STOP);
        //     Motor_SetDirection(&rightMotor, MOTOR_STOP);
        //     Bluetooth_SendMessage(&bluetooth, "OBSTACLE DETECTED! Stopping.");
        //     BlinkStatusLED(5);
        //     return;
        // }
        delay_ms(check_interval);
        elapsed_time += check_interval;
    }

    // Stop briefly
    Motor_SetDirection(&leftMotor, MOTOR_STOP);
    Motor_SetDirection(&rightMotor, MOTOR_STOP);
    delay_ms(1000);

    // Step 3: Reverse into parking space
    Bluetooth_SendMessage(&bluetooth, "Step 3: moving forward to parked space");
    Motor_SetDirection(&leftMotor, MOTOR_FORWARD);
    Motor_SetDirection(&rightMotor, MOTOR_FORWARD);
    Motor_SetSpeed(&leftMotor, 70);
    Motor_SetSpeed(&rightMotor, 70); // Slightly slower to curve into space

    // Check for obstacles while backing in
    elapsed_time = 0;
    target_time = 1200;

    while (elapsed_time < target_time)
    {
        if (CheckForObstacle())
        {
            Motor_SetDirection(&leftMotor, MOTOR_STOP);
            Motor_SetDirection(&rightMotor, MOTOR_STOP);
            Bluetooth_SendMessage(&bluetooth, "OBSTACLE DETECTED! Stopping.");
            BlinkStatusLED(5);
            return;
        }
        delay_ms(check_interval);
        elapsed_time += check_interval;
    }

    // Stop briefly
    Motor_SetDirection(&leftMotor, MOTOR_STOP);
    Motor_SetDirection(&rightMotor, MOTOR_STOP);
    delay_ms(1000);

    Bluetooth_SendMessage(&bluetooth, "Right-side parking complete!");
    SetStatusLED(GPIO_HIGH);
}
// Left-side parking maneuver with obstacle detection
void ParkBabyLeft(void)
{
    Motor_SetDirection(&leftMotor, MOTOR_STOP);
    Motor_SetDirection(&rightMotor, MOTOR_STOP);

    Bluetooth_SendMessage(&bluetooth, "Starting left-side parking maneuver...");

    // Check for obstacles while moving forward
    uint16_t elapsed_time = 0;
    uint16_t target_time = 500;
    uint16_t check_interval = 50; // Check every 50ms

    // Stop briefly
    Motor_SetDirection(&leftMotor, MOTOR_STOP);
    Motor_SetDirection(&rightMotor, MOTOR_STOP);
    delay_ms(1000);

    // Step 2: Turn left to begin parking
    Bluetooth_SendMessage(&bluetooth, "Step 2: Turning left");
    Motor_SetDirection(&leftMotor,MOTOR_BACKWARD );
    Motor_SetDirection(&rightMotor,MOTOR_FORWARD );

    Motor_SetSpeed(&leftMotor, 80);
    Motor_SetSpeed(&rightMotor, 80);

    // Check for obstacles while turning
    elapsed_time = 0;
    target_time = 1000;

    while (elapsed_time < target_time)
    {
        if (CheckForObstacle())
        {
            Motor_SetDirection(&leftMotor, MOTOR_STOP);
            Motor_SetDirection(&rightMotor, MOTOR_STOP);
            Bluetooth_SendMessage(&bluetooth, "OBSTACLE DETECTED! Stopping.");
            BlinkStatusLED(5);
            return;
        }
        delay_ms(check_interval);
        elapsed_time += check_interval;
    }

    // Stop briefly
    Motor_SetDirection(&leftMotor, MOTOR_STOP);
    Motor_SetDirection(&rightMotor, MOTOR_STOP);
    delay_ms(1000);

    // Step 3: Move forward to parked space
    Bluetooth_SendMessage(&bluetooth, "Step 3: moving forward to parked space");
    Motor_SetDirection(&leftMotor, MOTOR_FORWARD);
    Motor_SetDirection(&rightMotor, MOTOR_FORWARD);
    Motor_SetSpeed(&leftMotor, 90);
    Motor_SetSpeed(&rightMotor, 90); // Equal speed for both motors

    // Check for obstacles while moving forward
    elapsed_time = 0;
    target_time = 1000;

    while (elapsed_time < target_time)
    {
        if (CheckForObstacle())
        {
            Motor_SetDirection(&leftMotor, MOTOR_STOP);
            Motor_SetDirection(&rightMotor, MOTOR_STOP);
            Bluetooth_SendMessage(&bluetooth, "OBSTACLE DETECTED! Stopping.");
            BlinkStatusLED(5);
            return;
        }
        delay_ms(check_interval);
        elapsed_time += check_interval;
    }

    // Stop briefly
    Motor_SetDirection(&leftMotor, MOTOR_STOP);
    Motor_SetDirection(&rightMotor, MOTOR_STOP);
    delay_ms(1000);

    Bluetooth_SendMessage(&bluetooth, "Left-side parking complete!");
    BlinkStatusLED(2);
}
// Simple serial data reading function with blocking behavior
Command CheckForCommand(void)
{
    // This will wait until data is received from the bluetooth module
    uint8_t data = UART_Receive();

    // Process the received data
    if (data == '0')
    {
        // Send response
        Bluetooth_SendMessage(&bluetooth, "Zero");
    }
    else if (data == '1')
    {
        // Send response
        Bluetooth_SendMessage(&bluetooth, "One");
        return COMMAND_START_PARKING;
    }

    return COMMAND_NONE;
}
// int main(void)
// {
//     // Initialize hardware
//     HardwareInit();
//     // Define parking state variables

//     // Send startup message and indication
//     BlinkStatusLED(2);

//     // Main loop
//     while (1)
//     {
//         Command cmd = CheckForCommand();
//         // if(Ultrasonic_MeasureDistance(&frontRightUltrasonicSensor) <= 25)
//         // {
//         //     BlinkStatusLED(3);
//         // }
//         // Process commands
//         if (cmd == COMMAND_START_PARKING)
//         {
//             Bluetooth_SendMessage(&bluetooth, "lets go!");
//             while (1)
//             {
//                 // Here you would add your parking logic code
//                 // ParkingLogic_StartParking(&leftMotor, &rightMotor, &frontRightUltrasonicSensor, &leftIRSensor, &rightIRSensor);
//                 // State machine for parking logic

//                 if (IR_DetectObstacle(&rightIRSensor)){
//                     // Obstacle detected, stop motors
//                     Motor_SetDirection(&leftMotor, MOTOR_STOP);
//                     Motor_SetDirection(&rightMotor, MOTOR_STOP);
//                     break;
//                 }
//                 DetectParkingSpaceReturn detectedSlot = DetectParkingSpace(&leftMotor, &rightMotor, &rearLeftUltrasonicSensor, &frontLeftUltrasonicSensor, &frontRightUltrasonicSensor, &rearRightUltrasonicSensor, &rightIRSensor, NULL);
//                 // Check for parking space
//                 if (detectedSlot.detected)
//                 {
//                     if (detectedSlot.direction == RIGHT)
//                     {
//                         Bluetooth_SendMessage(&bluetooth, "RIGHT parking space detected!");
//                         BlinkStatusLED(2);
//                         SetStatusLED(GPIO_HIGH);
//                         // Execute right-side parking maneuver
//                         ParkBabyRight();
//                         //ParkBaby();
//                     }
//                     else if (detectedSlot.direction == LEFT)
//                     {
//                         Bluetooth_SendMessage(&bluetooth, "LEFT parking space detected!");
//                         SetStatusLED(GPIO_HIGH);
//                         // Execute left-side parking maneuver
//                         ParkBabyLeft();
//                         // ParkBaby();
//                     }
//                     else
//                     {
//                         SetStatusLED(GPIO_LOW);
//                     }

//                     Bluetooth_SendMessage(&bluetooth, "6666666666666");
//                     BlinkStatusLED(3);

//                     // BlinkStatusLED(2); // Visual indication of state change
//                     // break;
//                 }

//                 Bluetooth_SendMessage(&bluetooth, "5555555555");
//             }
//         }
//         // Small delay to prevent CPU hogging
//         delay_ms(100);
//     }

//     return 0;
// }
int main(void)
{
    // Initialize hardware
    HardwareInit();
    // Define parking state variables

    // Send startup message and indication
    BlinkStatusLED(2);

    // Main loop
    while (1)
    {
        Command cmd = CheckForCommand();
     
        // Process commands
        if (cmd == COMMAND_START_PARKING)
        {
            Bluetooth_SendMessage(&bluetooth, "I WILL FIND A SPACE NIGGA");
            while (1)
            {
                if (IR_DetectObstacle(&rightIRSensor))
                {
                    // Obstacle detected, stop motors
                    Motor_SetDirection(&leftMotor, MOTOR_STOP);
                    Motor_SetDirection(&rightMotor, MOTOR_STOP);
                    break;
                }
                DetectParkingSpaceReturn hi = DetectParkingSpace(&leftMotor, &rightMotor, &rearLeftUltrasonicSensor, &frontLeftUltrasonicSensor, &frontRightUltrasonicSensor, &rearRightUltrasonicSensor, &rightIRSensor, NULL);
                // Process commands
                if (hi.detected && hi.direction == LEFT)
                {
                    Bluetooth_SendMessage(&bluetooth, "I AM LEFT PARKING NIGGA");
                    ParkBabyLeft();
                   
                    break;
                }
                if (hi.detected && hi.direction == RIGHT)
                {
                    // Execute right-side parking maneuver
                    Bluetooth_SendMessage(&bluetooth, "I AM RIGHT PARKING NIGGA");
                    ParkBabyRight();
                   
                    break;
                }
            
            }
        }
        Bluetooth_SendMessage(&bluetooth, "fck you");
    }
    return 0;
}
