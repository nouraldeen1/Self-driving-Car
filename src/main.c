#include <stdint.h>
#include <stdbool.h>

// External function prototypes
extern void GPIO_ClockEnable(uint32_t gpio_base);
extern void GPIO_PinInit(uint32_t gpio_base, uint8_t pin, int mode);
extern void GPIO_PinWrite(uint32_t gpio_base, uint8_t pin, int state);
extern void Motor_Init(void* motor);
extern void Motor_SetDirection(void* motor, int direction);
extern void Motor_SetSpeed(void* motor, uint8_t speed);
extern void Ultrasonic_Init(void* sensor);
extern uint32_t Ultrasonic_MeasureDistance(void* sensor);
extern void Bluetooth_Init(void* bt);
extern bool Bluetooth_IsCommandReceived(void* bt);
extern int Bluetooth_GetCommand(void* bt);
extern void Bluetooth_SendMessage(void* bt, const char* message);
extern bool DetectParkingSpace(void* leftSensor, void* rightSensor, void* slot);
extern void ExecuteParallelParking(void* leftMotor, void* rightMotor, void* frontSensor, void* leftSensor, void* rightSensor);
extern void ExecutePerpendicularParking(void* leftMotor, void* rightMotor, void* frontSensor, void* leftSensor, void* rightSensor);
extern void Timer_Init(uint32_t timer_base, uint32_t prescaler, uint32_t auto_reload);
extern void Timer_Start(uint32_t timer_base);
extern void Timer_Stop(uint32_t timer_base);
extern uint32_t Timer_GetCounter(uint32_t timer_base);

// GPIO constants
#define GPIOA_BASE 0x40020000
#define GPIOB_BASE 0x40020400
#define GPIOC_BASE 0x40020800
#define GPIO_OUTPUT 1
#define GPIO_HIGH 1
#define GPIO_LOW 0

// Timer constants
#define TIM5_BASE 0x40000C00
#define DELAY_TIMER_BASE TIM5_BASE

// LED indicator
#define LED_PORT GPIOC_BASE
#define LED_PIN  13

// Command definitions (must match bluetooth_driver.c)
#define COMMAND_NONE 0
#define COMMAND_START_PARKING 1
#define COMMAND_STOP 2
#define COMMAND_MOVE_FORWARD 3
#define COMMAND_MOVE_BACKWARD 4
#define COMMAND_TURN_LEFT 5
#define COMMAND_TURN_RIGHT 6

// Parking types (must match parking_logic.c)
#define PARALLEL_PARKING 0
#define PERPENDICULAR_PARKING 1

// Motor structure (must match motor_driver.c)
typedef struct {
    uint32_t in1_port;
    uint8_t in1_pin;
    uint32_t in2_port;
    uint8_t in2_pin;
    uint32_t ena_port;
    uint8_t ena_pin;
    uint8_t pwm_channel;
} Motor;

// Ultrasonic sensor structure (must match ultrasonic_driver.c)
typedef struct {
    uint32_t trig_port;
    uint8_t trig_pin;
    uint32_t echo_port;
    uint8_t echo_pin;
} UltrasonicSensor;

// UART configuration structure (must match uart_driver.c)
typedef struct {
    uint32_t uart_base;
    uint32_t tx_port;
    uint8_t tx_pin;
    uint32_t rx_port;
    uint8_t rx_pin;
    uint8_t alt_func;
    int baud_rate;
} UART_Config;

// Bluetooth module structure (must match bluetooth_driver.c)
typedef struct {
    UART_Config uart;
    char command_buffer[20];
    uint8_t buffer_index;
    bool command_ready;
    int last_command;
} BluetoothModule;

// Parking slot structure (must match parking_logic.c)
typedef struct {
    int type;
    uint32_t size;
    uint32_t position;
} ParkingSlot;

// Global variables for car hardware
Motor leftMotor, rightMotor;
UltrasonicSensor frontSensor, leftSensor, rightSensor;
BluetoothModule bluetooth;

// Function to initialize delay timer
void delay_init(void) {
    Timer_Init(DELAY_TIMER_BASE, 16-1, 0xFFFFFFFF); // 16MHz / 16 = 1MHz -> 1μs resolution
}

void delay_us(uint32_t us) {
    // Reset counter
    Timer_Start(DELAY_TIMER_BASE);
    
    // Wait until counter reaches desired value
    while (Timer_GetCounter(DELAY_TIMER_BASE) < us);
    
    // Stop timer
    Timer_Stop(DELAY_TIMER_BASE);
}

void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        delay_us(1000);
    }
}

void SetStatusLED(int state) {
    GPIO_PinWrite(LED_PORT, LED_PIN, state);
}

void BlinkStatusLED(uint8_t times) {
    for (uint8_t i = 0; i < times; i++) {
        SetStatusLED(GPIO_HIGH);
        delay_ms(200);
        SetStatusLED(GPIO_LOW);
        delay_ms(200);
    }
}

void SystemInit(void) {
    // Enable HSI (High-Speed Internal) oscillator - 16 MHz
    // RCC_CR register: enable HSI
    *(volatile uint32_t*)(0x40023800 + 0x00) |= (1 << 0);
    
    // Wait for HSI to be ready
    while (!(*(volatile uint32_t*)(0x40023800 + 0x00) & (1 << 1)));
    
    // Set HSI as system clock source
    // RCC_CFGR register: set system clock source
    *(volatile uint32_t*)(0x40023800 + 0x08) &= ~0x03; // Clear bits
    *(volatile uint32_t*)(0x40023800 + 0x08) |= 0x00; // HSI as system clock
    
    // Wait for HSI to be used as system clock
    while ((*(volatile uint32_t*)(0x40023800 + 0x08) & 0x0C) != 0x00);
    
    // Initialize delay functions
    delay_init();
}

void HardwareInit(void) {
    // Initialize GPIO for status LED
    GPIO_ClockEnable(LED_PORT);
    GPIO_PinInit(LED_PORT, LED_PIN, GPIO_OUTPUT);
    SetStatusLED(GPIO_LOW);
    
    // Configure left motor
    leftMotor.in1_port = GPIOB_BASE;
    leftMotor.in1_pin = 0;
    leftMotor.in2_port = GPIOB_BASE;
    leftMotor.in2_pin = 1;
    leftMotor.ena_port = GPIOB_BASE;
    leftMotor.ena_pin = 4;
    leftMotor.pwm_channel = 1;
    Motor_Init(&leftMotor);
    
    // Configure right motor
    rightMotor.in1_port = GPIOB_BASE;
    rightMotor.in1_pin = 2;
    rightMotor.in2_port = GPIOB_BASE;
    rightMotor.in2_pin = 3;
    rightMotor.ena_port = GPIOB_BASE;
    rightMotor.ena_pin = 5;
    rightMotor.pwm_channel = 2;
    Motor_Init(&rightMotor);
    
    // Configure sensors
    frontSensor.trig_port = GPIOA_BASE;
    frontSensor.trig_pin = 0;
    frontSensor.echo_port = GPIOA_BASE;
    frontSensor.echo_pin = 1;
    Ultrasonic_Init(&frontSensor);
    
    leftSensor.trig_port = GPIOA_BASE;
    leftSensor.trig_pin = 2;
    leftSensor.echo_port = GPIOA_BASE;
    leftSensor.echo_pin = 3;
    Ultrasonic_Init(&leftSensor);
    
    rightSensor.trig_port = GPIOA_BASE;
    rightSensor.trig_pin = 4;
    rightSensor.echo_port = GPIOA_BASE;
    rightSensor.echo_pin = 5;
    Ultrasonic_Init(&rightSensor);
    
    // Initialize Bluetooth (UART2)
    bluetooth.uart.uart_base = 0x40004400; // USART2_BASE
    bluetooth.uart.tx_port = GPIOA_BASE;
    bluetooth.uart.tx_pin = 2;
    bluetooth.uart.rx_port = GPIOA_BASE;
    bluetooth.uart.rx_pin = 3;
    bluetooth.uart.alt_func = 7; // AF7 for USART2 on GPIOA
    bluetooth.uart.baud_rate = 9600; // UART_BAUD_9600
    Bluetooth_Init(&bluetooth);
    
    // Blink LED to indicate initialization complete
    BlinkStatusLED(3);
}

// Function to be called from the USART2 IRQ handler
void USART2_IRQHandler(void) {
    extern void USART_IRQHandler(BluetoothModule* bt);
    USART_IRQHandler(&bluetooth);
}

int main(void) {
    // Initialize system and hardware
    SystemInit();
    HardwareInit();
    
    // Send startup message
    Bluetooth_SendMessage(&bluetooth, "Self-Parking Car Ready");
    
    // Main loop
    while (1) {
        // Check for Bluetooth commands
        if (Bluetooth_IsCommandReceived(&bluetooth)) {
            int cmd = Bluetooth_GetCommand(&bluetooth);
            
            switch (cmd) {
                case COMMAND_START_PARKING:
                    // Blink LED to indicate parking process starts
                    BlinkStatusLED(2);
                    
                    // Start the parking sequence
                    ParkingSlot slot;
                    
                    // Move forward and detect parking space
                    Bluetooth_SendMessage(&bluetooth, "Scanning for parking space...");
                    if (DetectParkingSpace(&leftSensor, &rightSensor, &slot)) {
                        // Execute parking maneuver based on detected slot
                        if (slot.type == PARALLEL_PARKING) {
                            Bluetooth_SendMessage(&bluetooth, "Parallel parking slot detected");
                            ExecuteParallelParking(&leftMotor, &rightMotor, 
                                                &frontSensor, &leftSensor, &rightSensor);
                        } else {
                            Bluetooth_SendMessage(&bluetooth, "Perpendicular parking slot detected");
                            ExecutePerpendicularParking(&leftMotor, &rightMotor,
                                                    &frontSensor, &leftSensor, &rightSensor);
                        }
                        
                        // Send completion status via Bluetooth
                        Bluetooth_SendMessage(&bluetooth, "Parking completed");
                        BlinkStatusLED(4); // Signal successful parking
                    } else {
                        // No suitable parking space found
                        Bluetooth_SendMessage(&bluetooth, "No parking space found");
                        BlinkStatusLED(1); // Signal failure
                    }
                    break;
                    
                case COMMAND_STOP:
                    // Emergency stop
                    Motor_SetDirection(&leftMotor, 2); // MOTOR_STOP
                    Motor_SetDirection(&rightMotor, 2); // MOTOR_STOP
                    Bluetooth_SendMessage(&bluetooth, "Emergency stop");
                    break;
                    
                case COMMAND_MOVE_FORWARD:
                    // Manual forward control
                    Motor_SetDirection(&leftMotor, 0); // MOTOR_FORWARD
                    Motor_SetDirection(&rightMotor, 0); // MOTOR_FORWARD
                    Motor_SetSpeed(&leftMotor, 50);
                    Motor_SetSpeed(&rightMotor, 50);
                    Bluetooth_SendMessage(&bluetooth, "Moving forward");
                    break;
                    
                case COMMAND_MOVE_BACKWARD:
                    // Manual backward control
                    Motor_SetDirection(&leftMotor, 1); // MOTOR_BACKWARD
                    Motor_SetDirection(&rightMotor, 1); // MOTOR_BACKWARD
                    Motor_SetSpeed(&leftMotor, 50);
                    Motor_SetSpeed(&rightMotor, 50);
                    Bluetooth_SendMessage(&bluetooth, "Moving backward");
                    break;
                    
                case COMMAND_TURN_LEFT:
                    // Manual left turn
                    Motor_SetDirection(&leftMotor, 1); // MOTOR_BACKWARD
                    Motor_SetDirection(&rightMotor, 0); // MOTOR_FORWARD
                    Motor_SetSpeed(&leftMotor, 40);
                    Motor_SetSpeed(&rightMotor, 40);
                    Bluetooth_SendMessage(&bluetooth, "Turning left");
                    break;
                    
                case COMMAND_TURN_RIGHT:
                    // Manual right turn
                    Motor_SetDirection(&leftMotor, 0); // MOTOR_FORWARD
                    Motor_SetDirection(&rightMotor, 1); // MOTOR_BACKWARD
                    Motor_SetSpeed(&leftMotor, 40);
                    Motor_SetSpeed(&rightMotor, 40);
                    Bluetooth_SendMessage(&bluetooth, "Turning right");
                    break;
                    
                default:
                    break;
            }
        }
        
        // Obstacle detection - safety feature
        if (Ultrasonic_MeasureDistance(&frontSensor) < 10) {
            // Emergency stop if obstacle is too close
            Motor_SetDirection(&leftMotor, 2); // MOTOR_STOP
            Motor_SetDirection(&rightMotor, 2); // MOTOR_STOP
            // Add notification about emergency stop
            Bluetooth_SendMessage(&bluetooth, "Emergency stop: Obstacle detected");
            // Visual indicator
            SetStatusLED(GPIO_HIGH);
            delay_ms(500);
            SetStatusLED(GPIO_LOW);
        }
    }
}