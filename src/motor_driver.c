#include <stdint.h>

// External GPIO functions (would be linked at compile time)
extern void GPIO_ClockEnable(uint32_t gpio_base);
extern void GPIO_PinInit(uint32_t gpio_base, uint8_t pin, int mode);
extern void GPIO_PinWrite(uint32_t gpio_base, uint8_t pin, int state);
extern void GPIO_PinSetAlternateFunction(uint32_t gpio_base, uint8_t pin, uint8_t alt_func);

// GPIO pin modes and states
#define GPIO_OUTPUT 1
#define GPIO_ALT_FUNCTION 2
#define GPIO_LOW 0
#define GPIO_HIGH 1

// PWM Timer Configuration
#define TIM3_BASE 0x40000400
#define TIM_CR1_OFFSET 0x00
#define TIM_CCMR1_OFFSET 0x18
#define TIM_CCMR2_OFFSET 0x1C
#define TIM_CCER_OFFSET 0x20
#define TIM_CCR1_OFFSET 0x34
#define TIM_CCR2_OFFSET 0x38
#define TIM_CCR3_OFFSET 0x3C
#define TIM_CCR4_OFFSET 0x40
#define TIM_ARR_OFFSET 0x2C

// RCC base and APB1 peripheral clock enable register offset
#define RCC_BASE 0x40023800
#define RCC_APB1ENR_OFFSET 0x40

// Motor direction
typedef enum {
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
    MOTOR_STOP
} MotorDirection;

// Motor structure
typedef struct {
    uint32_t in1_port;
    uint8_t in1_pin;
    uint32_t in2_port;
    uint8_t in2_pin;
    uint32_t ena_port;
    uint8_t ena_pin;
    uint8_t pwm_channel;
} Motor;

void PWM_Init() {
    // Enable TIM3 clock
    uint32_t rcc_apb1enr = *(volatile uint32_t*)(RCC_BASE + RCC_APB1ENR_OFFSET);
    rcc_apb1enr |= (1 << 1); // TIM3 enable
    *(volatile uint32_t*)(RCC_BASE + RCC_APB1ENR_OFFSET) = rcc_apb1enr;
    
    // Set auto-reload value (period)
    *(volatile uint32_t*)(TIM3_BASE + TIM_ARR_OFFSET) = 999; // 1000 steps (0-999)
    
    // Configure Channel 1 as PWM mode 1
    *(volatile uint32_t*)(TIM3_BASE + TIM_CCMR1_OFFSET) |= (6 << 4); // PWM mode 1
    *(volatile uint32_t*)(TIM3_BASE + TIM_CCMR1_OFFSET) |= (1 << 3); // Preload enable
    
    // Configure Channel 2 as PWM mode 1
    *(volatile uint32_t*)(TIM3_BASE + TIM_CCMR1_OFFSET) |= (6 << 12); // PWM mode 1
    *(volatile uint32_t*)(TIM3_BASE + TIM_CCMR1_OFFSET) |= (1 << 11); // Preload enable
    
    // Configure Channel 3 as PWM mode 1
    *(volatile uint32_t*)(TIM3_BASE + TIM_CCMR2_OFFSET) |= (6 << 4); // PWM mode 1
    *(volatile uint32_t*)(TIM3_BASE + TIM_CCMR2_OFFSET) |= (1 << 3); // Preload enable
    
    // Configure Channel 4 as PWM mode 1
    *(volatile uint32_t*)(TIM3_BASE + TIM_CCMR2_OFFSET) |= (6 << 12); // PWM mode 1
    *(volatile uint32_t*)(TIM3_BASE + TIM_CCMR2_OFFSET) |= (1 << 11); // Preload enable
    
    // Enable all channels
    *(volatile uint32_t*)(TIM3_BASE + TIM_CCER_OFFSET) |= 0x1111; // Enable CH1, CH2, CH3, CH4
    
    // Start timer
    *(volatile uint32_t*)(TIM3_BASE + TIM_CR1_OFFSET) |= 0x01;
}

void Motor_Init(Motor* motor) {
    // Enable GPIO clocks
    GPIO_ClockEnable(motor->in1_port);
    GPIO_ClockEnable(motor->in2_port);
    GPIO_ClockEnable(motor->ena_port);
    
    // Configure IN1 and IN2 as output pins
    GPIO_PinInit(motor->in1_port, motor->in1_pin, GPIO_OUTPUT);
    GPIO_PinInit(motor->in2_port, motor->in2_pin, GPIO_OUTPUT);
    
    // Configure ENA as alternate function for PWM
    GPIO_PinInit(motor->ena_port, motor->ena_pin, GPIO_ALT_FUNCTION);
    GPIO_PinSetAlternateFunction(motor->ena_port, motor->ena_pin, 2); // AF2 for TIM3
    
    // Initialize PWM
    static uint8_t pwm_initialized = 0;
    if (!pwm_initialized) {
        PWM_Init();
        pwm_initialized = 1;
    }
    
    // Stop motor initially
    Motor_SetDirection(motor, MOTOR_STOP);
    Motor_SetSpeed(motor, 0);
}

void Motor_SetDirection(Motor* motor, MotorDirection direction) {
    switch (direction) {
        case MOTOR_FORWARD:
            GPIO_PinWrite(motor->in1_port, motor->in1_pin, GPIO_HIGH);
            GPIO_PinWrite(motor->in2_port, motor->in2_pin, GPIO_LOW);
            break;
            
        case MOTOR_BACKWARD:
            GPIO_PinWrite(motor->in1_port, motor->in1_pin, GPIO_LOW);
            GPIO_PinWrite(motor->in2_port, motor->in2_pin, GPIO_HIGH);
            break;
            
        case MOTOR_STOP:
        default:
            GPIO_PinWrite(motor->in1_port, motor->in1_pin, GPIO_LOW);
            GPIO_PinWrite(motor->in2_port, motor->in2_pin, GPIO_LOW);
            break;
    }
}

void Motor_SetSpeed(Motor* motor, uint8_t speed) {
    // Limit speed to 0-100%
    if (speed > 100) speed = 100;
    
    // Convert percentage to PWM value (0-999)
    uint32_t pwm_value = (speed * 999) / 100;
    
    // Set PWM duty cycle based on channel
    switch (motor->pwm_channel) {
        case 1:
            *(volatile uint32_t*)(TIM3_BASE + TIM_CCR1_OFFSET) = pwm_value;
            break;
        case 2:
            *(volatile uint32_t*)(TIM3_BASE + TIM_CCR2_OFFSET) = pwm_value;
            break;
        case 3:
            *(volatile uint32_t*)(TIM3_BASE + TIM_CCR3_OFFSET) = pwm_value;
            break;
        case 4:
            *(volatile uint32_t*)(TIM3_BASE + TIM_CCR4_OFFSET) = pwm_value;
            break;
    }
}