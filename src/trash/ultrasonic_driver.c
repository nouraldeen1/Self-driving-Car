#include <stdint.h>

// External GPIO and Timer functions (would be linked at compile time)
extern void GPIO_ClockEnable(uint32_t gpio_base);
extern void GPIO_PinInit(uint32_t gpio_base, uint8_t pin, int mode);
extern void GPIO_PinWrite(uint32_t gpio_base, uint8_t pin, int state);
extern int GPIO_PinRead(uint32_t gpio_base, uint8_t pin);
extern void Timer_Init(uint32_t timer_base, uint32_t prescaler, uint32_t auto_reload);
extern void Timer_Start(uint32_t timer_base);
extern uint32_t Timer_GetCounter(uint32_t timer_base);

// GPIO pin modes and states
#define GPIO_INPUT 0
#define GPIO_OUTPUT 1
#define GPIO_LOW 0
#define GPIO_HIGH 1

// Timer base address
#define TIM2_BASE 0x40000000
#define SYSTEM_CLOCK 16000000 // 16 MHz clock assumed

// SysTick definitions for delay
#define SYSTICK_BASE 0xE000E010
#define SYSTICK_CTRL_OFFSET 0x00
#define SYSTICK_LOAD_OFFSET 0x04
#define SYSTICK_VAL_OFFSET  0x08

typedef struct {
    uint32_t trig_port;
    uint8_t trig_pin;
    uint32_t echo_port;
    uint8_t echo_pin;
} UltrasonicSensor;

void delay_us(uint32_t us) {
    // Configure SysTick for microsecond delay
    *(volatile uint32_t*)(SYSTICK_BASE + SYSTICK_LOAD_OFFSET) = (SYSTEM_CLOCK / 1000000) * us - 1;
    *(volatile uint32_t*)(SYSTICK_BASE + SYSTICK_VAL_OFFSET) = 0; // Clear current value
    *(volatile uint32_t*)(SYSTICK_BASE + SYSTICK_CTRL_OFFSET) = 0x05; // Enable SysTick without interrupt
    
    // Wait until the counter reaches zero
    while((*(volatile uint32_t*)(SYSTICK_BASE + SYSTICK_CTRL_OFFSET) & 0x10000) == 0);
    
    // Disable SysTick
    *(volatile uint32_t*)(SYSTICK_BASE + SYSTICK_CTRL_OFFSET) = 0;
}

void Ultrasonic_Init(UltrasonicSensor* sensor) {
    // Enable GPIO clocks
    GPIO_ClockEnable(sensor->trig_port);
    GPIO_ClockEnable(sensor->echo_port);
    
    // Configure trigger pin as output
    GPIO_PinInit(sensor->trig_port, sensor->trig_pin, GPIO_OUTPUT);
    
    // Configure echo pin as input
    GPIO_PinInit(sensor->echo_port, sensor->echo_pin, GPIO_INPUT);
    
    // Initialize timer for pulse measurement
    static uint8_t timer_initialized = 0;
    if (!timer_initialized) {
        // Configure timer for microsecond resolution
        Timer_Init(TIM2_BASE, (SYSTEM_CLOCK / 1000000) - 1, 0xFFFFFFFF); // 1 μs resolution
        timer_initialized = 1;
    }
}

uint32_t Ultrasonic_MeasureDistance(UltrasonicSensor* sensor) {
    // Generate 10us trigger pulse
    GPIO_PinWrite(sensor->trig_port, sensor->trig_pin, GPIO_LOW);
    delay_us(2);
    GPIO_PinWrite(sensor->trig_port, sensor->trig_pin, GPIO_HIGH);
    delay_us(10);
    GPIO_PinWrite(sensor->trig_port, sensor->trig_pin, GPIO_LOW);
    
    // Wait for echo to go high
    uint32_t timeout = 0;
    while(GPIO_PinRead(sensor->echo_port, sensor->echo_pin) == GPIO_LOW) {
        timeout++;
        if (timeout > 30000) return 0; // Timeout if no response
    }
    
    // Start timer
    Timer_Start(TIM2_BASE);
    
    // Wait for echo to go low or timeout (max distance)
    timeout = 0;
    while(GPIO_PinRead(sensor->echo_port, sensor->echo_pin) == GPIO_HIGH) {
        timeout++;
        if (timeout > 30000) break; // Timeout if echo stuck high
    }
    
    // Get pulse duration in microseconds
    uint32_t duration = Timer_GetCounter(TIM2_BASE);
    
    // Calculate distance in cm: duration (μs) / 58 = distance (cm)
    return duration / 58;
}