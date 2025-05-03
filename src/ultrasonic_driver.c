// ultrasonic_driver.c
#include "../inc/ultrasonic_driver.h"

// Timer configuration for accurate delay and pulse measurement
#define TIM2_BASE 0x40000000
#define TIM_CR1_OFFSET 0x00
#define TIM_CNT_OFFSET 0x24
#define TIM_PSC_OFFSET 0x28
#define TIM_ARR_OFFSET 0x2C
#define RCC_APB1ENR_OFFSET 0x40

#define SYSTICK_BASE 0xE000E010
#define SYSTICK_CTRL_OFFSET 0x00
#define SYSTICK_LOAD_OFFSET 0x04
#define SYSTICK_VAL_OFFSET 0x08

#define SYSTEM_CLOCK 16000000 // 16 MHz clock assumed

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

void Timer_Init() {
    // Enable Timer2 clock
    uint32_t rcc_apb1enr = *(volatile uint32_t*)(RCC_BASE + RCC_APB1ENR_OFFSET);
    rcc_apb1enr |= (1 << 0); // TIM2 enable
    *(volatile uint32_t*)(RCC_BASE + RCC_APB1ENR_OFFSET) = rcc_apb1enr;
    
    // Configure timer for microsecond resolution
    *(volatile uint32_t*)(TIM2_BASE + TIM_PSC_OFFSET) = (SYSTEM_CLOCK / 1000000) - 1; // 1 μs resolution
    *(volatile uint32_t*)(TIM2_BASE + TIM_ARR_OFFSET) = 0xFFFFFFFF; // Maximum count
}

void Timer_Start() {
    // Reset counter
    *(volatile uint32_t*)(TIM2_BASE + TIM_CNT_OFFSET) = 0;
    
    // Enable timer
    *(volatile uint32_t*)(TIM2_BASE + TIM_CR1_OFFSET) |= 0x01;
}

uint32_t Timer_GetCounter() {
    return *(volatile uint32_t*)(TIM2_BASE + TIM_CNT_OFFSET);
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
    Timer_Init();
}

uint32_t Ultrasonic_MeasureDistance(UltrasonicSensor* sensor) {
    // Generate 10us trigger pulse
    GPIO_PinWrite(sensor->trig_port, sensor->trig_pin, GPIO_LOW);
    delay_us(2);
    GPIO_PinWrite(sensor->trig_port, sensor->trig_pin, GPIO_HIGH);
    delay_us(10);
    GPIO_PinWrite(sensor->trig_port, sensor->trig_pin, GPIO_LOW);
    
    // Wait for echo to go high
    while(GPIO_PinRead(sensor->echo_port, sensor->echo_pin) == GPIO_LOW);
    
    // Start timer
    Timer_Start();
    
    // Wait for echo to go low or timeout (max distance)
    uint32_t timeout = 0;
    while(GPIO_PinRead(sensor->echo_port, sensor->echo_pin) == GPIO_HIGH && timeout < 23200) {
        timeout++;
    }
    
    // Get pulse duration in microseconds
    uint32_t duration = Timer_GetCounter();
    
    // Calculate distance in cm: duration (μs) / 58 = distance (cm)
    // Speed of sound is approximately 343 m/s or 34300 cm/s
    // Time for sound to travel 1 cm and back = 2 / 34300 s = 58.31 μs
    return duration / 58;
}