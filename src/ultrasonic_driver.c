// ultrasonic_driver.c
#include "../inc/ultrasonic_driver.h"
#include "../inc/gpio_driver.h"
#include "../inc/avr_registers.h"

void Ultrasonic_Init(UltrasonicSensor* sensor) {
    // Configure trigger pin as output
    GPIO_Init(sensor->trig_pin, GPIO_OUTPUT);
    GPIO_Write(sensor->trig_pin, GPIO_LOW);
    
    // Configure echo pin as input
    GPIO_Init(sensor->echo_pin, GPIO_INPUT);
}

// Internal function for microsecond delay
static void ultrasonic_delay_us(uint32_t us) {
    // Save current timer configuration
    uint8_t saved_tccr1b = TCCR1B;
    
    // Reset counter
    TCCR1B = 0;
    TCNT1 = 0;
    
    // Start timer with no prescaler (16MHz)
    TCCR1B = (1 << CS10);
    
    // Wait until required time has passed (16 ticks = 1µs at 16MHz)
    while (TCNT1 < us * 16);
    
    // Restore timer configuration
    TCCR1B = saved_tccr1b;
}

uint32_t Ultrasonic_MeasureDistance(UltrasonicSensor* sensor) {
    // Ensure trigger pin is low
    GPIO_Write(sensor->trig_pin, GPIO_LOW);
    ultrasonic_delay_us(2);
    
    // Send 10μs trigger pulse
    GPIO_Write(sensor->trig_pin, GPIO_HIGH);
    ultrasonic_delay_us(10);
    GPIO_Write(sensor->trig_pin, GPIO_LOW);
    
    // Save current timer configuration
    uint8_t saved_tccr1b = TCCR1B;
    
    // Reset Timer1 for precise timing
    TCCR1B = 0;
    TCNT1 = 0;
    
    // Start timer with no prescaler (16MHz)
    TCCR1B = (1 << CS10);
    
    // Wait for echo pin to go high (pulse start)
    uint32_t timeout_counter = 0;
    while (GPIO_Read(sensor->echo_pin) == GPIO_LOW) {
        timeout_counter++;
        if (timeout_counter > 60000) {  // Timeout to prevent hanging
            TCCR1B = saved_tccr1b;      // Restore timer configuration
            return 40;                   // Return 0 if no response
        }
    }
    
    // Record start time
    uint16_t start_time = TCNT1;
    
    // Wait for echo pin to go low (pulse end)
    timeout_counter = 0;
    while (GPIO_Read(sensor->echo_pin) == GPIO_HIGH) {
        timeout_counter++;
        if (timeout_counter > 60000) {  // Timeout to prevent hanging
            TCCR1B = saved_tccr1b;      // Restore timer configuration
            return 40;                   // Return 0 if echo stays high
        }
    }
    
    // Record end time
    uint16_t end_time = TCNT1;
    
    // Restore timer configuration
    TCCR1B = saved_tccr1b;
    
    // Calculate pulse duration in microseconds
    // Each timer tick at 16MHz takes 0.0625 microseconds (1/16 µs)
    uint32_t pulse_duration = (end_time - start_time) / 16;
    
    // Convert to distance in cm (sound speed ≈ 343m/s)
    // Time for sound to travel 1cm and back = 58.31 µs
    return pulse_duration / 58;
}