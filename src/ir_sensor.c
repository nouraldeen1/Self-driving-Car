// src/ir_sensor.c
#include "../inc/ir_sensor.h"

// ADC Registers
#define ADMUX   (*(volatile uint8_t*)(0x7C))
#define ADCSRA  (*(volatile uint8_t*)(0x7A))
#define ADCSRB  (*(volatile uint8_t*)(0x7B))
#define ADCL    (*(volatile uint8_t*)(0x78))
#define ADCH    (*(volatile uint8_t*)(0x79))

// ADC bits
#define ADEN    7  // ADC Enable
#define ADSC    6  // ADC Start Conversion
#define ADPS0   0  // ADC Prescaler Select Bit 0
#define ADPS1   1  // ADC Prescaler Select Bit 1
#define ADPS2   2  // ADC Prescaler Select Bit 2
#define REFS0   6  // Reference Selection Bit 0

// Get ADC channel from Arduino pin number
uint8_t GetADCChannel(uint8_t pin) {
    // For Arduino Uno:
    // A0 = 14 → ADC0
    // A1 = 15 → ADC1
    // A2 = 16 → ADC2
    // A3 = 17 → ADC3
    // A4 = 18 → ADC4
    // A5 = 19 → ADC5
    
    if (pin >= 14 && pin <= 19) {
        return pin - 14;
    }
    return 0;  // Default to ADC0
}

void IR_Init(IRSensor* sensor) {
    if (sensor->isAnalog) {
        // Set pin as input
        GPIO_Init(sensor->pin, GPIO_INPUT);
        
        // Enable ADC
        ADCSRA |= (1 << ADEN);
        
        // Set ADC clock prescaler to 128 (16MHz/128 = 125kHz)
        ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
        
        // Set reference voltage to AVCC
        ADMUX |= (1 << REFS0);
    } else {
        // For digital IR sensor, set pin as input
        GPIO_Init(sensor->pin, GPIO_INPUT);
    }
}

bool IR_DetectObstacle(IRSensor* sensor) {
    if (sensor->isAnalog) {
        // Get analog reading
        uint16_t value = IR_GetRawValue(sensor);
        return (value < sensor->threshold);  // Lower value typically means closer object
    } else {
        // For digital sensors, read pin directly (typically LOW when obstacle detected)
        return (GPIO_Read(sensor->pin) == GPIO_LOW);
    }
}

uint16_t IR_GetRawValue(IRSensor* sensor) {
    if (!sensor->isAnalog) {
        return GPIO_Read(sensor->pin) == GPIO_HIGH ? 1 : 0;
    }
    
    // Select ADC channel
    uint8_t channel = GetADCChannel(sensor->pin);
    ADMUX = (ADMUX & 0xF0) | channel;
    
    // Start conversion
    ADCSRA |= (1 << ADSC);
    
    // Wait for conversion to complete
    while (ADCSRA & (1 << ADSC));
    
    // Read result
    uint8_t low = ADCL;
    uint8_t high = ADCH;
    
    return (high << 8) | low;
}