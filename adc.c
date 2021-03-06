/**
 * Author: Alex Bourdage
 * 
 * Goal: Module for managing ADC inputs for temperature and photoresistor.
 * Due to fluctuations in the readings from the sensors, this module takes 
 * measurements every second and records an average over the last 10 readings. 
 * It is the average reading that the API exposes to other modules.
 */

#include "adc.h"

#include <string.h>
#include <stdio.h>

#include "stm32f407xx.h"
#include "stm32f4xx.h"

#include "event_queue.h"
#include "lab_timers.h"
#include "LCD.h"
#include "lcd_layout.h"

/* Preprocessor Definitions */
#define ADC_INTERVAL_MS 1000
#define NUM_OF_SAMPLES 10

/* Static Function Declarations */
static void update_averages(int num_samples_in_array);
static void update_temp_and_light_values_cb(void);
static double get_average(double *data_array, int num_samples);
static void adc_timer_cb(void);

// Enum to streamline indexing into array of averages. Could scale easily to accommodate more 
// ADC-based sensors.
typedef enum adc_channel_t {
    LIGHT_SENSOR = 0,
    TEMP_SENSOR,
    NUM_SENSORS
} adc_channel_t;

/* File Scope Variables */
extern void ADC_IRQHandler(void);
static double adc_data[NUM_SENSORS][NUM_OF_SAMPLES] = {0};
static int current_channel = 0;
static timer_t adc_timer;
static double averages[NUM_SENSORS];
static char temperature[5];


/* Function Definitions */

// Getter method that returns teh current average light measurement.
double adc_get_light_level(void)
{
    return averages[LIGHT_SENSOR];
}

// Getter method that returns the current average temperature measurement
int adc_get_temp(void)
{
    double temp_c = ((averages[TEMP_SENSOR] - 0.5) * 100);
    int temp_f = (int)((9 * temp_c) / 5 + 32.0);
    return temp_f;
}

// This function updates the LCD with the current temperature. It is only ever called from the event queue
// in the main loop.
static void update_temp_and_light_values_cb(void)
{
    int temp_f = adc_get_temp();
    sprintf(temperature, "%*dF", 3, temp_f);
    LCD_write_string_at_addr(temperature, ON_WHILE_WRITING, TEMP_START_ADDR, (int)strlen(temperature));
}

// This function calculates the average of an array of ADC measurments.
static double get_average(double *data_array, int num_samples)
{
    double sum = 0;
    for (int i = 0; i < num_samples; i++)
    {
        sum += data_array[i];
    }

    return sum / num_samples;
}

static void update_averages(int num_samples_in_array)
{
    // Update averages
    averages[LIGHT_SENSOR] = get_average(adc_data[LIGHT_SENSOR], num_samples_in_array);
    averages[TEMP_SENSOR] = get_average(adc_data[TEMP_SENSOR], num_samples_in_array);
}

static void adc_timer_cb(void)
{
    // Turn ADC On to start conversion
    ADC1->CR2 |= ADC_CR2_ADON;
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

/**
 * Function for initializing continuous ADC conversion on PA1 and PA2
 */

void adc_init(void)
{
    // Configure PA1 for alternate function mode

    // Enable clock to GPIOA
    if (!(RCC->AHB1ENR & (1 << RCC_AHB1ENR_GPIOAEN_Pos)))
    {
        RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOAEN_Pos);
    }

    // Enable clock for ADC
    RCC->APB2ENR |= (0x01 << RCC_APB2ENR_ADC1EN_Pos);

    // Configure pin PA1 in analog mode
    GPIOA->MODER |= (0x03 << GPIO_MODER_MODER1_Pos);

    // Configure PA2 as analog input
    GPIOA->MODER |= (0x03 << GPIO_MODER_MODER2_Pos);

    // Set 10 bit resolution
    ADC1->CR1 |= (ADC_CR1_RES_0);

    // Set sampling rate for ADC1_IN1 (PA1) to 84 cycles
    ADC1->SMPR2 |= ADC_SMPR2_SMP1_2;
    
    // Set sampling rate for ADCIN2 to 84 cycles
     ADC1->SMPR2 |= ADC_SMPR2_SMP2_2;

    // Specify channel 1 of conversion 1 in SQR3 Register
    ADC1->SQR3 |= 0x01 << ADC_SQR3_SQ1_Pos;
    
    // Set second conversion in regular sequence to happen on channel 2 (PA2)
     ADC1->SQR3 |= (0x02 << ADC_SQR3_SQ2_Pos);
    
    // Set number of conversions in L register
     ADC1->SQR1 |= (0x01 << ADC_SQR1_L_Pos);
     
     // Enable scan mode
     ADC1->CR1 |= (0x01 << ADC_CR1_SCAN_Pos);
    
    // Interrupt after each conversion in sequence
     ADC1->CR2 |= (0x01 << ADC_CR2_EOCS_Pos);

    // Enable ADC EOC interrupt
    ADC1->CR1 |= ADC_CR1_EOCIE;

    // Turn ADC On to start conversion
    ADC1->CR2 |= ADC_CR2_ADON;
    ADC1->CR2 |= ADC_CR2_SWSTART;

    // Let NVIC handle interrupts
    __enable_irq();
    NVIC_ClearPendingIRQ(ADC_IRQn);
    NVIC_SetPriority(ADC_IRQn, 0);
    NVIC_EnableIRQ(ADC_IRQn);

    // Create timer for periodic ADC
    timer_create_timer(&adc_timer, true, ADC_INTERVAL_MS, adc_timer_cb);

    // Start Periodic Timer
    timer_start_timer(&adc_timer);
}

// ADC interrupt handler. This is triggered after the end of each conversion in the sequence
void ADC_IRQHandler(void)
{
    static int next_index_to_write = 0;
    static bool sample_array_full = false;

    // Clear any pending ADC IRQs
    NVIC_ClearPendingIRQ(ADC_IRQn);
    
    // Convert ADC reading to to 1000 * the input voltage to preserve 3 digits of precision
    uint32_t adc_data_int = (ADC1->DR * 3000 / 1024);  // Reading this register clears the interrupt flag
    
    // Add the measured voltage to the next slot in the array to hold measurements for each sensor.
    adc_data[current_channel][next_index_to_write] = (double)adc_data_int / 1000.0;

    if (current_channel == TEMP_SENSOR)
    {
        // Both sensors now have another value in array, update averages and increment data index
        // Continue to loop through the array, returning to the beginning when full
        next_index_to_write = (next_index_to_write + 1) % NUM_OF_SAMPLES;
        if (next_index_to_write == 0)
        {
            // Array is now full. From now on,
            // all averages will be based on 10 measurements
            sample_array_full = true;
        }

        if (sample_array_full)
        {
            // Calculate averages based on full array
            update_averages(NUM_OF_SAMPLES);
        }
        else
        {
            // Calculate based on current values (used when array is not full yet)
            update_averages(next_index_to_write);
        }
    }
    
    // Increment current channel and take mod(2) to track which sensor is being measured
    current_channel = (current_channel + 1) % NUM_SENSORS;

    // Queue an event to update the LCD with the new temperature 
    queue_add_event(update_temp_and_light_values_cb);
    
    // Clear interrupt flag
    ADC1->SR &= ~(1u << ADC_SR_EOC_Pos);
}
