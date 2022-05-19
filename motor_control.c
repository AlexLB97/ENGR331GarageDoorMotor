/**
 * Alex Bourdage
 * Sophie Woessner
 * 
 * Module to control the opening and closing operations of the DC motor that simulates the 
 * garage door opener
 */

/* Includes */
#include "motor_control.h"

#include <stdbool.h>
#include <stdint.h>

#include "adc.h"
#include "event_queue.h"
#include "global_config_info.h"
#include "lab_gpio.h"
#include "lab_timers.h"
#include "motion_detector.h"
#include "stm32f407xx.h"


/* Preprocessor Definitions */
#define MOTOR_PWM_PIN          BLUE_LED
#define MOTOR_PWM_AF_NUM       2u
#define PWM_FREQ_HZ            150u
#define MOTOR_DUTY_CYCLE_PCT   75
#define SPIN_FORWARD_PIN       RED_LED
#define SPIN_BACKWARDS_PIN     ORANGE_LED
#define FAN_TIMER_PERIOD_MS    1000
#define FAN_ON_THRESH_DEGREES  85
#define FAN_OFF_THRESH_DEGREES (FAN_ON_THRESH_DEGREES - 3)
#define FAN_START_DUTY_PCT     20
#define MAX_SPEED_TEMP         (FAN_ON_THRESH_DEGREES + 10)
#define DUTY_SLOPE             ((100 - FAN_START_DUTY_PCT) / (MAX_SPEED_TEMP - FAN_ON_THRESH_DEGREES))

/* File Scope Variables */
static timer_t periodic_fan_timer;
static bool fan_on = false;

/* Static function declarations */

/**
 * @brief Initializes the PWM for TIM4 Channel 4, which is used to drive the enable
 * signal for the DC motor.
 */
static void motor_pwm_init(void);
/**
 * @brief Function for changing the duty cycle of the motor PWM
 */
static void set_pwm_duty_cycle(uint32_t duty_pct);

/* Functions */

static void motor_pwm_init(void)
{
    // Enable clock to GPIOD
    gpio_clock_enable(RCC_AHB1ENR_GPIODEN_Pos);

    // Enable clock to TIM4
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    // Put pin PD13 in alternate function mode
    gpio_pin_set_mode(GPIOD, GPIO_CREATE_MODE_MASK(MOTOR_PWM_PIN, GPIO_MODE_AF));

    // Connect pin in AFR
    GPIOD->AFR[MOTOR_PWM_PIN / 8] |= (MOTOR_PWM_AF_NUM << ((MOTOR_PWM_PIN % 8) * 4));

    // Set timer 4 channel 4 as an output
    TIM4->CCMR2 &= ~(0x3u << 8);
    
    // Set polarity to active high
    TIM4->CCER &= ~(0x1u << 13);
    
    // Set PWM to mode 1
    TIM4->CCMR2 &= ~(0x7u << 12);
    TIM4->CCMR2 |= (0x6u << 12);
    
    // Load the prescaler such that PWM frequencies as low as 1 HZ can be reached
    TIM4->PSC = 244;
    
    // Set the ARR to set the period of the PWM
    TIM4->ARR = ((SYSCLK_FREQ_HZ / (TIM4->PSC + 1)) / PWM_FREQ_HZ) - 1;

    TIM4->CCR4 = 0; // Set initial duty cycle to 0
    
    // Set output preload enable bit
    TIM4->CCMR2 |= (1 << 11);

    // Set the auto-preload enable bit
    TIM4->CR1 |= (1 << 7);
    
    // Clear the alignment and direction bits.
    TIM4->CR1 &= ~(0x7u << 4);
    

    // Set the compare/capture polarity
    TIM4->CCER |= (1 << 12);
    
    // Enable the counter
    TIM4->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief Helper function for configuring a specific duty cycle for the motor
 */
static void set_pwm_duty_cycle(uint32_t duty_pct)
{
    TIM4->CCR4 = (TIM4->ARR * duty_pct) / 100;
}

// Updates the state of the fan based on occupancy and temperature
static void update_fan_state_cb(void)
{
    if (motion_detector_get_occupancy_state() == GARAGE_OCCUPIED)
    {
        // Get current temp
        int temp = adc_get_temp();
        if (temp >= FAN_ON_THRESH_DEGREES)
        {
            // Temperature is above threshold - turn on fan
            fan_on = true;

            // Scale fan speed based on temperature until it hits max speed.
            uint32_t duty_pct = (uint32_t) (FAN_START_DUTY_PCT + (DUTY_SLOPE * (temp - FAN_ON_THRESH_DEGREES)));
            if (duty_pct > 100)
            {
                // Prevent duty cycle from exceeding 100
                duty_pct = 100;
            }
            set_pwm_duty_cycle(duty_pct);
        }
        else if (temp > FAN_OFF_THRESH_DEGREES && fan_on)
        {
            // Maintain minimum speed until temperature is below the fan on threshold
            // by some margin
            set_pwm_duty_cycle(FAN_START_DUTY_PCT);
        }
        else
        {
            // Turn off fan
            fan_on = false;
            set_pwm_duty_cycle(0);
        }
    }
    else
    {
        // Garage is unoccupied - fan is off regardless of temperature.
        set_pwm_duty_cycle(0);
    }
}

// Function to periodicall update the state of the fan
static void fan_timer_cb(void)
{
    queue_add_event(update_fan_state_cb);
}

void motor_control_init(void)
{
    // Initialize the motor control pin
    motor_pwm_init();

    // Initialize the logic pins for door direction
    	// Enable the clock for GPIO port D
	gpio_clock_enable(RCC_AHB1ENR_GPIODEN_Pos);
	
	// Set LED pins to output
	gpio_pin_set_mode(GPIOD, GPIO_CREATE_MODE_MASK(SPIN_FORWARD_PIN, GPIO_MODE_OUTPUT));
	gpio_pin_set_mode(GPIOD, GPIO_CREATE_MODE_MASK(SPIN_BACKWARDS_PIN, GPIO_MODE_OUTPUT));
	
	// Set pull to none for all LEDs in use
	gpio_set_pupdr(GPIOD, GPIO_CREATE_PUPDR_MASK(SPIN_FORWARD_PIN, GPIO_PUPDR_NO_PULL));
	gpio_set_pupdr(GPIOD, GPIO_CREATE_PUPDR_MASK(SPIN_BACKWARDS_PIN, GPIO_PUPDR_NO_PULL));

    // Create timer to update the fan state
    timer_create_timer(&periodic_fan_timer, true, FAN_TIMER_PERIOD_MS, fan_timer_cb);
    timer_start_timer(&periodic_fan_timer);

    gpio_pin_clear(GPIOD, SPIN_BACKWARDS_PIN);
    gpio_pin_clear(GPIOD, SPIN_FORWARD_PIN);

    // Set logic pin on H-Bridge to be always on. PWM will control speed
    gpio_pin_set(GPIOD, SPIN_FORWARD_PIN);
}
