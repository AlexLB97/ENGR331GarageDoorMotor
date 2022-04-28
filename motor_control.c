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

#include "global_config_info.h"
#include "lab_gpio.h"
#include "lab_timers.h"
#include "stm32f407xx.h"
#include "system_events.h"


/* Preprocessor Definitions */
#define MOTOR_PWM_PIN          BLUE_LED
#define MOTOR_PWM_AF_NUM       2u
#define PWM_FREQ_HZ            150u
#define MOTOR_DUTY_CYCLE_PCT   75
#define OPEN_DOOR_PIN          RED_LED
#define CLOSE_DOOR_PIN         GREEN_LED
#define DOOR_TIMER_PERIOD_MS   10000

/* Type Definitions */

/* File Scope Variables */
static door_state_t previous_state = DOOR_STATE_CLOSING;
static door_state_t current_state = DOOR_STATE_STOPPED;
static timer_t door_timer;

/* Static function declarations */

/**
 * @brief Initializes the PWM for TIM4 Channel 4, which is used to drive the enable
 * signal for the DC motor.
 */
static void motor_pwm_init(void);
static void stop_door(void);
static void open_door(void);
static void close_door(void);
static void set_initial_door_state(void);
static void door_timer_cb(void);

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
    TIM4->CNT = 0;
}

door_state_t motor_control_get_current_state(void)
{
    return current_state;
}

/**
 * @brief Function that manages the sequence of state for the door and returns the next state based
 * on current and the event that occurred.
 *
 * @param [in] current_state  The current state of the door.
 * @param [in] event          Event that occurred in the system
 * @param [out] state         The next state of the system
 */
door_state_t motor_control_get_next_state(void)
{
    // The sequence of events should be STOPPED->OPENING->STOPPED->CLOSING->REPEAT
    switch (current_state)
    {
        case DOOR_STATE_OPENING:
            return DOOR_STATE_STOPPED;
        
        case DOOR_STATE_CLOSING:
            return DOOR_STATE_STOPPED;
        
        case DOOR_STATE_UNKNOWN:
            return DOOR_STATE_STOPPED;

        case DOOR_STATE_STOPPED:
            if (previous_state == DOOR_STATE_CLOSING)
            {
                return DOOR_STATE_OPENING;
            }
            else
            {
                return DOOR_STATE_CLOSING;
            }
    }
}

/**
 * @brief Stops the door after a set period of time. Will eventually be replaced with
 * a pressure switch.
 */
static void door_timer_cb(void)
{
    motor_control_handle_state_transition(DOOR_STATE_STOPPED);
}

/**
 * @brief Configures the H-bridge and pwm to stop the door
 */
static void stop_door(void)
{
    // Set PWM duty cycle to 0
    set_pwm_duty_cycle(0);

    // Set both H-Bridge logic pins to low
    gpio_pin_set_level(GPIOD, OPEN_DOOR_PIN, false);
    gpio_pin_set_level(GPIOD, CLOSE_DOOR_PIN, false);
    timer_stop_timer(&door_timer);
}

/**
 * @brief Configures the H-bridge and pwm to open the door. Currently runs motor for a fixed duration,
 * but should be adapted later to run until a pressure switch indicates that the door is open.
 */
static void open_door(void)
{
    // Always call stop_door() first to ensure there is no short between open and close pins on H-Bridge
    gpio_pin_set_level(GPIOD, OPEN_DOOR_PIN, false);
    gpio_pin_set_level(GPIOD, CLOSE_DOOR_PIN, false);
    set_pwm_duty_cycle(MOTOR_DUTY_CYCLE_PCT);
    gpio_pin_set_level(GPIOD, OPEN_DOOR_PIN, true);
    timer_start_timer(&door_timer);
}

/**
 * @brief Configures the H-bridge and pwm to close the door. Currently runs motor for a fixed duration,
 * but should be adapted later to close until a pressure switch indicates that the door is closed.
 */
static void close_door(void)
{
    // Always call stop_door() first to ensure there is no short between open and close pins on H-Bridge
    gpio_pin_set_level(GPIOD, OPEN_DOOR_PIN, false);
    gpio_pin_set_level(GPIOD, CLOSE_DOOR_PIN, false);
    set_pwm_duty_cycle(MOTOR_DUTY_CYCLE_PCT);
    gpio_pin_set_level(GPIOD, CLOSE_DOOR_PIN, true);
    timer_start_timer(&door_timer);
}

void motor_control_handle_state_transition(door_state_t next_state)
{
    previous_state = current_state;
    current_state = next_state;

    switch (current_state)
    {
        case DOOR_STATE_STOPPED:
        {
            stop_door();
            break;
        }

        case DOOR_STATE_OPENING:
        {
            open_door();
            break;

        }

        case DOOR_STATE_CLOSING:
        {
            close_door();
            break;
        }

        case DOOR_STATE_UNKNOWN:
        {
            close_door();
            break;
        }
    }
}

static void set_initial_door_state(void)
{
    // TODO: Once pressure switches are implemented, create logic here to determine starting state of door.
}

void motor_control_init(void)
{
    // Initialize the motor control pin
    motor_pwm_init();

    // Initialize the logic pins for door direction
    	// Enable the clock for GPIO port D
	gpio_clock_enable(RCC_AHB1ENR_GPIODEN_Pos);
	
	// Set LED pins to output
	gpio_pin_set_mode(GPIOD, GPIO_CREATE_MODE_MASK(OPEN_DOOR_PIN, GPIO_MODE_OUTPUT));
	gpio_pin_set_mode(GPIOD, GPIO_CREATE_MODE_MASK(CLOSE_DOOR_PIN, GPIO_MODE_OUTPUT));
	
	// Set pull to none for all LEDs in use
	gpio_set_pupdr(GPIOD, GPIO_CREATE_PUPDR_MASK(OPEN_DOOR_PIN, GPIO_PUPDR_NO_PULL));
	gpio_set_pupdr(GPIOD, GPIO_CREATE_PUPDR_MASK(CLOSE_DOOR_PIN, GPIO_PUPDR_NO_PULL));

    // (NOT IMPLEMENTED YET: Get door state based on pressure switches)
    set_initial_door_state();

    timer_create_timer(&door_timer, false, DOOR_TIMER_PERIOD_MS, door_timer_cb);
}
