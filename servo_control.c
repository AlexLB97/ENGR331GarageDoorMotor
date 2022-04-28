/**
 * Alex Bourdage
 * Sophie Woessner
 * 
 * Module for controlling the servo motor
 */

#include "servo_control.h"

#include <stdint.h>

#include "global_config_info.h"
#include "lab_gpio.h"
#include "lab_timers.h"
#include "stm32f407xx.h"

/* Preprocessor Definitions */
#define SERVO_PIN 8
#define SERVO_AF_NUM 2
#define SERVO_PWM_FREQ_HZ 50
#define PSC_FREQ_HZ 180000
#define POSITION_0_COMPARE_VAL 88
#define MAX_ANGLE 180

/* File Scope Variables */
static timer_t servo_timer;
static uint32_t servo_angle = 0;

/* Static Function Declarations */
static void servo_timer_cb(void);
static void servo_set_angle(uint32_t angle);

/*Function Definitions */

static void servo_timer_cb(void)
{
    servo_angle++;
    if (servo_angle == 1)
    {
        timer_change_timer_period(&servo_timer, 10);
    }
    else if (servo_angle > MAX_ANGLE)
    {
        servo_angle = 0;
        timer_change_timer_period(&servo_timer, 2000);
    }

    servo_set_angle(servo_angle);
}


static void servo_pwm_init(void)
{
    // Enable clock to GPIOD
    gpio_clock_enable(RCC_AHB1ENR_GPIOCEN_Pos);

    // Enable clock to TIM4
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // Put pin PD13 in alternate function mode
    gpio_pin_set_mode(GPIOC, GPIO_CREATE_MODE_MASK(SERVO_PIN, GPIO_MODE_AF));

    // Connect pin in AFR
    GPIOC->AFR[SERVO_PIN / 8] |= (SERVO_AF_NUM << ((SERVO_PIN % 8) * 4));

    // Set timer 4 channel 4 as an output
    TIM3->CCMR2 &= ~(0x2u << 0);
    
    // Set polarity to active high
    TIM3->CCER &= ~(0x1u << 9);
    
    // Set PWM to mode 1
    TIM3->CCMR2 &= ~(0x7u << 4);
    TIM3->CCMR2 |= (0x6u << 4);
    
    // Set the PSC such that we have 1 degree resolution (180000 Hz)
    TIM3->PSC = (SYSCLK_FREQ_HZ / PSC_FREQ_HZ);
    
    // Set the ARR to set the period of the PWM
    TIM3->ARR = ((SYSCLK_FREQ_HZ / (TIM3->PSC + 1)) / SERVO_PWM_FREQ_HZ) - 1;

    TIM3->CCR3 = POSITION_0_COMPARE_VAL; // Set initial duty cycle to 1ms pulse length for position 0
    
    // Set output preload enable bit
    TIM3->CCMR2 |= (1 << 3);

    // Set the auto-preload enable bit
    TIM3->CR1 |= (1 << 7);
    
    // Clear the alignment and direction bits.
    TIM3->CR1 &= ~(0x7u << 4);
    
    // Enable CC3 compare
    TIM3->CCER |= (1 << 8);
    
    // Enable the counter
    TIM3->CR1 |= TIM_CR1_CEN;
}

static void servo_set_angle(uint32_t angle)
{
    // Each tick has a resolution of 1 degree due to PSC value
    // 0 degrees is at 180 ticks and 180 degrees is at 360 ticks
    // 180 degrees is maximum movement

    if (angle > MAX_ANGLE)
    {
        angle = MAX_ANGLE;
    }

    TIM3->CCR3 = POSITION_0_COMPARE_VAL + (angle * 2);
}

/**
 * @brief Function for initializing the pins and PWWM associated with the servo
 * motor.
 */
void servo_init(void)
{
    servo_pwm_init();
    timer6_delay(100);
    timer_create_timer(&servo_timer, true, 10, servo_timer_cb);
    timer_start_timer(&servo_timer);
}