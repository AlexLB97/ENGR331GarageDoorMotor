#include "lab_pwm.h"

#include "stm32f4xx.h"
#include "stm32f407xx.h"

#include "lab_gpio.h"

#define SYSCLK_FREQ 16000000


void init_pwm_pin_PD15(void)
{
    // Enable clock to GPIOD
    gpio_clock_enable(RCC_AHB1ENR_GPIODEN_Pos);

    // Enable clock to TIM4
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    // Put pin PD13 in alternate function mode
    gpio_pin_set_mode(GPIOD, GPIO_MODER_MODER15_Pos, GPIO_MODER_MODER15_1);

    // Connect pin PB6 to TIM4
    GPIOD->AFR[1] |= (0x2u << 28);

    // Set timer channel as an output
    TIM4->CCMR2 &= ~(0x3u << 8);
    
    // Set polarity to active high
    TIM4->CCER &= ~(0x1u << 13);
    
    // Set PWM to mode 1
    TIM4->CCMR2 &= ~(0x7u << 12);
    TIM4->CCMR2 |= (0x6u << 12);
    
    // Load the prescaler such that PWM frequencies as low as 1 HZ can be reached
    TIM4->PSC = 244;
    
    // Set the ARR to set the period of the PWM
    TIM4->ARR = ((SYSCLK_FREQ / (TIM4->PSC + 1)) / PWM_FREQ_HZ) - 1;
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

void pwm_pd15_change_duty_pct(uint32_t new_duty_pct)
{
    // Set compare register to new value to change duty cycle
    TIM4->CCR4 = TIM4->ARR * new_duty_pct / 100;

    // Reset the counter
    TIM4->CNT = 0;
}
