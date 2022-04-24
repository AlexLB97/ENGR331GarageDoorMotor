/* Authors: Alex Bourdage, Sophie Woessner
 * Description - Module for initializing and interacting with GPIOs on STM32F407 discovery board.
 */
#include "lab_gpio.h"

#include <stdint.h>
 
#include "lab_interrupts.h"
#include "stm32f4xx.h"


#define GREEN_LED 12
#define ORANGE_LED 13
#define RED_LED 14
#define BLUE_LED 15
#define USER_BTN 0


void LED_init(void)
{
	// Enable the clock for GPIO port D
	gpio_clock_enable(RCC_AHB1ENR_GPIODEN_Pos);
	
	// Set LED pins to output
	gpio_pin_set_mode(GPIOD, GPIO_MODER_MODE13_Pos, GPIO_MODER_MODER13_0);
	gpio_pin_set_mode(GPIOD, GPIO_MODER_MODE12_Pos, GPIO_MODER_MODER12_0);
	gpio_pin_set_mode(GPIOD, GPIO_MODER_MODE14_Pos, GPIO_MODER_MODER14_0);
	gpio_pin_set_mode(GPIOD, GPIO_MODER_MODE15_Pos, GPIO_MODER_MODER15_0);
	
	// Set Pull up to none for all LEDs in use
	gpio_set_pupdr(GPIOD, GPIO_PUPDR_PUPD13_Pos, 0x00);
	gpio_set_pupdr(GPIOD, GPIO_PUPDR_PUPD12_Pos, 0x00);
	gpio_set_pupdr(GPIOD, GPIO_PUPDR_PUPD14_Pos, 0x00);
	gpio_set_pupdr(GPIOD, GPIO_PUPDR_PUPD15_Pos, 0x00);
	
	// Set output type to push-pull for all LEDs
	gpio_set_output_type(GPIOD, GPIO_OTYPER_OT13_Pos, 0x00);
	gpio_set_output_type(GPIOD, GPIO_OTYPER_OT12_Pos, 0x00);
	gpio_set_output_type(GPIOD, GPIO_OTYPER_OT14_Pos, 0x00);
}

void user_button_init(void)
{
	// Enable clock for port A
	gpio_clock_enable(RCC_AHB1ENR_GPIOAEN_Pos);

	// Set button to input
	gpio_pin_set_mode(GPIOA, GPIO_MODER_MODE0_Pos, 0x00);

	// Set PUPDR for button
	gpio_set_pupdr(GPIOA, GPIO_PUPDR_PUPD0_Pos,  0x00);
    
    // Enable user button interrupt
    
    // Set ICR Register
    SYSCFG->EXTICR[0] &= ~(0x000Fu);
    EXTI->IMR |= (1 << USER_BTN);
    EXTI->RTSR |= (1 << USER_BTN);
    NVIC_SetPriority(EXTI0_IRQn, 0);
    NVIC_ClearPendingIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(EXTI0_IRQn);
    
}
 
void gpio_clock_enable(uint32_t port_position)
{
	// Set the bit corresponding to the clock enable for the port of interest
	RCC->AHB1ENR |= (1U << port_position);
}
 
void gpio_set_output_type(GPIO_TypeDef *port, uint32_t pin_pos, uint32_t mask)
{
	// Clear current state
	port->OTYPER &= ~(1U << pin_pos);
	
	// Set new state
	port->OTYPER |= mask;
}

void gpio_set_pupdr(GPIO_TypeDef *port, uint32_t pos, uint32_t mask)
{
	// Clear bits associated with pin
	port->PUPDR &= ~(3U << pos);
	
	// Set pull up mode
	port->PUPDR |= mask;
}

void gpio_pin_set(GPIO_TypeDef *port, uint32_t pin)
{
	// Set bit corresponding to pin in ODR
	port->ODR |= (1U << pin);
}

void gpio_pin_clear(GPIO_TypeDef *port, uint32_t pin)
{
	// Clear bit corresponding to pin in ODR
	port->ODR &= ~(1 << pin);
}

void gpio_pin_set_mode(GPIO_TypeDef *port, uint32_t pos, uint32_t mask)
{
	// Clear any pre-existing mode
	port->MODER &= ~(3U << pos);
	
	// Set the desired mode
	port->MODER |= mask;
}

int gpio_pin_get_level(GPIO_TypeDef *port, uint32_t pos)
{
	// Check whether an input is high or low.
	if (port->IDR & (1 << pos))
	{
		// Return 1 if input is high
		return 1;
	}
	else
	{
		// Return 0 if input is low
		return 0;
	}
}

// Toggle an output pin
void gpio_pin_toggle_output(GPIO_TypeDef *port, uint32_t pos)
{
	// Use XOR to toggle pin state.
	port->ODR ^= (1U << pos);
}

void gpio_pin_set_level(GPIO_TypeDef *port, uint32_t pin, uint32_t level)
{
	if (level == 1)
	{
		port->ODR |= (level << pin);
	}
	else
	{
		port->ODR &= ~(1 << pin);
	}
}
