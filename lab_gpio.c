/* Authors: Alex Bourdage, Sophie Woessner
 * Description - Module for initializing and interacting with GPIOs on STM32F407 discovery board.
 */
#include "lab_gpio.h"

#include <stdint.h>
 
#include "lab_interrupts.h"
#include "stm32f4xx.h"


void LED_init(void)
{
	// Enable the clock for GPIO port D
	gpio_clock_enable(RCC_AHB1ENR_GPIODEN_Pos);
	
	// Set LED pins to output
	gpio_pin_set_mode(GPIOD, GPIO_CREATE_MODE_MASK(GREEN_LED, GPIO_MODE_OUTPUT));
	gpio_pin_set_mode(GPIOD, GPIO_CREATE_MODE_MASK(ORANGE_LED, GPIO_MODE_OUTPUT));
	gpio_pin_set_mode(GPIOD, GPIO_CREATE_MODE_MASK(RED_LED, GPIO_MODE_OUTPUT));
	gpio_pin_set_mode(GPIOD, GPIO_CREATE_MODE_MASK(BLUE_LED, GPIO_MODE_OUTPUT));

	
	// Set Pull up to none for all LEDs in use
	gpio_set_pupdr(GPIOD, GPIO_CREATE_PUPDR_MASK(GREEN_LED, GPIO_PUPDR_NO_PULL));
	gpio_set_pupdr(GPIOD, GPIO_CREATE_PUPDR_MASK(ORANGE_LED, GPIO_PUPDR_NO_PULL));
	gpio_set_pupdr(GPIOD, GPIO_CREATE_PUPDR_MASK(RED_LED, GPIO_PUPDR_NO_PULL));
	gpio_set_pupdr(GPIOD, GPIO_CREATE_PUPDR_MASK(BLUE_LED, GPIO_PUPDR_NO_PULL));
	
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
	gpio_pin_set_mode(GPIOA, GPIO_CREATE_MODE_MASK(USER_BTN, GPIO_MODE_INPUT));

	// Set PUPDR for button
	gpio_set_pupdr(GPIOA, GPIO_CREATE_MODE_MASK(USER_BTN, GPIO_PUPDR_PULLDOWN));
    
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

void gpio_set_pupdr(GPIO_TypeDef *port, uint32_t mask)
{
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

void gpio_pin_set_mode(GPIO_TypeDef *port, uint32_t mask)
{
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
