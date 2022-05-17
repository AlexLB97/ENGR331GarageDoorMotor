/**
 * Authors: Alex Bourdage, Sophie Woessner
 */
#ifndef _LAB_GPIO_H
#define _LAB_GPIO_H

#include <stdint.h>

#include "stm32f4xx.h"

#define GREEN_LED    12u
#define ORANGE_LED   13u
#define RED_LED      14u
#define BLUE_LED     15u
#define USER_BTN     0u


#define GPIO_MODE_INPUT    0x0u
#define GPIO_MODE_OUTPUT   0x1u
#define GPIO_MODE_AF       0x2u
#define GPIO_MODE_ANALOG   0x3u

#define GPIO_PUPDR_NO_PULL  0x00u
#define GPIO_PUPDR_PULLUP   0x01u
#define GPIO_PUPDR_PULLDOWN 0x02u

#define GPIO_CREATE_MODE_MASK(pin, mode)   (mode << (pin * 2))
#define GPIO_CREATE_PUPDR_MASK(pin, mode) GPIO_CREATE_MODE_MASK(pin, mode)

/* Function Prototypes */
void gpio_clock_enable(uint32_t pin_position);
void gpio_set_output_type(GPIO_TypeDef *port, uint32_t pin_pos, uint32_t mask);
void gpio_set_pupdr(GPIO_TypeDef *port, uint32_t mask);
void gpio_pin_set(GPIO_TypeDef *port, uint32_t pin);
void gpio_pin_clear(GPIO_TypeDef *port, uint32_t pin);
void gpio_pin_set_mode(GPIO_TypeDef *port, uint32_t mask);
int gpio_pin_get_level(GPIO_TypeDef *port, uint32_t pos);
void gpio_pin_toggle_output(GPIO_TypeDef *port, uint32_t pos);
void user_button_init(void);
void LED_init(void);
void gpio_pin_set_level(GPIO_TypeDef *port, uint32_t pin, uint32_t level);

#endif // _LAB_GPIO_H
