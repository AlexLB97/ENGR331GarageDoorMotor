#ifndef _LAB_GPIO_H
#define _LAB_GPIO_H

#include <stdint.h>

#include "stm32f4xx.h"

#define GREEN_LED 12
#define ORANGE_LED 13
#define RED_LED 14
#define BLUE_LED 15
#define USER_BTN 0

/* Function Prototypes */
void gpio_clock_enable(uint32_t pin_position);
void gpio_set_output_type(GPIO_TypeDef *port, uint32_t pin_pos, uint32_t mask);
void gpio_set_pupdr(GPIO_TypeDef *port, uint32_t pos, uint32_t mask);
void gpio_pin_set(GPIO_TypeDef *port, uint32_t pin);
void gpio_pin_clear(GPIO_TypeDef *port, uint32_t pin);
void gpio_pin_set_mode(GPIO_TypeDef *port, uint32_t pos, uint32_t mask);
int gpio_pin_get_level(GPIO_TypeDef *port, uint32_t pos);
void gpio_pin_toggle_output(GPIO_TypeDef *port, uint32_t pos);
void user_button_init(void);
void LED_init(void);
void gpio_pin_set_level(GPIO_TypeDef *port, uint32_t pin, uint32_t level);

#endif // _LAB_GPIO_H
