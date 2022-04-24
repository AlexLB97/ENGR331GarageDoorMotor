/**
 * Alex Bourdage
 * Sophie Woessner
 * 
 * Module for managing input from the keypad.
 * 
 */

#include <stdbool.h>
#include <stdint.h>

#include "keypad.h"
#include "lab_gpio.h"
#include "lab_timers.h"
#include "stm32f407xx.h"

/* Preprocessor Definitions */
#define KEYPAD_DEBOUNCE_PERIOD_MS 200

/* File scope variables */
static char keypad_array[4][3] = {{'1','2','3'},
                                  {'4','5','6'},
                                  {'7','8','9'},
                                  {'*','0','#'}};

static timer_t keypad_debounce_timer;

static bool button_press_allowed = true;

static char key_pressed = '\0';

/* External Function Declarations */
extern void EXTI1_IRQHandler(void);
extern void EXTI2_IRQHandler(void);
extern void EXTI3_IRQHandler(void);


/* Static function declarations */
static void keypad_debounce_cb(void);



/* Functions */

static void keypad_debounce_cb(void) 
{
    button_press_allowed = true;
}

static void keypad_button_press_handler(uint8_t column)
{  
    uint8_t rows[4] = {R0, R1, R2, R3};
    uint8_t row;
    // Prevent further erroneous button pressed
    button_press_allowed = false;

    // Start debounce timer
    timer_start_timer(&keypad_debounce_timer);

    // Determine which key was pressed
    for (int i = 0; i < 4; i++)
    {
        gpio_pin_clear(GPIOD, rows[i]);
        if (!gpio_pin_get_level(GPIOD, column))
        {
            row = rows[i];
        }
        gpio_pin_set(GPIOD, rows[i]);
    }

    key_pressed = keypad_array[row - 4][column - 1];
}

void EXTI1_IRQHandler(void)
{
    if (button_press_allowed)
    {
        keypad_button_press_handler(C0);
    }    
    EXTI->PR |= 1 << 1;
}

void EXTI2_IRQHandler(void)
{
    if (button_press_allowed)
    {
        keypad_button_press_handler(C1);
    }    
    EXTI->PR |= 1 << 2;
}

void EXTI3_IRQHandler(void)
{
    if (button_press_allowed)
    {
        keypad_button_press_handler(C2);
    }    
    EXTI->PR |= 1 << 3;
}


void keypad_init(void)
{
    // Enable clock for port D
    gpio_clock_enable(RCC_AHB1ENR_GPIODEN_Pos);

    // Configure rows as outputs
    gpio_pin_set_mode(GPIOD, GPIO_CREATE_MODE_MASK(R0, GPIO_MODE_OUTPUT));
    gpio_pin_set_mode(GPIOD, GPIO_CREATE_MODE_MASK(R1, GPIO_MODE_OUTPUT));
    gpio_pin_set_mode(GPIOD, GPIO_CREATE_MODE_MASK(R2, GPIO_MODE_OUTPUT));
    gpio_pin_set_mode(GPIOD, GPIO_CREATE_MODE_MASK(R3, GPIO_MODE_OUTPUT));

    gpio_set_pupdr(GPIOD, GPIO_CREATE_PUPDR_MASK(R0, GPIO_PUPDR_NO_PULL));
    gpio_set_pupdr(GPIOD, GPIO_CREATE_PUPDR_MASK(R1, GPIO_PUPDR_NO_PULL));
    gpio_set_pupdr(GPIOD, GPIO_CREATE_PUPDR_MASK(R2, GPIO_PUPDR_NO_PULL));
    gpio_set_pupdr(GPIOD, GPIO_CREATE_PUPDR_MASK(R3, GPIO_PUPDR_NO_PULL));

    // Set outputs to be high permanently
    gpio_pin_set_level(GPIOD, R0, true);
    gpio_pin_set_level(GPIOD, R1, true);
    gpio_pin_set_level(GPIOD, R2, true);
    gpio_pin_set_level(GPIOD, R3, true);

    // Configure columns as inputs
    gpio_pin_set_mode(GPIOD, GPIO_CREATE_MODE_MASK(C0, GPIO_MODE_INPUT));
    gpio_pin_set_mode(GPIOD, GPIO_CREATE_MODE_MASK(C1, GPIO_MODE_INPUT));
    gpio_pin_set_mode(GPIOD, GPIO_CREATE_MODE_MASK(C2, GPIO_MODE_INPUT));

    gpio_set_pupdr(GPIOD, GPIO_CREATE_PUPDR_MASK(C0, GPIO_PUPDR_PULLDOWN));
    gpio_set_pupdr(GPIOD, GPIO_CREATE_PUPDR_MASK(C1, GPIO_PUPDR_PULLDOWN));
    gpio_set_pupdr(GPIOD, GPIO_CREATE_PUPDR_MASK(C2, GPIO_PUPDR_PULLDOWN));

    // Configure orange LED to be status LED. This will be replaced later by an external LED
    gpio_pin_set_mode(GPIOD, GPIO_CREATE_MODE_MASK(ORANGE_LED, GPIO_MODE_OUTPUT));
    gpio_set_pupdr(GPIOD, GPIO_CREATE_PUPDR_MASK(ORANGE_LED, GPIO_PUPDR_NO_PULL));


    // Configure interrupt for column 0
    SYSCFG->EXTICR[0] &= ~(0x00F0u);
    SYSCFG->EXTICR[0] |=(0x0030u);
    EXTI->IMR |= (1 << C0);
    EXTI->RTSR |= (1 << C0);
    NVIC_SetPriority(EXTI1_IRQn, 0);
    NVIC_ClearPendingIRQ(EXTI1_IRQn);
    NVIC_EnableIRQ(EXTI1_IRQn);

    // Configure interrupt for column 1
    SYSCFG->EXTICR[0] &= ~(0x0F00u);
    SYSCFG->EXTICR[0] |= (0x0300u);
    EXTI->IMR |= (1 << C1);
    EXTI->RTSR |= (1 << C1);
    NVIC_SetPriority(EXTI2_IRQn, 0);
    NVIC_ClearPendingIRQ(EXTI2_IRQn);
    NVIC_EnableIRQ(EXTI2_IRQn);

    // Configure interrupt for column 2
    SYSCFG->EXTICR[0] &= ~(0xF000u);
    SYSCFG->EXTICR[0] |= (0x3000u);
    EXTI->IMR |= (1 << C2);
    EXTI->RTSR |= (1 << C2);
    NVIC_SetPriority(EXTI3_IRQn, 0);
    NVIC_ClearPendingIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI3_IRQn);

    timer_create_timer(&keypad_debounce_timer, false, KEYPAD_DEBOUNCE_PERIOD_MS, keypad_debounce_cb);
}
