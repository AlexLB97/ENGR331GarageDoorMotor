/**
 * Alex Bourdage
 * Sophie Woessner
 * 
 * Module for managing input from the keypad.
 * 
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "keypad.h"
#include "lab_gpio.h"
#include "lab_timers.h"
#include "motor_control.h"
#include "stm32f407xx.h"

/* Preprocessor Definitions */
#define KEYPAD_DEBOUNCE_PERIOD_MS 200
#define CODE_ENTRY_TIMEOUT_MS 5000    // Clear passcode and return to default state if there are 5 seconds between button presses
#define CODE_LENGTH 4

/* Type Definitions */
typedef enum entry_state_t {
    ENTRY_INACTIVE = 0,
    ENTRY_ACTIVE,
    ENTRY_COMPLETE
} entry_state_t;

/* File scope variables */
static char keypad_array[4][3] = {{'1','2','3'},
                                  {'4','5','6'},
                                  {'7','8','9'},
                                  {'*','0','#'}};

static char correct_code[] = "2539";
static char default_code[] = "0000";
static char entered_code[CODE_LENGTH + 1];

static int current_code_index = 0;


static timer_t keypad_debounce_timer;
static timer_t code_entry_timer;

static bool button_press_allowed = true;

static entry_state_t current_entry_state = ENTRY_INACTIVE;

/* External Function Declarations */
extern void EXTI1_IRQHandler(void);
extern void EXTI2_IRQHandler(void);
extern void EXTI3_IRQHandler(void);


/* Static function declarations */
static void keypad_debounce_cb(void);
static void code_entry_timeout_cb(void);
static void reset_code(void);



/* Functions */

static void reset_code(void)
{
    current_entry_state = ENTRY_INACTIVE;
    current_code_index = 0;
    gpio_pin_clear(GPIOD, ORANGE_LED);
    strncpy(entered_code, default_code, CODE_LENGTH + 1);
    if (timer_is_timer_active(&code_entry_timer))
    {
        timer_stop_timer(&code_entry_timer);
    }
}

static void keypad_debounce_cb(void) 
{
    button_press_allowed = true;
}

static void evaluate_code(void)
{
    if (!strncmp(entered_code, correct_code, CODE_LENGTH + 1))
    {
        door_state_t next_state = motor_control_get_next_state();
        motor_control_handle_state_transition(next_state);
    }
}


static void code_entry_timeout_cb(void)
{
    reset_code();
}

/**
* @brief Responds to a button press on the keyboard
* By scanning to determine which row and column was pressed.
*/
static void keypad_button_press_handler(uint8_t column)
{  
    uint8_t rows[4] = {R0, R1, R2, R3};
    uint8_t row = 0;

    // Prevent further erroneous button pressed
    button_press_allowed = false;

    // Transition to active state if not active
    if (current_entry_state == ENTRY_INACTIVE)
    {
        current_entry_state = ENTRY_ACTIVE;
        gpio_pin_set(GPIOD, ORANGE_LED);
    }

    // Start debounce timer
    timer_start_timer(&keypad_debounce_timer);

    // Start or reset code entry timer
    if (timer_is_timer_active(&code_entry_timer))
    {
        timer_reset_timer(&code_entry_timer);
    }
    else
    {
        timer_start_timer(&code_entry_timer);
    }

    // Determine which key was pressed
    for (int i = 0; i < 4; i++)
    {
        gpio_pin_clear(GPIOD, rows[i]);
        if (!gpio_pin_get_level(GPIOD, column))
        {
            row = rows[i];
            gpio_pin_set(GPIOD, rows[i]);
            break;
        }
        gpio_pin_set(GPIOD, rows[i]);
    }

    // Record pressed button and increment index
    entered_code[current_code_index++] = keypad_array[row - 4][column - 1];

    if (current_code_index == CODE_LENGTH)
    {
        evaluate_code();
        reset_code();
    }

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
    timer_create_timer(&code_entry_timer, false, CODE_ENTRY_TIMEOUT_MS, code_entry_timeout_cb);
}
