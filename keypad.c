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

#include "event_queue.h"
#include "lab_gpio.h"
#include "lab_timers.h"
#include "LCD.h"
#include "lcd_layout.h"
#include "motor_control.h"
#include "servo_control.h"
#include "stm32f407xx.h"

/* Preprocessor Definitions */
#define KEYPAD_DEBOUNCE_PERIOD_MS 200
#define CODE_ENTRY_TIMEOUT_MS 5000    // Clear passcode and return to default state if there are 5 seconds between button presses
#define CODE_LENGTH 4
#define GREEN_STATUS_LED 9
#define RED_STATUS_LED  11

/* Type Definitions */
typedef enum entry_state_t {
    ENTRY_INACTIVE = 0,
    ENTRY_ACTIVE,
    WAITING_FOR_ENTER,
    DISPLAYING_MESSAGE
} entry_state_t;

/* File scope variables */
static char keypad_array[4][3] = {{'1','2','3'},
                                  {'4','5','6'},
                                  {'7','8','9'},
                                  {'*','0','#'}};

static char correct_code[] = "2539";
static char default_code[] = "0000";
static char entered_code[CODE_LENGTH + 1];

static char idle_string[] = "Enter Code";
static char success_string[] = "Success";
static char fail_string[] = "Wrong Code";
static char empty_row[] = "                ";
static char ENTER_BUTTON = '#';

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
static void clear_passcode_region(void);
static void change_keypad_message(char *message);
static void handle_keypad_action(char button);
static void handle_state_transition(entry_state_t next_state);
static void reset_button_press_timer(void);
static void transition_to_active_state_cb(void);
static void entry_active_cb(void);
static void display_fail_string_cb(void);
static void display_success_string_cb(void);
static void display_idle_string_cb(void);


/* Callbacks to execute LCD writes from the main loop */
static void transition_to_active_state_cb(void)
{
        clear_passcode_region();
        // State just transitioned. Display first character.
        LCD_write_char(entered_code[0]);
}

static void entry_active_cb(void)
{
    LCD_write_char(entered_code[current_code_index - 1]);
}

static void display_fail_string_cb(void)
{
    change_keypad_message(fail_string);
}

static void display_success_string_cb(void)
{
    change_keypad_message(success_string);
}

static void display_idle_string_cb(void)
{
    change_keypad_message(idle_string);
}


/* Functions */

static void reset_button_press_timer(void)
{
    // Start or reset code entry timer
    if (timer_is_timer_active(&code_entry_timer))
    {
        timer_reset_timer(&code_entry_timer);
    }
    else
    {
        timer_start_timer(&code_entry_timer);
    }
}

static void change_keypad_message(char *message)
{
    clear_passcode_region();
    LCD_write_string_at_addr(message, ON_WHILE_WRITING, SECOND_LINE_STRT_ADDR, (int)strlen(message));
}

static void clear_passcode_region(void)
{
    LCD_write_string_at_addr(empty_row, ON_WHILE_WRITING, SECOND_LINE_STRT_ADDR, (int)strlen(empty_row));
    LCD_place_cursor(SECOND_LINE_STRT_ADDR);
}

static void keypad_debounce_cb(void) 
{
    button_press_allowed = true;
}

static void evaluate_code(void)
{
    if (!strncmp(entered_code, correct_code, CODE_LENGTH + 1))
    {
        door_state_t next_state = servo_control_get_next_state();
        servo_control_handle_state_transition(next_state);
        queue_add_event(display_success_string_cb);
        gpio_pin_set(GPIOE, GREEN_STATUS_LED);
    }
    else
    {
        queue_add_event(display_fail_string_cb);
        gpio_pin_set(GPIOE, RED_STATUS_LED);
    }
}


static void code_entry_timeout_cb(void)
{
    handle_state_transition(ENTRY_INACTIVE);
}

static bool record_button_press(char button)
{
    if (button >= '0' && button <= '9')
    {
        entered_code[current_code_index++] = button;
        return true;
    }
    return false;
}

static void handle_state_transition(entry_state_t next_state)
{
    current_entry_state = next_state;

    switch(next_state)
    {
        case ENTRY_INACTIVE:
        {
            // reset code index
            current_code_index = 0;

            // Reset LEDs
            gpio_pin_clear(GPIOE, GREEN_STATUS_LED);
            gpio_pin_clear(GPIOE, RED_STATUS_LED);

            // reset the recorded code
            strncpy(entered_code, default_code, CODE_LENGTH + 1);

            // stop the entry timer if active
            if (timer_is_timer_active(&code_entry_timer))
            {
                timer_stop_timer(&code_entry_timer);
            }

            // change message back to idle message
            queue_add_event(display_idle_string_cb);
            break;
        }

        case ENTRY_ACTIVE:
        {
            queue_add_event(transition_to_active_state_cb);
            break;
        }

        case WAITING_FOR_ENTER:
        {
            break;
        }

        case DISPLAYING_MESSAGE:
        {
            evaluate_code();
            break;
        }
    }
}

static void handle_keypad_action(char button)
{
    switch (current_entry_state)
    {
        case ENTRY_INACTIVE:
        {
            bool valid_button = record_button_press(button);
            if (valid_button)
            {
                handle_state_transition(ENTRY_ACTIVE);
                reset_button_press_timer();
            }
            break;
        }

        case ENTRY_ACTIVE:
        {
            bool valid_button = record_button_press(button);
            if (valid_button)
            {
                reset_button_press_timer();

                // Queue event to display character just entered on LCD
                queue_add_event(entry_active_cb);

                // If code is fully entered, go to next state
                if (current_code_index == CODE_LENGTH)
                {
                    handle_state_transition(WAITING_FOR_ENTER);
                } 
            }
            break;
        }

        case WAITING_FOR_ENTER:
        {
            if (button == ENTER_BUTTON)
            {
                reset_button_press_timer();

                // Evaluate code and take action accordingly
                handle_state_transition(DISPLAYING_MESSAGE);
            }
            break;
        }

        case DISPLAYING_MESSAGE:
        {
            break;
        }
    }
}

static int get_button_row(int col)
{
    uint8_t rows[4] = {R0, R1, R2, R3};
    int row = 0;
    for (int i = 0; i < 4; i++)
    {
        gpio_pin_clear(GPIOD, rows[i]);
        if (!gpio_pin_get_level(GPIOD, (uint32_t)col))
        {
            row = i;
            gpio_pin_set(GPIOD, rows[i]);
            break;
        }
        gpio_pin_set(GPIOD, rows[i]);
    }
    return row;
}

/**
* @brief Responds to a button press on the keyboard
* By scanning to determine which row and column was pressed.
*/
static void keypad_button_press_handler(uint8_t column)
{  
    int row;

    // Prevent further erroneous button pressed
    button_press_allowed = false;

    // Start debounce timer
    timer_start_timer(&keypad_debounce_timer);

    // Determine which key was pressed
    row = get_button_row(column);

    handle_keypad_action(keypad_array[row][column - 1]);
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
    gpio_clock_enable(RCC_AHB1ENR_GPIOEEN_Pos);

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

    // Configure the red and green status LEDs
    gpio_pin_set_mode(GPIOE, GPIO_CREATE_MODE_MASK(GREEN_STATUS_LED, GPIO_MODE_OUTPUT));
    gpio_set_pupdr(GPIOE, GPIO_CREATE_PUPDR_MASK(GREEN_STATUS_LED, GPIO_PUPDR_NO_PULL));

    gpio_pin_set_mode(GPIOE, GPIO_CREATE_MODE_MASK(RED_STATUS_LED, GPIO_MODE_OUTPUT));
    gpio_set_pupdr(GPIOE, GPIO_CREATE_PUPDR_MASK(RED_STATUS_LED, GPIO_PUPDR_NO_PULL));

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

    // Write initial keypad message to LCD
    handle_state_transition(ENTRY_INACTIVE);
}
