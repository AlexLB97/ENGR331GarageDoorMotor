/**
* Authors: Alex Bourdage, Sophie Woessner
* Module for controlling the LCD screen.
*/
#ifndef _LCD_H_
#define _LCD_H_

#include <stdint.h>

#include "stm32f4xx.h"

#define DISPLAY_WIDTH_CHARS 16
#define LINE_WIDTH_CHARS 40

typedef enum {
    ON_WHILE_WRITING = 0,
    OFF_WHILE_WRITING
} write_type_t;

/*******************************
 * LCD Commands and Flags
 *******************************
 */

// Commands without options
#define LCD_CMD_CLEAR_DISPLAY 0x01
#define LCD_CMD_RETURN_HOME   0x02

/*** Entry Mode Command and Flags ***/
#define LCD_CMD_SET_ENTRY_MODE 0x04

// Default direction is to the left when bit 1 is clear
#define ENTRY_MODE_CURSOR_DIRECTION_RIGHT (1 << 1)

// When enabled, display shifts left if I/D is 1 and right if I/D is 0
#define ENTRY_MODE_SHIFT_ENABLE  (1)

/*** Display On Off Control Commands ***/
#define LCD_CMD_DISPLAY_ON_OFF 0x08
#define DISPLAY_MODE_ON        (1 << 2)
#define DISPLAY_MODE_CURSOR    (1 << 1)
#define DISPLAY_MODE_BLINK     (1)

/*** Cursor / Shift Command ***/
#define LCD_CMD_CURSOR_DISPLAY_SHIFT 0x10
#define SHIFT_OPERATION (1 << 3)
#define MOVE_RIGHT (1 << 2)

/*** Function Set Commands ***/
#define LCD_CMD_FUNCTION_SET 0x20
#define EIGHT_BIT_DATA (1 << 4)
#define TWO_LINE       (1 << 3)
#define TALL_FONT      (1 << 2)

/** Set DDRAM **/
#define SET_CURSOR_ADDRESS 0x80








/*******************************
 * FUNCTION PROTOTYPES
 *******************************
 */

// LCD related functions
void LCD_port_init(void);
void LCD_init(void);
void LCD_place_cursor(uint8_t address);
void LCD_send_cmd(uint8_t cmd);
void LCD_write_char(unsigned char data);
void LCD_write_string(char *message, write_type_t write_type);
void LCD_clear_display(void);
void LCD_write_string_at_addr(char *message, write_type_t write_type, uint8_t addr, int num_chars);
// END Functions


#endif // _LCD_H_
