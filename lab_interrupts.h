/**
 * Authors: Alex Bourdage, Sophie Woessner
 */
#ifndef LAB_INTERRUPTS_H
#define LAB_INTERRUPTS_H

#include "stm32f4xx.h"

void interrupts_init_interrupts(void);
void interrupts_enable_interrupt(IRQn_Type interrupt_number);

#endif
