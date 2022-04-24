/* Authors: Alex Bourdage, Sophie Woessner
 * Description - Module for initializing and interacting with interrupts.
 */
#include "lab_interrupts.h"

#include "core_cm4.h"
#include "cmsis_armclang.h"
#include "stm32f4xx.h"


void interrupts_init_interrupts(void)
{
    __enable_irq();
}

void interrupts_enable_interrupt(IRQn_Type interrupt_number)
{
    NVIC_EnableIRQ(interrupt_number);
}
