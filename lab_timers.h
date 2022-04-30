#ifndef LAB_TIMERS_H
#define LAB_TIMERS_H
#include "stm32f4xx.h"
#include <stdint.h>
#include <stdbool.h>

#define HSI_SPEED_HZ 16000000u

typedef void (* timer_callback_t)(void);


typedef struct {
    uint32_t expiration_ticks;
    uint32_t duration_ms;
    // Pointer to the callback function
    timer_callback_t cb;
    bool timer_active;
    bool repeating;
    char pad_bytes[2]; // Padding to eliminate padding warning
} timer_t;


// Delay Functions
void timer6_delay(int ms);
void delay(uint32_t ms);
void timers_init_timer(TIM_TypeDef * timer, uint32_t apb1enr_bus_position, uint32_t period_ms);
void timers_change_period(TIM_TypeDef *timer, uint32_t period_ms);

void timers_init(void);
void timer_create_timer(timer_t *pTimer, bool repeating, uint32_t period_ms, timer_callback_t cb);
void timer_start_timer(timer_t *pTimer);
uint32_t timer_get_time_millis(void);
void timer_reset_timer(timer_t *pTimer);
void timer_stop_timer(timer_t *pTimer);
void timer_change_timer_period(timer_t *pTimer, uint32_t new_period_ms);
bool timer_is_timer_active(timer_t *pTimer);





#endif // LAB_TIMERS_H
