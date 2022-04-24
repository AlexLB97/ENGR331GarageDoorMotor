#ifndef LAB_PWM_H
#define LAB_PWM_H

#include <stdint.h>

#define PWM_FREQ_HZ 100 // 1ms period

void init_pwm_pin_PD15(void);

void pwm_pd15_change_duty_pct(uint32_t new_duty_pct);

#endif
