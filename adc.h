/**
 * Author: Alex Bourdage
 * 
 * Goal: Module for managing ADC inputs for temperature and photoresistor.
 * Due to fluctuations in the readings from the sensors, this module takes 
 * measurements every second and records an average over the last 10 readings. 
 * It is the average reading that the API exposes to other modules.
 */

#ifndef ADC_H
#define ADC_H

void adc_init(void);

double adc_get_light_level(void);

int adc_get_temp(void);

#endif
