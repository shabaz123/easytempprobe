/**
 ****************************************************************************************
 *
 * @file adc_temperature.h
 *
 * @brief Driver interface for ADC.
 *
 *
 ****************************************************************************************
 */

#ifndef _ADC_TEMPERATURE_H_
#define _ADC_TEMPERATURE_H_

/**
 ****************************************************************************************
 * @addtogroup APP
 * @ingroup RICOW
 *
 * @brief 
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>
#include <stdbool.h>

#define NO_AVERAGING 0
#define WITH_AVERAGING 1
// number of samples to average over (ideally a power of 2)
#define THERMISTOR_AVG 4
#define THERMOCOUPLE_AVG 8

void adc_ic_init(void);

double adc_get_temperature(void);


int get_thermistor_temperature(char chan, char do_avg, double* temperature, double* tresistance);
int get_thermocouple_voltage(char chan, char do_avg, double tresistance, double* voltage);
int thermocouple_t2v(double temperature, double* voltage);
int thermocouple_v2t(double meas, double* temperature);




/// @} APP

#endif // _ADC_TEMPERATURE_H_
