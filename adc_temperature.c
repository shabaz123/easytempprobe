/**
 ****************************************************************************************
 *
 * @file adc_temperature.c
 *
 * @brief Driver implementation for ADC.
 * rev 1 - shabaz - August 2022
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>
#include <math.h>

#include "gpio.h"
#include "i2cm.h"
#include "adc_temperature.h"


// GPIO
#define LED_ON GPIO_SetActive(GPIO_PORT_0, GPIO_PIN_0)
#define LED_OFF GPIO_SetInactive(GPIO_PORT_0, GPIO_PIN_0)

#define THERMISTOR_SELECT GPIO_SetActive(GPIO_PORT_0, GPIO_PIN_7)
#define THERMISTOR_DESELECT GPIO_SetInactive(GPIO_PORT_0, GPIO_PIN_7)

#define DELAY_1MSEC 1777 


// components and voltages
#define VPLUS 3.310
#define RTOP 10045.00
// thermocouple filter resistor value
#define RFILT 46.85

#define BETA 3650.00
// RINF is (R@25degC) * (e^(-BETA/(25+273.15))
// in Excel, formula would be =R*EXP((0-B)/(25+273.15))
// where R is resistance in ohms at 25 deg C
// and B is the Beta value
#define RINF 0.00482278
// ADC resistance between the differential inputs
#define RADCDIFF 281250.00

// MCP3426 ADC
#define ADC_I2C_SLAVE_ADDR (0x68)
#define ADC_MODE_SINGLE 0
#define ADC_MODE_CONT 1



// ************************ Thermocouple Parameters *****************************
// The current values are set for Type K thermocouples.
#define K_START_TEMP (-260.0 - 10.0)
#define K_TOT 165
// this table contains the thermocouple voltages for steps of 10 degrees.
double k_voltage_range[K_TOT] = {
    -6.458,-6.441,-6.404,-6.344,-6.262,-6.158,-6.035,-5.891,-5.730,-5.550, /*  -260-10 to  -170-10 */
    -5.354,-5.141,-4.913,-4.669,-4.411,-4.138,-3.852,-3.554,-3.243,-2.920, /*  -160-10 to   -70-10 */
    -2.587,-2.243,-1.889,-1.527,-1.156,-0.778,-0.392,  /* -60-10 to 0-10*/
    0.000 ,0.397,  0.798, /*   0 to  20 */
    1.203, 1.612, 2.023, 2.436, 2.851, 3.267, 3.682, 4.096, 4.509, 4.920,  /*    30 to   120 */
    5.328, 5.735, 6.138, 6.540, 6.941, 7.340, 7.739, 8.138, 8.539, 8.940,  /*   130 to   220 */
    9.343, 9.747, 10.153,10.561,10.971,11.382,11.795,12.209,12.624,13.040, /*   230 to   320 */
    13.457,13.874,14.293,14.713,15.133,15.554,15.975,16.397,16.820,17.243, /*   330 to   420 */
    17.667,18.091,18.516,18.941,19.366,19.792,20.218,20.644,21.071,21.497, /*   430 to   520 */
    21.924,22.350,22.776,23.203,23.629,24.055,24.480,24.905,25.330,25.755, /*   530 to   620 */
    26.179,26.602,27.025,27.447,27.869,28.289,28.710,29.129,29.548,29.965, /*   630 to   720 */
    30.382,30.798,31.213,31.628,32.041,32.453,32.865,33.275,33.685,34.093, /*   730 to   820 */
    34.501,34.908,35.313,35.718,36.121,36.524,36.925,37.326,37.725,38.124, /*   830 to   920 */
    38.522,38.918,39.314,39.708,40.101,40.494,40.885,41.276,41.665,42.053, /*   930 to  1020 */
    42.440,42.826,43.211,43.595,43.978,44.359,44.740,45.119,45.497,45.873, /*  1030 to  1120 */
    46.249,46.623,46.995,47.367,47.737,48.105,48.473,48.838,49.202,49.565, /*  1130 to  1220 */
    49.926,50.286,50.644,51.000,51.355,51.708,52.060,52.410,52.759,53.106, /*  1230 to  1320 */
    53.451,53.795,54.138,54.479,54.819  /* 1330 to 1370*/
};
// ****************** End of Thermocouple Parameters ****************************

// global variables
//double thermistor_resistance = 0.0;

// *** delay routine x * 1 msec
void delay_ms(unsigned long dd)
{
    unsigned long j, jj;
    jj = dd * DELAY_1MSEC;
    for (j = 1; j<=jj; j++)
    {
        __nop();
        __nop();
    }
}

// thermocouple_t2v converts a temperature into a voltage in mV.
// this is needed for cold junction compensation
int thermocouple_t2v(double temperature, double* voltage)
{
    int idx;
    double ival;
    double span10;
    double vcalc;

    if (temperature >= (((K_TOT - 1) * 10.0) + K_START_TEMP)) {
        //printf("thermocouple_t2v: error, input out of range (too high)\n");
        return(-1);
    }
    if (temperature <= K_START_TEMP) {
        //printf("thermocouple_t2v: error, input out of range (too low)\n");
        return(-1);
    }

    ival = (temperature - K_START_TEMP) / 10.0;
    idx = (int)ival;
    span10 = k_voltage_range[idx + 1] - k_voltage_range[idx];
    vcalc = k_voltage_range[idx] + ((span10 / 10.0) * (temperature - ((idx * 10) + K_START_TEMP)));
    *voltage = vcalc;
    return(0);
}


// thermocouple_v2t: searches the thermocouple table to determine the 
// temperature from the voltage measurement
// where meas is in millivolts, and temperature output is in degC.
int thermocouple_v2t(double meas, double* temperature)
{
    int leftidx = 0;
    int rightidx;
    int mid = 0;
    int iter = 0;
    double span10 = 0.0;
    double tval = 0.0;

    if (meas <= k_voltage_range[1]) {
        //printf("error, temperature too low to measure\n");
        return(-1);
    }
    if (meas >= k_voltage_range[K_TOT - 1]) {
        //printf("error, temperature too high to measure\n");
        return(-1);
    }

    rightidx = K_TOT - 1;
    while (leftidx <= rightidx) {
        mid = (leftidx + rightidx) / 2;
        if (meas > k_voltage_range[mid]) {
            leftidx = mid + 1;
        }
        else if (meas < k_voltage_range[mid]) {
            rightidx = mid - 1;
        }
        else {
            break;
        }
        iter++;
    }
    tval = meas;
    if (meas < 0.0) {
        span10 = k_voltage_range[mid] - k_voltage_range[mid - 1];
        tval = (10.0 * (tval - k_voltage_range[mid])) / span10;
        tval = tval + (K_START_TEMP + (10.0 * mid));
    }
    else {
        span10 = k_voltage_range[mid] - k_voltage_range[mid - 1];
        tval = (10.0 * (tval - k_voltage_range[mid - 1])) / span10;
        tval = tval + (K_START_TEMP - 10 + (10.0 * mid));
    }
    *temperature = tval;
    return(0);
}


// MCP3426 I2C ADC functions
// chan is 0 or 1, mode is ADC_MODE_CONT or ADC_MODE_SINGLE, gain is 1, 2 4 or 8.
void
adc_reg_config(char chan, char mode, char gain)
{
    uint8_t buf[1] = {0};

    buf[0] = 0x80; // set *RDY to 1
    buf[0] |= 0x08; // set to 16-bit mode (15 SPS max)

    if (chan==1) {
        buf[0] |= 0x20; // set C0 to 1
    }
    if (mode==ADC_MODE_CONT) {
        buf[0] |= 0x10; // set *O/C bit to 1
    }
    switch(gain) {
        case 1:
            break;
        case 2:
            buf[0] |= 0x01;
            break;
        case 4:
            buf[0] |= 0x02;
            break;
        case 8:
            buf[0] |= 0x03;
            break;
        default:
            break;
    }
    i2cm_write(ADC_I2C_SLAVE_ADDR, buf, 1, true);
}

// get ADC result as a 16-bit value
// returns -1 if the ADC result is not ready
// returns 0 if the ADC result is complete
int
get_adc_result(int16_t* result)
{
    uint16_t adcval;
    uint8_t buf[3] = {0, 0, 0};
    i2cm_read(ADC_I2C_SLAVE_ADDR, buf, 3, true);
    if (buf[2] & 0x80) { // *RDY is high so the conversion is not complete
        return(-1);
    }

    adcval = ((uint16_t)buf[0])<<8;
    adcval |= (uint16_t)buf[1];
    *result = (int16_t)adcval;
    //printf("adc read: 0x%02x, %02x  =  0x%04x\n", buf[0], buf[1], *result);
    return(0);
}

int get_thermistor_voltage(char chan, double* voltage)
{
    int err=0;
    double result=0.0;
    int16_t adcval;
    int not_finished = 1;
    char gain = 8;
    int timeout;
    THERMISTOR_SELECT;

    while(not_finished) {
        timeout = 20;
        adc_reg_config(chan, ADC_MODE_SINGLE, gain);
        do {
            delay_ms(10);
            timeout--;
        } while ( (get_adc_result(&adcval) != 0) && (timeout>0) );
        if (timeout<=0) {
            err = 1;
            not_finished = 0; // abort
        }
        if (adcval>=0x7fff) {
            if (gain>1) {
                gain = gain >> 1;
            } else {
                err=1;
                not_finished = 0;
            }
        } else {
            not_finished = 0;
        }
    }

    result = ((double)(adcval))/32767.0;
    switch(gain) {
        case 8:
            result *= 0.256;
            break;
        case 4:
            result *= 0.512;
            break;
        case 2:
            result *= 1.024;
            break;
        case 1:
            result *= 2.048;
            break;
        default:
            break;
    }

    if(err) {
        result = 0.0;
        if (timeout<=0) {
            //printf("error: ADC unexpected result!\n");
        } else {
            // need to investigate fputc
            //printf("error: thermistor appears to be missing or open circuit!\n");
        }
    }

    *voltage = result;
    return(err);
}

int get_thermistor_temperature(char chan, char do_avg, double* temperature, double* tresistance)
{
    double thermistor_voltage = 0.0;
    double thermistor_temperature;
    double thermistor_resistance = 0.0;
    double vsingle;
    int ret;
    int i;
    int err=0;

    if (do_avg) {
        for (i=0; i<THERMISTOR_AVG; i++) {
            ret = get_thermistor_voltage(chan, &vsingle);
            if (ret!=0) { // error, abort
                err=1;
                i = THERMISTOR_AVG;
            }
            thermistor_voltage +=  vsingle;
        }
        if (err==0) {
            thermistor_voltage = thermistor_voltage / THERMISTOR_AVG;
        }
    } else {
        ret = get_thermistor_voltage(chan, &vsingle);
        if (ret!=0) err=1;
        thermistor_voltage = vsingle;
    }
    if (err==0) {
        thermistor_resistance = (thermistor_voltage * RTOP) / (VPLUS - thermistor_voltage);
        thermistor_temperature = (BETA / log(thermistor_resistance/RINF)) - 273.15;
    } else {
        thermistor_resistance = 0.0;
        thermistor_temperature = 0.0;
    }

    *tresistance = thermistor_resistance;
    *temperature = thermistor_temperature;
    return(err);
}

// only call this function after get_thermistor_temperature(),
// because it relies on the thermistor measurement.
int get_thermocouple_voltage(char chan, char do_avg, double tresistance, double* voltage)
{
    double result=0.0;
    int16_t adcval;
    int i;
    int timeout;
    int err = 0;
    THERMISTOR_DESELECT;

    if (do_avg) {
        for (i=0; i<THERMOCOUPLE_AVG; i++) {
            adc_reg_config(chan, ADC_MODE_SINGLE, 8/*gain*/);
            timeout = 20;
            do {
                delay_ms(10);
                timeout--;
            } while ( (get_adc_result(&adcval) != 0) && (timeout>0) );
            result += (((double)(adcval))/32767.0);
            if (timeout <=0 ) {
                // abort since something went wrong
                err = 1;
                i = THERMOCOUPLE_AVG;
            }
        }
        if (err==0) {
            result = result / THERMOCOUPLE_AVG;
        }
    } else {
        adc_reg_config(chan, ADC_MODE_SINGLE, 8/*gain*/);
        timeout = 20;
        do {
            delay_ms(10);
            timeout--;
        } while ( (get_adc_result(&adcval) != 0) && (timeout > 0) );
        if (timeout>0) {
            result = ((double) (adcval)) / 32767.0;
        } else {
            err = 1;
        }
    }

    if (err==0) {
        result *= 0.256; // gain is 8 and Vref is 2.048V, so multiply by (2.048/8)

        result = result / RADCDIFF;
        result = result * (tresistance + RFILT + RADCDIFF);
    }

    *voltage = result;
    return(err);
}


/**
 ****************************************************************************************
 * @brief Initialize the temperature sensor, the function sets the resolution to maximum
 * @return void
 ****************************************************************************************
 */
void adc_ic_init(void){
	//
}



double adc_get_temperature(void){
    int ret;
    int err = 0;
    double cjtemperature;
    double vthermocouple;
    double tresistance;
    
    // interim values
    double compvoltage;
    double vthermocouple_compensated;
    
    // desired final output
    double temperature;
    
    ret =  get_thermistor_temperature(0, WITH_AVERAGING, &cjtemperature, &tresistance); // must get thermistor temperature first
    if (ret!=0) err = 1;
    if (err==0) {
        ret = get_thermocouple_voltage(0, NO_AVERAGING, tresistance, &vthermocouple); // dummy read, value is not settled
        if (ret!=0) err = 1;
    }
    if (err==0) {
        ret = get_thermocouple_voltage(0, WITH_AVERAGING, tresistance, &vthermocouple); // get the thermocouple voltage (in Volts)
        if (ret!=0) err = 1;
    }
    
    if (err==0) {
        vthermocouple = vthermocouple * 1000.0; // convert the thermocouple voltage to millivolts
        thermocouple_t2v(cjtemperature, &compvoltage); // calculate the voltage to compensate with
        vthermocouple_compensated = vthermocouple + compvoltage;

        thermocouple_v2t(vthermocouple_compensated, &temperature); // get the probe temperature
    }
    
    if (err == 1) {
        temperature = -999.99;
    }
	return(temperature);
}

struct __FILE
{
  int dummyVar; //Just for the sake of redefining __FILE, we won't we using it anyways ;)
};

FILE __stdout;
FILE __stdin;

int fputc(int c, FILE * stream)
{
	//uartWrite(c);
	return c; //return the character written to denote a successfull write
}

int fgetc(FILE * stream)
{
    char c = '\0';
	//char c = uartRead();
	//uartWrite(c); //To echo Received characters back to serial Terminal
	return c;
}




/// @} APP
