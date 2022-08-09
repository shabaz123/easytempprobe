/**
 ****************************************************************************************
 *
 * @file user_app.c
 *
 * @brief Pressure sensor user application.
 *
 * Copyright (c) 2012-2020 Dialog Semiconductor. All rights reserved.
 *
 * This software ("Software") is owned by Dialog Semiconductor.
 *
 * By using this Software you agree that Dialog Semiconductor retains all
 * intellectual property and proprietary rights in and to this Software and any
 * use, reproduction, disclosure or distribution of the Software without express
 * written permission or a license agreement from Dialog Semiconductor is
 * strictly prohibited. This Software is solely for use on or in conjunction
 * with Dialog Semiconductor products.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE
 * SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. EXCEPT AS OTHERWISE
 * PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * DIALOG SEMICONDUCTOR BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL,
 * OR CONSEQUENTIAL DAMAGES, OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF
 * USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
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

#include "app_api.h"
#include "adc_temperature.h"
#include "user_app.h"
#include "user_periph_setup.h"
#include "wkupct_quadec.h"
#include "custs1_task.h"
#include "user_custs1_def.h"

#ifdef CFG_PRINTF
#include "arch_console.h"
#endif

#define MEASURE_PERIOD          500         /* Units of 10ms */

static void set_pressure_char_value(uint32_t value);
static void set_temperature_char_value(char* value);
static void send_pressure_char_notify(uint32_t value);
static void send_temperature_char_notify(char* value);
static void wakeup_callback(void);

static uint8_t pressure_notify      __SECTION_ZERO("retention_mem_area0");
static uint8_t temperature_notify   __SECTION_ZERO("retention_mem_area0");
static timer_hnd measure_timer      __SECTION_ZERO("retention_mem_area0");

int iter = 0;
int meas_state = 0;
int err = 0;
double cjtemperature;
double vthermocouple;
double tresistance;


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
*/

/**
 ****************************************************************************************
 * @brief   Called at system start up.
 *
 * @param   None
 *
 * @return  None
 ****************************************************************************************
*/
void user_app_on_init(void)
{

  #ifdef CFG_PRINTF
    arch_printf("\n\r%s", __FUNCTION__);
    arch_printf("\n\rInitializing BMP388...");
  #endif

  /* Configure handler for BMP388 interrupts */
  
  #ifdef JUNK
  wkupct_enable_irq(WKUPCT_PIN_SELECT(BMP388_INT_PORT, BMP388_INT_PIN),
                    WKUPCT_PIN_POLARITY(BMP388_INT_PORT, BMP388_INT_PIN, WKUPCT_PIN_POLARITY_LOW),
                    1,
                    0);

  #endif
  

  
  /* Initialize ADC interface and sensor */
  adc_ic_init();


  #ifdef CFG_PRINTF
  {
    arch_printf("\n\rBMP388 Register Dump");

  }
  #endif

  default_app_on_init();
}

/**
 ****************************************************************************************
 * @brief   Called on database initialization completion event.
 *
 * @param   None
 *
 * @return  None
 ****************************************************************************************
*/
void user_app_on_db_init_complete(void)
{
  #ifdef CFG_PRINTF
    arch_printf("\n\r%s", __FUNCTION__);
  #endif

    char tstring[10];
    sprintf(tstring, "-999.99  ");
    
    
  /* Set initial values of pressure and temperature characteristics */
  set_pressure_char_value(0);
  set_temperature_char_value(tstring);

  /* Start a single first measurement here if needed*/

  measure_timer = app_easy_timer(100 /* 1 sec */, wakeup_callback);
  //wkupct_enable_irq(0x200, 0x200, 1, 0);
  //wkupct_register_callback(wakeup_callback);

  default_app_on_db_init_complete();
}

/**
 ****************************************************************************************
 * @brief   Called when central connects to peripheral.
 *
 * @param   conidx - Connection index.
 *
 * @return  None
 ****************************************************************************************
*/
void user_app_on_connection(uint8_t conidx, struct gapc_connection_req_ind const *param)
{
  #ifdef CFG_PRINTF
    arch_printf("\n\r%s", __FUNCTION__);
  #endif

  default_app_on_connection(conidx, param);
}

/**
 ****************************************************************************************
 * @brief   Called when central disconnects from peripheral.
 *
 * @param   param - Disconnect parameters
 *
 * @return  None
 ****************************************************************************************
*/
void user_app_on_disconnect(struct gapc_disconnect_ind const *param)
{
  #ifdef CFG_PRINTF
    arch_printf("\n\r%s", __FUNCTION__);
  #endif

  default_app_on_disconnect(param);
}

/**
 ****************************************************************************************
 * @brief   Callback for handling messages from custom profile.
 *
 * @param   msgid - Message ID
 *
 * @return  None
 ****************************************************************************************
*/
void user_app_catch_rest_hndl(ke_msg_id_t const msgid,
                              void const *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
  #ifdef CFG_PRINTF
    arch_printf("\n\r%s", __FUNCTION__);
  #endif

  if (msgid == CUSTS1_VAL_WRITE_IND) {
    struct custs1_val_write_ind const *msg_param = (struct custs1_val_write_ind const *)(param);

    switch (msg_param->handle)
    {
      case SVC1_IDX_PRESSURE_NTF_CFG:
      {
        pressure_notify = msg_param->value[0];
      }
      break;

      case SVC1_IDX_TEMPERATURE_NTF_CFG:
      {
        temperature_notify = msg_param->value[0];
      }
      break;

      default:
      {
      }
      break;
    }
  }
}

/**
 ****************************************************************************************
 * @brief   Wakeup callback.
 *
 * @param   None.
 *
 * @return  None.
 ****************************************************************************************
*/
static void wakeup_callback(void)
{
    periph_init();
    uint32_t press = 0;
    char tstring[10];
    int ret;
    double tempv;
    
    // interim values
    double compvoltage;
    double vthermocouple_compensated;
    
    // desired final output
    double temperature;
    
    #ifdef CFG_PRINTF
        arch_printf("\n\r%s", __FUNCTION__);
    #endif
    
    
    switch(meas_state) {
        case 0:
            err = 0;
            ret =  get_thermistor_temperature(0, WITH_AVERAGING, &cjtemperature, &tresistance); // must get thermistor temperature first
            if (ret!=0) err = 1;
            meas_state=1;
            break;
        case 1:
            if (err==0) {
                ret = get_thermocouple_voltage(0, NO_AVERAGING, tresistance, &vthermocouple); // dummy read, value is not settled
                if (ret!=0) err = 1;
            }
            meas_state = 2;
            iter = 0;
            vthermocouple = 0.0;
            break;
        case 2: // enter this state multiple times, until the averaging is complete, or if there is an error
            if (err==0) {
                ret = get_thermocouple_voltage(0, NO_AVERAGING, tresistance, &tempv);
                vthermocouple += tempv;
                iter++;
                if (ret!=0) err = 1;
                if (iter>=THERMOCOUPLE_AVG) {
                    vthermocouple = vthermocouple / THERMOCOUPLE_AVG;
                    meas_state = 3; // averaging complete!
                }
            }
            if (err==1) {
                meas_state = 3; // there was some error, we need to exit this state
            }
            break;
        case 3:
            if (err==0) {
                vthermocouple = vthermocouple * 1000.0; // convert the thermocouple voltage to millivolts
                thermocouple_t2v(cjtemperature, &compvoltage); // calculate the voltage to compensate with
                vthermocouple_compensated = vthermocouple + compvoltage;

                thermocouple_v2t(vthermocouple_compensated, &temperature); // get the probe temperature
            }
            if (err == 1) {
                temperature = -999.99;
            }
            sprintf(tstring, "         ");
            snprintf(tstring, 8, "%0.2f", temperature);
            tstring[strlen(tstring)] = ' '; 
            tstring[9]='\0';
            set_pressure_char_value(press);
            set_temperature_char_value(tstring);
            /* Send notifications if central has subscribed */
            if (ke_state_get(TASK_APP) == APP_CONNECTED) {
                if (pressure_notify) {
                    send_pressure_char_notify((int16_t)temperature);
                }
                if (temperature_notify) {
                    send_temperature_char_notify(tstring);
                }
            }
            meas_state = 0;
            break;
        default:
            break;
    }

    //arch_printf("\n\rtemp: %d.%dC", temp/100, temp%100);
    //arch_printf("\n\rpress: %dPa", press);
    //arch_printf("\n\rpress: %d.%dinHg", press/3386, press%3386);

    /* Restart timer that will trigger periodic measurement */
    //measure_timer = app_easy_timer(MEASURE_PERIOD, measure_timer_cb);
    measure_timer = app_easy_timer(10 /* 100 msec */, wakeup_callback);
}

/**
 ****************************************************************************************
 * @brief   Set temperature characteristic value.
 *
 * @param   value - Temperature value
 *
 * @return  None
 ****************************************************************************************
*/
static void set_temperature_char_value(char* value)
{
  struct custs1_val_set_req *req = KE_MSG_ALLOC_DYN(CUSTS1_VAL_SET_REQ,
                                                    prf_get_task_from_id(TASK_ID_CUSTS1),
                                                    TASK_APP,
                                                    custs1_val_set_req,
                                                    DEF_SVC1_TEMPERATURE_CHAR_LEN);

  req->handle = SVC1_IDX_TEMPERATURE_VAL;
  req->length = DEF_SVC1_TEMPERATURE_CHAR_LEN;
  memcpy(req->value, value, DEF_SVC1_TEMPERATURE_CHAR_LEN);

  ke_msg_send(req);
}

/**
 ****************************************************************************************
 * @brief   Set pressure characteristic value.
 *
 * @param   value - Pressure value
 *
 * @return  None
 ****************************************************************************************
*/
static void set_pressure_char_value(uint32_t value)
{
  struct custs1_val_set_req *req = KE_MSG_ALLOC_DYN(CUSTS1_VAL_SET_REQ,
                                                    prf_get_task_from_id(TASK_ID_CUSTS1),
                                                    TASK_APP,
                                                    custs1_val_set_req,
                                                    DEF_SVC1_PRESSURE_CHAR_LEN);

  req->handle = SVC1_IDX_PRESSURE_VAL;
  req->length = DEF_SVC1_PRESSURE_CHAR_LEN;
  memcpy(req->value, &value, DEF_SVC1_PRESSURE_CHAR_LEN);

  ke_msg_send(req);
}

/**
 ****************************************************************************************
 * @brief   Send temperature notification.
 *
 * @param   value - Temperature value
 *
 * @return  None
 ****************************************************************************************
*/
static void send_temperature_char_notify(char* value)
{
  struct custs1_val_ntf_ind_req *req = KE_MSG_ALLOC_DYN(CUSTS1_VAL_NTF_REQ,
                                                        prf_get_task_from_id(TASK_ID_CUSTS1),
                                                        TASK_APP,
                                                        custs1_val_ntf_ind_req,
                                                        DEF_SVC1_TEMPERATURE_CHAR_LEN);

  req->handle = SVC1_IDX_TEMPERATURE_VAL;
  req->length = DEF_SVC1_TEMPERATURE_CHAR_LEN;
  req->notification = true;
  memcpy(req->value, value, DEF_SVC1_TEMPERATURE_CHAR_LEN);

  ke_msg_send(req);
}

/**
 ****************************************************************************************
 * @brief   Send pressure notification.
 *
 * @param   value - Pressure value
 *
 * @return  None
 ****************************************************************************************
*/
static void send_pressure_char_notify(uint32_t value)
{
  struct custs1_val_ntf_ind_req *req = KE_MSG_ALLOC_DYN(CUSTS1_VAL_NTF_REQ,
                                                        prf_get_task_from_id(TASK_ID_CUSTS1),
                                                        TASK_APP,
                                                        custs1_val_ntf_ind_req,
                                                        DEF_SVC1_PRESSURE_CHAR_LEN);

  req->handle = SVC1_IDX_PRESSURE_VAL;
  req->length = DEF_SVC1_PRESSURE_CHAR_LEN;
  req->notification = true;
  memcpy(req->value, &value, DEF_SVC1_PRESSURE_CHAR_LEN);

  ke_msg_send(req);
}

/// @} APP
