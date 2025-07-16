/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file By James D. Jeffers, Copyright University of Oklahoma 4/11/2023
 *
 * @defgroup ble_sdk_app_eco_main main.c
 * @{
 * @ingroup ble_sdk_app_eco
 * @brief Environmental Sensing - CO Concentration (ppm) Application main file.
 *
 * This file contains the source code for an application using a new Environmental Sensing service.
 * This file also contains the code for bi-directional communication for testing and timing.
 * It implements hardware v1.0.93, v1.1.
 *    Wake-up button - Pin 20
 *    ADC input - Pin 5
 *    Power Switch - Pin 4
 */
 
// TODO: BT humidity & pressure

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "app_timer.h"

#include "nrf_sdh.h"             // Softdevice operation: Expected Softdevice 112 v7.0
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"

#include "ble.h"                 // Softdevice BLE libraries
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"

#include "nrf_pwr_mgmt.h"        // Softdevice module power management, sleep and idle mode

#include "nrf_gpio.h"            // Wake-up and power switch IO library

#include "nrf_nvmc.h"            // Flash memory write operations

#include "nrf_temp.h"            // Temperature sensor measurements

#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ble_eco.h"                   // Custom BLE services library, Environmental Sensing

#include "nrf_drv_twi.h"
#include "boards.h"
#include "app_util_platform.h"
#include "ble_dis.h"
#include "ble_srv_common.h"

#define FIRMWARE_VERSION "v4.0"

#define DEVICE_NAME                   "XHale Health"         /**< Name of device. Will be included in the advertising data. */
#define SENSOR_MEAS_INTERVAL          APP_TIMER_TICKS(250)                   /**< Sensor measurement interval (ticks). */

#define APP_BLE_OBSERVER_PRIO         3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG          1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL              40                                      /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */

#define APP_ADV_DURATION              3000                                   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL             MSEC_TO_UNITS(500, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL             MSEC_TO_UNITS(1000, UNIT_1_25_MS)       /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                 0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT              MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_UPDATE_DELAY       APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of indication) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_UPDATE_DELAY        APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_UPDATE_COUNT         3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                     0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define POWER_SWITCH_PIN              4
#define POWER_SWITCH_OFF              0
#define POWER_SWITCH_ON               1

#define WAKE_SWITCH_PIN               20

#define ANALOG_GAIN			NRF_SAADC_GAIN1_4

										 /* ================================= CALIBRATION STEPS ================================= */
#define CO_PPM_UNCAL_AT_MAX		0x20					 /**< 1) Set this to 0x32 to calibrate & run 'N2 Flush' script.
                                                                                   2) Run 'CO 50 PPM' script, wait 50 seconds, & replace 0x32 with the value shown in the nRF app. */
#define CO_HAS_FS_CAL			0					 /**< 3) Set to 1. */
#define CO_PPM_MAX                	50
#define CO_CAL_VALUE			CO_PPM_MAX / CO_PPM_UNCAL_AT_MAX


#define FLASH_LEN                     (10u)


/* BME280 Settings */
#define USE_TEMPERATURE		1
//#define USE_HUMIDITY			1
//#define USE_PRESSURE			1

/* temperature sensor. */
#define SAMPLE_COUNT           4
#define BME280_I2C_ADDR        0x76
#define BME280_CHIP_ID         0x60
#define TWI_INSTANCE_ID        0

/* even if connected, after 5 mins of inactivity it disconnects*/
// Remove disconnect timer definition
// #define DISCONNECT_TIMEOUT_MS 300000  // 5 minutes in milliseconds
// APP_TIMER_DEF(m_disconnect_timer_id);


/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

typedef struct
{
    uint32_t addr;
    uint32_t buffer[FLASH_LEN];
} flash_data_t;

static flash_data_t m_data;


BLE_ECO_DEF(m_eco);                                                     /**< Structure for environmental sensing carbon monoxide */
NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                 /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                     /**< Advertising module instance. */

static uint16_t             m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static bool                 m_bps_meas_ind_conf_pending = false;        /**< Flag to keep track of when an indication confirmation is pending. */

static ble_uuid_t m_adv_uuids[] =                                       /**< Universally unique service identifiers. */
{
    {BLE_UUID_ECO_SERVICE,                BLE_UUID_TYPE_BLE}
};


static void advertising_start(void);
bool get_temperature();
//bool get_humidity();
//bool get_pressure();

/* Analog input
*/
APP_TIMER_DEF(m_sensor_timer_id);                                      /**< Data update timer. */
#define SAMPLES_IN_BUFFER 20

static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(2);       /**< SAADC sampling timer. */
static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t     m_ppi_channel;

static uint16_t              m_adc_value;
static uint16_t 	      m_adc_cal_autozero_value;
static uint8_t		      adc_cal_autozero_set = 0;
static uint16_t              m_status_value;
static uint16_t              m_test_value;

static uint16_t              m_test_timer = 0;
static uint32_t              m_temp_value = 0;
//static uint32_t	      m_humidity_value = 0;
//static uint32_t              m_pressure_value = 0;

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;
static ble_advdata_manuf_data_t m_manuf_data;
// static uint8_t m_device_mac[6];
// static char              m_serial_str[17];
// static ble_srv_utf8_str_t m_serial_utf8;


// static void read_ble_address(void)
// {
//     ble_gap_addr_t gap_addr;
//     // pull the current BLE address (public or random) out of the SoftDevice
//     sd_ble_gap_addr_get(&gap_addr);
//     memcpy(m_device_mac, gap_addr.addr, 6);   // BLE spec orders it [0]=LSB ... [5]=MSB
// }

/* global instance of temperature sensor bme280*/
struct bme280_driver bme280; 

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/****************************************************************************** 
*   Flash memory initialization
*/
static void flash_page_init(void)
{
    m_data.addr = 0x0002F000;

    for (int i = 0; i < FLASH_LEN; i++)
    {
        m_data.buffer[i] = ((uint32_t *)m_data.addr)[i];
    }
    uint32_t temp = m_data.buffer[0];
    
    if (temp == 0xFFFFFFFF)
    {
      nrf_nvmc_page_erase((uint32_t *)m_data.addr);
      nrf_nvmc_write_word((uint32_t *)m_data.addr, 0x0);
    }
    else
    {
      //nrf_nvmc_page_erase(m_data.addr);
      //nrf_nvmc_write_word((uint32_t *)(m_data.addr+9), 0xA55A5AA5);
      m_data.buffer[0]++;
    }
    m_status_value = temp;
    m_data.buffer[3] = m_data.addr;
    m_data.buffer[5] = 0x12345678;
    m_data.buffer[6] = 0xA55A5AA5;
}

static void flash_page_close(void)
{
    nrf_nvmc_page_erase((uint32_t *)m_data.addr);
    nrf_nvmc_write_words((uint32_t *)m_data.addr, m_data.buffer, FLASH_LEN);

    for (int i = 9; i < FLASH_LEN; i++)
    {
        //((uint32_t *)m_data.addr)[i] = m_data.buffer[i];
        //nrf_nvmc_write_words((uint32_t *)(m_data.addr), m_data.buffer[i]);
    }
}
/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */
static void sensor_update(void)
{
    ret_code_t err_code;

    //NRF_TEMP->TASKS_START = 1; /** Start the temperature measurement. */

    /* Busy wait while temperature measurement is not finished, you can skip waiting if you enable interrupt for DATARDY event and read the result in the interrupt. */
    /*lint -e{845} // A zero has been given as right argument to operator '|'" */
    //while (NRF_TEMP->EVENTS_DATARDY == 0)
    //{
         // Do nothing.
    //}
    //NRF_TEMP->EVENTS_DATARDY = 0;

 //   temperature = ((float)nrf_temp_read()/ 4)*100;
//    NRF_LOG_INFO("temperature: %d",temperature);
    /**@note Workaround for PAN_028 rev2.0A anomaly 30 - TEMP: Temp module analog front end does not power down when DATARDY event occurs. */
    //NRF_TEMP->TASKS_STOP = 1; /** Stop the temperature measurement. */
    err_code = ble_eco_concentration_update(&m_eco, BLE_CONN_HANDLE_ALL);
    
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
    APP_ERROR_HANDLER(err_code);
    }
    
    err_code = ble_eco_temperature_update(&m_eco, BLE_CONN_HANDLE_ALL);
    
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
    	NRF_LOG_INFO("err_code for temperature: %d", err_code);
        APP_ERROR_HANDLER(err_code);
    }
    
//    err_code = ble_eco_humidity_update(&m_eco, BLE_CONN_HANDLE_ALL);
//    
//    if ((err_code != NRF_SUCCESS) &&
//        (err_code != NRF_ERROR_INVALID_STATE) &&
//       (err_code != NRF_ERROR_RESOURCES) &&
//        (err_code != NRF_ERROR_BUSY) &&
//        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
//       )
//    {
//    	NRF_LOG_INFO("err_code for humidity: %d", err_code);
//        APP_ERROR_HANDLER(err_code);
//    }
//    
//    err_code = ble_eco_pressure_update(&m_eco, BLE_CONN_HANDLE_ALL);
//    
//    if ((err_code != NRF_SUCCESS) &&
//        (err_code != NRF_ERROR_INVALID_STATE) &&
//        (err_code != NRF_ERROR_RESOURCES) &&
//        (err_code != NRF_ERROR_BUSY) &&
//        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
//       )
//    {
//    	NRF_LOG_INFO("err_code for pressure: %d", err_code);
 //       APP_ERROR_HANDLER(err_code);
//    }

    if (m_status_value > 1)
    {
        m_status_value = (m_test_timer << 8) + m_status_value;
        m_test_timer++;
        if (m_test_timer >= 15)
        {
          m_status_value = 0;
          m_test_timer = 0;
        }
        err_code = ble_eco_status_update(&m_eco, BLE_CONN_HANDLE_ALL);
    }
    m_data.buffer[1]++;

}


/**@brief Function for handling the sensor measurement timer timeout.
 *
 * @details This function will be called each time the sensor measurement timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void sensor_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    sensor_update();
    
    #if USE_TEMPERATURE
    get_temperature();
    #endif
    
//    #if USE_HUMIDITY
//    get_humidity();
//    #endif
    
//    #if USE_PRESSURE
//    get_pressure();
//   #endif
    
}

/**@brief Function for initializing low frequency clock.
 */
void saadc_sampling_event_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event every 50ms */
    /* given 20 adc averages, 1 sample is transferred every 1s */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, 50);
    nrf_drv_timer_extended_compare(&m_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   ticks,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   false);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer,
                                                                                NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,
                                          timer_compare_event_addr,
                                          saadc_sample_task_addr);
    APP_ERROR_CHECK(err_code);
}

void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);

    APP_ERROR_CHECK(err_code);
}

/* Process the Analog Measurement *****************************************************
*  
*  Default transmit all points by log with serial ASCII
*     Average = SAMPLES_IN_BUFFER
* 
*/
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        int i;
        m_adc_value = 0;
        for (i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
          
            m_adc_value += p_event->data.done.p_buffer[i];
          
        }
        m_adc_value /= SAMPLES_IN_BUFFER;
        
        if (m_adc_value > 0x03FF)
        {
            m_adc_value = 0;
        }
        
        if (adc_cal_autozero_set == 0)
        {
            m_adc_cal_autozero_value = m_adc_value;
            m_adc_value = 0;
            adc_cal_autozero_set = 1;
        }
        else
        {
            if (m_adc_value - m_adc_cal_autozero_value < 0)
            {
            	m_adc_cal_autozero_value = m_adc_value;
            	m_adc_value = 0;
            }
            else
            {
            	m_adc_value = (m_adc_value - m_adc_cal_autozero_value) * CO_CAL_VALUE;;
            	
            }
            
            
        }
        
        // Show "High" on app if CO value read is CO_PPM_MAX + 1
        // Discourages users from going for a "highscore"
        if (CO_HAS_FS_CAL && m_adc_value > CO_PPM_MAX)
        {
            m_adc_value = CO_PPM_MAX + 1;
        }
        
        // Convert endian-ness for viewing in nRF Connect app and mask to 10-bits (resolution of ADC)
        // NOTE: Value shown in nRF Connect app is 16-bit and little endian
        m_adc_value = ((m_adc_value >> 8) | (m_adc_value << 8)) & 0x0000FF03UL;
        
        NRF_LOG_INFO("CO: %x", m_adc_value);
    }
}

/* Configure the Analog-to-Digital Converter*****************************************************
*  
*  Default single-ended measurement modified by
*     Gain = 1
*     Acqusition Time = 40 us
*/
void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN3);
    channel_config.gain = ANALOG_GAIN;
    channel_config.acq_time = NRF_SAADC_ACQTIME_40US;

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
    err_code = app_timer_create(&m_sensor_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                sensor_timeout_handler);
    APP_ERROR_CHECK(err_code);
    
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_THERMOMETER);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);

}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Blood Pressure, Battery, and Device Information services.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_eco_init_t     eco_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Environmental Sensing Service.
    memset(&eco_init, 0, sizeof(eco_init));
    
    //eco_init.evt_handler             = eco_command_handler;
    eco_init.error_handler           = service_error_handler;
    eco_init.concentration_value     = &m_adc_value;
    eco_init.temperature_value       = &m_temp_value;
//    eco_init.humidity_value          = &m_humidity_value;	// Test value for humidity and pressure
//    eco_init.pressure_value          = &m_pressure_value;
    eco_init.status_value            = &m_status_value;
    eco_init.test_value              = &m_test_value;
    eco_init.device_info             = m_data.buffer;

    err_code = ble_eco_init(&m_eco, &eco_init);
    APP_ERROR_CHECK(err_code);

}

static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    // 1) Zero the init struct
    memset(&init, 0, sizeof(init));

    // 2) Simple advertising like the old working code - no manufacturer data
    // m_manuf_data.company_identifier = 0x0059;      // Your Bluetooth SIG company ID
    // m_manuf_data.data.p_data        = m_device_mac;
    // m_manuf_data.data.size          = sizeof(m_device_mac);

    // 3) Tell the advertising module about it
    // init.advdata.p_manuf_specific_data     = &m_manuf_data;

    // 4) The rest of your advertising payload - simplified
    init.advdata.name_type                 = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance        = true;
    init.advdata.flags                     = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt   = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids    = m_adv_uuids;

    // 5) Advertising parameters
    init.config.ble_adv_fast_enabled       = true;
    init.config.ble_adv_fast_interval      = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout       = APP_ADV_DURATION;  // seconds*100

    init.evt_handler                       = on_adv_evt;

    // 6) Initialize & set TX power
    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);

    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV,
                                       m_advertising.adv_handle,
                                       -8);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing "wake-up" button.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void gpio_init(void)
{

    nrf_gpio_cfg_output(POWER_SWITCH_PIN);
    nrf_gpio_pin_write(POWER_SWITCH_PIN, POWER_SWITCH_ON);

}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

/*
  A Function to read data from the BME280
*/ 
bool bme280_register_read(uint8_t register_address, uint8_t * destination, uint8_t number_of_bytes)
{
    ret_code_t err_code;

    //Set the flag to false to show the receiving is not yet completed
    m_xfer_done = false;
    
    // Send the Register address where we want to write the data
    err_code = nrf_drv_twi_tx(&m_twi, BME280_I2C_ADDR, &register_address, 1, true);
	  
    //Wait for the transmission to get completed
    while (m_xfer_done == false){}
    
    // If transmission was not successful, exit the function with false as return value
    if (NRF_SUCCESS != err_code)
    {
        return false;
    }

    //set the flag again so that we can read data from the BME280's internal register
    m_xfer_done = false;
	  
    // Receive the data from the BME280
    err_code = nrf_drv_twi_rx(&m_twi, BME280_I2C_ADDR, destination, number_of_bytes);
		
    //wait until the transmission is completed
    while (m_xfer_done == false){}
	
    // if data was successfully read, return true else return false
    if (NRF_SUCCESS != err_code)
    {
        return false;
    }
    
    return true;
}


/*
   A function to write a Single Byte 
*/ 
bool bme280_register_write(uint8_t register_address, uint8_t value)
{
    ret_code_t err_code;
    uint8_t tx_buf[BME280_ADDRESS_LEN+1];
	
    //Write the register address and data into transmit buffer
    tx_buf[0] = register_address;
    tx_buf[1] = value;

    //Set the flag to false to show the transmission is not yet completed
    m_xfer_done = false;
    
    //Transmit the data over TWI Bus
    err_code = nrf_drv_twi_tx(&m_twi, BME280_I2C_ADDR, tx_buf, BME280_ADDRESS_LEN+1, false);
    
    //Wait until the transmission of the data is finished
    while (m_xfer_done == false)
    {
      }

    // if there is no error then return true else return false
    if (NRF_SUCCESS != err_code)
    {
        return false;
    }
    
    return true;	
}



/*
  A Function to verify the product id
  (its a basic test to check if we are communicating with the right slave, every type of I2C Device has 
  a special WHO_AM_I register which holds a specific value, we can read it from the MPU6050 or any device
  to confirm we are communicating with the right device)
*/ 
bool bme280_verify_product_id(void)
{
    uint8_t who_am_i; // create a variable to hold the who am i value


    // Note: All the register addresses including WHO_AM_I are declared in 
    // MPU6050.h file, you can check these addresses and values from the
    // datasheet of your slave device.
    if (bme280_register_read(BME280_REG_CHIP_ID, &who_am_i, 1))
    {
        if (who_am_i != BME280_CHIP_ID)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        return false;
    }
}



/*!
 * @brief This API gets the power mode of the sensor.
 */
bool bme280_get_sensor_mode(uint8_t *sensor_mode)
{
    bool rslt = true;

    /* Read the power mode register */
    rslt = bme280_register_read(BME280_REG_CTRL_MEAS, sensor_mode, 1);

    /* Assign the power mode to variable */
    *sensor_mode = BME280_GET_BITS_POS_0(*sensor_mode, BME280_SENSOR_MODE);
    return rslt;
}


/*!
 * @brief This API performs the soft reset of the sensor.
 */
bool bme280_soft_reset(void)
{
    bool rslt = true;
    uint8_t sensor_mode;

    rslt = bme280_register_write(BME280_REG_RESET, BME280_SOFT_RESET_COMMAND);

    if (rslt==true)
    {
      bme280_get_sensor_mode(&sensor_mode);
      if(sensor_mode==BME280_POWERMODE_SLEEP)
      {
            rslt= true;
      }
      else
      {
            rslt=false;
      }  
    }
    return rslt;
}


/*
  Function to initialize the bme280
*/ 
bool BME280_Init(void)
{   
    bool transfer_succeeded = true;
    uint8_t sensor_mode;
    uint8_t reg_data[17] = {0};
    uint8_t mydata = 0x00;
      
    //Check the id to confirm that we are communicating with the right device
    transfer_succeeded &= bme280_verify_product_id();
      
    if(bme280_verify_product_id() == false)
    {
       NRF_LOG_INFO("Cannot find device.");
       NRF_LOG_FLUSH();
       return false;
    }

    // load calibration data...
    bme280_register_read(BME280REG_CALIB_T, &reg_data, 6);
    bme280.cp.dig_T1  = reg_data[0];
    bme280.cp.dig_T1 |= reg_data[1] << 8;
    bme280.cp.dig_T2  = reg_data[2];
    bme280.cp.dig_T2 |= reg_data[3] << 8;
    bme280.cp.dig_T3  = reg_data[4];
    bme280.cp.dig_T3 |= reg_data[5] << 8;
    
    bme280_register_read(BME280REG_CALIB_H1, &reg_data, 1);
    bme280.cp.dig_H1  = reg_data[0];
    
    bme280_register_read(BME280REG_CALIB_H2, &reg_data, 7);
    bme280.cp.dig_H2  = reg_data[0];
    bme280.cp.dig_H2 |= reg_data[1] << 8;
    bme280.cp.dig_H3  = reg_data[2];
    bme280.cp.dig_H4  = reg_data[3] << 4;
    bme280.cp.dig_H4 |= reg_data[4] & 0x0F;
    bme280.cp.dig_H5  = reg_data[4] & 0xF0 >> 4;
    bme280.cp.dig_H5 |= reg_data[5] << 4;
    bme280.cp.dig_H6  = reg_data[6];
    
    bme280_register_read(BME280REG_CALIB_P, &reg_data, 18);
    bme280.cp.dig_P1  = reg_data[0];
    bme280.cp.dig_P1 |= reg_data[1] << 8;
    bme280.cp.dig_P2  = reg_data[2];
    bme280.cp.dig_P2 |= reg_data[3] << 8;
    bme280.cp.dig_P3  = reg_data[4];
    bme280.cp.dig_P3 |= reg_data[5] << 8;
    bme280.cp.dig_P4  = reg_data[6];
    bme280.cp.dig_P4 |= reg_data[7] << 8;
    bme280.cp.dig_P5  = reg_data[8];
    bme280.cp.dig_P5 |= reg_data[9] << 8;
    bme280.cp.dig_P6  = reg_data[10];
    bme280.cp.dig_P6 |= reg_data[11] << 8;
    bme280.cp.dig_P7  = reg_data[12];
    bme280.cp.dig_P7 |= reg_data[13] << 8;
    bme280.cp.dig_P8  = reg_data[14];
    bme280.cp.dig_P8 |= reg_data[15] << 8;
    bme280.cp.dig_P9  = reg_data[16];
    bme280.cp.dig_P9 |= reg_data[17] << 8;

    /* Configuring the over-sampling rate, filter coefficient and standby time */
    /* Overwrite the desired settings */
    bme280.settings.filter = BME280_FILTER_COEFF_2;

    /* Over-sampling rate for temperature, humidity, and pressure */
    bme280.settings.oversampling = BME280_OVERSAMPLING_1X;

    /* Setting the standby time */
    bme280.settings.standby_time = BME280_STANDBY_TIME_0_5_MS;
  
    bme280_get_sensor_mode(&sensor_mode);
  
    if(sensor_mode!=BME280_POWERMODE_SLEEP)
    {
       if(!bme280_soft_reset())
       {
          NRF_LOG_INFO("Cannot reset device.");
          NRF_LOG_FLUSH();
          return false;
       }
    }  
    mydata=0x00;
    mydata = (bme280.settings.standby_time << BME280_STANDBY_POS)|(bme280.settings.filter << BME280_FILTER_POS);
    bme280_register_write(BME280_REG_CONFIG, mydata);

    mydata=0x00;
    mydata = (bme280.settings.oversampling << BME280_CTRL_TEMP_POS)|(BME280_POWERMODE_NORMAL);
    bme280_register_write(BME280_REG_CTRL_MEAS, mydata);	

    return transfer_succeeded;
}


static uint32_t compensate_T_int32(int32_t adc_T)
{
	int32_t var1, var2;

	var1 = ((((adc_T>>3) - ((int32_t)bme280.cp.dig_T1<<1))) * 
               ((int32_t)bme280.cp.dig_T2)) >> 11;
               
	var2 = (((((adc_T>>4) - ((int32_t)bme280.cp.dig_T1)) *
               ((adc_T>>4) - ((int32_t)bme280.cp.dig_T1))) >> 12) * 
               ((int32_t)bme280.cp.dig_T3)) >> 14;

	bme280.t_fine = var1 + var2;

	uint32_t T = (bme280.t_fine * 5 + 128) >> 8;
	
	//printf("T=%f\n",T);
  return T; ///100.0f; // to
}

static uint32_t compensate_H_int32(int32_t adc_H)
{
	int32_t v_x1_u32r;
	
	v_x1_u32r = (bme280.t_fine - ((int32_t)76800));
	
	v_x1_u32r = (((((adc_H << 14) - (((int32_t)bme280.cp.dig_H4) << 20) - (((int32_t)bme280.cp.dig_H5) *
	v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r *
	((int32_t)bme280.cp.dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)bme280.cp.dig_H3)) >> 11) +
	((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)bme280.cp.dig_H2) +
	8192) >> 14));
	
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
	((int32_t)bme280.cp.dig_H1)) >> 4));
	
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	
	return (uint32_t)(v_x1_u32r>>12);

}

static uint32_t compensate_P_int32(int32_t adc_P)
{
	int64_t var1, var2, p;
	
	var1 = ((int64_t)bme280.t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)bme280.cp.dig_P6;
	var2 = var2 + ((var1*(int64_t)bme280.cp.dig_P5)<<17);
	var2 = var2 + (((int64_t)bme280.cp.dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)bme280.cp.dig_P3)>>8) + ((var1 * (int64_t)bme280.cp.dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)bme280.cp.dig_P1)>>33;
	
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	
	p = 1048576 - adc_P;
	p = (((p<<31) - var2)*3125)/var1;
	var1 = (((int64_t)bme280.cp.dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)bme280.cp.dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)bme280.cp.dig_P7)<<4);
	return (uint32_t)p;
	
}



bool get_temperature()
{
    bool rslt = true;
    int8_t idx = 0;
    uint8_t status_reg;
    uint8_t reg_data[3]={0};
    uint32_t temperature;
    
    /* Variables to store the sensor data */
    uint32_t data_xlsb;
    uint32_t data_lsb;
    uint32_t data_msb;

    temperature = 0;
    while (idx < SAMPLE_COUNT)
    {
        rslt = bme280_register_read(BME280_REG_STATUS, &status_reg, 1);
        if (status_reg & BME280_STATUS_MEAS_DONE)
        {
          nrf_delay_ms(1);
          /* Read compensated data */
          bme280_register_read(BME280_REG_DATA_TEMP, &reg_data, 3);
          
          /* Store the parsed register values for temperature data */
          data_msb = (uint32_t)reg_data[0] << 12;
          data_lsb = (uint32_t)reg_data[1] << 4;
          data_xlsb = (uint32_t)reg_data[2] >> 4;
          bme280.adc_t = data_msb | data_lsb | data_xlsb;
          temperature += compensate_T_int32(bme280.adc_t);
          idx++;
        }       
     }
     m_temp_value = temperature/SAMPLE_COUNT;
     NRF_LOG_INFO("Temperature:   %d deg C\n", m_temp_value);
     return rslt;
}

/*bool get_humidity()
{
    bool rslt = true;
    int8_t idx = 0;
    uint8_t status_reg;
    uint8_t reg_data[2]={0};
    uint32_t humidity;
    
    // Variables to store the sensor data
    uint32_t data_xlsb;
    uint32_t data_lsb;
    uint32_t data_msb;

    humidity = 0;
    while (idx < SAMPLE_COUNT)
    {
        rslt = bme280_register_read(BME280_REG_STATUS, &status_reg, 1);

        if (status_reg & BME280_STATUS_MEAS_DONE)
        {
          nrf_delay_ms(1);
          // Read compensated data 
          bme280_register_read(BME280_REG_DATA_HUMD, &reg_data, 2);
          
          // Store the parsed register values for humidity data 
          data_msb = (uint32_t)reg_data[0] << 8;
          data_lsb = (uint32_t)reg_data[1];
          bme280.adc_h = data_msb | data_lsb;
          humidity += compensate_H_int32(bme280.adc_h);
          idx++;
        }       
     }
     m_humidity_value = humidity/SAMPLE_COUNT;
     m_humidity_value = m_humidity_value * 100 / 1024;
     NRF_LOG_INFO("Humidity:   %d ?unit\n", m_humidity_value);
     return rslt;
}*/

/*bool get_pressure()
{
    bool rslt = true;
    int8_t idx = 0;
    uint8_t status_reg;
    uint8_t reg_data[3]={0};
    uint32_t pressure;
    
    // Variables to store the sensor data
    uint32_t data_xlsb;
    uint32_t data_lsb;
    uint32_t data_msb;

    pressure = 0;
    while (idx < SAMPLE_COUNT)
    {
        rslt = bme280_register_read(BME280_REG_STATUS, &status_reg, 1);

        if (status_reg & BME280_STATUS_MEAS_DONE)
        {
          nrf_delay_ms(1);
          // Read compensated data
          bme280_register_read(BME280_REG_DATA_PRES, &reg_data, 3);
          
          // Store the parsed register values for pressure data 
          data_msb = (uint32_t)reg_data[0] << 12;
          data_lsb = (uint32_t)reg_data[1] << 4;
          data_xlsb = (uint32_t)reg_data[2] >> 4;
          bme280.adc_h = data_msb | data_lsb | data_xlsb;
          pressure += compensate_P_int32(bme280.adc_h);
          idx++;
        }       
     }
     m_pressure_value = pressure/SAMPLE_COUNT;
     m_pressure_value = m_pressure_value * 14.9 / 256;
     NRF_LOG_INFO("Pressure:   %d ?unit\n", m_pressure_value);
     return rslt;
}*/


//Event Handler
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    //Check the event to see what type of event occurred
    switch (p_event->type)
    {
        //If data transmission or receiving is finished
	case NRF_DRV_TWI_EVT_DONE:
        m_xfer_done = true;//Set the flag
        break;
        
        default:
        // do nothing
          break;
    }
}

void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = 12,
       .sda                = 18,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

        APP_ERROR_CHECK(err_code);
    
}


/**@brief Function for application main entry.
 */
int main(void)
{   
    // Initialize.
    log_init();
    timers_init();
    
    // Delay to allow CO signal chain to settle.
    nrf_delay_ms(500);
    
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    // read_ble_address();
    advertising_init();
    services_init();
    dis_init();    
    conn_params_init();

    // Start execution.
    application_timers_start();
    gpio_init();
    
    saadc_init();
    saadc_sampling_event_init();
    
    twi_init(); 
    BME280_Init();

    //nrf_temp_init();
    flash_page_init();

    advertising_start();
    
    NRF_LOG_INFO("Application started.");

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}
