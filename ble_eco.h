/**
 * Copyright (c) 2012 - 2021, Nordic Semiconductor ASA
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
/** @file
 *
 * @defgroup ble_eco Exhaled Carbon Monoxide Concentration
 * @{
 * @ingroup ble_sdk_srv
 * @brief Exhaled Carbon Monoxide Concentration module.
 *
 * @details This module implements the Exhaled Carbon Monoxide Concentration service.
 *          During initialization it adds the Exhaled Carbon Monoxide Concentration to the BLE stack database.
 *          It then encodes the supplied information, and adds the corresponding characteristics.
 *
 * @note Attention!
 *  To maintain compliance with Nordic Semiconductor ASA Bluetooth profile
 *  qualification listings, this section of source code must not be modified.
 */

#ifndef BLE_ECO_H__
#define BLE_ECO_H__

#include <stdint.h>
#include "ble_srv_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief Macro for defining a ble_eco instance.
 *
 * @param   _name  Name of the instance.
 * @hideinitializer
 */
#define BLE_ECO_DEF(_name)                          \
    static ble_eco_t _name;                         \
    NRF_SDH_BLE_OBSERVER(_name ## _obs,             \
                         BLE_ECO_BLE_OBSERVER_PRIO, \
                         ble_eco_on_ble_evt,        \
                         &_name)

/**@brief Environmental Sensing Service event type. */
typedef enum
{
    BLE_ECO_EVT_RX_DATA,              /**< Data received. */
    BLE_ECO_EVT_NOTIFICATION_ENABLED, /**< Concentration notification enabled event. */
    BLE_ECO_EVT_NOTIFICATION_DISABLED /**< Concentration notification disabled event. */
} ble_eco_evt_type_t;

/**@brief Battery Service event. */
typedef struct
{
    ble_eco_evt_type_t evt_type;    /**< Type of event. */
    uint16_t           conn_handle; /**< Connection handle. */
} ble_eco_evt_t;

/** @defgroup DIS_VENDOR_ID_SRC_VALUES Vendor ID Source values as defined by Assigned Numbers Specification
 * @{
 */
#define BLE_UUID_ECO_SERVICE                                     0x181A     /**< Environmental Sensing Service UUID*/
#define BLE_UUID_CO_CONCENTRATION                                0x2BD0     /**< Object Transfer Service UUID*/
#define BLE_UUID_STATUS_BYTE                                     0x2BBB     /**< Object Transfer Service UUID*/
#define BLE_UUID_COMMAND_BYTE                                    0x2A9F     /**< Object Transfer Service UUID*/
#define BLE_UUID_TEST_BYTE                                       0x2AF4     /**< Object Transfer Service UUID*/
#define BLE_UUID_TEMPERATURE                                     0x2A6E
#define BLE_UUID_HUMIDITY                                        0x2A6F
#define BLE_UUID_PRESSURE                                        0x2A6D


/* Common addresses definition for temperature sensor. */
#define BME280_I2C_ADDR        0x76
#define BME280_CHIP_ID         0x60
#define BME280_ADDRESS_LEN     1 

#define BME280_REG_CHIP_ID     0xD0
#define BME280_REG_RESET       0xE0
#define BME280_REG_STATUS      0xF3
#define BME280_REG_CTRL_MEAS   0xF4
#define BME280_REG_CONFIG      0xF5
#define BME280_REG_DATA_TEMP   0xFA
#define BME280_REG_DATA_HUMD   0xFD
#define BME280_REG_DATA_PRES   0xF7
#define BME280REG_CALIB_T      0x88	// Start Address for Temperature (6 bytes)
#define BME280REG_CALIB_H1     0xA1	// Start Address 1 for Humidity (1 byte)
#define BME280REG_CALIB_H2     0xE1	// Start Address 2 for Humidity (7 bytes)
#define BME280REG_CALIB_P      0x8E	// Start Address for Pressure (18 bytes)

#define BME280_CTRL_TEMP_MSK     0xE0
#define BME280_CTRL_TEMP_POS     0x05
#define BME280_FILTER_MSK        0x1C
#define BME280_FILTER_POS        0x02
#define BME280_STANDBY_MSK       0xE0
#define BME280_STANDBY_POS       0x05
#define BME280_SENSOR_MODE_MSK   0x03
#define BME280_SENSOR_MODE_POS   0x00


#define BME280_OVERSAMPLING_1X   0x01

/*! @name Standby duration selection macros */
#define BME280_STANDBY_TIME_0_5_MS       0x00
#define BME280_STANDBY_TIME_62_5_MS      0x01
#define BME280_STANDBY_TIME_125_MS       0x02
#define BME280_STANDBY_TIME_250_MS       0x03
#define BME280_STANDBY_TIME_500_MS       0x04
#define BME280_STANDBY_TIME_1000_MS      0x05
#define BME280_STANDBY_TIME_10_MS        0x06
#define BME280_STANDBY_TIME_20_MS        0x07


/*! @name Filter coefficient selection macros */
#define BME280_FILTER_COEFF_2            0x01


/*! @name Sensor power modes */
#define BME280_POWERMODE_SLEEP           0x00
#define BME280_POWERMODE_FORCED          0x01
#define BME280_POWERMODE_NORMAL          0x03

/*! @name Soft reset command */
#define BME280_SOFT_RESET_COMMAND        0xB6

#define BME280_STATUS_IM_UPDATE          0x01
#define BME280_STATUS_MEAS_DONE          0x08

#define BME280_GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname##_MSK))


// Forward declaration of the ble_eco_t type.
typedef struct ble_eco_s ble_eco_t;

/**@brief Battery Service event handler type. */
typedef void (* ble_eco_evt_handler_t) (ble_eco_t * p_eco, ble_eco_evt_t * p_evt);

/**@brief Device Information Service init structure. This contains all possible characteristics
 *        needed for initialization of the service.
 */
typedef struct
{
    ble_eco_evt_handler_t          evt_handler;                    /**< Event handler to be called for handling events in the Battery Service. */
    ble_srv_error_handler_t        error_handler;                  /**< Function to be called in case of an error. */
    uint16_t *                     concentration_value;            /**< Last Battery Level measurement passed to the Battery Service. */
    uint16_t *                     temperature_value;
    uint16_t *                     humidity_value;
    uint16_t *                     pressure_value; 
    uint16_t *                     status_value;                   /**< Last Battery Level measurement passed to the Battery Service. */
    uint16_t *                     test_value;                     /**< Last Battery Level measurement passed to the Battery Service. */
    uint32_t *                     device_info;

} ble_eco_init_t;

/**@brief Battery Service structure. This contains various status information for the service. */
struct ble_eco_s
{
    ble_eco_evt_handler_t    evt_handler;               /**< Event handler to be called for handling events in the Battery Service. */
    uint16_t                 service_handle;            /**< Handle of Battery Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t concentration_handles;     /**< Handles related to the Battery Level characteristic. */
    ble_gatts_char_handles_t temperature_handles;
    ble_gatts_char_handles_t humidity_handles;
    ble_gatts_char_handles_t pressure_handles;   
    ble_gatts_char_handles_t status_handles;            /**< Handles related to the Battery Level characteristic. */
    ble_gatts_char_handles_t command_handles;           /**< Handles related to the Battery Level characteristic. */
    ble_gatts_char_handles_t test_handles;              /**< Handles related to the Battery Level characteristic. */
    uint16_t                 report_ref_handle;         /**< Handle of the Report Reference descriptor. */
    uint32_t *               concentration_value;       /**< Last Battery Level measurement passed to the Battery Service. */
    uint16_t                 concentration_last;        /**< Last Battery Level measurement passed to the Battery Service. */
    uint32_t *               temperature_value;       
    uint16_t                 temperature_last;
    uint32_t *               humidity_value;       
    uint16_t                 humidity_last;
    uint32_t *               pressure_value;       
    uint32_t                 pressure_last;   
    uint16_t *               status_value;              /**< Last Battery Level measurement passed to the Battery Service. */
    uint16_t                 status_last;               /**< Last Battery Level measurement passed to the Battery Service. */
    uint16_t *               test_value;
    uint32_t *               device_info;               /**< Pointer to device info array: [0] resets, [1] time. */
};


/*!
 * @brief bme280 sensor settings structure which comprises of mode,
 * oversampling and filter settings.
 */
struct bme280_settings
{

    uint8_t oversampling;            /*! Temperature oversampling */
    uint8_t filter;                  /*! Filter coefficient */
    uint8_t standby_time;            /*! Standby time */
};

struct comp_params {
	// Temperature calibration values
	uint16_t dig_T1;
	int16_t  dig_T2;
	int16_t  dig_T3;
	
	// Pressure calibration values
	uint16_t dig_P1;
	int16_t  dig_P2;
	int16_t  dig_P3;
	int16_t  dig_P4;
	int16_t  dig_P5;
	int16_t  dig_P6;
	int16_t  dig_P7;
	int16_t  dig_P8;
	int16_t  dig_P9;
		
	// Humidity calibration values
	uint8_t  dig_H1;
	int16_t  dig_H2;
	uint8_t  dig_H3;
	int16_t  dig_H4;
	int16_t  dig_H5;
	uint8_t  dig_H6;
	
	
};

struct bme280_driver {
	bool sensor_available;
	int32_t adc_t;		///< RAW temp
	int32_t adc_h;		///< RAW humidity
	int32_t adc_p;		///< RAW pressure
	int32_t t_fine;	///< calibrated temperature
	struct comp_params cp;	///< calibration data
        struct bme280_settings settings; 
};

/**@brief Function for initializing the Device Information Service.
 *
 * @details This call allows the application to initialize the device information service.
 *          It adds the DIS service and DIS characteristics to the database, using the initial
 *          values supplied through the p_dis_init parameter. Characteristics which are not to be
 *          added, shall be set to NULL in p_dis_init.
 *
 * @param[in]   p_dis_init   The structure containing the values of characteristics needed by the
 *                           service.
 *
 * @return      NRF_SUCCESS on successful initialization of service.
 */
uint32_t ble_eco_init(ble_eco_t * p_eco, ble_eco_init_t const * p_eco_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Battery Service.
 *
 * @note For the requirements in the BAS specification to be fulfilled,
 *       ble_bas_battery_level_update() must be called upon reconnection if the
 *       battery level has changed while the service has been disconnected from a bonded
 *       client.
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 * @param[in]   p_context   Battery Service structure.
 */
void ble_eco_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief Function for updating the battery level.
 *
 * @details The application calls this function after having performed a battery measurement.
 *          The battery level characteristic will only be sent to the clients which have
 *          enabled notifications. \ref BLE_CONN_HANDLE_ALL can be used as a connection handle
 *          to send notifications to all connected devices.
 *
 * @param[in]   p_bas          Battery Service structure.
 * @param[in]   battery_level  New battery measurement value (in percent of full capacity).
 * @param[in]   conn_handle    Connection handle.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
ret_code_t ble_eco_concentration_update(ble_eco_t * p_eco, uint16_t conn_handle);
                                        
ret_code_t ble_eco_temperature_update(ble_eco_t * p_eco, uint16_t conn_handle);

ret_code_t ble_eco_humidity_update(ble_eco_t * p_eco, uint16_t conn_handle); 

ret_code_t ble_eco_pressure_update(ble_eco_t * p_eco, uint16_t conn_handle);                                      

                                        /**@brief Function for updating the battery level.
 *
 * @details The application calls this function after having performed a battery measurement.
 *          The battery level characteristic will only be sent to the clients which have
 *          enabled notifications. \ref BLE_CONN_HANDLE_ALL can be used as a connection handle
 *          to send notifications to all connected devices.
 *
 * @param[in]   p_bas          Battery Service structure.
 * @param[in]   battery_level  New battery measurement value (in percent of full capacity).
 * @param[in]   conn_handle    Connection handle.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
ret_code_t ble_eco_status_update(ble_eco_t * p_eco,
                                        uint16_t    conn_handle);

/**@brief Function for sending the last battery level when bonded client reconnects.
 *
 * @details The application calls this function, in the case of a reconnection of
 *          a bonded client if the value of the battery has changed since
 *          its disconnection.
 *
 * @note For the requirements in the BAS specification to be fulfilled,
 *       this function must be called upon reconnection if the battery level has changed
 *       while the service has been disconnected from a bonded client.
 *
 * @param[in]   p_bas          Battery Service structure.
 * @param[in]   conn_handle    Connection handle.
 *
 * @return      NRF_SUCCESS on success,
 *              NRF_ERROR_INVALID_STATE when notification is not supported,
 *              otherwise an error code returned by @ref sd_ble_gatts_hvx.
 */
ret_code_t ble_eco_concentration_on_reconnection_update(ble_eco_t * p_eco,
                                                        uint16_t    conn_handle);

#ifdef __cplusplus
}
#endif

#endif // BLE_DIS_H__

/** @} */
